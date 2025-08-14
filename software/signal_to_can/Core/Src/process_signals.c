#include "process_signals.h"
#include "adc_module.h"   /* DMA-backed readings */   /* uses ADC_Module_Get_Buffer() */
#include "can_module.h"   /* CAN send + node id */    /* uses CAN_Module_Send_Std() */

#include <string.h>
#include <math.h>

/* ---------- Internal state ---------- */

typedef struct {
    float gain;     /* V_in = gain * V_pin + offset */
    float offset;   /* volts */
    float v_min;    /* device-input min (V) */
    float v_max;    /* device-input max (V) */
} ps_cal_t;

static ps_cal_t s_cal[PS_NUM_CHANNELS];

static uint16_t s_raw[PS_NUM_CHANNELS];       /* snapshot of ADC counts */
static float    s_v_pin[PS_NUM_CHANNELS];     /* volts at MCU pin */
static float    s_v_in[PS_NUM_CHANNELS];      /* volts at device input */
static uint16_t s_v_in_mV[PS_NUM_CHANNELS];   /* device input in millivolts */
static uint8_t  s_oor_mask = 0u;

static uint32_t s_last_send_tick = 0u;

/* ---------- Helpers ---------- */

/* Convert raw ADC count to pin voltage, using PS_ADC_VREF_V and PS_ADC_FULL_SCALE. */
static inline float adc_raw_to_vpin(uint16_t raw)
{
    return (PS_ADC_VREF_V * (float)raw) / PS_ADC_FULL_SCALE;
}

/* Saturate float volts to uint16 millivolts. */
static inline uint16_t volts_to_mV_u16(float v)
{
    /* Clamp to 0..65.535 V -> 0..65535 mV for transport.
       Adjust if your device input can exceed this. */
    if (v <= 0.0f) return 0u;
    float mv = v * 1000.0f;
    if (mv >= 65535.0f) return 65535u;
    return (uint16_t)(mv + 0.5f);
}

/* Big-endian write of u16 into a byte buffer. */
static inline void put_u16_be(uint8_t *buf, uint16_t v)
{
    buf[0] = (uint8_t)((v >> 8) & 0xFFu);
    buf[1] = (uint8_t)(v & 0xFFu);
}

/* ---------- Public API ---------- */

void Process_Signals_Init(void)
{
    for (uint8_t i = 0; i < PS_NUM_CHANNELS; ++i) {
        s_cal[i].gain   = 1.0f;             /* unity by default */
        s_cal[i].offset = 0.0f;
        s_cal[i].v_min  = PS_DEFAULT_MIN_V; /* device-input domain */
        s_cal[i].v_max  = PS_DEFAULT_MAX_V;
        s_raw[i]        = 0u;
        s_v_pin[i]      = 0.0f;
        s_v_in[i]       = 0.0f;
        s_v_in_mV[i]    = 0u;
    }
    s_oor_mask = 0u;
    s_last_send_tick = HAL_GetTick();
}

void Process_Signals_Update(void)
{
    /* Take a stable snapshot of the DMA buffer first. */
    const volatile uint16_t *dma = ADC_Module_Get_Buffer();
    for (uint8_t i = 0; i < PS_NUM_CHANNELS; ++i) {
        s_raw[i] = dma[i];
    }

    /* Convert to pin volts, then device-input volts. */
    uint8_t mask = 0u;
    for (uint8_t i = 0; i < PS_NUM_CHANNELS; ++i) {
        const float vpin = adc_raw_to_vpin(s_raw[i]);
        const float vin  = s_cal[i].gain * vpin + s_cal[i].offset;

        s_v_pin[i]   = vpin;
        s_v_in[i]    = vin;
        s_v_in_mV[i] = volts_to_mV_u16(vin);

        if (vin < s_cal[i].v_min || vin > s_cal[i].v_max) {
            mask |= (uint8_t)(1u << i);
        }
    }
    s_oor_mask = mask;
}

void Process_Signals_Get_All_Raw(uint16_t *out_raw)
{
    if (!out_raw) return;
    for (uint8_t i = 0; i < PS_NUM_CHANNELS; ++i) {
        out_raw[i] = s_raw[i];
    }
}

float Process_Signals_Get_Input_V(uint8_t ch)
{
    if (ch >= PS_NUM_CHANNELS) return 0.0f;
    return s_v_in[ch];
}

uint16_t Process_Signals_Get_Input_mV(uint8_t ch)
{
    if (ch >= PS_NUM_CHANNELS) return 0u;
    return s_v_in_mV[ch];
}

const uint16_t* Process_Signals_Get_All_Input_mV(void)
{
    return s_v_in_mV;
}

uint8_t Process_Signals_Get_OutOfRange_Mask(void)
{
    return s_oor_mask;
}

void Process_Signals_Set_MinMax(uint8_t ch, float v_min, float v_max)
{
    if (ch >= PS_NUM_CHANNELS) return;
    s_cal[ch].v_min = v_min;
    s_cal[ch].v_max = v_max;
}

void Process_Signals_Get_MinMax(uint8_t ch, float *v_min_out, float *v_max_out)
{
    if (ch >= PS_NUM_CHANNELS) return;
    if (v_min_out) *v_min_out = s_cal[ch].v_min;
    if (v_max_out) *v_max_out = s_cal[ch].v_max;
}

void Process_Signals_Set_Divider(uint8_t ch, float r_top_ohm, float r_bottom_ohm)
{
    if (ch >= PS_NUM_CHANNELS) return;
    if (r_bottom_ohm <= 0.0f) return;
    /* V_in = V_pin * (Rtop + Rbottom)/Rbottom */
    const float gain = (r_top_ohm + r_bottom_ohm) / r_bottom_ohm;
    s_cal[ch].gain   = gain;
    s_cal[ch].offset = 0.0f;
}

void Process_Signals_Set_GainOffset(uint8_t ch, float gain, float offset)
{
    if (ch >= PS_NUM_CHANNELS) return;
    s_cal[ch].gain   = gain;
    s_cal[ch].offset = offset;
}

HAL_StatusTypeDef Process_Signals_Send_Can(uint32_t timeout_ms)
{
    /* Ensure latest conversions are used. */
    /* Caller may have called Update() already; we do not force it. */

    uint8_t frame[8];

    /* Frame 1: channels 0..3 */
    put_u16_be(&frame[0], s_v_in_mV[0]);
    put_u16_be(&frame[2], s_v_in_mV[1]);
    put_u16_be(&frame[4], s_v_in_mV[2]);
    put_u16_be(&frame[6], s_v_in_mV[3]);

    uint16_t id1 = (uint16_t)(CAN_Module_Get_Node_Id() + 0x1u);
    HAL_StatusTypeDef st = CAN_Module_Send_Std(id1, frame, 8u, timeout_ms);
    if (st != HAL_OK) {
        return st;
    }

    /* Frame 2: channels 4..7 */
    put_u16_be(&frame[0], s_v_in_mV[4]);
    put_u16_be(&frame[2], s_v_in_mV[5]);
    put_u16_be(&frame[4], s_v_in_mV[6]);
    put_u16_be(&frame[6], s_v_in_mV[7]);

    uint16_t id2 = (uint16_t)(CAN_Module_Get_Node_Id() + 0x2u);
    st = CAN_Module_Send_Std(id2, frame, 8u, timeout_ms);
    return st;
}

HAL_StatusTypeDef Process_Signals_Send_Can_If_Due(uint32_t period_ms, uint32_t timeout_ms)
{
    const uint32_t now = HAL_GetTick();
    if ((now - s_last_send_tick) < period_ms) {
        return HAL_BUSY;
    }
    s_last_send_tick = now;

    /* Update snapshot right before sending to minimize staleness. */
    Process_Signals_Update();
    return Process_Signals_Send_Can(timeout_ms);
}
