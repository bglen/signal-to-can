/* can_module.c
 *
 * CAN utility module for STM32F042 (bxCAN + STM32 HAL).
 * This module provides:
 *  - CAN initialization with selectable baud rate
 *  - Filter configuration to receive a set of Standard IDs
 *  - Send a Standard ID data frame
 *  - Receive a Standard ID data frame
 *  - Store and access a CAN node ID
 *  - Update baud rate at runtime (re-inits CAN and reapplies filters)
 *
 */

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* ===== Module configuration ===== */

/* Maximum number of distinct STD IDs we will keep in our internal filter list.
 * Each filter bank in 16-bit IDLIST mode can hold 4 STD IDs.
 * The F0 has 14 banks -> up to 56 IDs theoretical. Adjust as needed.
 */
#ifndef CAN_MODULE_MAX_FILTER_IDS
#define CAN_MODULE_MAX_FILTER_IDS 32u
#endif

/* Select which RX FIFO to use for receives and filter assignment. */
#ifndef CAN_MODULE_RX_FIFO
#define CAN_MODULE_RX_FIFO CAN_RX_FIFO0
#endif

/* Total number of filter banks available on this device (bxCAN). */
#ifndef CAN_MODULE_FILTER_BANKS
#define CAN_MODULE_FILTER_BANKS 14u
#endif

/* ===== Private state ===== */

static CAN_HandleTypeDef *s_can = NULL;
static uint8_t s_node_id = 0u;

/* Store the last applied baud selection so we can report or re-use it. */
static uint32_t s_baud_enum = 2u; /* 0=125k, 1=250k, 2=500k, 3=1000k */

/* Keep a copy of configured STD IDs so we can re-apply after re-init. */
static uint16_t s_filter_ids[CAN_MODULE_MAX_FILTER_IDS];
static size_t   s_filter_id_count = 0u;

/* ===== Helpers ===== */

/* Encodes an 11-bit Standard ID into the 16-bit filter element format.
 * Layout (16-bit scale, IDLIST mode): STDID[10:0] at bits 15:5, IDE=0, RTR=0.
 */
static uint16_t encode_filter16_std_id(uint16_t std_id)
{
    return (uint16_t)((std_id & 0x7FFu) << 5);
}

/* Configure CAN bit timing for a 48 MHz CAN kernel clock (typical on F0 when the
 * APB runs at 48 MHz). Uses 16 TQ per bit and ~87.5% sample point.
 *  - 1 Mbps: prescaler 3
 *  - 500 kbps: prescaler 6
 *  - 250 kbps: prescaler 12
 *  - 125 kbps: prescaler 24
 *
 * If your CAN kernel clock is not 48 MHz, adjust these values accordingly.
 */
static void set_bit_timing_for_baud(CAN_HandleTypeDef *hcan, uint32_t baud_enum)
{
    /* Common time segments: 1 + 13 + 2 = 16 TQ, SJW=1 TQ */
    hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan->Init.TimeSeg1      = CAN_BS1_13TQ;
    hcan->Init.TimeSeg2      = CAN_BS2_2TQ;

    switch (baud_enum) {
    default:
    case 0: /* 125 kbps */
        hcan->Init.Prescaler = 24u;
        break;
    case 1: /* 250 kbps */
        hcan->Init.Prescaler = 12u;
        break;
    case 2: /* 500 kbps */
        hcan->Init.Prescaler = 6u;
        break;
    case 3: /* 1 Mbps */
        hcan->Init.Prescaler = 3u;
        break;
    }
}

/* Applies an "accept all" filter (mask=0) so all STD/EXT frames pass to FIFO0.
 * This is used at first init and if no explicit ID list has been configured.
 */
static HAL_StatusTypeDef apply_accept_all_filter(void)
{
    if (s_can == NULL) {
        return HAL_ERROR;
    }

    CAN_FilterTypeDef filter;
    memset(&filter, 0, sizeof(filter));

    filter.FilterBank = 0u;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    /* Accept all: mask = 0x00000000. For STD IDs, ID and mask use 32-bit format:
       STDID at bits 31:21. Setting mask to zero makes all bits "don't care". */
    filter.FilterIdHigh      = 0x0000;
    filter.FilterIdLow       = 0x0000;
    filter.FilterMaskIdHigh  = 0x0000;
    filter.FilterMaskIdLow   = 0x0000;

    filter.FilterFIFOAssignment = CAN_MODULE_RX_FIFO;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = CAN_MODULE_FILTER_BANKS; /* not used on single CAN */

    return HAL_CAN_ConfigFilter(s_can, &filter);
}

/* Re-apply the currently stored ID list to hardware filters. */
static HAL_StatusTypeDef reapply_id_list_filters(void)
{
    if (s_can == NULL) {
        return HAL_ERROR;
    }

    if (s_filter_id_count == 0u) {
        return apply_accept_all_filter();
    }

    HAL_StatusTypeDef st = HAL_OK;

    /* Program as many banks as needed in 16-bit IDLIST mode (4 IDs per bank). */
    size_t remaining = s_filter_id_count;
    size_t idx = 0u;
    uint8_t bank = 0u;

    while (remaining > 0u && bank < CAN_MODULE_FILTER_BANKS) {
        CAN_FilterTypeDef filter;
        memset(&filter, 0, sizeof(filter));

        filter.FilterBank  = bank;
        filter.FilterMode  = CAN_FILTERMODE_IDLIST;
        filter.FilterScale = CAN_FILTERSCALE_16BIT;
        filter.FilterFIFOAssignment = CAN_MODULE_RX_FIFO;
        filter.FilterActivation = ENABLE;
        filter.SlaveStartFilterBank = CAN_MODULE_FILTER_BANKS;

        /* Up to 4 IDs per bank. Fill missing entries with a value that should not match (unused ID). */
        uint16_t e0 = encode_filter16_std_id(s_filter_ids[idx + 0u]);
        uint16_t e1 = (remaining > 1u) ? encode_filter16_std_id(s_filter_ids[idx + 1u]) : 0u;
        uint16_t e2 = (remaining > 2u) ? encode_filter16_std_id(s_filter_ids[idx + 2u]) : 0u;
        uint16_t e3 = (remaining > 3u) ? encode_filter16_std_id(s_filter_ids[idx + 3u]) : 0u;

        filter.FilterIdHigh      = e0;
        filter.FilterIdLow       = e1;
        filter.FilterMaskIdHigh  = e2;
        filter.FilterMaskIdLow   = e3;

        st = HAL_CAN_ConfigFilter(s_can, &filter);
        if (st != HAL_OK) {
            return st;
        }

        size_t used = (remaining >= 4u) ? 4u : remaining;
        remaining -= used;
        idx += used;
        bank++;
    }

    /* Deactivate any remaining banks to avoid unintended matches. */
    for (; bank < CAN_MODULE_FILTER_BANKS; bank++) {
        CAN_FilterTypeDef filter;
        memset(&filter, 0, sizeof(filter));
        filter.FilterBank = bank;
        filter.FilterMode = CAN_FILTERMODE_IDMASK;
        filter.FilterScale = CAN_FILTERSCALE_32BIT;
        filter.FilterActivation = DISABLE;
        filter.SlaveStartFilterBank = CAN_MODULE_FILTER_BANKS;
        (void)HAL_CAN_ConfigFilter(s_can, &filter);
    }

    return HAL_OK;
}

/* Waits until a TX mailbox is free or the timeout elapses. */
static HAL_StatusTypeDef wait_for_tx_mailbox(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(s_can) == 0u) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/* ===== Public API ===== */

/* Initializes the CAN peripheral with the requested baud, starts it, and applies
 * a default permissive filter. Stores the provided node_id.
 */
HAL_StatusTypeDef CAN_Module_Init(CAN_HandleTypeDef *hcan, uint32_t baud_enum)
{
    if (hcan == NULL) {
        return HAL_ERROR;
    }

    s_can = hcan;
    s_baud_enum = baud_enum;

    /* Base init fields. These may be overridden by CubeMX; set key ones here. */
    s_can->Init.Mode = CAN_MODE_NORMAL;
    s_can->Init.TimeTriggeredMode = DISABLE;
    s_can->Init.AutoBusOff = ENABLE;
    s_can->Init.AutoWakeUp = ENABLE;
    s_can->Init.AutoRetransmission = ENABLE;
    s_can->Init.ReceiveFifoLocked = DISABLE;
    s_can->Init.TransmitFifoPriority = ENABLE;

    set_bit_timing_for_baud(s_can, baud_enum);

    if (HAL_CAN_Init(s_can) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Default to accept-all until user configures specific IDs. */
    if (apply_accept_all_filter() != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(s_can) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* Updates the internal node ID value stored by this module. */
void CAN_Module_Set_Node_Id(uint8_t node_id)
{
    s_node_id = node_id;
}

/* Returns the current node ID stored by this module. */
uint8_t CAN_Module_Get_Node_Id(void)
{
    return s_node_id;
}

/* Updates the CAN baud rate at runtime.
 * This function stops CAN, re-initializes timing, reapplies filters, and restarts CAN.
 */
HAL_StatusTypeDef CAN_Module_Update_Baud(uint32_t baud_enum)
{
    if (s_can == NULL) {
        return HAL_ERROR;
    }

    if (HAL_CAN_Stop(s_can) != HAL_OK) {
        return HAL_ERROR;
    }

    s_baud_enum = baud_enum;
    set_bit_timing_for_baud(s_can, baud_enum);

    if (HAL_CAN_Init(s_can) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Re-apply stored filters (or accept-all if none). */
    if (reapply_id_list_filters() != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(s_can) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* Programs hardware filters to receive the provided list of Standard IDs.
 * The list is copied into module storage and will be re-applied after baud changes.
 * If id_count is zero, an accept-all filter is applied.
 */
HAL_StatusTypeDef CAN_Module_Update_StdId_Filters(const uint16_t *id_list, size_t id_count)
{
    if (s_can == NULL) {
        return HAL_ERROR;
    }

    if (id_count == 0u || id_list == NULL) {
        s_filter_id_count = 0u;
        return reapply_id_list_filters();
    }

    if (id_count > CAN_MODULE_MAX_FILTER_IDS) {
        id_count = CAN_MODULE_MAX_FILTER_IDS;
    }

    memcpy(s_filter_ids, id_list, id_count * sizeof(uint16_t));
    s_filter_id_count = id_count;

    return reapply_id_list_filters();
}

/* Sends a Standard ID data frame with the given payload and DLC.
 * timeout_ms applies to waiting for a free TX mailbox.
 */
HAL_StatusTypeDef CAN_Module_Send_Std(uint16_t std_id, const uint8_t *data, uint8_t dlc, uint32_t timeout_ms)
{
    if (s_can == NULL || data == NULL || dlc > 8u) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef st = wait_for_tx_mailbox(timeout_ms);
    if (st != HAL_OK) {
        return st;
    }

    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;

    tx_header.StdId = (std_id & 0x7FFu);
    tx_header.ExtId = 0u;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(s_can, &tx_header, (uint8_t *)data, &mailbox);
}

/* Receives a Standard ID data frame from FIFO0 with a simple timeout poll.
 * On success, fills std_id, dlc, and copies payload to data (assumes data has space for 8 bytes).
 * Returns HAL_OK on success, HAL_TIMEOUT if no frame arrived in time.
 */
HAL_StatusTypeDef CAN_Module_Receive_Std(uint16_t *std_id, uint8_t *data, uint8_t *dlc, uint32_t timeout_ms)
{
    if (s_can == NULL || std_id == NULL || data == NULL || dlc == NULL) {
        return HAL_ERROR;
    }

    uint32_t start = HAL_GetTick();
    while (HAL_CAN_GetRxFifoFillLevel(s_can, CAN_MODULE_RX_FIFO) == 0u) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return HAL_TIMEOUT;
        }
    }

    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(s_can, CAN_MODULE_RX_FIFO, &rx_header, rx_data) != HAL_OK) {
        return HAL_ERROR;
    }

    if (rx_header.IDE != CAN_ID_STD) {
        /* Ignore non-STD frames in this simple API. */
        return HAL_ERROR;
    }

    *std_id = (uint16_t)(rx_header.StdId & 0x7FFu);
    *dlc = rx_header.DLC;

    /* Copy exactly DLC bytes. */
    if (*dlc > 0u) {
        memcpy(data, rx_data, *dlc);
    }

    return HAL_OK;
}

/* Optional utility: returns the last configured baud enum. */
uint32_t CAN_Module_Get_Baud_Enum(void)
{
    return s_baud_enum;
}
