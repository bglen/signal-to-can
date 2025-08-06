/*
 * can_module.c
 *
 *  Created on: Aug 6, 2025
 *      Author: bglen
 */

/* === can_module.c === */
#include "can_module.h"
#include "main.h"      // for extern CAN_HandleTypeDef hcan
#include <string.h>

#define MAX_CAN_RX_CALLBACKS 10

typedef struct {
    uint32_t msgId;
    CanRxCallback_t callback;
} CanRxCbEntry_t;

static CanRxCbEntry_t rxCbs[MAX_CAN_RX_CALLBACKS];
static uint8_t rxCbCount = 0;

extern CAN_HandleTypeDef hcan;

HAL_StatusTypeDef CAN_Module_Init(void) {
    HAL_StatusTypeDef ret;
    // Start CAN peripheral
    ret = HAL_CAN_Start(&hcan);
    if (ret != HAL_OK) {
        return ret;
    }
    // Enable RX FIFO0 message pending interrupt
    ret = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    return ret;
}

HAL_StatusTypeDef CAN_Module_Send(uint32_t id, uint8_t *data, uint8_t length) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    memset(&txHeader, 0, sizeof(txHeader));
    txHeader.StdId = id;
    txHeader.IDE   = CAN_ID_STD;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = length;
    return HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox);
}

HAL_StatusTypeDef CAN_Module_UpdateFilter(CAN_FilterTypeDef *filterConfig) {
    // Reconfigure CAN filter (bank 0 by default)
    return HAL_CAN_ConfigFilter(&hcan, filterConfig);
}

void CAN_Module_RegisterRxCallback(uint32_t msgId, CanRxCallback_t callback) {
    if (rxCbCount < MAX_CAN_RX_CALLBACKS) {
        rxCbs[rxCbCount++] = (CanRxCbEntry_t){ .msgId = msgId, .callback = callback };
    }
}

// Weak callback called by HAL when RX FIFO0 has pending message
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        return;
    }
    for (uint8_t i = 0; i < rxCbCount; i++) {
        if ((rxCbs[i].msgId == rxHeader.StdId) && rxCbs[i].callback) {
            rxCbs[i].callback(&rxHeader, rxData);
        }
    }
}
