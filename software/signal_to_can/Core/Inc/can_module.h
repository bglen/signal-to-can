/*
 * can.h
 *
 *  Created on: Aug 6, 2025
 *      Author: bglen
 */

/* === can_module.h === */
#ifndef CAN_MODULE_H
#define CAN_MODULE_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

// Callback type for received CAN messages
// header: pointer to received CAN header
// data: pointer to received data buffer (max 8 bytes)
typedef void (*CanRxCallback_t)(CAN_RxHeaderTypeDef *header, uint8_t *data);

// Initialize CAN peripheral, start and enable RX interrupt
HAL_StatusTypeDef CAN_Module_Init(void);

// Send a CAN message with standard ID
// id: 11-bit identifier
// data: pointer to data bytes (max 8)
// length: number of bytes (0..8)
HAL_StatusTypeDef CAN_Module_Send(uint32_t id, uint8_t *data, uint8_t length);

// Update CAN filter configuration (e.g. to change accepted IDs)
// filterConfig must be fully populated by caller
HAL_StatusTypeDef CAN_Module_UpdateFilter(CAN_FilterTypeDef *filterConfig);

// Register a callback for a specific standard message ID
// msgId: 11-bit identifier to match
// callback: function to call when a matching message arrives
void CAN_Module_RegisterRxCallback(uint32_t msgId, CanRxCallback_t callback);

#endif // CAN_MODULE_H
