/* can_module.h
 *
 * Minimal CAN utility API for STM32F042 (bxCAN + STM32 HAL).
 * This header pairs with can_module.c and exposes a small surface area:
 *  - Initialize CAN with selectable baud
 *  - Configure hardware filters for a list of Standard IDs
 *  - Send a Standard ID data frame
 *  - Receive a Standard ID data frame
 *  - Store and access a CAN node ID
 *  - Update baud rate at runtime (re-init + reapply filters)
 *
 * Notes:
 *  - No application business logic is included here.
 *  - Only Standard (11-bit) identifiers are supported by this API.
 *  - GPIO pins, clocks, and NVIC setup should be handled elsewhere.
 */

#ifndef CAN_MODULE_H
#define CAN_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"
#include <stdint.h>
#include <stddef.h>

/* Optional compile-time overrides and constants. */

/* If you want to cap how many Standard IDs can be installed across filters,
 * define CAN_MODULE_MAX_FILTER_IDS before including this header.
 * The can_module.c provides a default of 32 if not defined.
 */
/* #define CAN_MODULE_MAX_FILTER_IDS 32u */

/* Select which RX FIFO to use (CAN_RX_FIFO0 or CAN_RX_FIFO1).
 * The can_module.c provides a default of CAN_RX_FIFO0 if not defined.
 */
/* #define CAN_MODULE_RX_FIFO CAN_RX_FIFO0 */

/* Baud rate selector values used by the module. */
#define CAN_MODULE_BAUD_125K   0u
#define CAN_MODULE_BAUD_250K   1u
#define CAN_MODULE_BAUD_500K   2u
#define CAN_MODULE_BAUD_1000K  3u

/* ===== Public API ===== */

/**
 * Initialize the CAN peripheral and start it.
 *
 * Performs HAL CAN init, applies a permissive filter (accept all) by default,
 * and starts the peripheral. Stores the provided node_id internally.
 *
 * Parameters:
 *  - hcan: Pointer to an initialized CAN handle (GPIO and clocks should be ready).
 *  - node_id: Device node identifier to store in the module.
 *  - baud_enum: One of CAN_MODULE_BAUD_xxx constants.
 *
 * Returns:
 *  - HAL_OK on success, otherwise a HAL error code.
 */
HAL_StatusTypeDef CAN_Module_Init(CAN_HandleTypeDef *hcan, uint32_t baud_enum);

/**
 * Set the stored CAN node identifier.
 *
 * Parameters:
 *  - node_id: New node identifier to store.
 */
void CAN_Module_Set_Node_Id(uint8_t node_id);

/**
 * Get the stored CAN node identifier.
 *
 * Returns:
 *  - Current node identifier stored by the module.
 */
uint8_t CAN_Module_Get_Node_Id(void);

/**
 * Update the CAN baud rate at runtime.
 *
 * Stops CAN, reconfigures bit timing based on baud_enum, re-initializes HAL CAN,
 * reapplies any previously configured Standard ID filters, and restarts CAN.
 *
 * Parameters:
 *  - baud_enum: One of CAN_MODULE_BAUD_xxx constants.
 *
 * Returns:
 *  - HAL_OK on success, otherwise a HAL error code.
 */
HAL_StatusTypeDef CAN_Module_Update_Baud(uint32_t baud_enum);

/**
 * Configure hardware filters for a list of Standard IDs.
 *
 * Copies the provided list into internal storage and programs the filter banks
 * using 16-bit IDLIST mode (up to 4 IDs per bank). If id_count is zero or
 * id_list is NULL, an accept-all filter is applied.
 *
 * Parameters:
 *  - id_list: Pointer to an array of 11-bit Standard IDs.
 *  - id_count: Number of IDs in id_list.
 *
 * Returns:
 *  - HAL_OK on success, otherwise a HAL error code.
 */
HAL_StatusTypeDef CAN_Module_Update_StdId_Filters(const uint16_t *id_list, size_t id_count);

/**
 * Send a Standard ID data frame.
 *
 * Parameters:
 *  - std_id: 11-bit Standard ID (lower 11 bits are used).
 *  - data: Pointer to payload bytes (up to 8 bytes).
 *  - dlc: Data length code (0..8).
 *  - timeout_ms: Timeout in milliseconds to wait for a free TX mailbox.
 *
 * Returns:
 *  - HAL_OK on success, HAL_TIMEOUT if no mailbox freed in time, or other HAL error.
 */
HAL_StatusTypeDef CAN_Module_Send_Std(uint16_t std_id, const uint8_t *data, uint8_t dlc, uint32_t timeout_ms);

/**
 * Receive a Standard ID data frame.
 *
 * Polls the configured RX FIFO until timeout. On success, fills out parameters
 * with ID, DLC, and copies the payload bytes.
 *
 * Parameters:
 *  - std_id: Output pointer to receive the 11-bit Standard ID.
 *  - data: Output buffer for payload (must hold at least 8 bytes).
 *  - dlc: Output pointer to receive the DLC.
 *  - timeout_ms: Timeout in milliseconds to wait for a frame.
 *
 * Returns:
 *  - HAL_OK on success, HAL_TIMEOUT if no frame arrived, or other HAL error.
 */
HAL_StatusTypeDef CAN_Module_Receive_Std(uint16_t *std_id, uint8_t *data, uint8_t *dlc, uint32_t timeout_ms);

/**
 * Get the last configured baud selector value.
 *
 * Returns:
 *  - One of CAN_MODULE_BAUD_xxx constants.
 */
uint32_t CAN_Module_Get_Baud_Enum(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_MODULE_H */
