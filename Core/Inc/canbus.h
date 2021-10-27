/*
 * canbus.h
 * STM32 HAL FDCAN wrapper library to make CAN programming a bit more sane.
 * How to use:
 * * Enable an FDCAN module in the IOC configuration
 *
 * For simplicity, this removes some features from the HAL driver:
 * * Only classic frames are supported, not FD frames (limiting the maximum length to 8 bytes).
 * * Only data frames can be sent, not remote frames.
 * * Extended CAN IDs are not supported (for now)
 *
 *  Created on: Oct 24, 2021
 *      Author: aidan
 */

#ifndef INC_CANBUS_H_
#define INC_CANBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "stm32g0xx_hal.h"

// valid data lengths for classic (non-FD) CAN
#define CANBUS_LEN_0B FDCAN_DLC_BYTES_0
#define CANBUS_LEN_1B FDCAN_DLC_BYTES_1
#define CANBUS_LEN_2B FDCAN_DLC_BYTES_2
#define CANBUS_LEN_3B FDCAN_DLC_BYTES_3
#define CANBUS_LEN_4B FDCAN_DLC_BYTES_4
#define CANBUS_LEN_5B FDCAN_DLC_BYTES_5
#define CANBUS_LEN_6B FDCAN_DLC_BYTES_6
#define CANBUS_LEN_7B FDCAN_DLC_BYTES_7
#define CANBUS_LEN_8B FDCAN_DLC_BYTES_8

// CANBus error codes

// CANBus filter modes
#define CANBUS_FILTER_TO_FIFO0    FDCAN_FILTER_TO_RXFIFO0
#define CANBUS_FILTER_TO_FIFO1    FDCAN_FILTER_TO_RXFIFO1
#define CANBUS_FILTER_REJECT      FDCAN_FILTER_REJECT
#define CANBUS_FILTER_HP_TO_FIFO0 FDCAN_FILTER_TO_RXFIFO0_HP
#define CANBUS_FILTER_HP_TO_FIFO1 FDCAN_FILTER_TO_RXFIFO1_HP

// CANBus filter types
#define CANBUS_FILTER_1_TO_2         FDCAN_FILTER_RANGE
#define CANBUS_FILTER_1_OR_2         FDCAN_FILTER_DUAL
#define CANBUS_FILTER_1_MASK_2       FDCAN_FILTER_MASK
#define CANBUS_FILTER_1_TO_2_NO_EIDM FDCAN_FILTER_RANGE_NO_EIDM

// CANBus non-matching frame behavior
#define CANBUS_NONMATCH_TO_FIFO0 FDCAN_ACCEPT_IN_RX_FIFO0
#define CANBUS_NONMATCH_TO_FIFO1 FDCAN_ACCEPT_IN_RX_FIFO1
#define CANBUS_NONMATCH_REJECT   FDCAN_REJECT

// RX callback
typedef void (*CANBus_RXCallback)(FDCAN_HandleTypeDef *, uint32_t);

/*
 * Structure containing the internal state of the CAN bus
 * Do not touch any of these values; use the proper CANBus
 * functions to change/read them.
 */
typedef struct CANBus
{
  FDCAN_HandleTypeDef *__fdcan;
  uint32_t __identifier;
  uint32_t __next_filter;
  int __error;
} CANBus;

/*
 * Structure for an individual CAN message (might be unnecessary)
 */
typedef struct CANBus_TxFrame
{
  FDCAN_TxHeaderTypeDef __header;
  uint8_t *__data;
} CANBus_TxFrame;


/*
 * Initialize the CAN bus internal structures, but don't start
 * transmitting/receiving data.
 * Returns 0 on success and 1 on failure. If successful, clears the error state
 * of the (uninitialized) CANBus.
 */
int CANBus_Init(CANBus *can, FDCAN_HandleTypeDef *fdcan, uint32_t identifier);

/*
 * Get the error state of the CANBus. Returns 0 if there is no error;
 * returns one of CANBUS_ERR_* if there is an error.
 */
int CANBus_GetError(const CANBus *can);

/*
 * Add a filter to the list. There must be space to add this filter
 * (defined by StdFiltersNbr in the FDCAN IOC configuration), otherwise this function will fail.
 * Returns 0 on success; returns 1 if the filter list is full.
 */
int CANBus_AddFilter(CANBus *can, uint32_t filt_type, uint32_t filt_mode, uint32_t filt_id1, uint32_t filt_id2);

/*
 * Set the filtering mode for all messages to handle receiving non-matching and remote messages.
 */
int CANBus_SetFilterMode(CANBus *can, uint32_t nonmatch_mode, int reject_remote);

// TODO: function to set callbacks

/*
 * Attempt to start the CAN bus, allowing transmission and reception of messages.
 * If this fails, the CANBus's error state is set.
 * Returns 0 on success and 1 on failure.
 */
int CANBus_Start(CANBus *can);

// TODO: add functions to manipulate/remove existing filters if necessary

/*
 * Attempt to send a message on the CAN bus. There *must* be as many bytes in the data buffer as are indicated
 * by the length_code; otherwise, this function will read past the end of the buffer.
 *
 * If a fatal error occurs when sending the data, the error state of the CANBus will be set.
 * This function will return 0 on success and 1 if any error occurs.
 */
int CANBus_SendMsg(CANBus *can, uint32_t length_code, uint8_t *data);

// TODO: function to receive messages
// TODO: stop function (if necessary)

// TODO: example code/documentation

#ifdef __cplusplus
}
#endif

#endif /* INC_CANBUS_H_ */
