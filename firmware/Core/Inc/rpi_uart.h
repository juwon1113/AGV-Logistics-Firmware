/*
 * rpi_uart.h
 *
 *  Created on: 2026. 5. 24.
 *      Author: juwon
 */

#ifndef INC_RPI_UART_H_
#define INC_RPI_UART_H_

#include "main.h"
#include "stm32f7xx_hal.h"
#include "stdint.h"

typedef enum
{
	STATE_MOVE = 0x01,
	STATE_READY = 0x02
} AGV_State_t;

typedef enum
{
	CMD_FORWARD = 0x01,
	CMD_STOP = 0x02,
	CMD_LIFT_UP = 0x03,
	CMD_LIFT_DOWN = 0x04,
	CMD_ROTATE_LEFT = 0x05,
	CMD_ROTATE_RIGHT = 0x06,
	CMD_ROTATE_180 = 0x07
} AGV_Command_t;

typedef enum
{
	EVT_DONE = 0x81,
	EVT_ACK = 0xFF
} AGV_Event_t;

typedef struct
{
	uint8_t command;
	int x_offset;
	int y_offset;
	int yaw_offset;
} RPI_Data_t;

#define RPI_RxBuf_SIZE	21

int Parse_ArUcoMarker_Data(const uint8_t* buf);

void Read_RPI_Data(RPI_Data_t* data, const uint8_t* buf);

void Send_Event(UART_HandleTypeDef* huart, AGV_Event_t evt);

void Reset_RPI_Data(RPI_Data_t* data);

#endif /* INC_RPI_UART_H_ */
