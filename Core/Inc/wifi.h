/*
 * wifi.h
 *
 *  Created on: Oct 18, 2020
 *      Author: demavag
 */

#ifndef INC_WIFI_H_
#define INC_WIFI_H_

#include <stdint.h>
#if defined(TEST)
#include "stm32_hal_mock.h"
#else
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_uart.h"
#endif

#define uart_ptr UART_HandleTypeDef *

enum GosServerCommands
{
	GOS_OPEN_CMD,
	GOS_CLOSE_CMD,
	GOS_GET_CMD,
	UNKNOWN_CMD
};

int8_t check_esp_available(uart_ptr huart);

int8_t init_esp(uart_ptr huart);

int8_t connect_to_network(uart_ptr huart, const char * name, const char * password);
int8_t connect_to_server(uart_ptr huart, const char * address, uint16_t port);

int8_t send_hello(uart_ptr huart, const char * id);
int8_t send_password(uart_ptr huart, const char * password);
int8_t send_locked(uart_ptr huart);

enum GosServerCommands recieve_command(uart_ptr huart);

#endif /* INC_WIFI_H_ */
