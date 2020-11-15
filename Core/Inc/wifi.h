/*
 * wifi.h
 *
 *  Created on: Oct 18, 2020
 *      Author: demavag
 */

#ifndef INC_WIFI_H_
#define INC_WIFI_H_

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_uart.h"

int8_t check_esp_available(UART_HandleTypeDef * transmit_huart, UART_HandleTypeDef * receive_huart);


#endif /* INC_WIFI_H_ */
