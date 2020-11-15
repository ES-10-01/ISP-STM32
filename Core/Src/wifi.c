/*
 * wifi.c
 *
 *  Created on: Oct 18, 2020
 *      Author: demavag
 */
#include "wifi.h"
#include "string.h"

#define COMMAND_LENGTH(command) (sizeof(command)/sizeof(uint8_t))

uint8_t * AT = (uint8_t *)"AT";
uint8_t * AT_ECHO_STOP = (uint8_t *)"ATE0";
uint8_t * AT_RESET = (uint8_t *)"AT+RST\r\n";
uint8_t * AT_CHANGE_MODE_TEMPL = (uint8_t *)"AT+UART_CUR?";
uint8_t * AT_CONNECT = (uint8_t *)"AT+CWJAP=\"%s\",\"%s\"\r\n";
uint8_t * AT_delete = (uint8_t *)"\x08 \x08";
uint8_t * AT_CMD_END = (uint8_t *)"\r\n";

uint8_t recieve_buff[128];
volatile uint8_t tmp_buf = 0;

int8_t check_esp_available(UART_HandleTypeDef * transmit_huart, UART_HandleTypeDef * receive_huart)
{
	for (int i = 0; i < strlen(AT); i++) {
		if (HAL_UART_Transmit(receive_huart,AT+i,1,100) == HAL_ERROR) {
			return -1;
		}
	}
	if (HAL_UART_Transmit(receive_huart,AT_CMD_END,COMMAND_LENGTH(AT_CMD_END),100) == HAL_ERROR) {
		return -1;
	}

	uint8_t recieve_started = 0;
	uint8_t * buf_it = recieve_buff;

	while (tmp_buf != 0 || !recieve_started) {
		tmp_buf = 0;
		recieve_started = 1;
		if (HAL_UART_Receive(receive_huart, &tmp_buf, 1, 5) == HAL_ERROR) {
			return -1;
		}

		if (tmp_buf == '\r' || tmp_buf == '\n') {
			continue;
		}

		*buf_it = tmp_buf;

		if (tmp_buf) buf_it++;
	}

	if (recieve_buff[0] == 'O'
		&& recieve_buff[1] == 'K') {
		return 0;
	}

	return -1;
}
