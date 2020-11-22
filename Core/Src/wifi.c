/*
 * wifi.c
 *
 *  Created on: Oct 18, 2020
 *      Author: demavag
 */

#include "string.h"
#include "utils.h"
#include "wifi.h"

#include <stdio.h>

char tmp_command_buffer[255];
char tmp_message_buffer[255];

char recieve_buffer[255];

uint8_t * AT = (uint8_t *)"AT";
uint8_t * AT_ECHO_STOP = (uint8_t *)"ATE0";
uint8_t * AT_RESET = (uint8_t *)"AT+RST\r\n";
uint8_t * AT_CHANGE_MODE = (uint8_t *)"AT+CWMODE=1";
uint8_t * AT_NETWORK_CONNECT = (uint8_t *)"AT+CWJAP=\"%s\",\"%s\"";	// AT+CWJAP="<network name>","<password>"
uint8_t * AT_TCP_CONNECT = (uint8_t *)"AT+CIPSTART=\"TCP\",\"%s\",%d";		// AT+CIPSTART=TCP,<address>,<port>
uint8_t * AT_TCP_SEND = (uint8_t *)"AT+CIPSEND=%d";					// AT+CIPSEND-<length>; in bytes
uint8_t * AT_TCP_CLOSE = (uint8_t *)"AT+CIPCLOSE";
uint8_t * AT_GET_NETWORK_ADDRESS = (uint8_t *)"AT+CIFSR";
uint8_t * AT_CMD_END = (uint8_t *)"\r\n";

uint8_t * GOS_HELLO = (uint8_t *)"GOS_HELLO=%s,%s";		// GOS_HELLO=<address>,<id>
uint8_t * GOS_PASSWORD = (uint8_t *)"GOS_PASS=%s";	// GOS_PASSWORD=<pin password>

uint8_t * GOS_OPEN = (uint8_t *)"GOS_OPEN";
uint8_t * GOS_CLOSE = (uint8_t *)"GOS_CLOSE";
uint8_t * GOS_GET = (uint8_t *)"GOS_GET";

uint8_t send(uart_ptr huart, uint8_t * message)
{
	for (size_t i = 0; i < strlen((char *)message); i++) {
		if (HAL_UART_Transmit(huart,message+i,1,100) == HAL_ERROR) {
			return -1;
		}
	}

	return 0;
}

uint8_t send_at_command(uart_ptr huart, uint8_t * command)
{
	if (RC_FAIL(send(huart, command))) {
		return -1;
	}
	if (HAL_UART_Transmit(huart,AT_CMD_END,strlen(AT_CMD_END),100) == HAL_ERROR) {
		return -1;
	}

	return 0;
}

uint8_t recieve_at_response(uart_ptr huart, uint8_t * buffer, uint8_t exit_on_timeout)
{
	uint8_t tmp_buf = 0;
	uint8_t recieve_started = 0;
	uint8_t * buf_it = buffer;

	while (tmp_buf != 0 || !recieve_started) {
		tmp_buf = 0;
		recieve_started = 1;
		HAL_StatusTypeDef rc = HAL_UART_Receive(huart, &tmp_buf, 1, 5);
		if (rc == HAL_ERROR || (rc == HAL_TIMEOUT && exit_on_timeout)) {
			return -1;
		}

		if (tmp_buf == '\r' || tmp_buf == '\n') {
			continue;
		}

		*buf_it = tmp_buf;

		if (tmp_buf) {
			buf_it++;
		}
	}

	return 0;
}

uint8_t send_at_and_check_response(uart_ptr huart, uint8_t * command)
{
	memset(recieve_buffer, 0, sizeof(recieve_buffer));

	if (RC_FAIL(send_at_command(huart, command))) {
		return -1;
	}

	if (RC_FAIL(recieve_at_response(huart, recieve_buffer, 0))) {
		return -1;
	}

	if (strstr(recieve_buffer, "OK") == NULL) {
		return -1;
	}

	return 0;
}

int8_t check_esp_available(uart_ptr huart)
{
	if (RC_FAIL(send_at_and_check_response(huart, AT))) {
		return -1;
	}

	return 0;
}

int8_t init_esp(uart_ptr huart)
{
	if (RC_FAIL(send_at_and_check_response(huart, AT_ECHO_STOP))) {
		return -1;
	}

	if (RC_FAIL(send_at_and_check_response(huart, AT_CHANGE_MODE))) {
		return -1;
	}

	return 0;
}

int8_t connect_to_network(uart_ptr huart, const char * name, const char * password)
{
	memset(tmp_command_buffer, 0, sizeof(tmp_command_buffer));

	if (RC_FAIL(sprintf(tmp_command_buffer, AT_NETWORK_CONNECT, name, password))) {
		return -1;
	}

	if (RC_FAIL(send_at_and_check_response(huart, tmp_command_buffer))) {
		return -1;
	}

	return 0;
}

int8_t connect_to_server(uart_ptr huart, const char * address, uint16_t port)
{
	memset(tmp_command_buffer, 0, sizeof(tmp_command_buffer));

	if (RC_FAIL(sprintf(tmp_command_buffer, AT_TCP_CONNECT, address, port))) {
		return -1;
	}

	if (RC_FAIL(send_at_and_check_response(huart, tmp_command_buffer))) {
		return -1;
	}

	return 0;
}

uint8_t send_message(uart_ptr huart, const char * message)
{
	memset(tmp_command_buffer, 0, sizeof(tmp_command_buffer));
	memset(recieve_buffer, 0, sizeof(recieve_buffer));

	if (RC_FAIL(sprintf(tmp_command_buffer, AT_TCP_SEND, strlen(message)))) {
		return -1;
	}

	if (RC_FAIL(send_at_command(huart, tmp_command_buffer))) {
		return -1;
	}

	if (RC_FAIL(recieve_at_response(huart, recieve_buffer, 0))) {
		return -1;
	}

	if (strstr(recieve_buffer, ">") == NULL) {
		return -1;
	}

	if (RC_FAIL(send(huart, message))) {
		return -1;
	}

	memset(recieve_buffer, 0, sizeof(recieve_buffer));

	if (RC_FAIL(recieve_at_response(huart, recieve_buffer, 0))) {
		return -1;
	}

	if (strstr(recieve_buffer, "SEND OK") == NULL) {
		return -1;
	}

	return 0;
}

int8_t send_hello(uart_ptr huart, const char * id)
{
	memset(recieve_buffer, 0, sizeof(recieve_buffer));
	memset(tmp_message_buffer, 0, sizeof(tmp_message_buffer));

	if (RC_FAIL(send_at_command(huart, AT_GET_NETWORK_ADDRESS))) {
		return -1;
	}

	if (RC_FAIL(recieve_at_response(huart, recieve_buffer, 0))) {
		return -1;
	}

	// ESP response format: +CIFSR:STAIP,"<ip address>"
	char * ip_address_start = strstr(recieve_buffer, "\"");
	ip_address_start++;// move pointer to the next char after \"
	*(strstr(ip_address_start, "\"")) = '\0'; // set null terminator after end of ip address

	if (RC_FAIL(sprintf(tmp_message_buffer, GOS_HELLO, ip_address_start, id))) {
		return -1;
	}

	if (RC_FAIL(send_message(huart, tmp_message_buffer))) {
		return -1;
	}

	return 0;
}

int8_t send_password(uart_ptr huart, const char * password)
{
	memset(tmp_message_buffer, 0, sizeof(tmp_message_buffer));

	if (RC_FAIL(sprintf(tmp_message_buffer, GOS_PASSWORD, password))) {
		return -1;
	}

	if (RC_FAIL(send_message(huart, tmp_message_buffer))) {
		return -1;
	}

	return 0;
}

enum GosServerCommands recieve_command(uart_ptr huart)
{
	memset(recieve_buffer, 0, sizeof(recieve_buffer));

	if (RC_FAIL(recieve_at_response(huart, recieve_buffer, 1))) {
		return UNKNOWN_CMD;
	}

	// ESP response format: +IPD,<size>:<message>
	char * message_start = strstr(recieve_buffer, ":");
	message_start++;

	if (strcmp(GOS_OPEN, message_start) == 0) {
		return GOS_OPEN_CMD;
	}

	if (strcmp(GOS_CLOSE, message_start) == 0) {
		return GOS_CLOSE_CMD;
	}

	if (strcmp(GOS_GET, message_start) == 0) {
		return GOS_GET_CMD;
	}

	return UNKNOWN_CMD;
}
