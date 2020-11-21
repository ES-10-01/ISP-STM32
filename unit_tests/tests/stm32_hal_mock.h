#ifndef _HAL_MOCK_
#define _HAL_MOCK_

#include <stdint.h>

typedef struct UART_Handle
{
	int esp_fd;
} UART_HandleTypeDef;

typedef enum
{
	HAL_OK       = 0x00U,
	HAL_ERROR    = 0x01U,
	HAL_BUSY     = 0x02U,
	HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

int HAL_MOCK_init(const char * comport_name, UART_HandleTypeDef * uart_mock);
int HAL_MOCK_flush(UART_HandleTypeDef * uart_mock);

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
#endif
