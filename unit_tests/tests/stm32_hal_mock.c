#include "stm32_hal_mock.h"

#include "utils.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <stdint.h>
#include <stdio.h>

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
        	printf("tcgetattr error\n");
            return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
        	printf("tcsetattr error\n");
        	return -1;
        }
        return 0;
}

int set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0) {
        	printf("tcgetattr error\n");
        	return -1;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 50;            // 5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        	printf("tcsetattr error\n");
        	return -1;
        }
        return 0;
}

int HAL_MOCK_init(const char * comport_name, UART_HandleTypeDef * uart_mock)
{
	uart_mock->esp_fd = open (comport_name, O_RDWR | O_NOCTTY | O_SYNC);

	if (uart_mock->esp_fd < 0) {
		printf("open error\n");
		return -1;
	}

	set_interface_attribs (uart_mock->esp_fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (uart_mock->esp_fd, 0);

	return 0;
}

int HAL_MOCK_flush(UART_HandleTypeDef * uart_mock)
{
	if (RC_FAIL(tcflush(uart_mock->esp_fd, TCIOFLUSH))) {
		return -1;
	}

	return 0;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	if (RC_FAIL(write(huart->esp_fd, (char *)pData, Size))) {
		return HAL_ERROR;
	}

	return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	if (RC_FAIL(read(huart->esp_fd, (char *)pData, Size))) {
		return HAL_ERROR;
	}

	return HAL_OK;
}
