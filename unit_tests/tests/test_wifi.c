#include "wifi.h"

#include "stm32_hal_mock.h"
#include "tcp_server_mock.h"
#include "utils.h"

#include "unity.h"
#include "mock_rules.h"

#include "pthread.h"
#include <stdlib.h>
#include <string.h>

UART_HandleTypeDef uart_mock = { .esp_fd = 0 };

char * network_name = "OnePlus";
char * network_password = "123454321";

char * server_address = "192.168.43.231";
int server_port = 8888;

extern char tmp_command_buffer[255];
extern char tmp_message_buffer[255];

extern char recieve_buffer[255];

extern uint8_t send_at_command(uart_ptr huart, uint8_t * command);
extern uint8_t recieve_at_response(uart_ptr huart, uint8_t * buffer);

void suiteSetUp()
{
	TEST_ASSERT(RC_OK(HAL_MOCK_init("/dev/ttyUSB0", &uart_mock)));

	TEST_MESSAGE("UART mock initialized");

	TEST_ASSERT(RC_OK(server_init(server_port)));

	TEST_MESSAGE("Server mock initialized");

	TEST_PRINTF("Our address \"%s\":%d", server_address, server_port);
}

int suiteTearDown(int num_failures)
{
	server_close();

	return num_failures;
}

void setUp(void)
{
	TEST_ASSERT(RC_OK(HAL_MOCK_flush(&uart_mock)));
}

void tearDown(void)
{

}

void test_esp_available()
{
	TEST_ASSERT_MESSAGE(RC_OK(check_esp_available(&uart_mock)), recieve_buffer);
}

void test_esp_init()
{
	TEST_ASSERT(RC_OK(init_esp(&uart_mock)));

	// Echo should be turned off and wi-fi mode should be 1
	TEST_ASSERT(RC_OK(send_at_command(&uart_mock, "AT+CWMODE?")));

	char recieve_buffer[255];
	TEST_ASSERT(RC_OK(recieve_at_response(&uart_mock, recieve_buffer)));

	TEST_ASSERT(RC_OK(strncmp("+CWMODE:1OK", recieve_buffer,11)));
}

void test_connect_to_network()
{
	TEST_ASSERT(RC_OK(connect_to_network(&uart_mock, network_name, network_password)));
}

void test_connect_to_server()
{
	pthread_t thread;

	int server_rc = pthread_create( &thread, NULL, server_wait_for_connection, NULL);
	TEST_ASSERT(RC_OK(connect_to_server(&uart_mock, server_address, server_port)));

	//pthread_join( thread, NULL);

	TEST_ASSERT(RC_OK(server_rc));
	TEST_ASSERT_MESSAGE(server_has_connection(), tmp_command_buffer);
}

void test_send_hello()
{
	if (!server_has_connection()) {
		test_connect_to_server();
	}

	TEST_ASSERT(RC_OK(send_hello(&uart_mock, "tst_id")));

	char message[128];
	TEST_ASSERT(RC_OK(server_recieve_message(message)));
	TEST_ASSERT_NOT_NULL(strstr(message, "GOS_HELLO"));
}

void test_send_password()
{
	if (!server_has_connection()) {
		test_connect_to_server();
	}

	TEST_ASSERT(RC_OK(send_password(&uart_mock, "tst_password")));

	char message[128];
	TEST_ASSERT(RC_OK(server_recieve_message(message)));
	TEST_ASSERT_NOT_NULL(strstr(message, "GOS_PASS"));
}

void test_recieve_command()
{
	if (!server_has_connection()) {
		test_connect_to_server();
	}

	TEST_ASSERT(RC_OK(server_send_message("GOS_OPEN")));
	TEST_ASSERT_EQUAL(GOS_OPEN_CMD, recieve_command(&uart_mock));

	TEST_ASSERT(RC_OK(server_send_message("GOS_CLOSE")));
	TEST_ASSERT_EQUAL(GOS_CLOSE_CMD, recieve_command(&uart_mock));

	TEST_ASSERT(RC_OK(server_send_message("GOS_GET")));
	TEST_ASSERT_EQUAL(GOS_GET_CMD, recieve_command(&uart_mock));
}
