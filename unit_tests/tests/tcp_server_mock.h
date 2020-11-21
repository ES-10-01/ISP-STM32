#ifndef _TCP_SERVER_MOCK_
#define _TCP_SERVER_MOCK_

int server_init(int port);

int server_has_connection();

int server_wait_for_connection();
int server_send_message(char * message);
int server_recieve_message(char * buffer);

int server_close();

#endif
