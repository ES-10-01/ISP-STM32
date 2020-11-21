#include "tcp_server_mock.h"

#include "utils.h"

#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>

int listenfd = 0;
int connfd = 0;
int connections  = 0;

struct sockaddr_in serv_addr;

int server_init(int port)
{
	listenfd = socket(AF_INET, SOCK_STREAM, 0);

	if (RC_FAIL(listenfd)) {
		printf("socket() failed\n");
		return -1;
	}

	memset(&serv_addr, '0', sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(port);

	if (RC_FAIL(bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)))) {
		printf("bind() failed\n");
		return -1;
	}

	if (RC_FAIL(listen(listenfd, 10))) {
		printf("listen() failed\n");
		return -1;
	}

	return 0;
}

int server_has_connection()
{
	return connections;
}

int server_wait_for_connection()
{
	connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);

	if (RC_FAIL(connfd)) {
		return -1;
	}

	connections++;

	return 0;
}

int server_send_message(char * message)
{
	if (RC_FAIL(write(connfd, message, strlen(message)))) {
		return -1;
	}

	return 0;
}

int server_recieve_message(char * buffer)
{
	if (RC_FAIL(read(connfd, buffer, 128))) {
		return -1;
	}

	return 0;
}

int server_close()
{
	if (RC_FAIL(close(connfd))) {
		return -1;
	}

	if (RC_FAIL(close(listenfd))) {
		return -1;
	}

	connections--;

	return 0;
}
