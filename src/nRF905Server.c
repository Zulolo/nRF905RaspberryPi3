/*
 * nRF905Server.c
 *
 *  Created on: Jul 24, 2016
 *      Author: zulolo
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <wait.h>
#include <sys/time.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <stddef.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "nRF905_d.h"


#define NRF905_SERVER_PORT					701




int32_t nWriteDataToNRF905Pipe(int32_t nTaskPipeFD, nRF905CommTask_t* pNRF905CommTask, uint8_t* pACK_Payload)
{
	uint8_t unBuffer[sizeof(nRF905CommTask_t) + NRF905_TX_PAYLOAD_LEN];
	memcpy(unBuffer, pNRF905CommTask, sizeof(nRF905CommTask_t));
	memcpy(unBuffer + sizeof(nRF905CommTask_t), pACK_Payload, NRF905_TX_PAYLOAD_LEN);
	return write(nTaskPipeFD, unBuffer, sizeof(nRF905CommTask_t) + NRF905_TX_PAYLOAD_LEN);
}




int32_t pNRF905Server(int32_t nTaskPipeFD)
{
    pthread_t tACK_Thread;
    pthread_attr_t tACK_ThreadAttr;
    int32_t nServerSock, nClientSock;
    struct sockaddr_in tEchoServer, tEchoClient;
    uint32_t unClientLen = sizeof(tEchoClient);

	struct sigaction tSignalAction;

	NRF905D_LOG_INFO("nRoutine work thread start.");






	/* Create the TCP socket */
	if ((nServerSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
		NRF905D_LOG_ERR("Failed to create socket");
	}
	/* Construct the server sockaddr_in structure */
	memset(&tEchoServer, 0, sizeof(tEchoServer));       /* Clear struct */
	tEchoServer.sin_family = AF_INET;                  /* Internet/IP */
	tEchoServer.sin_addr.s_addr = htonl(INADDR_ANY);   /* Incoming addr */
	tEchoServer.sin_port = NRF905_SERVER_PORT;       /* server port */

    /* Bind the server socket */
    if (bind(nServerSock, (struct sockaddr *)(&tEchoServer), sizeof(tEchoServer)) < 0) {
    	NRF905D_LOG_ERR("Failed to bind the server socket with code:%d", errno);
    	exit(-1);
    }
    /* Listen on the server socket */
    if (listen(nServerSock, MAX_CONNECTION_PENDING) < 0) {
    	NRF905D_LOG_ERR("Failed to listen on server socket with code:%d", errno);
    	exit(-1);
    }







	close(nServerSock);
	close(nTaskPipeFD);
	NRF905D_LOG_INFO("Routine work thread exit.");
	exit(0);
}

