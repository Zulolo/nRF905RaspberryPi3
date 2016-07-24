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
#include <sys/time.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "nRF905_d.h"

#define ACK_TASK_INTERVAL_US				100000
#define NRF905_SERVER_PORT					701
#define PID_EMPTY							0
#define MAX_CONNECTION_PENDING 				8    /* Max connection requests */
#define RECEIVE_BUFFER_LENGTH				256

pid_t tClientPid[MAX_CONNECTION_PENDING] = {0, };

int32_t nWriteDataToNRF905Pipe(int32_t nTaskPipeFD, nRF905CommTask_t* pNRF905CommTask, uint8_t* pACK_Payload)
{
	uint8_t unBuffer[sizeof(nRF905CommTask_t) + NRF905_TX_PAYLOAD_LEN];
	memcpy(unBuffer, pNRF905CommTask, sizeof(nRF905CommTask_t));
	memcpy(unBuffer + sizeof(nRF905CommTask_t), pACK_Payload, NRF905_TX_PAYLOAD_LEN);
	return write(nTaskPipeFD, unBuffer, sizeof(nRF905CommTask_t) + NRF905_TX_PAYLOAD_LEN);
}

void* ackRoutine(void* pTaskPipeFD)
{
	nRF905CommTask_t tNRF905CommTask;
	static uint8_t unACK_Payload[NRF905_TX_PAYLOAD_LEN] = {0xA5, 0xA5, 0xDC, 0xCD,
			0x01, 0x02, 0x03, 0x04,
			0x05, 0x06, 0x07, 0x08,
			0x09, 0x0A, 0x0B, 0x0C};

	tNRF905CommTask.tCommType = NRF905_COMM_TYPE_TX_PKG;
	tNRF905CommTask.unCommByteNum = NRF905_TX_PAYLOAD_LEN;

	while (NRF905_FALSE == unNeedtoClose){
		if (nWriteDataToNRF905Pipe(*pTaskPipeFD, &tNRF905CommTask, unACK_Payload) < 0 ){
			printf("Write task communication payload to pipe error with code:%d", errno);
			NRF905D_LOG_ERR("Write task communication payload to pipe error with code:%d", errno);
		}
		usleep(ACK_TASK_INTERVAL_US);
	}

	pthread_exit(NULL);
}

int32_t nRecordPid(pid_t tChildPid, pid_t* pClientPid, uint32_t unMaxArrayLength){
	uint32_t unIndex;
	for (unIndex = 0; unIndex < unMaxArrayLength; unIndex++){
		if (PID_EMPTY == *(pClientPid + unIndex)){
			*(pClientPid + unIndex) = tChildPid;
			return 0;
		}
	}
	return -1;
}

int32_t nClearPid(pid_t tChildPid, pid_t* pClientPid, uint32_t unMaxArrayLength){
	uint32_t unIndex;
	for (unIndex = 0; unIndex < unMaxArrayLength; unIndex++){
		if (tChildPid == *(pClientPid + unIndex)){
			*(pClientPid + unIndex) = PID_EMPTY;
			return 0;
		}
	}
	return -1;
}

int32_t nKillAllChild(pid_t* pClientPid, uint32_t unMaxArrayLength){
	uint32_t unIndex;
	for (unIndex = 0; unIndex < unMaxArrayLength; unIndex++){
		if (PID_EMPTY != *(pClientPid + unIndex)){
			kill(*(pClientPid + unIndex), SIGINT);
		}
	}
	return 0;
}

int32_t nWaitAllChild(pid_t* pClientPid, uint32_t unMaxArrayLength){
	uint32_t unIndex;
	for (unIndex = 0; unIndex < unMaxArrayLength; unIndex++){
		if (PID_EMPTY != *(pClientPid + unIndex)){
			sleep(5);
		}
	}
	return 0;
}

void sigCHLD_Handler(int32_t signum, siginfo_t *info, void *ptr)
{
	pid_t tChildPid;
	tChildPid = waitpid(-1, NULL, WNOHANG);
	if (tChildPid > 0){
		nClearPid(tChildPid, tClientPid, MAX_CONNECTION_PENDING);
	}
}

void clientHandler(int32_t nClientSock, int32_t nTaskPipeFD)
{
	int32_t nReceivedCNT;
	nRF905CommTask_t tNRF905CommTask;
	uint8_t unACK_Payload[NRF905_TX_PAYLOAD_LEN];

    /* Receive message */
    if ((nReceivedCNT = recv(nClientSock, &tNRF905CommTask, sizeof(nRF905CommTask_t), 0)) < 0) {
    	NRF905D_LOG_ERR("Failed to receive task data from client.");
    	close(nClientSock);
    	exit(-1);
    }
    if ((nReceivedCNT = recv(nClientSock, unACK_Payload, NRF905_TX_PAYLOAD_LEN, 0)) < 0) {
    	NRF905D_LOG_ERR("Failed to receive payload data from client.");
    	close(nClientSock);
    	exit(-1);
    }
    while (NRF905_FALSE == unNeedtoClose) {
    	if (nWriteDataToNRF905Pipe(nTaskPipeFD, &tNRF905CommTask, unACK_Payload) < 0){
			printf("Write task communication payload to pipe error with code:%d", errno);
			NRF905D_LOG_ERR("Write task communication payload to pipe error with code:%d", errno);
        	close(nClientSock);
        	exit(-1);
    	}
        /* Receive message */
        if ((nReceivedCNT = recv(nClientSock, &tNRF905CommTask, sizeof(nRF905CommTask_t), 0)) < 0) {
        	NRF905D_LOG_ERR("Failed to receive task data from client.");
        	close(nClientSock);
        	exit(-1);
        }
        if ((nReceivedCNT = recv(nClientSock, unACK_Payload, NRF905_TX_PAYLOAD_LEN, 0)) < 0) {
        	NRF905D_LOG_ERR("Failed to receive payload data from client.");
        	close(nClientSock);
        	exit(-1);
        }
    }
	close(nClientSock);
	exit(0);
}

int32_t pNRF905Server(int32_t nTaskPipeFD)
{
    pthread_t tACK_Thread;
    pthread_attr_t tACK_ThreadAttr;
    int32_t nServerSock, nClientSock;
    struct sockaddr_in tEchoServer, tEchoClient;
    uint32_t unClientLen = sizeof(tEchoClient);
    pid_t tChildPid;
	struct sigaction tSignalAction;

	NRF905D_LOG_INFO("nRoutine work thread start.");
	memset(tClientPid, 0, sizeof(tClientPid));

    tSignalAction.sa_sigaction = sigCHLD_Handler;
    tSignalAction.sa_flags = SA_SIGINFO;
    sigaction(SIGCHLD, &tSignalAction, NULL);

	pthread_attr_init(&tACK_ThreadAttr);
	pthread_create(&tACK_Thread, &tACK_ThreadAttr, ackRoutine, (void *)(&nTaskPipeFD));

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
    	NRF905D_LOG_ERR("Failed to bind the server socket");
    	exit(-1);
    }
    /* Listen on the server socket */
    if (listen(nServerSock, MAX_CONNECTION_PENDING) < 0) {
    	NRF905D_LOG_ERR("Failed to listen on server socket");
    	exit(-1);
    }

    /* Run until cancelled */
	while (NRF905_FALSE == unNeedtoClose) {
		/* Wait for client connection */
		if ((nClientSock = accept(nServerSock, (struct sockaddr *)(&tEchoClient), &unClientLen)) < 0) {
			NRF905D_LOG_ERR("Failed to accept client connection");
			break;
		}
		NRF905D_LOG_INFO("Client connected: %s", inet_ntoa(tEchoClient.sin_addr));
		tChildPid = fork();
		if (tChildPid < 0){
			NRF905D_LOG_ERR("Fork failed.");
		}else if (0 == tChildPid){
			// Child process
			clientHandler(nClientSock, nTaskPipeFD);
		}else{
			// Parent process
			nRecordPid(tChildPid, tClientPid, MAX_CONNECTION_PENDING);
		}
	}

	nKillAllChild(tClientPid, MAX_CONNECTION_PENDING);
	pthread_join(tACK_Thread, NULL);
	nWaitAllChild(tClientPid, MAX_CONNECTION_PENDING);
	close(nServerSock);
	close(nTaskPipeFD);
	NRF905D_LOG_INFO("Routine work thread exit.");
	exit(0);
}

