/*
 ============================================================================
 Name        : nRF905_d.c
 Author      : zulolo
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
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
#include <getopt.h>
#include <fcntl.h>
#include <semaphore.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stddef.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define __USED_NRF905_INTERNAL__
#include "nRF905_d.h"


extern int32_t pNRF905Server(int32_t nTaskPipeFD);

void sigINT_Handler(int32_t signum, siginfo_t *info, void *ptr)
{
	unNeedtoClose = NRF905_TRUE;
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

void sigCHLD_Handler(int32_t signum, siginfo_t *info, void *ptr)
{
	pid_t tChildPid;
	tChildPid = waitpid(-1, NULL, WNOHANG);
	if (tChildPid > 0){
		nClearPid(tChildPid, tClientPid, MAX_CONNECTION_PENDING);
	}
}

int32_t setNRF905Mode(nRF905Mode_t tNRF905Mode)
{
	static nRF905Mode_t tNRF905ModeGlobal = NRF905_MODE_PWR_DOWN;

	if (tNRF905Mode >= NRF905_MODE_MAX){
		NRF905D_LOG_ERR("tNRF905Mode error.");
		return (-1);
	}
	if (tNRF905Mode == tNRF905ModeGlobal){
		return 0;
	}

	if (-1 == GPIOWrite(NRF905_TX_EN_PIN, unNRF905MODE_PIN_LEVEL[tNRF905Mode][NRF905_TX_EN_PIN_POS])){
		NRF905D_LOG_ERR("Write GPIO pin %u failed.", NRF905_TX_EN_PIN);
		return (-1);
	}

	if (-1 == GPIOWrite(NRF905_TRX_CE_PIN, unNRF905MODE_PIN_LEVEL[tNRF905Mode][NRF905_TRX_CE_PIN_POS])){
		NRF905D_LOG_ERR("Write GPIO pin %u failed.", NRF905_TRX_CE_PIN);
		return (-1);
	}

	if (-1 == GPIOWrite(NRF905_PWR_UP_PIN, unNRF905MODE_PIN_LEVEL[tNRF905Mode][NRF905_PWR_UP_PIN_POS])){
		NRF905D_LOG_ERR("Write GPIO pin %u failed.", NRF905_PWR_UP_PIN);
		return (-1);
	}
	tNRF905ModeGlobal = tNRF905Mode;
	usleep(800);

	return 0;
}

static int32_t nRF905_SPI_WR(int32_t nRF905SPIfd, uint8_t unCMD, uint8_t* pTX_Frame, uint8_t unFrameLength)
{
	uint8_t unRF905_SPI_TX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
	uint8_t unRF905_SPI_RX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
	struct spi_ioc_transfer tSPI_Transfer;

	if (unFrameLength > NRF905_RX_PAYLOAD_LEN){
		NRF905D_LOG_ERR("The frame need to send to nRF905 is too long.");
		return (-1);
	}

	if (setNRF905Mode(NRF905_MODE_STD_BY) < 0){
		// printf("Set nRF905 mode failed.\n");
		NRF905D_LOG_ERR("Set nRF905 mode failed");
		return (-1);
	}

	memset(&tSPI_Transfer, 0, sizeof(struct spi_ioc_transfer));
	unRF905_SPI_TX_Frame[0] = unCMD;
	memcpy(unRF905_SPI_TX_Frame + 1, pTX_Frame, unFrameLength);
	tSPI_Transfer.tx_buf = (unsigned long)unRF905_SPI_TX_Frame;
	tSPI_Transfer.rx_buf = (unsigned long)unRF905_SPI_RX_Frame;
	tSPI_Transfer.len = unFrameLength + 1;
	tSPI_Transfer.delay_usecs = unSPI_Delay;
	tSPI_Transfer.speed_hz = unSPI_Speed;
	tSPI_Transfer.bits_per_word = unSPI_Bits;

	if (ioctl(nRF905SPIfd, SPI_IOC_MESSAGE(1), &tSPI_Transfer) < 0){
		// printf("Can't send spi message during SPI WR with error code %d. \n", errno);
		NRF905D_LOG_ERR("can't send spi message");
		return (-1);
	}
	return 0;
}

static uint8_t* pRF905_SPI_RD(int32_t nRF905SPIfd, uint8_t unCMD, uint8_t unFrameLength)
{
	uint8_t unRF905_SPI_TX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
	static 	uint8_t unRF905_SPI_RX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
	struct spi_ioc_transfer tSPI_Transfer;

	if (unFrameLength > NRF905_RX_PAYLOAD_LEN){
		NRF905D_LOG_ERR("The frame need to read from nRF905 is too long.");
		return NULL;
	}

	if (setNRF905Mode(NRF905_MODE_STD_BY) < 0){
		NRF905D_LOG_ERR("Set nRF905 mode failed");
		return NULL;
	}

	memset(&tSPI_Transfer, 0, sizeof(struct spi_ioc_transfer));
	unRF905_SPI_TX_Frame[0] = unCMD;
	tSPI_Transfer.tx_buf = (unsigned long)unRF905_SPI_TX_Frame;
	tSPI_Transfer.rx_buf = (unsigned long)unRF905_SPI_RX_Frame;
	tSPI_Transfer.len = unFrameLength + 1;
	tSPI_Transfer.delay_usecs = unSPI_Delay;
	tSPI_Transfer.speed_hz = unSPI_Speed;
	tSPI_Transfer.bits_per_word = unSPI_Bits;

	if (ioctl(nRF905SPIfd, SPI_IOC_MESSAGE(1), &tSPI_Transfer) < 0){
		// printf("Can't send spi message during SPI RD with error code %d. \n", errno);
		NRF905D_LOG_ERR("can't send spi message");
		return NULL;
	}
	return unRF905_SPI_RX_Frame;
}

static nRF905Boolean_t bIsCarrierDetected(void)
{
	return ((GPIORead(NRF905_CD_PIN) == GPIO_LEVEL_HIGH) ? NRF905_TRUE : NRF905_FALSE);
}

static nRF905Boolean_t bIsAddressMatch(void)
{
	return ((GPIORead(NRF905_AM_PIN) == GPIO_LEVEL_HIGH) ? NRF905_TRUE : NRF905_FALSE);
}

static nRF905Boolean_t bIsDataReady(void)
{
	return ((GPIORead(NRF905_DR_PIN) == GPIO_LEVEL_HIGH) ? NRF905_TRUE : NRF905_FALSE);
}

static int32_t nGetAddrFromCH_NO(uint16_t unChannelNumber, uint8_t* pRX_Address)
{
	pRX_Address[0] = (uint8_t)(unChannelNumber & 0x03);
	pRX_Address[1] = (uint8_t)(unChannelNumber & 0x0D);
	pRX_Address[2] = (uint8_t)(unChannelNumber & 0x50);
	pRX_Address[3] = (uint8_t)(unChannelNumber & 0xAA);
	return 0;
}

static int32_t nRF905CR_Initial(int32_t nRF905SPIfd)
{
	uint8_t* pRXwStatus;
	uint32_t unIndex;

	if (nRF905_SPI_WR(nRF905SPIfd, NRF905_CMD_WC(0), NRF905_CR_DEFAULT, ARRAY_SIZE(NRF905_CR_DEFAULT)) < 0){
		// printf("nRF905 control register initialization failed at nRF905 SPI WR.\n");
		NRF905D_LOG_ERR("nRF905 control register initialization failed at nRF905 SPI WR.");
		return -1;
	}
	// printf("nRF905_SPI_WR OK in nRF905 CR initialization.\n");

	pRXwStatus = pRF905_SPI_RD(nRF905SPIfd, NRF905_CMD_RC(0), ARRAY_SIZE(NRF905_CR_DEFAULT));
	if (NULL == pRXwStatus){
		// printf("pRF905_SPI_RD failed. \n");
		NRF905D_LOG_ERR("nRF905 control register initialization failed at nRF905 RX.");
		return -1;
	}
	for (unIndex = 0; unIndex < ARRAY_SIZE(NRF905_CR_DEFAULT) + 1; unIndex++){
		// printf("0x%02X\n", pRXwStatus[unIndex]);
	}
	if (memcmp(pRXwStatus + 1, NRF905_CR_DEFAULT, ARRAY_SIZE(NRF905_CR_DEFAULT)) != 0){
		NRF905D_LOG_ERR("nRF905 control register initialization failed at comparing CR value.");
		return -1;
	}
	nGetAddrFromCH_NO(NRF905_CR_DEFAULT[0] | ((uint16_t)(NRF905_CR_DEFAULT[1] & 0x01) << 8),
			(uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)));
	if (nRF905_SPI_WR(nRF905SPIfd, NRF905_CMD_WTA,
			(uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)), NRF905_TX_ADDR_LEN) < 0){
		// printf("Set nRF905's TX address in CR failed. \n");
		NRF905D_LOG_ERR("Set nRF905's TX address in CR failed.");
		return (-1);
	}
	return 0;
}

int32_t nRF905SpiInitial(int32_t nRF905SPIfd)
{
	/*
	 * spi mode
	 */
	if (ioctl(nRF905SPIfd, SPI_IOC_WR_MODE, &unSPI_Mode) < 0){
		NRF905D_LOG_ERR("can't set spi mode");
		return (-1);
	}

	if (ioctl(nRF905SPIfd, SPI_IOC_RD_MODE, &unSPI_Mode) < 0){
		NRF905D_LOG_ERR("can't get spi mode");
		return (-1);
	}

	/*
	 * bits per word
	 */
	if (ioctl(nRF905SPIfd, SPI_IOC_WR_BITS_PER_WORD, &unSPI_Bits) < 0){
		NRF905D_LOG_ERR("can't set bits per word");
		return (-1);
	}

	if (ioctl(nRF905SPIfd, SPI_IOC_RD_BITS_PER_WORD, &unSPI_Bits) < 0){
		NRF905D_LOG_ERR("can't get bits per word");
		return (-1);
	}

	/*
	 * max speed hz
	 */
	if (ioctl(nRF905SPIfd, SPI_IOC_WR_MAX_SPEED_HZ, &unSPI_Speed) < 0){
		NRF905D_LOG_ERR("can't set max speed hz");
		return (-1);
	}

	if (ioctl(nRF905SPIfd, SPI_IOC_RD_MAX_SPEED_HZ, &unSPI_Speed) < 0){
		NRF905D_LOG_ERR("can't get max speed hz");
		return (-1);
	}

	// printf("SPI parameter using is speed:%u \n", unSPI_Speed);
	// printf("SPI parameter using is stop bit:%u \n", unSPI_Bits);
	// printf("SPI parameter using is mode:%u \n", unSPI_Mode);
	return 0;
}

static int32_t nSetNRF905ChnPwr(int32_t nRF905SPI_Fd, uint16_t unFrqPwr)
{
//	uint8_t unSPI_WR_Data[5];
	unFrqPwr = NRF905_CMD_CC(unFrqPwr);
	if (nRF905_SPI_WR(nRF905SPI_Fd, unFrqPwr >> 8, (uint8_t*)(&unFrqPwr), 1) < 0){
		NRF905D_LOG_ERR("Set nRF905's frequency and power failed.");
		return (-1);
	}
	tRemoteControlMap.unNRF905ChNoAndPwr = unFrqPwr;
	nGetAddrFromCH_NO(unFrqPwr & CH_MSK_IN_CC_REG, (uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)));
	if (nRF905_SPI_WR(nRF905SPI_Fd, NRF905_CMD_WC(NRF905_RX_ADDRESS_IN_CR),
			(uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)), NRF905_RX_ADDR_LEN) < 0){
		// printf("Set nRF905's RX address in CR failed. \n");
		NRF905D_LOG_ERR("Set nRF905's RX address in CR failed.");
		return (-1);
	}
	if (nRF905_SPI_WR(nRF905SPI_Fd, NRF905_CMD_WTA,
			(uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)), NRF905_TX_ADDR_LEN) < 0){
		// printf("Set nRF905's TX address in CR failed. \n");
		NRF905D_LOG_ERR("Set nRF905's TX address in CR failed.");
		return (-1);
	}
	NRF905D_LOG_INFO("nRF905's channel and power was changed to %04X. \n", unFrqPwr);
	return 0;
}

uint64_t getTimeDiffInUs(struct timeval tLastTime, struct timeval tCurrentTime)
{
	return (((tCurrentTime.tv_sec - tLastTime.tv_sec) * US_PER_SECONDE + tCurrentTime.tv_usec) - tLastTime.tv_usec);
}

static int32_t nRF905ReceiveFrame(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask)
{
	struct timeval tLastTime, tCurrentTime;
	uint8_t* pSPI_ReadData;

	if (NULL == tNRF905CommTask.pRX_Frame){
		NRF905D_LOG_ERR("No place to save received frame.");
		return (-1);
	}
	// printf("nRF905 start receive frame.\n");
	// Start listen
	setNRF905Mode(NRF905_MODE_BURST_RX);

	// delay until CD, AM and DR set with timeout
	gettimeofday(&tLastTime, NULL);
	gettimeofday(&tCurrentTime, NULL);
	while (getTimeDiffInUs(tLastTime, tCurrentTime) < AFTER_RX_MODE_MAX_DR_DELAY_US){
		if ((bIsCarrierDetected() == NRF905_TRUE) &&
				(bIsAddressMatch() == NRF905_TRUE) &&
				(bIsDataReady() == NRF905_TRUE)){
			break;
		}
		usleep(200);
		gettimeofday(&tCurrentTime, NULL);
	}
	if (getTimeDiffInUs(tLastTime, tCurrentTime) >= AFTER_RX_MODE_MAX_DR_DELAY_US){
		setNRF905Mode(NRF905_MODE_STD_BY);
		// printf("Receive timeout.\n");
		NRF905D_LOG_ERR("Receive timeout.");
		return (-1);
	}

	// printf("Data ready.\n");
	// start SPI read RX payload from nRF905
	pSPI_ReadData = pRF905_SPI_RD(nRF905SPI_Fd, NRF905_CMD_RRP, NRF905_RX_PAYLOAD_LEN);
	if (NULL == pSPI_ReadData){
		// printf("Read RX payload from nRF905 failed.\n");
		NRF905D_LOG_ERR("Read RX payload from nRF905 failed.");
		return (-1);
	}
	memcpy(tNRF905CommTask.pRX_Frame, pSPI_ReadData + 1, tNRF905CommTask.unCommByteNum);

	return 0;
}
int32_t nRF905CheckReceivedFrame(nRF905CommTask_t tNRF905CommTask)
{
	// printf("pTX_Frame[0] is %d and received frame is %u.\n", tNRF905CommTask.pTX_Frame[0], tNRF905CommTask.pRX_Frame[0]);
	switch (tNRF905CommTask.pTX_Frame[0]){
	case RF_READ_SENSOR_VALUE:
		if (RF_READ_SENSOR_VALUE == tNRF905CommTask.pRX_Frame[0]){
			return 0;
		} else {
			return (-1);
		}
	default:
		return (-1);
	}
}

static int32_t nRF905SendFrame(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask)
{
	struct timeval tLastTime, tCurrentTime;
//	static uint32_t unSendOutFrame = 0;
//	static uint32_t unResponseFrame = 0;
	int32_t unIndex;

//	// printf("nRF905 start send frame.\n");
//	NRF905D_LOG_INFO("nRF905 start send frame.");

	if (nRF905_SPI_WR(nRF905SPI_Fd, NRF905_CMD_WTP, tNRF905CommTask.pTX_Frame, NRF905_TX_PAYLOAD_LEN) < 0){
		// printf("Write TX payload error.\n");
		NRF905D_LOG_ERR("Write TX payload error.");
		return (-1);
	}else{
		// TX payload was successfully written into nRF905's register
		// Start transmit
		setNRF905Mode(NRF905_MODE_BURST_TX);
		// delay until DR set with timeout
		gettimeofday(&tLastTime, NULL);
		gettimeofday(&tCurrentTime, NULL);
		while (getTimeDiffInUs(tLastTime, tCurrentTime) < AFTER_SET_BURST_TX_MAX_DELAY_US){
			if (bIsDataReady() == NRF905_TRUE){
				break;
			}
			usleep(200);
			gettimeofday(&tCurrentTime, NULL);
		}
		if (getTimeDiffInUs(tLastTime, tCurrentTime) >= AFTER_SET_BURST_TX_MAX_DELAY_US){
			// printf("Payload sent out by nRF905 timeout.\n");
			setNRF905Mode(NRF905_MODE_STD_BY);
			NRF905D_LOG_ERR("Data transmit failed.");
			return (-1);
		}
		// sleep a while to make sure data has been sent out and no reflect return to receive
		setNRF905Mode(NRF905_MODE_STD_BY);
//		unSendOutFrame++;

		// start to read response
		if (nRF905ReceiveFrame(nRF905SPI_Fd, tNRF905CommTask) < 0){
			// printf("Data receive failed.\n");
			NRF905D_LOG_ERR("Data receive failed.");
			return (-1);
		}
		// If receive OK, frame was saved in the tNRF905CommTask.pRX_Frame
		setNRF905Mode(NRF905_MODE_STD_BY);

		for (unIndex = 0; unIndex < tNRF905CommTask.unCommByteNum; unIndex++){
			// printf("0x%02X\n", tNRF905CommTask.pRX_Frame[unIndex]);
		}
		if (nRF905CheckReceivedFrame(tNRF905CommTask) < 0){
			// printf("Check received frame error.\n");
			NRF905D_LOG_ERR("The received frame is different with sent one.");
			return (-1);
		}
//		unResponseFrame++;
//		// printf("Total %u frame has been sent out. \nThe difference between send out and receive is:%d \n",
//				unSendOutFrame, unSendOutFrame - unResponseFrame);
	}
//	// printf("One frame was successfully sent out. \n");
	return 0;
}

static int32_t nRF905Hopping(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask)
{
//	uint32_t unCD_RetryCNT;
	uint32_t unTX_RetryCNT;
	static uint8_t unHoppingTableIndex = 0;

	// printf("Hopping procedure start.\n");
	NRF905D_LOG_INFO("Hopping procedure start.");

	for (unTX_RetryCNT = 0; unTX_RetryCNT < HOPPING_MAX_RETRY_NUM; unTX_RetryCNT++){
		if (unNeedtoClose != NRF905_FALSE){
			break;
		}
		if (unHoppingTableIndex < (ARRAY_SIZE(unRF_HOPPING_TABLE) - 1)){
			unHoppingTableIndex++;
		}else{
			unHoppingTableIndex = 0;
		}
		if (nSetNRF905ChnPwr(nRF905SPI_Fd, unRF_HOPPING_TABLE[unHoppingTableIndex]) < 0){
//			// printf("Can not set nRF905's frequency and power. Try next frequency and power. \n");
			NRF905D_LOG_ERR("Can not set nRF905's frequency and power.");
			continue;
		}else{
//			// printf("Change frequency and power OK. Try to send one frame. \n");
			tRemoteControlMap.unNRF905HoppingNumer++;
			if (nRF905SendFrame(nRF905SPI_Fd, tNRF905CommTask) < 0){
				// printf("Try to send one frame failed during hopping. Try next channel.\n");
				NRF905D_LOG_INFO("Send frame error during hopping. Try next channel.");
			}else{
				// printf("Hopping success, %08X was used as parameter.\n", tRemoteControlMap.unNRF905ChNoAndPwr);
				NRF905D_LOG_INFO("Hopping success, %08X was used as RX address.", tRemoteControlMap.unNRF905RX_Address);
				tRemoteControlMap.unNRF905CommSendFrameErr = 0;
				return 0;
			}
			usleep(HOPPING_TX_RETRY_DELAY_US);
		}
	}
	// printf("Hopping failed. \n");
	NRF905D_LOG_INFO("Hopping failed.");
	return (-1);
}

static int32_t nNRF905ExecuteTask(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask)
{
//	// printf("Start to execute one task.\n");
	if (nRF905SendFrame(nRF905SPI_Fd, tNRF905CommTask) < 0){
		// printf("nRF905 execute task error, start hopping.\n");
		NRF905D_LOG_ERR("nRF905 execute task error, start hopping.");
		tRemoteControlMap.unNRF905CommSendFrameErr++;
		tRemoteControlMap.unNRF905CommSendFrameErrTotal++;
		if (nRF905Hopping(nRF905SPI_Fd, tNRF905CommTask) < 0){
			NRF905D_LOG_ERR("Can not find any valid receiver in air.");
			return (-1);
		}else{
			tRemoteControlMap.unNRF905CommSendFrameErr = 0;
			tRemoteControlMap.unNRF905CommSendFrameOK++;
			return 0;
		}
//		return 0;
	}else{
		tRemoteControlMap.unNRF905CommSendFrameErr = 0;
		tRemoteControlMap.unNRF905CommSendFrameOK++;
		return 0;
	}
}

int32_t nUnixSocketListen(const char *pNRF905ServerName)
{
	int32_t nLocalSocketFd;
	struct sockaddr_un tSocketAddrUn;
	int32_t nLen;
	if ((nLocalSocketFd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0){
		NRF905D_LOG_ERR("Create AF_UNIX socket failed with error %d.", errno);
		return (-1);
	}
	unlink(pNRF905ServerName);               /* in case it already exists */
	memset(&tSocketAddrUn, 0, sizeof(tSocketAddrUn));
	tSocketAddrUn.sun_family = AF_UNIX;
	strcpy(tSocketAddrUn.sun_path, pNRF905ServerName);
	nLen = offsetof(struct sockaddr_un, sun_path) + strlen(pNRF905ServerName);
	/* bind the name to the descriptor */
	if (bind(nLocalSocketFd, (struct sockaddr *)&tSocketAddrUn, nLen) < 0){
		close(nLocalSocketFd);
		NRF905D_LOG_ERR("Bind local socket failed with error %d.", errno);
		return (-1);
	}else{
		if (listen(nLocalSocketFd, MAX_CONNECTION_PENDING) < 0){
			close(nLocalSocketFd);
			NRF905D_LOG_ERR("Listen local socket failed with error %d.", errno);
			return (-1);
		}else{
			return nLocalSocketFd;
		}
	}
}

int32_t nUnixSocketAccept(int32_t nServerSocket, uid_t* pAcceptUID)
{
	int32_t nClientFd;
	socklen_t nLeng;
	struct sockaddr_un tSocketAddrUn;
	struct stat tStatBuf;
	nLeng = sizeof(tSocketAddrUn);

	if ((nClientFd = accept(nServerSocket, (struct sockaddr *)(&tSocketAddrUn), &nLeng)) < 0){
		return (-1);
	}
	/* obtain the client's uid from its calling address */
	nLeng -= offsetof(struct sockaddr_un, sun_path);  /* len of pathname */
	tSocketAddrUn.sun_path[nLeng] = 0; /* null terminate */
	if (stat(tSocketAddrUn.sun_path, &tStatBuf) < 0){
		close(nClientFd);
		NRF905D_LOG_ERR("Stat local socket failed with error %d.", errno);
		return (-1);
	}else{
		if (S_ISSOCK(tStatBuf.st_mode)){
			if (pAcceptUID != NULL){
				*pAcceptUID = tStatBuf.st_uid;    /* return uid of caller */
			}
			unlink(tSocketAddrUn.sun_path);       /* we're done with pathname now */
			return nClientFd;
		}else{
			close(nClientFd);
			NRF905D_LOG_ERR("S_ISSOCK local socket failed with error %d.", errno);
			return (-1);
		}
	}
}

/* Create a client endpoint32_t and connect to a server.   Returns fd if all OK, <0 on error. */
int32_t nUnixSocketConn(const char *pNRF905ServerName)
{
	int32_t nConnectSocketFd;
	int32_t nLen;
	struct sockaddr_un tSocketAddrUn;

	if ((nConnectSocketFd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0){     /* create a UNIX domain stream socket */
		return(-1);
	}
	memset(&tSocketAddrUn, 0, sizeof(tSocketAddrUn));            /* fill socket address structure with our address */
	tSocketAddrUn.sun_family = AF_UNIX;
	sprintf(tSocketAddrUn.sun_path, "scktmp%05d", getpid());
	nLen = offsetof(struct sockaddr_un, sun_path) + strlen(tSocketAddrUn.sun_path);
	unlink(tSocketAddrUn.sun_path);               /* in case it already exists */
	if (bind(nConnectSocketFd, (struct sockaddr *)&tSocketAddrUn, nLen) < 0) {
		close(nConnectSocketFd);
		NRF905D_LOG_ERR("Bind socket failed during client connect with error %d.", errno);
		return (-1);
	}else{
		/* fill socket address structure with server's address */
		memset(&tSocketAddrUn, 0, sizeof(tSocketAddrUn));
		tSocketAddrUn.sun_family = AF_UNIX;
		strcpy(tSocketAddrUn.sun_path, pNRF905ServerName);
		nLen = offsetof(struct sockaddr_un, sun_path) + strlen(pNRF905ServerName);
		if (connect(nConnectSocketFd, (struct sockaddr *)&tSocketAddrUn, nLen) < 0)   {
			close(nConnectSocketFd);
			NRF905D_LOG_ERR("Connect server failed with error %d.", errno);
			return (-1);
		}else{
			return (nConnectSocketFd);
		}
	}
}

int32_t nSendDataToNRF905Socket(int32_t nConnectedSocketFD, nRF905CommTask_t* pNRF905CommTask, uint8_t* pACK_Payload)
{
	uint8_t unBuffer[sizeof(nRF905CommTask_t) + NRF905_TX_PAYLOAD_LEN];
	memcpy(unBuffer, pNRF905CommTask, sizeof(nRF905CommTask_t));
	memcpy(unBuffer + sizeof(nRF905CommTask_t), pACK_Payload, NRF905_TX_PAYLOAD_LEN);
	return send(nConnectedSocketFD, unBuffer, sizeof(nRF905CommTask_t) + NRF905_TX_PAYLOAD_LEN, 0);
}

void clientHandler(int32_t nClientSock, int32_t nRF905SPI_FD, sem_t* pSem)
{
	int32_t nReceivedCNT;
	nRF905CommTask_t tNRF905CommTask;
	uint8_t unACK_Payload[NRF905_TX_PAYLOAD_LEN];
	int32_t nRslt;

    /* Receive message */
    if ((nReceivedCNT = recv(nClientSock, &tNRF905CommTask, sizeof(nRF905CommTask_t), 0)) < 0) {
    	NRF905D_LOG_ERR("Failed to receive task data from client with code:%d.", errno);
    	close(nClientSock);
    	exit(-1);
    }
    if ((nReceivedCNT = recv(nClientSock, unACK_Payload, NRF905_TX_PAYLOAD_LEN, 0)) < 0) {
    	NRF905D_LOG_ERR("Failed to receive payload data from client with code:%d.", errno);
    	close(nClientSock);
    	exit(-1);
    }
    while (NRF905_FALSE == unNeedtoClose) {
    	tNRF905CommTask.pTX_Frame = unACK_Payload;
		tNRF905CommTask.pRX_Frame = malloc(tNRF905CommTask.unCommByteNum);
		if (NULL != tNRF905CommTask.pRX_Frame){
			NRF905D_LOG_INFO("One ACK task was successfully gotten from socket.");
			sem_wait(pSem);
//			nRslt = nNRF905ExecuteTask(nRF905SPI_FD, tNRF905CommTask);
			printf("nRF905 server %d received one message from socket %d.\n", getpid(), nClientSock);
			sem_post(pSem);
//			if (nRslt < 0){
//				// execute task error
//				tNRF905CommTask.pRX_Frame[0] = RF_CMD_FAILED;
//			}
			send(nClientSock, tNRF905CommTask.pRX_Frame + 1, NRF905_RX_PAYLOAD_LEN, 0);
			free(tNRF905CommTask.pRX_Frame);
		}else{
			NRF905D_LOG_ERR("WTF malloc fail??");
        	close(nClientSock);
        	exit(-1);
		}

        /* Receive message */
        if ((nReceivedCNT = recv(nClientSock, &tNRF905CommTask, sizeof(nRF905CommTask_t), 0)) < 0) {
        	NRF905D_LOG_ERR("Failed to receive task data from client with error %d.", errno);
        	close(nClientSock);
        	exit(-1);
        }
        if ((nReceivedCNT = recv(nClientSock, unACK_Payload, NRF905_TX_PAYLOAD_LEN, 0)) < 0) {
        	NRF905D_LOG_ERR("Failed to receive payload data from client with error %d.", errno);
        	close(nClientSock);
        	exit(-1);
        }
    }
	close(nClientSock);
	exit(0);
}


void* ackRoutine(void* pArgu)
{
	nRF905CommTask_t tNRF905CommTask;
	static uint8_t unACK_TX_Payload[NRF905_TX_PAYLOAD_LEN] = {RF_READ_SENSOR_VALUE, 0x00, };
	static uint8_t unACK_RX_Payload[NRF905_RX_PAYLOAD_LEN];
	int32_t nConnectedSocketFd;
	int32_t nReceivedCNT;

	sleep(2);
	nConnectedSocketFd = nUnixSocketConn(NRF905_SERVER_NAME);
	if(nConnectedSocketFd < 0)
	{
		// printf("Error[%d] when connecting...", errno);
		pthread_exit(NULL);
	}

	tNRF905CommTask.tCommType = NRF905_COMM_TYPE_TX_PKG;
	tNRF905CommTask.unCommByteNum = NRF905_TX_PAYLOAD_LEN;

	while (NRF905_FALSE == unNeedtoClose){
		if (nSendDataToNRF905Socket(nConnectedSocketFd, &tNRF905CommTask, unACK_TX_Payload) < 0 ){
			// printf("Write task communication payload to pipe error with code:%d.", errno);
			NRF905D_LOG_ERR("Write task communication payload to pipe error with code:%d.", errno);
		} else {
			if ((nReceivedCNT = recv(nConnectedSocketFd, unACK_RX_Payload, NRF905_RX_PAYLOAD_LEN, 0)) < 0) {
				// printf("Receive from unix domain socket error with code:%d.", errno);
				NRF905D_LOG_ERR("Receive from unix domain socket error with code:%d.", errno);
			}else{
				printf("Socket %d received one message from nRF905 server.\n", nConnectedSocketFd);
			}
		}
		usleep(ACK_TASK_INTERVAL_US);
	}
	close(nConnectedSocketFd);
	pthread_exit(NULL);
}

int32_t main(void) {
	int32_t nRF905SPI_FD;
	struct sigaction tSignalINT_Action, tSignalCHLD_Action;
	int32_t nServerSock, nClientSock;
	uid_t tAcceptUID;
	pid_t tChildPid;
    pthread_t tACK_Thread;
    pthread_attr_t tACK_ThreadAttr;
    sem_t* pSem;

	memset(tClientPid, 0, sizeof(tClientPid));

    tSignalINT_Action.sa_sigaction = sigINT_Handler;
    tSignalINT_Action.sa_flags = SA_SIGINFO;
    sigaction(SIGINT, &tSignalINT_Action, NULL);

    tSignalCHLD_Action.sa_sigaction = sigCHLD_Handler;
    tSignalCHLD_Action.sa_flags = SA_SIGINFO;
    sigaction(SIGCHLD, &tSignalCHLD_Action, NULL);

	puts("!!!nRF905 Daemon start!!!"); /* prints !!!nRF905 Daemon start!!! */

	NRF905D_LOG_INFO("nRF905 Daemon start...");

    /* initialize semaphores for shared processes */
	pSem = sem_open(NRF905_SERVER_SEMAPHORE, O_CREAT | O_EXCL, 0644, 1);
	if (pSem == NULL){
		NRF905D_LOG_ERR("Semaphore open failed.");
		exit(-1);
	}
    /* name of semaphore is "pSem", semaphore is reached using this name */
    sem_unlink (NRF905_SERVER_SEMAPHORE);

	// printf("Initialize GPIO.\n");
	if (nInitNRF905GPIO() < 0){
		NRF905D_LOG_ERR("nRF905 communication thread exit because nRF905 GPIO initialization failed.");
		exit(-1);
	}

	nRF905SPI_FD = open(NRF905_SPI_DEVICE, O_RDWR);
	if (nRF905SPI_FD < 0) {
		NRF905D_LOG_ERR("nRF905 communication thread exit because SPI port %s open failed.", NRF905_SPI_DEVICE);
		nDisableSPI_GPIO();
		exit(-1);
	}

	// printf("Initialize SPI.\n");
	if (nRF905SpiInitial(nRF905SPI_FD) < 0){
		close(nRF905SPI_FD);
		nDisableSPI_GPIO();
		NRF905D_LOG_ERR("nRF905 communication thread exit because SPI initialization failed.");
		exit(-1);
	}

	// printf("Initialize nRF905 control register.\n");
	if (nRF905CR_Initial(nRF905SPI_FD) < 0){
		close(nRF905SPI_FD);
		nDisableSPI_GPIO();
		NRF905D_LOG_ERR("nRF905 communication thread exit because nRF905 CR initialization failed.");
		exit(-1);
	}

	// printf("Initialize local socket.\n");
	nServerSock = nUnixSocketListen(NRF905_SERVER_NAME);
	if(nServerSock < 0){
		close(nRF905SPI_FD);
		nDisableSPI_GPIO();
		NRF905D_LOG_ERR("Error[%d] when listening...\n",errno);
		exit(-1);
	}

//	pthread_attr_init(&tACK_ThreadAttr);
//	pthread_create(&tACK_Thread, &tACK_ThreadAttr, ackRoutine, NULL);

    /* Run until cancelled */
	while (NRF905_FALSE == unNeedtoClose) {
		/* Wait for client connection */
		nClientSock = nUnixSocketAccept(nServerSock, &tAcceptUID);
		if(nClientSock < 0){
			NRF905D_LOG_ERR("Error[%d] when accepting...\n",errno);
			break;
		}
		printf("New socket %d was connected in.\n",nClientSock );
		tChildPid = fork();
		if (tChildPid < 0){
			NRF905D_LOG_ERR("Fork failed.");
		}else if (0 == tChildPid){
			// Child process
			clientHandler(nClientSock, nRF905SPI_FD, pSem);
		}else{
			// Parent process
			nRecordPid(tChildPid, tClientPid, MAX_CONNECTION_PENDING);
		}
	}

	nKillAllChild(tClientPid, MAX_CONNECTION_PENDING);
//	pthread_join(tACK_Thread, NULL);
	nWaitAllChild(tClientPid, MAX_CONNECTION_PENDING);
	sem_destroy(pSem);
	nDisableSPI_GPIO();
	close(nRF905SPI_FD);
	close(nServerSock);

	NRF905D_LOG_INFO("INT signal was received, exit.");

	exit(0);
}
