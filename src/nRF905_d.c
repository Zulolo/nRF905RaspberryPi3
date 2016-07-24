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
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define __USED_NRF905_INTERNAL__
#include "nRF905_d.h"

extern int32_t pNRF905Server(int32_t nTaskPipeFD);

void sigINT_Handler(int32_t signum, siginfo_t *info, void *ptr)
{
	unNeedtoClose = NRF905_TRUE;
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
		printf("Set nRF905 mode failed.\n");
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
		printf("Can't send spi message during SPI WR with error code %d. \n", errno);
		NRF905D_LOG_ERR("can't send spi message");
		return (-1);
	}
	return 0;
}

static uint8_t* nRF905_SPI_RD(int32_t nRF905SPIfd, uint8_t unCMD, uint8_t* pRX_Frame, uint8_t unFrameLength)
{
	uint8_t unRF905_SPI_TX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
//	uint32_t unIndex;
	static 	uint8_t unRF905_SPI_RX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
//	static uint8_t unRF905_SPI_RD_RX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
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
	if (NULL == pRX_Frame){
		tSPI_Transfer.rx_buf = (unsigned long)unRF905_SPI_RX_Frame;
	}else{
		tSPI_Transfer.rx_buf = (unsigned long)pRX_Frame;
	}
	tSPI_Transfer.len = unFrameLength + 1;
	tSPI_Transfer.delay_usecs = unSPI_Delay;
	tSPI_Transfer.speed_hz = unSPI_Speed;
	tSPI_Transfer.bits_per_word = unSPI_Bits;

	if (ioctl(nRF905SPIfd, SPI_IOC_MESSAGE(1), &tSPI_Transfer) < 0){
		printf("Can't send spi message during SPI RD with error code %d. \n", errno);
		NRF905D_LOG_ERR("can't send spi message");
		return NULL;
	}
	if (NULL == pRX_Frame){
//		memcpy(unRF905_SPI_RD_RX_Frame, unRF905_SPI_RX_Frame, unFrameLength + 1);
		return unRF905_SPI_RX_Frame;
	}else{
//		for (unIndex = 0; unIndex < unFrameLength; unIndex++){
//			printf("0x%02X\n", pRX_Frame[unIndex]);
//		}
		return pRX_Frame;
	}
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
		printf("nRF905 control register initialization failed at nRF905 SPI WR.\n");
		NRF905D_LOG_ERR("nRF905 control register initialization failed at nRF905 SPI WR.");
		return -1;
	}
	printf("nRF905_SPI_WR OK in nRF905 CR initialization.\n");

	pRXwStatus = nRF905_SPI_RD(nRF905SPIfd, NRF905_CMD_RC(0), NULL, ARRAY_SIZE(NRF905_CR_DEFAULT));
	if (NULL == pRXwStatus){
		printf("nRF905_SPI_RD failed. \n");
		NRF905D_LOG_ERR("nRF905 control register initialization failed at nRF905 RX.");
		return -1;
	}
	for (unIndex = 0; unIndex < ARRAY_SIZE(NRF905_CR_DEFAULT) + 1; unIndex++){
		printf("0x%02X\n", pRXwStatus[unIndex]);
	}
	if (memcmp(pRXwStatus + 1, NRF905_CR_DEFAULT, ARRAY_SIZE(NRF905_CR_DEFAULT)) != 0){
		NRF905D_LOG_ERR("nRF905 control register initialization failed at comparing CR value.");
		return -1;
	}
	nGetAddrFromCH_NO(NRF905_CR_DEFAULT[0] | ((uint16_t)(NRF905_CR_DEFAULT[1] & 0x01) << 8),
			(uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)));
	if (nRF905_SPI_WR(nRF905SPIfd, NRF905_CMD_WTA,
			(uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)), NRF905_TX_ADDR_LEN) < 0){
		printf("Set nRF905's TX address in CR failed. \n");
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

	printf("SPI parameter using is speed:%u \n", unSPI_Speed);
	printf("SPI parameter using is stop bit:%u \n", unSPI_Bits);
	printf("SPI parameter using is mode:%u \n", unSPI_Mode);
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
		printf("Set nRF905's RX address in CR failed. \n");
		NRF905D_LOG_ERR("Set nRF905's RX address in CR failed.");
		return (-1);
	}
	if (nRF905_SPI_WR(nRF905SPI_Fd, NRF905_CMD_WTA,
			(uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)), NRF905_TX_ADDR_LEN) < 0){
		printf("Set nRF905's TX address in CR failed. \n");
		NRF905D_LOG_ERR("Set nRF905's TX address in CR failed.");
		return (-1);
	}
	NRF905D_LOG_INFO("nRF905's channel and power was changed to %04X. \n", unFrqPwr);
	return 0;
}

uint64_t getTimeDiffInUs(struct timeval tLastTime, struct timeval tCurrentTime)
{
	return abs(tCurrentTime.tv_usec - tLastTime.tv_usec + (tCurrentTime.tv_sec - tLastTime.tv_sec) * US_PER_SECONDE);
}

static int32_t nRF905ReceiveFrame(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask)
{
	struct timeval tLastTime, tCurrentTime;
	if (NULL == tNRF905CommTask.pRX_Frame){
		NRF905D_LOG_ERR("No place to save received frame.");
		return (-1);
	}
//	printf("nRF905 start receive frame.\n");
	// Start listen
	setNRF905Mode(NRF905_MODE_BURST_RX);

	// delay until CD, AM and DR set with timeout
	gettimeofday(&tLastTime, NULL);
	gettimeofday(&tCurrentTime, NULL);
	while (getTimeDiffInUs(tLastTime, tCurrentTime) < AFTER_CD_MAX_AM_DELAY_US){
		if ((bIsCarrierDetected() == NRF905_TRUE) &&
				(bIsAddressMatch() == NRF905_TRUE) &&
				(bIsDataReady() == NRF905_TRUE)){
			break;
		}
		usleep(200);
		gettimeofday(&tCurrentTime, NULL);
	}
	if (getTimeDiffInUs(tLastTime, tCurrentTime) >= AFTER_CD_MAX_AM_DELAY_US){
		setNRF905Mode(NRF905_MODE_STD_BY);
		NRF905D_LOG_ERR("Receive timeout.");
		return (-1);
	}

//	printf("Data ready.\n");
	// start SPI read RX payload from nRF905
	if (nRF905_SPI_RD(nRF905SPI_Fd, NRF905_CMD_RRP, tNRF905CommTask.pRX_Frame, NRF905_RX_PAYLOAD_LEN) == NULL){
		NRF905D_LOG_ERR("Read RX payload from nRF905 failed.");
		return (-1);
	}

	return 0;
}

static int32_t nRF905SendFrame(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask)
{
	struct timeval tLastTime, tCurrentTime;
	uint32_t unIndex;
	static uint32_t unSendOutFrame = 0;
	static uint32_t unResponseFrame = 0;

//	printf("nRF905 start send frame.\n");
//	NRF905D_LOG_INFO("nRF905 start send frame.");

	if (nRF905_SPI_WR(nRF905SPI_Fd, NRF905_CMD_WTP, tNRF905CommTask.pTX_Frame, NRF905_TX_PAYLOAD_LEN) < 0){
		printf("Write TX payload error.\n");
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
			printf("Payload sent out by nRF905 timeout.\n");
			setNRF905Mode(NRF905_MODE_STD_BY);
			NRF905D_LOG_ERR("Data transmit failed.");
			return (-1);
		}
		// sleep a while to make sure data has been sent out and no reflect return to receive
		setNRF905Mode(NRF905_MODE_STD_BY);
		unSendOutFrame++;

		// start to read response
		if (nRF905ReceiveFrame(nRF905SPI_Fd, tNRF905CommTask) < 0){
//			printf("Data receive failed.\n");
			NRF905D_LOG_ERR("Data receive failed.");
			return (-1);
		}
		// If receive OK, frame was saved in the tNRF905CommTask.pRX_Frame
		setNRF905Mode(NRF905_MODE_STD_BY);

		for (unIndex = 0; unIndex < tNRF905CommTask.unCommByteNum; unIndex++){
			printf("0x%02X\n", tNRF905CommTask.pRX_Frame[unIndex]);
		}
		if (memcmp(tNRF905CommTask.pTX_Frame, tNRF905CommTask.pRX_Frame + 1, tNRF905CommTask.unCommByteNum) != 0){
			NRF905D_LOG_ERR("The received frame is different with sent one.");
			return (-1);
		}
		unResponseFrame++;
		printf("Total %u frame has been sent out. \nThe difference between send out and receive is:%d \n",
				unSendOutFrame, unSendOutFrame - unResponseFrame);
	}
	printf("One frame was successfully sent out. \n");
	return 0;
}

static int32_t nRF905Hopping(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask)
{
//	uint32_t unCD_RetryCNT;
	uint32_t unTX_RetryCNT;
	static uint8_t unHoppingTableIndex = 0;

	printf("Hopping procedure start.\n");
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
//			printf("Can not set nRF905's frequency and power. Try next frequency and power. \n");
			NRF905D_LOG_ERR("Can not set nRF905's frequency and power.");
			continue;
		}else{
//			printf("Change frequency and power OK. Try to send one frame. \n");
			tRemoteControlMap.unNRF905HoppingNumer++;
			if (nRF905SendFrame(nRF905SPI_Fd, tNRF905CommTask) < 0){
//				printf("Try to send one frame failed during hopping. Try next channel.\n");
				NRF905D_LOG_INFO("Send frame error during hopping. Try next channel.");
			}else{
				printf("Hopping success, %08X was used as parameter.\n", tRemoteControlMap.unNRF905ChNoAndPwr);
				NRF905D_LOG_INFO("Hopping success, %08X was used as RX address.", tRemoteControlMap.unNRF905RX_Address);
				tRemoteControlMap.unNRF905CommSendFrameErr = 0;
				return 0;
			}
			usleep(HOPPING_TX_RETRY_DELAY_US);
		}
	}
	printf("Hopping failed. \n");
	NRF905D_LOG_INFO("Hopping failed.");
	return (-1);
}

static int32_t nNRF905ExecuteTask(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask)
{
//	printf("Start to execute one task.\n");
	if (nRF905SendFrame(nRF905SPI_Fd, tNRF905CommTask) < 0){
		printf("nRF905 execute task error, start hopping.\n");
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

int32_t pNRF905Comm(int32_t nTaskPipeFD)
{
	int32_t nRF905SPI_FD;
	nRF905CommTask_t tNRF905CommTask;
//	nRF905ThreadPara_t tNRF905CommThreadPara = *((nRF905ThreadPara_t *)ptr);

	NRF905D_LOG_INFO("nRF905 communication thread start.");
	printf("Initialize GPIO.\n");
	if (nInitNRF905GPIO() < 0){
		NRF905D_LOG_ERR("nRF905 communication thread exit because nRF905 GPIO initialization failed.");
		exit(-1);
	}

	nRF905SPI_FD = open(NRF905_SPI_DEVICE, O_RDWR);
	if (nRF905SPI_FD < 0) {
		NRF905D_LOG_ERR("nRF905 communication thread exit because SPI port %s open failed.", NRF905_SPI_DEVICE);
		exit(-1);
	}

	printf("Initialize SPI.\n");
	if (nRF905SpiInitial(nRF905SPI_FD) < 0){
		close(nRF905SPI_FD);
		NRF905D_LOG_ERR("nRF905 communication thread exit because SPI initialization failed.");
		exit(-1);
	}

	printf("Initialize nRF905 control register.\n");
	if (nRF905CR_Initial(nRF905SPI_FD) < 0){
		close(nRF905SPI_FD);
		NRF905D_LOG_ERR("nRF905 communication thread exit because nRF905 CR initialization failed.");
		exit(-1);
	}

	while (NRF905_FALSE == unNeedtoClose){
		if (read(nTaskPipeFD, &tNRF905CommTask, sizeof(nRF905CommTask_t)) > 0){
			if (tNRF905CommTask.unCommByteNum > 0){
				tNRF905CommTask.pTX_Frame = malloc(tNRF905CommTask.unCommByteNum);
				if (NULL == tNRF905CommTask.pTX_Frame){
					NRF905D_LOG_ERR("WTF malloc fail??");
				}else{
					if (read(nTaskPipeFD, tNRF905CommTask.pTX_Frame, tNRF905CommTask.unCommByteNum) > 0){
						tNRF905CommTask.pRX_Frame = malloc(tNRF905CommTask.unCommByteNum + 1);
						if (NULL != tNRF905CommTask.pRX_Frame){
							NRF905D_LOG_INFO("One ACK task was successfully gotten from pipe.");
							nNRF905ExecuteTask(nRF905SPI_FD, tNRF905CommTask);
							free(tNRF905CommTask.pRX_Frame);
						}else{
							NRF905D_LOG_ERR("WTF malloc fail??");
						}
					}else{
						NRF905D_LOG_ERR("Read task communication payload from pipe error with code:%d", errno);
					}
					free(tNRF905CommTask.pTX_Frame);
				}
			}
		}else{
			NRF905D_LOG_ERR("Read task communication type from pipe error with code:%d", errno);
		}
	}
	nDisableSPI_GPIO();
	close(nRF905SPI_FD);
	NRF905D_LOG_INFO("nRF905 communication thread exit.");
	exit(0);
}



int32_t main(void) {
	int32_t nRet;
	struct sigaction tSignalAction;
	pid_t tTemp, tNRF905CommPID, tRoutinePID;
	int32_t nTaskExecPipe[2];

	unNeedtoClose = NRF905_FALSE;

    tSignalAction.sa_sigaction = sigINT_Handler;
    tSignalAction.sa_flags = SA_SIGINFO;

    sigaction(SIGINT, &tSignalAction, NULL);

	puts("!!!nRF905 Daemon start!!!"); /* prints !!!nRF905 Daemon start!!! */

	NRF905D_LOG_INFO("nRF905 Daemon start...");

	// Prepare the pipe
	nRet = pipe(nTaskExecPipe);
	if(nRet < 0){
		NRF905D_LOG_ERR("Open task execution pipe failed with error:%d.", nRet);
		exit(-1);
	}

	// Start little birds
	printf("Start nRF905 communication thread.\n");
	tTemp = fork();
	if (tTemp < 0){
		NRF905D_LOG_ERR("Start nRF905 communication process failed.");
		close(nTaskExecPipe[0]);
		close(nTaskExecPipe[1]);
		exit(-1);
	}else if(0 == tTemp){
		// Child process
		pNRF905Comm(nTaskExecPipe[0]);
		exit(0);
	}else{
		// Parent process
		tNRF905CommPID = tTemp;
	}

	// Only parent process can get here
	printf("Start routine work thread.\n");
	tTemp = fork();
	if (tTemp < 0){
		NRF905D_LOG_ERR("Start routine thread failed with error:%d.", nRet);
		kill(tNRF905CommPID, SIGINT);
		waitpid(tNRF905CommPID, NULL, 0);
		close(nTaskExecPipe[0]);
		close(nTaskExecPipe[1]);
		exit(-1);
	}else if(0 == tTemp){
		// Child process
		pNRF905Server(nTaskExecPipe[1]);
		exit(0);
	}else{
		// Parent process
		tRoutinePID = tTemp;
	}

	// If no one open the other side of the two FIFO, and also no INT, here will never reach
	while (NRF905_FALSE == unNeedtoClose){
		usleep(500000);
	}
	NRF905D_LOG_INFO("INT signal was received, exit.");

	kill(tNRF905CommPID, SIGINT);
	kill(tRoutinePID, SIGINT);
	waitpid(tNRF905CommPID, NULL, 0);
	waitpid(tRoutinePID, NULL, 0);

	close(nTaskExecPipe[0]);
	close(nTaskExecPipe[1]);
	exit(0);
}
