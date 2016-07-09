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
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define __USED_NRF905_INTERNAL__
#include "nRF905_d.h"

void sighandler(int32_t signum, siginfo_t *info, void *ptr)
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
	static uint8_t unRF905_SPI_TX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
	static uint8_t unRF905_SPI_RX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
	struct spi_ioc_transfer tSPI_Transfer;

	if (unFrameLength > NRF905_RX_PAYLOAD_LEN){
		NRF905D_LOG_ERR("The frame need to send to nRF905 is too long.");
		return (-1);
	}
	unRF905_SPI_TX_Frame[0] = unCMD;
	memcpy(unRF905_SPI_TX_Frame + 1, pTX_Frame, unFrameLength);
	tSPI_Transfer.tx_buf = (unsigned long)unRF905_SPI_TX_Frame;
	tSPI_Transfer.rx_buf = (unsigned long)unRF905_SPI_RX_Frame;
	tSPI_Transfer.len = unFrameLength + 1;
	tSPI_Transfer.delay_usecs = unSPI_Delay;
	tSPI_Transfer.speed_hz = unSPI_Speed;
	tSPI_Transfer.bits_per_word = unSPI_Bits;

	if (setNRF905Mode(NRF905_MODE_STD_BY) < 0){
		NRF905D_LOG_ERR("Set nRF905 mode failed");
		return (-1);
	}

	if (ioctl(nRF905SPIfd, SPI_IOC_MESSAGE(1), &tSPI_Transfer) < 1){
		NRF905D_LOG_ERR("can't send spi message");
		return (-1);
	}
	return 0;
}

static uint8_t* nRF905_SPI_RD(int32_t nRF905SPIfd, uint8_t unCMD, uint8_t* pRX_Frame, uint8_t unFrameLength)
{
	static uint8_t unRF905_SPI_TX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
	static uint8_t unRF905_SPI_RX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
	struct spi_ioc_transfer tSPI_Transfer;

	if (unFrameLength > NRF905_RX_PAYLOAD_LEN){
		NRF905D_LOG_ERR("The frame need to read from nRF905 is too long.");
		return NULL;
	}
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

	if (setNRF905Mode(NRF905_MODE_STD_BY) < 0){
		NRF905D_LOG_ERR("Set nRF905 mode failed");
		return NULL;
	}

	if (ioctl(nRF905SPIfd, SPI_IOC_MESSAGE(1), &tSPI_Transfer) < 1){
		NRF905D_LOG_ERR("can't send spi message");
		return NULL;
	}
	if (NULL == pRX_Frame){
		return unRF905_SPI_RX_Frame;
	}else{
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
		NRF905D_LOG_ERR("nRF905 control register initialization failed at nRF905 TX.");
		return -1;
	}
	nGetAddrFromCH_NO(NRF905_CR_DEFAULT[0] | ((uint16_t)(NRF905_CR_DEFAULT[1] & 0x01) << 8), (uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)));
	pRXwStatus = nRF905_SPI_RD(nRF905SPIfd, NRF905_CMD_RC(0), NULL, ARRAY_SIZE(NRF905_CR_DEFAULT));
	if (NULL == pRXwStatus){
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
		NRF905D_LOG_ERR("Set nRF905's RX address in CR failed.");
		return (-1);
	}
	if (nRF905_SPI_WR(nRF905SPI_Fd, NRF905_CMD_WTA,
			(uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)), NRF905_TX_ADDR_LEN) < 0){
		NRF905D_LOG_ERR("Set nRF905's TX address in CR failed.");
		return (-1);
	}
	return 0;
}

uint64_t getTimeDiffInUs(struct timeval tLastTime, struct timeval tCurrentTime)
{
	return abs(tCurrentTime.tv_usec - tLastTime.tv_usec + (tCurrentTime.tv_usec - tLastTime.tv_usec) * US_PER_SECONDE);
}

static int32_t nRF905ReceiveFrame(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask)
{
	struct timeval tLastTime, tCurrentTime;
	if (NULL == tNRF905CommTask.pRX_Frame){
		NRF905D_LOG_ERR("No place to save received frame.");
		return (-1);
	}

	// Start listen
	setNRF905Mode(NRF905_MODE_BURST_RX);
	// delay until CD set with timeout
	gettimeofday(&tLastTime, NULL);
	gettimeofday(&tCurrentTime, NULL);
	while (getTimeDiffInUs(tLastTime, tCurrentTime) < AFTER_SET_BURST_RX_MAX_CD_DELAY_US){
		if (bIsCarrierDetected() == NRF905_TRUE){
			break;
		}
		gettimeofday(&tCurrentTime, NULL);
	}
	if (getTimeDiffInUs(tLastTime, tCurrentTime) >= AFTER_SET_BURST_RX_MAX_CD_DELAY_US){
		setNRF905Mode(NRF905_MODE_STD_BY);
		NRF905D_LOG_ERR("Detect carrier failed.");
		return (-1);
	}
	// delay until AM set with timeout
	gettimeofday(&tLastTime, NULL);
	gettimeofday(&tCurrentTime, NULL);
	while (getTimeDiffInUs(tLastTime, tCurrentTime) < AFTER_CD_MAX_AM_DELAY_US){
		if (bIsAddressMatch() == NRF905_TRUE){
			break;
		}
		gettimeofday(&tCurrentTime, NULL);
	}
	if (getTimeDiffInUs(tLastTime, tCurrentTime) >= AFTER_CD_MAX_AM_DELAY_US){
		setNRF905Mode(NRF905_MODE_STD_BY);
		NRF905D_LOG_ERR("Address match failed.");
		return (-1);
	}
	// delay until DR set with timeout
	gettimeofday(&tLastTime, NULL);
	gettimeofday(&tCurrentTime, NULL);
	while (getTimeDiffInUs(tLastTime, tCurrentTime) < AFTER_AM_MAX_DR_DELAY_US){
		if (bIsDataReady() == NRF905_TRUE){
			break;
		}
		gettimeofday(&tCurrentTime, NULL);
	}
	if (getTimeDiffInUs(tLastTime, tCurrentTime) >= AFTER_AM_MAX_DR_DELAY_US){
		setNRF905Mode(NRF905_MODE_STD_BY);
		NRF905D_LOG_ERR("Address match failed.");
		return (-1);
	}

	// start SPI read RX payload from nRF905
	if (nRF905_SPI_RD(nRF905SPI_Fd, NRF905_CMD_RRP, tNRF905CommTask.pRX_Frame, NRF905_RX_ADDR_LEN) == NULL){
		NRF905D_LOG_ERR("Read RX payload from nRF905 failed.");
		return (-1);
	}

	return 0;
}

static int32_t nRF905SendFrame(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask)
{
	struct timeval tLastTime, tCurrentTime;

	NRF905D_LOG_INFO("nRF905 start send frame.");

	if (nRF905_SPI_WR(nRF905SPI_Fd, NRF905_CMD_WTP, tNRF905CommTask.pTX_Frame, NRF905_TX_PAYLOAD_LEN) < 0){
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
			gettimeofday(&tCurrentTime, NULL);
		}
		if (getTimeDiffInUs(tLastTime, tCurrentTime) >= AFTER_SET_BURST_TX_MAX_DELAY_US){
			setNRF905Mode(NRF905_MODE_STD_BY);
			NRF905D_LOG_ERR("Data transmit failed.");
			return (-1);
		}
		// start to read response
		if (nRF905ReceiveFrame(nRF905SPI_Fd, tNRF905CommTask) < 0){
			NRF905D_LOG_ERR("Data receive failed.");
			return (-1);
		}
		// If receive OK, frame was saved in the tNRF905CommTask.pRX_Frame
		setNRF905Mode(NRF905_MODE_STD_BY);
		if (memcmp(tNRF905CommTask.pTX_Frame, tNRF905CommTask.pRX_Frame + 1, tNRF905CommTask.unCommByteNum) != 0){
			NRF905D_LOG_ERR("The received frame is different with sent one.");
			return (-1);
		}
	}

	return 0;
}

static int32_t nRF905Hopping(int32_t nRF905SPI_Fd)
{
//	uint32_t unCD_RetryCNT;
	uint32_t unTX_RetryCNT;
	uint32_t unHoppingTableIndex;
	nRF905CommTask_t tNRF905HoppingTask;
	static uint8_t unHoppingPayload[NRF905_TX_PAYLOAD_LEN] = {0xA5, 0xA5, 0xDC, 0xCD, };

	NRF905D_LOG_INFO("Hopping procedure start.");
	tNRF905HoppingTask.tCommType = NRF905_COMM_TYPE_TX_PKG;
	tNRF905HoppingTask.unCommByteNum = NRF905_TX_PAYLOAD_LEN;
	tNRF905HoppingTask.pTX_Frame = unHoppingPayload;

	for (unTX_RetryCNT = 0; unTX_RetryCNT < HOPPING_MAX_RETRY_NUM; unTX_RetryCNT++){
		for (unHoppingTableIndex = 0; unHoppingTableIndex < ARRAY_SIZE(unRF_HOPPING_TABLE); unHoppingTableIndex++){
			if (nSetNRF905ChnPwr(nRF905SPI_Fd, unRF_HOPPING_TABLE[unHoppingTableIndex]) < 0){
				NRF905D_LOG_ERR("Can not set nRF905's frequency and power.");
				continue;
			}else{
				if (nRF905SendFrame(nRF905SPI_Fd, tNRF905HoppingTask) < 0){
					NRF905D_LOG_ERR("Send frame error during hopping. Try next channel.");
					break;
				}else{
					NRF905D_LOG_INFO("Hopping success, %u was used as parameter.", tRemoteControlMap.unNRF905RX_Address);
					tRemoteControlMap.unNRF905CommSendFrameErr = 0;
					return 0;
				}
				usleep(HOPPING_TX_RETRY_DELAY_US);
			}
		}
	}
	NRF905D_LOG_INFO("Hopping failed.");
	return (-1);
}

static int32_t nNRF905ExecuteTask(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask)
{
	if (nRF905SendFrame(nRF905SPI_Fd, tNRF905CommTask) < 0){
		NRF905D_LOG_ERR("nRF905 send frame error, start hopping.");
		tRemoteControlMap.unNRF905CommSendFrameErr++;
		tRemoteControlMap.unNRF905CommSendFrameErrTotal++;
		if (nRF905Hopping(nRF905SPI_Fd) < 0){
			NRF905D_LOG_ERR("Can not find any valid receiver in air.");
			return (-1);
		}else{
			if (nRF905SendFrame(nRF905SPI_Fd, tNRF905CommTask) < 0){
				NRF905D_LOG_ERR("WTF, hopping OK but normal frame can not be sent out?");
				return (-1);
			}else{
				return 0;
			}
		}
	}else{
		tRemoteControlMap.unNRF905CommSendFrameErr = 0;
		tRemoteControlMap.unNRF905CommSendFrameOK++;
		return 0;
	}
}

static void* pNRF905Comm(void *ptr)
{
	nRF905CommTask_t tNRF905CommTask;
	nRF905ThreadPara_t tNRF905CommThreadPara = *((nRF905ThreadPara_t *)ptr);

	NRF905D_LOG_INFO("nRF905 communication thread start.");
	while (NRF905_FALSE == unNeedtoClose){
		if (read(tNRF905CommThreadPara.nTaskReadPipe, &tNRF905CommTask, sizeof(nRF905CommTask_t)) > 0){
			if (tNRF905CommTask.unCommByteNum > 0){
				tNRF905CommTask.pTX_Frame = malloc(tNRF905CommTask.unCommByteNum);
				if (NULL == tNRF905CommTask.pTX_Frame){
					NRF905D_LOG_ERR("WTF malloc fail??");
				}else{
					if (read(tNRF905CommThreadPara.nTaskReadPipe, tNRF905CommTask.pTX_Frame, tNRF905CommTask.unCommByteNum) > 0){
						tNRF905CommTask.pRX_Frame = malloc(tNRF905CommTask.unCommByteNum + 1);
						if (NULL != tNRF905CommTask.pRX_Frame){
							NRF905D_LOG_INFO("One ACK task was successfully gotten from pipe.");
							nNRF905ExecuteTask(tNRF905CommThreadPara.nRF905SPI_Fd, tNRF905CommTask);
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
	NRF905D_LOG_INFO("nRF905 communication thread exit.");
	pthread_exit("nRF905 communication thread exit.");
}

void* pRoutineWork(void *ptr)
{
	nRF905ThreadPara_t tRoutineWorkThreadPara = *((nRF905ThreadPara_t *)ptr);
	nRF905CommTask_t tNRF905CommTask;
	static int8_t unACK_Payload[NRF905_TX_PAYLOAD_LEN] = {0xA5, 0xA5, 0xDC, 0xCD, };

	NRF905D_LOG_INFO("nRoutine work thread start.");
	tNRF905CommTask.tCommType = NRF905_COMM_TYPE_TX_PKG;
	tNRF905CommTask.unCommByteNum = NRF905_TX_PAYLOAD_LEN;

	while (NRF905_FALSE == unNeedtoClose){
		if (write(tRoutineWorkThreadPara.nTaskReadPipe, &tNRF905CommTask, sizeof(nRF905CommTask_t)) > 0 ){
			if (write(tRoutineWorkThreadPara.nTaskReadPipe, unACK_Payload, NRF905_TX_PAYLOAD_LEN) > 0 ){
				// increase some statistic internal variable here
				NRF905D_LOG_INFO("One ACK task was successfully added into pipe.");
			}else{
				NRF905D_LOG_ERR("Write task communication payload to pipe error with code:%d", errno);
			}
		}else{
			NRF905D_LOG_ERR("Write task communication type to pipe error with code:%d", errno);
		}
		usleep(ROUTINE_TASK_INTERVAL_US);
	}
	NRF905D_LOG_INFO("Routine work thread exit.");
	pthread_exit("Routine work exit.");
}

int32_t main(void) {
	int32_t nRF905SPI_Fd;
	nRF905ThreadPara_t tNRF905CommThreadPara;
	nRF905ThreadPara_t tRoutineWorkThreadPara;
	int32_t nRet;
	struct sigaction tSignalAction;
	pthread_t tRoutineThread, tNRF905CommThread;
	int32_t nTaskExecPipe[2];
	int32_t nNRF905DataFIFO_C_Read, nNRF905DataFIFO_C_Write;

    tSignalAction.sa_sigaction = sighandler;
    tSignalAction.sa_flags = SA_SIGINFO;

    sigaction(SIGTERM, &tSignalAction, NULL);

	puts("!!!nRF905 Daemon start!!!"); /* prints !!!nRF905 Daemon start!!! */

	NRF905D_LOG_INFO("nRF905 Daemon start...");

	printf("Initialize GPIO.\n");
	if (nInitNRF905GPIO() < 0){
		NRF905D_LOG_ERR("Initialize nRF905 GPIO failed.");
		return nDisableSPI_GPIO();
	}

	nRF905SPI_Fd = open(NRF905_SPI_DEVICE, O_RDWR);
	if (nRF905SPI_Fd < 0) {
		NRF905D_LOG_ERR("Can't open device %s.", NRF905_SPI_DEVICE);
		return nDisableSPI_GPIO();
	}

	printf("Initialize SPI.\n");
	if (nRF905SpiInitial(nRF905SPI_Fd) < 0){
		close(nRF905SPI_Fd);
		return nDisableSPI_GPIO();
	}

	printf("Initialize nRF905 control register.\n");
	if (nRF905CR_Initial(nRF905SPI_Fd) < 0){
		close(nRF905SPI_Fd);
		NRF905D_LOG_ERR("nRF905CR_Initial failed.");
		return nDisableSPI_GPIO();
	}

	// Prepare the pipe
	nRet = pipe(nTaskExecPipe);
	if(nRet < 0){
		NRF905D_LOG_ERR("Open task execution pipe failed with error:%d.", nRet);
		close(nRF905SPI_Fd);
		return nDisableSPI_GPIO();
	}

	// Start little birds
	printf("Start nRF905 communication thread.\n");
	tNRF905CommThreadPara.nRF905SPI_Fd = nRF905SPI_Fd;
	tNRF905CommThreadPara.nTaskReadPipe = nTaskExecPipe[0];
	nRet = pthread_create(&tNRF905CommThread, NULL, pNRF905Comm, &tNRF905CommThreadPara);
	if(nRet < 0){
		NRF905D_LOG_ERR("Start nRF905 communication thread failed with error:%d.", nRet);
		close(nTaskExecPipe[0]);
		close(nTaskExecPipe[1]);
		close(nRF905SPI_Fd);
		return nDisableSPI_GPIO();
	}

	printf("Start routine work thread.\n");
	tRoutineWorkThreadPara.nRF905SPI_Fd = nRF905SPI_Fd;
	tRoutineWorkThreadPara.nTaskReadPipe = nTaskExecPipe[1];
	nRet = pthread_create(&tRoutineThread, NULL, pRoutineWork, &tRoutineWorkThreadPara);
	if(nRet < 0){
		NRF905D_LOG_ERR("Start routine thread failed with error:%d.", nRet);
		close(nTaskExecPipe[0]);
		close(nTaskExecPipe[1]);
		close(nRF905SPI_Fd);
		return nDisableSPI_GPIO();
	}

	printf("Create FIFO to handle data sent by PHP from web application.\n");
	unlink(NRF905_DATA_FIFO_C_WR_PATH);
	unlink(NRF905_DATA_FIFO_C_RD_PATH);
	nRet = mkfifo(NRF905_DATA_FIFO_C_WR_PATH, S_IRUSR| S_IWUSR);
	if (nRet < 0) {
		NRF905D_LOG_ERR("mkfifo failed with error:%d.", errno);
		close(nTaskExecPipe[0]);
		close(nTaskExecPipe[1]);
		close(nRF905SPI_Fd);
		return nDisableSPI_GPIO();
	}

	nRet = mkfifo(NRF905_DATA_FIFO_C_RD_PATH, S_IRUSR| S_IWUSR);
	if (nRet < 0) {
		NRF905D_LOG_ERR("mkfifo failed with error:%d.", errno);
		close(nTaskExecPipe[0]);
		close(nTaskExecPipe[1]);
		close(nRF905SPI_Fd);
		unlink(NRF905_DATA_FIFO_C_WR_PATH);
		return nDisableSPI_GPIO();
	}

	// in PHP NRF905_DATA_FIFO_C_RD_PATH should also be opened first
	nNRF905DataFIFO_C_Read = open(NRF905_DATA_FIFO_C_RD_PATH, O_RDONLY);
	if (nNRF905DataFIFO_C_Read < 0){
		NRF905D_LOG_ERR("Open FIFO read pipe with error:%d.", errno);
		close(nTaskExecPipe[0]);
		close(nTaskExecPipe[1]);
		close(nRF905SPI_Fd);
		unlink(NRF905_DATA_FIFO_C_WR_PATH);
		unlink(NRF905_DATA_FIFO_C_RD_PATH);
		return nDisableSPI_GPIO();
	}

	nNRF905DataFIFO_C_Write = open(NRF905_DATA_FIFO_C_WR_PATH, O_WRONLY);
	if (nNRF905DataFIFO_C_Write < 0){
		NRF905D_LOG_ERR("Open FIFO write pipe with error:%d.", errno);
		close(nTaskExecPipe[0]);
		close(nTaskExecPipe[1]);
		close(nRF905SPI_Fd);
		unlink(NRF905_DATA_FIFO_C_WR_PATH);
		unlink(NRF905_DATA_FIFO_C_RD_PATH);
		return nDisableSPI_GPIO();
	}

	// If no one open the other side of the two FIFO, and also no INT, here will never reach
	while (NRF905_FALSE == unNeedtoClose){
		usleep(500000);
	}
	NRF905D_LOG_INFO("INT signal was received, exit.");

	pthread_join(tNRF905CommThread, NULL);
	pthread_join(tRoutineThread, NULL);

	close(nTaskExecPipe[0]);
	close(nTaskExecPipe[1]);
	unlink(NRF905_DATA_FIFO_C_WR_PATH);
	unlink(NRF905_DATA_FIFO_C_RD_PATH);
	close(nRF905SPI_Fd);
	return nDisableSPI_GPIO();
}
