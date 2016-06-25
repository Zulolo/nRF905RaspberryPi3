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
	unNeedtoClose = 1;
}

int32_t setNRF905Mode(nRF905Mode_t tNRF905Mode)
{
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

static int32_t nRF905_SPI_WR(int32_t nRF905SPIfd, uint8_t* pTX_Frame, uint8_t unFrameLength)
{
	uint8_t* pRX_Frame;
	struct spi_ioc_transfer tSPI_Transfer;

	pRX_Frame = malloc(unFrameLength + 1);
	if (NULL == pRX_Frame){
		NRF905D_LOG_ERR("nRF905TX failed because of no RAM.");
		return (-1);
	}
	tSPI_Transfer.tx_buf = (unsigned long)pTX_Frame;
	tSPI_Transfer.rx_buf = (unsigned long)pRX_Frame;
	tSPI_Transfer.len = unFrameLength;
	tSPI_Transfer.delay_usecs = unSPI_Delay;
	tSPI_Transfer.speed_hz = unSPI_Speed;
	tSPI_Transfer.bits_per_word = unSPI_Bits;

	if (setNRF905Mode(NRF905_MODE_STD_BY) < 0){
		NRF905D_LOG_ERR("Set nRF905 mode failed");
		return (-1);
	}

	if (ioctl(nRF905SPIfd, SPI_IOC_MESSAGE(1), &tSPI_Transfer) < 1){
		free(pRX_Frame);
		NRF905D_LOG_ERR("can't send spi message");
		return (-1);
	}
	free(pRX_Frame);
	return 0;
}

static int32_t nRF905_SPI_READ(int32_t nRF905SPIfd, uint8_t* pTX_Frame, uint8_t* pRX_Frame, uint8_t unFrameLength)
{
	struct spi_ioc_transfer tSPI_Transfer;

	tSPI_Transfer.tx_buf = (unsigned long)pTX_Frame;
	tSPI_Transfer.rx_buf = (unsigned long)pRX_Frame;
	tSPI_Transfer.len = unFrameLength;
	tSPI_Transfer.delay_usecs = unSPI_Delay;
	tSPI_Transfer.speed_hz = unSPI_Speed;
	tSPI_Transfer.bits_per_word = unSPI_Bits;

	if (setNRF905Mode(NRF905_MODE_STD_BY) < 0){
		NRF905D_LOG_ERR("Set nRF905 mode failed");
		return (-1);
	}

	if (ioctl(nRF905SPIfd, SPI_IOC_MESSAGE(1), &tSPI_Transfer) < 1){
		NRF905D_LOG_ERR("can't send spi message");
		return -1;
	}
	return 0;
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
	uint8_t* pTXwCMD;
	uint8_t* pRXwStatus;

	pTXwCMD = malloc(ARRAY_SIZE(NRF905_CR_DEFAULT) + 1);
	if (NULL == pTXwCMD){
		NRF905D_LOG_ERR("nRF905CR_Initial failed because of no TX RAM.");
		return (-1);
	}
	pTXwCMD[0] = NRF905_CMD_WC(0);
	memcpy(pTXwCMD + 1, NRF905_CR_DEFAULT, ARRAY_SIZE(NRF905_CR_DEFAULT));
	if (nRF905_SPI_WR(nRF905SPIfd, pTXwCMD, ARRAY_SIZE(NRF905_CR_DEFAULT) + 1) < 0){
		free(pTXwCMD);
		NRF905D_LOG_ERR("nRF905 control register initialization failed at nRF905 TX.");
		return -1;
	}
	nGetAddrFromCH_NO(NRF905_CR_DEFAULT[0] | ((uint16_t)(NRF905_CR_DEFAULT[1] & 0x01) << 8), unNRF905RX_AddressGlobal);
	pRXwStatus = malloc(ARRAY_SIZE(NRF905_CR_DEFAULT) + 1);
	if (NULL == pRXwStatus){
		free(pTXwCMD);
		NRF905D_LOG_ERR("nRF905CR_Initial failed because of no RX RAM.");
		return (-1);
	}
	pTXwCMD[0] = NRF905_CMD_RC(0);
	if (nRF905_SPI_READ(nRF905SPIfd, pTXwCMD, pRXwStatus, ARRAY_SIZE(NRF905_CR_DEFAULT) + 1) < 0){
		free(pTXwCMD);
		free(pRXwStatus);
		NRF905D_LOG_ERR("nRF905 control register initialization failed at nRF905 RX.");
		return -1;
	}
	if (memcmp(pRXwStatus + 1, NRF905_CR_DEFAULT, ARRAY_SIZE(NRF905_CR_DEFAULT)) != 0){
		free(pTXwCMD);
		free(pRXwStatus);
		NRF905D_LOG_ERR("nRF905 control register initialization failed at comparing CR value.");
		return -1;
	}
	free(pTXwCMD);
	free(pRXwStatus);
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

static int32_t nSetNRF905FrqPwr(int32_t nRF905SPI_Fd, uint16_t unFrqPwr)
{
//	uint8_t unSPI_WR_Data[5];
	unFrqPwr = NRF905_CMD_CC(unFrqPwr);
	if (nRF905_SPI_WR(nRF905SPI_Fd, (uint8_t*)(&unFrqPwr), 2) < 0){
		NRF905D_LOG_ERR("Set nRF905's frequency and power failed.");
		return (-1);
	}
	nGetAddrFromCH_NO(unFrqPwr & CH_MSK_IN_CC_REG, unNRF905RX_AddressGlobal);
//	unSPI_WR_Data[0] = NRF905_CMD_WC(NRF905_RX_ADDRESS_IN_CR);
//	memcpy(unSPI_WR_Data + 1, unNRF905RX_AddressGlobal, 4);
//	if (nRF905_SPI_WR(nRF905SPI_Fd, unSPI_WR_Data, 5) < 0){
//		NRF905D_LOG_ERR("Set nRF905's RX address in CR failed.");
//		return (-1);
//	}
	return 0;
}

static int32_t nRF905Hopping(int32_t nRF905SPI_Fd)
{
	uint32_t unCD_RetryCNT;
	uint32_t unHoppingTableIndex;

	for (unHoppingTableIndex = 0; unHoppingTableIndex < ARRAY_SIZE(unRF_HOPPING_TABLE); unHoppingTableIndex++){
		if (nSetNRF905FrqPwr(nRF905SPI_Fd, unRF_HOPPING_TABLE[unHoppingTableIndex]) < 0){
			NRF905D_LOG_ERR("Can not set nRF905's frequency and power.");
			continue;
		}else{
			if (setNRF905Mode(NRF905_MODE_BURST_RX) < 0){
				NRF905D_LOG_ERR("Set nRF905 mode error during hopping.");
				continue;
			}
			for (unCD_RetryCNT = 0; unCD_RetryCNT < MAX_CD_RETRY_NUM; unCD_RetryCNT++){
				if (bIsCarrierDetected() == NRF905_TRUE){
					return 0;
				}
				usleep(CD_RETRY_DELAY_US);
			}
		}
	}
	return (-1);
}

static int32_t nRF905SendFrame(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask, uint8_t* pNRF905TX_Address)
{
	uint8_t unSPI_WR_TX_AddressFrame[NRF905_TX_ADDR_LEN + 1];
	uint8_t unSPI_WR_TX_PayloadFrame[NRF905_TX_PAYLOAD_LEN + 1];
	uint8_t unSPI_RD_Frame[NRF905_RX_PAYLOAD_LEN + 1];

	unSPI_WR_TX_AddressFrame[0] = NRF905_CMD_WTA;
	memcpy(unSPI_WR_TX_AddressFrame + 1, pNRF905TX_Address, NRF905_TX_ADDR_LEN);
	if (nRF905_SPI_WR(nRF905SPI_Fd, unSPI_WR_TX_AddressFrame, NRF905_TX_ADDR_LEN + 1) < 0){
		NRF905D_LOG_ERR("Set TX address error.");
		return (-1);
	}else{
		unSPI_WR_TX_PayloadFrame[0] = NRF905_CMD_WTP;
		memcpy(unSPI_WR_TX_PayloadFrame + 1, tNRF905CommTask.pTX_Frame, NRF905_TX_PAYLOAD_LEN);
		if (nRF905_SPI_WR(nRF905SPI_Fd, unSPI_WR_TX_PayloadFrame, NRF905_TX_PAYLOAD_LEN + 1) < 0){
			NRF905D_LOG_ERR("Write TX payload error.");
			return (-1);
		}else{
			// TX payload was successfully written into nRF905's register
			// Start transmit

			// delay a little. Once the TX_EN has been set, the data will be sent out no matter TX_EN is reset or not.

			// wait until DR reset

			// start to read response

			// monitor CD, AM and DR with timeout

		}
	}

	return 0;
}

static int32_t nNRF905ExecuteTask(int32_t nRF905SPI_Fd, nRF905CommTask_t tNRF905CommTask)
{
	nRF905State_t tNRF905State = NRF905_STATE_STDBY;
	uint32_t unCD_RetryCNT;
	uint32_t unAM_RetryCNT;
	uint32_t unDR_RetryCNT;
	uint32_t unTX_RetryCNT;
	uint32_t unRX_RetryCNT;
	// Use state machine here to control mode and hopping
	while(tNRF905State != NRF905_STATE_END){
		switch(tNRF905State){
		case NRF905_STATE_STDBY:
			if (setNRF905Mode(NRF905_MODE_BURST_RX) < 0){
				NRF905D_LOG_ERR("Set nRF905 mode error during executing task.");
				return (-1);
			}
			for (unCD_RetryCNT = 0; unCD_RetryCNT < MAX_CD_RETRY_NUM; unCD_RetryCNT++){
				if (bIsCarrierDetected() == NRF905_TRUE){
					break;
				}
				usleep(CD_RETRY_DELAY_US);
			}
			if (MAX_CD_RETRY_NUM == unCD_RetryCNT){
				tNRF905State = NRF905_STATE_NO_CD;
			}else{
				tNRF905State = NRF905_STATE_CD;
			}
			break;

		case NRF905_STATE_NO_CD:
			tNRF905State = NRF905_STATE_HOPPING;
			break;

		case NRF905_STATE_HOPPING:
			if (nRF905Hopping(nRF905SPI_Fd) < 0){
				NRF905D_LOG_ERR("Can not find any valid carrier in air.");
				tNRF905State = NRF905_STATE_END;
			}else{
				tNRF905State = NRF905_STATE_CD;
			}
			break;

		case NRF905_STATE_CD:
			if (nRF905SendFrame(nRF905SPI_Fd, tNRF905CommTask, unNRF905RX_AddressGlobal) < 0){
				NRF905D_LOG_ERR("nRF905 send frame error.");
				tNRF905State = NRF905_STATE_END;
			}else{
				tNRF905State = NRF905_STATE_TXING;
			}
			break;

		default:
			NRF905D_LOG_ERR("Invalid nRF905 state");
			return (-1);
			break;
		}
	}

	return 0;
}

static void* pNRF905Comm(void *ptr)
{
	nRF905CommTask_t tNRF905CommTask;
	nRF905CommThreadPara_t tNRF905CommThreadPara = *((nRF905CommThreadPara_t *)ptr);

	while (unNeedtoClose == 0){
		if (read(tNRF905CommThreadPara.nTaskReadPipe, &tNRF905CommTask, sizeof(nRF905CommTask_t)) > 0){
			if (tNRF905CommTask.unCommByteNum > 0){
				tNRF905CommTask.pTX_Frame = malloc(tNRF905CommTask.unCommByteNum);
				if (NULL == tNRF905CommTask.pTX_Frame){
					NRF905D_LOG_ERR("WTF malloc fail??");
				}else{
					if (read(tNRF905CommThreadPara.nTaskReadPipe, tNRF905CommTask.pTX_Frame, tNRF905CommTask.unCommByteNum) > 0){
						nNRF905ExecuteTask(tNRF905CommThreadPara.nRF905SPI_Fd, tNRF905CommTask);
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
	pthread_exit(0);
}

void* pRoutineWork(void *ptr)
{

	pthread_exit("Routine work exit.");
}

int32_t main(void) {
	int32_t nRF905SPI_Fd;
	nRF905CommThreadPara_t tNRF905CommThreadPara;
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

	if (nInitNRF905GPIO() < 0){
		NRF905D_LOG_ERR("Initialize nRF905 GPIO failed.");
		return nDisableSPI_GPIO();
	}

	nRF905SPI_Fd = open(nRF905SPI_Device, O_RDWR);
	if (nRF905SPI_Fd < 0) {
		NRF905D_LOG_ERR("Can't open device %s.", nRF905SPI_Device);
		return nDisableSPI_GPIO();
	}

	if (nRF905SpiInitial(nRF905SPI_Fd) < 0){
		close(nRF905SPI_Fd);
		return nDisableSPI_GPIO();
	}

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
	nRet = pthread_create(&tNRF905CommThread, NULL, pNRF905Comm, nTaskExecPipe);
	if(nRet < 0){
		NRF905D_LOG_ERR("Start nRF905 communication thread failed with error:%d.", nRet);
		close(nRF905SPI_Fd);
		return nDisableSPI_GPIO();
	}

	tNRF905CommThreadPara.nRF905SPI_Fd = nRF905SPI_Fd;
	tNRF905CommThreadPara.nTaskReadPipe = nTaskExecPipe[0];
	nRet = pthread_create(&tRoutineThread, NULL, pRoutineWork, &tNRF905CommThreadPara);
	if(nRet < 0){
		NRF905D_LOG_ERR("Start routine thread failed with error:%d.", nRet);
		close(nRF905SPI_Fd);
		return nDisableSPI_GPIO();
	}

	unlink(NRF905_DATA_FIFO_C_WR_PATH);
	unlink(NRF905_DATA_FIFO_C_RD_PATH);
	nRet = mkfifo(NRF905_DATA_FIFO_C_WR_PATH, S_IRUSR| S_IWUSR);
	if (nRet < 0) {
		NRF905D_LOG_ERR("mkfifo failed with error:%d.", errno);
		close(nRF905SPI_Fd);
		return nDisableSPI_GPIO();
	}

	nRet = mkfifo(NRF905_DATA_FIFO_C_RD_PATH, S_IRUSR| S_IWUSR);
	if (nRet < 0) {
		NRF905D_LOG_ERR("mkfifo failed with error:%d.", errno);
		close(nRF905SPI_Fd);
		unlink(NRF905_DATA_FIFO_C_WR_PATH);
		return nDisableSPI_GPIO();
	}

	// in PHP NRF905_DATA_FIFO_C_RD_PATH should also be opened first
	nNRF905DataFIFO_C_Read = open(NRF905_DATA_FIFO_C_RD_PATH, O_RDONLY);
	if (nNRF905DataFIFO_C_Read < 0){
		NRF905D_LOG_ERR("Open FIFO read pipe with error:%d.", errno);
		close(nRF905SPI_Fd);
		unlink(NRF905_DATA_FIFO_C_WR_PATH);
		unlink(NRF905_DATA_FIFO_C_RD_PATH);
		return nDisableSPI_GPIO();
	}

	nNRF905DataFIFO_C_Write = open(NRF905_DATA_FIFO_C_WR_PATH, O_WRONLY);
	if (nNRF905DataFIFO_C_Write < 0){
		NRF905D_LOG_ERR("Open FIFO write pipe with error:%d.", errno);
		close(nRF905SPI_Fd);
		unlink(NRF905_DATA_FIFO_C_WR_PATH);
		unlink(NRF905_DATA_FIFO_C_RD_PATH);
		return nDisableSPI_GPIO();
	}

	// If no one open the other side of the two FIFO, and also no INT, here will never reach
	while (unNeedtoClose == 0){
		usleep(500000);
	}
	NRF905D_LOG_INFO("INT signal was received, exit.");

	pthread_join(tNRF905CommThread, NULL);
	pthread_join(tRoutineThread, NULL);

	unlink(NRF905_DATA_FIFO_C_WR_PATH);
	unlink(NRF905_DATA_FIFO_C_RD_PATH);
	close(nRF905SPI_Fd);
	return nDisableSPI_GPIO();
}
