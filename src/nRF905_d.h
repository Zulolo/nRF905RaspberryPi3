/*
 * nRF905_d.h
 *
 *  Created on: Jun 19, 2016
 *      Author: zulolo
 */

#ifndef NRF905_D_H_
#define NRF905_D_H_

#include <syslog.h>

#define NRF905_TX_EN_PIN				17
#define NRF905_TRX_CE_PIN				18
#define NRF905_PWR_UP_PIN				27

#define NRF905_CD_PIN					22
#define NRF905_AM_PIN					23
#define NRF905_DR_PIN					24

#define NRF905D_LOG_ERR(arg...)			openlog("nRF905.D", LOG_PID, 0);\
										syslog(LOG_USER | LOG_ERR, arg);\
										closelog()

#define NRF905D_LOG_INFO(arg...)		openlog("nRF905.D", LOG_PID, 0);\
										syslog(LOG_USER | LOG_INFO, arg);\
										closelog()

#define US_PER_SECONDE					1000000

typedef enum _nRF905Boolean {
	NRF905_FALSE = 0,
	NRF905_TRUE = !NRF905_FALSE
}nRF905Boolean_t;

#ifdef __USED_NRF905_INTERNAL__

	#include "GPIOcontrol.h"

	#define __NRF905_EXTERN__
	#define NRF905_DATA_FIFO_FOLDER_PATH		"/var/tmp/"
	#define NRF905_DATA_FIFO_C_WR_PATH			NRF905_DATA_FIFO_FOLDER_PATH "data_fifo_c_wr_php_rd"
	#define NRF905_DATA_FIFO_C_RD_PATH			NRF905_DATA_FIFO_FOLDER_PATH "data_fifo_c_rd_php_wr"
	#define ARRAY_SIZE(a) 						(sizeof(a) / sizeof((a)[0]))
	#define NRF905_SPI_DEVICE					"/dev/spidev0.0"

	/* Here is the RF rule:
	 * UP keep transfer carrier and hopping according to unRF_HOPPING_TABLE every 200ms in no match state which
	 * could be caused by DOWN not powered up or interference (miss communication or communication error exceed certain number).
	 * DOWN will keep hopping according to unRF_HOPPING_TABLE every 10ms if not match. When DOWN detected CD high, send ACK frame.
	 *
	 * The TX&RX address is generated by some algorithm
	 *
	 */
	#define NRF905_RX_ADDRESS_IN_CR					5
	#define NRF905_CMD_WC_MASK						0x0F
	#define NRF905_CMD_WC(unWR_CFG_ByteIndex)		((unWR_CFG_ByteIndex) & NRF905_CMD_WC_MASK)
	#define NRF905_CMD_RC_MASK						0x0F
	#define NRF905_CMD_RC(unRD_CFG_ByteIndex)		(((unRD_CFG_ByteIndex) & NRF905_CMD_RC_MASK) | 0x10)
	#define NRF905_CMD_WTP							0x20
	#define NRF905_CMD_RTP							0x21
	#define NRF905_CMD_WTA							0x22
	#define NRF905_CMD_RTA							0x23
	#define NRF905_CMD_RRP							0x24
	#define NRF905_CMD_CC(unPwrChn)					((unPwrChn) | 0x1000)
	#define CH_MSK_IN_CC_REG						0x01FF

	#define NRF905_TX_ADDR_LEN						4
	#define NRF905_RX_ADDR_LEN						4
	#define NRF905_RX_PAYLOAD_LEN					16
	#define NRF905_TX_PAYLOAD_LEN					16
	#define HOPPING_MAX_CD_RETRY_NUM				20
	#define HOPPING_MAX_TX_RETRY_NUM				20
	#define EXEC_TSK_MAX_CD_RETRY_NUM				HOPPING_MAX_CD_RETRY_NUM
	#define CD_RETRY_DELAY_US						50
	#define HOPPING_TX_RETRY_DELAY_US				500

	#define AFTER_SET_BURST_TX_MAX_DELAY_US			5000
	#define AFTER_SET_BURST_RX_MAX_CD_DELAY_US		10000
	#define AFTER_CD_MAX_AM_DELAY_US				5000
	#define AFTER_AM_MAX_DR_DELAY_US				5000
	#define ROUTINE_TASK_INTERVAL_US				500000

	#define NRF905_MAX_COMM_ERR_BEFORE_HOPPING		20

	typedef enum _nRF905Modes {
		NRF905_MODE_PWR_DOWN = 0,
		NRF905_MODE_STD_BY,
		NRF905_MODE_RD_RX,
		NRF905_MODE_BURST_RX,
		NRF905_MODE_BURST_TX,
		NRF905_MODE_MAX
	}nRF905Mode_t;

	typedef enum _nRF905CommType {
		NRF905_COMM_TYPE_RX_PKG = 0,
		NRF905_COMM_TYPE_TX_PKG
	}nRF905CommType_t;

	typedef enum _nRF905State {
		NRF905_STATE_STDBY = 0,
		NRF905_STATE_NO_CD,
		NRF905_STATE_HOPPING,
		NRF905_STATE_CD,
		NRF905_STATE_AM,
		NRF905_STATE_DR,
		NRF905_STATE_TXING,
		NRF905_STATE_RXING,
		NRF905_STATE_END
	}nRF905State_t;

	typedef struct _CommTask {
		nRF905CommType_t tCommType;
		uint8_t unCommByteNum;
		uint8_t* pTX_Frame;
		uint8_t* pRX_Frame;
	}nRF905CommTask_t;

	typedef struct _NRF905CommThreadPara{
		int32_t nTaskReadPipe;
		int32_t nRF905SPI_Fd;
	}nRF905ThreadPara_t;

	// MSB of CH_NO will always be 0
	static uint8_t NRF905_CR_DEFAULT[] =	{0x4C, 0x08,		// F=(422.4+(0x6C<<1)/10)*1; No retransmission; +6db; NOT reduce receive power
		(NRF905_RX_ADDR_LEN << 4) | NRF905_TX_ADDR_LEN,	// 4 bytes RX & TX address;
		NRF905_RX_PAYLOAD_LEN,
		NRF905_TX_PAYLOAD_LEN, 	// 16 bytes RX & TX package length;
		0x13,
		0xEB,
		0x8A,
		0x01,	// RX address is the CRC32 of CH_NO
		0x0F};	// 16MHz crystal; enable CRC; CRC16

	static const uint16_t unRF_HOPPING_TABLE[] = {	0x884C, 0x883A, 0x8846, 0x8832, 0x884A, 0x8835,
													0x884B, 0x8837, 0x884F, 0x883E, 0x8847, 0x8838,
													0x8844, 0x8834, 0x8843, 0x8834, 0x884B, 0x8839,
													0x884D, 0x883A, 0x884E, 0x883C, 0x8832, 0x883F};

	enum _nRF905PinPosInModeLevel{
		NRF905_PWR_UP_PIN_POS = 0,
		NRF905_TRX_CE_PIN_POS,
		NRF905_TX_EN_PIN_POS,
		NRF905_TX_POS_MAX
	};

	static const uint8_t unNRF905MODE_PIN_LEVEL[][NRF905_TX_POS_MAX] = {{GPIO_LEVEL_LOW, GPIO_LEVEL_LOW, GPIO_LEVEL_LOW},
																		{GPIO_LEVEL_HIGH, GPIO_LEVEL_LOW, GPIO_LEVEL_LOW},
																		{GPIO_LEVEL_HIGH, GPIO_LEVEL_LOW, GPIO_LEVEL_LOW},
																		{GPIO_LEVEL_HIGH, GPIO_LEVEL_HIGH, GPIO_LEVEL_LOW},
																		{GPIO_LEVEL_HIGH, GPIO_LEVEL_HIGH, GPIO_LEVEL_HIGH}};
	static uint8_t unSPI_Mode = SPI_MODE_0;
	static uint8_t unSPI_Bits = 8;
	static uint32_t unSPI_Speed = 5000000;
	static uint16_t unSPI_Delay = 1000;
	static uint8_t unNeedtoClose = NRF905_FALSE;

#else
	#define __NRF905_EXTERN__			extern
#endif

typedef struct _remoteControlMap {
	uint32_t unNRF905CommSendFrameErr;
	uint32_t unNRF905CommSendFrameErrTotal;
	uint32_t unNRF905CommSendFrameOK;
	uint32_t unNRF905ChNoAndPwr;
	uint32_t unNRF905RX_Address;
}RemoteControlMap_t;

__NRF905_EXTERN__ RemoteControlMap_t tRemoteControlMap;
#endif /* NRF905_D_H_ */
