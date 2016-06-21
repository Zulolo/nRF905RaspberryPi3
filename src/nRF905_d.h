/*
 * nRF905_d.h
 *
 *  Created on: Jun 19, 2016
 *      Author: zulolo
 */

#ifndef NRF905_D_H_
#define NRF905_D_H_

#include <syslog.h>

#define NRF905D_LOG_ERR(arg...)			openlog("nRF905.D", LOG_PID, 0);\
										syslog(LOG_USER | LOG_ERR, arg);\
										closelog()

#define NRF905D_LOG_INFO(arg...)		openlog("nRF905.D", LOG_PID, 0);\
										syslog(LOG_USER | LOG_INFO, arg);\
										closelog()

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
	NRF905_COMM_TYPE_TX_PKG,
	NRF905_COMM_TYPE_WR_CFG,
	NRF905_COMM_TYPE_RD_CFG
}nRF905CommType_t;

typedef struct _CommTask {
	nRF905CommType_t tCommType;
	uint8_t unCommByteNum;
}nRF905CommTask_t;

#endif /* NRF905_D_H_ */
