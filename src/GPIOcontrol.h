/*
 * GPIOcontrol.h
 *
 *  Created on: Jun 25, 2016
 *      Author: zulolo
 */

#ifndef GPIOCONTROL_H_
#define GPIOCONTROL_H_

#define GPIO_LEVEL_LOW  					0
#define GPIO_LEVEL_HIGH 					1



int32_t GPIORead(int32_t pin);
int32_t GPIOWrite(int32_t pin, int32_t value);
int32_t nInitNRF905GPIO(void);
int32_t nDisableSPI_GPIO(void);

#endif /* GPIOCONTROL_H_ */
