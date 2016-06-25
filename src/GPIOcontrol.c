
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "GPIOcontrol.h"
#include "nRF905_d.h"

#define GPIO_INPUT  						0
#define GPIO_OUTPUT 						1

static int32_t GPIOExport(int32_t pin)
{
#define BUFFER_MAX 3
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;
	int32_t fd;

	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open export for writing!\n");
		return(-1);
	}

	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);
	return(0);
}

static int32_t GPIOUnexport(int32_t pin)
{
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;
	int32_t fd;

	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open unexport for writing!\n");
		return(-1);
	}

	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);
	return(0);
}

int32_t nDisableSPI_GPIO(void)
{
	/*
	 * Disable GPIO pins
	 */
	if ((-1 == GPIOUnexport(NRF905_TX_EN_PIN)) || (-1 == GPIOUnexport(NRF905_TRX_CE_PIN)) ||
			(-1 == GPIOUnexport(NRF905_PWR_UP_PIN)) || (-1 == GPIOExport(NRF905_CD_PIN)) ||
			(-1 == GPIOExport(NRF905_AM_PIN)) || (-1 == GPIOExport(NRF905_DR_PIN))){
		NRF905D_LOG_ERR("GPIO pin disable failed.");
		return (-1);
	}
	return 0;
}

static int32_t nEnableSPI_GPIO(void)
{
	/*
	 * Enable GPIO pins
	 */
	if ((-1 == GPIOExport(NRF905_TX_EN_PIN)) || (-1 == GPIOExport(NRF905_TRX_CE_PIN)) ||
			(-1 == GPIOExport(NRF905_PWR_UP_PIN)) || (-1 == GPIOExport(NRF905_CD_PIN)) ||
			(-1 == GPIOExport(NRF905_AM_PIN)) || (-1 == GPIOExport(NRF905_DR_PIN))){
		NRF905D_LOG_ERR("GPIO pin enable failed.");
		return (-1);
	}
	return 0;
}

static int32_t GPIODirection(int32_t pin, int32_t dir)
{
	static const char s_directions_str[]  = "in\0out";

#define DIRECTION_MAX 35
	char path[DIRECTION_MAX];
	int32_t fd;

	snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
	fd = open(path, O_WRONLY);
	if (-1 == fd) {
		printf("Failed to open pin %d direction for writing!\n", pin);
		fprintf(stderr, "Failed to open gpio direction for writing!\n");
		return(-1);
	}

	if (-1 == write(fd, &s_directions_str[GPIO_INPUT == dir ? 0 : 3], GPIO_INPUT == dir ? 2 : 3)) {
		fprintf(stderr, "Failed to set direction!\n");
		return(-1);
	}

	close(fd);
	return(0);
}

int32_t nInitNRF905GPIO(void)
{
	if (nEnableSPI_GPIO() < 0){
		NRF905D_LOG_ERR("Enable SPI GPIO pin failed.");
		return (-1);
	}
	usleep(100000);
	/*
	 * Set GPIO directions
	 */
	if ((-1 == GPIODirection(NRF905_TX_EN_PIN, GPIO_OUTPUT)) || (-1 == GPIODirection(NRF905_TRX_CE_PIN, GPIO_OUTPUT)) ||
			(-1 == GPIODirection(NRF905_PWR_UP_PIN, GPIO_OUTPUT))){
		NRF905D_LOG_ERR("Config GPIO output pin failed.");
		return (-1);
	}

	if ((-1 == GPIODirection(NRF905_CD_PIN, GPIO_INPUT)) || (-1 == GPIODirection(NRF905_AM_PIN, GPIO_INPUT)) ||
			(-1 == GPIODirection(NRF905_DR_PIN, GPIO_INPUT))){
		NRF905D_LOG_ERR("Config GPIO output pin failed.");
		return (-1);
	}
	usleep(1000);
	return 0;
}

int32_t GPIORead(int32_t pin)
{
#define VALUE_MAX 30
	char path[VALUE_MAX];
	char value_str[3];
	int32_t fd;

	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_RDONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open gpio value for reading!\n");
		return (-1);
	}

	if (-1 == read(fd, value_str, 3)) {
		fprintf(stderr, "Failed to read value!\n");
		return (-1);
	}

	close(fd);

	return(atoi(value_str));
}

int32_t GPIOWrite(int32_t pin, int32_t value)
{
	static const char s_values_str[] = "01";

	char path[VALUE_MAX];
	int32_t fd;

	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_WRONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open gpio value for writing!\n");
		return (-1);
	}

	if (1 != write(fd, &s_values_str[GPIO_LEVEL_LOW == value ? 0 : 1], 1)) {
		fprintf(stderr, "Failed to write value!\n");
		return (-1);
	}

	close(fd);
	return(0);
}
