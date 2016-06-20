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
#include <stdlib.h>
#include <signal.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "nRF905_d.h"

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

#define GPIO_LEVEL_LOW  		0
#define GPIO_LEVEL_HIGH 		1

#define NRF905_TX_EN_PIN		17
#define NRF905_TRX_CE_PIN		18
#define NRF905_PWR_UP_PIN		27

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

enum _nRF905PinPosInModeLevel{
	NRF905_PWR_UP_PIN_POS = 0,
	NRF905_TRX_CE_PIN_POS,
	NRF905_TX_EN_PIN_POS
};

static const char NRF905MODE_PIN_LEVEL[][] = {	{GPIO_LEVEL_LOW, GPIO_LEVEL_LOW, GPIO_LEVEL_LOW},
												{GPIO_LEVEL_HIGH, GPIO_LEVEL_LOW, GPIO_LEVEL_LOW},
												{GPIO_LEVEL_HIGH, GPIO_LEVEL_LOW, GPIO_LEVEL_LOW},
												{GPIO_LEVEL_HIGH, GPIO_LEVEL_HIGH, GPIO_LEVEL_LOW},
												{GPIO_LEVEL_HIGH, GPIO_LEVEL_HIGH, GPIO_LEVEL_HIGH}};
static const char nRF905SPI_Device[] = "/dev/spidev0.0";
static uint8_t unSPI_Mode = SPI_MODE_0;
static uint8_t unSPI_Bits = 8;
static uint32_t unSPI_Speed = 500000;
static uint16_t unSPI_Delay = 1000;
static uint8_t unNeedtoClose = 0;

void sighandler(int32_t signum, siginfo_t *info, void *ptr)
{
	unNeedtoClose = 1;
}

static int32_t nReadRF_CR(int32_t nRF905SPIfd)
{
	int32_t nRet;
	uint8_t unTx[] = {
		0x10,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF
	};
	uint8_t unRx[ARRAY_SIZE(unTx)] = {0, };
	struct spi_ioc_transfer tSPI_Transfer = {
		.tx_buf = (unsigned long)unTx,
		.rx_buf = (unsigned long)unRx,
		.len = ARRAY_SIZE(unTx),
		.delay_usecs = unSPI_Delay,
		.speed_hz = unSPI_Speed,
		.bits_per_word = unSPI_Bits,
	};

	nRet = ioctl(nRF905SPIfd, SPI_IOC_MESSAGE(1), &tSPI_Transfer);
	if (nRet < 1){
		NRF905D_LOG_ERR("can't send spi message");
		return -1;
	}

	for (nRet = 0; nRet < ARRAY_SIZE(unTx); nRet++) {
		if (!(nRet % 6))
			puts("");
		printf("%.2X ", unRx[nRet]);
	}
	puts("");
	return 0;
}

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

int32_t nDisableGPIO(void)
{
	/*
	 * Disable GPIO pins
	 */
	if ((-1 == GPIOUnexport(NRF905_TX_EN_PIN)) || (-1 == GPIOUnexport(NRF905_TRX_CE_PIN)) ||
			(-1 == GPIOUnexport(NRF905_PWR_UP_PIN))){
		NRF905D_LOG_ERR("GPIO pin disable failed.");
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

int32_t nInitGPIO(void)
{
	/*
	 * Enable GPIO pins
	 */
	if ((-1 == GPIOExport(NRF905_TX_EN_PIN)) || (-1 == GPIOExport(NRF905_TRX_CE_PIN)) ||
			(-1 == GPIOExport(NRF905_PWR_UP_PIN))){
		NRF905D_LOG_ERR("GPIO pin enable failed.");
		return (-1);
	}
	usleep(100000);
	/*
	 * Set GPIO directions
	 */
	if ((-1 == GPIODirection(NRF905_TX_EN_PIN, GPIO_OUTPUT)) || (-1 == GPIODirection(NRF905_TRX_CE_PIN, GPIO_OUTPUT)) ||
			(-1 == GPIODirection(NRF905_PWR_UP_PIN, GPIO_OUTPUT))){
		NRF905D_LOG_ERR("Config GPIO pin failed.");
		return (-1);
	}
	return 0;
}

static int32_t GPIORead(int32_t pin)
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

static int32_t GPIOWrite(int32_t pin, int32_t value)
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

int32_t nRF905SetMode(nRF905Modes_t tNRF905Mode)
{
	if (tNRF905Mode >= NRF905_MODE_MAX)
	{
		NRF905D_LOG_ERR("tNRF905Mode error.");
		return (-1);
	}

	if (-1 == GPIOWrite(NRF905_TX_EN_PIN, NRF905MODE_PIN_LEVEL[tNRF905Mode][NRF905_TX_EN_PIN_POS])){
		NRF905D_LOG_ERR("Write GPIO pin %u failed.", NRF905_TX_EN_PIN);
		return (-1);
	}

	if (-1 == GPIOWrite(NRF905_TRX_CE_PIN, NRF905MODE_PIN_LEVEL[tNRF905Mode][NRF905_TRX_CE_PIN_POS])){
		NRF905D_LOG_ERR("Write GPIO pin %u failed.", NRF905_TRX_CE_PIN);
		return (-1);
	}

	if (-1 == GPIOWrite(NRF905_PWR_UP_PIN, NRF905MODE_PIN_LEVEL[tNRF905Mode][NRF905_PWR_UP_PIN_POS])){
		NRF905D_LOG_ERR("Write GPIO pin %u failed.", NRF905_PWR_UP_PIN);
		return (-1);
	}

	return 0;
}

int32_t nRF905Initial(int32_t nRF905SPIfd)
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

int32_t main(void) {

	int32_t nRet = 0;
	int32_t nRF905SPIfd;
	struct sigaction act;

    act.sa_sigaction = sighandler;
    act.sa_flags = SA_SIGINFO;

    sigaction(SIGTERM, &act, NULL);

	puts("!!!nRF905 Daemon start!!!"); /* prints !!!nRF905 Daemon start!!! */

	NRF905D_LOG_INFO("nRF905 Daemon start...");

	nInitGPIO();

	usleep(1000);

	if (nRF905SetMode(NRF905_MODE_PWR_DOWN) < 0)
	{
		return nDisableGPIO();
	}

	usleep(1000);

	nRF905SPIfd = open(nRF905SPI_Device, O_RDWR);
	if (nRF905SPIfd < 0) {
		NRF905D_LOG_ERR("Can't open device %s.", nRF905SPI_Device);
		return nDisableGPIO();
	}

	if (nRF905Initial(nRF905SPIfd) < 0)
	{
		close(nRF905SPIfd);
		return nDisableGPIO();
	}

	while (unNeedtoClose == 0){
		if (nReadRF_CR(nRF905SPIfd) < 0){
			close(nRF905SPIfd);
			return nDisableGPIO();
		}
		usleep(500000);
	}
	NRF905D_LOG_INFO("INT signal was received, exit.");

	close(nRF905SPIfd);
	return nDisableGPIO();
}
