/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define MAX_LEN	SPIDEV_MAX_BUFFSIZE
#define DATA_LEN 16

static const char *device = "/dev/spi0";
static const char *data = NULL;
static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t pktsz = MAX_LEN/2;
static uint8_t recv = 0;
static uint8_t send = 0;
static uint32_t speed = 500000;
static uint16_t delay;

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static void transfer(int fd)
{
	int l, ret;
	uint8_t txbuf[MAX_LEN];
	uint8_t rxbuf[MAX_LEN];
	struct spi_ioc_transfer tr = {
		.tx_buf = NULL,
		.rx_buf = NULL,
		.len = pktsz,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	if(send){
	   if(data != NULL){
	      l = strlen(data);
	      printf("data=%s datalen=%d\n", data, strlen(data));
	      for(ret=0; ret<pktsz; ret++){
		txbuf[ret] = *(char *)(data + (ret % l));
	      }
	   }else{
	      for(ret=0; ret<pktsz; ret++)
		txbuf[ret] = ret;
	   }
	   tr.tx_buf = (unsigned long)txbuf;
	}

	if(recv){
	   tr.rx_buf = (unsigned long)rxbuf;
	}

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret <= 0)
		pabort("spi message aborted");

	if(recv)
	   printf("Data Recvd:- %dbytes\n", pktsz);
	for(ret=0; recv && ret<pktsz; ret++){
#if 1
	   if(ret%16 == 0)
		printf("\n");
	   printf("%.2x ", rxbuf[ret]);
#else
	   printf("%c", rxbuf[ret]);
#endif
	}
	printf("\n");
}

void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbpdlHOLC3]\n", prog);
	puts("  -D --device       device to use (default /dev/spi0)\n"
	     "  -v --data         data pattern\n"
	     "  -s --speed        max speed (Hz)\n"
	     "  -d --delay        delay (usec)\n"
	     "  -b --bpw          bits per word \n"
	     "  -p --pkt          packet size \n"
	     "  -x --xfer{0,1,2}  Tx(0)/Rx(1)/TxRx(2) \n"
	     "  -l --loop         loopback\n"
	     "  -H --cpha         clock phase\n"
	     "  -O --cpol         clock polarity\n"
	     "  -S --slave        Slave mode\n"
	     "  -L --lsb          least significant bit first\n"
	     "  -C --cs-high      chip select active high\n"
	     "  -3 --3wire        SI/SO signals shared\n");
	exit(1);
}

void parse_opts(int argc, char *argv[])
{
	int xfr = 0;

	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "data",    1, 0, 'v' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "pkt",     1, 0, 'p' },
			{ "xfer",    1, 0, 'x' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "slave",   0, 0, 'S' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:v:s:d:b:p:x:lHOSLC3", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 'v':
			data = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'p':
			pktsz = atoi(optarg);
			break;
		case 'x':
			xfr = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'S':
			mode |= SPI_SLAVE;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}

	if(xfr == 2){	/* Full Duplex */
	   recv = 1;
	   send = 1;
	}else if(xfr == 1){ /* Rx only */
	   recv = 1;
	   send = 0;
	}else{		/* Tx only */
	   recv = 0;
	   send = 1;
	}

}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;

	parse_opts(argc, argv);

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	transfer(fd);

	close(fd);

	return ret;
}
