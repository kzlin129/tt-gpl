/* ltc3577-pmic.h
 *
 * Control driver for LTC3577 PMIC.
 *
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Authors: Benoit Leffray <benoit.leffray@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef INCLUDE_LINUX_LTC3577_PMIC_H
#define INCLUDE_LINUX_LTC3577_PMIC_H


typedef enum {eCHARGING_500mA, eCHARGING_1A} charge_e;

typedef struct
{
	void (*init)(void);
	void (*set_charge)(charge_e charge);
	unsigned char i2c_addr;
}  ltc3577_pdata_t;

typedef struct
{
	unsigned char *buf;
	int len;
} ltc3577_msg_t;

typedef void (*free_msgs)(ltc3577_msg_t *msgs);

int ltc3577_i2c_send(const char *buf, int count);
int ltc3577_i2c_recv(char *buf, int count);
int ltc3577_i2c_transfer(ltc3577_msg_t *msgs, int num);
int ltc3577_i2c_transfer_async(ltc3577_msg_t *msgs, int num, free_msgs callback);

#define LTC3577_DEVNAME			"tomtom-ltc3577-pmic"
#endif
