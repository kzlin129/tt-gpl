/* himax_hx8520.h
 *
 * Control driver for Himax Capacitive Touch Screen IC
 *
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Authors: Chris Liu <chris.liu@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
#ifndef INCLUDE_LINUX_HIMAX_CTSIC_H
#define INCLUDE_LINUX_HIMAX_CTSIC_H

#define POINT_DATA_REG                                  0x86
#define DEVICE_ID_REG                                   0x31
#define VERSION_ID_REG                                  0x32

#define XMIN                                            0
#define XMAX                                            800
#define YMIN                                            0
#define YMAX                                            480
#define PRESSUREMIN                                     0
#define PRESSUREMAX                                     1
#define FINGERSMIN                                      0
#define FINGERSMAX                                      2
#define GESTUREMIN                                      0
#define GESTUREMAX                                      0x10
#define MAXNUMTOUCHES                                   2

#define POINT1_MASK                                     0x1
#define POINT2_MASK                                     0x2

#define ERROR_I2C_ACCESS                                70

struct hx_drv_data_t
{
	struct mutex				lock;
};                                                            

struct hx_i2c_data_received
{
	unsigned char header[4]; // Should be 0xAA, 0x55, 0xAA, 0x55
	unsigned char x1_msb;
	unsigned char x1_lsb;
	unsigned char y1_msb;
	unsigned char y1_lsb;
	unsigned char x2_msb;
	unsigned char x2_lsb;
	unsigned char y2_msb;
	unsigned char y2_lsb;
	unsigned char point_count;
	unsigned char point_id;
	unsigned char CS_High; // checksum high byte
	unsigned char CS_Low; // checksum low byte
} __attribute__((__packed__));                       

struct hx_pdata_t
{
	void (*reset_ts)(void);
	int (*get_intr_line)(void);
};

#define HX_DEVNAME			"himax-cap-ts"
#endif

