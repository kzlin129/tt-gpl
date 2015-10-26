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

#define XMIN                                            0
#define XMAX                                            800
#define YMIN                                            0
#define YMAX                                            480
#define PRESSUREMIN                                     0
#define PRESSUREMAX                                     1
#define FINGERSMIN                                      0
#define FINGERSMAX                                      2
#define MAXNUMTOUCHES                                   2

#define POINT1_MASK                                     0x1
#define POINT2_MASK                                     0x2
#define POINT_COUNT_MASK				0xF

#define LOW_BYTE_MASK					0xFF

#define OFFSET_COMPANY					0x2000
#define OFFSET_PROJECTNAME				0x200C
#define OFFSET_ENDUSER					0x2018
#define OFFSET_FW_VERSION				0x2024
#define OFFSET_DATE					0x2030

#define LEN_FW_VER					12

#define ERROR_I2C_ACCESS                                70

#define HX_CMD_DEVICEID       0x31
#define HX_CMD_VERSIONID      0x32
#define HX_CMD_MICROOFF       0x35
#define HX_CMD_ROMRDY         0x36
#define HX_CMD_ADCSETTING     0x39
#define HX_CMD_SETFLASHTIM    0x41
#define HX_CMD_SETFLASHTEST   0x42
#define HX_CMD_SETFLASHEN     0x43
#define HX_CMD_SETFLASHADDR   0x44
#define HX_CMD_SETFLASHDATA   0x45
#define HX_CMD_FLASHR         0x46
#define HX_CMD_FLASHPWSTART   0x47
#define HX_CMD_FLASHPW        0x48
#define HX_CMD_FLASHPWEND     0x49
#define HX_CMD_FLASHBPWSTART  0x4A
#define HX_CMD_FLASHBPW       0x4B
#define HX_CMD_FLASHBPWEND    0x4C
#define HX_CMD_FLASHPE        0x4D
#define HX_CMD_FLASHSE        0x4E
#define HX_CMD_FLASHME        0x4F
#define HX_CMD_TSSLPIN        0x80
#define HX_CMD_TSSLPOUT       0x81
#define HX_CMD_TSSOFF         0x82
#define HX_CMD_TSSON          0x83
#define HX_CMD_READONEEVENT   0x85
#define HX_CMD_READALLEVENT   0x86
#define HX_CMD_READLSTEVENT   0x87
#define HX_CMD_CLREVENTSTACK  0x88
#define HX_CMD_TSSWRST        0x9E
#define HX_CMD_SETDEEPSTB     0xD7
#define HX_CMD_SETIDLE        0xF2
#define HX_CMD_SETIDLEDELAY   0xF3

struct hx_drv_data_t
{
	struct mutex					lock;
};                                                            

struct hx_i2c_data_received
{
	unsigned char header[4]; 	// Should be 0xAA, 0x55, 0xA5, 0x5A
	unsigned char x1_msb;
	unsigned char x1_lsb;
	unsigned char y1_msb;
	unsigned char y1_lsb;
	unsigned char x2_msb;
	unsigned char x2_lsb;
	unsigned char y2_msb;
	unsigned char y2_lsb;
	unsigned char p1_area;
	unsigned char p2_area;
	unsigned char p3_area;
	unsigned char p4_area;
	unsigned char point_count;
	unsigned char point_id;
	unsigned char hotkey_info;	// Don't care 
	unsigned char CS_LSB;		// LSB of checksum
} __attribute__((__packed__));                       

struct hx_pdata_t
{
	void (*reset_ts)(void);
	int (*get_intr_line)(void);
};

struct hx_init_settings_t
{
  unsigned char c48[2];
  unsigned char c45[2];
	unsigned char c1[4];
	unsigned char c2[3];
	unsigned char c3[5];
	unsigned char c4[2];
	unsigned char c5[2];
	unsigned char c6[2];
	unsigned char c7[3];
	unsigned char c8[2];
	unsigned char c9[4];
	unsigned char c10[6];
	unsigned char c11[15];
	unsigned char c12[2];
	unsigned char c13[8];
	unsigned char c14[4];
	unsigned char c15[11];
	unsigned char c16[4];
	unsigned char c17[2];
	unsigned char c18[11];
	unsigned char c19[11];
	unsigned char c20[11];
	unsigned char c21[11];
	unsigned char c22[11];
	unsigned char c23[11];
	unsigned char c24[11];
	unsigned char c25[11];
	unsigned char c26[11];
	unsigned char c27[11];
	unsigned char c28[11];
	unsigned char c29[11];
	unsigned char c30[29];
	unsigned char c31[69];
	unsigned char c32[11];
	unsigned char c33[3];
	unsigned char c34[5];
	unsigned char c35[5];
	unsigned char c36[10];
	unsigned char c37[3];
	unsigned char c38[2];
	unsigned char c39[5];
	unsigned char c40[5];
	unsigned char c41[2];
	unsigned char c42[5];
	unsigned char c43[6];
	unsigned char c44[6];
	unsigned char c46[2];
	unsigned char c47[3];
};

#define HX_DEVNAME			"himax-cap-ts"
#endif

