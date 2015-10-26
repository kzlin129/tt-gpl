/* himax-hx8526.c
 *
 * Control driver for Himax Capacitive TouchScreen IC
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Chris Liu <chris.liu@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/himax_hx8526.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/firmware.h>

static struct _hx_i2c {
        struct i2c_client       *client;
        struct work_struct      read_info_work;
} hx_i2c;

static struct workqueue_struct *readinfo_workqueue;
static struct input_dev *hx_cap_ts_input_dev;     /* Representation of an input device */
static struct hx_i2c_data_received hx_i2c_packet_received;
static struct hx_pdata_t pdata;
static struct hx_drv_data_t *drv_data;
static struct firmware hx_firmware;

static int len_i2c_frame_finger_position;

static struct hx_init_settings_t hx_init_settings = {
	.c1 = {0x36,0x0F,0x53,0x01},
	.c2 = {0xDD,0x05,0x02},
	.c3 = {0x37,0xFF,0x08,0xFF,0x08},
	.c4 = {0x39,0x03},
	.c5 = {0x3A,0x00},
	.c6 = {0x6E,0x04},
	.c7 = {0x76,0x01,0x3F},
	.c8 = {0x78,0x03},
	.c9 = {0x7A,0x00,0xD8,0x0C},
	.c10 = {0x7D,0x00,0x04,0x0A,0x0A,0x04},
	.c11 = {0x7F,0x0B,0x01,0x01,0x01,0x01,0x04,0x0F,0x0B,0x0D,0x0C,0x0D,0x06,0x04,0x00},
	.c12 = {0xC2,0x11},
	.c13 = {0xC5,0x0A,0x1C,0x05,0x10,0x18,0x1F,0x0B},
	.c14 = {0xC6,0x12,0x10,0x15},
	.c15 = {0xCB,0x01,0xF5,0xFF,0xFF,0x01,0x00,0x05,0x00,0x05,0x00},
	.c16 = {0xD4,0x01,0x04,0x07},
	.c17 = {0xD5,0xA5},
	.c18 = {0x62,0x12,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x00},
	.c19 = {0x63,0x21,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x21,0x00},
	.c20 = {0x64,0x12,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x00},
	.c21 = {0x65,0x21,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x21,0x00},
	.c22 = {0x66,0x12,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x00},
	.c23 = {0x67,0x21,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x21,0x00},
	.c24 = {0x68,0x12,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x00},
	.c25 = {0x69,0x21,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x21,0x00},
	.c26 = {0x6A,0x10,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00},
	.c27 = {0x6B,0x01,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00},
	.c28 = {0x6C,0x10,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00},
	.c29 = {0x6D,0x01,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00},
	.c30 = {0xC9,0x00,0x00,0x00,0x00,0x30,0x30,0x2F,0x2F,0x2E,0x2E,0x2D,0x2D,0x2C,0x2C,0x2B,0x2B,0x2A,0x2A,0x29,0x29,0x28,0x28,0x27,0x27,0x26,0x26,0x25,0x25},
	.c31 = {0x8A,0x00,0x03,0x20,0x01,0xE0,0x00,0x00,0x00,0x00,0x2A,0x1B,0x00,0x00,0x00,0x00,0x00,0x2B,0xA3,0x00,0x00,0x13,0x00,0xFF,0x14,0x12,0x01,0xFF,0x15,0x11,0x02,0xFF,0x16,0x10,0x03,0xFF,0x17,0x0F,0x04,0xFF,0x18,0x0E,0x05,0xFF,0x19,0x0D,0x06,0xFF,0x1A,0x0C,0x07,0xFF,0x1B,0x0B,0xFF,0xFF,0x1C,0x0A,0xFF,0xFF,0x1D,0x09,0xFF,0xFF,0x1E,0x08,0xFF,0xFF,0x1F},
	.c32 = {0x8C,0x30,0x0B,0x0A,0x0B,0x0A,0x0A,0x0B,0x32,0x24,0x40},
	.c33 = {0xE9,0x00,0x00},
	.c34 = {0xEA,0x14,0x0C,0x00,0x24},
	.c35 = {0xEB,0x32,0x32,0x8A,0x84},
	.c36 = {0xEC,0x02,0x0F,0x0A,0x2D,0x2D,0x00,0x00,0x00,0x00},
	.c37 = {0xEF,0x11,0x00},
	.c38 = {0xF0,0x20},
	.c39 = {0xF1,0x07,0x04,0x06,0x03},
	.c40 = {0xF2,0x0A,0x06,0x14,0x3C},
	.c41 = {0xF3,0x57},
	.c42 = {0xF4,0x7D,0xB2,0x28,0x35},
	.c43 = {0xF6,0x00,0x00,0x1B,0x76,0x0B},
	.c44 = {0xF7,0x50,0x4F,0x4F,0x0C,0x40},
	.c45 = {0xAB,0x01},
	.c46 = {0xAB,0x10},
	.c47 = {0xAC,0xEF,0x47},
	.c48 = {0xAB,0x00},
};

static char FW_VER[LEN_FW_VER];

/**
 * hx_i2c_send - issue a single I2C message in master transmit mode
 * @buf: Data that will be written to the slave
 * @count: How many bytes to write
 *
 * Returns negative errno, or else the number of bytes written.
 */
static int hx_i2c_send(const char *buf, int count)
{
	int ret = -ENODEV;

	BUG_ON(!buf);
	
	ret = i2c_master_send(hx_i2c.client, buf, count);

	return ret;
}

/**
 * hx_i2c_recv - issue a single I2C message in master receive mode
 * @buf: Where to store data read from slave
 * @count: How many bytes to read
 *
 * Returns negative errno, or else the number of bytes read.
 */
static int hx_i2c_recv(char *buf, int count)
{
	int ret = -ENODEV;

	BUG_ON(!buf);

	ret = i2c_master_recv(hx_i2c.client, buf, count);
	
	return ret;
}

static int hx_set_offset_register(char offset)
{
	int ret;

	ret = hx_i2c_send(&offset, 1);

	return (ret == 1) ? 0 : ERROR_I2C_ACCESS;

}

static void hx_print_finger_position_data(char* buffer)
{
	printk(KERN_INFO "Data: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", 
		buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9], 
		buffer[10], buffer[11], buffer[12], buffer[13], buffer[14], buffer[15], buffer[16], buffer[17], buffer[18], buffer[19]);
}

/**
 * hx_get_finger_position - read finger position data
 */
static int hx_get_finger_position(void)
{
	int ret, finger1_valid = 0, finger2_valid = 0, point_id_valid = 0, checksum_len = 0, i;
	int fingers = 0;
	char buff_temp[sizeof(struct hx_i2c_data_received)];
	char* checksum_ptr;
	unsigned short finger1_x = 0, finger1_y = 0, finger2_x = 0, finger2_y = 0, checksum = 0;
	char hx8526_data_header[4] = {0xAA, 0x55, 0xA5, 0x5A};

	while(!pdata.get_intr_line()) {
		hx_set_offset_register(POINT_DATA_REG);
		ret = hx_i2c_recv(buff_temp, len_i2c_frame_finger_position);

		if (ret != len_i2c_frame_finger_position) {
			printk(KERN_ERR HX_DEVNAME ": Error reading finger positions, return value of i2c read: [%d] instead of [%d]\n", ret, len_i2c_frame_finger_position);
			return ERROR_I2C_ACCESS;
		}
		
		finger1_valid = 0;
		finger2_valid = 0;
		point_id_valid = 0;
		fingers = 0;
		checksum = 0;
		
		memcpy((void*) &hx_i2c_packet_received, (void*) buff_temp, len_i2c_frame_finger_position);

		// Check header
		if(memcmp(hx_i2c_packet_received.header, hx8526_data_header, sizeof(hx8526_data_header))) {
			printk(KERN_INFO HX_DEVNAME ": Wrong header\n");
			hx_print_finger_position_data(buff_temp);
			continue;
		}

		checksum_len = sizeof(struct hx_i2c_data_received) - sizeof(hx_i2c_packet_received.header) - sizeof(hx_i2c_packet_received.CS_LSB);
		checksum_ptr = &(hx_i2c_packet_received.x1_msb);
		for(i = 0; i < checksum_len; i++)
		{
			checksum += *checksum_ptr;
			checksum_ptr++;
		}
		
		checksum &= LOW_BYTE_MASK;

		if(checksum != hx_i2c_packet_received.CS_LSB) {
			printk(KERN_ERR HX_DEVNAME ": Checksum error. Calculated result is: 0x%x, CS_LSB is 0x%x\n", checksum, hx_i2c_packet_received.CS_LSB);
			hx_print_finger_position_data(buff_temp);
			continue;
		}

		if(!((hx_i2c_packet_received.x1_lsb == 0xFF) && (hx_i2c_packet_received.x1_msb == 0xFF) && (hx_i2c_packet_received.y1_lsb == 0xFF) && (hx_i2c_packet_received.y1_msb == 0xFF))) {
			finger1_x = (hx_i2c_packet_received.x1_lsb | (hx_i2c_packet_received.x1_msb << 8));
			finger1_y = (hx_i2c_packet_received.y1_lsb | (hx_i2c_packet_received.y1_msb << 8));
			finger1_valid = 1;
		}

		if(!((hx_i2c_packet_received.x2_lsb == 0xFF) && (hx_i2c_packet_received.x2_msb == 0xFF) && (hx_i2c_packet_received.y2_lsb == 0xFF) && (hx_i2c_packet_received.y2_msb == 0xFF))) {
			finger2_x = (hx_i2c_packet_received.x2_lsb | (hx_i2c_packet_received.x2_msb << 8));
			finger2_y = (hx_i2c_packet_received.y2_lsb | (hx_i2c_packet_received.y2_msb << 8));
			finger2_valid = 1;
		}
		
		// Check point count and point id
		switch (hx_i2c_packet_received.point_count & POINT_COUNT_MASK) {
			case 0x1:
				if(finger1_valid && (hx_i2c_packet_received.point_id & POINT1_MASK))
					point_id_valid = 1;
				else if(finger2_valid && (hx_i2c_packet_received.point_id & POINT2_MASK))
					point_id_valid = 1;
				fingers = 1;
				break;
			case 0x2:
				if(hx_i2c_packet_received.point_id == (POINT1_MASK | POINT2_MASK))
					point_id_valid = 1;
				fingers = 2;
				break;
			case 0xF:
				point_id_valid = 1;
				fingers = 0;
				break;
			default:
				break;
		}

		if(!point_id_valid) {
			printk(KERN_INFO HX_DEVNAME ": Invalid point id\n");
			hx_print_finger_position_data(buff_temp);
			continue;
		}
		
		//Report position of fingers to the input layer
		if(fingers > 0) {
			if(hx_i2c_packet_received.point_id & POINT1_MASK)  {
				input_report_abs(hx_cap_ts_input_dev, ABS_X, finger1_x);
				input_report_abs(hx_cap_ts_input_dev, ABS_Y, finger1_y);
			}
			
			if(hx_i2c_packet_received.point_id & POINT2_MASK) {
				if((hx_i2c_packet_received.point_id & POINT1_MASK) == 0) {
					input_report_abs(hx_cap_ts_input_dev, ABS_X, finger2_x);
					input_report_abs(hx_cap_ts_input_dev, ABS_Y, finger2_y);
				}
				else {
					input_report_abs(hx_cap_ts_input_dev, ABS_FINGER2_X, finger2_x);
					input_report_abs(hx_cap_ts_input_dev, ABS_FINGER2_Y, finger2_y);
				}
			}
		}

		input_report_abs(hx_cap_ts_input_dev, ABS_FINGERS, fingers);

		if (fingers > 0)
			input_report_abs(hx_cap_ts_input_dev, ABS_PRESSURE, 1);
		else
			input_report_abs(hx_cap_ts_input_dev, ABS_PRESSURE, 0);

		input_sync(hx_cap_ts_input_dev);
	} 

	return 0;
}

/**
 * hx_read_info_work - read information from ts controller
 */
static void hx_read_info_work(struct work_struct *work)
{
	mutex_lock(&drv_data->lock);
	hx_get_finger_position();
	mutex_unlock(&drv_data->lock);
	enable_irq(hx_i2c.client->irq);
}

static irqreturn_t hx_irq_handler(int irq, void *dev_id)
{
	disable_irq(hx_i2c.client->irq);
	queue_work(readinfo_workqueue, &hx_i2c.read_info_work);
	return IRQ_HANDLED;
};

static int hx_lock_hx8526(void)
{
	int ret, res = 1;
	char i2c_data[6];

	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x06;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(10);
	
	i2c_data[0] = HX_CMD_SETFLASHADDR;
	i2c_data[1] = 0x03;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x00;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(10);

	i2c_data[0] = HX_CMD_SETFLASHDATA;
	i2c_data[1] = 0x00;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x7D;
	i2c_data[4] = 0x03;
	ret = hx_i2c_send(i2c_data, 5);
	if(ret != 5)
		return res;

	udelay(10);

	i2c_data[0] = HX_CMD_FLASHBPWSTART;
	ret = hx_i2c_send(i2c_data, 1);
	if(ret != 1)
		return res;

	msleep(50);

	res = 0;

	return res;
}

static int hx_unlock_hx8526(void)
{
	int ret, res = 1;
	char i2c_data[6];
	
	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x06;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(10);

	i2c_data[0] = HX_CMD_SETFLASHADDR;
	i2c_data[1] = 0x03;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x00;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(10);

	i2c_data[0] = HX_CMD_SETFLASHDATA;
	i2c_data[1] = 0x00;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x3D;
	i2c_data[4] = 0x03;
	ret = hx_i2c_send(i2c_data, 5);
	if(ret != 5)
		return res;

	udelay(10);

	i2c_data[0] = HX_CMD_FLASHBPWSTART;
	ret = hx_i2c_send(i2c_data, 1);
	if(ret != 1)
		return res;

	msleep(50);

	res = 0;

	return res;
}

/**
 * Manual mode 0x47: Latch flash address
 */
static int hx_enter_manual_mode_for_0x47(void)
{
	int ret, res = 1;
	char i2c_data[6];

	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x09;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(5);

	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x0D;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(5);

	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x09;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(5);

	res = 0;

	return res;
}

/**
 * Manual mode 0x48: Let data be written into flash buffer
 */
static int hx_enter_manual_mode_for_0x48(void)
{
	int ret, res = 1;
	char i2c_data[6];

	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x0D;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(5);
	
	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x09;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(5);

	res = 0;

	return res;
}

/**
 * Manual mode 0x49: Write the data into flash when the flash buffer collect one page's data
 */
static int hx_enter_manual_mode_for_0x49(void)
{	
	int ret, res = 1;
	char i2c_data[6];

	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x01;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(5);
	
	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x05;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(5);

	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x01;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(5);

	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	res = 0;

	return res;
}

static int hx_enter_manual_mode(int value)
{
	int res = 1;
	
	switch(value) {
		case 0x47:
			res = hx_enter_manual_mode_for_0x47();
			break;
		case 0x48:
			res = hx_enter_manual_mode_for_0x48();
			break;
		case 0x49:
			res = hx_enter_manual_mode_for_0x49();
			break;
	}

	if(res)
		printk(KERN_ERR HX_DEVNAME ": hx_enter_manual_mode for 0x%x failed.\n", value);

	return res;
}

/**
 * Flash Reading Flow
 *  A. Set command TSSLPOUT (0x81H) to turn on internal power
 *  B. Set FLASH_EN = 1, 0x43H: {0x01H, 0x00H, 0x02H}
 *  C. Set flash address, 0x44H
 *  D. Set command (0x46H) to change to flash read mode
 *  E. Read 4 bytes data from register 0x59H, i=0,1,2...
 *      - 1st PA = data[4*i]
 *      - 2nd PA = data[4*i+1]
 *      - 3rd PA = data[4*i+2]
 *      - 4th PA = data[4*i+3]
 *  F. Finish Reading Flash ?
 *     (1) [No]:
 *          - renew flash memory address
 *     (2) [Yes]:
 *          - Set FLASH_EN = 0, 0x43H: {0x00H, 0x00H, 0x02H}
 */
static int hx_flash_read(int start_addr, int len, char* buffer)
{
	int i, round;
	char i2c_data[6];
	unsigned char index_byte, index_page, index_sector;
	char* tmp;
	int ret, res = 1;

	if(len <= 0)
		return 1;

	tmp = buffer;

	round = (len+3)>>2;
	
	i2c_data[0] = HX_CMD_TSSLPOUT;
	ret = hx_i2c_send(i2c_data, 1);
	if(ret != 1)
		return res;

	msleep(120);

	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(10);
	
	start_addr >>= 2;
	
	for(i = 0; i < round; i++) {
		index_byte = ((i + start_addr)& 0x001F);
		index_page = (((i + start_addr) & 0x03E0) >> 5);
		index_sector = (((i + start_addr) & 0x1C00) >> 10);

		i2c_data[0] = HX_CMD_SETFLASHADDR;
		i2c_data[1] = index_byte;
		i2c_data[2] = index_page;
		i2c_data[3] = index_sector;
		ret = hx_i2c_send(i2c_data, 4);
		if(ret != 4)
			return res;

		udelay(5);

		i2c_data[0] = HX_CMD_FLASHR;
		ret = hx_i2c_send(i2c_data, 1);
		if(ret != 1)
			return res;

		udelay(5);

		hx_set_offset_register(0x59);
		hx_i2c_recv(tmp, 4);
		tmp += 4;

		udelay(2);
	}
	
	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x00;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		return res;

	udelay(10);

	res = 0;

	return res;
}

/**
 * Flash Programming Flow
 *  A. Hardware reset
 *  B. Set command TSSLPOUT (0x81H) to turn on internal power  
 *  C. Unlock Flash Function
 *  D. Erase all flash memory
 *     (1) Set MASS_Erase_EN = 1, 0x43H: {0x07H, 0x00H, 0x02H}
 *     (2) Write MASS_Erase CMD (0x4FH)
 *  E. Enter Manual Mode (0x42H, 0x01H)
 *  F. Set MASS_Erase_EN = 0, 0x43H: {0x01H, 0x00H, 0x02H} 
 *  G. Move to flash address 0x0000, 0x44H: {0x00H, 0x00H, 0x00H}
 *  H. Set commad (0x47H manual mode) to latch flash address
 *  I. Calculate flash address counter
 *     - hx_firmware.size is the total number of burning bytes
 *     - due to there is 4 bytes burning to flash per time, round is the number of burning times
 *  J. Set flash address (0x44H)
 *     - 1st PAs (column)
 *     - 2nd PAs (page)
 *     - 3rd PAs (sector)
 *  K. When page address change, write 4 bytes data to register 0x45H, i=0,1,2...
 *     - 1st PA = data[4*i]
 *     - 2nd PA = data[4*i+1]
 *     - 3rd PA = data[4*i+2]
 *     - 4th PA = data[4*i+3]
 *  L. Set command (0x48H manual mode), data will be written into flash buffer
 *  M. If column address == 0x1FH ?
 *     (1) [No]:
 *          - go back to step J.
 *     (2) [Yes]
 *          - set command (0x49H manual mode) to program data from buffer to this page
 *  M. Finish program ?
 *		 (1) [No]:
 *          - go back to step J.
 *     (2) [Yes]:
 *          - Set FLASH_EN = 0, 0x43H: {0x00H, 0x00H, 0x02H}
 *          - Leave Manual Mode (0x42H, 0x00H)
 *          - Lock Flash Function
 */
static int hx_flash_write(void)
{
	char i2c_data[6];
	int ret, res = 1, i, round, page_change_flag = 0;
	unsigned char index_byte, index_page, index_sector;
	
	pdata.reset_ts();
	msleep(30);

	i2c_data[0] = HX_CMD_TSSLPOUT;
	ret = hx_i2c_send(i2c_data, 1);
	if(ret != 1)
	{
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
		return res;
	}
	
	msleep(120);

	if(hx_unlock_hx8526())
	{
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
		return res;
	}

	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x07;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
	{
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
		return res;
	}

	udelay(10);

	i2c_data[0] = HX_CMD_FLASHME;
	ret = hx_i2c_send(i2c_data, 1);
	if(ret != 1)
	{
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
		return res;
	}

	msleep(50);

	i2c_data[0] = HX_CMD_SETFLASHTEST;
	i2c_data[1] = 0x01;
	ret = hx_i2c_send(i2c_data, 2);
	if(ret != 2)
	{
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
		return res;
	}

	udelay(5);

	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x01;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
	{
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
		return res;
	}

	udelay(5);
	
	i2c_data[0] = HX_CMD_SETFLASHADDR;
	i2c_data[1] = 0x00;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x00;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
	{
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
		return res;
	}

	udelay(5);

	if(hx_enter_manual_mode(0x47))
	{
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
		return res;
	}

	round = (hx_firmware.size+3)>>2;

	for(i = 0; i < round; i++) {
		index_byte = (i & 0x001F);
		index_page = ((i & 0x03E0) >> 5);
		index_sector = ((i & 0x1C00) >> 10);

		if(page_change_flag == 1) {
			i2c_data[0] = HX_CMD_SETFLASHADDR;
			i2c_data[1] = index_byte;
			i2c_data[2] = index_page;
			i2c_data[3] = index_sector;
			ret = hx_i2c_send(i2c_data, 4);
			if(ret != 4)
			{
				printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
				return res;
			}

			udelay(5);
			
			if(hx_enter_manual_mode(0x47))
			{
				printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
				return res;
			}

			page_change_flag = 0;
		}

		i2c_data[0] = HX_CMD_SETFLASHADDR;
		i2c_data[1] = index_byte;
		i2c_data[2] = index_page;
		i2c_data[3] = index_sector;
		ret = hx_i2c_send(i2c_data, 4);
		if(ret != 4)
		{
			printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
			return res;
		}

		udelay(5);

		i2c_data[0] = HX_CMD_SETFLASHDATA;
		i2c_data[1] = hx_firmware.data[4*i+0];
		i2c_data[2] = hx_firmware.data[4*i+1];
		i2c_data[3] = hx_firmware.data[4*i+2];
		i2c_data[4] = hx_firmware.data[4*i+3];
		ret = hx_i2c_send(i2c_data, 5);
		if(ret != 5)
		{
			printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
			return res;
		}

		udelay(5);
		
		if(hx_enter_manual_mode(0x48))
		{
			printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
			return res;
		}

		if(index_byte == 0x1F) {
			if(hx_enter_manual_mode(0x49))
			{
				printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
				return res;
			}

			msleep(5);
			page_change_flag = 1;
		}
	}

	if(hx_enter_manual_mode(0x49))
	{
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
		return res;
	}

	msleep(10);

	i2c_data[0] = HX_CMD_SETFLASHEN;
	i2c_data[1] = 0x00;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x02;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
	{
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
		return res;
	}

	udelay(5);

	i2c_data[0] = HX_CMD_SETFLASHTEST;
	i2c_data[1] = 0x00;
	ret = hx_i2c_send(i2c_data, 2);
	if(ret != 2)
	{
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
		return res;
	}
		

	udelay(5);

	if(hx_lock_hx8526())
	{
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
		return res;
	}

	res = 0;

	if(res)
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
	else
		printk(KERN_INFO HX_DEVNAME " %s: %d bytes written!\n", __FUNCTION__, hx_firmware.size);

	return res;
}

/**
 * FW information @ Flash Address 0x2000 ~ 0x203A (60 bytes)
 *  1. 0x2000 is Company[12]
 *  2. 0x200C is Project Name[12]
 *  3. 0x2018 is End User[12]
 *  4. 0x2024 is FW Version[12]
 *  5. 0x2030 is Dated[12]
 *  firmware version format: HX8526-Vxxxx, Ex: HX8526-V0001, HX8526-V0002...
 */
static void hx_i2c_read_fw_version(void)
{
	// Read Current Firmware Version(12 Bytes) From Flash Address 0x2024
	memset(FW_VER,0,sizeof(FW_VER));
	
	hx_flash_read(OFFSET_FW_VERSION, LEN_FW_VER, FW_VER);
}

static int hx_i2c_check_fw_version(void)
{
	int current_fw_ver, new_fw_ver;
	
	// Compare firmware special pattern("HX8526-V") first, if not equal, force to do 1st firmware update 	
	if(memcmp(&hx_firmware.data[OFFSET_FW_VERSION],&FW_VER[0],(LEN_FW_VER-4)))
	{
		return 1;
	}
	
	//get current firmware version number
	current_fw_ver = (((FW_VER[8]-0x30)<<24) + ((FW_VER[9]-0x30)<<16) + ((FW_VER[10]-0x30)<<8) + (FW_VER[11]-0x30));
	
	//get new firmware version number
	new_fw_ver = (((hx_firmware.data[OFFSET_FW_VERSION+8]-0x30)<<24) + ((hx_firmware.data[OFFSET_FW_VERSION+9]-0x30)<<16) + 
		            ((hx_firmware.data[OFFSET_FW_VERSION+10]-0x30)<<8) + (hx_firmware.data[OFFSET_FW_VERSION+11]-0x30));
	
	// if new_fw_ver > current_fw_ver, do firmware update
	if(new_fw_ver > current_fw_ver)
	{
		return 1;
	}
	else
	{
		return 0;
	}	
}

/**
 * Initial CMD set flow provided by chip vendor
 */
static void hx_i2c_write_initialsettings(void)
{
	hx_i2c_send(hx_init_settings.c48, sizeof(hx_init_settings.c48));
	hx_i2c_send(hx_init_settings.c45, sizeof(hx_init_settings.c45));
	hx_i2c_send(hx_init_settings.c1, sizeof(hx_init_settings.c1));
	hx_i2c_send(hx_init_settings.c2, sizeof(hx_init_settings.c2));
	hx_i2c_send(hx_init_settings.c3, sizeof(hx_init_settings.c3));
	hx_i2c_send(hx_init_settings.c4, sizeof(hx_init_settings.c4));
	hx_i2c_send(hx_init_settings.c5, sizeof(hx_init_settings.c5));
	hx_i2c_send(hx_init_settings.c6, sizeof(hx_init_settings.c6));
	hx_i2c_send(hx_init_settings.c7, sizeof(hx_init_settings.c7));
	hx_i2c_send(hx_init_settings.c8, sizeof(hx_init_settings.c8));
	hx_i2c_send(hx_init_settings.c9, sizeof(hx_init_settings.c9));
	hx_i2c_send(hx_init_settings.c10, sizeof(hx_init_settings.c10));
	hx_i2c_send(hx_init_settings.c11, sizeof(hx_init_settings.c11));
	hx_i2c_send(hx_init_settings.c12, sizeof(hx_init_settings.c12));
	hx_i2c_send(hx_init_settings.c13, sizeof(hx_init_settings.c13));
	hx_i2c_send(hx_init_settings.c14, sizeof(hx_init_settings.c14));
	hx_i2c_send(hx_init_settings.c15, sizeof(hx_init_settings.c15));
	hx_i2c_send(hx_init_settings.c16, sizeof(hx_init_settings.c16));
	hx_i2c_send(hx_init_settings.c17, sizeof(hx_init_settings.c17));
	hx_i2c_send(hx_init_settings.c18, sizeof(hx_init_settings.c18));
	hx_i2c_send(hx_init_settings.c19, sizeof(hx_init_settings.c19));
	hx_i2c_send(hx_init_settings.c20, sizeof(hx_init_settings.c20));
	hx_i2c_send(hx_init_settings.c21, sizeof(hx_init_settings.c21));
	hx_i2c_send(hx_init_settings.c22, sizeof(hx_init_settings.c22));
	hx_i2c_send(hx_init_settings.c23, sizeof(hx_init_settings.c23));
	hx_i2c_send(hx_init_settings.c24, sizeof(hx_init_settings.c24));
	hx_i2c_send(hx_init_settings.c25, sizeof(hx_init_settings.c25));
	hx_i2c_send(hx_init_settings.c26, sizeof(hx_init_settings.c26));
	hx_i2c_send(hx_init_settings.c27, sizeof(hx_init_settings.c27));
	hx_i2c_send(hx_init_settings.c28, sizeof(hx_init_settings.c28));
	hx_i2c_send(hx_init_settings.c29, sizeof(hx_init_settings.c29));
	hx_i2c_send(hx_init_settings.c30, sizeof(hx_init_settings.c30));
	hx_i2c_send(hx_init_settings.c31, sizeof(hx_init_settings.c31));
	hx_i2c_send(hx_init_settings.c32, sizeof(hx_init_settings.c32));
	hx_i2c_send(hx_init_settings.c33, sizeof(hx_init_settings.c33));
	hx_i2c_send(hx_init_settings.c34, sizeof(hx_init_settings.c34));
	hx_i2c_send(hx_init_settings.c35, sizeof(hx_init_settings.c35));
	hx_i2c_send(hx_init_settings.c36, sizeof(hx_init_settings.c36));
	hx_i2c_send(hx_init_settings.c37, sizeof(hx_init_settings.c37));
	hx_i2c_send(hx_init_settings.c38, sizeof(hx_init_settings.c38));
	hx_i2c_send(hx_init_settings.c39, sizeof(hx_init_settings.c39));
	hx_i2c_send(hx_init_settings.c40, sizeof(hx_init_settings.c40));
	hx_i2c_send(hx_init_settings.c41, sizeof(hx_init_settings.c41));
	hx_i2c_send(hx_init_settings.c42, sizeof(hx_init_settings.c42));
	hx_i2c_send(hx_init_settings.c43, sizeof(hx_init_settings.c43));
	hx_i2c_send(hx_init_settings.c44, sizeof(hx_init_settings.c44));
	hx_i2c_send(hx_init_settings.c46, sizeof(hx_init_settings.c46));
	hx_i2c_send(hx_init_settings.c47, sizeof(hx_init_settings.c47));
}

/**
 * Software Power On Sequence
 *  A. Set command TSSLPOUT (0x81H) to turn on internal power
 *  B. Write initial settings
 *  C. Turn on reload disable (0x42H, 0x02H)
 *  D. Set command MICROOFF (0x35H), PA1 = 0x02H to turn on internal MCU
 *  E. Set command ROMRDY (0x36H), PA1 = 0x0FH, PA2 = 0x53H to turn on internal flash.
 *  F. Set command TSSON (0x83H) to starting sensing.                
 */
static void hx_i2c_send_init_cmds(void)
{
	char i2c_data[3];

	// Himax HX8526 initial commands
	i2c_data[0] = HX_CMD_TSSLPOUT;
	hx_i2c_send(i2c_data, 1);
	msleep(150);

	hx_i2c_read_fw_version();

	// Write initial settings
	hx_i2c_write_initialsettings();

	// Turn on reload disable
	i2c_data[0] = HX_CMD_SETFLASHTEST;
	i2c_data[1] = 0x02;
	hx_i2c_send(i2c_data, 2);

	// Set MICROOFF
	i2c_data[0] = HX_CMD_MICROOFF;
	i2c_data[1] = 0x02;
	hx_i2c_send(i2c_data, 2);
	msleep(1);

	i2c_data[0] = HX_CMD_ROMRDY;
	i2c_data[1] = 0x0F;
	i2c_data[2] = 0x53;
	hx_i2c_send(i2c_data, 3);
	msleep(1);

	i2c_data[0] = HX_CMD_TSSON;
	hx_i2c_send(i2c_data, 1);
}

/**
 * hx_resume_work - perform initialization works after resume
 */
static void hx_resume_work(struct work_struct *work)
{
	pdata.reset_ts();
	hx_i2c_send_init_cmds();
}

/**
 * hx_loading_finished - read back the data which has burned into flash to make sure 
 *                       that there is no any data missed during programming flow 
 */
static void hx_loading_finished(const struct firmware *fw, void* context)
{
	char flash_buff[2048];
	int remain, size, idx, write_ok = 1;
	
	printk(KERN_INFO HX_DEVNAME ": request firmware finished\n");
	
	if (fw != NULL)
		memcpy((void*)&hx_firmware, (void*)fw, sizeof(struct firmware));
	else
		return;
	
	printk(KERN_INFO HX_DEVNAME " %s: %d bytes firmware received!\n", __FUNCTION__, hx_firmware.size);

	if(hx_i2c_check_fw_version() == 0)
	{
		printk(KERN_INFO HX_DEVNAME "	No need to do firmware update!\n");
		return;
	}
	
	disable_irq(hx_i2c.client->irq);

	mutex_lock(&drv_data->lock);

	if(hx_flash_write()) {
		printk(KERN_ERR HX_DEVNAME " %s: Write to flash failed!\n", __FUNCTION__);
	}
	else {
		remain = hx_firmware.size;
		idx = 0;
		while(remain > 0)
		{
			memset(flash_buff,0,sizeof(flash_buff));
			if(sizeof(flash_buff) > remain)
			{	
				size = remain;
				remain = 0;
			}
			else
			{	
				size = sizeof(flash_buff);
				remain -= sizeof(flash_buff);
			}
			
			if(hx_flash_read(idx, size, flash_buff)) {
				printk(KERN_ERR HX_DEVNAME " %s: Read flash error!\n", __FUNCTION__);
				write_ok = 0;
				break;
			}
			else {
				if(memcmp(&hx_firmware.data[idx], &flash_buff[0], size)) {
					printk(KERN_ERR HX_DEVNAME " %s: Check flash error!\n", __FUNCTION__);
					write_ok = 0;
					break;
				}
				if(remain > 0)
					idx += sizeof(flash_buff);
			}
		}
		if(write_ok)
		{	
			msleep(200);
			pdata.reset_ts();
			hx_i2c_send_init_cmds();
			printk(KERN_INFO HX_DEVNAME " %s: Check flash ok. Himax HX8526 firmware updated successfully.\n", __FUNCTION__);
			enable_irq(hx_i2c.client->irq);
		}
		else
			printk(KERN_INFO HX_DEVNAME " %s: Check flash fail. Himax HX8526 firmware updated fail.\n", __FUNCTION__);	
	}

	mutex_unlock(&drv_data->lock);
}

static int hx_i2c_init(struct i2c_client *client)
{
	BUG_ON(!client);

	//Init hx_i2c struct
	memset(&hx_i2c, 0x00, sizeof(hx_i2c));
	hx_i2c.client = client;
	memcpy((void*) &pdata, (void*) client->dev.platform_data, sizeof(struct hx_pdata_t));
	INIT_WORK(&hx_i2c.read_info_work, hx_read_info_work);
	drv_data = kmalloc(sizeof(struct hx_drv_data_t), GFP_KERNEL);

	if(NULL == drv_data) {
		printk(KERN_ERR HX_DEVNAME ": Can allocate driver data!\n");
		return 1;
	}

	dev_set_drvdata(&client->dev, drv_data);
	mutex_init(&drv_data->lock);

	return 0;
}


static int hx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret  = 0;

	len_i2c_frame_finger_position = sizeof(struct hx_i2c_data_received);

	printk(KERN_INFO HX_DEVNAME " :probing driver\n");
	drv_data = NULL;

	printk(KERN_INFO HX_DEVNAME " :Himax HX8526 Capacitive Touch Screen Driver, (C) 2008 TomTom BV\n" );

	BUG_ON(!client);
	BUG_ON(!client->dev.platform_data);

	if(hx_i2c_init(client)) {
		ret = -ENOMEM;
		return ret;
	}

	readinfo_workqueue = create_singlethread_workqueue("himax_read_info");
	if(!readinfo_workqueue) {
		printk(KERN_ERR HX_DEVNAME ": Cannot allocate workqueue!\n");
		ret = -ENOMEM;
		mutex_destroy(&drv_data->lock);
		kfree(drv_data);
		return ret;
	}

	hx_cap_ts_input_dev = input_allocate_device();

	if (!hx_cap_ts_input_dev) {
		printk(KERN_ERR HX_DEVNAME ": Badf init_alloc_device()\n");
		ret = -ENOMEM;
		destroy_workqueue(readinfo_workqueue);
		return ret;
	}

	printk(KERN_INFO HX_DEVNAME ": Input device allocated\n");

	/* Announce that the ts will generate absolute coordinates */
	set_bit(EV_ABS, hx_cap_ts_input_dev->evbit);
	printk(KERN_INFO HX_DEVNAME ": Event Abs positions declared\n");

	// Declare max and min of X, Y and pressure
	input_set_abs_params(hx_cap_ts_input_dev, ABS_X, XMIN, XMAX, 0, 0);
	input_set_abs_params(hx_cap_ts_input_dev, ABS_Y, YMIN, YMAX, 0, 0);
	input_set_abs_params(hx_cap_ts_input_dev, ABS_PRESSURE, PRESSUREMIN, PRESSUREMAX, 0, 0);

	input_set_abs_params(hx_cap_ts_input_dev, ABS_FINGERS, FINGERSMIN, FINGERSMAX, 0, 0);
	input_set_abs_params(hx_cap_ts_input_dev, ABS_FINGER2_X, XMIN, XMAX, 0, 0);
	input_set_abs_params(hx_cap_ts_input_dev, ABS_FINGER2_Y, YMIN, YMAX, 0, 0);

	hx_cap_ts_input_dev->name = "Himax HX8526 Capacitive TouchScreen";
	hx_cap_ts_input_dev->phys = "input(ts)";
	hx_cap_ts_input_dev->id.vendor = 0xDEAD;
	hx_cap_ts_input_dev->id.product = 0xBEEF;
	hx_cap_ts_input_dev->id.version = 0x0101;

	/* Register with the input subsystem */
	ret = input_register_device(hx_cap_ts_input_dev);
	if (ret) {
		printk(KERN_ERR HX_DEVNAME ": Could not register input device(capacitive touchscreen)!\n");
		ret = -EIO;
		input_free_device(hx_cap_ts_input_dev);
		return ret;
	}

	printk(KERN_INFO HX_DEVNAME ": Input device registered\n");

	printk(KERN_INFO HX_DEVNAME ": irq number [%d]\n", client->irq);
	if (request_irq(client->irq, &hx_irq_handler, IRQ_TYPE_EDGE_FALLING, "hx_1", NULL)) {
		printk(KERN_ERR HX_DEVNAME " Could not allocate IRQ (hx_1)!\n");
		ret = -EIO;
		input_unregister_device(hx_cap_ts_input_dev);
		return ret;
	}

	hx_i2c_send_init_cmds();

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "fw_hx_cap_ts.bin", &client->dev, NULL, hx_loading_finished);
	if (ret) {
		printk(KERN_ERR HX_DEVNAME ": request_firmware_nowait returned error!\n");
		ret = -EIO;
		disable_irq(client->irq);
		free_irq(client->irq, NULL);
		return ret;
	}

	return ret;
}

static int hx_i2c_remove(struct i2c_client *client)
{

	printk(KERN_INFO HX_DEVNAME ": start remove\n");
	
	disable_irq(client->irq);
	free_irq(client->irq, NULL);
	
	input_unregister_device(hx_cap_ts_input_dev);
	input_free_device(hx_cap_ts_input_dev);

	destroy_workqueue(readinfo_workqueue);

	mutex_destroy(&drv_data->lock);

	kfree(drv_data);

	printk(KERN_INFO HX_DEVNAME ": exit remove\n");

	return 0;
}

static int hx_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk(KERN_INFO HX_DEVNAME ": start suspending\n");
	return 0;
}

static int hx_i2c_resume(struct i2c_client *client)
{
	printk(KERN_INFO HX_DEVNAME ": start resuming\n");
	hx_resume_work(0);
	return 0;
}

static const struct i2c_device_id hx_id[] = {
	{ HX_DEVNAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hx_id);

static struct i2c_driver hx_i2c_driver=
{
	.probe	= hx_i2c_probe,
	.remove	= hx_i2c_remove,
	.suspend= hx_i2c_suspend,
	.resume	= hx_i2c_resume,
	.id_table = hx_id,
	.driver = {
		.name	= HX_DEVNAME,
		.owner	= THIS_MODULE,
	},
};

static int __init hx_init(void)
{
	int err;

	printk(KERN_INFO HX_DEVNAME " :start init\n");

	if ((err = i2c_add_driver(&hx_i2c_driver))) {
		printk(KERN_ERR HX_DEVNAME ": Could Not Be Added. Err Code: [%i]\n", err);
	}

	return err;
}

static void __exit hx_exit(void)
{
	printk(KERN_INFO HX_DEVNAME " :start exit\n");
	i2c_del_driver(&hx_i2c_driver);
}

MODULE_AUTHOR("Chris Liu <chris.liu@tomtom.com>");
MODULE_DESCRIPTION("Driver for I2C connected Himax Capacitive Touchscreen IC.");
MODULE_LICENSE("GPL");

module_init(hx_init);
module_exit(hx_exit);
