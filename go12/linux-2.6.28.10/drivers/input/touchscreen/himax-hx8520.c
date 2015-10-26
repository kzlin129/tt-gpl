/* himax-hx8520.c
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
#include <linux/himax_hx8520.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/firmware.h>

static struct _hx_i2c {
        struct i2c_client       *client;
        struct work_struct      read_info_work;
        struct work_struct      resume_work;
} hx_i2c;

static struct workqueue_struct *readinfo_workqueue;
static struct workqueue_struct *resume_workqueue;
static struct input_dev *hx_cap_ts_input_dev;     /* Representation of an input device */
static struct hx_i2c_data_received hx_i2c_packet_received;
static struct hx_pdata_t pdata;
static struct hx_drv_data_t *drv_data;
static struct firmware hx_firmware;

static int len_i2c_frame_finger_position;
static char date_code[4];
static char fw_ver[10] = "unknown";


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

/**
 * hx_get_finger_position - read finger position data
 */
static int hx_get_finger_position()
{
	int ret, finger1_valid = 0, finger2_valid = 0, point_id_valid = 0;
	char touches;
	char* buff_temp;
	unsigned short finger1_x, finger1_y, finger2_x, finger2_y, cs_count = 0, cs_paylod = 0;

	buff_temp = (char*) kmalloc(len_i2c_frame_finger_position, GFP_KERNEL);
	if (buff_temp == NULL) {
		printk(KERN_ERR HX_DEVNAME ": Error reading finger positions, can't allocate a buffer of [%d] bytes\n", len_i2c_frame_finger_position);
		return 1;
	}

	while(!pdata.get_intr_line()) {
		hx_set_offset_register(POINT_DATA_REG);
		ret = hx_i2c_recv(buff_temp, len_i2c_frame_finger_position);

		if (ret != len_i2c_frame_finger_position) {
			printk(KERN_ERR HX_DEVNAME ": Error reading finger positions, return value of i2c read: [%d] instead of [%d]\n", ret, len_i2c_frame_finger_position);
			kfree(buff_temp);
			return ERROR_I2C_ACCESS;
		}

		finger1_valid = 0;
		finger2_valid = 0;
		point_id_valid = 0;
		
		memcpy((void*) &hx_i2c_packet_received, (void*) buff_temp, len_i2c_frame_finger_position);

		// Check if header is valid - should be 0xAA, 0x55, 0xAA, 0x55.
		if((hx_i2c_packet_received.header[0] != 0xAA) || (hx_i2c_packet_received.header[1] != 0x55) || (hx_i2c_packet_received.header[2] != 0xAA) || (hx_i2c_packet_received.header[3] != 0x55)) {
			printk(KERN_ERR HX_DEVNAME ": Wrong header!\n");
			continue;
		}

		// Check if checksum is valid
		if(hx_i2c_packet_received.point_count == 0)	{
			// All points left, checksum should be 0xFF for both high and low bytes.
			if((hx_i2c_packet_received.CS_High != 0xFF) || (hx_i2c_packet_received.CS_Low != 0xFF)) {
				printk(KERN_ERR HX_DEVNAME ": point_count is 0 but checksum bytes are not all 0xFF.\n");
				continue;
			}
		}
		else {
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

			if(hx_i2c_packet_received.point_count == 1) {
				if(finger1_valid) {
					cs_count = finger1_x + finger1_y + hx_i2c_packet_received.point_count + hx_i2c_packet_received.point_id;
				}
				else if(finger2_valid) {
					cs_count = finger2_x + finger2_y + hx_i2c_packet_received.point_count + hx_i2c_packet_received.point_id;
				}
				else {
					printk(KERN_ERR HX_DEVNAME ": Point count is 1 but no valid fingers!\n");
					continue;
				}
			}
			else if(hx_i2c_packet_received.point_count == 2) {
				if((finger1_valid == 1) && (finger2_valid == 1)) {
					cs_count = finger1_x+ finger1_y + finger2_x + finger2_y + hx_i2c_packet_received.point_count + hx_i2c_packet_received.point_id;
				}
				else {
					printk(KERN_ERR HX_DEVNAME ": Point count is 2 but not all fingers are valid!\n");
					continue;
				}
			}
			else {
				printk(KERN_ERR HX_DEVNAME ": Point count not supported!\n");
				continue;
			}

			cs_paylod = (hx_i2c_packet_received.CS_High << 8) | hx_i2c_packet_received.CS_Low;

			if(cs_paylod != cs_count) {
				printk(KERN_ERR HX_DEVNAME ": Invalid checksum! cs_count = 0x%x, cs_paylod = 0x%x\n", cs_count, cs_paylod);
				continue;
			}
		}		

		// Check point count and point id
		switch (hx_i2c_packet_received.point_count) {
			case 1:
				if(finger1_valid && (hx_i2c_packet_received.point_id & POINT1_MASK))
					point_id_valid = 1;
				if(finger2_valid && (hx_i2c_packet_received.point_id & POINT2_MASK))
					point_id_valid = 1;
				break;
			case 2:
				if(hx_i2c_packet_received.point_id == (POINT1_MASK | POINT2_MASK))
					point_id_valid = 1;
				break;
			case 0:
				if(hx_i2c_packet_received.point_id == 0x0)
					point_id_valid = 1;
				break;
			default:
				break;
		}

		// HX8520 only supports 2 fingers, so only the first 2 bits should be set.
		if(hx_i2c_packet_received.point_id >> MAXNUMTOUCHES) {
			point_id_valid = 0;
		}

		if(!point_id_valid) {
			printk(KERN_ERR HX_DEVNAME ": Invalid point id!\n");
			continue;
		}
		
		touches = hx_i2c_packet_received.point_count;

		if (touches <= MAXNUMTOUCHES) {
			//Report position of fingers to the input layer
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

			input_report_abs(hx_cap_ts_input_dev, ABS_FINGERS, touches);

			if (touches)
				input_report_abs(hx_cap_ts_input_dev, ABS_PRESSURE, 1);
			else
				input_report_abs(hx_cap_ts_input_dev, ABS_PRESSURE, 0);

			input_sync(hx_cap_ts_input_dev);
		}
	} 

	kfree(buff_temp);

	return 0;
}

/**
 * hx_write_register - write value to register
 * @reg: register address that will be written
 * @value: value to write
 */
static int hx_write_register(char reg, char value)
{
	int ret;
	char buff[2];

	buff[0] = reg;
	buff[1] = value;

	ret = hx_i2c_send(buff, 2);
	
	if (ret != 2)
		printk(KERN_ERR HX_DEVNAME ": Error setting register value, return value of i2c send: [%d] instead of 2\n", ret);

	return (ret == 2) ? 0 : ERROR_I2C_ACCESS;
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

static void hx_i2c_read_fw_version()
{
	char i2c_data[6];
	char read_buf[5];
	int i;

	i2c_data[0] = 0x43;
	i2c_data[1] = 0x1;
	i2c_data[2] = 0x0;
	i2c_data[3] = 0x2;
	hx_i2c_send(i2c_data, 4);
	udelay(1000);
	
	for(i = 0; i < 5; i++) {
		i2c_data[0] = 0x44;
		i2c_data[1] = 0x48 + i;
		i2c_data[2] = 0x1E;
		i2c_data[3] = 0x3;
		hx_i2c_send(i2c_data, 4);
		udelay(1000);
		
		i2c_data[0] = 0x46;
		hx_i2c_send(i2c_data, 1);
		udelay(1000);
		
		hx_set_offset_register(0x59);
		hx_i2c_recv(&read_buf[i], 1);
		udelay(1000);
	}
	
	i2c_data[0] = 0x43;
	i2c_data[1] = 0x0;
	i2c_data[2] = 0x0;
	i2c_data[3] = 0x2;
	hx_i2c_send(i2c_data, 4);
	udelay(100);
	
	memcpy(date_code, read_buf, 4);
	sprintf(fw_ver, "%x", read_buf[4]);
	printk(KERN_INFO HX_DEVNAME ": Date code: %02x%02x%02x%02x, firmware version: %s\n", date_code[0], date_code[1], date_code[2], date_code[3], fw_ver);
}

static void hx_i2c_send_init_cmds()
{
	char i2c_data[2];

	// Himax HX8520 initial commands
	i2c_data[0] = 0x81;
	hx_i2c_send(i2c_data, 1);
	msleep(200);

	hx_i2c_read_fw_version();
	
	i2c_data[0] = 0x35;
	i2c_data[1] = 0x2;
	hx_i2c_send(i2c_data, 2);
	udelay(200);

	i2c_data[0] = 0x36;
	i2c_data[1] = 0x1;
	hx_i2c_send(i2c_data, 2);
	udelay(200);

	i2c_data[0] = 0x83;
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

static int hx_flash_read()
{
	int i;
	char i2c_data[6];
	char read_buf;
	unsigned char index_byte, index_page, index_sector;
	char* flash_buff;
	char* tmp;
	int ret = 0;

	if(hx_firmware.size <= 0)
		return 1;

	flash_buff = (char*) kmalloc(hx_firmware.size, GFP_KERNEL);
	if (flash_buff == NULL) {
		printk(KERN_ERR HX_DEVNAME ": Can't allocate a buffer of [%d] bytes\n", hx_firmware.size);
		return 1;
	}

	tmp = flash_buff;
	
	i2c_data[0] = 0x81;
	hx_i2c_send(i2c_data, 1);

	msleep(200);

	i2c_data[0] = 0x42;
	i2c_data[1] = 0x1;
	hx_i2c_send(i2c_data, 2);

	msleep(50);

	i2c_data[0] = 0x43;
	i2c_data[1] = 0x1;
	i2c_data[2] = 0x0;
	i2c_data[3] = 0x2;
	hx_i2c_send(i2c_data, 4);

	msleep(50);

	for(i = 0; i < hx_firmware.size; i++) {
		index_byte = (i & 0x007F);
		index_page = ((i & 0x0F80) >> 7);
		index_sector = (( i & 0x3000) >> 12);
		i2c_data[0] = 0x44;
		i2c_data[1] = index_byte;
		i2c_data[2] = index_page;
		i2c_data[3] = index_sector;
		hx_i2c_send(i2c_data, 4);

		i2c_data[0] = 0x43;
		i2c_data[1] = 0x1;
		i2c_data[2] = 0x1;
		i2c_data[3] = 0x2;
		hx_i2c_send(i2c_data, 4);

		i2c_data[0] = 0x43;
		i2c_data[1] = 0x1;
		i2c_data[2] = 0x5;
		i2c_data[3] = 0x2;
		hx_i2c_send(i2c_data, 4);

		i2c_data[0] = 0x43;
		i2c_data[1] = 0x1;
		i2c_data[2] = 0x1;
		i2c_data[3] = 0x2;
		hx_i2c_send(i2c_data, 4);

		i2c_data[0] = 0x43;
		i2c_data[1] = 0x1;
		i2c_data[2] = 0x0;
		i2c_data[3] = 0x2;
		hx_i2c_send(i2c_data, 4);

		hx_set_offset_register(0x59);
		hx_i2c_recv(&read_buf, 1);

		*tmp = read_buf;
		tmp++;
	}

	i2c_data[0] = 0x49;
	hx_i2c_send(i2c_data, 1);

	i2c_data[0] = 0x43;
	i2c_data[1] = 0x0;
	i2c_data[2] = 0x0;
	i2c_data[3] = 0x2;
	hx_i2c_send(i2c_data, 4);

	i2c_data[0] = 0x42;
	i2c_data[1] = 0x0;
	hx_i2c_send(i2c_data, 2);

	ret = memcmp(hx_firmware.data, flash_buff, hx_firmware.size);

	kfree(flash_buff);

	return ret;
}

static int hx_flash_write()
{
	int i;
	char i2c_data[6];
	int ret, res = 1;
	unsigned char index_byte, index_page, index_sector;
	u8 *tmp;
	int page_change_flag = 0;
	char* flash_buff;

	// Allocate buffer and prepare data
	flash_buff = (char*) kmalloc(hx_firmware.size, GFP_KERNEL);
	if (flash_buff == NULL) {
		printk(KERN_ERR HX_DEVNAME ": Can't allocate a buffer of [%d] bytes\n", hx_firmware.size);
		return res;
	}
	
	memcpy(flash_buff, hx_firmware.data, hx_firmware.size);

	pdata.reset_ts();

	// Erase flash memory first
	i2c_data[0] = 0x81;
	ret = hx_i2c_send(i2c_data, 1);
	if(ret != 1)
		goto Exit;
	
	msleep(200);
	
	i2c_data[0] = 0x43;
	i2c_data[1] = 0x1;
	i2c_data[2] = 0x0;
	i2c_data[3] = 0x2;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		goto Exit;

	// Erase sector 0~2
	for(i = 0; i < 3; i++) {
		i2c_data[0] = 0x44;
		i2c_data[1] = 0x0;
		i2c_data[2] = 0x0;
		i2c_data[3] = i;
		ret = hx_i2c_send(i2c_data, 4);
		if(ret != 4)
			goto Exit;

		i2c_data[0] = 0x4E;
		ret = hx_i2c_send(i2c_data, 1);
		if(ret != 1)
			goto Exit;
		
		msleep(25);
	}

	// Erase sector 3
	for(i = 0; i < 31; i++) {
		i2c_data[0] = 0x44;
		i2c_data[1] = 0x0;
		i2c_data[2] = i;
		i2c_data[3] = 0x3;
		ret = hx_i2c_send(i2c_data, 4);
		if(ret != 4)
			goto Exit;

		i2c_data[0] = 0x4D;
		ret = hx_i2c_send(i2c_data, 1);
		if(ret != 1)
			goto Exit;

		msleep(25);
	}

	printk(KERN_INFO HX_DEVNAME " %s: Erase done!\n", __FUNCTION__);

	// Start flashing
	i2c_data[0] = 0x44;
	i2c_data[1] = 0;
	i2c_data[2] = 0;
	i2c_data[3] = 0;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		goto Exit;

	i2c_data[0] = 0x47;
	ret = hx_i2c_send(i2c_data, 1);
	if(ret != 1)
		goto Exit;
	
	tmp = flash_buff;
	for(i = 0; i < hx_firmware.size; i++) {
		index_byte = (i & 0x007F);
		index_page = ((i & 0x0F80) >> 7);
		index_sector = (( i & 0x3000) >> 12);

		if(page_change_flag == 1)
		{
			i2c_data[0] = 0x44;
			i2c_data[1] = index_byte;
			i2c_data[2] = index_page;
			i2c_data[3] = index_sector;
			ret = hx_i2c_send(i2c_data, 4);
			if(ret != 4)
				goto Exit;
			
			i2c_data[0] = 0x47;
			ret = hx_i2c_send(i2c_data, 1);
			if(ret != 1)
				goto Exit;
			
			page_change_flag = 0;
		}
		
		i2c_data[0] = 0x44;
		i2c_data[1] = index_byte;
		i2c_data[2] = index_page;
		i2c_data[3] = index_sector;
		ret = hx_i2c_send(i2c_data, 4);
		if(ret != 4)
			goto Exit;
		
		i2c_data[0] = 0x45;
		i2c_data[1] = *tmp;
		ret = hx_i2c_send(i2c_data, 2);
		if(ret != 2)
			goto Exit;
		tmp++;

		i2c_data[0] = 0x48;
		ret = hx_i2c_send(i2c_data, 1);
		if(ret != 1)
			goto Exit;

		if(index_byte == 0x7F) {
			i2c_data[0] = 0x49;
			ret = hx_i2c_send(i2c_data, 1);
			if(ret != 1)
				goto Exit;
			msleep(5);
			page_change_flag = 1;
		}
		
	}

	i2c_data[0] = 0x49;
	ret = hx_i2c_send(i2c_data, 1);
	if(ret != 1)
		goto Exit;

	msleep(200);

	i2c_data[0] = 0x43;
	i2c_data[1] = 0x0;
	i2c_data[2] = 0x0;
	i2c_data[3] = 0x2;
	ret = hx_i2c_send(i2c_data, 4);
	if(ret != 4)
		goto Exit;

	res = 0;
Exit:	
	kfree(flash_buff);
	if(res)
		printk(KERN_ERR HX_DEVNAME " %s: hx_i2c_send failed!\n", __FUNCTION__);
	else
		printk(KERN_INFO HX_DEVNAME " %s: %d bytes written!\n", __FUNCTION__, hx_firmware.size);

	return res;
}

static void hx_loading_finished(const struct firmware *fw, void* context)
{
	printk(KERN_INFO HX_DEVNAME ": request firmware finished\n");
	
	if (fw != NULL)
		memcpy((void*)&hx_firmware, (void*)fw, sizeof(struct firmware));
	else
		return;
	
	printk(KERN_INFO HX_DEVNAME " %s: %d bytes firmware received!\n", __FUNCTION__, hx_firmware.size);

	disable_irq(hx_i2c.client->irq);

	mutex_lock(&drv_data->lock);

	if(hx_flash_write()) {
		printk(KERN_ERR HX_DEVNAME " %s: Write to flash failed!\n", __FUNCTION__);
	}
	else {
		if(hx_flash_read()) {
			printk(KERN_ERR HX_DEVNAME " %s: Check flash error!\n", __FUNCTION__);
		}
		else {
			msleep(200);
			pdata.reset_ts();
			hx_i2c_send_init_cmds();
			printk(KERN_INFO HX_DEVNAME " %s: Check flash ok. Himax HX8520 firmware updated successfully.\n", __FUNCTION__);
			enable_irq(hx_i2c.client->irq);
		}
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
	INIT_WORK(&hx_i2c.resume_work, hx_resume_work);
	drv_data = kmalloc(sizeof(struct hx_drv_data_t), GFP_KERNEL);

	if(NULL == drv_data) {
		printk(KERN_ERR HX_DEVNAME ": Can allocate driver data!\n");
		return 1;
	}

	dev_set_drvdata(&client->dev, drv_data);
	mutex_init(&drv_data->lock);

	hx_i2c_send_init_cmds();

	return 0;
}


static int hx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret  = 1;

	len_i2c_frame_finger_position = sizeof(struct hx_i2c_data_received);

	printk(KERN_INFO HX_DEVNAME " :probing driver\n");
	drv_data = NULL;

	printk(KERN_INFO HX_DEVNAME " :Himax HX8520 Capacitive Touch Screen Driver, (C) 2008 TomTom BV\n" );

	BUG_ON(!client);
	BUG_ON(!client->dev.platform_data);

	if(hx_i2c_init(client)) {
		ret = -ENOMEM;
		goto error_hx_i2c_init;
	}

	readinfo_workqueue = create_singlethread_workqueue("himax_read_info");
	if(!readinfo_workqueue) {
		printk(KERN_ERR HX_DEVNAME ": Cannot allocate workqueue!\n");
		ret = -ENOMEM;
		goto error_create_read_info_workqueue;
	}

	resume_workqueue = create_singlethread_workqueue("himax_resume_work");
	if(!resume_workqueue) {
		printk(KERN_ERR HX_DEVNAME ": Cannot allocate workqueue!\n");
		ret = -ENOMEM;
		goto error_create_resume_workqueue;
	}

	hx_cap_ts_input_dev = input_allocate_device();

	if (!hx_cap_ts_input_dev) {
		printk(KERN_ERR HX_DEVNAME ": Badf init_alloc_device()\n");
		ret = -ENOMEM;
		goto error_input_allocate_device;
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

	hx_cap_ts_input_dev->name = "Himax HX8520 Capacitive TouchScreen";
	hx_cap_ts_input_dev->phys = "input(ts)";
	hx_cap_ts_input_dev->id.vendor = 0xDEAD;
	hx_cap_ts_input_dev->id.product = 0xBEEF;
	hx_cap_ts_input_dev->id.version = 0x0101;

	/* Register with the input subsystem */
	ret = input_register_device(hx_cap_ts_input_dev);
	if (ret) {
		printk(KERN_ERR HX_DEVNAME ": Could not register input device(capacitive touchscreen)!\n");
		ret = -EIO;
		goto error_input_register_device;
	}

	printk(KERN_INFO HX_DEVNAME ": Input device registered\n");

	printk(KERN_INFO HX_DEVNAME ": irq number [%d]\n", client->irq);
	if (request_irq(client->irq, &hx_irq_handler, IRQ_TYPE_EDGE_FALLING, "hx_1", NULL)) {
		printk(KERN_ERR HX_DEVNAME " Could not allocate IRQ (hx_1)!\n");
		ret = -EIO;
		goto error_request_irq;
	}

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "fw_hx_cap_ts.bin", &client->dev, NULL, hx_loading_finished);
	if (ret) {
		printk(KERN_ERR HX_DEVNAME ": request_firmware_nowait returned error!\n");
		ret = -EIO;
	} 
	else
		return 0;

error_request_firmware:
	disable_irq(client->irq);
	free_irq(client->irq, NULL);
error_request_irq:
	input_unregister_device(hx_cap_ts_input_dev);
error_input_register_device:
	input_free_device(hx_cap_ts_input_dev);
error_input_allocate_device:
	destroy_workqueue(resume_workqueue);
error_create_resume_workqueue:
	destroy_workqueue(readinfo_workqueue);
error_create_read_info_workqueue:
	mutex_destroy(&drv_data->lock);
	kfree(drv_data);
error_hx_i2c_init:
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
	destroy_workqueue(resume_workqueue);

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
	queue_work(resume_workqueue, &hx_i2c.resume_work);
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

module_param_string(fw_ver, fw_ver, sizeof(fw_ver), 0444);

MODULE_AUTHOR("Chris Liu <chris.liu@tomtom.com>");
MODULE_DESCRIPTION("Driver for I2C connected Himax Capacitive Touchscreen IC.");
MODULE_LICENSE("GPL");

module_init(hx_init);
module_exit(hx_exit);
