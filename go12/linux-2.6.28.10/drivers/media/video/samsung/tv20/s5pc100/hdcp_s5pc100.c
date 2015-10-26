/* linux/drivers/media/video/samsung/tv20/s5pc100/hdcp_s5pc100.c
 *
 * hdcp raw ftn  file for Samsung TVOut driver
 *
 * SangPil Moon, Copyright (c) 2009 Samsung Electronics
 * 	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include <asm/io.h>

#include "tv_out_s5pc100.h"
#include "regs/regs-hdmi.h"

//#ifdef COFIG_TVOUT_RAW_DBG
#define S5P_HDCP_DEBUG 1
//#endif

#ifdef S5P_HDCP_DEBUG
#define HDCPPRINTK(fmt, args...) \
	printk("\t\t[HDCP] %s: " fmt, __FUNCTION__ , ## args)
#else
#define HDCPPRINTK(fmt, args...)
#endif


//#define USE_PJ_UPDATE
#define HAES_DEBUG

enum hdmi_run_mode {
	DVI_MODE,
	HDMI_MODE
};

enum hdmi_resolution {
	SD480P,
	SD480I,
	WWSD480P,
	HD720P,
	SD576P,
	WWSD576P,
	HD1080I
};

enum hdmi_color_bar_type {
	HORIZONTAL,
	VERTICAL
};

enum hdcp_event {
	// Stop HDCP
	HDCP_EVENT_STOP,
	// Start HDCP
	HDCP_EVENT_START,
	// Start to read Bksv,Bcaps
	HDCP_EVENT_READ_BKSV_START,
	// Start to write Aksv,An
	HDCP_EVENT_WRITE_AKSV_START,
	// Start to check if Ri is equal to Rj
	HDCP_EVENT_CHECK_RI_START,
	// Start 2nd authentication process
	HDCP_EVENT_SECOND_AUTH_START,
};

enum hdcp_state {
	NOT_AUTHENTICATED,
	BKSV_READ_DONE,
	AKSV_WRITE_DONE,
	SECOND_AUTHENTICATION_RDY,
	AUTHENTICATION_DONE
};

/*
 * Below CSC_TYPE is temporary. CSC_TYPE enum. 
 * may be included in SetSD480pVars_60Hz etc.
 *
 * LR : Limited Range (16~235)
 * FR : Full Range (0~255)
 */
enum HDMI_INTR_SRC {
	WAIT_FOR_ACTIVE_RX,
	WDT_FOR_REPEATER,
	EXCHANGE_KSV,
	UPDATE_P_VAL,
	UPDATE_R_VAL,
	AUDIO_OVERFLOW,
	AUTHEN_ACK,
	UNKNOWN_INT
};

const u8 hdcp_key[288] = {
	0xf8, 0x81, 0x24, 0xe6, 0xca, 0x07, 0xa3, 0x7b, 0x46, 0x6f, 0x88, 0x69, 0xf2, 0x4d, 0x1e, 0x22,
	0x7c, 0x91, 0x2a, 0x3b, 0xc3, 0x3d, 0xb4, 0xbd, 0x28, 0x91, 0x6e, 0x8b, 0x75, 0x27, 0x14, 0x46,
	0x20, 0x17, 0xf4, 0x09, 0x9d, 0x52, 0x72, 0xf3, 0x0a, 0x11, 0x0e, 0x11, 0x65, 0x86, 0x28, 0xe1,
	0x13, 0xc4, 0x1f, 0xf2, 0xd9, 0x9c, 0xfd, 0x82, 0x95, 0xee, 0x18, 0xe0, 0xef, 0xcb, 0x5f, 0x49,
	0x98, 0x19, 0xc6, 0xc2, 0x4c, 0x2d, 0x00, 0xa0, 0x0a, 0x44, 0x7b, 0x14, 0x73, 0x06, 0x0d, 0x6a,
	0xf5, 0x22, 0x19, 0x96, 0x00, 0xcb, 0x57, 0x67, 0x0e, 0x6a, 0x33, 0xb0, 0x35, 0x4f, 0x86, 0xe5,
	0x29, 0xbb, 0x9d, 0xc4, 0x38, 0xde, 0x8e, 0x5e, 0x20, 0xed, 0xc8, 0x46, 0x58, 0x98, 0x45, 0x7c,
	0xf8, 0xff, 0x21, 0x45, 0x20, 0x50, 0x41, 0x8b, 0xdb, 0xf1, 0x76, 0x8b, 0xff, 0x22, 0xe0, 0x15,
	0x02, 0x40, 0x65, 0xe0, 0x67, 0xb6, 0xd2, 0xc4, 0xcd, 0xde, 0x25, 0x05, 0x5f, 0x60, 0x85, 0x1b,
	0x97, 0x00, 0xed, 0x78, 0x52, 0x98, 0xde, 0x46, 0x0c, 0x84, 0x5f, 0x3b, 0xd9, 0x61, 0x02, 0xec,
	0xd2, 0x58, 0x3b, 0xff, 0x83, 0x42, 0x80, 0xc5, 0x46, 0x38, 0xc6, 0xbf, 0x10, 0x7e, 0xee, 0xf3,
	0xf1, 0x24, 0xfd, 0x86, 0xe8, 0x69, 0x45, 0x1c, 0xc9, 0x13, 0x2f, 0x8a, 0x3c, 0xfd, 0x68, 0xc6,
	0x20, 0x5c, 0x45, 0xed, 0x96, 0xe3, 0x9a, 0xa7, 0xef, 0x1d, 0xda, 0x56, 0x5e, 0xbe, 0x30, 0xa2,
	0x14, 0xdb, 0x55, 0xb8, 0x97, 0x53, 0xd4, 0xf1, 0x78, 0x6b, 0x7d, 0x27, 0x16, 0x0e, 0xc5, 0x7d,
	0xc7, 0xc2, 0x53, 0x4b, 0xed, 0x12, 0xea, 0x58, 0x38, 0xd0, 0xf5, 0x3e, 0x3d, 0xe3, 0xe5, 0xf1,
	0x4c, 0xeb, 0xb0, 0x7e, 0x4b, 0x16, 0x6f, 0xf5, 0xce, 0x24, 0xd6, 0x8b, 0x86, 0x1a, 0x47, 0x52,
	0xf5, 0x6f, 0x0a, 0x7d, 0xf5, 0x15, 0xdf, 0x6e, 0xba, 0x03, 0xcc, 0x4f, 0x1a, 0x35, 0x4b, 0x00,
	0x78, 0xc1, 0xfd, 0x7b, 0xa6, 0x05, 0x19, 0xc9, 0xae, 0xd3, 0xac, 0xbd, 0x3a, 0x9c, 0x57, 0x9d
};

struct s5p_hdcp_info {
	bool	is_repeater;
	u32	time_out;
	u32	count;
	u32	hdcp_enable;

	spinlock_t 	lock;
	
	struct i2c_client 	*client;

	wait_queue_head_t 	waitq;
	enum hdcp_event 	event;
	enum hdcp_state 	auth_status;

	struct work_struct  	work;
};

static struct s5p_hdcp_info hdcp_info = {
	.is_repeater 	= false,
	.time_out	= 0,
	.count		= 0,
	.hdcp_enable	= false,
	.client		= NULL,
	.event 		= HDCP_EVENT_STOP,
	.auth_status	= NOT_AUTHENTICATED,
	
};

#define HDCP_RI_OFFSET          0x08

/*
 * Read the HDCP data from Rx by using IIC
 */
static int __s5p_hdcp_i2c_read(struct i2c_client *client, u8 subaddr, u8 *data, u16 len)
{
	u8 addr = subaddr;
	int ret = 0;

	struct i2c_msg msg[] = {
		{ client->addr, 0, 1, &addr}, 
		{ client->addr, I2C_M_RD,len,data }
	};

	HDCPPRINTK("sub addr = 0x%08x, data len = %d\n", subaddr, len);

	if (i2c_transfer(client->adapter, msg, 2) != 2)
		ret = -EIO;

	HDCPPRINTK("ret :%d\n", ret);

#ifdef S5P_HDCP_DEBUG
	{
		int loop = 0;
		HDCPPRINTK("read_data :: \n");
		printk("\t\t\t");

		for (loop = 0;loop < len;loop++)
			printk("0x%02x  ", data[loop]);

		printk("\n");
	}
#endif

	return ret;
}

/*
 * Write the HDCP data to receiver by using IIC 
 * 	- use i2c_master_send()
 */
static int __s5p_hdcp_i2c_write(struct i2c_client *client, u8 *data, u16 len)
{
	int ret = 0;

	HDCPPRINTK("sub addr = 0x%08x, data len = %d\n", 
		data[0], len);

	if (i2c_master_send(client, (const char *) data, len) != len)
		ret = -EIO;

	HDCPPRINTK("ret :%d\n", ret);

	return ret;
}

/*
 * 1st Authentication step func.
 * Write the Ainfo data to Rx
 */
void __s5p_write_ainfo(void)
{
	int ret = 0;
	u8 ainfo[2];

	ainfo[0] = HDCP_Ainfo;
	ainfo[1] = 1;

	ret = __s5p_hdcp_i2c_write(hdcp_info.client, ainfo, 2);
	if(ret < 0)
		HDCPPRINTK("Can't write ainfo data through i2c bus\n");
}

/*
 * Write the An data to Rx
 */
void __s5p_write_an(void)
{
	int ret = 0;
	u8 an[AN_SIZE+1];

	an[0] = HDCP_An;

	// Read An from HDMI
	an[1] = readb(tvout_base + S5P_HDCP_An_0_0);
	an[2] = readb(tvout_base + S5P_HDCP_An_0_1);
	an[3] = readb(tvout_base + S5P_HDCP_An_0_2);
	an[4] = readb(tvout_base + S5P_HDCP_An_0_3);
	an[5] = readb(tvout_base + S5P_HDCP_An_1_0);
	an[6] = readb(tvout_base + S5P_HDCP_An_1_1);
	an[7] = readb(tvout_base + S5P_HDCP_An_1_2);
	an[8] = readb(tvout_base + S5P_HDCP_An_1_3);

	ret = __s5p_hdcp_i2c_write(hdcp_info.client, an, AN_SIZE + 1);
	if(ret < 0)
		HDCPPRINTK("Can't write an data through i2c bus\n");
	
#ifdef S5P_HDCP_DEBUG
	{
		u16 i = 0;
		for (i = 1; i < AN_SIZE + 1; i++) {
			HDCPPRINTK("HDCPAn[%d]: 0x%x \n", i, an[i]);
		}
	}
#endif		
}

/*
 * Write the Aksv data to Rx
 */
void __s5p_write_aksv(void)
{
	int ret = 0;	
	u8 aksv[AKSV_SIZE+1];

	aksv[0] = HDCP_Aksv;

	// Read Aksv from HDMI
	aksv[1] = readb(tvout_base + S5P_HDCP_AKSV_0_0);
	aksv[2] = readb(tvout_base + S5P_HDCP_AKSV_0_1);
	aksv[3] = readb(tvout_base + S5P_HDCP_AKSV_0_2);
	aksv[4] = readb(tvout_base + S5P_HDCP_AKSV_0_3);
	aksv[5] = readb(tvout_base + S5P_HDCP_AKSV_1);

	ret = __s5p_hdcp_i2c_write(hdcp_info.client,  aksv, AKSV_SIZE + 1);
	if(ret < 0)
		HDCPPRINTK("Can't write aksv data through i2c bus\n");

#ifdef S5P_HDCP_DEBUG
	{
		u16 i = 0;
		for (i = 1; i < AKSV_SIZE + 1; i++) {
			HDCPPRINTK("HDCPAksv[%d]: 0x%x\n", i, aksv[i]);
		}
	}
#endif
}

void __s5p_read_bcaps(void)
{
	int ret = 0;
	u8 bcaps;

	ret = __s5p_hdcp_i2c_read(hdcp_info.client, HDCP_Bcaps, &bcaps, 0x1);

	if(ret < 0){
		HDCPPRINTK("Can't read bcaps data from i2c bus\n");
		return;
	}

	writel(bcaps, tvout_base + S5P_HDCP_BCAPS);

	if (bcaps & REPEATER_SET)
		hdcp_info.is_repeater = 1;
	else
		hdcp_info.is_repeater = 0;

	HDCPPRINTK("Attached device type :  %s !! \n\r",
		   hdcp_info.is_repeater ? "REPEATER" : "SINK");

#ifdef USE_PJ_UPDATE
	{
		u32  ctr_val;

		if (bcaps[0] & ONE_DOT_ONE_FEATURES_SET) {
			ctr_val = readl(tvout_base + S5P_HDCP_CTRL);
			writel((ctr_val | EN_PJ_EN), tvout_base + S5P_HDCP_CTRL);
		}
	}

#endif
}

void __s5p_read_bksv(void)
{
	int ret = 0;
	u8 bksv[BKSV_SIZE] = {0, 0, 0, 0, 0};

	ret = __s5p_hdcp_i2c_read(hdcp_info.client, HDCP_Bksv, bksv, BKSV_SIZE);

	if(ret < 0){
		HDCPPRINTK("Can't read bksv data from i2c bus\n");
		return;
	}

	writel(bksv[0], tvout_base + S5P_HDCP_BKSV_0_0);
	writel(bksv[1], tvout_base + S5P_HDCP_BKSV_0_1);
	writel(bksv[2], tvout_base + S5P_HDCP_BKSV_0_2);
	writel(bksv[3], tvout_base + S5P_HDCP_BKSV_0_3);
	writel(bksv[4], tvout_base + S5P_HDCP_BKSV_1);

#ifdef S5P_HDCP_DEBUG
	{
		u8 i;

		for (i = 0; i < BKSV_SIZE; i++) {
			HDCPPRINTK("HDCPBksv[%d]: 0x%x\n", i, bksv[i]);
		}
	}
#endif
}

/*
 * Compare the 'P' value of Tx with that of Rx
 */
bool __s5p_compare_p_value(void)
{
	// ref. to HDCP Revision v1.2 p.89
	int ret = 0;
	u8 tx_p_val1, rx_p_val1;
	u8 tx_p_val2, rx_p_val2;
	u32 count = 0;

	tx_p_val1 = readl(tvout_base + S5P_HDCP_Pj);
	ret = __s5p_hdcp_i2c_read(hdcp_info.client, HDCP_Pj, &rx_p_val1, 0x1);
	if(ret < 0){
		HDCPPRINTK("Can't read p_1 data from i2c bus\n");
		return false;
	}

	while (tx_p_val1 != rx_p_val1) {
		tx_p_val2 = readl(tvout_base + S5P_HDCP_Pj);
		
		ret = __s5p_hdcp_i2c_read(hdcp_info.client, HDCP_Pj, &rx_p_val2, 0x1);
		if(ret < 0){
			HDCPPRINTK("Can't read p_2 data from i2c bus\n");
			return false;
		}

		// Check for stable Pj values
		if ((tx_p_val1 == tx_p_val2) && (rx_p_val1 == rx_p_val2)) {
			// Count mismatches
			count++;   

			if (count >= 3)
				break;

			tx_p_val1 = tx_p_val2;
			rx_p_val1 = rx_p_val2;

			// Wait for any of the values to change
			do {
				tx_p_val2 = readl(tvout_base + S5P_HDCP_Pj);
				ret = __s5p_hdcp_i2c_read(hdcp_info.client, HDCP_Pj, 
							&rx_p_val2, 0x1);
				if(ret < 0){
					HDCPPRINTK("Can't read p_2 data from i2c bus\n");
					return false;
				}
			} while ((tx_p_val2 == tx_p_val1) && (rx_p_val2 == rx_p_val1));
		}

		tx_p_val1 = tx_p_val2;
		rx_p_val1 = rx_p_val2;

	}

	return ((count < 3) ? true : false);

}

/*
 * Compare the R value of Tx with that of Rx
 */
bool __s5p_compare_r_value(void)
{
	int ret = 0;
	u8 ri[2], rj[2];
	u16 i;

	for (i = 0; i < R_VAL_RETRY_CNT; i++) {
		// Read R value from Tx
		ri[0] = readl(tvout_base + S5P_HDCP_Ri_0);
		ri[1] = readl(tvout_base + S5P_HDCP_Ri_1);

		// Read R value from Rx
		ret = __s5p_hdcp_i2c_read(hdcp_info.client, HDCP_Ri, rj, 2);
		if(ret < 0){
			HDCPPRINTK("Can't read r data from i2c bus\n");
			return false;
		}

#ifdef S5P_HDCP_DEBUG
		HDCPPRINTK("r_value :: HDCP(%d) \n", i);
		printk("\t\t\t Rx(ddc) ->");
		printk("rj[0]: 0x%02x, rj[1]: 0x%02x\n", rj[0], rj[1]);
		printk("\t\t\t Tx(register) ->");
		printk("ri[0]: 0x%02x, ri[1]: 0x%02x\n", ri[0], ri[1]);
#endif

		// Compare R value

		if ((ri[0] == rj[0]) && (ri[1] == rj[1]) && (ri[0] | ri[1]))	{
			return true;
		}
	}

	return false;
}

/*
 * Reset Authentication
 */
void __s5p_reset_authentication(void)
{
	// Disable encryption
	writel(HDCP_ENC_DIS, tvout_base + S5P_ENC_EN);
	//Ri is not matched
	writel(Ri_MATCH_RESULT__NO, tvout_base + S5P_HDCP_CHECK_RESULT);
	// Disable HDMI
	writel(0x0 << 0, tvout_base + S5P_HDMI_CON_0);
	// Enable HDMI
	writel(0x1 << 0, tvout_base + S5P_HDMI_CON_0);
	// Init check result
	writel(0x0, tvout_base + S5P_HDCP_CHECK_RESULT);
}

void __s5p_make_aes_key(void)
{
	u32  aes_reg_val;

	aes_reg_val = readl(tvout_base + S5P_HAES_CON);
	aes_reg_val = SCRAMBLER_KEY_START_EN;

	// Start generation of AES key
	writel(aes_reg_val, tvout_base + S5P_HAES_CON);

	do {
		aes_reg_val = readl(tvout_base + S5P_HAES_CON);
	} while (!(aes_reg_val & SCRAMBLER_KEY_DONE));

	// for debugging AES key
#ifdef HAES_DEBUG
	{
		u32  aes_key_val[32];
		u16 i;
		// Enable to debug AES key after scrambling
		aes_reg_val |= (0x1 << 5);

		writel(aes_reg_val, tvout_base + S5P_HAES_CON);

		for (i = 0; i < 32; i++) {
			aes_key_val[i] = readl(tvout_base + (S5P_HAES_CON + 0x200 + 4 * i));
			HDCPPRINTK("HDCPAesKey Val[%d]: 0x%2x\n", 
				i, aes_key_val[i]);
		}
	}
#endif
}

void __s5p_set_av_mute_on_off(u32 on_off)
{
	u32  gcp_byte;

	HDCPPRINTK("state : %s \n\r", on_off ? "av mute on":"av mute off");

	if (on_off == 1)
		gcp_byte = 0x01;
	else
		gcp_byte = 0x10;

	writel(gcp_byte, tvout_base + S5P_GCP_BYTE1);

	// packet will be transmitted within 384 cycles after active sync.
	writel(0x1 << 1, tvout_base + S5P_GCP_CON);
}

/*
 * Start encryption
 */
void __s5p_start_encryption(void)
{
	u32  hdcp_status;

	// Ri == Ri' |Ready the compared result of Ri
	writel(Ri_MATCH_RESULT__YES, tvout_base + S5P_HDCP_CHECK_RESULT);

	do {
		hdcp_status = readl(tvout_base + S5P_STATUS);
		// Wait for STATUS[7] to '1'
	} while ((hdcp_status & AUTHENTICATED) != AUTHENTICATED);

	// Start encryption
	writel(HDCP_ENC_ENABLE, tvout_base + S5P_ENC_EN);
}

/*
 * HAES function
 */
void __s5p_start_decrypting(const u8 * hdcp_key, u32  hdcp_key_size)
{
	u16 i = 0;
	u32  aes_start = 0;
	u32  aes_reg_val = 0;

	__s5p_make_aes_key();

	writel(hdcp_key_size, tvout_base + S5P_HAES_DATA_SIZE_L);

	for (i = 0; i < hdcp_key_size; i++) {
		writel(hdcp_key[i], tvout_base + S5P_HAES_DATA);
	}

	aes_reg_val = readl(tvout_base + S5P_HAES_CON);

	aes_reg_val |= HAES_START_EN;
	writel(aes_reg_val, tvout_base + S5P_HAES_CON);

	do {
		i++;
		aes_start = readl(tvout_base + S5P_HAES_CON);
	} while (aes_start & HAES_START_EN);

#ifdef HAES_DEBUG
	{
		u8 decrypted_data[288];
		HDCPPRINTK("[HDCP key] \n");

		printk("\t\t\t\t");
		for (i = 0; i < hdcp_key_size; i++) {

			printk("0x%02x, ", hdcp_key[i]);

			if((!(i%6)) && i)
				printk("\n\t\t\t\t");
		}

		// Set AES debug mode
		writel(0x1 << 0, tvout_base + S5P_HAES_CON + 0x80);

		printk("\n");
		
		HDCPPRINTK("[Decrypted Data]\n");

		printk("\t\t\t\t");
		for (i = 0; i < hdcp_key_size; i++) {
			decrypted_data[i] = readl(tvout_base + S5P_HAES_CON + 0xb0);

			printk("0x%2x, ", decrypted_data[i]);
			
			if((!(i%6)) && i)
				printk("\n\t\t\t\t");
		}

		printk("\n");
	}
#endif
}

/*
 * Check  whether Rx is repeater or not
 */
bool __s5p_check_repeater(void)
{
	int ret = 0;
	// Wait for 5 second
	u32  init_time = 0;
	u32  present_time = 0;
	u8 bcaps[1];
	u32  hdcp_ctrl;
	bool is_ready_ok = 0;
	u8 status[2];
	u32  dev_cnt;
	u8 ksv_list[HDCP_MAX_DEVS*HDCP_KSV_SIZE];
	// 20 means the length of SHA-1 hash V
	u8 rx_v[20];

	HDCPPRINTK(" \n\r");
	// Check KSV_FIFO Ready during 5 sec.
//	init_time = GetTickCount();

	do {
//		present_time = GetTickCount();

		ret = __s5p_hdcp_i2c_read(hdcp_info.client, HDCP_Bcaps, bcaps, 1);

		if(ret < 0){
			HDCPPRINTK("Can't read bcaps data from i2c bus\n");
			return false;
		}
		
		if (bcaps[0]&(0x1 << 5)) {
			is_ready_ok = 1;
			break;
		} else {
			is_ready_ok = 0;
		}
	} while (present_time - init_time < 5000);

	if (is_ready_ok) {
		hdcp_ctrl = readl(tvout_base + S5P_HDCP_CTRL);
		writel(hdcp_ctrl | (0x1 << 2), 
			tvout_base + S5P_HDCP_CTRL);
	} else
		return false;

	// Check MAX_CASCADE_EXCEEDED or MAX_DEVS_EXCEEDED indicator
	ret = __s5p_hdcp_i2c_read(hdcp_info.client, HDCP_BStatus, status, 2);
	if(ret < 0){
		HDCPPRINTK("Can't read status data from i2c bus\n");
		return false;
	}
	// MAX_CASCADE_EXCEEDED || MAX_DEVS_EXCEEDED
	if ((status[0]) & (0x1 << 7) || (status[1]) & (0x1 << 4))
		return false;

	// Read KSV list
	dev_cnt = (*status) & 0x7f;

	ret = __s5p_hdcp_i2c_read(hdcp_info.client, HDCP_KSVFIFO,
			    ksv_list, dev_cnt*HDCP_KSV_SIZE);
	if(ret < 0){
		HDCPPRINTK("Can't read ksv_list data from i2c bus\n");
		return false;
	}
	// Read SHA-1 from receiver
	ret = __s5p_hdcp_i2c_read(hdcp_info.client, HDCP_SHA1, rx_v, 20);
	if(ret < 0){
		HDCPPRINTK("Can't read rx_v data from i2c bus\n");
		return false;
	}
	
	return true;
}

/*
 * Check whether the HDCP event occurred or not
 */
bool __s5p_is_occurred_hdcp_event(void)
{
	u32  status_val;

	status_val = readl(tvout_base + S5P_STATUS);

	return (((status_val == (0x1 << 0) || status_val == (0x1 << 1) ||
		  status_val == (0x1 << 2) || status_val == (0x1 << 3) ||
		  status_val == (0x1 << 4))) ? true : false);
}

static void __s5p_hdcp_work(void *arg)
{
	u32  r_cnt = 0, tmp_cnt = 0;

	HDCPPRINTK("hdcp_info.event : 0x%08x \n\r", hdcp_info.event);

	// I2C int. is occurred for reading Bksv and Bcaps
	if (hdcp_info.event & (1 << HDCP_EVENT_READ_BKSV_START)) {
		HDCPPRINTK("HDCP_EVENT_READ_BKSV_START bh\n");

		__s5p_read_bcaps();
		__s5p_read_bksv();

		hdcp_info.auth_status = BKSV_READ_DONE;
		hdcp_info.event  &= ~(1 << HDCP_EVENT_READ_BKSV_START);
	}

	// Watchdog timer int. is occurred for checking repeater
	if (hdcp_info.event & (1 << HDCP_EVENT_SECOND_AUTH_START)) {
		HDCPPRINTK("HDCP_EVENT_SECOND_AUTH_START bh\n");
		// Check repeater

		if (__s5p_check_repeater()) {
			__s5p_start_encryption();
		}

		hdcp_info.event  &= ~(1 << HDCP_EVENT_SECOND_AUTH_START);
	}

	// An_Write int. is occurred for writing Ainfo, An and Aksv
	if (hdcp_info.event & (1 << HDCP_EVENT_WRITE_AKSV_START)) {
		HDCPPRINTK("HDCP_EVENT_WRITE_AKSV_START bh\n");

		if (hdcp_info.auth_status == BKSV_READ_DONE){
			__s5p_write_ainfo();
			__s5p_write_an();
			__s5p_write_aksv();

			/*
			* Wait for 100ms. Transmitter must not read Ro' value
			* sooner than 100ms after writing Aksv
			*/
			msleep(1100);

			hdcp_info.auth_status = AKSV_WRITE_DONE;
		}

		hdcp_info.event  &= ~(1 << HDCP_EVENT_WRITE_AKSV_START);
	}

	// Pj_Update int. is occurred for checking Pj val
	if (hdcp_info.event & UPDATE_PJ_INT_OCCURRED) {
		HDCPPRINTK("UPDATE_PJ_INT_OCCURRED int\n");

#ifdef USE_PJ_UPDATE
		if (hdcp_info.event = AUTHENTICATION_DONE) {
			if (__s5p_compare_p_value()) {
				check_result = readl(tvout_base + S5P_HDCP_CHECK_RESULT);
				// PJ_READY || PJ_MATCH
				check_result |= Pi_MATCH_RESULT__YES;   
				// Clear Pj int
				writel(UPDATE_PJ_INT_OCCURRED, 
					tvout_base + S5P_STATUS); 

				return;
			} else {
				check_result = readl(tvout_base + S5P_HDCP_CHECK_RESULT);
				check_result |= (0x1 << 1);
				check_result &= ~(0x1 << 2);
				writel(check_result, 
					tvout_base + S5P_HDCP_CHECK_RESULT);

				return;
			}
		}

		hdcp_info.event  &= ~(1 << UPDATE_PJ_INT_OCCURRED);
#endif
	}

	// Ri int. is occurred for comparing Ri and Ri'(from HDMI sink)
	if (hdcp_info.event & (1 << HDCP_EVENT_CHECK_RI_START)) {
		HDCPPRINTK("HDCP_EVENT_CHECK_RI_START bh\n");

		// For debugging

		if (hdcp_info.auth_status == AKSV_WRITE_DONE ||
		    hdcp_info.auth_status == AUTHENTICATION_DONE) {
			tmp_cnt = 0;

			if (__s5p_compare_r_value()) {
				if (hdcp_info.auth_status == AKSV_WRITE_DONE) {
					// Check whether HDMI sink is repeater or not
					if (hdcp_info.is_repeater) {
						hdcp_info.auth_status = SECOND_AUTHENTICATION_RDY;
					} else {
						hdcp_info.auth_status = AUTHENTICATION_DONE;

						HDCPPRINTK("HDCP Not Repeater\n");
						__s5p_start_encryption();
					}
				}

				r_cnt++;

				HDCPPRINTK("HDCP====> Cnt: %d\n", r_cnt);
				// Clear Ri int.
				writel(0x1 << 4, tvout_base + S5P_STATUS);   
			} else {
#ifndef     USE_PJ_UPDATE
				// Ri == Ri' |Ready the compared result of Ri
				writel(Ri_MATCH_RESULT__NO, 
					tvout_base + S5P_HDCP_CHECK_RESULT);
#else
				// Authentication reset step (from EMSC code)
				writel(HDCP_ENC_DIS, tvout_base + S5P_ENC_EN);
				writel(0x2, tvout_base + S5P_HDCP_CHECK_RESULT);
				writel(0x0, tvout_base + S5P_HDMI_CON_0);
				writel(0x3, tvout_base + S5P_HDMI_CON_0);
				writel(0x0, tvout_base + S5P_HDCP_CHECK_RESULT);
#endif
			}
		}

		hdcp_info.event &= ~(1 << HDCP_EVENT_CHECK_RI_START);
	}

	// wake up
	wake_up_interruptible(&hdcp_info.waitq);
}


/*
 * HDCP ISR.
 * If HDCP IRQ occurs, set hdcp_event and wake up the waitqueue.
 */
irqreturn_t __s5p_hdmi_irq(int irq, void *dev_id)
{
	u8 flag;
	u32 event = 0;

	HDCPPRINTK("()\n");

	// check HDCP Status
	flag = readb(tvout_base + S5P_STATUS);
	HDCPPRINTK("(0x%08x)\n", readb(tvout_base + S5P_STATUS));

	// processing interrupt
	// I2C INT
	if (flag & WTFORACTIVERX_INT_OCCURRED) {
		event |= (1 << HDCP_EVENT_READ_BKSV_START);
		// clear pending
		writeb(WTFORACTIVERX_INT_OCCURRED, tvout_base + S5P_STATUS);
	}

	// AN INT
	if (flag & EXCHANGEKSV_INT_OCCURRED) {
		event |= (1 << HDCP_EVENT_WRITE_AKSV_START);
		// clear pending
		writeb(EXCHANGEKSV_INT_OCCURRED, tvout_base + S5P_STATUS);
	}

	// RI INT
	if (flag & UPDATE_RI_INT_OCCURRED) {
		event |= (1 << HDCP_EVENT_CHECK_RI_START);
		// clear pending
		writeb(UPDATE_RI_INT_OCCURRED, tvout_base + S5P_STATUS);
	}

	// WATCHDOG INT
	if (flag & WATCHDOG_INT_OCCURRED) {
		event |= (1 << HDCP_EVENT_SECOND_AUTH_START);
		// clear pending
		writeb(WATCHDOG_INT_OCCURRED, tvout_base + S5P_STATUS);
	}

	// set event
	spin_lock_irq(&hdcp_info.lock);

	hdcp_info.event |= event;

	schedule_work(&hdcp_info.work);

	spin_unlock_irq(&hdcp_info.lock);

	return IRQ_HANDLED;
}

bool __s5p_is_decrypting_done(void)
{

	HDCPPRINTK("()\n");

	return (((readl(tvout_base + S5P_HAES_CON) & (0x1 << 0)) == 0x0) ? true : false);
}

void __s5p_set_hpd_detection(u32 detection_type)
{
	u32  hpd_reg_val;

	hpd_reg_val = (detection_type == CABLE_INSERT) ?
		      0x1 << 0 : 0x0 << 0;

	writel(hpd_reg_val, tvout_base + S5P_HPD);
	HDCPPRINTK("HPD status :: %d\n\r", readl(tvout_base + S5P_HPD));
}

/*
 * start  - start functions are only called under stopping HDCP
 */
bool __s5p_start_hdcp(struct i2c_client *ddc_port)
{
	hdcp_info.client = ddc_port;

	if (!hdcp_info.client) {
		HDCPPRINTK("DDC port is not available!!"
		       "Check hdmi receiver's DDC Port \n");
		return false;
	}


	// for bh
	INIT_WORK(&hdcp_info.work, (work_func_t)__s5p_hdcp_work);

	init_waitqueue_head(&hdcp_info.waitq);

	// for dev_dbg err.
	spin_lock_init(&hdcp_info.lock);

	/*
	// check hpd
	if(!((readl(tvout_base+S5P_HPD) & CABLE_INSERT)))
	{
		HDCPPRINTK("HDMI Cable unplugged!!\n");
		return false;
	}
	*/

	/* 
	 * Hidden spec. 
	 * 	- if it is missed, 
	 *	interrupt will not be occurred!!
	 */
	writel(CABLE_PLUGGED, tvout_base+S5P_HPD);

	writel(0x00, tvout_base + S5P_HDCP_OFFSET_TX_0);

	writel(0xA0, tvout_base + S5P_HDCP_OFFSET_TX_1);

	writel(0x00, tvout_base + S5P_HDCP_OFFSET_TX_2);

	writel(0x00, tvout_base + S5P_HDCP_OFFSET_TX_3);

	// HDCP memory read cycle count(0x4 is recommanded)
	writel(0x04, tvout_base + S5P_HDCP_CYCLE_AA);

	__s5p_start_decrypting(hdcp_key, 288);

	// all int. enable
	writel(HDMI_STATUS_EN_ALL, tvout_base + S5P_STATUS_EN);

	/*
	 * Disable advance cipher option, Enable CP(Content Protection),
	 * Disable time-out (This bit is only available in a REPEATER)
	 * Disable XOR shift,Disable Pj port update,Use external key
	 */
	writel(0x0 << 0 | 0x1 << 1 | 0x0 << 2 | 0x0 << 3 | 0x0 << 4 | 0x0 << 5,
	       tvout_base + S5P_HDCP_CTRL);

	HDCPPRINTK("STATUS_EN 0x%08x, HDCP_CTRL 0x%08x\n\r",
		   readl(tvout_base + S5P_STATUS_EN),
		   readl(tvout_base + S5P_HDCP_CTRL));

	return true;
}

/*
 * stop  - stop functions are only called under running HDCP
 */
void __s5p_stop_hdcp(void)
{
	u32  sfr_val;

	sfr_val = readl(tvout_base + S5P_HDCP_CTRL);
	sfr_val &= ~(0x1 << 1);

	writel(sfr_val, tvout_base + S5P_HDCP_CTRL);
	writel(0x0, tvout_base + S5P_HPD);
	writel(0x0, tvout_base + S5P_STATUS_EN);
}

