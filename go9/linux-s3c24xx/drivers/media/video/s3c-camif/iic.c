#include <linux/config.h>
#include <linux/delay.h>

#include <barcelona/gopins.h>

#include "camera.h"

#define IIC_GetSDA() IO_GetInput(HW_IIC_SDA)

static __inline void
IIC_SDALow(void)
{
	IO_Deactivate(HW_IIC_SDA);
}

static __inline void
IIC_SDAHigh(void)
{
	IO_SetInput(HW_IIC_SDA);
}

static __inline void
IIC_SCLLow(void)
{
	IO_Deactivate(HW_IIC_SCL);
}

static __inline void
IIC_SCLHigh(void)
{
	IO_SetInput(HW_IIC_SCL);
}

static void
IOIIC_Start(void)
{
	// Make SDA low while SCL is high
	IIC_SDAHigh();
	IIC_SCLHigh();
	IIC_SDALow();
	IIC_SCLLow();
}

static void
IOIIC_Stop(void)
{
	// Make SDA high while SCL is high
	IIC_SCLLow();
	IIC_SDALow();
	IIC_SCLHigh();
	IIC_SDAHigh();
}

static int
IOIIC_Transmit(unsigned char data)
{
	int bit;
	int result;
	
	for (bit=0;bit<8;bit++)
	{
		if (data & 0x80) IIC_SDAHigh(); else IIC_SDALow();
		IIC_SCLHigh();
		data <<= 1;
		IIC_SCLLow();
	}
	IIC_SDAHigh();
	IIC_SCLHigh();
	if (IIC_GetSDA() == 0) result = 1; else result = 0;
	IIC_SCLLow();

	return result;
}

static unsigned char
IOIIC_Receive(int generateAck)
{
	int bit;
	unsigned char data = 0;
	
	IIC_SDAHigh();
	for (bit=0;bit<8;bit++)
	{
		data <<= 1; 
		IIC_SCLHigh();
		if (IIC_GetSDA()) data |= 1;
		IIC_SCLLow();
	}
	if (generateAck)
	{
		IIC_SDALow(); /* ack = low */
		IIC_SCLHigh();
		IIC_SCLLow();
	} 
	
	return data;
}

static const int VQ16FS_address = 0x60;

int
cam_read_reg(struct s3c_cam* cam, int reg)
{
	int val = 0;
	int rc = 0;
	
	IOIIC_Start();
	rc += IOIIC_Transmit(VQ16FS_address);//write
	rc += IOIIC_Transmit(reg);
	IOIIC_Stop();
	mdelay(10);

	IOIIC_Start();
	rc += IOIIC_Transmit(VQ16FS_address+1);//read
	val = IOIIC_Receive(0);
	IOIIC_Stop();
	mdelay(10);

	printk("rc=%d - ", rc);
	
	return val;
}

void
cam_write_reg(struct s3c_cam* cam, int reg, int val)
{
	int rc = 0;
	
	IOIIC_Start();
	rc += IOIIC_Transmit(VQ16FS_address);
	rc += IOIIC_Transmit(reg);
	rc += IOIIC_Transmit(val);
	IOIIC_Stop();
	mdelay(10);

	printk("rc=%d - ", rc);
}
