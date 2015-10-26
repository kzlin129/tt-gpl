#include <linux/delay.h>		/* for mdelay */
#include <barcelona/dock-iic.h>
#include <barcelona/gopins.h>

#define NS73_ADRESS_W 0xCE
#define NS73_ADRESS_R 0xCF

int NS73_Write_Reg(unsigned char Address,unsigned char Value)
{
	int result=0;
	IOIIC_Start();
	
	IOIIC_Transmit(NS73_ADRESS_W);
	IOIIC_Transmit(Address);
	if (IOIIC_Transmit(Value))
		result=1;

	IOIIC_Stop();
	mdelay(1);
	return result;
}

unsigned char NS73_Read_Reg(unsigned char Address)
{
	unsigned char Data;
	
	IOIIC_Start();
	IOIIC_Transmit(NS73_ADRESS_W);
	IOIIC_Transmit(Address);
	IOIIC_Stop();
	
	mdelay(1);

	IOIIC_Start();
	IOIIC_Transmit(NS73_ADRESS_R);
	Data=IOIIC_Receive(0);
	IOIIC_Stop();

	return Data;
}

void NS73_SetFrequency (int fTX)  
{
	int N;

	if (fTX==0)
	{
		NS73_Write_Reg(0x00,0x84);//power off  +  Mute on
	}
	if ((fTX >= 87000000) && (fTX <=108000000))	//transmitter range 
	{
		N= (fTX + 304000 + 4096)/8192;  //foff = 304kHz, f_synthesizer 8.192 kHz, add 4096 for auto round up or down
		NS73_Write_Reg(0x03,(N & 0x000000FF));
		NS73_Write_Reg(0x04,(N & 0x00003F00)>>8);
		if (fTX <= 90500000)
			NS73_Write_Reg(0x08,0x1b);//CEX band 3
		if ((fTX >90500000) && (fTX <= 96000000))
			NS73_Write_Reg(0x08,0x1a);//CEX band 2
		if ((fTX >96000000) && (fTX <= 102000000))
			NS73_Write_Reg(0x08,0x19);//CEX band 1
		if ((fTX >102000000) && (fTX <= 108000000))
			NS73_Write_Reg(0x08,0x18);//CEX band 0
		//mdelay(2000);//ramp up noise
		NS73_Write_Reg(0x00,0x41);//power on, mute off
	}
	else 
	{
		NS73_Write_Reg(0x00,0x84);//power off
	}
}

void NS73_PowerOff(void)
{
		NS73_Write_Reg(0x00,0x84);//power off  +  Mute on
}

int NS73_Init (void)
{
	int result=0;
	
	if (NS73_Write_Reg(0x0E,0x05))//soft reset
	{
		NS73_Write_Reg(0x00,0x84);//keep power off, Mute on
		NS73_Write_Reg(0x01,0xb4);//pilot on
		NS73_Write_Reg(0x02,0x03);//unlock detector (ULD)off
		NS73_Write_Reg(0x05,0x00);
		NS73_Write_Reg(0x06,0x1a);
		NS73_Write_Reg(0x07,0x00);
		NS73_Write_Reg(0x09,0x00);
		NS73_Write_Reg(0x0A,0x00);
		NS73_Write_Reg(0x0B,0x00);
		result=1;
	}
		
	mdelay(3);
	return result;
}

int NS73_PowerOn(int frequency)
{
	int result=0;
	if (NS73_Init())
	{
		NS73_Write_Reg(0x00,0x85);//turn power on, mute on
		mdelay(3);
		NS73_Write_Reg(0x0E,0x05);//soft reset
		NS73_Write_Reg(0x06,0x1e);//set CIA 80uA CIB 320uA
		NS73_SetFrequency(frequency);//set frequency
		NS73_Write_Reg(0x06,0x1a);//set CIA 1.25uA CIB 320uA
		result=1;
	}
	
	return result;
}

int NS73_Detected(void)
{
	int result;

   if (!IO_GetInput(DOCK_INT)) return 0;
	IOIIC_Start();
	result = IOIIC_Transmit(NS73_ADRESS_W);
	IOIIC_Stop();
	return result;
}
