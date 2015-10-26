/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/


/*****************************************************************************
*
*  pmu_bcm59040.h
*
*  PURPOSE:
*
*  This file defines the interface to the Broadcom BCM59040 PMU chip.
*
*  NOTES:
*       
*
*****************************************************************************/

//******************************************************************************
//                          definition block
//******************************************************************************
#if !defined( PMU_BCM59040_H )
#define PMU_BCM59040_H


#ifdef CONFIG_BCM_PMU
#include <linux/broadcom/pmu_chip.h>
#endif

#define PMU_59040_FG_LAYOUT

#define PMU_59040_B0

#define PMU_59040_A0_REV 0x04
#define PMU_59040_B0_REV 0x14
#define PMU_59040_B1_REV 0x24

typedef enum 
{
   BCM59040_REGULATOR_LDO1 = 0,     
   BCM59040_REGULATOR_LDO2,     
   BCM59040_REGULATOR_LDO3,    
   BCM59040_REGULATOR_LDO4,    
   BCM59040_REGULATOR_LDO5,    
   BCM59040_REGULATOR_LDO6,      
   BCM59040_REGULATOR_CSR,
   BCM59040_REGULATOR_IOSR,

   BCM59040_NUM_REGULATORS
} BCM59040_regulators_t;
              
/* Charger IDs */
typedef enum 
{
   BCM59040_CHARGER_MAIN = 0,
   BCM59040_CHARGER_USB,
   BCM59040_NUM_CHARGERS
} BCM59040_chargers_t;


typedef enum 
{
   BCM59040_USB_NONE = 0,
   BCM59040_USB_CLA,
   BCM59040_USB_IDLE,	
   BCM59040_USB_DEV
} bcm59040_usb_charger_t;


#define BCM59040_BIT_REG_OFF        0x2
#define BCM59040_BIT_REG_ON         0x0
#define BCM59040_BIT_REG_ON_ALT     0x3
#define BCM59040_BIT_REG_ECO        0x1

// I2C low speed address for BCM59040
//Note: This base address is 7 bits long and this is shifted by 1-bit
//      to accomodate the read/write bit.
#define BCM59040_I2C_BASE_ADDR   0x08    // write address 00010000
#define BCM59040_BASE_W      0x10        // write address 00010000
#define BCM59040_BASE_R      0x11        // read  address 00010001
                                         
// high speed address
#define BCM59040_I2C_HISPD_BASE_ADDR   0x04    
#define BCM59040_HISPD_BASE_W      0x08        // write address 00001000
#define BCM59040_HISPD_BASE_R      0x09        // read  address 00001001
                                         
// PMU REGISTERS MAPPING --------------------------------------------------------
/* page 0 registers */
#define BCM59040_REG_I2CNTRL        0x00    // R/W  i2c control register
#define BCM59040_REG_GPIOCTRL1      0x03    // R/W  GPIO1 control
#define BCM59040_REG_GPIOCTRL2      0x04    // R/W  GPIO2 control
#define BCM59040_REG_GPIOCTRL3      0x05    // R/W  GPIO3 control
#define BCM59040_REG_GPIOCTRL4      0x06    // R/W  GPIO4 control
#define BCM59040_REG_GPIOCTRL5      0x07    // R/W  GPIO5 control
#define BCM59040_REG_HOSTACT        0x0A    // R/W  Host Act register
#define BCM59040_REG_PMUGID         0x0B    // R/W  PMU Generation ID
#define BCM59040_REG_PLLADCCLKSEL   0x0C    // R/W
#define BCM59040_REG_PLLN           0x0D    // R/W  pll divider
#define BCM59040_REG_PLLCTRL        0x0E    // R/W  PLL control
#define BCM59040_REG_CMPCTRL1       0x10    // R/W  cmp1 control
#define BCM59040_REG_CMPCTRL2       0x11    // R/W  cmp2 control
#define BCM59040_REG_CMPCTRL3       0x12    // R/W  LOWBATCVS[1:0]
#define BCM59040_REG_CMPCTRL4       0x13    // R/W  cmp4 control
#define BCM59040_REG_CMPCTRL5       0x14    // R/W  cmp5 control
#define BCM59040_REG_CMPCTRL6       0x15    // R/W  cmp6 control
#define BCM59040_REG_CMPCTRL7       0x16    // R/W  OTPBG3VTRIM[7:0]
#define BCM59040_REG_CMPCTRL8       0x17    // R/W  cmp8 control
#define BCM59040_REG_CMPCTRL9       0x18    // R/W  cmp9 control
#define BCM59040_REG_CMPCTRL10      0x19    // R/W  cmp10 control
#define BCM59040_REG_CMPCTRL11      0x1A    // R/W  cmp11 control
#define BCM59040_REG_CMPCTRL12      0x1B    // R/W  cmp12 control
#define BCM59040_REG_NTCCTRL1       0x1C    // R/W  NTC1 control
#define BCM59040_REG_NTCCTRL2       0x1D    // R/W  NTC2 control
#define BCM59040_REG_MBTEMPHYS      0x1E    // R/W  

#define BCM59040_REG_PONKEYBCNTRL1  0x20    // R/W  ponkey control1
#define BCM59040_REG_PONKEYBCNTRL2  0x21    // R/W  ponkey control2                                            
#define BCM59040_REG_RSTRTCNTRL     0x22    // R/W  ponkey restart control
#define BCM59040_REG_PWMLEDCTRL1    0x24    // R/W  pwmled control1
#define BCM59040_REG_PLD1CTRL1      0x25    // R/W  pwmled1 control1
#define BCM59040_REG_PLD1CTRL2      0x26    // R/W  pwmled1 control2
#define BCM59040_REG_PLD1CTRL3      0x27    // R/W  pwmled1 control3
#define BCM59040_REG_PLD2CTRL1      0x28    // R/W  pwmled2 control1
#define BCM59040_REG_PLD2CTRL2      0x29    // R/W  pwmled2 control2
#define BCM59040_REG_PLD2CTRL3      0x2A    // R/W  pwmled2 control3
#define BCM59040_REG_PLD3CTRL1      0x2B    // R/W  pwmled3 control1
#define BCM59040_REG_PLD3CTRL2      0x2C    // R/W  pwmled3 control2
#define BCM59040_REG_PLD3CTRL3      0x2D    // R/W  pwmled3 control3

// interrupt section
#define BCM59040_REG_INT1           0x30    // R&C  interrupt 
#define BCM59040_REG_INT2           0x31    // R&C  interrupt 
#define BCM59040_REG_INT3           0x32    // R&C  interrupt 
#define BCM59040_REG_INT4           0x33    // R&C  interrupt 
#define BCM59040_REG_INT5           0x34    // R&C  interrupt 
#define BCM59040_REG_INT6           0x35    // R&C  interrupt 
#define BCM59040_REG_INT7           0x36    // R&C  interrupt 
#define BCM59040_REG_INT8           0x37    // R&C  interrupt 
#define BCM59040_REG_INT9           0x38    // R&C  interrupt 
#define BCM59040_REG_INT1M          0x3C    // R/W  interrupt mask 
#define BCM59040_REG_INT2M          0x3D    // R/W  interrupt mask 
#define BCM59040_REG_INT3M          0x3E    // R/W  interrupt mask 
#define BCM59040_REG_INT4M          0x3F    // R/W  interrupt mask 
#define BCM59040_REG_INT5M          0x40    // R/W  interrupt mask 
#define BCM59040_REG_INT6M          0x41    // R/W  interrupt mask 
#define BCM59040_REG_INT7M          0x42    // R/W  interrupt mask 
#define BCM59040_REG_INT8M          0x43    // R/W  interrupt mask 
#define BCM59040_REG_INT9M          0x44    // R/W  interrupt mask 

// otg section
#define BCM59040_REG_OTGCTRL1       0x48    // R/W  
#define BCM59040_REG_OTGCTRL2       0x49    // R/W  
#define BCM59040_REG_BBCCTRL        0x4C    // R/W 
#define BCM59040_REG_BBLOWDB        0x4D    // R/W  backup battery debounce register [3:0]
                                            
// RTC 
#define BCM59040_REG_RTCSC           0x50   // R/W  real time clock seconds [5:0]
#define BCM59040_REG_RTCMN           0x51   // R/W  real time clock minutes [5:0]
#define BCM59040_REG_RTCHR           0x52   // R/W  real time clock hours [4:0]
#define BCM59040_REG_RTCWD           0x53   // R/W  real time clock weekday [2:0]
#define BCM59040_REG_RTCDT           0x54   // R/W  real time clock day [4:0]
#define BCM59040_REG_RTCMT           0x55   // R/W  real time clock month [3:0]
#define BCM59040_REG_RTCYR           0x56   // R/W  real time clock year [7:0]
#define BCM59040_REG_RTCSC_A1        0x57   // R/W  alarm clock 1 seconds [5:0]
#define BCM59040_REG_RTCMN_A1        0x58   // R/W  alarm clock 1 minutes [5:0]
#define BCM59040_REG_RTCHR_A1        0x59   // R/W  alarm clock 1 hours [4:0]
#define BCM59040_REG_RTCWD_A1        0x5A   // R/W  alarm clock 1 weekday [6:0]
#define BCM59040_REG_RTCDT_A1        0x5B   // R/W  alarm clock 1 day [4:0]
#define BCM59040_REG_RTCMT_A1        0x5C   // R/W  alarm clock 1 month [3:0]
#define BCM59040_REG_RTCYR_A1        0x5D   // R/W  alarm clock 1 year [7:0]                                           // 
#define BCM59040_REG_RTCSECURE       0x5E   // R/W  RTC secure register
              
// Charger controls
#define BCM59040_REG_MBCCTRL1        0x60   // R/W  main battery charger control [6:0]
#define BCM59040_REG_MBCCTRL2        0x61   // R/W  main battery charger control [7:0]
#define BCM59040_REG_MBCCTRL3        0x62   // R/W  main battery charger control [7:0]
#define BCM59040_REG_MBCCTRL4        0x63   // R/W  main battery charger control [6:0]
#define BCM59040_REG_MBCCTRL5        0x64   // R/W  main battery charger control [7:0]
#define BCM59040_REG_MBCCTRL6        0x65   // R/W  main battery charger control [6:0]
#define BCM59040_REG_MBCCTRL7        0x66   // R/W  main battery charger control [7:0]
#define BCM59040_REG_MBCCTRL8        0x67   // R/W  main battery charger control [4:0]
#define BCM59040_REG_MBCCTRL9        0x68   // R/W  main battery charger control [6:0]
#define BCM59040_REG_MBCCTRL10       0x69   // R/W  main battery charger control [7:0]
#define BCM59040_REG_MBCCTRL11       0x6A   // R/W  main battery charger control [4:0]
#define BCM59040_REG_MBCCTRL12       0x6B   // R/W  main battery charger control []
#define BCM59040_REG_MBCCTRL13       0x6C   // R/W  main battery charger control []
#define BCM59040_REG_MBCCTRL14       0x6D   // R/W  main battery charger control []
#define BCM59040_REG_MBCCTRL15       0x6E   // R/W  main battery charger control []
#define BCM59040_REG_MBCCTRL16       0x6F   // R/W  main battery charger control []
#define BCM59040_REG_MBCCTRL17       0x70   // R/W  main battery charger control []
#define BCM59040_REG_MBCCTRL18       0x71   // R/W  OTPMBCTRM[5:0]: main charger trim code
#define BCM59040_REG_MBCCTRL19       0x72   // R/W  OTPUSBCCTRM[3:0]: USB CC mode current trimming code

// Fuel gauge registers
#define BCM59040_REG_FGSMPLB1        0x78    // R    Fuel gauge current sample outb register 1 [7:0]
#define BCM59040_REG_FGSMPLB2        0x79    // R    Fuel gauge current sample outb register 2 [5:0]
#define BCM59040_REG_FGSMPL1         0x7A    // R&C  fuel gauge current sample [7:0]
#define BCM59040_REG_FGSMPL2         0x7B    // R&C  fuel gauge current sample [5:0]
#define BCM59040_REG_FGSAMPLE1       0x7C    // R&C  fuel gauge sample [7:0] ???
#define BCM59040_REG_FGSAMPLE2       0x7D    // R&C  fuel gauge sample [7:0] ???
#define BCM59040_REG_FGSAMPLE3       0x7E    // R&C  fuel gauge sample [7:0] ???
#define BCM59040_REG_FGCTRL1         0x80    // R/W  fuel gauge control [7:0]
#define BCM59040_REG_FGCTRL2         0x81    // R/W  fuel gauge control [7:0]
#define BCM59040_REG_FGCTRL3         0x82    // R/W  fuel gauge control [6:0]
#define BCM59040_REG_FGCTRL4         0x83    // R/W  fuel gauge control [6:0]  ???                                           
#define BCM59040_REG_FGCICCTRL       0x84    // R/W  ATE writable [0]                                         
#define BCM59040_REG_FGOPMODCTRL     0x85    // R/W  Fuel gauge operation mode register [3:0]
#define BCM59040_REG_FGACCM1         0x8B    // R&C  fuel gauge accumulation [7:0]
#define BCM59040_REG_FGACCM2         0x8C    // R&C  fuel gauge accumulation [7:0]
#define BCM59040_REG_FGACCM3         0x8D    // R&C  fuel gauge accumulation [7:0]
#define BCM59040_REG_FGACCM4         0x8E    // R&C  fuel gauge accumulation [1:0]                                             
#define BCM59040_REG_FGSLEEPCNT1     0x90    // R    Fuel gauge sleep count register1 [7:0]
#define BCM59040_REG_FGSLEEPCNT2     0x91    // R    Fuel gauge sleep count register2 [7:0]
#define BCM59040_REG_FGCNT1          0x92    // R&C  fuel gauge sample count [7:0]
#define BCM59040_REG_FGCNT2          0x93    // R&C  fuel gauge sample count [3:0]
#define BCM59040_REG_WAKECNT1        0x94    // R/W  Fuel gauge wake up count register1 [7:0]
#define BCM59040_REG_WAKECNT2        0x95    // R/W  Fuel gauge wake up count register2 [7:0]
#define BCM59040_REG_WAKETIME1       0x96    // R/W  Fuel gauge wake up time register1 [7:0]
#define BCM59040_REG_WAKETIME2       0x97    // R/W  Fuel gauge wake up time register2 [7:0]
#define BCM59040_REG_WAKEACCM1       0x98    // R/W  Fuel gauge wake up accumulator register1 [7:0]                                       
#define BCM59040_REG_WAKEACCM2       0x99    // R/W  Fuel gauge wake up accumulator register2 [7:0]
#define BCM59040_REG_WAKEACCM3       0x9A    // R/W  Fuel gauge wake up accumulator register3 [7:0]
#define BCM59040_REG_WAKEACCM4       0x9B    // R/W  Fuel gauge wake up accumulator register4 [7:0]
#define BCM59040_REG_FGGNRL1         0x9C    // R/W  Fuel gauge general purpose register1 [7:0]
#define BCM59040_REG_FGGNRL2         0x9D    // R/W  Fuel gauge general purpose register2 [7:0]
#define BCM59040_REG_FGGNRL3         0x9E    // R/W  Fuel gauge general purpose register3 [7:0]
                                                                             
// Fuel gauge test registers
#define BCM59040_REG_FGTRIMGN1_1     0x87    // R    ATE reaeable [7:0]
#define BCM59040_REG_FGTRIMGN1_2     0x88    // R    ATE reaeable [5:0]
#define BCM59040_REG_FGTRIMGN2_1     0x89    // R    ATE reaeable [7:0]
#define BCM59040_REG_FGTRIMGN2_2     0x8A    // R    ATE reaeable [5:0]
                                             
// ADC registers
#define BCM59040_REG_ADCCTRL1        0xA0   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL2        0xA1   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL3        0xA2   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL4        0xA3   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL5        0xA4   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL6        0xA5   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL7        0xA6   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL8        0xA7   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL9        0xA8   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL10       0xA9   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL11       0xAA   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL12       0xAB   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL13       0xAC   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL14       0xAD   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL15       0xAE   // R/W  ADC control []
#define BCM59040_REG_ADCCTRL16       0xAF   // R/W  ADC control []

// Core switching regulator controls
#define BCM59040_REG_CSRCTRL1        0xB0    // R/W  core switching reg control [6:0]
#define BCM59040_REG_CSRCTRL2        0xB1    // R/W  core switching reg control [6:0]
#define BCM59040_REG_CSRCTRL3        0xB2    // R/W  core switching reg control [7:0]
#define BCM59040_REG_CSRCTRL4        0xB3    // R/W  core switching reg control [7:0]
#define BCM59040_REG_CSRCTRL5        0xB4    // R/W  core switching reg control [7:0]
#define BCM59040_REG_CSRCTRL6        0xB5    // R/W  core switching reg control [7:0]
#define BCM59040_REG_CSRCTRL7        0xB6    // R/W  core switching reg control [7:0]
#define BCM59040_REG_CSRCTRL8        0xB7    // R/W  core switching reg control [7:0]
#define BCM59040_REG_CSRCTRL9        0xB8    // R/W  core switching reg control [7:0]
#define BCM59040_REG_CSRCTRL10       0xB9    // R/W  core switching reg control [4:0]
#define BCM59040_REG_CSRCTRL11       0xBA    // R/W  core switching reg control [3:0]                                       // 
                 
// IO switching regulator controls
#define BCM59040_REG_IOSRCTRL1       0xBB    // R/W  io switching reg control [3:0]
#define BCM59040_REG_IOSRCTRL2       0xBC    // R/W  io switching reg control [7:0]
#define BCM59040_REG_IOSRCTRL3       0xBD    // R/W  io switching reg control [7:0]
#define BCM59040_REG_IOSRCTRL4       0xBE    // R/W  io switching reg control [7:0]
#define BCM59040_REG_IOSRCTRL5       0xBF    // R/W  io switching reg control [7:0]
#define BCM59040_REG_IOSRCTRL6       0xC0    // R/W  io switching reg control [7:0]
#define BCM59040_REG_IOSRCTRL7       0xC1    // R/W  io switching reg control [7:0]
#define BCM59040_REG_IOSRCTRL8       0xC2    // R/W  io switching reg control [7:0]
#define BCM59040_REG_IOSRCTRL9       0xC3    // R/W  io switching reg control [2:0]
                                        // bit layouts for PC2,PC1 status are: 0,0[1:0] 0,1[3:2] 1,0[5:4] 1,1[7:6]

// LDO swithcers voltage controls
#define BCM59040_REG_LDO1CTRL        0xC4    // R/W  ldo control 
#define BCM59040_REG_LDO2CTRL        0xC5    // R/W  ldo control 
#define BCM59040_REG_LDO3CTRL        0xC6    // R/W  ldo control 
#define BCM59040_REG_LDO4CTRL        0xC7    // R/W  ldo control 
#define BCM59040_REG_LDO5CTRL        0xC8    // R/W  ldo control 
#define BCM59040_REG_LDO6CTRL        0xC9    // R/W  ldo control 
#define BCM59040_REG_CSRDVSCTRL      0xCA    // R/W  
#define BCM59040_REG_CSRPCDVS0       0xCB    // R/W  
#define BCM59040_REG_CSRPCDVS1       0xCC    // R/W  
#define BCM59040_REG_CSRPCDVS2       0xCD    // R/W  
#define BCM59040_REG_CSRPCDVS3       0xCE    // R/W
#define BCM59040_REG_IOSRDVSCTRL     0xCF    // R/W  
#define BCM59040_REG_IOSRPCDVS0      0xD0    // R/W  
#define BCM59040_REG_IOSRPCDVS1      0xD1    // R/W  
#define BCM59040_REG_IOSRPCDVS2      0xD2    // R/W  
#define BCM59040_REG_IOSRPCDVS3      0xD3    // R/W                                            
#define BCM59040_REG_L1PMCTRL        0xD4    // R/W  
#define BCM59040_REG_L2PMCTRL        0xD5    // R/W  
#define BCM59040_REG_L3PMCTRL        0xD6    // R/W  
#define BCM59040_REG_L4PMCTRL        0xD7    // R/W  
#define BCM59040_REG_L5PMCTRL        0xD8    // R/W
#define BCM59040_REG_L6PMCTRL        0xD9    // R/W  
#define BCM59040_REG_CSRPMCTRL       0xDA    // R/W  
#define BCM59040_REG_IOSRPMCTRL      0xDB    // R/W  
#define BCM59040_REG_HBCTRL          0xDC    // R/W  
#define BCM59040_REG_CLKHALTCTRL      0xDD    // R/W    

#define BCM59040_REG_PWRCURMODE1     0xDE    // R                                            
#define BCM59040_REG_PWRCURMODE2     0xDF    // R  
#define BCM59040_REG_PM0GRPCTRL1     0xE0    // R/W  
#define BCM59040_REG_PM0GRPCTRL2     0xE1    // R/W  
#define BCM59040_REG_PM0GRPCTRL3     0xE2    // R/W  
#define BCM59040_REG_PM0GRPCTRL4     0xE3    // R/W
#define BCM59040_REG_PM1GRPCTRL1     0xE4    // R/W  
#define BCM59040_REG_PM1GRPCTRL2     0xE5    // R/W  
#define BCM59040_REG_PM1GRPCTRL3     0xE6    // R/W  
#define BCM59040_REG_PM1GRPCTRL4     0xE7    // R/W  
#define BCM59040_REG_PM2GRPCTRL1     0xE8    // R/W  
#define BCM59040_REG_PM2GRPCTRL2     0xE9    // R/W
#define BCM59040_REG_PM2GRPCTRL3     0xEA    // R/W  
#define BCM59040_REG_PM2GRPCTRL4     0xEB    // R/W 
#define BCM59040_REG_PM3GRPCTRL1     0xEC    // R/W  
#define BCM59040_REG_PM3GRPCTRL2     0xED    // R/W  
#define BCM59040_REG_PM3GRPCTRL3     0xEE    // R/W  
#define BCM59040_REG_PM3GRPCTRL4     0xEF    // R/W

// Environment registers
#define BCM59040_REG_ENV1            0xF0    // R/W  
#define BCM59040_REG_ENV2            0xF1    // R/W 
#define BCM59040_REG_ENV3            0xF2    // R/W  
#define BCM59040_REG_ENV4            0xF3    // R/W
#define BCM59040_REG_ENV5            0xF4    // R/W  
#define BCM59040_REG_ENV6            0xF5    // R/W

// Analog test registers
#define BCM59040_REG_PDCMPSYN        0xF7    // R    presence detection/comparator status [7:0]
#define BCM59040_REG_PDCMPSYN1       0xF8    // R    presence detection/comparator status [7:0]
#define BCM59040_REG_PDCMPSYN2       0xF9    // R    presence detection/comparator status [7:0]
#define BCM59040_REG_PDCMPSYN3       0xFA    // R    presence detection/comparator status [7:0]
#define BCM59040_REG_PDCMPSYN4       0xFB    // R    presence detection/comparator status [7:0]
#define BCM59040_REG_PDCMPSYN5       0xFC    // R    presence detection/comparator status [7:0]                                        
#define BCM59040_REG_PAGESEL         0xFF    // R    page select is in both page 0 and page 1

/* page 1 registers */

#define BCM59040_PAGE1_BASE_ADDR	0x100    // Only usable with new PMU drivers when added to
                                             // a register address it will select page 1 also

// Mode registers
#define BCM59040_REG_ADCTEST1        0xA0    // R/W  
#define BCM59040_REG_ADCTEST2        0xA1    // R/W  
#define BCM59040_REG_ADCTEST3        0xA2    // R/W  
#define BCM59040_REG_ADCTEST4        0xA3    // R/W  
#define BCM59040_REG_ADCTEST5        0xA4    // R/W  
#define BCM59040_REG_ADCTEST6        0xA5    // R/W  
#define BCM59040_REG_TPEN            0xD0    // R/W  enable test mux control [6:0]
#define BCM59040_REG_ATMCTRL         0xD1    // R/W  analog test mode control register [7:0]
#define BCM59040_REG_ANADBG1         0xD2    // R/W  analog debug register 1 [7:0]
#define BCM59040_REG_ANADBG2         0xD3    // R/W  analog debug register 2 [7:0]
#define BCM59040_REG_ANADBG3         0xD4    // R/W  analog debug register 3 [7:0]
#define BCM59040_REG_ANADBG4         0xD5    // R/W  analog debug register 4 [7:0]
#define BCM59040_REG_SRCTRL          0xD6    // R/W  
#define BCM59040_REG_CNPDBG1         0xD8    // R/W  
#define BCM59040_REG_CNPDBG2         0xD9    // R/W  
#define BCM59040_REG_CNPDBG3         0xDA    // R/W  
#define BCM59040_REG_CNPDBG4         0xDB    // R/W  
#define BCM59040_REG_CNPDBG5         0xDC    // R/W  
#define BCM59040_REG_CNPDBG6         0xDD    // R/W  
#define BCM59040_REG_CNPDBG7         0xDE    // R/W  
#define BCM59040_REG_CNPDBG8         0xDF    // R/W  
#define BCM59040_REG_CNPDBG9         0xE0    // R/W  
#define BCM59040_REG_DBGOTG1         0xE1    // R/W  
#define BCM59040_REG_DBGOTG2         0xE2    // R/W  
#define BCM59040_REG_OTPWRCTRL       0xF0    // R/W  
#define BCM59040_REG_OTPWRDATA       0xF1    // R/W  
#define BCM59040_REG_OTPWRADDR       0xF2    // R/W  
#define BCM59040_REG_OTPRDTST        0xF3    // R    Analog test mode OTP read register [7:0]  
#define BCM59040_REG_PMUMODE         0xF4    // R/W  
#define BCM59040_REG_PMUID           0xFE    // R    pmu mode control [1:0]                                                     


#define BCM59040_NUM_RTC_REG         7   // Number of RTC registers 

#define BCM59040_NUM_INT_REG         9   // Number of interrupt registers i.e. INT1-9
#define BCM59040_NUM_IRQ             (BCM59040_NUM_INT_REG * 8) // 9 bytewide interrupt bit registers 

#define BCM59040_MAX_GPIO            5
#define BCM59040_REG_LDO_SR_TOTAL    6
#define BCM59040_REG_LDO_SR_START_INDEX  BCM59040_REG_LDO1CTRL


// Bitwise defines
//
// HOSTACT
#define BCM59040_HOSTACT_WDT_CLR     0x01
#define BCM59040_HOSTACT_WDT_ON      0x02
#define BCM59040_HOSTACT_HOSTDICOFF  0x04
#define BCM59040_HOSTACT_AONCLK32ON  0x08   // AONCLK32 will be always on (1) or on during HOSTON (0)

// ENV1
#define BCM59040_ENV1_UBMBC          0x01   // Usb voltage higher than MB voltage
#define BCM59040_ENV1_CGPD           0x02
#define BCM59040_ENV1_UBPD           0x04
#define BCM59040_ENV1_MBWV           0x08
#define BCM59040_ENV1_WALLOV         0x10   // WAC input over voltage
#define BCM59040_ENV1_LOWBAT         0x20
#define BCM59040_ENV1_BBATUVB        0x40   // low backup battery level
#define BCM59040_ENV1_CGMBC          0x80   // WAC voltage higher than MB voltage
// ENV2
#define BCM59040_ENV2_PONKEYB        0x01
#define BCM59040_ENV2_CHIP_TOOWARM   0x02   
#define BCM59040_ENV2_CHIP_TOOHOT    0x04
#define BCM59040_ENV2_SYSUVB         0x08   // system under voltage
#define BCM59040_ENV2_USBOV          0x10   // USB input over voltage
#define BCM59040_ENV2_SYSWV          0x20   // system working voltage                                        
#define BCM59040_ENV2_HIBER_ST       0x40   // hibernate mode
#define BCM59040_ENV2_FCMBC          0x80   // fast charging enabled
// ENV3
#define BCM59040_ENV3_MBPD           0x01
#define BCM59040_ENV3_MBTEMPLOW      0x02
#define BCM59040_ENV3_MBTEMPHIGH     0x04
#ifdef PMU_59040_B0
#define BCM59040_ENV3_SYN_PC1        0x08
#define BCM59040_ENV3_SYN_PC2        0x10
#endif
#define BCM59040_ENV3_FGC            0x20   // FG comparator
#define BCM59040_ENV3_VERYLOWBAT     0x40   // very low battery
#define BCM59040_ENV3_NOBAT          0x80   // no main battery
// ENV4
#define BCM59040_ENV4_VBUS_VALID     0x01
#define BCM59040_ENV4_A_SESS_VALID   0x02
#define BCM59040_ENV4_B_SESS_END     0x04
#define BCM59040_ENV4_ID_IN_MASK     0x38
#define BCM59040_ENV4_ID_IN_SHIFT    3
#define BCM59040_ENV4_OFFVBUSb       0x40
#define BCM59040_ENV4_OTGSHDNb       0x80
// ENV5
#define BCM59040_ENV5_CGRM           0x01   // charger removal
#define BCM59040_ENV5_CHGCVON        0x02   // MBC in CV mode
#define BCM59040_ENV5_VSROVERI       0x04   // VSR over current
#define BCM59040_ENV5_VSROVERV       0x08   // VSR over voltage
#define BCM59040_ENV5_VCHGROK        0x10   // VCHGR differential voltage qualification
#define BCM59040_ENV5_MBC_ERR        0x20   // MBC error
#define BCM59040_ENV5_MBMC           0x40   // MB maintenance charger
#define BCM59040_ENV5_MBOV           0x80   // MB over voltage
// ENV6
#define BCM59040_ENV6_PWR_WAKEUP     0x01   // wakeup from hibernate mode
#define BCM59040_ENV6_NTCRM_SAVE     0x02   // main battery has been removed
#define BCM59040_ENV6_PWRUP_SAVE     0x04   // entered PWRUP from HOSTON or MBRDY 
#ifdef PMU_59040_B0
#define BCM59040_ENV6_CHGDET_IN      0x08   // The status on the pin of CHGDET
#define BCM59040_ENV6_P_UBPD_ENV     0x10   // USB presence detection (debounce for comparator of UBPD). 
#define BCM59040_ENV6_P_CHG_EN       0x20   // WAC is able to charge
#define BCM59040_ENV6_P_USB_EN       0x40   // USBC is able to charge
#endif

// INT1         R&C  interrupt [6:0], [7] reserved
#define BCM59040_INT1_PONKEYR          0x01U
#define BCM59040_INT1_PONKEYF          0x02U
#define BCM59040_INT1_PONKEYH          0x04U
#define BCM59040_INT1_PONKEYBHD        0x08U
#define BCM59040_INT1_RESTARTH         0x10U
#define BCM59040_INT1_HBINT            0x20U
#define BCM59040_INT1_PMUTOOWARM       0x40U
#ifdef PMU_59040_B0
#define BCM59040_INT1_RESTARTON        0x80U  // only for 59040 B0
#endif

//INT2          R&C  interrupt [7:0]   
#define BCM59040_INT2_CHGINS       0x01U
#define BCM59040_INT2_CHGRM        0x02U
#define BCM59040_INT2_CHGOVERV     0x04U
#define BCM59040_INT2_EOC          0x08U
#define BCM59040_INT2_USBINS       0x10U
#define BCM59040_INT2_USBRM        0x20U
#define BCM59040_INT2_USBOVERV     0x40U
#define BCM59040_INT2_CHGDET       0x80U

//INT3          R&C  interrupt [7:0]
#define BCM59040_INT3_VSROVERV     0x01U
#define BCM59040_INT3_VSROVERI     0x02U
#define BCM59040_INT3_VCHGRNOTOK   0x04U
#ifdef PMU_59040_B0
#define BCM59040_INT3_CHG_WDT_ALARM 0x08U
#else
#define BCM59040_INT3_VCHGRCLSP    0x08U
#endif
#define BCM59040_INT3_VBUSLOWBND   0x10U
#define BCM59040_INT3_CHGERRDIS    0x20U
#define BCM59040_INT3_CHGWDTEXP    0x40U
#define BCM59040_INT3_IDOVERI      0x80U

//INT4          R&C  interrupt [7:0]
#define BCM59040_INT4_LDO1OVRI    0x01U
#define BCM59040_INT4_LDO2OVRI    0x02U
#define BCM59040_INT4_LDO3OVRI    0x04U
#define BCM59040_INT4_LDO4OVRI    0x08U
#define BCM59040_INT4_LDO5OVRI    0x10U
#define BCM59040_INT4_LDO6OVRI    0x20U
#define BCM59040_INT4_BBLOW       0x40U
#define BCM59040_INT4_FGC         0x80U

// INT5         R&C  interrupt [6:0]
#define BCM59040_INT5_IOSROVRI     0x01U
#define BCM59040_INT5_CSROVRI      0x02U
#define BCM59040_INT5_IOSROVRV     0x04U
#define BCM59040_INT5_CSROVRV      0x08U
#define BCM59040_INT5_RTCADJ       0x10U
#define BCM59040_INT5_RTC1S        0x20U
#define BCM59040_INT5_RTC60S       0x40U
#define BCM59040_INT5_RTCA1        0X80U

// INT6         R&C  interrupt [4:0]
#define BCM59040_INT6_MBTEMPFAULT          0x01U
#define BCM59040_INT6_MBTEMPLOW            0x02U
#define BCM59040_INT6_MBTEMPHIGH           0x04U
#define BCM59040_INT6_MBRM                 0x08U
#define BCM59040_INT6_MBOV                 0x10U
#define BCM59040_INT6_BATINS               0x20U
#define BCM59040_INT6_LOWBAT               0x40U 
#define BCM59040_INT6_VERYLOWBAT           0x80U

// INT7         R&C  interrupt [7:0]
#define BCM59040_INT7_VBUS_VALID_F        0x01U
#define BCM59040_INT7_A_SESSVALID_F       0x02U
#define BCM59040_INT7_B_SESSEND_F         0x04U
#define BCM59040_INT7_ID_INSRT            0x08U
#define BCM59040_INT7_VBUS_VALID_R        0x10U
#define BCM59040_INT7_A_SESSVALID_R       0x20U
#define BCM59040_INT7_B_SESSEND_R         0x40U
#define BCM59040_INT7_ID_RMV              0x80U

// INT8         R&C  interrupt [7:0]
#define BCM59040_INT8_SARCONVRDY0     0x01U
#define BCM59040_INT8_SARCONVRDY1     0x02U
#define BCM59040_INT8_SARCONVRDY2     0x04U
#define BCM59040_INT8_SARCONVRDY3     0x08U
#define BCM59040_INT8_SARCONVRDY4     0x10U
#define BCM59040_INT8_SARCONVRDY5     0x20U  
#define BCM59040_INT8_SARCONVRDY6     0x40U
#define BCM59040_INT8_SARCONVRDY7     0x80U

// INT9         R&C  interrupt [3:0]
#define BCM59040_INT9_SARCONVRDY8     0x01U
#define BCM59040_INT9_SARCONVRDY9     0x02U
#define BCM59040_INT9_SARCONVEND      0x04U
#define BCM59040_INT9_SARCONTCONVFAIL 0x08U
#define BCM59040_INT9_SARASYNCONVOFF  0x10U
#define BCM59040_INT9_SARASYNREQFAIL  0x20U
#define BCM59040_INT9_RESUME_VBUS     0x40U
#define BCM59040_INT9_ID_CHNG         0x80U

// Fuel Gauge
#define BCM59040_FGCTRL1_FGHOSTEN        0x01
#define BCM59040_FGCTRL1_FGSTPCHOP       0x02 // if 1, stop chopper 
#define BCM59040_FGCTRL3_FGTRIM          0x01
#define BCM59040_FGCTRL3_FGCAL           0x02 // calibrate FG offset using fast output
#define BCM59040_FGCTRL3_FGRESET         0x04 // reset FG output, accumulator, counter
#define BCM59040_FGCTRL3_FGFRZREAD       0x08
#define BCM59040_FGCTRL3_FGFORCECAL      0x10 // force the ADC input to be shorted, offset will not be subtracted
#define BCM59040_FGCTRL3_FGSYNCMODE      0x20    // 0 = continuous mode; 1 = synchronous mode
#define BCM59040_FGCTRL3_FGMODON         0x40 // modulator on when FG is freezed; for sync mode only
#define BCM59040_FGCTRL3_LONGCAL         0x80 // calibrate FG offset using slow output

#define BCM59040_FGOFFSET_MASK           0x3FFF  // 14-bit, including sign bit
#define BCM59040_FGOFFSET_SIGN_BIT       0x2000  // bit 13 is sign bit                                                
#define BCM59040_FGSAMPLE_MASK           0x3FFF  // 14-bit, including sign bit
#define BCM59040_FGSAMPLE_SIGN_BIT       0x2000  // bit 13 is sign bit
#define BCM59040_FGACCUM_MASK            0x1FFFFFF // 25-bit 
#define BCM59040_FGACCUM_SIGN_BIT        0x2000000 // bit 25 is sign bit                                           
#define BCM59040_FGCOUNT_MASK            0xFFF   // 12-bit unsigned, no sign bit
#define BCM59040_FGACCM4_FGRDVALID       0x80    // FGRDVALID bit: FG accum and counter data read is valid

// GPIO
#define BCM59040_NUM_OF_GPIOS			5
#define BCM59040_GPIOCTRLX_GPIODIR_HIZ   0x00
#define BCM59040_GPIOCTRLX_GPIODIR_OUT   0x01
#define BCM59040_GPIOCTRLX_GPIODIR_IN    0x02
#define BCM59040_GPIOCTRLX_GPIODIR_MASK  0x03
#define BCM59040_GPIOCTRLX_GPIO_DATA     0x04    // for read or write, depending on direction
#define BCM59040_GPIOCTRLX_GPIO_MODE_MASK  0x38      
#define BCM59040_GPIOX_MODE_NORMAL   0x00
#ifdef PMU_59040_B0
#define BCM59040_GPIOCTRL2_MODE_ADC_DATARDY 0x08   // output
#endif
#define BCM59040_GPIOCTRL3_MODE_SYSUVBb  0x08   // output
#define BCM59040_GPIOCTRL3_MODE_MBRMb    0x10   // output
#ifdef PMU_59040_B0
#define BCM59040_GPIOCTRL3_MODE_ADC_SER_DATA   0x18   // output
#endif
#define BCM59040_GPIOCTRL3_MODE_MBATOV   0x20   // output
#define BCM59040_GPIOCTRL3_MODE_BBATUVb  0x28    // output
#define BCM59040_GPIOCTRL3_MODE_VA_SESS_VLD 0x30 // output
//#define BCM59040_GPIOCTRL4_MODE_NORMAL   0x00
#define BCM59040_GPIOCTRL4_MODE_ADCSYNCB 0x08   // input
#define BCM59040_GPIOCTRL4_MODE_VWALLOV  0x20   // output
#define BCM59040_GPIOCTRL4_MODE_SYSUVb   0x28   // output
#define BCM59040_GPIOCTRL4_MODE_VA_VBUS_VLD  0x30 // output
//#define BCM59040_GPIOCTRL5_MODE_NORMAL   0x00
#define BCM59040_GPIOCTRL5_MODE_ADCTCLK  0x08   // input
#define BCM59040_GPIOCTRL5_MODE_VUSBOV   0x20   // output
#define BCM59040_GPIOCTRL5_MODE_ADCOK    0x28   // output
#define BCM59040_GPIOCTRL5_MODE_VB_SESS_END   0x30   // output
                                                 
// PWM and LED
#define BCM59040_PWMLEDCTRL1_AN_CHG_EN1   0x01  // LED1 on/off in automomous charge (HOSTOFF)
#define BCM59040_PWMLEDCTRL1_AN_CHG_EN2   0x02  // LED2 on/off in automomous charge (HOSTOFF)
#define BCM59040_PWMLEDCTRL1_AN_CHG_EN3   0x04  // LED3 on/off in automomous charge (HOSTOFF)
#ifdef PMU_59040_B0
#define BCM59040_PWMLEDCTRL1_PWMLED_SW_CTRL 0x40  // SW control of pwmled2 and 3
#endif
#define BCM59040_PWMLEDCTRL1_PWMLED_PDN   0x80  // power down pwmled analog section
                                                
#define BCM59040_PLD1CTRL1_PWMLED1ON    0x01  //
#define BCM59040_PLD1CTRL1_PWMLED1SEL   0x02  // 
#define BCM59040_PLD1CTRL1_PWMDIV1_MASK 0x0C  
#define BCM59040_PLD1CTRL1_LCTRL1_MASK  0xF0  // current control
#define BCM59040_PLD1CTRL2_LEDRPT1_MASK 0x07  // LED pattern repeat period
#define BCM59040_PLD1CTRL2_LEDPAT1_MASK 0x38  // LED pattern                                            
#define BCM59040_PLD1CTRL2_LEDHP1_MASK  0x3F  // PWM High period
#define BCM59040_PLD1CTRL3_PWMLP1_MASK  0x3F  // PWM Low period

#define BCM59040_PLD2CTRL1_PWMLED2ON    0x01  //
#define BCM59040_PLD2CTRL1_PWMLED2SEL   0x02  // 
#define BCM59040_PLD2CTRL1_PWMDIV2_MASK 0x0C  
#define BCM59040_PLD2CTRL1_LCTRL2_MASK  0x30  // current control
#define BCM59040_PLD2CTRL2_LEDRPT2_MASK 0x07  // LED pattern repeat period
#define BCM59040_PLD2CTRL2_LEDPAT2_MASK 0x38  // LED pattern                                            
#define BCM59040_PLD2CTRL2_LEDHP2_MASK  0x3F  // PWM High period
#define BCM59040_PLD2CTRL3_PWMLP2_MASK  0x3F  // PWM Low period

#define BCM59040_PLD3CTRL1_PWMLED3ON    0x01  //
#define BCM59040_PLD3CTRL1_PWMLED3SEL   0x02  // 
#define BCM59040_PLD3CTRL1_PWMDIV3_MASK 0x0C  
#define BCM59040_PLD3CTRL1_LCTRL3_MASK  0x30  // current control
#define BCM59040_PLD3CTRL2_LEDRPT3_MASK 0x07  // LED pattern repeat period
#define BCM59040_PLD3CTRL2_LEDPAT3_MASK 0x38  // LED pattern                                            
#define BCM59040_PLD3CTRL2_LEDHP3_MASK  0x3F  // PWM High period
#define BCM59040_PLD3CTRL3_PWMLP3_MASK  0x3F  // PWM Low period


#define BCM59035_LEDON                  0x01 //0x03
#define BCM59035_PWMON                  0x02
#define BCM59035_SYSCLK_MASK            0x0C

#define BCM59040_SYSCLK_4               0x00    // for PWMDIV1~3 system clock
#define BCM59040_SYSCLK_16              0x04
#define BCM59040_SYSCLK_64              0x08
#define BCM59040_SYSCLK_512             0x0C

#define BCM59035_PWMLED_PDN             0x40
#define BCM59035_PWMLED1                0x02
#define BCM59035_PWMLED2                0x01

// CMPCTRL bit fields
#define BCM59040_CMPCTRL3_LOWBATCVS_MASK 0x03    // low main battery comparator threshold selector mask
    #define BCM59040_CMPCTRL3_LOWBATCVS_3_6V 0x03 //3.6V
#define BCM59040_CMPCTRL3_MBMC_HYS 0x08     //Maintenance charge comparator hysteresis
#define BCM59040_CMPCTRL4_MBMCVS_MASK 0x60 // main battery maintenance charge comparator threshold selector mask
    #define BCM59040_CMPCTRL4_MBMCVS_POS  5     // MBMCVS field starts at bit 5

// MBCCTRL bit fields
#define BCM59040_MBCCTRL1_CHGWDT_EN 0x01    // charger watch dog timer function enable
#define BCM59040_MBCCTRL1_CHGWDT_CLR 0x02    // clear watch dog timer
#define BCM59040_MBCCTRL1_CC2_RANGE_EN 0x04   // 2nd range enable for CC charging
#define BCM59040_MBCCTRL1_CHGWDT_DV_DIS 0x08 // disable CV watch dog timer
#define BCM59040_MBCCTRL2_CHGWDT_CC1_EXP_MASK 0x07 // CC charge timer setting - 1st range 
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_MASK 0xF8 // CC charge timer setting - 2nd range
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_DIS  (0x0<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_1MIN (0x1<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_2MIN (0x2<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_3MIN (0x3<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_4MIN (0x4<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_5MIN (0x5<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_6MIN (0x6<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_7MIN (0x7<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_8MIN (0x8<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_9MIN (0x9<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_10MIN (0xA<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_11MIN (0xB<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_12MIN (0xC<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_13MIN (0xD<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_14MIN (0xE<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_15MIN (0xF<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_16MIN (0x10<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_17MIN (0x11<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_18MIN (0x12<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_19MIN (0x13<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_20MIN (0x14<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_21MIN (0x15<<3)
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_22MIN (0x16<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_23MIN (0x17<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_24MIN (0x18<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_25MIN (0x19<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_26MIN (0x1A<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_27MIN (0x1B<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_28MIN (0x1C<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_29MIN (0x1D<<3)  
#define BCM59040_MBCCTRL2_CHGWDT_CC2_EXP_30MIN (0x1E<<3)  

#define BCM59040_MBCCTRL3_MBCHOSTEN 0x01    // enable main battery charger
#define BCM59040_MBCCTRL3_VWAGR_FC2_EN 0x02 // wall charger fast charge 2 enable
#define BCM59040_MBCCTRL3_VUBGR_FC2_EN 0x04 // usb charger fast charge 2 enable
#define BCM59040_MBCCTRL3_PAUSE 0x08    // pause charging enable
#define BCM59040_MBCCTRL3_MAINGCHRG 0x10    // maintenance charge enable
#define BCM59040_MBCCTRL3_WAC_FC_OPTION 0x20    // WAC regulates to TC2 level (if set) or stops (if clear)
#define BCM59040_MBCCTRL3_USB0_FC_OPTION 0x40   // USB charger regulates to TC2 level (if set) or stops when CHGDET = 0
#define BCM59040_MBCCTRL3_USB1_FC_OPTION 0x80   // USB charger regulates to TC2 level (if set) or stops when CHGDET = 1
#define BCM59040_MBCCTRL4_FC1_CV_MASK 0x0F  // FC1 regulated voltage selection
#define BCM59040_MBCCTRL4_FC1_CV_3_75V   0x0 // value for 3.75 V
#define BCM59040_MBCCTRL4_FC1_CV_3_80V   0x1 // value for 3.80 V
#define BCM59040_MBCCTRL4_FC1_CV_3_90V   0x2 // value for 3.90 V
#define BCM59040_MBCCTRL4_FC1_CV_4_00V   0x3 // value for 4.00 V
#define BCM59040_MBCCTRL4_FC1_CV_4_05V   0x4 // value for 4.05 V
#define BCM59040_MBCCTRL4_FC1_CV_4_10V   0x5 // value for 4.10 V
#define BCM59040_MBCCTRL4_FC1_CV_4_15V   0x6 // value for 4.15 V
#define BCM59040_MBCCTRL4_FC1_CV_4_20V   0x7 // value for 4.20 V
#define BCM59040_MBCCTRL4_FC1_CV_4_25V   0x8 // value for 4.25 V
#define BCM59040_MBCCTRL4_FC1_CV_4_30V   0x9 // value for 4.30 V
#define BCM59040_MBCCTRL4_FC1_CV_4_375V  0xA // value for 4.375 V
#define BCM59040_MBCCTRL4_FC2_CV_MASK 0xF0  // FC2 regulated voltage selection
#define BCM59040_MBCCTRL4_FC2_CV_3_75V   (0x0<<4) // value for 3.75 V
#define BCM59040_MBCCTRL4_FC2_CV_3_80V   (0x1<<4) // value for 3.80 V
#define BCM59040_MBCCTRL4_FC2_CV_3_90V   (0x2<<4) // value for 3.90 V
#define BCM59040_MBCCTRL4_FC2_CV_4_00V   (0x3<<4) // value for 4.00 V
#define BCM59040_MBCCTRL4_FC2_CV_4_05V   (0x4<<4) // value for 4.05 V
#define BCM59040_MBCCTRL4_FC2_CV_4_10V   (0x5<<4) // value for 4.10 V
#define BCM59040_MBCCTRL4_FC2_CV_4_15V   (0x6<<4) // value for 4.15 V
#define BCM59040_MBCCTRL4_FC2_CV_4_20V   (0x7<<4) // value for 4.20 V
#define BCM59040_MBCCTRL4_FC2_CV_4_25V   (0x8<<4) // value for 4.25 V
#define BCM59040_MBCCTRL4_FC2_CV_4_30V   (0x9<<4) // value for 4.30 V
#define BCM59040_MBCCTRL4_FC2_CV_4_375V  (0xA<<4) // value for 4.375 V                                            
#define BCM59040_MBCCTRL5_QC_CC_MASK 0x0F   // qualication CC charge current
#define BCM59040_MBCCTRL5_QC_CC_0MA 0x0 // value for 0 ma
#define BCM59040_MBCCTRL5_QC_CC_10MA 0x1 // value for 10 ma
#define BCM59040_MBCCTRL5_QC_CC_20MA 0x2 // value for 20 ma
#define BCM59040_MBCCTRL5_QC_CC_30MA 0x3 // value for 30 ma
#define BCM59040_MBCCTRL5_QC_CC_40MA 0x4 // value for 40 ma
#define BCM59040_MBCCTRL5_QC_CC_50MA 0x5 // value for 50 ma
#define BCM59040_MBCCTRL5_QC_CC_60MA 0x6 // value for 60 ma
#define BCM59040_MBCCTRL5_QC_CC_70MA 0x7 // value for 70 ma
#define BCM59040_MBCCTRL5_QC_CC_80MA 0x8 // value for 80 ma
#define BCM59040_MBCCTRL5_QC_CC_90MA 0x9 // value for 90 ma
#define BCM59040_MBCCTRL5_QC_CC_100MA 0xA // value for 100 ma
#define BCM59040_MBCCTRL5_QC_CC_110MA 0xB // value for 110 ma
#define BCM59040_MBCCTRL5_QC_CC_120MA 0xC // value for 120 ma
#define BCM59040_MBCCTRL5_QC_CC_130MA 0xD // value for 130 ma
#define BCM59040_MBCCTRL5_QC_CC_140MA 0xE // value for 140 ma
#define BCM59040_MBCCTRL5_QC_CC_150MA 0xF // value for 150 ma
                                             // 
#define BCM59040_MBCCTRL6_FC_CC_MASK 0x1F   // fast charge, CC charging current 
#define BCM59040_MBCCTRL6_FC_CC_0MA 0x0 // value for (0 ma, 0 ma)
#define BCM59040_MBCCTRL6_FC_CC_10MA 0x1 // value for (10 ma)
#define BCM59040_MBCCTRL6_FC_CC_20MA 0x2 // value for (20 ma)
#define BCM59040_MBCCTRL6_FC_CC_30MA 0x3 // value for (30 ma)
#define BCM59040_MBCCTRL6_FC_CC_40MA 0x4 // value for (40 ma)
#define BCM59040_MBCCTRL6_FC_CC_50MA 0x5 // value for (50 ma)
#define BCM59040_MBCCTRL6_FC_CC_60MA 0x6 // value for (60 ma)
#define BCM59040_MBCCTRL6_FC_CC_70MA 0x7 // value for (70 ma)
#define BCM59040_MBCCTRL6_FC_CC_80MA 0x8 // value for (80 ma)
#define BCM59040_MBCCTRL6_FC_CC_90MA 0x9 // value for (90 ma)
#define BCM59040_MBCCTRL6_FC_CC_100MA 0xA // value for (100 ma)
#define BCM59040_MBCCTRL6_FC_CC_110MA 0xB // value for (110 ma)
#define BCM59040_MBCCTRL6_FC_CC_120MA 0xC // value for (120 ma)
#define BCM59040_MBCCTRL6_FC_CC_130MA 0xD // value for (130 ma)
#define BCM59040_MBCCTRL6_FC_CC_140MA 0xE // value for (140 ma)
#define BCM59040_MBCCTRL6_FC_CC_150MA 0xF // value for (150 ma)

//#define BCM59040_MBCCTRL6_FC_CC_0MA 0x10 // value for (0 ma)
//#define BCM59040_MBCCTRL6_FC_CC_100MA 0x11 // value for (100 ma)
#define BCM59040_MBCCTRL6_FC_CC_200MA 0x12 // value for (200 ma)
#define BCM59040_MBCCTRL6_FC_CC_300MA 0x13 // value for (300 ma)
#define BCM59040_MBCCTRL6_FC_CC_400MA 0x14 // value for (400 ma)
#define BCM59040_MBCCTRL6_FC_CC_500MA 0x15 // value for (500 ma)
#define BCM59040_MBCCTRL6_FC_CC_600MA 0x16 // value for (600 ma)
#define BCM59040_MBCCTRL6_FC_CC_700MA 0x17 // value for (700 ma)
#define BCM59040_MBCCTRL6_FC_CC_800MA 0x18 // value for (800 ma)
#define BCM59040_MBCCTRL6_FC_CC_900MA 0x19 // value for (900 ma)
#define BCM59040_MBCCTRL6_FC_CC_1000MA 0x1A // value for (1000 ma)
#define BCM59040_MBCCTRL6_FC_CC_1100MA 0x1B // value for (1100 ma)
#define BCM59040_MBCCTRL6_FC_CC_1200MA 0x1C // value for (1200 ma)
#define BCM59040_MBCCTRL6_FC_CC_1300MA 0x1D // value for (1300 ma)
#define BCM59040_MBCCTRL6_FC_CC_1400MA 0x1E // value for (1400 ma)
#define BCM59040_MBCCTRL6_FC_CC_1500MA 0x1F // value for (1500 ma)                                            
#define BCM59040_MBCCTRL7_EOC_MASK  0x0F    // EOC current
#define BCM59040_MBCCTRL7_50MA   0x0
#define BCM59040_MBCCTRL7_200MA   0xF
#define BCM59040_MBCCTRL8_USB_DET_LDO_EN    0x01    // USB detection LDO on
#define BCM59040_MBCCTRL8_ADP_PRI   0x08    // ADP_PRI bit (1 = wall charger)
#define BCM59040_MBCCTRL8_SYS_TYP   0x20    // 1 = system can be boot-up with current > 100 mA
#define BCM59040_MBCCTRL8_IDDIODE_CTRL_MASK 0xC0    // ideal diode control
#define BCM59040_MBCCTRL8_IDDIODE_NORMAL_MODE 0xC0    // ideal diode control

#define BCM59040_MBCCTRL9_USB_ILIM_MASK 0x03  // VSR current limit when USB charger is applied
#define BCM59040_MBCCTRL9_USB_ILIMIT_100MA 0x00 // 100 mA (1X mode)
#define BCM59040_MBCCTRL9_USB_ILIMIT_500MA 0x01 // 500 mA (5x mode)
#define BCM59040_MBCCTRL9_USB_ILIMIT_900MA 0x02 // 900 mA (9x mode)
#define BCM59040_MBCCTRL9_USB_ILIMIT_1200MA 0x03 // 1200 mA (12x mode)                                                   
#define BCM59040_MBCCTRL9_WALL_ILIM_MASK 0x0C  // VSR current limit when Wall charger is applied
#define BCM59040_MBCCTRL9_WALL_ILIM_SHIFT 2    // bit position for this field
#define BCM59040_MBCCTRL9_WALL_ILIMIT_100MA 0x00 // 100 mA (1X mode)
#define BCM59040_MBCCTRL9_WALL_ILIMIT_500MA 0x04 // 500 mA (5x mode)
#define BCM59040_MBCCTRL9_WALL_ILIMIT_900MA 0x08 // 900 mA (9x mode)
#define BCM59040_MBCCTRL9_WALL_ILIMIT_1200MA 0x0C // 1200 mA (12x mode) 
#define BCM59040_MBCCTRL9_DIS_CG_UBMBC 0x10 // disable the conditionof CGMBC and UBMBC from CGPD (if set)
#define BCM59040_MBCCTRL9_CHGTYP_MASK 0x60  // charger type
#ifdef PMU_59040_B0
#define BCM59040_MBCCTRL9_CHGTYP_OFF 0x00 // bcd detector is off
#define BCM59040_MBCCTRL9_CHGTYP_CHP 0x40 // charging host port
#define BCM59040_MBCCTRL9_CHGTYP_DC  0x60 // dedicated charger
#define BCM59040_MBCCTRL9_CHGTYP_SHP 0x20 // standard host port
#define BCM59040_MBCCTRL9_USB_VSR_EN 0x80 // USB VSR enable
#else
#define BCM59040_MBCCTRL9_CHGTYP_CHP 0x00 // charging host port
#define BCM59040_MBCCTRL9_CHGTYP_DC 0x20 // dedicated charger
#define BCM59040_MBCCTRL9_CHGTYP_SHP 0x60 // standard host port
#endif

#define BCM59040_CNPDBG2_CHGDET_FLAG 0x10  // charger type

#define BCM59040_MBCCTRL10_DIS_BTEMP_FROM_IDCTRL 0x01    // 1 = ideal diode is not affected                                                         // by the condition of temp info                                                         
#define BCM59040_MBCCTRL10_VSRHOSTEN 0x02    // For B0 and later, this bit only controls Wall
#define BCM59040_MBCCTRL10_DIS_AUTO_RESUME 0x04  // 1 = disable auto resume mode
#define BCM59040_MBCCTRL10_DIS_PWRUP_FROM_IDCTRL 0x08 
#define BCM59040_MBCCTRL10_BTEMP_FAULT_CNT 0x10    // max number of times main battery temp can 
                                                   // too high/low
// BBCCTRL bit fields
#define BCM59040_BBCCTRL_BBCHOSTEN   0x01    // backup battery charger enable
#define BCM59040_BBCCTRL_BBCLOWIEN   0x40    // low charging current enable
#define BCM59040_BBCCTRL_BBCCS_MASK  0x4C    // mask for BBCCS field
#define BCM59040_BBCCTRL_BBCVS_MASK  0x30    // mask for BBCVS field
#define BCM59040_BBCCTRL_BBCRS_MASK  0x82    // mask for BBCRS field
#define BCM59040_BBCCTRL_BBCRS_P5K   0x00    // bit value for 1K resistor                                        
#define BCM59040_BBCCTRL_BBCRS_1K    0x02    // bit value for 1K resistor                                       
#define BCM59040_BBCCTRL_BBCRS_2K    0x80    // bit value for 1K resistor
#define BCM59040_BBCCTRL_BBCRS_4K    0x82    // bit value for 1K resistor

// OTG bit fields
#define BCM59040_OTGCTRL1_FLOATLEGACY  0x04 
#define BCM59040_OTGCTRL1_OTGSHUTDOWN  0x80
#define BCM59040_OTGCTRL2_HWVSSW       0x10
#define BCM59040_OTGCTRL2_VA_SESS_EN   0x40
#define BCM59040_ENV4_A_DEV         0
#define BCM59040_ENV4_B_DEV_LEGACY  0x08
#define BCM59040_ENV4_RID_A         0x18
#define BCM59040_ENV4_RID_B         0x20
#define BCM59040_ENV4_RID_C         0x28
#define BCM59040_ENV4_B_DEV_FLOAT   0x38

// PONKEY bit fields
#ifdef PMU_59040_B0
#define BCM59040_PONKEYBCNTRL1_OTP_MASK        0xFF
#else
#define BCM59040_PONKEYBCNTRL1_OTP_MASK        0x87
#endif
#define BCM59040_PONKEYBCNTRL1_ONHOLD_MASK     0x07
#define BCM59040_PONKEYBCNTRL1_PONKEYBRF_MASK  0x38
#define BCM59040_PONKEYBCNTRL1_KEYLOCK         0x40    // bit value for keylock bit
#define BCM59040_PONKEYBCNTRL1_ONOFF           0x80    // on off behavior select
#define BCM59040_PONKEYBCNTRL2_OFFHOLD_MASK    0x07 // power off hold debounce
#define BCM59040_PONKEYBCNTRL2_PONKEYBDEL_MASK 0x38  // Ponkeyb shutdown delay
#define BCM59040_RSTRTCNTRL_OTP_MASK           0x80
#define BCM59040_RSTRTCNTRL_RESTARTONDLY_MASK  0x03 // restart delay 
#define BCM59040_RSTRTCNTRL_RESTARTHOLD_15S    0x04   // 1 = 15 sec; 0 = 10 sec
#define BCM59040_RSTRTCNTRL_RESTARTEN          0x80  // restart enable bit

// ADC bit fields
#define BCM59040_ADCCTRL6_GSM_DEBOUNCE_EN    0x40
#define BCM59040_ADCCTRL6_RESET_COUNT_MASK   0x30
#define BCM59040_ADCCTRL6_SS_MASK_EN         0x80
#define BCM59040_ADCCTRL7_CHAN_SNAPSHOT_MASK 0x0F
#define BCM59040_ADCCTRL7_I2CSS_EN           0x10

// power mode bit fields
#define BCM59040_PMCTRL_PM0_MASK 0x03
#define BCM59040_PMCTRL_PM1_MASK 0X0C
#define BCM59040_PMCTRL_PM2_MASK 0X30
#define BCM59040_PMCTRL_PM3_MASK 0XC0
#define BCM59040_HBCTRL_PC_PIN_EN 0x01
#define BCM59040_HBCTRL_HB_EN    0x02
#define BCM59040_HBCTRL_HBMODE_PM0     0x00
#define BCM59040_HBCTRL_HBMODE_PM1     0x04
#define BCM59040_HBCTRL_HBMODE_PM2     0x08
#define BCM59040_HBCTRL_HBMODE_PM3     0x0C
#define BCM59040_HBCTRL_WAKEMODE_PM0   0x00
#define BCM59040_HBCTRL_WAKEMODE_PM1   0x10
#define BCM59040_HBCTRL_WAKEMODE_PM2   0x20
#define BCM59040_HBCTRL_WAKEMODE_PM3   0x30
#define BCM59040_HBCTRL_PC_I2C_MASK    0xC0
#define BCM59040_HBCTRL_PC2_PC1_00     0x00
#define BCM59040_HBCTRL_PC2_PC1_01     0x40
#define BCM59040_HBCTRL_PC2_PC1_10     0x80
#define BCM59040_HBCTRL_PC2_PC1_11     0xC0
#define BCM59040_CLKHALCTRL_HOSTRST_EN_PM0    0x01
#define BCM59040_CLKHALCTRL_HOSTRST_EN_PM1    0x04
#define BCM59040_CLKHALCTRL_HOSTRST_EN_PM2    0x10
#define BCM59040_CLKHALCTRL_HOSTRST_EN_PM3    0x40
#define BCM59040_CLKHALCTRL_CLK_HALT_EN_PM0   0x02
#define BCM59040_CLKHALCTRL_CLK_HALT_EN_PM1   0x08
#define BCM59040_CLKHALCTRL_CLK_HALT_EN_PM2   0x20
#define BCM59040_CLKHALCTRL_CLK_HALT_EN_PM3   0x80

// csr and iosr bit fields
#define BCM59040_CSRCTRL1_DVS_ENABLE          0x20 // DVS Enable bit
#define BCM59040_CSR_VOUT_MASK                0x1F
#define BCM59040_IOSRCTRL2_VOUT_MASK          0x1F

// OTG bit fields
#define BCM59040_OTGCTRL2_HWVSSW_MASK         0x10

//******************************************************************************
// Typedefs 
//******************************************************************************
//CON_TYPE related
typedef enum {
    USB_WALL_ONECON,   // type 1 or 2: coming thru one connector
    STANDARD_USB_WALL_TWOCON      // type 3: coming thru two connectors - dual connector
} Connector_Type_t;

//adaptor priority related
typedef enum {
    ADAPTOR_PRI_USB,                // usb has priority
    ADAPTOR_PRI_WALL                // wall has priority
} Adaptor_Pri_t;

//sys_typ System type related
typedef enum {
    NORMAL_CHARGING_NO_BATT,             // normal charging without battery
    STOP_CHARGING_NO_BATT                // stop charging and go to PWRUP without battery
} System_Type_t;

// LED Typedef enums ------------------------------------------------------------
// ** FixMe ** Peter Feng's team takes care of this
typedef enum{
    PMULED_Cyc_0p4s = 0x00,             ///< Add comments here
    PMULED_Cyc_1s   = 0x01,
    PMULED_Cyc_1p2s = 0x02,
    PMULED_Cyc_2s   = 0x03,
    PMULED_Cyc_2p6s = 0x04,
    PMULED_Cyc_4s   = 0x05,
    PMULED_Cyc_6s   = 0x06,
    PMULED_Cyc_8s   = 0x07
} PMULedCycle_t;

typedef enum{
    PMULED_Patt_On50msOff                   = 0x00,      ///< Add comments here
    PMULED_Patt_On100msOff                  = 0x08,
    PMULED_Patt_On200msOff                  = 0x10,
    PMULED_Patt_On500msOff                  = 0x18,
    PMULED_Patt_On50msOff50msOn50msOff      = 0x20,
    PMULED_Patt_On100msOff100msOn100msOff   = 0x28,
    PMULED_Patt_On200msOff200msOn200msOff   = 0x30,
    PMULED_Patt_On                          = 0x38
} PMULedPattern_t;

#ifdef JIM
typedef enum
{
    PMULED_ID1 = BCM59035_REG_PWMLEDCTRL1,
    PMULED_ID2 = BCM59035_REG_PWMLEDCTRL6,
    PMULED_ID_MAX
} PMULedID_t;

//--- for PWM
typedef enum{
    PMUPWM_ID1 = BCM59035_REG_PWMLEDCTRL1,
    PMUPWM_ID2 = BCM59035_REG_PWMLEDCTRL6
} PMUPwmID_t;
#endif

typedef struct
{
  u8 ldo_sr[BCM59040_REG_LDO_SR_TOTAL];
} LDO_Settings_tt; //FIXME

//use operation character '|' to select muti-operation mode like
//PMULED_Oper_Save | PMULED_Oper_Standby | PMULED_Oper_Active
typedef u8   PMULedOperMode_t;
#define PMULED_Oper_Off     0x00
#define PMULED_Oper_Charger 0x40
#define PMULED_Oper_Active  0x80
                                                                    
// Main battery charger EOC current
typedef enum{
    BCM59040_EOCS_50MA,
    BCM59040_EOCS_60MA,
    BCM59040_EOCS_70MA,
    BCM59040_EOCS_80MA,
    BCM59040_EOCS_90MA,
    BCM59040_EOCS_100MA,
    BCM59040_EOCS_110MA,
    BCM59040_EOCS_120MA,
    BCM59040_EOCS_130MA,
    BCM59040_EOCS_140MA,
    BCM59040_EOCS_150MA,
    BCM59040_EOCS_160MA,
    BCM59040_EOCS_170MA,
    BCM59040_EOCS_180MA,
    BCM59040_EOCS_190MA,
    BCM59040_EOCS_200MA
} BCM59040_EOCS_t;

// Maintenance charge voltage level, in CMPCTRL4[6:5]
typedef enum{
    BCM59040_MBMCVS_3P95,
    BCM59040_MBMCVS_4P00,
    BCM59040_MBMCVS_4P05,
    BCM59040_MBMCVS_4P10
} BCM59040_MBMCVS_t;

typedef enum {
   BCM59040_FG_OFF,           // FG is off
   BCM59040_FG_ON_CONT_MODE,  // FG is on, continuous mode
   BCM59040_FG_ON_SYNC_MODE   // FG is on, sync mode
} BCM59040_FuelGauge_State_t;

typedef enum {
   BCM59040_LDO1CTRL = 0,
   BCM59040_LDO2CTRL,
   BCM59040_LDO3CTRL,
   BCM59040_LDO4CTRL,
   BCM59040_LDO5CTRL,
   BCM59040_LDO6CTRL,
   BCM59040_INVALIDLDO
} BCM59040_LDO_t;

int BCM59040_UserPONKEYLock(u8 setToOn);

/* system parameter section: This section contains system parameters that are used to keep some
   of the registers' OTP values as well as to initialize the non-OTPable fields of some PMU 
   registers */
// ---------------------------------------------------------------------------
// LDO SETTINGS FOR 59040
// ---------------------------------------------------------------------------
#define BCM59040_PMU_LDO1_VAL   0x37
#define BCM59040_PMU_LDO2_VAL   0x77  
#define BCM59040_PMU_LDO3_VAL   0x55
#define BCM59040_PMU_LDO4_VAL   0x12
#define BCM59040_PMU_LDO5_VAL   0xB0
#define BCM59040_PMU_LDO6_VAL   0xC1

// ---------------------------------------------------------------------------
// USB Rapid Charge Current SETTING FOR 59040
// ---------------------------------------------------------------------------
#define BCM59040_PMU_USB_RC_CURRENT_VAL 0x15  // 475 mA

// 59040 power mode related control registers
#define BCM59040_PMU_REG_L1PMCTRL_VAL   0x90  // PM1 all on; PM2 all LPM; PM3 all off
#define BCM59040_PMU_REG_L2PMCTRL_VAL   0x90
#define BCM59040_PMU_REG_L3PMCTRL_VAL   0x90
#define BCM59040_PMU_REG_L4PMCTRL_VAL   0x90
#define BCM59040_PMU_REG_L5PMCTRL_VAL   0x90
#define BCM59040_PMU_REG_L6PMCTRL_VAL   0x90
#define BCM59040_PMU_REG_CSRPMCTRL_VAL  0x90
#define BCM59040_PMU_REG_IOSRPMCTRL_VAL 0x90

#define BCM59040_PMU_REG_HBCTRL_VAL       0x0F    // pc_pin_en for both; HB_EN, HB_mode = PM3, wakemode = PM0, pc_i2c = PM0
#define BCM59040_PMU_REG_CLKHALTCTRL_VAL  0x22
#define BCM59040_PMU_REG_PWRCURMODE1_VAL  0x11
#define BCM59040_PMU_REG_PWRCURMODE2_VAL  0x11
#define BCM59040_PMU_REG_PM1GRPCTRL1_VAL  0x22
#define BCM59040_PMU_REG_PM1GRPCTRL2_VAL  0x22
#define BCM59040_PMU_REG_PM1GRPCTRL3_VAL  0x11
#define BCM59040_PMU_REG_PM1GRPCTRL4_VAL  0x11
#define BCM59040_PMU_REG_PM2GRPCTRL1_VAL  0x22
#define BCM59040_PMU_REG_PM2GRPCTRL2_VAL  0x22
#define BCM59040_PMU_REG_PM2GRPCTRL3_VAL  0x11
#define BCM59040_PMU_REG_PM2GRPCTRL4_VAL  0x11
#define BCM59040_PMU_REG_PM3GRPCTRL1_VAL  0x22
#define BCM59040_PMU_REG_PM3GRPCTRL2_VAL  0x22
#define BCM59040_PMU_REG_PM3GRPCTRL3_VAL  0x11
#define BCM59040_PMU_REG_PM3GRPCTRL4_VAL  0x11

//below 8 PCDVSx_VAL only used in B0, TomTom case - DVS always enabled, need to be changed for other case
#define BCM59040_PMU_REG_CSRDVSCTRL_VAL  0x11
#define BCM59040_PMU_REG_CSRPCDVS0_VAL   0x0F  //CSR init voltage for PM0, 1.20V 
#define BCM59040_PMU_REG_CSRPCDVS1_VAL   0x0F  //CSR init voltage for PM1, 1.20V
#define BCM59040_PMU_REG_CSRPCDVS2_VAL   0x0F  //CSR init voltage for PM2, 1.20V
#define BCM59040_PMU_REG_CSRPCDVS3_VAL   0x0F  //CSR init voltage for PM3, 1.20V
#define BCM59040_PMU_REG_IOSRDVSCTRL_VAL 0x11
#define BCM59040_PMU_REG_IOSRPCDVS0_VAL  0x07  //IOSR init voltage for PM0, 1.8V
#define BCM59040_PMU_REG_IOSRPCDVS1_VAL  0x07  //IOSR init voltage for PM1, 1.8V
#define BCM59040_PMU_REG_IOSRPCDVS2_VAL  0x07  //IOSR init voltage for PM2, 1.8V
#define BCM59040_PMU_REG_IOSRPCDVS3_VAL  0x07  //IOSR init voltage for PM3, 1.8V
 
// Backup battery, Power On Key, ADC settings
#define BCM59040_PMU_REG_BBCCTRL_VAL       0x53 // ok
#ifdef PMU_59040_B0
#define BCM59040_PMU_REG_PONKEYBCNTRL1_VAL 0x79 // ok OTP value is 0x79
#else
#define BCM59040_PMU_REG_PONKEYBCNTRL1_VAL 0x84 // ok OTP value is 0x81
#endif                                                
#define BCM59040_PMU_REG_PONKEYBCNTRL2_VAL 0x2F // ok power off hold debounce=5s, shutdown delay=8s
#define BCM59040_PMU_REG_RSTRTCNTRL_VAL    0x85 // ok restart debounce=15s, restart delay=200ms
#define BCM59040_PMU_REG_ADCCTRL1_VAL      0xFF // Default value
#define BCM59040_PMU_REG_ADCCTRL2_VAL      0xFF //
#define BCM59040_PMU_REG_ADCCTRL3_VAL      0xFF //
#define BCM59040_PMU_REG_ADCCTRL4_VAL      0xFF //
#define BCM59040_PMU_REG_ADCCTRL5_VAL      0xFF //
#define BCM59040_PMU_REG_ADCCTRL6_VAL      0x90 // RESET_COUNT = 3, normal mode, enable SS, no ext chnl
#define BCM59040_PMU_REG_ADCCTRL7_VAL      0x0F // Default value

// Charger Settings 
#define BCM59040_PMU_REG_MBCCTRL1_VAL      0xAD  // disable CV charger wdt (same as OTP)
#ifdef PMU_59040_B0
#define BCM59040_PMU_REG_MBCCTRL2_VAL      0xF8  // cc1&2 charger wdt = off
#define BCM59040_PMU_REG_MBCCTRL3_VAL      0x73  // enable auto maintenance charge (same as OTP)
#else
#define BCM59040_PMU_REG_MBCCTRL2_VAL      0x50  // cc1 charger wdt = off; cc2 charger wdt = 10 min
#define BCM59040_PMU_REG_MBCCTRL3_VAL      0x63  // disable auto maintenance charge (OTP disables it)
#endif
#define BCM59040_PMU_REG_MBCCTRL6_VAL      0x15  // CC charging current set to 500mA, different from OTP
#define BCM59040_PMU_REG_MBCCTRL7_VAL      0x00  // eoc current is 50 mA; same as OTP 
#define BCM59040_PMU_REG_MBCCTRL8_VAL      0xF8  // system can boot up with current > 100 mA; set bit 4 to 1 (disable)
#ifdef PMU_59040_B0
#define BCM59040_PMU_REG_MBCCTRL9_VAL      0x9F  // Change USB_llimit to 1.2A (OTP is 500 mA)
#else
#define BCM59040_PMU_REG_MBCCTRL9_VAL      0x1F  // Change USB_llimit to 1.2A (OTP is 100 mA)
#endif
#define BCM59040_PMU_REG_MBCCTRL10_VAL     0x1F  // vsr_en = 1; same as OTP

// NTC Settings
#define BCM59040_PMU_REG_NTCCTRL1_VAL      0x02  // continuous mode,MBS is on, same as OTP 

// Fuel Gauge settings
#define BCM59040_PMU_REG_FGCTRL1_VAL       0xA8  // FG off, chop mode off
#define BCM59040_PMU_REG_FGCTRL3_VAL       0x00  // FG continous mode

// GPIO setting
#define BCM59040_PMU_REG_GPIOCTRL_VAL           0x09

 
#define BCM59040_PMU_REG_ANADBG1_VAL        0x80
 
#ifdef PMU_59040_B0
#define BCM59040_PMU_REG_OTGCTRL1_VAL       0x80  // float (aca support) is chosen; same as OTP
#define BCM59040_PMU_REG_OTGCTRL2_VAL       0xEE  // use SW based otg control, enable comparator
#else  // A0 
#define BCM59040_PMU_REG_OTGCTRL1_VAL       0xC0  // float (aca support) is chosen; same as OTP
#define BCM59040_PMU_REG_OTGCTRL2_VAL       0xEA  // use SW based otg control, enable comparator (different from OTP)
#endif

#ifndef CONFIG_PMU_DEVICE_BCM59040
/* ---- Function Prototypes ---------------------------------------------- */
int bcm59040_rtc_complete_register(bcm_evt_handler_t rtcCompletion);

// @KP: added for TOMTOM_OTAVALO
int bcm59040_usb_event_register(bcm_evt_handler_t usbEventHandler);
#else

typedef enum
{
   VWALL = 0,
   VBUS,
   VSYS,
   VMBAT,
   VBBAT,
   VIDIN,
   ADC1,
   ADC2,
   ADC3,
   ADC4
} BCM_PMU_ADC_Channel_t;

typedef enum {
   BCM59040_GPO_ACTIVE_LOW  = 0,
   BCM59040_GPO_ACTIVE_HIGH
/*BCM59040_GPO_LED1_OUTPUT,
 BCM59040_GPO_LED2_OUTPUT,
 BCM59040_GPO_PWM1_OUTPUT,
 BCM59040_GPO_N_PWM1_OUTPUT,
 BCM59040_GPO_PWM2_OUTPUT,
 BCM59040_GPO_N_PWM2_OUTPUT,
 BCM59040_GPO_HIGH_IMPEDANCE */
} BCM59040_GPOutput_t;

typedef enum {
   BCM59040_GPO1 = 1,
   BCM59040_GPO2 = 2,
   BCM59040_GPO3 = 3,
   BCM59040_GPO4 = 4,
   BCM59040_GPO5 = 5
} BCM59040_GPOpin_t;

typedef enum
{
   GPIO_HIGH_IMPEDANCE = 0,
   GPIO_OUTPUT,
   GPIO_INPUT
}BCM_PMU_gpio_dir_t;

typedef struct
{
   int gpioNum;
   BCM_PMU_gpio_dir_t	gpioDir;
}BCM_PMU_gpioDir_t;

typedef struct
{
   int gpioNum;
   int gpioData;
}BCM_PMU_gpioData_t;

typedef struct
{
   int gpioNum;
   int gpioMode;
}BCM_PMU_gpioMode_t;

typedef struct
{
   BCM_PMU_gpio_dir_t gpioDir;
   int	gpioData;
   int gpioMode;
}BCM_PMU_gpioInit_t;

typedef struct
{
	int					numGPIO;
	BCM_PMU_gpioInit_t	*gpioInitTable;
} BCM_PMU_gpioPlatformData_t;

typedef struct 
{
    unsigned int pwmledNum;
    unsigned int hi_per ;
} BCM_PMU_PWM_hi_per_t ;

typedef struct 
{
    unsigned int pwmledNum;
    unsigned int lo_per ;
} BCM_PMU_PWM_lo_per_t ;

typedef struct 
{
    unsigned int pwr_ctrl ;
} BCM_PMU_PWM_pwr_ctrl_t ;

typedef enum
{
   PMU_PCF50603 = 0,
   PMU_PCF50611,
   PMU_BCM59001,
   PMU_BCM59035,
   PMU_BCM59040,
   PMU_NONE,
   PMU_NUM_CHIPS,
} BCM_PMU_Chip_t;

typedef enum
{
   PMU_Regulator_Off,
   PMU_Regulator_On,
   PMU_Regulator_Eco
} BCM_PMU_Regulator_State_t;

typedef struct 
{
    unsigned int pwmledNum;
    unsigned int pwmledOn;
    unsigned int pwmled_ctrl ;
    unsigned int pwmdiv ; // divider value. fsys/x value.    
} BCM_PMU_PWM_ctrl_t ;



#define BCM59040_REG_VINT1		0x0300 /* Virtual interrupt 1 register */
#define BCM59040_REG_VINT2		0x0301 /* Virtual interrupt 2 register */

#define BCM59040_REG_VINT1M		0x0310 /* Virtual interrupt 1 mask register */
#define BCM59040_REG_VINT2M		0x0311 /* Virtual interrupt 1 mask register */

/* It is required that the order here matches the order in bcm59040_irqs.h for
 * the related interrupts.
 */
#define BCM59040_VINT1_PONKEY_BIT	0
#define BCM59040_VINT1_INVALID_HIB_BIT	1
#define BCM59040_VINT1_PMU_TOO_WARM_BIT	2
#define BCM59040_VINT1_CHARGER_BIT	3
#define BCM59040_VINT1_REGULATOR_BIT	4
#define BCM59040_VINT1_BATT_BIT		5
#define BCM59040_VINT1_RTC_BIT		6
#define BCM59040_VINT1_RTC_ALARM_BIT	7
#define BCM59040_VINT2_USB_BIT		0
#define BCM59040_VINT2_ADC_BIT		1

#endif

#endif
