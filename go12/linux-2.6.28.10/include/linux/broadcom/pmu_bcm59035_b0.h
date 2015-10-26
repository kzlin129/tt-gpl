/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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
*  pmu_bcm59035_b0.h
*
*  PURPOSE:
*
*  This file defines the interface to the Broadcom BCM59035-B0 PMU chip.
*
*  NOTES:
*      Revisit to remove unwanted code. 
*
*****************************************************************************/

//******************************************************************************
//                          definition block
//******************************************************************************
#if !defined( PMU_BCM59035_H )
#define PMU_BCM59035_H

typedef enum 
{
   BCM59035_REGULATOR_ALDO1 = 0,     
   BCM59035_REGULATOR_ALDO2,     
   BCM59035_REGULATOR_RFLDO1,    
   BCM59035_REGULATOR_RFLDO2,    
   BCM59035_REGULATOR_HCLDO1,    
   BCM59035_REGULATOR_HCLDO2,    
   BCM59035_REGULATOR_IOLDO,     
   BCM59035_REGULATOR_MSLDO,     
   BCM59035_REGULATOR_LCLDO,    
   BCM59035_REGULATOR_LV1LDO,
   BCM59035_REGULATOR_SIMLDO,    
   BCM59035_REGULATOR_LV2LDO,    
   BCM59035_REGULATOR_MS2LDO,
   BCM59035_REGULATOR_AX1LDO,
   BCM59035_REGULATOR_AX2LDO,
   BCM59035_REGULATOR_CSR,
   BCM59035_REGULATOR_IOSR,

   BCM59035_NUM_REGULATORS
} BCM59035_regulators_t;
              
/* Charger IDs */
typedef enum 
{
   BCM59035_CHARGER_MAIN = 0,
   BCM59035_CHARGER_USB,
   BCM59035_NUM_CHARGERS
} BCM59035_chargers_t;

typedef enum
{
    ESIM_3Point0Volt = 0,
    ESIM_2Point5Volt,
    ESIM_3Point1Volt,
    ESIM_1Point8Volt,
    ESIM_Max_Voltage    
} BCM59035_sim_voltage_t;

#define BCM59035_BIT_REG_OFF        0xaa
#define BCM59035_BIT_REG_ON         0x00
#define BCM59035_BIT_REG_ECO        0x55

// I2C addresses for BCM59035
//Note: This base address is 7 bits long and this is shifted by 1-bit
//      to accomodate the read/write bit.
#define BCM59035_I2C_BASE_ADDR   0x08    // write address 00010000

#define BCM59035_BASE_W      0x10        // write address 00010000
#define BCM59035_BASE_R      0x11        // read  address 00010001

// PMU REGISTERS MAPPING --------------------------------------------------------

// PMU ID and ENV status regs
#define BCM59035_REG_PMUID           0x00    // R    revision/id
#define BCM59035_REG_ENV1            0x4C    // R    environment monitor
#define BCM59035_REG_ENV2            0x4D    // R    environment monitor [5:0]
#define BCM59035_REG_ENV3            0x4E    // R/W  environment monitor [4:0]
#define BCM59035_REG_ENV4            0x4F    // R    environment monitor
#define BCM59035_REG_HOSTACT         0x51    // R/W  hostact [2:0]                                        

#define BCM59035_NUM_INT_REG  10   // Number of interrupt registers i.e. INT1-10
#define BCM59035_NUM_IRQ      (BCM59035_NUM_INT_REG * 8) // 10 bytewide interrupt bit registers

// Interrupts
#define BCM59035_REG_INT1            0x01    // R&C  interrupt
#define BCM59035_REG_INT2            0x02    // R&C  interrupt
#define BCM59035_REG_INT3            0x03    // R&C  interrupt
#define BCM59035_REG_INT4            0x04    // R&C  interrupt
#define BCM59035_REG_INT5            0x05    // R&C  interrupt
#define BCM59035_REG_INT6            0x06    // R&C  interrupt
#define BCM59035_REG_INT7            0x07    // R&C  interrupt
#define BCM59035_REG_INT8            0x08    // R&C  interrupt
#define BCM59035_REG_INT9            0x09    // R&C  interrupt
#define BCM59035_REG_INT10           0x0A    // R&C  interrupt
                                        
#define BCM59035_REG_INT1M           0x0B    // R/W  interrupt mask
#define BCM59035_REG_INT2M           0x0C    // R/W  interrupt mask
#define BCM59035_REG_INT3M           0x0D    // R/W  interrupt mask
#define BCM59035_REG_INT4M           0x0E    // R/W  interrupt mask
#define BCM59035_REG_INT5M           0x0F    // R/W  interrupt mask
#define BCM59035_REG_INT6M           0x10    // R/W  interrupt mask
#define BCM59035_REG_INT7M           0x11    // R/W  interrupt mask
#define BCM59035_REG_INT8M           0x12    // R/W  interrupt mask
#define BCM59035_REG_INT9M           0x13    // R/W  interrupt mask
#define BCM59035_REG_INT10M          0x14    // R/W  interrupt mask
                           
// LDO opmodes: the next group of registers are for LDO to PC2,PC1 line relationships
// bit layouts for PC2,PC1 status are: 0,0[1:0] 0,1[3:2] 1,0[5:4] 1,1[7:6]
#define BCM59035_REG_A1OPMODCTRL     0x16    // R/W  aldo1
#define BCM59035_REG_A2OPMODCTRL     0x17    // R/W  aldo2
#define BCM59035_REG_R1OPMODCTRL     0x18    // R/W  rfldo1
#define BCM59035_REG_R2OPMODCTRL     0x19    // R/W  rfldo2
#define BCM59035_REG_H1OPMODCTRL     0x1A    // R/W  hcldo1
#define BCM59035_REG_H2OPMODCTRL     0x1B    // R/W  hcldo2
#define BCM59035_REG_IOPMODCTRL      0x1C    // R/W  ioldo
#define BCM59035_REG_M1OPMODCTRL     0x1D    // R/W  ms1ldo                         
#define BCM59035_REG_LOPMODCTRL      0x1E    // R/W  lcldo                     
#define BCM59035_REG_LV1OPMODCTRL    0x1F    // R/W  lv1ldo                                        
#define BCM59035_REG_SOPMODCTRL      0x20    // R/W  simldo
#define BCM59035_REG_LV2OPMODCTRL    0xA4    // R/W  lv2ldo                                        
#define BCM59035_REG_M2OPMODCTRL     0xA5    // R/W  ms2ldo
#define BCM59035_REG_AX1OPMODCTRL    0xA6    // R/W  ax1ldo
#define BCM59035_REG_AX2OPMODCTRL    0xA7    // R/W  ax2ldo

#define BCM59035_REG_MSPWRGRP		0x9E    // R/W  MSLDO power-up grouping control [5:0]
#define BCM59035_REG_AXPWRGRP		0x9F    // R/W  AXLDO power-up grouping control [5:0]
#define BCM59035_REG_LVPWRGRP		0xA0    // R/W  LVLDO power-up grouping control [5:0]
#define BCM59035_REG_IOPWRGRP		0xA1    // R/W  IOLDO power-up grouping control [2:0]
#define BCM59035_REG_SRPWRGRP		0xA2    // R/W  SR power-up grouping control [5:0]
#define BCM59035_REG_APWRGRP			0xAA    // R/W  ALDO power-up grouping control [5:0]
#define BCM59035_REG_RFPWRGRP		0xAB    // R/W  RFLDO power-up grouping control [5:0]
#define BCM59035_REG_LCSIMPWRGRP		0xAC    // R/W  LC and SIMLDO power-up grouping control [5:0]
#define BCM59035_REG_HCPWRGRP		0xAD    // R/W  HCLDO power-up grouping control [5:0]


// Core switching regulator controls
#define BCM59035_REG_CSRCTRL1        0x21    // R/W  core switching reg control [6:0]
	#define BCM59035_CSRCTRL1_DVS_ENABLE	0x20 // DVS Enable bit
#define BCM59035_REG_CSRCTRL2        0x22    // R/W  core switching reg control [6:0]
#define BCM59035_REG_CSRCTRL3        0x23    // R/W  core switching reg control [7:0]
#define BCM59035_REG_CSRCTRL4        0x24    // R/W  core switching reg control [7:0]
#define BCM59035_REG_CSRCTRL5        0x25    // R/W  core switching reg control [7:0]
#define BCM59035_REG_CSRCTRL6        0x26    // R/W  core switching reg control [7:0]
#define BCM59035_REG_CSRCTRL7        0x27    // R/W  core switching reg control [7:0]
#define BCM59035_REG_CSRCTRL8        0x28    // R/W  core switching reg control [7:0]
#define BCM59035_REG_CSRCTRL9        0x29    // R/W  core switching reg control [7:0]
#define BCM59035_REG_CSRCTRL10       0xAE    // R/W  core switching reg control [4:0]
#define BCM59035_REG_CSRCTRL11       0xCE    // R/W  core switching reg control [3:0]                                       // 

#define BCM59035_REG_CSROPMODCTRL    0x2A    // R/W  CSR to PC2,PC1 line relationships

// IO switching regulator controls
#define BCM59035_REG_IOSRCTRL1       0x2B    // R/W  io switching reg control [3:0]
#define BCM59035_REG_IOSRCTRL2       0x2C    // R/W  io switching reg control [7:0]
#define BCM59035_REG_IOSRCTRL3       0x2D    // R/W  io switching reg control [7:0]
#define BCM59035_REG_IOSRCTRL4       0x2E    // R/W  io switching reg control [7:0]
#define BCM59035_REG_IOSRCTRL5       0x2F    // R/W  io switching reg control [7:0]
#define BCM59035_REG_IOSRCTRL6       0x30    // R/W  io switching reg control [7:0]
#define BCM59035_REG_IOSRCTRL7       0x31    // R/W  io switching reg control [7:0]
#define BCM59035_REG_IOSRCTRL8       0x32    // R/W  io switching reg control [7:0]
#define BCM59035_REG_IOSRCTRL9       0x33    // R/W  io switching reg control [2:0]
                                        // bit layouts for PC2,PC1 status are: 0,0[1:0] 0,1[3:2] 1,0[5:4] 1,1[7:6]
#define BCM59035_REG_IOSROPMODCTRL   0x34    // R/W  IOSR to PC2,PC1 line relationships

#define BCM59035_REG_SRCTRL          0x35    // R/W  switching reg common control [6:0]

// Charger controls
#define BCM59035_REG_MBCCTRL1        0x36    // R/W  main battery charger control [6:0]
#define BCM59035_REG_MBCCTRL2        0x37    // R/W  main battery charger control [7:0]
#define BCM59035_REG_MBCCTRL3        0x38    // R/W  main battery charger control [7:0]
#define BCM59035_REG_MBCCTRL4        0x39    // R/W  main battery charger control [6:0]
#define BCM59035_REG_MBCCTRL5        0x3A    // R/W  main battery charger control [7:0]
#define BCM59035_REG_MBCCTRL6        0x3B    // R/W  main battery charger control [6:0]
#define BCM59035_REG_MBCCTRL7        0x3C    // R/W  main battery charger control [7:0]
#define BCM59035_REG_MBCCTRL8        0x3D    // R/W  main battery charger control [4:0]
#define BCM59035_REG_MBCCTRL9        0x3E    // R/W  main battery charger control [6:0]
#define BCM59035_REG_MBCCTRL10       0x3F    // R/W  main battery charger control [7:0]
#define BCM59035_REG_MBCCTRL11       0x40    // R/W  main battery charger control [4:0]
#define BCM59035_REG_MBCCTRL12       0x50    // R/W  main battery charger control [7:0]
#define BCM59035_REG_BBCCTRL         0x6F    // R/W  backup battery charger control [7:0]

// RTC 
#define BCM59035_REG_RTCSC           0xBC    // R/W  real time clock seconds [5:0]
#define BCM59035_REG_RTCMN           0xBD    // R/W  real time clock minutes [5:0]
#define BCM59035_REG_RTCHR           0xBE    // R/W  real time clock hours [4:0]
#define BCM59035_REG_RTCWD           0x41    // R/W  real time clock weekday [2:0]
#define BCM59035_REG_RTCDT           0x42    // R/W  real time clock day [4:0]
#define BCM59035_REG_RTCMT           0x43    // R/W  real time clock month [3:0]
#define BCM59035_REG_RTCYR           0x44    // R/W  real time clock year [7:0]
#define BCM59035_REG_RTCSC_A1        0x45    // R/W  alarm clock 1 seconds [5:0]
#define BCM59035_REG_RTCMN_A1        0x46    // R/W  alarm clock 1 minutes [5:0]
#define BCM59035_REG_RTCHR_A1        0x47    // R/W  alarm clock 1 hours [4:0]
#define BCM59035_REG_RTCWD_A1        0x48    // R/W  alarm clock 1 weekday [6:0]
#define BCM59035_REG_RTCDT_A1        0x49    // R/W  alarm clock 1 day [4:0]
#define BCM59035_REG_RTCMT_A1        0x4A    // R/W  alarm clock 1 month [3:0]
#define BCM59035_REG_RTCYR_A1        0x4B    // R/W  alarm clock 1 year [7:0]

// PWM and LED controls
#define BCM59035_REG_PWMLEDCTRL1     0x53    // R/W  pwm led control [3:0] control reg for output1
#define BCM59035_REG_PWMLEDCTRL2     0x54    // R/W  pwm led control [5:0]
#define BCM59035_REG_PWMLEDCTRL3     0x54    // R/W  pwm led control [5:0]
#define BCM59035_REG_PWMLEDCTRL4     0x56    // R/W  pwm led control [5:0]
#define BCM59035_REG_PWMLEDCTRL5     0x57    // R/W  pwm led control [6:0]
#define BCM59035_REG_PWMLEDCTRL6     0x58    // R/W  pwm led control [3:0] control reg for output2
#define BCM59035_REG_PWMLEDCTRL7     0x59    // R/W  pwm led control [5:0]
#define BCM59035_REG_PWMLEDCTRL8     0x59    // R/W  pwm led control [5:0]
#define BCM59035_REG_PWMLEDCTRL9     0x5B    // R/W  pwm led control [5:0]
#define BCM59035_REG_PWMLEDCTRL10    0x5C    // R/W  pwm led control [5:4][1:0]

// Debounce settings
#define BCM59035_REG_PONKEYBDB       0x5D    // R/W  power-on key debounce [6:0]
#define BCM59035_REG_ACDDB           0x5E    // R/W  acd debounce [6:0]
#define BCM59035_REG_PHFDDB          0x5F    // R/W  phfd debounce [4:0]
#define BCM59035_REG_PONKEYBDB1      0xBA    // R/W  power-on key debounce 
#define BCM59035_REG_I2CSLAVEID      0xBB    // R/W  i2c slave id [1:0]
#define BCM59035_REG_BBLOWDB         0xBF    // R/W  backup battery debounce register [3:0]
                                        
// Fuel gauge registers
#define BCM59035_REG_FGACCM1         0x60    // R&C  fuel gauge accumulation [7:0]
#define BCM59035_REG_FGACCM2         0x61    // R&C  fuel gauge accumulation [7:0]
#define BCM59035_REG_FGACCM3         0x62    // R&C  fuel gauge accumulation [7:0]
#define BCM59035_REG_FGACCM4         0x63    // R&C  fuel gauge accumulation [1:0]
#define BCM59035_REG_FGCNT1          0x64    // R&C  fuel gauge sample count [7:0]
#define BCM59035_REG_FGCNT2          0x65    // R&C  fuel gauge sample count [3:0]
#define BCM59035_REG_FGSMPL1         0x66    // R&C  fuel gauge current sample [7:0]
#define BCM59035_REG_FGSMPL2         0x67    // R&C  fuel gauge current sample [5:0]
#define BCM59035_REG_FGCTRL1         0x68    // R/W  fuel gauge control [7:0]
#define BCM59035_REG_FGCTRL2         0x69    // R/W  fuel gauge control [7:0]
#define BCM59035_REG_FGCTRL3         0x93    // R/W  fuel gauge control [6:0]

#define BCM59035_REG_FGOFFSET1		0x94    // R    Fuel gauge offset register1 [7:0]
#define BCM59035_REG_FGOFFSET2		0x95    // R    Fuel gauge offset register2 [0]
#define BCM59035_REG_FGSMPLB1		0x96    // R    Fuel gauge current sample outb register 1 [7:0]
#define BCM59035_REG_FGSMPLB2		0x97    // R    Fuel gauge current sample outb register 2 [5:0]
#define BCM59035_REG_FGSLEEPCNT1		0x98    // R    Fuel gauge sleep count register1 [7:0]
#define BCM59035_REG_FGSLEEPCNT2		0x99    // R    Fuel gauge sleep count register2 [7:0]
#define BCM59035_REG_FGOPMODCTRL		0x9A    // R/W  Fuel gauge operation mode register [3:0]
#define BCM59035_REG_FGGNRL1			0x9B    // R/W  Fuel gauge general purpose register1 [7:0]
#define BCM59035_REG_FGGNRL2			0x9C    // R/W  Fuel gauge general purpose register2 [7:0]
#define BCM59035_REG_FGGNRL3			0x9D    // R/W  Fuel gauge general purpose register3 [7:0]
#define BCM59035_REG_WAKEACCM1		0xC0    // R/W  Fuel gauge wake up accumulator register1 [7:0]                                       
#define BCM59035_REG_WAKEACCM2		0xC1    // R/W  Fuel gauge wake up accumulator register2 [7:0]
#define BCM59035_REG_WAKEACCM3		0xC2    // R/W  Fuel gauge wake up accumulator register3 [7:0]
#define BCM59035_REG_WAKEACCM4		0xC3    // R/W  Fuel gauge wake up accumulator register4 [7:0]
#define BCM59035_REG_WAKECNT1		0xC4    // R/W  Fuel gauge wake up count register1 [7:0]
#define BCM59035_REG_WAKECNT2		0xC5    // R/W  Fuel gauge wake up count register2 [7:0]
#define BCM59035_REG_WAKETIME1		0xC6    // R/W  Fuel gauge wake up time register1 [7:0]
#define BCM59035_REG_WAKETIME2		0xC7    // R/W  Fuel gauge wake up time register2 [7:0]
#define BCM59035_REG_FGCTRL4         0xC8    // R/W  fuel gauge control4 [5:0]
                                        
// Fuel gauge test registers
#define BCM59035_REG_FGCICCTRL       0x6A    // R/W  ATE writable [0]
#define BCM59035_REG_FGTRIMGN1_1     0x6B    // R    ATE reaeable [7:0]
#define BCM59035_REG_FGTRIMGN1_2     0x6C    // R    ATE reaeable [5:0]
#define BCM59035_REG_FGTRIMGN2_1     0x6D    // R    ATE reaeable [7:0]
#define BCM59035_REG_FGTRIMGN2_2     0x6E    // R    ATE reaeable [5:0]
                                        
// PLL registers
#define BCM59035_REG_PLLADCCLKSEL    0x71    // R/W  adc clock select [1:0]
#define BCM59035_REG_PLLN            0x72    // R/W  pll divider [4:0]
#define BCM59035_REG_PLLVREGTRM      0x73    // R/W  pll regulator trim [1:0]
#define BCM59035_REG_PLLFCURVE       0x74    // R/W  pll tuning curve [2:0]
#define BCM59035_REG_PLLCP           0x75    // R/W  pll current select [2:0]
#define BCM59035_REG_PLLCLKSEL       0x76    // R/W  pll frequency select [1:0]
#define BCM59035_REG_PLLCTRL         0x77    // R/W  pll control [7:0]

// GPIO register
#define BCM59035_REG_GPIOCTRL		0x8C    // R/W  GPIO control register [5:0]

// OTP registers
#define BCM59035_REG_OTPMBCTRM       0x78    // R/W  main charger trim code [5:0]
#define BCM59035_REG_OTPUSBCCTRM		0x79    // R/W  USB CC mode current trimming code register [3:0]
#define BCM59035_REG_OTPCGHCVS       0x7A    // R/W  comparator voltage select [1:0]
#define BCM59035_REG_OTPFGTRIMCTL    0x7B    // R/W  fuel gauge trim control [5:0]
#define BCM59035_REG_OTPBBMBVS       0x7C    // R/W  main/backup battery comparator [1:0]
#define BCM59035_REG_OTPMBWChys      0x7D    // R/W  main battery working comparator hysteresis[0]
#define BCM59035_REG_OTPCGMBSEL		0x7E    // R/W  WAC and Main battery offset voltage comparator selection control register [0]
#define BCM59035_REG_OTPUBPDhys      0x7F    // R/W  usb present detector hysteresis[0]
#define BCM59035_REG_VERYLOWBATgap   0x80    // R/W  very low battery voltage comparator hystersis[0]
#define BCM59035_REG_OTPCHPDhys      0x81    // R/W  charger high comparator hysteresis[0]
                                        
#define BCM59035_REG_OTPUBMBSEL		0x84    // R/W  USB charger and main battery comparator selection control register [0]
#define BCM59035_REG_LOWBATCVS       0x85    // R/W  Low main battery comparator threshold select [7:0]
#define BCM59035_REG_OTPBG3VTRIM		0x87    // R/W  3V BG trimming control bits [7:0] 
#define BCM59035_REG_OTPMBTEMPhys	0x89    // R/W  Main battery over/under temperature compsarator hysteresis [3:0] 
#define BCM59035_REG_OTPRDTST		0xA3    // R    Analog test mode OTP read register [7:0]
#define BCM59035_REG_OTPWRMODE		0xB2    // R/W  OTP programming mode in normal mode register [0]

// Analog test registers
#define BCM59035_REG_TSTRECH         0x82    // R/W  accessory recognition high voltage [2:0]
#define BCM59035_REG_TSTRECL         0x83    // R/W  accessory recognition low voltage [2:0]
#define BCM59035_REG_TSTTHS          0x86    // R/W  high temperature sensor control [5:0]
#define BCM59035_REG_PDCMPSYN        0xD6    // R    presence detection/comparator status [7:0]
#define BCM59035_REG_PDCMPSYN1       0xD7    // R    presence detection/comparator status [7:0]
#define BCM59035_REG_PDCMPSYN2       0xD8    // R    presence detection/comparator status [7:0]
#define BCM59035_REG_PDCMPSYN3       0xD9    // R    presence detection/comparator status [7:0]
#define BCM59035_REG_PDCMPSYN4       0xDA    // R    presence detection/comparator status [7:0]
#define BCM59035_REG_PDCMPSYN5       0xDB    // R    presence detection/comparator status [7:0]                                        
#define BCM59035_REG_TSTMBC2         0x8D    // R    main battery charger test[7:3]

// Mode registers
#define BCM59035_REG_PMUmode         0x8E    // R    pmu mode control [1:0]
#define BCM59035_REG_TPEN            0x8F    // R/W  enable test mux control [6:0]
#define BCM59035_REG_DIGDBG1         0x90    // R    digital debug state status [7:0]
#define BCM59035_REG_DIGDBG2         0x91    // R    digital debug status check [6:0]
#define BCM59035_REG_DIGDBG3         0x92    // R    digital debug power down status [6:0]
#define BCM59035_REG_DIGDBG4         0xCF    // R    digital debug
                                        
#define BCM59035_REG_ANADBG1			0xAF    // R/W  analog debug register 1 [7:0]
#define BCM59035_REG_ANADBG2			0xB0    // R/W  analog debug register 2 [7:0]
#define BCM59035_REG_ANADBG3			0xCC    // R/W  analog debug register 3 [7:0]
#define BCM59035_REG_ANADBG4			0xCD    // R/W  analog debug register 4 [7:0]
#define BCM59035_REG_ATMCTRL			0xB1    // R/W  analog test mode control register [7:0]


// LDO voltage controls
#define BCM59035_REG_ALDOCTRL        0xB3    // R/W  ldo control [6:4][2:0]
#define BCM59035_REG_RFDOCTRL        0xB4    // R/W  ldo control [6:4][2:0]
#define BCM59035_REG_HCLDOCTRL    	0xB5    // R/W  ldo control [6:4][2:0]
#define BCM59035_REG_IOLDOCTRL       0xB6    // R/W  ldo control [4:0]
#define BCM59035_REG_MSLDOCTRL       0xB7    // R/W  ldo control [7:0]
#define BCM59035_REG_LCSIMDOCTRL     0xB8    // R/W  ldo control [7:6][4:0]
#define BCM59035_REG_LVLDOCTRL	    0xB9    // R/W  ldo control [4:3][1:0]
#define BCM59035_REG_AX1LDOCTRL	    0xA8    // R/W  ldo control [4:0]
#define BCM59035_REG_AX2LDOCTRL	    0xA9    // R/W  ldo control [4:0]

// OTG registers
#define BCM59035_REG_OTGCTRL1        0xC9    // R/W  otg control1 [7:0]
#define BCM59035_REG_OTGCTRL2	    0xCA    // R/W  otg control2 [1:0]
#define BCM59035_REG_VBCTRL1         0xCB    // R/W  otg boost regulstor control [4:0]

#define BCM59035_REG_BOOSTCTRL1      0xD0    // R/W  otg boost regulator control1
#define BCM59035_REG_BOOSTCTRL2      0xD1    // R/W  otg boost regulator control2
#define BCM59035_REG_BOOSTCTRL3      0xD2    // R/W  otg boost regulator control3
#define BCM59035_REG_BOOSTCTRL4      0xD3    // R/W  otg boost regulator control4
#define BCM59035_REG_DBGOTG1 	    0xD4    // R/W  otg debug register1
#define BCM59035_REG_DBGOTG2         0xD5    // R/W  otg debug register2

                                        
#define BCM59035_REG_TOTAL			(BCM59035_REG_PDCMPSYN5+1)		// 0xDC registers for BCM59035 B0
#define BCM59035_REG_LDO_SR_TOTAL	9
#define BCM59035_REG_LDO_SR_START_INDEX	BCM59035_REG_ALDOCTRL


// Bitwise defines
//
// HOSTACT
#define BCM59035_HOSTACT_WDT_CLR     0x01
#define BCM59035_HOSTACT_WDT_ON      0x02
#define BCM59035_HOSTACT_HOSTDICOFF  0x04
// ENV1
#define BCM59035_ENV1_UBMBC          0x01    // Usb voltage higher than MB voltage
#define BCM59035_ENV1_CGPD           0x02
#define BCM59035_ENV1_UBPD           0x04
#define BCM59035_ENV1_MBWV           0x08
#define BCM59035_ENV1_CGHC           0x10
#define BCM59035_ENV1_BBLOW          0x40
#define BCM59035_ENV1_CGMBC          0x80
// ENV2
#define BCM59035_ENV2_PONKEYB        0x01
#define BCM59035_ENV2_ACD            0x02
#define BCM59035_ENV2_PHFD           0x04
#define BCM59035_ENV2_THSD           0x08
#define BCM59035_ENV2_UBHC           0x10    // USB input over voltage
#define BCM59035_ENV2_USBENTLVL      0x20
// ENV3
#define BCM59035_ENV3_MBPD           0x01
#define BCM59035_ENV3_MBTEMPLOW      0x02
#define BCM59035_ENV3_MBTEMPHIGH     0x04
#define BCM59035_ENV3_NTCRM_SAVE     0x08
#define BCM59035_ENV3_PWRUP_SAVE     0x10

// ENV4
#define BCM59035_ENV4_VBUS_VALID     0x01
#define BCM59035_ENV4_A_SESS_VALID   0x02
#define BCM59035_ENV4_B_SESS_END     0x04
#define BCM59035_ENV4_ID_IN          0x08

// INT1         R&C  interrupt [6:0], [7] reserved
#define BCM59035_INT1_PONKEYBR       0x01
#define BCM59035_INT1_PONKEYBF       0x02
#define BCM59035_INT1_PONKEYBH       0x04
#define BCM59035_INT1_RTC60S         0x08
#define BCM59035_INT1_RTCA1          0x10
#define BCM59035_INT1_RESERVED       0x20
#define BCM59035_INT1_RTCADJ         0x40
#define BCM59035_INT1_RTC1S          0x80

//INT2          R&C  interrupt [7:0]   
#define BCM59035_INT2_CHGINS         0x01
#define BCM59035_INT2_CHGRM          0x02
#define BCM59035_INT2_CHGERR         0x04
#define BCM59035_INT2_EOC              0x08
#define BCM59035_INT2_USBINS         0x10
#define BCM59035_INT2_USBRM          0x20
#define BCM59035_INT2_USBERR           0x40
#define BCM59035_INT2_MBCCHGERR      0x80

//INT3          R&C  interrupt [7:0]
#define BCM59035_INT3_ACDINS         0x01
#define BCM59035_INT3_ACDRM          0x02
#define BCM59035_INT3_PHFDRLS        0x04
#define BCM59035_INT3_PHFDINS        0x08
#define BCM59035_INT3_PHFDRM         0x10
#define BCM59035_INT3_PHFDPRS        0x20
#define BCM59035_INT3_VERYLOWBAT     0x40
#define BCM59035_INT3_BBLOWB         0x80

//INT4          R&C  interrupt [7:0]
#define BCM59035_INT4_A1OVRI         0x01
#define BCM59035_INT4_A2OVRI         0x02
#define BCM59035_INT4_R1OVRI         0x04
#define BCM59035_INT4_R2OVRI         0x08
#define BCM59035_INT4_H1OVRI         0x10
#define BCM59035_INT4_H2OVRI         0x20
#define BCM59035_INT4_M1OVRI         0x40
#define BCM59035_INT4_M2OVRI         0x80

// INT5         R&C  interrupt [6:0]
#define BCM59035_INT5_LOVRI          0x01
#define BCM59035_INT5_LV1OVRI        0x02
#define BCM59035_INT5_LV2OVRI        0x04
#define BCM59035_INT5_IOVRI          0x08
#define BCM59035_INT5_SOVRI          0x10
#define BCM59035_INT5_AX1OVRI    	0x20
#define BCM59035_INT5_AX2OVRI        0x40

// INT6         R&C  interrupt [4:0]
#define BCM59035_INT6_IOSROVRI           0x01
#define BCM59035_INT6_CSROVRI            0x02
#define BCM59035_INT6_IOSROVRV           0x04
#define BCM59035_INT6_CSROVRV            0x08
#define BCM59035_INT6_FGC                0x10
#define BCM59035_INT6_TOOWARM            0x20

// INT7         R&C  interrupt [7:0]
#define BCM59035_INT7_MBTEMPFAULT        0x01
#define BCM59035_INT7_MBTEMPLOW          0x02
#define BCM59035_INT7_MBTEMPHIGH         0x04
#define BCM59035_INT7_MBRM               0x08
#define BCM59035_INT7_MBOV               0x10
#define BCM59035_INT7_NOBAT              0x20
#define BCM59035_INT7_BATINS             0x40
#define BCM59035_INT7_LOWBAT             0x80

// INT8         R&C  interrupt [7:0]
#define BCM59035_INT8_VBUSVALID          0x01
#define BCM59035_INT8_A_SESSVALID        0x02
#define BCM59035_INT8_B_SESSEND          0x04
#define BCM59035_INT8_ID_INSRT           0x08
#define BCM59035_INT8_RESUME_VBUS        0x80

// INT9         R&C  interrupt [3:0]
#define BCM59035_INT9_VBUSVALID          0x01
#define BCM59035_INT9_A_SESSVALID        0x02
#define BCM59035_INT9_B_SESSEND          0x04
#define BCM59035_INT9_ID_RMV             0x08

// INT10         R&C  interrupt [2:0]
#define BCM59035_INT10_VBOVRI            0x01
#define BCM59035_INT10_VBOV              0x02


// Fuel Gauge
#define BCM59035_FGCTRL1_FGHOSTEN        0x01
#define BCM59035_FGCTRL1_FGSTPCHOP       0x02    // 
#define BCM59035_FGCTRL3_FGTRIM          0x01
#define BCM59035_FGCTRL3_FGCAL           0x02
#define BCM59035_FGCTRL3_FGRESET         0x04    // reset ADC output register
#define BCM59035_FGCTRL3_FGFRZREAD       0x08
#define BCM59035_FGCTRL3_FGFORCECAL      0x10    // force the ADC input to be shorted
#define BCM59035_FGCTRL3_FGMODE          0x20    // 0 = continuous mode; 1 = synchronous mode
#define BCM59035_FGCTRL3_FGMODON         0x40    // modulator off when FG is paused
#define BCM59035_FGCTRL3_LONGCAL         0x80

#define BCM59035_FGOFFSET_MASK           0x1FF
#define BCM59035_FGCOUNT_MASK            0xFFF

// LCSIMLDOCTRL
#define BCM59035_LCSIMLDOCTRL_SIM_BIT_SHIFT 6
#define BCM59035_LCSIMLDOCTRL_SIMMASK    0xC0
#define BCM59035_LCSIMLDOCTRL_SIM_1V8    0xC0  
#define BCM59035_LCSIMLDOCTRL_SIM_3V1    0x80
#define BCM59035_LCSIMLDOCTRL_SIM_2V5    0x40
#define BCM59035_LCSIMLDOCTRL_SIM_3V0    0x00

// GPIO
#define BCM59035_GPIOCTRL_GPIO1DIR_HIZ   0x00
#define BCM59035_GPIOCTRL_GPIO1DIR_OUT   0x01
#define BCM59035_GPIOCTRL_GPIO1DIR_IN    0x02
#define BCM59035_GPIOCTRL_GPIO1DIR_MASK  0x03
#define BCM59035_GPIOCTRL_GPIO1_DATA     0x04    // for read or write, depending on direction
#define BCM59035_GPIOCTRL_GPIO1_DATAPOS  2       // bit 2
#define BCM59035_GPIOCTRL_GPIO2DIR_HIZ   0x00
#define BCM59035_GPIOCTRL_GPIO2DIR_OUT   0x08
#define BCM59035_GPIOCTRL_GPIO2DIR_IN    0x10
#define BCM59035_GPIOCTRL_GPIO2DIR_MASK  0x18
#define BCM59035_GPIOCTRL_GPIO2_DATA     0x20    // for read or write, depending on direction
#define BCM59035_GPIOCTRL_GPIO2_DATAPOS  5       // bit 5
                                            
// PWM and LED
#define BCM59035_LEDON_MASK	    0x03
#define BCM59035_LEDON			0x01 //0x03
#define BCM59035_PWMON			0x02
#define BCM59035_SYSCLK_MASK	    0x0C
#define BCM59035_SYSCLK_4		0x00
#define BCM59035_SYSCLK_16		0x04
#define BCM59035_SYSCLK_64		0x08
#define BCM59035_SYSCLK_512		0x0C
#define BCM59035_PWMLED_PDN      0x40
#define BCM59035_PWMLED1		    0x02
#define BCM59035_PWMLED2		    0x01

// MBCCTRL bit fields

#define BCM59035_MBCCTRL1_WACTIMER_MASK   0x07
	#define BCM59035_MBCCTRL1_WACTIMER_180   0x06 // 180 min
	#define BCM59035_MBCCTRL1_WACTIMER_DIS   0x07 // Disabled 
#define BCM59035_MBCCTRL1_WAC_ELAPSED_TIMER_MASK   (0x07 << 4)
	#define BCM59035_MBCCTRL1_WAC_ELAPSED_TIMER_180   (0x06 << 4) // 180 min
	#define BCM59035_MBCCTRL1_WAC_ELAPSED_TIMER_DIS   (0x07 << 4) // Disabled 

#define BCM59035_MBCCTRL2_VCHGRRC    0x80    // wall charger rapid charge enable
#define BCM59035_MBCCTRL2_MBCHOSTEN  0x40    // charger enable
#define BCM59035_MBCCTRL2_USBTIMER_DIS 0x7
#define BCM59035_MBCCTRL2_USB_ELAPSED_TIMER_DIS (0x7 << 3)
#define BCM59035_MBCCTRL3_RC2_MASK   0x0F    // mask for wall RC2
   #define BCM59035_MBCCTRL3_WAC_RC2_4_2V   0x0 // value for 4.2V

#define BCM59035_MBCCTRL3_TC2_MASK   0xF0    // mask for wall RC2
   #define BCM59035_MBCCTRL3_WAC_TC2_3_2V   0xF // value for 3.2V
   #define BCM59035_MBCCTRL3_WAC_TC2_3_6V   (0x6<<4) // value for 3.6V

#define BCM59035_MBCCTRL4_RC1_MASK   0x1F    // mask for wall RC1
#define BCM59035_MBCCTRL4_MBCWRC1_4  0x10    // wall charger RC1 high bit (bit 4)
#define BCM59035_MBCCTRL4_MBCWRC1_LOW_MASK 0x0F    // wall charger RC1 low bits (bit 3:0)
#define BCM59035_MBCCTRL4_MBCWTC1_4  0x20    // wall charger TC1 high bit (bit 4)                                        
#define BCM59035_MBCCTRL4_MBCUTC1_4  0x40    // usb charger TC1 high bit (bit 4)
   #define BCM59035_MBCCTRL4_WAC_RC_950MA   0xF // Value for 950mA
#define BCM59035_MBCCTRL5_RC2_MASK   0x0F    // mask for usb RC2
   #define BCM59035_MBCCTRL5_USB_RC2_4_2V   0x0 // value for 4.2 V
   #define BCM59035_MBCCTRL5_USB_RC2_4_15V   0x1 // value for 4.15 V
   #define BCM59035_MBCCTRL5_USB_RC2_4_10V   0x2 // value for 4.10 V
   #define BCM59035_MBCCTRL5_USB_RC2_4_05V   0x3 // value for 4.05 V
   #define BCM59035_MBCCTRL5_USB_RC2_4_00V   0x4 // value for 4.00 V
   #define BCM59035_MBCCTRL5_USB_RC2_3_95V   0x5 // value for 3.95 V
   #define BCM59035_MBCCTRL5_USB_RC2_3_90V   0x6 // value for 3.90 V
   #define BCM59035_MBCCTRL5_USB_RC2_3_20V   0x7 // value for 3.20 V
   #define BCM59035_MBCCTRL5_USB_RC2_3_30V   0x8 // value for 3.30 V
   #define BCM59035_MBCCTRL5_USB_RC2_3_40V   0x9 // value for 3.40 V
   #define BCM59035_MBCCTRL5_USB_RC2_3_50V   0xA // value for 3.50 V
   #define BCM59035_MBCCTRL5_USB_RC2_3_60V   0xB // value for 3.60 V

#define BCM59035_MBCCTRL5_TC2_MASK   0xF0    // mask for usb TC2 
   #define BCM59035_MBCCTRL5_USB_TC2_4_20V   (0x0<<4) // value for 4.20 V
   #define BCM59035_MBCCTRL5_USB_TC2_4_15V   (0x1<<4) // value for 4.15 V
   #define BCM59035_MBCCTRL5_USB_TC2_4_10V   (0x2<<4) // value for 4.10 V
   #define BCM59035_MBCCTRL5_USB_TC2_4_05V   (0x3<<4) // value for 4.05 V
   #define BCM59035_MBCCTRL5_USB_TC2_4_00V   (0x4<<4) // value for 4.00 V
   #define BCM59035_MBCCTRL5_USB_TC2_3_95V   (0x5<<4) // value for 3.95 V
   #define BCM59035_MBCCTRL5_USB_TC2_3_90V   (0x6<<4) // value for 3.90 V
   #define BCM59035_MBCCTRL5_USB_TC2_3_20V   (0x7<<4) // value for 3.20 V
   #define BCM59035_MBCCTRL5_USB_TC2_3_30V   (0x8<<4) // value for 3.30 V
   #define BCM59035_MBCCTRL5_USB_TC2_3_40V   (0x9<<4) // value for 3.40 V
   #define BCM59035_MBCCTRL5_USB_TC2_3_50V   (0xA<<4) // value for 3.50 V
   #define BCM59035_MBCCTRL5_USB_TC2_3_60V   (0xB<<4) // value for 3.60 V

#define BCM59035_MBCCTRL6_RC1_MASK   0x1F    // mask for usb RC1 current
   #define BCM59035_MBCCTRL6_USB_RC_950MA   0x1F // value for 950mA 

#define BCM59035_MBCCTRL6_MBCURC1_4  0x10    // usb charger RC1 high bit (bit 4)
#define BCM59035_MBCCTRL6_MBCURC1_LOW_MASK 0x0F    // usb charger RC1 low bits (bit 3:0)
#define BCM59035_MBCCTRL11_MBCWTC1_LOW_MASK 0x0F    // usb charger RC1 low bits (bit 3:0)
#define BCM59035_MBCCTRL11_MBCUTC1_LOW_MASK 0xF0    // usb charger RC1 low bits (bit 3:0)
   #define BCM59035_MBCCTRL11_WACTC1_100MA   0x5 //500 ma
   #define BCM59035_MBCCTRL11_USBTC1_950MA   (0xF<<4)
                                            
#define BCM59035_MBCCTRL7_EOCS_MASK  0x0F    // charger EOC field
   #define BCM59035_MBCCTRL7_50MA   0x1
   #define BCM59035_MBCCTRL7_200MA   0xF
#define BCM59035_MBCCTRL7_OV_DISABLE  0x80    // Over voltage disable 

#define BCM59035_MBCCTRL8_MBCFAULTCNT_MASK  0x0F    // mbc fault count mask
#define BCM59035_MBCCTRL8_PAUSECHARGE  0x20    // pause charge
#define BCM59035_MBCCTRL8_VUBGRRC      0x40    // usb charger rapid charge enable
#define BCM59035_MBCCTRL8_TTRST        0x80    // total charge timer reset

#define BCM59035_MBCCTRL9_MAINTCHRG   0x01    // maintenance charge enable
#define BCM59035_MBCCTRL9_CON_TYP     0x02    // connector type
#define BCM59035_MBCCTRL9_SP_TYP      0x04    // single connector type
#define BCM59035_MBCCTRL9_ADP_PRI     0x08    // adaptor priority bit
#define BCM59035_MBCCTRL9_OTG_I_LMT_MASK   0x30  // OTG current limit mask
#define BCM59035_MBCCTRL9_LOWCOST_WAC_EN  (0x1<<6) 
#define BCM59035_MBCCTRL9_LOWCOST_USB_EN  (0x1<<7) 
                                         
#define BCM59035_MBCCTRL10_NTCON       0x01    // NTC LDO enable
#define BCM59035_MBCCTRL10_BAT_DET_EN  0x02    // battery presence detection enable
#define BCM59035_MBCCTRL10_BTEMP_EN    0x04    // battery temp detect enable
#define BCM59035_MBCCTRL10_SYS_TYP     0x08    // system type bit: determine charger behavior when no battery
#define BCM59035_MBCCTRL10_TC2CVCCSEL  0x10    // TC2 CV and CC select

// LOWBATCVS bit fields
#define BCM59035_LOWBATCVS_LOWBATCVS_MASK 0x03 // low main battery comparator threshold selector mask
#define BCM59035_LOWBATCVS_LOWBATCVS_3_6V 0x03 //3.6V
#define BCM59035_LOWBATCVS_MBMCVS_MASK 0x0C // main battery maintenance charge comparator threshold selector mask
#define BCM59035_LOWBATCVS_MBMCVS_POS  2     // MBMCVS field starts at bit 2

// BBCCTRL bit fields
#define BCM59035_BBCCTRL_BBCHOSTEN   0x01    // backup battery charger enable
#define BCM59035_BBCCTRL_BBCLOWIEN   0x40    // low charging current enable
#define BCM59035_BBCCTRL_BBCCS_MASK  0x4C    // mask for BBCCS field
#define BCM59035_BBCCTRL_BBCVS_MASK  0x30    // mask for BBCVS field
#define BCM59035_BBCCTRL_BBCRS_MASK  0x82    // mask for BBCRS field
#define BCM59035_BBCCTRL_BBCRS_P5K   0x00    // bit value for 1K resistor                                        
#define BCM59035_BBCCTRL_BBCRS_1K    0x02    // bit value for 1K resistor                                       
#define BCM59035_BBCCTRL_BBCRS_2K    0x80    // bit value for 1K resistor
#define BCM59035_BBCCTRL_BBCRS_4K    0x82    // bit value for 1K resistor
#define BCM59035_BBLOWDB_BBLOWCVS_MASK 0x03  // mask for BBLOWCVS field

// PONKEY bit fields
#define BCM59035_PONKEYBDB_ONHOLD_MASK    0x07
#define BCM59035_PONKEYBDB_PONKEYBRF_MASK 0x38
#define BCM59035_PONKEYBDB_KEYLOCK        0x40    // bit value for keylock bit
#define BCM59035_ACDDB_PONKEYBDEL_MASK    0x70    // shutdown delay mask
#define BCM59035_PONKEYBDB1_OFFHOLD_MASK  0x07    // bit value for keylock bit

// PHFD bit fields
#define BCM59035_PHFDDB_PHFDEN       0x10    // bit value for PHFD enable

#define MAX_NUMBER_LDO		15

//******************************************************************************
// Typedefs 
//******************************************************************************
//CON_TYPE related
typedef enum {
    USB_WALL_ONECON,   // type 1 or 2: coming thru one connector
    STANDARD_USB_WALL_TWOCON,      // type 3: coming thru two connectors - dual connector
    //NON_MINI_USB,                  // type 1, 2, or 3
    MINI_USB_ONECON                // type 4: coming thru one connector
} Connector_Type_t;

//adaptor priority related
typedef enum {
    ADAPTOR_PRI_USB,                // usb has priority
    ADAPTOR_PRI_WALL                // wall has priority
} Adaptor_Pri_t;

//single connector type (when Con_type is 1)
typedef enum {
    SINGLE_CONN_USB_ADAPTOR,                // single connector, usb adaptor plugged in
    SINGLE_CONN_WALL_ADAPTOR                // single connector, wall adaptor plugged in
} Single_Conn_Type_t;

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

typedef struct
{
	u8 ldo_sr[BCM59035_REG_LDO_SR_TOTAL];
} LDO_Settings_tt; //FIXME

//use operation character '|' to select muti-operation mode like
//PMULED_Oper_Save | PMULED_Oper_Standby | PMULED_Oper_Active
typedef u8   PMULedOperMode_t;
#define PMULED_Oper_Off     0x00
#define PMULED_Oper_Charger 0x40
#define PMULED_Oper_Active  0x80

// Used for BBCCharge maintenance
#define BBC_STATE_ON    1
#define BBC_STATE_OFF   0
#define DEFAULT_BBC_INIT_ON_DURATION   4 * 60 * 60 * TICKS_ONE_SECOND  // 4 hours
#define DEFAULT_BBC_ON_DURATION     1 * 60 * 60 * TICKS_ONE_SECOND  // 1 hour
#define DEFAULT_BBC_OFF_DURATION    2 * 60 * 60 * TICKS_ONE_SECOND  // 2 hours
                                                                     
// Main battery charger EOC current
typedef enum{
    BCM59035_EOCS_50MA,
    BCM59035_EOCS_60MA,
    BCM59035_EOCS_80MA,
    BCM59035_EOCS_70MA,
    BCM59035_EOCS_200MA,
    BCM59035_EOCS_90MA,
    BCM59035_EOCS_100MA,
    BCM59035_EOCS_110MA,
    BCM59035_EOCS_120MA,
    BCM59035_EOCS_130MA,
    BCM59035_EOCS_140MA,
    BCM59035_EOCS_150MA,
    BCM59035_EOCS_160MA,
    BCM59035_EOCS_170MA,
    BCM59035_EOCS_180MA,
    BCM59035_EOCS_190MA
} BCM59035_EOCS_t;

// Maintenance charge voltage level
typedef enum{
    BCM59035_MBMCVS_3P95,
    BCM59035_MBMCVS_4P00,
    BCM59035_MBMCVS_4P05,
    BCM59035_MBMCVS_4P10
} BCM59035_MBMCVS_t;

// Backup battery charger related voltage, current, and resistor controls
typedef enum{
    BCM59035_BBCVS_2P5,
    BCM59035_BBCVS_3P0,
    BCM59035_BBCVS_3P3,
    BCM59035_BBCVS_NoChange = 0xff
} BCM59035_BBCVS_t;

typedef enum{
    BCM59035_BBCRS_P5K,
    BCM59035_BBCRS_1K,
    BCM59035_BBCRS_2K,
    BCM59035_BBCRS_4K,
    BCM59035_BBCRS_NoChange = 0xff
} BCM59035_BBCRS_t;

typedef enum{
    BCM59035_BBCCS_10UA = 0,
    BCM59035_BBCCS_20UA = 1,
    BCM59035_BBCCS_50UA = 2,
    BCM59035_BBCCS_100UA = 3,
    BCM59035_BBCCS_200UA = 4,
    BCM59035_BBCCS_400UA = 5,
    BCM59035_BBCCS_NoChange = 0xff
} BCM59035_BBCCS_t;


// Charger Watchdog Timer: total charge timer
typedef enum
{
	BCM59035_WDT_3HOUR = 0,
	BCM59035_WDT_4HOUR,
	BCM59035_WDT_5HOUR,
	BCM59035_WDT_6HOUR,
	BCM59035_WDT_7HOUR,
	BCM59035_WDT_8HOUR,
	BCM59035_WDT_9HOUR,
	BCM59035_WDT_DISABLE
}BCM59035_WDT_t;


typedef enum{
	BM_USBCHG_50mA,		// fast charging at 50mA by USB
	BM_USBCHG_100mA,	// fast charging at 100mA by USB
	BM_USBCHG_200mA,	// fast charging at 200mA by USB
	BM_USBCHG_300mA,	// fast charging at 300mA by USB
	BM_USBCHG_400mA,	// fast charging at 400mA by USB
	BM_USBCHG_500mA,	// fast charging at 500mA by USB
	BM_USBCHG_600mA		// fast charging at 600mA by USB
} BattMgrUSBCurrentLevel_t;

typedef enum {
	BCM59035_GPO_ACTIVE_LOW  = 0,
    BCM59035_GPO_ACTIVE_HIGH
/*	BCM59035_GPO_LED1_OUTPUT,
	BCM59035_GPO_LED2_OUTPUT,
	BCM59035_GPO_PWM1_OUTPUT,
	BCM59035_GPO_N_PWM1_OUTPUT,
	BCM59035_GPO_PWM2_OUTPUT,
	BCM59035_GPO_N_PWM2_OUTPUT,
	BCM59035_GPO_HIGH_IMPEDANCE */
} BCM59035_GPOutput_t;

typedef enum {
	BCM59035_GPO1 = 1,
	BCM59035_GPO2 = 2,
    BCM59035_GPO3 = 3
} BCM59035_GPOpin_t;

typedef enum {
	BCM59035_A1LDOCTRL = 0,
	BCM59035_A2LDOCTRL,
	BCM59035_R1LDOCTRL,
	BCM59035_R2LDOCTRL,
	BCM59035_H1LDOCTRL,
	BCM59035_H2LDOCTRL,
	BCM59035_ILDOCTRL,
	BCM59035_M1LDOCTRL,
	BCM59035_M2LDOCTRL,
	BCM59035_LLDOCTRL,
	BCM59035_SLDOCTRL,
	BCM59035_LV1LDOCTRL,
	BCM59035_LV2LDOCTRL,
	BCM59035_AX1LDOCTRL,
	BCM59035_AX2LDOCTRL,
	BCM59035_INVALIDLDO
} BCM59035_LDO_t;

void BCM59035_UserPONKEYLock(u8 setToOn);
void BCM59035_DRV_BBCCtrl(BCM59035_BBCVS_t pmuBBCVSVal, BCM59035_BBCCS_t pmuBBCCSVal, BCM59035_BBCRS_t pmuBBCRSVal);
void BCM59035_DRV_BBCTimerReconfig(u8 onHrInterval, u8 onMinInterval, u8 offHrInterval, u8 offMinInterval);

#endif
