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




/*
*
*****************************************************************************
*
*  pmu_bcm59001.h
*
*  PURPOSE:
*
*  This file defines the interface to the Broadcom BCM59001 PMU chip.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( PMU_BCM59001_H )
#define PMU_BCM59001_H

/* ---- Include Files ---------------------------------------------------- */

#include <linux/broadcom/pmu_chip.h>

/* ---- Constants and Types ---------------------------------------------- */

#define BCM59001_NUM_INT_REG  5                          // Number of interrupt registers i.e. INT1-5
#define BCM59001_NUM_IRQ      (BCM59001_NUM_INT_REG * 8) // 5 bytewide interrupt bit registers

typedef enum {
   BCM59001_REGULATOR_ALDO1 = 0,     
   BCM59001_REGULATOR_ALDO2,     
   BCM59001_REGULATOR_RFLDO1,    
   BCM59001_REGULATOR_RFLDO2,    
   BCM59001_REGULATOR_HCLDO,     
   BCM59001_REGULATOR_USBLDO,    
   BCM59001_REGULATOR_IOLDO,     
   BCM59001_REGULATOR_MSLDO,     
   BCM59001_REGULATOR_LCLDO,     
   // BCM59001_REGULATOR_LVLDO,     IOSR drives LVLDO directly
   BCM59001_REGULATOR_SIMLDO,    
   BCM59001_REGULATOR_IOSR,      
   
   BCM59001_NUM_REGULATORS
} BCM59001_regulators_t;

/* Charger IDs */
typedef enum {
   BCM59001_CHARGER_MAIN = 0,
   BCM59001_CHARGER_USB,
   BCM59001_NUM_CHARGERS
} BCM59001_chargers_t;

#define BCM59001_I2C_BASE_ADDR    0x08

#define BCM59001_BIT_REG_OFF        0xaa
#define BCM59001_BIT_REG_ON         0x00
#define BCM59001_BIT_REG_ECO        0x55

/* Register addresses for direct accesses */

// PMU ID and ENV status regs
#define BCM59001_REG_PMUID       0x00     // R    revision[7:4] ID[3:0]
#define BCM59001_REG_ENV1        0x01     // R    environment monitor [7:0]
   #define BCM59001_BIT_ENV1_UBMBC  (1<<0)   // USB voltage higher than Main Battery voltage 
   #define BCM59001_BIT_ENV1_CGPD   (1<<1)   // wall adapter charger presence if set
   #define BCM59001_BIT_ENV1_UBPD   (1<<2)   // USB charger presence detected if set
   #define BCM59001_BIT_ENV1_MBWV   (1<<3)   // Main Battery voltage higher than working voltage
   #define BCM59001_BIT_ENV1_CGHC   (1<<4)   // Wall Adapter Charger input Over-Voltage (if set)
   #define BCM59001_BIT_ENV1_BBLOWB (1<<5)   // Backup Battery input low-voltage
   #define BCM59001_BIT_ENV1_BBMBC  (1<<6)   // Main Battery voltage higher than Backup Battery (if set)
   #define BCM59001_BIT_ENV1_CGMBC  (1<<7)   // Wall Adapter Charger voltage higher than Main Battery voltage (if set)


#define BCM59001_REG_ENV2        0x02     // R    environment monitor [3:0]
   #define BCM59001_BIT_ENV2_PONKY  (1<<0)   // Power On Key button: 0=depressed, 1=released after debounce
   #define BCM59001_BIT_ENV2_ACD    (1<<1)   // Accessory Detection: 0=depressed, 1=released after debounce
   #define BCM59001_BIT_ENV2_PHFD   (1<<2)   // Phone Hands Free Detection: 0=depressed, 1=released after debounce 
   #define BCM59001_BIT_ENV2_THD    (1<<3)   // Die Over-Temperature detection: 0 = Die Temp is OK, 1 = Die Over-Temperature 
   #define BCM59001_BIT_ENV2_UBHC   (1<<4)   // USB input over-voltage if set

   #define BCM59001_BIT_ENV2_USB_ENTLVL   (1<<5)   // Main battery working voltage comparator when USB charger is
                                                   // present. This bit will be set to �1� when Main battery voltage is
                                                   // above the level specified in USBWVS.

#define BCM59001_REG_ENV3        0x03     // R/W  environment monitor [0:0]
   #define BCM59001_REG_ENV3_TEMPOK (1<<0)   // Temperature OK from the Host. If the Host writes a zero, the
                                             // battery charger (WAC or USB) will immediately shut-down.
                                             // The Main Battery temperature information is fed to the Host
                                             // controller. The PMU does not have direct access to this information.


#define BCM59001_REG_HOSTACT     0x04     // R/W  host action [2:0] (watchdog)
   #define BCM59001_BIT_HOSTACT_WDT_CLR      (1<<0)   // Watchdog Clear Bit. Writing a 1 will clear Watchdog timer (bit
                                                      // will be self-cleared after the write operation). Once the
                                                      // watchdog is enabled the host controller must write this bit
                                                      // within 8 seconds to avoid the watchdog to generate a reset.
   
   #define BCM59001_BIT_HOSTACT_WDT_ON       (1<<1)   // Watchdog Enable bit. Writing a 1? will Enable the Watchdog timer.
   #define BCM59001_BIT_HOSTACT_HOSTDICOFF   (1<<2)   // PMU Turned off by Host. Writing a 1 will turn off the PMU (bit
                                                      // will be self-cleared after the write operation). When the PMU is
                                                      // turned off the output voltages will be disabled hence the Host
                                                      // will be turned off as well.
                                                
// Interrupts
#define BCM59001_REG_INT1           0x05     // R&C  interrupt [6:0]
#define BCM59001_REG_INT2           0x06     // R&C  interrupt [7:0]
#define BCM59001_REG_INT3           0x07     // R&C  interrupt [7:0]
#define BCM59001_REG_INT4           0x08     // R&C  interrupt [7:0]
#define BCM59001_REG_INT5           0x09     // R&C  interrupt [4:0]

#define BCM59001_REG_INT1M          0x0A     // R/W  interrupt mask [6:0]
   #define BCM59001_BIT_INT1_PONKEYR   (1<<0)   // Power On Key released (Rising Event) if set to 1. Cleared after Reading
   #define BCM59001_BIT_INT1_PONKEYF   (1<<1)   // Power On Key depressed (Falling Event) if set to 1. Cleared after Reading
   #define BCM59001_BIT_INT1_PONKEYH   (1<<2)   // This flag will be set if PONKY is held for a time longer then
                                                // the value specified in PONKYHOLD debounce register (PONKYDB)
   #define BCM59001_BIT_INT1_RTC60S    (1<<3)   // This bit is set every 60 sec by the Real Time Clock block, This
                                                // bit is Cleared after Reading
   #define BCM59001_BIT_INT1_RTCA1     (1<<4)   // Alarm 1 generates, if set, by the Real Time Clock. This bit is
                                                // Cleared after Reading
   #define BCM59001_BIT_INT1_RTCADJ    (1<<6)   // When this bit is �1� the Real Time Clock needs adjustment (i.e.
                                                // starting from no power condition) This bit is Cleared after Reading
   #define BCM59001_BIT_INT1_RTC1S     (1<<7)   // RTC periodic 1s event if set

#define BCM59001_REG_INT2M           0x0B    // R/W  interrupt mask [7:0]
   #define BCM59001_BIT_INT2_CHGINS    (1<<0)   // wall adaptor inserted
   #define BCM59001_BIT_INT2_CHGRM     (1<<1)   // wall adaptor removed
   #define BCM59001_BIT_INT2_CHGERR    (1<<2)   // wall adaptor charger voltage too high
   #define BCM59001_BIT_INT2_CHGEOC    (1<<3)   // battery fully charged by wall adaptor
   #define BCM59001_BIT_INT2_USBINS    (1<<4)   // USB charger inserted
   #define BCM59001_BIT_INT2_USBRM     (1<<5)   // USB charger removed
   #define BCM59001_BIT_INT2_USBEOC    (1<<6)   // battery fully charged by USB charger
   #define BCM59001_BIT_INT2_MBCCHGERR (1<<7)   // battery charging timeout error

#define BCM59001_REG_INT3M           0x0C    // R/W  interrupt mask [7:0]
   #define BCM59001_BIT_INT3_ACDINS    (1<<0)   // accessory inserted
   #define BCM59001_BIT_INT3_ACDRM     (1<<1)   // accessory removed
   #define BCM59001_BIT_INT3_PHFDRLS   (1<<2)   // PHFD button released. Set when PHFD input transitions from LOW to HIGH (>0.3V)
   #define BCM59001_BIT_INT3_PHFDINS   (1<<3)   // This bit is set to 1 when Phone Hands Free Input is Inserted. This
                                                // corresponds to PHFD level to be below the upper threshold (typically
                                                // 1.3V). This bit is Cleared after Reading.
   #define BCM59001_BIT_INT3_PHFDRM    (1<<4)   // This bit is set to 1 when Phone Hands Free Input is Removed. This
                                                // corresponds to PHFD level to be above the upper threshold (typically
                                                // 1.3V). This bit is Cleared after Reading
   #define BCM59001_BIT_INT3_PHFDPRS   (1<<5)   // If the PHFD switch, is depressed a low level (i.e. PHFD is
                                                // grounded) will be present at the PHFD input, this condition is 
                                                // indicated in the register INT3, bit-5 (PHFDPRS) set to 1
   #define BCM59001_BIT_INT3_LOWBAT    (1<<6)   // This bit is set to 1 when the Main Battery voltage VMBAT is below
                                                // the Low Battery threshold (VMBWV + 0.1V) VMBWV is defined by the
                                                // OTP register MBCCWVS[3:0].
   
#define BCM59001_REG_INT4M          0x0D     // R/W  interrupt mask [7:0]
   #define BCM59001_BIT_INT4_A1OVRI    (1<<0)   // Over-Current condition on ALDO1
   #define BCM59001_BIT_INT4_A2OVRI    (1<<1)   // Over-Current condition on ALDO2
   #define BCM59001_BIT_INT4_R1OVRI    (1<<2)   // Over-Current condition on RFDO1
   #define BCM59001_BIT_INT4_R2OVRI    (1<<3)   // Over-Current condition on RFLDO1
   #define BCM59001_BIT_INT4_HOVRI     (1<<4)   // Over-Current condition on HCLDO
   #define BCM59001_BIT_INT4_UOVRI     (1<<5)   // Over-Current condition on USBLDO
   #define BCM59001_BIT_INT4_IOVRI     (1<<6)   // Over-Current condition on IOLDO
   #define BCM59001_BIT_INT4_MOVRI     (1<<7)   // Over-Current condition on MSLDO


#define BCM59001_REG_INT5M          0x0E     // R/W  interrupt mask [4:0]
   #define BCM59001_BIT_INT5_LOVRI     (1<<0)   // Over-Current condition on LCLDO
   #define BCM59001_BIT_INT5_LVOVRI    (1<<1)   // Over-Current condition on LVLDO
   #define BCM59001_BIT_INT5_SOVRI     (1<<2)   // Over-Current condition on SIMLDO
   #define BCM59001_BIT_INT5_CSROVRI   (1<<3)   // Over-Current condition on CSR
   #define BCM59001_BIT_INT5_IOSROVRI  (1<<4)   // Over-Current condition on IOSR
   #define BCM59001_BIT_INT5_USBERR    (1<<5)   // USB Input over-voltage error (> 7V)

// LDO controls
#define BCM59001_REG_ALDOCTRL        0x0F       // R/W  ldo control aldo1[2:0] aldo2[6:4]
   #define BCM59001_VAL_ALDO1_2_5V      0x00        
   #define BCM59001_VAL_ALDO1_2_6V      0x01        
   #define BCM59001_VAL_ALDO1_2_7V      0x02        
   #define BCM59001_VAL_ALDO1_2_8V      0x03        
   #define BCM59001_VAL_ALDO1_2_9V      0x04        
   #define BCM59001_VAL_ALDO1_3_1V      0x05        
   #define BCM59001_VAL_ALDO1_3_0V      0x07       // default 111
   #define BCM59001_VAL_ALD02_2_5V     (0x00<<4)    
   #define BCM59001_VAL_ALD02_2_6V     (0x01<<4)    
   #define BCM59001_VAL_ALD02_2_7V     (0x02<<4)    
   #define BCM59001_VAL_ALD02_2_8V     (0x03<<4)    
   #define BCM59001_VAL_ALD02_2_9V     (0x04<<4)    
   #define BCM59001_VAL_ALD02_3_1V     (0x05<<4)    
   #define BCM59001_VAL_ALD02_3_0V     (0x07<<4)   // default 111

#define BCM59001_REG_RFDOCTRL        0x10       // R/W  ldo control rf1[2:0] rf2[6:4]

   #define BCM59001_VAL_RFLD01_2_5V     0x00       
   #define BCM59001_VAL_RFLD01_2_6V     0x01       
   #define BCM59001_VAL_RFLD01_2_7V     0x07       // default 111
   #define BCM59001_VAL_RFLD01_2_8V     0x03        
   #define BCM59001_VAL_RFLD01_2_9V     0x04        
   #define BCM59001_VAL_RFLD01_3_0V     0x05        
   #define BCM59001_VAL_RFLD02_2_5V    (0x00<<4)    
   #define BCM59001_VAL_RFLD02_2_6V    (0x01<<4)    
   #define BCM59001_VAL_RFLD02_2_7V    (0x07<<4)   // default 111
   #define BCM59001_VAL_RFLD02_2_8V    (0x03<<4)    
   #define BCM59001_VAL_RFLD02_2_9V    (0x04<<4)    
   #define BCM59001_VAL_RFLD02_3_0V    (0x05<<4)    

#define BCM59001_REG_HCUSBDOCTRL     0x11       // R/W  ldo control hc[2:0] usb[6:4]
   #define BCM59001_VAL_HCLDO_2_5V      0x00
   #define BCM59001_VAL_HCLDO_2_6V      0x01
   #define BCM59001_VAL_HCLDO_2_7V      0x02
   #define BCM59001_VAL_HCLDO_2_8V      0x03
   #define BCM59001_VAL_HCLDO_2_9V      0x04
   #define BCM59001_VAL_HCLDO_3_2V      0x05
   #define BCM59001_VAL_HCLDO_3_1V      0x06
   #define BCM59001_VAL_HCLDO_3_0V      0x07       // default 111
   #define BCM59001_VAL_USBLDO_2_8V     (0x00<<4)
   #define BCM59001_VAL_USBLDO_2_9V     (0x01<<4)
   #define BCM59001_VAL_USBLDO_3_0V     (0x02<<4)
   #define BCM59001_VAL_USBLDO_3_1V     (0x03<<4)
   #define BCM59001_VAL_USBLDO_3_2V     (0x04<<4)
   #define BCM59001_VAL_USBLDO_3_4V     (0x06<<4)
   #define BCM59001_VAL_USBLDO_3_3V     (0x07<<4)  // default 111

#define BCM59001_REG_IOLDOCTRL       0x12    // R/W  ldo control  io[4:0]
   #define BCM59001_VAL_IOLDO_1_3V      0x00
   #define BCM59001_VAL_IOLDO_1_4V      0x01
   #define BCM59001_VAL_IOLDO_1_5V      0x02
   #define BCM59001_VAL_IOLDO_1_6V      0x03
   #define BCM59001_VAL_IOLDO_1_7V      0x04
   #define BCM59001_VAL_IOLDO_1_8V      0x08
   #define BCM59001_VAL_IOLDO_1_9V      0x09
   #define BCM59001_VAL_IOLDO_2_0V      0x0a
   #define BCM59001_VAL_IOLDO_2_1V      0x0b
   #define BCM59001_VAL_IOLDO_2_2V      0x0c
   #define BCM59001_VAL_IOLDO_2_3V      0x10
   #define BCM59001_VAL_IOLDO_2_4V      0x11
   #define BCM59001_VAL_IOLDO_2_5V      0x12
   #define BCM59001_VAL_IOLDO_2_6V      0x13
   #define BCM59001_VAL_IOLDO_2_7V      0x14
   #define BCM59001_VAL_IOLDO_2_8V      0x18
   #define BCM59001_VAL_IOLDO_2_9V      0x19
   #define BCM59001_VAL_IOLDO_3_1V      0x1b
   #define BCM59001_VAL_IOLDO_3_2V      0x1c
   #define BCM59001_VAL_IOLDO_3_0V      0x1f    // default 11111


#define BCM59001_REG_MSLDOCTRL       0x13    // R/W  ldo control  ms[4:0]
   #define BCM59001_VAL_MSLDO_1_3V      0x00
   #define BCM59001_VAL_MSLDO_1_4V      0x01
   #define BCM59001_VAL_MSLDO_1_5V      0x02
   #define BCM59001_VAL_MSLDO_1_6V      0x03
   #define BCM59001_VAL_MSLDO_1_7V      0x04
   #define BCM59001_VAL_MSLDO_1_8V      0x08
   #define BCM59001_VAL_MSLDO_1_9V      0x09
   #define BCM59001_VAL_MSLDO_2_0V      0x0a
   #define BCM59001_VAL_MSLDO_2_1V      0x0b
   #define BCM59001_VAL_MSLDO_2_2V      0x0c
   #define BCM59001_VAL_MSLDO_2_3V      0x10
   #define BCM59001_VAL_MSLDO_2_4V      0x11
   #define BCM59001_VAL_MSLDO_2_5V      0x12
   #define BCM59001_VAL_MSLDO_2_6V      0x13
   #define BCM59001_VAL_MSLDO_2_7V      0x14
   #define BCM59001_VAL_MSLDO_2_8V      0x1f    // default 11111
   #define BCM59001_VAL_MSLDO_2_9V      0x19
   #define BCM59001_VAL_MSLDO_3_0V      0x1a
   #define BCM59001_VAL_MSLDO_3_1V      0x1b
   #define BCM59001_VAL_MSLDO_3_2V      0x1c    

#define BCM59001_REG_LLDOCTRL        0x14    // R/W  ldo control  lc[4:0] lv[7:6]
   #define BCM59001_VAL_LCLDO_1_3V      0x00
   #define BCM59001_VAL_LCLDO_1_4V      0x01
   #define BCM59001_VAL_LCLDO_1_5V      0x02
   #define BCM59001_VAL_LCLDO_1_6V      0x03
   #define BCM59001_VAL_LCLDO_1_7V      0x04
   #define BCM59001_VAL_LCLDO_1_8V      0x08
   #define BCM59001_VAL_LCLDO_1_9V      0x09
   #define BCM59001_VAL_LCLDO_2_0V      0x0a
   #define BCM59001_VAL_LCLDO_2_1V      0x0b
   #define BCM59001_VAL_LCLDO_2_2V      0x0c
   #define BCM59001_VAL_LCLDO_2_3V      0x10
   #define BCM59001_VAL_LCLDO_2_4V      0x11
   #define BCM59001_VAL_LCLDO_2_5V      0x12
   #define BCM59001_VAL_LCLDO_2_6V      0x13
   #define BCM59001_VAL_LCLDO_2_7V      0x1f    // default 11111
   #define BCM59001_VAL_LCLDO_2_8V      0x18    
   #define BCM59001_VAL_LCLDO_2_9V      0x19
   #define BCM59001_VAL_LCLDO_3_0V      0x1a
   #define BCM59001_VAL_LCLDO_3_1V      0x1b
   #define BCM59001_VAL_LCLDO_3_2V      0x1c
   #define BCM59001_VAL_LVLDO_1_25V     (0x00<<6)
   #define BCM59001_VAL_LVLDO_1_3V      (0x01<<6)    
   #define BCM59001_VAL_LVLDO_1_4V      (0x02<<6)    
   #define BCM59001_VAL_LVLDO_1_5V      (0x03<<6)  // default 11
      
#define BCM59001_REG_SIMLDOCTRL      0x15    // R/W  ldo control  sim[4:0]
   #define BCM59001_VAL_SIMLDO_2_5V     0x00
   #define BCM59001_VAL_SIMLDO_3_0V     0x01    // 0x02 is also 3V
   #define BCM59001_VAL_SIMLDO_1_8V     0x03    // default 11
   

// LDO opmodes: the next 11 registers are for LDO to PC2,PC1 line relationships
// bit layouts for PC2,PC1 status are: 0,0[1:0] 0,1[3:2] 1,0[5:4] 1,1[7:6]
#define BCM59001_REG_A1OPMODCTRL     0x16    // R/W  aldo1
#define BCM59001_REG_A2OPMODCTRL     0x17    // R/W  aldo2

#define BCM59001_REG_R1OPMODCTRL     0x18    // R/W  rfldo1
   #define BCM59001_VAL_RFLDO1_OFF      0xaa    // turn off all power rails for LCD and camera
   #define BCM59001_VAL_RFLDO1_LOWPOWER 0x55    // switch to lower power mode for  power rails for LCD and camera
   #define BCM59001_VAL_RFLDO1_ON       0x00    // switch to on for power rails to LCD and camera

#define BCM59001_REG_R2OPMODCTRL     0x19    // R/W  rfldo2

#define BCM59001_REG_HOPMODCTRL      0x1A    // R/W  hcldo
   #define BCM59001_VAL_HCLDO_OFF       0xaa    // turn off all power rails for WLAN 3.2
   #define BCM59001_VAL_HCLDO_ON        0x00    // turn on all power rails for WLAN

#define BCM59001_REG_UOPMODCTRL      0x1B    // R/W  usbldo
#define BCM59001_REG_IOPMODCTRL      0x1C    // R/W  ioldo
#define BCM59001_REG_MOPMODCTRL      0x1D    // R/W  msldo
#define BCM59001_REG_LOPMODCTRL      0x1E    // R/W  lcldo
#define BCM59001_REG_LVOPMODCTRL     0x1F    // R/W  lvldo
#define BCM59001_REG_SOPMODCTRL      0x20    // R/W  simldo

// Core switching regulator controls
#define BCM59001_REG_CSRCTRL1        0x21    // R/W  core switching reg control [7:0]
#define BCM59001_REG_CSRCTRL2        0x22    // R/W  core switching reg control [7:0]
#define BCM59001_REG_CSRCTRL3        0x23    // R/W  core switching reg control [7:0]
#define BCM59001_REG_CSRCTRL4        0x24    // R/W  core switching reg control [7:0]
#define BCM59001_REG_CSRCTRL5        0x25    // R/W  core switching reg control [7:0]
#define BCM59001_REG_CSRCTRL6        0x26    // R/W  core switching reg control [7:0]
#define BCM59001_REG_CSRCTRL7        0x27    // R/W  core switching reg control [7:0]
#define BCM59001_REG_CSRCTRL8        0x28    // R/W  core switching reg control [7:0]
#define BCM59001_REG_CSRCTRL9        0x29    // R/W  core switching reg control [2:0]
                                        // bit layouts for PC2,PC1 status are: 0,0[1:0] 0,1[3:2] 1,0[5:4] 1,1[7:6]
#define BCM59001_REG_CSROPMODCTRL    0x2A    // R/W  CSR to PC2,PC1 line relationships

// IO switching regulator controls
#define BCM59001_REG_IOSRCTRL1       0x2B    // R/W  io switching reg control [7:0]
#define BCM59001_REG_IOSRCTRL2       0x2C    // R/W  io switching reg control [7:0]
#define BCM59001_REG_IOSRCTRL3       0x2D    // R/W  io switching reg control [7:0]
#define BCM59001_REG_IOSRCTRL4       0x2E    // R/W  io switching reg control [7:0]
#define BCM59001_REG_IOSRCTRL5       0x2F    // R/W  io switching reg control [7:0]
#define BCM59001_REG_IOSRCTRL6       0x30    // R/W  io switching reg control [7:0]
#define BCM59001_REG_IOSRCTRL7       0x31    // R/W  io switching reg control [7:0]
#define BCM59001_REG_IOSRCTRL8       0x32    // R/W  io switching reg control [7:0]
#define BCM59001_REG_IOSRCTRL9       0x33    // R/W  io switching reg control [2:0]
                                        // bit layouts for PC2,PC1 status are: 0,0[1:0] 0,1[3:2] 1,0[5:4] 1,1[7:6]
#define BCM59001_REG_IOSROPMODCTRL   0x34    // R/W  IOSR to PC2,PC1 line relationships
   #define BCM59001_VAL_IOSR_OFF        0xaa    // turn off all power rails to WLAN 1.8
   #define BCM59001_VAL_IOSR_ON         0x00    // turn on all power rails to WLAN 1.8

// Charger controls
#define BCM59001_REG_SRCTRL          0x35    // R/W  switching reg common control [5:0]
#define BCM59001_REG_MBCCTRL1        0x36    // R/W  main battery charger control [6:0]
#define BCM59001_REG_MBCCTRL2        0x37    // R/W  main battery charger control [7:0]

   #define BCM59001_BIT_MBCCTRL2_MBCHOSTRC  (0x01<<7)  // enable rapid charge mode
   #define BCM59001_BIT_MBCCTRL2_MBCHOSTEN  (0x01<<6)  // MBC enabled from Host (overwrite the 4 enable
                                                       // signals from MBC controller to MBC) 
#define BCM59001_REG_MBCCTRL3        0x38    // R/W  main battery charger control [7:0]
   #define BCM59001_VAL_TRICKLE_3_2V   (0x7<<4) // trickle charge mode threshold
   #define BCM59001_VAL_TRICKLE_3_3V   (0x8<<4) // trickle charge mode threshold
   #define BCM59001_VAL_TRICKLE_3_4V   (0x9<<4) // trickle charge mode threshold
   #define BCM59001_VAL_TRICKLE_3_5V   (0xa<<4) // trickle charge mode threshold
   #define BCM59001_VAL_TRICKLE_3_6V   (0xb<<4) // trickle charge mode threshold

#define BCM59001_REG_MBCCTRL4        0x39    // R/W  main battery charger control [7:0]
   #define BCM59001_VAL_TC1_600MA      (0x00)   // trickle charge mode 600mA
   #define BCM59001_VAL_TC1_500MA      (0x01)   // trickle charge mode 500mA
   #define BCM59001_VAL_TC1_400MA      (0x02)   // trickle charge mode 400mA
   #define BCM59001_VAL_TC1_300MA      (0x03)   // trickle charge mode 300mA
   #define BCM59001_VAL_TC1_200MA      (0x04)   // trickle charge mode 200mA
   #define BCM59001_VAL_TC1_100MA      (0x05)   // trickle charge mode 100mA
   #define BCM59001_VAL_TC1_50MA       (0x06)   // trickle charge mode 50mA
   #define BCM59001_VAL_RC1_600MA      (0x00<<4)// rapid charge mode 600mA
   #define BCM59001_VAL_RC1_500MA      (0x01<<4)// rapid charge mode 500mA
   #define BCM59001_VAL_RC1_400MA      (0x02<<4)// rapid charge mode 400mA
   #define BCM59001_VAL_RC1_300MA      (0x03<<4)// rapid charge mode 300mA
   #define BCM59001_VAL_RC1_200MA      (0x04<<4)// rapid charge mode 200mA
   #define BCM59001_VAL_RC1_100MA      (0x05<<4)// rapid charge mode 100mA
   #define BCM59001_VAL_RC1_50MA       (0x06<<4)// rapid charge mode 50mA

#define BCM59001_REG_MBCCTRL5        0x3A    // R/W  main battery charger control [7:0]
#define BCM59001_REG_MBCCTRL6        0x3B    // R/W  main battery charger control [6:0]
#define BCM59001_REG_LDOPWRGRP1      0x3C    // R/W  LDO power-up grouping control [7:0]
#define BCM59001_REG_BBCCTRL         0x3D    // R/W  backup battery charger control [5:0]

// RTC
#define BCM59001_REG_RTCSC           0x3E    // R/W  real time clock seconds [5:0]
#define BCM59001_REG_RTCMN           0x3F    // R/W  real time clock minutes [5:0]
#define BCM59001_REG_RTCHR           0x40    // R/W  real time clock hours [4:0]
#define BCM59001_REG_RTCWD           0x41    // R/W  real time clock weekday [2:0]
#define BCM59001_REG_RTCDT           0x42    // R/W  real time clock day [4:0]
#define BCM59001_REG_RTCMT           0x43    // R/W  real time clock month [3:0]
#define BCM59001_REG_RTCYR           0x44    // R/W  real time clock year [7:0]
#define BCM59001_REG_RTCSC_A1        0x45    // R/W  alarm clock 1 seconds [5:0]
#define BCM59001_REG_RTCMN_A1        0x46    // R/W  alarm clock 1 minutes [5:0]
#define BCM59001_REG_RTCHR_A1        0x47    // R/W  alarm clock 1 hours [4:0]
#define BCM59001_REG_RTCWD_A1        0x48    // R/W  alarm clock 1 weekday [6:0]
#define BCM59001_REG_RTCDT_A1        0x49    // R/W  alarm clock 1 day [4:0]
#define BCM59001_REG_RTCMT_A1        0x4A    // R/W  alarm clock 1 month [3:0]
#define BCM59001_REG_RTCYR_A1        0x4B    // R/W  alarm clock 1 year [7:0]
#define BCM59001_REG_RTCSC_A2        0x4C    // R/W  alarm clock 2 seconds [5:0]
#define BCM59001_REG_RTCMN_A2        0x4D    // R/W  alarm clock 2 minutes [5:0]
#define BCM59001_REG_RTCHR_A2        0x4E    // R/W  alarm clock 2 hours [4:0]
#define BCM59001_REG_RTCWD_A2        0x4F    // R/W  alarm clock 2 weekday [6:0]
#define BCM59001_REG_RTCDT_A2        0x50    // R/W  alarm clock 2 day [4:0]
#define BCM59001_REG_RTCMT_A2        0x51    // R/W  alarm clock 2 month [3:0]
#define BCM59001_REG_RTCYR_A2        0x52    // R/W  alarm clock 2 year [7:0]

// PWM and LED controls
#define BCM59001_REG_PWMLEDCTRL1     0x53    // R/W  pwm led control [3:0] control reg for output1
#define BCM59001_REG_PWMLEDCTRL2     0x54    // R/W  pwm led control [7:0]
#define BCM59001_REG_PWMLEDCTRL3     0x55    // R/W  pwm led control [7:0]
#define BCM59001_REG_PWMLEDCTRL4     0x56    // R/W  pwm led control [7:0]
#define BCM59001_REG_PWMLEDCTRL5     0x57    // R/W  pwm led control [6:0]
#define BCM59001_REG_PWMLEDCTRL6     0x58    // R/W  pwm led control [3:0] control reg for output2
#define BCM59001_REG_PWMLEDCTRL7     0x59    // R/W  pwm led control [7:0]
#define BCM59001_REG_PWMLEDCTRL8     0x5A    // R/W  pwm led control [7:0]
#define BCM59001_REG_PWMLEDCTRL9     0x5B    // R/W  pwm led control [7:0]
#define BCM59001_REG_PWMLEDCTRL10    0x5C    // R/W  pwm led control [3:0]
   #define BCM59001_VAL_LED_ALWAYS_ON       (0x38)

// Debounce settings
#define BCM59001_REG_PONKEYBDB       0x5D    // R/W  power-on key debounce [2:0], power-on key lock [6]
   #define BCM59001_VAL_KEYLOCK         (0x01<<6)
#define BCM59001_REG_ACDDB           0x5E    // R/W  acd debounce [3:0], power-on key delay [6:4]
   #define BCM59001_VAL_PONKEYDEL_MASK     0x07
   #define BCM59001_VAL_PONKEYDEL_SHIFT    4
   #define BCM59001_VAL_PONKEYDEL_NO_DELAY (0x00<<4)
   #define BCM59001_VAL_PONKEYDEL_500MS    (0x01<<4)
   #define BCM59001_VAL_PONKEYDEL_1s       (0x02<<4)
   #define BCM59001_VAL_PONKEYDEL_2s       (0x03<<4)
   #define BCM59001_VAL_PONKEYDEL_4s       (0x04<<4)
   #define BCM59001_VAL_PONKEYDEL_8s       (0x05<<4)

#define BCM59001_REG_PHFDDB          0x5F    // R/W  phfd debounce [3:0]
   #define BCM59001_VAL_PHFDDDB_400MS   (0x03)
   #define BCM59001_VAL_PHFDDDB_200MS   (0x02)
   #define BCM59001_VAL_PHFDDDB_100MS   (0x01)
   #define BCM59001_VAL_PHFDRDB_400MS   (0x03<<2)
   #define BCM59001_VAL_PHFDRDB_200MS   (0x02<<2)
   #define BCM59001_VAL_PHFDRDB_100MS   (0x01<<2)

// Fuel gauge registers
#define BCM59001_REG_FGACCM1         0x60    // R&C  fuel gauge accumulation [7:0]
#define BCM59001_REG_FGACCM2         0x61    // R&C  fuel gauge accumulation [7:0]
#define BCM59001_REG_FGACCM3         0x62    // R&C  fuel gauge accumulation [7:0]
#define BCM59001_REG_FGACCM4         0x63    // R&C  fuel gauge accumulation [1:0]
#define BCM59001_REG_FGCNT1          0x64    // R&C  fuel gauge sample count [7:0]
#define BCM59001_REG_FGCNT2          0x65    // R&C  fuel gauge sample count [3:0]
#define BCM59001_REG_FGSMPL1         0x66    // R&C  fuel gauge current sample [7:0]
#define BCM59001_REG_FGSMPL2         0x67    // R&C  fuel gauge current sample [5:0]
#define BCM59001_REG_FGCTRL1         0x68    // R/W  fuel gauge control [7:0]
#define BCM59001_REG_FGCTRL1_FGHOSTEN  (1<<0)    // Fuel gauge enable. 1 = enable, 0 = disable
#define BCM59001_REG_FGCTRL2         0x69    // R/W  fuel gauge control [7:0]

// Test registers
#define BCM59001_REG_FGCICCTRL       0x6A    // R/W  ATE writable [4:0]
#define BCM59001_REG_FGTRIMGN1_1     0x6B    // R    ATE reaeable [7:0]
#define BCM59001_REG_FGTRIMGN1_2     0x6C    // R    ATE reaeable [5:0]
#define BCM59001_REG_FGTRIMGN2_1     0x6D    // R    ATE reaeable [7:0]
#define BCM59001_REG_FGTRIMGN2_2     0x6E    // R    ATE reaeable [5:0]
#define BCM59001_REG_FGTRIMIDEALDELTA1   0x6F    // R/W  ATE writable [7:0]
#define BCM59001_REG_FGTRIMIDEALDELTA2   0x70    // R/W  ATE writable [5:0]

// PLL registers
#define BCM59001_REG_PLLADCCLKSEL    0x71    // R/W  adc clock select [1:0]
#define BCM59001_REG_PLLN            0x72    // R/W  pll divider [4:0]
#define BCM59001_REG_PLLVREGTRM      0x73    // R/W  pll regulator trim [1:0]
#define BCM59001_REG_PLLFCURVE       0x74    // R/W  pll tuning curve [2:0]
#define BCM59001_REG_PLLCP           0x75    // R/W  pll current select [2:0]
#define BCM59001_REG_PLLCLKSEL       0x76    // R/W  pll frequency select [1:0]
#define BCM59001_REG_PLLCTRL         0x77    // R/W  pll control [7:0]

// OTP registers
#define BCM59001_REG_OTPMBCTRM       0x78    // R/W  main charger trim code [5:0]
#define BCM59001_REG_OTPUSBCCTRM     0x79    // R/W  Main battery charger current trimming register [3:0]
#define BCM59001_REG_OTPCGHCVS       0x7A    // R/W  comparator voltage select [1:0]
#define BCM59001_REG_OTPFGTRIMCTL    0x7B    // R/W  fuel gauge trim control [5:0]
#define BCM59001_REG_OTPBBMBVS       0x7C    // R/W  main/backup battery comparator [1:0]
#define BCM59001_REG_OTPMBWChys      0x7D    // R/W  main battery working comparator hysteresis[2:0]
#define BCM59001_REG_OTPCGMBhys      0x7E    // R/W  charger & main battery comparator hysteresis[2:0]
#define BCM59001_REG_OTPUBPDhys      0x7F    // R/W  usb present detector hysteresis[2:0]
#define BCM59001_REG_OTPMBPDhys      0x80    // R/W  main battery present detector hysteresis[2:0]
#define BCM59001_REG_OTPCHPDhys      0x81    // R/W  charger high comparator hysteresis[2:0]

// Analog test registers
#define BCM59001_REG_TSTRECH         0x82    // R/W  accessory recognition high voltage [2:0]
   #define BCM59001_VAL_PHFD_HIGH_1_3V  0x00
   #define BCM59001_VAL_PHFD_HIGH_1_5V  0x01
   #define BCM59001_VAL_PHFD_HIGH_1_7V  0x02
   #define BCM59001_VAL_PHFD_HIGH_1_9V  0x03
   #define BCM59001_VAL_PHFD_HIGH_2_1V  0x04

#define BCM59001_REG_TSTRECL         0x83    // R/W  accessory recognition low voltage [2:0]
#define BCM59001_REG_OTPUBMBsel      0x84    // R/W  accessory recognition high voltage hysteresis[2:0]
#define BCM59001_REG_TSTUBHChys      0x85    // R/W  accessory recognition high voltage hysteresis[2:0]
#define BCM59001_REG_TSTTHS          0x86    // R/W  high temperature sensor control [5:0]
#define BCM59001_REG_TSTBBHCVS       0x87    // R/W  backup battery high comparator voltage control [1:0]
#define BCM59001_REG_TSTBBHChys      0x88    // R/W  backup battery high comparator hysteresis[2:0]
#define BCM59001_REG_TSTCGHChys      0x89    // R/W  charger high comparator hysteresis[2:0]
#define BCM59001_REG_TSTBBMBhys      0x8A    // R/W  main/backup battery comparator hysteresis[2:0]
#define BCM59001_REG_PDCMPSYN        0x8B    // R    presence detection/comparator status [7:0]
#define BCM59001_REG_TSTMBC1         0x8C    // R/W  main battery charger test[7:0]
#define BCM59001_REG_TSTMBC2         0x8D    // R/W  main battery charger test[7:0]

// Mode registers
#define BCM59001_REG_PMUmode         0x8E    // R/W  pmu mode control [1:0]
#define BCM59001_REG_TPEN            0x8F    // R/W  enable test mux control [3:0]
#define BCM59001_REG_DIGDBG1         0x90    // R    digital debug state status [7:0]
#define BCM59001_REG_DIGDBG2         0x91    // R    digital debug status check [7:0]
#define BCM59001_REG_DIGDBG3         0x92    // R    digital debug power down status [7:0]
#define BCM59001_REG_FGCTRL3         0x93    // R/W  Fuel Gauge Control 3 register [3:1]
#define BCM59001_REG_FGCTRL3_FGCAL   (1<<1)  // Host to calibrate the fuel gauge. Set to 1 to start
                                             // the offset calibration process. This bit reset to 0
#define BCM59001_REG_FGCTRL3_FGRESET (1<<2)  // Fuel gauge reset. Set to 1 to resets the fuel gauge output registers.
#define BCM59001_REG_FGCTRL3_FGSNAPSHOT (1<<3)  // Fuel gauge snapshot. Set to 1 to latch the
                                                // accumulator and counter value before reading.

#define BCM59001_REG_FGOFFSET1       0x94    // R  Fuel Gauge offset register [7:0], lower 8 bits
#define BCM59001_REG_FGOFFSET2       0x95    // R  Fuel Gauge offset register [5:0]  upper 6 bits
#define BCM59001_REG_FGSMPLB1        0x96    // R  Fast output fuel gauge current sample, lower 8 bits
#define BCM59001_REG_FGSMPLB2        0x97    // R  Fast output fuel gauge current sample, upper 6 bits
#define BCM59001_REG_LDOPWRGRP2      0x98    // R/W  LDO power-up grouping control 2
#define BCM59001_REG_LDOPWRGRP3      0x99    // R/W  LDO power-up grouping control 3
#define BCM59001_REG_LDOPWRGRP4      0x9A    // R/W  LDO power-up grouping control 4

#define BCM59001_REG_TOTAL			BCM59001_REG_LDOPWRGRP4 + 1		// Total number of registers for BCM59001

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

#endif  /* PMU_BCM59001_H */

