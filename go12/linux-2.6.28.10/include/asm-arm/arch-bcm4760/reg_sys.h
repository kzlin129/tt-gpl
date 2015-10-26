/*****************************************************************************
* Copyright 2005 - 2009 Broadcom Corporation.  All rights reserved.
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
*  PURPOSE:
*     This file contains definitions for the system configuration control registers:
*
*  NOTES:
*
*****************************************************************************/


#if !defined( __ASM_ARCH_REG_SYS_H )
#define __ASM_ARCH_REG_SYS_H

/* ---- Include Files ---------------------------------------------------- */
#include <asm/arch/hardware.h>

/* ---- Constants and Types ---------------------------------------------- */

#define REG_SYS_IOCR0           __REG32( HW_SYS_BASE + 0x00 )  // IO control register0
#define REG_SYS_IOCR1           __REG32( HW_SYS_BASE + 0x04 )  // IO control register1
#define REG_SYS_SUCR            __REG32( HW_SYS_BASE + 0x08 )  // Startup mode register
#define REG_SYS_IOCR2           __REG32( HW_SYS_BASE + 0x0c )  // IO control register2
#define REG_SYS_IOCR3           __REG32( HW_SYS_BASE + 0x1c )  // IO control register3
#define REG_SYS_IOCR4           __REG32( HW_SYS_BASE + 0x20 )  // IO control register4
#define REG_SYS_PIDR            __REG32( HW_SYS_BASE + 0x10 )  // Product ID register
#define REG_SYS_DSPCTRL         __REG32( HW_SYS_BASE + 0x14 )  // DSP Control register
#define REG_SYS_PUMR            __REG32( HW_SYS_BASE + 0x18 )  // Power Up Mode register
#define REG_SYS_MCR             __REG32( HW_SYS_BASE + 0x40 )  // BootROM remap register
#define REG_SYS_MRR             __REG32( HW_SYS_BASE + 0x44 )  // BootROM restore register
#define REG_SYS_RAMCTRL         __REG32( HW_SYS_BASE + 0x48 )  // RAM Control register
#define REG_SYS_SECCTRL         __REG32( HW_SYS_BASE + 0x50 )  // Security Control register
#define REG_SYS_SECSTAT         __REG32( HW_SYS_BASE + 0x54 )  // Security Status register
#define REG_SYS_ANACR0          __REG32( HW_SYS_BASE + 0x80 )  // Analog configuration registers
#define REG_SYS_ANACR1          __REG32( HW_SYS_BASE + 0x84 )
#define REG_SYS_ANACR2          __REG32( HW_SYS_BASE + 0x88 )
#define REG_SYS_ANACR3          __REG32( HW_SYS_BASE + 0x8c )
#define REG_SYS_ANACR4          __REG32( HW_SYS_BASE + 0x90 )
#define REG_SYS_ANACR5          __REG32( HW_SYS_BASE + 0x94 )
#define REG_SYS_ANACR6          __REG32( HW_SYS_BASE + 0x98 )
#define REG_SYS_ANACR7          __REG32( HW_SYS_BASE + 0x9c )
#define REG_SYS_ANACR8          __REG32( HW_SYS_BASE + 0xa0 )
#define REG_SYS_ANACR9          __REG32( HW_SYS_BASE + 0xa4 )
#define REG_SYS_ANACR10         __REG32( HW_SYS_BASE + 0xa8 )
#define REG_SYS_ANACR11         __REG32( HW_SYS_BASE + 0xac )
#define REG_SYS_ANACR12         __REG32( HW_SYS_BASE + 0xb0 )
#define REG_SYS_ANACR13         __REG32( HW_SYS_BASE + 0xb4 )
#define REG_SYS_ANACR14         __REG32( HW_SYS_BASE + 0xb8 )
#define REG_SYS_ANACR15         __REG32( HW_SYS_BASE + 0xbc )
#define REG_SYS_IRDROP0         __REG32( HW_SYS_BASE + 0xc0 )
#define REG_SYS_IRDROP1         __REG32( HW_SYS_BASE + 0xc4 )
#define REG_SYS_IRDROP2         __REG32( HW_SYS_BASE + 0xc8 )

// REG_SYS_IOCR0 bits
#define REG_SYS_IOCR0_SPI_RXD           0x80000000      // Select SPI RXD MUX
                                                        // 0: SD1DAT0
                                                        // 1: LCDRE
#define REG_SYS_IOCR0_GPEN_PULL         0x40000000      // GPEN pull up/down during deep sleep
                                                        // 0: pull down
                                                        // 1: pull up (Not used in 2153)
#define REG_SYS_IOCR0_LCD_MASK          0x30000000      // lcd/spi
                                                        // 0: lcd interface pin select (Z80 interface)
                                                        // 1: lcd interface pin select (M68 interface)
                                                        // 2: spi interface pin select
                                                        // 3: spi interface pin select
#define REG_SYS_IOCR0_GPEN_MASK         0x0ffff000      // gpen/gpio/rf/tx/rx refer to pin mux table for info
#define REG_SYS_IOCR0_PCM               0x00000800      // 0:pcm, 1:gpio[51:48]
#define REG_SYS_IOCR0_BUZZER            0x00000400      // 0:gpio16 1: buzzer(periodic timer)
#define REG_SYS_IOCR0_BKLIGHT           0x00000200      // 0:gpio17 1: backlight
#define REG_SYS_IOCR0_GPIO_USB          0x00000100      // 0:gpio[43:38] 1:usb i/f
#define REG_SYS_IOCR0_IRDA_UARTB        0x00000080      // 0:irda 1:uartb
#define REG_SYS_IOCR0_GPIO_I2S          0x00000040      // gpio / i2s (pcmcia=0)
                                                        // 0:i2s pin select 1:gpio[47:44] pin select
#define REG_SYS_IOCR0_MSPRO_SD2         0x00000020      // MSPRO/SD2
                                                        // 0:SD2
                                                        // 1:MSPRO
#define REG_SYS_IOCR0_SPI_GPIO_MASK     0x00000018      // sdio/spi/gpio
                                                        // 0:SD1
                                                        // 1:spi interface pin select
                                                        // 2 or 3:gpio[37:32]
#define REG_SYS_IOCR0_UARTA_DAI         0x0000002       // UART-A / DAI (PCMCIA =0 )
                                                        // 0:UART_A DPDCD pin select
                                                        // 1:Monitor Clock enable on DPDCD
#define REG_SYS_IOCR0_CAM_TVO           0x0000001       // camera/TVO dac pin select
                                                        // 0:camera 1:TVO

// REG_SYS_IOCR1 bits
// Selecting I2S mode means that GPIO[31:26] take on special meanings:
#define REG_SYS_IOCR1_I2S               0x00100000      // 0:gpio[27] pin select 1:i2s output pin select
#define REG_SYS_IOCR1_I2S_RXDI          0x00080000      // gpio31/RXDI
#define REG_SYS_IOCR1_I2S_RXDQ          0x00040000      // gpio30/RXDQ
#define REG_SYS_IOCR1_I2S_RXDST         0x00020000      // gpio29/RXDST
#define REG_SYS_IOCR1_I2S_RXDCK         0x00010000      // gpio28/RXDCK
                                                        // gpio27/DAILR
                                                        // gpio26/DAID

// REG_SYS_IOCR2 bits
#define REG_SYS_IOCR2_RSVD31            0x80000000      // reserved
#define REG_SYS_IOCR2_LCD3VOLT          0x40000000      // lcd i/f 3 volt pin 0:not selected  1:selected
#define REG_SYS_IOCR2_SIM3VOLT          0x20000000      // sim i/f 3 volt pin 0:not selected  1:selected
#define REG_SYS_IOCR2_RF3VOLT           0x10000000      // rf i/f 3 volt pin 0:not selected  1:selected
#define REG_SYS_IOCR2_CAM3VOLT          0x08000000      // camera i/f 3 volt pin 0:not selected  1:selected
#define REG_SYS_IOCR2_MEM4MA            0x04000000      // Memory interface 7mA/4mA drive select
                                                        // For all memories except MBCK and SDMCLK
                                                        // 1:4mA
                                                        // 0:7mA
#define REG_SYS_IOCR2_MBCK4MA           0x02000000      // MBCK-4mA Memory interface 7mA/4mA drive select
                                                        // 1:4mA
                                                        // 0:7mA
#define REG_SYS_IOCR2_SDMCLK4MA         0x01000000      // SDMCLK-4mA Memory interface 7mA/4mA drive select
                                                        // 1:4mA
                                                        // 0:7mA
#define REG_SYS_IOCR2_MSPRO_FBCK        0x00800000      // MSPRO feedback clock
                                                        // 0:feedback clock from pad
                                                        // 1:feedback using delayed clock (not used in 2153)
#define REG_SYS_IOCR2_SPI_MSPRO         0x00400000      // SPI/MSPRO
                                                        // 0:SPI interface selected
                                                        // 1:MSPRO interface selected (not used in 2153)
#define REG_SYS_IOCR2_SD2DAT_PULL       0x00300000      // SD2DAT pull
                                                        // 0:SD2DAT pad no pull
                                                        // 1:MSD2DAT pad pull up
                                                        // 2:MSD2DAT pad pull down
                                                        // 3:illegal setting will cause leakage
#define REG_SYS_IOCR2_SD2CMD_PULL       0x000c0000      // SD2CMD pull
                                                        // 0:SD2CMD pad no pull
                                                        // 1:MSD2CMD pad pull up
                                                        // 2:MSD2CMD pad pull down
                                                        // 3:illegal setting will cause leakage
#define REG_SYS_IOCR2_SD2CK_PULL        0x00030000      // SD2CK pull
                                                        // 0:SD2CK pad no pull
                                                        // 1:MSD2CK pad pull up
                                                        // 2:MSD2CK pad pull down
                                                        // 3:illegal setting will cause leakage
#define REG_SYS_IOCR2_USBN_PDN          0x00000080      // USB-PDN (USBN line pulldown)
                                                        // 1:USBN interface pin PDN select
                                                        // 0:pin PDN not selected
#define REG_SYS_IOCR2_USBP_PDN          0x00000040      // USB-PDN (USBP line pulldown)
                                                        // 1:USBP interface pin PDN select
                                                        // 0:pin PDN not selected
#define REG_SYS_IOCR2_USB_SUSPEND       0x00000020      // USB Suspend
                                                        // 1:Force USB suspend
                                                        // 0:normal operation
#define REG_SYS_IOCR2_USBP_PUP_IDLE     0x00000010      // USBP_PUP_IDLE
                                                        // 1:USBP pad PUP_IDLE select
                                                        // 0:USBP pad PUP_IDLE not select

// REG_SYS_SUCR bits
#define REG_SYS_SUCR_PLLBYPASS              0x00080000      // PLL Bypass strap value
#define REG_SYS_SUCR_BIGEND                 0x00040000      // BIGEND strap value
#define REG_SYS_SUCR_DEV_PRG                0x00020000      // DEV_PRG strap value
#define REG_SYS_SUCR_DVLP                   0x00010000      // DVLP strap value
#define REG_SYS_SUCR_DSPXPM                 0x00008000      // DSPXPM strap value
#define REG_SYS_SUCR_CFGBIGEND              0x00004000      // DCFGBIGEND flag from ARM11 0:Little 1:Big Endian
#define REG_SYS_SUCR_DEFAULT_STRAP          0x00002000      // DEFAULT STRAP value
                                                            // 0:Use External Strap
                                                            // 1:Use internal default pull for strap of
                                                            // TSTEN0, TSTEN1, EXTID0, EXTID1, EXTID2, TCKEN,
                                                            // SEL13_26, ARMJTAGENB, MEM3V
#define REG_SYS_SUCR_RSVD                   0x00001000      // Reserved
#define REG_SYS_SUCR_ARMJTAGENB             0x00000800      // 0:ARM JTAG selected & enabled 1 ARM/DSP JTAGs disabled
#define REG_SYS_SUCR_SEL13_26               0x00000400      // 0:26MHz Main PLL ref clock 1:13MHz
#define REG_SYS_SUCR_TCKEN                  0x00000200      // TCKEN  0:No bypass clock  1:Select ext bypass clock
#define REG_SYS_SUCR_NAND_TYPE_LARGE_PAGE   0x00000100      // 1=large page nand (e.g. 2048 bytes), 0=small page nand (512 bytes)
#define REG_SYS_SUCR_EXTID_MASK             0x000000e0      // extid 2/1/0 status
#define REG_SYS_SUCR_RESET_STATUS_SOFT      0x00000010      // 1=soft reset from either arm/dsp or wdog, wr 1 to clear. 0=hard
#define REG_SYS_SUCR_NAND_PRESENT           0x00000008      // 1=NAND flash present
#define REG_SYS_SUCR_BOOT_NOR               0x00000004      // 1=boot from NOR flash, 0=boot from internal boot rom
#define REG_SYS_SUCR_NAND_BUS_WIDTH_16      0x00000002      // NAND bus width: 1=16 bit, 0=8-bit
#define REG_SYS_SUCR_DNLD_UART              0x00000001      // Download program through UART

// REG_SYS_PIDR bits
#define REG_SYS_PIDR_PRODUCTID_MASK         0xf0            // TBD
#define REG_SYS_PIDR_REVID_MASK             0x0f            // TBD

// REG_SYS_DSPCTRL bits
#define REG_SYS_DSPCTRL_DSPRST              0x80            // 0:operating 1:reset
#define REG_SYS_DSPCTRL_SYNCEXTPRAM         0x40            // Obsolete: DSP only supports synchronous external PRAM
#define REG_SYS_DSPCTRL_JTAGINTWAKE         0x20            // Control whether DSP JTAG interrupt wakes up DSP from sleep mode
                                                            // 0 - DSP JTAG interrupt does not not wake DSP
                                                            // 1 = DSP JTAG interrupt  wakes up DSP


// REG_SYS_PUMR bits
#define REG_SYS_PUMR_FLASH_BOOT             0x01            // 1:boot from flash, 0:boot from uart (override extid bits)

// REG_SYS_MCR bits - None: any write will remove the bootrom and remap the address 0x0 to external nor flash

// REG_SYS_MRR bits
// This register is to restore the bootrom mapping back. It is done by first writing pattern1
// and then writing pattern2.
#define REG_SYS_MRR_PATTERN1                0x5a5a5a5a
#define REG_SYS_MRR_PATTERN2                0x10c510c5

// REG_SYS_RAMCTRL bits
#define REG_SYS_RAMCTRL_SRAM_SAM_MASK       0xc
#define REG_SYS_RAMCTRL_SRAM_STBY_MASK      0x3

// REG_SYS_SECCTRL bits
// These bits are one-way sticky.  Coming out of reset, the default is 0.
// Once a "1" is written to this bit,  it will be stuck high and will
// not be resetable to "0" unless the entire system is reset.
#define REG_SYS_SECCTRL_CRYPTO_DIS          (1 << 4)  // 0:Enable access to the DES and Crypto (AES/SHA1)
                                                      // 1:Disable access to the DES and Crypto (AES/SHA1)
#define REG_SYS_SECCTRL_PKE_DIS             (1 << 3)  // 0:Enable access to the PKE (Public Key Encryption Engine)
                                                      // 1:Disable access to the PKE (Public Key Encryption Engine)
#define REG_SYS_SECCTRL_OTP_DIS             (1 << 2)  // 0:Enable access to the OTP (secure storage)
                                                      // 1:Disable access to the OTP (secure storage)
#define REG_SYS_SECCTRL_RTC_DIS_WR          (1 << 1)  // 0:Enable write access to the RTC (real-time counter)
                                                      // 1:Disable write access to the RTC (real-time counter)
#define REG_SYS_SECCTRL_BRM_DIS_RD          (1 << 0)  // 0:Enable read access to the boot ROM
                                                      // 1:Disable read access to the boot ROM
// REG_SYS_SECSTAT bits
#define REG_SYS_SECSTAT_JTAG_DIS            (1 << 10) // nvm_glb_disable_jtag (READ-ONLY)
                                                      // This is the post XORed signal to disable the JTAG logic.

#define REG_SYS_SECSTAT_TSTDBG_DIS          (1 << 9)  // nvm_glb_disable_scan_bist_debug (READ-ONLY)
                                                      // This is the post XORed signal to disable the scan,
                                                      // bist, and visibility (snoop) debug logic.
#define REG_SYS_SECSTAT_CRYPTO_DIS          (1 << 8)  // nvm_glb_disable_crypto (READ-ONLY)
#define REG_SYS_SECSTAT_SEC_CONF_MASK       0xff      // Secure Configuration Status (READ-ONLY)
                                                      // This is the 8-bit value of the system configuration bits.
                                                      // Coming out of reset, data is read out of address 0 of the NVM,
                                                      // and either the upper or lower byte is latched, based on the valid bits.
                                                      // Writing to the NVM will not affect these bits; these bits are only
                                                      // updated when coming out of reset.
#define REG_SYS_SECSTAT_OTP_BLANK           0x03      // OTP blank pattern


#define REG_SYS_ANACR0_PWDDIG               (1<<11)
#define REG_SYS_ANACR0_PWDMIC               (1<<12)
#define REG_SYS_ANACR0_PWDAUXMIC            (1<<13)

#define REG_SYS_ANACR10_MICBIAS_ON          (1<<17)

#define REG_SYS_ANACR0_SIMVCC_SHIFT         8
#define REG_SYS_ANACR0_SIMVCC_MASK          0xFFFFE0FF

#define REG_SYS_ANACR0_SIM_LDO_1_2          0           // 0 = 1.2V
#define REG_SYS_ANACR0_SIM_LDO_1_3          1           // 1 = 1.3V
#define REG_SYS_ANACR0_SIM_LDO_1_4          2           // 2 = 1.4V
#define REG_SYS_ANACR0_SIM_LDO_1_5          3           // 3 = 1.5V
#define REG_SYS_ANACR0_SIM_LDO_1_6          4           // 4 = 1.6V
#define REG_SYS_ANACR0_SIM_LDO_1_7          5           // 5 = 1.7V
#define REG_SYS_ANACR0_SIM_LDO_1_8          6           // 6 = 1.8V
#define REG_SYS_ANACR0_SIM_LDO_1_9          7           // 7 = 1.9V
#define REG_SYS_ANACR0_SIM_LDO_2_0          8           // 8 = 2.0V
#define REG_SYS_ANACR0_SIM_LDO_2_1          9           // 9 = 2.1V
#define REG_SYS_ANACR0_SIM_LDO_2_2          10          // A = 2.2V
#define REG_SYS_ANACR0_SIM_LDO_2_3          11          // B = 2.3V
#define REG_SYS_ANACR0_SIM_LDO_2_4          12          // C = 2.4V
#define REG_SYS_ANACR0_SIM_LDO_2_5          13          // D = 2.5V
#define REG_SYS_ANACR0_SIM_LDO_2_6          14          // E = 2.6V
#define REG_SYS_ANACR0_SIM_LDO_2_7          15          // F = 2.7V
#define REG_SYS_ANACR0_SIM_LDO_2_8          16          // 0x10 = 2.8V
#define REG_SYS_ANACR0_SIM_LDO_2_9          17          // 0x11 = 2.9V
#define REG_SYS_ANACR0_SIM_LDO_3_0          18          // 0x12 = 3.0V
#define REG_SYS_ANACR0_SIM_LDO_3_1          19          // 0x13 = 3.1V
#define REG_SYS_ANACR0_SIM_LDO_3_2          20          // 0x14 = 3.2V


// REG_SYS_IRDROP bits
// The IRDROP monitors work by counting the number of pulses of a ring oscillator during 4 cycles of a 32.76KHz clock.
// The frequency of the ring oscillator varies between 1MHz and 3MHz depending upon the voltage at the oscillator.
// Software should enable the oscillator by writing MON_EN and waiting an appropriate amount of time before reading
// the CNT_OUT value. Given the frequencies of operation, the value of CNT_OUT will vary between 16F (no IR drop)
// and 7A (maximum IR drop).
#define REG_SYS_IRDROP_OSC_EN               (1 << 11)   // Not used
#define REG_SYS_IRDROP_MON_EN               (1 << 10)   // Write 1 to enable counter. Clear to 0 before re-initialization.
#define REG_SYS_IRDROP_CNT_OUT_MASK         0x3ff       // Software can read this value to determine IRDROP (Read-Only)

#endif
