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
*  REG_UMI.h
*
*  PURPOSE:
*
*     This file contains definitions for the nand registers:
*
*  NOTES:
*
*****************************************************************************/


#if !defined( __ASM_ARCH_REG_UMI_H )
#define __ASM_ARCH_REG_UMI_H

/* ---- Include Files ---------------------------------------------------- */
#include <asm/arch/hardware.h>

/* ---- Constants and Types ---------------------------------------------- */

#define REG_UMI_FLASH0_TCR      __REG32( HW_UMI_BASE  + 0x00 )     // Flash bank 0 timing and control register
#define REG_UMI_FLASH1_TCR      __REG32( HW_UMI_BASE  + 0x04 )     // Flash bank 1 timing and control register
#define REG_UMI_FLASH2_TCR      __REG32( HW_UMI_BASE  + 0x08 )     // Flash bank 2 timing and control register
#define REG_UMI_MMD_ICR         __REG16( HW_UMI_BASE  + 0x0c + BCM_WORD_OFFSET)  // MMD interface and control register
#define REG_UMI_SRAM0_TCR       __REG32( HW_UMI_BASE  + 0x10 )     // SRAM bank 0 timing and control register
#define REG_UMI_SRAM1_TCR       __REG32( HW_UMI_BASE  + 0x14 )     // SRAM bank 1 timing and control register
#define REG_UMI_NAND_TCR        __REG32( HW_UMI_BASE  + 0x18 )     // NAND timing and control register
#define REG_UMI_NAND_RCSR       __REG8(  HW_UMI_BASE  + 0x1c + BCM_BYTE_OFFSET)  // NAND ready/chip select register
#define REG_UMI_NAND_ECC_CSR    __REG32( HW_UMI_BASE  + 0x20 )     // NAND ECC control & status register
#define REG_UMI_NAND_ECC_DATA   __REG32( HW_UMI_BASE  + 0x24 )     // NAND ECC data register XXB2B1B0
#define REG_UMI_PSRAM_RTCR      __REG16( HW_UMI_BASE  + 0x28 + BCM_WORD_OFFSET ) // PSRAM Refresh Terminal Count Register   
#define REG_UMI_PSRAM_RFCR      __REG16( HW_UMI_BASE  + 0x2c + BCM_BYTE_OFFSET ) // PSRAM Refresh Control Register
#define REG_UMI_SDRAM_TCR1      __REG16( HW_UMI_BASE  + 0x40 + BCM_WORD_OFFSET)  // SDRAM timing and control register 1
#define REG_UMI_SDRAM_TCR2      __REG16( HW_UMI_BASE  + 0x44 + BCM_WORD_OFFSET)  // SDRAM timing and control register 2
#define REG_UMI_SDRAM_RFR       __REG16( HW_UMI_BASE  + 0x48 + BCM_WORD_OFFSET)  // SDRAM refresh timer control register
#define REG_UMI_SDRAM_NMR       __REG16( HW_UMI_BASE  + 0x4c + BCM_WORD_OFFSET)  // SDRAM normal mode register
#define REG_UMI_SDRAM_XMR       __REG16( HW_UMI_BASE  + 0x50 + BCM_WORD_OFFSET)  // SDRAM extended mode register
#define REG_UMI_SDRAM_CTR       __REG16( HW_UMI_BASE  + 0x54 + BCM_WORD_OFFSET)  // SDRAM control/status register
#define REG_UMI_SDRAM_ICR       __REG32( HW_UMI_BASE  + 0x5c)      // SDRAM interface control register

// REG_UMI_FLASH0/1/2_TCR, REG_UMI_SRAM0/1_TCR bits
#define REG_UMI_TCR_WAITEN              0x80000000          // Enable wait pin during burst write or read
#define REG_UMI_TCR_LOWFREQ             0x40000000          // Enable mem ctrlr to work iwth ext mem of lower freq than AHB clk
#define REG_UMI_TCR_MEMTYPE_SYNCWRITE   0x20000000          // 1=synch write, 0=async write
#define REG_UMI_TCR_MEMTYPE_SYNCREAD    0x10000000          // 1=synch read, 0=async read
#define REG_UMI_TCR_MEMTYPE_PAGEREAD    0x08000000          // 1=page mode read, 0=normal mode read
#define REG_UMI_TCR_MEMTYPE_PGSZ_MASK   0x07000000          // page size/burst size (wrap only)
#define REG_UMI_TCR_MEMTYPE_PGSZ_4      0x00000000          // 4 word
#define REG_UMI_TCR_MEMTYPE_PGSZ_8      0x01000000          // 8 word
#define REG_UMI_TCR_MEMTYPE_PGSZ_16     0x02000000          // 16 word
#define REG_UMI_TCR_MEMTYPE_PGSZ_32     0x03000000          // 32 word
#define REG_UMI_TCR_MEMTYPE_PGSZ_64     0x04000000          // 64 word
#define REG_UMI_TCR_MEMTYPE_PGSZ_128    0x05000000          // 128 word
#define REG_UMI_TCR_MEMTYPE_PGSZ_256    0x06000000          // 256 word
#define REG_UMI_TCR_MEMTYPE_PGSZ_512    0x07000000          // 512 word
#define REG_UMI_TCR_TPRC_TWLC_MASK      0x00f80000          // Page read access cycle / Burst write latency (n+2 / n+1)
#define REG_UMI_TCR_TBTA_MASK           0x00070000          // Bus turnaround cycle (n)
#define REG_UMI_TCR_TWP_MASK            0x0000f800          // Write pulse width cycle (n+1)
#define REG_UMI_TCR_TWR_MASK            0x00000600          // Write recovery cycle (n+1)
#define REG_UMI_TCR_TAS_MASK            0x00000180          // Write address setup cycle (n+1)
#define REG_UMI_TCR_TOE_MASK            0x00000060          // Output enable delay cycle (n)
#define REG_UMI_TCR_TRC_TLC_MASK        0x0000001f          // Read access cycle / Burst read latency (n+2 / n+1)


// REG_UMI_MMD_ICR bits
#define REG_UMI_MMD_ICR_FLASH_WP            0x8000          // Flash write protection pin control
#define REG_UMI_MMD_ICR_XHCS                0x4000          // Extend hold time for sram0, sram1 csn (39 MHz operation)
#define REG_UMI_MMD_ICR_SDRAM2EN            0x2000          // Enable SDRAM 2 interface control
#define REG_UMI_MMD_ICR_INST512             0x1000          // Enable merge of flash banks 0/1 to 512 MBit bank
#define REG_UMI_MMD_ICR_DATA512             0x0800          // Enable merge of flash banks 1/2 to 512 MBit bank
#define REG_UMI_MMD_ICR_SDRAMEN             0x0400          // Enable SDRAM interface control
#define REG_UMI_MMD_ICR_WAITPOL             0x0200          // Polarity of busy state of Burst Wait Signal
#define REG_UMI_MMD_ICR_BCLKSTOP            0x0100          // Enable burst clock stopped when not accessing external burst flash/sram
#define REG_UMI_MMD_ICR_PERI1EN             0x0080          // Enable the peri1_csn to replace flash1_csn in 512 Mb flash mode
#define REG_UMI_MMD_ICR_PERI2EN             0x0040          // Enable the peri2_csn to replace sdram_csn
#define REG_UMI_MMD_ICR_PERI3EN             0x0020          // Enable the peri3_csn to replace sdram2_csn
#define REG_UMI_MMD_ICR_MRSB1               0x0010          // Enable sram bank1 for H/W controlled MRS
#define REG_UMI_MMD_ICR_MRSB0               0x0008          // Enable sram bank0 for H/W controlled MRS
#define REG_UMI_MMD_ICR_MRSPOL              0x0004          // Polarity for assert3ed state of H/W controlled MRS
#define REG_UMI_MMD_ICR_MRSMODE             0x0002          // 0: S/W controllable ZZ/MRS/CRE/P-Mode pin
                                                            // 1: H/W controlled ZZ/MRS/CRE/P-Mode, same timing as CS
#define REG_UMI_MMD_ICR_MRSSTATE            0x0001          // MRS state for S/W controlled mode

// REG_UMI_NAND_TCR bits
#define REG_UMI_NAND_TCR_CS_SWCTRL          0x80000000      // Enable software to control CS
#define REG_UMI_NAND_TCR_WORD16             0x40000000      // 16-bit nand wordsize if set
#define REG_UMI_NAND_TCR_TBTA_MASK          0x00070000      // Bus turnaround cycle (n)
#define REG_UMI_NAND_TCR_TWP_MASK           0x0000f800      // Write pulse width cycle (n+1)
#define REG_UMI_NAND_TCR_TWR_MASK           0x00000600      // Write recovery cycle (n+1)
#define REG_UMI_NAND_TCR_TAS_MASK           0x00000180      // Write address setup cycle (n+1)
#define REG_UMI_NAND_TCR_TOE_MASK           0x00000060      // Output enable delay cycle (n)
#define REG_UMI_NAND_TCR_TRC_TLC_MASK       0x0000001f      // Read access cycle (n+2)

// REG_UMI_NAND_RCSR bits
#define REG_UMI_NAND_RCSR_RDY               0x02            // Status: Ready=1, Busy=0
#define REG_UMI_NAND_RCSR_CS_ASSERTED       0x01            // Keep CS asserted during operation

// REG_UMI_NAND_ECC_CSR bits
#define REG_UMI_NAND_ECC_CSR_NANDINT        0x80000000      // Interrupt status - read-only
#define REG_UMI_NAND_ECC_CSR_ECCINT_RAW     0x00800000      // Read: Status of ECC done, Write: clear ECC interrupt
#define REG_UMI_NAND_ECC_CSR_RBINT_RAW      0x00400000      // Read: Status of R/B, Write: clear R/B interrupt
#define REG_UMI_NAND_ECC_CSR_ECCINT_ENABLE  0x00008000      // 1 = Enable ECC Interrupt
#define REG_UMI_NAND_ECC_CSR_RBINT_ENABLE   0x00004000      // 1 = Assert interrupt at rising edge of R/B_
#define REG_UMI_NAND_ECC_CSR_256BYTE        0x00000080      // Calculate ECC by 0=512 bytes, 1=256 bytes
#define REG_UMI_NAND_ECC_CSR_ECC_ENABLE     0x00000001      // Enable ECC in hardware

// REG_UMI_PSRAM_RFCR bits
#define REG_UMI_PSRAM_RFCR_F0               (1<<0)          // Enable PSRAM Refresh Control on Flash Bank 0
#define REG_UMI_PSRAM_RFCR_F1               (1<<1)          // Enable PSRAM Refresh Control on Flash Bank 1
#define REG_UMI_PSRAM_RFCR_F2               (1<<2)          // Enable PSRAM Refresh Control on Flash Bank 2
#define REG_UMI_PSRAM_RFCR_S0               (1<<3)          // Enable PSRAM Refresh Control on SRAM Bank 0
#define REG_UMI_PSRAM_RFCR_S1               (1<<4)          // Enable PSRAM Refresh Control on SRAM Bank 1

// REG_UMI_SDRAM_TCR1 bits
#define REG_UMI_SDRAM_TCR1_TRAS_SHIFT       12              // Row Active Time - 4 bits
#define REG_UMI_SDRAM_TCR1_TRCD_SHIFT       8               // RAS to CAS delay - 3 bits
#define REG_UMI_SDRAM_TCR1_TRRD_SHIFT       4               // Row active to row active delay - 4 bits
#define REG_UMI_SDRAM_TCR1_TRP_SHIFT        0               // Row precharge time - 3 bits

// REG_UMI_SDRAM_TCR2 bits
#define REG_UMI_SDRAM_TCR2_TRFC_SHIFT       8               // Auto refresh cycle time - 5 bits
#define REG_UMI_SDRAM_TCR2_TRSC_MASK        4               // Mode register set cycle - 4 bits
#define REG_UMI_SDRAM_TCR2_TWR_MASK         0               // Write recovery time - 3 bits

// REG_UMI_SDRAM_RFR bits
#define REG_UMI_SDRAM_RFR_MASK              0x07ff          // Refresh time terminal count period

// REG_UMI_SDRAM_NMR bits
#define REG_UMI_SDRAM_NMR_WM                (1<<9)          // Write burst mode
#define REG_UMI_SDRAM_NMR_CASL_SHIFT        4               // CAS Latency - 3 bits
#define REG_UMI_SDRAM_NMR_BT                (1<<3)          // Burst type
#define REG_UMI_SDRAM_NMR_BL_SHIFT          0               // Burst length - 3 bits

// REG_UMI_SDRAM_XMR bits
#define REG_UMI_SDRAM_XMR_TCSR_MASK         0x0018          // Temperature compensated self refresh
#define REG_UMI_SDRAM_XMR_PASR_MASK         0x0007          // Partial array self refresh

// REG_UMI_SDRAM_CTR bits
#define REG_UMI_SDRAM_CTR_SRST              0x0400          // Soft Reset to the controller (write 1)
#define REG_UMI_SDRAM_CTR_REFA              0x0200          // Trigger to set auto refresh command (write 1)
#define REG_UMI_SDRAM_CTR_PON               0x0100          // Trigger initialization sequence (write 1)
#define REG_UMI_SDRAM_CTR_REFA_RDY          0x0004          // REFA ready status (read)
#define REG_UMI_SDRAM_CTR_MRS_RDY           0x0002          // Mode register setting is done status (read)
#define REG_UMI_SDRAM_CTR_PON_RDY           0x0001          // Initialization sequence is done status (read)
#define REG_UMI_SDRAM_CTR_ALL_RDY           0x0007          // Initialization sequence of all is done status (read)

// REG_UMI_SDRAM_ICR bits
#define REG_UMI_SDRAM_ICR_ADVMODE           0x80000000      // Enable Advanced Address Mapping Mode
#define REG_UMI_SDRAM_ICR_HPM               0x00000000      // 0: High Performance mode
                                                            // -    AHB Address = {Row, Bank, Col} 
                                                            // -	Utilize all four banks
#define REG_UMI_SDRAM_ICR_LPM               0x01000000      // 1: Low Power mode 
                                                            // -	AHB Address = {Bank, Row, Col} 
                                                            // -	Can use Partial Bank Self Refresh
#define REG_UMI_SDRAM_ICR_ROWWIDTH_11BIT    0x00000000      // Row Address Width for Advanced Address Mapping Mode
#define REG_UMI_SDRAM_ICR_ROWWIDTH_12BIT    0x00100000      // Row Address Width for Advanced Address Mapping Mode
#define REG_UMI_SDRAM_ICR_ROWWIDTH_13BIT    0x00200000      // Row Address Width for Advanced Address Mapping Mode
#define REG_UMI_SDRAM_ICR_COLWIDTH_8BIT     0x00000000      // Column Address Width for Advanced Address Mapping Mode
#define REG_UMI_SDRAM_ICR_COLWIDTH_9BIT     0x00010000      // Column Address Width for Advanced Address Mapping Mode
#define REG_UMI_SDRAM_ICR_COLWIDTH_10BIT    0x00020000      // Column Address Width for Advanced Address Mapping Mode
#define REG_UMI_SDRAM_ICR_COLWIDTH_11BIT    0x00030000      // Column Address Width for Advanced Address Mapping Mode
#define REG_UMI_SDRAM_ICR_COLWIDTH_12BIT    0x00040000      // Column Address Width for Advanced Address Mapping Mode
#define REG_UMI_SDRAM_ICR_SFEN              0x00001000      // Enable sdram self refresh mode with arm and dsp sleep
#define REG_UMI_SDRAM_ICR_32BIT             0x00000800      // 1:32bit 0:16 bit word part
#define REG_UMI_SDRAM_ICR_PURETNO_MASK      0x00000070      // Number of refresh commands during initialization sequence
#define REG_UMI_SDRAM_ICR_PURETNO_1         0x00000010      // 1 refresh command
#define REG_UMI_SDRAM_ICR_PURETNO_7         0x00000070      // 7 refresh commands
#define REG_UMI_SDRAM_ICR_MSIZE_MASK        0x00000003      // SDRAM memory size mask
#define REG_UMI_SDRAM_ICR_MSIZE_4MBY16_32   0x00000000      // SDRAM memory - 4M X 16/32 bits
#define REG_UMI_SDRAM_ICR_MSIZE_8MBY16_32   0x00000001      // SDRAM memory - 8M X 16/32 bits
#define REG_UMI_SDRAM_ICR_MSIZE_16MBY16_32  0x00000002      // SDRAM memory - 16M X 16/32 bits
#define REG_UMI_SDRAM_ICR_MSIZE_32MBY16_32  0x00000003      // SDRAM memory - 32M X 16/32 bits

#endif
