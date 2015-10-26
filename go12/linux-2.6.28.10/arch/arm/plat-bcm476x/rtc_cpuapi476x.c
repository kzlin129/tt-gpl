/*****************************************************************************
* Copyright 2009 Broadcom Corporation.  All rights reserved.
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
/*  rtc_cpuapi.c
 *
 * Register access for BCM4760 RTC block.
 */
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/delay.h>

#ifdef CONFIG_PM

#include <asm/io.h>

#include <asm/arch/bcm4760_reg.h>
#include <asm/arch/rtc_cpuapi4760.h>
#include <asm/arch/bbl4760.h>
#include <asm/arch/hardware.h>

#define RTC_POLL_TIMEOUT 100		// Timeout in milliseconds (10 ms resolution only)

// ## Battery Backup Logic (RTC) Address Map
// 
// set RTC_ADDRESS_REGISTER           [ expr vhex($RTC_ADDRBASE0 + 0x000)]
// set RTC_DATA_REGISTER              [ expr vhex($RTC_ADDRBASE0 + 0x004)]
// set RTC_PWSEQ_TAMPER_RST_REGISTER  [ expr vhex($RTC_ADDRBASE0 + 0x008)]
// 
// ## The following addresses can be accessed by writing to the above
// ## RTC_ADDRESS_REGISTER and read or written via the RTC_DATA_REGISTER:
// 
// set RTC_PER                        [ expr vhex(0x01)]
// set RTC_MATCH                      [ expr vhex(0x02)]
// set RTC_SET_DIV                    [ expr vhex(0x03)]
// set RTC_SET_RTC                    [ expr vhex(0x04)]
// set RTC_CLR_INT_MATCH              [ expr vhex(0x05)]
// set RTC_DLY1                       [ expr vhex(0x06)]
// set RTC_DLY2                       [ expr vhex(0x07)]
// set RTC_TIMEOUT                    [ expr vhex(0x08)]
// set RTC_MASK                       [ expr vhex(0x09)]
// set RTC_EVENT0                     [ expr vhex(0x0a)]
// set RTC_EVENT1                     [ expr vhex(0x0b)]
// set RTC_EVENT2                     [ expr vhex(0x0c)]
// set RTC_CLK_DIV_REG                [ expr vhex(0x0d)]
// set RTC_RTC_REG                    [ expr vhex(0x0e)]
// set RTC_TRIG_STAT                  [ expr vhex(0x0f)]
// set RTC_INTERRUPT_ENABLE           [ expr vhex(0x10)]           
// set RTC_RESET_ACCESS               [ expr vhex(0x11)]

static int32_t Wait_RTC_Not_Busy(unsigned long delay)
{
	unsigned long timeout;

	// Wait until RTC is not busy
	timeout = delay/10;
	while ( readl(IO_ADDRESS(RTC_R_BLKCTL_MEMADDR)) & 0x01 )
	{
		mdelay(10);
		if (--timeout == 0)
			return 1;
	}

	return 0;
}

uint32_t rtc_read ( uint32_t addr)
{
	if (Wait_RTC_Not_Busy(RTC_POLL_TIMEOUT))
		return 0;

	// Issuing Address Register Command, bit[0]=GO, bit[1]=Write/Read
	writel((0x0001 | (addr & 0xff) << 8), IO_ADDRESS(RTC_R_BLKCTL_MEMADDR));

	if (Wait_RTC_Not_Busy(RTC_POLL_TIMEOUT))
		return 0;

	return readl(IO_ADDRESS(RTC_R_BLKSTS_MEMADDR)); 
}

void rtc_write ( uint32_t addr, uint32_t data )
{
	if (Wait_RTC_Not_Busy(RTC_POLL_TIMEOUT))
		return;

	// Issuing Address Register Command, bit[0]=GO, bit[1]=Write/Read
	writel(data, IO_ADDRESS(RTC_R_BLKSTS_MEMADDR)); 
	writel((0x0003 | (addr & 0xff) << 8), IO_ADDRESS(RTC_R_BLKCTL_MEMADDR));
}

void init_bbl(void* warm_boot_address)
{
   uint32_t rows, cols, cl, gr_edge_sel, drive_strength;
   uint32_t pad_control, ddr_read_ncdl_offset, ddr_write_ncdl_offset, dram_timing_0, dram_timing_1;
   
   // in Linux, we need to get these info from the EMI registers
   rows = readl(IO_ADDRESS(DDR_R_EMI_CNTRLR_CONFIG_MEMADDR));
   cols = (rows & DDR_F_COL_BITS_MASK) >> DDR_F_COL_BITS_R;
   rows = (rows & DDR_F_ROW_BITS_MASK) >> DDR_F_ROW_BITS_R;
   cl = (readl(IO_ADDRESS(DDR_R_EMI_DRAM_MODE_SET_MEMADDR)) & DDR_F_CL_MASK) >> DDR_F_CL_R;
   gr_edge_sel = (readl(IO_ADDRESS(DDR_R_EMI_DDR_RDDQS_GATE_CNTRL_MEMADDR)) & DDR_F_GR_EDGE_SEL_MASK) >> DDR_F_GR_EDGE_SEL_R;
   drive_strength = (readl(IO_ADDRESS(DDR_R_EMI_DRAM_MODE_SET_MEMADDR)) & DDR_F_DS_MASK) >> DDR_F_DS_R;
   
   pad_control = readl(IO_ADDRESS(DDR_R_EMI_PAD_CONTROL_MEMADDR));      
   ddr_read_ncdl_offset = readl(IO_ADDRESS(DDR_R_EMI_DDR_READ_NCDL_OFFSET_MEMADDR));
   ddr_write_ncdl_offset = readl(IO_ADDRESS(DDR_R_EMI_DDR_WRITE_NCDL_OFFSET_MEMADDR));
   dram_timing_0 = readl(IO_ADDRESS(DDR_R_EMI_DRAM_TIMING_0_MEMADDR));  
   dram_timing_1 = readl(IO_ADDRESS(DDR_R_EMI_DRAM_TIMING_1_MEMADDR));  
   
   // BBL_OFFSET_DRAM_ADDRESS is the starting address to be run on wakeup
   rtc_write(BBL_OFFSET_DRAM_ADDRESS, (uint32_t)warm_boot_address);
   rtc_write(BBL_OFFSET_DRAM_MAX_ADDR, 0); // [JLH] - fixup later with correct value - not used in A0 Boot ROM at all.
   rtc_write(BBL_OFFSET_EMI_CFG_BITS, ((drive_strength << 16) | (gr_edge_sel << 12) | (cl << 8) | (cols << 4) | (rows << 0)));
   rtc_write(BBL_OFFSET_EMI_DRAM_TIMING_0, dram_timing_0);
   rtc_write(BBL_OFFSET_EMI_DRAM_TIMING_1, dram_timing_1);

#ifdef CONFIG_BCM4760_PMSTR_BBL4760_WARMSTART
   rtc_write(BBL_OFFSET_WARM_TAG, WARM_BOOT_TAG4760);
#elif defined(CONFIG_BCM4760_PMSTR_BBL4760_LUKEWARMSTART)
   rtc_write(BBL_OFFSET_DDR_LUKEWARM_BOOT_OFFSET, LUKEWARM_BOOT_TAG4760);
#else
#error CONFIG_PM set but no suspend/resume mechanism chosen!
#endif

   rtc_write(BBL_OFFSET_EMI_PAD_CONTROL, pad_control);
   rtc_write(BBL_OFFSET_DDR_READ_NCDL_OFFSET, ddr_read_ncdl_offset);
   rtc_write(BBL_OFFSET_DDR_WRITE_NCDL_OFFSET, ddr_write_ncdl_offset);
}

void deinit_bbl (void)
{
	rtc_write(BBL_OFFSET_WARM_TAG, 0x0);
	rtc_write(BBL_OFFSET_DDR_LUKEWARM_BOOT_OFFSET, 0x0);
	rtc_write(BBL_OFFSET_EMI_PAD_CONTROL, 0x0);
	rtc_write(BBL_OFFSET_DDR_READ_NCDL_OFFSET, 0x0);
	rtc_write(BBL_OFFSET_DDR_WRITE_NCDL_OFFSET, 0x0);
	rtc_write(BBL_OFFSET_DRAM_ADDRESS, 0x0);
	rtc_write(BBL_OFFSET_DRAM_MAX_ADDR, 0x0);
	rtc_write(BBL_OFFSET_EMI_CFG_BITS, 0x0);
	rtc_write(BBL_OFFSET_EMI_DRAM_TIMING_0, 0x0);
	rtc_write(BBL_OFFSET_EMI_DRAM_TIMING_1, 0x0);
}

#endif // end #IFDEF CONFIG_PM
