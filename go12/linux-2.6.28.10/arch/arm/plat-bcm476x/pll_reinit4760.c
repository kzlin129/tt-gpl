/*****************************************************************************
* Copyright 2008 - 2009 Broadcom Corporation.  All rights reserved.
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
 *   @file   pll_reinit4760.c
 * 
 *   @brief  BCM4760 PLL Reinitialization routines for coming out of STR
 * 
 ****************************************************************************/

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <asm/arch/bcm4760_reg.h>

#define __UNLOCK_CMU()  writel(0xBCBC4760, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR))
#define __LOCK_CMU()    writel(0, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR))

#define ASM_NOP         __asm("nop")
#define ASM_STOPHERE    __asm("1: nop; b 1b")

/* defines used for CMU_R_OTP_STRAP_MEMADDR */
#define	CMU_F_CLK_FREQ_MASK			0xE0
#define	CMU_F_CLK_FREQ_R				(5)

#define PLL_NDIV_MODE 0x2
#define CMGR_REFCLK_MAX	(6) 
#define CMGR_PLLCLK_MAX	(20) 
#define	NUMBER_UART_ENTRIES (4)
#define	CMGR_REFCLK_MARGIN	0x10

/* UART Init settings */

#define UART_CLK_12_0MHZ				12000000
#define UART_CLK_19_2MHZ       	19200000
#define UART_CLK_24_5MHZ       	24553500
#define UART_CLK_26_0MHZ       	26000000
#define UART_CLK_38_4MHZ       	38400000
#define UART_CLK_52_0MHZ       	52000000
#define UART_CLK_78_0MHZ       	78000000
#define UART_CLK_78_26MHZ				78260000
#define UART_CLK_76_8MHZ				76800000

#define UART_BAUD_RATE      115200              /* Baud rate */
#define BaudRateInteger_115dot2_12_0MHZ		(UART_CLK_12_0MHZ / (16 * UART_BAUD_RATE))
#define BaudRateRemainder_115dot2_12_0MHZ	(UART_CLK_12_0MHZ % (16 * UART_BAUD_RATE))
#define BaudRateFraction_115dot2_12_0MHZ	(4 * BaudRateRemainder_115dot2_12_0MHZ)/UART_BAUD_RATE

#define BaudRateInteger_115dot2_19_2MHZ		(UART_CLK_19_2MHZ / (16 * UART_BAUD_RATE))
#define BaudRateRemainder_115dot2_19_2MHZ	(UART_CLK_19_2MHZ % (16 * UART_BAUD_RATE))
#define BaudRateFraction_115dot2_19_2MHZ	(4 * BaudRateRemainder_115dot2_19_2MHZ)/UART_BAUD_RATE

#define BaudRateInteger_115dot2_24_5MHZ		(UART_CLK_24_5MHZ / (16 * UART_BAUD_RATE))
#define BaudRateRemainder_115dot2_24_5MHZ	(UART_CLK_24_5MHZ % (16 * UART_BAUD_RATE))
#define BaudRateFraction_115dot2_24_5MHZ	(4 * BaudRateRemainder_115dot2_24_5MHZ)/UART_BAUD_RATE

#define BaudRateInteger_115dot2_26_0MHZ		(UART_CLK_26_0MHZ / (16 * UART_BAUD_RATE))
#define BaudRateRemainder_115dot2_26_0MHZ	(UART_CLK_26_0MHZ % (16 * UART_BAUD_RATE))
#define BaudRateFraction_115dot2_26_0MHZ	(4 * BaudRateRemainder_115dot2_26_0MHZ)/UART_BAUD_RATE

#define BaudRateInteger_115dot2_38_4MHZ		(UART_CLK_38_4MHZ / (16 * UART_BAUD_RATE))
#define BaudRateRemainder_115dot2_38_4MHZ	(UART_CLK_38_4MHZ % (16 * UART_BAUD_RATE))
#define BaudRateFraction_115dot2_38_4MHZ	(4 * BaudRateRemainder_115dot2_38_4MHZ)/UART_BAUD_RATE

#define BaudRateInteger_115dot2_52_0MHZ		(UART_CLK_52_0MHZ / (16 * UART_BAUD_RATE))
#define BaudRateRemainder_115dot2_52_0MHZ	(UART_CLK_52_0MHZ % (16 * UART_BAUD_RATE))
#define BaudRateFraction_115dot2_52_0MHZ	(4 * BaudRateRemainder_115dot2_52_0MHZ)/UART_BAUD_RATE

#define BaudRateInteger_115dot2_78_0MHZ		(UART_CLK_78_0MHZ / (16 * UART_BAUD_RATE))
#define BaudRateRemainder_115dot2_78_0MHZ	(UART_CLK_78_0MHZ % (16 * UART_BAUD_RATE))
#define BaudRateFraction_115dot2_78_0MHZ	(4 * BaudRateRemainder_115dot2_78_0MHZ)/UART_BAUD_RATE

#define BaudRateInteger_115dot2_78_26MHZ		(UART_CLK_78_26MHZ / (16 * UART_BAUD_RATE))
#define BaudRateRemainder_115dot2_78_26MHZ	(UART_CLK_78_26MHZ % (16 * UART_BAUD_RATE))
#define BaudRateFraction_115dot2_78_26MHZ	(4 * BaudRateRemainder_115dot2_78_26MHZ)/UART_BAUD_RATE

#define BaudRateInteger_115dot2_76_8MHZ		(UART_CLK_76_8MHZ / (16 * UART_BAUD_RATE))
#define BaudRateRemainder_115dot2_76_8MHZ	(UART_CLK_76_8MHZ % (16 * UART_BAUD_RATE))
#define BaudRateFraction_115dot2_76_8MHZ	(4 * BaudRateRemainder_115dot2_76_8MHZ)/UART_BAUD_RATE

/* private typedef */
typedef struct _tag_clk_reg_t {
  uint32_t  addr;
  uint32_t  val;
} clk_reg_t;
  

/* 26000000Hz [0] - Default settings */
static const clk_reg_t plls_ref_0[CMGR_PLLCLK_MAX] =
{
    /* pwrdn_chx=0, en_cmlbufx=0, ndiv_pwrdn=0, bypass_sdmod=0, refcomp_pwrdn=0, pwrdn=0*/
    { CMU_R_PLC_PLL_CTL0_MEMADDR,	0x0 },
    /* plc_dreset and plc_areset */
    { CMU_R_PLC_PLL_CTL1_MEMADDR,	0x0 },
    /* unused */
    { CMU_R_PLC_PLL_CTL2_MEMADDR,	0x0 },
    /* ndiv_dither_mfb=0, ndiv_mode=001, ndiv_int=0, p1div=0, p2div=0*/
    { CMU_R_PLC_PLL_CTL3_MEMADDR,	(\
        ((0x0 <<CMU_F_PLC_NDIV_DITHER_MFB_R) & CMU_F_PLC_NDIV_DITHER_MFB_MASK)|\
        ((PLL_NDIV_MODE <<CMU_F_PLC_NDIV_MODE_R) & CMU_F_PLC_NDIV_MODE_MASK) |\
        ((77 <<CMU_F_PLC_NDIV_INT_R) & CMU_F_PLC_NDIV_INT_MASK) |\
        ((1 <<CMU_F_PLC_P2DIV_R) & CMU_F_PLC_P2DIV_MASK) |\
        ((1 <<CMU_F_PLC_P1DIV_R) & CMU_F_PLC_P1DIV_MASK) )},
    /* ndiv_frac */
    { CMU_R_PLC_PLL_CTL4_MEMADDR,	(0x0 & CMU_F_PLC_NDIV_FRAC_MASK)},
    /* m1div - m4div */
    { CMU_R_PLC_PLL_CTL5_MEMADDR,	(\
        ((8 << CMU_F_PLC_M4DIV_R) & CMU_F_PLC_M4DIV_MASK)|\
        ((9 << CMU_F_PLC_M3DIV_R) & CMU_F_PLC_M3DIV_MASK)|\
        ((21 << CMU_F_PLC_M2DIV_R) & CMU_F_PLC_M2DIV_MASK)|\
        ((7<< CMU_F_PLC_M1DIV_R) & CMU_F_PLC_M1DIV_MASK))},
    /* m5div - m6div*/
    { CMU_R_PLC_PLL_CTL6_MEMADDR,	(\
        ((12 << CMU_F_PLC_M6DIV_R) & CMU_F_PLC_M6DIV_MASK)|\
        ((4 << CMU_F_PLC_M5DIV_R) & CMU_F_PLC_M5DIV_MASK))},
    /* plc_vco_rng, bypclkx, enb_clkout, bypen */
    { CMU_R_PLC_PLL_CTL7_MEMADDR,	0x0 },

    /* pwrdn_chx=0, en_cmlbufx=0, ndiv_pwrdn=0, bypass_sdmod=0, refcomp_pwrdn=0, pwrdn=0*/
    { CMU_R_PLA_PLL_CTL0_MEMADDR,	0x0 },
    /* plc_dreset and plc_areset */
    { CMU_R_PLA_PLL_CTL1_MEMADDR,	0x0 },
    /* unused */
    { CMU_R_PLA_PLL_CTL2_MEMADDR,	0x0 },
    /* ndiv_dither_mfb=0, ndiv_mode=001, ndiv_int=0, p1div=0, p2div=0*/
    { CMU_R_PLA_PLL_CTL3_MEMADDR,	(\
        ((0x0 <<CMU_F_PLA_NDIV_DITHER_MFB_R) & CMU_F_PLA_NDIV_DITHER_MFB_MASK)|\
        ((PLL_NDIV_MODE <<CMU_F_PLA_NDIV_MODE_R) & CMU_F_PLA_NDIV_MODE_MASK) |\
        ((48 <<CMU_F_PLA_NDIV_INT_R) & CMU_F_PLA_NDIV_INT_MASK) |\
        ((1 <<CMU_F_PLA_P2DIV_R) & CMU_F_PLA_P2DIV_MASK) |\
        ((1 <<CMU_F_PLA_P1DIV_R) & CMU_F_PLA_P1DIV_MASK) )},
    /* ndiv_frac */
    { CMU_R_PLA_PLL_CTL4_MEMADDR,	(0x0 & CMU_F_PLA_NDIV_FRAC_MASK)},
    /* m1div - m4div */
    { CMU_R_PLA_PLL_CTL5_MEMADDR,	(\
        ((25 << CMU_F_PLA_M4DIV_R) & CMU_F_PLA_M4DIV_MASK)|\
        ((48 << CMU_F_PLA_M3DIV_R) & CMU_F_PLA_M3DIV_MASK)|\
        ((96 << CMU_F_PLA_M2DIV_R) & CMU_F_PLA_M2DIV_MASK)|\
        ((4 << CMU_F_PLA_M1DIV_R) & CMU_F_PLA_M1DIV_MASK))},
    /* m5div - m6div*/
    { CMU_R_PLA_PLL_CTL6_MEMADDR,	(\
        ((26 << CMU_F_PLA_M6DIV_R) & CMU_F_PLA_M6DIV_MASK)|\
        ((/*4*/24 << CMU_F_PLA_M5DIV_R) & CMU_F_PLA_M5DIV_MASK))},
    /* plc_vco_rng, bypclkx, enb_clkout, bypen */
    { CMU_R_PLA_PLL_CTL7_MEMADDR,	0x0 },

    /* The following entries are for setting up uart clock in bypass mode*/
    /* uart divider integer */
    { URT0_R_UARTIBRD_MEMADDR, BaudRateInteger_115dot2_26_0MHZ },
    /* uart divider fraction */
    { URT0_R_UARTFBRD_MEMADDR, BaudRateFraction_115dot2_26_0MHZ },

    /* The following entries are for setting up uart clock in pll mode*/
    /* uart divider integer */
    { URT0_R_UARTIBRD_MEMADDR, BaudRateInteger_115dot2_78_0MHZ },
    /* uart divider fraction */
    { URT0_R_UARTFBRD_MEMADDR, BaudRateFraction_115dot2_78_0MHZ }
};

/* 26000000Hz [1] - 700 MHz */
static const clk_reg_t plls_ref_CPU700_175[CMGR_PLLCLK_MAX] =
{
    /* pwrdn_chx=0, en_cmlbufx=0, ndiv_pwrdn=0, bypass_sdmod=0, refcomp_pwrdn=0, pwrdn=0*/
    { CMU_R_PLC_PLL_CTL0_MEMADDR,	0x0 },
    /* plc_dreset and plc_areset */
    { CMU_R_PLC_PLL_CTL1_MEMADDR,	0x0 },
    /* unused */
    { CMU_R_PLC_PLL_CTL2_MEMADDR,	0x0 },
    /* ndiv_dither_mfb=0, ndiv_mode=001, ndiv_int=0, p1div=0, p2div=0*/
    { CMU_R_PLC_PLL_CTL3_MEMADDR,	(\
        ((0x0 <<CMU_F_PLC_NDIV_DITHER_MFB_R) & CMU_F_PLC_NDIV_DITHER_MFB_MASK)|\
        ((PLL_NDIV_MODE <<CMU_F_PLC_NDIV_MODE_R) & CMU_F_PLC_NDIV_MODE_MASK) |\
        ((81 <<CMU_F_PLC_NDIV_INT_R) & CMU_F_PLC_NDIV_INT_MASK) |\
        ((1 <<CMU_F_PLC_P2DIV_R) & CMU_F_PLC_P2DIV_MASK) |\
        ((1 <<CMU_F_PLC_P1DIV_R) & CMU_F_PLC_P1DIV_MASK) )},
    /* ndiv_frac */
    { CMU_R_PLC_PLL_CTL4_MEMADDR,	(0x0 & CMU_F_PLC_NDIV_FRAC_MASK)},
    /* m1div - m4div */
    { CMU_R_PLC_PLL_CTL5_MEMADDR,	(\
        ((8 << CMU_F_PLC_M4DIV_R) & CMU_F_PLC_M4DIV_MASK)|\
        ((9 << CMU_F_PLC_M3DIV_R) & CMU_F_PLC_M3DIV_MASK)|\
        ((12 << CMU_F_PLC_M2DIV_R) & CMU_F_PLC_M2DIV_MASK)|\
        ((3<< CMU_F_PLC_M1DIV_R) & CMU_F_PLC_M1DIV_MASK))},
    /* m5div - m6div*/
    { CMU_R_PLC_PLL_CTL6_MEMADDR,	(\
        ((24 << CMU_F_PLC_M6DIV_R) & CMU_F_PLC_M6DIV_MASK)|\
        ((6 << CMU_F_PLC_M5DIV_R) & CMU_F_PLC_M5DIV_MASK))},
    /* plc_vco_rng, bypclkx, enb_clkout, bypen */
    { CMU_R_PLC_PLL_CTL7_MEMADDR,	CMU_F_PLC_VCO_RNG_MASK },

    /* pwrdn_chx=0, en_cmlbufx=0, ndiv_pwrdn=0, bypass_sdmod=0, refcomp_pwrdn=0, pwrdn=0*/
    { CMU_R_PLA_PLL_CTL0_MEMADDR,	0x0 },
    /* plc_dreset and plc_areset */
    { CMU_R_PLA_PLL_CTL1_MEMADDR,	0x0 },
    /* unused */
    { CMU_R_PLA_PLL_CTL2_MEMADDR,	0x0 },
    /* ndiv_dither_mfb=0, ndiv_mode=001, ndiv_int=0, p1div=0, p2div=0*/
    { CMU_R_PLA_PLL_CTL3_MEMADDR,	(\
        ((0x0 <<CMU_F_PLA_NDIV_DITHER_MFB_R) & CMU_F_PLA_NDIV_DITHER_MFB_MASK)|\
        ((PLL_NDIV_MODE <<CMU_F_PLA_NDIV_MODE_R) & CMU_F_PLA_NDIV_MODE_MASK) |\
        ((48 <<CMU_F_PLA_NDIV_INT_R) & CMU_F_PLA_NDIV_INT_MASK) |\
        ((1 <<CMU_F_PLA_P2DIV_R) & CMU_F_PLA_P2DIV_MASK) |\
        ((1 <<CMU_F_PLA_P1DIV_R) & CMU_F_PLA_P1DIV_MASK) )},
    /* ndiv_frac */
    { CMU_R_PLA_PLL_CTL4_MEMADDR,	(0x0 & CMU_F_PLA_NDIV_FRAC_MASK)},
    /* m1div - m4div */
    { CMU_R_PLA_PLL_CTL5_MEMADDR,	(\
        ((25 << CMU_F_PLA_M4DIV_R) & CMU_F_PLA_M4DIV_MASK)|\
        ((48 << CMU_F_PLA_M3DIV_R) & CMU_F_PLA_M3DIV_MASK)|\
        ((96 << CMU_F_PLA_M2DIV_R) & CMU_F_PLA_M2DIV_MASK)|\
        ((4 << CMU_F_PLA_M1DIV_R) & CMU_F_PLA_M1DIV_MASK))},
    /* m5div - m6div*/
    { CMU_R_PLA_PLL_CTL6_MEMADDR,	(\
        ((26 << CMU_F_PLA_M6DIV_R) & CMU_F_PLA_M6DIV_MASK)|\
        ((/*4*/24 << CMU_F_PLA_M5DIV_R) & CMU_F_PLA_M5DIV_MASK))},
    /* plc_vco_rng, bypclkx, enb_clkout, bypen */
    { CMU_R_PLA_PLL_CTL7_MEMADDR,	 0x0  },

    /* The following entries are for setting up uart clock in bypass mode*/
    /* uart divider integer */
    { URT0_R_UARTIBRD_MEMADDR, BaudRateInteger_115dot2_26_0MHZ },
    /* uart divider fraction */
    { URT0_R_UARTFBRD_MEMADDR, BaudRateFraction_115dot2_26_0MHZ },

    /* The following entries are for setting up uart clock in pll mode*/
    /* uart divider integer */
    { URT0_R_UARTIBRD_MEMADDR, BaudRateInteger_115dot2_78_0MHZ },
    /* uart divider fraction */
    { URT0_R_UARTFBRD_MEMADDR, BaudRateFraction_115dot2_78_0MHZ }
};


clk_reg_t *clk_reg_ref[CMGR_REFCLK_MAX] =
{
    (clk_reg_t*) plls_ref_0,
    (clk_reg_t*) plls_ref_CPU700_175,
};

static uint64_t init_cnt = 0;
static uint64_t tar_cnt = 0;

/** @brief Initialization function for utility calls using ripple counter.
 *
 * To use the the timeout utility, first call timeoutInit set the timeout value,
 * then call timeoutChk to check if the timeout is triggerred.
 *
 * @param[in] cnt delay count in ripple counter increments (32Khz)
 * @return int 0 if ok, 1 if error
 */
int timeoutInit(uint32_t cnt)
{
    uint32_t hword;
    init_cnt = readl(IO_ADDRESS(RPC_R_RPLC_TIMERLWORD_MEMADDR));
    hword = readl(IO_ADDRESS(RPC_R_RPLC_TIMERHWORD_MEMADDR));
    if(hword & 0x80000000U)
    {
        /* try one more time, failed here */
        init_cnt = readl(IO_ADDRESS(RPC_R_RPLC_TIMERLWORD_MEMADDR));
        hword = readl(IO_ADDRESS(RPC_R_RPLC_TIMERHWORD_MEMADDR));
        if(hword & 0x80000000U)
        {
            return 1;
        }
    }
    init_cnt |= ((uint64_t)hword << 32);
    tar_cnt = init_cnt + cnt;
    return 0;
}

/** @brief Check function for utility calls using ripple counter.
 *
 * To use the the timeout utility, first call timeoutInit set the timeout value,
 * then call timeoutChk to check if the timeout is triggerred.
 *
 * @return int 0 if no timeout, 1 if timeout or error
 */
int timeoutChk(void)
{
    uint64_t rplc_cur;
    uint32_t hword;
    static uint32_t tmpWord = 0;
    static uint32_t err_chk = 0;

    rplc_cur = readl(IO_ADDRESS(RPC_R_RPLC_TIMERLWORD_MEMADDR));
    hword = readl(IO_ADDRESS(RPC_R_RPLC_TIMERHWORD_MEMADDR));

    if(hword & 0x80000000U)
    {
        /* try one more time, failed here */
        rplc_cur = readl(IO_ADDRESS(RPC_R_RPLC_TIMERLWORD_MEMADDR));
        hword = readl(IO_ADDRESS(RPC_R_RPLC_TIMERHWORD_MEMADDR));
        if(hword & 0x80000000U)
        {
            return 1;
        }
    }
    rplc_cur |= ((uint64_t)hword << 32);

    if(tmpWord == (uint32_t)rplc_cur)
    {
        err_chk++;
    }
    else
    {
        tmpWord = (uint32_t)rplc_cur;
        err_chk = 0;
    }

    /* If the ripple counter does not increment, return error */

    if(err_chk > 1000000)
    {
        err_chk = 0;
        return 1;
    }

    if(rplc_cur >= init_cnt)
    {
        /* the ripple count did not wrap over */

        if(rplc_cur >= tar_cnt)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        /* the ripple count wrapped over */
        if((rplc_cur | 0x8000000000000000ULL) >= tar_cnt)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

}

/** @brief Delay in 32Khz ticks using ripple counter.
 *
 * param[in] cnt Number of 32Khz ticks to delay.
 * @return int 0 if no timeout, 1 if timeout or error
 */
int delay32kHz(uint32_t cnt)
{
    uint8_t ret;

    if(timeoutInit(cnt))
    {
        return 1;
    }

    do
    {
        ret = timeoutChk();
    } while(!ret);

    return ret;
}

/** @brief Write to CMU registers
 *
 * @param[in] regs The register address and value to write (a structure).
 * @param[in] size The size of the register in bytes.
 * @return Nothing.
 */  
static void _clocks_regs_set(const clk_reg_t *regs, int size) {
   while(size--)
   {
      writel(regs->val, IO_ADDRESS(regs->addr));
      ++regs;
   }

   return;
}

/** @brief Assert A_RESET on PLC and PLA
 *
 * @return Nothing.
 */  
static void _assert_a_reset(void) {
    writel((readl(IO_ADDRESS(CMU_R_PLC_PLL_CTL1_MEMADDR)) | CMU_F_PLC_ARESET_MASK),
           IO_ADDRESS(CMU_R_PLC_PLL_CTL1_MEMADDR));
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    writel((readl(IO_ADDRESS(CMU_R_PLC_PLL_CTL1_MEMADDR)) & ~CMU_F_PLC_ARESET_MASK),
           IO_ADDRESS(CMU_R_PLC_PLL_CTL1_MEMADDR));

    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    writel((readl(IO_ADDRESS(CMU_R_PLA_PLL_CTL1_MEMADDR)) | CMU_F_PLA_ARESET_MASK),
           IO_ADDRESS(CMU_R_PLA_PLL_CTL1_MEMADDR));
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    writel((readl(IO_ADDRESS(CMU_R_PLA_PLL_CTL1_MEMADDR)) & ~CMU_F_PLA_ARESET_MASK),
           IO_ADDRESS(CMU_R_PLA_PLL_CTL1_MEMADDR));
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
   return;
}

/** @brief Assert D_RESET on PLC and PLA
 *
 * @return Nothing.
 */  
static void _assert_d_reset(void) {
    writel((readl(IO_ADDRESS(CMU_R_PLC_PLL_CTL1_MEMADDR)) | CMU_F_PLC_DRESET_MASK),
           IO_ADDRESS(CMU_R_PLC_PLL_CTL1_MEMADDR));
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    writel((readl(IO_ADDRESS(CMU_R_PLC_PLL_CTL1_MEMADDR)) & ~CMU_F_PLC_DRESET_MASK),
           IO_ADDRESS(CMU_R_PLC_PLL_CTL1_MEMADDR));

    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    writel((readl(IO_ADDRESS(CMU_R_PLA_PLL_CTL1_MEMADDR)) | CMU_F_PLA_DRESET_MASK),
           IO_ADDRESS(CMU_R_PLA_PLL_CTL1_MEMADDR));
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    writel((readl(IO_ADDRESS(CMU_R_PLA_PLL_CTL1_MEMADDR)) & ~CMU_F_PLA_DRESET_MASK),
           IO_ADDRESS(CMU_R_PLA_PLL_CTL1_MEMADDR));
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
   return;
}

/** @brief Write to CMU PLL Registers. Requires delays and other special handling.
 *
 * @param[in] regs The register address and value to write (a structure).
 * @param[in] size The size of the register in bytes.
 * @return int 0 if ok, 1 if error.
 */  
static int _clocks_plls_regs_set(const clk_reg_t *regs, size_t size)
{
	int res = 0;
	uint32_t _isTout;

	/* set PLLs registers  */
	_clocks_regs_set (regs,size);

	/* asert ARESET to PLC&PLA  */
	_assert_a_reset();

	/* wait for PLLs (PLC & PLA) to lock in. Per Bhavi, we need to wait up to 400 us */
	_isTout = 1;
	timeoutInit(13);			/* Wait up to 13x31.25us (~406us)*/
		
	while(!timeoutChk())
	{
		if ((readl(IO_ADDRESS(CMU_R_PLL_STS_MEMADDR)) & (CMU_F_PLC_PLL_LOCK_MASK + CMU_F_PLA_PLL_LOCK_MASK)) ==
			(CMU_F_PLC_PLL_LOCK_MASK + CMU_F_PLA_PLL_LOCK_MASK))
		{
			/* PLL is locked ok */
			_isTout = 0;
			break;
		}
	}

	if(!((readl(IO_ADDRESS(CMU_R_PLL_STS_MEMADDR)) & (CMU_F_PLC_PLL_LOCK_MASK + CMU_F_PLA_PLL_LOCK_MASK)) == (CMU_F_PLC_PLL_LOCK_MASK + CMU_F_PLA_PLL_LOCK_MASK)) && (_isTout == 1))
	{
		res = 1;
	}

	/* Assert DRESET for PLC&PLA */
	_assert_d_reset();

	return res;
}

/** @brief Bypass PLC to allow frequency changing.
 *
 * @return Nothing. 
 */  
void cds_plc_bypass(void)
{
    /* enable bypass mode and edge trigger freq change sequence */
    writel((readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)) & ~(CMU_F_PLC_BYPASS_DISABLE_MASK | CMU_F_CHANGE_CORE_FREQ_MASK)),
           IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
	
    writel((readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)) | CMU_F_CHANGE_CORE_FREQ_MASK),
           IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;		
}

/** @brief Bypass PLA to allow frequency changing.
 *
 * @return Nothing. 
 */  
static void cds_pla_bypass(void)
{
    /* enable bypass mode and edge trigger freq change sequence */
    writel((readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)) & ~(CMU_F_PLA_BYPASS_DISABLE_MASK | CMU_F_CHANGE_CORE_FREQ_MASK)),
           IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));					 
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;

    writel((readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)) | CMU_F_CHANGE_CORE_FREQ_MASK),
           IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
}

/** @brief Initialize the PLL based upon the table index provided.
 *
 * @param[in] ref_clk_idx index into clock setup table.
 * @return int 0 if ok, 1 if error.
 */  
int cds_pll_init(uint32_t ref_clk_idx)
{
    /* unlock clock manager */
    __UNLOCK_CMU();

    /* Since we're running on DDR so we need to enable wait-for-DDR lock when CMU do the freq switching */
    writel((readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)) | (CMU_F_WAIT_DDR_LOCK_MASK)),
           IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));

    /* enable bypass mode */
    cds_plc_bypass();
    cds_pla_bypass();

    /* configure PLLs and wait for lock*/
    if (_clocks_plls_regs_set(clk_reg_ref[ref_clk_idx],(CMGR_PLLCLK_MAX-NUMBER_UART_ENTRIES)))
    {
        /* lock access to clock manager */
        __LOCK_CMU();
        return 1;
    }
    else
    {
        /* disable pla&plc bypass mode using hw controlled method */
        writel((readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)) | CMU_F_SM_CLKSWITCH_MASK),
               IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));

		    ASM_NOP;
    		ASM_NOP;
    		ASM_NOP;
    		ASM_NOP;
				ASM_NOP;
		    ASM_NOP;
    		ASM_NOP;
    		ASM_NOP;
    		ASM_NOP;
				ASM_NOP;

        /* disable pla&plc bypass mode */
        writel((readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)) | (CMU_F_PLC_BYPASS_DISABLE_MASK+CMU_F_PLA_BYPASS_DISABLE_MASK)),
               IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));

		    ASM_NOP;
    		ASM_NOP;
    		ASM_NOP;
    		ASM_NOP;
				ASM_NOP;
		    ASM_NOP;
    		ASM_NOP;
    		ASM_NOP;
    		ASM_NOP;
				ASM_NOP;

       /* Use asynB mode for 700/667MHz */
        
       if(ref_clk_idx == 1)
	    	writel(((readl(IO_ADDRESS(CMU_R_MODE_CTL_MEMADDR)) & ~CMU_F_CMU_ARM_CLKMODE_MASK) |
	    		(1 << CMU_F_CMU_ARM_CLKMODE_R)), IO_ADDRESS(CMU_R_MODE_CTL_MEMADDR));

        /* edge trigger freq change sequence */
        writel((readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)) & ~CMU_F_CHANGE_CORE_FREQ_MASK),
               IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));

				ASM_NOP;
				ASM_NOP;
				ASM_NOP;
				ASM_NOP;
				ASM_NOP;
				ASM_NOP;
				ASM_NOP;
				ASM_NOP;
				ASM_NOP;
				ASM_NOP;

        writel((readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)) | CMU_F_CHANGE_CORE_FREQ_MASK),
               IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));
    }
	
    /* lock access to clock manager */
    __LOCK_CMU();
	
    return 0;
}


/** @brief Switch to alternate of indexed PLL table entry. 
 *
 * @param[in] ref_clk_idx index into clock setup table.
 * @return int 0 if ok, 1 if error.
 */  
int cds_pll_switch(uint32_t ref_index, int alternate)
{
    volatile uint32_t temp;
    int res=0;

    /* unlock clock manager */
    __UNLOCK_CMU();
		
		temp = readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));
		temp |= (CMU_F_PLC_BYPASS_DISABLE_MASK + CMU_F_PLA_BYPASS_DISABLE_MASK);
		if (alternate)
			temp |= CMU_F_SELECT_ALTERNATE_MASK;
		else
			temp &= ~CMU_F_SELECT_ALTERNATE_MASK;
					
		writel(temp, IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));
		
    /* edge trigger freq change sequence */
    writel((readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)) & ~CMU_F_CHANGE_CORE_FREQ_MASK),
           IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));
    
		/* hw requires to delay a bit (3clks) after switching the clocks */
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
					 
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
			
    writel((readl(IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR)) | CMU_F_CHANGE_CORE_FREQ_MASK),
           IO_ADDRESS(CMU_R_CLOCK_GATE_CTL_MEMADDR));

    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;

    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
    ASM_NOP;
		
    /* lock access to clock manager */
    __LOCK_CMU();

    return res;
}
