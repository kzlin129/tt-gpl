/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
/** 
 * 
 *   @file   chipmgr.c 
 * 
 *   @brief  Chip manager APIs
 * 
 ****************************************************************************/

#include "bcm_basetypes.h"
#include "bcm_privtypes.h"
#include "bcm_basedefs.h"
#include "bcm_os_support.h"
#include "bcm_log.h"
#include "bcm_amba_io.h"

#include "chipmgr.h"
// #include "chipmgr_psh.h"
#include "chipmgr_regs.h"
#include <asm-arm/arch-bcm4760/bcm4760_addressmap.h>

#define CHIPMGR_OSC_ON_WAIT_TIME_USEC (10*1000)  // 10 milliseconds to wait to ensure oscillator is ON.
#define CHIPMGR_EMI_DLL_RST_COUNTER_DEFAULT 150

/* Chip manager block base address */
static chipmgr_regs_t *pchipmgr_base_addr = 0;
                                    
//--------------------------------------------------
/**
    Chip manager public APIs
*/
//--------------------------------------------------
/**
    @fn void chipmgr_api_init(void)
*/
void chipmgr_api_init(void)
{
//    pchipmgr_base_addr = (chipmgr_regs_t *)bcm_xlate_to_virt_addr(BCM28XX_CHIPMGR_ARM_ADDRBASE0);
    pchipmgr_base_addr = (chipmgr_regs_t *)bcm_xlate_to_virt_addr(CMU_REG_BASE_ADDR);

    //Initialize GPIO addresses.
    // chipmgr_gpio_api_init();
}


/**
    @fn uint32_t chipmgr_set_emi_dram_clkmode(chipmgr_dram_clkmode_t mode);
*/
uint32_t chipmgr_set_emi_dram_clkmode(chipmgr_dram_clkmode_t mode)
{
    chipmgr_perictrl0_u_t perip0_val;

    //read-modify-write
    perip0_val.w_data = amba_reg_read32(pchipmgr_base_addr, chip_perip_ctrl0);

	switch(mode)
	{
		case ECHIPMGR_EMI_DRAM_1by1:
		case ECHIPMGR_EMI_DRAM_2by1:
        perip0_val.bf.clkmode_2by3 = 0;
        perip0_val.bf.clk_emi_eq_dram = 1;
		break;

		case ECHIPMGR_EMI_DRAM_3by2:
        perip0_val.bf.clkmode_2by3 = 1;
        perip0_val.bf.clk_emi_eq_dram = 1;
		break;

		case ECHIPMGR_EMI_DRAM_ASYNC:
        perip0_val.bf.clkmode_2by3 = 0;
        perip0_val.bf.clk_emi_eq_dram = 0;
		break;

		default:
		BCM_ASSERT(!"(Attempting to set invalid EMI mode in CHIPMGR!");
		return BCM_ERROR;
	}

	/* during a clock switch, a register must be done with the
	 * the shadow bit (bit 31 aka emi_clk_req_extend) turned on, followed
	 *  by a register write with that bit turned off
     */
	perip0_val.bf.emi_clk_req_extend = 1;
    amba_reg_write32(pchipmgr_base_addr, chip_perip_ctrl0, perip0_val.w_data);
	perip0_val.bf.emi_clk_req_extend = 0;
    amba_reg_write32(pchipmgr_base_addr, chip_perip_ctrl0, perip0_val.w_data);
        
    return(BCM_OK);
}

/**
    @fn uint32_t chipmgr_enable_vc_sram(bool_t enable);
*/
uint32_t chipmgr_enable_vc_sram(bool_t enable)
{
    chipmgr_perictrl0_u_t perip0_val;

    //read-modify-write
    perip0_val.w_data = amba_reg_read32(pchipmgr_base_addr, chip_perip_ctrl0);

	if(enable == FALSE )
	{
		//set to STBY mode
		perip0_val.bf.enable_10mbit = 0;
		perip0_val.bf.stby_10mbit = 1;

	}
	else
	{
		perip0_val.bf.enable_10mbit = 1;
		perip0_val.bf.stby_10mbit = 0;
	}

    amba_reg_write32(pchipmgr_base_addr, chip_perip_ctrl0, perip0_val.w_data);
	return BCM_OK;
}


/**
    @fn chipmgr_dram_clkmode_t chipmgr_get_emi_dram_clkmode(void);
*/
chipmgr_dram_clkmode_t chipmgr_get_emi_dram_clkmode(void)
{      
    chipmgr_perictrl0_u_t perip0_val;

    perip0_val.w_data = amba_reg_read32(pchipmgr_base_addr, chip_perip_ctrl0);

    if ((perip0_val.bf.clk_emi_eq_dram == 0) && (perip0_val.bf.clkmode_2by3 == 0))
    {
        return(ECHIPMGR_EMI_DRAM_ASYNC);
    }
    else if ((perip0_val.bf.clk_emi_eq_dram == 1) && (perip0_val.bf.clkmode_2by3 == 0))
    {
        //The value '01' implies 1by1 or 2by1 mode. 
        return(ECHIPMGR_EMI_DRAM_1by1);
    }
    else if ((perip0_val.bf.clk_emi_eq_dram == 1) && (perip0_val.bf.clkmode_2by3 == 1))
    {
        return(ECHIPMGR_EMI_DRAM_3by2);
    }

	BCM_ASSERT(!"Chipmgr Peripheral Control Register 0 has invalid dram settings!");
    return(ECHIPMGR_EMI_DRAM_1by1); /* this is really invalid */
}


/**
    @fn void chipmgr_set_emi_dll_rst_counter(void);
*/
void chipmgr_set_emi_dll_rst_counter(void)
{
    chipmgr_iisctrl_u_t iis_ctrl_val;

    iis_ctrl_val.w_data = amba_reg_read32(pchipmgr_base_addr, iis_ctrl);
    iis_ctrl_val.bf.emi_dll_rst_counter = CHIPMGR_EMI_DLL_RST_COUNTER_DEFAULT;  
    amba_reg_write32(pchipmgr_base_addr, iis_ctrl, iis_ctrl_val.w_data);

    return;
}


/**
    @fn uint32_t chipmgr_config_usb(chipmgr_usbmode_t usb_mode);
*/
uint32_t chipmgr_config_usb(chipmgr_usbmode_t usb_mode)
{
    chipmgr_usbctrl_u_t usb_ctrl_val;

    BCM_ASSERT(usb_mode == ECHIPMGR_USBMODE_DEVICE || 
               usb_mode == ECHIPMGR_USBMODE_HOST   || 
               usb_mode == ECHIPMGR_USBMODE_IDDQ);

    if (usb_mode == ECHIPMGR_USBMODE_IDDQ)
    {
        usb_ctrl_val.w_data = 0;
        usb_ctrl_val.bf.usb_phy_pll_clksel_i     = 0;    
        usb_ctrl_val.bf.usb_phy_tp_mux_sel_i     = 0;   
        usb_ctrl_val.bf.usb_phy_tp_phy_sel_i     = 0;   
        usb_ctrl_val.bf.usb_phy_tp_port_sel_i    = 0;   
        usb_ctrl_val.bf.usb_phy_pll_bypass       = 0;  
        usb_ctrl_val.bf.usb_phy_iddq_en          = 1;  
        usb_ctrl_val.bf.usb_phy_resetb_i         = 0;  
        usb_ctrl_val.bf.usb_phy_reset_hi_usb_pll = 1;
        usb_ctrl_val.bf.usb_pll_suspend_en_i     = 1;
        usb_ctrl_val.bf.usb_host_mode            = 0;
        amba_reg_write32(pchipmgr_base_addr, usb_ctrl, usb_ctrl_val.w_data);
    }
    else
    {
        usb_ctrl_val.w_data = amba_reg_read32(pchipmgr_base_addr, usb_ctrl);
        usb_ctrl_val.bf.usb_phy_iddq_en = 0; 
        usb_ctrl_val.bf.usb_phy_reset_hi_usb_pll = 0;
        usb_ctrl_val.bf.usb_pll_suspend_en_i     = 0;         
        usb_ctrl_val.bf.usb_host_mode   = usb_mode;
        amba_reg_write32(pchipmgr_base_addr, usb_ctrl, usb_ctrl_val.w_data);
    }

    return(BCM_OK);
}


/**
    @fn uint32_t chipmgr_reset_usb_blk(chipmgr_usbblk_t usb_blk);
*/
uint32_t chipmgr_reset_usb_blk(chipmgr_usbblk_t usb_blk)
{
    chipmgr_usbctrl_u_t usb_ctrl_val;

    usb_ctrl_val.w_data = amba_reg_read32(pchipmgr_base_addr, usb_ctrl);

    if (usb_blk == ECHIPMGR_USBBLK_PHY)
    {
        usb_ctrl_val.bf.usb_phy_resetb_i = BCM_BIT_CLR;
    }
    else if (usb_blk == ECHIPMGR_USBBLK_PLL)
    {
        usb_ctrl_val.bf.usb_phy_reset_hi_usb_pll = BCM_BIT_SET;
    }
    else
    {
		BCM_ASSERT(!"Invalid usb_blk!");
        return(BCM_ERROR);
    }  

    amba_reg_write32(pchipmgr_base_addr, usb_ctrl, usb_ctrl_val.w_data);

    return(BCM_OK);
}


/**
    @fn uint32_t chipmgr_config_rng_iddq(bcm_cfg_stat_t stat);
*/
uint32_t chipmgr_config_rng_iddq(bcm_cfg_stat_t stat)
{
    chipmgr_perictrl0_u_t perip0_val;

    perip0_val.w_data = amba_reg_read32(pchipmgr_base_addr, chip_perip_ctrl0);
    perip0_val.bf.rng_iddq_en = stat;
    amba_reg_write32(pchipmgr_base_addr, chip_perip_ctrl0, perip0_val.w_data);

    return(BCM_OK);
}

/**
    @fn uint32_t chipmgr_config_otp_iddq(bcm_cfg_stat_t stat);
*/
uint32_t chipmgr_config_otp_iddq(bcm_cfg_stat_t stat)
{
    chipmgr_perictrl0_u_t perip0_val;

    perip0_val.w_data = amba_reg_read32(pchipmgr_base_addr, chip_perip_ctrl0);
    perip0_val.bf.otp_iddq_en = stat;
    amba_reg_write32(pchipmgr_base_addr, chip_perip_ctrl0, perip0_val.w_data);

    return(BCM_OK);
}


/**
    @fn uint32_t chipmgr_config_oscillators(chipmgr_clk_t clk, chipmgr_clkstat_t stat, bool_t should_wait);
*/
uint32_t chipmgr_config_oscillators(chipmgr_clk_t clk, chipmgr_clkstat_t stat, bool_t should_wait)
{
    chipmgr_perictrl0_u_t perip0_val;

    // retain operation for oscialltor. No action when retain.
    if ( stat == ECHIPMGR_CLK_RETAIN ) 
    {
        // bcm_log_info("**** Oscillator %d left retained... *** \n", clk) ;        
        return(BCM_OK);
    }

    perip0_val.w_data = amba_reg_read32(pchipmgr_base_addr, chip_perip_ctrl0);

    if (clk == ECHIPMGR_CLK_24MHZ)
    {
		should_wait = should_wait && (stat==ECHIPMGR_CLK_ON) && (perip0_val.bf.xtal24_xtalpd==1);
		perip0_val.bf.xtal24_xtalpd = (stat==ECHIPMGR_CLK_ON) ? 0 : 1 ;
    }
    else if (clk == ECHIPMGR_CLK_27MHZ)
    {
		should_wait = should_wait && (stat==ECHIPMGR_CLK_ON) && (perip0_val.bf.xtal27_xtalpd==1);
        perip0_val.bf.xtal27_xtalpd = (stat==ECHIPMGR_CLK_ON) ? 0 : 1;
    }
    else if (clk == ECHIPMGR_CLK_24_AND_27MHZ)
    {
		should_wait = should_wait && (stat==ECHIPMGR_CLK_ON) && (perip0_val.bf.xtal27_xtalpd==1 || perip0_val.bf.xtal24_xtalpd==1);
        perip0_val.bf.xtal24_xtalpd = (stat==ECHIPMGR_CLK_ON) ? 0 : 1;
        perip0_val.bf.xtal27_xtalpd = (stat==ECHIPMGR_CLK_ON) ? 0 : 1;
    }
    else if (clk == ECHIPMGR_CLK_32KHZ)
    {
        //can't control the 32KHz crystal
        return(BCM_OK);
    }
    else
    {
		BCM_ASSERT(!"Invalid oscillator value!");
        return(BCM_ERROR);
    }  

    amba_reg_write32(pchipmgr_base_addr, chip_perip_ctrl0, perip0_val.w_data);

    if (should_wait)
    {
        bcm_delay_usec(CHIPMGR_OSC_ON_WAIT_TIME_USEC);
    }  

    return(BCM_OK);
}


/**
    @fn bool_t chipmgr_is_oscillator_on(chipmgr_clk_t clk);
*/
bool_t chipmgr_is_oscillator_on(chipmgr_clk_t clk)
{
    chipmgr_perictrl0_u_t perip0_val;

    BCM_ASSERT(clk < ECHIPMGR_CLK_24_AND_27MHZ);

    perip0_val.w_data = amba_reg_read32(pchipmgr_base_addr, chip_perip_ctrl0);

    if (clk == ECHIPMGR_CLK_24MHZ)
    {
		return (perip0_val.bf.xtal24_xtalpd == 0);
    }
    else if (clk == ECHIPMGR_CLK_27MHZ)
    {
        return (perip0_val.bf.xtal27_xtalpd == 0);
    }
    else if (clk == ECHIPMGR_CLK_32KHZ)
    {
        return TRUE; //always on
    }
	else
	{
		BCM_ASSERT(!"Invalid clk value");
		return FALSE;
	}
}


uint32_t	chipmgr_set_pll_config(chipmgr_pll_t pll, chipmgr_pll_config_t pll_config)
{
	chipmgr_pll_ctrl_u_t	chipmgr_pll;

	BCM_ASSERT(pll < CHIPMGR_PLL_COUNT); 

    chipmgr_pll.w_data = amba_reg_read32(pchipmgr_base_addr, pll_ctrl[pll]);
	switch(pll_config)
	{
		case ECHIPMGR_PLL_CONFIG_NORMAL:
		chipmgr_pll.bf.bypen_p = 0;
		break;

		case ECHIPMGR_PLL_BYPASS_WITH_24MHZ:
		chipmgr_pll.bf.bypen_p = 1;
		break;

		case ECHIPMGR_PLL_BYPASS_WITH_32KHZ:
		chipmgr_pll.bf.bypen_p = 2;
		break;

		default:
		BCM_ASSERT(!"Invalid PLL config");
		return BCM_ERROR;
	}

	amba_reg_write32(pchipmgr_base_addr, pll_ctrl[pll], chipmgr_pll.w_data);
	return BCM_OK;
}

chipmgr_pll_config_t	chipmgr_get_pll_config(chipmgr_pll_t pll)
{
	chipmgr_pll_ctrl_u_t	chipmgr_pll;

	BCM_ASSERT(pll < CHIPMGR_PLL_COUNT); 

    chipmgr_pll.w_data = amba_reg_read32(pchipmgr_base_addr, pll_ctrl[pll]);
	switch(chipmgr_pll.bf.bypen_p)
	{
		case 0:
		return ECHIPMGR_PLL_CONFIG_NORMAL;

		case 1:
		return ECHIPMGR_PLL_BYPASS_WITH_24MHZ;

		case 2:
		return ECHIPMGR_PLL_BYPASS_WITH_32KHZ;

		default:
		BCM_ASSERT(!"Invalid PLL config");
		return ECHIPMGR_PLL_CONFIG_NORMAL;
	}
}

/**
    @fn uint32_t chipmgr_config_vc2_audioblk_control(chipmgr_audio_blk_freq_t freq_mode);
*/
uint32_t chipmgr_config_vc2_audioblk_control(chipmgr_audio_blk_freq_t freq_mode)
{
	chipmgr_iisctrl_u_t	iisctrl_val;

	BCM_ASSERT(freq_mode == ECHIPMGR_AUDIO_BLK_12MHZ ||
	           freq_mode == ECHIPMGR_AUDIO_BLK_22_579474MHZ ||
	           freq_mode == ECHIPMGR_AUDIO_BLK_12_288MHZ    ||
	           freq_mode == ECHIPMGR_AUDIO_BLK_24_576MHZ);

    iisctrl_val.w_data = amba_reg_read32(pchipmgr_base_addr, iis_ctrl);    
    iisctrl_val.bf.aud_bclk_ctrl = freq_mode;

	amba_reg_write32(pchipmgr_base_addr, iis_ctrl, iisctrl_val.w_data);	    

    return(BCM_OK);
}

/**
    @fn uint32_t chipmgr_config_spi(chipmgr_spi_port_t spi_port, bcm_cfg_stat_t stat);
 */
uint32_t chipmgr_config_spi(chipmgr_spi_port_t spi_port, bcm_cfg_stat_t stat)
{
    chipmgr_perictrl0_u_t perip0_val;

    perip0_val.w_data = amba_reg_read32(pchipmgr_base_addr, chip_perip_ctrl0);

    if (spi_port == ECHIPMGR_SPI_PORT0)
    {
// BCM2820 SPI0 and SPI1 enable bits in the chip manager peripheral control register are swapped.
//		perip0_val.bf.spi0_en = (stat==disable) ? 0 : 1;
		perip0_val.bf.spi1_en = (stat==disable) ? 0 : 1;
    }
    else if (spi_port == ECHIPMGR_SPI_PORT1)
    {
// BCM2820 SPI0 and SPI1 enable bits in the chip manager peripheral control register are swapped.
//		perip0_val.bf.spi1_en = (stat==disable) ? 0 : 1;
		perip0_val.bf.spi0_en = (stat==disable) ? 0 : 1;
    } 
    
    amba_reg_write32(pchipmgr_base_addr, chip_perip_ctrl0, perip0_val.w_data);
    
    return(BCM_OK);       
}



#define PINSHARE_REG_8_CONNECT_ARM_I2S   0xF00BA8C0
#define PINSHARE_REG_8_CONNECT_VC_I2S    0xF00300C0

/**
    @fn uint32_t chipmgr_config_pinshare(chipmgr_pinshare_mode_t mode);
*/
uint32_t chipmgr_config_pinshare(chipmgr_pinshare_mode_t mode)
{
    uint32_t ret = BCM_OK;
    chipmgr_pin_ctrl_u_t pin_val;

    switch(mode)
    {
        case ECHIPMGR_CONNECT_ARM_I2S:
        {
        	amba_reg_write32(pchipmgr_base_addr, pin_share_ctrl[8], PINSHARE_REG_8_CONNECT_ARM_I2S);            

            pin_val.w_data = amba_reg_read32(pchipmgr_base_addr, pin_share_ctrl[8]);
            bcm_log_debug("%s: [%p][0x%08x]\n", __FUNCTION__, &pchipmgr_base_addr->pin_share_ctrl[8], pin_val.w_data);
        }
        break;

        case ECHIPMGR_CONNECT_VC_I2S:
        {
        	amba_reg_write32(pchipmgr_base_addr, pin_share_ctrl[8], PINSHARE_REG_8_CONNECT_VC_I2S);             
            pin_val.w_data = amba_reg_read32(pchipmgr_base_addr, pin_share_ctrl[8]);
            bcm_log_debug("%s: [%p][0x%08x]\n", __FUNCTION__, &pchipmgr_base_addr->pin_share_ctrl[8], pin_val.w_data);
        }
        break;

        default:
        {
            ret = BCM_ERROR;
        }
        break;
    }

    return(ret);
}

/**
    @fn chipmgr_regs_t* get_chipmgr_base_address(void);
*/
chipmgr_regs_t * get_chipmgr_base_address(void)
{
	return pchipmgr_base_addr;
}


