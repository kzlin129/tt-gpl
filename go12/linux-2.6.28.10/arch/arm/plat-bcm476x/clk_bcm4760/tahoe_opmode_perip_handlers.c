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
 *   @file   tahoe_opmode_perip_handlers.c 
 * 
 *   @brief  Perip handlers, special functions.
 * 
 ****************************************************************************/


#include "bcm_basetypes.h"
#include "bcm_basedefs.h"
#include "bcm_log.h"

#include "chipmgr.h"
// #include "usbphy.h"
// #include "vec.h"

#include "tahoe_opmode_priv.h"
// #include "tahoe_psh.h"

static int default_core_custom_logic(bcm_cfg_stat_t stat, void *data);

#if 0
static int core_nor_custom_logic(bcm_cfg_stat_t stat, void *data);
static int core_usb_custom_logic(bcm_cfg_stat_t stat, void *data);
static int core_otp_custom_logic(bcm_cfg_stat_t stat, void *data);
static int core_rng_custom_logic(bcm_cfg_stat_t stat, void *data);
static int core_vcpu_custom_logic(bcm_cfg_stat_t stat, void *data);
static int core_vtvo_vec_custom_logic(bcm_cfg_stat_t stat, void *data) ;
static int core_spdif_custom_logic(bcm_cfg_stat_t stat, void *data);
static int core_vi2s_custom_logic(bcm_cfg_stat_t stat, void *data);
static int core_spi0_custom_logic(bcm_cfg_stat_t stat, void *data);
static int core_spi1_custom_logic(bcm_cfg_stat_t stat, void *data);
#endif

typedef int (*core_specific_processor)(bcm_cfg_stat_t stat, void *);

static core_specific_processor perip_handlers[OPMODE_PERIP_CLKCORE_COUNT] =
{
    default_core_custom_logic,  // CORE_IDE,      //0
    default_core_custom_logic,  // CORE_CEATA,      
    default_core_custom_logic,  // CORE_NAND,      
    default_core_custom_logic,  // core_usb_custom_logic,      // CORE_USB,      

    default_core_custom_logic,  // CORE_SDIO0,    //4  
    default_core_custom_logic,  // CORE_SDIO1,      
    default_core_custom_logic,  // CORE_VFIR,      
    default_core_custom_logic,  // CORE_CRPT,      

    default_core_custom_logic,  // CORE_AMC,      //8 
    default_core_custom_logic,  // CORE_PWM,      
    default_core_custom_logic,  // CORE_UART0,      
    default_core_custom_logic,  // CORE_UART1,      

    default_core_custom_logic,  // CORE_UART2,    //12 
    default_core_custom_logic,  // CORE_PKE,      
    default_core_custom_logic,  // core_otp_custom_logic,      // CORE_OTP,      
    default_core_custom_logic,  // CORE_TIM,      

    default_core_custom_logic,  // CORE_TWSPI,    //16 
    default_core_custom_logic,  // core_spi0_custom_logic,     // CORE_SPI0,      
    default_core_custom_logic,  // core_spi1_custom_logic,     // CORE_SPI1,      
    default_core_custom_logic,  // CORE_INTC,      

    default_core_custom_logic,  // core_rng_custom_logic,      // CORE_RNG,      //20
    default_core_custom_logic,  // CORE_RTC,      
    default_core_custom_logic,  // CORE_I2C,      
    default_core_custom_logic,  // CORE_SYSM,      

    default_core_custom_logic,  // CORE_GPIO,     //24
    default_core_custom_logic,  // CORE_PM,      
    default_core_custom_logic,  // CORE_I2S,      
    default_core_custom_logic,  // CORE_RMP,      

    default_core_custom_logic,  // CORE_RPC,      //28 
    default_core_custom_logic,  // CORE_WDT,      
                               
    default_core_custom_logic,  // CORE_MMTX,     //30
    default_core_custom_logic,  // CORE_MDMA,
    default_core_custom_logic,  // CORE_MROM,     
    default_core_custom_logic,  // core_nor_custom_logic,	//_MNOR,     
                                      
    default_core_custom_logic,  // CORE_MSRAM,    //34   
    default_core_custom_logic,  // CORE_MEMI, 
    default_core_custom_logic,  // CORE_MIPS,    
    default_core_custom_logic,  // CORE_CUART,    
                                   
    default_core_custom_logic,  // CORE_CI2S,     //38
    default_core_custom_logic,  // CORE_CDC,
    default_core_custom_logic,  // CORE_CSMI,     
    default_core_custom_logic,  // core_vcpu_custom_logic,  // CORE_VCPU,     
                                      
    default_core_custom_logic,  // CORE_VCAM,     //42
    default_core_custom_logic,  // CORE_VBG,
    default_core_custom_logic,  // CORE_VINT,    
    default_core_custom_logic,  // CORE_VTIM,    
                                     
    default_core_custom_logic,  // CORE_VI2C,     //46
    default_core_custom_logic,  // core_vi2s_custom_logic,     // CORE_VI2S,
    default_core_custom_logic,  // core_spdif_custom_logic,    // CORE_SPDIF,
    default_core_custom_logic,  // core_vtvo_vec_custom_logic, // CORE_VTVO,     
                                       
    default_core_custom_logic,  // CORE_ASTVO,    //50                                     
    default_core_custom_logic,  // CORE_GEN, 
};                                                                                                                    

tahoe_stat_t tahoe_opmode_perip_custom_handler(tahoe_opmode_core_t core, bcm_cfg_stat_t stat, void *data)
{
    tahoe_stat_t ret = OpOk;

    BCM_ASSERT(core < OPMODE_PERIP_CLKCORE_COUNT);
    
    //Perform peripheral specific handling now..
    if (perip_handlers[core](stat, data) != BCM_OK)
    {
        bcm_log_warn("%s: perip_custom_handler(%d) failed\n", __FUNCTION__, core);
        ret = OpPeripCustomLogicFailed;
    }

    return(ret);
}

static int default_core_custom_logic(bcm_cfg_stat_t stat, void *data)
{
    //bcm_log_debug("%s: executing default custom logic api\n", __FUNCTION__);
    return(BCM_OK);
}

#if 0
static int core_usb_custom_logic(bcm_cfg_stat_t stat, void *data)
{
    uint32_t ret = BCM_OK;

    if (stat == disable)
    {
        ret = chipmgr_config_usb(ECHIPMGR_USBMODE_IDDQ);
        if (ret != BCM_OK)
        {
            bcm_log_error("%s: chipmgr_config_usb() failed\n", __FUNCTION__);
        }

        /**
            HACK ALERT!!
            We are shutting down USB core twice because the USB does not get
            into IDDQ power-save mode otherwise.
         */
        ret = usbphy_shutdown();
        if (ret != BCM_OK)
        {
            bcm_log_error("%s: usbphy_shutdown() failed\n", __FUNCTION__);
        }

        ret = usbphy_shutdown();
        if (ret != BCM_OK)
        {
            bcm_log_error("%s: usbphy_shutdown() failed\n", __FUNCTION__);
        }
    }
    else if (stat == enable)
    {
        //TODO: Waiting for response from IC team on what to do when enabling USB core!!
        ret = usbphy_init();
        if (ret != BCM_OK)
        {
            bcm_log_error("%s: usbphy_init() failed\n", __FUNCTION__);
        }

        ret = chipmgr_config_usb(ECHIPMGR_USBMODE_DEVICE);
        if (ret != BCM_OK)
        {
            bcm_log_error("%s: chipmgr_config_usb() failed\n", __FUNCTION__);
        }
    }

    return(ret);
}

static int core_otp_custom_logic(bcm_cfg_stat_t stat, void *data)
{
    //put OTP in IDDQ when it's being disable to reduce power consumption
    if (stat == disable)
    {
        chipmgr_config_otp_iddq(enable);
    }
    else
    {
        chipmgr_config_otp_iddq(disable);
    }

    return(BCM_OK);
}

static int core_rng_custom_logic(bcm_cfg_stat_t stat, void *data)
{
    //put RNG in IDDQ when it's being disable to reduce power consumption
    if (stat == disable)
    {
        chipmgr_config_rng_iddq(enable);
    }
    else
    {
        chipmgr_config_rng_iddq(disable);
    }

    return(BCM_OK);
}

static int core_nor_custom_logic(bcm_cfg_stat_t stat, void *data)
{
    int res = BCM_ERROR;
    tahoe_stat_t fret = OpOk;
    tahoe_opmode_mode_t current_mode = OPMODE_DEFAULT_MODE;
    char current_mode_str[MAX_MODE_STR_SZ];

    if ((fret = tahoe_opmode_get_mode(current_mode_str)) != OpOk)
        return(res);

    current_mode = map_modestr_to_opmode(current_mode_str);

    /**
        Note: If this is invoked from application with LCD enabled, the UI disappears as
              LCD and NOR share the pins. 
     */  
    switch(stat)
    {
        case enable:
            res = tahoe_psh_enable_nor();
        break;

        case disable:
            res = tahoe_psh_disable_nor();
        break;

        default:
        break;
    }
  
    return(res);
}

static int core_vcpu_custom_logic(bcm_cfg_stat_t stat, void *data)
{
  return chipmgr_enable_vc_sram(stat == enable ? TRUE : FALSE);
}

static int core_vtvo_vec_custom_logic(bcm_cfg_stat_t stat, void *data)
{
    uint32_t ret = BCM_OK;

    if (stat == disable)
    {
        ret = vec_shutdown();
        if (ret != BCM_OK)
        {
            bcm_log_error("%s: vec_shutdown() failed\n", __FUNCTION__);
        }
    }
    else if (stat == enable)
    {
	ret = vec_init();
        if (ret != BCM_OK)
        {
            bcm_log_error("%s: vec_init() failed\n", __FUNCTION__);
        }
    }
    return(ret);
}


static int core_vi2s_custom_logic(bcm_cfg_stat_t stat, void *data)
{
    if ((stat == enable) || (stat == disable))
    {
        return(chipmgr_config_vc2_audioblk_control(ECHIPMGR_AUDIO_BLK_12MHZ));
    }

    return(BCM_ERROR);
}

static int core_spdif_custom_logic(bcm_cfg_stat_t stat, void *data)
{
    if (stat == enable)
    {
        return(chipmgr_config_vc2_audioblk_control(ECHIPMGR_AUDIO_BLK_24_576MHZ));
    }
    else if (stat == disable)
    {
        return(chipmgr_config_vc2_audioblk_control(ECHIPMGR_AUDIO_BLK_12MHZ));
    }

    return(BCM_ERROR);
}


static int core_spi0_custom_logic(bcm_cfg_stat_t stat, void *data)
{
    chipmgr_config_spi(ECHIPMGR_SPI_PORT0, stat);

    return(BCM_OK);
}

static int core_spi1_custom_logic(bcm_cfg_stat_t stat, void *data)
{
    chipmgr_config_spi(ECHIPMGR_SPI_PORT1, stat);

    return(BCM_OK);
}
#endif
