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
 *   @file   tahoe_opmode_cmd_params.c 
 * 
 *   @brief  Proc interfacing functions.
 * 
 ****************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>

#include "tahoe.h"
// #include "tahoe_lnx_log.h"
#include "tahoe_opmode_cmd_params.h"
                               
#define DEFAULT_LOGSTR  "ALL"
#define ON_STR "ON"
#define OFF_STR "OFF"

// #define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static const char *g_err_str = "INVALID";                                                                                    

/********************************************************************************
    NOTE: Most of the structures data structures don't have to store
          the enum + str as the enum is "usually" a series of ints.
          However, in order to ensure there are no errors and for the
          sake of clarity, they are included here.

          So, just don't try to "optimize" the code.
*********************************************************************************/

/** Structure to hold the display strings */
#define MAX_STR_SZ 50

//STATUS INFORMATION
static const char *g_stat_str[] = 
{
    "DISABLED",
    "ENABLED"
};

//CLOCK SOURCE
typedef struct _tahoe_csrc_strs_t
{
    tahoe_opmode_csrc_t csrc;
    char str[MAX_STR_SZ];
}tahoe_csrc_strs_t;

static const tahoe_csrc_strs_t g_csrcs[] = 
{
    { OPMODE_TCSRC_NONE,       "NONE"  },
    { OPMODE_TCSRC_NONE,       "UARTCLK" },
    { OPMODE_TCSRC_XTAL_24MHZ, "24MHZ" },
    { OPMODE_TCSRC_XTAL_27MHZ, "27MHZ" },
    { OPMODE_TCSRC_XTAL_32KHZ, "32KHZ" },
    { OPMODE_TCSRC_PLLS,       "PLLS"  },
    { OPMODE_TCSRC_PLL1,       "PLL1"  },
    { OPMODE_TCSRC_PLL2,       "PLL2"  },
    { OPMODE_TCSRC_PLL3,       "PLL3"  }
};

//DRAM MODES
typedef struct _tahoe_dram_mode_strs_t
{
    tahoe_opmode_dram_mode_t dram_mode;
    char in_str[MAX_STR_SZ];
    char str[MAX_STR_SZ];
}tahoe_dram_mode_strs_t;

static const tahoe_dram_mode_strs_t g_drammode_strs[] = 
{
   { OPMODE_DRAM_MODE_ASYNC,    "ASYNC",     "Asynchronous to EMI clock"              },
   { OPMODE_DRAM_MODE_1by1,     "SYNC_1BY1", "Synchronous to  EMI clock, EMI:DRAM 1:1"},
   { OPMODE_DRAM_MODE_3by2,     "SYNC_3BY2", "Synchronous to  EMI clock, EMI:DRAM 3:2"},
   { OPMODE_DRAM_MODE_2by1,     "SYNC_2BY1", "Synchronous to  EMI clock, EMI:DRAM 2:1"},
   { OPMODE_DRAM_MODE_INVALID,   ""           "Invalid mode"                          },
};

#if 0
//LOG LEVELS
typedef struct _tahoe_log_strs_t
{
    unsigned long log_level;
    char str[MAX_STR_SZ];
}tahoe_log_strs_t;

static const tahoe_log_strs_t g_log_str[] =
{
   { BCMLOG_CRIT,  "CRITICAL" },
   { BCMLOG_ERROR, "ERROR"    },
   { BCMLOG_WARN,  "WARN"     },
   { BCMLOG_INFO,  "INFO"     },
   { BCMLOG_DEBUG, "DEBUG"    },
   { BCMLOG_ALL,   "ALL"      },
};
#endif

//CORES
#if 0
typedef struct _tahoe_core_strs_t
{
    tahoe_opmode_core_t core;
    char str[MAX_STR_SZ];
}tahoe_core_strs_t;
#endif

const tahoe_core_strs_t g_cores[OPMODE_CORE_COUNT] =
{  
   {  CORE_IDE,      "IDE"    },    
   {  CORE_CEATA,    "CEATA"  },    
   {  CORE_NAND,     "NAND"   },    
   {  CORE_USB,      "USB"    },    

   {  CORE_SDIO0,     "SDIO0" },    
   {  CORE_SDIO1,     "SDIO1" },    
   {  CORE_VFIR,      "VFIR"  },    
   {  CORE_CRPT,      "CRPT"  },    

   {  CORE_AMC,       "AMC"   },    
   {  CORE_PWM,       "PWM"   },    
   {  CORE_UART0,     "UART0" },    
   {  CORE_UART1,     "UART1" },    

   {  CORE_UART2,     "UART2" },    
   {  CORE_PKE,       "PKE"   },    
   {  CORE_OTP,       "OTP"   },    
   {  CORE_TIM,       "TIM"   },    

   {  CORE_TWSPI,     "TWSPI" },    
   {  CORE_SPI0,      "SPI0"  },    
   {  CORE_SPI1,      "SPI1"  },    
   {  CORE_INTC,      "INTC"  },    

   {   CORE_RNG,      "RNG"   },    
   {   CORE_RTC,      "RTC"   },    
   {   CORE_I2C,      "I2C"   },    
   {   CORE_SYSM,     "SYSM"  },    

   {   CORE_GPIO,     "GPIO"  },    
   {   CORE_PM,       "PM"    },   
   {   CORE_I2S,      "I2S"   },   
   {   CORE_RMP,      "RMP"   },   

   {   CORE_RPC,      "RPC"   },   
   {   CORE_WDT,      "WDT"   },   

   {   CORE_MMTX,     "MMTX"  },    
   {   CORE_MDMA,     "MDMA"  },    
   {   CORE_MROM,     "MROM"  },    
   {   CORE_MNOR,     "MNOR"  },    

   {  CORE_MSRAM,     "MSRAM" },    
   {  CORE_MEMI,      "MEMI"  },   
   {  CORE_MIPS,      "MIPS"  },   
   {  CORE_CUART,     "CUART" },   

   {   CORE_CI2S,     "CI2S"  },   
   {   CORE_CDC,      "CDC"   },   
   {   CORE_CSMI,     "CSMI"  },   
   {   CORE_VCPU,     "VCPU"  },   

   {   CORE_VCAM,     "VCAM"  },   
   {   CORE_VBG,      "VBG"   },   
   {   CORE_VINT,     "VINT"  },   
   {   CORE_VTIM,     "VTIM"  },   

   {   CORE_VI2C,     "VI2C"  },   
   {   CORE_VI2S,     "VI2S"  },   
   {   CORE_SPDIF,    "SPDIF" },   
   {   CORE_VTVO,     "VTVO"  },   

   {   CORE_ASTVO,    "ASTVO" },
   
   {   CORE_GEN,      "GEN"   },   

   {   CORE_ARM,      "ARM"   },
   {   CORE_DRAM,     "DRAM"  }
}; 
   

//PLLS
typedef struct _tahoe_pll_strs
{
    int pll;
    char str[MAX_STR_SZ];
}tahoe_pll_strs_t;

static const char *g_plls[PLL_COUNT] =
{
    "PLLS",
    "PLL1",
    "PLL2",
    "PLL3",
};                        

//SYNCTOPS
typedef struct _tahoe_synctop_strs_t
{
    tahoe_opmode_synctop_t synctop;
    char str[MAX_STR_SZ];
}tahoe_synctop_strs_t;

static const tahoe_synctop_strs_t g_syntops[] = 
{
    { SYNCTOP_AI,   "AI" },
    { SYNCTOP_AO,   "AO" },
    { SYNCTOP_PI,   "PI" },
    { SYNCTOP_VI,   "VI" },
    { SYNCTOP_VO,   "VO" }
};

//SYNCTOP MODES
typedef struct _tahoe_synctop_mode_strs_t
{
    tahoe_opmode_synctop_mode_t synctop_mode;
    char str[MAX_STR_SZ];
}tahoe_synctop_mode_strs_t;

static const tahoe_synctop_mode_strs_t g_syntop_modes[] = 
{
    { SYNCTOP_MODE_BYPASS   ,   "BYPASS"    },
    { SYNCTOP_MODE_BANDWIDTH,   "BANDWIDTH" },
    { SYNCTOP_MODE_LATENCY  ,   "LATENCY"   }
};

//SYNCTOP CLOCK MODES
typedef struct _tahoe_synctop_clkmode_strs_t
{
    tahoe_opmode_synctop_clkmode_t synctop_clkmode;
    char str[MAX_STR_SZ];
}tahoe_synctop_clkmode_strs_t;

static const tahoe_synctop_clkmode_strs_t g_syntop_clkmodes[] = 
{
    { SYNCTOP_CLKMODE_1to1,   "1TO1" },
    { SYNCTOP_CLKMODE_Nto1,   "NTO1" },
    { SYNCTOP_CLKMODE_1toN,   "1TON" }
};

//OPMODE
typedef struct _tahoe_opmode_strs
{
    tahoe_opmode_mode_t mode;
    char str[MAX_STR_SZ];
}tahoe_opmode_strs_t;

static const tahoe_opmode_strs_t g_opmode_str[] =
{
    { OPMODE_HIBERNATE,            "HIBERNATE"      },
    { OPMODE_SLEEP,                "SLEEP"          },
    { OPMODE_MENU,                 "MENU"           },
    { OPMODE_AUDIO_PLAYBACK,       "AUDIO_PLAYBACK" },
    { OPMODE_VIDEO_PLAYBACK,       "VIDEO_PLAYBACK" },
    { OPMODE_USB_HOST,             "USB_HOST"       },
    { OPMODE_USB_DEV,              "USB_DEV"        },
    { OPMODE_SOFTWARE_IDLE,        "SOFTWARE_IDLE"  },
    { OPMODE_AMCSS_AUDIOPLAYBACK,  "AMCSS_AUDIOPLAYBACK" },
    { OPMODE_DEFAULT_MODE,         "DEFAULT_MODE"   }
};
   

/************************************************************************
        String to ENUM conversion routines
*************************************************************************/
/**
    @fn  int tahoe_opmode_get_cfg(char *cfg);
*/
int tahoe_opmode_get_cfg(char *cfg)
{
    if (strcmp(cfg, ON_STR) == 0)
    {
        return(1);
    }
    else if (strcmp(cfg, OFF_STR) == 0)
    {
        return(0);
    }

    return(-1);
}

/**
    @fn  tahoe_opmode_core_t tahoe_opmode_get_core(char *core_str);
*/
tahoe_opmode_core_t tahoe_opmode_get_core(char *core_str)
{
    int i, sz;

    if (core_str == NULL)
        return(OPMODE_CORE_COUNT);
    
    sz = ARRAY_SIZE(g_cores);

    for (i = 0; i < sz; i++)
    {
        if (strcmp(g_cores[i].str, core_str) == 0)
        {
			// printk(" ####### FOUND : %ld \n", g_cores[i].core ) ;
            return(g_cores[i].core);
        }
    }

    return(OPMODE_CORE_COUNT);
}

/**
    @fn  tahoe_opmode_csrc_t tahoe_opmode_get_csrc(char *csrc_str);
*/
tahoe_opmode_csrc_t tahoe_opmode_get_csrc(char *csrc_str)
{
    int i, sz;

    if (csrc_str == NULL)
        return(OPMODE_TCSRC_COUNT);
    
    sz = ARRAY_SIZE(g_csrcs);

    for (i = 0; i < sz; i++)
    {
        if (strcmp(g_csrcs[i].str, csrc_str) == 0)
        {
            return(g_csrcs[i].csrc);
        }
    }

    return(OPMODE_TCSRC_COUNT);
}

/**
    @fn  int tahoe_opmode_get_pll(char *pll_str);
*/
int tahoe_opmode_get_pll(char *pll_str)
{
    int i;

    if (pll_str == NULL)
        return(PLL_COUNT);
    
    for (i = 0; i < PLL_COUNT; i++)
    {
        if (strcmp(g_plls[i], pll_str) == 0)
        {
            return(i);
        }
    }

    return(PLL_COUNT);
}
                              
/**
    @fn  tahoe_opmode_mode_t tahoe_opmode_get_opmode(char *opmode_str);
*/
tahoe_opmode_mode_t tahoe_opmode_get_opmode(char *opmode_str)
{
    int i, sz;

    if (opmode_str == NULL)
    {
    }
    
    sz = ARRAY_SIZE(g_opmode_str);

    for (i = 0; i < sz; i++)
    {
        if (strcmp(g_opmode_str[i].str, opmode_str) == 0)
        {
            return(g_opmode_str[i].mode);
        }
    }

    return(OPMODE_DEFAULT_MODE);
}

/**
    @fn  tahoe_opmode_dram_mode_t tahoe_opmode_get_drammode(char *dram_mode_str);
*/
tahoe_opmode_dram_mode_t tahoe_opmode_get_drammode(char *dram_mode_str)
{
    int i, sz;

    if (dram_mode_str == NULL)
        return(OPMODE_DRAM_MODE_INVALID);
    
    sz = ARRAY_SIZE(g_drammode_strs);

    for (i = 0; i < sz; i++)
    {
        if (strcmp(g_drammode_strs[i].in_str, dram_mode_str) == 0)
        {
            return(g_drammode_strs[i].dram_mode);
        }
    }

    return(OPMODE_DRAM_MODE_INVALID);
}

#if 0
/**
    @fn  unsigned long tahoe_opmode_get_log(char *log_str);
*/
unsigned long tahoe_opmode_get_log(char *log_str)
{
    int i, sz;

    if (log_str == NULL)
        return(BCMLOG_ALL);
    
    sz = ARRAY_SIZE(g_log_str);

    for (i = 0; i < sz; i++)
    {
        if (strcmp(g_log_str[i].str, log_str) == 0)
        {
            return(g_log_str[i].log_level);
        }
    }

    return(BCMLOG_ALL);
}
#endif

tahoe_opmode_synctop_t tahoe_opmode_get_synctop_idx(char *synctop_str)
{
    int i, sz;

    if (synctop_str == NULL)
        return(SYNCTOP_INVALID);
    
    sz = ARRAY_SIZE(g_syntops);

    for (i = 0; i < sz; i++)
    {
        if (strcmp(g_syntops[i].str, synctop_str) == 0)
        {
            return(g_syntops[i].synctop);
        }
    }

    return(SYNCTOP_INVALID);
}

tahoe_opmode_synctop_mode_t tahoe_opmode_get_synctop_mode(char *synctop_mode_str)
{
    int i, sz;

    if (synctop_mode_str == NULL)
        return(-1);    

    sz = ARRAY_SIZE(g_syntop_modes);

    for (i = 0; i < sz; i++)
    {
        if (strcmp(g_syntop_modes[i].str, synctop_mode_str) == 0)
        {
            return(g_syntop_modes[i].synctop_mode);
        }
    }

    return(-1);
}

tahoe_opmode_synctop_clkmode_t tahoe_opmode_get_synctop_clkmode(char *synctop_clkmode_str)
{
    int i, sz;

    if (synctop_clkmode_str == NULL)
        return(-1);
    
    sz = ARRAY_SIZE(g_syntop_clkmodes);

    for (i = 0; i < sz; i++)
    {
        if (strcmp(g_syntop_clkmodes[i].str, synctop_clkmode_str) == 0)
        {
            return(g_syntop_clkmodes[i].synctop_clkmode);
        }
    }

    return(-1);
}


/****************************************************************************************
    DISPLAY Strings
*****************************************************************************************/
#if 0
/**
    @fn  char *tahoe_opmode_log_str(unsigned long level);
*/
const char *tahoe_opmode_log_str(unsigned long level)
{
    int i, sz;
   
    sz = ARRAY_SIZE(g_log_str);

    for (i = 0; i < sz; i++)
    {
        if (g_log_str[i].log_level == level)
        {
            return(g_log_str[i].str);
        }
    }

    return(DEFAULT_LOGSTR);
}
#endif

const char *tahoe_opmode_stat_str(int stat)
{
    if (stat >= ARRAY_SIZE(g_stat_str))
        return(g_err_str);

    return(g_stat_str[stat]);
}
   
const char *tahoe_opmode_mode_str(int mode)
{
    if (mode >= ARRAY_SIZE(g_opmode_str))
        return(g_err_str);

    return(g_opmode_str[mode].str);
}

const char *tahoe_opmode_drammode_str(int dram_mode)
{
    if (dram_mode >= ARRAY_SIZE(g_drammode_strs))
        return(g_err_str);

    return(g_drammode_strs[dram_mode].str);
}

const char *tahoe_opmode_core_str(int core_idx)
{
    if (core_idx >= ARRAY_SIZE(g_cores))
        return(g_err_str);

    return(g_cores[core_idx].str);
}

const char *tahoe_opmode_pll_str(int pll)
{
    if (pll >= ARRAY_SIZE(g_plls))
        return(g_err_str);

    return(g_plls[pll]);
}

const char *tahoe_opmode_csrc_str(int csrc)
{
    if (csrc >= ARRAY_SIZE(g_csrcs))
        return(g_err_str);
   
    return(g_csrcs[csrc].str);
}  
  
const char *tahoe_opmode_synctop_str(int synctop)
{
    if (synctop >= ARRAY_SIZE(g_syntops))
        return(g_err_str);

    return(g_syntops[synctop].str);    
}

const char *tahoe_opmode_synctop_mode_str(int synctop_mode)
{
    if (synctop_mode >= ARRAY_SIZE(g_syntop_modes))
        return(g_err_str);

    return(g_syntop_modes[synctop_mode].str);    
}

const char *tahoe_opmode_synctop_clkmode_str(int synctop_clkmode)
{
    if (synctop_clkmode >= ARRAY_SIZE(g_syntop_clkmodes))
        return(g_err_str);

    return(g_syntop_clkmodes[synctop_clkmode].str);    
}
 
 
 
   
   
  
  
  



