/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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
*  hpm.c
*
*  PURPOSE:
*
*     This implements the BCMRING hpm driver for setting the programmable bus
*     arbitration and scheduling algorithms.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */
#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <asm/arch/csp/hpmHw_reg.h>
#include <cfg_global.h>

#if CFG_GLOBAL_CHIP_FAMILY != CFG_GLOBAL_CHIP_FAMILY_BCMRING
#error HPM must only be built with BCMRING architecture
#endif

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
#ifndef ARRAY_LEN
#define ARRAY_LEN(x) (sizeof(x)/sizeof(x[0]))
#endif

#define HPM_DRIVER_NAME "hpm"

// A lookup table entry
typedef struct
{
   int code;
   const char *str;
}
HPM_ENTRY;


/* ---- Private Function Prototypes -------------------------------------- */
static int proc_dointvec_wxb(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_dostring_custom(ctl_table *table, int write, struct file *filp,
			 void __user *buffer, size_t *lenp, loff_t *ppos);
static int proc_dointvec_dump(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_dointvec_tables(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_dointvec_cfg(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_dointvec_qos_enmask(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static int proc_dointvec_qos_tidemark(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos );
static const char * Code2Str( int code, const HPM_ENTRY *list, int tbllen );
static int Str2Code( char *str, const HPM_ENTRY *list, int tbllen );
static int dump( void );
static int tables( void );
static int hpm_cfg(uint32_t scenario);

/* ---- Private Variables ------------------------------------------------ */
static uint32_t gScenario;

static char banner[] __initdata = KERN_INFO "HPM Driver: 1.00 (built on " __DATE__ " " __TIME__ ")\n";

/* sysctl */
static struct ctl_table_header *gHpmSysCtlHeader = NULL;

// Some custom lookup tables for tranlating between strings and codes.
HPM_ENTRY vramBusTbl[] =
{
   { HPMHW_VRAMBUS_VPMP,      "vpmp"      },
   { HPMHW_VRAMBUS_VPMD,      "vpmd"      },
   { HPMHW_VRAMBUS_DMA0M2,    "dma0m2"    },
   { HPMHW_VRAMBUS_DMA1M2,    "dma1m2"    },
   { HPMHW_VRAMBUS_WHBM,      "whbm"      }
};
HPM_ENTRY whbSbusTbl[] =
{
   { HPMHW_WHBSBUS_VPMP,      "vpmp"      },
   { HPMHW_WHBSBUS_VPMD,      "vpmd"      },
   { HPMHW_WHBSBUS_LCDC,      "lcdc"      }
};
HPM_ENTRY nhbSbusTbl[] =
{
   { HPMHW_NHBSBUS_GEM,       "gem"       },
   { HPMHW_NHBSBUS_USBH0M,    "usbh0m"    },
   { HPMHW_NHBSBUS_USBH1M,    "usbh1m"    },
   { HPMHW_NHBSBUS_SDIOH0M,   "sdioh0m"   },
   { HPMHW_NHBSBUS_SDIOH1M,   "sdioh1m"   },
   { HPMHW_NHBSBUS_I2CSM,     "i2csm"     },
   { HPMHW_NHBSBUS_SPISM,     "spism"     }
};
HPM_ENTRY wxbTbl[] =
{
   { HPMHW_WXB_ARMI,          "armi"      },
   { HPMHW_WXB_ARMRW,         "armrw"     },
   { HPMHW_WXB_WHBS,          "whbs"      },
   { HPMHW_WXB_DMA0,          "dma0"      },
   { HPMHW_WXB_DMA1,          "dma1"      },
   { HPMHW_WXB_VDEC,          "vdec"      },
   { HPMHW_WXB_NHBS,          "nhbs"      }
};

// WXB bus entries have both read and write priorities
typedef struct
{
	int r;	// read priority
	int w;	// write priority
}
rwprio_t;

// These are the different blocks on the wxb bus
typedef struct
{
	rwprio_t armi;
	rwprio_t armrw;
	rwprio_t whbs;
	rwprio_t dma0;
	rwprio_t dma1;
	rwprio_t vdec;
	rwprio_t nhbs;
}
wxb_t;

// The vram, whbs, and nhbs busses are programmed by the user
// writing a name string to the appropriate priority field for
// that bus.
typedef struct
{
	char name[80];
}
name_t;

// The HPM object
typedef struct
{
	wxb_t ddrc;
	wxb_t aram;
	name_t vram[1+HPMHW_VRAMBUS_MAX_PRIO];
	name_t whbs[1+HPMHW_WHBSBUS_MAX_PRIO];
	name_t nhbs[1+HPMHW_NHBSBUS_MAX_PRIO];
	int dump;
	int tables;
   int cfg;
   uint32_t qos_enmask;
   uint32_t qos_tidemark;
}
hpm_t;

static hpm_t hpm;

// Canned priority settings
typedef struct
{
   int ddrc_rd_wxb_armi;
   int ddrc_rd_wxb_armrw;
   int ddrc_rd_wxb_whbs;
   int ddrc_rd_wxb_dma0;
   int ddrc_rd_wxb_dma1;
   int ddrc_rd_wxb_vdec;
   int ddrc_rd_wxb_nhbs;

   int ddrc_wr_wxb_armi;
   int ddrc_wr_wxb_armrw;
   int ddrc_wr_wxb_whbs;
   int ddrc_wr_wxb_dma0;
   int ddrc_wr_wxb_dma1;
   int ddrc_wr_wxb_vdec;
   int ddrc_wr_wxb_nhbs;

   int aram_rd_wxb_armi;
   int aram_rd_wxb_armrw;
   int aram_rd_wxb_whbs;
   int aram_rd_wxb_dma0;
   int aram_rd_wxb_dma1;
   int aram_rd_wxb_vdec;
   int aram_rd_wxb_nhbs;

   int aram_wr_wxb_armi;
   int aram_wr_wxb_armrw;
   int aram_wr_wxb_whbs;
   int aram_wr_wxb_dma0;
   int aram_wr_wxb_dma1;
   int aram_wr_wxb_vdec;
   int aram_wr_wxb_nhbs;

   int vram_whb_vpmp;
   int vram_whb_vpmd;
   int vram_whb_dma0;
   int vram_whb_dma1;
   int vram_whb_whbm;

   int whbs_whb_vpmp;
   int whbs_whb_vpmd;
   int whbs_whb_lcdc;

   int nhbs_nhb_ge;
   int nhbs_nhb_usb0;
   int nhbs_nhb_usb1;
   int nhbs_nhb_sdio0;
   int nhbs_nhb_sdio1;
   int nhbs_nhb_i2cs;
   int nhbs_nhb_spis;

   int qos_tidemark;
   int qos_enmask;
}
HPM_PRIORITY;

// following enum must match order in priority table
typedef enum
{
   HPM_SETTING_POWER_ON = 0,
   HPM_SETTING_STANDALONE,
   HPM_SETTING_1181
}
HPM_SETTING;

static HPM_PRIORITY priorityTable[] =
{
   // power-on default
   {
      0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0,
      0, 1, 2, 3, 4,
      1, 2, 0,
      0, 1, 2, 3, 4, 5, 6,
      0, 0
   },

   // opt standalone
   // ARM has lowest priority
   // QoS is enabled when tide mark = 2, ARM is blocked when QoS is enabled
   {
      6, 6, 0, 1, 1, 3, 4,
      6, 6, 0, 1, 1, 3, 4,
      0, 0, 5, 2, 2, 6, 4,
      0, 0, 5, 2, 2, 6, 4,
      2, 3, 0, 1, 4,
      1, 2, 0,
      0, 1, 2, 3, 4, 6, 5,
      2, 0x7c
   },

   // opt with 1181
   // ARM has lowest priority
   // WHBS (used in VPM code fetches) has same priority as DMAC in DDR arbitration
   // QoS is enabled when tide mark = 2, ARM is blocked when QoS is enabled
   {
      6, 6, 0, 0, 0, 3, 4,
      6, 6, 0, 0, 0, 3, 4,
      0, 0, 5, 2, 2, 6, 4,
      0, 0, 5, 2, 2, 6, 4,
      2, 3, 0, 1, 4,
      1, 2, 0,
      0, 1, 2, 3, 4, 6, 5,
      2, 0x7c
   },
};

// Macros to save typing.

#define BCM_SYSCTL_WXB(bus, name, dataloc)           	\
   {                                                 	\
      .ctl_name      = BCM_SYSCTL_##bus##_##name,		\
      .procname      = #name,   								\
      .data          = dataloc,								\
		.maxlen			= sizeof(int),							\
      .mode          = 0644,                          \
      .proc_handler  = &proc_dointvec_wxb             \
   },

#define BCM_SYSCTL_VRAM(prio)	\
   {                                                	\
      .ctl_name      = BCM_SYSCTL_vram_##prio,      	\
      .procname      = #prio,                       	\
      .data          = &hpm.vram[prio].name[0],      	\
		.maxlen			= sizeof(hpm.vram[prio].name),	\
      .mode          = 0644,                          \
      .proc_handler  = &proc_dostring_custom          \
   },
#define BCM_SYSCTL_WHBS(prio)	\
   {                                                	\
      .ctl_name      = BCM_SYSCTL_whbs_##prio,      	\
      .procname      = #prio,                       	\
      .data          = &hpm.whbs[prio].name[0],      	\
		.maxlen			= sizeof(hpm.whbs[prio].name),	\
      .mode          = 0644,                          \
      .proc_handler  = &proc_dostring_custom          \
   },
#define BCM_SYSCTL_NHBS(prio)	\
   {                                                	\
      .ctl_name      = BCM_SYSCTL_nhbs_##prio,      	\
      .procname      = #prio,                       	\
      .data          = &hpm.nhbs[prio].name[0],      	\
		.maxlen			= sizeof(hpm.nhbs[prio].name),	\
      .mode          = 0644,                          \
      .proc_handler  = &proc_dostring_custom          \
   },

// Define local sysctl identifiers. These are all under
// the HPM major sysctl number and can be local.
#define BCM_SYSCTL_ddrc_armi_r	100
#define BCM_SYSCTL_ddrc_armi_w	101
#define BCM_SYSCTL_ddrc_armrw_r	102
#define BCM_SYSCTL_ddrc_armrw_w	103
#define BCM_SYSCTL_ddrc_whbs_r	104
#define BCM_SYSCTL_ddrc_whbs_w	105
#define BCM_SYSCTL_ddrc_dma0_r	106
#define BCM_SYSCTL_ddrc_dma0_w	107
#define BCM_SYSCTL_ddrc_dma1_r	108
#define BCM_SYSCTL_ddrc_dma1_w	109
#define BCM_SYSCTL_ddrc_vdec_r	110
#define BCM_SYSCTL_ddrc_vdec_w	111
#define BCM_SYSCTL_ddrc_nhbs_r	112
#define BCM_SYSCTL_ddrc_nhbs_w	113

#define BCM_SYSCTL_aram_armi_r	200
#define BCM_SYSCTL_aram_armi_w	201
#define BCM_SYSCTL_aram_armrw_r	202
#define BCM_SYSCTL_aram_armrw_w	203
#define BCM_SYSCTL_aram_whbs_r	204
#define BCM_SYSCTL_aram_whbs_w	205
#define BCM_SYSCTL_aram_dma0_r	206
#define BCM_SYSCTL_aram_dma0_w	207
#define BCM_SYSCTL_aram_dma1_r	208
#define BCM_SYSCTL_aram_dma1_w	209
#define BCM_SYSCTL_aram_vdec_r	210
#define BCM_SYSCTL_aram_vdec_w	211
#define BCM_SYSCTL_aram_nhbs_r	212
#define BCM_SYSCTL_aram_nhbs_w	213

#define BCM_SYSCTL_vram_0			300
#define BCM_SYSCTL_vram_1			301
#define BCM_SYSCTL_vram_2			302
#define BCM_SYSCTL_vram_3			303
#define BCM_SYSCTL_vram_4			304

#define BCM_SYSCTL_whbs_0			400
#define BCM_SYSCTL_whbs_1			401
#define BCM_SYSCTL_whbs_2			402

#define BCM_SYSCTL_nhbs_0			500
#define BCM_SYSCTL_nhbs_1			501
#define BCM_SYSCTL_nhbs_2			502
#define BCM_SYSCTL_nhbs_3			503
#define BCM_SYSCTL_nhbs_4			504
#define BCM_SYSCTL_nhbs_5			505
#define BCM_SYSCTL_nhbs_6			506

// Child sysctl tables
static struct ctl_table gHpmSysCtl_ddrc[] = {
	BCM_SYSCTL_WXB(ddrc, armi_r, &hpm.ddrc.armi.r)
	BCM_SYSCTL_WXB(ddrc, armi_w, &hpm.ddrc.armi.w)
	BCM_SYSCTL_WXB(ddrc, armrw_r, &hpm.ddrc.armrw.r)
	BCM_SYSCTL_WXB(ddrc, armrw_w, &hpm.ddrc.armrw.w)
	BCM_SYSCTL_WXB(ddrc, whbs_r, &hpm.ddrc.whbs.r)
	BCM_SYSCTL_WXB(ddrc, whbs_w, &hpm.ddrc.whbs.w)
	BCM_SYSCTL_WXB(ddrc, dma0_r, &hpm.ddrc.dma0.r)
	BCM_SYSCTL_WXB(ddrc, dma0_w, &hpm.ddrc.dma0.w)
	BCM_SYSCTL_WXB(ddrc, dma1_r, &hpm.ddrc.dma1.r)
	BCM_SYSCTL_WXB(ddrc, dma1_w, &hpm.ddrc.dma1.w)
	BCM_SYSCTL_WXB(ddrc, vdec_r, &hpm.ddrc.vdec.r)
	BCM_SYSCTL_WXB(ddrc, vdec_w, &hpm.ddrc.vdec.w)
	BCM_SYSCTL_WXB(ddrc, nhbs_r, &hpm.ddrc.nhbs.r)
	BCM_SYSCTL_WXB(ddrc, nhbs_w, &hpm.ddrc.nhbs.w)
	{}
};
static struct ctl_table gHpmSysCtl_aram[] = {
	BCM_SYSCTL_WXB(aram, armi_r, &hpm.aram.armi.r)
	BCM_SYSCTL_WXB(aram, armi_w, &hpm.aram.armi.w)
	BCM_SYSCTL_WXB(aram, armrw_r, &hpm.aram.armrw.r)
	BCM_SYSCTL_WXB(aram, armrw_w, &hpm.aram.armrw.w)
	BCM_SYSCTL_WXB(aram, whbs_r, &hpm.aram.whbs.r)
	BCM_SYSCTL_WXB(aram, whbs_w, &hpm.aram.whbs.w)
	BCM_SYSCTL_WXB(aram, dma0_r, &hpm.aram.dma0.r)
	BCM_SYSCTL_WXB(aram, dma0_w, &hpm.aram.dma0.w)
	BCM_SYSCTL_WXB(aram, dma1_r, &hpm.aram.dma1.r)
	BCM_SYSCTL_WXB(aram, dma1_w, &hpm.aram.dma1.w)
	BCM_SYSCTL_WXB(aram, vdec_r, &hpm.aram.vdec.r)
	BCM_SYSCTL_WXB(aram, vdec_w, &hpm.aram.vdec.w)
	BCM_SYSCTL_WXB(aram, nhbs_r, &hpm.aram.nhbs.r)
	BCM_SYSCTL_WXB(aram, nhbs_w, &hpm.aram.nhbs.w)
	{}
};

static struct ctl_table gHpmSysCtl_vram[] = {
	BCM_SYSCTL_VRAM(0)
	BCM_SYSCTL_VRAM(1)
	BCM_SYSCTL_VRAM(2)
	BCM_SYSCTL_VRAM(3)
	BCM_SYSCTL_VRAM(4)
	{}
};
static struct ctl_table gHpmSysCtl_whbs[] = {
	BCM_SYSCTL_WHBS(0)
	BCM_SYSCTL_WHBS(1)
	BCM_SYSCTL_WHBS(2)
	{}
};
static struct ctl_table gHpmSysCtl_nhbs[] = {
	BCM_SYSCTL_NHBS(0)
	BCM_SYSCTL_NHBS(1)
	BCM_SYSCTL_NHBS(2)
	BCM_SYSCTL_NHBS(3)
	BCM_SYSCTL_NHBS(4)
	BCM_SYSCTL_NHBS(5)
	BCM_SYSCTL_NHBS(6)
	{}
};

// Parent sysctl table
static struct ctl_table gHpmSysCtlLocal[] = {
   {
      .ctl_name      = 1,
      .procname      = "ddrc",
      .mode          = 0555,
      .child         = gHpmSysCtl_ddrc
   },
   {
      .ctl_name      = 2,
      .procname      = "aram",
      .mode          = 0555,
      .child         = gHpmSysCtl_aram
   },
   {
      .ctl_name      = 3,
      .procname      = "vram",
      .mode          = 0555,
      .child         = gHpmSysCtl_vram
   },
   {
      .ctl_name      = 4,
      .procname      = "whbs",
      .mode          = 0555,
      .child         = gHpmSysCtl_whbs
   },
   {
      .ctl_name      = 5,
      .procname      = "nhbs",
      .mode          = 0555,
      .child         = gHpmSysCtl_nhbs
   },
   {
      .ctl_name      = 6,
      .procname      = "dump",
      .mode          = 0644,
      .data				= &hpm.dump,
		.maxlen			= sizeof(int),
      .proc_handler  = &proc_dointvec_dump
   },
   {
      .ctl_name      = 7,
      .procname      = "tables",
      .mode          = 0644,
      .data				= &hpm.tables,
		.maxlen			= sizeof(int),
      .proc_handler  = &proc_dointvec_tables
   },
   {
      .ctl_name      = 8,
      .procname      = "cfg",
      .mode          = 0644,
      .data          = &hpm.cfg,
      .maxlen        = sizeof(int),
      .proc_handler  = &proc_dointvec_cfg
   },
   {
      .ctl_name      = 9,
      .procname      = "qos_enmask",
      .mode          = 0644,
      .data          = &hpm.qos_enmask,
      .maxlen        = sizeof(int),
      .proc_handler  = &proc_dointvec_qos_enmask
   },
   {
      .ctl_name      = 10,
      .procname      = "qos_tidemark",
      .mode          = 0644,
      .data          = &hpm.qos_tidemark,
      .maxlen        = sizeof(int),
      .proc_handler  = &proc_dointvec_qos_tidemark
   },
   {}
};
static struct ctl_table gHpmSysCtl[] = {
   {
      .ctl_name      = CTL_BCM_HPM,
      .procname      = HPM_DRIVER_NAME,
      .mode          = 0555,
      .child         = gHpmSysCtlLocal
   },
   {}
};

/* ---- Functions -------------------------------------------------------- */
//------------------------------------------------------------
/*****************************************************************************
*
*  FUNCTION:   Code2Str
*
*  PURPOSE:    search a list for a string based on the supplied integer value
*
*  PARAMETERS: code     - input  - integer value to search the list for
*              list     - input  - list to search
*              tbllen   - input  - table size in entries
*
*  RETURNS:    str or "Unknown"
*
*  NOTES:
*
*****************************************************************************/
static const char * Code2Str( int code, const HPM_ENTRY *list, int tbllen )
{
   int i;
   for (i = 0; i < tbllen; i++, list++)
   {
      if( list->code == code )
      {
         return( list->str );
      }
   }
   return( "Unknown" );
}

/*****************************************************************************
*
*  FUNCTION:   Str2Code
*
*  PURPOSE:    search a list for a code based on the supplied string
*
*  PARAMETERS: str      - input  - integer value to search the list for
*              list     - input  - list to search
*              tbllen   - input  - table size in entries
*
*  RETURNS:     code or -1 if failed
*
*  NOTES:
*
*****************************************************************************/
static int Str2Code( char *str, const HPM_ENTRY *list, int tbllen )
{
   int i;
   /* check if it is an empty command */
   if( (!str) || (strlen(str) == 0) )
   {
      return( -1 );
   }

   for (i = 0; i < tbllen; i++, list++)
   {
      if( strcasecmp(str, list->str) == 0 )
      {
         return list->code;
      }
   }
   return( -1 );
}
/****************************************************************************/
/**
*  @brief   Custom dointvec for setting priorities of wxb ddrc/aram busses.
*
*  @return
*/
/****************************************************************************/
static int proc_dointvec_wxb(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
      /* get value from buffer into knllog.entries */
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
      if (rc < 0)
         return rc;

		switch(table->ctl_name)
		{
			case BCM_SYSCTL_ddrc_armi_r: 	hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_ARMI, hpm.ddrc.armi.r ); break;
			case BCM_SYSCTL_ddrc_armi_w: 	hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_ARMI, hpm.ddrc.armi.w ); break;
			case BCM_SYSCTL_ddrc_armrw_r: hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_ARMRW, hpm.ddrc.armrw.r ); break;
			case BCM_SYSCTL_ddrc_armrw_w: hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_ARMRW, hpm.ddrc.armrw.w ); break;
			case BCM_SYSCTL_ddrc_whbs_r: 	hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_WHBS, hpm.ddrc.whbs.r ); break;
			case BCM_SYSCTL_ddrc_whbs_w: 	hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_WHBS, hpm.ddrc.whbs.w ); break;
			case BCM_SYSCTL_ddrc_dma0_r: 	hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_DMA0, hpm.ddrc.dma0.r ); break;
			case BCM_SYSCTL_ddrc_dma0_w: 	hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_DMA0, hpm.ddrc.dma0.w ); break;
			case BCM_SYSCTL_ddrc_dma1_r: 	hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_DMA1, hpm.ddrc.dma1.r ); break;
			case BCM_SYSCTL_ddrc_dma1_w: 	hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_DMA1, hpm.ddrc.dma1.w ); break;
			case BCM_SYSCTL_ddrc_vdec_r: 	hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_VDEC, hpm.ddrc.vdec.r ); break;
			case BCM_SYSCTL_ddrc_vdec_w: 	hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_VDEC, hpm.ddrc.vdec.w ); break;
			case BCM_SYSCTL_ddrc_nhbs_r: 	hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_NHBS, hpm.ddrc.nhbs.r ); break;
			case BCM_SYSCTL_ddrc_nhbs_w: 	hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_NHBS, hpm.ddrc.nhbs.w ); break;
			case BCM_SYSCTL_aram_armi_r: 	hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_ARMI, hpm.aram.armi.r ); break;
			case BCM_SYSCTL_aram_armi_w: 	hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_ARMI, hpm.aram.armi.w ); break;
			case BCM_SYSCTL_aram_armrw_r: hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_ARMRW, hpm.aram.armrw.r ); break;
			case BCM_SYSCTL_aram_armrw_w: hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_ARMRW, hpm.aram.armrw.w ); break;
			case BCM_SYSCTL_aram_whbs_r: 	hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_WHBS, hpm.aram.whbs.r ); break;
			case BCM_SYSCTL_aram_whbs_w: 	hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_WHBS, hpm.aram.whbs.w ); break;
			case BCM_SYSCTL_aram_dma0_r: 	hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_DMA0, hpm.aram.dma0.r ); break;
			case BCM_SYSCTL_aram_dma0_w: 	hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_DMA0, hpm.aram.dma0.w ); break;
			case BCM_SYSCTL_aram_dma1_r: 	hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_DMA1, hpm.aram.dma1.r ); break;
			case BCM_SYSCTL_aram_dma1_w: 	hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_DMA1, hpm.aram.dma1.w ); break;
			case BCM_SYSCTL_aram_vdec_r: 	hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_VDEC, hpm.aram.vdec.r ); break;
			case BCM_SYSCTL_aram_vdec_w: 	hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_VDEC, hpm.aram.vdec.w ); break;
			case BCM_SYSCTL_aram_nhbs_r: 	hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_NHBS, hpm.aram.nhbs.r ); break;
			case BCM_SYSCTL_aram_nhbs_w: 	hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_NHBS, hpm.aram.nhbs.w ); break;
			default: break;
		}
   }
   else
   {
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); // No special processing for read.
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   Custom dointvec for dumping arbiter state.
*
*  @return
*/
/****************************************************************************/
static int proc_dointvec_dump(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
      /* get value from buffer into knllog.entries */
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
      if (rc < 0)
         return rc;

		dump();
   }
   else
   {
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); // No special processing for read.
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   Custom dointvec for showing all table strings.
*
*  @return
*/
/****************************************************************************/
static int proc_dointvec_tables(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
      /* get value from buffer into knllog.entries */
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
      if (rc < 0)
         return rc;

		tables();
   }
   else
   {
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); // No special processing for read.
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   Custom dointvec for configuring canned priority settings
*
*  @return
*/
/****************************************************************************/
static int proc_dointvec_cfg(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
      /* get scenario / configuration number */
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
      if (rc < 0)
         return rc;

      if (hpm_cfg((uint32_t)hpm.cfg) < 0 )
      {
         printk("bad hpm configuration number\n");
      }
   }
   else
   {
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); // No special processing for read.
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   Custom dointvec for configuring canned priority settings
*
*  @return
*/
/****************************************************************************/
static int proc_dointvec_qos_enmask(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
      /* get scenario / configuration number */
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
      if (rc < 0)
         return rc;
      priorityTable[gScenario].qos_enmask = hpm.qos_enmask;
      hpmHw_SetDdrcQosEnMask( hpm.qos_enmask );
   }
   else
   {
      hpm.qos_enmask = priorityTable[gScenario].qos_enmask;
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); // No special processing for read.
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   Custom dointvec for configuring canned priority settings
*
*  @return
*/
/****************************************************************************/
static int proc_dointvec_qos_tidemark(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
      /* get scenario / configuration number */
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );
      if (rc < 0)
         return rc;
      priorityTable[gScenario].qos_tidemark = hpm.qos_tidemark;
      hpmHw_SetDdrcQosTmark( hpm.qos_tidemark );
   }
   else
   {
      hpm.qos_tidemark = priorityTable[gScenario].qos_tidemark;
      rc = proc_dointvec(table, write, filp, buffer, lenp, ppos ); // No special processing for read.
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   Custom dostring for setting blocks into priority fields.
*
*  @return
*/
/****************************************************************************/
static int proc_dostring_custom(ctl_table *table, int write, struct file *filp,
           void __user *buffer, size_t *lenp, loff_t *ppos )
{
   int rc;

   if ( !table || !table->data )
      return -EINVAL;

   if ( write )
   {
      /* get value from buffer into knllog.entries */
      rc = proc_dostring(table, write, filp, buffer, lenp, ppos );
      if (rc < 0)
         return rc;

		switch(table->ctl_name)
		{
			case BCM_SYSCTL_vram_0: hpmHw_SetVRAMArbCtrl(0, Str2Code(hpm.vram[0].name, vramBusTbl, ARRAY_LEN(vramBusTbl)));	break;
			case BCM_SYSCTL_vram_1: hpmHw_SetVRAMArbCtrl(1, Str2Code(hpm.vram[1].name, vramBusTbl, ARRAY_LEN(vramBusTbl)));	break;
			case BCM_SYSCTL_vram_2: hpmHw_SetVRAMArbCtrl(2, Str2Code(hpm.vram[2].name, vramBusTbl, ARRAY_LEN(vramBusTbl)));	break;
			case BCM_SYSCTL_vram_3: hpmHw_SetVRAMArbCtrl(3, Str2Code(hpm.vram[3].name, vramBusTbl, ARRAY_LEN(vramBusTbl)));	break;
			case BCM_SYSCTL_vram_4: hpmHw_SetVRAMArbCtrl(4, Str2Code(hpm.vram[4].name, vramBusTbl, ARRAY_LEN(vramBusTbl)));	break;
			case BCM_SYSCTL_whbs_0: hpmHw_SetWHBSArbCtrl(0, Str2Code(hpm.whbs[0].name, whbSbusTbl, ARRAY_LEN(whbSbusTbl)));	break;
			case BCM_SYSCTL_whbs_1: hpmHw_SetWHBSArbCtrl(1, Str2Code(hpm.whbs[1].name, whbSbusTbl, ARRAY_LEN(whbSbusTbl)));	break;
			case BCM_SYSCTL_whbs_2: hpmHw_SetWHBSArbCtrl(2, Str2Code(hpm.whbs[2].name, whbSbusTbl, ARRAY_LEN(whbSbusTbl)));	break;
			case BCM_SYSCTL_nhbs_0: hpmHw_SetNHBSArbCtrl(0, Str2Code(hpm.nhbs[0].name, nhbSbusTbl, ARRAY_LEN(nhbSbusTbl)));	break;
			case BCM_SYSCTL_nhbs_1: hpmHw_SetNHBSArbCtrl(1, Str2Code(hpm.nhbs[1].name, nhbSbusTbl, ARRAY_LEN(nhbSbusTbl)));	break;
			case BCM_SYSCTL_nhbs_2: hpmHw_SetNHBSArbCtrl(2, Str2Code(hpm.nhbs[2].name, nhbSbusTbl, ARRAY_LEN(nhbSbusTbl)));	break;
			case BCM_SYSCTL_nhbs_3: hpmHw_SetNHBSArbCtrl(3, Str2Code(hpm.nhbs[3].name, nhbSbusTbl, ARRAY_LEN(nhbSbusTbl)));	break;
			case BCM_SYSCTL_nhbs_4: hpmHw_SetNHBSArbCtrl(4, Str2Code(hpm.nhbs[4].name, nhbSbusTbl, ARRAY_LEN(nhbSbusTbl)));	break;
			case BCM_SYSCTL_nhbs_5: hpmHw_SetNHBSArbCtrl(5, Str2Code(hpm.nhbs[5].name, nhbSbusTbl, ARRAY_LEN(nhbSbusTbl)));	break;
			case BCM_SYSCTL_nhbs_6: hpmHw_SetNHBSArbCtrl(6, Str2Code(hpm.nhbs[6].name, nhbSbusTbl, ARRAY_LEN(nhbSbusTbl)));	break;
			default: break;
		}
   }
   else
   {
      rc = proc_dostring(table, write, filp, buffer, lenp, ppos ); // No special processing for read.
   }
   return rc;
}
/****************************************************************************/
/**
*  @brief   Dump HPM state.
*
*  @return
*/
/****************************************************************************/
static int dump( void )
{
   unsigned int i;
   unsigned int priority;

   printk("DDRC QoS tidemark value: %d\n", hpmHw_GetDdrcQosTmark());
   printk("DDRC QoS bitwise enable value: 0x%x\n", hpmHw_GetDdrcQosEnMask());
   printk("\n");
   for ( i = 0; i < ARRAY_LEN(wxbTbl); i++ )
   {
      hpmHw_GetDdrcSlaveReadPriority( (HPMHW_WXB)i, &priority );
      printk("DDRC: \'%s\' (%d) read priority = %d\n", Code2Str(i, wxbTbl, ARRAY_LEN(wxbTbl)), i, priority );
      hpmHw_GetDdrcSlaveWritePriority( (HPMHW_WXB)i, &priority );
      printk("DDRC: \'%s\' (%d) write priority = %d\n\n", Code2Str(i, wxbTbl, ARRAY_LEN(wxbTbl)), i, priority );
      hpmHw_GetAramSlaveReadPriority( (HPMHW_WXB)i, &priority );
      printk("ARAM: \'%s\' (%d) read priority = %d\n", Code2Str(i, wxbTbl, ARRAY_LEN(wxbTbl)), i, priority );
      hpmHw_GetAramSlaveWritePriority( (HPMHW_WXB)i, &priority );
      printk("ARAM: \'%s\' (%d) write priority = %d\n\n", Code2Str(i, wxbTbl, ARRAY_LEN(wxbTbl)), i, priority );
   }
   printk("VRAM reg: 0x%08x\n", hpmRegp->hpmHw_arbctrl_vram);
   for ( i = 0; i <= HPMHW_VRAMBUS_MAX_PRIO; i++ )
   {
      HPMHW_VRAMBUS interface;
      hpmHw_GetVRAMArbCtrl( i, &interface );
      printk("VRAM bus: \'%s\' (%d) priority = %d\n", Code2Str(interface, vramBusTbl, ARRAY_LEN(vramBusTbl)), interface, i );
   }
   printk("\n");
   printk("WHBS reg: 0x%08x\n", hpmRegp->hpmHw_arbctrl_whbs);
   for ( i = 0; i <= HPMHW_WHBSBUS_MAX_PRIO; i++ )
   {
      HPMHW_WHBSBUS interface;
      hpmHw_GetWHBSArbCtrl( i, &interface );
      printk("WHBS bus: \'%s\' (%d) priority = %d\n", Code2Str(interface, whbSbusTbl, ARRAY_LEN(whbSbusTbl)), interface, i );
   }
   printk("\n");
   printk("NHBS reg: 0x%08x\n", hpmRegp->hpmHw_arbctrl_nhbs);
   for ( i = 0; i <= HPMHW_NHBSBUS_MAX_PRIO; i++ )
   {
      HPMHW_NHBSBUS interface;
      hpmHw_GetNHBSArbCtrl( i, &interface);
      printk("NHBS bus: \'%s\' (%d) priority = %d\n", Code2Str(interface, nhbSbusTbl, ARRAY_LEN(nhbSbusTbl)), interface, i );
   }
   printk("\n");
   return 0;
}
/****************************************************************************/
/**
*  @brief   Dump string tables.
*
*  @return
*/
/****************************************************************************/

static int tables( void )
{
   unsigned int i;
   printk("\nddrc/aram blocks\n");
   for (i = 0; i<ARRAY_LEN(wxbTbl); i++)
   {
      printk("%s (%d)\n", wxbTbl[i].str, wxbTbl[i].code);
   }
   printk("\nvram blocks\n");
   for (i = 0; i<ARRAY_LEN(vramBusTbl); i++)
   {
      printk("%s (%d)\n", vramBusTbl[i].str, vramBusTbl[i].code);
   }
   printk("\nwhbs blocks\n");
   for (i = 0; i<ARRAY_LEN(whbSbusTbl); i++)
   {
      printk("%s (%d)\n", whbSbusTbl[i].str, whbSbusTbl[i].code);
   }
   printk("\nnhbs blocks\n");
   for (i = 0; i<ARRAY_LEN(nhbSbusTbl); i++)
   {
      printk("%s (%d)\n", nhbSbusTbl[i].str, nhbSbusTbl[i].code);
   }
   return 0;
}


/****************************************************************************
*
*  hpm_cleanup
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
static void hpm_cleanup(void)
{
   /* unregister sysctl table */
   if (gHpmSysCtlHeader != NULL)
   {
      unregister_sysctl_table(gHpmSysCtlHeader);
   }
}

/****************************************************************************
*
*  hpm_cfg
*
*       Called to configure HPM priorities to canned settings.
*
***************************************************************************/
static int hpm_cfg(uint32_t scenario)
{
   if ( scenario >= ARRAY_LEN(priorityTable) )
   {
      return -1;
   }
   gScenario = scenario;

   // DDRC read arbitration (highest is 0)
   hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_ARMI, priorityTable[scenario].ddrc_rd_wxb_armi );
   hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_ARMRW, priorityTable[scenario].ddrc_rd_wxb_armrw );
   hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_WHBS, priorityTable[scenario].ddrc_rd_wxb_whbs );
   hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_DMA0, priorityTable[scenario].ddrc_rd_wxb_dma0 );
   hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_DMA1, priorityTable[scenario].ddrc_rd_wxb_dma1 );
   hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_VDEC, priorityTable[scenario].ddrc_rd_wxb_vdec );
   hpmHw_SetDdrcSlaveReadPriority( HPMHW_WXB_NHBS, priorityTable[scenario].ddrc_rd_wxb_nhbs );

   // DDRC write arbitration (highest is 0)
   hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_ARMI, priorityTable[scenario].ddrc_wr_wxb_armi );
   hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_ARMRW, priorityTable[scenario].ddrc_wr_wxb_armrw );
   hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_WHBS, priorityTable[scenario].ddrc_wr_wxb_whbs );
   hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_DMA0, priorityTable[scenario].ddrc_wr_wxb_dma0 );
   hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_DMA1, priorityTable[scenario].ddrc_wr_wxb_dma1 );
   hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_VDEC, priorityTable[scenario].ddrc_wr_wxb_vdec );
   hpmHw_SetDdrcSlaveWritePriority( HPMHW_WXB_NHBS, priorityTable[scenario].ddrc_wr_wxb_nhbs );

   // ARM read arbitration (highest is 0)
   hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_ARMI, priorityTable[scenario].aram_rd_wxb_armi );
   hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_ARMRW, priorityTable[scenario].aram_rd_wxb_armrw );
   hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_WHBS, priorityTable[scenario].aram_rd_wxb_whbs );
   hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_DMA0, priorityTable[scenario].aram_rd_wxb_dma0 );
   hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_DMA1, priorityTable[scenario].aram_rd_wxb_dma1 );
   hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_VDEC, priorityTable[scenario].aram_rd_wxb_vdec );
   hpmHw_SetAramSlaveReadPriority( HPMHW_WXB_NHBS, priorityTable[scenario].aram_rd_wxb_nhbs );

   // ARAM write arbitration (highest is 0)
   hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_ARMI, priorityTable[scenario].aram_wr_wxb_armi );
   hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_ARMRW, priorityTable[scenario].aram_wr_wxb_armrw );
   hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_WHBS, priorityTable[scenario].aram_wr_wxb_whbs );
   hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_DMA0, priorityTable[scenario].aram_wr_wxb_dma0 );
   hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_DMA1, priorityTable[scenario].aram_wr_wxb_dma1 );
   hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_VDEC, priorityTable[scenario].aram_wr_wxb_vdec );
   hpmHw_SetAramSlaveWritePriority( HPMHW_WXB_NHBS, priorityTable[scenario].aram_wr_wxb_nhbs );

   // VRAM arbitration (highest is 0)
   hpmHw_SetVRAMArbCtrl( priorityTable[scenario].vram_whb_vpmp, HPMHW_VRAMBUS_VPMP );
   hpmHw_SetVRAMArbCtrl( priorityTable[scenario].vram_whb_vpmd, HPMHW_VRAMBUS_VPMD );
   hpmHw_SetVRAMArbCtrl( priorityTable[scenario].vram_whb_dma0, HPMHW_VRAMBUS_DMA0M2 );
   hpmHw_SetVRAMArbCtrl( priorityTable[scenario].vram_whb_dma1, HPMHW_VRAMBUS_DMA1M2 );
   hpmHw_SetVRAMArbCtrl( priorityTable[scenario].vram_whb_whbm, HPMHW_VRAMBUS_WHBM );

   // WHBS arbitration (highest is 0)
   hpmHw_SetWHBSArbCtrl( priorityTable[scenario].whbs_whb_vpmp, HPMHW_WHBSBUS_VPMP );
   hpmHw_SetWHBSArbCtrl( priorityTable[scenario].whbs_whb_vpmd, HPMHW_WHBSBUS_VPMD );
   hpmHw_SetWHBSArbCtrl( priorityTable[scenario].whbs_whb_lcdc, HPMHW_WHBSBUS_LCDC );

   // NHBS arbitration (highest is 0)
   hpmHw_SetNHBSArbCtrl( priorityTable[scenario].nhbs_nhb_ge, HPMHW_NHBSBUS_GEM );
   hpmHw_SetNHBSArbCtrl( priorityTable[scenario].nhbs_nhb_usb0, HPMHW_NHBSBUS_USBH0M );
   hpmHw_SetNHBSArbCtrl( priorityTable[scenario].nhbs_nhb_usb1, HPMHW_NHBSBUS_USBH1M );
   hpmHw_SetNHBSArbCtrl( priorityTable[scenario].nhbs_nhb_sdio0, HPMHW_NHBSBUS_SDIOH0M );
   hpmHw_SetNHBSArbCtrl( priorityTable[scenario].nhbs_nhb_sdio1, HPMHW_NHBSBUS_SDIOH1M );
   hpmHw_SetNHBSArbCtrl( priorityTable[scenario].nhbs_nhb_i2cs, HPMHW_NHBSBUS_I2CSM );
   hpmHw_SetNHBSArbCtrl( priorityTable[scenario].nhbs_nhb_spis, HPMHW_NHBSBUS_SPISM );

   hpmHw_SetDdrcQosTmark( priorityTable[scenario].qos_tidemark );
   hpmHw_SetDdrcQosEnMask( priorityTable[scenario].qos_enmask );

   return 0;
}

/****************************************************************************
*
*  hpm_init
*
*     Called to perform module initialization when the module is loaded
*
***************************************************************************/
static int __init hpm_init(void)
{
	int i;
   printk(banner);

   memset(&hpm, 0, sizeof(hpm));

   /* configure the prioritization settings for standalone (no 1181) */
   hpm_cfg((uint32_t)HPM_SETTING_STANDALONE);

	for ( i = 0; i <= HPMHW_VRAMBUS_MAX_PRIO; i++ )
   {
      HPMHW_VRAMBUS interface;
      hpmHw_GetVRAMArbCtrl( i, &interface );
		strncpy(hpm.vram[i].name, Code2Str(interface, vramBusTbl, ARRAY_LEN(vramBusTbl)), sizeof(hpm.vram[i].name));
   }
   printk("\n");
   for ( i = 0; i <= HPMHW_WHBSBUS_MAX_PRIO; i++ )
   {
      HPMHW_WHBSBUS interface;
      hpmHw_GetWHBSArbCtrl( i, &interface );
		strncpy(hpm.whbs[i].name, Code2Str(interface, whbSbusTbl, ARRAY_LEN(whbSbusTbl)), sizeof(hpm.whbs[i].name));
   }
   printk("\n");
   for ( i = 0; i <= HPMHW_NHBSBUS_MAX_PRIO; i++ )
   {
      HPMHW_NHBSBUS interface;
      hpmHw_GetNHBSArbCtrl( i, &interface);
		strncpy(hpm.nhbs[i].name, Code2Str(interface, nhbSbusTbl, ARRAY_LEN(nhbSbusTbl)), sizeof(hpm.nhbs[i].name));
   }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
   /* register sysctl table */
   gHpmSysCtlHeader = register_sysctl_table(gHpmSysCtl, 0);
   if (gHpmSysCtlHeader == NULL)
   {
      goto fail;
   }
   gHpmSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
#else
   gHpmSysCtlHeader = register_sysctl_table(gHpmSysCtl);
   if (gHpmSysCtlHeader == NULL)
   {
      goto fail;
   }
#endif

   return 0;

 fail:
   hpm_cleanup();
   return -EINVAL;

}

/****************************************************************************
*
*  hpm_exit
*
*       Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/

static void __exit hpm_exit(void)
{
   hpm_cleanup();
}

module_init(hpm_init);
module_exit(hpm_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("HPM Programmable Arbiter Driver");
