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




/****************************************************************************/
/**
*  @file    rtcHw_reg.h
*
*  @brief   Definitions for low level RTC and BBL registers
*
*/
/****************************************************************************/
#ifndef RTCHW_REG_H
#define RTCHW_REG_H

#include <asm/arch/hardware.h>
#include <asm/arch/bcm4760_reg.h>

#define rtchw_MODULE_BASE_ADDR       IO_ADDRESS(RTC_REG_BASE_ADDR)

typedef struct
{
   uint32_t addr;                /* BBL address register */
   uint32_t data;                /* BBL data register */
   uint32_t tamper;              /* Tamper interrupt */
}rtchw_REG_t;

#define pRtcHw (( volatile rtchw_REG_t *) rtchw_MODULE_BASE_ADDR)



/* RTC Timer Register Address */
#define rtchw_RSRST_ADDR				              ( 0x00000000 )
#define rtchw_PERIODIC_TIMER_ADDR             ( 0x00000001 )
#define rtchw_MATCH_REGISTER_ADDR             ( 0x00000002 )
#define rtchw_RTC_REGISTER_ADDR               ( 0x00000004 )
#define rtchw_CLEAR_INTR_ADDR                 ( 0x00000005 )
#define rtchw_CURRENT_TIME_ADDR               ( 0x0000000e )
#define rtchw_INTERRUPT_STATUS_ADDR           ( 0x0000000f )
#define rtchw_CONTROL_ADDR                    ( 0x00000010 )


/* Macro for pRtcHw->addr register */
//In 4760, there is no different between rtchw_REG_SET_BBL_ADDRESS(address) and rtchw_REG_SET_RTC_ADDRESS(address)
//#define rtchw_REG_SET_BBL_ADDRESS(address)    ( pRtcHw->addr = ( pRtcHw->addr & 0x00000003 ) | (((address) << 8) & 0x00001F00 ) )
#define rtchw_REG_SET_BBL_ADDRESS(address)    rtchw_REG_SET_RTC_ADDRESS(address)
#define rtchw_REG_SET_RTC_ADDRESS(address)    ( pRtcHw->addr = ( pRtcHw->addr & 0x00000003 ) | (((address) << 8) & 0x00001F00 ) )
#define rtchw_REG_IS_COMMAND_COMPLETE()       ( ! ( pRtcHw->addr & 0x00000001 ) )
#define rtchw_REG_COMMAND_READ()              ( pRtcHw->addr = ( pRtcHw->addr & 0x00001F00 ) | 0x00000001 )
#define rtchw_REG_COMMAND_WRITE()             ( pRtcHw->addr = ( pRtcHw->addr & 0x00001F00 ) | 0x00000003 ) 

/* Macro for pRtcHw->data register */
#define rtchw_REG_GET_DATA()                  ( pRtcHw->data )
#define rtchw_REG_SET_DATA(val)               ( pRtcHw->data = (uint32_t)(val) ) 

/* Macro for BBL RSRST control bit defines */
#define rtchw_CMD_BBL_RTC_STOP                0x00000080
//#define rtchw_CMD_BBL_RTC_LOCK                0x00000004

/* Macro for BBL control bit defines */
#define rtchw_CMD_RTC_INT_ENABLE              0x00000002
#define rtchw_CMD_PER_INT_ENABLE              0x00000001

/* Macro for BBL interrupt clear register */
#define rtchw_CMD_PER_INTR_CLEAR              0x00000001
#define rtchw_CMD_RTC_INTR_CLEAR              0x00000002

/* Timer interrupt status */
#define rtchw_CMD_PERIODIC_INTERRUPT_STATUS   (1 << BCM4760_INTR_RTC)
#define rtchw_CMD_ONESHOT_INTERRUPT_STATUS    (1 << (BCM4760_INTR_RTC_MTCH-IRQ_INTC1_START))


/* Tamper interrupt */
#define rtchw_REG_TAMPER_INTERRUPT_STATUS     0x00000002
#define rtchw_REG_TAMPER_INTERRUPT_ENABLE     0x00000001


#endif /* RTCHW_REG_H */
