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
*  @file    rngHw_reg.h
*
*  @brief   Definitions for low level random number generator registers
*
*/
/****************************************************************************/
#ifndef RNGHW_REG_H
#define RNGHW_REG_H

//#include <mach/csp/mm_io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/bcm4760_reg.h>

#define rngHw_BASE_ADDR              IO_ADDRESS(RNG_REG_BASE_ADDR)

typedef struct
{
   uint32_t Control;            /* Random Number Generator Register */
   uint32_t Status;             /* Random Number Generator Status Register */
   uint32_t Data;               /* Random Number Generator Data <#> Register */
   uint32_t FifoThreshold;      /* RNG FIFO Threshold */
   uint32_t InterruptMask;      /* RNG Interrupt Mask Register */
}rnghw_REG_t;

#define pRngHw  (( volatile rnghw_REG_t* ) rngHw_BASE_ADDR)


/* Bit definition for pRngHw->Control */
#define rngHw_REG_SET_RANDOM_BIT_GEN_ENABLE()   ( pRngHw->Control |= 0x00000001 )
#define rngHw_REG_SET_RANDOM_BIT_GEN_DISABLE()  ( pRngHw->Control &= 0xFFFFFFFE )
#define rngHw_REG_SET_RANDOM_BIT_SPEED_DOUBLE() ( pRngHw->Control |= 0x00000002 )
#define rngHw_REG_SET_RANDOM_BIT_SPEED_NORMAL() ( pRngHw->Control &= ~0x00000002 )

/* Bit definition for pRngHw->Status */
#define rngHw_REG_WARMUP_MASK                   0x000FFFFF
#define rngHw_REG_SET_WARMUP_CYCLE(val)         ( pRngHw->Status = rngHw_REG_WARMUP_MASK - ((val) & rngHw_REG_WARMUP_MASK) )
#define rngHw_REG_GET_VALID_WORDS()             ( pRngHw->Status & 0xFF000000) >> 24 

#define rngHw_REG_SET_INT_ENABLE()              ( pRngHw->InterruptMask = 0x00000000 )
#define rngHw_REG_SET_INT_DISABLE()             ( pRngHw->InterruptMask = 0x00000001 )


#endif /* RNGHW_REG_H */
