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
/*  sleep476x.S - routines to save/restore processor state when entering STANDBY_MEM.\
 *
 * This file contains the routines to save all processor state when the
 * system will be entering the standby to memory suspend state. Since the
 * core and nearly all logic in the BCM4760 except for the PML/A-ON domain is
 * powered off all information is either saved in the BBL logic, also in
 * the A-ON domain, or DDR. DDR is placed into self-refresh since the
 * DDR controller is also powered off. Prior to entering the save
 * processor state function in this file the information necessary to
 * re-configure the DDR controller must have been saved in the BBL
 * registers reserved for that purpose. The Boot ROM will restore
 * those values to the DDR controller upon the warm boot and the PML
 * logic for resume will take the DDR back out of self-refresh mode.
 */
/*
 * linux/arch/arm/mach-bcm476x/sleep476x.S
 * derived from heavily modifieed linux/arch/arm/mach-omap2/sleep34xx.S
 *
 * (C) Copyright 2007
 * Texas Instruments
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * (C) Copyright 2004
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR /PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <linux/linkage.h>
#include <linux/autoconf.h>
#include <asm/assembler.h>
#include <asm/memory.h>
#include <asm/arch/bcm4760_reg.h>
//#include <mach/io.h>
//#include <mach/pm.h>
//#include <mach/control.h>

//#define CONFIG_BCM4760_PMSTR_SW_PWRDOWN

#if 0
#include "prm.h"
#include "sdrc.h"

#define PM_PREPWSTST_CORE_V	OMAP34XX_PRM_REGADDR(CORE_MOD, \
				OMAP3430_PM_PREPWSTST)
#define PM_PREPWSTST_MPU_V	OMAP34XX_PRM_REGADDR(MPU_MOD, \
				OMAP3430_PM_PREPWSTST)
#define PM_PWSTCTRL_MPU_P	OMAP34XX_PRM_REGADDR(MPU_MOD, PM_PWSTCTRL)
#define SCRATCHPAD_MEM_OFFS	0x310 /* Move this as correct place is
				       * available */
#define SCRATCHPAD_BASE_P	OMAP343X_CTRL_REGADDR(\
				OMAP343X_CONTROL_MEM_WKUP +\
				SCRATCHPAD_MEM_OFFS)
#define SDRC_POWER_V		OMAP34XX_SDRC_REGADDR(SDRC_POWER)
#endif

#define MODE_USR       0x10
#define MODE_FIQ       0x11
#define MODE_IRQ       0x12
#define MODE_SVC      0x13
#define MODE_ABT       0x17
#define MODE_UND      0x1B
#define MODE_SYS       0x1F

	.text
	
pgoffset:	.word	PAGE_OFFSET
phoffset:	.word	PHYS_OFFSET

/*
 * Forces BCM4760 into idle state
 *
 * bcm476x_cpu_enter_sleep() - Save processor state and execute WFI.
 *
 * r0 - Address to store state information to in DDR.
 *
 */
	.globl	bcm476x_cpu_enter_sleep
ENTRY(bcm476x_cpu_enter_sleep)
	stmfd	sp!, {r0-r12, lr}		/* save registers */

//	str		r0, context_vbase

	cpsid	aif

	mov		r1, r0					/* Store SDRAM address in r1 */

	/*
	 * We are going to store all processor context information to DDR. Since
	 * the processor core is going to be powered off it will be lost. The
	 * DDR will be in self-refresh so it will be preserved. Since the processor
	 * context is in DDR we will reload it back into the processor after the
	 * warm boot occurs due to some wake-up event such as power button, alarm, etc.
	 */
	
 	/* Store sp and spsr to SDRAM */
	mov		r4, sp
	mrs		r5, spsr
	mov		r6, lr
	stmia	r1!, {r4-r6}

	/* Save CPSR */
	mrs		r2, cpsr
	str		r2, [r1], #4
	
	/* Store SP, LR, SPSR registers for SUP, FIQ, IRQ, ABORT and USER
	modes into SDRAM */

	cps		#MODE_FIQ			/* disable interrupts and enter FIQ mode */

	/* Save FIQ mode */
	stmia	r1!, {r8-r14}
	mrs		r6, spsr
	str		r6, [r1], #4

	/* Save IRQ mode */
	cps		#MODE_IRQ
	stmia	r1!, {r13-r14}
	mrs		r6, spsr
	str		r6, [r1], #4

	/* Save Abort mode */
	cps		#MODE_ABT
	stmia	r1!, {r13-r14}
	mrs		r6, spsr
	str		r6, [r1], #4

	/* Save UNDEF mode */
	cps		#MODE_UND
	stmia	r1!, {r13-r14}
	mrs		r6, spsr
	str		r6, [r1], #4

	/* Save System (USER mode) */
	cps		#MODE_SVC
	stmia	r1!, {r13-r14}
	mrs		r6, spsr
	str		r6, [r1], #4

	/* Back to system mode */
	cps		#MODE_SYS

	/* Save domain access control register, data fault status register,
	 * and instruction fault status register
	 */
	mrc		p15, 0, r4, c3, c0, 0
	mrc		p15, 0, r5, c5, c0, 0
	mrc		p15, 0, r6, c5, c0, 1
	stmia	r1!, {r4-r6}

	/* Save TTBR0, TTBR1 and Translation table base control */
	mrc		p15, 0, r4, c2, c0, 0
	mrc		p15, 0, r5, c2, c0, 1
	mrc		p15, 0, r6, c2, c0, 2
	stmia	r1!, {r4-r6}

	/* Save data aux fault status register, instruction aux fault status,
	 * data fault address register and instruction fault address register
	 * actually Data Fault Address and Watchpoint Fault Address registers.
	 */
//	mrc		p15, 0, r4, c5, c1, 0
//	mrc		p15, 0, r5, c5, c1, 1
	mrc		p15, 0, r6, c6, c0, 0
	mrc		p15, 0, r7, c6, c0, 1
//	mrc		p15, 0, r7, c6, c0, 2
	stmia	r1!, {r6-r7}

	/* Save user r/w thread and process ID, user r/o thread and process ID,
	 * priv only thread and process ID, cache size selection
	 */
	mrc		p15, 0, r4, c13, c0, 2
	mrc		p15, 0, r5, c13, c0, 3
	mrc		p15, 0, r6, c13, c0, 4
//	mrc		p15, 2, r7, c0, c0, 0
	stmia	r1!, {r4-r6}

	/* Save data TLB lockdown, instruction TLB lockdown registers */
	mrc		p15, 0, r5, c10, c0, 0
//	mrc		p15, 0, r6, c10, c0, 1
	stmia	r1!, {r5}

	/* Save secure or non secure vector base address, FCSE PID, Context PID*/
//	mrc		p15, 0, r4, c12, c0, 0
	mrc		p15, 0, r5, c13, c0, 0
	mrc		p15, 0, r6, c13, c0, 1
	mrc		p15, 0, r7, c10, c0, 0
	stmia	r1!, {r5-r7}

	/* Save primary remap, normal remap registers */
	mrc		p15, 0, r4, c10, c2, 0
	mrc		p15, 0, r5, c10, c2, 1
	stmia	r1!, {r4-r5}

	/* Save data, instruction, DMA and debug remap registers */
	mrc		p15, 0, r4, c15, c2, 0
	mrc		p15, 0, r5, c15, c2, 1
	mrc		p15, 0, r6, c15, c2, 2
	mrc		p15, 0, r7, c15, c2, 4
	stmia	r1!, {r4-r7}

	/* Save cache debug control and TLB debug control */
//	mrc		p15, 7, r6, c15, c0, 0
//	mrc		p15, 7, r7, c15, c1, 0
//	stmia	r1!, {r6-r7}

	/* Save control register */
//	mrc		p15, 0, r4, c1, c0, 0
//	stmia	r1!, {r4}

	/* Save VFP State */
	fmrx	r2, fpexc
	orr	r0, r2, #1 << 30
	fmxr	FPEXC,r0
	fmrx	r3, fpscr
	fmrx	r4, fpinst
	fmrx	r5, fpinst2
	stmia	r1!, {r2-r5}
	fstmiax	r1!, {d0-d15}

	/* Save control, aux control and coprocessor access control register */
	mrc		p15, 0, r5, c1, c0, 0
	mrc		p15, 0, r6, c1, c0, 1
	mrc		p15, 0, r7, c1, c0, 2
	stmia	r1!, {r5-r7}

	/* Save performance monitoring registers */
	mrc		p15, 0, r2, c15, c12, 0
	mrc		p15, 0, r3, c15, c12, 1
	mrc		p15, 0, r4, c15, c12, 2
	mrc		p15, 0, r5, c15, c12, 3
	stmia	r1!, {r2-r5}

	/* [JLH] hack! - need more portable way to convert virt to phys in ARM assembly */
//	ldr		r0, pgoffset
//	sub		r1, r0
//	ldr		r0, phoffset
//	add		r1, r0

	str		r1, context_pend		/* save for warm boot process */

//	ldr		r0, gporc10
//	mov		r2, #0xD0000000
//	add		r0, r2
//	mov		r2, #0x00080000
//	str		r2, [r0] 
//	ldr		r0, gpctr24
//	mov		r2, #0xD0000000
//	add		r0, r2
//	mov		r2, #0x00000200
//	str		r2, [r0]

	b		1f
	.align 5
1:
	cpsid	aif						/* Make double sure interrupts are disabled */

	/* Clean Data or unified cache to POU*/
	/* Data memory barrier and Data sync barrier */

	mov     r0, #0
	mcr     p15, 0, r0, c7, c14, 0  /* clean and invalidate data L1 cache */
	mcr     p15, 0, r0, c7, c10, 4  /* DSB */
	mcr     p15, 0, r0, c7, c10, 5  /* DMB */

#ifdef CONFIG_BCM4760_PMSTR_SW_PWRDOWN
	ldr		r1, sw_isolate
	mov		r2, #0xD0000000
	add		r1, r2
	ldr		r2, sw_isomask
	str		r2, [r1]				/* write sw isolation mask register */
#endif
	mcr		p15, 0, r0, c7, c0, 4	/*wait for interrupt */

	/* Change from Broadcom code. If we get here, don't hang. Go through the resume path instead. */
	mov		r0, #0x500000	/* Simple delay loop around 10ms. */
1:	subs		r0, r0, #1		/* This is to ensure that we don't go through the resume path while PML logic is still suspending. */
	bne		1b			/* If we fall through, at least we resume. */

	ldr		r1, sleep_stat
	orr		r1, #2                          /* indicate sleep cancelled */
	str		r1, sleep_stat
//	cpsie	i

//	mov		r0,#0
//	mcr		p15, 0, r0, c7, c0, 4

//	wfi                             /* wait for interrupt */

	/* Reenable VFP. Otherwise it will cause a hang. */
	fmrx	r0, fpexc
	bic	r0, r0, #1 << 30
	fmxr	FPEXC,r0
	b bcm476x_cpu_exit_sleep

noploophere:
	nop
	b		noploophere

//context_vbase:	.word		0
context_pend:	.word		0
//context_size:	.word		0
uart0:		.word		URT0_REG_BASE_ADDR
gpctr24:	.word		GIO1_R_GPCTR26_MEMADDR
gporc10:	.word		GIO1_R_GPORC0_MEMADDR
gpors10:	.word		GIO1_R_GPORS0_MEMADDR
sw_isolate:	.word		PML_R_PML_SW_ISOLATE_MEMADDR
sw_isomask:	.word		PML_R_PML_SW_ISOLATE_MASK
sw_phy1:	.word		CMU_R_USB_PHY1_MEMADDR
sw_bc_det:      .word           CMU_R_BC_DETECT_CTRL_MEMADDR
sleep_stat: .word       0

/*
 * Brings BCM4760 back into active state
 *
 * bcm476x_cpu_exit_sleep() - Restore processor state and return to original caller.
 *
 *
 */
	.globl	bcm476x_cpu_exit_sleep
ENTRY(bcm476x_cpu_exit_sleep)
	cpsid	aif, #MODE_SYS			/* enter system mode */
	ldr		r1, context_pend		/* load end of context save area */

	/*
	 * We basically walk through the context save area in reverse order to
	 * restore everything to the pre-sleep mode settings.
	 */

	/* Restore performance monitoring registers */
	ldmdb	r1!, {r2-r5}
	mcr		p15, 0, r2, c15, c12, 0
	mcr		p15, 0, r3, c15, c12, 1
	mcr		p15, 0, r4, c15, c12, 2
	mcr		p15, 0, r5, c15, c12, 3

	/* Restore control, aux control and coprocessor access control register */
	ldmdb	r1!, {r5-r7}
	bic		r5, r5, #1				// force MMU off for now
	mcr		p15, 0, r5, c1, c0, 0
	mcr		p15, 0, r6, c1, c0, 1
	mcr		p15, 0, r7, c1, c0, 2

	/* Restore VFP State */
	fmrx	r2, fpexc
	orr		r0, r2, #1 << 30
	fmxr	FPEXC,r0
	fldmdbx	r1!, {d0-d15}
	ldmdb	r1!, {r2-r5}
	fmxr	fpscr, r3
	fmxr	fpinst, r4
	fmxr	fpinst2, r5

	/* Restore control register */
//	ldmdb	r1!, {r4}
//	mcr		p15, 0, r4, c1, c0, 0

	/* Restore cache debug control and TLB debug control */
//	ldmdb	r1!, {r6-r7}
//	mcr		p15, 7, r6, c15, c0, 0
//	mcr		p15, 7, r7, c15, c1, 0

	/* Restore data, instruction, DMA and debug remap registers */
	ldmdb	r1!, {r4-r7}
	mcr		p15, 0, r4, c15, c2, 0
	mcr		p15, 0, r5, c15, c2, 1
	mcr		p15, 0, r6, c15, c2, 2
	mcr		p15, 0, r7, c15, c2, 4

	/* Restore primary remap, normal remap registers */
	ldmdb	r1!, {r4-r5}
	mcr		p15, 0, r4, c10, c2, 0
	mcr		p15, 0, r5, c10, c2, 1

	/* Restore secure or non secure vector base address, FCSE PID, Context PID*/
	ldmdb	r1!, {r5-r7}
//	mcr		p15, 0, r4, c12, c0, 0
	mcr		p15, 0, r5, c13, c0, 0
	mcr		p15, 0, r6, c13, c0, 1
	mcr		p15, 0, r7, c10, c0, 0

	/* Restore data TLB lockdown, instruction TLB lockdown registers */
	ldmdb	r1!, {r5}
	mcr		p15, 0, r5, c10, c0, 0
//	mcr		p15, 0, r6, c10, c0, 1

	/* Restore user r/w thread and process ID, user r/o thread and process ID,
	 * priv only thread and process ID, cache size selection
	 */
	ldmdb	r1!, {r4-r6}
	mcr		p15, 0, r4, c13, c0, 2
	mcr		p15, 0, r5, c13, c0, 3
	mcr		p15, 0, r6, c13, c0, 4
//	mcr		p15, 2, r7, c0, c0, 0

	/* Restore data aux fault status register, instruction aux fault status,
	 * data fault address register and instruction fault address register
	 * actually Data Fault Address and Watchpoint Fault Address registers.
	 */
	ldmdb	r1!, {r6-r7}
//	mcr		p15, 0, r4, c5, c1, 0
//	mcr		p15, 0, r5, c5, c1, 1
	mcr		p15, 0, r6, c6, c0, 0
	mcr		p15, 0, r7, c6, c0, 1
//	mcr		p15, 0, r7, c6, c0, 2

	/* Restore TTBR0, TTBR1 and Translation table base control */
	ldmdb	r1!, {r4-r6}
	mcr		p15, 0, r4, c2, c0, 0
	mcr		p15, 0, r5, c2, c0, 1
	mcr		p15, 0, r6, c2, c0, 2

	/* Restore domain access control register, data fault status register,
	 * and instruction fault status register
	 */
	ldmdb	r1!, {r4-r6}
	mcr		p15, 0, r4, c3, c0, 0
	mcr		p15, 0, r5, c5, c0, 0
	mcr		p15, 0, r6, c5, c0, 1

	cpsid	if, #MODE_SVC		/* disable interrupts and enter SVC mode */

	/* Restore System (USER mode) */
	ldr		r6, [r1, #-4]!
	msr		spsr_cxsf, r6
	ldmdb	r1!, {r13-r14}

	/* Restore UNDEF mode */
	cps		#MODE_UND
	ldr		r6, [r1, #-4]!
	msr		spsr_cxsf, r6
	ldmdb	r1!, {r13-r14}

	/* Restore ABT mode */
	cps		#MODE_ABT
	ldr		r6, [r1, #-4]!
	msr		spsr_cxsf, r6
	ldmdb	r1!, {r13-r14}

	/* Restore IRQ mode */
	cps		#MODE_IRQ
	ldr		r6, [r1, #-4]!
	msr		spsr_cxsf, r6
	ldmdb	r1!, {r13-r14}

	/* Restore FIQ mode */
	cps		#MODE_FIQ
	ldr		r6, [r1, #-4]!
	msr		spsr_cxsf, r6
	ldmdb	r1!, {r8-r14}

	cpsid	if, #MODE_SVC		/* disable interrupts and enter SVC (user) mode */

	/* Restore CPSR */
	ldr		r2, [r1, #-4]!
	orr		r2, r2, #0x80		/* Ensure interrupts stay disabled for a while */ 
	msr		cpsr_cxsf, r2
	
 	/* Restore sp and spsr */
	ldmdb	r1!, {r4-r6}
	mov		sp, r4
	msr		spsr_cxsf, r5
	mov		lr, r6

	/* Turn MMU back on */
	mrc		p15, 0, r0, c1, c0, 0
	orr		r0, r0, #5
	mcr		p15, 0, r0, c1, c0, 0

	ldmia	sp!, {r0-r12, lr}		/* restore registers */

	ldr		r0, sleep_stat		/* Function return is final sleep status */

	mov		pc, r14





