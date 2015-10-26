/*
 *  linux/arch/arm/mach-s3c2410/cpufreq_change.S
 *
 *  Copyright (C) 2006, 2007 TomTom B.V.
 *
 */
#include <linux/config.h>
#include <asm/hardware.h>
#include <asm/arch/map.h>
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/arch/regs-mem.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-rtc.h>
#include <asm/arch/regs-irq.h>

	.text

@ Input (R0) is the current CPU clockrate.
delay2ms:
	mov	r0, r0, LSL #1					@ 2 instr, so divide by 2.
delay2ms_loop:							@ Delay for about 2ms (2*976us).
	subs	r0, r0, #1					@
	bne	delay2ms_loop					@ 2nd of the 2 instructions.
	mov	pc, lr						@ Return.... (R0 is destroyed!)

asm_div:							@ Input (R0=R10/R12).
	mov	r0, #0						@ Zero out R0.
asm_div_loop:							@
	subs	r10, r10, r12					@ Substract ...
	bls	asm_div_end					@ Result below 0? Then we're done.
	add	r0, r0, #1					@ Increment R0 for every substraction
	b	asm_div_loop					@
asm_div_end:							@ Endresult in R0. R10 is destroyed!
	mov	pc, lr						@ Return....

ENTRY( asm_div_64 )
asm_div_64:							@ Input (R0=R11:R10/R12).
	mov	r0, #0						@ Zero out R0.
asm_div_loop_64:						@
	subs	r10, r10, r12					@ Substract ...
	sbcs	r11, r11, #0					@ Read this as R11:R10=R11:R10 - 0:R12
	blo	asm_div_end_64					@ Result 0 or lower, then we're done 64 bit version.
	cmpeq	r10, #0						@
	beq	asm_div_end_64					@
	add	r0, r0, #1					@ Increment R0 for every substraction
	b	asm_div_loop_64					@
asm_div_end_64:							@ Endresult in R0. R10 and R11 are destroyed!
	mov	pc, lr						@ Return....

@ Initialize RTC Tick interrupt for delay routine. Note that the first tick happens after 300 ms.
@ To work around this, this initializing routine saves the registers in R0, then waits for the first tick
@ to happen. R2, R3 are destroyed.
s3c2412_rtc_init_delay:
	mov	r0, #0

	@ Now set the interrupt controller, RTCCON and TICNT.
	ldr	r3, =S3C2410_INTMSK		@ Get the interrupt mask.
	ldr	r2, [ r3 ]			@
	tst	r2, #0x00000100			@ Check if INT_TICK is masked.
	orrne	r0, r0, #0x00000080		@ Set bit 7 if INT_TICK is masked.
	orr	r2, r2, #0x00000100		@ Ensure INT_TICK is masked.
	str	r2, [ r3 ]			@ Write back. INT_TICK is now masked.

	ldr	r3, =S3C2410_RTCCON		@ Load RTCCON.
	ldr	r2, [ r3 ]			@
	and	r2, r2, #0xFF			@ Mask all but low 8 bits.
	orr	r0, r0, r2, LSL #23		@ Put RTCCON into [31:24].
	mov	r2, #0x11			@ Set 32KHz, RTC Control Enable.
	str	r2, [ r3 ]

	ldr	r3, =S3C2410_TICNT1		@ RTC Tick Count low 8 bits
	ldr	r2, [ r3 ]			@
	and	r2, r2, #0xFF			@ Mask all but low 7 bits.
	orr	r0, r0, r2, LSL #8		@ Put RTC Tick count (low 8 bits) into [15:8]
	mov	r2, #0x21			@ Set tickcount to 33.
	str	r2, [ r3 ]			@ Write back.

	ldr	r3, =S3C2410_TICNT0		@ RTC Tick Count high 7 bits + enable
	ldr	r2, [ r3 ]			@
	and	r2, r2, #0xFF			@ Mask all but low 8 bits.
	orr	r0, r0, r2, LSL #16		@ Put RTC Tick count (high 8 bits) into [23:16]
	mov	r2, #0x80			@ Enable the interrupt.
	str	r2, [ r3 ]			@ Write back.

	@ Now clear any pending interrupts.
	ldr	r3, =S3C2410_SRCPND		@ Src Pending register.
	mov	r2, #0x00000100			@ Clear outstanding RTC Tick IRQ.
	str	r2, [ r3 ]			@ 

	@ We are now running 32768 ticks per second. This means that 32 ticks is about 1ms.
s3c2412_rtc_init_delay_loop:
	ldr	r3, =S3C2410_SRCPND		@ Src Pending register.
	ldr	r2, [ r3 ]			@ Check src pending.
	tst	r2, #0x00000100			@ Check if Tick IRQ happened.
	beq	s3c2412_rtc_init_delay_loop	@ Just continue if no IRQ.
	mov	r2, #0x00000100			@ Clear Tick IRQ.
	ldr	r3, =S3C2410_SRCPND		@ Src Pending register.
	str	r2, [ r3 ]			@
	ldr	r3, =S3C2410_INTPND		@ INT Pending register.
	str	r2, [ r3 ]			@

	@ If we get here, the first interrupt happened, so it's initialized. Return saved registers in R0.
	mov	pc, lr				@ Return. R0 has saved regs. R3, R4 destroyed.

@ This routine restores the RTC Timer Tick to it's original settings, saved values being in R0.
@ R2 and R3 are destroyed upon exit.
s3c2412_rtc_restore_delay:
	@ Done delaying. Restore registers.
	and	r3, r0, #0x00FF0000		@ Get RTC Tick Count high 7 bits + enable saved value.
	mov	r3, r3, LSR #16			@ in low byte.
	ldr	r2, =S3C2410_TICNT0		@ Write high count back.
	str	r3, [ r2 ]			@

	and	r3, r0, #0x0000FF00		@ Get RTC Tick Count high 7 bits + enable saved value.
	mov	r3, r3, LSR #8			@ in low byte.
	ldr	r2, =S3C2410_TICNT1		@ Write low count back.
	str	r3, [ r2 ]			@

	and	r3, r0, #0x00FF0000		@ Get RTCCON saved value.
	mov	r3, r3, LSR #23			@ in low byte.
	ldr	r2, =S3C2410_RTCCON		@ Write low count back.
	str	r3, [ r2 ]			@

	@ Now clear any pending interrupts.
	ldr	r3, =S3C2410_SRCPND		@ INT Pending register.
	mov	r2, #0x00000100			@ Clear outstanding RTC Tick IRQ.
	str	r2, [ r3 ]			@ 
	ldr	r3, =S3C2410_INTPND		@ Src Pending register.
	mov	r2, #0x00000100			@ Clear outstanding RTC Tick IRQ.
	str	r2, [ r3 ]			@ 

	ldr	r3, =S3C2410_INTMSK		@ Get the interrupt mask.
	ldr	r2, [ r3 ]			@
	tst	r0, #0x00000080			@ Check if the interrupt was masked or not.
	orrne	r2, r2, #0x00000100		@ Mask if it was.
	biceq	r2, r2, #0x00000100		@ Clear if it wasnt.
	str	r2, [ r3 ]			@ Store the result back.

	mov	pc, lr				@ Return. R0, R2, R3 destroyed.

@ Input: R0 - number of ms to delay. R0, R3, R2 are destroyed upon return.
s3c2412_rtc_delay_ms:
	cmp	r0, #0				@ Check if its zero.
	beq	s3c2412_rtc_delay_done		@ Go to done if it is zero.

	@ Now clear any pending interrupts.
	ldr	r3, =S3C2410_SRCPND		@ Src Pending register.
	mov	r2, #0x00000100			@ Clear outstanding RTC Tick IRQ.
	str	r2, [ r3 ]			@ 

	@ We are now running 32768 ticks per second. This means that 32 ticks is about 1ms.
s3c2412_rtc_delay_loop:
	ldr	r3, =S3C2410_SRCPND		@ Src Pending register.
	ldr	r2, [ r3 ]			@ Check src pending.
	tst	r2, #0x00000100			@ Check if Tick IRQ happened.
	beq	s3c2412_rtc_delay_loop		@ Just continue if no IRQ.
	mov	r2, #0x00000100			@ Clear Tick IRQ.
	ldr	r3, =S3C2410_SRCPND		@ Src Pending register.
	str	r2, [ r3 ]			@
	ldr	r3, =S3C2410_INTPND		@ INT Pending register.
	str	r2, [ r3 ]			@
	subs	r0, r0, #1			@ Substract one....
	bne	s3c2412_rtc_delay_loop

s3c2412_rtc_delay_done:
	mov	pc, lr				@ Return. R0, R2, R3 destroyed.

@ R0 - MPLLCON, R1 - CLKDIVN, R2 - n->freq, R3 - o->freq, R4 - new_hclk, R5 - old_hclk
ENTRY( s3c2410_freq_change )
	stmfd	sp!, {r0-r10, r12, r14}				@ Save used regs.

	@ Save and disable interrupts (Both FIQ and IRQ).
	mrs	r9, cpsr					@ Save flags
	stmfd	sp!, {r9}					@
	orr	r9, r9, #(3 << 6)				@ Disable FIQ and IRQ
	msr	cpsr_c, r9					@

	@ Save MPLLCON before it gets destroyed.
	mov	r8, r0						@ Save MPLLCON in R8.

	@ Calculate the new refresh value.
	ldr	r7, =S3C2410_REFRESH				@ Load address of the REFRESH register.
	ldr	r6, [ r7 ]					@ Get the old refresh value.
	ldr	r9, =0x000007FF					@ Set mask to mask out only the refresh counter bits.
	and	r12, r6, r9					@ Mask out only the refresh bits.
	bic	r6, r6, r9					@ Clear out only the refresh bits.
	add	r9, r9, #0x02					@ Two higher, we need to calculate what the rate is...
	sub	r12, r9, r12					@ R12 now contains the actual refresh count.
	mla	r10, r12, r2, r3				@ Calculate new refresh. R10=(R12*n->freq)+o->freq
	sub	r10, r10, #1					@ R10-=1
	mov	r12, r3						@ Set R12=o->freq
	bl	asm_div						@ R0=R10/R12 (=(R12 * n->freq + o->freq - 1)/o->freq)
	sub	r12, r9, r0					@ Back to value in register...
	orr	r12, r12, r6					@ Set the other bits back also.

	@ Compact the parameters. We only need to know whether or not n->freq > o->freq and new_hclk > old_hclk.
	mov	r10, #0						@ Set to 0.
	cmp	r5, r4						@ if( old_hclk < new_hclk )
	orrlo	r10, r10, #0x00000001				@ flag old_hclk < new_hclk bit.
	bichs	r10, r10, #0x00000001				@ clear old_hclk < new_hclk bit.
	cmp	r3, r2						@ if( o->freq < n->freq )
	orrlo	r10, r10, #0x00000002				@ flag o->freq < n->freq bit
	bichs	r10, r10, #0x00000002				@ clear o->freq < n->freq bit
	movhi	r3, r3						@ Save highest freq (o->freq or n->freq) in R3. This
	movls	r3, r2						@ will give the longest delay.
	mov	r4, r10						@ Save flags.
	mov	r2, r8						@ Put MPLLCON in the final reg.

	@ Save max refresh value..
	mov	r0, r6						@ Max refresh, is lowest 11 bits to 0.

	@ Load the contents of the registers.
	ldr	r5, =S3C2410_MPLLCON				@ Get the addresses of MPLLCON, CLKSLOW, CLKDIVN.
	ldr	r6, =S3C2410_CLKSLOW				@ 
	ldr	r8, =S3C2410_CLKDIVN				@
	ldr	r9, [ r6 ]					@ Load CLKSLOW register

	@ Set highest refresh rate. Will always work, and we don't need memory right now anyway, so performance doesnt matter.
	@ Always works. Later we'll set the accurate one.
	str	r0, [ r7 ]					@ Set refresh.
	mov	r0, r3						@ delay about 2ms (1952us)
	bl	delay2ms					@ 

	@ Now check if we go from a lower clock to a higher clock. If so
	@ we need to set the dividers before switching.		
	tst	r4, #0x00000002					@ if( n->freq > o->freq )
	strne	r1, [ r8 ]					@       set CLKDIVN (dividers)
	mov	r0, r3						@	Save lowest freq (o->freq or n->freq) in R0.

	@ Check if MPLLCON is 0. If so, we need to deal with it differently.
	cmp	r2, #0						@ if( MPLLCON == 0 )
	bne	mpll2410_notnull				@

	@ MPLLCON is 0.
	orr	r9, r9, #(1 << 4)				@ FCLK=XTPll (or extclk ) 

	@ Use CLKSLOW to set the clock to skip the PLL.

	str	r9, [ r6 ]					@ CLKSLOW = new value (e.g skip PLL). 
	bl	delay2ms					@ delay about 8ms (7812us). Right clockrate comes
	mov	r0, r3						@ from previous code. Will always use the lowest
	bl	delay2ms					@ clockrate for delay, so might take longer.
	mov	r0, r3						@
	bl	delay2ms					@
	mov	r0, r3						@
	bl	delay2ms					@
								@
	@ MPLLCON should now be changed. Turn off the MPLL.
	str	r2, [ r5 ]					@ MPLLCON=New MPLL value
	orr	r9, r9, #(1 << 5)				@ Turn off PLL.
	str	r9, [ r6 ]					@
	b	mpll2410_finish					@

	@ MPLLCON is not zero.
mpll2410_notnull:						@
	bic	r9, r9, #(1 << 5)				@ Set CLKSLOW=Start PLL. 

	@ MPLLCON should now be changed, then turn on the MPLL. Wait a short while
	@ before changing the clocksource, as it takes a while for the MPLL to lock.
	str	r2, [ r5 ]					@ MPLLCON=New MPLL value
	bl	delay2ms					@ delay about 2ms (1952us). Right clockrate comes
								@ from previous code. Will always use the lowest
								@ clockrate for delay, so might take longer.
								@
	mov	r0, r3						@ Set clockrate for delay.

	@ Now start the PLL. It has the right value.
	str	r9, [ r6 ]					@ Write CLKSLOW register.
	bl	delay2ms					@ delay about 2ms (1952us).
	mov	r0, r3						@ Set clockrate for delay.

	@ Lastly, switch over from the crystal to the MPLL.
	bic	r9, r9, #(1 << 4)				@ Set CLKSLOW=MPLL Input.
	str	r9, [ r6 ]					@ Change input.
	bl	delay2ms					@ delay about 2ms (1952us).

mpll2410_finish:
	@ Lastly, check if we went lower in frequency. If so, we can now set the dividers.
	tst	r4, #0x00000002					@ if( n->freq <= o->freq )
	streq	r1, [ r8 ]					@       set CLKDIVN (dividers)
	mov	r0, r3						@	Save lowest freq (o->freq or n->freq) in R0.

	@ Now that all dividers etc are set ok, set the refresh to the right value (it's refreshing WAAAAAY too fast at the moment.
	str	r12, [ r7 ]					@ Store ....
	mov	r0, r3						@ Get the lowest clock, and delay.... At least 2ms (1952us)
	bl	delay2ms					@ delay....

	@ Restore the interrupt flags (FIQ and IRQ) and the used regs.
	ldmfd	sp!, {r10}					@ Restore flags from stack.
	msr	cpsr_c, r10					@
	ldmfd	sp!, {r0-r10, r12, r14}				@ Restore used regs
	RETINSTR(mov,pc,lr)


@ R0 - MPLLCON, R1 - CLKDIVN, R2 - n->freq, R3 - o->freq, R4 - new_hclk, R5 - old_hclk
ENTRY( asm_s3c2412_freq_change )
	stmfd	sp!, {r0-r12, r14}				@ Save used regs.

	@ Save and disable interrupts (Both FIQ and IRQ).
	mrs	r9, cpsr					@ Save flags
	stmfd	sp!, {r9}					@
	orr	r9, r9, #(3 << 6)				@ Disable FIQ and IRQ
	msr	cpsr_c, r9					@

	@ Save MPLLCON before it gets destroyed.
	mov	r8, r0						@ Save MPLLCON in R8.

	@ Calculate the new refresh value.
	ldr	r7, =S3C2412_REFRESH				@ Load address of the REFRESH register.
	ldr	r12, [ r7 ]					@ Get the old refresh value.
	mov	r11, #0						@ High dword.
	sub	r10, r5, #1					@ Low dword - 1. (R11:10=oldhclk - 1)
	umlal	r10, r11, r12, r4				@ R11:R10=(old_hclk - 1) + R12 * newhclk
	mov	r12, r5						@ Set R12=old_hclk
	bl	asm_div_64					@ R0=R11:R10/R12 (=(R12 * new_hclk + old_hclk - 1)/old_hclk)
	mov	r12, r0						@ Set R12=New refresh.

	@ Compact the parameters. We only need to know whether or not n->freq > o->freq and new_hclk > old_hclk.
	mov	r10, #0						@ Set to 0.
	cmp	r5, r4						@ if( old_hclk < new_hclk )
	orrlo	r10, r10, #0x00000001				@ flag old_hclk < new_hclk bit.
	bichs	r10, r10, #0x00000001				@ clear old_hclk < new_hclk bit.
	cmp	r3, r2						@ if( o->freq < n->freq )
	orrlo	r10, r10, #0x00000002				@ flag o->freq < n->freq bit
	bichs	r10, r10, #0x00000002				@ clear o->freq < n->freq bit

	@ Remember, R3 is current frequency, R2 is new frequency
	mov	r4, r10						@ Save flags.
	mov	r11, r8						@ Put MPLLCON in the final reg.

	@ Set the RTC to do delays.
	bl	s3c2412_rtc_init_delay				@ Init RTC.
	mov	r3, r0						@ Save original state of the RTC. We can now use RTC to delay.
	
	@ Load the contents of the registers.
	ldr	r14, =S3C2410_CLKCON				@ Load CLKCON. Need to disable all but the memory clocks.
	ldr	r10, [ r14 ]					@ We leave the RTC enabled for our delay loops. LCD needs
								@ to be enabled, otherwise the system hangs during change.
	mov	r6, #((1 << 5) | (1 << 8) | (1 << 9) | (1 << 10))
								@ Only required clocks. 
	orr	r6, r6, #((1 << 23))				@
	str	r6, [ r14 ]					@ Store and disable.
	ldr	r5, =S3C2410_MPLLCON				@ Get the addresses of MPLLCON, CLKSRC, CLKDIVN.
	ldr	r6, =S3C2412_CLKSRC				@ 
	ldr	r8, =S3C2410_CLKDIVN				@
	ldr	r9, [ r6 ]					@ Load CLKSRC register

	@ Set highest refresh rate. Will always work, and we don't need memory right now anyway, so performance doesnt matter.
	mov	r0, #0x5D					@ Set highest refreshrate possible. Always works. Later we'll set
								@ the accurate one.
	str	r0, [ r7 ]					@
	mov	r0, #4						@ delay about 4ms.
	bl	s3c2412_rtc_delay_ms				@

	@ Now check if we go from a lower clock to a higher clock. If so
	@ we need to set the dividers before switching.		
	tst	r4, #0x00000002					@ if( n->freq > o->freq )
	strne	r1, [ r8 ]					@       set CLKDIVN (dividers)
	movne	r0, #4						@ delay about 4ms.
	blne	s3c2412_rtc_delay_ms				@

	@ Check if MPLLCON is 0. If so, we need to deal with it differently.
	cmp	r11, #0						@ if( MPLLCON == 0 )
	bne	mpll2412_notnull				@

	@ MPLLCON is 0.
	bic	r9, r9, #(1 << 4)				@ MSYSCLK=MDIVCLK
	orr	r11, r11, #(1 << 20)				@ Turn off MPLL

	@ Now set the right clocksource. Wait a short while before turning
	@ off the MPLL as the switch to the new clocksource takes longer
	@ than it takes the CPU to write to it!
	mov	r0, #8						@ delay about 16ms
	str	r9, [ r6 ]					@ CLKSRC=New MSYSCLK value (e.g MSYSCLK=MDIVCLK)
	bl	s3c2412_rtc_delay_ms				@
								@
	@ MPLLCON should now be changed. Turn off the MPLL.
	str	r11, [ r5 ]					@ MPLLCON=New MPLL value (e.g turn off)
	mov	r0, #4						@ delay about 4ms.
	bl	s3c2412_rtc_delay_ms				@
	b	mpll2412_finish					@

	@ MPLLCON is not zero.
mpll2412_notnull:
	@ First go 12 MHz. We can't use the MPLL once we're going to switch it.
	bic	r9, r9, #(1 << 4)				@ MSYSCLK=MDIVCLK
	mov	r0, #8						@ delay about 8ms
	str	r9, [ r6 ]					@ CLKSRC=New MSYSCLK value (e.g MSYSCLK=MDIVCLK)
	bl	s3c2412_rtc_delay_ms				@

	@ Set the bits for the actual switch.
	bic	r11, r11, #(1 << 20)				@ MPLLCON=Turn on MPLL
	orr	r9, r9, #(1 << 4)				@ CLKSRC=FOUTmpll

	@ MPLLCON should now be changed. Turn on the MPLL. Wait a short while
	@ before changing the clocksource, as it takes a while for the MPLL to lock.
	mov	r0, #4						@ Delay about 4ms.
	str	r11, [ r5 ]					@ MPLLCON=New MPLL value (e.g turn on)
								@ from previous code. Will always use the lowest
								@ clockrate for delay, so might take longer.
	bl	s3c2412_rtc_delay_ms				@

	@ Change the clocksource.
	str	r9, [ r6 ]					@ Write CLKSRC register.
	mov	r0, #8						@ delay about 8ms.
	bl	s3c2412_rtc_delay_ms				@

mpll2412_finish:
	@ Lastly, check if we went lower in frequency. If so, we can now set the dividers.
	tst	r4, #0x00000002					@ if( n->freq <= o->freq )
	streq	r1, [ r8 ]					@       set CLKDIVN (dividers)
	moveq	r0, #4						@ delay about 4ms.
	bleq	s3c2412_rtc_delay_ms				@

	@ Now that all dividers etc are set ok, set the refresh to the right value (it's refreshing WAAAAAY too fast at the moment.
	str	r12, [ r7 ]					@ Store ....
	mov	r0, #4						@ delay about 4ms.
	bl	s3c2412_rtc_delay_ms				@

	@ Now restore the clocks to all peripherals.
	ldr	r14, =S3C2410_CLKCON				@ Load CLKCON. Need to disable all but the memory clocks.
	str	r10, [ r14 ]					@ Restore CLKCON. Need to reenable all clocks again.

	@ We are done using the RTC for delay. Restore its state.
	mov	r0, r3						@ Saved state.
	bl	s3c2412_rtc_restore_delay			@ Restore...

	@ Restore the interrupt flags (FIQ and IRQ) and the used regs.
	ldmfd	sp!, {r10}					@ Restore flags from stack.
	msr	cpsr_c, r10					@
	ldmfd	sp!, {r0-r12, r14}				@ Restore used regs
	RETINSTR(mov,pc,lr)
