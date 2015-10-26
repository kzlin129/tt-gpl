/* linux/arch/arm/plat-s5p64xx/include/plat/irqs.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * S5P64XX - Common IRQ support
 */

#ifndef __ASM_PLAT_S5P64XX_IRQS_H
#define __ASM_PLAT_S5P64XX_IRQS_H __FILE__

/* we keep the first set of CPU IRQs out of the range of
 * the ISA space, so that the PC104 has them to itself
 * and we don't end up having to do horrible things to the
 * standard ISA drivers....
 *
 * note, since we're using the VICs, our start must be a
 * mulitple of 32 to allow the common code to work
 */

#define S3C_IRQ_OFFSET	(32)

#define S3C_IRQ(x)	((x) + S3C_IRQ_OFFSET)

#define S3C_VIC0_BASE	S3C_IRQ(0)
#define S3C_VIC1_BASE	S3C_IRQ(32)

/* UART interrupts, each UART has 4 intterupts per channel so
 * use the space between the ISA and S3C main interrupts. Note, these
 * are not in the same order as the S3C24XX series! */

#define IRQ_S3CUART_BASE0	(16)
#define IRQ_S3CUART_BASE1	(20)
#define IRQ_S3CUART_BASE2	(24)
#define IRQ_S3CUART_BASE3	(28)

#define UART_IRQ_RXD		(0)
#define UART_IRQ_ERR		(1)
#define UART_IRQ_TXD		(2)
#define UART_IRQ_MODEM		(3)

#define IRQ_S3CUART_RX0		(IRQ_S3CUART_BASE0 + UART_IRQ_RXD)
#define IRQ_S3CUART_TX0		(IRQ_S3CUART_BASE0 + UART_IRQ_TXD)
#define IRQ_S3CUART_ERR0	(IRQ_S3CUART_BASE0 + UART_IRQ_ERR)

#define IRQ_S3CUART_RX1		(IRQ_S3CUART_BASE1 + UART_IRQ_RXD)
#define IRQ_S3CUART_TX1		(IRQ_S3CUART_BASE1 + UART_IRQ_TXD)
#define IRQ_S3CUART_ERR1	(IRQ_S3CUART_BASE1 + UART_IRQ_ERR)

#define IRQ_S3CUART_RX2		(IRQ_S3CUART_BASE2 + UART_IRQ_RXD)
#define IRQ_S3CUART_TX2		(IRQ_S3CUART_BASE2 + UART_IRQ_TXD)
#define IRQ_S3CUART_ERR2	(IRQ_S3CUART_BASE2 + UART_IRQ_ERR)

#define IRQ_S3CUART_RX3		(IRQ_S3CUART_BASE3 + UART_IRQ_RXD)
#define IRQ_S3CUART_TX3		(IRQ_S3CUART_BASE3 + UART_IRQ_TXD)
#define IRQ_S3CUART_ERR3	(IRQ_S3CUART_BASE3 + UART_IRQ_ERR)

/* VIC based IRQs */

#define S5P64XX_IRQ_VIC0(x)	(S3C_VIC0_BASE + (x))
#define S5P64XX_IRQ_VIC1(x)	(S3C_VIC1_BASE + (x))

/* VIC0 */

#define IRQ_EINT0_3		S5P64XX_IRQ_VIC0(0)
#define IRQ_EINT4_11		S5P64XX_IRQ_VIC0(1)
#define IRQ_RTC_TIC		S5P64XX_IRQ_VIC0(2)
#define IRQ_IIC1		S5P64XX_IRQ_VIC0(5)
#define IRQ_IISV40		S5P64XX_IRQ_VIC0(6)
#define IRQ_GPS			S5P64XX_IRQ_VIC0(7)
#define IRQ_POST0		S5P64XX_IRQ_VIC0(9)
#define IRQ_2D			S5P64XX_IRQ_VIC0(11)
#define IRQ_TIMER0_VIC		S5P64XX_IRQ_VIC0(23)
#define IRQ_TIMER1_VIC		S5P64XX_IRQ_VIC0(24)
#define IRQ_TIMER2_VIC		S5P64XX_IRQ_VIC0(25)
#define IRQ_WDT			S5P64XX_IRQ_VIC0(26)
#define IRQ_TIMER3_VIC		S5P64XX_IRQ_VIC0(27)
#define IRQ_TIMER4_VIC		S5P64XX_IRQ_VIC0(28)
#define IRQ_DISPCON0		S5P64XX_IRQ_VIC0(29)
#define IRQ_DISPCON1		S5P64XX_IRQ_VIC0(30)
#define IRQ_DISPCON2		S5P64XX_IRQ_VIC0(31)

/* VIC1 */

#define IRQ_EINT12_15		S5P64XX_IRQ_VIC1(0)
#define IRQ_PCM0		S5P64XX_IRQ_VIC1(2)
#define IRQ_UART0		S5P64XX_IRQ_VIC1(5)
#define IRQ_UART1		S5P64XX_IRQ_VIC1(6)
#define IRQ_UART2		S5P64XX_IRQ_VIC1(7)
#define IRQ_UART3		S5P64XX_IRQ_VIC1(8)
#define IRQ_DMA0		S5P64XX_IRQ_VIC1(9)
#define IRQ_NFC			S5P64XX_IRQ_VIC1(13)
#define IRQ_SPI0		S5P64XX_IRQ_VIC1(16)
#define IRQ_SPI1		S5P64XX_IRQ_VIC1(17)
#define IRQ_IIC			S5P64XX_IRQ_VIC1(18)
#define IRQ_DISPCON3		S5P64XX_IRQ_VIC1(19)
#define IRQ_FIMGVG		S5P64XX_IRQ_VIC1(20)
#define IRQ_EINT_GROUPS		S5P64XX_IRQ_VIC1(21)
#define IRQ_PMUIRQ		S5P64XX_IRQ_VIC1(23)
#define IRQ_HSMMC0		S5P64XX_IRQ_VIC1(24)
#define IRQ_HSMMC1		S5P64XX_IRQ_VIC1(25)
#define IRQ_HSMMC2		IRQ_SPI1	/* shared with SPI1 */
#define IRQ_OTG			S5P64XX_IRQ_VIC1(26)
#define IRQ_DSI			S5P64XX_IRQ_VIC1(27)
#define IRQ_RTC_ALARM		S5P64XX_IRQ_VIC1(28)
#define IRQ_TSI			S5P64XX_IRQ_VIC1(29)
#define IRQ_PENDN		S5P64XX_IRQ_VIC1(30)
#define IRQ_TC			IRQ_PENDN
#define IRQ_ADC			S5P64XX_IRQ_VIC1(31)

#define S5P64XX_TIMER_IRQ(x)	S3C_IRQ(64 + (x))

#define IRQ_TIMER0		S5P64XX_TIMER_IRQ(0)
#define IRQ_TIMER1		S5P64XX_TIMER_IRQ(1)
#define IRQ_TIMER2		S5P64XX_TIMER_IRQ(2)
#define IRQ_TIMER3		S5P64XX_TIMER_IRQ(3)
#define IRQ_TIMER4		S5P64XX_TIMER_IRQ(4)

/* Since the IRQ_EINT(x) are a linear mapping on current s5p64xx series
 * we just defined them as an IRQ_EINT(x) macro from S3C_IRQ_EINT_BASE
 * which we place after the pair of VICs. */

#define S3C_IRQ_EINT_BASE	S3C_IRQ(64+5)

#define S3C_EINT(x)		((x) + S3C_IRQ_EINT_BASE)
#define IRQ_EINT(x)		S3C_EINT(x)

/* Next the external interrupt groups. These are similar to the IRQ_EINT(x)
 * that they are sourced from the GPIO pins but with a different scheme for
 * priority and source indication.
 *
 * The IRQ_EINT(x) can be thought of as 'group 0' of the available GPIO
 * interrupts, but for historical reasons they are kept apart from these
 * next interrupts.
 *
 * Use IRQ_EINT_GROUP(group, offset) to get the number for use in the
 * machine specific support files.
 *
 * Basically, this is an ugly rip-off from 6410. Only groups 1,2,5,6 and 8
 * are present. Groups 1 and 8 are rather braindead, as they miss quite a few
 * lines (6 and 7 on group 1; 0, 1 and 2 on group 8). We fake them for
 * the sake of sanity, but beware...
 */

#define IRQ_EINT_GROUP1_NR	(15)
#define IRQ_EINT_GROUP2_NR	(8)
#define IRQ_EINT_GROUP5_NR	(7)
#define IRQ_EINT_GROUP6_NR	(10)
#define IRQ_EINT_GROUP8_NR	(11)

#define IRQ_EINT_GROUP_BASE	S3C_EINT(16)
#define IRQ_EINT_GROUP1_BASE	(IRQ_EINT_GROUP_BASE + 0x00)
#define IRQ_EINT_GROUP2_BASE	(IRQ_EINT_GROUP1_BASE + IRQ_EINT_GROUP1_NR)
#define IRQ_EINT_GROUP5_BASE	(IRQ_EINT_GROUP2_BASE + IRQ_EINT_GROUP2_NR)
#define IRQ_EINT_GROUP6_BASE	(IRQ_EINT_GROUP5_BASE + IRQ_EINT_GROUP5_NR)
#define IRQ_EINT_GROUP8_BASE	(IRQ_EINT_GROUP6_BASE + IRQ_EINT_GROUP6_NR)

#define IRQ_EINT_GROUP(group, no)	(IRQ_EINT_GROUP##group##_BASE + (no))

/* Set the default NR_IRQS */

#define NR_IRQS	(IRQ_EINT_GROUP8_BASE + IRQ_EINT_GROUP8_NR + 1)

#endif /* __ASM_PLAT_S5P64XX_IRQS_H */

