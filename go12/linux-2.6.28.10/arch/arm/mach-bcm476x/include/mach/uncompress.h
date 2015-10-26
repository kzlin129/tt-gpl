/*
 *
 *  Copyright (C) 1999 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * Map IO 1 to 1 with physical addresses.
 * Do not move this below the include of hardware.h
 */
#define HW_IO_PHYS_TO_VIRT(x)  (x)

#include <asm/arch/hardware.h>

#include <linux/serial_reg.h>   // For UART_LSR_TEMT constant

#define REG_UART_A_LSR  (*(volatile unsigned int *)(URT2_REG_BASE_ADDR + URT2_R_UARTFR_SEL))
#define REG_UART_A_FIFO (*(volatile unsigned int *)(URT2_REG_BASE_ADDR + URT2_R_UARTDR_SEL))

#ifdef UART_LSR_TEMT
 #undef UART_LSR_TEMT
#endif /* UART_LSR_TEMT */

#define UART_LSR_TEMT   URT2_F_TXFE_MASK


/*
 * This does not append a newline
 */
static inline void putc(int c)
{
   while (( REG_UART_A_LSR & UART_LSR_TEMT ) == 0 )
      barrier();

   REG_UART_A_FIFO = c;
}

static inline void flush(void)
{
   while (( REG_UART_A_LSR & UART_LSR_TEMT ) == 0 )
      barrier();
}

#define arch_decomp_setup()
#define arch_decomp_wdog()
