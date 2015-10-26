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

/*
 * This does not append a newline
 */
static inline void putc(int c)
{
#if 0
   fwhile (( REG_UART_A_LSR & UART_LSR_TEMT ) == 0 )
   {
      barrier();
   }

   REG_UART_A_FIFO = c;
#endif
}

static inline void flush(void)
{
#if 0
   while (( REG_UART_A_LSR & UART_LSR_TEMT ) == 0 )
   {
      barrier();
	}
#endif
}

#define arch_decomp_setup()
#define arch_decomp_wdog()
