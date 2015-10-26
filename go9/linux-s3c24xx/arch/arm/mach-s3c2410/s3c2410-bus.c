/* arch/arm/mach-s3c2410/s3c2410-bus.c
 *
 * Implementation for specifying bank settings like timing, bus width etc.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author: Mark Vels <Mark.Vels@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/types.h>
#include <asm/io.h>
#include <linux/errno.h>
#include <linux/module.h>

#include <asm/system.h>
#include <asm/arch/regs-mem.h>
#include "s3c2410-bus.h"

int s3c2410_write_bank_settings( const struct s3c2410_bank_settings *settings ){

	void* bankcon_addr;
	u16 bankcon_reg;
	u32 reg;
	u32 mask;
	u32 tmp;
	
	if( settings == NULL)
		return -EINVAL;
	
	if( settings->bank_nr < 1 && settings->bank_nr > 5) /* other nGCS BANKCON registers have a different mapping	*/
		return -EINVAL;

	/* Calculate address of apropriate bankcon register	*/
	bankcon_addr = S3C2410_MEMREG( settings->bank_nr*0x0004 + 0x0004 );
	
	bankcon_reg = (settings->timing.pmc & 0x03 ) | (( settings->timing.t_acp & 0x03 ) << 2) |
		(( settings->timing.t_cah & 0x03 ) << 4) | (( settings->timing.t_coh & 0x03 ) << 6) |
		(( settings->timing.t_acc & 0x07 ) << 8) | (( settings->timing.t_cos & 0x03 ) << 11) |
		(( settings->timing.t_acs & 0x03 ) << 13);
	__raw_writel( bankcon_reg, bankcon_addr );
	
	/* Determine values of BWSCON bits for this bank	*/
	reg = ((settings->bus_width & 0x03) | (( settings->wait_status & 0x01) << 2));
	reg = reg << (settings->bank_nr * 4);
	mask = 0x07 << (settings->bank_nr * 4);
	 
	/* And write to S3C2410_BWSCON	*/
	local_irq_disable();
	tmp = __raw_readl( S3C2410_BWSCON);
	tmp &= ~( mask );
	tmp |= reg;
	__raw_writel( tmp, S3C2410_BWSCON );
	local_irq_enable();
	return 0;
}

EXPORT_SYMBOL( s3c2410_write_bank_settings ); 
