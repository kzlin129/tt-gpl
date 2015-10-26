/* arch/arm/mach-s3c2410/s3c2410-bus.h
 *
 * Definitions of function for specifying bank settings.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author: Mark Vels <Mark.Vels@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __S3C2410_BUS_H__
#define __S3C2410_BUS_H__

enum { 
	BUS_WIDTH_8 = 0, 
	BUS_WIDTH_16, 
	BUS_WIDTH_32
};

enum{ 
	WAIT_DISABLE = 0, 
	WAIT_ENABLE 
};

enum { 
	T_ACS_CLKS_0 = 0,
	T_ACS_CLKS_1,
	T_ACS_CLKS_2,
	T_ACS_CLKS_4
};

enum { 
	T_COS_CLKS_0 = 0,
	T_COS_CLKS_1,
	T_COS_CLKS_2,
	T_COS_CLKS_4
};

enum{
	T_ACC_CLKS_1	= 0,
	T_ACC_CLKS_2,
	T_ACC_CLKS_3,
	T_ACC_CLKS_4,
	T_ACC_CLKS_6,
	T_ACC_CLKS_8,
	T_ACC_CLKS_10,
	T_ACC_CLKS_14
};

enum { 
	T_COH_CLKS_0 = 0,
	T_COH_CLKS_1,
	T_COH_CLKS_2,
	T_COH_CLKS_4
};

enum { 
	T_CAH_CLKS_0 = 0,
	T_CAH_CLKS_1,
	T_CAH_CLKS_2,
	T_CAH_CLKS_4
};

enum { 
	T_ACP_CLKS_2 = 0,
	T_ACP_CLKS_3,
	T_ACP_CLKS_4,
	T_ACP_CLKS_6
};

enum {
	PMC_NORMAL = 0,
	PMC_DATA_4,
	PMC_DATA_8,
	PMC_DATA_16,
};


struct bus_timing {
	unsigned char t_acs;		/* Address set-up time before nGCSn	*/
	unsigned char t_cos;		/* Chip selection set-up time before nOE	*/
	unsigned char t_acc;		/* Access cycle	*/
	unsigned char t_coh;		/* Chip selection hold time after nOE */
	unsigned char t_cah;		/* Address hold time after nGCSn */
	unsigned char t_acp; 		/* Page mode access cycle @ Page mode	*/
	unsigned char pmc;			/* Page mode configuration */
};

struct s3c2410_bank_settings {
	unsigned char				bank_nr; 	/* Bank 1 to bank 7, don't use for bank 0	*/
	unsigned char				bus_width;
	unsigned char 			wait_status;
	struct bus_timing 	timing;
};

extern int s3c2410_write_bank_settings( const struct s3c2410_bank_settings *settings );

#endif /* S3C2410_BUS_H__ */

