/* linux/drivers/video/samsung/s3cfb2_lms430wqv.c
 *
 * Samsung lms430wqv Display Panel Support
 *
 * Marc Zyngier <marc.zyngier@tomtom.com>:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include "s3cfb2.h"

static struct s3cfb_lcd lms430wqv = {
	.width = 480,
	.height = 272,
	.bpp = 24,
	.freq = 30,

	.timing = {
		.h_fp = 8,
		.h_bp = 45,
		.h_sw = 41,
		.v_fp = 7,
		.v_fpe = 1,
		.v_bp = 11,
		.v_bpe = 1,
		.v_sw = 1,
	},

	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

/* name should be fixed as 's3cfb_set_lcd_info' */
void s3cfb_set_lcd_info(struct s3cfb_global *ctrl)
{
	ctrl->lcd = &lms430wqv;
}

