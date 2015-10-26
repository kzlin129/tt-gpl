/* drivers/barcelona/sound/codec.c
 *
 * Definitions of codec-related types, variables and functions.
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include <barcelona/debug.h>
#include "codec.h"
#include "wm8750.h"
#include "wm8711.h"
#include "wm8971.h"
#include "alc5628.h"

#define PFX "codec: "
#define PK_DBG PK_DBG_FUNC

struct codec_ops codec_ops;

void codec_init(void)
{
	switch (IO_GetCodecType()) {
	case GOCODEC_WM8711:
		wm8711_init(&codec_ops);
		break;
	case GOCODEC_WM8750:
		wm8750_init(&codec_ops);
		break;
	case GOCODEC_WM8971:
		wm8971_init(&codec_ops);
		break;
	default:
		PK_WARN("codec_init: unknown codec\n");
#ifdef CONFIG_BARCELONA_SOUND_ALC5628
	case GOCODEC_ALC5628:
#endif
		break;
	}
}

void codec_init_post_iis_hw_init(void)
{
	switch (IO_GetCodecType()) {
#ifdef CONFIG_BARCELONA_SOUND_ALC5628
	case GOCODEC_ALC5628:
		alc5628_init(&codec_ops);
		break;
#endif
	default:
		PK_WARN("codec_init: unknown codec\n");
	case GOCODEC_WM8711:
	case GOCODEC_WM8750:
	case GOCODEC_WM8971:
		break;
	}
}

/* EOF */
