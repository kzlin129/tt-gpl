/*
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef _S5M8751_H
#define _S5M8751_H

#define S5M8751_SYSCLK 0
#define S5M8751_MCLK   1
#define S5M8751_BCLK   2

#define MUTE_OFF  0
#define MUTE_ON	  1

extern struct snd_soc_dai s5m8751_dai;
extern struct snd_soc_codec_device soc_codec_dev_s5m8751;

#endif /* _S5M8751_H */
