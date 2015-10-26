/* drivers/barcelona/sound/codec.h
 *
 * Declarations of codec-related types, variables and functions.
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DRIVERS_BARCELONA_SOUND_CODEC_H
#define __DRIVERS_BARCELONA_SOUND_CODEC_H

#include <linux/errno.h>

enum codec_fs {
	FS256 = 256,
	FS384 = 384,
};

struct codec_ops {
	int (*set_volume)(unsigned volume);
	int (*set_volume_internal)(unsigned volume);
	int (*set_volume_external)(unsigned volume);
	int (*set_register)(unsigned data);
	void (*suspend)(void);
	void (*resume)(void);
	int (*set_treble)(unsigned treble);
	int (*set_bass)(unsigned bass);
	int (*set_fs)(enum codec_fs fs, unsigned int samplerate, unsigned int clockrate);
	int (*set_volume_external_left)(unsigned volume);
	int (*set_volume_external_right)(unsigned volume);
	int (*switch_line_in)(unsigned line_in);
	int (*set_volume_dB)(unsigned volume);
};

extern struct codec_ops codec_ops;

void codec_init(void);

static inline int codec_set_volume_dB(unsigned volume)
{
	return codec_ops.set_volume_dB ?
		codec_ops.set_volume_dB(volume) : -EINVAL;
}

static inline int codec_set_volume(unsigned volume)
{
	return codec_ops.set_volume ?
		codec_ops.set_volume(volume) : -EINVAL;
}

static inline int codec_set_volume_internal(unsigned volume)
{
	return codec_ops.set_volume_internal ?
		codec_ops.set_volume_internal(volume) : -EINVAL;
}

static inline int codec_set_volume_external(unsigned volume)
{
	return codec_ops.set_volume_external ?
		codec_ops.set_volume_external(volume) : -EINVAL;
}

static inline int codec_set_register(unsigned data)
{
	return codec_ops.set_register ?
		codec_ops.set_register(data) : -EINVAL;
}

static inline void codec_suspend(void)
{
	return codec_ops.suspend ?
		codec_ops.suspend() : -EINVAL;
}

static inline void codec_resume(void)
{
	return codec_ops.resume ?
		codec_ops.resume() : -EINVAL;
}

static inline int codec_set_treble(unsigned treble)
{
	return codec_ops.set_treble ?
		codec_ops.set_treble(treble) : -EINVAL;
}

static inline int codec_set_bass(unsigned bass)
{
	return codec_ops.set_bass ?
		codec_ops.set_bass(bass) : -EINVAL;
}

static inline int codec_set_fs(enum codec_fs fs,unsigned int samplerate, unsigned int clockrate)
{
	return codec_ops.set_fs ?
		codec_ops.set_fs(fs,samplerate, clockrate) : -EINVAL;
}

static inline int codec_set_volume_external_left(unsigned volume)
{
	return codec_ops.set_volume_external_left ?
		codec_ops.set_volume_external_left(volume) : -EINVAL;
}

static inline int codec_set_volume_external_right(unsigned volume)
{
	return codec_ops.set_volume_external_right ?
		codec_ops.set_volume_external_right(volume) : -EINVAL;
}

static inline int codec_switch_line_in(unsigned line_in)
{
	return codec_ops.switch_line_in ?
		codec_ops.switch_line_in(line_in) : -EINVAL;
}
#endif /* __DRIVERS_BARCELONA_SOUND_CODEC_H */

/* EOF */
