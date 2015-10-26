/* drivers/video/tomtom/screeninfo.h
 *
 * TomTom GO LCD Controller Frame Buffer Driver
 * based on skeletonfb.c, sa1100fb.c, tomtomgofb.c
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __DRIVERS_VIDEO_TOMTOM_SCREENINFO_H 
#define __DRIVERS_VIDEO_TOMTOM_SCREENINFO_H
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
extern int lcd_def_freq_policy( struct notifier_block *nb, unsigned long val, void *data );
extern int lcd_def_freq_trans( struct notifier_block *nb, unsigned long val, void *data );
#endif

/* PNR Defines. */
#define PNR_4BIT_DUALSCAN_STN           0
#define PNR_4BIT_SINGLESCAN_STN         1
#define PNR_8BIT_SINGLESCAN_STN         2
#define PNR_TFT                         3
        
/* BPP Defines. */
#define BPP_1BPP_STN                    0
#define BPP_2BPP_STN                    1
#define BPP_4BPP_STN                    2
#define BPP_8BPP_STN                    3
#define BPP_12BPP_STN                   4
#define BPP_12BPP_STN_UNPACKED          5
#define BPP_16BPP_STN                   6
#define BPP_1BPP_TFT                    8
#define BPP_2BPP_TFT                    9
#define BPP_4BPP_TFT                    10
#define BPP_8BPP_TFT                    11
#define BPP_16BPP_TFT                   12 
#define BPP_24BPP_TFT                   13

struct lcd_screen_info
{
	unsigned char		manufacturer[4];	/* Manufacturer ID. */
	unsigned char		lcd[14];		/* Monitor name. */
	unsigned char		x_width;		/* X length of display (mm) */
	unsigned char		y_width;		/* Y length of display (mm) */
	unsigned short		x_res;			/* Xresolution */			
	unsigned short		y_res;			/* Yresolution */
	unsigned char		refresh_min;		/* Number of frames per second (min). */
	unsigned char		refresh_max;		/* Number of frames per second (max). */
	void			(*Init)(void);		/* Init routine. */
	void			(*Off)(void);		/* Off routine. */
	void			(*Screensave)(int on);	/* Screensave routine. */
	unsigned char		pnr_mode;		/* Display mode. See PNR_* defines. */
	unsigned char		bpp_mode;		/* Bits per pixel mode. See BPP_* defines. */
	unsigned long		curr_hclk;		/* Current HCLK value. */
	union
	{
		struct					/* Parameters for TFT displays only. */
		{
			unsigned short	vspw;		/* Vertical Sync Pulse Width */
			unsigned short	vbpd;		/* Vertical Back Porch */
			unsigned short	vfpd;		/* Vertical Front Porch */
			unsigned short	hspw;		/* Horizontal Sync Pulse Width */
			unsigned short	hbpd;		/* Horizontal Back Porch */
			unsigned short	hfpd;		/* Horizontal Front Porch */
		} tft;

		struct					/* Parameters for STN displays only. */
		{
			unsigned short	wlh;
			unsigned short	wdly;
			unsigned short	lineblank;
		} stn;
	} screen;

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	int			(*lcd_freq_policy_handler)( struct notifier_block *nb, unsigned long val, void *data );
							/* Frequency change policy handler routine. */
	int			(*lcd_freq_transition_handler)( struct notifier_block *nb, unsigned long val, void *data );
							/* Frequency change transition handler routine. */
	struct notifier_block	freq_policy;
	struct notifier_block	freq_transition;
#endif
};
#endif /* __DRIVERS_VIDEO_TOMTOM_SCREENINFO_H */
