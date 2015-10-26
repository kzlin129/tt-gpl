/* drivers/video/tomtom/framerate.c
 *
 * LCD screen support functions.
 *
 * Copyright (C) 2005,2007 TomTom BV <http://www.tomtom.com/>
 * Author: Rogier Stam <rogier.stam@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef __KERNEL__
#include <barcelona/gopins.h>
#include <linux/err.h>
#include <asm/hardware/clock.h>
#include <barcelona/gopins.h>
#include <linux/err.h>
#include <linux/cpufreq.h>
#endif /* __KERNEL */
#ifdef __BOOTLOADER__
#include "timer.h"
#include "gopins.h"
#include "cpu.h"
#endif /* __BOOTLOADER__ */
#include "lcdregs.h"
#include "screeninfo.h"
#include "framerate.h"

#if defined CONFIG_CPU_S3C2410 || defined CONFIG_CPU_S3C2442 || defined CONFIG_CPU_S3C2440 || defined CONFIG_CPU_S3C2443 || defined __BOOTLOADER__
/* Calculate the HOZVAL and LINEVAL values. */
static void s3c2410_get_hozlineval( struct lcd_screen_info *lcdinfo, unsigned long int *hozval,
					   unsigned long int *lineval )
{
	/* If this is not TFT, check if we use color. If so, we need to set the correct hozval. */
	if( lcdinfo->pnr_mode != PNR_TFT )
	{
		/* Determine the horizontal number of pixels because of color (STN only). */
		if( (lcdinfo->bpp_mode == BPP_8BPP_STN) || (lcdinfo->bpp_mode == BPP_12BPP_STN) ||
		    (lcdinfo->bpp_mode == BPP_12BPP_STN_UNPACKED) )
			*hozval=lcdinfo->x_res * 3;
		else
			*hozval=lcdinfo->x_res;
	}
		
	/* Check what kind of display this is. This determines what kind of nominator we get. */
	switch( lcdinfo->pnr_mode )
	{
		case PNR_4BIT_DUALSCAN_STN :
			*hozval/=4;
			*lineval=lcdinfo->y_res/2;
			break;
		case PNR_4BIT_SINGLESCAN_STN :
			*hozval/=4;
			*lineval=lcdinfo->y_res;
			break;
		case PNR_8BIT_SINGLESCAN_STN :
			*hozval/=8;
			*lineval=lcdinfo->y_res;
			break;
		case PNR_TFT :
			/* No need to modify hozval. */
			*hozval=lcdinfo->x_res;
			*lineval=lcdinfo->y_res;
			break;
	}

	/* Last correction. */
	*hozval-=1;
	*lineval-=1;
	return;
}

static unsigned short calc_framerate_s3c2410_stn( struct lcd_screen_info *lcdinfo, unsigned long hclk, unsigned short clkval )
{
	unsigned short	framerate;
	unsigned long	nom;
	unsigned long	hozval;
	unsigned long	lineval;

	/* Determine the hoz and line values. */	
	s3c2410_get_hozlineval( lcdinfo, &hozval, &lineval );

	/* Calculate the nominator. */
	nom=((unsigned long) clkval) * 2 * (hozval + 1);
	nom+=(1 << (4 + lcdinfo->screen.stn.wlh)) + (1 << (4 + lcdinfo->screen.stn.wdly));
	nom+=lcdinfo->screen.stn.lineblank * 8;
	nom*=lineval;

	/* Calculate the framerate. */
	framerate=((unsigned short) (hclk/nom));
	return framerate;
}

static unsigned long int calc_hclk_s3c2410_stn( struct lcd_screen_info *lcdinfo, unsigned short clkval, unsigned short framerate )
{
	unsigned long	hclk;
	unsigned long	nom;
	unsigned long	hozval;
	unsigned long	lineval;

	/* Determine the hoz and line values. */	
	s3c2410_get_hozlineval( lcdinfo, &hozval, &lineval );

	/* Calculate the nominator. */
	nom=((unsigned long) framerate) * (lineval + 1);
	nom*=((unsigned long) clkval) * 2 * (hozval + 1) +
	     (1 << (4 + lcdinfo->screen.stn.wlh)) +
	     (1 << (4 + lcdinfo->screen.stn.wdly)) +
	     (lcdinfo->screen.stn.lineblank * 8);

	/* Calculate the HCLK. */
	hclk=((unsigned long) framerate) * nom;
	return hclk;
}

static unsigned short calc_clkval_s3c2410_stn( struct lcd_screen_info *lcdinfo, unsigned short framerate, unsigned long hclk )
{
	unsigned short	clkval;
	unsigned long	nom;
	unsigned long	hozval;
	unsigned long	lineval;

	/* Determine the hoz and line values. */	
	s3c2410_get_hozlineval( lcdinfo, &hozval, &lineval );

	/* Calculate the nominator. */
	nom=(1 << (3 + lcdinfo->screen.stn.wlh)) + (1 << (3 + lcdinfo->screen.stn.wdly)) + (lcdinfo->screen.stn.lineblank * 4);
	nom/=(hozval+1);

	/* Calculate the CLKVAL. */
	clkval=(unsigned short) ((hclk/(2 * ((unsigned long) framerate) * (hozval+1) * (lineval+1))) - nom);
	return clkval;
}


/* Calculate the nominator of the fraction to be multiplied with 1/VCLK to get the framerate. */
static unsigned long int calc_nom_s3c2410_tft( struct lcd_screen_info *lcdinfo )
{
	unsigned long int	nom;
	unsigned long int	hozval;
	unsigned long int	lineval;

	/* Determine the hoz and line values. */	
	s3c2410_get_hozlineval( lcdinfo, &hozval, &lineval );

	/* Determine NOM. */
	nom=((lcdinfo->screen.tft.vspw + 1) + (lcdinfo->screen.tft.vbpd + 1) +
	     (lcdinfo->screen.tft.vfpd + 1) + (lineval + 1)) *
	    ((lcdinfo->screen.tft.hspw + 1) + (lcdinfo->screen.tft.hbpd + 1) +
	     (lcdinfo->screen.tft.hfpd + 1) + (hozval + 1));
	nom*=2;

	/* Calculated NOM. */
	return nom;
}

static unsigned short calc_framerate_s3c2410_tft( struct lcd_screen_info *lcdinfo, unsigned long hclk, unsigned short clkval )
{
	unsigned long int	nom=calc_nom_s3c2410_tft( lcdinfo );

	/* Calculate the framerate. */
	return hclk/((((unsigned long) clkval) + 1) * nom);
}

static unsigned long int calc_hclk_s3c2410_tft( struct lcd_screen_info *lcdinfo, unsigned short clkval, unsigned short framerate )
{
	unsigned long int	nom=calc_nom_s3c2410_tft( lcdinfo );

	/* Calculate the HCLK freq. */
	return (((unsigned long) framerate) * nom * ((unsigned long) clkval));
}

static unsigned short calc_clkval_s3c2410_tft( struct lcd_screen_info *lcdinfo, unsigned short framerate, unsigned long hclk )
{
	unsigned long int	nom=calc_nom_s3c2410_tft( lcdinfo );
	unsigned long int	divider=nom*((unsigned long int) framerate);

	/* Calculate the CLKVAL. */
	return (((unsigned short) ((hclk + (divider - 1))/divider)) - 1);
}

static unsigned short calc_framerate_s3c2410( struct lcd_screen_info *lcdinfo, unsigned long hclk, unsigned short clkval )
{
	if( lcdinfo->pnr_mode == PNR_TFT )
		return calc_framerate_s3c2410_tft( lcdinfo, hclk, clkval );
	else
		return calc_framerate_s3c2410_stn( lcdinfo, hclk, clkval );
}

static unsigned long int calc_hclk_s3c2410( struct lcd_screen_info *lcdinfo, unsigned short clkval, unsigned short framerate )
{
	if( lcdinfo->pnr_mode == PNR_TFT )
		return calc_hclk_s3c2410_tft( lcdinfo, clkval, framerate );
	else
		return calc_hclk_s3c2410_stn( lcdinfo, clkval, framerate );
}

static unsigned short calc_clkval_s3c2410( struct lcd_screen_info *lcdinfo, unsigned short framerate, unsigned long hclk )
{
	if( lcdinfo->pnr_mode == PNR_TFT )
		return calc_clkval_s3c2410_tft( lcdinfo, framerate, hclk );
	else
		return calc_clkval_s3c2410_stn( lcdinfo, framerate, hclk );
}
#endif

#if defined  CONFIG_CPU_S3C2412 || defined CONFIG_CPU_S3C2413 || defined __BOOTLOADER__
static void s3c2412_get_hozlineval( struct lcd_screen_info *lcdinfo, unsigned long int *hozval,
					   unsigned long int *lineval )
{
	/* Check whether this is a TFT screen or STN. */
	if( lcdinfo->pnr_mode != PNR_TFT )
	{
		*hozval=lcdinfo->x_res;
		*lineval=lcdinfo->y_res;
	}
	else
	{
		*hozval=lcdinfo->x_res - 1;
		*lineval=lcdinfo->y_res - 1;
	}

	return;
}

/* Calculate the nominator of the fraction to be multiplied with 1/VCLK to get the framerate. */
static unsigned long int calc_nom_s3c2412( struct lcd_screen_info *lcdinfo )
{
	unsigned long int	nom;
	unsigned long int	hozval;
	unsigned long int	lineval;

	/* Determine the HOZVAL and LINEVAL values. */
	s3c2412_get_hozlineval( lcdinfo, &hozval, &lineval );

	/* Calculate the nominator. */
	if( lcdinfo->pnr_mode != PNR_TFT )
	{
		/* STN mode. */
		nom=lineval * (lcdinfo->screen.stn.wlh + lcdinfo->screen.stn.wdly +
			       hozval + lcdinfo->screen.stn.lineblank);
	}
	else
	{
		/* TFT mode. */
		nom=((lcdinfo->screen.tft.vspw + 1) + (lcdinfo->screen.tft.vbpd + 1) +
		     (lcdinfo->screen.tft.vfpd + 1) + (lineval + 1)) *
		    ((lcdinfo->screen.tft.hspw + 1) + (lcdinfo->screen.tft.hbpd + 1) +
		     (lcdinfo->screen.tft.hfpd + 1) + (hozval + 1));
		nom*=2;
	}

	/* Done. */
	return nom;
}

static unsigned short calc_framerate_s3c2412( struct lcd_screen_info *lcdinfo, unsigned long hclk, unsigned short clkval )
{
	unsigned short		act_clkval=(lcdinfo->pnr_mode == PNR_TFT ? (clkval + 1) : (clkval));
	unsigned long int	nom=calc_nom_s3c2412( lcdinfo );

	return hclk/(((unsigned long int) act_clkval) * nom);
}

static unsigned long int calc_hclk_s3c2412( struct lcd_screen_info *lcdinfo, unsigned short clkval, unsigned short framerate )
{
	unsigned short		act_clkval=(lcdinfo->pnr_mode == PNR_TFT ? (clkval + 1) : (clkval));
	unsigned long int	nom=calc_nom_s3c2412( lcdinfo );

	return (((unsigned long) framerate) * nom * ((unsigned long int) act_clkval));
}

static unsigned short calc_clkval_s3c2412( struct lcd_screen_info *lcdinfo, unsigned short framerate, unsigned long hclk )
{
	unsigned long int	nom=calc_nom_s3c2412( lcdinfo );
	unsigned long int	divider=nom*((unsigned long int) framerate);
	unsigned short		clkval;

	/* Ensure the value gets rounded correctly .*/
	clkval=(unsigned short) ((hclk + (divider - 1))/divider);
	if( lcdinfo->pnr_mode == PNR_TFT )
		return clkval - 1;
	else
		return clkval;
}
#endif

void get_hozlineval( struct lcd_screen_info *lcdinfo, unsigned long int *hozval, unsigned long int *lineval )
{
	/* Depending on the CPU, the LCD framerate is determined differently. */
	switch( IO_GetCpuType() )
	{
		case GOCPU_S3C2412 :
		{
			/* S3C2412 or S3C2413 CPU. */
			s3c2412_get_hozlineval( lcdinfo, hozval, lineval );
			break;
		}

		default :
		{
			/* All other CPUs. */
			s3c2410_get_hozlineval( lcdinfo, hozval, lineval );
			break;
		}
	}
	return;
}

unsigned short get_clkval( struct lcd_screen_info *screeninfo, unsigned short framerate, unsigned long int hclk_rate )
{
	switch( IO_GetCpuType( ) )
	{
		case GOCPU_S3C2412 :
			return calc_clkval_s3c2412( screeninfo, framerate, hclk_rate );

		default :
			return calc_clkval_s3c2410( screeninfo, framerate, hclk_rate );
	}
}

unsigned short get_framerate( struct lcd_screen_info *screeninfo, unsigned short clkval, unsigned long int hclk_rate )
{
	switch( IO_GetCpuType( ) )
	{
		case GOCPU_S3C2412 :
			return calc_framerate_s3c2412( screeninfo, hclk_rate, clkval );

		default :
			return calc_framerate_s3c2410( screeninfo, hclk_rate, clkval );
	}
}

unsigned long get_hclkrate( struct lcd_screen_info *screeninfo, unsigned short clkval, unsigned short framerate )
{
	switch( IO_GetCpuType( ) )
	{
		case GOCPU_S3C2412 :
			return calc_hclk_s3c2412( screeninfo, clkval, framerate );

		default :
			return calc_hclk_s3c2410( screeninfo, clkval, framerate );
	}
}

/* Returns the actual set clock divider value. */
unsigned short set_clkval( struct lcd_screen_info *screeninfo, unsigned short clkval )
{
	unsigned long int	regval;

	/* Is this a legal value? */
	if( clkval > 1023 )
		clkval=1023;

	/* Set the CLK value only if the divider is within bounds. */
	regval=rLCDCON1 & 0xFFFC00FF;
	rLCDCON1=regval | (((unsigned long int) clkval) << 8);

	/* Return the actual divider calculated. */
	return clkval;
}

/* Returns the actual set framerate. */
unsigned short set_framerate( struct lcd_screen_info *screeninfo, unsigned short framerate, unsigned long hclk_rate )
{
	unsigned short		clkval;

	clkval=get_clkval( screeninfo, framerate, hclk_rate );
	return get_framerate( screeninfo, set_clkval( screeninfo, clkval ), hclk_rate );
}

unsigned long int get_hclkfreq( void )
{
	unsigned long int	curr_rate;
#ifdef __KERNEL__ 
	struct clk		*hclk;
 
	hclk=clk_get( NULL, "hclk" );
	if( IS_ERR( hclk ) ) 
		return 0;

	/* Get the current rate. */
	curr_rate=clk_get_rate( hclk );

	/* Done with the clock. */
	clk_put( hclk );
#else
	curr_rate=CPU_GetMemclk();
#endif
	return curr_rate;
}

