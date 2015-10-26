/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/



#if !defined( HALMIXER_RESAMPLER_H )
#define HALMIXER_RESAMPLER_H

#include <linux/broadcom/halaudio_mixer.h>

#define DECIM32TO16TAPLEN  46
#define DECIM48TO16TAPLEN  60
#define RESAMP48TO32INTPR  2
#define RESAMP48TO32FILTLEN 28

#define RESAMP48TO40INTPR  5
#define RESAMP48TO40FILTLEN 24

#define RESAMP8TO16INTPR   2
#define RESAMP8TO16FILTLEN 18

#define RESAMP32TO40INTPR     5
#define RESAMP32TO40FILTLEN   20

extern short decim32to16fir[DECIM32TO16TAPLEN];
extern short decim48to16fir[DECIM48TO16TAPLEN];

extern short resamp48to32fir[RESAMP48TO32INTPR * RESAMP48TO32FILTLEN];

extern short resamp48to40fir[RESAMP48TO40INTPR * RESAMP48TO40FILTLEN];

extern short resamp8to16fir[RESAMP8TO16INTPR * RESAMP8TO16FILTLEN];

extern short resamp32to40fir[RESAMP32TO40INTPR * RESAMP32TO40FILTLEN];

extern RESAMPFUNC gResampleFunc;

void mixerResample
(
   short *insamp,
   short *outsamp,
   short  numsamp,
   short *filtcoeff,
   unsigned short filtlen,
   unsigned short interpfac,
   unsigned short decimfac
);

#endif
