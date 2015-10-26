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




/**
*
*  @file    resampler.c
*
*  @brief   C model for the resampler.
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */
#include "halmixer_resamp.h"
/* ---- Public Variables ------------------------------------------------- */
short decim32to16fir[DECIM32TO16TAPLEN] =
{
    27,	   -39,	
   -51,	    43,	
    86,	   -35,	
  -133,	     6,	
   188,	    61,	
  -235,	  -178,	
   229,	   301,	
  -195,	  -463,	
    76,	   619,	
   143,	  -719,	
  -462,	   703,	
   853,	  -507,	
 -1249,	    83,	
  1544,	   585,	
 -1592,	 -1450,	
  1220,	  2363,	
  -246,	 -3021,	
 -1446,	  2867,	
  3733,	  -977,	
 -5709,	 -3894,	
  3975,	 11126,	
 12245,	  8200,	
  3387,	   700,	
};

short decim48to16fir[DECIM48TO16TAPLEN] =
{
    -8,	   -14,	    -7,	
    12,	    27,	    18,	
   -15,	   -46,	   -38,	
    13,	    70,	    74,	
     3,	   -95,	  -131,	
   -47,	   109,	   209,	
   141,	   -74,	  -272,	
  -263,	    -6,	   319,	
   429,	   174,	  -296,	
  -603,	  -438,	   147,	
   719,	   775,	   172,	
  -689,	 -1118,	  -677,	
   408,	  1339,	  1325,	
   225,	 -1243,	 -1968,	
 -1262,	   553,	  2270,	
  2595,	  1065,	 -1543,	
 -3636,	 -3768,	 -1524,	
  2252,	  5973,	  8199,	
  8335,	  6761,	  4436,	
  2305,	   884,	   203,
};

short resamp48to32fir[RESAMP48TO32INTPR * RESAMP48TO32FILTLEN] =
{
     4,	    71,	  -139,	   113,	    59,	  -305,	   386,	   -59,	  -468,	   818,	  -480,	  -540,	  1458,	 -1225,	  -414,	  2257,	 -2346,	  -157,	  3455,	 -3913,	  -458,	  6291,	 -5490,	 -6102,	 13689,	 18627,	  7105,	   552,	
   -53,	    80,	   -42,	   -76,	   214,	  -226,	   -12,	   413,	  -558,	   237,	   524,	 -1119,	   807,	   504,	 -1834,	  1736,	   284,	 -2768,	  3067,	   134,	 -4505,	  4847,	  1785,	 -9460,	  3391,	 19411,	 13315,	  2651,	
};

short resamp48to40fir[RESAMP48TO40INTPR * RESAMP48TO40FILTLEN] =
{
    21,	   -52,	    56,	    15,	  -168,	   302,	  -201,	  -286,	   811,	  -803,	  -173,	  1613,	 -2033,	   268,	  2797,	 -4030,	   658,	  5604,	 -7047,	 -3757,	 16598,	 17365,	  5041,	   164,	
    25,	   -40,	    18,	    68,	  -190,	   224,	    -8,	  -467,	   750,	  -386,	  -720,	  1739,	 -1334,	  -900,	  3294,	 -2904,	 -1459,	  6291,	 -4460,	 -6874,	 13258,	 19168,	  7199,	   486,	
    24,	   -23,	   -17,	    99,	  -170,	   113,	   162,	  -541,	   556,	    58,	 -1081,	  1538,	  -457,	 -1816,	  3184,	 -1377,	 -3174,	  5955,	 -1475,	 -8833,	  9165,	 20150,	  9689,	  1064,	
    20,	    -5,	   -42,	   105,	  -119,	    -4,	   278,	  -505,	   278,	   445,	 -1209,	  1079,	   426,	 -2336,	  2530,	   266,	 -4237,	  4720,	  1460,	 -9479,	  4678,	 20109,	 12360,	  1979,	
    13,	    10,	   -53,	    87,	   -52,	  -104,	   325,	  -381,	   -22,	   708,	 -1101,	   466,	  1156,	 -2400,	  1488,	  1736,	 -4520,	  2840,	  3930,	 -8826,	   225,	 18924,	 15005,	  3294,	
};

short resamp8to16fir[RESAMP8TO16INTPR * RESAMP8TO16FILTLEN] = 
{
   -234,	   386,	  -573,	   681,	  -745,	   610,	  -173,	  -643,	  1830,	 -3258,	  4600,	 -5257,	  4227,	    16,	 -9270,	 21309,	 17600,	  1588,
   -122,	   111,	   -24,	  -207,	   512,	  -979,	  1538,	 -2060,	  2344,	 -2127,	  1121,	   929,	 -4095,	  7943,	-10566,	  5685,	 25201,	  7495,
};

short resamp32to40fir[RESAMP32TO40INTPR * RESAMP32TO40FILTLEN] = 
{
    -2,	    30,	   -95,	   210,	  -381,	   615,	  -903,	  1178,	 -1512,	  1849,	 -2186,	  2543,	 -2986,	  3677,	 -4969,	  7665,	-13584,	 25216,	 16067,	   322,	
    26,	   -26,	     2,	    68,	  -202,	   418,	  -719,	  1069,	 -1554,	  2118,	 -2766,	  3511,	 -4399,	  5546,	 -7184,	  9754,	-13861,	 17944,	 21792,	  1224,	
    37,	   -59,	    80,	   -78,	    32,	    81,	  -273,	   536,	  -969,	  1519,	 -2203,	  3028,	 -4013,	  5199,	 -6657,	  8447,	-10229,	  8545,	 26687,	  3080,	
    32,	   -61,	   108,	  -165,	   214,	  -237,	   219,	  -153,	   -65,	   381,	  -829,	  1415,	 -2131,	  2954,	 -3812,	  4476,	 -4074,	 -1140,	 29499,	  6189,	
    14,	   -37,	    85,	  -165,	   275,	  -407,	   553,	  -701,	   760,	  -787,	   735,	  -599,	   398,	  -195,	   140,	  -603,	  2576,	 -9075,	 29176,	 10619,	
};

/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Function Prototypes -------------------------------------- */
/* ---- Functions -------------------------------------------------------- */


long saturate32( long long num )
{
   long sum = (long)num;
   if ( num > 0x07fffffff )
   {
      //printf( "Saturated MAX_SINT32.\n" );
      sum = 0x7fffffff;
   }
   else if ( num < (-2147483647L - 1) )
   {
      //printf( "Saturated MIN_SINT32.\n" );
      sum = 0x80000000;
   }
   return sum;
}

void mixerResample
(
   short *insamp,
   short *outsamp,
   short  numsamp,
   short *filtcoeff,
   unsigned short filtlen,
   unsigned short interpfac,
   unsigned short decimfac
)
{
   int i;
   int samptoskip;
   int interppos = 0;
   short *filt;


   /* adjust sample pointer to start at the beginning of history so the filtering 
    * can be done in forward traversal */
   insamp = insamp - filtlen + 1;

   for( i=0; i < numsamp; i++ )
   {
      short * sampptr;
      long long sum;
      long satnum;
      int j;

      filt = &filtcoeff[interppos*filtlen];
      sampptr = insamp;
      sum = 0x08000L;
      for( j = 0; j < filtlen; j++ )
      {
         /* do filter in forward traversal */
         sum += (long)(filt[j]) * (long)(*sampptr++) * 2;
      }

      satnum = saturate32( sum );
      *outsamp = (short)(satnum >> 16);
      outsamp++;
      interppos += decimfac;
      samptoskip = interppos / interpfac;
      interppos %= interpfac;
      insamp += samptoskip;
   }
}
