/*
 * File:	sound/soc/codecs/adau1761.h
 * Based on:	wm8753.h by Liam Girdwood
 * Author:	John McCarty <john.mccarty@analog.com>
 *
 * Created:      Fri Jan 16 2009
 * Description:  register and program definitions for adau1761 signal-flow modes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __ADAU1761_H__
#define __ADAU1761_H__

/* adau1761_set_dai_sysclk clk_id */
#define ADAU1761_MCLK_ID	0
#define ADAU1761_BCLK_ID	0x33

/* ADAU1761 Memory */
#define ADAU_PROGRAM	0x0800
#define ADAU_PARAMS		0x0000
#define ADAU_PROGSIZE	0x0400
#define ADAU_PARASIZE	0x0400

/* ADAU1761 control registers */
#define ADAU_FIRSTREG	0x4000

#define ADAU_CLKCTRL 	0x4000
#define ADAU_PLLCTRL	0x4002
#define ADAU_MICCTRL	0x4008
#define ADAU_RECPWRM	0x4009
#define ADAU_RECMLC0	0x400A
#define ADAU_RECMLC1	0x400B
#define ADAU_RECMRC0	0x400C
#define ADAU_RECMRC1	0x400D
#define ADAU_RECVLCL	0x400E
#define ADAU_RECVLCR	0x400F

#define ADAU_RECMBIA	0x4010
#define ADAU_ALCCTR0	0x4011
#define ADAU_ALCCTR1	0x4012
#define ADAU_ALCCTR2	0x4013
#define ADAU_ALCCTR3	0x4014
#define ADAU_SPRTCT0	0x4015
#define ADAU_SPRTCT1	0x4016
#define ADAU_CONVCT0	0x4017
#define ADAU_CONVCT1	0x4018
#define ADAU_ADCCTL0	0x4019
#define ADAU_ADCCTL1	0x401A
#define ADAU_ADCCTL2	0x401B
#define ADAU_PLBMLC0	0x401C
#define ADAU_PLBMLC1	0x401D
#define ADAU_PLBMRC0	0x401E
#define ADAU_PLBMRC1	0x401F

#define ADAU_PLBMLLO	0x4020
#define ADAU_PLBMRLO	0x4021
#define ADAU_PLBLRMC	0x4022
#define ADAU_PLBHPVL	0x4023
#define ADAU_PLBHPVR	0x4024
#define ADAU_PLBLOVL	0x4025
#define ADAU_PLBLOVR	0x4026
#define ADAU_PLBMNOC	0x4027
#define ADAU_PLBCTRL	0x4028
#define ADAU_PLBPWRM	0x4029

#define ADAU_DACCTL0	0x402A
#define ADAU_DACCTL1	0x402B
#define ADAU_DACCTL2	0x402C
#define ADAU_SERPAD0	0x402D
#define ADAU_SERPAD1	0x402E
#define ADAU_COMPAD0	0x402F
#define ADAU_COMPAD1	0x4030
#define ADAU_MCLKPAD	0x4031

#define ADAU_DEJITTER	0x4036

/* DSP Core */
#define ADAU_DSP_CRC	0x40C0
#define ADAU_DSP_GPI	0x40C6
#define ADAU_NON_MOD	0x40E9
#define ADAU_WATCGDG	0x40D0
#define ADAU_DSP_SR0	0x40EA
#define ADAU_DSP_SR1	0x40EB

#define ADAU_INP_ROT	0x40F2
#define ADAU_OUT_ROT	0x40F3
#define ADAU_SER_DAT	0x40F4
#define ADAU_DSP_ENA	0x40F5
#define ADAU_DSP_RUN	0x40F6
#define ADAU_DSP_SLW	0x40F7
#define ADAU_SER_SRT	0x40F8
#define ADAU_CLK_EN0	0x40F9
#define ADAU_CLK_EN1	0x40FA

#define ADAU_LASTREG	0x40FA
#define ADAU_NUMCACHEREG	251
	
/* Register field definitions */
/* Clock Control */
#define CLKCTRL_SRC_MCLK	0x0
#define CLKCTRL_SRC_PLL		0x8
#define CLKCTRL_FRQ_256		0x0
#define CLKCTRL_FRQ_512		0x2
#define CLKCTRL_FRQ_768		0x4
#define CLKCTRL_FRQ_1024	0x6
#define CLKCTRL_DISABLE		0x0
#define CLKCTRL_ENABLE		0x1

/* PLL Control -- 6 bytes*/
/*Bytes 5-6*/
#define PLLCTRL_DEN_MSB		0x00
#define PLLCTRL_DEN_LSB		0x00
/*Bytes 3-4*/
#define PLLCTRL_NUM_MSB		0x00
#define PLLCTRL_NUM_LSB		0x00
/*Byte 2*/
#define PLLCTRL_INTPART_R2	0x10
#define PLLCTRL_INTPART_R3	0x18
#define PLLCTRL_INTPART_R4	0x20
#define PLLCTRL_INTPART_R5	0x28
#define PLLCTRL_INTPART_R6	0x30
#define PLLCTRL_INTPART_R7	0x38
#define PLLCTRL_INTPART_R8	0x40
#define PLLCTRL_INPUT_DIV1	0x00
#define PLLCTRL_INPUT_DIV2	0x02
#define PLLCTRL_INPUT_DIV3	0x04
#define PLLCTRL_INPUT_DIV4	0x06
#define PLLCTRL_TYPE_INT	0x0
#define PLLCTRL_TYPE_FRAC	0x1
/*Byte 1*/
#define PLLCTRL_DISABLE		0x0
#define PLLCTRL_ENABLE		0x1

/*ADC*/
#define ADCCTL_DISABLE_MASK	0xFC
#define ADCCTL_ENABLE_MASK	0x03

/*ALC*/
#define ALCCTR0_PGASLEW_MASK	0xC0
#define ALCCTR0_PGASLEW_BITS	0x06
#define ALCCTR0_ALCMAX_MASK		0x38
#define ALCCTR0_ALCMAX_BITS		0x03
#define ALCCTR0_ALCSEL_MASK		0x07
#define ALCCTR0_ALCSEL_BITS		0x00

#define ALCCTR3_NGTYP_MASK		0xC0
#define ALCCTR3_NGTYP_BITS		0x06
#define ALCCTR3_NGEN_MASK		0x20
#define ALCCTR3_NGEN_BITS		0x05
#define ALCCTR3_NGTHR_MASK		0x1F
#define ALCCTR3_NGTHR_BITS		0x00


/*MIC*/
#define RECMBIA_DISABLE		0x00
#define RECMBIA_ENABLE		0x01
#define RECMBIA_MBIEN_MASK		0x01
#define RECMBIA_MBIEN_BITS		0x00
#define RECMBIA_MBI_MASK		0x04
#define RECMBIA_MBI_BITS		0x02
#define RECMBIA_MPERF_MASK		0x08
#define RECMBIA_MPERF_BITS		0x03
#define RECVLC_DISABLE_MASK	0xFC
#define RECVLC_ENABLE_MASK	0x03
#define RECVLC_LDEN_MASK	0x01
#define RECVLC_LDEN_BITS	0x00
#define RECVLC_LDMUTE_MASK	0x02
#define RECVLC_LDMUTE_BITS	0x01
#define RECVLC_LDVOL_MASK	0xFC
#define RECVLC_LDVOL_BITS	0x02

#define RECMLC_LINPG_MASK	0x70
#define RECMLC_LINPG_BITS	0x04
#define RECMLC_LINNG_MASK	0x0E
#define RECMLC_LINNG_BITS	0x01
#define RECMLC_MX1EN_MASK	0x01
#define RECMLC_MX1EN_BITS	0x00
#define RECMLC_LDBOOST_MASK	0x18
#define RECMLC_LDBOOST_BITS	0x03
#define RECMLC_MX1AUXG_MASK	0x07
#define RECMLC_MX1AUXG_BITS	0x00
#define RECMLC_MIC_0DB		0x08
#define RECMLC_MIC_20DB		0x10
#define RECMLC_LINE_0DB		0x05

/* PWN MNGMNT */
#define RECPWRM_LOW_PWR		0x0E
#define PLBPWRM_LOW_PWR		0x5C
#define PLBCTRL_POP_LPWR	0x10
#define PLBCTRL_POP_OFF		0x06
#define PLBCTRL_POP_ON		0x00
#define CLK_EN1_MASTER		0x03
#define CLK_EN1_SLAVE		0x01
#define CLK_EN1_OFF		0x00
#define CLK_EN0_OFF		0x00
#define CLK_EN0_ON		0x5F
#define RECPWRM_RUN_PWR		0x00
#define PLBPWRM_RUN_PWR		0x03
#define DAPM_LINE_DEF		0xE6
#define DAPM_HP_DEF		0xE7
#define PLB_MUTE_MASK		0x03

/*playback output control*/
#define ADAU1761_VOLUME_MASK 0xFC
#define ADAU1761_VOLUME_BITS 0x2
#define ADAU1761_MUTE_MASK 0x02
#define ADAU1761_MUTE_BITS 0x1
#define ADAU1761_ADVOL_MASK 0xff

/* DEJITTER */
#define DEJITTER_BYPASS		0x00
#define DEJITTER_ACTIVE		0x03

struct adau1761_setup_data {
        unsigned short i2c_bus;
	unsigned short i2c_address;
};

extern struct snd_soc_dai adau1761_dai;
extern struct snd_soc_codec_device soc_codec_dev_adau1761;

#endif
