/*
 * Copyright 2009 Broadcom Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _BCM476X_CODEC_H
#define _BCM476X_CODEC_H
/* ---- Include Files ---------------------------------------------------- */
#include <asm/arch/bcm4760_reg.h>

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

#define REG_PCMRATE_ADDR            (AUD0_REG_BASE_ADDR + 0x0108)   /* PCM interface rate control register */
#define REG_AMCR_ADDR               (AUD0_REG_BASE_ADDR + 0x0100)   /* Audio module control register */
#define REG_VICTRL_ADDR             (AUD0_REG_BASE_ADDR + 0x0104)   /* Voice input path control register */
#define REG_AUXCOMP_ADDR            (AUD0_REG_BASE_ADDR + 0x010c)   /* AUX COMP, AUX_MIC_DET COMP1  (Read-Only) */
//#define REG_AACR_ADDR               (AUD0_REG_BASE_ADDR + 0x010c)   /* Audio analog interface control register  ???*/
#define	REG_AICTRL_ADDR  	        (AUD0_REG_BASE_ADDR + 0x0110)	/* Audio input paths control register */
//#define REG_AUDATT_ADDR             (AUD0_REG_BASE_ADDR + 0x0112)   /* DSP Audio attentuation register ?? Does not exist in BCM4760*/
//#define REG_AMPCR_ADDR              (AUD0_REG_BASE_ADDR + 0x0118)   /* Baseband analog modules power control register. Does not exist in BCM4760 */
#define	REG_AUDIOINCM_MEM_ADDR	    (AUD0_REG_BASE_ADDR + 0x011c)	/* Audio input compensation filter coefficient memory address */
#define	REG_AUDIOINCM_DATA_ADDR     (AUD0_REG_BASE_ADDR + 0x0120)	/* Audio input compensation filter coefficient data address */
#define	REG_ALOOPBACKCTRL_ADDR      (AUD0_REG_BASE_ADDR + 0x0124)	/* The DAC and ADC analog loopback control */

#define	REG_VCOEFR_ADDR	            (AUD0_REG_BASE_ADDR + 0x0340)	/* Voice IIR Coefficient registers */
#define  REG_VCOEFR0                (AUD0_REG_BASE_ADDR + 0x0340) // R/W // 
#define  REG_VCOEFR1                (AUD0_REG_BASE_ADDR + 0x0344) // R/W // 
#define  REG_VCOEFR2                (AUD0_REG_BASE_ADDR + 0x0348) // R/W // 
#define  REG_VCOEFR3                (AUD0_REG_BASE_ADDR + 0x034c) // R/W // 
#define  REG_VCOEFR4                (AUD0_REG_BASE_ADDR + 0x0350) // R/W // 
#define  REG_VCOEFR5                (AUD0_REG_BASE_ADDR + 0x0354) // R/W // 
#define  REG_VCOEFR6                (AUD0_REG_BASE_ADDR + 0x0358) // R/W // 
#define  REG_VCOEFR7                (AUD0_REG_BASE_ADDR + 0x035c) // R/W // 
#define  REG_VCOEFR8                (AUD0_REG_BASE_ADDR + 0x0360) // R/W // 
#define  REG_VCOEFR9                (AUD0_REG_BASE_ADDR + 0x0364) // R/W // 
#define  REG_VCOEFR10               (AUD0_REG_BASE_ADDR + 0x0368) // R/W // 
#define  REG_VCOEFR11               (AUD0_REG_BASE_ADDR + 0x036c) // R/W // 
#define  REG_VCOEFR12               (AUD0_REG_BASE_ADDR + 0x0370) // R/W // 
#define  REG_VCOEFR13               (AUD0_REG_BASE_ADDR + 0x0374) // R/W // 
#define  REG_VCOEFR14               (AUD0_REG_BASE_ADDR + 0x0378) // R/W // 
#define  REG_VCOEFR15               (AUD0_REG_BASE_ADDR + 0x037c) // R/W // 
#define  REG_VCOEFR16               (AUD0_REG_BASE_ADDR + 0x0380) // R/W // 
#define  REG_VCOEFR17               (AUD0_REG_BASE_ADDR + 0x0384) // R/W // 
#define  REG_VCOEFR18               (AUD0_REG_BASE_ADDR + 0x0388) // R/W // 
#define  REG_VCOEFR19               (AUD0_REG_BASE_ADDR + 0x038c) // R/W // 
#define  REG_VCOEFR20               (AUD0_REG_BASE_ADDR + 0x0390) // R/W // 
#define  REG_VCOEFR21               (AUD0_REG_BASE_ADDR + 0x0394) // R/W // 
#define  REG_VCOEFR22               (AUD0_REG_BASE_ADDR + 0x0398) // R/W // 
#define  REG_VCOEFR23               (AUD0_REG_BASE_ADDR + 0x039c) // R/W // 
#define  REG_VCOEFR24               (AUD0_REG_BASE_ADDR + 0x03a0) // R/W //
#define  REG_VCOEFR25               (AUD0_REG_BASE_ADDR + 0x03a4) // R/W // 
#define  REG_VCOEFR26               (AUD0_REG_BASE_ADDR + 0x03a8) // R/W // 
#define  REG_VCOEFR27               (AUD0_REG_BASE_ADDR + 0x03ac) // R/W // 
#define  REG_VCOEFR28               (AUD0_REG_BASE_ADDR + 0x03b0) // R/W // 
#define  REG_VCOEFR29               (AUD0_REG_BASE_ADDR + 0x03b4) // R/W // 
#define  REG_VCOEFR30               (AUD0_REG_BASE_ADDR + 0x03b8) // R/W // 
#define  REG_VCOEFR31               (AUD0_REG_BASE_ADDR + 0x03bc) // R/W // 
#define  REG_VCOEFR32               (AUD0_REG_BASE_ADDR + 0x03c0) // R/W // 
#define  REG_VCOEFR33               (AUD0_REG_BASE_ADDR + 0x03c4) // R/W // 
#define  REG_VCOEFR34               (AUD0_REG_BASE_ADDR + 0x03c8) // R/W //

#define	REG_AUDIR2_ADDR	            (AUD1_REG_BASE_ADDR + 0x0280)	/* The 2nd (right) channel voice sample input register */
#define	REG_AUDIOINFIFO_STATUS_ADDR (AUD1_REG_BASE_ADDR + 0x0284)	/* ADC path input FIFO status */
#define REG_VCFGR_ADDR              (AUD1_REG_BASE_ADDR + 0x028c)   /* Voice compensation filter gain (RO) */ 
#define   AUDVOC_VCFGR_REG           REG_VCFGR_ADDR
#define	REG_AUDIR_ADDR	            (AUD1_REG_BASE_ADDR + 0x0290)	/* AUDIR Voice input sample (RO) */
#define   AUDVOC_AUDIR_REG          REG_AUDIR_ADDR
#define	REG_AUDOR_ADDR	            (AUD1_REG_BASE_ADDR + 0x0294)	/* AUDOR Voice output sample */
#define   AUDVOC_AUDOR_REG          REG_AUDOR_ADDR
#define	REG_PCMDIR_ADDR	            (AUD1_REG_BASE_ADDR + 0x0298)	/* PCM Input sample register */
#define	REG_PCMDOR_ADDR	            (AUD1_REG_BASE_ADDR + 0x029c)	/* PCM Output sample register */
#define REG_VMUT_ADDR               (AUD1_REG_BASE_ADDR + 0x02a0)   /* Voice mute control register */
#define   AUDVOC_VMUT_REG           REG_VMUT_ADDR
#define REG_VFIFOCTRL_ADDR          (AUD1_REG_BASE_ADDR + 0x02a4)   /* Voice OFIFO control register */
#define   AUDVOC_VFIFOCTRL_REG      REG_VFIFOCTRL_ADDR
#define REG_VSLOPGAIN_ADDR          (AUD1_REG_BASE_ADDR + 0x02ac)   /* Voice channel slop gain control register */
#define   AUDVOC_VSLOPGAIN_REG      REG_VSLOPGAIN_ADDR
#define REG_VADCCTRL_ADDR          (AUD1_REG_BASE_ADDR + 0x02b0)   /* Voice ADC Control Register */
#define   AUDVOC_VADC_REG           REG_VADCCTRL_ADDR           /* This is mapped into the voice ADC control register */
#define REG_APCTRK_ADDR             (AUD1_REG_BASE_ADDR + 0x02b4)   /* Audio precision interface control register */
#define   AUDVOC_APCTRK_REG         REG_APCTRK_ADDR
#define REG_AIR_ADDR                (AUD1_REG_BASE_ADDR + 0x02b8)   /* AIR Analog interface register */
#define   AUDVOC_AIR_REG            REG_AIR_ADDR
#define REG_APRR_ADDR 	            (AUD1_REG_BASE_ADDR + 0x02bc)   /* APRR Analog power ramp register */  
#define   AUDVOC_APRR_REG           REG_APRR_ADDR

#define REG_PFIFODATA0_ADDR         (AUD1_REG_BASE_ADDR + 0x0304)  
#define REG_PFIFODATA1_ADDR		    (AUD1_REG_BASE_ADDR + 0x0308)
#define REG_PFIFODATA2_ADDR		    (AUD1_REG_BASE_ADDR + 0x030c)

#define REG_MIXER_IRR_COEFF_ADD_ADDR (AUD1_REG_BASE_ADDR + 0x0334)   /*  Mixer compensation filtering coefficients register */
#define REG_MIXER_IRR_COEFF_H_ADDR (AUD1_REG_BASE_ADDR + 0x0338)/* The high bits of write data to mixer compensation filtering coefficients */
#define REG_MIXER_IRR_COEFF_L_ADDR (AUD1_REG_BASE_ADDR + 0x033c)/* The low bits of write data to mixer compensation filtering coefficients */
#define REG_AUDMOD_ADDR 		    (AUD1_REG_BASE_ADDR + 0x0340)  /* Stereo Audio mode control register */
#define REG_AFIFOCTRL_ADDR          (AUD1_REG_BASE_ADDR + 0x0344)  /* Audio FIFO control register */
#define REG_AIFIFOST_ADDR           (AUD1_REG_BASE_ADDR + 0x0348)  /* Audio input FIFO status register */
#define REG_ALRCH_ADDR              (AUD1_REG_BASE_ADDR + 0x034c)  /* Audio left/right channel register */ 
#define REG_AEQPATHOFST0_ADDR       (AUD1_REG_BASE_ADDR + 0x0350)  /* Audio equalizer path delay */
#define REG_AEQPATHOFST1_ADDR       (AUD1_REG_BASE_ADDR + 0x0354)  /* Audio equalizer path delay */
#define REG_AEQPATHOFST2_ADDR       (AUD1_REG_BASE_ADDR + 0x0358)  /* Audio equalizer path delay */
#define REG_AEQEVT_ADDR    	        (AUD1_REG_BASE_ADDR + 0x035c)  /* Audio equalizer band envelope detection */
#define REG_ALSLOPGAIN_ADDR         (AUD1_REG_BASE_ADDR + 0x0360)  /* Audio left channel slope gain control */
#define REG_ARSLOPGAIN_ADDR         (AUD1_REG_BASE_ADDR + 0x0364)  /* Audio right channel slope gain control */
#define REG_BTNBDINL_ADDR    	    (AUD1_REG_BASE_ADDR + 0x0368)  /* Bluetooth taps narrow band left path audio samples */
#define REG_BTNBDINR_ADDR    	    (AUD1_REG_BASE_ADDR + 0x036c)  /* Bluetooth taps narrow band right path audio samples */
#define REG_BTMIXER_CFG1_ADDR	   	(AUD1_REG_BASE_ADDR + 0x0370)  /* BT mixer config 1 register */ 
#define REG_BTMIXER_CFG2_ADDR	   	(AUD1_REG_BASE_ADDR + 0x0374)  /* BT mixer config 2 register */
#define REG_BTMIXER_GAIN_L_ADDR	   	(AUD1_REG_BASE_ADDR + 0x0378)  /* BT mixer left channel gain config register */
#define REG_BTMIXER_GAIN_R_ADDR	   	(AUD1_REG_BASE_ADDR + 0x037c)  /* BT mixer right channel gain config register */
#define REG_AEQPATHGAIN1_ADDR       (AUD1_REG_BASE_ADDR + 0x0384)
#define REG_AEQPATHGAIN2_ADDR       (AUD1_REG_BASE_ADDR + 0x0388)
#define REG_AEQPATHGAIN3_ADDR       (AUD1_REG_BASE_ADDR + 0x038c)
#define REG_AEQPATHGAIN4_ADDR       (AUD1_REG_BASE_ADDR + 0x0390)
#define REG_AEQPATHGAIN5_ADDR       (AUD1_REG_BASE_ADDR + 0x0394)
#define REG_LSDMSEEDL_ADDR          (AUD1_REG_BASE_ADDR + 0x0398)
#define REG_LSDMSEEDH_ADDR          (AUD1_REG_BASE_ADDR + 0x03a0)
#define REG_LSDMPOLYLL_ADDR         (AUD1_REG_BASE_ADDR + 0x03a4)
#define REG_LSDMPOLYH_ADDR          (AUD1_REG_BASE_ADDR + 0x03a8)
#define REG_RSDMSEEDL_ADDR          (AUD1_REG_BASE_ADDR + 0x03ac)
#define REG_RSDMSEEDH_ADDR          (AUD1_REG_BASE_ADDR + 0x03b0)
#define REG_RSDMPOLYLL_ADDR         (AUD1_REG_BASE_ADDR + 0x03b4)
#define REG_RSDMPOLYH_ADDR          (AUD1_REG_BASE_ADDR + 0x03b8)
#define REG_SDMDTHER_ADDR           (AUD1_REG_BASE_ADDR + 0x03bc)  /* Sigma-Delta Dithering control register */
#define REG_AUDVOC_ISR_ADDR	    	(AUD1_REG_BASE_ADDR + 0x03c0)  /* Audio block interrupt status register */ 
#define REG_AIFIFODATA0_ADDR    	(AUD1_REG_BASE_ADDR + 0x03c4)  /* Audio input fifo/BT taps output fifo data0 */
#define REG_AIFIFODATA1_ADDR    	(AUD1_REG_BASE_ADDR + 0x03c8)  /* Audio input fifo/BT taps output fifo data1 */ 
#define REG_AIFIFODATA2_ADDR    	(AUD1_REG_BASE_ADDR + 0x03cc)  /* Audio input fifo/BT taps output fifo data2 */ 
#define REG_AEQCOFADD_ADDR    	    (AUD1_REG_BASE_ADDR + 0x03d0)  /* Audio equalizer coefficient memory address */ 
#define REG_AEQCOFDATA_ADDR    	    (AUD1_REG_BASE_ADDR + 0x03d4)  /* Audio equalizer coefficient memory data */  
#define REG_ACOMPFIRCOFADD_ADDR     (AUD1_REG_BASE_ADDR + 0x03d8)  /* Audio compensation FIR filter coefficient memory address */ 
#define REG_ACOMPFIRCOFDATA_ADDR    (AUD1_REG_BASE_ADDR + 0x03dc)  /* Audio compensation FIR filter coefficient memory data */  
#define REG_MIXER_INPUT_SEL_ADDR    (AUD1_REG_BASE_ADDR + 0x03e0)  /* The DAC mixer input channel selection register */
#define REG_MIXER_GAIN_CHSEL_ADDR   (AUD1_REG_BASE_ADDR + 0x03e4)  /* The DAC mixer compensation gain channel program selection register */
#define REG_MIXER_GAIN_ADJUST_ADDR  (AUD1_REG_BASE_ADDR + 0x03e8)  /* The mixer gain compensation gain register */
#define REG_MIXER_BIQUAD_CFG_ADDR   (AUD1_REG_BASE_ADDR + 0x03ec)  /* The MPMBIQUAD_CFG the mixer compensation filtering configuration register */
#define REG_MIXER_FORCEOFF_ADDR     (AUD1_REG_BASE_ADDR + 0x03f0)  /* The MPM_FORCEOFF force to turn off the mixer and post mixing processor */
#define REG_INDIRECT_ADDR           (AUD1_REG_BASE_ADDR + 0x03f4) // R/W //
#define REG_INDIRECT_DATA1          (AUD1_REG_BASE_ADDR + 0x03f8) // R/W //
#define REG_INDIRECT_DATA2          (AUD1_REG_BASE_ADDR + 0x03fc) // R/W //

#define REG_APRR_UPDWON_MASK        0x0080

/* AMPCR bit mask definitions */
#define REG_AMPCR_APCENB_MASK       (1<<15) /* GMSK APC DAC enable */
#define REG_AMPCR_VDAC_MASK         (1<<9)  /* Voiceband DAC */
#define REG_AMPCR_VADCP_MASK        (1<<8)  /* Voiceband ADC and PGA */
#define REG_AMPCR_RXADC_MASK        (1<<7)  /* Baseband RX ADC and PGA */
#define REG_AMPCR_APCPDAC_MASK      (1<<6)  /* 8PSK APC DAC */
#define REG_AMPCR_TXDAC_MASK        (1<<5)  /* Baseband TX DAC and Low Pass filter */
#define REG_AMPCR_AUXADC_MASK       (1<<4)  /* Auxiliary (battery monitor) ADC */
#define REG_AMPCR_APCDAC_MASK       (1<<2)  /* GMSK APC DAC */
#define REG_AMPCR_AFCDAC_MASK       (1<<1)  /* AFC DAC */
#define REG_AMPCR_BGAP_MASK         (1<<0)  /* Band gap reference circuit power down enable */

/* Register AMCR bit mask definitions */

#define REG_AMCR_AUDT_MASK          0x0003  /* Select Digital audio interface */
#define REG_AMCR_DIGLOOPBACK_MASK   0x0004  /* 1 to enable digital loopback */
#define REG_AMCR_AUDEN_MASK         0x0020  /* Audio block enable bit */
#define REG_AMCR_MODE_16K_MASK      0x0040  /* 0=8K mode, 1=16K mode */
#define REG_AMCR_CRAMSEL_MASK       0x0080  /* Coefficient RAM select for DSP read/write */
#define REG_AMCR_PCMEN_MASK         0x4000  /* PCM interface enable */
#define REG_AMCR_DAISTAT_MASK       0x8000  /* Speech Frame Sync. Indicator (1=Frame Sync, 0 No sync) */

/* Register AUDMOD bit mask definitions */
#define REG_AUDMOD_AUDEN_MASK       0x0001  /* Enable audio path */
#define REG_AUDMOD_AMUTE_MASK       0x0002  /* Mute audio */
#define REG_AUDMOD_ACOMPMOD_MASK    0x0004  /* 0=FIR compensation filter selected, 1=IIR compensation filer selected */
#define REG_AUDMOD_I2SMODE_MASK     0x0008  /* 0= Audio path take data from DSP, 1=Audio path is drivend by I2S direct path */
#define REG_AUDMOD_AUDINTDIS_MASK   0x0010  /* 1=Disable the audio interrupt */
#define REG_AUDMOD_AHBIFMOD_MASK    0x00e0  /* 3 bits HBIF mode  (RO) */
#define REG_AUDMOD_ARATEADP_MASK    0x0300  /* 2 bits audio ADP rate (RO) */
#define REG_AUDMOD_ASAMPRATE_MASK   0x3c00  /* 4 bits audio sampling rate */

#define REG_AUDMOD_ASAMPRATE_BIT_SHIFT  10
#define REG_AUDMOD_ASAMPRATE_8KHz   (0 << REG_AUDMOD_ASAMPRATE_BIT_SHIFT)
#define REG_AUDMOD_ASAMPRATE_12KHz  (1 << REG_AUDMOD_ASAMPRATE_BIT_SHIFT)
#define REG_AUDMOD_ASAMPRATE_16KHz  (2 << REG_AUDMOD_ASAMPRATE_BIT_SHIFT)
#define REG_AUDMOD_ASAMPRATE_24KHz  (3 << REG_AUDMOD_ASAMPRATE_BIT_SHIFT)
#define REG_AUDMOD_ASAMPRATE_32KHz  (4 << REG_AUDMOD_ASAMPRATE_BIT_SHIFT)
#define REG_AUDMOD_ASAMPRATE_48KHz  (5 << REG_AUDMOD_ASAMPRATE_BIT_SHIFT)
#define REG_AUDMOD_ASAMPRATE_11025  (6 << REG_AUDMOD_ASAMPRATE_BIT_SHIFT)
#define REG_AUDMOD_ASAMPRATE_22005  (7 << REG_AUDMOD_ASAMPRATE_BIT_SHIFT)
#define REG_AUDMOD_ASAMPRATE_44100  (8 << REG_AUDMOD_ASAMPRATE_BIT_SHIFT)

/* AUDVOC_ISR bit mask definitions */
#define REG_AUDVOC_BTTAP_NB_INT_MASK    (1<<7)  /* BlueTooth taps narrow band interrupt */
#define REG_AUDVOC_BTTAP_WB_INT_MASK    (1<<6)  /* BlueTooth taps wide band interrupt */
#define REG_AUDVOC_VIBRA_INT_MASK       (1<<5)  /* Vibra path interrupt (removed in BCM4760)*/
#define REG_AUDVOC_POLYRINGER_INT_MASK  (1<<4)  /* DAC Polyringer path interrupt (removed in BCM4760)*/
#define REG_AUDVOC_ADC_AUDIO_INT_MASK   (1<<3)  /* ADC audio path interrupt */
#define REG_AUDVOC_DAC_AUDIO_INT_MASK   (1<<2)  /* DAC audio path interrupt */
#define REG_AUDVOC_VOICE_INT_MASK       (1<<1)  /* Voice interrupt */
#define REG_AUDVOC_PCM_INT_MASK         (1<<0)  /* PCM interrupt */


#define REG_AIFIFOEMTRYCOUNT_MASK       (0x7f)  /* INPUT FIFO empty entry count mask */
#define REG_AIFIFOTHMET_MASK            (1<<8)  /* output fifo threadhold */
#define REG_AIFIFOUDF_MASK              (1<<9)  /* input fifo underflow */
#define REG_AIFIFOOFV_MASK              (1<<9)  /* input fifo overflow */

/* Slope gain control */
#define REG_SLOPGAIN_ENABLE_MASK        0x8000  /* Slope gain enable */
#define REG_SLOPMOD_MASK                0x7800
#define REG_SLOPMOD_BIT_SHIFT           11
#define REG_SLOPTARGAIN_MASK            0x01ff
#define REG_SLOPTARGAIN_BIT_SHIFT       0

#define REG_SLOPTARGAIN_DIGMUTE         0       /* Slope gain digital mute value */

/* Audio input paths control */
#define REG_AIN_ENABLE_MASK             0x0001
#define REG_AIN_INPUTSEL_MASK           0x0006

/* Video input paths control */
#define REG_VIN_ENABLE_MASK             0x0001
#define REG_VIN_INPUTSEL_MASK           0x0006

/* Register REG_VADCCTRL_ADDR (0xb702b0) bit mask definitions */
#define REG_VADCCTRL_AIN_SEL_MASK       0x8000  /* Audio in path channel select (0=left, 1=right) */
#define REG_VADCCTRL_VIN_SEL_MASK       0x4000  /* Voice in path channel select (0=left, 1=right) */
#define REG_VADCCTRL_DICMIC_CLK_SEL_MASK 0x2000 /* Offchip digital MIC clock select (0=3.250Mhz clock, 1=1.625Mhz clock) */
#define REG_VADCCTRL_VINPATH2_16K_MASK  0x1000  /* 16K mode for voice in 2nd path (right channel) */
#define REG_VADCCTRL_AIN_16BITMODE_MASK 0x0800  /* Audio output bit width (0-18 bits, 1=16 bits) */
#define REG_VADCCTRL_AUD_OVF_MODE_MASK  0x0400  /* Method to handle audio overflow in audio IIR filter (0=clipped value stored,1=0 stored in memory) */
#define REG_VADCCTRL_AUD_INT_EN_MASK    0x0200  /* Audio interrupt disable for path 2 */
#define REG_VADCCTRL_VIN_IIRCM_SEL_MASK 0x0100  /* Write IIR coefficient to voice path filter select (0=left channel, 1=right channel) */
#define REG_VADCCTRL_DIGMIC_PH_SEL_MASK 0x0080  /* Digital MIC Phase select */
#define REG_VADCCTRL_DIGMIG_EN_MASK     0x0040  /* Digital MIC enable (not valid for BCM476x) */
#define REG_VADCCTRL_RXANA2_EN_MASK     0x0020  /* Analog ADC2 enable, need to enable for digital MIC2 */
#define REG_VADCCTRL_VIN_INTOFFSET_MASK 0x001c  /* Voice int interrupt offset  (should be 0) */
#define REG_VADCCTRL_LOOPEN_MASK        0x0002  /* Analog loopback enable */
#define REG_VADCCTRL_SW_ADCAP_MASK      0x0001  /* ADC API interface enable */


/* VIC audio interrupt masks */
#define VIC_AUDIO_INT_MASK              (REG_AUDVOC_ADC_AUDIO_INT_MASK | REG_AUDVOC_DAC_AUDIO_INT_MASK)
#define VIC_VOICE_INT_MASK              (REG_AUDVOC_VOICE_INT_MASK | REG_AUDVOC_PCM_INT_MASK)
#define VIC_BTMIXER_INT_MASK            (REG_AUDVOC_BTTAP_WB_INT_MASK)

#define BCM476X_STEREO_DAI  0
#define BCM476X_VOICE_DAI   1
#define BCM476X_BT_DAI      2
#define BCM476X_NUM_DAIS    3
extern struct snd_soc_dai bcm476x_dai[BCM476X_NUM_DAIS];
extern struct snd_soc_codec_device soc_codec_dev_bcm476x;

typedef void (bcm476x_codec_xferdone_cb)(void *substream, int err, int xfer_size);
void bcm476x_codec_register_stream_xfer_done_nofify(bcm476x_codec_xferdone_cb *cb);

/* Virtual register addresses (0xb7e00-0xb7fff) */
#define REG_VIRTUAL_ADDR_BASE   (AUD1_REG_BASE_ADDR+0x0e00)
#define REG_VIRTUAL_LGAIN       (REG_VIRTUAL_ADDR_BASE+0x00)           /* gain for both audio & voice path */
#define REG_VIRTUAL_RGAIN       (REG_VIRTUAL_ADDR_BASE+0x04)           /* gain for both audio & voice path */
#define REG_VIRTUAL_MUTE        (REG_VIRTUAL_ADDR_BASE+0x08)           /* mute for both audio & voice path */
#define REG_VIRTUAL_ADDR_LAST   (REG_VIRTUAL_ADDR_BASE+0x0c)

#endif
