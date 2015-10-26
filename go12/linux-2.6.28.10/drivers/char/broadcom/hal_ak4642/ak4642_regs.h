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



typedef enum
{
   AK4642_REG_PM1 = 0x00,  /* Power Management 1 */
#define AK4642_REG_PM1_PMADL  0x01  /* MIC-Amp Lch and ADC Lch Power Management */
#define AK4642_REG_PM1_PMDAC  0x04
#define AK4642_REG_PM1_PMLO   0x08
#define AK4642_REG_PM1_PMSPK  0x10
#define AK4642_REG_PM1_PMBP   0x20
#define AK4642_REG_PM1_PMVCM  0x40

   AK4642_REG_PM2 = 0x01,  /* Power Management 2 */
#define AK4642_REG_PM2_PMPLL  0x01
#define AK4642_REG_PM2_MCK0   0x02
#define AK4642_REG_PM2_MS     0x08
#define AK4642_REG_PM2_PMHPR  0x10
#define AK4642_REG_PM2_PMHPL  0x20
#define AK4642_REG_PM2_HPMTN  0x40  /* Headphone-Amp Mute Control */

   AK4642_REG_SS1 = 0x02,  /* Signal Select 1 */
#define AK4642_REG_SS1_MGAIN0 0x01
#define AK4642_REG_SS1_PMMP   0x04  /* MPWR pin Power Management */
#define AK4642_REG_SS1_DACL   0x10
#define AK4642_REG_SS1_DACS   0x20
#define AK4642_REG_SS1_BEEPS  0x40
#define AK4642_REG_SS1_SPPSN  0x80

   AK4642_REG_SS2 = 0x03,
#define AK4642_REG_SS2_SPKG_MASK 0x18
#define AK4642_REG_SS2_SPKG_4_43DB  0x00
#define AK4642_REG_SS2_SPKG_6_43DB  0x08
#define AK4642_REG_SS2_SPKG_10_65DB 0x10
#define AK4642_REG_SS2_SPKG_12_65DB 0x18
#define AK4642_REG_SS2_MGAIN1       0x20


   AK4642_REG_MC1 = 0x04,  /* Mode Control 1 */
   AK4642_REG_MC2 = 0x05,  /* Mode Control 2 */
#define AK4642_REG_MC2_8KHZ      0x00
#define AK4642_REG_MC2_12KHZ     0x01
#define AK4642_REG_MC2_16KHZ     0x02
#define AK4642_REG_MC2_24KHZ     0x03
#define AK4642_REG_MC2_7_35KHZ   0x04
#define AK4642_REG_MC2_11_025KHZ 0x05
#define AK4642_REG_MC2_14_7KHZ   0x06
#define AK4642_REG_MC2_22_05KHZ  0x07
#define AK4642_REG_MC2_32KHZ     0x22
#define AK4642_REG_MC2_48KHZ     0x23
#define AK4642_REG_MC2_29_4KHZ   0x26
#define AK4642_REG_MC2_44_1KHZ   0x27

   AK4642_REG_06,          /* Timer Select */
   AK4642_REG_07,          /* ALC Mode Control 1 */
   AK4642_REG_08,          /* ALC Mode Control 2 */
   AK4642_REG_IVL = 0x09,  /* Lch Input Volume Control */
   AK4642_REG_DVL = 0x0A,  /* Lch Digital Volume Control */
   AK4642_REG_0B,          /* ALC Mode Control 3 */
   AK4642_REG_IVR = 0x0C,  /* Rch Input Volume Control */
   AK4642_REG_DVR = 0x0D,  /* Rch Digital Volume Control */
   AK4642_REG_MC3 = 0x0E,  /* Mode Control 3 */
#define AK4642_REG_DVOLC      0x10
#define AK4642_REG_MC3_SMUTE  0x20

   AK4642_REG_MC4 = 0x0F,  /* Mode Control 4 */
#define AK4642_REG_MC4_DACH   0x01  /* Switch control from DAC to Headphone-Amp */

   AK4642_REG_PM3 = 0x10,   /* Power Management 3 */
#define AK4642_REG_PM3_PMADR  0x01
#define AK4642_REG_PM3_INL    0x02  /* ADC Lch Input Source Select 0: LIN1 pin 1: LIN2 pin */
#define AK4642_REG_PM3_INR    0x04
#define AK4642_REG_PM3_MDIF1  0x08
#define AK4642_REG_PM3_MDIF2  0x10
#define AK4642_REG_PM3_HPG    0x20

   AK4642_REG_11, /* Digital Filter Select */
#define AK4646_REG_11_FIL3    0x04
#define AK4646_REG_11_EQ      0x08
#define AK4646_REG_11_FIL1    0x10
#define AK4646_REG_11_GN0     0x40
#define AK4646_REG_11_GN1     0x80

   AK4642_REG_12, /* FIL3 Co-efficient 0 */
   AK4642_REG_13, /* FIL3 Co-efficient 1 */
   AK4642_REG_14, /* FIL3 Co-efficient 2 */
   AK4642_REG_15, /* FIL3 Co-efficient 3 */
   AK4642_REG_16, /* EQ Co-efficient 0 */
   AK4642_REG_17, /* EQ Co-efficient 1 */
   AK4642_REG_18, /* EQ Co-efficient 2 */
   AK4642_REG_19, /* EQ Co-efficient 3 */
   AK4642_REG_1A, /* EQ Co-efficient 4 */
   AK4642_REG_1B, /* EQ Co-efficient 5 */
   AK4642_REG_1C, /* FIL1 Co-efficient 0 */
   AK4642_REG_1D, /* FIL1 Co-efficient 1 */
#define AK4646_REG_1D_F1AS    0x80  /* F1 A select bit: 1 = LPF, 0 = HFP */

   AK4642_REG_1E, /* FIL1 Co-efficient 2 */
   AK4642_REG_1F  /* FIL1 Co-efficient 3 */

} AK4642_REG;

#define AK4642_MAX_REGS          0x20
