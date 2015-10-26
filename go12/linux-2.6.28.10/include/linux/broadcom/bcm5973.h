/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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

/*
 * include/linux/broadcom/bcm5973.h
 * User space access to bcm5973
 */

#ifndef BCM5973_H
#define BCM5973_H

/* IOCTL commands */

#define BCM5973_IOC_MAGIC			'b'

/* Get/Set number of sensor inputs and drive outputs */
#define BCM5973_IOC_RD_NUMINPUTS		_IOR(BCM5973_IOC_MAGIC, 1, __u8)
#define BCM5973_IOC_WR_NUMINPUTS		_IOW(BCM5973_IOC_MAGIC, 1, __u8)
#define BCM5973_IOC_RD_NUMOUTPUTS		_IOR(BCM5973_IOC_MAGIC, 2, __u8)
#define BCM5973_IOC_WR_NUMOUTPUTS		_IOW(BCM5973_IOC_MAGIC, 2, __u8)

/* Read/Write BCM5973 onchip registers */
#define BCM5973_IOC_REG 0x20
#define BCM5973_REGMASK 0x1f // max 32 registers allowed
#define BCM5973_IOC_RD_REG(reg)     _IOR(BCM5973_IOC_MAGIC, BCM5973_IOC_REG+(reg & BCM5973_REGMASK), __u8)
#define BCM5973_IOC_WR_REG(reg)     _IOW(BCM5973_IOC_MAGIC, BCM5973_IOC_REG+(reg & BCM5973_REGMASK), __u8)

/* BCM5973 onchip random access registers */
/* Use BCM5973_IOC_RD_REG(), BCM5973_IOC_WR_REG() IOCTL to access */

enum BCM5973_REG {
                /* adr bits*/
    WAKE,       /*  0 [2:0] */
    LO_CH,      /*  1 [5:0] */
    HI_CH,      /*  2 [5:0] */
    CFB_UNIV,   /*  3 cfb_univ[5:0], amp_bias[6] */
    RFB_UNIV,   /*  4 [1:0] */
    VOFFSET,    /*  5 voff_univ[3:0], voff_zero[4] */
    GAIN_REG,   /*  6 gain_reg[3:0], cadc_univ[5:4] */
    STMCTL,     /*  7 [1:0] */
    DCL,        /*  8 [5:0] */
    MCNTCTL,    /*  9 [1:0] */
    NUMCTL,     /* 10 [6:0] */
    MUX,        /* 11 [5:0] */
    MUXINC,     /* 12 [1:0] */
    BCNT_REQ,   /* 13 bcnt_req[5:0], auto_inc[6] */
    BPRE,       /* 14 [1:0] */
    RESET,      /* 15 [0] */
    BEEN_RESET, /* 16 [0] */
    VERSION,    /* 17 [7:0] */
    ANA_TEST,   /* 18 ana_test[5:0], self_test[6], pga_bypass[7] */
    DIAG_MUX,   /* 19 [7:0] */
    ACC_TEST,   /* 20 [3:0] */
    PREAMP,     /* 21 preamp_bias[1:0], preamp_vref[3:2] */
    BIAS,       /* 22 refamp_bias[1:0], flashcmp_bias[3:2], intamp_bias[5:4], adcin_bias[7:6] */
    VREF,       /* 23 flashcmp_vref[2:0], intamp_vref[5:4] */
    ANALOG_CTRL3, /* 24 [7:0] */
    MAX_BCM5973_REG
};

/* Register-specific fields and constants */

#define REG_WAKE_RUN    3
#define REG_WAKE_SLEEP  0

#define REG_BCNT_AUTOINC    (1<<6)
#define REG_BCNT_MASK       0x3f


/* READ command */
/* Read initiates 1 scan of the full sensor image and returns the scan data
   Bufsize must be 2*NumInputs*NumOutputs bytes (2 bytes per sense input)
 */

/* WRITE command */
/* Write loads the BCM5973 internal lookup table data.
   (CFB_CH[64], VOFF_CH[64] and Demod[512] tables)
   Array of signed byte values
 */

/* OPEN command */
/* Initialize the device, with default register values */

#endif /* BCM5973_H */
