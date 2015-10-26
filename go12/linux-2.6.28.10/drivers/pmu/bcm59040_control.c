/*****************************************************************************
 * Copyright 2003 - 2010 Broadcom Corporation.  All rights reserved.
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
 * BCM59040 PMU Control Driver
 *****************************************************************************
 *
 *     This implements the BCM59040 PMU Control driver layer. This layer
 *     implements several generic functions such as low-level GPIO configuration
 *     and control, basic PMU configuration and status reporting, low-level
 *     control of the regulators and LDOs along with a few other miscellaneous
 *     functions that don't have a direct relationship to the other sub-
 *     systems.
 *
 *
 ****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/fs.h>
#include <linux/pm.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>

#include <linux/platform_device.h>
#include <linux/pmu_device.h>
#include <asm/arch/pmu_device_bcm59040.h>

#include <linux/broadcom/pmu_bcm59040.h>

/* Include external interfaces */
#include <asm/arch/bcm59040_irqs.h>

/*
* Structure and macro definitions.
*/
#define BCM59040_CONTROL_DRIVER			"bcm59040_control"
#define BCM59040_CONTROL_MOD_DESCRIPTION	"BCM59040 Control Driver"
#define BCM59040_CONTROL_MOD_VERSION		1.0

#undef BCM59040_DEVNAME
#define BCM59040_DEVNAME			BCM59040_CONTROL_DRIVER
#define PFX					BCM59040_DEVNAME ": "

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
//#define DEBUG 1

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

#ifdef DEBUG
//#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO | DBG_TRACE)
#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO)

static int 		gLevel = DBG_DEFAULT_LEVEL;

#	define PMU_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define PMU_DEBUG(level,x)
#endif

typedef struct bcm59040_control_config_struct
{
	//bcm_control_handler_t			*control_handler_cb;

	struct pmu_client			*pclient;
	struct work_struct			work;
	struct workqueue_struct		*workqueue;
} bcm59040_control_config_t;


/*
* Local variables.
*/

static struct pmu_client *bcm59040_power_off_pclient;

static int pwmled_sw_ctrl_enable = 0;   /* initialize to disabled; only matters for pwmled 2 & 3 */

static int TempKeyLockBitSet = 0;          // KEYLOCK bit set flag
static int PonkeyONOFF = 0;            // PONKEYONOFF bit set flag
static int PonkeyKeylockOTP = 0;        // OTP value of keylock bit

static int bcm59040_ponkey_config(struct pmu_client *pclient);
static int bcm59040_UserPonkeyLock(struct pmu_client *pclient, int enable);
static int bcm59040_pwmled_config(struct pmu_client *pclient);
static int bcm59040_set_pwm_hiper(struct pmu_client *pclient, BCM_PMU_PWM_hi_per_t hiper);
static int bcm59040_set_pwm_loper(struct pmu_client *pclient, BCM_PMU_PWM_lo_per_t loper);
static int bcm59040_set_pwm_ctrl(struct pmu_client *pclient, BCM_PMU_PWM_ctrl_t pwmctrl);
static int bcm59040_set_pwm_pwr_ctrl(struct pmu_client *pclient, BCM_PMU_PWM_pwr_ctrl_t pwrctrl);
static int bcm59040_set_pwm_sw_ctrl(struct pmu_client *pclient, int sw_enable);

static int bcm59040_control_remove(struct pmu_client *pclient);

static ssize_t reboot_mode_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	ssize_t ret = count;
	uint8_t data;
	bcm59040_control_config_t *control_config;
	struct pmu_client *pclient;

	control_config = (bcm59040_control_config_t *) dev->driver_data;
	pclient = control_config->pclient;
	if (!strncmp(buf, "0\n", 2)) {
		data = 0x81;	/* 10 s */
	} else if (!strncmp(buf, "1\n", 2)) {
		data = 0x85;	/* 15 s */
	} else {
		ret = -EINVAL;
		return ret;
	}
	pmu_bus_seqwrite(pclient, BCM59040_REG_RSTRTCNTRL, &data, 1);

	return ret;
}

DEVICE_ATTR(reboot_mode, S_IRUGO| S_IWUGO, NULL, reboot_mode_store);

/*
 * Function definitions.
 */

static int bcm59040_ponkey_config(struct pmu_client *pclient)
{
	int				 rc = 0;
	uint8_t				 data;
	bcm59040_client_platform_data_t  *pdata;
	struct bcm59040_control_defaults *defaults;

	if (pclient->dev.platform_data == NULL) {
		printk (KERN_ERR "Error, platform data not initialized\n");
		return -1;
	}

	pdata = (bcm59040_client_platform_data_t*)(pclient->dev.platform_data);
	if (pdata->defaults == NULL) {
		printk (KERN_ERR "Error, default not initialized for charger\n");
		return -1;
	}

	defaults = (struct bcm59040_control_defaults *) pdata->defaults;

	/*
	 * Bits of PONKEYBCNTRL1 are all OTPed for 59040B0, so don't need
	 * to configure it anymore.
	 */

	/* configure PONKEYBCNTRL2: there's no OTPable fields in this register */
	data = defaults->poncntrl2 &
		   (BCM59040_PONKEYBCNTRL2_OFFHOLD_MASK |
			BCM59040_PONKEYBCNTRL2_PONKEYBDEL_MASK);
	if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_PONKEYBCNTRL2, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_PONKEYBCNTRL2 register.\n"));
		return rc;
	}

	/* configure RSTRTCNTRL, preserve OTP fields */
	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_RSTRTCNTRL, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_RSTRTCNTRL register.\n"));
		return rc;
	}

	data = ((defaults->rstrtcntrl & (~BCM59040_RSTRTCNTRL_OTP_MASK)) | (data & BCM59040_RSTRTCNTRL_OTP_MASK) );
	if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_RSTRTCNTRL, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_RSTRTCNTRL register.\n"));
		return rc;
	}

	return rc;
}

static int bcm59040_UserPonkeyLock(struct pmu_client *pclient, int enable)
{
	int			rc=0;
	uint8_t		data;

	if (PonkeyKeylockOTP == 1) { // OTP value is 1 for B0 and TomTom case, otherwise is 0 (A0 or B0)
		TempKeyLockBitSet = 0;
	} else {
		if(enable) { //make sure the shutdown delay is not 0
			if((rc = pmu_bus_seqread(pclient, BCM59040_REG_PONKEYBCNTRL2, &data, 1)) < 0) {
				PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_PONKEYBCNTRL2 register.\n"));
				return rc;
			}
			if((data & BCM59040_PONKEYBCNTRL2_PONKEYBDEL_MASK) == 0) {
				data = data | 0x08;  // set to 500 msec
				if((rc = pmu_bus_seqwrite(pclient,
										  BCM59040_REG_PONKEYBCNTRL2,
										  &data, 1)) < 0) {
				PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_PONKEYBCNTRL2 register.\n"));
				return rc;
				}
			}
		} else {
			// restore default values
			data = BCM59040_PMU_REG_PONKEYBCNTRL2_VAL & (BCM59040_PONKEYBCNTRL2_OFFHOLD_MASK | BCM59040_PONKEYBCNTRL2_PONKEYBDEL_MASK);
			if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_PONKEYBCNTRL2, &data, 1)) < 0) {
				PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_PONKEYBCNTRL2 register.\n"));
				return rc;
			}
		}
	
		TempKeyLockBitSet = (enable > 0) ? 1 : 0;
	}

	return rc;
}

static int bcm59040_pwmled_config(struct pmu_client *pclient)
{
	int			rc=0;
	uint8_t		data;

	// read current settings
	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_PWMLEDCTRL1, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_PWMLEDCTRL1 register.\n"));
		return rc;
	}

	if ( data & BCM59040_PWMLEDCTRL1_PWMLED_SW_CTRL ) {
		pwmled_sw_ctrl_enable = 1;
	} else {
		pwmled_sw_ctrl_enable = 0;
	}

	return rc;
}

static int bcm59040_set_pwm_hiper(struct pmu_client *pclient, BCM_PMU_PWM_hi_per_t hiper)
{
	int			rc=0;
	uint8_t		data;
	u8			regAddr;
	
	if((hiper.pwmledNum > 3) || (hiper.pwmledNum < 1)) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_set_pwm_hiper: PWMLED %d not available.\n", hiper.pwmledNum));
		return -EINVAL;
	}
	
	if (hiper.hi_per > 0x3f) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_set_pwm_hiper: High per out of range 0-0x3f, given val = 0x%x\n", hiper.hi_per));
		return -EINVAL;
	}
	
	regAddr = BCM59040_REG_PLD1CTRL2 + (3 * (hiper.pwmledNum - 1));
	data = (uint8_t)(hiper.hi_per);

	// set high period value.
	if((rc = pmu_bus_seqwrite(pclient, regAddr, &data, 1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_PLD%dCTRL2 register.\n", hiper.pwmledNum));
			return rc;
	}
	
	if (hiper.pwmledNum != 1) {
		bcm59040_set_pwm_sw_ctrl(pclient, 1);	//PWMLED2&3 are affected by PWMLED_SW_CTRL
	}
	
	return rc;
} 

static int bcm59040_set_pwm_loper(struct pmu_client *pclient, BCM_PMU_PWM_lo_per_t loper)
{
	int			rc=0;
	uint8_t		data;
	u8			regAddr;

	if ((loper.pwmledNum > 3) || (loper.pwmledNum < 1)) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_set_pwm_loper: PWMLED %d not available.\n", loper.pwmledNum));
		return -EINVAL;
	}

	if ( loper.lo_per > 0x3f ) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Low per out of range 0-0x3f, given val = 0x%x\n", loper.lo_per));
		return -EINVAL;
	}

	regAddr = BCM59040_REG_PLD1CTRL3 + (3 * (loper.pwmledNum - 1));
	data = (uint8_t)(loper.lo_per);

	// set low period value.
	if((rc = pmu_bus_seqwrite(pclient, regAddr, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_PLD%dCTRL2 register.\n", loper.pwmledNum));
		return rc;
	}

	if (loper.pwmledNum != 1) {
		bcm59040_set_pwm_sw_ctrl(pclient, 1);
	}

	return rc;
}

static int bcm59040_set_pwm_ctrl(struct pmu_client *pclient, BCM_PMU_PWM_ctrl_t pwmctrl)
{
	int			rc=0;
	uint8_t		data;
	u8			regAddr;

	if ((pwmctrl.pwmledNum > 3) || (pwmctrl.pwmledNum < 1)) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_set_pwm_ctrl: PWMLED %d not available.\n", pwmctrl.pwmledNum));
		return -EINVAL;
	}

	regAddr = BCM59040_REG_PLD1CTRL1 + (3 * (pwmctrl.pwmledNum - 1));
	if((rc = pmu_bus_seqread(pclient, regAddr, &data, 1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_PLD%dCTRL1 register.\n",pwmctrl.pwmledNum));
			return rc;
	}

	if(pwmctrl.pwmledNum == 1) {
		/* preserve the OTP value of the current field */
		rc = (BCM59040_PLD1CTRL1_LCTRL1_MASK & (unsigned char)(rc));
	} else {
		bcm59040_set_pwm_sw_ctrl(pclient, 1);
		/* preserve the OTP value of the current field; pwmled2&3 have same format */
		data = (BCM59040_PLD2CTRL1_LCTRL2_MASK & data);
	}

	data = ((pwmctrl.pwmdiv << 2) | (pwmctrl.pwmled_ctrl << 1) | (pwmctrl.pwmledOn) | data);

	if((rc = pmu_bus_seqwrite(pclient, regAddr, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_PLD%dCTRL1 register.\n", pwmctrl.pwmledNum));
		return rc;
	}

	return rc;
}

static int bcm59040_set_pwm_pwr_ctrl(struct pmu_client *pclient, BCM_PMU_PWM_pwr_ctrl_t pwrctrl)
{
	uint8_t		data;
	int			rc;

	if ( pwrctrl.pwr_ctrl > 1 ) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_set_pwm_pwr_ctrl: Power contrl value can be either 0 or 1, given val = 0x%x\n", pwrctrl.pwr_ctrl));
		return -EINVAL;
	}

	// read current settings
	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_PWMLEDCTRL1, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_PWMLEDCTRL1 register.\n"));
		return rc;
	}

	if ( pwrctrl.pwr_ctrl == 0 ) {
		data = data & (~(BCM59040_PWMLEDCTRL1_PWMLED_PDN));		// Disable bit 7, set to 0.
	} else {
		data = data | (BCM59040_PWMLEDCTRL1_PWMLED_PDN);		// Enable bit 7, set to 1.
	}

	// update register
	if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_PWMLEDCTRL1, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_PWMLEDCTRL1 register.\n"));
		return rc;
	}

	return rc;
}

static int bcm59040_set_pwm_sw_ctrl(struct pmu_client *pclient, int sw_enable) 
{
	int			rc=0;
	uint8_t		data;

	if ((sw_enable && (pwmled_sw_ctrl_enable == 0)) ||
		((sw_enable == 0) && (pwmled_sw_ctrl_enable))) {
		// read current settings
		if((rc = pmu_bus_seqread(pclient, BCM59040_REG_PWMLEDCTRL1, &data, 1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_PWMLEDCTRL1 register.\n"));
			return rc;
		}

		if (sw_enable && (pwmled_sw_ctrl_enable == 0)) {
			data = data | BCM59040_PWMLEDCTRL1_PWMLED_SW_CTRL;
		} else {
			data = data & (~BCM59040_PWMLEDCTRL1_PWMLED_SW_CTRL) ;
		}

		if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_PWMLEDCTRL1, &data, 1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_PWMLEDCTRL1 register.\n"));
			return rc;
		}
	}

	return rc;
}

static void bcm59040_power_off(void)
{
	int		rc=0;
	uint8_t		data;

	if ((rc = pmu_bus_seqread(bcm59040_power_off_pclient, BCM59040_REG_HOSTACT, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_HOSTACT register.\n"));
		return;
	}
	else
	{
		data |= BCM59040_HOSTACT_HOSTDICOFF;
		if ((rc = pmu_bus_seqwrite(bcm59040_power_off_pclient, BCM59040_REG_HOSTACT, &data, 1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_HOSTACT register.\n"));
			return;
		}
	}

	while(1)
	{
		data++;
	}
}

/** bcm59040_control_work - Work function for BCM59040 control
*  IRQ
* @work:   Pointer to work structure.
*
*/
static void bcm59040_control_work(struct work_struct *work)
{
	int				rc = 0;
	uint8_t				data, intbit = 1 << BCM59040_VINT1_PONKEY_BIT;
	bcm59040_control_config_t	*control_config;
	struct pmu_client		*pclient;

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Entering bcm59040_control_work:\n"));

	control_config = container_of(work, bcm59040_control_config_t, work);
	pclient = control_config->pclient;

	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_INT1, &data, 1 )) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_VINT1 register.\n"));

		/* clear the interrupt */
		pmu_bus_seqwrite(pclient, BCM59040_REG_VINT1, &intbit, 1 );
		return;
	}

	/* clear the interrupt */
	pmu_bus_seqwrite(pclient, BCM59040_REG_VINT1, &intbit, 1 );

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Read %d bytes (%02X) from virtual int status registers.\n", rc, data));

	if (data & BCM59040_INT1_PONKEYR) {  // rising event
		if(TempKeyLockBitSet) {
			if(PonkeyONOFF) {
				if((rc = pmu_bus_seqread(pclient,
										 BCM59040_REG_PONKEYBCNTRL1,
										 &data, 1)) < 0) {
					PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_PONKEYBCNTRL1 register.\n"));
					return;
				} else {
					data =  data | BCM59040_PONKEYBCNTRL1_KEYLOCK;
					if((rc = pmu_bus_seqwrite(pclient,
											  BCM59040_REG_PONKEYBCNTRL1,
											  &data, 1)) < 0) {
						PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_PWMLEDCTRL1 register.\n"));
						return;
					}
				}
	
				if((rc = pmu_bus_seqread(pclient,
										 BCM59040_REG_PONKEYBCNTRL1,
										 &data, 1)) < 0) {
					// read back before set it off
					PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_PONKEYBCNTRL1 register.\n"));
					return;
				}
				data = data & 0xBF;
				if((rc = pmu_bus_seqwrite(pclient,
										  BCM59040_REG_PONKEYBCNTRL1,
										  &data, 1)) < 0) {
					PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_PWMLEDCTRL1 register.\n"));
					return;
				}
			}
		}
	}

	if (data & BCM59040_INT1_PONKEYF) {		// falling event
	}

	if (data & BCM59040_INT1_PONKEYH) {
		if(TempKeyLockBitSet) {
			if(!PonkeyONOFF) {
				if((rc = pmu_bus_seqread(pclient,
										 BCM59040_REG_PONKEYBCNTRL1,
										 &data, 1)) < 0) {
					PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_PONKEYBCNTRL1 register.\n"));
					return;
				} else {
					data =  data | BCM59040_PONKEYBCNTRL1_KEYLOCK;
					if((rc = pmu_bus_seqwrite(pclient,
											  BCM59040_REG_PONKEYBCNTRL1,
											  &data, 1)) < 0) {
						PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_PWMLEDCTRL1 register.\n"));
						return;
					}
				}
	
				if((rc = pmu_bus_seqread(pclient,
										 BCM59040_REG_PONKEYBCNTRL1,
										 &data, 1)) < 0) {
					// read back before set it off
					PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_PONKEYBCNTRL1 register.\n"));
					return;
				}
				data = data & 0xBF;
				if((rc = pmu_bus_seqwrite(pclient,
										  BCM59040_REG_PONKEYBCNTRL1,
										  &data, 1)) < 0) {
					PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_PWMLEDCTRL1 register.\n"));
					return;
				}
			}
		}
	}

	if(data & BCM59040_INT1_PONKEYBHD) {
	}
}

/** bcm59040_control_isr - Interrupt service routine for BCM59040
 *                  control IRQ vector number
 *
 * This ISR handles events for the control in the BCM59040. The
 * primary event we're interested in is the
 *
 * @dev_id: Data structure associated with this interrupt event.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */
static irqreturn_t bcm59040_control_isr(int irq, void *dev_id)
{
	bcm59040_control_config_t	*control_config = dev_id;

	queue_work(control_config->workqueue, &control_config->work);

	return IRQ_HANDLED;
}

/** bcm59040_control_probe - BCM59040 control probe function.
 *
 * Probe to determine if BCM59040 is really present.
 *
 * @client:	Pointer to client structure for this device.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */
static int bcm59040_control_probe(struct pmu_client *pclient,
								  const struct pmu_device_id *id)
{
	int							rc=0;
	bcm59040_control_config_t	*control_config;
	u8							data;

	bcm59040_ponkey_config(pclient);
	bcm59040_pwmled_config(pclient);

	if((control_config = kmalloc(sizeof(*control_config), GFP_KERNEL)) == NULL) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Out of memory allocating control_config\n"));
		return -ENOMEM;
	}
	control_config->workqueue = create_workqueue(BCM59040_CONTROL_DRIVER);
	control_config->pclient = pclient;

	INIT_WORK(&control_config->work, &bcm59040_control_work);

	pmu_set_drvdata(pclient, control_config);

	rc = request_irq(BCM59040_IRQ_PONKEY,
					  bcm59040_control_isr,
					  0,
					  "bcm59040_ctrl",
					  control_config);

	if (rc < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_control_probe failed to attach interrupt, rc = %d\n", rc) );
		kfree(control_config);
		return rc;
	}

	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_HBCTRL, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_HBCTRL register.\n"));
		goto out_err;
	}
	data &= ~BCM59040_HBCTRL_HB_EN;
	data |= BCM59040_HBCTRL_HBMODE_PM3 | BCM59040_HBCTRL_PC_PIN_EN;
	if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_HBCTRL, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_HBCTRL register.\n"));
		goto out_err;
	}

//	pm_power_off = bcm59040_power_off;
	bcm59040_power_off_pclient = pclient;

	rc = device_create_file(&pclient->dev, &dev_attr_reboot_mode);
	if (rc < 0) {
		printk(KERN_ERR "error creating BCM59040 reboot_mode sysfs: %d\n", rc);
		goto out_err;
	}

	return 0;

out_err:
	free_irq(BCM59040_IRQ_PONKEY, control_config);
	kfree(control_config);
	return rc;
}

/** bcm59040_control_remove - BCM59040 control remove
 *
 * Need to remove BCM59040 control driver.
 *
 * @client:	Pointer to client structure for this device.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 * @todo	Need to determine what suspend does for this driver still.
 *
 */

static int bcm59040_control_remove(struct pmu_client *pclient)
{
	bcm59040_control_config_t			*control_config;

	control_config = pmu_get_drvdata(pclient);
	if(control_config)
	{
		free_irq(BCM59040_IRQ_PONKEY, control_config);
		kfree(control_config);
	}

	return 0;
}

#ifdef CONFIG_PM

/** bcm59040_control_suspend - BCM59040 control suspend
 *
 * Ensure that PONKEY held interrupt is unmasked as we want this to
 * be a wakeup event and therefore it must be unmasked. The PMU
 * interrupt must also be selected as a wake-up event to the SoC.
 *
 * @pclient:	Pointer to client structure for this device.
 * @state:		Power Management state change message.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */

static int bcm59040_control_suspend(struct pmu_client *pclient, pm_message_t state)
{
	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "suspend processed for %s (event=%d).\n",
						  pclient->name, state.event));

	disable_irq(BCM59040_IRQ_PONKEY);

	return 0;
} /* bcm59040_control_suspend */

/** bcm59040_control_resume - BCM59040 control resume
 *
 * Restore actual INT1M to saved value (virtual copy).
 *
 * @pclient:	Pointer to client structure for this device.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 * @todo	Need to determine what suspend does for this driver still.
 *
 */

static int bcm59040_control_resume(struct pmu_client *pclient)
{
	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "resume processed for %s.\n",
						  pclient->name ));

	enable_irq(BCM59040_IRQ_PONKEY);

	return 0;
} /* bcm59040_control_resume */

#endif

struct pmu_device_id bcm59040_control_idtable[] = {
	{ "control" },
	{ }
};

/** bcm59040_control_driver - BCM59040 control driver registration data
 *
 */
struct pmu_driver bcm59040_control_driver =
{
	.driver =
	{
		.name	= BCM59040_CONTROL_DRIVER,
	},
	.probe		= &bcm59040_control_probe,
	.remove		= &bcm59040_control_remove,
#ifdef CONFIG_PM
	.suspend    = &bcm59040_control_suspend,
	.resume     = &bcm59040_control_resume,
#endif
	.id_table	= bcm59040_control_idtable,
};

/** bcm59040_control_init - BCM59040 control module Initialization
*
* Registers the BCM59040 control driver with the PMU device
* framework.
*
* Returns 0 if successful otherwise standard Linux error codes.
*
*/
static int __init bcm59040_control_init(void)
{
	return pmu_register_driver(&bcm59040_control_driver);
}

/** bcm59040_control_exit - BCM59040 control module Exit
*
* Deregisters the BCM59040 control driver with the PMU device
* framework.
*
* Returns 0 if successful otherwise standard Linux error codes.
*
*/

static void __exit bcm59040_control_exit(void)
{
	pmu_unregister_driver(&bcm59040_control_driver);
}


device_initcall(bcm59040_control_init);
module_exit(bcm59040_control_exit);

MODULE_DESCRIPTION(BCM59040_CONTROL_MOD_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_VERSION(BCM59040_CONTROL_MOD_VERSION);
