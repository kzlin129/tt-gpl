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

/** BCM59040 PMU Control Driver
 *****************************************************************************
 *
 *     This implements the BCM59040 battery maintenance layer. This layer
 *     implements functionality manage attached batteries. The number of
 *     batteries and their purpose is defined via platform specific
 *     structures and can be queried via the API implemented by this driver.
 *     The power supply class driver interface extensively with this driver
 *     to determine battery status such as charge/discharge state, remaining
 *     capacity, temperature, etc. This driver will use the standard
 *     power supply class properties to report this data wherever possible
 *     although the deliver mechanism may be different than the power
 *     supply class so the two do not interfere with each other.
 *
 *     This driver also calls the ADC driver framework to obtain ADC readings
 *     required in calculating battery information such as remaining capacity.
 *     This interface to the ADC driver framework is to the generic layer and
 *     not directly to the BCM59040 specific ADCs. This allows isolation from
 *     the specific ADC device and channel being used and allows use of, for
 *     example, BCM4760 ADCs when reason to do so exists.
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
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/interrupt.h>

#include <linux/platform_device.h>
#include <linux/pmu_device.h>
#include <asm/arch/pmu_device_bcm59040.h>
#include <asm/plat-bcm/pmu_device_bcm.h>

#include <linux/power_supply.h>

#include <linux/broadcom/pmu_bcm59040.h>
#include <linux/broadcom/bcm_adc.h>

#include <asm/atomic.h>

#include <asm/arch/bcm_gen_battery.h>
#include <asm/arch/bcm59040_irqs.h>
#include <linux/time.h>

/* ---- Structure and macro definitions ---------------------------------- */
#define BCM59040_BATT_DRIVER		"bcm59040_batt"
#define BCM59040_BATT_MOD_DESCRIPTION	"BCM59040 Battery Driver"
#define BCM59040_BATT_MOD_VERSION	1.0

#define PFX			BCM59040_BATT_DRIVER ": "

#define BAT_SAMPLES_AVG		60
#define BAT_MIN_SAMPLES		2

#define BCM59040_CHARGE_THRESHOLD	15000	/* 15 mA */
#define BCM59040_ADC_CURRENT_SCALING	976563
#define BCM59040_ADC_VOLTAGE_SCALING	6

/* ---- Logging ---------------------------------------------------------- */
#ifdef DEBUG
#undef DEBUG
#endif

#define DEBUG 1

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_HW_MEASURE	0x20

#ifdef DEBUG
 #define DBG_DEBUGGING_LEVEL	(DBG_ERROR | DBG_INFO | DBG_TRACE | DBG_TRACE2)
 #define DBG_HARDWARE_LEVEL	(DBG_ERROR | DBG_HW_MEASURE)
 #define DBG_DEFAULT_LEVEL	(DBG_ERROR)

 static int gLevel = DBG_DEFAULT_LEVEL;

 #define PMU_DEBUG(level,x) {if(level & gLevel) printk x;}
#else
 #define PMU_DEBUG(level,x)
#endif

struct atomic_adc_polled_average_value {
	spinlock_t	lock;
	s32		the_samples[BAT_SAMPLES_AVG];
	u32		num_sample;
	s32		current_average;
	u8    		buffer_wrapped;
};

typedef struct bcm_power_supply {
	struct power_supply				psy;

	struct bcm_battery_platform_info		*platform_info;

	struct pmu_client				*pclient;

	struct atomic_adc_polled_average_value		avg_batt_volt;
	struct atomic_adc_polled_average_value		avg_batt_curr;

	atomic_t					current_capacity;
	atomic_t					remaining_time;
	atomic_t					current_consumption;
	atomic_t					running;
	atomic_t					chrg_status;
	atomic_t					online;
	atomic_t					health;

	struct delayed_work				work;
	struct workqueue_struct*			workqueue;
} bcm_power_supply_t;


static int battery_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val);
static u32 battery_get_charge(struct bcm_power_supply *psy, u32 result_as_percent);

static enum power_supply_property bcm_battery_properties[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_MODEL_NAME
};

/** struct bcm59040_batt_config_struct - Driver data for BCM59040 Battery
 * @pclient:	Pointer to associated device.
 * @work:	Work generated from ISR.
 * @workqueue:	Work queue for foreground ISR processing of ADC interrupts.
 *
 */
typedef struct bcm59040_adc_config_struct {
	struct pmu_client		*pclient;
	struct work_struct		 work;
	struct delayed_work		 vbus_collapse_work;
	struct workqueue_struct		*vbus_collapse_workqueue;
	struct workqueue_struct		*workqueue;
} bcm59040_batt_config_t;

/** struct bcm59040_fuelgauge - Driver data for BCM59040 Fuel Guage */
typedef struct bcm59040_fuelguage {
	BCM59040_FuelGauge_State_t fg_state;
	int fg_calibrate_done;
	unsigned long fg_calibrate_done_time;
	short fg_offset;      	// value of offset; in 59040, offset can be positive or negative
	int fg_cont_mode;   	// 1 for continuous mode; 0 for sync mode
	short fg_sample;   	// in PMU register, bit 13 is sign bit.  We shift it to bit 15
} FG_Maintain_t;  		// in the future, can also add timer (used to dump FG registers periodically, e.g. 2 min)
				// and timer1 (used to delay for at least 50 msec after FGCAL to get FG offset registers)

/* ---- Local variables -------------------------------------------------- */
static struct bcm_power_supply bcm59040_battery_psy = {
	.psy = {
		.name			= "battery",
		.type			= POWER_SUPPLY_TYPE_BATTERY,
		.properties		= bcm_battery_properties,
		.num_properties		= ARRAY_SIZE(bcm_battery_properties),
		.get_property		= battery_get_property,
		.use_for_apm		= 1,
	},
	.platform_info			= NULL,
	.avg_batt_volt = {
		.lock			= SPIN_LOCK_UNLOCKED,
		.the_samples        	= {},
		.num_sample		= 0,
		.buffer_wrapped     	= 0,
	},
	.avg_batt_curr = {
		.lock			= SPIN_LOCK_UNLOCKED,
		.the_samples		= {},
		.num_sample 		= 0,
		.buffer_wrapped 	= 0,
	},
	.current_capacity		= ATOMIC_INIT(-1),
	.remaining_time			= ATOMIC_INIT(0),
	.current_consumption   		= ATOMIC_INIT(350),
	.running			= ATOMIC_INIT(0),
	.chrg_status			= ATOMIC_INIT(POWER_SUPPLY_STATUS_UNKNOWN),
	.online				= ATOMIC_INIT(0),
	.health				= ATOMIC_INIT(POWER_SUPPLY_HEALTH_GOOD),
};

static FG_Maintain_t fg_maintain;
static int suspended = 0;

atomic_t bcm59040_batt_status = ATOMIC_INIT( POWER_SUPPLY_HEALTH_GOOD );

/* Local function declarations */

static int bcm59040_get_fuelgauge_sample_reg(struct pmu_client *pclient, short *fgsmpl, int fast);
static int bcm59040_fuelgauge_clear_acc_cnt(struct pmu_client *pclient);
static int bcm59040_fuelgauge_calibrate(struct pmu_client *pclient);
static int bcm59040_fuelgauge_sample_to_integer(u8 *smpl);

/* External function declarations */

int bcm59040_get_charging_current(struct pmu_client *pclient);

/*
 * Function definitions.
 */

/** bcm59040_get_fuelgauge_sample_reg - Read fuel gauge sample registers.
 *
 * Read either FGSMPL1~2 sample A registers (slower update: every 500 msec) or
 * FGSMPLB1~2 sample B registers (faster update).Sample registers are 14-bit signed, 
 * with bit-13 being the sign bit.
 *
 * @client:		Pointer to device structure.
 * @fgsmpl:		sample value
 * @fast:		1 means fast mode, 0 means slow mode (every
 *
 * Returns 0 if successful, otherwise standard Linux error codes.
 *
 */
static int bcm59040_get_fuelgauge_sample_reg(struct pmu_client *pclient, short *fgsmpl, int fast)
{
	int		rc=0;
	int		smpl1, smpl2;
	u8		sample[2], fgsmpl_reg;
	short		tmp;

	PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "bcm59040_get_fuelgauge_sample_reg.\n"));

	if(!(fg_maintain.fg_calibrate_done)) {
		// check time elapsed since calibration; need to have at least 50 msec
		if(time_after(jiffies,fg_maintain.fg_calibrate_done_time)) {
			rc = pmu_bus_seqread(pclient, BCM59040_REG_FGTRIMGN1_1, &sample, 2);
			if (rc < 0) {
				PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_FGTRIMGN1_1 register.\n"));
				return -EINVAL;
			}
			//The fuel gauge trim value is 13 bits wide
			tmp = (sample[0] | (sample[1] << 8)) & BCM59040_FGOFFSET_MASK;
			//max positive number in a 13 bit signed variable = 8191
			if (tmp <= (BCM59040_FGOFFSET_MASK >>1)){ 
				fg_maintain.fg_offset = tmp;
			} else {
				//negative number, do 2's complement
				tmp = (~tmp + 1) & BCM59040_FGOFFSET_MASK;
				fg_maintain.fg_offset = -tmp;
			}
			fg_maintain.fg_calibrate_done = 1;
		} else {
			PMU_DEBUG(DBG_ERROR, (KERN_INFO PFX "PMU Fuel gauge calibration is not yet done.\n"));
			return -EINVAL;
		}
	}

	if (fast) {
		fgsmpl_reg = BCM59040_REG_FGSMPLB1;
	} else {
		fgsmpl_reg = BCM59040_REG_FGSMPL1;
	}

	if ((rc = pmu_bus_seqread(pclient, fgsmpl_reg, &sample, 2)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading FG sample register.\n"));
		return -EINVAL;
	}

	smpl1 = bcm59040_fuelgauge_sample_to_integer(sample);

	if((rc = pmu_bus_seqread(pclient, fgsmpl_reg, &sample, 2)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading FG sample register.\n"));
		return -EINVAL;
	}

	smpl2 = bcm59040_fuelgauge_sample_to_integer(sample);

	if( ((fast) && ((smpl2-smpl1)>50 || (smpl2-smpl1)<-50))  ||
		 ((!fast) && ((smpl2-smpl1)>2 || (smpl2-smpl1)<-2))) {
		if((rc = pmu_bus_seqread(pclient, fgsmpl_reg, &sample, 2)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading FG sample register.\n"));
			return -EINVAL;
		}

		smpl2 = bcm59040_fuelgauge_sample_to_integer(sample);
	}

	if(fg_maintain.fg_offset < 0) { //negative offset
		//make offset a positive number
		fg_maintain.fg_sample = smpl2 + (-fg_maintain.fg_offset);
		#ifdef PMU_59040_FG_LAYOUT
		smpl1 = fg_maintain.fg_sample;
		smpl1 = smpl1 * 50 / 57;   //1.14
		fg_maintain.fg_sample = smpl1;
		#endif
	} else {
		fg_maintain.fg_sample = smpl2 - fg_maintain.fg_offset;
		#ifdef PMU_59040_FG_LAYOUT
		smpl1 = fg_maintain.fg_sample;
		smpl1 = smpl1 * 20 / 23;   //1.15
		fg_maintain.fg_sample = smpl1;
		#endif
	}

	*fgsmpl = fg_maintain.fg_sample;

	PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "bcm59040_get_fuelgauge_sample_reg: sample = %d\n",
						 fg_maintain.fg_sample));

	return rc;
}

/** bcm59040_fuelgauge_enable - Enable fuelgauge.
 *
 * Start or stop Fuel Gauge
 *
 * @client:		Pointer to device structure.
 * @enable:		1 means enable, 0 means disable FG
 *
 * Returns 0 if successful, otherwise standard Linux error codes.
 *
*/
static int bcm59040_fuelgauge_enable(struct pmu_client *pclient, int enable )
{
	int			rc=0;
	uint8_t			data;

	PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "bcm59040_fuelgauge_enable\n"));

	/* read FGCTRL1 */
	if ((rc = pmu_bus_seqread(pclient, BCM59040_REG_FGCTRL1, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_FGCTRL1 register.\n"));
		return -EINVAL;
	}

	if (enable) {
		data |= BCM59040_FGCTRL1_FGHOSTEN;
		if(fg_maintain.fg_cont_mode) {
			fg_maintain.fg_state = BCM59040_FG_ON_CONT_MODE;
		} else {
			fg_maintain.fg_state = BCM59040_FG_ON_SYNC_MODE;
		}
	} else {
		data &= ~BCM59040_FGCTRL1_FGHOSTEN;
		fg_maintain.fg_state = BCM59040_FG_OFF;
	}

	if ((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_FGCTRL1, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_FGCTRL1 register.\n"));
		return rc;
	}

	return rc;
}

/** bcm59040_fuelgauge_init - Initialize fuelgauge hardware.
 *
 * @client:		Pointer to device structure.
 *
 * Returns 0 if successful, otherwise standard Linux error codes.
 *
*/
static int bcm59040_fuelgauge_init(struct pmu_client *pclient)
{
	bcm59040_client_platform_data_t *pdata;
	struct bcm59040_batt_defaults *defaults;
	int	rc = 0;
	uint8_t	data;

	if (pclient->dev.platform_data == NULL) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR "Error, platform data not initialized\n"));
		return -1;
	}

	pdata = (bcm59040_client_platform_data_t*)(pclient->dev.platform_data);
	if (pdata->defaults == NULL) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR "Error, default not initialized for battery\n"));
		return -1;
	}
        
	defaults = (struct bcm59040_batt_defaults *) pdata->defaults;

	/* configure FGCTRL registers for initializing some non-OTPable fields */
	data = defaults->fgctrl1;
	if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_FGCTRL1, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_FGCTRL1 register.\n"));
		return rc;
	}

	data = defaults->fgctrl3;
	if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_FGCTRL3, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_FGCTRL3 register.\n"));
		return rc;
	}

	/* initialize FG data structure */
	fg_maintain.fg_state = BCM59040_FG_OFF;
	if(BCM59040_PMU_REG_FGCTRL3_VAL & BCM59040_FGCTRL3_FGSYNCMODE) {
		fg_maintain.fg_cont_mode = 0;
	} else {
		fg_maintain.fg_cont_mode = 1;
	}

	fg_maintain.fg_sample = 0;
	fg_maintain.fg_calibrate_done = 0;

	bcm59040_fuelgauge_enable(pclient, 1);
	bcm59040_fuelgauge_clear_acc_cnt(pclient);	// clears Accum and Counter registers before starting
	bcm59040_fuelgauge_calibrate(pclient);

	return 0;
}

/** bcm59040_fuelgauge_clear_acc_cnt - Clear fuelgauge accumulator
 *
 * @client:		Pointer to device structure.
 *
 * Returns 0 if successful, otherwise standard Linux error codes.
 *
*/
static int bcm59040_fuelgauge_clear_acc_cnt(struct pmu_client *pclient)
{
	int		rc=0;
	uint8_t		data;

	PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "bcm59040_fuelgauge_clear_acc_cnt.\n"));

	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_FGCTRL3, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Error reading BCM59040_REG_FGCTRL3 register.\n"));
		return -EINVAL;
	}
	data = BCM59040_FGCTRL3_FGFRZREAD | (unsigned char)data;

	if((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_FGCTRL3, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Error writing BCM59040_REG_FGCTRL3 register.\n"));
		return rc;
	}

	return rc;
}

/** bcm59040_fuelgauge_calibrate - Calibrate fuelgauge.
 *
 * Perform calibration of fuelgauge.
 *
 * @client:		Pointer to device structure.
 *
 * Returns 0 if successful, otherwise standard Linux error codes.
 *
*/
static int bcm59040_fuelgauge_calibrate(struct pmu_client *pclient)
{
	int	rc = 0;
	uint8_t	data;

	PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "bcm59040_fuelgauge_calibrate.\n"));

	if ((rc = pmu_bus_seqread(pclient, BCM59040_REG_FGCTRL3, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Error reading BCM59040_REG_FGCTRL3 register.\n"));
		return -EINVAL;
	}

	data = BCM59040_FGCTRL3_FGCAL | (unsigned char)data;
	if ((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_FGCTRL3, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_FGCTRL3 register.\n"));
		return rc;
	}

	fg_maintain.fg_calibrate_done = 0;
	/* wait for 70 msec before checking calibration offset */
	fg_maintain.fg_calibrate_done_time = jiffies + HZ/14; 

	return rc;
}

/** bcm59040_fuelgauge_sample_to_integer - Convert fuelgauge sample to an integer.
 *
 * Convert the provided fuelgauge sample value into an integer result.
 *
 * @client:		Pointer to device structure.
 * @enable:		1 means enable, 0 means disable FG
 *
 * Returns 0 if successful, otherwise standard Linux error codes.
 *
*/
static int bcm59040_fuelgauge_sample_to_integer(u8 *smpl)
{
	int	rc = 0;
	u16	tmp;

	tmp = (smpl[0] | (smpl[1] << 8)) & BCM59040_FGSAMPLE_MASK;

	if(tmp & BCM59040_FGSAMPLE_SIGN_BIT) { //negative sample value
		rc = -((~tmp + 1) & BCM59040_FGSAMPLE_MASK);
	} else {
		rc = tmp;
	}

	return rc;
}

#if 0
/** bcm59040_startfuel_gauge_sample - Enable fuelgauge to start sample process
 *
 * Start fuelgauge sampling process. Come back and read later via
 *
 * @client:		Pointer to device structure.
 * @enable:		1 means enable, 0 means disable FG
 *
 * Returns 0 if successful, otherwise standard Linux error codes.
 *
*/
static int bcm59040_start_fuelgauge_sample(struct pmu_client *pclient)
{
	bcm59040_fuelgauge_enable(pclient,1);

	return 0;
}
#endif

/** bcm59040_get_fuelgauge_sample - Enable fuelgauge to start sample process
 *
 * Start fuelgauge sampling process. Come back and read later via
 *
 * @client:		Pointer to device structure.
 *
 * Returns 0 if successful, otherwise standard Linux error codes.
 *
*/
static s16 bcm59040_get_fuelgauge_sample(struct pmu_client *pclient)
{
	short sample = 0;

	bcm59040_get_fuelgauge_sample_reg(pclient, &sample, 1);

	return (s16)sample;
}

/** bcm59040_batt_work - Work function for BCM59040 battery IRQ
 * @work:   Pointer to work structure.
 *
 */
static struct timespec	bcm59040_previous_eoc={0,NSEC_PER_SEC};
static int		bcm59040_eoc_count=0;

void vbus_collapse_work_handler(struct work_struct *vbus_collapse_work)
{
	uint8_t			pdcmpsyn1, mbcctrl9;
	int			rc;
	bcm59040_batt_config_t	*batt_config;
	struct pmu_client	*pclient;
	struct delayed_work	*vbus_collapse_delayed_work = (struct delayed_work*)vbus_collapse_work;

	batt_config = (bcm59040_batt_config_t *)container_of(vbus_collapse_delayed_work, bcm59040_batt_config_t,
							 vbus_collapse_work);
	pclient = batt_config->pclient;

	PMU_DEBUG(DBG_HW_MEASURE, ( "VBUS collapse detected\n"));

	if (!pclient) {
		return;
	}

	if ((rc = pmu_bus_seqread(pclient, BCM59040_REG_PDCMPSYN1, &pdcmpsyn1, 1)) < 0) {
		PMU_DEBUG (DBG_ERROR, (KERN_ERR "Error reading BCM59040_REG_PDCMPSYN1\n"));
		return;
	}

	if ((pdcmpsyn1 & (1 << 3)) != 0) {
		PMU_DEBUG(DBG_HW_MEASURE, ( "Setting the current back up after fault condition is gone\n"));

		rc = pmu_bus_seqread(pclient, BCM59040_REG_MBCCTRL9, &mbcctrl9, 1);
		BUG_ON (rc < 0);

		mbcctrl9|= BCM59040_MBCCTRL9_USB_ILIMIT_900MA;

		rc = pmu_bus_seqwrite(pclient, BCM59040_REG_MBCCTRL9, &mbcctrl9, 1);
		BUG_ON (rc < 0);
	} else {
		queue_delayed_work(batt_config->vbus_collapse_workqueue, &batt_config->vbus_collapse_work, HZ * 5);
	}
}

static int bcm59040_batt_health(struct bcm_power_supply *bcmpsy)
{
	struct pmu_client	*pclient=bcmpsy->pclient;
	uint8_t			data[3];
	int			health=atomic_read( &bcmpsy->health );
	int			next_health=health;

	/* Get the battery status. */
	if (pmu_bus_seqread(pclient, BCM59040_REG_ENV3, data, sizeof( data )) < 0)
	{
		PMU_DEBUG (DBG_ERROR, (KERN_ERR "Error reading BCM59040_REG_ENV3. Battery status may be invalid.\n" ));
		return -1;
	}

	/* Check the bits. */
	/* Battery temperature too high condition. */
	if( data[0] & BCM59040_ENV3_MBTEMPHIGH )
		next_health=POWER_SUPPLY_HEALTH_OVERHEAT;

	/* Battery temperature too low condition. */
	else if( data[0] & BCM59040_ENV3_MBTEMPLOW )
		next_health=POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;

	/* Overvoltage condition. */
	else if( data[2] & BCM59040_ENV5_MBOV )
		next_health=POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	/* General battery error. We don't know what. */
	else if( data[2] & BCM59040_ENV5_MBC_ERR )
		next_health=POWER_SUPPLY_HEALTH_UNKNOWN;

	/* The default. If there is no bad condition, health must be good. */
	else if( health != POWER_SUPPLY_HEALTH_GOOD )
		next_health=POWER_SUPPLY_HEALTH_GOOD;

	/* Only inform on health change. */
	if( health != next_health )
	{
		atomic_set( &bcmpsy->health, next_health );
		return 1;
	}

	return 0;
}

static void bcm59040_batt_work(struct work_struct *work)
{
	struct  bcm_power_supply *bcmpsy;
	bcm59040_batt_config_t	 *batt_config;
	struct pmu_client	 *pclient;
	struct timespec		 eoc_time;
	uint8_t	data[3], intbit;
	int	rc;

	batt_config = (bcm59040_batt_config_t *)container_of(work, bcm59040_batt_config_t, work);
 	intbit = 1 << BCM59040_VINT1_BATT_BIT;
	bcmpsy = &bcm59040_battery_psy;
	pclient = batt_config->pclient;

	if ((rc = pmu_bus_seqread(pclient, BCM59040_REG_INT3, &(data[0]), 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Error reading BCM59040_REG_VINT3\n"));

		goto out_err;
	}

	if ((rc = pmu_bus_seqread(pclient, BCM59040_REG_INT6, &(data[1]), 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Error reading BCM59040_REG_VINT6\n"));

		goto out_err;
	}

	rc = pmu_bus_seqread(pclient, BCM59040_REG_INT2, &(data[2]), 1);
	if (rc < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Error reading BCM59040_REG_VINT2\n"));

		goto out_err;
	}

	/* we got the data, clear the interrupt */
	pmu_bus_seqwrite(pclient, BCM59040_REG_VINT1, &intbit, 1 );

	if (data[0] & BCM59040_INT3_VBUSLOWBND) {
		queue_delayed_work(batt_config->vbus_collapse_workqueue, &batt_config->vbus_collapse_work, HZ * 5);
		return;
	}

	/* Check workaround for dead battery. */
	if( data[2] & BCM59040_INT2_EOC )
	{
		getnstimeofday( &eoc_time );
		if( timespec_valid( &bcm59040_previous_eoc ) )
		{
			struct timespec	delta;
			delta=timespec_sub( eoc_time, bcm59040_previous_eoc );
			if( timespec_to_ns( &delta ) < NSEC_PER_SEC )
			{
				/* Another EOC within one second? This is odd. */
				bcm59040_eoc_count+=1;

				/* Check if this happened 5 times. If so, then we have the error condition. */
				if( bcm59040_eoc_count > 5 )
				{
					uint8_t mbcctrl3;

					/* Mark the battery as dead. */
					atomic_set( &bcmpsy->health, POWER_SUPPLY_HEALTH_DEAD );
					PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Battery is dead!\n"));

					/* Turn off the maintenance charge, or we'd have led blinking. */
					/* Note that when USB is reinserted, this will change. */
					if( pmu_bus_seqread( pclient, BCM59040_REG_MBCCTRL3, &mbcctrl3, 1 ) < 0 ) {
						PMU_DEBUG( DBG_ERROR, (KERN_ERR PFX "Can't read Maintenance charge in MBCCTRL3!\n" ) );
						return;
					}

					mbcctrl3&=~BCM59040_MBCCTRL3_MAINGCHRG;
					if( pmu_bus_seqwrite( pclient, BCM59040_REG_MBCCTRL3, &mbcctrl3, 1 ) < 0 ) {
						PMU_DEBUG( DBG_ERROR, (KERN_ERR PFX "Can't disable Maintenance charge in MBCCTRL3!\n" ) );
						return;
					}

					/* Reset count back to 0. This IRQ shouldn't happen anymore as maintenance charge is gone. */
					bcm59040_eoc_count=0;
					queue_work(bcmpsy->workqueue, &bcmpsy->work.work);
				}
			}
		}

		/* Copy the time. Since the time elapsed since the previous EOC is long enough that we */
		/* don't have the error condition, clear the error count. */
		bcm59040_previous_eoc = eoc_time;
	}

	return;

out_err:
	/* clear the interrupt */
	pmu_bus_seqwrite(pclient, BCM59040_REG_VINT1, &intbit, 1 );
	return;
}

/** bcm59040_batt_isr - Interrupt service routine for BCM59040 Battery
 *
 * This ISR handles events for the battery in the BCM59040. The primary
 * event we're interested in is the
 * @irq:    IRQ vector number
 * @dev_id: Data structure associated with this interrupt event.
 * 
 */
static irqreturn_t bcm59040_batt_isr(int irq, void *dev_id)
{
	bcm59040_batt_config_t	*batt_config = dev_id;

	queue_work(batt_config->workqueue, &batt_config->work);

	return IRQ_HANDLED;
}

static ssize_t battery_show_current(struct device *dev, struct device_attribute *attr, char *buffer)
{
	snprintf(buffer, PAGE_SIZE, "%d",
			 atomic_read(&bcm59040_battery_psy.current_consumption));
	return strlen(buffer);
}

static ssize_t battery_store_current(struct device *dev,
				     struct device_attribute *attr,
				     const char *buffer, size_t size)
{
	long new_current_consumption = simple_strtol(buffer, NULL, 10);
	if(new_current_consumption) {
		atomic_set(&bcm59040_battery_psy.current_consumption,
				   new_current_consumption);
		return size;
	}

	return -EINVAL;
}

static DEVICE_ATTR(current_consumption, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
				   battery_show_current, battery_store_current);

static inline void update_average_atomic_adc_value(struct atomic_adc_polled_average_value *arg)
{
	u32 i;
	s32 result = 0; 
	u32 sample_count;
	
	sample_count = (arg->buffer_wrapped) ? BAT_SAMPLES_AVG : arg->num_sample;

	for (i = 0; i < sample_count; i++) {
		result += arg->the_samples[i];
	}

	if (sample_count != 0) {
		result /= (s32)sample_count;
		arg->current_average = result;
	}
}

static inline void reset_atomic_average(struct atomic_adc_polled_average_value *arg)
{
	unsigned long	flags;

	spin_lock_irqsave(&(arg->lock), flags);
	arg->num_sample = 0;
	arg->buffer_wrapped = 0;
	arg->current_average = 0;
	memset(arg->the_samples, 0, BAT_SAMPLES_AVG);
	spin_unlock_irqrestore(&(arg->lock), flags);
}

static inline void add_atomic_adc_value(struct atomic_adc_polled_average_value *arg, s32 value)
{
	unsigned long flags;

	spin_lock_irqsave(&(arg->lock), flags);
	arg->the_samples[arg->num_sample] = value;

	arg->num_sample++;
	if (arg->num_sample == BAT_SAMPLES_AVG) {
		arg->num_sample = 0;
		arg->buffer_wrapped = 1;
	}

	update_average_atomic_adc_value(arg);
	spin_unlock_irqrestore(&(arg->lock), flags);
}

static inline s32 get_average_atomic_adc_value(struct atomic_adc_polled_average_value *arg)
{
	unsigned long	flags;
	s32		retval;

	spin_lock_irqsave(&(arg->lock), flags);
	retval = arg->current_average;
	spin_unlock_irqrestore(&(arg->lock), flags);

	return retval;
}

static inline u32 get_last_atomic_adc_value(struct atomic_adc_polled_average_value *arg)
{
	unsigned long	flags;
	u32		retval;

	spin_lock_irqsave(&(arg->lock), flags);
	retval = arg->the_samples[arg->num_sample == 0 ? BAT_SAMPLES_AVG - 1 : arg->num_sample - 1];
	spin_unlock_irqrestore(&(arg->lock), flags);
	return retval;
}

/* Return: y0 */
static inline long linear_interpolate(long x0,  long x1, long y1, long x2, long y2)
{
	int64_t y0;
	long div, sign = 0;

	if (x1 == x2)
		return y1;

	y0 = (x0 - x1);
	y0 *= (y2 - y1);

	if(y0 < 0) 	{
		y0 = -y0;
		sign = !sign;
	}

	div = x2 - x1;

	if(div < 0) {
		div = -div;
		sign = !sign;
	}

	do_div(y0, div); // This works only for positive numbers, tweak needed

	if(sign)
		y0 = -y0;

	y0 += y1;

	return (long)y0;
}


/** convert_volts_to_capacity - voltage to capacity conversion
 *
 * Converts the battery voltage into a capacity measurement using
 * precalculated look up tables: we first select the correct 2 tables
 * based on current consumption, then evaluate capacity/remaining time
 * based on battery voltage.
 *
 * @battery_volts:		Current battery voltage
 * @result_as_percent:	1 = return in percent, 0 = return in seconds
 *
 * Returns capacity in percent or seconds.
 *
 */
static u32 convert_volts_to_capacity(u32 battery_volts, int result_as_percent)
{
	batt_discharge_lut_t	*entry1, *entry2, *curves;
	struct bcm_batt_info	*batt_info;
	u32			i, cap1, cap2;

	batt_info = bcm59040_battery_psy.platform_info->battery_info;

	if (batt_info->battery->discharge_curve_entries == 0)
		return 0;

	curves = batt_info->battery->discharge_curves;

	/* Find the two nearest points of the curve */
	entry1 = entry2 = NULL;
	for (i = 0; i < batt_info->battery->discharge_curve_entries; i++) {
		if (battery_volts > curves[i].vbatt) {
			entry1 = &(curves[i]);
			if (i > 0) {
				entry2 = &(curves[i - 1]);
			}
			break;
		}
	}

	if (entry1 == NULL)
		entry1 = &(curves[batt_info->battery->discharge_curve_entries - 1]);

	if (result_as_percent)
		cap1 = entry1->remaining_capacity;
	else
		cap1 = entry1->remaining_time;

	if(entry2) {
		if(result_as_percent)
			cap2 = entry2->remaining_capacity;
		else
			cap2 = entry2->remaining_time;
	} else {
		cap2 = cap1;
		entry2 = entry1;
	}

	/* Do a linear interpolation between the two values */
	return linear_interpolate(battery_volts, entry1->vbatt, cap1, entry2->vbatt, cap2);

	/* we don't do the interpolation, we now want capacity in steps of 5%
	   but maybe in the future when we can select from different curves
	   based on load avarage we might need it again*/
	//return curve1->remaining_capacity;
}

#if 0
/** getbatteryesr - find battery ESR based upon current voltage
 *
 * Uses precalcuated curves and linear interpolation to find
 * correct ESR values given the supplied battery voltage.
 *
 * @vmbatt:		supplied battery voltage in uV
 *
 * Returns ESR value.
 *
 */
static u32 getbatteryesr(u32 vmbat)
{
	batt_esr_curve_t	*esr_curve1, *esr_curve2, *esr_curves;
	struct bcm_batt_info	*batt_info;
	u32			esr1, esr2, i;

	batt_info = bcm59040_battery_psy.platform_info->battery_info;

	if(batt_info->battery->esr_curve_entries == 0)
		return 1;

	esr_curves = batt_info->battery->esr_curves;

	/* Find the two nearest curves */
	esr_curve1 = esr_curve2 = NULL;
	for(i = 0; i < batt_info->battery->esr_curve_entries; i++) {
		if(vmbat > esr_curves[i].vm_bat) {
			esr_curve1 = &(esr_curves[i]);
			if(i > 0)
				esr_curve2 = &(esr_curves[i - 1]);
			break;
		}
	}

	if(esr_curve1 == NULL)
		esr_curve1 = &(esr_curves[batt_info->battery->esr_curve_entries - 1]);

	esr1 = esr_curve1->esr;

	if(esr_curve2) {
		esr2 = esr_curve2->esr;
	} else {
		return esr1;
	}

	/* Do a linear interpolation between the two values */
	return linear_interpolate(vmbat, esr_curve1->vm_bat, esr1, esr_curve2->vm_bat, esr2);

}
#endif

/** batteryadc2voltage - Calculate adjusted battery voltage
 *
 * Calculate the current battery voltage using an ADC reading
 * of the voltage adjusted by the fuel gauge reading. The fuel
 * gauge reading corrects for offset in the measured voltage
 * due to charging or discharging the battery.
 *
 * @bcmpsr:	pointer to battery structure
 *
 * Returns adjusted battery voltage in uV.
 *
 */
static u32 batteryadc2voltage(bcm_power_supply_t *bcmpsy)
{	
	struct bcm_battery_platform_info *bcm_plat_info;
	u64 hpval;

	bcm_plat_info = bcmpsy->platform_info;

	hpval = (u64)(bcm_plat_info->adc_read(BCM59040_ADC_DEVICE, VMBAT));

	if (hpval == 0)
		return 0;

	hpval *= BCM59040_ADC_VOLTAGE_SCALING;
	hpval *= 1000000;
	hpval >>= 10;

	return ((u32)hpval);
}


/** batteryadc2current - battery charging current from setting
 *
 * Get charging current from the fuelthing.
 *
 * @bcmpsy:	pointer to battery structure
 * @charging:	pointer to charging state to be returned (1=charging)
 *
 * Returns set charging current in uA.
 *
 */
static s32 batteryadc2current(bcm_power_supply_t *bcmpsy)
{
	s32 	sample;
	u64 	hpval;
	u8	charging;

	charging = 0;
	sample = -(s32)bcm59040_get_fuelgauge_sample(bcmpsy->pclient);

	if (sample == 0 ) {
		return 0;
	}

	/* Current is going into the battery so charging */
	if (sample < 0 ) {
		sample = -sample;
		charging = 1;
	}

	hpval = ((u64) sample) * BCM59040_ADC_CURRENT_SCALING;
	hpval *= 1000000;
        do_div(hpval, 1000000000);

	sample = (s32) hpval;

	if (charging) {
		sample=-sample;
	}

	return sample;
}

static int battery_has_samples(struct bcm_power_supply *bcm_psy)
{
	struct atomic_adc_polled_average_value	*arg;
	unsigned long				flags;
	int					has_samples = 0;

	arg = &bcm_psy->avg_batt_volt;

	spin_lock_irqsave(&(arg->lock), flags);

	if (arg->buffer_wrapped || arg->num_sample >= BAT_MIN_SAMPLES) {
		has_samples = 1;
	}

	spin_unlock_irqrestore( &(arg->lock), flags );	

	return has_samples;
}

static u32 battery_get_charge(struct bcm_power_supply *bcmpsy, u32 result_as_percent)
{
	/* [CORKER-677] this function shouldn't block, at least not indefinitely,
	as it can run from the shared kernel workqueue */
	if (result_as_percent)
		return atomic_read(&bcmpsy->current_capacity);
	else
		return atomic_read(&bcmpsy->remaining_time);
}

static int battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val )
{
	struct bcm_power_supply *bcmpsy;

	bcmpsy = (struct bcm_power_supply *)container_of(psy, struct bcm_power_supply, psy);

	switch(psp) {
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:

			if (atomic_read( &bcmpsy->chrg_status ) == POWER_SUPPLY_STATUS_CHARGING) {
				/* time is invalid while charging */
				val->intval = 1;
			} else {
				val->intval = battery_get_charge(bcmpsy, 0);
			}
			break;
			
		case POWER_SUPPLY_PROP_CAPACITY:/* added to support APM code */
			val->intval = battery_get_charge(bcmpsy, 1);
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			val->intval = 100;
			break;
		case POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN:
			val->intval = 0;
			break;

		case POWER_SUPPLY_PROP_PRESENT :
			val->intval = 1;
			break;

		case POWER_SUPPLY_PROP_ONLINE :
			val->intval = atomic_read(&bcmpsy->online);
			break;

		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = atomic_read( &bcmpsy->health);
			break;

		case POWER_SUPPLY_PROP_STATUS:
			val->intval = atomic_read(&bcmpsy->chrg_status);
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = get_last_atomic_adc_value(&bcmpsy->avg_batt_volt);
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_AVG:
			val->intval = get_average_atomic_adc_value(&bcmpsy->avg_batt_volt);
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = get_last_atomic_adc_value(&bcmpsy->avg_batt_curr);
			break;

		case POWER_SUPPLY_PROP_CURRENT_AVG:
			val->intval = get_average_atomic_adc_value(&bcmpsy->avg_batt_curr);
			break;

		case POWER_SUPPLY_PROP_MODEL_NAME:
			val->strval = bcmpsy->platform_info->battery_info->battery->model;
		  	break;

		default :
			return -EINVAL;
	}

	return 0;
}

/** bcm59040_batt_charger_is_inserted - True if charger is inserted currently.
 *
 * This function will return TRUE if either charger is inserted
 *
 * @pclient:		Pointer to PMU Client structure.
 * @chargerID:		Selects USB or wall charger.
 *
 * Returns 0 if false, 1 if true otherwise standard Linux error codes.
 *
 */
int bcm59040_batt_charger_is_inserted(struct pmu_client *pclient, int *chargerID)
{
	struct bcm_power_supply *bcmpsy;
	int		rc;
	uint8_t		data;
	uint8_t		mbcctrl3;

	bcmpsy = &bcm59040_battery_psy;

	if ((rc = pmu_bus_seqread(pclient, BCM59040_REG_ENV1, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_ENV1 register.\n"));
		return -ENODEV;
	}

	if (data != 0xFF) {

		if (data & BCM59040_ENV1_CGPD) {
			if (chargerID != NULL) {
				*chargerID = BCM59040_CHARGER_MAIN;
				return 1;
			}
		}

		if (data & BCM59040_ENV1_UBPD) {
			if (chargerID != NULL) {
				*chargerID = BCM59040_CHARGER_USB;
				return 1;
			}
		}

		/* Since we remove the usb, restore battery state to good. If we can still run on it */
		/* battery status HAS to be good. */
		if (pmu_bus_seqread( pclient, BCM59040_REG_MBCCTRL3, &mbcctrl3, 1) < 0 ) {
			PMU_DEBUG( DBG_ERROR, (KERN_ERR PFX "Can't read Maintenance charge in MBCCTRL3!\n" ) );
			return -1;
		}

		mbcctrl3|=BCM59040_MBCCTRL3_MAINGCHRG;
		if (pmu_bus_seqwrite( pclient, BCM59040_REG_MBCCTRL3, &mbcctrl3, 1) < 0 ) {
			PMU_DEBUG( DBG_ERROR, (KERN_ERR PFX "Can't enable Maintenance charge in MBCCTRL3!\n" ) );
			return -1;
		}
	} 

	return 0;
}

static int battery_update_capacity(struct bcm_power_supply *a_bcm_psy, s16 a_charging_status)
{
	u32 	averaged_batt_voltage;
	u32	previous_cap, cap, rem, health;
	int	should_update = 0;

	averaged_batt_voltage = get_average_atomic_adc_value(&a_bcm_psy->avg_batt_volt);

	previous_cap = atomic_read(&a_bcm_psy->current_capacity);

	health = atomic_read(&a_bcm_psy->health);
	if (health == POWER_SUPPLY_HEALTH_DEAD) {
		cap = 0;
		if (cap != previous_cap)
			should_update = 1;
	} else {
		/* We compute capacity at every ~1 sec poll and cache the value
		 * for client to read for performance reasons */
		cap = convert_volts_to_capacity(averaged_batt_voltage, 1);

		/* prevent sending to much apm events to userland */
		if (a_charging_status) {
			if (cap > previous_cap)
				should_update = 1;
		} else {
			if (cap < previous_cap)
				should_update = 1;
		}
		if (previous_cap == -1) {
			should_update = 1;
		}

	}

	if (should_update) {

		PMU_DEBUG(DBG_HW_MEASURE, ( "updating capacity\n"));

		atomic_set(&a_bcm_psy->current_capacity, cap);

		rem = convert_volts_to_capacity(averaged_batt_voltage, 0);
		atomic_set(&a_bcm_psy->remaining_time, rem);
	}

	return should_update;
}

/** battery_poll - work function to poll for battery status changes
 *
 * Poll periodically to update battery/charging status. Reads
 * battery boltage, charge and consumed current and charging status.
 * Updates averages for measured values. Notifies Linux power supply
 * infrastructure of changes.
 *
 * work:		pointer to work structure.
 *
 */
static void battery_poll(struct work_struct *work)
{
	struct  bcm_power_supply *bcmpsy;
	s32	battery_current, averaged_current;
	s32	battery_voltage;
	s16	charging_state;
	u32	online_state, online_state_prev;
	int	chrg_status, chrg_status_prev;
	int	charger_id;
	int	capacity_updated = 0;
	int	health_change = 0;

	bcmpsy = &bcm59040_battery_psy;

	PMU_DEBUG(DBG_TRACE,( "Entering battery_poll\n" ));

	if (suspended) {
		if (atomic_read(&bcmpsy->running)) {
			queue_delayed_work(bcmpsy->workqueue, &bcmpsy->work, BATT_POLL_INTERVAL);
		}

		return;
	}

	/* Is used to see if we're actually charging, this is then used in 
	 * battery_update_capacity() to minimize userland power_supply_changed() 
	 * updates related to capacity changes */
	charging_state = 0;

	/* This only indicates that we're on- or offline
	 * NOTE: We moved this piece of code up here because there are in some 
	 * situations in which we can actually read zero capacity from userspace  */
	online_state_prev = atomic_read(&(bcmpsy->online));
	online_state = bcm59040_batt_charger_is_inserted(bcmpsy->pclient, &charger_id);

	PMU_DEBUG (DBG_HW_MEASURE, ("charger state: %s\n", online_state ? "on-line" : "off-line"));

	/* Verify if our battery health has changed. Note that once the battery is dead it remains dead. */
	if( atomic_read( &bcmpsy->health ) != POWER_SUPPLY_HEALTH_DEAD )
	{
		health_change=bcm59040_batt_health(bcmpsy);
		switch( health_change )
		{
			case 0:
				/* No change. */
				break;
			case 1:
				/* Health change!. Also ensure our averages get cleared as they are no longer valid. */
				reset_atomic_average(&bcmpsy->avg_batt_curr);
				reset_atomic_average(&bcmpsy->avg_batt_volt);
				break;
			default:
				PMU_DEBUG( DBG_HW_MEASURE, ("Error reading battery health. Status may be inaccurate.\n"));
				health_change=0;
				break;
		}
	}

	if (online_state != online_state_prev) {
		atomic_set(&(bcmpsy->online), online_state);
		reset_atomic_average(&bcmpsy->avg_batt_curr);
		reset_atomic_average(&bcmpsy->avg_batt_volt);
	}

	/* get new measurements and update administration */
	battery_current = batteryadc2current(bcmpsy);
	battery_voltage = batteryadc2voltage(bcmpsy);

	add_atomic_adc_value(&bcmpsy->avg_batt_curr, battery_current);
	averaged_current = get_average_atomic_adc_value(&bcmpsy->avg_batt_curr);
	if (averaged_current < -BCM59040_CHARGE_THRESHOLD) {
		charging_state = 1;
		PMU_DEBUG(DBG_HW_MEASURE, ("charging status = %d\n", charging_state ));
	}

	/* battery_voltage == 0 sometimes happens during resume */
	if (battery_voltage > 0) {
		add_atomic_adc_value(&bcmpsy->avg_batt_volt, battery_voltage);

		/* Update the capacity */
		capacity_updated = battery_update_capacity(bcmpsy, charging_state);
	}

	PMU_DEBUG(DBG_HW_MEASURE, ("current(ist)=%u voltage(ist)=%u\n", battery_current, battery_voltage ));

	chrg_status_prev = atomic_read(&bcmpsy->chrg_status);
	if ((averaged_current < BCM59040_CHARGE_THRESHOLD &&
	     averaged_current > -BCM59040_CHARGE_THRESHOLD))
	{
		int capacity = atomic_read(&bcmpsy->current_capacity);
		if (capacity == 100)
			chrg_status = POWER_SUPPLY_STATUS_FULL;
		else
			chrg_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	} else if (averaged_current >= BCM59040_CHARGE_THRESHOLD) {
		chrg_status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		/* averaged_current <= -BCM59040_CHARGE_THRESHOLD */
		chrg_status = POWER_SUPPLY_STATUS_CHARGING;
	}
	atomic_set(&bcmpsy->chrg_status, chrg_status);

	if (capacity_updated || health_change ||
	    (online_state != online_state_prev) ||
	    (chrg_status != chrg_status_prev)) {
		power_supply_changed(&bcmpsy->psy);
	}

	if (atomic_read(&bcmpsy->running)) {
		queue_delayed_work(bcmpsy->workqueue, &bcmpsy->work, BATT_POLL_INTERVAL);
	}
}

/** battery_probe - probe function for Linux power supply driver
 *
 * Initializes the work queue, work function, registers us with the
 * Linux power supply driver infrastructure and generally kicks things
 * into motion. This is initiated by the PMU battery driver which
 * has finished initializing first.
 *
 * @pdev:	pointer to platform device structure
 *
 * Returns Linux error code value or 0 for successful.
 *
 */
static int battery_probe(struct platform_device *pdev)
{
	struct bcm_battery_platform_info *bcm_platinfo;
	struct bcm_batt_info *bcm_battinfo;

	bcm_platinfo = (struct bcm_battery_platform_info *)pdev->dev.platform_data;
	bcm_battinfo = bcm_platinfo->battery_info;

	/* require a decent string */
	BUG_ON(bcm_battinfo->battery->model == NULL);
	BUG_ON(bcm_battinfo->battery->model[0] == 0);

	/* Initialize platform dependant stuff. */
	if (bcm_platinfo->platform_init(pdev))
		return -EINVAL;

	/* Register the battery. */
	bcm59040_battery_psy.platform_info = bcm_platinfo;
	if (power_supply_register(&pdev->dev, &bcm59040_battery_psy.psy))
		return -EINVAL;

	/* Setup the workqueue. Needs to be done before calling
	   battery_state_changed */
	bcm59040_battery_psy.workqueue = create_workqueue(BCM_BATTERY_DRIVER);
	INIT_DELAYED_WORK(&bcm59040_battery_psy.work, battery_poll);

	if (device_create_file(&pdev->dev, &dev_attr_current_consumption)) {
		printk(KERN_ERR PFX
			"unable to create the 'current_consumption' "
			"attribute of the battery driver.\n");
		power_supply_unregister(&bcm59040_battery_psy.psy); 
		return -EINVAL;
	}
		
	/* Start up the battery polling work Q asap. */
	atomic_set(&bcm59040_battery_psy.running, 1);

	/* make sure everything has at least something in it */
	atomic_set(&bcm59040_battery_psy.online, -1);
	atomic_set(&bcm59040_battery_psy.current_capacity, -1);
	battery_poll(&bcm59040_battery_psy.work.work);

	return 0;
}

static int battery_remove(struct platform_device *pdev)
{
	struct bcm_battery_platform_info *bcm_platinfo;

	bcm_platinfo = (struct bcm_battery_platform_info *)pdev->dev.platform_data;

	/* Cancel polling work Q. */
	atomic_set(&bcm59040_battery_psy.running, 0);
	cancel_delayed_work(&bcm59040_battery_psy.work);
	flush_workqueue(bcm59040_battery_psy.workqueue);
	destroy_workqueue(bcm59040_battery_psy.workqueue);

	power_supply_unregister(&bcm59040_battery_psy.psy);

	bcm_platinfo->platform_remove(pdev);
	return 0;
}

#ifdef CONFIG_PM

/** bcm59040_battery_suspend - suspend handler for battery driver
 *
 * Stop workqueue making sure to flush it to clear any
 * active or pending work.
 *
 * @dev:		pointer to device structure.
 *
 * 0 for successful or Linux error code otherwise.
 *
 */
static int bcm59040_battery_suspend(struct device *dev)
{
	struct platform_device *pdev;

	pdev = to_platform_device(dev);

	atomic_set(&bcm59040_battery_psy.running, 0);
	cancel_delayed_work_sync(&bcm59040_battery_psy.work);
	flush_workqueue(bcm59040_battery_psy.workqueue);

	PMU_DEBUG(DBG_TRACE, ("Suspend processed for %s.\n", pdev->name ));

	return 0;
} /* bcm59040_battery_suspend */

/** bcm59040_battery_resume - resume handler for battery driver
 *
 * Start workqueue to start polling for battery status changes again.
 *
 * @dev:		pointer to device structure.
 *
 * 0 for successful or Linux error code otherwise.
 *
 */
static int bcm59040_battery_resume(struct device *dev)
{
	struct platform_device *pdev;

	pdev = to_platform_device(dev);

	atomic_set(&bcm59040_battery_psy.running, 1);

	/* make sure everything has at least something in it */
	atomic_set(&bcm59040_battery_psy.online, -1);
	atomic_set(&bcm59040_battery_psy.current_capacity, -1);

	/* Initially, assume the battery is OK again. */
	atomic_set(&bcm59040_battery_psy.health, POWER_SUPPLY_HEALTH_GOOD);
	battery_poll(&bcm59040_battery_psy.work.work);

	PMU_DEBUG(DBG_TRACE,( "Resume processed for %s.\n", pdev->name ));

	return 0;
}

#endif

#ifdef CONFIG_PM
static struct pm_ext_ops bcm59040_battery_pm_ops = {
    .base = {
        .suspend    = bcm59040_battery_suspend,
        .resume     = bcm59040_battery_resume
    }
};
#endif

static struct platform_driver bcm59040_battery_driver = {
	.driver = {
		.name	= BCM_BATTERY_DRIVER,
	},
#ifdef CONFIG_PM
	.pm             = &bcm59040_battery_pm_ops,
#endif
	.probe 		= battery_probe,
	.remove 	= battery_remove,
};

/** bc59040_batt_probe - BCM59040 battery probe for present.
 *
 * Probe to determine if BCM59040 is really present.
 *
 * @pclient:		Pointer to PMU Client structure.
 * @id:				id table.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */
static int bcm59040_batt_probe(struct pmu_client *pclient, const struct pmu_device_id *id)
{
	int					rc;
	bcm59040_batt_config_t			*batt_config;

	bcm59040_battery_psy.pclient = pclient;

	bcm59040_fuelgauge_init(pclient);

	if(!(batt_config = kmalloc(sizeof(*batt_config), GFP_KERNEL))) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Out of memory allocating adc_config\n"));
		return -ENOMEM;
	}

	batt_config->workqueue = create_workqueue(BCM59040_BATT_DRIVER);
	batt_config->vbus_collapse_workqueue = create_workqueue(BCM59040_BATT_DRIVER);
	batt_config->pclient = pclient;

	INIT_WORK(&batt_config->work, &bcm59040_batt_work);
	INIT_DELAYED_WORK(&batt_config->vbus_collapse_work, &vbus_collapse_work_handler);

	pmu_set_drvdata(pclient, batt_config);

	if ((rc = request_irq(BCM59040_IRQ_BATT, bcm59040_batt_isr, 0, "bcm59040_batt", batt_config)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_batt_probe failed to attach interrupt, rc = %d\n", rc) );
		kfree(batt_config);
		return rc;
	}

	if ((rc = platform_driver_register(&bcm59040_battery_driver)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_batt_probe failed to register platform driver, rc = %d!\n", rc));
		kfree(batt_config);
		free_irq(BCM59040_IRQ_BATT, batt_config);
		return rc;
	}

	return rc;
}

/** bcm59040_batt_remove - BCM59040 battery remove
 *
 * Need to remove BCM59040 battery driver.
 *
 * @pclient:	Pointer to client structure for this device.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */
static int bcm59040_batt_remove(struct pmu_client *pclient)
{
	bcm59040_batt_config_t *batt_config;

	batt_config = pmu_get_drvdata(pclient);
	if(batt_config) {
		free_irq(BCM59040_IRQ_BATT, batt_config);
		kfree(batt_config);
	}
	platform_driver_unregister(&bcm59040_battery_driver);

	return 0;
}

#ifdef CONFIG_PM

/** bcm59040_batt_suspend - Suspend battery functioning.
 *
 * Battery suspend handler that is called when sysem is entering
 * a suspend state.
 *
 * @pclient:	Pointer to client structure for this device.
 * @state:	PM state being entered.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 * @todo	Need to determine what suspend does for this driver still.
 *
 */
static int bcm59040_batt_suspend(struct pmu_client *pclient, pm_message_t state)
{
	// Prevent workqueues from running 
	// apparently cancelling the jobs results in a suspend not working some times
	// This method makes suspend work more reliably
	suspended = 1;

	bcm59040_fuelgauge_enable(pclient,0);
	disable_irq(BCM59040_IRQ_BATT);

	PMU_DEBUG (DBG_TRACE, ("Suspend processed for --- %s.\n", pclient->name));

	return 0;
} /* bcm59040_batt_suspend */

/** bcm59040_batt_resume - Resume battery functioning.
 *
 * Battery resume handler that is called when sysem is exiting
 * a suspend state.
 *
 * @client:	Pointer to client structure for this device.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 * @todo	Need to determine what suspend does for this driver still.
 *
 */
static int bcm59040_batt_resume(struct pmu_client *pclient)
{
	u32 cap;

	bcm59040_fuelgauge_enable(pclient,1);
	enable_irq(BCM59040_IRQ_BATT);

	suspended = 0;

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Resume processed for --- %s.\n", pclient->name ));

	/* Initialize capacity, because else battery_update_capacity() will not see the
       	increased capacity due to charging while in suspend; then if you resume without a charger 
       	connected apm_proxy will not allow resume because capacity is still 0*/ 
   	cap = convert_volts_to_capacity(batteryadc2voltage(&bcm59040_battery_psy), 1);
    	atomic_set(&bcm59040_battery_psy.current_capacity, cap);

	return 0;
} /* bcm59040_batt_resume */

#endif

struct pmu_device_id bcm59040_batt_idtable[] = {
	{ "battery" },
	{ }
};
/** @brief BCM59040 adc driver registration data
 *
 */

struct pmu_driver bcm59040_batt_driver = {
	.driver	= {
		.name	= BCM59040_BATT_DRIVER,
		.owner	= THIS_MODULE,
	},
	.probe		= &bcm59040_batt_probe,
	.remove		= &bcm59040_batt_remove,
#ifdef CONFIG_PM
	.suspend	= &bcm59040_batt_suspend,
	.resume		= &bcm59040_batt_resume,
#endif
	.id_table	= bcm59040_batt_idtable,
};

/** bcm59040_batt_init - BCM59040 batt module Initialization
 *
 * Registers the BCM59040 batt driver with the PMU device
 * framework.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */

static int __init bcm59040_batt_init(void)
{
	return pmu_register_driver(&bcm59040_batt_driver);
}

/** bcm59040_batt_exit - BCM59040 batt module Exit
 *
 * Deregisters the BCM59040 batt driver with the PMU device
 * framework.
 *
 */

static void __exit bcm59040_batt_exit(void)
{
	pmu_unregister_driver(&bcm59040_batt_driver);
}


late_initcall(bcm59040_batt_init);
module_exit(bcm59040_batt_exit);

MODULE_DESCRIPTION(BCM59040_BATT_MOD_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_VERSION(BCM59040_BATT_MOD_VERSION);
