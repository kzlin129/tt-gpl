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

/** BCM59040 PMU Regulator Driver
 *****************************************************************************
 *
 *     This implements the BCM59040 PMU Regulator driver layer. This layer
 *     implements support for controlling the LDOs and switchers of the
 *     BCM59040 PMU. Functionality for each regulator depends upon the
 *     capabilities and programming of the switchers and LDOs. The switchers
 *     allow changing the supply voltage, enable and disable. The LDOS only
 *     allow enable and disable.
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
#include <linux/workqueue.h>
#include <linux/interrupt.h>

#include <linux/platform_device.h>
#include <linux/pmu_device.h>
#include <asm/arch/pmu_device_bcm59040.h>

#include <linux/regulator/bcm59040-regulators.h>

/* Include external interfaces */

#include <linux/broadcom/pmu_bcm59040.h>

#include <asm/arch/bcm59040_irqs.h>
#include <asm/arch/reg_pwrseq.h>

#include <asm/arch/bcm59040.h>

/*
 * Structure and macro definitions.
 */

#define BCM59040_REGULATOR_DRIVER		"bcm59040_regulator"
#define BCM59040_REGULATOR_MOD_DESCRIPTION	"BCM59040 Regulator Driver"
#define BCM59040_REGULATOR_MOD_VERSION		1.0

#undef BCM59040_DEVNAME
#define BCM59040_DEVNAME			BCM59040_REGULATOR_DRIVER
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
#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO | DBG_TRACE)
//#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO)

static int	gLevel = DBG_DEFAULT_LEVEL;

#	define PMU_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define PMU_DEBUG(level,x)
#endif

typedef struct bcm59040_regulator_config_struct
{
	struct pmu_client		*pclient;
	struct work_struct		work;
	struct workqueue_struct		*workqueue;
} bcm59040_regulator_config_t;

struct bcm_regulator_info {
	struct regulator_desc		desc;
	struct regulator_dev		*rdev;
	bcm59040_regulator_config_t	*regulator_config;
};

/*
 * Local variables.
 */

static u32 bcm59040_csr_dvs_vout_to_mV_map[] =
{
	1500,
	1480,
	1460,
	1440,
	1420,
	1400,
	1380,
	1360,
	1340,
	1320,
	1300,
	1280,
	1260,
	1240,
	1220,
	1200,
	1180,
	1160,
	1140,
	1120,
	1100,
	1080,
	1060,
	1040,
	1020,
	1000,
	980,
	960,
	940,
	920,
	900,
	0,
};

/* output voltage mapping table */
static u32 bcm59040_ldo1_vout_to_mV_map[] =
{
	1200,
	1300,
	1400,
	1500,
	1600,
	1700,
	1800,
	1900,
	2000,
	2100,
	2200,
	2300,
	2400,
	2500,
	2600,
	2700,
	2800,
	2900,
	3000,
	3100,
	3200,
	3300,
	3400,
	2400,
	2400,
	2600,
	2600,
	2800,
	2800,
	3000,
	3200,
	3400,
};

/* output voltage mapping table */
static u32 bcm59040_ldo2_vout_to_mV_map[] =
{
	1200,
	1300,
	1400,
	1500,
	1600,
	1700,
	1800,
	1900,
	2000,
	2100,
	2200,
	2300,
	2400,
	2500,
	2600,
	2700,
	2800,
	2900,
	3000,
	3100,
	3200,
	3300,
	3400,
	2400,
	2400,
	2600,
	2600,
	2800,
	2800,
	3000,
	3200,
	3400,
};

/* output voltage mapping table */
static u32 bcm59040_ldo3_vout_to_mV_map[] =
{
	1200,
	1300,
	1400,
	1500,
	1600,
	1700,
	1800,
	1900,
	2000,
	2100,
	2200,
	2300,
	2400,
	2500,
	2600,
	2700,
	2800,
	2900,
	3000,
	3100,
	3200,
	3300,
	3400,
	2400,
	2400,
	2600,
	2600,
	2800,
	2800,
	3000,
	3200,
	3400,
};

/* output voltage mapping table */
static u32 bcm59040_ldo4_vout_to_mV_map[] =
{
	1200,
	1300,
	1400,
	1500,
	1600,
	1700,
	1800,
	1900,
	2000,
	2100,
	2200,
	2300,
	2400,
	2500,
	2600,
	2700,
	2800,
	2900,
	3000,
	3100,
	3200,
	3300,
	3400,
	2400,
	2400,
	2600,
	2600,
	2800,
	2800,
	3000,
	3200,
	3400,
};

/* output voltage mapping table */
static u32 bcm59040_ldo5_vout_to_mV_map[] =
{
	1200,
	1300,
	1400,
	1500,
	1600,
	1700,
	1800,
	1900,
	2000,
	2100,
	2200,
	2300,
	2400,
	2500,
	2600,
	2700,
	2800,
	2900,
	3000,
	3100,
	3200,
	3300,
	3400,
	2400,
	2400,
	2600,
	2600,
	2800,
	2800,
	3000,
	3200,
	3400,
};

/* output voltage mapping table */
static u32 bcm59040_ldo6_vout_to_mV_map[] =
{
	1200,
	1300,
	1400,
	1500,
	1600,
	1700,
	1800,
	1900,
	2000,
	2100,
	2200,
	2300,
	2400,
	2500,
	2600,
	2700,
	2800,
	2900,
	3000,
	3100,
	3200,
	3300,
	3400,
	2400,
	2400,
	2600,
	2600,
	2800,
	2800,
	3000,
	3200,
	3400,
};

/* output voltage mapping table */
static u32 bcm59040_csr_nodvs_vout_to_mV_map[] =
{
	2500, /*00000*/
	2400,
	2300,
	2200,
	2100,
	2000,
	1900,
	1800,
	1700,
	1600,
	1500,
	1400,
	1300,
	1200,
	1100,
	1000,
	900,  /*10000*/
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,   /*11111*/
};

/* output voltage mapping table */
static u32 bcm59040_iosr_vout_to_mV_map[] =
{
	2500, /*00000*/
	2400,
	2300,
	2200,
	2100,
	2000,
	1900,
	1800,
	1700,
	1600,
	1500,
	1400,
	1300,
	1200,
	1100,
	1000,
	900,  /*10000*/
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,   /*11111*/
};

static bcm59040_regulator_map_t bcm59040_regulator_map[BCM59040_NUM_REGULATORS] =
{
	{
		available:     1,
		programmable:  0,
		reg_addr:      BCM59040_REG_L1PMCTRL,
		reg_addr_volt: BCM59040_REG_LDO1CTRL,     // BCM59040_REGULATOR_LDO1
		min_mV:        1200,
		max_mV:        3400,
		mV_step:       100,
		vout_mask:     0x1F,
		vout_shift:    0,
		vout_to_mV_map: bcm59040_ldo1_vout_to_mV_map,
		map_size:      (sizeof(bcm59040_ldo1_vout_to_mV_map)/sizeof(bcm59040_ldo1_vout_to_mV_map[0])),
	},
	{
		available:     1,
		programmable:  0,
		reg_addr:      BCM59040_REG_L2PMCTRL,
		reg_addr_volt: BCM59040_REG_LDO2CTRL,     // BCM59040_REGULATOR_LDO2
		min_mV:        1200,
		max_mV:        3400,
		mV_step:       100,
		vout_mask:     0x1F,
		vout_shift:    0,
		vout_to_mV_map: bcm59040_ldo2_vout_to_mV_map,
		map_size:      (sizeof(bcm59040_ldo2_vout_to_mV_map)/sizeof(bcm59040_ldo2_vout_to_mV_map[0])),
	},
	{
		available:     1,
		programmable:  0,
		reg_addr:      BCM59040_REG_L3PMCTRL,
		reg_addr_volt: BCM59040_REG_LDO3CTRL,     // BCM59040_REGULATOR_LDO3
		min_mV:        1200,
		max_mV:        3400,
		mV_step:       100,
		vout_mask:     0x1F,
		vout_shift:    0,
		vout_to_mV_map: bcm59040_ldo3_vout_to_mV_map,
		map_size:      (sizeof(bcm59040_ldo3_vout_to_mV_map)/sizeof(bcm59040_ldo3_vout_to_mV_map[0])),
	},
	{
		available:     1,
		programmable:  0,
		reg_addr:      BCM59040_REG_L4PMCTRL,
		reg_addr_volt: BCM59040_REG_LDO4CTRL,     // BCM59040_REGULATOR_LDO4
		min_mV:        1200,
		max_mV:        3400,
		mV_step:       100,
		vout_mask:     0x1F,
		vout_shift:    0,
		vout_to_mV_map: bcm59040_ldo4_vout_to_mV_map,
		map_size:      (sizeof(bcm59040_ldo4_vout_to_mV_map)/sizeof(bcm59040_ldo4_vout_to_mV_map[0])),
	},
	{
		available:     1,
		programmable:  0,
		reg_addr:      BCM59040_REG_L5PMCTRL,
		reg_addr_volt: BCM59040_REG_LDO5CTRL,     // BCM59040_REGULATOR_LDO5
		min_mV:        1200,
		max_mV:        3400,
		mV_step:       100,
		vout_mask:     0x1F,
		vout_shift:    0,
		vout_to_mV_map: bcm59040_ldo5_vout_to_mV_map,
		map_size:      (sizeof(bcm59040_ldo5_vout_to_mV_map)/sizeof(bcm59040_ldo5_vout_to_mV_map[0])),
	},
		{
		available:     1,
		programmable:  0,
		reg_addr:      BCM59040_REG_L6PMCTRL,
		reg_addr_volt: BCM59040_REG_LDO6CTRL,     // BCM59040_REGULATOR_LDO6
		min_mV:        1200,
		max_mV:        3400,
		mV_step:       100,
		vout_mask:     0x1F,
		vout_shift:    0,
		vout_to_mV_map: bcm59040_ldo6_vout_to_mV_map,
		map_size:      (sizeof(bcm59040_ldo6_vout_to_mV_map)/sizeof(bcm59040_ldo6_vout_to_mV_map[0])),
	},
	{
		available:     1,
		programmable:  1,
		reg_addr:      BCM59040_REG_CSRPMCTRL,   // CSR Mode control
		reg_addr_volt: BCM59040_REG_CSRPCDVS0,      // CSR Voltage control  
		min_mV:        900,
		max_mV:        1500,
		mV_step:       20,
		vout_mask:     0x1F,
		vout_shift:    0,
		vout_to_mV_map: bcm59040_csr_dvs_vout_to_mV_map,
		map_size:      (sizeof(bcm59040_csr_dvs_vout_to_mV_map)/sizeof(bcm59040_csr_dvs_vout_to_mV_map[0])),
	},
	{
		available:     1,
		programmable:  0,
		reg_addr:      BCM59040_REG_IOSRPMCTRL,  // BCM59040_REGULATOR_IOSR
		reg_addr_volt: BCM59040_REG_IOSRPCDVS0,     // IOSR Voltage control
		min_mV:        900,
		max_mV:        2500,
		mV_step:       100,
		vout_mask:     0x1F,
		vout_shift:    0,
		vout_to_mV_map: bcm59040_iosr_vout_to_mV_map,
		map_size:      (sizeof(bcm59040_iosr_vout_to_mV_map)/sizeof(bcm59040_iosr_vout_to_mV_map[0])),
	}
};

/*
 * Local function and data declarations.
 */

static int bcm59040_mV_to_vout(int regulatorID, u32 mV, u8 *vout, u32 min_mV, u32 max_mV, u32 mV_step);
static int bcm59040_update_reg_addr_volt(struct pmu_client *pclient);
BCM_PMU_Regulator_State_t bcm59040_regulator_get_state(struct pmu_client *pclient, int regulatorID);
int bcm59040_vout_to_mV(int regulatorID, u8 vout, u32 *mV);
static int bcm59040_state_to_opmod(BCM_PMU_Regulator_State_t state, u8 *opmod);
static int bcm59040_opmod_to_state(struct pmu_client *pclient, u8 opmod, BCM_PMU_Regulator_State_t *state);
static int bcm59040_state_to_opmod(BCM_PMU_Regulator_State_t state, u8 *opmod);
static struct platform_driver bcm_regulator_driver;
static int bcm59040_regulator_probe(struct pmu_client *pclient,
									const struct pmu_device_id *id);
static int bcm59040_regulator_remove(struct pmu_client *pclient);

/** bcm59040_copy_pm_states - Copy one PM state config to another.
 *
 * Copies settings in LDO and regulator PM control registers from
 * one to another. In other words if source is 0 and destination is 1
 * then it copies the PM state value in all the PMCTRL registers
 * for the PM0 position (bits 1:1) to the PM1 position (bits 3:2).
 *
 * @pclient:	pointer to PMU Client structure.
 * @src:		source PM state of copy operation.
 * @dest:		destination PM state of copy operation.
 *
 */
static void bcm59040_copy_pm_states(struct pmu_client *pclient, int src, int dest)
{
	u8			pmctrl_regs[8], src_bits, data;
	int			rc, i;

	if((rc = pmu_bus_seqread(pclient,
							 BCM59040_REG_L1PMCTRL,
							 pmctrl_regs,
							 sizeof(pmctrl_regs))) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_copy_pm_states: error reading PMCTRL registers (%d).\n", rc));
		return;
	}

	for (i=0 ; i<sizeof(pmctrl_regs) ; i++) {

		/*
		 * First we clear out destination PM state configuration and
		 * then we copy the src state configuration bits up to destination bits.
		 */

		src_bits = ((pmctrl_regs[i] >> (src << 1)) & 0x03);
		pmctrl_regs[i] &= ~(0x03 << (dest << 1));
		pmctrl_regs[i] |= (src_bits << (dest <<1));
	}

	if((rc = pmu_bus_seqwrite(pclient,
							  BCM59040_REG_L1PMCTRL,
							  pmctrl_regs,
							  sizeof(pmctrl_regs))) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_copy_pm_states: error writing PMCTRL registers (%d).\n", rc));
		return;
	}

	/*
	 * Program CSR and IOSR voltages for the new state to be the same as the current_pm_state
	 * state.
	 */

	if((rc = pmu_bus_seqread(pclient,
							 BCM59040_REG_CSRPCDVS0+src,
							 &data,
							 sizeof(data))) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_copy_pm_states: error reading BCM59040_REG_CSRPCDVSx register (%d).\n", rc));
		return;
	}

	if((rc = pmu_bus_seqwrite(pclient,
							  BCM59040_REG_CSRPCDVS0+dest,
							  &data,
							  sizeof(data))) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_copy_pm_states: error writing BCM59040_REG_CSRPCDVSx register (%d).\n", rc));
		return;
	}

	if((rc = pmu_bus_seqread(pclient,
							 BCM59040_REG_IOSRPCDVS0+src,
							 &data,
							 sizeof(data))) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_copy_pm_states: error reading BCM59040_REG_IOSRPCDVSx register (%d).\n", rc));
		return;
	}

	if((rc = pmu_bus_seqwrite(pclient,
							  BCM59040_REG_IOSRPCDVS0+dest,
							  &data,
							  sizeof(data))) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_copy_pm_states: error writing BCM59040_REG_IOSRPCDVSx register (%d).\n", rc));
		return;
	}
}

/** bcm59040_regulator_set_voltage - Set voltage for a regulator.
 *
 * Set voltage level for specific regulator
 *
 * @dev:			Pointer to device structure.
 * @regulatorID:	Specific regulator ID, from 0 to 7
 * @mV:				Voltage level - milli Volt
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */
int bcm59040_regulator_set_voltage(struct pmu_client *pclient,
								   int regulatorID,
								   u32 mV)
{
	int	rc;
	u8	val;
	u8	vout;

	if(!bcm59040_regulator_map[regulatorID].available) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_regulator_set_voltage:regulaor %d not available\n", regulatorID));
		return -EINVAL;
	}

	/* Convert voltage */
	rc = bcm59040_mV_to_vout(regulatorID,
							 mV,
							 &vout,
							 bcm59040_regulator_map[regulatorID].min_mV,
							 bcm59040_regulator_map[regulatorID].max_mV,
							 bcm59040_regulator_map[regulatorID].mV_step);

	if(rc < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_regulator_set_voltage: error converting %d mV.\n", mV));
		return -EINVAL;
	}

	if((regulatorID == 6) || (regulatorID == 7)) {
		bcm59040_update_reg_addr_volt(pclient);
	}
	val=vout;

	// write settings
	if((rc = pmu_bus_seqwrite(pclient,
							  bcm59040_regulator_map[regulatorID].reg_addr_volt,
							  &val, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_set_voltage: error writing regulator control register.\n"));
		return rc;
	}

	return 0;
}

/** bcm59040_regulator_get_voltage - Get current voltage setting of a regulator.
 *
 * Retrieve the current voltage level and optionally the valid range of settings.
 *
 * @dev:			Pointer to device structure.
 * @regulatorID:	Specific regulator ID, from 0 to 7
 * @min_mV:			Minimum voltage level for this regulator
 * @max_mV:			Maximum voltage level for this regulator
 * @mV_step:		Step size of the voltage setting for this regulator
 *
 * Returns current voltage for specific regulator if successful, otherwise standard Linux error codes.
 *
 */
u32 bcm59040_regulator_get_voltage(struct pmu_client *pclient, int regulatorID, u32 *min_mV, u32 *max_mV, u32 *mV_step)
{
	int							rc;
	u32							mV;
	uint8_t						data;
	BCM_PMU_Regulator_State_t	state;

	if((regulatorID > 7) || (regulatorID < 0)) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_get_voltage: regulatorID %d is out of boundary.\n", regulatorID));
		return -EINVAL;
	}

	if(!bcm59040_regulator_map[regulatorID].available) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_get_voltage: regulator %d not available.\n", regulatorID));
		return -EINVAL;
	}

	state = bcm59040_regulator_get_state(pclient, regulatorID);
	if(state == PMU_Regulator_Off) {
		mV = 0;
	} else {
		//NM or LPM
		if((regulatorID == 6) || (regulatorID == 7)) {
			bcm59040_update_reg_addr_volt(pclient);
		}
		if((rc = pmu_bus_seqread(pclient,
								 bcm59040_regulator_map[regulatorID].reg_addr_volt,
								 &data, 1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading %d register.\n", bcm59040_regulator_map[regulatorID].reg_addr_volt));
			return rc;
		}
		if((rc = bcm59040_vout_to_mV(regulatorID, (u8)data, &mV))) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_get_voltage: error converting voltage.\n"));
			return rc;
		}
	}

	if (min_mV)
		*min_mV = bcm59040_regulator_map[regulatorID].min_mV;
	if (max_mV)
		*max_mV = bcm59040_regulator_map[regulatorID].max_mV;
	if (mV_step)
		*mV_step = bcm59040_regulator_map[regulatorID].mV_step;

	return mV;
}

/** bcm59040_regulator_set_state_for_pm - Set new state for a regulator.
 *
 * Set the state (NM, LPM, OFF) of a regulator
 *
 * @dev:			Pointer to device structure.
 * @regulatorID:	Specific regulator ID, from 0 to 7
 * @pmState:		Power mode state, from 0 to 3  
 * @state:			Regulator state, NM, LPM or OFF
 *
 * Returns 0 if successful, otherwise standard Linux error codes.
 *
 */
int bcm59040_regulator_set_state_for_pm(struct pmu_client *pclient, int regulatorID, int pmState, BCM_PMU_Regulator_State_t state)
{
	int			rc;
	u8			val;
	uint8_t		data;
	u8			opmod;
	u8			mask = 0;

	if(!bcm59040_regulator_map[regulatorID].available) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "regulator %d not available.\n", regulatorID));
		return -EINVAL;
	}

	// convert state
	rc = bcm59040_state_to_opmod(state, &opmod);
	if(rc < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error converting state %d.\n", state));
		return -EINVAL;
	}

	switch (pmState) {
		case 0:
		mask = BCM59040_PMCTRL_PM0_MASK;
		break;

		case 1:
		opmod = opmod << 2;
		mask = BCM59040_PMCTRL_PM1_MASK;
		break;

		case 2:
		opmod = opmod << 4;
		mask = BCM59040_PMCTRL_PM2_MASK;
		break;

		case 3:
		opmod = opmod << 6;
		mask = BCM59040_PMCTRL_PM3_MASK;
		break;
	}

	// read current settings
	if((rc = pmu_bus_seqread(pclient,
							 bcm59040_regulator_map[regulatorID].reg_addr,
							 &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "set_state_for_pm: error reading regulator control register (%d).\n", rc));
		return rc;
	}

	// update register
	val = opmod;

	if(val != ((u8)data & mask)) {
		// write settings only if a change in value is detected
		data = ((u8)data & ~mask) | val;
		if((rc = pmu_bus_seqwrite(pclient,
								  bcm59040_regulator_map[regulatorID].reg_addr,
								  &data, 1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing regulator control register.\n"));
			return rc;
		}
	}

	return 0;
}

/** bcm59040_regulator_get_state_for_pm - Return current state of a regulator.
 *
 * Retrieve the current state (NM, LPM or OFF) of a regulator
 *
 * @dev:			Pointer to device structure.
 * @regulatorID:	specific regulator ID, from 0 to 7
 * @pmState:		power mode state, from 0 to 3  

 * Returns regulator state if successful, otherwise standard Linux error codes.
 *
 */
BCM_PMU_Regulator_State_t bcm59040_regulator_get_state_for_pm(struct pmu_client *pclient, int regulatorID, int pmState)
{
	int		rc;
	uint8_t		data;
	BCM_PMU_Regulator_State_t	state;

	if((regulatorID > 7) || (regulatorID < 0)) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_get_state: regulatorID %d is out of boundary.\n", regulatorID));
		return -EINVAL;
	}

	if(!bcm59040_regulator_map[regulatorID].available) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "regulator %d not available.\n", regulatorID));
		return PMU_Regulator_Off;
	}

	if((rc = pmu_bus_seqread(pclient,
							 bcm59040_regulator_map[regulatorID].reg_addr,
							 &data,1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "get_state_for_pm: error reading regulator control register.\n"));
		return PMU_Regulator_Off;
	}

	data = data >> (pmState *2);
	if((rc = bcm59040_opmod_to_state(pclient, (u8)data, &state))) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error converting state.\n"));
		return PMU_Regulator_Off;
	}

	return state;
}

static int bcm59040_update_reg_addr_volt(struct pmu_client *pclient)
{
	int			rc;
	uint8_t		data;

	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_HBCTRL, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_HBCTRL register.\n"));
		return rc;
	}
	data = (data & BCM59040_HBCTRL_PC_I2C_MASK) >> 6; // data has the value of PMx

	switch (data) {
		case 0:
			bcm59040_regulator_map[6].reg_addr_volt = BCM59040_REG_CSRPCDVS0;
			bcm59040_regulator_map[7].reg_addr_volt = BCM59040_REG_IOSRPCDVS0;
			break;

		case 1:
			bcm59040_regulator_map[6].reg_addr_volt = BCM59040_REG_CSRPCDVS1;
			bcm59040_regulator_map[7].reg_addr_volt = BCM59040_REG_IOSRPCDVS1;
			break;

		case 2:
			bcm59040_regulator_map[6].reg_addr_volt = BCM59040_REG_CSRPCDVS2;
			bcm59040_regulator_map[7].reg_addr_volt = BCM59040_REG_IOSRPCDVS2;
			break;

		case 3:
			bcm59040_regulator_map[6].reg_addr_volt = BCM59040_REG_CSRPCDVS3;
			bcm59040_regulator_map[7].reg_addr_volt = BCM59040_REG_IOSRPCDVS3;
			break;
	}

	return 0;
}

static int bcm59040_opmod_to_state(struct pmu_client *pclient, u8 opmod, BCM_PMU_Regulator_State_t *state)
{
	int			rc;
	uint8_t		data;

	if (!state)
		return -1;

	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_HBCTRL, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_HBCTRL register.\n"));
		return PMU_Regulator_Off;
	}

	data = (data & BCM59040_HBCTRL_PC_I2C_MASK) >> 6; // rc has the value of PMx

	switch (data) {
		case 0:
		opmod = opmod & BCM59040_PMCTRL_PM0_MASK;
		break;

		case 1:
		opmod = (opmod & BCM59040_PMCTRL_PM1_MASK) >> 2;
		break;

		case 2:
		opmod = (opmod & BCM59040_PMCTRL_PM2_MASK) >> 4;
		break;

		case 3:
		opmod = (opmod & BCM59040_PMCTRL_PM3_MASK) >> 6;
		break;
	}

	switch (opmod) {
		case BCM59040_BIT_REG_OFF:
			*state = PMU_Regulator_Off;
			break;

		case BCM59040_BIT_REG_ON:
		case BCM59040_BIT_REG_ON_ALT:
			*state = PMU_Regulator_On;
			break;

		case BCM59040_BIT_REG_ECO:
			*state = PMU_Regulator_Eco;
			break;

		default:
			return -1;
	}

	return 0;
}


static int bcm59040_state_to_opmod(BCM_PMU_Regulator_State_t state, u8 *opmod)
{
	if(!opmod)
		return -1;

	switch (state) {
		case PMU_Regulator_Off:
			*opmod = BCM59040_BIT_REG_OFF;
			break;

		case PMU_Regulator_On:
			*opmod = BCM59040_BIT_REG_ON;
			break;

		case PMU_Regulator_Eco:
			*opmod = BCM59040_BIT_REG_ECO;
			break;

		default:
			return -1;
	}

	return 0;
}



static int bcm59040_mV_to_vout(int regulatorID, u32 mV, u8 *vout, u32 min_mV, u32 max_mV, u32 mV_step)
{
	u32		*vout_to_mV_map;
	int		map_size;
	int		i;

	if(!vout)
		return -1;

	/* Validate input mV */
	if((mV < min_mV) || (mV > max_mV)) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040: invalid %d mV setting for regulator %d.\n", mV, regulatorID));
		return -1;
	}

	if(mV_step != -1) {
		if ((mV - min_mV) % mV_step) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040: invalid %d mV setting for regulator %d.\n", mV, regulatorID));
			return -1;
		}
	}

	if(bcm59040_regulator_map[regulatorID].vout_to_mV_map) {
		vout_to_mV_map = bcm59040_regulator_map[regulatorID].vout_to_mV_map;
		map_size = bcm59040_regulator_map[regulatorID].map_size;
	} else {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Not supported\n"));
		return -1;
	}

	/* Find matching voltage in table */
	for (i = 0; i < map_size; i++) {
		if (vout_to_mV_map[i] == mV) {
			*vout = i;
			return 0;
		}
	}

	PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040: corrupt mapping table.\n"));
	return -1;
}

int bcm59040_vout_to_mV(int regulatorID, u8 vout, u32 *mV)
{
	u32		*vout_to_mV_map;
	int		map_size;

	if(!mV) 
		return -1;

	if(bcm59040_regulator_map[regulatorID].vout_to_mV_map) {
		vout_to_mV_map = bcm59040_regulator_map[regulatorID].vout_to_mV_map;
		map_size = bcm59040_regulator_map[regulatorID].map_size;
	} else {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Not supported\n"));
		return -1;
	}

	// Mapping register value to voltage
	if(vout >= map_size) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040: vout out of range\n"));
		*mV = 0;
		return -1;
	}

	*mV = vout_to_mV_map[vout];
	return 0;
}

BCM_PMU_Regulator_State_t bcm59040_regulator_get_state(struct pmu_client *pclient,
													   int regulatorID)
{
	int							rc;
	uint8_t						data;
	BCM_PMU_Regulator_State_t	state;

	if((regulatorID > 7) || (regulatorID < 0)) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_get_state: regulatorID %d is out of boundary.\n", regulatorID));
		return -EINVAL;
	}

	if (!bcm59040_regulator_map[regulatorID].available) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "regulator %d not available.\n", regulatorID));
		return PMU_Regulator_Off;
	}

	if((rc = pmu_bus_seqread(pclient,
							 bcm59040_regulator_map[regulatorID].reg_addr,
							 &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading regulator control register.\n"));
		return PMU_Regulator_Off;
	}

	if((rc = bcm59040_opmod_to_state(pclient, (u8)data, &state))) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error converting state.\n"));
		return PMU_Regulator_Off;
	}

	return state;
}

/** bcm59040_regulator_work - Work function for BCM59040 REGULATOR IRQ
 * @work:   Pointer to work structure.
 *
 */
static void bcm59040_regulator_work(struct work_struct *work)
{
	int								rc;
	uint8_t							data;
	bcm59040_regulator_config_t	    *regulator_config;
	struct pmu_client				*pclient;

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Entering bcm59040_regulator_work:\n"));

	regulator_config = container_of(work, bcm59040_regulator_config_t, work);
	pclient = regulator_config->pclient;

	if((rc = pmu_bus_seqread(pclient, BCM59040_REG_INT4, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_VINT4 register.\n"));
	}
	else {
		PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Read %d bytes (%02X) from virtual int status registers.\n", rc, data));

		if (data & 0x01) {
			PMU_DEBUG(DBG_INFO,(KERN_INFO PFX "LDO1 overcurrent\n"));
		}
		if (data & 0x02) {
			PMU_DEBUG(DBG_INFO,(KERN_INFO PFX "LDO2 overcurrent\n"));
		}
		if (data & 0x04) {
			PMU_DEBUG(DBG_INFO,(KERN_INFO PFX "LDO3 overcurrent\n"));
		}
		if (data & 0x08) {
			PMU_DEBUG(DBG_INFO,(KERN_INFO PFX "LDO4 overcurrent\n"));
		}
		if (data & 0x10) {
			PMU_DEBUG(DBG_INFO,(KERN_INFO PFX "LDO5 overcurrent\n"));
		}
		if (data & 0x20) {
			PMU_DEBUG(DBG_INFO,(KERN_INFO PFX "LDO6 overcurrent\n"));
		}
	}

	if ((rc = pmu_bus_seqread(pclient, BCM59040_REG_INT5, &data, 1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_VINT5 register.\n"));
	}
	else {
		if (data & 0x01) {
			PMU_DEBUG(DBG_INFO,(KERN_INFO PFX "IOSR overcurrent\n"));
		}
		if (data & 0x02) {
			PMU_DEBUG(DBG_INFO,(KERN_INFO PFX "CSR overcurrent\n"));
		}
		if (data & 0x04) {
			PMU_DEBUG(DBG_INFO,(KERN_INFO PFX "IOSR overvoltage\n"));
		}
		if (data & 0x08) {
			PMU_DEBUG(DBG_INFO,(KERN_INFO PFX "CSR overvoltage\n"));
		}
	}

	/* clear the interrupt */
	data = 1 << BCM59040_VINT1_REGULATOR_BIT;
	pmu_bus_seqwrite(pclient, BCM59040_REG_VINT1, &data, 1 );
}

/** bcm59040_regulator_isr - Interrupt service routine for BCM59040 regulator
 * @irq:    IRQ vector number
 * @dev_id: Data structure associated with this interrupt event.
 *
 * This ISR handles events for the regulator in the BCM59040. 
 */
static irqreturn_t bcm59040_regulator_isr(int irq, void *dev_id)
{
	bcm59040_regulator_config_t	*regulator_config = dev_id;

	queue_work(regulator_config->workqueue, &regulator_config->work);

   return IRQ_HANDLED;
}

struct pmu_device_id bcm59040_regulator_idtable[] = {
	{ "regulator" },
	{ }
};

/** @brief BCM59040 regulator driver registration data
 */

struct pmu_driver bcm59040_regulator_driver =
{
	.driver	= {
		.name	= BCM59040_REGULATOR_DRIVER,
		.owner	= THIS_MODULE,
	},
	.probe		= &bcm59040_regulator_probe,
	.remove		= &bcm59040_regulator_remove,
	.id_table	= bcm59040_regulator_idtable,
};

static int bcm_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV)
{
	struct bcm_regulator_info *info = rdev_get_drvdata(rdev);

	if (bcm59040_regulator_set_voltage(info->regulator_config->pclient,
									   info->desc.id,
									   min_uV/1000))
	{
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "invalid voltage level %d-%d uV", min_uV, max_uV));
		return -EINVAL;
	}
	return 0;
}

static int bcm_get_voltage(struct regulator_dev *rdev)
{
	int mV;
	u32 min_mV, max_mV, mV_step;
	struct bcm_regulator_info *info = rdev_get_drvdata(rdev);

	mV = (int)bcm59040_regulator_get_voltage(info->regulator_config->pclient,
											 info->desc.id,
											 &min_mV,
											 &max_mV,
											 &mV_step);
	return (mV <= 0) ? mV : (mV * 1000);
}

static int bcm_enable(struct regulator_dev *rdev)
{
	struct bcm_regulator_info *info;
	int new_pm_state,rc;

	info = rdev_get_drvdata(rdev);

	new_pm_state = pwrseq_switch_pm_states_begin();
	bcm59040_copy_pm_states(info->regulator_config->pclient,
							(new_pm_state == 1) ? 2 : 1,
							new_pm_state);
	rc = bcm59040_regulator_set_state_for_pm(info->regulator_config->pclient,
							 info->desc.id,
							 new_pm_state,
							 PMU_Regulator_On);
	pwrseq_switch_pm_states_finalize(new_pm_state);

	return rc;
}

static int bcm_disable(struct regulator_dev *rdev)
{
	struct bcm_regulator_info *info;
	int new_pm_state,rc;

	info = rdev_get_drvdata(rdev);

	new_pm_state = pwrseq_switch_pm_states_begin();
	bcm59040_copy_pm_states(info->regulator_config->pclient,
							(new_pm_state == 1) ? 2 : 1,
							new_pm_state);
	rc = bcm59040_regulator_set_state_for_pm(info->regulator_config->pclient,
											 info->desc.id,
											 new_pm_state,
											 PMU_Regulator_Off);
	pwrseq_switch_pm_states_finalize(new_pm_state);

	return rc;
}

static int bcm_is_enabled(struct regulator_dev *rdev)
{
	int current_pm_state;

	struct bcm_regulator_info *info = rdev_get_drvdata(rdev);

	current_pm_state = pwrseq_get_current_pm_state();

	return (bcm59040_regulator_get_state_for_pm(info->regulator_config->pclient,
												info->desc.id,
												current_pm_state) !=
			PMU_Regulator_Off);
}

static int bcm_suspend_enable(struct regulator_dev *rdev)
{
	struct bcm_regulator_info *info;
	struct pmu_client *pclient;

	info = rdev_get_drvdata(rdev);
	pclient = info->regulator_config->pclient;

	return bcm59040_regulator_set_state_for_pm (pclient, info->desc.id, 3, PMU_Regulator_On);  //PM3
}

static int bcm_suspend_disable(struct regulator_dev *rdev)
{
	struct bcm_regulator_info *info;
	struct pmu_client *pclient;

	info = rdev_get_drvdata(rdev);
	pclient = info->regulator_config->pclient;

	return bcm59040_regulator_set_state_for_pm (pclient, info->desc.id, 3, PMU_Regulator_Off);  //PM3
}

static struct regulator_ops bcm_LDO_ops = {
    .get_voltage           = bcm_get_voltage,
    .enable                = bcm_enable,
    .disable               = bcm_disable,
    .is_enabled            = bcm_is_enabled,
    .set_suspend_enable    = bcm_suspend_enable,
    .set_suspend_disable   = bcm_suspend_disable,
};

static struct regulator_ops bcm_switcher_ops = {
	.set_voltage            = bcm_set_voltage,
	.get_voltage            = bcm_get_voltage,
    .enable                 = bcm_enable,
    .disable                = bcm_disable,
    .is_enabled             = bcm_is_enabled,
    .set_suspend_enable     = bcm_suspend_enable,
    .set_suspend_disable    = bcm_suspend_disable,
};

struct bcm_regulator_info bcm_regulators[] = {
    {
        .desc =
        {
            .name       = "LDO1",
            .id         = BCM59040_LDO1_ID,
            .ops        = &bcm_LDO_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "LDO2",
            .id         = BCM59040_LDO2_ID,
            .ops        = &bcm_LDO_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "LDO3",
            .id         = BCM59040_LDO3_ID,
            .ops        = &bcm_LDO_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "LDO4",
            .id         = BCM59040_LDO4_ID,
            .ops        = &bcm_LDO_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "LDO5",
            .id         = BCM59040_LDO5_ID,
            .ops        = &bcm_LDO_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "LDO6",
            .id         = BCM59040_LDO6_ID,
            .ops        = &bcm_LDO_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "CSR",
            .id         = BCM59040_CSR_ID,
            .ops        = &bcm_switcher_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "IOSR",
            .id         = BCM59040_IOSR_ID,
            .ops        = &bcm_switcher_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        }
    },
};

/** bcm59040_regulator_probe - BCM59040 regulator probe 
 *
 * Probe to determine if BCM59040 is really present.
 *
 * @pclient:		Pointer to PMU Client structure.
 * @id:				id table.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */
static int bcm59040_regulator_probe(struct pmu_client *pclient,const struct pmu_device_id *id)
{
	int	rc, i;
	u8	data,data1;
	bcm59040_regulator_config_t	*regulator_config;

	if ((regulator_config = kmalloc(sizeof(*regulator_config), GFP_KERNEL)) == NULL) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Out of memory allocating regulator_data\n"));
		return -ENOMEM;
	}

	regulator_config->pclient = pclient;
	regulator_config->workqueue = create_workqueue(BCM59040_REGULATOR_DRIVER);
	INIT_WORK(&regulator_config->work, bcm59040_regulator_work);

	pmu_set_drvdata(pclient, regulator_config);

	rc = request_irq(BCM59040_IRQ_REGULATORS, bcm59040_regulator_isr,
					 0,
					 "bcm59040_regulator",
					 regulator_config);
	if (rc < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_regulator_probe failed to attach interrupt, rc = %d\n", rc));
		kfree(regulator_config);
		return rc;
	}

	for(i=0 ; i<ARRAY_SIZE(bcm_regulators) ; i++) {
		bcm_regulators[i].regulator_config = regulator_config;
	}

	/*
	 * Copy PM0 state to PM1 and PM2 for each LDO and regulator so those states have valid default
	 * power-on configuration.
	 */

	bcm59040_copy_pm_states(pclient, 0, 1);
	bcm59040_copy_pm_states(pclient, 0, 2);

	if ((rc = platform_driver_register(&bcm_regulator_driver)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_regulator_probe failed to register platform driver, rc = %d!\n", rc));
		kfree(regulator_config);
		free_irq(BCM59040_IRQ_REGULATORS, regulator_config);
		return rc;
	}

	/*
	 * Program CSR and IOSR voltages for the new state to be the same as the current_pm_state
	 * state.
	 */

	if ((rc = pmu_bus_seqread(pclient, BCM59040_REG_CSRPCDVS0, &data, sizeof(data))) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_probe: error reading BCM59040_REG_CSRPCDVSx register.\n"));
		return rc;
	}

	if ((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_CSRPCDVS3, &data, sizeof(data))) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_probe: error writing BCM59040_REG_CSRPCDVSx register.\n"));
		return rc;
	}

	if ((rc = pmu_bus_seqread(pclient, BCM59040_REG_IOSRPCDVS0, &data, sizeof(data))) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_probe: error reading BCM59040_REG_IOSRPCDVSx register.\n"));
		return rc;
	}

	mdelay(20);

	if ((rc = pmu_bus_seqwrite(pclient, BCM59040_REG_IOSRPCDVS3, &data, sizeof(data))) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_probe: error writing BCM59040_REG_IOSRPCDVSx register.\n"));
		return rc;
	}
	if ((rc = pmu_bus_seqread(pclient, BCM59040_REG_IOSRPCDVS3, &data1, sizeof(data1))) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_probe: error reading BCM59040_REG_IOSRPCDVSx register.\n"));
		return rc;
	}

	if (data != data1)
		printk (KERN_ERR "Value of IOSRPCDVS3 could not be loaded properly\n");

	return 0;
}

/** bcm59040_regulator_remove - BCM59040 regulator removal handler.
 *
 * Need to remove BCM59040 regulator driver.
 *
 * @pclient:	Pointer to client structure for this device.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */
static int bcm59040_regulator_remove(struct pmu_client *pclient)
{
	bcm59040_regulator_config_t	*regulator_config;

	regulator_config = pmu_get_drvdata(pclient);
	if(regulator_config) {
		free_irq(BCM59040_IRQ_REGULATORS, regulator_config);
		kfree(regulator_config);
	}
	platform_driver_unregister(&bcm_regulator_driver);

	return 0;
}

static int __devinit bcm_regulator_probe(struct platform_device *pdev)
{
    struct regulator_init_data *init_data = pdev->dev.platform_data;
    struct regulator_dev *rdev;
    int rc;

    rdev = regulator_register(&bcm_regulators[pdev->id].desc, &pdev->dev,
                              &bcm_regulators[pdev->id]);

    if (IS_ERR(rdev))
        return PTR_ERR(rdev);

    /* As we don't really have firmware/bootloader enabling of the LDOs, ensure that what is set as boot_on */
    /* really is enabled. */
    if( init_data->constraints.boot_on )
    {
	printk( "Enabling regulator \"%s\" because of boot_on\n", bcm_regulators[pdev->id].desc.name );
        rc=bcm_enable( rdev );
        if( rc < 0 )
            return rc;
    }

    return 0;
}

static int bcm_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;
}

static struct platform_driver bcm_regulator_driver = {
	.driver	= {
		.name	= "bcmpmu_regulator",
		.owner	= THIS_MODULE,
	},
	.probe		= bcm_regulator_probe,
	.remove		= bcm_regulator_remove,
};

/** bcm59040_regulator_init - BCM59040 regulator module Initialization
 *
 * Registers the BCM59040 regulator driver with the PMU device
 * framework.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */

static int __init bcm59040_regulator_init(void)
{
	return pmu_register_driver(&bcm59040_regulator_driver);
}

/** bcm59040_regulator_exit - BCM59040 regulator module Exit
 *
 * Deregisters the BCM59040 regulator driver with the PMU device
 * framework.
 *
 * Returns 0 if successful otherwise standard Linux error codes.
 *
 */

static void __exit bcm59040_regulator_exit(void)
{
	pmu_unregister_driver(&bcm59040_regulator_driver);
}

device_initcall(bcm59040_regulator_init);
module_exit(bcm59040_regulator_exit);

MODULE_DESCRIPTION(BCM59040_REGULATOR_MOD_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_VERSION(BCM59040_REGULATOR_MOD_VERSION);
