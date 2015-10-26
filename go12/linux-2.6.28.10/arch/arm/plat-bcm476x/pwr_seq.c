/*****************************************************************************
* Copyright 2008 - 2009 Broadcom Corporation.  All rights reserved.
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
 *   @file   pwr_seq.c 
 *
 *   @brief  4760 power manager/sequencer driver.
 *
 ****************************************************************************/

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include <asm/arch/reg_pwrseq.h>
#include <asm/arch/rtc_cpuapi4760.h>
#include <asm/arch/bbl4760.h>
#include <asm/arch/pm_bcm476x.h>

#if defined(CONFIG_BCM_PMU_BCM59040)
#include <linux/broadcom/pmu_bcm59040.h>
#endif

#include <asm/mach/irq.h>
#include <asm/io.h>

#define RET_ERROR 1
#define RET_OK    0

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG 1

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

//#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO | DBG_TRACE)
#define DBG_DEFAULT_LEVEL	DBG_INFO | DBG_TRACE

#if DEBUG
#	define PWRSEQ_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define PWRSEQ_DEBUG(level,x)
#endif

#define ASM_STOPHERE    __asm("1: nop; b 1b")

//#define CONFIG_BCM4760_PMSTR_SW_PWRDOWN

// Variables that will be used before sleep and after sleep.
unsigned int  g_wake_up_set_register_val_lower;
unsigned int  g_wake_up_set_register_val_higher;

static int gLevel = DBG_DEFAULT_LEVEL;

/** @brief Currently active Power Management State.
 *
 * Current PM state. This is used to remember what the
 * power management unit state is. Currently valid states
 * are PM0, PM1, PM2 and PM3 (represented by values from
 * 0-3) in the CurrentPmState variable below.
 *
 * Entering suspend requires us  to return to PM0, for
 * safest operation, rather than the PM state we're in
 * when the suspend sequence begins. For that reason we
 * need to transition from PM0 to the state defined by
 * CurrentPmState as the final action of the resume
 * operation. This also means that CurrentPmState
 * is never modified by the suspend/resume code but
 * is read by it to determine the final PM state we
 * want to be in.
 */

static int CurrentPmState = 0;

/*
 * Define this to use pwrseq_trigger_pm_state_switch
 * method of causing manual PM state changes. Otherwise
 * the pwrseq_force_pm_state_sw_override and
 * pwrseq_stop_pm_state_sw_override functions are
 * used (the SW override method of PM state changes).
 */
//#define BCM4760_PWRSEQ_PML_TRIGGER_SWITCH

DEFINE_MUTEX(pwrseq_pmu_state_switch_mutex);

static void*	cpu_context = NULL;

static struct PwrSeqRegisterSave bcm4760_hw_hib_regs[] =
{
	{ DDR_R_EMI_REFRESH_CNTRL_MEMADDR, 0, 4, DDR_R_EMI_REFRESH_CNTRL_MASK },
	{ CMU_R_CHIP_PIN_MUX0_MEMADDR, 0, 4, CMU_R_CHIP_PIN_MUX0_MASK },
	{ CMU_R_CHIP_PIN_MUX1_MEMADDR, 0, 4, CMU_R_CHIP_PIN_MUX1_MASK },
	{ CMU_R_CHIP_PIN_MUX2_MEMADDR, 0, 4, CMU_R_CHIP_PIN_MUX2_MASK },
	{ CMU_R_CHIP_PIN_MUX3_MEMADDR, 0, 4, CMU_R_CHIP_PIN_MUX3_MASK },
	{ CMU_R_CHIP_PIN_MUX4_MEMADDR, 0, 4, CMU_R_CHIP_PIN_MUX4_MASK },
	{ CMU_R_CHIP_PIN_MUX5_MEMADDR, 0, 4, CMU_R_CHIP_PIN_MUX5_MASK },
	{ CMU_R_CHIP_PIN_MUX6_MEMADDR, 0, 4, CMU_R_CHIP_PIN_MUX6_MASK },
	{ CMU_R_CHIP_PIN_MUX7_MEMADDR, 0, 4, CMU_R_CHIP_PIN_MUX7_MASK },
	{ CMU_R_CHIP_PIN_MUX8_MEMADDR, 0, 4, CMU_R_CHIP_PIN_MUX8_MASK },
	{ CMU_R_CHIP_PIN_MUX9_MEMADDR, 0, 4, CMU_R_CHIP_PIN_MUX9_MASK },
	{ CMU_R_CHIP_PIN_MUX10_MEMADDR, 0, 4, CMU_R_CHIP_PIN_MUX10_MASK },
	{ CMU_R_CHIP_PIN_MUX11_MEMADDR, 0, 4, CMU_R_CHIP_PIN_MUX11_MASK },
//	{ CMU_R_DEBUG_SELECT0_MEMADDR, 0, 4, CMU_F_NOR_BOOTSTRAP_MASK_MASK },
//	{ CMU_R_PLA_PLL_CTL5_MEMADDR, 0, 4, (CMU_F_PLA_M1DIV_MASK | CMU_F_PLA_M2DIV_MASK | CMU_F_PLA_M3DIV_MASK | CMU_F_PLA_M4DIV_MASK) },
	{ CMU_R_PLA_PLL_CTL6_MEMADDR, 0, 4, (CMU_F_PLA_M6DIV_MASK | CMU_F_PLA_M5DIV_MASK) },
//	{ CMU_R_PLC_PLL_CTL5_MEMADDR, 0, 4, (CMU_F_PLC_M1DIV_MASK | CMU_F_PLC_M2DIV_MASK | CMU_F_PLC_M3DIV_MASK | CMU_F_PLC_M4DIV_MASK) },
//	{ CMU_R_PLC_PLL_CTL6_MEMADDR, 0, 4, (CMU_F_PLC_M6DIV_MASK | CMU_F_PLC_M5DIV_MASK) },
	{ CMU_R_FREQ_SELECT_MEMADDR, 0, 4, CMU_R_FREQ_SELECT_MASK },
	{ CMU_R_PLA_PLL_CTL0_MEMADDR, 0, 4, (CMU_F_PLA_EN_CMLBUF2_MASK | CMU_F_PLA_EN_CMLBUF6_MASK) },
	{ CMU_R_USB_PHY0_MEMADDR, 0, 4, CMU_R_USB_PHY0_MASK },
//	{ CMU_R_USB_PHY1_MEMADDR, 0, 4, CMU_R_USB_PHY1_MASK },
	{ GIO1_R_GPCTR28_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR29_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR30_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR31_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR32_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR33_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR34_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR35_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR36_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR37_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR15_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR16_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR17_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR18_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR19_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR20_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR21_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR76_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR77_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR78_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR79_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR80_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR81_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR82_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR83_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR84_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR85_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR86_MEMADDR, 0, 4, GIO1_F_SEL_MASK },
	{ GIO1_R_GPCTR87_MEMADDR, 0, 4, GIO1_F_SEL_MASK }
};

/** @brief Save registers into preservation struct for resume from suspend to memory
 *
 * This function will scan the structure defined by struct PwrSeqRegisterSave
 * and preserve all the registers defined by that structure. This structure
 * is then used by the matching resume function when we come back out suspend to
 * memory to restore the values in the registers.
 *
 */
static void SaveChipRegisters(void)
{
	unsigned int	i;

	for (i=0 ; i<(sizeof(bcm4760_hw_hib_regs) / sizeof(bcm4760_hw_hib_regs[0])) ; i++)
	{
		if (bcm4760_hw_hib_regs[i].Size == 1)
			bcm4760_hw_hib_regs[i].OrValue = readb(IO_ADDRESS(bcm4760_hw_hib_regs[i].Address));
		else if (bcm4760_hw_hib_regs[i].Size == 2)
			bcm4760_hw_hib_regs[i].OrValue = readw(IO_ADDRESS(bcm4760_hw_hib_regs[i].Address));
		else if (bcm4760_hw_hib_regs[i].Size == 4)
			bcm4760_hw_hib_regs[i].OrValue = readl(IO_ADDRESS(bcm4760_hw_hib_regs[i].Address));
		else
			PWRSEQ_DEBUG(DBG_ERROR, (KERN_ERR "Bad Size setting at index %d\n", i)); 
	}
}
  
/** @brief Restore registers from preservation struct for resume from suspend to memory
 *
 * This function will scan the structure defined by struct PwrSeqRegisterSave
 * and reload all the registers defined by that structure. The Mask value is
 * used to ensure that only the correct bits are updated. The inverse of the mask
 * is used to remove uninteresting or protected bits from the hardware register and
 * the normal representation of the Mask value is used to ensure that only the correct
 * bits from the saved value are presenved. The two calculated values are ORed together
 * and written back into the hardware register.
 *
 */
static void RestoreChipRegisters(void)
{
	unsigned int	i,j;

	for (i=0 ; i<(sizeof(bcm4760_hw_hib_regs) / sizeof(bcm4760_hw_hib_regs[0])) ; i++)
	{
		if (bcm4760_hw_hib_regs[i].Size == 1)
		{
			j = readb(IO_ADDRESS(bcm4760_hw_hib_regs[i].Address)) & ~bcm4760_hw_hib_regs[i].Mask;
			j |= (bcm4760_hw_hib_regs[i].OrValue & bcm4760_hw_hib_regs[i].Mask);
			writeb(j, IO_ADDRESS(bcm4760_hw_hib_regs[i].Address));
		}
		else if (bcm4760_hw_hib_regs[i].Size == 2)
		{
			j = readw(IO_ADDRESS(bcm4760_hw_hib_regs[i].Address)) & ~bcm4760_hw_hib_regs[i].Mask;
			j |= (bcm4760_hw_hib_regs[i].OrValue & bcm4760_hw_hib_regs[i].Mask);
			writew(j, IO_ADDRESS(bcm4760_hw_hib_regs[i].Address));
		}
		else if (bcm4760_hw_hib_regs[i].Size == 4)
		{
			j = readl(IO_ADDRESS(bcm4760_hw_hib_regs[i].Address)) & ~bcm4760_hw_hib_regs[i].Mask;
			j |= (bcm4760_hw_hib_regs[i].OrValue & bcm4760_hw_hib_regs[i].Mask);
			writel(j, IO_ADDRESS(bcm4760_hw_hib_regs[i].Address));
		}
		else
			PWRSEQ_DEBUG(DBG_ERROR, (KERN_ERR "Bad Size setting at index %d\n", i)); 
	}
}

/** @brief Force PM state change via PML state machine.
 *
 * This function forces a PM state change to signal a different power
 * management operating mode to the PMU via the PC lines. This is the
 * most desired way of managing manual PM state changes so that it does
 * not incur any manual override mechanisms. The PML logic needs to be
 * used to put the SoC into suspend states so if we don't use this method
 * of manual state switching then we have to re-enable the PML state
 * machine prior to entering suspend. While this seems to work under
 * test it is less desirable and probably more risky from a hardware
 * stability and timing race condition perspective.
 *
 * The primary reason for manual PM state machine changes is when a
 * different configuration of regulators/LDOs is required.
 *
 * The new_pm_state value is first loaded into the PM field of the
 * PML_R_PML_RETURN_ISOLATE register and then the PML state machine
 * is triggered manually via the PML_SW_START_PWRDOWN in the CMU_PML_CTL
 * register. We configure the PML block to generally do nothing, i.e.,
 * don't stop clocks or enable isolation logic. The only value we set
 * intended to cause some action is the new PM state of course. This
 * is set in the PC_NEXT_FOR_PMU in the PML_NEXT_ISOLATE register and
 * the pc_return_for_pmu in the PML_RETURN_ISOLATE register. It is set
 * in the same place just to make sure the PML never does two state
 * changes by going through the PM state value in both registers
 * sequentially.
 *
 * Typically new_pm_state will be different than the current
 * state so it will cause the PMU to transition to the new
 * state with an appropriate reconfiguration of LDO and
 * regulator state. If the current and new states are the
 * same this basically does nothing other than cause PML
 * state machine transitions.
 *
 * @param[in]	new_pm_state	New PM state to transition to.
 */
void pwrseq_trigger_pm_state_switch(int new_pm_state)
{
	uint32_t temp;

	/* unlock clock manager */
	writel(UNLOCK_CMU_SIGNATURE, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));

	writel(new_pm_state, IO_ADDRESS(PML_R_PML_NEXT_ISOLATE_MEMADDR));
	writel(new_pm_state, IO_ADDRESS(PML_R_PML_RETURN_ISOLATE_MEMADDR));

	writel(0, IO_ADDRESS(PML_R_PML_NEXT_STOPCLKS_MEMADDR));
	writel(0, IO_ADDRESS(PML_R_PML_RETURN_STOPCLKS_MEMADDR));

	writel(0, IO_ADDRESS(PML_R_PML_RETURN_POR_MEMADDR));

	temp =  (PML_DELAY_PWRDOWN<<PML_F_PWRDOWN_DELAY_R)	|\
		(PML_DELAY_ISOLATE<<PML_F_ISOLATE_DELAY_R)	|\
		(PML_DELAY_STOPCLOCKS<<PML_F_STOPCLKS_DELAY_R)	|\
		(PML_DELAY_SREFRESH<<PML_F_SREFRESH_DELAY_R);
	writel(temp, IO_ADDRESS(PML_R_PML_DELAYS0_MEMADDR));

	temp =  (PML_DELAY_ENABLEPOR<<PML_F_ENABLEPOR_DELAY_R)	|\
		(PML_DELAY_ENABLECLKS<<PML_F_ENABLECLKS_DELAY_R)|\
		(PML_DELAY_ENABLEISO<<PML_F_ENABLEISO_DELAY_R)	|\
		(PML_DELAY_ENABLEPWR<<PML_F_ENABLEPWR_DELAY_R);
	writel(temp, IO_ADDRESS(PML_R_PML_DELAYS1_MEMADDR));

	writel( (readl(IO_ADDRESS(PML_R_PML_CTL0_MEMADDR)) & 
		  ~PML_F_SW_PWRDOWN_EN_MASK), IO_ADDRESS(PML_R_PML_CTL0_MEMADDR));
	writel( (readl(IO_ADDRESS(PML_R_PML_CTL0_MEMADDR)) | 
		  (PML_F_MASK_ARM_STANDBYWFI_MASK | PML_F_MASK_ARM_DBGNOPWRDWN_MASK)),
		 IO_ADDRESS(PML_R_PML_CTL0_MEMADDR));

	/* edge trigger PML to start power down sequence */
	writel((readl(IO_ADDRESS(CMU_R_PML_CTL_MEMADDR)) & ~CMU_F_PML_SW_START_PWRDOWN_MASK),
		IO_ADDRESS(CMU_R_PML_CTL_MEMADDR));
	udelay(100);

	writel((readl(IO_ADDRESS(CMU_R_PML_CTL_MEMADDR)) | CMU_F_PML_SW_START_PWRDOWN_MASK),
		IO_ADDRESS(CMU_R_PML_CTL_MEMADDR));
	udelay(100);

	/* lock clock manager */
	writel(LOCK_CMU_SIGNATURE, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));
}

/** @brief Stop override of PM state via PML to software control/override.
 *
 * This function stops or ends an override of the PM state via the PML
 * software override mechanism. When the termination of the override occurs
 * the PML logic will return to the PM state in the PML_R_PML_RETURN_ISOLATE
 * register.
 *
 * The new_pm_state value is first loaded into the PM field of the
 * PML_R_PML_RETURN_ISOLATE register and then the PML_F_SW_PWRDOWN_EN bit
 * is cleared (set to zero).
 *
 * Other PML registers and fields related to this action are generally set
 * to "do nothing" sort of values.
 *
 * Typically new_pm_state will be different than the current
 * state so it will cause the PMU to transition to the new
 * state with an appropriate reconfiguration of LDO and
 * regulator state. If the current and new states are the
 * same this basically does nothing.
 *
 * @param[in]	new_pm_state	New PM state to transition to.
 */
void pwrseq_stop_pm_state_sw_override(int new_pm_state)
{
	uint32_t	temp;

	writel(new_pm_state, IO_ADDRESS(PML_R_PML_NEXT_ISOLATE_MEMADDR));
	writel(new_pm_state, IO_ADDRESS(PML_R_PML_RETURN_ISOLATE_MEMADDR));

	writel(0, IO_ADDRESS(PML_R_PML_NEXT_STOPCLKS_MEMADDR));
	writel(0, IO_ADDRESS(PML_R_PML_RETURN_STOPCLKS_MEMADDR));

	writel(0, IO_ADDRESS(PML_R_PML_RETURN_POR_MEMADDR));

	temp = (PML_DELAY_PWRDOWN<<PML_F_PWRDOWN_DELAY_R)|\
		   (PML_DELAY_ISOLATE<<PML_F_ISOLATE_DELAY_R)|\
		   (PML_DELAY_STOPCLOCKS<<PML_F_STOPCLKS_DELAY_R)|\
		   (PML_DELAY_SREFRESH<<PML_F_SREFRESH_DELAY_R);
	writel(temp, IO_ADDRESS(PML_R_PML_DELAYS0_MEMADDR));

	temp = (PML_DELAY_ENABLEPOR<<PML_F_ENABLEPOR_DELAY_R)|\
		   (PML_DELAY_ENABLECLKS<<PML_F_ENABLECLKS_DELAY_R)|\
		   (PML_DELAY_ENABLEISO<<PML_F_ENABLEISO_DELAY_R)|\
		   (PML_DELAY_ENABLEPWR<<PML_F_ENABLEPWR_DELAY_R);
	writel(temp, IO_ADDRESS(PML_R_PML_DELAYS1_MEMADDR));

	writel(readl(IO_ADDRESS(PML_R_PML_CTL0_MEMADDR)) &
            ~(PML_F_MASK_ARM_STANDBYWFI_MASK | PML_F_SW_PWRDOWN_EN_MASK),
	       IO_ADDRESS(PML_R_PML_CTL0_MEMADDR));
}

/** @brief Force PML to software control/override on PM state.
 *
 * This function sets the new PM state into the software
 * override register PM value and the sets the software
 * override bit in the PML logic.
 *
 * Typically new_pm_state will be different than the current
 * state so it will cause the PMU to transition to the new
 * state with an appropriate reconfiguration of LDO and
 * regulator state. If the current and new states are the
 * same this basically does nothing.
 *
 * @param[in]	new_pm_state	New PM state to transition to.
 */
void pwrseq_force_pm_state_sw_override(int new_pm_state)
{
	writel(new_pm_state, IO_ADDRESS(PML_R_PML_SW_PC_CTRL_MEMADDR));
	writel(readl(IO_ADDRESS(PML_R_PML_CTL0_MEMADDR)) | PML_F_SW_PWRDOWN_EN_MASK,
	       IO_ADDRESS(PML_R_PML_CTL0_MEMADDR));
}

/** @brief Return current PM state value.
 *
 * This function simply returns the value of the local variable
 * CurrentPmState.
 *
 * @return	Value of CurrentPmState variable.
 */
int pwrseq_get_current_pm_state(void)
{
	return CurrentPmState;
}

/** @brief Start switch to a new PM state using PML logic.
 *
 *	This function uses the PML logic to cause the PMU to perform a PM
 *	state transition. This is generally required as for Broadcom PMUs the
 *	only means of turning LDOs/regulators on and off is via such state
 *	transitions. This function actually only pre-configures the state
 *	transition but does not actually perform it. The concept is that
 *	A state transition is begun with this function, the new state
 *	is configured and then the state is "finalized". When the state is
 *	finalized the state transition actually occurs.
 *
 *	The basic flow for this function is as follows:
 *
 *	    -#. Get interlock mutex to prevent conflicts.
 *	    -#. Determine current state (CurrentPmState).
 *	    -#. Choose new state for new LDO/regulator configuration.
 *	    -#. Copy current state configuration to new state.
 *	    -#. Return allocated PM state (1 or 2).
 *
 * 	Note: Leaving the mutex locked is not an error. The mutex is
 *	      locked until the "finalize" is performed since the state
 *	      information must remained allocated and locked until.
 *	      the new state is configured and we switch to it. At that
 *	      time the mutex is unlocked and a new state transition
 *	      process can begin.
 *
 * @return			Value from 1-2 representing the new power
 *				management state allocated.
 *
 */
int pwrseq_switch_pm_states_begin(void)
{
	int	new_pm_state;
#if defined(CONFIG_BCM_PMU_BCM59040)
	uint8_t	read_val;
	int reg;
#endif

	if(mutex_lock_interruptible(&pwrseq_pmu_state_switch_mutex))
		return -1;

	if (CurrentPmState == 1) {
		new_pm_state = 2;
	} else {
		new_pm_state = 1;
	}

#if defined(CONFIG_BCM_PMU_BCM59040)
	for(reg=0 ; reg < 8 ; reg++) {
		if(new_pm_state == 2)
		{
			read_val = pmu_i2c_read(BCM59040_REG_L1PMCTRL + reg);
			read_val &= ~BCM59040_PMCTRL_PM2_MASK;
			read_val |= ((read_val & BCM59040_PMCTRL_PM1_MASK) << 2);
			(void)pmu_i2c_write(BCM59040_REG_L1PMCTRL + reg,
					    read_val);
		}
		else
		{
			read_val = pmu_i2c_read(BCM59040_REG_L1PMCTRL + reg);
			read_val &= ~BCM59040_PMCTRL_PM1_MASK;
			if(CurrentPmState == 0)
			{
				read_val |= ((read_val & BCM59040_PMCTRL_PM0_MASK) << 2);
				(void)pmu_i2c_write(BCM59040_REG_L1PMCTRL + reg,
							read_val);
			}
			else
			{
				read_val |= ((read_val & BCM59040_PMCTRL_PM2_MASK) >> 2);
				(void)pmu_i2c_write(BCM59040_REG_L1PMCTRL + reg,
							read_val);
			}
		}
	}
#endif

	return new_pm_state;
}

/** @brief Finalize switch to a new PM state using PML logic.
 *
 *	This function uses the PML logic to cause the PMU to perform a PM
 *	state transition. This is generally required as for Broadcom PMUs the
 *	only means of turning LDOs/regulators on and off is via such state
 *	transitions. This function "finalizes" the state transition and
 *	is the closing operation for bcm4760_switch_pm_states_begin. The
 *	new_pm_state argument must match that for the matching call to
 *	bcm4760_switch_pm_states_begin or the function is terminated and
 *	the state transition does not occur. Regardless of the success
 *	or failure of this function the mutex is released.
 *
 *	This function actually does the state transition whereas the
 *	bcm4760_switch_pm_states_begin function only grabs the mutex,
 *	allocates the next state and prepares the PMU LDO/regulator
 *	configuration for the new PM state.
 *
 *	The basic flow for this function is as follows:
 *
 *	    -#. Verify new_pm_state is correct value. Exit if not without
 *	        doing the actual PM state transition. Release the mutex.
 *	        Return failure exit code.
 *	    -#. Setup PML logic to perform state transition.
 *	    -#. Trigger PML logic to cause PM state change.
 *	    -#. Release mutex.
 *	    -#. Return success exit code.
 *
 * 	Note: Leaving the mutex locked is not an error. The mutex is
 *	      locked until the "finalize" is performed since the state
 *	      information must remained allocated and locked until.
 *	      the new state is configured and we switch to it. At that
 *	      time the mutex is unlocked and a new state transition
 *	      process can begin.
 *
 * @param[in]	new_pm_state	State to transition to. This value
 *								must be 1 or 2 and must match the
 *								value returned from the matching
 *								bcm4760_switch_pm_states_begin
 *								function call.
 * @return						0 if successful, 1 otherwise
 *
 */
int pwrseq_switch_pm_states_finalize(int new_pm_state)
{
	if((((new_pm_state == 1) && (CurrentPmState != 2)) ||
	    ((new_pm_state == 2) && (CurrentPmState != 1))) &&
	    (CurrentPmState != 0))
	{
		mutex_unlock(&pwrseq_pmu_state_switch_mutex);
		return 1;
	}

#ifndef BCM4760_PWRSEQ_PML_TRIGGER_SWITCH
	pwrseq_force_pm_state_sw_override(new_pm_state);
#else
	pwrseq_trigger_pm_state_switch(new_pm_state);
#endif

	/*
	 * We should be in the new PM state now actually with
	 * whatever LDO/regulator changes were configured. In a bit
	 * of hardware tap dancing we actually configure the PML logic
	 * for a state change to the "next" state and then clear
	 * the software override. When this bit is cleared the PML
	 * state machine returns to the settings programmed in the
	 * "next_state" registers. We make sure these are all basically
	 * no action except that the next state PM value is set to
	 * new_pm_state. This should be ignored by the PMU since no
	 * change in the PC lines actually occurs. At the end of this
	 * we should be nice and settled in the new PM state with all
	 * hardware in a friendly and open-minded state of mind for
	 * future activities.
	 */

#ifndef BCM4760_PWRSEQ_PML_TRIGGER_SWITCH
	pwrseq_stop_pm_state_sw_override(new_pm_state);
#endif

	CurrentPmState = new_pm_state;

	mutex_unlock(&pwrseq_pmu_state_switch_mutex);
	return 0;
}


static uint32_t init_pmu(void)
{
    uint32_t status = RET_OK;

#if defined(CONFIG_BCM_PMU_BCM59040)
	/* Before writing the HBCTRL register, ensure it keeps the right value of the PC lines. */
	read_val = pmu_i2c_read(BCM59040_REG_HBCTRL);
	status = pmu_i2c_write(BCM59040_REG_HBCTRL, (read_val & 0xC0) | 0xd); /* enable PCx to control PMx; disable hibernate mode */	
#endif

	return status;
}


/**
Args : val : value to write into IVICINTENCLEAR(0/1) register.
       hl  : low/high word to be write - An '1' to disable the corresponding interrupt.

Returns : RET_OK or RET_ERROR.
*/
uint32_t intrc_disable_intrs(uint32_t val, high_low_t hl)
{
    if ( hl == LOW_WORD ) { writel(val, IO_ADDRESS(VIC0_R_VICINTENCLEAR_MEMADDR) ) ; }
    else if ( hl == HIGH_WORD ) { writel(val, IO_ADDRESS(VIC1_R_VICINTENCLEAR_MEMADDR)) ; }
    else { return RET_ERROR ; }
    return(RET_OK);
}

/**
Args : val : value to write into VICINTENABLE(0/1) register.
       hl  : low/high word to be write. - An '1' to enable the corresponding interrupt.

Returns : RET_OK or RET_ERROR.
*/
uint32_t intrc_enable_intrs(uint32_t val, high_low_t hl)
{
    if ( hl == LOW_WORD ) { writel(val, IO_ADDRESS(VIC0_R_VICINTENABLE_MEMADDR) ) ; } 
    else if ( hl == HIGH_WORD ) {  writel(val, IO_ADDRESS(VIC1_R_VICINTENABLE_MEMADDR)) ; }
    else { return RET_ERROR ; }
    return(RET_OK);
}

/**
Args :  mode : Transition type.

Returns : RET_OK or RET_ERROR.
*/
static unsigned int bcm4760_pwrmgr_start_transition (PWRMGR_PWRMODE_T power_mode)
{
    uint32_t status = RET_OK;
    uint32_t nextState, retState, nextStopClock, retStopClock, retPOR;
    uint32_t delays0, delays1;
    uint32_t temp;

/* unlock clock manager */
	writel(UNLOCK_CMU_SIGNATURE, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));

/* Unmask activity on GPIO3 (A0) or GPIO1 (B0) (PONKEY) */

#ifndef CONFIG_BCM4760_PMSTR_SW_PWRDOWN
	writel(readl(IO_ADDRESS(PML_R_PML_CTL0_MEMADDR)) &
            ~(PML_F_MASK_ARM_STANDBYWFI_MASK | PML_F_SW_PWRDOWN_EN_MASK),
	       IO_ADDRESS(PML_R_PML_CTL0_MEMADDR));
	writel((readl(IO_ADDRESS(PML_R_PML_CTL0_MEMADDR)) |
           PML_F_MASK_ARM_DBGNOPWRDWN_MASK),
	       IO_ADDRESS(PML_R_PML_CTL0_MEMADDR));

	udelay(50); // delay a bit to let new PM state settle, we should be back in PM0

	nextState = power_mode;
	nextState |= (PML_F_GPS_BB_NEXT_ISOLATE_MASK |
				  PML_F_GPS_RF_NEXT_ISOLATE_MASK |
				  PML_F_AON_NEXT_ISOLATE_MASK);
	writel(nextState, IO_ADDRESS(PML_R_PML_NEXT_ISOLATE_MEMADDR));
	
	retState = PWRMGR_PWRMODE_0;	//PCx = 2b'00 - Normal mode	
	writel(retState, IO_ADDRESS(PML_R_PML_RETURN_ISOLATE_MEMADDR));
#else
	writel(readl(IO_ADDRESS(PML_R_PML_CTL0_MEMADDR)) &
            ~PML_F_SW_PWRDOWN_EN_MASK,
	       IO_ADDRESS(PML_R_PML_CTL0_MEMADDR));
	writel(readl(IO_ADDRESS(PML_R_PML_CTL0_MEMADDR)) |
            (PML_F_SW_ISOLATE_EN_MASK | PML_F_MASK_ARM_DBGNOPWRDWN_MASK),
	       IO_ADDRESS(PML_R_PML_CTL0_MEMADDR));
	nextState = power_mode;
	writel(nextState, IO_ADDRESS(PML_R_PML_NEXT_ISOLATE_MEMADDR));
	
	retState = PWRMGR_PWRMODE_0;	//PCx = 2b'00 - Normal mode
	writel(retState, IO_ADDRESS(PML_R_PML_RETURN_ISOLATE_MEMADDR));
#endif

	nextStopClock = PML_F_BM4_NEXT_STOPCLK_MASK + PML_F_BM3_NEXT_STOPCLK_MASK + \
			PML_F_BM2_NEXT_STOPCLK_MASK + PML_F_BM1_NEXT_STOPCLK_MASK + \
			PML_F_BM0_NEXT_STOPCLK_MASK + PML_F_ARM_NEXT_STOPCLK_MASK + \
			PML_F_GPS_APB_NEXT_STOPCLK_MASK + PML_F_DDR_NEXT_STOPCLK_MASK;
	writel(nextStopClock, IO_ADDRESS(PML_R_PML_NEXT_STOPCLKS_MEMADDR));

	retStopClock = 0;
	writel(retStopClock, IO_ADDRESS(PML_R_PML_RETURN_STOPCLKS_MEMADDR));
	
	retPOR = PML_F_GPSBB_POR_MASK + PML_F_CORE_POR_MASK;
	writel(retPOR, IO_ADDRESS(PML_R_PML_RETURN_POR_MEMADDR));

	delays0 = (PML_DELAY_PWRDOWN << PML_F_PWRDOWN_DELAY_R) |
			  (PML_DELAY_ISOLATE << PML_F_ISOLATE_DELAY_R) |
			  (PML_DELAY_STOPCLOCKS << PML_F_STOPCLKS_DELAY_R) |
			  (PML_DELAY_SREFRESH << PML_F_SREFRESH_DELAY_R);
    PWRSEQ_DEBUG(DBG_TRACE2,( "Write %08X to PML_DELAYS0.\n", delays0 ));
	writel(delays0, IO_ADDRESS(PML_R_PML_DELAYS0_MEMADDR));
	delays0 = readl(IO_ADDRESS(PML_R_PML_DELAYS0_MEMADDR));
    PWRSEQ_DEBUG(DBG_TRACE2,( "Read-back %08X from PML_DELAYS0.\n", delays0 ));
	
	delays1 = (PML_DELAY_ENABLEPOR << PML_F_ENABLEPOR_DELAY_R) |
			  (PML_DELAY_ENABLECLKS << PML_F_ENABLECLKS_DELAY_R) |
			  (PML_DELAY_ENABLEISO << PML_F_ENABLEISO_DELAY_R) |
			  (PML_DELAY_ENABLEPWR << PML_F_ENABLEPWR_DELAY_R);
    PWRSEQ_DEBUG(DBG_TRACE2,( "Write %08X to PML_DELAYS1.\n", delays1 ));
	writel(delays1, IO_ADDRESS(PML_R_PML_DELAYS1_MEMADDR));
	delays1 = readl(IO_ADDRESS(PML_R_PML_DELAYS1_MEMADDR));
    PWRSEQ_DEBUG(DBG_TRACE2,( "Read-back %08X from PML_DELAYS1.\n", delays1 ));

    	writel( (readl(IO_ADDRESS(PML_R_PML_SW_PWRDOWN_MEMADDR)) & 
		 ~PML_F_SIO_GPIO_HOLD_3P3_MASK) | 
		  PML_F_RTC_CLAMP_ON_3P3_MASK, 
		  IO_ADDRESS(PML_R_PML_SW_PWRDOWN_MEMADDR) );

	udelay(100);

/* Turn off DbgNoPwrDown in CP14 */
	__asm ("mrc p14,#0,r0,c0,c1,#0");
	__asm ("bic r0,r0,#1<<9");
	__asm ("mcr p14,#0,r0,c0,c1,#0");

#if 1
/* edge trigger PML to start power down sequence */
	temp = readl(IO_ADDRESS(CMU_R_PML_CTL_MEMADDR));
	writel(temp & ~CMU_F_PML_SW_START_PWRDOWN_MASK,
		   IO_ADDRESS(CMU_R_PML_CTL_MEMADDR));

	udelay(100);

	writel(temp | CMU_F_PML_SW_START_PWRDOWN_MASK, IO_ADDRESS(CMU_R_PML_CTL_MEMADDR)); 
#else
/* For testing, use SW to manually control PCx */
	temp = readl(IO_ADDRESS(PML_R_PML_CTL0_MEMADDR));
	writel((temp | (PML_F_MASK_ARM_STANDBYWFI_MASK | PML_F_SW_PWRDOWN_EN_MASK | PML_F_MASK_ARM_DBGNOPWRDWN_MASK)),
	       IO_ADDRESS(PML_R_PML_CTL0_MEMADDR));
	temp = readl (IO_ADDRESS(PML_R_PML_SW_PC_CTRL_MEMADDR));
	writel ((temp | 0x1),IO_ADDRESS(PML_R_PML_SW_PC_CTRL_MEMADDR)); /* hibernate mode PCx */
#endif

	udelay(100);

/* lock clock manager */
	writel(LOCK_CMU_SIGNATURE, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));
				
    return status;
}

/**
Args : None.

Returns : RET_OK or RET_ERROR.
*/
unsigned int bcm4760_pwrmgr_restore_run_to_sleep_intrs(void)
{
	uint32_t ret_val = RET_OK ;
	
	// Enable all the low word interrupts.
	ret_val = intrc_enable_intrs(g_wake_up_set_register_val_lower, LOW_WORD) ;
	if ( ret_val != RET_OK )  { return ret_val ; }

	// Enable all the high word interrupts.
	ret_val = intrc_enable_intrs(g_wake_up_set_register_val_higher, HIGH_WORD) ;
	if ( ret_val != RET_OK ) { return ret_val ; }

	g_wake_up_set_register_val_lower = 0 ;
	g_wake_up_set_register_val_higher = 0 ;

	return ret_val ;
}

/**
Args : *val : value of IMR0/1 register.
       hl   : low/high word to be read.

Returns : RET_OK or RET_ERROR.
*/
uint32_t intrc_save_intrs(uint32_t *val, high_low_t hl)
{
	if ( hl == LOW_WORD ) { 
		*val = readl(IO_ADDRESS(VIC0_R_VICINTENABLE_MEMADDR)); 
	} else if ( hl == HIGH_WORD ) { 
		*val = readl(IO_ADDRESS(VIC1_R_VICINTENABLE_MEMADDR)); 
	} else { 
		return RET_ERROR ; 
	}

	return(RET_OK);
}

/**
Args : None.

Returns : RET_OK or RET_ERROR.
*/
unsigned int bcm4760_pwrmgr_save_run_to_sleep_intrs(void)
{
	uint32_t ret_val = RET_OK ;

	//clear
	g_wake_up_set_register_val_lower = 0;
	g_wake_up_set_register_val_higher = 0;
	ret_val = intrc_save_intrs(&g_wake_up_set_register_val_lower, LOW_WORD ) ;
	if ( ret_val != RET_OK ) { return ret_val ; }

	ret_val = intrc_save_intrs(&g_wake_up_set_register_val_higher, HIGH_WORD ) ;
	if ( ret_val != RET_OK )  { return ret_val ; }

	return ret_val;
}

/**
Args : None.

Returns : RET_OK or RET_ERROR.
*/
unsigned int bcm4760_pwrmgr_sleep_to_run_intr_handler(void)
{
	uint32_t ret_val = RET_OK ;

	PWRSEQ_DEBUG(DBG_INFO, (KERN_INFO "pwrseq: bcm4760_pwrmgr_sleep_to_run_intr_handler()\n"));
	
	// Clear the interrupt status of pml
	writel(PML_F_CLR_INTR_MASK, IO_ADDRESS(PML_R_PML_CTL1_MEMADDR));
	udelay(5);
	writel(0, IO_ADDRESS(PML_R_PML_CTL1_MEMADDR));

	return ret_val ;
}


/****
Args : state is suspend state being requested.

Returns : 0 if state not supported, 1 if OK.
****/
unsigned int bcm4760_pm_suspend_valid(suspend_state_t state)
{

	if ((state != PM_SUSPEND_STANDBY) && (state != PM_SUSPEND_MEM)) {
		return 0;
	}

	PWRSEQ_DEBUG(DBG_INFO, (KERN_INFO "pwrseq: pm_suspend_valid\n"));

	return 1;
}

/****
Args : None.

Returns : 0 if prepare is ok, linux error code otherwise.
****/
unsigned int bcm4760_pm_suspend_prepare(void)
{

	PWRSEQ_DEBUG(DBG_INFO, (KERN_INFO "pwrseq: pm_suspend_prepare\n"));

	// Allocate memory needed to store context for suspend to memory.
	if((cpu_context = kmalloc(8192,GFP_KERNEL)) == NULL)
		return -ENOMEM;

	/*
	* Return the PML state machine to the state needed for suspend/resume
	* operation and force PMU state to PM0.
	*/

	pwrseq_stop_pm_state_sw_override(0);

	return 0;
}

/****
Args : None.

Returns : RET_OK or RET_ERROR.
****/
unsigned int bcm4760_pm_suspend_standby(void)
{
    // Transition to sleep
    if( bcm4760_pwrmgr_start_transition(PWRMGR_PWRMODE_3) )	
		return RET_ERROR;

	PWRSEQ_DEBUG(DBG_INFO, (KERN_INFO "pwrseq: just before WFI\n"));

	/*
	 * NOTE: the Wait-for-Interrupt instruction needs to be
	 * in icache so no SDRAM accesses are needed until the
	 * wakeup IRQ occurs and self-refresh is terminated.
	 */
	asm("b 1f; .align 5; 1:");
	asm("cpsid i");						/* disable IRQ */
	asm("mov r0,#0");                   /* make sure R0 is 0 */
	asm("mcr p15, 0, r0, c7, c10, 4");	/* drain write buffer */
	asm("mcr p15, 0, r0, c7, c0, 4");	/* wait for interrupt */

	PWRSEQ_DEBUG(DBG_INFO, (KERN_INFO "pwrseq: just after WFI\n"));

	return RET_OK ;
}


/****
Args : None.

Returns : RET_OK or RET_ERROR.
****/
unsigned int bcm4760_pm_suspend_mem(void)
{
	uint32_t sleep_status;

	// Save intrs.
	if( bcm4760_pwrmgr_save_run_to_sleep_intrs() )
		return RET_ERROR;

	// Now the real work starts. First we must call the init_bbl
	// function to save the DDR restore values for the Boot ROM
	// warm boot procedure. We also store the restart address
	// in BBL also. The restart address is the argument to the
	// init_bbl function.

	PWRSEQ_DEBUG(DBG_INFO, (KERN_INFO "Calling init_bbl\n"));

	init_bbl((void*)bcm476x_cpu_exit_sleep);

	PWRSEQ_DEBUG(DBG_INFO, (KERN_INFO "BBL Warm Boot Address is %08X\n",
						    rtc_read(BBL_OFFSET_DRAM_ADDRESS)));

	/*
	 * Disable all VIC interrupts except interrupts that can also be PML
	 * wake-up events until after wakeup. Linux isn't set up to handle them
	 * after we enter suspend to memory code/state.
	 */

	writel(0xFFFFFFFF & ~((1 << BCM4760_INTR_PML0) |
						  (1 << BCM4760_INTR_PML1) |
						  (1 << BCM4760_INTR_GPIO00) |
						  (1 << BCM4760_INTR_TIM2_CNTR1)),
		   IO_ADDRESS(VIC0_R_VICINTENCLEAR_MEMADDR));
	writel(0xFFFFFFFF & ~(1 << (BCM4760_INTR_TIM2_CNTR2-32)),
		   IO_ADDRESS(VIC1_R_VICINTENCLEAR_MEMADDR));
	writel(((1 << BCM4760_INTR_PML0) |
			(1 << BCM4760_INTR_PML1) |
			(1 << BCM4760_INTR_GPIO00) |
			(1 << BCM4760_INTR_TIM2_CNTR1)),
		   IO_ADDRESS(VIC0_R_VICINTENABLE_MEMADDR));
	writel((1 << (BCM4760_INTR_TIM2_CNTR2 - 32)),
		   IO_ADDRESS(VIC1_R_VICINTENABLE_MEMADDR));

	// Save hardware registers that are generally used, device specific registers
	// should be handled by drivers for those devices.
	SaveChipRegisters();

	// Transition to sleep
	if( bcm4760_pwrmgr_start_transition(PWRMGR_PWRMODE_3) )	
		return RET_ERROR;

	// As movinands are rather picky about the amount of idle time before powering off, delay here for 500ms.
	mdelay( 500 );

	// Next we must call the actual sleep entry function. It will
	// preserve all processor state information and perform the
	// WFI. The WFI will generate the internal STANDBY_WFI signal
	// that triggers the PML logic to enter the state programmed
	// previously. This includes putting the DDR in self-refresh,
	// turning off pretty much all logic blocks including the
	// processor core. On a warm boot (triggered by one of the
	// programmed wake-up events) the Boot ROM will restore the
	// DDR controller settings and jump to the address we provided
	// with the init_bbl call above. That will restore the saved
	// processor state and other key settings and eventually the
	// sleep entry function will return and execution will resume

	PWRSEQ_DEBUG(DBG_INFO, (KERN_INFO "pwrseq: just before bcm476x_cpu_enter_sleep\n"));

	sleep_status = bcm476x_cpu_enter_sleep(cpu_context);

	// Disable GPIO PAD retention as this would effectively make JTAG useless
	writel( readl(IO_ADDRESS(PML_R_PML_SW_PWRDOWN_MEMADDR)) & 
		 ~(PML_F_SIO_GPIO_HOLD_3P3_MASK | PML_F_RTC_CLAMP_ON_3P3_MASK), 
		  IO_ADDRESS(PML_R_PML_SW_PWRDOWN_MEMADDR) );
	/* switch off terminators */
	writel(readl(IO_ADDRESS(CMU_R_USB_PHY1_MEMADDR)) | 0x00001000,
		IO_ADDRESS(CMU_R_USB_PHY1_MEMADDR));

	/* disable BC detect logic */
	writel(readl(IO_ADDRESS(CMU_R_BC_DETECT_CTRL_MEMADDR)) | 0x00000006,
		IO_ADDRESS(CMU_R_BC_DETECT_CTRL_MEMADDR));

	PWRSEQ_DEBUG(DBG_INFO, (KERN_INFO "pwrseq: just after bcm476x_cpu_enter_sleep (sleep_status=%08X)\n", sleep_status));

#ifdef CONFIG_BCM4760_PMSTR_BBL4760_WARMSTART

	if(((rtc_read(BBL_OFFSET_DDR_TOMTOM_OFFSET)&0x00FF0000)>>16)==0x01)
	{
		cds_pll_init(1);	/* reprogram core PLL to 700MHz */
		cds_pll_switch(0, 0);	/* switch to primary channel for high freq (700/667MHz) */
	}
	else
	{
		cds_pll_init(0);	/* reprogram core PLL */
		cds_pll_switch(0, 1);	/* switch from 400 to 500 Mhz on processor */
	}
	

#endif

	// Restore hardware registers that are generally used.
	RestoreChipRegisters();

	/*
	 * Turn off audio block DAC power to eliminate noise when audio LDO
	 * is turned on. It should have been off on reset but hardware was
	 * incorrectly configured. This is a workaround for A0, B0 and B1
	 * BCM4760 devices.
	 */

	IF_BCM4760_B1_OR_EARLIER {
		writel(inw(IO_ADDRESS(CMU_R_DAC_AUDIO_CTRL0_MEMADDR)) |
			(CMU_F_DAH_PWRDNDRVR_MASK |
				CMU_F_DAH_PWRDNDRVL_MASK | CMU_F_DAH_PWRDNDACR_MASK |
				CMU_F_DAH_PWRDNDACL_MASK | CMU_F_DAH_PWRDND2C_MASK),
			IO_ADDRESS(CMU_R_DAC_AUDIO_CTRL0_MEMADDR));
	}

	/*
	 * This call basically puts us back into the PM state we were in when
	 * we started prior to the suspend. For the most stable boot conditions
	 * we let the PML start us in PM0 which is the power-on PM state. We
	 * may have configured regulators to lower voltages or turned LDOs off
	 * prior to entering suspend. That might cause problems or even failures
	 * during boot. Rather than trying to verify that the state we're in
	 * when suspend starts is valid to boot in we just let the boot occur in
	 * PM0 which we know is good.
	 */

#ifndef BCM4760_PWRSEQ_PML_TRIGGER_SWITCH
	pwrseq_force_pm_state_sw_override(CurrentPmState);
#endif

	deinit_bbl();

	// Restore all interrupts.
	if (bcm4760_pwrmgr_restore_run_to_sleep_intrs())
		return RET_ERROR;

	return RET_OK;
}

/****
Args : None.

Returns : RET_OK or RET_ERROR.
****/
void bcm4760_pm_suspend_finish(void)
{

	PWRSEQ_DEBUG(DBG_INFO, (KERN_INFO "pwrseq: pm_suspend_finish\n"));

	/*
	* Return the PML state machine to the state we were in
	* prior to starting the suspend. We should be here already but
	* in case the suspend terminated prematurely we may not
	* have correct restored the PM state so we do it here to be certain.
	*/

	pwrseq_stop_pm_state_sw_override(CurrentPmState);

	if(cpu_context)
	{
		kfree(cpu_context);
		cpu_context = NULL;
	}

}

/****
Args : 
    irq : irq number.

Returns : IRQ_HANDLED.
****/
static irqreturn_t bcm47xx_pwrseq_interrupt_handler(int irq, void *dev_id)
{
	bcm4760_pwrmgr_sleep_to_run_intr_handler() ;

	return IRQ_HANDLED ;
}

static struct __init irqaction bcm47xx_pwrseq_callback_irq_action = {
	.name = "BCM28XX POWER SEQ CALLBACK ACTION",
	.flags = 0,
	.handler = &bcm47xx_pwrseq_interrupt_handler
};

/* bcm4760_i2c_probe
 *
 * called by the bus driver when a suitable device is found
 */
static int bcm4760_pwrseq_probe(struct platform_device *pdev)
{
	int	ret=0;
	
	/* unmask and clear out any pending pml intr */    
	writel(PML_F_CLR_INTR_MASK, IO_ADDRESS(PML_R_PML_CTL1_MEMADDR));
	writel(0, IO_ADDRESS(PML_R_PML_CTL1_MEMADDR));
	
	/* register interrupt handler */
	setup_irq(BCM4760_INTR_PML0, &bcm47xx_pwrseq_callback_irq_action);

	/*
	 *QP TODO: Init Power mode settings in pmu. This is a temp place for the routine. Eventually it
	 * may be in bcm59040.c
	 */
	init_pmu();

	mutex_init(&pwrseq_pmu_state_switch_mutex);

	/*
	 * Initial PM state switch to get us from PM0 into PM1. PM1 is our initial
	 * operating PM state.
	 */

	CurrentPmState = 1;
#ifndef BCM4760_PWRSEQ_PML_TRIGGER_SWITCH
	pwrseq_force_pm_state_sw_override(CurrentPmState);
#else
	pwrseq_trigger_pm_state_switch(CurrentPmState);
#endif

	PWRSEQ_DEBUG(DBG_INFO, (KERN_INFO "pwrseq is loaded\n"));

	return ret;
}

#ifdef CONFIG_PM
static int bcm4760_pwrseq_suspend(struct device *dev)
{
	struct platform_device *pdev;

	pdev = to_platform_device(dev);
	PWRSEQ_DEBUG(DBG_TRACE,( "Suspend processed for %s.\n", pdev->name ));

	return 0;
} /* bcm4760_pwrseq_suspend */

static int bcm4760_pwrseq_resume(struct device *dev)
{
	struct platform_device *pdev;

	pdev = to_platform_device(dev);
	PWRSEQ_DEBUG(DBG_TRACE,( "Resume processed for %s.\n", pdev->name ));

	/*
	 * Put us back in original PM state.
	 */

	mutex_lock(&pwrseq_pmu_state_switch_mutex);

#ifndef BCM4760_PWRSEQ_PML_TRIGGER_SWITCH
	pwrseq_force_pm_state_sw_override(CurrentPmState);
#else
	pwrseq_trigger_pm_state_switch(CurrentPmState);
#endif

	mutex_unlock(&pwrseq_pmu_state_switch_mutex);

    /*
     * Turn off GPIO/pin state retention.
     *
     * Caution: All GPIOs and pins should have been remuxed and reprogrammed
     *          properly by now. If they aren't then there will be glitches
     *          on the lines.
     */
    writel(readl(IO_ADDRESS(PML_R_PML_SW_PWRDOWN_MEMADDR)) &
           ~(PML_F_SIO_GPIO_HOLD_3P3_MASK | PML_F_RTC_CLAMP_ON_3P3_MASK),
           IO_ADDRESS(PML_R_PML_SW_PWRDOWN_MEMADDR));

    return 0;
} /* bcm4760_pwrseq_resume */

#else
#define bcm4760_pwrseq_suspend NULL
#define bcm4760_pwrseq_resume NULL
#endif

static struct pm_ext_ops bcm4760_pwrseq_pm_ops =
{
    .base =
    {
        .suspend    = bcm4760_pwrseq_suspend,
        .resume     = bcm4760_pwrseq_resume
    }
};

static struct platform_driver bcm4760_pwrseq_driver = {
	.probe 		= bcm4760_pwrseq_probe,
	.pm		= &bcm4760_pwrseq_pm_ops,
	.driver         = {
		.name   = "bcm4760_pwrseq",
		.owner  = THIS_MODULE,
	},
};

/****
Args : None.

Returns : 0 on success or -ve number.
****/
static int __init bcm4760_init_pwrseq(void)
{
	int ret;
	/* Register as a platform driver */

	ret = platform_driver_register(&bcm4760_pwrseq_driver);

	if (ret)
		PWRSEQ_DEBUG(DBG_ERROR, (KERN_ERR "pwrseq driver failed to register!"));

	return ret;
}

late_initcall(bcm4760_init_pwrseq);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM4760 PwrSeq Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.2");
