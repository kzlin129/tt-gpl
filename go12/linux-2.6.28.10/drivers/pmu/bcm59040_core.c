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

/** BCM59040 Core Driver Module
 *****************************************************************************
 *
 *     This implements the BCM59040 device core functionality. The core
 *     includes the IRQChip "emulation" layer, the PMU Bus backend driver
 *     and the generic IOCTL layer.
 *
 *     The IRQChip emulation layer translates BCM59040 specific interrupt
 *     events identified from the interrupt registers into individual
 *     IRQ events that are passed onwards as different IRQChip events. Other
 *     devices register standard Linux ISRs for handling these events. It
 *     also ensures that PMU interrupts are properly configured for the
 *     suspend state. This means that any interrupt exported by the IRQChip
 *     emulation layer that is set as a wakeup event remains enabled during
 *     suspend while any interrupt that is not set as a wakeup event is
 *     masked while in suspend. The prior state of the interrupt masking
 *     is preserved and correctly restored during the resume process.
 *
 *     The PMU Bus backend device translates the underlying I2C communication
 *     required for the BCM59040 into a more generic interface. This allows
 *     abstraction from the specific communications method used by a specific
 *     PMU device.
 *
 *     The generic IOCTL layer takes all input IOCTL calls and routes them
 *     to a set of registered child devices based upon the range of IOCTL
 *     values claimed by a particular child device. The child devices make
 *     calls to this core driver to request a range of IOCTL values for
 *     themselves. If the range overlaps the values already claimed by
 *     another child driver then the request is refused, otherwise it
 *     is accepted by the core driver and registered to that child device.
 *     When an IOCTL is received that has a value within the range claimed
 *     by a child device the IOCTL is forward to that child device for
 *     handling. If the IOCTl value is unknown (unclaimed) and standard
 *     error code is returned.
 *
 * Copyright (c) 2009-2010 TomTom B.V
 * Copyright (c) 2009-2010 Broadcom Corporation
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
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#ifdef CONFIG_REGULATOR
#include <linux/regulator/machine.h>
#endif

#include <linux/pmu_device.h>
#include <asm/arch/pmu_device_bcm59040.h>

/* Include external interfaces */

#include <linux/broadcom/pmu_bcm59040.h>
#include <asm/arch/bcm59040_irqs.h>

/*
 * Structure and macro definitions.
 */

#define BCM59040_CORE_MOD_DESCRIPTION	"BCM59040 Core Driver"
#define BCM59040_CORE_MOD_VERSION		1.0

#undef BCM59040_DEVNAME
#define BCM59040_DEVNAME	"bcm59040"
#define PFX			BCM59040_DEVNAME ": "

#define BCM59040_I2C_ADDR_IND	0
#define BCM59040_I2C_DATA_IND	1

#define BCM59040_I2C_MSG_LEN	2

#define BCM59040_PAGE_MAX_ADDR 0xff

#define BCM59040_VIRQ_NAME_MAX_LEN 64

typedef enum {EPAGE_0, EPAGE_1} e_page;
typedef enum {EIRQ_UNMASK, EIRQ_MASK} e_irq_mask;

typedef struct
{
	u8			masked;
	u8			pending;
	u8			wakeup;
} bcm59040_virq_state_t;

typedef struct _bcm59040_ctxt
{
	struct i2c_client	*client;
	unsigned char		page;

	struct semaphore	sem_isr;
	struct task_struct	*ktask;

	struct work_struct	update_mask_work;

	/* virtualized register values */
	/* interrupt register */
	unsigned char           int_reg[BCM59040_NUM_INT_REG];
	/* mask register that users see and use */
	unsigned char           mask_reg[BCM59040_NUM_INT_REG];
	/* shadow mask register that holds the value written to the device */
	unsigned char           device_mask_reg[BCM59040_NUM_INT_REG];
	/* spinlock to protect the virtual mask registers */
	spinlock_t		virtual_mask_lock;

	/* virtual IRQ chip */
	bcm59040_virq_state_t	virq[BCM59040_NUM_IRQS];

	struct pmu_adapter	adapter;
} bcm59040_ctxt_t;

/* Assumption is that the offset of the interrupt and its related mask
 * register from their respective first registers are always equal.
 * I.e (INTx - INT1) == (INTxM - INT1M)
 */
typedef struct virtual_int_registers
{
	u16			int_reg_addr;	/* interrupt register address */
	u16			reg_mask;	/* mask value for register */
} virtual_int_registers_t;

typedef struct virtual_int_entry
{
	int				int_vector;		/* vector number of interrupt */
	int				num_registers;		/* number of registers in register list */
	const virtual_int_registers_t	*reg_list;		/* List of associated interrupt registers */
} virtual_int_entry_t;

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG 1

#define DBG_INFO	0x01
#define DBG_TRACE	0x02
#define DBG_TRACE2	0x04
#define DBG_DATA	0x08
#define DBG_VIRQ_STATE	0x10
#define DBG_VIRQ_DATA	0x20

#ifdef DEBUG
//#define DBG_DEFAULT_LEVEL	(DBG_INFO | DBG_DATA)
#define DBG_DEFAULT_LEVEL	0


static const int gLevel = DBG_DEFAULT_LEVEL;

#	define PMU_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define PMU_DEBUG(level,x)
#endif

static int bcm59040_bus_read(struct pmu_adapter *adap, int address, void *data, int len);
static int bcm59040_bus_write(struct pmu_adapter *adap, int address, void *data, int len);

static void bcm59040_virq_update_pending_irq(bcm59040_ctxt_t *ctxt, int irq);

static void bcm59040_virq_set_mask(unsigned int irq, e_irq_mask mask);



/*
 * Local variables.
 */

static const virtual_int_registers_t bcm59040_vint_table_ponkey[] = {
	{ BCM59040_REG_INT1, (BCM59040_INT1_PONKEYR |
				BCM59040_INT1_PONKEYF |
				BCM59040_INT1_PONKEYH |
				BCM59040_INT1_PONKEYBHD |
				BCM59040_INT1_RESTARTH |
				BCM59040_INT1_RESTARTON) }
};

static const virtual_int_registers_t bcm59040_vint_table_hibint[] = {
	{ BCM59040_REG_INT1, BCM59040_INT1_HBINT }
};

static const virtual_int_registers_t bcm59040_vint_table_pmutoowarm[] = {
	{ BCM59040_REG_INT1, BCM59040_INT1_PMUTOOWARM }
};

static const virtual_int_registers_t bcm59040_vint_table_chargerstatus[] = {
	{ BCM59040_REG_INT2, (BCM59040_INT2_CHGINS |
				BCM59040_INT2_CHGRM |
				BCM59040_INT2_CHGOVERV |
				BCM59040_INT2_USBINS |
				BCM59040_INT2_USBRM |
				BCM59040_INT2_USBOVERV ) },
	{ BCM59040_REG_INT3, (BCM59040_INT3_VSROVERV |
				BCM59040_INT3_VSROVERI |
				BCM59040_INT3_VCHGRNOTOK |
				BCM59040_INT3_CHG_WDT_ALARM |
				BCM59040_INT3_VBUSLOWBND |
				BCM59040_INT3_CHGWDTEXP |
				BCM59040_INT3_IDOVERI) },
	{ BCM59040_REG_INT9, (BCM59040_INT9_RESUME_VBUS |
				BCM59040_INT9_ID_CHNG) }
};

static const virtual_int_registers_t bcm59040_vint_table_regulatorstatus[] = {
	{ BCM59040_REG_INT4, (BCM59040_INT4_LDO1OVRI |
				BCM59040_INT4_LDO2OVRI |
				BCM59040_INT4_LDO3OVRI |
				BCM59040_INT4_LDO4OVRI |
				BCM59040_INT4_LDO5OVRI |
				BCM59040_INT4_LDO6OVRI) },
	{ BCM59040_REG_INT5, (BCM59040_INT5_IOSROVRI |
				BCM59040_INT5_CSROVRI |
				BCM59040_INT5_IOSROVRV |
				BCM59040_INT5_CSROVRV) }
};

static const virtual_int_registers_t bcm59040_vint_table_battstatus[] = {
	{ BCM59040_REG_INT2, BCM59040_INT2_EOC },
	{ BCM59040_REG_INT4, (BCM59040_INT4_BBLOW |
				BCM59040_INT4_FGC) },
	{ BCM59040_REG_INT6, (BCM59040_INT6_MBRM | BCM59040_INT6_LOWBAT | BCM59040_INT6_VERYLOWBAT)},
	{ BCM59040_REG_INT3, BCM59040_INT3_VBUSLOWBND }
};

static const virtual_int_registers_t bcm59040_vint_table_rtc[] = {
	{ BCM59040_REG_INT5, (BCM59040_INT5_RTCADJ |
				BCM59040_INT5_RTC1S |
				BCM59040_INT5_RTC60S) }
};

static const virtual_int_registers_t bcm59040_vint_table_rtc_alarm[] = {
	{ BCM59040_REG_INT5, BCM59040_INT5_RTCA1 }
};

static const virtual_int_registers_t bcm59040_vint_table_usb[] = {
	{ BCM59040_REG_INT2, BCM59040_INT2_CHGDET },
	{ BCM59040_REG_INT7, (BCM59040_INT7_VBUS_VALID_F |
				BCM59040_INT7_B_SESSEND_F |	/* Very important for Windows to recognize the device the first time... */
				BCM59040_INT7_ID_INSRT |	/* For Poznan */
				BCM59040_INT7_VBUS_VALID_R |	/* Not sure why */
				BCM59040_INT7_A_SESSVALID_R |	/* For device mode */
				BCM59040_INT7_ID_RMV)		/* For Poznan */
	},
};

static const virtual_int_registers_t bcm59040_vint_table_adc[] = {
	{ BCM59040_REG_INT8, (BCM59040_INT8_SARCONVRDY0 |
				BCM59040_INT8_SARCONVRDY1 |
				BCM59040_INT8_SARCONVRDY2 |
				BCM59040_INT8_SARCONVRDY3 |
				BCM59040_INT8_SARCONVRDY4 |
				BCM59040_INT8_SARCONVRDY5 |
				BCM59040_INT8_SARCONVRDY6 |
				BCM59040_INT8_SARCONVRDY7) },
	{ BCM59040_REG_INT9, (BCM59040_INT9_SARCONVRDY8 |
				BCM59040_INT9_SARCONVRDY9 |
				BCM59040_INT9_SARCONVEND |
				BCM59040_INT9_SARCONTCONVFAIL |
				BCM59040_INT9_SARASYNCONVOFF |
				BCM59040_INT9_SARASYNREQFAIL) }
};

/*
 * For speed in handling virtual interrupts this
 * table is expected to be in order of increasing
 * IRQ number! Do not break this rule or code will fail!
 */
static const virtual_int_entry_t bcm59040_vint_table[] = {
	{ BCM59040_IRQ_PONKEY,
	  ARRAY_SIZE(bcm59040_vint_table_ponkey),
	  bcm59040_vint_table_ponkey },

	{ BCM59040_IRQ_INVALID_HIB_MODE,
	  ARRAY_SIZE(bcm59040_vint_table_hibint),
	  bcm59040_vint_table_hibint },

	{ BCM59040_IRQ_PMU_TOO_WARM,
	  ARRAY_SIZE(bcm59040_vint_table_pmutoowarm),
	  bcm59040_vint_table_pmutoowarm },

	{ BCM59040_IRQ_CHARGER,
	  ARRAY_SIZE(bcm59040_vint_table_chargerstatus),
	  bcm59040_vint_table_chargerstatus },

	{ BCM59040_IRQ_REGULATORS,
	  ARRAY_SIZE(bcm59040_vint_table_regulatorstatus),
	  bcm59040_vint_table_regulatorstatus },

	{ BCM59040_IRQ_BATT,
	  ARRAY_SIZE(bcm59040_vint_table_battstatus),
	  bcm59040_vint_table_battstatus },

	{ BCM59040_IRQ_RTC,
	  ARRAY_SIZE(bcm59040_vint_table_rtc),
	  bcm59040_vint_table_rtc },

	{ BCM59040_IRQ_RTC_ALARM,
	  ARRAY_SIZE(bcm59040_vint_table_rtc_alarm),
	  bcm59040_vint_table_rtc_alarm },

	{ BCM59040_IRQ_USB,
	  ARRAY_SIZE(bcm59040_vint_table_usb),
	  bcm59040_vint_table_usb },

	{ BCM59040_IRQ_ADC,
	  ARRAY_SIZE(bcm59040_vint_table_adc),
	  bcm59040_vint_table_adc }
};

static struct pmu_client *bcm59040_client_list[BCMPMU_MAX_CLIENT_TYPES] = {
	0
};



/*
 * Function declarations.
 */
/** bcm59040_bus_lock
 * This function locks the PMU bus so the caller is guaranteed that no other process,
 * thread or interrupt will access the bus while it is running.
 *
 * This is needed for example to ensure that ktask_fn does not interfere with
 * pmu_bus_set_bit.
 */
static __must_check int bcm59040_bus_lock(bcm59040_ctxt_t *ctxt)
{
	int ret;

	// adapter not yet initialized
	if (!ctxt->adapter.dev.release)
		return 1;

	//WARN_ON(mutex_is_locked(&ctxt->mutex_i2c_rw));
	if (in_atomic() || irqs_disabled()) {
		ret = mutex_trylock (&ctxt->adapter.bus_lock);
		if (!ret) {
			printk(KERN_WARNING PFX "bus_lock: Bus activity is ongoing ... retry later: (%d|%d)\n", in_atomic(), irqs_disabled());
		}
	} else {
		mutex_lock(&ctxt->adapter.bus_lock);
		ret = 1;
	}

	return ret;
}

/** bcm59040_bus_lock_wait
 * Try to lock the bus and retry until it succeeds. Don't try calling
 * this from interrupt context or any such limited environments.
 */
static void bcm59040_bus_lock_wait(bcm59040_ctxt_t *ctxt)
{
	// can't return an error so must get the lock
	while (!bcm59040_bus_lock(ctxt)) {
		//WARN_ON(1);
		msleep(2);
	}
}

static void bcm59040_bus_unlock(bcm59040_ctxt_t *ctxt)
{
	if (ctxt->adapter.dev.release)
		mutex_unlock(&ctxt->adapter.bus_lock);
}

static int bcm59040_bus_is_locked(bcm59040_ctxt_t *ctxt)
{
	return !ctxt->adapter.dev.release || mutex_is_locked(&ctxt->adapter.bus_lock);
}



static void print_virq_state(bcm59040_ctxt_t *ctxt)
{
	if (gLevel & DBG_DATA) {
		int irq;
		printk(KERN_INFO PFX "virq state: ");
		for (irq = 0; irq < BCM59040_NUM_IRQS; irq++) {
			printk(" %d/%d/%d",
					ctxt->virq[irq].masked,
					ctxt->virq[irq].pending,
					ctxt->virq[irq].wakeup);
		}
		printk("\n");
	}
}



static inline unsigned char bcm59040_reg_addr(int addr)
{
	if (addr >= BCM59040_PAGE1_BASE_ADDR)
		return (unsigned char)(addr - BCM59040_PAGE1_BASE_ADDR);
	else
		return (unsigned char)addr;
}

static int bcm59040_read_page(struct i2c_client *client, unsigned char *page)
{
	unsigned char 		reg_pagesel	= BCM59040_REG_PAGESEL;
	struct i2c_msg		msgs[BCM59040_I2C_MSG_LEN];
	int			nb_msg=0;
	int			ret=0, retries=3;

	msgs[BCM59040_I2C_ADDR_IND].addr	= client->addr;
	msgs[BCM59040_I2C_ADDR_IND].flags	= 0;
	msgs[BCM59040_I2C_ADDR_IND].len		= sizeof(reg_pagesel);
	msgs[BCM59040_I2C_ADDR_IND].buf		= &reg_pagesel;

	msgs[BCM59040_I2C_DATA_IND].addr	= client->addr;
	msgs[BCM59040_I2C_DATA_IND].flags	= I2C_M_RD;
	msgs[BCM59040_I2C_DATA_IND].len		= sizeof(*page);
	msgs[BCM59040_I2C_DATA_IND].buf		= page;

	while(retries--) {
		if((nb_msg = i2c_transfer(client->adapter,
				msgs,
				BCM59040_I2C_MSG_LEN)) == BCM59040_I2C_MSG_LEN)
			break;
	}

	if (nb_msg != BCM59040_I2C_MSG_LEN) {
		printk(KERN_ERR PFX "Can't read the pagesel register!\n");
		ret = -EIO;
	} else {
		PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX 
				"read the pagesel register sucessfully [page = %d]\n", *page));
	}

	return ret;
}

static int bcm59040_write_page(struct i2c_client *client, unsigned char page)
{
	unsigned char 		reg_pagesel	= BCM59040_REG_PAGESEL;
	struct i2c_msg		msgs[BCM59040_I2C_MSG_LEN];
	int					nb_msg=0;
	int					ret=0, retries=3;

	msgs[BCM59040_I2C_ADDR_IND].addr	= client->addr;
	msgs[BCM59040_I2C_ADDR_IND].flags	= 0;
	msgs[BCM59040_I2C_ADDR_IND].len		= sizeof(reg_pagesel);
	msgs[BCM59040_I2C_ADDR_IND].buf		= &reg_pagesel;

	msgs[BCM59040_I2C_DATA_IND].addr	= client->addr;
	msgs[BCM59040_I2C_DATA_IND].flags	= I2C_M_NOSTART;
	msgs[BCM59040_I2C_DATA_IND].len		= sizeof(page);
	msgs[BCM59040_I2C_DATA_IND].buf		= &page;

	while(retries--) {
		if((nb_msg = i2c_transfer(client->adapter,
				msgs,
				BCM59040_I2C_MSG_LEN)) == BCM59040_I2C_MSG_LEN)
			break;
	}

	if (nb_msg != BCM59040_I2C_MSG_LEN) {
		printk(KERN_ERR PFX "Can't write the pagesel register! [page = %d]\n", page);
		ret = -EIO;
	} else {
		PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX 
				"wrote the pagesel register sucessfully [page = %d]\n", page));
	}

	return ret;
}

static int bcm59040_select_page(bcm59040_ctxt_t *ctxt, int address)
{
	struct i2c_client	*client	= NULL;
	unsigned char		page	= EPAGE_0;
	int 			ret		= 0;

	BUG_ON(!ctxt);

	client = ctxt->client;
	BUG_ON(!client);

	if (address > BCM59040_PAGE_MAX_ADDR) {
		page = EPAGE_1;
	} else {
		page = EPAGE_0;
	}

	if (ctxt->page != page) {
		/* We have to set the appropriate page. */

		unsigned char actual_page = EPAGE_0;

		if (0 == (ret = bcm59040_read_page(client, &actual_page))) {
			switch (page)
			{
			  case EPAGE_0:
				actual_page &= 0xfe;
				break;
			  case EPAGE_1:
				actual_page |= 0x01;
				break;
			  default:
				WARN(1, PFX "invalid page [page = %d]!\n", page);
				return -EINVAL;				
				break;
			}

			if (0 == (ret = bcm59040_write_page(client, actual_page))) {
				ctxt->page = page;
				udelay(100);
			}
		}
	}

	return ret;
}

static int bcm59040_read_data(bcm59040_ctxt_t *ctxt, int address, void *data, int len)
{
	struct i2c_msg		msgs[BCM59040_I2C_MSG_LEN];
	struct i2c_client	*client		= NULL;
	int					nb_msg = 0;
	unsigned char		reg_addr	= bcm59040_reg_addr(address);
	int					ret	= 0, retries = 3;

	BUG_ON(!ctxt);
	BUG_ON(!data);

	client = ctxt->client;
	BUG_ON(!client);

	msgs[BCM59040_I2C_ADDR_IND].addr	= client->addr;
	msgs[BCM59040_I2C_ADDR_IND].flags	= 0;
	msgs[BCM59040_I2C_ADDR_IND].len		= sizeof(reg_addr);
	msgs[BCM59040_I2C_ADDR_IND].buf		= &reg_addr;

	msgs[BCM59040_I2C_DATA_IND].addr	= client->addr;
	msgs[BCM59040_I2C_DATA_IND].flags	= I2C_M_RD;
	msgs[BCM59040_I2C_DATA_IND].len		= len;
	msgs[BCM59040_I2C_DATA_IND].buf		= data;

	while(retries--) {
		if((nb_msg = i2c_transfer(client->adapter,
				msgs,
				BCM59040_I2C_MSG_LEN)) == BCM59040_I2C_MSG_LEN)
			break;
	}

	if (nb_msg != BCM59040_I2C_MSG_LEN) {
		printk(KERN_ERR PFX "i2c read failure!, %d\n", nb_msg);
		ret = -EIO;
	}

	return ret;
}

static int bcm59040_write_data(bcm59040_ctxt_t *ctxt, int address, void *data, int len)
{
	struct i2c_msg		msgs[BCM59040_I2C_MSG_LEN];
	struct i2c_client	*client		= NULL;
	int					nb_msg = 0;
	unsigned char		reg_addr	= bcm59040_reg_addr(address);
	int					ret	= 0, retries = 3;

	BUG_ON(!ctxt);
	BUG_ON(!data);

	client = ctxt->client;
	BUG_ON(!client);

	msgs[BCM59040_I2C_ADDR_IND].addr	= client->addr;
	msgs[BCM59040_I2C_ADDR_IND].flags	= 0;
	msgs[BCM59040_I2C_ADDR_IND].len		= sizeof(reg_addr);
	msgs[BCM59040_I2C_ADDR_IND].buf		= &reg_addr;

	msgs[BCM59040_I2C_DATA_IND].addr	= client->addr;
	msgs[BCM59040_I2C_DATA_IND].flags	= I2C_M_NOSTART;
	msgs[BCM59040_I2C_DATA_IND].len		= len;
	msgs[BCM59040_I2C_DATA_IND].buf		= data;

	while(retries--) {
		if((nb_msg = i2c_transfer(client->adapter,
				msgs,
				BCM59040_I2C_MSG_LEN)) == BCM59040_I2C_MSG_LEN)
			break;
	}

	if (nb_msg != BCM59040_I2C_MSG_LEN) {
		printk(KERN_ERR PFX "i2c write failure! %d\n", nb_msg);
		ret = -EIO;
	}

	return ret;
}



/** bcm59040_bus_read_i2c - BCM59040 bus driver read from i2c
 *
 * This function reads data from the BCM59040 device using the I2C interface.
 *
 * Note that the caller needs to lock the mutex and validate adap and data.
 *
 * @adap:    Adapter to read data from.
 * @address: Register address within adapter I/O space.
 * @data:    Pointer to data buffer to hold read data.
 * @len:     Length of data to read in bytes.
 *
 * Returns 0 if successful otherwise standard Linux error codes
 *
 */
static int bcm59040_bus_read_i2c(bcm59040_ctxt_t *ctxt, int address, void *data, int len)
{
	int     ret;

	if (WARN(len > 10, "can't read %d registers per burst! [max = 10]\n", len)) {
		return -EINVAL;
	}
	if (WARN_ON(!bcm59040_bus_is_locked(ctxt))) {
		return -EINVAL;
	}

	if(0 == (ret = bcm59040_select_page(ctxt, address)) &&
			0 == (ret = bcm59040_read_data(ctxt, address, data, len))) {
		/* Read operation successful, return the number of byte read */
		ret = len;
	}

	return ret;
}

/** bcm59040_bus_write_i2c - BCM59040 bus driver write
 *
 * This function writes data to the BCM59040 device using the I2C interface.
 *
 * Note that the caller needs to lock the mutex and validate adap and data.
 *
 * @adap:    Adapter to write data to.
 * @address: Register address within adapter I/O space.
 * @data:    Pointer to data buffer holding write data.
 * @len:     Length of data to write in bytes.
 *
 * Returns 0 if successful otherwise standard Linux error codes
 *
 */
static int bcm59040_bus_write_i2c(bcm59040_ctxt_t *ctxt, int address, void *data, int len)
{
	int			ret;

	if (WARN(len > 10, "can't write %d registers per burst! [max = 10]\n", len)) {
		return -EINVAL;
	}
	if (WARN_ON(!bcm59040_bus_is_locked(ctxt))) {
		return -EINVAL;
	}

	if(0 == (ret = bcm59040_select_page(ctxt, address)) &&
			0 == (ret = bcm59040_write_data(ctxt, address, data, len))) {
		/* Write operation successful, return the number of byte written */
		ret = len;
	}

	return ret;
}



/** bcm59040_bus_is_virtual
 * Check whether an address is virtual. Obviously this must match with the read/write
 * below...
 */
static inline int bcm59040_bus_is_virtual_interrupt(int address)
{
	return (address >= BCM59040_REG_INT1) && (address <= BCM59040_REG_INT9);
}

static inline int bcm59040_bus_is_virtual_mask(int address)
{
	return (address >= BCM59040_REG_INT1M) && (address <= BCM59040_REG_INT9M);
}

static inline int bcm59040_bus_is_virtual(int address)
{
	return bcm59040_bus_is_virtual_interrupt(address) || bcm59040_bus_is_virtual_mask(address);
}

/** bcm59040_bus_read_virtualized - BCM59040 bus driver read
 *
 * This function reads data from the BCM59040 device for the virtualized registers.
 *
 * @adap:    Adapter to read data from.
 * @address: Register address within adapter I/O space.
 * @data:    Pointer to data buffer to hold read data.
 * @len:     Length of data to read in bytes.
 *
 * Returns 0 if successful otherwise standard Linux error codes
 *
 */
static int bcm59040_bus_read_virtualized(bcm59040_ctxt_t *ctxt, int address, void *data, int len)
{
	int			ret;

	if (WARN_ON(!bcm59040_bus_is_locked(ctxt))) {
		return -EINVAL;
	}

	if((address >= BCM59040_REG_INT1) && (address <= BCM59040_REG_INT9)) {
		int i,j;

		if(WARN_ON((address + len - 1) > BCM59040_REG_INT9))
			len = (BCM59040_REG_INT9 - address) + 1;

		ret = len;
		for(i = (address - BCM59040_REG_INT1), j = 0; j < len; i++,j++) {
			*(((u8*)data) + j) = ctxt->int_reg[i];
		}
	} else if((address >= BCM59040_REG_INT1M) && (address <= BCM59040_REG_INT9M)) {
		int i,j;
		unsigned long flags;

		if(WARN_ON((address + len - 1) > BCM59040_REG_INT9M))
			len = (BCM59040_REG_INT9M - address) + 1;

		ret = len;
		spin_lock_irqsave(&ctxt->virtual_mask_lock, flags);
		for(i = (address - BCM59040_REG_INT1M), j = 0; j < len; i++,j++) {
			*(((u8*)data) + j) = ctxt->mask_reg[i];
		}
		spin_unlock_irqrestore(&ctxt->virtual_mask_lock, flags);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

/** bcm59040_bus_write_virtualized - BCM59040 bus driver write
 *
 * This function writes data to the BCM59040 device for the virtualized registers.
 *
 * @adap:    Adapter to write data to.
 * @address: Register address within adapter I/O space.
 * @data:    Pointer to data buffer holding write data.
 * @len:     Length of data to write in bytes.
 *
 * Returns 0 if successful otherwise standard Linux error codes
 *
 */
static int bcm59040_bus_write_virtualized(bcm59040_ctxt_t *ctxt, int address, void *data, int len)
{
	int     		ret;

	if (WARN_ON(!bcm59040_bus_is_locked(ctxt))) {
		return -EINVAL;
	}

	if((address >= BCM59040_REG_INT1) && (address <= BCM59040_REG_INT9)) {
		int i,j;

		if(WARN_ON((address + len - 1) > BCM59040_REG_INT9) )
			len = (BCM59040_REG_INT9 - address) + 1;

		ret = len;

		// clear the interrupt bits that are being written
		PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Clearing interrupts from reg %d: ", (address - BCM59040_REG_INT1)));
		for(i = (address - BCM59040_REG_INT1), j = 0; j < len; i++,j++) {
			PMU_DEBUG(DBG_TRACE, (" %x-%x", ctxt->int_reg[i], *(((u8*)data) + j)));
			ctxt->int_reg[i] &= ~*(((u8*)data) + j);
		}
		PMU_DEBUG(DBG_TRACE, ("\n"));
	} else if((address >= BCM59040_REG_INT1M) && (address <= BCM59040_REG_INT9M)) {
		int i,j;
		unsigned long flags;

		if(WARN_ON((address + len - 1) > BCM59040_REG_INT9M))
			len = (BCM59040_REG_INT9M - address) + 1;

		ret = len;

		spin_lock_irqsave(&ctxt->virtual_mask_lock, flags);
		for(i = (address - BCM59040_REG_INT1M), j = 0; j < len; i++,j++) {
			ctxt->mask_reg[i] = *(((u8*)data) + j);
			ctxt->int_reg[i] &= ~ctxt->mask_reg[i];
		}
		spin_unlock_irqrestore(&ctxt->virtual_mask_lock, flags);

		schedule_work(&ctxt->update_mask_work);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

/** bcm59040_virtual_mask_update
 * Check for and write updated mask values to the device.
 */
static void bcm59040_virtual_mask_update(bcm59040_ctxt_t *ctxt)
{
	unsigned long flags;
	int i;
	unsigned char new_masks[BCM59040_NUM_INT_REG];
	int updated;

	/* if the masks got updated while we were running, just do it again */
	do {
		spin_lock_irqsave(&ctxt->virtual_mask_lock, flags);
		/* take a copy so we can release the lock again */
		/* because we need interrupts down below when writing to i2c */
		for (i = 0; i < BCM59040_NUM_INT_REG; i++) {
			new_masks[i] = ctxt->mask_reg[i];
		}
		spin_unlock_irqrestore(&ctxt->virtual_mask_lock, flags);

		updated = 0;
		for (i = 0; i < BCM59040_NUM_INT_REG; i++) {
			if (new_masks[i] !=  ctxt->device_mask_reg[i]) {
				int ret;
				int address = BCM59040_REG_INT1M + i;

				if (!updated)
					PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "Updating BCM59040 mask registers:"));

				PMU_DEBUG(DBG_INFO, (KERN_CONT " %d:%x->%x", i, ctxt->device_mask_reg[i], new_masks[i]));

				ctxt->device_mask_reg[i] = new_masks[i];
				ret = bcm59040_bus_write_i2c(ctxt, address, &new_masks[i], 1);
				if (ret < 0) {
					// not sure how to recover...
					ret = bcm59040_bus_write_i2c(ctxt, address, &new_masks[i], 1);
					WARN_ON(ret < 0);
				}

				updated = 1;
			}
		}
		if (updated)
			PMU_DEBUG(DBG_INFO, (KERN_CONT "\n"));

	} while (updated);
}

static void bcm59040_update_mask_work(struct work_struct *work)
{
	bcm59040_ctxt_t *ctxt = container_of(work, bcm59040_ctxt_t, update_mask_work);

	bcm59040_bus_lock_wait(ctxt);
	bcm59040_virtual_mask_update(ctxt);
	bcm59040_bus_unlock(ctxt);
}



/** bcm59040_bus_is_virq
 * Check whether an address is a virq related address.
 */
static inline int bcm59040_bus_is_virq(int address)
{
	return (address == BCM59040_REG_VINT1 || address == BCM59040_REG_VINT2 ||
			address == BCM59040_REG_VINT1M || address == BCM59040_REG_VINT2M);
}

/** bcm59040_bus_read_virq - BCM59040 bus driver read
 *
 * This function reads data from the BCM59040 device for the virtualized interrupt registers.
 *
 * @adap:    Adapter to read data from.
 * @address: Register address within adapter I/O space.
 * @data:    Pointer to data buffer to hold read data.
 * @len:     Length of data to read in bytes.
 *
 * Returns 0 if successful otherwise standard Linux error codes
 *
 */
static int bcm59040_bus_read_virq(bcm59040_ctxt_t *ctxt, int address, void *data, int len)
{
	int	ret;
	u8	*reg = data;
	int	i, mask;

	if (WARN_ON(!bcm59040_bus_is_locked(ctxt))) {
		return -EINVAL;
	}
	if (WARN_ON(len != 1)) {
		return -EINVAL;
	}

	*reg = 0;
	ret = len;
	if(address == BCM59040_REG_VINT1) {
		for(i = 0, mask = 1; i < 8; i++, mask <<= 1) {
			if (ctxt->virq[i].pending)
				*reg |= mask;
		}
	} else if(address == BCM59040_REG_VINT2) {
		for(i = 8, mask = 1; i < BCM59040_NUM_IRQS; i++, mask <<= 1) {
			if (ctxt->virq[i].pending)
				*reg |= mask;
		}
	} else if (address == BCM59040_REG_VINT1M) {
		for(i = 0, mask = 1; i < 8; i++, mask <<= 1) {
			if (ctxt->virq[i].masked)
				*reg |= mask;
		}
	} else if (address == BCM59040_REG_VINT2M) {
		for(i = 8, mask = 1; i < BCM59040_NUM_IRQS; i++, mask <<= 1) {
			if (ctxt->virq[i].masked)
				*reg |= mask;
		}
	} else {
		ret = -EINVAL;
	}

	PMU_DEBUG(DBG_VIRQ_DATA, (KERN_INFO PFX "%s: %x is %x\n", __FUNCTION__, address, *reg));
	return ret;
}

/** bcm59040_bus_write_virq - BCM59040 bus driver write
 *
 * This function writes data to the BCM59040 device for the virtualized interrupt registers.
 * The interrupt register is a clear-on-write-1. So each bit that is set causes the pending
 * flag of the related interrupt to be cleared.
 *
 * @adap:    Adapter to write data to.
 * @address: Register address within adapter I/O space.
 * @data:    Pointer to data buffer holding write data.
 * @len:     Length of data to write in bytes.
 *
 * Returns 0 if successful otherwise standard Linux error codes
 *
 */
static int bcm59040_bus_write_virq(bcm59040_ctxt_t *ctxt, int address, void *data, int len)
{
	int	ret;
	u8	*reg = data;
	int	i, j, mask;
	int	start, end;

	if (WARN_ON(!bcm59040_bus_is_locked(ctxt))) {
		return -EINVAL;
	}
	if (WARN_ON(len != 1)) {
		return -EINVAL;
	}

	/* if it gets larger then another register is needed to store this */
	BUILD_BUG_ON(BCM59040_NUM_IRQS >= 16);

	PMU_DEBUG(DBG_VIRQ_DATA, (KERN_INFO PFX "%s: %x is %x\n", __FUNCTION__, address, *reg));

	ret = 1;
	if(address == BCM59040_REG_VINT1 || address == BCM59040_REG_VINT2) {
		start = (address == BCM59040_REG_VINT1) ? 0 : 8;
		end = (address == BCM59040_REG_VINT1) ? 8 : BCM59040_NUM_IRQS;

		/* implement clear-on-write-1 */
		for(i = start, mask = 1; i < end; i++, mask <<= 1) {
			if ((*reg & mask) && ctxt->virq[i].pending) {
				ctxt->virq[i].pending = 0;

				/* also clear the virtualized device registers */
				for (j = 0; j < bcm59040_vint_table[i].num_registers; j++) {
					bcm59040_bus_write_virtualized(ctxt,
							bcm59040_vint_table[i].reg_list[j].int_reg_addr,
							(void *)&bcm59040_vint_table[i].reg_list[j].reg_mask,
							1);
				}
			}
		}
		print_virq_state(ctxt);
	} else if (address == BCM59040_REG_VINT1M || address == BCM59040_REG_VINT2M) {
		start = (address == BCM59040_REG_VINT1M) ? 0 : 8;
		end = (address == BCM59040_REG_VINT1M) ? 8 : BCM59040_NUM_IRQS;

		/* need to change all the masks */
		for(i = start, mask = 1; i < end; i++, mask <<= 1) {
			if (*reg & mask) {
				if (!ctxt->virq[i].masked)
					bcm59040_virq_set_mask(IRQ_BCM59XXX_FIRST + i, EIRQ_MASK);
			} else {
				if (ctxt->virq[i].masked)
					bcm59040_virq_set_mask(IRQ_BCM59XXX_FIRST + i, EIRQ_UNMASK);
			}
		}
	} else {
		ret = -EINVAL;
	}

	return ret;
}



/** bcm59040_bus_read - BCM59040 bus driver read
 *
 * This function reads data from the BCM59040 device.
 * It is called via the bcm59040_bus_ops read callback. The
 * caller is the pmu device framework. Since BCM59040 has byte sized
 * registers any alignment and transfer size granularity is valid. We
 * never report an error on alignment. We should report an attempt to
 * read an invalid register.
 *
 * @adap:    Adapter to read data from.
 * @address: Register address within adapter I/O space.
 * @data:    Pointer to data buffer to hold read data.
 * @len:     Length of data to read in bytes.
 *
 * Returns 0 if successful otherwise standard Linux error codes
 *
 */

static int bcm59040_bus_read(struct pmu_adapter *adap, int address, void *data, int len)
{
	struct i2c_client	*client;
	bcm59040_ctxt_t		*ctxt;
	int			ret;

	BUG_ON(!adap);
	BUG_ON(!data);

	client = container_of(adap->dev.parent, struct i2c_client, dev);
	BUG_ON(!client);

	ctxt = i2c_get_clientdata(client);
	BUG_ON(!ctxt);

	PMU_DEBUG(DBG_TRACE2, (KERN_INFO PFX "Reading from 0x%x.\n", address));
	if (bcm59040_bus_is_virtual_interrupt(address)) {
		ret = bcm59040_bus_read_virtualized(ctxt, address, data, len);
	} else if (bcm59040_bus_is_virtual_mask(address)) {
		/* clients should NOT be reading/writing these masks */
		WARN_ON(1);
		ret = -EINVAL;
	} else if (bcm59040_bus_is_virq(address)) {
		ret = bcm59040_bus_read_virq(ctxt, address, data, len);
	} else {
		ret = bcm59040_bus_read_i2c(ctxt, address, data, len);
	}

	PMU_DEBUG(DBG_TRACE2, (KERN_INFO PFX "Read from 0x%x returns %d.\n", address, ret));
	return ret;
}

/** bcm59040_bus_write - BCM59040 bus driver write
 *
 * This function writes data to the BCM59040 device.
 * It is called via the bcm59040_bus_ops write callback. The
 * caller is the pmu device framework. Since BCM59040 has byte sized
 * registers any alignment and transfer size granularity is valid. We
 * never report an error on alignment. We should report an attempt to
 * write an invalid register.
 *
 * @adap:    Adapter to write data to.
 * @address: Register address within adapter I/O space.
 * @data:    Pointer to data buffer holding write data.
 * @len:     Length of data to write in bytes.
 *
 * Returns 0 if successful otherwise standard Linux error codes
 *
 */
static int bcm59040_bus_write(struct pmu_adapter *adap, int address, void *data, int len)
{
	struct i2c_client	*client;
	bcm59040_ctxt_t		*ctxt;
	int			ret;

	BUG_ON(!adap);
	BUG_ON(!data);

	client = container_of(adap->dev.parent, struct i2c_client, dev);
	BUG_ON(!client);

	ctxt = i2c_get_clientdata(client);
	BUG_ON(!ctxt);

	PMU_DEBUG(DBG_TRACE2, (KERN_INFO PFX "Write to 0x%x.\n", address));
	if (bcm59040_bus_is_virtual(address)) {
		/* clients should NOT be reading/writing these interrupt registers */
		WARN_ON(1);
		ret = -EINVAL;
	} else if (bcm59040_bus_is_virq(address)) {
		ret = bcm59040_bus_write_virq(ctxt, address, data, len);
	} else {
		ret = bcm59040_bus_write_i2c(ctxt, address, data, len);
	}

	PMU_DEBUG(DBG_TRACE2, (KERN_INFO PFX "Write to 0x%x returns %d.\n", address, ret));
	return ret;
}

static const struct pmu_algorithm bcm59040_algorithm = {
	.read	= bcm59040_bus_read,
	.write	= bcm59040_bus_write,
};



static void print_device_masks(bcm59040_ctxt_t *ctxt, const char *desc)
{
	if (DBG_DATA & gLevel) {
		int		i, ret;
		unsigned char	int_reg[BCM59040_NUM_INT_REG];
		unsigned char	mask_reg[BCM59040_NUM_INT_REG];

		ret = bcm59040_bus_read_i2c(ctxt, BCM59040_REG_INT1, int_reg, sizeof(int_reg));
		if(ret != sizeof(int_reg)) {
			printk(KERN_ERR PFX "Failed to read BCM59040 interrupt registers.\n");
			return;
		}

		ret = bcm59040_bus_read_i2c(ctxt, BCM59040_REG_INT1M, mask_reg, sizeof(mask_reg));
		if(ret != sizeof(mask_reg)) {
			printk(KERN_ERR PFX "Failed to read BCM59040 mask registers.\n");
			return;
		}

		printk(KERN_INFO PFX "%s: ", desc);
		for (i = 0; i < BCM59040_NUM_INT_REG; i++) {
			printk(" %x/%x", (int)int_reg[i], (int)mask_reg[i]);
		}
		printk("\n");
	}
}



static void bcm59040_virq_set_mask(unsigned int irq, e_irq_mask mask)
{
	bcm59040_ctxt_t				*ctxt = get_irq_chip_data(irq);
	int					i, nb_byte;
	unsigned char				reg_int, reg_msk, reg_val;

	BUG_ON(!ctxt);

	WARN_ON(!bcm59040_bus_is_locked(ctxt));
	WARN_ON_ONCE(irq < IRQ_BCM59XXX_FIRST);

	PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "Setting mask of %d to %d.\n", irq, mask));
	irq -= IRQ_BCM59XXX_FIRST;
	if(mask == EIRQ_MASK)
		ctxt->virq[irq].masked = 1;
	else
		ctxt->virq[irq].masked = 0;

	for(i = 0; i < bcm59040_vint_table[irq].num_registers ; i++) {
		reg_int = bcm59040_vint_table[irq].reg_list[i].int_reg_addr - BCM59040_REG_INT1 + BCM59040_REG_INT1M;
		if((nb_byte = bcm59040_bus_read_virtualized(ctxt, reg_int, &reg_val, sizeof(reg_val))) < 0) {
			printk(KERN_ERR PFX "error reading mask register [irq=%d/%d] !\n", irq, i);
			return;
		}

		reg_msk = bcm59040_vint_table[irq].reg_list[i].reg_mask;
		if(mask == EIRQ_MASK) {
			reg_val |= reg_msk;	/* mask all related bits for this IRQ */
		} else {
			reg_val &= ~reg_msk;
		}

		if((nb_byte = bcm59040_bus_write_virtualized(ctxt, reg_int, &reg_val, sizeof(reg_val))) < 0) {
			printk(KERN_ERR PFX "mask setting failure [irq=%d/%d] !\n", irq, i);
			return;
		}
	}

	irq += IRQ_BCM59XXX_FIRST;
	bcm59040_virq_update_pending_irq(ctxt, irq);
}

/** bcm59040_irq_ack - IRQChip function to acknowledge an interrupt
 *
 * For BCM59040 these are actually "virtual" interrupts we're being
 * requested to disable. Of course virtual interrupts need to be
 * translated into the appropriate physical interrupts here so we
 * can acknowledge the proper the hardware interrupts.
 *
 * @irq:    number of BCM59040 virtual interrupt to ack
 *
 * @note Since several BCM59040 interrupts may be hidden beneath
 *       a single virtual interrupt we may actually acknowledge several physical
 *       interrupts when we acknowledge the specified virtual interrupts.
 *
 * @todo	Needs to be implemented.
 *
 */
static void bcm59040_virq_ack(unsigned irq)
{
	// Since this function is ONLY called with interrupts disabled, we don't
	// need to disable irqs around the following
	return;
}

/** bcm59040_irq_mask - IRQChip function to mask, i.e., disable an interrupt
 *
 * For BCM59040 these are actually "virtual" interrupts we're being
 * requested to disable. Of course virtual interrupts need to be
 * translated into the appropriate physical interrupts here so we
 * can configure the hardware appropriately.
 *
 * @irq:	Number of BCM59040 virtual interrupt to mask.
 *
 * @note Since several BCM59040 interrupts may be hidden beneath
 *       a single virtual interrupt we may actually disable several physical
 *       interrupts when we turn off the specified virtual interrupts.
 *
 */
static void bcm59040_virq_mask(unsigned irq)
{
	bcm59040_ctxt_t *ctxt = get_irq_chip_data(irq);

	// can't return an error so must get the lock
	bcm59040_bus_lock_wait(ctxt);

	bcm59040_virq_set_mask(irq, EIRQ_MASK);

	bcm59040_bus_unlock(ctxt);
}

/** bcm59040_irq_unmask - IRQChip function to unmask, i.e., enable an interrupt
 *
 * For BCM59040 these are actually "virtual" interrupts we're being
 * requested to enable. Of course virtual interrupts need to be
 * translated into the appropriate physical interrupts here so we
 * can configure the hardware appropriately.
 *
 * @irq:		Number of BCM59040 virtual interrupt to unmask.
 *
 * @note Since several BCM59040 interrupts may be hidden beneath
 *       a single virtual interrupt we may actually enable several physical
 *       interrupts when we turn off the specified virtual interrupts.
 *
 */
static void bcm59040_virq_unmask(unsigned irq)
{
	bcm59040_ctxt_t *ctxt = get_irq_chip_data(irq);

	// can't return an error so must get the lock
	bcm59040_bus_lock_wait(ctxt);

	bcm59040_virq_set_mask(irq, EIRQ_UNMASK);

	bcm59040_bus_unlock(ctxt);
}

/** bcm59040_irq_set_wake - IRQChip function to set a BCM59040 interrupt as a wakeup event
 *
 * For BCM59040 these are actually "virtual" interrupts we're being
 * requested to disable. Of course virtual interrupts need to be
 * translated into the appropriate physical interrupts here so we
 * can configure the hardware appropriately. Since we are configuring
 * what interrupts are wakeup events this means we need to mask all
 * interrupts except for these when we enter suspend. Conversely we
 * have to remember what the mask/unmask status was when we entered
 * suspend and restore it on resume.
 *
 * @irq:		Number of BCM59040 virtual interrupt to set as wakeup event.
 * @on			0 to disable irq as wakeup event, 1 to enable as wakeup event.
 *
 * Return 0 if no error.
 *
 * @note    Since several BCM59040 interrupts may be hidden beneath
 *          a single virtual interrupt we may actually choose several physical
 *          interrupts as wakeup events as a response for configuring the
 *          specified virtual interrupt as a wakeup event.
 *
 * @note    The way this ought to work is that others should enable or disable a
 *          BCM59040 IRQ event as a wakeup event. This function should track the
 *          number of interrupts selected as wakeup events. When the count total
 *          becomes zero (and initially) the primary wakeup event into the SoC
 *          should be disabled also. When the count transitions from zero to one
 *          or greater the primary wakeup event into the SoC should be enabled. In
 *          other words hide the need to manage the wakeup event into the SoC.
 *
 * @todo	Needs to be implemented.
 *
 */
static int bcm59040_virq_set_wake(unsigned irq, unsigned on)
{
	bcm59040_ctxt_t				*ctxt = get_irq_chip_data(irq);

	BUG_ON(!ctxt);

	PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "Setting wakeup of %d to %d.\n", irq, on));
	irq -= IRQ_BCM59XXX_FIRST;
	ctxt->virq[irq].wakeup = (on ? 1 : 0);
	return 0;
} // bcm59040_irq_set_wake

static int bcm59040_virq_need_wakeup(bcm59040_ctxt_t *ctxt)
{
	int irq;
	for (irq = 0; irq < BCM59040_NUM_IRQS; irq++) {
		if (ctxt->virq[irq].wakeup)
			return 1;
	}
	return 0;
}

static struct irq_chip bcm59040_irq_chip =
{
	.name		= BCM59040_DEVNAME "_virq",
	// to disable an IRQ is the same as masking it
	.disable	= bcm59040_virq_mask,
	.ack		= bcm59040_virq_ack,
	.mask		= bcm59040_virq_mask,
	.unmask		= bcm59040_virq_unmask,
	.set_wake	= bcm59040_virq_set_wake,
};

static irqreturn_t pmu_bus_handle_irq(int irq, void *data)
{
	bcm59040_ctxt_t *ctxt = (bcm59040_ctxt_t *)data;

	disable_irq(irq);
	up(&ctxt->sem_isr);

	return IRQ_HANDLED;
}

static int bcm59040_pmu_irqs_apply_masks(bcm59040_ctxt_t * ctxt)
{
	int nb_byte;
	int i;

	BUG_ON(!ctxt);

	nb_byte = bcm59040_bus_write_i2c(ctxt, BCM59040_REG_INT1M, ctxt->mask_reg, BCM59040_NUM_INT_REG);
	if (nb_byte != BCM59040_NUM_INT_REG) {
		printk(KERN_ERR PFX "virtual mask registers initialization failure!\n");
		return -EIO;
	}
	for (i = 0; i < BCM59040_NUM_INT_REG; i++) {
		ctxt->device_mask_reg[i] = ctxt->mask_reg[i];
	}
	
	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "virtual masks applied\n"));

	return 0;
}



static int bcm59040_virq_calculate_pending(bcm59040_ctxt_t *ctxt, int irq)
{
	int j;
	int irq_idx = irq - IRQ_BCM59XXX_FIRST;
	unsigned long flags;

	WARN_ON(!bcm59040_bus_is_locked(ctxt));
	WARN_ON_ONCE(irq < IRQ_BCM59XXX_FIRST);

	spin_lock_irqsave(&ctxt->virtual_mask_lock, flags);
	for(j = 0; j < bcm59040_vint_table[irq_idx].num_registers; j++) {

		uint8_t	ints, mask;

		int reg_offset = bcm59040_vint_table[irq_idx].reg_list[j].int_reg_addr - BCM59040_REG_INT1;
		int reg_mask = bcm59040_vint_table[irq_idx].reg_list[j].reg_mask;

		ints = ctxt->int_reg[reg_offset];
		mask = ctxt->mask_reg[reg_offset];

		/* see if any of the bits in the interrupt register add to this interrupt */
		PMU_DEBUG(DBG_VIRQ_STATE, (KERN_INFO PFX "pending: irq %d(%d), reg %d, 0x%x 0x%x 0x%x %d %d\n", irq_idx, irq, reg_offset, ints, mask, reg_mask, ctxt->virq[irq_idx].masked, ctxt->virq[irq_idx].pending));
		if((ints & ~mask) & reg_mask && !ctxt->virq[irq_idx].masked) {
			spin_unlock_irqrestore(&ctxt->virtual_mask_lock, flags);
			PMU_DEBUG(DBG_TRACE2, (KERN_INFO PFX "pending: irq %d is pending\n", irq_idx));
			return 1;
		}
	}
	spin_unlock_irqrestore(&ctxt->virtual_mask_lock, flags);

	PMU_DEBUG(DBG_TRACE2, (KERN_INFO PFX "pending: irq %d is not pending\n", irq_idx));
	return 0;
}

/** bcm59040_virq_update_pending_mask
 * Update the pending flags of the virtual interrupts after a mask or unmask of
 * the virtual interrupt.
 */
static void bcm59040_virq_update_pending_irq(bcm59040_ctxt_t *ctxt, int irq)
{
	int old_pending, new_pending;
	int irq_idx = irq - IRQ_BCM59XXX_FIRST;

	WARN_ON_ONCE(irq < IRQ_BCM59XXX_FIRST);

	old_pending = ctxt->virq[irq_idx].pending;
	new_pending = bcm59040_virq_calculate_pending(ctxt, irq);

	// strange if pending is now set
	//WARN_ON(new_pending && !old_pending);

	// only CLEAR pending here; raising it is the responsibility of the kthread
	if (!new_pending)
		ctxt->virq[irq_idx].pending = new_pending;
}

/** bcm59040_virq_handle - Process virtual BCM59040 PMU interrupts
 *
 * This function will scan the virtual (saved copy) of the interrupt
 * registers for the BCM59040 PMU and generate the different virtual
 * interrupts appropriate depending upon the interrupt status bits
 * that are set.
 *
 */
void bcm59040_virq_calculate_new_irqs(bcm59040_ctxt_t *ctxt, u8 new_irqs[])
{
	int		i;

	if (WARN_ON(!bcm59040_bus_is_locked(ctxt)))
		return;

	// go over the interrupts and check which ones match the newly raised bits
	for(i = 0; i < ARRAY_SIZE(bcm59040_vint_table); i++) {
		int new_pending;

		new_irqs[i] = 0;

		new_pending = bcm59040_virq_calculate_pending(ctxt, i + IRQ_BCM59XXX_FIRST);
		if (new_pending != ctxt->virq[i].pending) {
			new_irqs[i] = new_pending;
			ctxt->virq[i].pending = new_pending;
		}
	}
}

static int ktask_fn(void *data)
{
	bcm59040_ctxt_t				*ctxt;
	struct i2c_client			*client;

	BUG_ON(!data);
	ctxt = data;

	client = ctxt->client;
	BUG_ON(!client);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "kernel process spawned\n"));
	while(! kthread_should_stop()) {

		if (down_interruptible(&ctxt->sem_isr)) {
			printk(PFX "ktask received a signal\n");
			return -ERESTART;
		}

		do {
			int i;
			unsigned char new_interrupts[BCM59040_NUM_INT_REG];
			unsigned char new_irqs[BCM59040_NUM_IRQS];

			for( ; ; ) {
				int ret;

				// it's a thread so !in_atomic && !irqs_disabled
				WARN_ON_ONCE(!bcm59040_bus_lock(ctxt));

				// first update the masks in the device
				bcm59040_virtual_mask_update(ctxt);

				// then read the interrupts
				ret = bcm59040_bus_read_i2c(ctxt, BCM59040_REG_INT1, new_interrupts, sizeof(new_interrupts));
				if (ret == sizeof(new_interrupts))
					break;

				printk(KERN_INFO PFX "can't get interrupt status... trying again!: %d\n", ret);

				bcm59040_bus_unlock(ctxt);
			}
			/* NOTE!!! the bus is locked here so all changes are atomic! */
			/* can't have clients changing the mask from under us. */

			PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "Processing BCM59040 virtual interrupts:"));
			for(i = 0; i < sizeof(ctxt->int_reg); i++) {
				PMU_DEBUG(DBG_INFO, (KERN_CONT " %x+%x", ctxt->int_reg[i], new_interrupts[i]));

				/* since new_interrupts only contains the newly set bits they need to be added into the existing set of bits */
				ctxt->int_reg[i] = ctxt->int_reg[i] | new_interrupts[i];
			}
			PMU_DEBUG(DBG_TRACE, (KERN_CONT "\n"));

			bcm59040_virq_calculate_new_irqs(ctxt, new_irqs);

			PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX ""));
			PMU_DEBUG(DBG_INFO, (KERN_CONT " =>"));
			for(i = 0; i < ARRAY_SIZE(bcm59040_vint_table); i++) {
				if (new_irqs[i]) {
					int int_vector = bcm59040_vint_table[i].int_vector;
					PMU_DEBUG(DBG_INFO, (KERN_CONT " %d", int_vector));
				}
			}
			PMU_DEBUG(DBG_INFO, (KERN_CONT "\n"));

			/* must call the handlers outside the lock because they likely need to read the registers */
			bcm59040_bus_unlock(ctxt);

			PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Dispatching BCM59040 virtual interrupts:\n"));

			for(i = 0; i < ARRAY_SIZE(bcm59040_vint_table); i++) {
				if (new_irqs[i]) {
					int int_vector = bcm59040_vint_table[i].int_vector;

					PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Dispatching virtual interrupt %d\n", int_vector));

					/* disable interrupts; that is normal while handling interrupts */
					local_irq_disable();
					generic_handle_irq(int_vector);
					local_irq_enable();
				}
			}

			PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Processing BCM59040 virtual interrupts done.\n"));

		/* CLEANUP: gpio_get_value() should not be called here. temporary. */
		} while ((! gpio_get_value(1)) && (! kthread_should_stop()));

		enable_irq(client->irq);
	}

	return 0;
}

static bcm59040_ctxt_t * bcm59040_ctxt_setup(struct i2c_client *client)
{
	bcm59040_ctxt_t *ctxt = NULL;
	int i;

	BUG_ON(!client);
	BUILD_BUG_ON(sizeof(ctxt->int_reg) != BCM59040_NUM_INT_REG);
	BUILD_BUG_ON(ARRAY_SIZE(bcm59040_vint_table) != BCM59040_NUM_IRQS);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Context initialization\n"));

	ctxt = (bcm59040_ctxt_t *) kzalloc(sizeof(bcm59040_ctxt_t), GFP_KERNEL);
	if(NULL == ctxt) {
		printk(KERN_ERR PFX "Can't allocate work!\n");
		return NULL;
	}

	sema_init(&ctxt->sem_isr, 0);
	spin_lock_init(&ctxt->virtual_mask_lock);
	INIT_WORK(&ctxt->update_mask_work, bcm59040_update_mask_work);

	ctxt->client = client;
	i2c_set_clientdata(client, ctxt);

	/* virtualized register mask initialization: Initially, all irqs are masked. */
	for(i = 0 ; i < BCM59040_NUM_INT_REG ; i++) {
		ctxt->mask_reg[i] = 0xFF;
	}
	/* virtualized interrupt mask initialization: Initially, all irqs are masked. */
	for(i = 0 ; i < BCM59040_NUM_IRQS ; i++) {
		ctxt->virq[i].masked = 1;
	}

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Kernel thread creation\n"));

	ctxt->ktask = kthread_create(ktask_fn, ctxt, client->name);
	if (IS_ERR(ctxt->ktask)) {
		int ret = PTR_ERR(ctxt->ktask);
		printk(KERN_ERR PFX "Kernel thread creation failure! err=%d\n", ret);
		kfree(ctxt);
		return NULL;
	}

	/* Preparing for Bus Backend Registration. */
	ctxt->adapter.owner	= THIS_MODULE;
	ctxt->adapter.algo	= &bcm59040_algorithm;

	/* set up the sysfs linkage to our parent device */
	ctxt->adapter.dev.parent = &client->dev;
	strncpy(ctxt->adapter.name, client->name, sizeof(ctxt->adapter.name));

	return ctxt;
}

static void bcm59040_ctxt_cleanup(bcm59040_ctxt_t * ctxt)
{
	kthread_stop(ctxt->ktask);
	
	kfree(ctxt);
}

static int bcm59040_virq_setup(bcm59040_ctxt_t * ctxt)
{
	int					i;

	BUG_ON(!ctxt);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "setup virqs\n"));

	for (i = 0; i < BCM59040_NUM_IRQS ; i++) {
		unsigned int irq = IRQ_BCM59XXX_FIRST + i;

		dynamic_irq_init(irq);

		set_irq_chip_and_handler(irq, &bcm59040_irq_chip, handle_simple_irq);
		set_irq_chip_data(irq, ctxt);

		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	return 0;
}

static int bcm59040_pmu_irqs_cleanup(bcm59040_ctxt_t * ctxt)
{
	int	i;

	BUG_ON(!ctxt);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "cleanup virqs\n"));

	for (i = 0; i < BCM59040_NUM_IRQS ; i++) {
		dynamic_irq_cleanup(IRQ_BCM59XXX_FIRST + i);
	}

	return 0;
}

/** bcm_get_pclient_from_client_type - Obtain PMU_CLIENT from a supplied type.
 *
 * This function helps isolate different PMU client drivers from each other. Rather
 * than hardcoding variables and such each client can call this function to obtain
 * the pmu_client value of one of it's sibling devices/drivers.
 *
 * @pclient:		struct pmu_client of caller (for possible verification)
 * @client_type:	type ID of pmu_client to look up.
 *
 * Returns pclient pointer or NULL if no valid matching entry found.
 *
 */
struct pmu_client *bcm_get_pclient_from_client_type(struct pmu_client *pclient,
													bcmpmu_clients_t client_type)
{
	int		i;

	if(!pclient || (client_type > BCMPMU_MAX_CLIENT_TYPES))
		return NULL;

	for(i=0; i<BCMPMU_MAX_CLIENT_TYPES; i++) {
		if(pclient == bcm59040_client_list[i])
			break;
	}

	if(i == BCMPMU_MAX_CLIENT_TYPES) {
		return NULL;
	} else {
		PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "type=%d, pclient=%08X\n",
						client_type,
						(uint32_t)bcm59040_client_list[client_type]));
		return bcm59040_client_list[client_type];
	}
}
EXPORT_SYMBOL_GPL(bcm_get_pclient_from_client_type);

/** bcm59040_i2c_probe - probe for BCM59040 I2C client
 *
 * This function checks for existence of a BCM59040 PMU and initializes
 * the I2C connection and driver routines if one is found. It also adds
 * the PMU client devices to the PMU framework so the PMU drivers can
 * register and load against them.
 */
static int bcm59040_i2c_probe(struct i2c_client *client, const struct i2c_device_id * id)
{
	bcm59040_ctxt_t *ctxt = NULL;
	struct pmu_bcm59040_platform_data *pdata =
		(struct pmu_bcm59040_platform_data*)client->dev.platform_data;
	int ret = 0;

	BUG_ON(!client);

	ctxt = bcm59040_ctxt_setup(client);

	{
		u8	pmu_id;

		ret = bcm59040_bus_read(&ctxt->adapter,
				BCM59040_PAGE1_BASE_ADDR + BCM59040_REG_PMUID,
				&pmu_id, sizeof(pmu_id));
		if(ret != sizeof(pmu_id)) {
			printk("Failed to read BCM59040 chip ID!\n");
			bcm59040_ctxt_cleanup(ctxt);
			return -EINVAL;
		}
		PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "BCM59040 chip ID: %02X\n", (unsigned)pmu_id));

		if ((pmu_id != PMU_59040_B0_REV) &&
			(pmu_id != PMU_59040_B1_REV)) {
			printk(KERN_ERR PFX "PMU Chip Version and software does not match\n");
			bcm59040_ctxt_cleanup(ctxt);
			return -EINVAL;
		}
	}

	// this is impossible to fail
	BUG_ON(!bcm59040_bus_lock(ctxt));

	print_device_masks(ctxt, "Initial interrupt status registers (before mask init)");

	/* no need to protect against interrupts here */
	if (0 != (ret = bcm59040_pmu_irqs_apply_masks(ctxt))) {
		bcm59040_bus_unlock(ctxt);
		bcm59040_ctxt_cleanup(ctxt);
		return ret;
	}

	print_device_masks(ctxt, "Initial interrupt status registers (after mask init)");

	{
		uint8_t		otgctrl[2];

		/* Default set the OTG controller part off. If we need it, it is enabled by it's driver. */
		ret = bcm59040_bus_read_i2c(ctxt, BCM59040_REG_OTGCTRL1, otgctrl, sizeof( otgctrl ) );
		if( ret != sizeof( otgctrl ) )
		{
			printk( KERN_WARNING PFX "Unable to read OTGCTRL registers!\n" );
			bcm59040_bus_unlock(ctxt);
			bcm59040_ctxt_cleanup(ctxt);
			return -EINVAL;
		}

		otgctrl[0]&=0x7F;
		otgctrl[1]&=0xBF;

		ret = bcm59040_bus_write_i2c(ctxt, BCM59040_REG_OTGCTRL1, otgctrl, sizeof( otgctrl ) );
		if( ret != sizeof( otgctrl ) )
		{
			printk( KERN_WARNING PFX "Unable to initialize OTGCTRL registers!\n" );
			bcm59040_bus_unlock(ctxt);
			bcm59040_ctxt_cleanup(ctxt);
			return -EINVAL;
		}
	}

	bcm59040_virq_setup(ctxt);

	/* all initialization now done */
	bcm59040_bus_unlock(ctxt);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "wake up process\n"));
	wake_up_process(ctxt->ktask);

	{
		if(0 != request_irq(client->irq, pmu_bus_handle_irq, IRQF_DISABLED | IRQF_TRIGGER_LOW, 
						client->name, ctxt)) {
			printk(KERN_ERR PFX "Can't request irq %d\n", client->irq);
			bcm59040_pmu_irqs_cleanup(ctxt);
			bcm59040_ctxt_cleanup(ctxt);
			return -ENXIO;
		}
	}

	ret = pmu_add_adapter(&ctxt->adapter);
	if (0 != ret) {
		printk(KERN_ERR PFX "Can't register backend!\n");
		free_irq(client->irq, ctxt);
		bcm59040_pmu_irqs_cleanup(ctxt);
		bcm59040_ctxt_cleanup(ctxt);
		return -ENXIO;
	}

	{
		int			i;
		struct pmu_client	*pclient;
		bcm59040_client_platform_data_t		*cpdata;

		for(i = 0; i < pdata->pmu_board_info_entries; i++) {
			pclient = pmu_register_device(&pdata->pmu_board_info[i]);
			if(!pclient) {
				free_irq(client->irq, ctxt);
				bcm59040_pmu_irqs_cleanup(ctxt);
				(void)pmu_del_adapter(&ctxt->adapter);
				bcm59040_ctxt_cleanup(ctxt);
				return -ENXIO;
			}

			/*
			 * Register client into our tracking table for possible lookup later.
			 */

			cpdata = pdata->pmu_board_info[i].platform_data;
			if(cpdata->base.client_type < BCMPMU_MAX_CLIENT_TYPES) {
				bcm59040_client_list[cpdata->base.client_type] = pclient;
				PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "type=%d, pclient=%08X\n",
						cpdata->base.client_type,
						(uint32_t)pclient));
			}
		}
	}

	printk(KERN_INFO PFX "BCM59040 pmu bus backend driver, Copyright (c) 2010 TomTom B.V and Broadcom Corporation\n");
	return 0;
}

static int bcm59040_i2c_remove(struct i2c_client *client)
{
	bcm59040_ctxt_t *ctxt = i2c_get_clientdata(client);
	int irq_idx;

	disable_irq(client->irq);
	pmu_del_adapter(&ctxt->adapter);

	/* take away the interrupts from the devices */
	for( irq_idx = 0; irq_idx < BCM59040_NUM_IRQS; irq_idx++ ) {
		bcm59040_virq_set_mask( bcm59040_vint_table[ irq_idx ].int_vector, EIRQ_UNMASK );
	}
	bcm59040_virtual_mask_update(ctxt);

	cancel_work_sync(&ctxt->update_mask_work);

	free_irq(client->irq, ctxt);
	bcm59040_pmu_irqs_cleanup(ctxt);
	bcm59040_ctxt_cleanup(ctxt);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "removed\n"));
	
	return 0;
}

#ifdef CONFIG_PM
static int bcm59040_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	bcm59040_ctxt_t				*ctxt = get_irq_chip_data(IRQ_BCM59XXX_FIRST);
	int					irq_idx;
	int					ret = 0;

	// don't want to break suspend over this
	bcm59040_bus_lock_wait(ctxt);

	/* make sure disabled interrupts get to the device */
	bcm59040_virtual_mask_update(ctxt);

	print_device_masks(ctxt, "Interrupt status registers (before suspend)");
	print_virq_state(ctxt);

	/* Now check our wakeupsources. If it is set as a wakeupsource, enable it otherwise disable it. */
	for( irq_idx = 0; irq_idx < BCM59040_NUM_IRQS; irq_idx++ )
	{
		int j;

		for (j = 0; j < bcm59040_vint_table[irq_idx].num_registers; j++) {
			u8 data;
			int mask_reg = bcm59040_vint_table[irq_idx].reg_list[j].int_reg_addr - BCM59040_REG_INT1 + BCM59040_REG_INT1M;
			int rc;

			rc = bcm59040_bus_read_virtualized(ctxt, mask_reg, &data, 1);
			if (rc > 0) {
				if( ctxt->virq[irq_idx].wakeup )
					data &= ~bcm59040_vint_table[irq_idx].reg_list[j].reg_mask;
				else
					data |= bcm59040_vint_table[irq_idx].reg_list[j].reg_mask;
				rc = bcm59040_bus_write_virtualized(ctxt, mask_reg, &data, 1);
				if (rc < 0)
					printk(KERN_ERR PFX "Failed to write mask register %d during suspend: %d\n", mask_reg, rc);
			} else {
				printk(KERN_ERR PFX "Failed to read mask register %d during suspend: %d\n", mask_reg, rc);
			}
		}
	}
	bcm59040_virtual_mask_update(ctxt);

	print_device_masks(ctxt, "Interrupt status registers (after suspend)");
	print_virq_state(ctxt);

	printk ("Suspend processed for bcm59040-core\n");

	bcm59040_bus_unlock(ctxt);

	/* it is scheduled by the above bus_write_virtualized but not needed */
	cancel_work_sync(&ctxt->update_mask_work);

	if (bcm59040_virq_need_wakeup(ctxt)) {
		enable_irq_wake(client->irq);
	} else {
		disable_irq(client->irq);
	}

#ifdef CONFIG_REGULATOR
	/* If we support regulators, put it in the right state. */
	if( state.event == PM_EVENT_SUSPEND )
		ret = regulator_suspend_prepare( PM_SUSPEND_MEM );
#endif

	return ret;
}

static int bcm59040_i2c_resume(struct i2c_client *client)
{
	bcm59040_ctxt_t				*ctxt=get_irq_chip_data(IRQ_BCM59XXX_FIRST);
	int					irq_idx;

	if (bcm59040_virq_need_wakeup(ctxt)) {
		disable_irq_wake(client->irq);
	} else {
		enable_irq(client->irq);
	}

	// it can't be locked here
	WARN_ON(!bcm59040_bus_lock(ctxt));

	print_device_masks(ctxt, "Interrupt status registers (before resume)");
	print_virq_state(ctxt);

	// this restores all the interrupt masks in the device etc.
	for( irq_idx = 0; irq_idx < BCM59040_NUM_IRQS; irq_idx++ ) {
		bcm59040_virq_set_mask(bcm59040_vint_table[ irq_idx].int_vector,
				ctxt->virq[irq_idx].masked ? EIRQ_MASK : EIRQ_UNMASK );
	}
	bcm59040_virtual_mask_update(ctxt);

	print_device_masks(ctxt, "Interrupt status registers (after resume)");
	print_virq_state(ctxt);
	printk ("Resume processed for bcm59040-core\n");

	bcm59040_bus_unlock(ctxt);

	return 0;
}

#else
#define bcm59040_i2c_suspend	NULL
#define bcm59040_i2c_resume	NULL
#endif /* CONFIG_PM	*/

static const struct i2c_device_id bcm59040_id[] = {
	{ BCM59040_DEVNAME, 1, },
	{ }
};

/* BCM59040 core driver registration data
 *
 */

static struct i2c_driver bcm59040_i2c_driver =
{
	.id		= /*I2C_DRIVERID_PMU*/ /*I2C_DRIVERID_BCM59040*/1049, /* CLEANUP ... */
	.probe	= bcm59040_i2c_probe,
	.remove	= bcm59040_i2c_remove,
	.suspend= bcm59040_i2c_suspend,
	.resume	= bcm59040_i2c_resume,
	.driver = {
		.name	= BCM59040_DEVNAME,
		.owner	= THIS_MODULE,
	},
	.id_table	= bcm59040_id,
};

/** bcm59040_core_init - BCM59040 core module Initialization
 *
 * Registers the BCM59040 core driver with the PMU device
 * framework.
 *
 * @return 0 if successful otherwise standard Linux error codes.
 *
 */

static int __init bcm59040_core_init(void)
{
	int err;

	if ((err = i2c_add_driver(&bcm59040_i2c_driver))) {
		printk(KERN_ERR PFX "Could Not Be Added. Err Code: [%i]\n", err);
		return err;
	}

	PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "i2c driver registered\n"));

	return 0;
}

/** bcm59040_core_exit - BCM59040 core module Exit
 *
 * Deregisters the BCM59040 core driver with the PMU device
 * framework.
 *
 * @return 0 if successful otherwise standard Linux error codes.
 *
 */

static void __exit bcm59040_core_exit(void)
{
	i2c_del_driver(&bcm59040_i2c_driver);
}


subsys_initcall(bcm59040_core_init);
module_exit(bcm59040_core_exit);

MODULE_DESCRIPTION(BCM59040_CORE_MOD_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation and TomTom B.V");
MODULE_VERSION(BCM59040_CORE_MOD_VERSION);
