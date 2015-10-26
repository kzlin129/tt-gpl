/* drivers/barcelona/adc/adc.c
 *
 * Implementation of the ADC driver.
 *
 * Copyright (C) 2004,2005,2006,2007,2008,2009,2010 TomTom BV <http://www.tomtom.com/>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 * Author: Koen Martens <kmartens@sonologic.nl>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 * Author: Rogier Stam <rogier.stam@tomtom.com> (CPU frequency scaling part)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Description:
 *
 * The ADC driver, used by other drivers.  Starts a timer upon initialisation,
 * and takes care of ADC sampling.  Other drivers access these values through
 * a public array.  Previously part of the accelerometer driver.
 */

/* Includes */
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/hardware/clock.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-adc.h>
#include <asm/arch/regs-timer.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-dsc.h>
#include <barcelona/timer.h>
#include <barcelona/Barc_adc.h>
#include <barcelona/gopins.h>
#include <barcelona/gotype.h>
#include <barcelona/debug.h>
#include <linux/cpufreq.h>

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <barcelona/cpufreq_order.h>

static struct notifier_block	freq_transition;
static struct notifier_block	freq_policy;
#endif

/* Defines */
//#define ADC_DELAY            0x80eb // 500us @ 66 MHz 660us @ 50 MHz
#define ADC_DELAY            0xf000 // 930 us @ 66 MHz
#define ADC_PRESCALE_DIV	256
#define PFX "adc: "
#define PK_DBG PK_DBG_FUNC
#undef ADC_SPEED_TEST

#define ADC_RAWRATE (adc_num_interrupts)
#define ADC_DIVIDER(divider) ((SYS_TIMER234_PRESCALER + 1) * divider * ADC_RAWRATE)
#define ADC_NOT_USED 255

#define S3C2410_ADCTSC_XM_ENABLE	S3C2410_ADCTSC_XM_SEN
#define S3C2410_ADCTSC_XM_DISABLE	0
#define S3C2410_ADCTSC_XP_ENABLE	0
#define S3C2410_ADCTSC_XP_DISABLE	S3C2410_ADCTSC_XP_SEN
#define S3C2410_ADCTSC_YM_ENABLE	S3C2410_ADCTSC_YM_SEN
#define S3C2410_ADCTSC_YM_DISABLE	0
#define S3C2410_ADCTSC_YP_ENABLE	0
#define S3C2410_ADCTSC_YP_DISABLE	S3C2410_ADCTSC_YP_SEN
#define S3C2410_ADCTSC_PU_ENABLE	0
#define S3C2410_ADCTSC_PU_DISABLE	S3C2410_ADCTSC_PULL_UP_DISABLE

/* Kernel resources */
static int adc_irq;
static struct clk *adc_clock;

/* ADC measured values buffer */
static short adc_buffer[ADC_CHANNELS];
static rwlock_t adc_buffer_lock = RW_LOCK_UNLOCKED;

/* ADC order list */
struct adc_chanlist_t {
	unsigned int	channel;
	unsigned int	rate;
};
static struct adc_chanlist_t adc_chanlist[ADC_CHANNELS];

/* ADC prescaler */
static unsigned long adc_prescaler = ADC_PRESCALE_DIV;

/* Poll function list */
struct adc_pollfunc_node {
	struct list_head	node;
	adc_pollfunc_t		func;
	void			*arg;
	unsigned int		rate;
};
static LIST_HEAD(adc_pollfunc_list);
static spinlock_t adc_pollfunc_list_lock = SPIN_LOCK_UNLOCKED;

static int adc_channel = ADC_CHANNELS;
static unsigned int adc_tscfg_xy = S3C2410_ADCTSC_XM_DISABLE | S3C2410_ADCTSC_XP_DISABLE | S3C2410_ADCTSC_YM_DISABLE | S3C2410_ADCTSC_YP_DISABLE | S3C2410_ADCTSC_PU_DISABLE;
static int adc_num_channels   = ADC_CHANNELS;
static int adc_num_interrupts = 0;

static unsigned adc_divider=SYS_TIMER3_DIVIDER;

static ssize_t adc_show( struct device *dev, struct device_attribute *attr, char *buffer )
{
    int   s32ChannelIndex = 0 ;
    short u16AdcLevel     = 0 ;

    /* Retrieve the device-related data. There is no specific accesser to do so. */
    struct adc_channels * padc_channels = (struct adc_channels *) dev->platform_data ;


    /* Cover the ADC device structure to find the corresponding ADC channel */
    while (    (NULL != padc_channels[s32ChannelIndex].name) 
            && (0 != strcmp(padc_channels[s32ChannelIndex].name, attr->attr.name))
          )
    {
        s32ChannelIndex ++ ;
    }

    if (NULL != padc_channels[s32ChannelIndex].name)
    {
        /* The corresponding channel has been found */
        int s32ChannelNb = padc_channels[s32ChannelIndex].number ;

        if (s32ChannelNb < ADC_CHANNELS)
        {
            /* The channel number seems right */
            u16AdcLevel = adc_buffer[s32ChannelNb] ;
            snprintf(buffer, PAGE_SIZE, "%i", u16AdcLevel);
        }
        else
        {
            snprintf(buffer, PAGE_SIZE, "Wrong Channel Number\n");
        }
    }
    else
    {
        snprintf(buffer, PAGE_SIZE, "Channel Name Not Found\n");
    }

    return strlen(buffer);
}

static struct device_attribute * pdev_attr = NULL ;

static void adc_create_device_attr(const char * a_name, struct device_attribute * a_attr)
{
    a_attr->attr.name  = a_name ;
    a_attr->attr.owner = THIS_MODULE ;
    a_attr->attr.mode  = S_IRUSR | S_IRGRP ;

    a_attr->show       = adc_show ;
    a_attr->store      = NULL ; /* No store callback, since Read Only */
}

static int adc_nb_attr(struct adc_channels * a_padc_channels)
{
    int index ;

    for (index = 0 ; (a_padc_channels[index].name != NULL) ; index ++) ;

    return index ;
}

static int adc_register_device_attributes(struct device *dev)
{
    int ret    = 0 ;
    int nbAttr = 0 ;
    int index ;

    /* Retrieve the device-related data. There is no specific accesser to do so. */
    struct adc_channels * padc_channels = (struct adc_channels *) dev->platform_data ;

    nbAttr = adc_nb_attr(padc_channels) ;

    /* pdev_attr will be FREED in adc_unregister_device_attributes */
    pdev_attr = kmalloc(sizeof(struct device_attribute) * nbAttr, GFP_KERNEL);

    if (pdev_attr != NULL) 
    {
        for (index = 0 ; (index < nbAttr) && (0 == ret) ; index ++)
        {
            adc_create_device_attr(padc_channels[index].name, &pdev_attr[index]) ;
            ret = device_create_file(dev, &pdev_attr[index]) ;
        }

        if (0 != ret)
        {
            /* something went wrong */
            for(index -- ; index >= 0 ; index--)
                device_remove_file(dev, &pdev_attr[index]) ;
        }
    }
    else
    {
        PK_ERR("Unable to allocate ADC attributes\n");
        ret = -ENOMEM ; /* Could not allocate the device_attribute structure */
    }

    return ret ;
}

static void adc_unregister_device_attributes(struct device *dev)
{
    int nbAttr ;
    int index  ;

    /* Retrieve the device-related data. There is no specific accesser to do so. */
    struct adc_channels * padc_channels = (struct adc_channels *) dev->platform_data ;

    nbAttr = adc_nb_attr(padc_channels) ;

    if (pdev_attr != NULL) 
    {
        for (index = 0 ; (index < nbAttr) ; index ++)
            device_remove_file(dev, &pdev_attr[index]) ;
    }

    /* Allocated in adc_register_device_attributes */
    kfree(pdev_attr) ;
}

void adc_get_buffer(short *buf)
{
	unsigned long flags;

	read_lock_irqsave(&adc_buffer_lock, flags);
	memcpy(buf, adc_buffer, ADC_BUFSIZE);
	read_unlock_irqrestore(&adc_buffer_lock, flags);
}
EXPORT_SYMBOL(adc_get_buffer);

short adc_get_channel(int chan)
{
	short val;
	unsigned long flags;

	if (chan < 0 || chan >= ADC_CHANNELS)
		return -EINVAL;

	read_lock_irqsave(&adc_buffer_lock, flags);
	val = adc_buffer[chan];
	read_unlock_irqrestore(&adc_buffer_lock, flags);

	return val;
}
EXPORT_SYMBOL(adc_get_channel);

int adc_register_poll(adc_pollfunc_t func, void* arg, unsigned int rate)
{
	struct adc_pollfunc_node *buf;
	unsigned long flags;

	buf = kmalloc(sizeof(struct adc_pollfunc_node), GFP_KERNEL);
	if (buf == NULL) {
		PK_ERR("Unable to allocate pollfunc node\n");
		return -ENOMEM;
	}

	/* Quite a bit of overhead for a simple pointer... */
	buf->func = func;
	buf->arg  = arg;
	buf->rate = rate;

	spin_lock_irqsave(&adc_pollfunc_list_lock, flags);
	list_add_tail(&buf->node, &adc_pollfunc_list);
	spin_unlock_irqrestore(&adc_pollfunc_list_lock, flags);

	return 0;
}
EXPORT_SYMBOL(adc_register_poll);

int adc_unregister_poll(adc_pollfunc_t func)
{
	unsigned long flags;
	struct adc_pollfunc_node *p;

	spin_lock_irqsave(&adc_pollfunc_list_lock, flags);
	list_for_each_entry(p, &adc_pollfunc_list, node) {
		if (p->func == func) {
			list_del(&p->node);
			kfree(p);
			spin_unlock_irqrestore(&adc_pollfunc_list_lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&adc_pollfunc_list_lock, flags);

	PK_WARN("Pollfunc %p not found\n", func);
	return -EINVAL;
}
EXPORT_SYMBOL(adc_unregister_poll);

inline void PrepTsForSampling(int chan, short *buffer)
{
	/* Touchscreen switching is done one period earlier than the actual measurement to give the touchscreen signals time to settle. This is because of capacitors on the touchscreen siganls. So don't modify the sequence! */ 
 	switch(chan) {
	case ADC_TSW_DOWN:
		if (IO_HasPin(TSDOWN)) {
			// Set pen down measurement with enabled pullup and GPIO control
			__raw_writel(S3C2410_ADCTSC_XM_DISABLE | S3C2410_ADCTSC_XP_DISABLE | S3C2410_ADCTSC_YM_DISABLE | S3C2410_ADCTSC_YP_DISABLE | S3C2410_ADCTSC_PU_ENABLE, S3C2410_ADCTSC);
			// Enable TS Z plane for reading
			IO_Deactivate(XMON);
			IO_Deactivate(XPON);
			IO_Deactivate(YMON);
			IO_Deactivate(YPON);
			IO_Activate(TSDOWN);
		} else {
			// Set pen down measurement using TS controller with enabled pullup
			__raw_writel(S3C2410_ADCTSC_XM_DISABLE | S3C2410_ADCTSC_XP_DISABLE | S3C2410_ADCTSC_YM_ENABLE | S3C2410_ADCTSC_YP_DISABLE | S3C2410_ADCTSC_PU_ENABLE, S3C2410_ADCTSC);
		}
		break;
		
	case ADC_TSW_X:
		if (buffer[ADC_TS_DOWN] < TSINPUT_PRESSURE_TRIGGER)
		{
			if (IO_HasPin(TSDOWN)) {
				// Set X measurement with GPIO control
				__raw_writel(adc_tscfg_xy, S3C2410_ADCTSC);
				// Enable TS X plane for reading
				IO_Deactivate(TSDOWN);
				IO_Deactivate(YMON);
				IO_Deactivate(YPON);
				IO_Activate(XMON);
				IO_Activate(XPON);
			} else {
				// Set X measurement using TS controller with disabled pullup
				__raw_writel(S3C2410_ADCTSC_XM_ENABLE | S3C2410_ADCTSC_XP_ENABLE | S3C2410_ADCTSC_YM_DISABLE | S3C2410_ADCTSC_YP_DISABLE | S3C2410_ADCTSC_PU_DISABLE, S3C2410_ADCTSC);
			}
		}
		break;
		
	case ADC_TSW_Y:
		if (buffer[ADC_TS_DOWN] < TSINPUT_PRESSURE_TRIGGER)
		{
			if (IO_HasPin(TSDOWN)) {
				// Set Y measurement with GPIO control
				__raw_writel(adc_tscfg_xy, S3C2410_ADCTSC);
				// Enable TS Y plane for reading
				IO_Deactivate(TSDOWN);
				IO_Deactivate(XMON);
				IO_Deactivate(XPON);
				IO_Activate(YMON);
				IO_Activate(YPON);
			} else {
				// Set Y measurement using TS controller with disabled pullup
				__raw_writel(S3C2410_ADCTSC_XM_DISABLE | S3C2410_ADCTSC_XP_DISABLE | S3C2410_ADCTSC_YM_ENABLE | S3C2410_ADCTSC_YP_ENABLE | S3C2410_ADCTSC_PU_DISABLE, S3C2410_ADCTSC);
			}
		}
		break;
	default:
		// Do nothing here!!!
		break;
	}
}

inline void adc_conversion_complete(short *tmpbuf, unsigned long *lrate)
{
	unsigned long flags;
	struct adc_pollfunc_node *p;

	/* When all channels have been converted, update the buffer atomically. */
	write_lock_irqsave(&adc_buffer_lock, flags);
	memcpy((void *) adc_buffer, (const void *) tmpbuf, ADC_BUFSIZE);
	write_unlock_irqrestore(&adc_buffer_lock, flags);

	/* Then call all the registered poll functions. */
	spin_lock_irqsave(&adc_pollfunc_list_lock, flags);

	if (*lrate < ADC_LTRESHOLD) {
		list_for_each_entry(p, &adc_pollfunc_list, node) {
			if (p->rate != ADC_LRATE)
				p->func(adc_buffer, p->arg );
		}
		(*lrate)++;
	} else {
		list_for_each_entry(p, &adc_pollfunc_list, node) {
			p->func(adc_buffer, p->arg );
		}
		*lrate = 0;
	}

	spin_unlock_irqrestore(&adc_pollfunc_list_lock, flags);
}

static irqreturn_t adc_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	static short tmpbuf[ADC_CHANNELS];
	static unsigned long lrate = 0;
	unsigned long reg;
	unsigned long cond;

#if 0
	/* 
	 * This code checks distribution of IRQ's for each registered ADC channel
	 */

	static unsigned long prev = 0;
	static unsigned long cnt = 0;
	static unsigned long stats[ADC_CHANNELS];
	unsigned int pom;
	
	stats[adc_channel]++;
	
	if (cnt == 0) {
		prev = jiffies;
	}
	cnt++;
	
	if((jiffies - prev) >= HZ) {
		printk(KERN_ERR "--- %s:%u ---\n", __func__, cnt);
		for (pom = 0; pom < ADC_CHANNELS; pom++) {
			printk(KERN_ERR "%2u : %4u\n", pom, stats[pom]);
			stats[pom] = 0;
		}
		cnt = 0;
	}
#endif

	if (adc_channel >= ADC_CHANNELS) 
	{
		/* Don't store conversion the first time */
		adc_channel = 0;
	} 
	else 
	{
		/* This wait is needed, else the conversion will not take place */
		while (__raw_readl(S3C2410_ADCCON) & S3C2410_ADCCON_ENABLE_START)
			;

		/* At even interrupt read and store the value and select the next channel */
		tmpbuf[adc_channel] = __raw_readl(S3C2410_ADCDAT0) & S3C2410_ADCDAT0_XPDATA_MASK;

		/* Jump to next channel and skip unused channels */
		do {
			if (++adc_channel >= ADC_CHANNELS) {
				adc_channel = 0;
				adc_conversion_complete(tmpbuf, &lrate);
			}

			cond = (adc_chanlist[adc_channel].rate == ADC_LRATE) && \
			       (lrate < ADC_LTRESHOLD);

		} while ((adc_chanlist[adc_channel].channel == ADC_NOT_USED) || cond);
	}

	/* Control the touchscreen as required by the chan */
	PrepTsForSampling(adc_channel, tmpbuf);

	if (IO_GetCpuType() == GOCPU_S3C2443) {
		__raw_writel(adc_chanlist[adc_channel].channel, S3C2443_ADCMUX);
	} else if (IO_GetCpuType() == GOCPU_S3C2450) {
		__raw_writel(adc_chanlist[adc_channel].channel, S3C2450_ADCMUX);
	} else {
		reg = __raw_readl(S3C2410_ADCCON);
		reg &= ~S3C2410_ADCCON_MUXMASK;
		reg |= S3C2410_ADCCON_SELMUX(adc_chanlist[adc_channel].channel);
		__raw_writel(reg, S3C2410_ADCCON);
	}

	reg = __raw_readl(S3C2410_ADCCON);
	reg |= S3C2410_ADCCON_ENABLE_START;
	__raw_writel(reg, S3C2410_ADCCON);

	return IRQ_HANDLED;
}

static ssize_t adc_read(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
	int ret;
	unsigned long flags;

	/* validate the size count passed */
	if (len < ADC_BUFSIZE)
		return -EINVAL;

	/* check user buffer sanity */
	if (!access_ok(VERIFY_WRITE, data, ADC_BUFSIZE))
		return -EFAULT;

	read_lock_irqsave(&adc_buffer_lock, flags);
	ret = __copy_to_user(data, adc_buffer, ADC_BUFSIZE) ? -EFAULT : ADC_BUFSIZE;
	read_unlock_irqrestore(&adc_buffer_lock, flags);

	return ret;
}

static int adc_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int adc_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* Kernel interface */
static struct file_operations adc_fops = {
	.owner		= THIS_MODULE,
	.read		= adc_read,
	.open		= adc_open,
	.release	= adc_release,
};

static void adc_set_rate(unsigned rate)
{
	unsigned tcntb3;
	unsigned tcntb3_mod;
	unsigned tcon;

	/* Set the reload bit. */
        tcon = __raw_readl(S3C2410_TCON);
        tcon |= S3C2410_TCON_T3RELOAD;
        __raw_writel(tcon, S3C2410_TCON);

	/* Determine the value of the count/compare register. */
	tcntb3 = rate / ADC_DIVIDER(adc_divider) - 1;
	tcntb3_mod = rate % ADC_DIVIDER(adc_divider);
	PK_DBG("tcntb3 = %#x, tcntb3_mod = %#x\n", tcntb3, tcntb3_mod);
	__raw_writew(tcntb3, S3C2410_TCNTB(3));
	__raw_writew(0, S3C2410_TCMPB(3));

        /* Set manual update to allow us to write to the registers. */
        tcon = __raw_readl(S3C2410_TCON);
        tcon |= S3C2410_TCON_T3MANUALUPD;
        __raw_writel(tcon, S3C2410_TCON);

        /* Now clear the inverter. */
        tcon &= ~S3C2410_TCON_T3INVERT;
        __raw_writel(tcon, S3C2410_TCON);

        /* Clear the manual update bit. */
	tcon &= ~S3C2410_TCON_T3MANUALUPD;
        __raw_writel(tcon, S3C2410_TCON);

	/* Start the timer. */
	PK_DBG("Starting timer 3\n");
        __raw_writel((tcon | S3C2410_TCON_T3START), S3C2410_TCON);

}

static int adc_hw_init(struct device *dev, unsigned long adccon)
{
	unsigned rate;

	adc_channel = ADC_CHANNELS;

	// With external FETs the pullup needs to be enabled during X and Y measurement, because this was also the case during calibration (only valid vor GO Classic)
	if (IO_HaveTsFets())
		adc_tscfg_xy = S3C2410_ADCTSC_XM_DISABLE | S3C2410_ADCTSC_XP_DISABLE | S3C2410_ADCTSC_YM_DISABLE | S3C2410_ADCTSC_YP_DISABLE | S3C2410_ADCTSC_PU_ENABLE;
	else
		adc_tscfg_xy = S3C2410_ADCTSC_XM_DISABLE | S3C2410_ADCTSC_XP_DISABLE | S3C2410_ADCTSC_YM_DISABLE | S3C2410_ADCTSC_YP_DISABLE | S3C2410_ADCTSC_PU_DISABLE;

	PK_DBG("Getting ADC clock\n");
	adc_clock = clk_get(dev, "adc");
	if (IS_ERR(adc_clock)) {
		adc_clock = NULL;
		PK_ERR("Failed to get ADC clock\n");
		return -ENOENT;
	}

	PK_DBG("Using and enabling ADC clock\n");
	clk_use(adc_clock);
	clk_enable(adc_clock);

	/* ADCCON needs to be written AFTER the ADC clock is turned on, or the
	 * writes will be cheerfully ignored, at least on S3C2443. */
	__raw_writel(adccon, S3C2410_ADCCON);
	__raw_writel(ADC_DELAY, S3C2410_ADCDLY);

	rate = clk_get_rate(adc_clock);
	PK_DBG("ADC clock rate: %u\n", rate);
	
	adc_set_rate(rate);

	PK_DBG("Turning on AIN4_PWR\n");
	IO_Activate(AIN4_PWR);

	PK_DBG("Done\n");
	return 0;
}

#define S3C2410_TCON_T0OFFSET  0
#define S3C2410_TCON_T1OFFSET  8
#define S3C2410_TCON_T2OFFSET  12
#define S3C2410_TCON_T3OFFSET  16
#define S3C2410_TCON_T4OFFSET  20

static void adc_hw_exit(void)
{
	PK_DBG("Turning off AIN4_PWR\n");
	IO_Deactivate(AIN4_PWR);

	PK_DBG("Stopping timer 1\n");
	__raw_writel((__raw_readl(S3C2410_TCON) & ~(0xf << S3C2410_TCON_T3OFFSET)) | (0xa <<  S3C2410_TCON_T3OFFSET), S3C2410_TCON); /* Stop and manual load */  // temp _NT_
	if (adc_clock != NULL) {
		PK_DBG("Removing clock\n");
		clk_disable(adc_clock);
		clk_unuse(adc_clock);
		clk_put(adc_clock);
		adc_clock = NULL;
	}

	// Put ADC in standby mode
	__raw_writel(S3C2410_ADCCON_PRSCEN | S3C2410_ADCCON_PRSCVL(adc_prescaler - 1) | S3C2410_ADCCON_STDBM, S3C2410_ADCCON);
	__raw_writel(S3C2410_ADCTSC_PULL_UP_DISABLE | S3C2410_ADCTSC_XP_SEN | S3C2410_ADCTSC_YP_SEN, S3C2410_ADCTSC);

	// Put touchscreen in standby mode
	IO_Suspend(TSDOWN);
	IO_Suspend(XMON);
	IO_Suspend(XPON);
	IO_Suspend(YMON);
	IO_Suspend(YPON);

	PK_DBG("Done\n");
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
/* Max PCLK freq is the first PCLK frequency set when the frequency scaling code is called. This */
/* will always be the transition from max freq -> max freq. */
static unsigned long int	max_pclk_freq=0;
/*
 * CPU clock speed change handler. Change adc samplerate, reduce channels, or refuse policy.
 */
static int
adc_freq_transition(struct notifier_block *nb, unsigned long val,
			 void *data)
{
	unsigned long int	div=2 << ((__raw_readl( S3C2410_TCFG1 ) >> 4) & 0x0F);
	struct cpufreq_freqs	*f = data;
	unsigned long int	adccon;
	unsigned long int	prescale=(__raw_readl( S3C2410_TCFG0 ) & 0x000000FF) + 1;
	unsigned long int	pclk_max=(prescale*ADC_RAWRATE*16*65535)/1000;
	unsigned long int	pclk_min=(prescale*ADC_RAWRATE*2*1)/1000;
	unsigned long int	pclk_new;
	unsigned long int	pclk_old;
	unsigned long int	tcon;
	unsigned long int	pref_adc_clk;
	unsigned long int	new_adc_prescale;
	unsigned long int	flags;
	unsigned long int	res, den;

	/* Get the minimum and maximum PCLK frequencies. */
	f->trans2pclk( f, &pclk_old, &pclk_new );

	/* Save the first frequency. This will be the transition from max -> max */
	if( max_pclk_freq == 0 )
		max_pclk_freq=pclk_new;

	/* Get preferred adc clock. */
	pref_adc_clk=(max_pclk_freq + ADC_PRESCALE_DIV - 1)/ADC_PRESCALE_DIV;

	/* Determine if the ADC can handle the min and maximum values. */
	new_adc_prescale=(pclk_min + pref_adc_clk - 1)/pref_adc_clk;
	if( new_adc_prescale < 1 )
		pclk_min=pref_adc_clk*1;

	new_adc_prescale=(pclk_max + pref_adc_clk - 1)/pref_adc_clk;
	if( new_adc_prescale > ADC_PRESCALE_DIV )
		pclk_max=pref_adc_clk*ADC_PRESCALE_DIV;

	/* Div <= 16 means that we derive from PCLK. */
	if( div <= 16 )
	{
		/* Save flags and disable IRQs. */
		local_irq_save(flags);
		local_fiq_disable( );

		switch (val) {
		case CPUFREQ_PRECHANGE:
 
			/* Disabling the ADC we do here, enabling not however, as that would mean we'd be using the */
			/* old (illegal) values still. */
			/* Calculate new divider. */
			adc_divider=pclk_new/(ADC_DIVIDER( 1 )); 

			/* Stop the PWM associated with the ADC. */
			tcon = __raw_readl(S3C2410_TCON);
			tcon &= ~(S3C2410_TCON_T3START);
			__raw_writel(tcon, S3C2410_TCON); /* Stop and manual load */

			/* Was the old value legal ? */
			if( (pclk_old >= pclk_min) && (pclk_old <= pclk_max) )
			{
				/* Is the new value legal? */
				if( (pclk_new >= pclk_min) && (pclk_new <= pclk_max) )
				{
					/* Legal -> Legal. */
					clk_disable( adc_clock );
				}
				else
				{
					/* Legal -> Illegal. */
					adc_hw_exit( );
				}
			}
			else
			{
				/* Previous value was illegal. No need to do anything, it's still disabled. */
			}
			break;

		case CPUFREQ_POSTCHANGE:
			/* Only reenable if the values are legal. */
			/* Was the old value legal ? */
			if( (pclk_old >= pclk_min) && (pclk_old <= pclk_max) )
			{
				/* Is the new value legal? */
				if( (pclk_new >= pclk_min) && (pclk_new <= pclk_max) )
				{
					/* Legal -> Legal. */
					/* As the clock divider usually won't be one of the legal values, calculate */
					/* the nearest legal value. */
					adc_divider=1;
					do
					{
						adc_divider<<=1;
						den=ADC_DIVIDER( adc_divider );
						res=((pclk_new + den - 1)/den - 1);
					}
					while( (res > 0xFFFF) && (adc_divider < 16) );
						
					if( (adc_divider > 16) || (res > 0xFFFF) )
					{
						printk( "WARNING! Illegal value for ADC divider detected! Adjusted to legal value. Expect artefacts. (transition from %lu to %lu)\n", f->old, f->new );
						if( adc_divider > 16 ) adc_divider=16;
					}

					/* Enable the clock. */
					clk_enable( adc_clock );

					/* Adjust the prescaler. This makes sure that the conversion goes with the same speed */
					/* as before. */
					adccon = __raw_readl(S3C2410_ADCCON);
					adc_prescaler = pclk_new / pref_adc_clk;
					adccon &= ~S3C2410_ADCCON_PRSCVLMASK;
					adccon |= S3C2410_ADCCON_PRSCVL(adc_prescaler - 1);
					__raw_writel(adccon, S3C2410_ADCCON);

					/* Set the rate using the new divider. */
					adc_set_rate( pclk_new * 1000 );
				}
				else
				{
					/* No need to do anything. Leave it disabled. */
				}
			}
			else
			{
				/* Old value was illegal. New value? */
				if( (pclk_new >= pclk_min) && (pclk_new <= pclk_max) )
				{
					/* Illegal -> Legal. */
					/* As the clock divider usually won't be one of the legal values, calculate */
					/* the nearest legal value. */
					div=adc_divider;
					adc_divider=2;
					while( adc_divider < div ) adc_divider<<=1;
					if( adc_divider > 16 )
					{
						printk( "WARNING! Illegal value for ADC divider detected! Adjusted to legal value. Expect artefacts. (transition from %lu to %lu)\n", f->old, f->new );
						adc_divider=16;
					}

					/* Unuse the clock to get right counters. */
					clk_unuse( adc_clock );
					clk_put( adc_clock );

					/* Adjust the prescaler. This makes sure that the conversion goes with the same speed */
					/* as before. */
					adccon = __raw_readl(S3C2410_ADCCON);
					new_adc_prescale = ((adccon & S3C2410_ADCCON_PRSCVLMASK) >> 6) + 1;
					adc_prescaler = (new_adc_prescale * pclk_old + pclk_new - 1) / pclk_new;
					adccon &= ~S3C2410_ADCCON_PRSCVLMASK;
					adccon |= S3C2410_ADCCON_PRSCVL(adc_prescaler - 1);

					/* Reinit with new clock. */
					adc_hw_init(NULL, adccon);
				}
			}
			break;
		}

		/* Restore IRQ flags. */
		local_irq_restore(flags);
		return 0;
	}
	else return 0;
}

static int
adc_freq_policy(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	struct cpufreq_policy	*policy = data;
	__u32			div=(__raw_readl( S3C2410_TCFG1 ) >> 4) & 0x0F;
	__u32			prescale=(__raw_readl( S3C2410_TCFG0 ) & 0x000000FF) + 1;
	__u32			pclk_max=(prescale*ADC_RAWRATE*16*65535)/1000;
	__u32			pclk_min=(prescale*ADC_RAWRATE*2*1)/1000;
	unsigned long int	pclk_old;
	unsigned long int	pclk_new;
	unsigned long int	pref_adc_clk;
	unsigned long int	new_adc_prescale;

	/* Get current and suggested next PCLK clock. */
	policy->policy2pclk( policy, &pclk_old, &pclk_new );

	/* Save the first frequency. This will be the transition from max -> max */
	if( max_pclk_freq == 0 )
		max_pclk_freq=pclk_new;

	/* Get preferred adc clock. */
	pref_adc_clk=(max_pclk_freq + ADC_PRESCALE_DIV - 1)/ADC_PRESCALE_DIV;

	/* Determine if the ADC can handle the min and maximum values. */
	new_adc_prescale=(pclk_min + pref_adc_clk)/pref_adc_clk;
	if( new_adc_prescale < 1 )
		pclk_min=pref_adc_clk*1;

	new_adc_prescale=(pclk_max + pref_adc_clk)/pref_adc_clk;
	if( new_adc_prescale > ADC_PRESCALE_DIV )
		pclk_max=pref_adc_clk*ADC_PRESCALE_DIV;

	if( div < 0x4 )
	{
		switch (val) {
			case CPUFREQ_ADJUST:
				policy->pclk2policy( policy, pclk_min, pclk_max );
				break;
			case CPUFREQ_INCOMPATIBLE:
				policy->policy2pclk( policy, &pclk_old, &pclk_new );
				if( (pclk_old >= pclk_min) && (pclk_old <= pclk_max) )
					pclk_min=pclk_old;

				if( (pclk_new >= pclk_min) && (pclk_new <= pclk_max) )
					pclk_max=pclk_new;

				if( (pclk_min != pclk_old) || (pclk_max != pclk_new) )
					policy->pclk2policy( policy, pclk_min, pclk_max );
				break;
			case CPUFREQ_NOTIFY:
				/* Illegal values are handled in the transition notifier. */
				break;
		}
	}
	return 0;
}
#endif

static int adc_probe(struct device *dev)
{
	int ret;
	int chan;
	
	/* "clear" channel list and channel data */
	for (chan = 0; chan < ADC_CHANNELS; chan++) {
		adc_chanlist[chan].channel = ADC_NOT_USED;
		adc_chanlist[chan].rate    = ADC_LRATE;
		adc_buffer[chan] = 0;
	}
	
	/* Setup channel list according to available hardware*/

	/* Touchscreen channels setup, ADC_TSW_X, ADC_TSW_Y, ADC_TSW_DOWN are used as dummy periods to switch the touchscreen before the actual measurement takes place. */
	if (IO_GetCpuType() == GOCPU_S3C2443) {
		adc_chanlist[ADC_TSW_DOWN].channel = 9;
		adc_chanlist[ADC_TS_DOWN].channel = 9;
	}
	else
	if (IO_GetCpuType() == GOCPU_S3C2450) {
		adc_chanlist[ADC_TSW_DOWN].channel = 9;
		adc_chanlist[ADC_TS_DOWN].channel = 9;
	}
	else {
		adc_chanlist[ADC_TSW_DOWN].channel = 7;
		adc_chanlist[ADC_TS_DOWN].channel = 7;
	}
	adc_chanlist[ADC_TSW_DOWN].rate = ADC_RATE_TS;
	adc_chanlist[ADC_TS_DOWN].rate = ADC_RATE_TS;
	adc_chanlist[ADC_TSW_X].channel = IO_GetTsXChannel();
	adc_chanlist[ADC_TSW_X].rate    = ADC_RATE_TS;
	adc_chanlist[ADC_TS_X].channel = IO_GetTsXChannel();
	adc_chanlist[ADC_TS_X].rate    = ADC_RATE_TS;
	adc_chanlist[ADC_TSW_Y].channel = IO_GetTsYChannel();
	adc_chanlist[ADC_TSW_Y].rate    = ADC_RATE_TS;
	adc_chanlist[ADC_TS_Y].channel = IO_GetTsYChannel();
	adc_chanlist[ADC_TS_Y].rate    = ADC_RATE_TS;

	adc_chanlist[ADC_BAT].channel  = IO_GetBatVoltageChannel(); //0
	adc_chanlist[ADC_BAT].rate     = ADC_RATE_BAT;
	adc_chanlist[ADC_CHRG].channel = IO_GetBatChargeCurrentChannel(); //6
	adc_chanlist[ADC_CHRG].rate    = ADC_RATE_CHRG;

	if (IO_HaveADCAIN4Ref()) {
		adc_chanlist[ADC_REF].channel  = IO_GetRefVoltageChannel(); //4;
		adc_chanlist[ADC_REF].rate     = ADC_RATE_REF;
	}

	if (IO_GetAccType() == GOACC_MXR3999 || IO_GetAccType() == GOACC_MXR2312 || IO_GetAccType() == GOACC_MXR9500) {
		adc_chanlist[ADC_ACC_X].channel = 3;
		adc_chanlist[ADC_ACC_X].rate    = ADC_RATE_ACC;
		adc_chanlist[ADC_ACC_Y].channel = 2;
		adc_chanlist[ADC_ACC_Y].rate    = ADC_RATE_ACC;
	}
	if (IO_GetAccType() == GOACC_MXR9500 || IO_GetAccType() == GOACC_MXM9301) {
		adc_chanlist[ADC_ACC_Z_TEMP].channel = 1;
		adc_chanlist[ADC_ACC_Z_TEMP].rate    = ADC_RATE_ACC;
	}
	
	if ( IO_HaveDeadReckoning() ) {
		switch( IO_GetAccType( ) )
		{
			/* When using these accelerometers, the channels are reserved and we */
			/* are not using the gyro interface. This is for old models. */
			case GOACC_MXR2312:
			case GOACC_MXR3999:
			case GOACC_MXM9301:
			case GOACC_MXR9500:
				adc_chanlist[ADC_GYRO].channel = 3;
				adc_chanlist[ADC_GYRO].rate    = ADC_RATE_GYRO;
				break;

			case GOACC_KXR94:
				adc_chanlist[ADC_GYRO_Y].channel = IO_GetGyroYChannel( );
				adc_chanlist[ADC_GYRO_Y].rate    = ADC_RATE_GYRO;
				break;

			default:
				adc_chanlist[ADC_GYRO_X].channel = IO_GetGyroXChannel( );
				adc_chanlist[ADC_GYRO_X].rate    = ADC_RATE_GYRO;
				adc_chanlist[ADC_GYRO_Y].channel = IO_GetGyroYChannel( );
				adc_chanlist[ADC_GYRO_Y].rate    = ADC_RATE_GYRO;
				break;
		}
	}

	if (IO_HaveLightSensor()) {
		adc_chanlist[ADC_LX_OUT].channel = IO_GetLightSensorChannel();
		adc_chanlist[ADC_LX_OUT].rate    = ADC_RATE_LX_OUT;
	}

	/* Determine amount of used channels */
	adc_num_channels = 0;
	for (chan = 0; chan < ADC_CHANNELS; chan++) {
		if (adc_chanlist[chan].channel != ADC_NOT_USED) {
			adc_num_interrupts += adc_chanlist[chan].rate;
			adc_num_channels++;
		}
	}

	disable_irq(IRQ_TC);
	disable_irq(IRQ_ADC);

	PK_DBG("Requesting timer IRQ\n");
	ret = request_irq(IRQ_TIMER3, adc_irq_handler, 0, "ADC poll timer", dev);
	if (ret < 0) {
		PK_ERR("Failed to request timer IRQ (%d)\n", ret);
		return ret;
	}
	adc_irq = 1;
	ret = adc_hw_init(dev, S3C2410_ADCCON_PRSCEN | S3C2410_ADCCON_PRSCVL(adc_prescaler - 1));
	if (ret < 0) {
		PK_ERR("Failed to initialize hardware (%d)\n", ret);
		return ret;
	}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	freq_transition.notifier_call = adc_freq_transition;
	freq_transition.priority = CPUFREQ_ORDER_S3C24XX_ADC_PRIO;
	freq_policy.notifier_call = adc_freq_policy;
	cpufreq_register_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	PK_DBG("Registering chardev\n");
	ret = register_chrdev(ADC_MAJOR, ADC_DEVNAME, &adc_fops);
	if (ret < 0) {
		PK_ERR("Unable to register chardev on major=%d (%d)\n", ADC_MAJOR, ret);
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
		cpufreq_unregister_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
		cpufreq_unregister_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif
		return ret;
	}

	/* Register the attribute files */
	ret = adc_register_device_attributes(dev) ;

	PK_DBG("Done\n");
	return ret ;
}

static int adc_remove(struct device *dev)
{
	PK_DBG("Unregistering chardev\n");
	unregister_chrdev(ADC_MAJOR, ADC_DEVNAME);

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	cpufreq_unregister_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	adc_hw_exit();

	if (adc_irq != 0) {
		PK_DBG("Freeing IRQ\n");
		free_irq(IRQ_TIMER3, dev);
		adc_irq = 0;
	}

	if (!list_empty(&adc_pollfunc_list)) {
		struct adc_pollfunc_node *p;
		struct adc_pollfunc_node *q;

		PK_WARN("Pollfunc list is not empty\n");
		list_for_each_entry_safe(p, q, &adc_pollfunc_list, node) {
			list_del(&p->node);
			kfree(p);
		}
		INIT_LIST_HEAD(&adc_pollfunc_list);
	}

	adc_unregister_device_attributes(dev) ;

	PK_DBG("Done\n");
	return 0;
}

static void adc_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");
	adc_hw_exit();
}

#ifdef CONFIG_PM

static int adc_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);
	if (level == SUSPEND_POWER_DOWN) {
		adc_hw_exit();
	}
	return 0;
}

static int adc_resume(struct device *dev, u32 level)
{
	PK_DBG("dev = %p, level = %u\n", dev, level);
	if (level == RESUME_POWER_ON) {
		int ret = adc_hw_init(dev, S3C2410_ADCCON_PRSCEN | S3C2410_ADCCON_PRSCVL(adc_prescaler - 1));
		if (ret < 0) {
			PK_ERR("Failed to initialize hardware (%d)\n", ret);
			return ret;
		}
	}
	return 0;
}

#else /* CONFIG_PM */
#define adc_suspend NULL
#define adc_resume  NULL
#endif /* CONFIG_PM */

static struct device_driver adc_driver = {
	.name		= "tomtomgo-adc",
	.bus		= &platform_bus_type,
	.probe		= adc_probe,
	.remove		= adc_remove,
	.shutdown	= adc_shutdown,
	.suspend	= adc_suspend,
	.resume		= adc_resume,
};

static int __init adc_mod_init(void)
{
	int ret;

	printk(KERN_INFO "TomTom GO ADC Driver, (C) 2004-2010 TomTom BV\n");
	PK_DBG("Registering driver\n");
	ret = driver_register(&adc_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit adc_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&adc_driver);
	PK_DBG("Done\n");
}

module_init(adc_mod_init);
module_exit(adc_mod_exit);

MODULE_AUTHOR("Koen Martens <kmartens@sonologic.nl>, Jeroen Taverne <jeroen.taverne@tomtom.com> and Dimitry Andric <dimitry.andric@tomtom.com>");
MODULE_DESCRIPTION("TomTom GO ADC Driver");
MODULE_LICENSE("GPL");

/* EOF */
