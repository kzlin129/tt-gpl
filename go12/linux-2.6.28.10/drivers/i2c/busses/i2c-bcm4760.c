/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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

/* ---- Include files ---------------------------------------------------- */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <asm/arch/reg_i2c.h>
#include <asm/arch/hardware.h>
#include <linux/broadcom/bcm_sysctl.h>
#include <linux/broadcom/timer.h>

/* ---- Macro definitions ------------------------------------------------ */

#define I2C_POLL_COMMAND_DONE 0

#define PFX "I2C: "

#define BSC_TIM_DIV_MASK 	0x03
#define BSC_TIM_P_VAL_MASK 	0x38
#define BSC_TIM_P_VAL_BIT_SHIFT    3

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
#define DBG_DATA2	0x20

#define DBG_DEFAULT_LEVEL  (DBG_ERROR)
//#define DBG_DEFAULT_LEVEL  (DBG_ERROR|DBG_INFO)

static int gLevel = DBG_DEFAULT_LEVEL;

#ifdef DEBUG
 #define I2C_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
 #define I2C_DEBUG(level,x)
#endif

/* ---- Structure definitions -------------------------------------------- */

struct bcm4760_i2c_speeds
{
    u8 input_clock;     /* input clock value in Mhz */
    u16 i2c_clock_req;  /* I2C clock speed required in Kbps */
    u8 bsctim_val;      /* I2C BSCTIM register load value */
    u8 bscclken_val;    /* I2C BSCCLKEN register load value */
};
	 

struct bcm4760_i2c
{
    u32 interrupt;      /* isr value */
    u32 ctl_id;         /* controller id */
    u32 base;           /* base address */
    u32 speed;          /* i2c speed */
#if !I2C_POLL_COMMAND_DONE
    wait_queue_head_t queue;
#endif
#ifdef CONFIG_PM
    u32 suspended:1;    /* 1 if suspended, 0 if not */
#endif
    struct i2c_adapter adap;
};

/* ---- Function prototypes
 */
static void bcm4760_i2c_enable_controller(struct bcm4760_i2c *i2c);

/*
 * Lookup table used to translate input reference clock and
 * desired I2C speed into correct register settings.
 *
 * Note: This table must be in increasing order of input_clock
 *       and i2c_clock_req, in that order. If this isn't maintained
 *       the simplistic lookup routine will not set the correct
 *       maximum value should the caller attempt to set a value
 *       higher than that allowed given the input reference clock
 *       and the desired I2C transfer clock.
 */
static struct bcm4760_i2c_speeds i2c_speed_list[] =
{
    {
        .input_clock =      12,
        .i2c_clock_req =    100,
        .bsctim_val =       (REG_I2C_TIM_DIV8| REG_I2C_TIM_P2),
        .bscclken_val =     0
    },
    {
        .input_clock =      12,
        .i2c_clock_req =    200,
        .bsctim_val =       (REG_I2C_TIM_DIV4| REG_I2C_TIM_P2),
        .bscclken_val =     0
    },
    {
        .input_clock =      12,
        .i2c_clock_req =    400,
        .bsctim_val =       (REG_I2C_TIM_DIV2| REG_I2C_TIM_P0),
        .bscclken_val =     (REG_I2C_CLKEN_M3 | REG_I2C_CLKEN_N4)
    },
    {
        .input_clock =      24,
        .i2c_clock_req =    100,
        .bsctim_val =       (REG_I2C_TIM_DIV16| REG_I2C_TIM_P2),
        .bscclken_val =     0
    },
    {
        .input_clock =      24,
        .i2c_clock_req =    200,
        .bsctim_val =       (REG_I2C_TIM_DIV8| REG_I2C_TIM_P2),
        .bscclken_val =     0
    },
    {
        .input_clock =      24,
        .i2c_clock_req =    400,
        .bsctim_val =       (REG_I2C_TIM_DIV4| REG_I2C_TIM_P0),
        .bscclken_val =     (REG_I2C_CLKEN_M3 | REG_I2C_CLKEN_N3)
    },
    {
        .input_clock =      24,
        .i2c_clock_req =    800,
        .bsctim_val =       (REG_I2C_TIM_DIV2| REG_I2C_TIM_P1),
        .bscclken_val =     (REG_I2C_CLKEN_AUTO_SENSE_OFF | REG_I2C_CLKEN_M2 | REG_I2C_CLKEN_N2)
    },
    {
        .input_clock =      24,
        .i2c_clock_req =    1300,
        .bsctim_val =       (REG_I2C_TIM_DIV2| REG_I2C_TIM_P0),
        .bscclken_val =     (REG_I2C_CLKEN_AUTO_SENSE_OFF | REG_I2C_CLKEN_M1 | REG_I2C_CLKEN_N1)
    },
};

/* ---- Internal variables ------- */
static struct ctl_table_header *i2c_sysCtlHeader;      /* sysctl table */
static struct bcm4760_i2c *i2c0 = NULL;
static struct bcm4760_i2c *i2c1 = NULL;
static unsigned int i2c_errors = 0;
static unsigned int i2c_max_irq_wait = 0;
static unsigned int i2c_max_cmdbusy_cnt = 0;
static unsigned int i2c_max_cmdbusy_clk = 0;
static unsigned int i2c_bus_speed = 400;
static unsigned long i2c_bus_clk_period = 0; /* in micro seconds */

static int set_i2c_speed( struct bcm4760_i2c *i2c )
{
    int i;
    u32 temp;
    u8 input_clock;

    /*
     * Calculate the input reference clock value
     * based upon the divider value in FREQ_SELECT
     * for the appropriate I2C channel.
     */
    temp = readl(IO_ADDRESS(CMU_R_FREQ_SELECT_MEMADDR));
    if (0 == i2c->ctl_id)
    {
        temp &= CMU_F_CKG_I2C_0_DIV_MASK;
        temp >>= CMU_F_CKG_I2C_0_DIV_R;
    } 
    else if (1 == i2c->ctl_id)
    {
        temp &= CMU_F_CKG_I2C_1_DIV_MASK;
        temp >>= CMU_F_CKG_I2C_1_DIV_R;
    }
    I2C_DEBUG(DBG_INFO, (KERN_INFO PFX "temp=%d\n", temp));
    if (temp == 1)
        input_clock = 12;       /* PLL divider is by 2 so I2C clock is 12 Mhz */
    else
        input_clock = 24;       /* PLL divider is by 1 so I2C clock is 24 Mhz */ 
    I2C_DEBUG(DBG_INFO, (KERN_INFO PFX "temp=%d, input_clock=%d\n", temp, input_clock));

    /*
     * Scan the table to find the best match.
     */
    for (i=0 ; i<(sizeof(i2c_speed_list)/sizeof(struct bcm4760_i2c_speeds)) ; i++)
    {
        if ((i2c_speed_list[i].input_clock == input_clock) &&
            (i2c_speed_list[i].i2c_clock_req == i2c->speed))
        {
            /*
             * This is an exact match which is obviously the most desirable condition.
             */
            I2C_DEBUG(DBG_INFO, (KERN_INFO PFX "list_input_clock=%d, list_i2c_clock=%d, BSCTIM=%04X, BSCCLKEN=%04X\n",
                                 i2c_speed_list[i].input_clock,
                                 i2c_speed_list[i].i2c_clock_req,
                                 i2c_speed_list[i].bsctim_val,
                                 i2c_speed_list[i].bscclken_val));
            REG_I2C_TIM(i2c->base) &= ~(REG_I2C_TIM_DIVMSK | REG_I2C_TIM_PMSK);
            REG_I2C_TIM(i2c->base) |= i2c_speed_list[i].bsctim_val;
            REG_I2C_CLKEN(i2c->base) &= ~(REG_I2C_CLKEN_MMSK | REG_I2C_CLKEN_NMSK);
            REG_I2C_CLKEN(i2c->base) |= i2c_speed_list[i].bscclken_val;
            return 0;
        }
        else if (i2c_speed_list[i].input_clock > input_clock)
        {
            if ((i != 0) && (i2c_speed_list[i-1].input_clock != input_clock))
            {
                /*
                * The input reference clock we are programmed to 
                * does not match the the previous entry in the table
                * therefore we have no valid settings to use.
                * We return an error.
                */
                I2C_DEBUG(DBG_INFO, (KERN_INFO PFX"No valid entry in speed table found!\n"));
                return -EINVAL;
            }
            else
            {
                /*
                * The I2C clock requested exceeded the maximum allowed based upon
                * the current input reference clock. We therefore use the max
                * value for the previous reference clock entry. The table should
                * be ordered such that this is the maximum value for that input
                * reference clock.
                */
                I2C_DEBUG(DBG_INFO, (KERN_INFO PFX "list_input_clock=%d, list_i2c_clock=%d, BSCTIM=%04X, BSCCLKEN=%04X\n",
                                     i2c_speed_list[i].input_clock,
                                     i2c_speed_list[i].i2c_clock_req,
                                     i2c_speed_list[i].bsctim_val,
                                     i2c_speed_list[i].bscclken_val));
                REG_I2C_TIM(i2c->base) &= ~(REG_I2C_TIM_DIVMSK | REG_I2C_TIM_PMSK);
                REG_I2C_TIM(i2c->base) |= i2c_speed_list[i-1].bsctim_val;
                REG_I2C_CLKEN(i2c->base) &= ~(REG_I2C_CLKEN_MMSK | REG_I2C_CLKEN_NMSK);
                REG_I2C_CLKEN(i2c->base) |= i2c_speed_list[i-1].bscclken_val;
            return 0;
            }
        }
    }

    if (i2c_speed_list[i-1].input_clock != input_clock)
    {
        /*
         * We ran off the end of the table but the reference
         * clock in the final entry does not match the
         * reference clock we are currently programmed to in
         * the FREQ_SELECT register therefore we have no
         * valid settings to use. We return an error.
         */
        I2C_DEBUG(DBG_INFO, (KERN_INFO PFX "No valid entry in speed table found!\n"));
        return -EINVAL;
    }
    else
    {
        /*
        * We ran off the end of the table. Thus we have verified that
        * the reference clock in the last entry of the table matches
        * the reference clock we are set at and by using the final
        * entry in the table we are programming the fastest I2C
        * speed we can achieve.
        */
        I2C_DEBUG(DBG_INFO, (KERN_INFO PFX "list_input_clock=%d, list_i2c_clock=%d, BSCTIM=%04X, BSCCLKEN=%04X\n",
                             i2c_speed_list[i-1].input_clock,
                             i2c_speed_list[i-1].i2c_clock_req,
                             i2c_speed_list[i-1].bsctim_val,
                             i2c_speed_list[i-1].bscclken_val));
        REG_I2C_TIM(i2c->base) &= ~(REG_I2C_TIM_DIVMSK | REG_I2C_TIM_PMSK);
        REG_I2C_TIM(i2c->base) |= i2c_speed_list[i-1].bsctim_val;
        REG_I2C_CLKEN(i2c->base) &= ~(REG_I2C_CLKEN_MMSK | REG_I2C_CLKEN_NMSK);
        REG_I2C_CLKEN(i2c->base) |= i2c_speed_list[i-1].bscclken_val;
    }
    return 0;
}

static int proc_i2c_speed(ctl_table *table, int write, struct file *filp,
                          void __user *buffer, size_t *lenp, loff_t *ppos )
{
    int rc;

    struct bcm4760_i2c *i2c = (0 == strcmp(table->procname, "i2c0_speed")) ? i2c0 : i2c1;

    if (!i2c)
        return -EINVAL;

    if (write)
    {
        /* get value from buffer */
        rc = proc_dointvec(table, write, filp, buffer, lenp, ppos );

        if (!rc)
        {
            /* check for valid speed */

            if (i2c_bus_speed >= 100 && i2c_bus_speed <= 1300)
            {
                i2c->speed = i2c_bus_speed;         /* set new speed */
                bcm4760_i2c_enable_controller(i2c); /* reprogram with new speed */
             }
            else
            {
                rc = -EINVAL;
            }
        }
    }
    else
    {
        i2c_bus_speed = i2c->speed;
        rc = proc_dointvec(table, write, filp, buffer, lenp, ppos);
    }

    return rc;
}

static struct ctl_table gSysCtlLocal[] =
{
    {
        .ctl_name      = 1,
        .procname      = "level",
        .data          = &gLevel,
        .maxlen        = sizeof( int ),
        .mode          = 0644,
        .proc_handler  = &proc_dointvec
    },
    {
        .ctl_name      = 2,
        .procname      = "errors",
        .data          = &i2c_errors,
        .maxlen        = sizeof( int ),
        .mode          = 0644,
        .proc_handler  = &proc_dointvec
    },
    {
        .ctl_name      = 3,
        .procname      = "max_irq_wait",
        .data          = &i2c_max_irq_wait,
        .maxlen        = sizeof( int ),
        .mode          = 0644,
        .proc_handler  = &proc_dointvec
    },
    {
        .ctl_name      = 4,
        .procname      = "max_cmdbusy_cnt",
        .data          = &i2c_max_cmdbusy_cnt,
        .maxlen        = sizeof( int ),
        .mode          = 0644,
        .proc_handler  = &proc_dointvec
    },
    {
        .ctl_name      = 5,
        .procname      = "max_cmdbusy_clk",
        .data          = &i2c_max_cmdbusy_clk,
        .maxlen        = sizeof( int ),
        .mode          = 0644,
        .proc_handler  = &proc_dointvec
    },
    {
        .ctl_name      = 6,
        .procname      = "i2c0_speed",
        .data          = &i2c_bus_speed,
        .maxlen        = sizeof( int ),
        .mode          = 0644,
        .proc_handler  = &proc_i2c_speed
    },
    {
        .ctl_name      = 7,
        .procname      = "i2c1_speed",
        .data          = &i2c_bus_speed,
        .maxlen        = sizeof( int ),
        .mode          = 0644,
        .proc_handler  = &proc_i2c_speed
    },
    {}
};

static struct ctl_table gSysCtl[] =
{
    {
        .ctl_name      = CTL_BCM_I2C,
        .procname      = "i2c",
        .mode          = 0555,
        .child         = gSysCtlLocal
    },
    {}
};

/*
 * Turn-around delay (2 bus clocks) after write/read needed for BCM4760 BSC.
 */
#define TURN_AROUND_DELAY	(i2c_bus_clk_period * 2)

/*
 * Compute the clock period required for the delay between i2c transactions
 */
static unsigned long bcm4760_i2c_bus_clk_period(struct bcm4760_i2c *i2c) 
{
    u8 tim, div, p;

    tim = REG_I2C_TIM(i2c->base);
    div = tim & BSC_TIM_DIV_MASK;
    p = (tim & BSC_TIM_P_VAL_MASK) >> BSC_TIM_P_VAL_BIT_SHIFT;

    I2C_DEBUG(DBG_INFO, (KERN_INFO PFX " tim[%d]\t div[%d]\t p[%d]\n", tim, div, p));

    div = 1 << (4 - div);
    p = ((2 * (p + 1)) + 1) + ((2 * (p + 1)) + 2);

    return ((div * p * 1000000 + (BSC_MASTER_CLK_FREQ >> 1) - 1) / BSC_MASTER_CLK_FREQ);
}

/*
 * I2C controller reset
 */
static void i2c_softreset(struct bcm4760_i2c *i2c)
{
    u32 reset_mask, isr, reg;
    u32 cmu_reg = readl(IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR));

    /* Unlock the CMU */
    writel(0xbcbc4760, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));

    /* i2c reset is at CMU block reset register at offset 0xA0 */
    if (0 == i2c->ctl_id)
        reset_mask = (1 << CMU_F_CMU_I2C_0_PRESETN_R);
    else
        reset_mask = (1 << CMU_F_CMU_I2C_1_PRESETN_R);

    reg = cmu_reg & ~reset_mask;
    writel(reg, IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR)); // write 0
    udelay(10);
    reg = cmu_reg | reset_mask;
    writel(reg, IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR)); // write 1
    /* Lock the CMU */
    writel(0, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));

    isr = REG_I2C_ISR(i2c->base) & (REG_I2C_ISR_CMD_BUSY | 
                                    REG_I2C_ISR_SES_DONE |
                                    REG_I2C_ISR_I2CERR |
                                    REG_I2C_ISR_NOACK);          
    if (0 != isr)
    {
         I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "ISR= 0x%02X not clear after soft-reset\n",isr));
    }
}

static u64 get_clk_counter(void)
{
    u64 clk;

	/* Must read both low & high of ripple counter for valid count */
    clk = REG_SMT_CLK_L;
    clk |= ((u64) REG_SMT_CLK_H) << 32;
    return clk;
}

static int bcm4760_check_cmdbusy(struct bcm4760_i2c *i2c)
{
    int i = 0;
    u64 clk = get_clk_counter();
    u64 clk2;

    while(REG_I2C_ISR(i2c->base) & REG_I2C_ISR_CMD_BUSY)		//wait for I2C CMD not busy
    {
        if (i > 100000)
        {
            i2c_errors++;
            return -ETIMEDOUT;
        }
        i++;
    }

    clk2 = get_clk_counter();
    clk = clk2 - clk;
    if (i > i2c_max_cmdbusy_cnt)
    {
        i2c_max_cmdbusy_cnt = i;
    }
    if (clk > i2c_max_cmdbusy_clk)
    {
        i2c_max_cmdbusy_clk = clk;
    }

    return 0;
}

static int bcm4760_check_rdystatus(struct bcm4760_i2c *i2c)
{
    int i = 0;
    u64 clk = get_clk_counter();
    u64 clk2;

    while((REG_I2C_CS(i2c->base) & (REG_I2C_CS_RDY | REG_I2C_CS_BUSY)) !=
          REG_I2C_CS_RDY)
    {
        if (i > 100000)
        {
            i2c_errors++;
            return -ETIMEDOUT;
        }
        i++;
    }

    clk2 = get_clk_counter();
    clk = clk2 - clk;

    return 0;
}

#if !I2C_POLL_COMMAND_DONE


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t bcm4760_i2c_isr(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t bcm4760_i2c_isr(int irq, void *dev_id)
#endif
{
    struct bcm4760_i2c *i2c = dev_id;

    if (i2c && REG_I2C_ISR(i2c->base))
    {
        u8 isr;

        /* Read again to allow register to stabilise */
        isr = REG_I2C_ISR(i2c->base);
        i2c->interrupt |= isr;
        REG_I2C_ISR(i2c->base) = isr;       /* clear interrupts */
        wake_up_interruptible(&i2c->queue);
    }

    return IRQ_HANDLED;
}

static int bcm4760_wait_interrupt( struct bcm4760_i2c *i2c )
{
    int res = 0;
    u64 clk;
    u64 time, timeout = HZ;

    clk = get_clk_counter();
    time = get_jiffies_64();
    do
    {
        res = wait_event_interruptible_timeout(i2c->queue, (i2c->interrupt), timeout);
        if (res < 0)
        {
            u64 now = get_jiffies_64();

            if (now - time < timeout)
            {
                timeout -= now - time ;
                time = now;
            }
            else
            {
                timeout = 0;	
            }
            printk(KERN_INFO PFX "Interupted by signal, ignoring\n");
        }
    }
    while (res < 0);

    clk = get_clk_counter() - clk;
    if (clk > i2c_max_irq_wait)
    {
        i2c_max_irq_wait = clk;
    }

    
   /* Note that there CAN be a race between the wakeup from  the ISR
    * and the timeoutif the device is busy. In that case both res == 0
    * and i2c->interrupt != 0.
    * So need to check whether there was an interrupt after all.
    */
    if (i2c->interrupt)
    {
        u32 irq_mask = (1 == i2c->ctl_id) ? BCM4760_INTR_I2C1 :  BCM4760_INTR_I2C0;
        disable_irq(irq_mask);
        res = i2c->interrupt;
        i2c->interrupt = 0;
        enable_irq(irq_mask);
        I2C_DEBUG(DBG_TRACE, (KERN_INFO PFX "Interrupt condition = %04X\n", res));
    }
    else
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "wait timed out, clk=%lld, isr=0x%02X, cs=0x%02X\n",
                              clk,
                              REG_I2C_ISR(i2c->base),
                              REG_I2C_CS(i2c->base)));

        if (REG_I2C_ISR(i2c->base) & REG_I2C_ISR_CMD_BUSY) {
            /* Try to recover from this command hung */
            bcm4760_i2c_enable_controller(i2c);     /* reset the controller */
        }

        i2c_errors++;
        res = -ETIMEDOUT;
    }

    return res;
}

#else

/* Wait for I2C Controller Interrupt */
static int bcm4760_wait_interrupt(  struct bcm4760_i2c *i2c  )
{

    unsigned int i = 0;
    u64 clk;
    u64 clk2;
    int isr;

    clk = get_clk_counter();

    while( ( REG_I2C_ISR(i2c->base) & REG_I2C_ISR_SES_DONE ) == 0 )
    {
        i++;
        if (i == (unsigned int)(-1))
        {
            I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "wait timed out, clk=%lld, isr=0x%02X, cs=0x%02X\n",
                                  clk,
                                  REG_I2C_ISR(i2c->base),
                                  REG_I2C_CS(i2c->base)));
            return -ETIMEDOUT;
        }
    }

    clk2 = get_clk_counter();
    clk = clk2 - clk;
    if (clk > i2c_max_irq_wait)
    {
        i2c_max_irq_wait = clk;
    }

    isr = REG_I2C_ISR(i2c->base);
    REG_I2C_ISR(i2c->base) = isr;       /* clear interrupts */
    return isr;
}
#endif

static int bcm4760_wait_sesdone(struct bcm4760_i2c *i2c)
{
    int ret;
    u8 isr;
    u8 cs;

    do
    {
        ret = bcm4760_wait_interrupt(i2c);
        cs = REG_I2C_CS(i2c->base);

        if(ret & REG_I2C_ISR_SES_DONE)
        {
            break;
        }
    } while(ret > 0);

    if (ret < 0)
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "wait interrupt timed out\n"));
        i2c_errors++;
        return ret;
    }

    /* Get status from interrupt status register */

    isr = ret;
    if( isr & REG_I2C_ISR_I2CERR )
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "bus error detected\n"));
        i2c_errors++;
        return -EIO;
    }

    if( isr & REG_I2C_ISR_NOACK )
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "no ack\n"));
        i2c_errors++;
    }

    if( !(isr & REG_I2C_ISR_SES_DONE) )
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "ses done timed out\n"));
        i2c_errors++;
        return -EIO;
    }

    /*
     * If ACK bit in CCS is zero then the device ACKed.
     */

    return (cs & REG_I2C_CS_ACK)?0:1;
}

static int bcm4760_i2c_start( struct bcm4760_i2c *i2c )
{
    int try_cnt;
    I2C_DEBUG(DBG_TRACE, ("\n"));

    try_cnt = 2;
    while (try_cnt)
    {
        try_cnt--;

        /* Wait for controller to be done with last command */
        if ( bcm4760_check_cmdbusy(i2c) < 0 )
        {
            I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "start: cmdbusy always high, ISR = 0x%x\n",
                                  REG_I2C_ISR(i2c->base)));
            return -EIO;
        }

        if(bcm4760_check_rdystatus(i2c) < 0)
        {
            I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "start: never ready for command\n"));
            return -EIO;
        }

        /* Send normal start condition */
        REG_I2C_CS(i2c->base) = ((REG_I2C_CS(i2c->base) &
            (~(REG_I2C_CS_CMDMASK | REG_I2C_CS_ACK))) |
            REG_I2C_CS_CMDSTART);

        /* Wait for command to be done */
        if ( bcm4760_wait_sesdone(i2c) < 0)
        {
            I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "start: sesdone timed out\n"));
            if (!try_cnt)
                return -ETIMEDOUT;
        }
        else
            try_cnt = 0;
    }

    /* a small note below the register description says to set cmd back to 0 after a start or stop. */
    REG_I2C_CS(i2c->base) = REG_I2C_CS(i2c->base) & ~REG_I2C_CS_CMDMASK;

    return 0;
}

static int bcm4760_i2c_repstart(  struct bcm4760_i2c *i2c )
{
    I2C_DEBUG(DBG_TRACE, ("\n"));

    /* Wait for controller to be done with last command */
    if ( bcm4760_check_cmdbusy(i2c) < 0 )
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "repstart: cmdbusy always high\n"));
        return -EIO;
    }

    if(bcm4760_check_rdystatus(i2c) < 0)
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "repstart: never ready for command\n"));
        return -EIO;
    }

    /* Send repeated start condition */
    REG_I2C_CS(i2c->base) = ((REG_I2C_CS(i2c->base) &
        (~(REG_I2C_CS_CMDMASK | REG_I2C_CS_ACK))) |
        REG_I2C_CS_CMDRESTART);

    /* Wait for command to be done */
    if ( bcm4760_wait_sesdone(i2c) < 1)
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "repstart: sesdone timed out\n"));
        return -ETIMEDOUT;
    }

    /* a small note below the register description says to set cmd back to 0 after a start or stop. */
    REG_I2C_CS(i2c->base) = REG_I2C_CS(i2c->base) & ~REG_I2C_CS_CMDMASK;

    return 0;
}

static int bcm4760_i2c_stop( struct bcm4760_i2c *i2c )
{

    I2C_DEBUG(DBG_TRACE, ("\n"));

    /* Wait for controller to be done with last command */
    if ( bcm4760_check_cmdbusy(i2c) < 0 )
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "stop: cmdbusy always high\n"));
        return -EIO;
    }

    if(bcm4760_check_rdystatus(i2c) < 0)
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "stop: never ready for command\n"));
        return -EIO;
    }

    /* Send stop condition */
    REG_I2C_CS(i2c->base) = ((REG_I2C_CS(i2c->base) &
        (~(REG_I2C_CS_CMDMASK | REG_I2C_CS_ACK))) |
        REG_I2C_CS_CMDSTOP);

    /* Wait for command to be done */
    if ( bcm4760_wait_sesdone(i2c) < 0)
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "stop: sesdone timed out, cs = 0x%02x\n",
                              REG_I2C_CS(i2c->base)));
        return -ETIMEDOUT;
    }

    /* there must be a minimum 1.3us between a stop and start on fullspeed devices. */
    udelay(2);

    /* a small note below the register description says to set cmd back to 0 after a start or stop. */
    REG_I2C_CS(i2c->base) = REG_I2C_CS(i2c->base) & ~REG_I2C_CS_CMDMASK;

    return 0;
}

static void bcm4760_i2c_terminate(struct bcm4760_i2c *i2c)
{
    if(bcm4760_i2c_stop(i2c) < 0)
    {
	I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "resetting device.\n"));
        bcm4760_i2c_enable_controller(i2c);     /* reset the controller */

        if(bcm4760_i2c_stop(i2c) < 0)
            return;
    }

    return;
}

/* send a byte without start cond., look for arbitration,
 * check ackn. from slave
 *
 * returns:
 *      1 if the device acknowledged
 *      0 if the device did not ack
 *      -ETIMEDOUT if an error occurred (while raising the scl line)
 */
static int bcm4760_i2c_outb(struct bcm4760_i2c *i2c, char c)
{
    int ack;

    I2C_DEBUG(DBG_TRACE, ("0x%2x\n", c));

    /* Wait for controller to be done with last command */

    if(bcm4760_check_cmdbusy(i2c) < 0)
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "outb: cmdbusy always high\n"));
        return -EIO;
    }

    if(bcm4760_check_rdystatus(i2c) < 0)
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "outb: never ready for next write command\n"));
        return -EIO;
    }

    /* Send data */
    if(REG_I2C_CS(i2c->base) & REG_I2C_CS_CMDMASK)
        REG_I2C_CS(i2c->base) &=  (~(REG_I2C_CS_CMDMASK | REG_I2C_CS_ACK));

    REG_I2C_DAT(i2c->base) = (u8)c;

    /* Wait for command to be done */
    if ( (ack = bcm4760_wait_sesdone(i2c)) < 0)
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "outb: sesdone timed out\n"));
        return -ETIMEDOUT;
    }

    return ack;
}

static int bcm4760_i2c_inb(struct bcm4760_i2c *i2c, int no_ack)
{
    u8 data;

    I2C_DEBUG(DBG_TRACE, ("%d\n", no_ack));

    /* Wait for controller to be done with last command */
    if ( bcm4760_check_cmdbusy(i2c) < 0 )
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "inb: cmdbusy always high\n"));
        return -EIO;
    }

    if(bcm4760_check_rdystatus(i2c) < 0)
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "inb: never ready for next read command\n"));
        return -EIO;
    }

    /* Initiate data read with ACK low */
    REG_I2C_CS(i2c->base) = ((REG_I2C_CS(i2c->base) &
        (~(REG_I2C_CS_CMDMASK | REG_I2C_CS_ACK))) |
        (REG_I2C_CS_CMDREAD | (no_ack?REG_I2C_CS_ACK:0)));

    /* Wait for command to be done */
    if (bcm4760_wait_sesdone(i2c) < 0)
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "inb: sesdone timed out, cs = 0x%02x\n",
                              REG_I2C_CS(i2c->base)));
        return -ETIMEDOUT;
    }

    /* Read data */
    data = REG_I2C_DAT(i2c->base);

    return (int)data;
}

/* try_address tries to contact a chip for a number of
 * times before it gives up.
 * return values:
 *      1 chip answered
 *      0 chip did not answer
 *      -x transmission error
 */
static int bcm4760_try_address(struct i2c_adapter *i2c_adap,
                               unsigned char addr, int retries)
{
    int i,ret = -1;
    struct bcm4760_i2c *i2c = i2c_get_adapdata(i2c_adap);

    I2C_DEBUG(DBG_TRACE, ("0x%02x, %d\n", addr, retries));

    for (i=0;i<=retries;i++)
    {
        ret = bcm4760_i2c_outb(i2c, addr);
        if (ret==1)
            break;          /* success! */
        bcm4760_i2c_stop(i2c);
        if (i==retries)     /* no success */
            break;
        bcm4760_i2c_start(i2c);
    }

    if (i)
    {
        I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX"Used %d tries to %s client at 0x%02x : %s\n",
                             i+1,
                             addr & 1 ? "read" : "write",
                             addr>>1,
                             ret==1 ? "success" : ret==0 ? "no ack" : "failed, timeout?"));
    }

    return ret;
}

static int bcm4760_sendbytes(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
    char c;
    const char *temp = msg->buf;
    int count = msg->len;
    unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;
    int retval;
    int wrcount=0;
    struct bcm4760_i2c *i2c = i2c_get_adapdata(i2c_adap);

    while (count > 0)
    {
        c = *temp;
        I2C_DEBUG(DBG_TRACE2, (KERN_INFO PFX "writing %2.2X\n", c&0xff));
        retval = bcm4760_i2c_outb(i2c, c);
        if ((retval>0) || (nak_ok && (retval==0)))
        {

            /* ok or ignored NAK */
            count--;
            temp++;
            wrcount++;
        }
        else
        {

            /* arbitration or no acknowledge */
            I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "error %d/%d.\n",
                                  wrcount,
                                  msg->len));
            i2c_errors++;
            bcm4760_i2c_stop(i2c);
            return (retval<0)? retval : -EFAULT;    /* got a better one ?? */
        }
    }

    return wrcount;
}

static int bcm4760_readbytes(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
    int inval;
    int rdcount=0;              /* counts bytes read */
    char *temp = msg->buf;
    int count = msg->len;
    struct bcm4760_i2c *i2c = i2c_get_adapdata(i2c_adap);

    while (count > 0)
    {
        inval = bcm4760_i2c_inb(i2c, (msg->flags & I2C_M_NO_RD_ACK) || (count == 1));
        if (inval>=0)
        {
            I2C_DEBUG(DBG_TRACE2, (KERN_INFO PFX "reading %2.2X\n",
                                   inval&0xff));
            *temp = inval;
            rdcount++;
        }
        else
        {
            /* read timed out */
            i2c_errors++;
            I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "timed out.\n"));
            break;
        }

        temp++;
        count--;
    }

    return rdcount;
}

/* doAddress initiates the transfer by generating the start condition (in
 * try_address) and transmits the address in the necessary format to handle
 * reads, writes as well as 10bit-addresses.
 *
 * returns:
 *      0 everything went okay, the chip ack'ed, or IGNORE_NAK flag was set
 *      -x an error occurred (like: -EREMOTEIO if the device did not answer, or
 *      -ETIMEDOUT, for example if the lines are stuck...)
 */
static int bcm4760_doAddress(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
    unsigned short flags = msg->flags;
    unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;

    unsigned char addr;
    int ret, retries;
    struct bcm4760_i2c *i2c = i2c_get_adapdata(i2c_adap);

    I2C_DEBUG(DBG_TRACE, (KERN_INFO PFX "\n"));

    retries = nak_ok ? 0 : i2c_adap->retries;

    if (flags & I2C_M_TEN)
    {

        /* a ten bit address */

        addr = 0xf0 | (( msg->addr >> 7) & 0x03);
        I2C_DEBUG(DBG_TRACE2, (KERN_INFO PFX "addr: %d\n", addr));

        /* try extended address code...*/
        ret = bcm4760_try_address(i2c_adap, addr, retries);
        if ((ret != 1) && !nak_ok) 
        {
            I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "died at extended address code.\n"));
            return -EREMOTEIO;
        }

        /* the remaining 8 bit address */
        ret = bcm4760_i2c_outb(i2c, (msg->addr & 0x7f));
        if (((ret == 0) && !nak_ok) || (ret < 0))
        {

            /* the chip did not ack / xmission error occurred */
            I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "died at 2nd address code.\n"));
            return -EREMOTEIO;
        }

        if ( flags & I2C_M_RD )
        {
            ret = bcm4760_i2c_repstart(i2c);
            if (ret < 0)
                return -EIO;

            /* okay, now switch into reading mode */

            addr |= 0x01;
            ret = bcm4760_try_address(i2c_adap, addr, retries);
            if ((ret!=1) && !nak_ok)
            {
                I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "died at extended address code.\n"));
                return -EREMOTEIO;
            }
        }
    }
    else
    {

        /* normal 7bit address	*/
        addr = ( msg->addr << 1 );
        if (flags & I2C_M_RD )
            addr |= 1;
        if (flags & I2C_M_REV_DIR_ADDR )
            addr ^= 1;
        ret = bcm4760_try_address(i2c_adap, addr, retries);
        if (((ret == 0) && !nak_ok) || (ret < 0))
        {
            I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "NAK from device addr %2.2x\n",
                                 addr));
            i2c_errors++;
            return -EREMOTEIO;
        }
    }

    return 0;
}

/*
 * Master transfer function.
 */
static int bcm4760_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[], int num)
{
    /* Based on i2c-algo-bit */
    struct i2c_msg *pmsg;
    int i,ret;
    unsigned short nak_ok;
    struct bcm4760_i2c *i2c = i2c_get_adapdata(i2c_adap);

    if (i2c->suspended)
        return -EIO;

    I2C_DEBUG(DBG_TRACE, (KERN_INFO PFX "\n"));

    /* Send first start */
    ret = bcm4760_i2c_start(i2c);
    if (ret < 0)
    {
        bcm4760_i2c_terminate(i2c);
        return ret;
    }

    /* Loop through all messages */
    for (i = 0; i < num; i++)
    {
        pmsg = &msgs[i];
        nak_ok = pmsg->flags & I2C_M_IGNORE_NAK;

        I2C_DEBUG(DBG_INFO, (KERN_INFO PFX "%d byte %s message on device address %02X.\n",
                             pmsg->len,
                             (pmsg->flags & I2C_M_RD) ? "read" : "write",
                             pmsg->addr));

        if (!(pmsg->flags & I2C_M_NOSTART))
        {
            /* Send repeated start only on subsequent messages */
            if (i)
            {
                if(i2c->speed == 400)
                    udelay(75);
                ret = bcm4760_i2c_repstart(i2c);
                if (ret < 0)
                {
                    bcm4760_i2c_terminate(i2c);
                    return ret;
                }
            }
            ret = bcm4760_doAddress(i2c_adap, pmsg);
            if (ret < 0)
            {
                bcm4760_i2c_terminate(i2c);
                I2C_DEBUG(DBG_ERROR, (KERN_ERR PFX "error on device addr %2.2x msg #%d\n",
                                     msgs[i].addr,
                                     i));
                return (ret<0) ? ret : -EREMOTEIO;
            }
        }

        if (pmsg->flags & I2C_M_RD )
        {
            /* read bytes into buffer*/

            ret = bcm4760_readbytes(i2c_adap, pmsg);
            I2C_DEBUG(DBG_INFO, (KERN_INFO PFX "read %d bytes.\n", ret));
            if (ret < pmsg->len )
            {
                bcm4760_i2c_terminate(i2c);
                return (ret<0)? ret : -EREMOTEIO;
            }
        }
        else
        {
            /* write bytes from buffer */

            ret = bcm4760_sendbytes(i2c_adap, pmsg);
            I2C_DEBUG(DBG_INFO, (KERN_INFO PFX "wrote %d bytes.\n", ret));
            if (ret < pmsg->len )
            {
                bcm4760_i2c_terminate(i2c);
                return (ret<0) ? ret : -EREMOTEIO;
            }
        }
    }
    ret = bcm4760_i2c_stop(i2c);

    return (ret < 0) ? ret : num;
}

/*
 * Controller initialization function
 */
static void bcm4760_i2c_enable_controller(struct bcm4760_i2c *i2c)
{
    i2c_softreset(i2c);
    REG_I2C_IER(i2c->base) = 0;                                 /* disable interrupts */
    REG_I2C_CS(i2c->base) = REG_I2C_CS_SDA | REG_I2C_CS_SCL;    /* reset controller (EN=0) */

    (void)set_i2c_speed(i2c);
    REG_I2C_CLKEN(i2c->base) |= REG_I2C_CLKEN_CLKEN;               /* enable clock */
    REG_I2C_FCR(i2c->base) = REG_I2C_FCR_FLUSH;                   /* flush fifo */

    /* Clear interrupts */
    REG_I2C_ISR(i2c->base) = REG_I2C_ISR_CMD_BUSY | 
                             REG_I2C_ISR_SES_DONE |
                             REG_I2C_ISR_I2CERR |
                             REG_I2C_ISR_TXFIFOEMPTY |
                             REG_I2C_ISR_NOACK;
    REG_I2C_IER(i2c->base) = REG_I2C_IER_INT_EN |
                             REG_I2C_IER_ERRINT_EN |
                             REG_I2C_IER_NOACK_EN;                /* enable interrupts */

    REG_I2C_TOUT(i2c->base) = 0;                                  /* disable timeout */
    REG_I2C_CS(i2c->base) = REG_I2C_CS_SDA | REG_I2C_CS_SCL | REG_I2C_CS_EN;  /* enable controller */
}

/*
 * BCM4760 I2C adaptor and algorithm definitions
 */
static u32 bcm4760_functionality(struct i2c_adapter *adap)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL |
           I2C_FUNC_10BIT_ADDR | I2C_FUNC_PROTOCOL_MANGLING;
}

static struct i2c_algorithm bcm4760_algo =
{
    .master_xfer = bcm4760_xfer,
    .functionality = bcm4760_functionality,
};

static struct i2c_adapter bcm4760_0_ops =
{
    .owner = THIS_MODULE,
	.name 	 = "bcm4760-i2c0",
    .id = I2C_HW_B_BCM4760,
    .algo = &bcm4760_algo,
    .timeout = 1,
    .retries = 5
};

static struct i2c_adapter bcm4760_1_ops =
{
    .owner = THIS_MODULE,
	.name 	 = "bcm4760-i2c1",
    .id = I2C_HW_B_BCM4760_1,
    .algo = &bcm4760_algo,
    .timeout = 1,
    .retries = 5
};

/*
 * I2C Adapter driver initialization function.
 *
 * This function is called when the I2C adapter is probed. It will
 * initialize the controller hardware and perform various device
 * initialization and registration functions.
 */
static int i2c_ctl_init(int i2c_ctl_id, struct bcm4760_i2c **i2c_handle)
{
    int rc = 0;
    struct bcm4760_i2c *i2c = *i2c_handle;

    I2C_DEBUG(DBG_TRACE, ("\n"));

    if (i2c)
    {
        printk( KERN_INFO "BCM4760 i2c adapter module already initialized.\n" );
        return 0;
    }

    printk( KERN_INFO "BCM4760 i2c adapter module\n" );

    /* Allocate memory for control structure */
    if (!(i2c = kmalloc(sizeof(*i2c), GFP_KERNEL)))
    {
        return -ENOMEM;
    }
    memset(i2c, 0, sizeof(*i2c));
    i2c->ctl_id = i2c_ctl_id;
    i2c->speed = i2c_bus_speed; // set to default bus speed

    if (0 == i2c_ctl_id)
    {
        i2c->base = I2C0_REG_BASE_ADDR;
        i2c->adap = bcm4760_0_ops;
        i2c0 = i2c;
    }
    else
    {
        i2c->base = I2C1_REG_BASE_ADDR;
        i2c->adap = bcm4760_1_ops;
        i2c1 = i2c;
    }

#if !I2C_POLL_COMMAND_DONE
    /* Initialize IRQ and wait queue */

    init_waitqueue_head(&i2c->queue);
    if (0 == i2c_ctl_id)
    {
        rc = request_irq(BCM4760_INTR_I2C0, bcm4760_i2c_isr,
                         IRQF_DISABLED,
                         "bcm4760-i2c0", i2c);
    }
    else
    {
        rc = request_irq(BCM4760_INTR_I2C1, bcm4760_i2c_isr,
                         IRQF_DISABLED,
                         "bcm4760-i2c1", i2c);
    }

    if (rc < 0)
    {
        printk( KERN_ERR "i2c-bcm4760%d: %s failed to attach interrupt, rc = %d\n", i2c_ctl_id, __FUNCTION__, rc );
        goto fail_irq;
    }
#endif

    /* Enable controller */
    bcm4760_i2c_enable_controller(i2c);

    i2c_bus_clk_period = bcm4760_i2c_bus_clk_period(i2c);

    /* Add I2C adaptor */
    i2c_set_adapdata(&i2c->adap, i2c);
    i2c->adap.nr = i2c->ctl_id;
    if ((rc = i2c_add_numbered_adapter(&i2c->adap)) < 0)
    {
        printk( KERN_ERR "i2c-bcm4760: %s failed to add adapter, rc = %d\n", __FUNCTION__, rc );
        goto fail_add;
    }

    *i2c_handle = i2c;      /* returns handle */
    return rc;

fail_add:

#if !I2C_POLL_COMMAND_DONE
    free_irq((1 == i2c_ctl_id) ? BCM4760_INTR_I2C1 :  BCM4760_INTR_I2C0, NULL);
fail_irq:
#endif

    kfree(i2c);
    return rc;
}

/*
 * I2C Adapter Driver exit function
 *
 * This function is called when the I2S adapter driver is being removed.
 */
static void i2c_ctl_exit( struct bcm4760_i2c *i2c )
{
	if (!i2c) {
        return;
	}

    i2c_del_adapter(&i2c->adap);

#if !I2C_POLL_COMMAND_DONE
    free_irq((1 == i2c->ctl_id) ? BCM4760_INTR_I2C1 :  BCM4760_INTR_I2C0, NULL);
#endif

    kfree(i2c);
    printk( KERN_INFO "BCM4760 i2c adapter module exited\n" );
}

/*
 * Set I2C master clock
 */
static void i2c_set_mclk(void)
{
    u32 cmu_reg = readl(IO_ADDRESS(CMU_R_FREQ_SELECT_MEMADDR));

    /* Make sure the i2c_x_div (bits 12-17) are 0s for 24Mhz MCLK */
	cmu_reg &= ~(CMU_F_CKG_I2C_0_DIV_MASK | CMU_F_CKG_I2C_1_DIV_MASK);

#if (BSC_MASTER_CLK_FREQ == 24000000UL)
	cmu_reg &= 0xfffc0fff;	/* Make sure the i2c_x_div (bits 12-17) are 0s */
#elif (BSC_MASTER_CLK_FREQ == 12000000UL)
    cmu_reg &= 0xfffc0fff;      /* Make sure the i2c_x_div (bits 12-17) are 0s */

    /* div 1 for 12Mhz MCLK */
    cmu_reg |= ((1 << CMU_F_CKG_I2C_1_DIV_R) | (1 << CMU_F_CKG_I2C_0_DIV_R));          
#endif

    writel(cmu_reg, IO_ADDRESS(CMU_R_FREQ_SELECT_MEMADDR));
}


/* bcm4760_i2c_probe
 *
 * called by the bus driver when a suitable device is found
 */
static int bcm4760_i2c_probe(struct platform_device *pdev)
{
    struct bcm4760_i2c *i2c = NULL;
    struct resource *res;
    int ret = 0;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (res == NULL)
    {
        dev_err(&pdev->dev, "cannot find IO resource\n");
        ret = -ENOENT;
        goto err;
    }

    if (res->start == I2C0_REG_BASE_ADDR)
    {
        i2c_set_mclk();
        i2c_ctl_init(0, &i2c);
    }
    else if  (res->start == I2C1_REG_BASE_ADDR)
    {
        i2c_ctl_init(1, &i2c);
    }
    else
        goto err;

    platform_set_drvdata(pdev, i2c);

err:
    return ret;
}

/* bcm4760_i2c_remove
 *
 * called when device is removed from the bus
 */
static int bcm4760_i2c_remove(struct platform_device *pdev)
{
	struct bcm4760_i2c *i2c;

	i2c = platform_get_drvdata(pdev);
    i2c_ctl_exit(i2c);                  /* shutdown I2C controller */

    platform_set_drvdata(pdev, NULL);   /* mark driver private data as gone */

	return 0;
}

#ifdef CONFIG_PM
static int bcm4760_i2c_suspend(struct device *dev)
{
	struct platform_device *pdev;
	struct bcm4760_i2c *i2c;

	pdev = to_platform_device(dev);
	I2C_DEBUG(DBG_INFO, (KERN_INFO "Suspend processed for %s.\n", pdev->name));

	i2c = platform_get_drvdata(pdev);
	i2c->suspended = 1;

	return 0;
}

static int bcm4760_i2c_resume(struct device *dev)
{
    struct platform_device *pdev;
    struct bcm4760_i2c *i2c;

    pdev = to_platform_device(dev);
    I2C_DEBUG(DBG_INFO, (KERN_INFO "Resume processed for %s.\n", pdev->name));

    i2c = platform_get_drvdata(pdev);

    i2c->suspended = 0;
    if (i2c->ctl_id == 0)
    {
        i2c_set_mclk();
    }
    bcm4760_i2c_enable_controller(i2c);

    return 0;
}
#endif

#ifdef CONFIG_PM
static struct pm_ext_ops bcm4760_pm_ops = {
    .base =
    {
        .suspend    = bcm4760_i2c_suspend,
        .resume     = bcm4760_i2c_resume
    }
};
#endif

/* device driver for platform bus bits */

static struct platform_driver bcm4760_i2c0_driver = {
    .probe          = bcm4760_i2c_probe,
    .remove         = bcm4760_i2c_remove,
#ifdef CONFIG_PM
    .pm             = &bcm4760_pm_ops,
#endif
    .driver       =
    {
        .owner  = THIS_MODULE,
        .name   = "bcm4760-i2c-0"
    }
};

static struct platform_driver bcm4760_i2c1_driver = {
    .probe        = bcm4760_i2c_probe,
    .remove       = bcm4760_i2c_remove,
#ifdef CONFIG_PM
    .pm             = &bcm4760_pm_ops,
#endif
    .driver       = {
        .owner  = THIS_MODULE,
        .name   = "bcm4760-i2c-1"
    }
};


/*
 * Module initialization function
 */
static int __init i2c_bcm4760_init(void)
{
    int ret;

    ret = platform_driver_register(&bcm4760_i2c0_driver);
    if (ret == 0)
    {
        ret = platform_driver_register(&bcm4760_i2c1_driver);
        if (ret)
        {
            platform_driver_unregister(&bcm4760_i2c0_driver);
        }
    }

    i2c_sysCtlHeader = register_sysctl_table( gSysCtl );

    return 0;
}

/*
 * Module exit function
 */
static void i2c_bcm4760_exit( void )
{
    platform_driver_unregister(&bcm4760_i2c0_driver);
    platform_driver_unregister(&bcm4760_i2c1_driver);

    /* unregister sysctl table */
	if ( i2c_sysCtlHeader != NULL ) {
        unregister_sysctl_table( i2c_sysCtlHeader );
        i2c_sysCtlHeader = NULL;
    }
}

module_init(i2c_bcm4760_init);
module_exit(i2c_bcm4760_exit);

MODULE_AUTHOR("Broadcom Corp.");
MODULE_DESCRIPTION("I2C hardware driver for BCM4760");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.0");
MODULE_ALIAS("platform:bcm4760-i2c0");
MODULE_ALIAS("platform:bcm4760-i2c1");

