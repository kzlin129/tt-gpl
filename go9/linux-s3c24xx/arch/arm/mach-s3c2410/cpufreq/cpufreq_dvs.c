/*
 * cpufreq_dvs.c: CPU clock scaling for s3c24xx family
 *
 * Copyright (C) 2008, TomTom International B.V.
 *
 * Author: Rogier Stam <rogier.stam@tomtom.com> 
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <asm/hardware/clock.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/arch/regs-clock.h>
#include <barcelona/gopins.h>
#include "cpufreq_dvs.h"

static struct cpufreq_driver		s3c24xx_dvs_driver;
static struct cpufreq_frequency_table	s3c24xx_dvs_valid_freq[CPUFREQ_DVS_NR_FREQS+1];
static struct s3c24xx_dvs_handlers	s3c24xx_dvs_handlestate={NULL, NULL};

/* CPU Specific routines. */
/* Routines to get the state of the DVS bit. */
static int s3c2450_get_dvs_state( void )
{
	return readl( S3C2450_CLKDIV0 ) & S3C2450_CLKDIV0_DVS;
}

static int s3c2443_get_dvs_state( void )
{
	return readl( S3C2443_CLKDIV0 ) & S3C2443_CLKDIV0_DVS;
}

static int s3c2412_get_dvs_state( void )
{
	return readl( S3C2410_CLKDIVN ) & S3C2412_CLKDIVN_DVSEN;
}

static int s3c2440_get_dvs_state( void )
{
	return readl( S3C2440_CAMDIVN ) & S3C2440_CAMDIVN_DVSEN;
}

/* Routines to set the state of the DVS bit. */
static void s3c2450_set_dvs_state( int state )
{
	uint32_t	dvsreg=readl( S3C2450_CLKDIV0 );

	if( state ) dvsreg|=S3C2450_CLKDIV0_DVS;
	else dvsreg&=~S3C2450_CLKDIV0_DVS;
	writel( dvsreg, S3C2450_CLKDIV0 );
	return;
}

static void s3c2443_set_dvs_state( int state )
{
	uint32_t	dvsreg=readl( S3C2443_CLKDIV0 );

	if( state ) dvsreg|=S3C2443_CLKDIV0_DVS;
	else dvsreg&=~S3C2443_CLKDIV0_DVS;
	writel( dvsreg, S3C2443_CLKDIV0 );
	return;
}

static void s3c2412_set_dvs_state( int state )
{
	uint32_t	dvsreg=readl( S3C2410_CLKDIVN );

	if( state ) dvsreg|=S3C2412_CLKDIVN_DVSEN;
	else dvsreg&=~S3C2412_CLKDIVN_DVSEN;
	writel( dvsreg, S3C2410_CLKDIVN );
	return;
}

static void s3c2440_set_dvs_state( int state )
{
	uint32_t	dvsreg=readl( S3C2440_CAMDIVN );

	if( state ) dvsreg|=S3C2440_CAMDIVN_DVSEN;
	else dvsreg&=~S3C2440_CAMDIVN_DVSEN;
	writel( dvsreg, S3C2440_CAMDIVN );
	return;
}

/* Generic set/get DVS state routines. These call the above mentioned routines. */
static unsigned int s3c24xx_dvs_getspeed( unsigned int cpu )
{
	if( s3c24xx_dvs_handlestate.get( ) )
		return s3c24xx_dvs_valid_freq[0].frequency;
	else
		return s3c24xx_dvs_valid_freq[1].frequency;
}

static unsigned int s3c24xx_dvs_setspeed( unsigned int frequency )
{
	int	count=0;

	/* Search through the table for a matching frequency. */
	for( count=0; s3c24xx_dvs_valid_freq[count].frequency != CPUFREQ_TABLE_END; count++ )
	{
		if( s3c24xx_dvs_valid_freq[count].frequency == frequency )
		{
			if( count == 0 ) s3c24xx_dvs_handlestate.set( 1 );
			else s3c24xx_dvs_handlestate.set( 0 );
			return frequency;
		}
	}

	/* If we get here, we couldn't set the frequency. */
	return 0;
}

/* Generic routines that form the driver handlers. */
static int s3c24xx_dvs_target(struct cpufreq_policy *policy,
			      unsigned int target_freq,
			      unsigned int relation)
{
	struct clk		*fclk;
	struct cpufreq_freqs	freqs;
	unsigned int		curr_freq;

	/* Get the current frequency. */
	curr_freq=s3c24xx_dvs_getspeed( 0 );

	/* relation either is CPUFREQ_RELATION_L or CPUFREQ_RELATION_H. */
	/* The meaning is that if it's _L and not exactly either the min*/
	/* or the max frequency, it should choose the next frequency */
	/* above the target frequency, e.g policy->max. If it's _H it */
	/* works opposite: Choose the next frequency below the target */
	/* frequency, e.g policy->min. */
	if( (target_freq != policy->min) && (target_freq != policy->max) )
	{
		if( relation == CPUFREQ_RELATION_L )
			target_freq=policy->max;
		else
			target_freq=policy->min;
	}

	/* Check if we're actually changing something. If not, don't do anything. */
	if( target_freq == curr_freq )
		return 0;

	freqs.cpu = 0;
	freqs.old = curr_freq;
	freqs.new = target_freq;

	/* Notify all registered drivers of imminent change. */
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	fclk = clk_get( NULL, "fclk" );

	/* Set the new speed. */
	if( !s3c24xx_dvs_setspeed( target_freq ) )
	{
		/* We shouldn't get here. */
		printk( KERN_WARNING "WARNING! CPUFREQ (DVS) Could not set new frequency (%u)!\n",
			target_freq );
		return 0;
	}

	/* Make sure the clockrates are set correctly. */
	clk_set_rate(fclk, target_freq * 1000);
	clk_put(fclk);

	/* Notify all registered drivers of change occurred. */
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

static unsigned long int s3c24xx_dvs_get_nearest_freq( unsigned long int freq )
{
	int			count;
	int			lowdiff=-1;
	unsigned long int	lowest;
	unsigned long int	highest;
	unsigned long int	diff=0xFFFFFFFF;

	/* Now make sure that it gets as close as possible to the specified limits. */
	for( count=0; s3c24xx_dvs_valid_freq[count].frequency != CPUFREQ_TABLE_END; count++ )
	{
		if( s3c24xx_dvs_valid_freq[count].frequency > freq )
		{
			highest=s3c24xx_dvs_valid_freq[count].frequency;
			lowest=freq;
		}
		else
		{
			highest=freq;
			lowest=s3c24xx_dvs_valid_freq[count].frequency;
		}
		
		if( (highest-lowest) < diff )
		{
			diff=highest-lowest; 
			lowdiff=count;
		}
	}

	if( count == -1 )
		return 0;
	else
		return s3c24xx_dvs_valid_freq[lowdiff].frequency;
}

static int s3c24xx_dvs_verify_speed(struct cpufreq_policy *policy)
{
	int			retval;
	unsigned long int	nearest;

	if( policy->cpu != 0 )
		return -EINVAL;

	/* Verify if the speed is in the allowed limits. */
	retval=cpufreq_frequency_table_verify( policy, s3c24xx_dvs_valid_freq );
	if( retval )
		return retval;

	nearest=s3c24xx_dvs_get_nearest_freq( policy->min );
	if( nearest == 0 )
		return -EINVAL;
	else
		policy->min=nearest;

	nearest=s3c24xx_dvs_get_nearest_freq( policy->max );
	if( nearest == 0 )
		return -EINVAL;
	else
		policy->max=nearest;
	
	return 0;
}

static int __init s3c24xx_dvs_cpu_init(struct cpufreq_policy *policy)
{
	struct clk		*fclk;

	printk(KERN_INFO "S3C24xx CpuFrequency Scaling (DVS), (c) 2008 TomTom B.V.\n");

	/* We only support one CPU. */
	if (policy->cpu != 0)
	{
		printk( KERN_INFO "CPUFREQ (DVS) Detected more than one CPU ! This is not supported!\n" );
		return -EINVAL;
	}

	/* CPU will always start with highest frequency. So use this. */
	fclk=clk_get( NULL, "fclk" );
	policy->cur=clk_get_rate(fclk) / 1000;
	clk_put( fclk );

	policy->min=CPUFREQ_DVS_MEMORY_FREQ;
	policy->max=policy->cur;
	policy->cpuinfo.min_freq=policy->min;
	policy->cpuinfo.max_freq=policy->max;
	policy->governor=CPUFREQ_DEFAULT_GOVERNOR;
	policy->cpuinfo.transition_latency=CPUFREQ_DVS_TRANSITION_LATENCY;

	/* Initialize the valid freq table, to be used by the verify routine. */
	s3c24xx_dvs_valid_freq[0].index=0;
	s3c24xx_dvs_valid_freq[0].frequency=policy->min;
	s3c24xx_dvs_valid_freq[1].index=1;
	s3c24xx_dvs_valid_freq[1].frequency=policy->max;
	s3c24xx_dvs_valid_freq[2].index=2;
	s3c24xx_dvs_valid_freq[2].frequency=CPUFREQ_TABLE_END;

	return 0;
}
#ifdef CONFIG_PM
static unsigned int	pre_suspend_speed;

static int s3c24xx_dvs_suspend( struct cpufreq_policy *policy, pm_message_t pmsg )
{
	struct clk		*fclk;

	/* Get the current frequency. */
	pre_suspend_speed=s3c24xx_dvs_getspeed( 0 );
	policy->cur=pre_suspend_speed;

	/* Set DVS to lowest frequency. */
	if( !s3c24xx_dvs_setspeed( policy->min ) )
	{
		/* We shouldn't get here. */
		printk( KERN_WARNING "WARNING! CPUFREQ (DVS) Could not set new frequency (%u) on suspend!\n",
			policy->min );
		return 0;
	}

	/* Set the clock right. */
	fclk=clk_get( NULL, "fclk" );
	clk_set_rate(fclk, policy->min * 1000);
	clk_put( fclk );

	return 0;
}

static int s3c24xx_dvs_resume( struct cpufreq_policy *policy )
{
	struct clk		*fclk;

	/* Get current speed. */
	policy->cur=s3c24xx_dvs_getspeed( 0 );

	/* Set DVS to new frequency. */
	if( !s3c24xx_dvs_setspeed( pre_suspend_speed ) )
	{
		/* We shouldn't get here. */
		printk( KERN_WARNING "WARNING! CPUFREQ (DVS) Could not set new frequency (%u) on suspend!\n",
			pre_suspend_speed );
		return 0;
	}

	/* Set the clock right. */
	fclk=clk_get( NULL, "fclk" );
	clk_set_rate(fclk, pre_suspend_speed * 1000);
	clk_put( fclk );
	return 0;
}
#else
#define s3c24xx_dvs_suspend (NULL)
#define s3c24xx_dvs_resume (NULL)
#endif /* CONFIG_PM */
static struct cpufreq_driver s3c24xx_dvs_driver = {
	.flags		= CPUFREQ_STICKY,
	.init		= s3c24xx_dvs_cpu_init,
	.verify		= s3c24xx_dvs_verify_speed,
	.target		= s3c24xx_dvs_target,
	.setpolicy	= NULL,
	.suspend	= s3c24xx_dvs_suspend,
	.resume		= s3c24xx_dvs_resume,
	.get		= s3c24xx_dvs_getspeed,
	.name		= "s3c24xx_dvs",
};

static int __init s3c24xx_cpufreq_dvs_init(void)
{
	/* Check the CPU, and set the correct handlers. */
	switch( IO_GetCpuType( ) )
	{
		case GOCPU_S3C2440 :
			s3c24xx_dvs_handlestate.get=s3c2440_get_dvs_state;
			s3c24xx_dvs_handlestate.set=s3c2440_set_dvs_state;
			break;
		case GOCPU_S3C2450 :
			s3c24xx_dvs_handlestate.get=s3c2450_get_dvs_state;
			s3c24xx_dvs_handlestate.set=s3c2450_set_dvs_state;
			break;
		case GOCPU_S3C2443 :
			s3c24xx_dvs_handlestate.get=s3c2443_get_dvs_state;
			s3c24xx_dvs_handlestate.set=s3c2443_set_dvs_state;
			break;
		case GOCPU_S3C2412 :
			s3c24xx_dvs_handlestate.get=s3c2412_get_dvs_state;
			s3c24xx_dvs_handlestate.set=s3c2412_set_dvs_state;
			break;
		default :
			printk(KERN_INFO "ERROR: Unknown CPU detected. CPUFREQ (DVS) will not be loaded!");
			return -ENODEV;
			break;
	}
	
	return cpufreq_register_driver(&s3c24xx_dvs_driver);
}

arch_initcall(s3c24xx_cpufreq_dvs_init);
