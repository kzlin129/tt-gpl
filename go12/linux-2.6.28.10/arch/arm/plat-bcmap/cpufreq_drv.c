/**************************************************************************** 
 * 
 *     Copyright (c) 2007-2008 Broadcom Corporation 
 * 
 *   Unless you and Broadcom execute a separate written software license  
 *   agreement governing use of this software, this software is licensed to you  
 *   under the terms of the GNU General Public License version 2, available  
 *    at http://www.gnu.org/licenses/old-licenses/gpl-2.0.html (the "GPL").  
 * 
 *   Notwithstanding the above, under no circumstances may you combine this  
 *   software in any way with any other Broadcom software provided under a license  
 *   other than the GPL, without Broadcom's express prior written consent. 
 * 
 ****************************************************************************/ 
/** 
 * 
 *   @file   cpufreq_drv.c 
 * 
 *   @brief  cpufreq driver for freq tuning.
 * 
 ****************************************************************************/
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/system.h>

#include <asm-arm/clock_fw.h>

#define DEFAULT_TRANSITION_LATENCY 1000000 

freq_tab *p_freq_table ; 
struct cpufreq_frequency_table *p_arm_freq_table  ;
dvfs_init_info *p_dvfs_init_data ;

/*** Args : 
 * input args :
 * 
 *
 * Returns : On success : clock freq in Hz.
 *           On failure : 0.
 *           Same return value as clk_get_rate.
 *           
 ****/
int get_cur_arm_freq(void)
{
    struct clk *clk ;

	clk = clk_get(NULL, "ARM") ;
	if ( clk == NULL ) { return 0 ; }

	return (clk_get_rate(clk) ) ;
}

// +ve number on error .
/*** Args : 
 * input args :
 *  struct cpufreq_policy *policy : policy structure to verify.
 *
 * Returns : On success : 0
 *           On failure : +ve number.
 * 
 * Comment : Return values are choosen depending on the usage of these functions in other parts of system.           
 ****/
int verify_speed(struct cpufreq_policy *policy)
{
	struct clk * clk;
	long round_rate = 0 ;

	if (policy == NULL)
		return EINVAL;
	
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);

	clk = clk_get(NULL, "ARM");
	if ( clk == NULL ) { return EINVAL ; }

	round_rate = clk_round_rate(clk, policy->min * 1000) / 1000 ; 

	if ( round_rate < 0 ) { return EINVAL ; }
	else { policy->min = round_rate ; }

	round_rate = clk_round_rate(clk, policy->max * 1000) / 1000 ; 

	if ( round_rate < 0 ) { return EINVAL ; }
	else { policy->max = round_rate ; }

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	return 0;
}


/*** Args : 
 * input args :
 * struct cpufreq_policy *policy : allocated policy structure
 * unsigned int target_freq : desired target freq in kilo hertz.
 *
 * Returns : On success : 0 
 *           On failure : -ve number.
 *           
 ****/
static int target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	struct clk *clk;
	struct cpufreq_freqs freqs;
	int ret = 0;
	unsigned int cur_freq = 0 ;
	long new_freq = 0 ;

	clk = clk_get(NULL, "ARM");
	if ( clk == NULL ) { return -EINVAL ; }

	cur_freq = get_cur_arm_freq() ;
	if ( cur_freq == 0 ) { return -EINVAL ; }

	freqs.old = cur_freq;

    new_freq =  clk_round_rate(clk, target_freq * 1000) ;

	if ( new_freq < 0 ) { return -EINVAL ; }

	freqs.new = new_freq / 1000 ; 
	freqs.cpu = 0;

	// TBD : Change the voltage here. Call pmu functions here.

    cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	ret = clk_set_rate(clk, target_freq * 1000);
	if ( ret < 0 ) { return ret ; }

	// Setting new freq has passed, so post change notification.
    cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

/*** Args : 
 * input args :
 * struct cpufreq_policy *policy : allocated policy structure
 *
 * Returns : On success : 0 
 *           On failure : +ve number.
 *           
 * Comments : The caller of this from driver registration.
 ****/
static int __init cpufreq_init(struct cpufreq_policy *policy)
{
	unsigned int cur_freq = 0 ;
	if (policy == NULL)
		return EINVAL;
		// printk("...... cpufreq_init() called ...... \n") ;



	cur_freq = get_cur_arm_freq() ;
	if ( cur_freq == 0 ) { return EINVAL ; }

	policy->cur = cur_freq/1000 ; // In Kilo hz.
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	policy->cpuinfo.transition_latency = DEFAULT_TRANSITION_LATENCY ; // 1000000 ;
    
	dvfs_init_info_get(&p_dvfs_init_data) ;
	if ( p_dvfs_init_data != NULL ) 
	{
    	policy->max = p_dvfs_init_data->policy_max ; 
    	policy->min = p_dvfs_init_data->policy_min ; 
    	policy->cpuinfo.max_freq = p_dvfs_init_data->cpuinfo_max ;
    	policy->cpuinfo.min_freq = p_dvfs_init_data->cpuinfo_min ;
	}
	else
	{
    	policy->max = DEFAULT_POLICY_MAX_FREQ_IN_KHZ ; // 300000 ;
    	policy->min = DEFAULT_POLICY_MIN_FREQ_IN_KHZ ; // 1000 ;
    	policy->cpuinfo.min_freq = DEFAULT_CPU_MIN_FREQ_IN_KHZ ; // 1000000 / 1000;
    	policy->cpuinfo.max_freq = DEFAULT_CPU_MAX_FREQ_IN_KHZ ; // 300000000 / 1000;
	}
	
	// Now register the freq table so that governor will use this table to choose it's next freq.
    // cpufreq_frequency_table_get_attr(&arm_freq_table[0], 0 ) ;
	//
	dvfs_freq_table_get(&p_freq_table) ;

	if ( p_freq_table != NULL )
	{
        p_arm_freq_table = (struct cpufreq_frequency_table *)(p_freq_table) ;

        cpufreq_frequency_table_get_attr(p_arm_freq_table, 0 ) ;
    }

	return 0;
}

static struct freq_attr* bcm_clockmod_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver bcm282x_cpufreq_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= verify_speed,
	.target		= target,
	.init		= cpufreq_init,
	.name		= "cpufreq-bcmap",
	.attr		= bcm_clockmod_attr,	
};

static int __init bcm282x_cpu_init(void)
{
	return cpufreq_register_driver(&bcm282x_cpufreq_driver);
}

arch_initcall(bcm282x_cpu_init);
