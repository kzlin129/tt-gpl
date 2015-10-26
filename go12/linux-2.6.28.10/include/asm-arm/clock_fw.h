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
 *   @file   clock_fw.h 
 * 
 *   @brief  clock frame functions.
 * 
 ****************************************************************************/

#ifndef __ARCH_ARM_BCMCOMM_CLOCK_H
#define __ARCH_ARM_BCMCOMM_CLOCK_H

struct clk;

#include <asm-arm/arch-bcm4760/clock_bcm4760.h>   

// Default values that can be used by cpufreq (in KHz).
#define DEFAULT_POLICY_MIN_FREQ_IN_KHZ    FREQ_1_IN_MHZ/1000
#define DEFAULT_POLICY_MAX_FREQ_IN_KHZ    FREQ_5_IN_MHZ/1000
#define DEFAULT_CPU_MIN_FREQ_IN_KHZ       FREQ_1_IN_MHZ/1000
#define DEFAULT_CPU_MAX_FREQ_IN_KHZ       FREQ_5_IN_MHZ/1000

// Common clock structure.
struct clk {
    const char     *name;
    int             cnt ;                    // Count of drivers using this clock.
    void            (*init)(struct clk *);   // Clock init function.
    int             (*enable)(struct clk *); // enable function for this clock.
    void            (*disable)(struct clk *);// disable
    int             (*set_rate)(struct clk *, unsigned long); // Set rate.
    unsigned long   (*get_rate)(struct clk *) ; // get present clock rate.
    long            (*round_rate)(struct clk *, unsigned long) ; // rounded freq for a desired freq.
    void            *arch_dep_props ;         // Address to data structures that hold architecture specific information.
    unsigned int    core_pll_osci ;          // Whether this is a oscillator, or pll or a core clock.
    struct clk     *parent;
    unsigned long   rate;                    // Current clock rate (Hz) of parent or self ? Need to 
    int             id;
} ;

// Functions that will be registered per architecture.
struct clk_functions {
    // These are the functions as defined in clk.h
    int	  (*clk_enable)(struct clk *clk);
    int   (*clk_disable)(struct clk *clk);
    unsigned long (*clk_get_rate)(struct clk *clk);
    int	  (*clk_set_rate)(struct clk *clk, unsigned long rate);
    int	  (*clk_set_parent)(struct clk *clk, struct clk *parent);
    struct clk *(*clk_get_parent)(struct clk *clk);
    long  (*clk_round_rate)(struct clk *clk, unsigned long rate);
    // include/linux/clk.h defined functions end.
};


// This is the voltage, and freq structure. A table of this struct type will be registered 
// with clock framework, so that cpufreq can use it while doing freq/voltage scaling.
typedef struct freq_tab_s
{
    unsigned int val ;  // Voltage setting.
    unsigned int freq ; // Freq in kilo hz.
} freq_tab ;

typedef struct dvfs_init_info_s
{
    unsigned int policy_min ;   // Policy min freq.
    unsigned int policy_max ;   // Policy max freq.
    unsigned int cpuinfo_min ;  // CPU's min freq.
    unsigned int cpuinfo_max ;  // CPU's max freq.
} dvfs_init_info ;

// Every architechture calls this function to register its freq/voltage table.
void dvfs_freq_table_register(freq_tab *freq_table) ;

// Use this function to get freq/voltage table during dvfs.
void dvfs_freq_table_get(freq_tab **freq_table) ;

// Every architechture calls this function to register its init values of cpufreq.
void dvfs_init_info_register(dvfs_init_info *init_info) ;

// Use this function to get init information for cpufreq.
void dvfs_init_info_get(dvfs_init_info **init_info ) ;

// common functions between various architectures.
extern int clk_init(struct clk_functions * custom_clocks);
extern int clk_register(struct clk **clk, unsigned int max_clks);
extern void clk_unregister(struct clk *clk);
#endif
