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
 *   @file   clock_fw.c 
 * 
 *   @brief  Clock frame work.
 * 
 ****************************************************************************/

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>

#include <asm/io.h>          // include/asm-arm/io.h
#include <asm-arm/clock_fw.h>

#define PROC_CLOCK_ROOT "clocks_all"
static DEFINE_SPINLOCK(clkfw_lock) ; 

struct clk_functions *g_clocks = NULL ;
struct clk **g_clk_structs = NULL ;
unsigned int g_clk_structs_max_elements = 0 ;

// Freq/Voltage table.
freq_tab *g_freq_table = NULL ;

// init information for cpufreq.
dvfs_init_info *g_dvfs_init_info = NULL ;

/*** Args : 
 *input args:
 * struct device *dev : Device structure
 * const char *id : Clock name as specified in clk structure. 
 *
 * Returns : On success : Pointer to clk struct when id string matches a clock name
 *                        in registered clock list.
 *           On failure : NULL when clock is not found in registered clock list
 * Comments : Each architechture registers the list of clocks it has making use of
 *            clk_register functions. This list is looked at to obtain the requested 
 *            clock name ( *id ).
 ****/
struct clk * clk_get(struct device *dev, const char *id)  // Done.
{
    struct clk **clk = NULL ;
    struct clk *clk_not_found = NULL ;
    unsigned int i = 0 ;
    unsigned long flags = 0 ;

	spin_lock_irqsave(&clkfw_lock, flags ) ;

	// All the calls to this function will happen with a dev value of NULL, and 
	// a string for id argument. So id can't be NULL. id is the clock name as 
	// specified in the name element of clock structure. Look for struct clk in 
	// header file.
	if ( id == NULL ) 
	{ 
        return clk_not_found ; 
	}

    clk = g_clk_structs ;

	// Search through the clocks list, and get the clk structure with clock name 
	// that matches with id string.
    for(i=0; i<g_clk_structs_max_elements; i++ )
    {
        if ( strcmp(id, (*clk)->name ) == 0 ) 
        {
            spin_unlock_irqrestore(&clkfw_lock, flags ) ;
            return *clk ;
    	}
    	clk++ ;
    }

    spin_unlock_irqrestore(&clkfw_lock, flags ) ;
    return clk_not_found ;
}
EXPORT_SYMBOL(clk_get);

/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 *
 * Returns : On Success : 0
 *           On Failure : -ve number.
 ****/
int clk_enable(struct clk *clk)
{
    int ret = -EINVAL ; 

    if ( clk == NULL ) { return ret ; }

    if ( g_clocks->clk_enable ) { ret = g_clocks->clk_enable(clk) ; }

    return ret;
}
EXPORT_SYMBOL(clk_enable);


/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 *
 * Returns : Nothing.
 *           
 * Comments : This function as per clk.h is not supposed to return anything.
 ****/
void clk_disable(struct clk *clk)
{
	int ret = 0 ;

    if ( clk == NULL ) { return ; }

    if ( g_clocks->clk_disable ) { ret = g_clocks->clk_disable(clk) ; }

    return ;
}
EXPORT_SYMBOL(clk_disable);

/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 *
 * Returns : On success : clock freq in Hz.
 *           On failure : 0.
 *           
 ****/
unsigned long clk_get_rate(struct clk *clk)
{
    unsigned long ret = 0 ; 

    if ( clk == NULL ) { return ret ; }

    if ( g_clocks->clk_get_rate ) { ret = g_clocks->clk_get_rate(clk) ; }

    return ret;
}
EXPORT_SYMBOL(clk_get_rate);

/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 * unsigned ling rate : desired freq.
 *
 * Returns : On success : actual clock freq in Hz.
 *           On failure : -ve value.
 *           
 ****/
long clk_round_rate(struct clk *clk, unsigned long rate)
{
    long ret = -EINVAL ; 

    if ( clk == NULL ) { return ret ; }

    if ( g_clocks->clk_round_rate ) { ret = g_clocks->clk_round_rate(clk, rate) ; }

    return ret;
}
EXPORT_SYMBOL(clk_round_rate);

/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 * unsigned long rate : desired freq.
 *
 * Returns : On success : 0
 *           On failure : -ve value.
 ****/
int clk_set_rate(struct clk *clk, unsigned long rate) // Done.
{
    int ret = -EINVAL ; 

    if ( clk == NULL ) { return ret ; }

    if ( g_clocks->clk_set_rate ) { ret = g_clocks->clk_set_rate(clk, rate) ; }

    return ret;
}
EXPORT_SYMBOL(clk_set_rate);

/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 * struct clk *parent : parent clk struct .
 *
 * Returns : On success : 0
 *           On failure : -ve value.
 ****/
int clk_set_parent(struct clk *clk, struct clk *parent)
{
    int ret = -EINVAL ; 

    if ( ( clk == NULL ) || ( parent == NULL ) ) { return ret ; }

    if ( g_clocks->clk_set_parent ) { ret = g_clocks->clk_set_parent(clk, parent) ; }

    return ret;
}
EXPORT_SYMBOL(clk_set_parent);

/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 *
 * Returns : On success : clk's parent's clk structure.
 *           On failure : NULL. 
 ****/
struct clk *clk_get_parent(struct clk *clk)
{
    struct clk *ret = NULL ;

    if ( clk == NULL ) { return ret ; }

    if ( g_clocks->clk_get_parent ) { ret = g_clocks->clk_get_parent(clk) ; }

    return ret;
}
EXPORT_SYMBOL(clk_get_parent);

/*** Args : 
 * input args :
 * struct clk **clk : list of all clocks supported
 * unsigned int max_elements : total number of clocks supported.
 *
 * Returns : On success : 0.
 ****/
int clk_register(struct clk **clk, unsigned int max_elements)
{
    // struct clk **temp = NULL ;
    // unsigned int i = 0 ;

    if ( clk != NULL ) 
    { 
        g_clk_structs = clk ;    
        g_clk_structs_max_elements = max_elements ;
    }
	
    return 0;
}
EXPORT_SYMBOL(clk_register);

void clk_unregister(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_unregister);

/*** Args : 
 * input args :
 * struct clk_functions *custom_clocks : list of all functionality for clocks.
 *
 * Returns : On success : 0.
 ****/
int clk_init(struct clk_functions * custom_clocks)
{
	if (!custom_clocks) {
		printk(KERN_ERR "No custom clock functions registered\n");
		BUG();
	}

	g_clocks  = custom_clocks;

	printk("##### Clock framework registration done... \n") ;

	return 0;
}

/*** Args : 
 * input args :
 * freq_tab *freq_table : list of freq, and voltage combination.
 *
 * Returns : None.
 ****/
void dvfs_freq_table_register(freq_tab *freq_table)
{
    g_freq_table = freq_table ;
    return ;
}

/*** Args : 
 * input args :
 * freq_tab **freq_table : get list of freq, and voltage combination.
 *
 * Returns : None.
 ****/
void dvfs_freq_table_get(freq_tab **freq_table)
{
    *freq_table = g_freq_table ;
	return ;
}

/*** Args : 
 * input args :
 * dvfs_init_info *init_info : register init information for cpufreq.
 *
 * Returns : None.
 ****/
void dvfs_init_info_register(dvfs_init_info *init_info )
{
    g_dvfs_init_info = init_info ;
    return ;
}

/*** Args : 
 * input args :
 * dvfs_init_info **init_info : get init information for cpufreq.
 *
 * Returns : None.
 ****/
void dvfs_init_info_get(dvfs_init_info **init_info )
{
    *init_info = g_dvfs_init_info ;
    return ;
}
