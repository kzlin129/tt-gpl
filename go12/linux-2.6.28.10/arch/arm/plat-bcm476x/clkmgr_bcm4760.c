/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
 *   @file   clkmgr_bcm4760.c 
 * 
 *   @brief  clock apis to interface with clock framework.
 * 
 ****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>

#include <asm/io.h>
#include <asm/div64.h>

#include <asm-arm/clock_fw.h>
#include <asm-arm/arch/bbl4760.h>
#include <asm-arm/arch/rtc_cpuapi4760.h>
#include <asm-arm/arch-bcm4760/clock_bcm4760.h>

#include "./clk_bcm4760/tahoe_opmode_cmd_stats.h"
#include "./clk_bcm4760/tahoe_opmode_cmd_params.h"
#include "./clk_bcm4760/tahoe_opmode_priv.h"
#include "./clk_bcm4760/chipmgr.h"
#include "./clk_bcm4760/clkmgr.h"
#include "./clk_bcm4760/armbusmtx.h"
#include "./clk_bcm4760/emi.h"
#include "./clk_bcm4760/tahoe_opmode.h"

#include <linux/cpufreq.h>

#define PROC_CLOCK_ROOT "clocks_all"

extern int cds_pll_switch(uint32_t ref_index, int cds_pll_switch);
extern int cds_pll_init(uint32_t ref_clk_idx);

void bcm4760_test(void) ;
static struct clk arm_clk = {
	.name		= "ARM",
	.core_pll_osci = BCM4760_TYPE_PLL_CLOCK ,
	.set_rate	= &bcm4760_clk_set_rate ,
};

static struct clk uart_clk = {
	.name	= "UARTCLK",
	.core_pll_osci = BCM4760_TYPE_OSCI ,
	.rate	= 78000000,         // 12Mhz for ChipIt, 78Mhz for SoC
//	.rate	= 13000000,         // 12Mhz for ChipIt, 26Mhz bypass SoC

//	.set_rate	= &bcm4760_clk_set_rate ,
};

static struct clk lcd_clk = {
	.name	= "CLCDCLK",
	.core_pll_osci = BCM4760_TYPE_OSCI ,
	.rate	= 10000000,         // 10Mhz
};

static struct clk pll1 = {
	.name		= "PLL1",
	.core_pll_osci = BCM4760_TYPE_PLL_CLOCK ,
	.set_rate	= &bcm4760_clk_set_rate ,
};

static struct clk clk_24MHz = {
	.name		= "24MHZ",
	.core_pll_osci = BCM4760_TYPE_OSCI ,
	.rate = 24000000 ,
	.set_rate	= &bcm4760_clk_set_rate ,
};

static struct clk crpt = {
	.name		= "CRPT",
	.core_pll_osci = BCM4760_TYPE_CORE_CLOCK ,
	.set_rate	= &bcm4760_clk_set_rate ,
};

// static struct clk *bcm4760_clocks[BCM4760_CF_MAX_CLOCKS] =
static struct clk *bcm4760_clocks[] = 
{
    &arm_clk ,
    &uart_clk ,
    &lcd_clk,
	&pll1 ,
	&clk_24MHz,
	&crpt
} ;

// cpufreq related init information.
dvfs_init_info  g_dvfs_init_data ;

// Frequency table, that will be registered, for usage in cpufreq.
freq_tab bcm4760_freq_table[] = 
{
    { 0 , 300000 } ,
    { 0 , 240000 } ,
    { 0 , 220000 } ,
    { 0 , 200000 } ,
    { 0 , 180000 } ,
    { 0 , 160000 } ,
    { 0 , 140000 } ,
    { 0 , 130000 } ,
    { 0 , 120000 } ,
    { 0 , 100000 } ,
    { 0 ,  80000 } ,
    { 0 ,  60000 } ,
    { 0 ,  40000 } ,
    { 0 ,  20000 } ,
    { 0 ,  15000 } ,
    { 0 ,  10000 } ,
    { 0 ,  1000 } ,
    { 0 ,  750 } ,
    { 0 , CPUFREQ_TABLE_END }
} ;

unsigned long g_clks_registered = 0 ;
static struct clk *bcm4760_convert_val_to_name_str_and_search_clk_struct(int val) ;
static int bcm4760_convert_name_str_to_val(struct clk *clk, unsigned int *val) ;

int bcm4760_clk_proc_summary ( char *page, char **start, off_t off, int count, int *eof, void *data) ;
/*** Args :
 * struct clk *clk : Clock structure
 *
 * Returns : Success - 0
 *           Failure - -ve number.
 ****/
// This function is completed per coding standards.
int bcm4760_clk_disable(struct clk *clk) // Done.
{
	// unsigned long val = 0 ;
	int ret = 0 ;
	unsigned int core_pll_osci_val = 0 ;
    tahoe_opmode_corectrl_t core_details ;
    bcm47xx_csrc_info_t csrc_details ;

    ret = bcm4760_convert_name_str_to_val(clk, &core_pll_osci_val) ;
	if ( ret != 0 ) { return (-ret) ; }

	if ( clk->core_pll_osci == BCM4760_TYPE_CORE_CLOCK )
	{
        core_details.core = core_pll_osci_val ;
		core_details.stat = disable ;
        ret = tahoe_opmode_config_core(&core_details, 1) ;
	}
#if 0 //DCN, is it a bug? could not disable OSCI
	else if ( ( clk->core_pll_osci == BCM4760_TYPE_PLL_CLOCK ) || ( clk->core_pll_osci == BCM4760_TYPE_OSCI ) )
	{
	   csrc_details.csrc = core_pll_osci_val ;
	   csrc_details.stat = disable ;
       ret = bcm47xx_config_csrcs(&csrc_details, 1 ) ;
	}
#else
	else if ( clk->core_pll_osci == BCM4760_TYPE_PLL_CLOCK )
	{
	   csrc_details.csrc = core_pll_osci_val ;
	   csrc_details.stat = disable ;
       ret = bcm47xx_config_csrcs(&csrc_details, 1 ) ;
	}
#endif

	if ( ret != OpOk ) { return (-ret) ; }
	else { return 0 ; }
}

/*** Args :
 * struct clk *clk : Clock structure
 *
 * Returns : Success - 0
 *           Failure - -ve number.
 ****/
// This function is completed per coding standards.
static int bcm4760_clk_enable(struct clk *clk)
{
	int ret = 0 ;
	unsigned int core_pll_osci_val = 0 ;
    tahoe_opmode_corectrl_t core_details ;
    bcm47xx_csrc_info_t csrc_details ;

    if ( clk == NULL ) { return -EINVAL ; }

    ret = bcm4760_convert_name_str_to_val(clk, &core_pll_osci_val) ;
	if ( ret != 0 ) { return (-ret) ; }

	if ( clk->core_pll_osci == BCM4760_TYPE_CORE_CLOCK )
	{
        core_details.core = core_pll_osci_val ;
		core_details.stat = enable ;
        ret = tahoe_opmode_config_core(&core_details, 1) ;
	}
	else if ( clk->core_pll_osci == BCM4760_TYPE_PLL_CLOCK )
	{
	   csrc_details.csrc = core_pll_osci_val ;
	   csrc_details.stat = enable ;
       ret = bcm47xx_config_csrcs(&csrc_details, 1 ) ;
	}

	if ( ret != OpOk ) { return (-ret) ; }
	else { return 0 ; }
}

/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 *
 * Returns : On success : clock freq in Hz.
 *           On failure : 0.
 ****/
unsigned long bcm4760_clk_get_rate(struct clk *clk)
{
	unsigned long val = 0 ;
	int ret = 0 ;
	unsigned int core_pll_osci_val = 0 ;

    ret = bcm4760_convert_name_str_to_val(clk, &core_pll_osci_val) ;
	// When there is error, return the freq as 0, indication error, for this function.
	if ( ret != 0 ) { return 0 ; }


	// Look for the type of clock, and call corresponding low level functions.
	if ( clk->core_pll_osci == BCM4760_TYPE_CORE_CLOCK )
	{
        ret = tahoe_opmode_get_core_freq(core_pll_osci_val, &val) ;
	}
	else if ( clk->core_pll_osci == BCM4760_TYPE_PLL_CLOCK )
	{
       ret = tahoe_opmode_get_pll_freq( (core_pll_osci_val - ECLKMGR_TCSRC_PLLS), &val ) ;
	}
	else if ( clk->core_pll_osci == BCM4760_TYPE_OSCI )
	{
	   val = clk->rate ;
	}

	// When there is error, return the freq as 0.
	if ( ret != OpOk ) { return 0 ; }
	else { return val ; }
}

/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 * unsigned ling rate : desired freq.
 *
 * Returns : On success : actual clock freq in Hz.
 *           On failure : -ve value.
 *
 ****/
// This function is completed per coding standards.
long bcm4760_clk_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long val = 0 ;
	int ret = -EINVAL ;

	// Round rate presently supported for arm freq only, for usage in cpufreq.
	if ( strcmp(clk->name, "ARM") != 0 ) { return rate ; }

    ret = tahoe_opmode_round_rate_arm_freq( rate, 0, &val) ;
	if ( ret == OpOk ) { ret = val ; }
	else { ret = -ret ; }

	return ret ;
}


/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 * unsigned ling rate : desired freq.
 *
 * Returns : On success : 0.
 *           On failure : -ve value.
 *
 ****/
// This function is completed per coding standards.
int bcm4760_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long val = 0 ;
	int ret = 0 ;
	unsigned int core_pll_osci_val = 0 ;

    ret = bcm4760_convert_name_str_to_val(clk, &core_pll_osci_val) ;
	if ( ret != 0 ) { return (-ret) ; }

	if ( clk->core_pll_osci == BCM4760_TYPE_CORE_CLOCK )
	{
        ret = tahoe_opmode_set_core_freq(core_pll_osci_val, rate, 0, &val) ;
	}
	else if ( clk->core_pll_osci == BCM4760_TYPE_PLL_CLOCK )
	{
       ret = tahoe_opmode_set_pll_freq( (core_pll_osci_val - ECLKMGR_TCSRC_PLLS), rate, 0, &val ) ;
	}
	else if ( clk->core_pll_osci == BCM4760_TYPE_OSCI )
	{
        // Can't set osci freq.
		ret = OpBadParam ;
	}

	// printk("Setting freq %ld \n", rate ) ;

	if ( ret != OpOk ) { return (-ret) ; }
	else { return 0 ; }
}

/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 *
 * Returns : On success : clk's parent's clk structure.
 *           On failure : NULL.
 ****/
// This function is completed per coding standards.
struct clk *bcm4760_clk_get_parent(struct clk *clk)
{
	struct clk *ret = NULL ;
	int local = 0 ;
	unsigned int core_pll_osci_val = 0 ;
    tahoe_opmode_csrc_t csrc ;
	int div_val = 0 ;

	// No parent for oscillator
	if ( clk->core_pll_osci == BCM4760_TYPE_OSCI )
	{
        // This is oscillator. So return null.
		ret = NULL ;
	}

    local = bcm4760_convert_name_str_to_val(clk, &core_pll_osci_val) ;
	if ( local != 0 ) { return (ret) ; }

	if ( clk->core_pll_osci == BCM4760_TYPE_CORE_CLOCK )
	{
        local = tahoe_opmode_get_core_csrc_and_div(core_pll_osci_val, &csrc, &div_val) ;
		if ( local == OpOk )
		{
            // Now we need to convert csrc into string.
			// Then search this string in the list of bcm4760_clocks.
			// and return the clk struct.
			return ( bcm4760_convert_val_to_name_str_and_search_clk_struct(csrc ) ) ;
		}

	}
	else if ( clk->core_pll_osci == BCM4760_TYPE_PLL_CLOCK )
	{
       // 24MHz oscillator is the only source for all plls.
	   ret = &clk_24MHz ;
	}
	return ret ;
}

/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 * struct clk *parent : parent clk struct .
 *
 * Returns : On success : 0
 *           On failure : -ve value.
 ****/
// This function is completed per coding standards.
int bcm4760_clk_set_parent(struct clk *clk, struct clk *parent)
{
	int local = 0 ;
	unsigned int clk_pll_osci_val = 0 ;
	unsigned int parent_pll_osci_val = 0 ;

	// A core clock can't become parent to anyone. It is a leaf
	// All PLLs have a single parent (24MHz Oscil) for 4760. So we can't change PLL's parent.
	// There is no parent for oscillators.
	if ( ( parent->core_pll_osci == BCM4760_TYPE_CORE_CLOCK ) ||
		 ( clk->core_pll_osci == BCM4760_TYPE_PLL_CLOCK ) ||
		 ( clk->core_pll_osci == BCM4760_TYPE_OSCI ) )
	{
        return -EINVAL ;
	}

    local = bcm4760_convert_name_str_to_val(clk, &clk_pll_osci_val) ;
	if ( local != 0 ) { return (-local) ; }

	if ( clk->core_pll_osci == BCM4760_TYPE_CORE_CLOCK )
	{
        local = bcm4760_convert_name_str_to_val(parent, &parent_pll_osci_val) ;
    	if ( local != 0 ) { return (-local) ; }

		if ( local == OpOk )
		{
            local = tahoe_opmode_set_core_csrc_and_div(clk_pll_osci_val, (tahoe_opmode_csrc_t)parent_pll_osci_val, 1) ;
			return (-local) ;
		}
	}

	return 0 ;
}

/*** Args : 
 * input args :
 * int val : enum of core/clock value.
 *
 * Returns : On success : clk structure.
 *           On failure : NULL.
 ****/
static struct clk *bcm4760_convert_val_to_name_str_and_search_clk_struct(int val)
{
    // tahoe_opmode_core_t  core = 0 ;
    // tahoe_opmode_csrc_t csrc = 0 ;
	struct clk **pclk_list ;
	unsigned int i = 0 ;
	const char *ptr ;

#if 0
    pclk_list = &(bcm4760_clocks[0]) ;

    for(i=0 ; i<g_clks_registered ; i++ )
	{
        printk("$$$$$$ Clk name is %s \n", ((struct clk *)(*(pclk_list)))->name ) ;
	    pclk_list++ ;
	}
#endif

    ptr = tahoe_opmode_csrc_str(val) ;
    if ( ( strcmp(ptr,"INVALID") ) == 0 ) { return NULL ; }
	else
	{
		pclk_list = &(bcm4760_clocks[0]) ;

        for(i=0 ; i<g_clks_registered; i++ )
		{
			if ( ( strcmp(ptr, ((struct clk *)(*(pclk_list)))->name )) == 0 ) { return bcm4760_clocks[i] ; }
	        pclk_list++ ;
		}
	}

	return NULL ;
}

/*** Args : 
 * input args:
 * struct clk *clk : Clock structure
 *
 * output args :
 * unsigned int *val : Core equivalent for clock name. Will be used
 *                     by caller to perform other operations based on
 *                     the enum of core
 *
 * Returns : Success - 0
 *           Failure - -ve number.
 ****/
// This function is completed per coding standards.
static int bcm4760_convert_name_str_to_val(struct clk *clk, unsigned int *val)
{
    tahoe_opmode_core_t  core = 0 ;
    tahoe_opmode_csrc_t csrc = 0 ;

	// Clock will be for core, or it could be a pll, or an oscillator.
	// clk structure has this information as an attribute of clock.
	if ( clk->core_pll_osci == BCM4760_TYPE_CORE_CLOCK )
	{
        core = tahoe_opmode_get_core((char *)(clk->name)) ;
	    if ( core == OPMODE_CORE_COUNT ) {  return (BCM4760_ERR_CODE_INVALID_CORE_CLOCK) ; }
		else { *val = core ; }
	}
	else if ( ( clk->core_pll_osci == BCM4760_TYPE_PLL_CLOCK ) || ( clk->core_pll_osci == BCM4760_TYPE_OSCI ) )
	{
		// Try for oscialltor/pll now.
        csrc = tahoe_opmode_get_csrc((char *)(clk->name));
        if ( csrc == OPMODE_TCSRC_COUNT ) { return (BCM4760_ERR_CODE_INVALID_PLL_OSCI_CLOCK) ; }
		else { *val = csrc ; }
	}
	else
	{
        return (BCM4760_ERR_CODE_INVALID_CLOCK_TYPE) ;
	}

	return 0 ;
}

/*** Args : 
 * input args :
 *
 * Returns : bytes we have to return.
 ****/
static int proc_read_from_buffer(char *page, char **start, off_t off,
        int count, int *eof, const char *buffer, size_t buffer_size)
{
    int bytes_we_have;

    if (buffer_size <= off)
    {
        *eof = 1;
        return 0;
    }
    bytes_we_have = (buffer_size - off);
    if (bytes_we_have < count)
        *eof = 1;
    else
        bytes_we_have = count;
    memcpy(page + off, buffer + off, bytes_we_have);
    *start = page + off;

    return bytes_we_have;
}

/*** Args : 
 * input args :
 * struct clk *clk : Clock structure
 *
 * Returns : On success : clock freq in Hz.
 *           On failure : 0.
 ****/
static int bcm4760_clock_proc_arm_round_rate (struct file *file, const char *buf, unsigned long count, void *data)
{
	char *ptr ;
    long freq = 0 ;
	struct clk *clk ;
	int ret_val = 0 ;

	ptr = (char *)data ;

	// printk("bcm4760_clock_proc_arm_round_rate function getting called. %s count = %d buf[0]%c,  [1]%c,  [2]%c  \n", ptr, count, buf[0], buf[1], buf[2]  ) ;
    freq = simple_strtoul(buf, NULL, 0 ) ;
	clk = clk_get(NULL,"ARM") ;
	if ( clk == NULL ) { return -EACCES ; }

	freq = clk_round_rate(clk, freq) ;
    if ( ret_val < 0 ) { return -EACCES ; }

	printk("Round_freq to %ld \n", freq ) ;
    return count;
}


/*** Args : 
 * input args :
 *
 * Returns : On success : count number of bytes.
 *           On failure : -ve number.
 ****/
static int bcm4760_clock_proc_arm_set_rate (struct file *file, const char *buf, unsigned long count, void *data)
{
	char *ptr ;
    // char text_buffer[64];
    // char *follow_text;
    unsigned long freq = 0 ;
	struct clk *clk ;
	int ret_val = 0 ;

	ptr = (char *)data ;

	// printk("bcm4760_clock_proc_arm_set_rate function getting called. %s count = %d buf[0]%c,  [1]%c,  [2]%c  \n", ptr, count, buf[0], buf[1], buf[2]  ) ;

    freq = simple_strtoul(buf, NULL, 0 ) ;
	clk = clk_get(NULL,"ARM") ;
	if ( clk == NULL ) { return -EACCES ; }

	ret_val = clk_set_rate(clk, freq) ;
    if ( ret_val == 0 ) { return count ; }

    return -EACCES;
}

/*** Args :
 * input args :
 *
 * Returns : On success : number of bytes to be sent to user space.
 *           On failure : -ve number.
 ****/
static int bcm4760_clock_proc_arm_get_rate( char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *ptr ;
	char *temp ;
    char text_buffer[64] = {'\0'} ;
    char *follow_text;
    unsigned long freq = 0 ;
	struct clk *clk ;
    unsigned int i = 0 ;
	ptr = (char *)data ;

    if ( ptr )
	{
		temp = &text_buffer[0] ; i = 0 ;
        while (*ptr != '\0' ) { *temp = isalpha(*ptr) ? toupper(*ptr) : *ptr ; temp++; ptr++ ; }
	}
	else
	{
        return -EACCES ; // Error .
	}

	// temp = &text_buffer[0] ;
	// printk("bcm4760_clock_proc_no_read function getting called. ***** data is %s count is %d toupper is %s \n", ptr, count, temp ) ;

    follow_text = &(text_buffer[0]);

	// clk = clk_get(NULL,"ARM") ;
	clk = clk_get(NULL,&text_buffer[0]) ;
	freq = clk_get_rate(clk) ;

    follow_text += snprintf(follow_text,
            4096 - (follow_text - &(text_buffer[0])), "%ld",
			freq ) ;

    return proc_read_from_buffer(page, start, off, count, eof,
            &(text_buffer[0]), (follow_text - &(text_buffer[0])));

    return -EACCES;
}


static int bcm4760_clock_proc_no_write(struct file *file, const char *buf, unsigned long count, void *data)
{
	printk("bcm4760_clock_proc_no_write function getting called.\n") ;
    return -EACCES;
}

static int bcm4760_clock_proc_no_read( char *page, char **start, off_t off, int count, int *eof, void *data)
{
	printk("bcm4760_clock_proc_no_read function getting called.\n") ;

    return -EACCES;
}


/*** Args : 
 * input args :
 *
 * Returns : None
 *
 * Comments : Create a proc entry with content of name_buffer.
 ****/
void bcm4760_create_clock_proc_entry(
        char *name_buffer, size_t sizeof_name_buffer,
		char *branch_name, // char *leaf_name,
        int (*read_func)(char *page, char **start, off_t off, int count,
                         int *eof, void *data),
        int (*write_func)(struct file *file, const char *buf,
                          unsigned long count, void *data))
{
    struct proc_dir_entry *entry;

    entry = create_proc_entry(name_buffer, S_IFREG | S_IRUGO | S_IWUSR, 0);
    if (entry == NULL)
        return;
    entry->nlink = 1;
    entry->data = (void *)branch_name ;
    entry->read_proc = read_func;
    entry->write_proc = write_func;
}

// void bcm4760_clk_proc_tree(char *clk_root_name, char *page, char **start, off_t off, int count, int *eof, void *data )
/*** Args : 
 * input args :
 * char *clk_root_name : Root name of clock from where to register more elements.
 *
 * Returns : None.
 ****/
void bcm4760_clk_proc_tree(char *clk_root_name)
{
    void *mkdir_result;
    char name_buffer[1024];

	// Create dir for root of clock tree.
    // mkdir_result = proc_mkdir(PROC_CLOCK_ROOT, 0);
    mkdir_result = proc_mkdir(clk_root_name, 0);
    if (mkdir_result == NULL)
        return;

	// Minimum register a summary leaf that shows all clock details.
	// Basically list of all clocks of system, it's parent, and present rate.
    snprintf(&(name_buffer[0]), sizeof(name_buffer), "%s/%s", clk_root_name, "summary") ;
    bcm4760_create_clock_proc_entry(&(name_buffer[0]), sizeof(name_buffer), "summary",
					&bcm4760_clk_proc_summary, &bcm4760_clock_proc_no_write);

    snprintf(&(name_buffer[0]), sizeof(name_buffer), "%s/%s", clk_root_name, "cores" ) ;
    mkdir_result = proc_mkdir(&(name_buffer[0]), 0);
        if (mkdir_result == NULL)
            return;

    snprintf(&(name_buffer[0]), sizeof(name_buffer), "%s/%s", clk_root_name, "plls" ) ;
    mkdir_result = proc_mkdir(&(name_buffer[0]), 0);
        if (mkdir_result == NULL)
            return;

    snprintf(&(name_buffer[0]), sizeof(name_buffer), "%s/%s/%s", clk_root_name, "cores", "arm" ) ;
    mkdir_result = proc_mkdir(&(name_buffer[0]), 0);
        if (mkdir_result == NULL)
            return;

    snprintf(&(name_buffer[0]), sizeof(name_buffer), "%s/%s/%s/%s", clk_root_name, "cores", "arm", "get_rate" ) ;
    bcm4760_create_clock_proc_entry(&(name_buffer[0]), sizeof(name_buffer), "arm",
					&bcm4760_clock_proc_arm_get_rate,
					&bcm4760_clock_proc_no_write);

    snprintf(&(name_buffer[0]), sizeof(name_buffer), "%s/%s/%s/%s", clk_root_name, "cores", "arm", "set_rate" ) ;
    bcm4760_create_clock_proc_entry(&(name_buffer[0]), sizeof(name_buffer), "arm",
					&bcm4760_clock_proc_no_read,
					&bcm4760_clock_proc_arm_set_rate ) ;

    snprintf(&(name_buffer[0]), sizeof(name_buffer), "%s/%s/%s/%s", clk_root_name, "cores", "arm", "round_rate" ) ;
    bcm4760_create_clock_proc_entry(&(name_buffer[0]), sizeof(name_buffer), "arm",
					&bcm4760_clock_proc_no_read,
					&bcm4760_clock_proc_arm_round_rate ) ;
}


/*** Args : 
 * input args :
 * int val : All parameters for proc.
 *
 * Returns : Number of bytes send to user space.
 ****/
int bcm4760_clk_proc_summary ( char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int cnt = 0 ;

    cnt = tahoe_opmode_stats(page, count) ;

	return cnt ;

}

struct clk_functions bcm4760_clk_functions = {
	.clk_enable		    = bcm4760_clk_enable,
	.clk_disable		= bcm4760_clk_disable,
	.clk_set_rate		= bcm4760_clk_set_rate,
	.clk_round_rate		= bcm4760_clk_round_rate,
	.clk_get_rate       = bcm4760_clk_get_rate,
	.clk_set_parent		= bcm4760_clk_set_parent,
	.clk_get_parent		= bcm4760_clk_get_parent,
	// .clk_proc_summary   = bcm4760_clk_proc_summary,
	// .clk_proc_entries   = bcm4760_clk_proc_tree
};

// Should move this to clock_fw.c
void clk_put(struct clk *clk)
{
    if ( clk == NULL ) { return; }

    // DCN, incompleted, what we need to do here?
	//module_put(clk->owner);
}

EXPORT_SYMBOL(clk_put);
/*** Args : 
 * input args : None.
 * struct clk *clk : Clock structure
 *
 * Returns : On success : 0
 ****/
int __init bcm4760_clk_init(void)
{
	// Initialize low level register inits.
	chipmgr_api_init() ;
    clkmgr_api_init() ;
    armbusmtx_api_init();
    emi_api_init();

	// Register these functions with common clock manager layer.
	clk_init(&bcm4760_clk_functions);

	g_clks_registered = sizeof(bcm4760_clocks) / sizeof(struct clk *) ;

	// Inform clock frame work about all clock information.
	clk_register(&(bcm4760_clocks[0]), g_clks_registered ) ;

	// Register the freq table with clock frame work. This will be later used by cpufreq 
	dvfs_freq_table_register(&(bcm4760_freq_table[0])) ;

	// Register init information for cpufreq.
	g_dvfs_init_data.policy_min = 1000 ;
	g_dvfs_init_data.policy_max = 300000 ;
	g_dvfs_init_data.cpuinfo_min = 1000 ;
	g_dvfs_init_data.cpuinfo_max = 300000 ;
	dvfs_init_info_register(&g_dvfs_init_data) ;

	/* Re-init pll to the the settings defined in clk_reg_ref[] */
	if(((rtc_read(BBL_OFFSET_DDR_TOMTOM_OFFSET)&0x00FF0000)>>16)==0x01)
		cds_pll_init(1);
	else
		cds_pll_init(0);

	// Create proc fs.
#ifdef CONFIG_PROC_FS
    bcm4760_clk_proc_tree(PROC_CLOCK_ROOT) ;
#endif

	return 0;
}


void bcm4760_test(void)
{
    struct clk *clk ;
    struct clk *clk_parent ;
	unsigned long freq = 0 ;
	int temp = 0 ;

	clk = clk_get(NULL, "CRPT");
	freq = clk_get_rate(clk)  ;
	printk("========= \n") ;
	printk("========= CRPT get rate is %ld \n", freq ) ;

	clk_set_rate(clk, 60000000 ) ;
	printk("========= Set crpt clk to 60MHz \n") ;

	freq = clk_get_rate(clk)  ;
	printk("========= CRPT get rate is %ld \n", freq ) ;

	clk_parent = clk_get_parent(clk) ;
	printk("========= CRPT parent name is %s \n", clk_parent->name ) ;

	clk_parent = clk_get(NULL,"PLL1") ;
	temp = clk_set_parent(clk, clk_parent) ;
	printk("========= Completed setting new parent, ret val is %d \n", temp ) ;

	clk_parent = clk_get_parent(clk) ;
	printk("========= CRPT parent name is %s \n", clk_parent->name ) ;

    clk = clk_get(NULL, "CRPT" ) ;
	clk_disable(clk) ;
	printk("========= Disabled CRPT \n") ;

    clk = clk_get(NULL, "CRPT" ) ;
	clk_enable(clk) ;
	printk("========= Enabled CRPT \n") ;
}

