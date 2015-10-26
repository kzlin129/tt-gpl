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
 *   @file   emi.c 
 * 
 *   @brief  Emi apis
 * 
 ****************************************************************************/

/* These should come from a top-level include file. Until then... */
#define UINT32_MAX	0xFFFFFFFF
#define UINT16_MAX	0xFFFF
#define UINT8_MAX	0xFF
//#define UINT_MAX    0xFFFFFFFF


//Uncomment for SDR
//#define BCM_USING_SDR


#include "bcm_basetypes.h"
#include "bcm_privtypes.h"
#include "bcm_basedefs.h"
#include "bcm_log.h"
#include "bcm_amba_io.h"
#include "bcm_divide.h"
#include "emi.h"
#include "emi_regs.h"
#include <asm-arm/arch-bcm4760/bcm4760_addressmap.h>

static emi_regs_t *pemi_base_addr;
static uint32_t		g_emi_frequency = 0;
static uint32_t		g_ref_cntrl_value=0;
static uint32_t		g_stored_read_ncdl_value;
static uint32_t		g_stored_write_ncdl_value;
static int			g_are_ncdl_values_stored;
static emi_dram_mode_t	g_dram_mode=EEMI_MODE_1by1;


static void wait(uint32_t emi_clock_count)
{
	// TODO:
	// This function waits (blocking) the specified number of emi
	// clock cycles.
	// Currently, we do not really have a way to implement this
	// wait, so we will simply increment a counter.
	// This needs to get replaced by a better implementation.

	volatile unsigned int i=100000*emi_clock_count;
	while(--i);
}

// pull memory controller out of reset
static void dram_unreset(void)
{
	amba_reg_write32(pemi_base_addr,  emi_emi_soft_reset, 0x0);
}

static void set_client_priorities(void)
{
	#define EMI_PREFERRED_MODE 0x1f
	#define EMI_NONPREFERRED_MODE 0x0
	#define EMI_TO_DCACHE   32
	#define EMI_TO_ICACHE   32 
	#define EMI_TO_NONCACHE 128
	#define EMI_TO_ARM      16
	#define EMI_TO_CONTROL  64

	amba_reg_write32(pemi_base_addr,  emi_client_preferred_mode_enable,  EMI_PREFERRED_MODE) ;
	amba_reg_write32(pemi_base_addr,  emi_non_pref_client_req_disable ,  EMI_NONPREFERRED_MODE) ;

	amba_reg_write32(pemi_base_addr,  emi_client_access_timeout_period[0], EMI_TO_DCACHE); //dcache
	amba_reg_write32(pemi_base_addr,  emi_client_access_timeout_period[1], EMI_TO_ICACHE); //icache
	amba_reg_write32(pemi_base_addr,  emi_client_access_timeout_period[2], EMI_TO_NONCACHE); //noncache
	amba_reg_write32(pemi_base_addr,  emi_client_access_timeout_period[3], EMI_TO_ARM); //arm
	amba_reg_write32(pemi_base_addr,  emi_client_access_timeout_period[4], EMI_TO_CONTROL); //control
}

// reset the EMI logic
static void dram_reset(void)
{
	#define EMI_SEQ_START_N 0x0

	// disable the sequencer
	amba_reg_write32(pemi_base_addr,  emi_cntrlr_start_seq, EMI_SEQ_START_N);

	// do the actual reset
	amba_reg_write32(pemi_base_addr,  emi_emi_soft_reset, 0x3);
	amba_reg_read32(pemi_base_addr,  emi_soft_commands_client);
}

static void calc_and_remember_ref_cntrl(void)
{
	uint8_t		refresh_multiplier = 1;
	uint8_t		ref_clust = 0;
	uint32_t	ref_period;

	#ifdef BCM_USING_SDR
		#define REFRESH_CYCLE_US	14
	#else
		#define REFRESH_CYCLE_US	7
	#endif

	BCM_ASSERT(g_emi_frequency != 0);

	#define MAX_REF_PERIOD 0xFFFF // bits15:0
	#define MIN_REF_PERIOD 1 // bits15:0

	// SDRAM should be refreshed once every refresh_cycle_us micro-seconds or more frequently.
	//
	// Therefore, the following should hold true:
	// actual_refresh_period <= (refresh_cycle_us/1000000)
	// 
	// which is the same as:
	// ((mtx_period * ref_period) / refresh_multiplier) <= (refresh_cycle_us/1000000)
	
	// which is the same as:
	// (ref_period / (refresh_multiplier * emi_frequency)) <= (refresh_cycle_us/1000000);

	// which is the same as:
	// (refresh_cycle_us * emi_frequency * refresh_multiplier) >= ref_period * 1000000

	
	// First, figure out what multiplier to use. Note that the maximum multiplier
	// we can use is 8 (or ref_clust value of 3)
	while( ((uint64_t)REFRESH_CYCLE_US * g_emi_frequency * refresh_multiplier)  
										< MIN_REF_PERIOD * 1000000)
	{
		refresh_multiplier *= 2;
		ref_clust += 1;

		if( ref_clust > 3 )
		{
			//TODO:
			//FATAL ERROR
			//cldb_display_fatal_error("emi_frequency (%llu) is too low to refresh SDRAM at %uus or less\n", 
			//		(unsigned long long)emi_frequency,
			//		(unsigned int)refresh_cycle_us);
		}
	}

	//now, compute the ref_period.
	ref_period = bcm_udivide64(((uint64_t)g_emi_frequency * 
						REFRESH_CYCLE_US * refresh_multiplier), 1000000); 

	if( ref_period > MAX_REF_PERIOD )
	{
		//TODO:
		//FATAL ERROR
		//cldb_display_fatal_error("REF_PERIOD (bit15:00 of EMI_REFRESH_CNTRL) needed is too high %u (0x%x)\n", 
		//			(unsigned int)ref_period, (unsigned int)ref_period);
	}

	if( ref_period < MIN_REF_PERIOD )
	{
		//TODO:
		//FATAL ERROR
		//cldb_display_fatal_error("REF_PERIOD (bit15:00 of EMI_REFRESH_CNTRL) needed is too low %u (0x%x)\n", 
		//			(unsigned int)ref_period, (unsigned int)ref_period);
	}

	//cldb_display_info("EMI_REFRESH_CNTRL=0x%x (mtx=%llu, period=%u, multiplier=%u, actual_refresh_period=%u us)\n", 
	//					(unsigned int)(((ref_clust & 0x3) << 16)| (ref_period & 0xFFFF)),
	//					(unsigned long long)emi_frequency, 
	//					(unsigned int)ref_period,
	//					(unsigned int)refresh_multiplier,
	//					(unsigned int) (ref_period * 1000000LL / (refresh_multiplier * emi_frequency))
	//					);

	// store it in a global variable
	g_ref_cntrl_value =  ((ref_clust & 0x3) << 16)| (ref_period & 0xFFFF);
}


//SSO test data structure
//used by SHMOO
static uint64_t sso[32] = {
  0xFFFFFFFE00000001ULL,
  0xFFFFFFFD00000002ULL,
  0xFFFFFFFB00000004ULL,
  0xFFFFFFF700000008ULL,

  0xFFFFFFEF00000010ULL,
  0xFFFFFFDF00000020ULL,
  0xFFFFFFBF00000040ULL,
  0xFFFFFF7F00000080ULL,

  0xFFFFFEFF00000100ULL,
  0xFFFFFDFF00000200ULL,
  0xFFFFFBFF00000400ULL,
  0xFFFFF7FF00000800ULL,

  0xFFFFEFFF00001000ULL,
  0xFFFFDFFF00002000ULL,
  0xFFFFBFFF00004000ULL,
  0xFFFF7FFF00008000ULL,

  0xFFFEFFFF00010000ULL,
  0xFFFDFFFF00020000ULL,
  0xFFFBFFFF00040000ULL,
  0xFFF7FFFF00080000ULL,

  0xFFEFFFFF00100000ULL,
  0xFFDFFFFF00200000ULL,
  0xFFBFFFFF00400000ULL,
  0xFF7FFFFF00800000ULL,

  0xFEFFFFFF01000000ULL,
  0xFDFFFFFF02000000ULL,
  0xFBFFFFFF04000000ULL,
  0xF7FFFFFF08000000ULL,

  0xEFFFFFFF10000000ULL,
  0xDFFFFFFF20000000ULL,
  0xBFFFFFFF40000000ULL,
  0x7FFFFFFF80000000ULL,
};

// Test if the memory is readable and can write back correctly
// returns FALSE on failure, TRUE on success
static int test_dram_access(uint32_t addr, uint32_t r1, uint32_t r2)
{
    int i;
    volatile uint64_t *y = (volatile uint64_t *)addr;

    y += 0x1c;
    for (i = 0; i < 32; i++) 
    {
		amba_write64( y++, sso[i]);
    }

    for (i = 0; i < 32; i++) 
    {
		amba_write64( y, sso[i]);
		if (amba_read64( y) != sso[i]) 
		{
			//bcm_log_info("sso failed %d read %llx, %llx\n", i, rd64(y), sso[31-i]);
			return FALSE;
		}
    }

    for (i=0; i<32; i++) 
    {
        --y;
        if (amba_read64( y) != sso[31-i]) 
        {
            //bcm_log_info("sso failed %d read %llx, %llx\n", i, rd64(y), sso[31-i]);
            return FALSE;
        }
	}

	return TRUE;
}


//test if the ncdl will pass the memory test
// Note: In the case of SDR, it is not really an "ncdl" but is a simple delay
static int is_ncdl_good(uint32_t read_ncdl, uint32_t write_ncdl)
{
    uint32_t addr;

	read_ncdl &= 0xFF;
	read_ncdl &= 0xFF;

	#ifdef BCM_USING_SDR
		write_ncdl = (write_ncdl & 0xFF00) >> 8;
	#else
		write_ncdl &= 0xFF;
	#endif

	// NOTE: The actual addres range is arbitrarily chosen
	// Basically, we are trying to trade off between testing
	// a larger range (slower) vs a faster boot
	for (addr=0xc0000000; addr < 0xc0006000; addr += 0x1000)
	{
		if (!test_dram_access(addr, read_ncdl, write_ncdl)) 
		{
  			return FALSE; // memory access failed
		}
	}

    return TRUE;
}


static void set_ncdl(uint32_t  read_ncdl, uint32_t  write_ncdl)
{
	#ifdef BCM_USING_SDR
		amba_reg_write32(pemi_base_addr, emi_sdr_read_reclock, read_ncdl);
	#else
		amba_reg_write32(pemi_base_addr,  emi_ddr_read_ncdl_offset, read_ncdl);
		amba_reg_write32(pemi_base_addr,  emi_ddr_write_ncdl_offset, write_ncdl);
	#endif
}

static	void set_ref_cntrl_register(void)
{
	amba_reg_write32(pemi_base_addr, emi_refresh_cntrl, g_ref_cntrl_value);
}

static void set_dram_mode(void)
{
	// In theory, should have set to EMI_TRANSFER_MODE=1 for "1:1" mode
	// and to 0x2 for other modes. (based on g_dram_mode)
    // But because of H/W issue (#42) in the A0, the safe thing 
	// (since we are doing dynamic mode changes) to do is to always 
	// set this register to 0x2.
   amba_reg_write32( pemi_base_addr,  emi_transfer_mode, 0x2); 
}

//NOTE: the read_ncdl and write_ncdl are not used in case of DDR
// They are only needed for SDR, where we write the delay during the
// initialization phase.
static void dram_init(uint32_t read_ncdl, uint32_t write_ncdl)
{
	#ifdef BCM_USING_SDR
		#define EMI_CNTRLR_CONFIG_VALUE	0x790
		#define EMI_DRAM_TIMING0		0x4434
		#define EMI_DRAM_TIMING1		0x14232708
	#else
		#define EMI_CNTRLR_CONFIG_VALUE	0x791
		#define EMI_DRAM_TIMING0		0x4434
		#define EMI_DRAM_TIMING1		0x1934490f
	#endif

	#define EMI_DRAM_MODE_SET_VALUE		0x1003
	#define EMI_PAD_CNTL_VALUE			0x107f0
	#define EMI_CNTRLR_START_SEQ_VALUE	0x1


	amba_reg_write32( pemi_base_addr,  emi_cntrlr_config, EMI_CNTRLR_CONFIG_VALUE);
    dram_unreset();
    amba_reg_write32( pemi_base_addr,  emi_dram_mode_set, EMI_DRAM_MODE_SET_VALUE);
    amba_reg_write32( pemi_base_addr,  emi_dram_timing0,  EMI_DRAM_TIMING0);
    amba_reg_write32( pemi_base_addr,  emi_dram_timing1,  EMI_DRAM_TIMING1);

#ifdef BCM_USING_SDR
	set_dram_mode();

    // do we need to change the PAD_CNTRL?
    amba_reg_write32( pemi_base_addr,  emi_pad_cntl, EMI_PAD_CNTL_VALUE); 

    set_client_priorities();

    // SDR relock
    amba_reg_write32( pemi_base_addr,  emi_sdr_read_reclock, read_ncdl); 
    // enables the "sequencer"
	amba_reg_write32(pemi_base_addr, emi_cntrlr_start_seq, EMI_CNTRLR_START_SEQ_VALUE);

	//TODO:	// why do we do it at different places for SDR vs DDR?
	set_ref_cntrl_register();
#else
	//TODO:	why do we do it at different places for SDR vs DDR?
	set_ref_cntrl_register();

	set_dram_mode();

    // TODO: do we need to change the PAD_CNTRL?
    amba_reg_write32( pemi_base_addr,  emi_pad_cntl, EMI_PAD_CNTL_VALUE); 

    set_client_priorities();

    // enables the "sequencer"
	amba_reg_write32(pemi_base_addr, emi_cntrlr_start_seq, EMI_CNTRLR_START_SEQ_VALUE);

    // DQS fine control
	amba_reg_write32( pemi_base_addr,  emi_ddr_rddqs_gate_cntrl, 0x1);

	// hardware recommended: keep dll_relock and toggling reset
	// use the EMI clock instead of the DQS clock
    amba_reg_write32( pemi_base_addr,  emi_ddr_dll_mode, 0x11);
    amba_reg_write32( pemi_base_addr,  emi_ddr_dll_mode, 0x10); 

	BCM_ASSERT(g_emi_frequency != 0);
	emi_wait_for_dll_phase_lock(g_emi_frequency);

#endif
}

static void generate_auto_refresh_cycle(void)
{

	#define EMI_REF_DELAY 50

	// Issue a "refresh" command by writing a "1" 
	// causes the memory controller to generate an auto refresh cycle to the SDRAM
	amba_reg_write32( pemi_base_addr,  emi_soft_commands_client, 0x1);

	#ifndef BCM_USING_SDR
	//TODO: 
	//as per sathish (conference call on 6/22/06), we need to wait
	// N emi clock cycles
	// - if SDRAM is "idle" (this function is called from 2nd stage
	//   boot, so it will be "idle"), then N is 50 
	// - if SDRAM is not idle, i.e. clients are accessing SDRAM,
	//   then N is 300
	// Presently, we do not have an API to wait. So we will simply
	// increment an integer 1000000 times
	wait(50);
	#endif


	// write a "0" to cause the refresh command to complete
	amba_reg_write32( pemi_base_addr,  emi_soft_commands_client, 0x0);
}



static void write_ddr_ncdl_values(uint32_t read_ncdl, uint32_t write_ncdl)
{
	dram_reset();
	dram_init(0, 0);
	set_ncdl(read_ncdl, write_ncdl);
	generate_auto_refresh_cycle();
}

// calculate the ncdl value for read and write ncdls.
// Note: Assume NCDL values are same for all byte lanes.
// (to compute NCDL values for each byte lane would take too long)
// return TRUE on success, FALSE on failure
static int calc_and_set_ddr_ncdl(void)
{
	//starting values
	uint32_t		emi_ddr_rd_ncdl=0x30303030;  
	uint32_t		emi_ddr_wr_ncdl=0x1f1f1f1f;
    int				rd, wr, rdt=0, wrt=0, passcnt= 0;
    volatile int	pass;

    uint32_t		dll;
	int				dll_low=0xffff, dll_high=0;

	// read the DLL lock value a few times, and note the high
	// and low values being read
    for (rd = 0; rd < 200; rd++) 
    {
        dll = amba_reg_read32( pemi_base_addr, emi_ddr_dll_phase_filtered);
        if ((uint32_t)dll_low > dll) 
        {
			dll_low = dll;
        }

        if ((uint32_t)dll_high < dll) 
        {
			dll_high = dll;
        }
    }

	//Note (clarify from Sathish):
	// for bringup, we are incrementing rd and wr by "1"
	// instead of incrementing by "((dll+15) >> 4)"
	// Why?

	for (rd= 0-dll_low; rd <= 63-dll_high; ++rd) 
	{
		emi_ddr_rd_ncdl = (rd & 0x7f) | ((rd & 0x7f)<<8) | ((rd & 0x7f)<<16) | ((rd & 0x7f)<<24);
		for (wr= (0-dll_low); wr <= (63-dll_high); ++wr) 
		{
			emi_ddr_wr_ncdl = (wr & 0x7f) | ((wr & 0x7f)<<8) | ((wr & 0x7f)<<16) | ((wr & 0x7f)<<24);
			write_ddr_ncdl_values(emi_ddr_rd_ncdl, emi_ddr_wr_ncdl);

			// bcm_log_info("dll is %d lock=%d rd=%d:0x%x wr=%d:0x%x\n", amba_reg_read32( pemi_base_addr, dll_filtered),amba_reg_read32( pemi_base_addr, dll_lock), rd, rd,  wr, wr);
			pass = is_ncdl_good(emi_ddr_rd_ncdl, emi_ddr_wr_ncdl);

			if (pass) 
			{
  				++passcnt;
				rdt += rd;
  				wrt += wr;
	        }
		}
    }

    if (passcnt)
    {
		// At least some (1 or more) rd and wr values "passed" the memory test
		// find the "average" of passing rd and wr values.
		// that's the best we can do.
        rd = bcm_udivide32(rdt,  passcnt);
        wr = bcm_udivide32(wrt,  passcnt);

        emi_ddr_rd_ncdl = (rd & 0x7f) | ((rd & 0x7f)<<8) | ((rd & 0x7f)<<16) | ((rd & 0x7f)<<24);
        emi_ddr_wr_ncdl = (wr & 0x7f) | ((wr & 0x7f)<<8) | ((wr & 0x7f)<<16) | ((wr & 0x7f)<<24);
        write_ddr_ncdl_values(emi_ddr_rd_ncdl, emi_ddr_wr_ncdl);
        pass = is_ncdl_good(emi_ddr_rd_ncdl, emi_ddr_wr_ncdl);

        if (!pass) 
        {
			// Error: The "average" values did not seem to work
            emi_ddr_rd_ncdl=emi_ddr_wr_ncdl=0;
            write_ddr_ncdl_values(emi_ddr_rd_ncdl, emi_ddr_wr_ncdl);
            return FALSE; 
        }
    } 
    else 
    {
		// Error: not a single attempted NCDL value passed.
        emi_ddr_rd_ncdl=emi_ddr_wr_ncdl=0;
        write_ddr_ncdl_values(emi_ddr_rd_ncdl, emi_ddr_wr_ncdl);
        return FALSE; 
    }

	g_stored_read_ncdl_value = emi_ddr_rd_ncdl;
	g_stored_write_ncdl_value = emi_ddr_wr_ncdl;
	g_are_ncdl_values_stored = TRUE;
    return TRUE;
}

#ifdef BCM_USING_SDR
//SHMOO for SDR
static int calc_sdr_ncdl(uint32_t *read_ncdl, uint32_t *write_ncdl)
{

    int				i, j, k=0, cnt=0;
    int				passcnt= 0;
    volatile int	pass;
	uint32_t		sdr_delay=1; //start from a "seed" value of "1"

	generate_auto_refresh_cycle();

    for (i=0; i<3; i++) 
    {  
		for (j=0; j<0x40; j++) 
        {
			if (i==0) 
			{
				sdr_delay = (j<<4) | 1 | (k<<11) | (1<<10);
			}
			else if (i==1) 
			{
				sdr_delay = (j<<4) | 3;
			}
			else 
			{
				sdr_delay = (j<<4) ;
			}

			dram_reset();
			dram_init(sdr_delay, sdr_delay);

			pass = is_ncdl_good(sdr_delay, sdr_delay);

			if (pass) 
			{
				passcnt ++;
				cnt += j;
			}
        }


		if (passcnt > 8) 
		{
			j = cnt / passcnt;

			if (i==0) 
			{
				sdr_delay = (j<<4) | 1;
			}
			else if (i==1) 
			{
				sdr_delay = (j<<4) | 3;
			}
			else
			{
				sdr_delay = (j<<4);      
			}

			dram_reset();
			dram_init(sdr_delay, sdr_delay);
			pass = is_ncdl_good(sdr_delay, sdr_delay);

			if (pass) 
			{
				*read_ncdl = *write_ncdl = sdr_delay;
				return TRUE;
			} 
        }

        passcnt = 0;
        cnt = 0;
    } 

	*read_ncdl = *write_ncdl = 1; 
	return FALSE; 
}
#endif /* ifdef BCM_USING_SDR */

static void remember_emi_frequency(uint32_t emi_frequency)
{
	g_emi_frequency = emi_frequency;
}

// here is the formula (simple linear relationship)
// emi_frequency = A + B*dll_phase
// or 
// dll_phase = (emi_frequency - A )/B
//
// Where "A" and "B" are constants.
// In our software, we will build up a history of working dll_phase
// values given and known emi_frequency, and we will calculate
// the "A" and "B" once we have gotten two or more values.
//
// Therefore:
//
// B = (emi_frequency_high - emi_frequency_low)/(dll_phase_high - dll_phase_low)
//
// and 
//
// A = emi_frequency_low - B*dll_phase_low
static uint32_t B=UINT32_MAX;
static uint32_t A;

static uint32_t lookup_dll_phase(uint32_t emi_frequency)
{
	BCM_ASSERT(B!=0);
	return bcm_udivide32((emi_frequency - A), B);
}

static void update_dll_phase_lookup_table(uint32_t emi_frequency, uint32_t dll_phase)
{
	static uint32_t emi_frequency_low=UINT32_MAX;
	static uint32_t emi_frequency_high;
	static uint32_t dll_phase_low=UINT32_MAX;
	static uint32_t dll_phase_high;
	int				update_needed = FALSE;

	// update the lookup table only if the current
	// frequency is significantly lower than the low value stored
	// in the table
	//   OR
	// the current frequency is significantly higher
	// then the high value stored in the table
	//
	// This way, the two values stored in the table will be
	// "widely-enough" placed, there-by producing better accuracy for 
	// "A" and "B"
	if( emi_frequency < emi_frequency_low >> 1)
	{
		emi_frequency_low = emi_frequency;
		dll_phase_low = dll_phase;
		update_needed = TRUE;
	}
	else if( emi_frequency > (emi_frequency_high*2)) 
	{
		emi_frequency_high = emi_frequency;
		dll_phase_high = dll_phase;
		update_needed = TRUE;
	}


	if( update_needed )
	{
		BCM_ASSERT((dll_phase_high - dll_phase_low) != 0)
		B = bcm_udivide32((emi_frequency_high - emi_frequency_low), 
							(dll_phase_high - dll_phase_low) );
		A = emi_frequency_low * B*dll_phase_low;
	}
}

static void	remember_dram_mode(emi_dram_mode_t dram_mode)
{
	g_dram_mode = dram_mode;
}

// API initialization
void emi_api_init(void)
{
	BCM_ASSERT(pemi_base_addr == NULL);
	pemi_base_addr = (emi_regs_t *)bcm_xlate_to_virt_addr(DDR_REG_BASE_ADDR);
}



void emi_set_ncdl(uint32_t read_ncdl, uint32_t write_ncdl)
{
	BCM_ASSERT(pemi_base_addr != NULL);

	#ifdef BCM_USING_SDR
		// On SDR, the two values should be the same
		BCM_ASSERT(read_ncdl==write_ncdl);
	#endif

	g_stored_read_ncdl_value = read_ncdl;
	g_stored_write_ncdl_value = write_ncdl;
	g_are_ncdl_values_stored = TRUE;
}

void emi_dram_init(uint32_t emi_frequency, emi_dram_mode_t dram_mode)
{
	uint32_t	read_ncdl=0, write_ncdl=0;

	BCM_ASSERT(pemi_base_addr != NULL);

	remember_emi_frequency(emi_frequency);
	calc_and_remember_ref_cntrl();
	remember_dram_mode(dram_mode);

	if( g_are_ncdl_values_stored )
	{
		read_ncdl = g_stored_read_ncdl_value;
		write_ncdl = g_stored_write_ncdl_value;
	}

	
	//TODO:
	//for now, we always run SHMOO.
	// This is inefficient. Later, we will switch to (optionally)
	// using precalculated NCDL values

#ifdef BCM_USING_SDR
	// in case of SDR, do SHMOO followed by init
	if(!g_are_ncdl_values_stored )
	{
		calc_sdr_ncdl(&read_ncdl, &write_ncdl);

		g_stored_read_ncdl_value = read_ncdl;
		g_stored_write_ncdl_value = write_ncdl;
		g_are_ncdl_values_stored = TRUE;

	}

	dram_init(read_ncdl, write_ncdl);
#else
	// in case of DDR, do init followed by SHMOO
	dram_init(0, 0); 
	if( g_are_ncdl_values_stored )
	{
		// (which does a whole bunch of other stuff) or can we
		// simply call set_ncdl()?
		write_ddr_ncdl_values(read_ncdl, write_ncdl);
	}
	else
	{
		// need to run SHMOO and set NCDL values
		calc_and_set_ddr_ncdl();
	}
#endif
}


void emi_get_ncdl(uint32_t * read_ncdl, uint32_t * write_ncdl)
{
	BCM_ASSERT(pemi_base_addr != NULL);

	#ifdef BCM_USING_SDR
	*read_ncdl = *write_ncdl = amba_reg_read32(pemi_base_addr, emi_sdr_read_reclock);
	#else
	*read_ncdl = amba_reg_read32(pemi_base_addr,  emi_ddr_read_ncdl_offset);
	*write_ncdl = amba_reg_read32(pemi_base_addr,  emi_ddr_write_ncdl_offset);
	#endif
}


void emi_change_frequency(uint32_t emi_frequency)
{
	BCM_ASSERT(pemi_base_addr != NULL);

	remember_emi_frequency(emi_frequency);

	calc_and_remember_ref_cntrl();
	set_ref_cntrl_register();
}

void emi_setup_dll_phase_load_value(uint32_t emi_frequency)
{
	// This routine will write to the EMI_DDR_DLL_PHASE_LOAD_VALUE register
	// This register serves as a "hint" for the DLL locking mechanism
	// The idea is to build up a table of 2 hint values, chosen wisely.
	// (There is a liner relationship between "emi_frequency" and a valid 
	// "hint".
	// The value of a valid hint can be read back from the EMI_DLL_PHASE_FILTERED
	// register if and when the DLL has locked to the new emi_frequency

	uint32_t dll_load_value;

	BCM_ASSERT(pemi_base_addr);

	//Note: this function does not update the stored mtx
	//frequency. That update is done from the "change_frequency" API...
	dll_load_value = lookup_dll_phase(emi_frequency);
	
	BCM_ASSERT(dll_load_value <= 0x3F ); /* bit5:0 */

	amba_reg_write32( pemi_base_addr, emi_ddr_dll_phase_load_value, dll_load_value);
}

void emi_wait_for_dll_phase_lock(uint32_t emi_frequency)
{
	uint32_t							lock_value_max=0;
	uint32_t							lock_value_min=UINT32_MAX;
	emi_ddr_dll_phase_u_t		phase_filtered;
	emi_ddr_dll_phase_u_t		phase_high;
	emi_ddr_dll_phase_u_t		phase_low;
	int									i;
  
	BCM_ASSERT(pemi_base_addr != NULL);
	BCM_ASSERT(g_emi_frequency != 0);
	BCM_ASSERT(g_emi_frequency != emi_frequency);

	// The normal way to check for DLL lock is to check to see 
	// if EMI_DDR_DLL_LOCK_BIT:dll_lock is 0x1
	// However, in "A0" this logic does not work. See H/W issue #48
	// Here is the workaround suggested by sathish via email(6/21/06)
	// and during conference call on 6/21/06
	//    "For A0 please do not even look at the lock bit....so 
	//    the dll_locked is TRUE under two conditions:
    //
    //    1. When saturation occurs, DLL has locked.
	//       If saturation has not occured, then...
	//           
    //    2. When the dll lock value continously hovers around a 
	//       particular tap : ie max_tap - min_tap < say 3 taps, 
	//       for the last 100 samples."
	//       Wait 100 EMI clock cycles between each read of a
	//		 sample.

	// check to see if saturation has occured
	phase_low.w_data = amba_reg_read32(pemi_base_addr, emi_ddr_dll_phase_low);
	phase_high.w_data = amba_reg_read32(pemi_base_addr, emi_ddr_dll_phase_high);

	if( phase_low.bf.lock_val == 0x3f && phase_high.bf.lock_val == 0x3f )
	{
		// we have saturation
		// succcessful DLL lock. Update the lookup table if needed
		update_dll_phase_lookup_table(emi_frequency, lock_value_min);
		return; // Success: DLL has locked.
	}

	// read the lock value 100 times and calculate the maximum and the minimum
	for(i=0; i< 100; ++i)
	{
		phase_filtered.w_data = amba_reg_read32(pemi_base_addr, emi_ddr_dll_phase_filtered);

		//TODO: for some reason, the "max" macro does not work with bit-fields...
		if( phase_filtered.bf.lock_val > lock_value_max) 
			lock_value_max = phase_filtered.bf.lock_val;

		if( phase_filtered.bf.lock_val < lock_value_min )
			lock_value_min = phase_filtered.bf.lock_val;


		//need to wait "100" emi clock cycles in between each read of
		//a sample.
		wait(100);
	}

	if( lock_value_max - lock_value_min < 3 )
	{
		// if the phase_filtered value is "stable", then
		// we have saturation
		// succcessful DLL lock. Update the lookup table if needed
		update_dll_phase_lookup_table(emi_frequency, lock_value_min);
		return; // Success: DLL has locked.
	}

	//ERROR: DLL has not locked.
	//TODO: report error
}


void emi_set_dram_mode(emi_dram_mode_t dram_mode)
{
	remember_dram_mode(dram_mode);
	set_dram_mode();
}

void emi_self_refresh_power_down_set(void)
{
    emi_pdown_mode_u_t pd_reg ;

    pd_reg.w_data = amba_reg_read32(pemi_base_addr, emi_power_down_mode);
    pd_reg.bf.pdn_enter_mode = 2 ;
    amba_reg_write32(pemi_base_addr,  emi_power_down_mode, pd_reg.w_data);
}


void emi_self_refresh_power_down_unset(void)
{
    emi_pdown_mode_u_t pd_reg ;

    pd_reg.w_data = amba_reg_read32(pemi_base_addr, emi_power_down_mode);
    pd_reg.bf.pdn_enter_mode = 0 ;
    amba_reg_write32(pemi_base_addr,  emi_power_down_mode, pd_reg.w_data);
}
