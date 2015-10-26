/*****************************************************************************
* Copyright 2007 - 2008 Broadcom Corporation.  All rights reserved.
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



/*****************************************************************************
*
* This file contains code which may be shared between the boot2 bootloader
* and the VideoCore driver.
*
* When required, use #if defined( __KERNEL__ ) to tell if the code is being
* compiled as part of the kernel or not.
*
*****************************************************************************/

#include <linux/broadcom/vc.h>
#include <linux/elf.h>

#include "vcinterface.h"
#include "vciface.h"

#if defined( __KERNEL__ )
#   include <linux/vmalloc.h>
#   include <linux/slab.h>
#   include <linux/kernel.h>
#   include <linux/broadcom/knllog.h>
#   include <linux/broadcom/gpio.h>
#   include <linux/crc32.h>
#   include <linux/time.h>
#   include "vchost_config.h"

#   define  PRINT_ERR(  fmt, args... )  printk( KERN_ERR  fmt, ## args )
#   define  PRINT_INFO( fmt, args... )  printk( KERN_INFO fmt, ## args )
#else
#   include <kerneltypes.h>
#   include <linux/broadcom/gpio.h>
#   undef __KERNEL__    // kerneltypes.h defines this and it confuses us
#   include <printf.h>
#   include <crc32.h>
#   include <string.h>

#   define  PRINT_ERR(  fmt, args... )  printf( fmt, ## args )
#   define  PRINT_INFO( fmt, args... )  printf( fmt, ## args )

#define vc_assert(expr) \
    if ( expr ) {} else \
    { \
        printf( "Assertion (%s) failed: FILE: %s LINE: %d\n", #expr, __FILE__, __LINE__ ); \
        while(1) {;} \
    }

#endif
#include <linux/broadcom/hw_cfg.h>
#include <linux/broadcom/vchost-port.h>

/*
 * The RLED and WLED values were tweaked by running video call
 * and flash writes at the same time on 1161VP.
 *
 * It's possible that different platforms will require different values,
 * but these are reasonable defaults.
 */

#if !defined( HW_CFG_VC02_HOST_PORT_RLED )
#  define     HW_CFG_VC02_HOST_PORT_RLED     1
#endif

#if !defined( HW_CFG_VC02_HOST_PORT_WLED )
#  define     HW_CFG_VC02_HOST_PORT_WLED     1
#endif

int   gVcHostPortRLED = HW_CFG_VC02_HOST_PORT_RLED;
int   gVcHostPortWLED = HW_CFG_VC02_HOST_PORT_WLED;

#if (CFG_GLOBAL_CPU == MIPS32)
#include <asm/broadcom/bcm1103/bcm1103.h>
#if !defined( HW_CFG_VC02_EBI_WTST_VAL )
#  define     HW_CFG_VC02_EBI_WTST_VAL    1
#endif

#if !defined( HW_CFG_VC02_EBI_SETUP_VAL )
#  define     HW_CFG_VC02_EBI_SETUP_VAL   0
#endif

#if !defined( HW_CFG_VC02_EBI_HOLD_VAL )
#  define     HW_CFG_VC02_EBI_HOLD_VAL    0
#endif

#endif  // (CFG_GLOBAL_CPU == MIPS32)

// Once we've initialized the VC02, we then figure out if we're in the
// "standard" mode (A1,2,3) or A6,7,8. If we're in A6,7,8 mode then we can
// use 32-bit writes.

#if VC_HOST_PORT_USE_DMA
int     gVcUseDma           = 1;
#endif
int     gVcUseFastXfer      = 1;
int     gVcUse32bitIO       = 0;
int     gVc32BitIOAvail     = 0;
char    gVcAddrModeStr[ 20 ];

#if VC_DEBUG_ENABLED
int     gVcDebugRead        = 0;
int     gVcDebugWrite       = 0;
int     gVcDebugTrace       = 0;
int     gVcDebugMsgFifo     = 0;
int     gVcProfileTransfers = 0;
#endif

// The normal connection for the VC02 is that A1, A2, and A3 from the host
// are connected up to the host port interface. This means that register
// 0 is at offset 0, and register 1 is at offset 2.
//
// There is an alternative way of conencting stuff up such that A6, 7, 8 are
// used instead, which gives each register a 64 byte area. This allows 32-bit
// accesses to be used and causes each 16-bits of the 32-bit write to goto
// the same host port register.
//
// For initialization, we use the address (reg * 2 ) + ( reg * 64 ) which is
// the same thing as (reg) * 66. This is a safe address to use and will work
// for either addressing scheme, provided you're only doing 16-bit accesses.

typedef uint16_t VC_Register_t;

// Channel 0 occupies registers 0 thru 3.
// Channel 1 occupied registers 4 thru 7.

#define  DATA     0
#define  WBASE    1
#define  RBASE    2
#define  HCS      3

#define  VC_REG_16( reg, chan )   (*((volatile uint16_t *)( REG_VCO2_BASE + (( (reg) + ( chan * 4 )) * 66 ))))

/* Channel 0 */
#define VCDATA0     VC_REG_16( DATA, 0 )
#define VCWBASE0    VC_REG_16( WBASE, 0 )
#define VCRBASE0    VC_REG_16( RBASE, 0 )
#define VCHCS0      VC_REG_16( HCS, 0 )
/* Channel 1 */
#define VCDATA1     VC_REG_16( DATA, 1 )
#define VCWBASE1    VC_REG_16( WBASE, 1 )
#define VCRBASE1    VC_REG_16( RBASE, 1 )
#define VCHCS1      VC_REG_16( HCS, 1 )

/******************************************************************************
Local types and defines.
******************************************************************************/

// locations in the HCS (for writes, upper nibble)
#define WTW     (0x0<<4)
#define RTW     (0x1<<4)
#define RMODE   (0x2<<4)
#define INT     (0x3<<4)
#define IWFW    (0x4<<4)
#define IRFR    (0x5<<4)
#define WTHR    (0x6<<4)
#define RTHR    (0x7<<4)
#define BW      (0x8<<4)
#define HRM     (0x9<<4)
#define RLED    (0xa<<4)
#define WLED    (0xb<<4)
#define HRDY_INV (0xc<<4)

// bitflags for HCS reads
#define WFE     (1<<0)
#define WFD     (1<<1)
#define WFW     (1<<2)
#define RFF     (1<<3)
#define RFD     (1<<4)
#define RFR     (1<<5)
#define RSTN    (1<<6)
#define BP      (1<<7)

// various useful constants for writing to the HCS registers
#define	HARDWARE_READY  1
#define SOFTWARE_READY  0

#define BUS_16_BIT      1
#define BUS_8_BIT       0

#define TRANSFER_32_BIT 2
#define TRANSFER_16_BIT 1
#define TRANSFER_8_BIT  0

#define THRS_NOT_EMPTY  0
#define THRS_EMPTY      0
#define THRS_QTR_FULL   1
#define THRS_HALF_FULL  2
#define THRS_3QTR_FULL  3
#define THRS_FULL       4
#define THRS_NOT_FULL   4

#define RMODE_NOINC     0
#define RMODE_AUTOINC   1
#define RMODE_FIFO      2

#define TRIGGER_INT     1
#define BOOTCHK         1     // comment out if you don't want boot check

// Address where bootloader code should be loaded
#define VC_BOOTLDR_ADDRESS	0xFC100

// Address where command line string is loaded to (assume that arguments
// follow at 16 bytes later
#define VC_CMDLINE_ADDRESS	0xFC000

// Address to where fixup should be applied
#define VC_FIXUP_ADDRESS		0x200

/****************************************************************************
*
*   Variable/s & types used to track CRC's in the code section of the
*   VideoCore.
*
***************************************************************************/

typedef struct
{
   uint32_t addr;
   size_t   len;
   uint32_t crc;

} CRC_Section_t;

#define  NUM_CRC_SECTIONS   10
static   CRC_Section_t  gCrcSection[ NUM_CRC_SECTIONS ];
static   int            gNumCrcSections;

void        vc_host_init_pins( void );
void        vc_host_set_run_pin( int arg );
void        vc_host_set_hibernate_pin( int arg );
void        vc_host_set_reset_pin( int arg );
void        vc_host_reset( void );

/****************************************************************************
*
*  InitGPIO
*
*     Initializes the VC02 GPIO pins.
*
***************************************************************************/

void vc_host_init_pins( void )
{
    /*
     * Initialize things with Reset high (since we don't really need to use 
     * Reset, but leave it in a reset condition with Run and Hibernate low.
     */

#if defined( HW_GPIO_VID_RUN_PIN )
    gpio_direction_output( HW_GPIO_VID_RUN_PIN, 0 );
#endif

#if defined( HW_GPIO_VID_HIB_PIN )
    gpio_direction_output( HW_GPIO_VID_HIB_PIN, 0 );
#endif

    // Ensure that if the reset line is hooked up that it doesn't cause a
    // reset.

    vc_host_set_reset_pin( 1 );

} // vc_host_init_pins

/****************************************************************************
*
*  vc_host_set_reset_pin
*
*     Manipulates the Reset GPIO line (for the VC02)
*
*  Note that the CORESET_N line from the VC02 is really an I/O line. From
*  the VC02 side there is an open-drain driver and a 30k pullup resistor.
*
*  What this means to the host processor is that when asked to set the Reset
*  line low we should configure the pin as an output and drive it low. When
*  asked to drive it high, we should configure the pin as an input and let the
*  VC02 internal pullup pull it high.
*
***************************************************************************/

void vc_host_set_reset_pin( int arg )
{
#ifdef HW_GPIO_VID_RST_PIN
   switch ( arg )
   {
      case 0:  // Drive the reset line low
      {
         gpio_direction_output( HW_GPIO_VID_RST_PIN, 0 );
         break;
      }

      case 1:  // Allow the pin to be pulled high by the VC02's internal pullup
      {
         /* If the host processor sets the VC02 RESET GPIO to tristate,
          * it allows the VC02 internal pullup to pull
          * the reset line high in normal operation. However, some designs
          * had an external 10K pull down connected to the VC02_RESET line
          * which would override the VC02's internal pullup.
          * For those designs, the reset line has to be driven output high
          * by the host. */
#if HW_GPIO_VID_RESET_TRISTATE_IN_RUNMODE
         gpio_direction_input( HW_GPIO_VID_RST_PIN );
#else
         gpio_direction_output( HW_GPIO_VID_RST_PIN, 1 );
#endif

         break;
      }

      case 2:  // Pulse the Reset line
      {
         vc_host_set_reset_pin( 0 );

         // We need to leave the reset line low for a minimum of 5 msec. We
         // pick 10 to be slightly conservative.

         vc_host_delay_msec( 10 );

         vc_host_set_reset_pin( 1 );
         break;
      }
   }
#else
   (void)arg;
#endif
} // vc_host_set_reset_pin

/****************************************************************************
*
*  vc_host_set_run_pin
*
*     Manipulates the Run GPIO line (for the VC02)
*
***************************************************************************/

void vc_host_set_run_pin( int arg )
{
#if defined( HW_GPIO_VID_RUN_PIN )
    gpio_set_value( HW_GPIO_VID_RUN_PIN, arg );
#endif

} // vc_host_set_run_pin

/****************************************************************************
*
*  vc_host_set_hibernate_pin
*
*     Manipulates the Hibernate GPIO line (for the VC02)
*
***************************************************************************/

void vc_host_set_hibernate_pin( int arg )
{
#if defined( HW_GPIO_VID_HIB_PIN )
    gpio_set_value( HW_GPIO_VID_HIB_PIN, arg );
#endif

} // vc_host_set_hibernate_pin

/****************************************************************************
*
*  vc_host_reset
*
*     Performs reset sequence on the VC02
*
***************************************************************************/

void vc_host_reset( void )
{
   /* --------------------------------------------------------------------------
   ** Reset the LCD.
   **
   ** Normally, the LCD is completely controlled by the VideoCore processor
   ** (with the exception of the backlight). However, on some reference designs,
   ** LCD power is controlled by the host processor.
   */

#if defined(HW_GPIO_LCD_TRISTATE_PIN) && defined(HW_GPIO_LCD_PWR_PIN)
   /* Pulse the power and tristate pins. */
   gpio_direction_output( HW_GPIO_LCD_TRISTATE_PIN,  1 );
   gpio_direction_output( HW_GPIO_LCD_PWR_PIN,       0 );

   vc_host_delay_msec( 50 );

   gpio_set_value( HW_GPIO_LCD_TRISTATE_PIN,  0 );
   gpio_set_value( HW_GPIO_LCD_PWR_PIN,       1 );
#endif


   /* --------------------------------------------------------------------------
   ** Reset the VideoCore processor.
   **
   **   Note: Pulling Run and Hibernate low will cause a Reset condition
   **         regardless of the condition of the Reset line.
   */

   vc_host_set_hibernate_pin( 0 );
   vc_host_set_run_pin( 0 );

   // Wait for 100 milliseconds (1/10 of a second)

   vc_host_delay_msec( 100 );

   vc_host_set_run_pin( 1 );

   // Wait for 100 milliseconds (1/10 of a second)

   vc_host_delay_msec( 100 );

} // vc_host_reset

/****************************************************************************
*
*  Initializes the Host Port Interface for use with the VideoCore.
*
*   Returns 0 on success, non-zero on failure.
*
***************************************************************************/
   
int vc_host_port_init( void )
{
    VC_Register_t dummy;

#if (CFG_GLOBAL_CPU == MIPS32)

    /* Configure EBI bus */
   bcm1103mmr->ebiCtl.cs[HW_CS_VC02].base = ((HW_VC02_BASE_ADDR & MMR1103_EBICTL_BASE_MASK) |
                                              MMR1103_EBICTL_BASE_SIZE_8K);

   /* Configure wait states and other EBI config vals */
   bcm1103mmr->ebiCtl.cs[HW_CS_VC02].config = 
         (MMR1103_EBICTL_CONFIG_ENABLE                                     |
         (HW_CFG_VC02_EBI_WTST_VAL << MMR1103_EBICTL_CONFIG_WTST_SHIFT)    |
         MMR1103_EBICTL_CONFIG_WORD_WIDE                                   |
         (HW_CFG_VC02_EBI_SETUP_VAL << MMR1103_EBICTL_CONFIG_SETUP_SHIFT)  |
         (HW_CFG_VC02_EBI_HOLD_VAL << MMR1103_EBICTL_CONFIG_HOLD_SHIFT));
#endif

	// setup a 16 bit interface with software ready and 32 bit xfer widths

	// 'global' host-interface port settings (set through HCS0)

    // When using the VC02 development board in conjuntion with the EagleRay
    // board, the two boards are connected by a ribbon cable. This can cause
    // skew to be introduced in the signals, which in turns causes false reads
    // and such. A setting of 7 (the max) causes a delay of approx 25.2 ns
    // in the generation of the internal strobe.

    VCHCS0 = (WLED | ( gVcHostPortWLED & 0x07 ));
    VCHCS0 = (RLED | ( gVcHostPortRLED & 0x07 ));
    
    VCHCS1 = (WLED | ( gVcHostPortWLED & 0x07 ));
    VCHCS1 = (RLED | ( gVcHostPortRLED & 0x07 ));
    
    VCHCS0 = (BW    | BUS_16_BIT);
    VCHCS0 = (HRM   | SOFTWARE_READY);
    
    dummy = VCWBASE0;
    VCWBASE0 = 0x5A;
    dummy = VCWBASE0;
    if ( dummy != 0x5A )
    {
        PRINT_ERR( "vc_host_port_init: channel 0 sync failed\n" );
        return 1;
    }
    dummy = VCWBASE1;
    VCWBASE1 = 0x5A;
    dummy = VCWBASE1;
    if ( dummy != 0x5A )
    {
        PRINT_ERR( "vc_host_port_init: channel 1 sync failed\n" );
        return 1;
    }
    
    // read VCRBASE0 and VCRBASE1 until bottom bit is set in both
    // (resets multi-cycle logic and clears read FIFOs)
    
    dummy = 0;
    while ((( VCRBASE0 & 0x1 ) == 0 ) || (( VCRBASE1 & 0x1 ) == 0 ))
    {
        dummy++;
        if ( dummy == 0xFFFF )
        {
            PRINT_ERR( "vc_host_port_init: No VC02 detected\n" );
            return 1;
        }
    }
    
    dummy = VCHCS0;
    if ( dummy != 0x47 )
    {
        PRINT_ERR( "vc_host_port_init: Channel 0 status != 0x47, found: 0x%02x\n", dummy );
        return 1;
    }
    
    dummy = VCHCS1;
    if ( dummy != 0x47 )
    {
        PRINT_ERR( "vc_host_port_init: Channel 1 status != 0x47, found: 0x%02x\n", dummy );
        return 1;
    }
    
    PRINT_INFO( "VC02 detected\n" );
    
    // Now check to see which way the address lines are setup. What we do is to
    // write different values to WBASE0 and WBASE1 and then read back at the
    // intersection of the two address spaces and see which value we get back.
    //
    // For A1,2,3 addresses, addr % 0x000E == 0x0002 is a WBASE0 address.
    // For A6,7,8 addresses, addr % 0x01C0 == 0x0142 is a WBASE1 address.
    //
    // So we can read from address offset of 0x142 and see whether we get back
    // 0x5A or 0x5B
    
    dummy = VCWBASE0;
    VCWBASE0 = 0x5A;
    dummy = VCWBASE1;
    VCWBASE1 = 0x5B;
    
    dummy = *((volatile uint16_t *)( REG_VCO2_BASE + 0x142 ));
    
    if ( dummy == 0x5A )
    {
        strncpy( gVcAddrModeStr, "A1,A2,A3", sizeof( gVcAddrModeStr ) );
        gVcUse32bitIO = 0;
        gVc32BitIOAvail = 0;
    }
    else
    if ( dummy == 0x5B )
    {
        strncpy( gVcAddrModeStr, "A6,A7,A8", sizeof( gVcAddrModeStr ) );
        gVcUse32bitIO = 1;
        gVc32BitIOAvail = 1;
    }
    else
    {
        PRINT_ERR( "vc_host_port_init: auto-detect of addressing scheme failed\n" );
        return 1;
    }
    // The following ensures that gVcAddeModeStr is properly null terminated
    // in the case that the source exactly fits. Normally, I'd have used
    // strlcpy, but it's not available from the bootloader, so strncpy was
    // used instead.

    gVcAddrModeStr[ sizeof( gVcAddrModeStr ) - 1 ] = '\0';

    PRINT_INFO( "VC02 with %s addressing detected\n", gVcAddrModeStr );
    
    dummy = VCWBASE0; // Reset things
    dummy = VCWBASE1;
    
    // individiual port settings - channel 0
    VCHCS0 = (WTW   | TRANSFER_32_BIT);
    VCHCS0 = (RTW   | TRANSFER_32_BIT);
    VCHCS0 = (RMODE | RMODE_FIFO);
    VCHCS0 = (RTHR  | THRS_HALF_FULL);
    VCHCS0 = (WTHR  | THRS_HALF_FULL);
    
    // individiual port settings - channel 1
    VCHCS1 = (WTW   | TRANSFER_32_BIT);
    VCHCS1 = (RTW   | TRANSFER_32_BIT);
    VCHCS1 = (RMODE | RMODE_FIFO);
    VCHCS1 = (RTHR  | THRS_HALF_FULL);
    VCHCS1 = (WTHR  | THRS_HALF_FULL);

    return 0;
    
} // vc_host_port_init

/******************************************************************************
NAME
   vc_host_init_sdram

SYNOPSIS
   void vc_host_init_sdram( void )

FUNCTION
   Initializes the external SDRAM on the VC02. This is needed in order to
   download code to the external SDRAM before the VC02 is running.

RETURNS
   void
******************************************************************************/

void vc_host_init_sdram( void )
{
   uint16_t        mccs[2];
   static uint16_t mccpsz[2]      = {0x0001, 0x0000};
   static uint16_t mccmap3[2]     = {0x003F, 0x0000};

#if ( HW_CFG_VC02_SDRAM_SIZE == 16 )

   // 16 Mb of SDRAM on the VC02

   static uint16_t emsdsb[2]      = {0xc834, 0x3057};

   const char *memSizeStr = "16";

#elif ( HW_CFG_VC02_SDRAM_SIZE == 32 )

   // 32 Mb of SDRAM on the EVM

   static uint16_t emsdsb[2]      = {0xc934, 0x3057};

   const char *memSizeStr = "32";
#else
#  error Unsupported value for HW_CFG_VC02_SDRAM_SIZE
#endif

   static uint16_t emsdcs[2]      = {0x0007, 0x0000};
   static uint16_t zero32bit[2]   = {0x0000, 0x0000};
   static uint16_t emsdptslow[2]  = {0x80E8, 0x0000};
   static uint16_t emsdptfast[2]  = {0x0064, 0x0000};

   VC_DEBUG( Trace, "Initializing VC02 external SDRAM (%s Mb)\n", memSizeStr );

   // poll for MBIS finished (check BP gone to zero and then for RSTN going
   // to 1 in the HCS1 register)
   while( VCHCS1 & BP );
   while( !(VCHCS1 & RSTN) );

   // Must set up the external RAM for VMCS
   // flush dcache first
   vc_host_read_byteswapped(mccs, 0x10000C00, 4, 0);
   mccs[0] |= 0x0000;
   mccs[1] |= 0x4000;
   vc_host_write_byteswapped(0x10000C00, mccs,  4, 0);

   // Disable caches for this test
   vc_host_write_byteswapped(0x10000C00, zero32bit, 4, 0);

   // flush dcache again
   vc_host_read_byteswapped(mccs, 0x10000C00, 4, 0);
   mccs[0] |= 0x0000;
   mccs[1] |= 0x4000;
   vc_host_write_byteswapped(0x10000C00, mccs,  4, 0);

   //Set the SDRAM powerup time to be 33000
   vc_host_write_byteswapped(0x10003430, emsdptslow,  4, 0);

   // Set cache partitioning: 8K each for int & ext memory
   // AVOIDS EXTERNAL MEMORY CACHING PROBLEMS
   vc_host_write_byteswapped(0x10000C10, mccpsz,  4, 0);
   vc_host_write_byteswapped(0x10000C1C, mccmap3, 4, 0);

   // Program EXTSDB - conservatively
   vc_host_write_byteswapped(0x10003428, emsdsb,    4, 0);

   // Enable SDRAM - full init
   vc_host_write_byteswapped(0x10003420, emsdcs, 4, 0);

   // wait until the caches are flushed (DCE [bit 0] is set in MCCS)
   do {
      vc_host_read_byteswapped(mccs, 0x10003420, 4, 0);
   } while( (mccs[0] & 0x8000) == 0);

   //Set the SDRAM powerup time to be 100
   vc_host_write_byteswapped(0x10003430, emsdptfast,  4, 0);

   // enable caching
   vc_host_read_byteswapped(mccs, 0x10000C00, 4, 0);
   mccs[0] &= 0xffff;
   mccs[0] |= 0x0000;
   mccs[1] &= 0xbfff;
   mccs[1] |= 0x0001;
   vc_host_write_byteswapped(0x10000C00, mccs,  4, 0);

} // vc_host_init_sdram

/******************************************************************************
NAME
   vc_host_load_elf

SYNOPSIS
   int vc_host_load_elf( void *voidBuf, size_t numBytes )

FUNCTION
   Transfers an in-memory copy of an ELF file downto the VC02.

RETURNS
   The id of the application that has just been booted
******************************************************************************/

static int vc_host_load_elf( void *voidBuf, size_t numBytes )
{
   int         i;
   size_t      bytesToLoad;
   uint8_t    *buf = voidBuf;
   Elf32_Ehdr  exeHdr;
   Elf32_Phdr  pgmHdr;
   void       *loadBuf;
   void       *zeroBuf = NULL;
#ifdef BOOTCHK
   uint32_t    errCount = 0;
#endif

   // Initialize SDRAM just in case the ELF file needs to load stuff there

   vc_host_init_sdram();

   memcpy( &exeHdr, buf, sizeof( exeHdr ));

   exeHdr.e_phoff     = VC_HTOV32( exeHdr.e_phoff );
   exeHdr.e_phnum     = VC_HTOV16( exeHdr.e_phnum );
   exeHdr.e_phentsize = VC_HTOV16( exeHdr.e_phentsize );

   gNumCrcSections = 0;

   for ( i = 0; i < exeHdr.e_phnum; i++ )
   {
      Elf32_Off   pgmHdrOffset = exeHdr.e_phoff + ( i * exeHdr.e_phentsize );

      if ( pgmHdrOffset >= numBytes )
      {
         PRINT_ERR( "pgmHdrOffset: %u is not within buffer of size %d\n", 
                 pgmHdrOffset, numBytes );
         return -1;
      }

      memcpy( &pgmHdr, &buf[ pgmHdrOffset ], sizeof( pgmHdr ));

      pgmHdr.p_offset = VC_HTOV32( pgmHdr.p_offset );
      pgmHdr.p_vaddr  = VC_HTOV32( pgmHdr.p_vaddr );
      pgmHdr.p_filesz = VC_HTOV32( pgmHdr.p_filesz );
      pgmHdr.p_memsz  = VC_HTOV32( pgmHdr.p_memsz );
      pgmHdr.p_flags  = VC_HTOV32( pgmHdr.p_flags );

      if ( pgmHdr.p_filesz > 0 )
      {
         if ( pgmHdr.p_offset >= numBytes )
         {
            PRINT_ERR( "pgmHdr.p_offset (0x%08x) >= numBytes (0x%08x)\n", pgmHdr.p_offset, numBytes );
            return -1;
         }

         if (( pgmHdr.p_offset + pgmHdr.p_filesz ) > numBytes )
         {
            PRINT_ERR( "( pgmHdr.p_offset (0x%08x) + pgmHdr.p_filesz (0x%08x) ) >= numBytes (0x%08x)\n", 
                    pgmHdr.p_offset, pgmHdr.p_filesz, numBytes );
            return -1;
         }

         bytesToLoad = ( pgmHdr.p_filesz + 3 ) & ~3;
         loadBuf = &buf[ pgmHdr.p_offset ];

         if (( pgmHdr.p_flags & 1 ) != 0 )
         {
            uint32_t    crc;
            size_t      len = bytesToLoad;

            if ( pgmHdr.p_vaddr < 0x200000 )
            {
               // The linker script used by the VC02 places some writable
               // objects at the end of the read only section, so we need
               // to ignore these.

               len -= 32;
            }

            // Section is marked as executable. Do a CRC on it.

            crc  = ~crc32( ~0, loadBuf, len );

            if ( gNumCrcSections >= NUM_CRC_SECTIONS )
            {
                PRINT_INFO( "Load from offset 0x%08x for len of 0x%08x to addr 0x%08x CRC: 0x%08x (crc not stored)\n",
                        pgmHdr.p_offset, pgmHdr.p_filesz, pgmHdr.p_vaddr, crc );
            }
            else
            {
                gCrcSection[ gNumCrcSections ].addr = pgmHdr.p_vaddr;
                gCrcSection[ gNumCrcSections ].len  = len;
    
                gCrcSection[ gNumCrcSections ].crc  = crc;
    
                PRINT_INFO( "Load from offset 0x%08x for len of 0x%08x to addr 0x%08x CRC: 0x%08x\n",
                        pgmHdr.p_offset, pgmHdr.p_filesz, pgmHdr.p_vaddr, crc );
    
                gNumCrcSections++;
            }
         }
         else
         {
            PRINT_INFO( "Load from offset 0x%08x for len of 0x%08x to addr 0x%08x\n",
                    pgmHdr.p_offset, pgmHdr.p_filesz, pgmHdr.p_vaddr );
         }
      }
      else
      if ( pgmHdr.p_memsz > 0 )
      {
         bytesToLoad = ( pgmHdr.p_memsz + 3 ) & ~3;

         PRINT_INFO( "Clear for length of 0x%08x start at addr 0x%08x\n",
                 pgmHdr.p_memsz, pgmHdr.p_vaddr );

#if defined( __KERNEL__ )
         zeroBuf = kcalloc( 1, bytesToLoad, GFP_KERNEL );
#else
         // We assume that when we're running under the bootloader, that the memory
         // starting immediately after the ELF image is available to use.

         numBytes = ( numBytes + 3 ) & ~3;  // Bumnp to next multiple of 4 bytes
         zeroBuf = &buf[ numBytes ];
#endif

         memset( zeroBuf, 0, bytesToLoad );

         loadBuf = zeroBuf;
      }
      else
      {
         bytesToLoad = 0;
      }

      if ( bytesToLoad > 0 )
      {
         vc_host_write_consecutive( pgmHdr.p_vaddr, loadBuf, bytesToLoad, 0 );

#ifdef BOOTCHK
         {
            /* do a read-back test and count the errors */

            uint32_t data;
            uint32_t bufOffset =  0;
            uint32_t vc_addr = pgmHdr.p_vaddr;

            for ( bufOffset = 0; bufOffset < bytesToLoad; bufOffset += 4, vc_addr += 4 )
            {
               vc_host_read_consecutive( &data, vc_addr, 4, 0 );

               if ( data != *((uint32_t *)((uint8_t*)loadBuf + bufOffset )))
               {
                  ++errCount;
               }
            }
         }
#endif

      }

      if ( zeroBuf != NULL )
      {
#if defined( __KERNEL__ )
         kfree( zeroBuf );
#endif
         zeroBuf = NULL;
      }
   }

#ifdef BOOTCHK
   if ( errCount > 0 )
   {
      PRINT_ERR( "vc_host_load_elf: errors found during readback : '%d'\n", errCount );
      return -1;
   }
#endif

   return 1;

} // vc_host_load_elf

/******************************************************************************
NAME
   vc_host_check_crc

SYNOPSIS
   int vc_host_check_crc( void )

FUNCTION
   Checks the CRC's of the VC02 code. It has an option to regulate how much
   time is consumed performing the check. If delayMsec is > zero, then it will
   delay for the requested number of milliseconds between each 4K block of code
    being checked. Currently there will be about 50 to 100 such delays to 
    checks the entire code base.

RETURNS
   Returns 1 if all CRC's pass, 0, if any CRC's fail
******************************************************************************/

#define  CRC_BUF_SIZE   4096

#if !defined( __KERNEL__ )
static  char    gStaticCrcBuf[ CRC_BUF_SIZE ];
#endif

int vc_host_check_crcs( unsigned delayMsec )
{
   void *buf;
   int   rc = 1;
   int   i;
#if defined( __KERNEL__ )
   struct timeval startTime;
   struct timeval endTime;
#endif

   PRINT_INFO( "vc_host_check_crcs: Starting...\n" );

   if ( gNumCrcSections == 0 )
   {
      PRINT_ERR( "vc_host_check_crcs: No CRCs calculated\n" );
      return 0;
   }

#if defined( __KERNEL__ )
   if (( buf = vmalloc( CRC_BUF_SIZE )) == NULL )
   {
      PRINT_ERR( "vc_host_check_crcs: Unable to allocate 4k buffer\n" );
      return 0;
   }

   do_gettimeofday( &startTime );
#else
   buf = gStaticCrcBuf;
#endif

   for ( i = 0; i < gNumCrcSections; i++ )
   {
      uint32_t vcAddr         = gCrcSection[ i ].addr;
      int      bytesRemaining = gCrcSection[ i ].len;
      uint32_t crc = ~0;

      while ( bytesRemaining > 0 )
      {
         int   bytesThisTime = bytesRemaining;

         if ( bytesThisTime > CRC_BUF_SIZE )
         {
            bytesThisTime = CRC_BUF_SIZE;
         }

         if ( vc_host_read_consecutive( buf, vcAddr, bytesThisTime, 1 ) != 0 )
         {
            PRINT_ERR( "vc_host_check_crcs: error trying to read %d bytes from VC addr 0x%08x\n", bytesThisTime, vcAddr );
            rc = 0;
            break;
         }

         crc = crc32( crc, buf, bytesThisTime );

         bytesRemaining -= bytesThisTime;
         vcAddr         += bytesThisTime;

         if ( delayMsec > 0 )
         {
             vc_host_delay_msec( delayMsec );
         }
      }

      crc = ~crc;

      if ( crc == gCrcSection[ i ].crc )
      {
         PRINT_INFO( "CRC for 0x%08x len 0x%06x is 0x%08x - Passed\n",
                     gCrcSection[ i ].addr, gCrcSection[ i ].len, crc );
      }
      else
      {
         PRINT_INFO( "CRC for 0x%08x len 0x%06x is 0x%08x - Failed - Expecting 0x%08x\n",
                     gCrcSection[ i ].addr, gCrcSection[ i ].len, crc, gCrcSection[ i ].crc );

         rc = 0;
      }
   }

#if defined( __KERNEL__ )
   vfree( buf );

   do_gettimeofday( &endTime );

   endTime.tv_sec -= startTime.tv_sec;
   endTime.tv_usec -= startTime.tv_usec;

   if ( endTime.tv_usec < 0 )
   {
      endTime.tv_usec += 1000000;
      endTime.tv_sec  -= 1;
   }

   PRINT_INFO( "CRC check took %d.%06d seconds\n", (int)endTime.tv_sec, (int)endTime.tv_usec );
#endif

   return rc;

} // vc_host_check_crcs

/******************************************************************************
NAME
   vc_host_boot

SYNOPSIS
   int vc_host_boot(char *cmd_line, void *binimg, int nbytes, bool_t bootloader)

FUNCTION
   Transfer a binary image to the VC02, if the binary image is a boot loader
   the command line is also uploaded (parameter ignored otherwise) and as the
   bootloader is loaded at 0xFC100 a fixup is applied at the initial IP pointing
   to the bootloader. Setup PLL registers, flushs the memory cache and wakes up
   the processor.

RETURNS
   The id of the application that has just been booted
******************************************************************************/

int vc_host_boot(char *cmd_line, void *binimg, size_t nbytes, int bootloader)
{
   int        i, j, cmd_len=0;
   char       name_buffer[16] = "           \0\0\0\0\0";
   uint16_t   mccs[2];
   uint32_t   vc_app = 0;

   // static const data that is uploaded to the VideoCore as part of the bootstrap
   static uint16_t bl_fixup[4] = {0x2f7fu, (uint16_t)VC_BOOTLDR_ADDRESS,
                                     (uint16_t)(VC_BOOTLDR_ADDRESS>>16), 0};
   static uint16_t zero32bit[2]   = {0x0000u, 0x0000u};
   static uint16_t mcacheflush[2] = {0x0000u, VC_HTOV16(0xC000u)};
   static uint16_t wakeup[2]      = {VC_HTOV16(0x0002u), 0xA5A5u};

   // poll for MBIS finished (check BP gone to zero and then for RSTN going
   // to 1 in the HCS1 register)
   while( VCHCS1 & BP );
   while( !(VCHCS1 & RSTN) );

   if( bootloader ) {
      // specified image is a bootloader image

      // form the command line in the correct format
      if( !cmd_line ) {
         strcpy(name_buffer, "VMCS    BIN");
      } else {
   	     // Expand the filename from 8.3 format into the required format
	       for (i = 0, j = 0; i < 11; i++) {
		        if (cmd_line[j] == '.') {
		           i=8;
		           j++;
		        }
            if (cmd_line[j] == '\0')
               break;
            name_buffer[i] = cmd_line[j];
            j++;
            // Change lower case to upper case.
            if ('a' <= name_buffer[i] && name_buffer[i] <= 'z')
               name_buffer[i] &= ~('a' - 'A');
         }
         // Copy bytes 11:15 of cmd_line into name_buffer
         ((uint32_t *)name_buffer)[3] = ((uint32_t *)cmd_line)[3];
         // calculate cmd_len and make it a multiple of 4/16
         cmd_len=(strlen(cmd_line)-16+3)&~3;
      }

      // + upload the command line and any remaining parameters to 0xFC0000
      // + upload the bootloader code
      // + upload a fixup to 0x200 (branch to bootloader code)

      vc_host_write_consecutive(VC_CMDLINE_ADDRESS, name_buffer, 16, 0);
      if( cmd_len )
         vc_host_write_consecutive(VC_CMDLINE_ADDRESS+0x10, &cmd_line[16], cmd_len, 0);
      vc_host_write_consecutive(VC_BOOTLDR_ADDRESS, binimg, nbytes, 0);
      vc_host_write_consecutive(VC_FIXUP_ADDRESS, bl_fixup, 8, 0);
   }
   else
   {
      uint8_t   *signature = binimg;

      if (( signature[0] == 0x7f )
      &&  ( signature[1] == 'E' )
      &&  ( signature[2] == 'L' )
      &&  ( signature[3] == 'F' ))
      {
         vc_host_load_elf( binimg, nbytes );
      }
      else
      {
#if defined( VC_HOST_IS_BIG_ENDIAN )
         // byte reverse image if necessary
         size_t     i;
         uint16_t  *binimg16 = binimg;

         for ( i = 0; i < ( nbytes / 2 ); i++)
         {
            binimg16[i] = VC_HTOV16(binimg16[i]);
         }
#endif
         // upload the supplied image so that it starts at 0x0
         vc_host_write_consecutive( 0x0, binimg, nbytes, 0 );

#ifdef BOOTCHK
         {
            /* do a read-back test and count the errors */
            uint32_t data;

            size_t   offset   = 0;
            uint32_t errCount = 0;

            for ( offset=0; offset<nbytes ; offset+=4 )
            {
               vc_host_read_consecutive(&data, offset, 4, 0);

               // check the data read back
               if( data != *((uint32_t*)((uint8_t*)binimg + offset)) )
               {
                  ++errCount;
               }
            }

            if ( errCount )
            {
               PRINT_ERR( "vc_host_boot: errors found during readback : '%d'\n", errCount );
               return -1;
            }
         }
#endif
      }
   }

    // set the currently running task ID to 0000
    vc_host_write_consecutive(VC_APP_ADDRESS, zero32bit, 4, 0);
    
    // flush the caches
    vc_host_write_consecutive(0x10000C00, mcacheflush, 4, 0);
    
    // wait until the caches are flushed (DCE [bit 0] is set in MCCS)
    do {
      vc_host_read_consecutive(mccs, 0x10000C00, 4, 0);
    } while( !(VC_HTOV16(mccs[0]) & 0x1) );
    
    // reset the cache register
    vc_host_write_consecutive(0x10000C00, zero32bit, 4, 0);
    
    // wakeup the processor
    vc_host_write_consecutive(0x10000400, wakeup, 4, 0);
    
    PRINT_INFO( "Waiting for VideoCore to initialize...\n" );
    
    // Wait for the application to initialise and grab its task ID
    do
    {
        vc_host_read_consecutive( &vc_app, VC_APP_ADDRESS, sizeof( vc_app ), 0 );

        vc_app = VC_VTOH32( vc_app );

    } while( vc_app == 0 );
    
    return vc_app;
}

/******************************************************************************
NAME
   vc_host_read_consecutive

SYNOPSIS
   int vc_host_read_consecutive (void *host_addr, uint32_t vc_addr, int nbytes,
                                 int channel)

FUNCTION
   Read nbytes consecutive bytes from VideoCore host port channel channel into
   the memory pointed to by host_addr. nbytes must be a multiple of 4, vc_addr
   must be aligned to 4 bytes and channel is 0 or 1. Returns non-zero for failure.

RETURNS
   int
*****************************************************************************/

int vc_host_read_consecutive (void *host_addr, uint32_t vc_addr, int nbytes,
                              int channel)
{
   volatile VC_Register_t *vcrbase, *vcdata, *vchcs;
   VC_Register_t        dummy;
   int                  nblocks, idx;
   int                  rc = 0;
#if VC_DEBUG_ENABLED && defined( KNLLOG )
   int debugIgnoreLog = 0;
#endif

   VC_DEBUG( Trace, "host_addr: 0x%08x vc_addr: 0x%08x nbytes: 0x%04x, channel: %d\n",
             (uint32_t)host_addr, vc_addr, nbytes, channel );

   // sanity check on the data (vc_addr aligned to 4 bytes, nbytes is a
   // multiple of 4 and channel is 0 or 1)
   vc_assert( (vc_addr &  3) == 0 );
   vc_assert( (nbytes  &  3) == 0 );
   vc_assert( (channel & ~1) == 0 );
   vc_assert( host_addr != NULL );

   // setup the pointers to reflect the selected channel registers
   vcrbase = &VC_REG_16( RBASE, channel );
   vcdata  = &VC_REG_16( DATA,  channel );
   vchcs   = &VC_REG_16( HCS,   channel );

   // calc number of 32 bit blocks to copy
   nblocks = nbytes >> 2;

   // acquire the channel host interface lock for exclusive access
   vc_host_lock_obtain( channel );

#if VC_DEBUG_ENABLED && defined( KNLLOG )
   if ( gVcProfileTransfers )
   {
      if (nbytes > 4)
      {
         KNLLOG("S: 0x%x %d\n", (int)host_addr, nbytes);
      }
      else
      {
         debugIgnoreLog = 1;
      }
   }
#endif

   *vchcs = (WLED | ( gVcHostPortWLED & 0x07 ));
   *vchcs = (RLED | ( gVcHostPortRLED & 0x07 ));

   LIMITED_WAIT_FOR(( *vcrbase & 1 ) != 0 );

   // set the base register for the read

   *vcrbase = (uint16_t)vc_addr;
   *vcrbase = (uint16_t)(vc_addr >> 16);

   // depending on the alignment of the host address do aligned or unaligned
   // reads
   if( ((uint32_t)host_addr)&1 ) {
      // host buffer is unaligned
      uint8_t *dest = (uint8_t *)host_addr;
      uint8_t *dummy_addr = (uint8_t *)&dummy;

      for(idx=0; idx<nblocks; idx++) {
         // wait until the read fifo contains data and then read 32 bits
         // unaligned into the host buffer

         LIMITED_WAIT_FOR(( *vchcs & RFD ) != 0 );

         dummy = *vcdata;
         dest[0] = dummy_addr[1];
         dest[1] = dummy_addr[0];
         dummy = *vcdata;
         dest[2] = dummy_addr[1];
         dest[3] = dummy_addr[0];
         dest += 4;
      }
   }
   else
   {
#if VC_HOST_PORT_USE_DMA
      if ( gVcUseDma  && (((int)host_addr & 3) == 0) )
      {
         /* DMA the rest of the data */
         vcHostDmaRead( host_addr, vcdata, vchcs, nblocks << 1, 2 );
      }
      else
#endif
      if ( gVcUseFastXfer )
      {
         // 16 bit aligned buffer on the host

         uint16_t *dest = (uint16_t *)host_addr;

         if ( gVcUse32bitIO )
         {
            uint32_t dummy32;

            while ( nbytes >= 16 )
            {
               // wait until the fifo is full, then read the entire contents

               LIMITED_WAIT_FOR(( *vchcs & RFF ) != 0 );

               dummy32 = *(volatile uint32_t *)vcdata;
               *dest++ = VC_HTOV16( (uint16_t)( dummy32 >> 16 ));
               *dest++ = VC_HTOV16( (uint32_t)dummy32 );

               dummy32 = *(volatile uint32_t *)vcdata;
               *dest++ = VC_HTOV16( (uint16_t)( dummy32 >> 16 ));
               *dest++ = VC_HTOV16( (uint32_t)dummy32 );

               dummy32 = *(volatile uint32_t *)vcdata;
               *dest++ = VC_HTOV16( (uint16_t)( dummy32 >> 16 ));
               *dest++ = VC_HTOV16( (uint32_t)dummy32 );

               dummy32 = *(volatile uint32_t *)vcdata;
               *dest++ = VC_HTOV16( (uint16_t)( dummy32 >> 16 ));
               *dest++ = VC_HTOV16( (uint32_t)dummy32 );

               nbytes -= 16;
            }
         }
         else
         {
            while ( nbytes >= 16 )
            {
               // wait until the fifo is full, then read the entire contents

               LIMITED_WAIT_FOR(( *vchcs & RFF ) != 0 );

               dummy = *vcdata;
               *dest++ = VC_HTOV16( dummy );
               dummy = *vcdata;
               *dest++ = VC_HTOV16( dummy );
               dummy = *vcdata;
               *dest++ = VC_HTOV16( dummy );
               dummy = *vcdata;
               *dest++ = VC_HTOV16( dummy );
               dummy = *vcdata;
               *dest++ = VC_HTOV16( dummy );
               dummy = *vcdata;
               *dest++ = VC_HTOV16( dummy );
               dummy = *vcdata;
               *dest++ = VC_HTOV16( dummy );
               dummy = *vcdata;
               *dest++ = VC_HTOV16( dummy );

               nbytes -= 16;
            }
         }

         while ( nbytes > 0 )
         {
            // wait until the read fifo contains data, then perform read

            LIMITED_WAIT_FOR(( *vchcs & RFD ) != 0 );

            // This function needs to present the data in the same byte order
            // as it is on the VC02. The caller is responsible for performing
            // 16 or 32 bit byte swaps as appropriate. Since our data bus is
            // connected to the VC02 data bus, we actually get the data as
            // 16 bit words, which we need to swap so that the byte order goes
            // back to what it was on the VC02.

            dummy = *vcdata;
            *dest++ = VC_HTOV16( dummy );
            dummy = *vcdata;
            *dest++ = VC_HTOV16( dummy );

            nbytes -= 4;
         }
      }
      else
      {
         // 16 bit aligned buffer on the host
         uint16_t *dest = (uint16_t *)host_addr;

         for ( idx = 0; idx < nblocks; idx++ )
         {
            // wait until the read fifo contains data, then perform read

            LIMITED_WAIT_FOR(( *vchcs & RFD ) != 0 );

            // This function needs to present the data in the same byte order
            // as it is on the VC02. The caller is responsible for performing
            // 16 or 32 bit byte swaps as appropriate. Since our data bus is
            // connected to the VC02 data bus, we actually get the data as
            // 16 bit words, which we need to swap so that the byte order goes
            // back to what it was on the VC02.

            dummy = *vcdata;
            dest[0] = VC_HTOV16( dummy );
            dummy = *vcdata;
            dest[1] = VC_HTOV16( dummy );
            dest += 2;
         }
      }
   }

#if VC_DEBUG_ENABLED
   if ( gVcDebugRead )
   {
       vc_dump_mem( "R:", vc_addr, host_addr, nbytes );
   }
#endif

   // reset the rbase register so that FIFO will be cleared more quickly for
   // the next call
   dummy = *vcrbase;

end:
#if VC_DEBUG_ENABLED && defined( KNLLOG )
   if ( gVcProfileTransfers )
   {
      if (debugIgnoreLog == 0)
      {
         KNLLOG("E: 0x%x %d\n", (int)host_addr, nbytes);
      }
   }
#endif

   // release the lock
   vc_host_lock_release( channel );
   return rc;
}

#ifdef VC_HOST_IS_BIG_ENDIAN
/******************************************************************************
NAME
   vc_host_read_byteswapped

SYNOPSIS
   int vc_host_read_byteswapped (void *host_addr, uint32_t vc_addr, int nbytes,
                                 int channel)

FUNCTION
   Read nbytes consecutive bytes from VideoCore host port channel channel into
   the memory pointed to by host_addr. nbytes must be a multiple of 4, vc_addr
   must be aligned to 4 bytes and channel is 0 or 1. Returns non-zero for failure.

RETURNS
   int
******************************************************************************/

int vc_host_read_byteswapped(void *host_addr, uint32_t vc_addr, int nbytes,
                              int channel)
{
   volatile VC_Register_t *vcrbase, *vcdata, *vchcs;
   VC_Register_t        dummy;
   int                  nblocks, idx;
   int                  rc = 0;

   // sanity check on the data (vc_addr aligned to 4 bytes, nbytes is a
   // multiple of 4 and channel is 0 or 1)
   vc_assert( (vc_addr &  3) == 0 );
   vc_assert( (nbytes  &  3) == 0 );
   vc_assert( (channel & ~1) == 0 );
   vc_assert( host_addr != NULL );

   // setup the pointers to reflect the selected channel registers
   vcrbase = &VC_REG_16( RBASE, channel );
   vcdata  = &VC_REG_16( DATA,  channel );
   vchcs   = &VC_REG_16( HCS,   channel );

   // calc number of 32 bit blocks to copy
   nblocks = nbytes >> 2;

   // acquire the channel host interface lock for exclusive access
   vc_host_lock_obtain( channel );

#if VC_DEBUG_ENABLED && defined( KNLLOG )
   if ( gVcProfileTransfers )
   {
      KNLLOG("S: 0x%x %d\n", (int)host_addr, nbytes);
   }
#endif

   *vchcs = (WLED | ( gVcHostPortWLED & 0x07 ));
   *vchcs = (RLED | ( gVcHostPortRLED & 0x07 ));

   LIMITED_WAIT_FOR(( *vcrbase & 1 ) != 0 );

   // set the base register for the read

   *vcrbase = (uint16_t)vc_addr;
   *vcrbase = (uint16_t)(vc_addr >> 16);

   // depending on the alignment of the host address do aligned or unaligned
   // reads
   if( ((uint32_t)host_addr)&1 ) {
      // host buffer is unaligned
      uint8_t *dest = (uint8_t *)host_addr;
      uint8_t *dummy_addr = (uint8_t *)&dummy;

      for(idx=0; idx<nblocks; idx++) {
         // wait until the read fifo contains data and then read 32 bits
         // unaligned into the host buffer

         LIMITED_WAIT_FOR(( *vchcs & RFD ) != 0 );

         if (dest != NULL)
         {
            dummy = *vcdata;
            dest[0] = dummy_addr[0];
            dest[1] = dummy_addr[1];
            dummy = *vcdata;
            dest[2] = dummy_addr[0];
            dest[3] = dummy_addr[1];

            dest += 4;
         }
      }
   }
   else
   {
#if VC_HOST_PORT_USE_DMA
      if ( gVcUseDma  && (((int)host_addr & 3) == 0) )
      {
         /* DMA the rest of the data */
         vcHostDmaRead( host_addr, vcdata, vchcs, nblocks << 1, 0 );
      }
      else
#endif
      if ( gVcUseFastXfer )
      {
         uint16_t *dest;

         // 16 bit aligned buffer on the host

         if ( gVcUse32bitIO )
         {
            uint32_t *dest32 = (uint32_t *)host_addr;

            while ( nbytes >= 16 )
            {
               // wait until the fifo is full, then read the entire contents

               LIMITED_WAIT_FOR(( *vchcs & RFF ) != 0 );

               *dest32++ = *(volatile uint32_t *)vcdata;
               *dest32++ = *(volatile uint32_t *)vcdata;
               *dest32++ = *(volatile uint32_t *)vcdata;
               *dest32++ = *(volatile uint32_t *)vcdata;

               nbytes -= 16;
            }

            dest = (uint16_t *)dest32;
         }
         else
         {
            dest = (uint16_t *)host_addr;

            while ( nbytes >= 16 )
            {
               // wait until the fifo is full, then read the entire contents

               LIMITED_WAIT_FOR(( *vchcs & RFF ) != 0 );

               *dest++ = *vcdata;
               *dest++ = *vcdata;
               *dest++ = *vcdata;
               *dest++ = *vcdata;
               *dest++ = *vcdata;
               *dest++ = *vcdata;
               *dest++ = *vcdata;
               *dest++ = *vcdata;

               nbytes -= 16;
            }
         }

         while ( nbytes > 0 )
         {
            LIMITED_WAIT_FOR(( *vchcs & RFD ) != 0 );

            *dest++ = *vcdata;
            *dest++ = *vcdata;

            nbytes -= 4;
         }
      }
      else
      {
         // 16 bit aligned buffer on the host
         uint16_t *dest = (uint16_t *)host_addr;

         for ( idx = 0; idx < nblocks; idx++ )
         {
            // wait until the read fifo contains data, then perform read

            LIMITED_WAIT_FOR(( *vchcs & RFD ) != 0 );

            // This function needs to present the data in the same byte order
            // as it is on the VC02. The caller is responsible for performing
            // 16 or 32 bit byte swaps as appropriate. Since our data bus is
            // connected to the VC02 data bus, we actually get the data as
            // 16 bit words, which we need to swap so that the byte order goes
            // back to what it was on the VC02.

            if (dest != NULL)
            {
               dest[0] = *vcdata;
               dest[1] = *vcdata;

               dest += 2;
            }
         }
      }
   }

#if VC_DEBUG_ENABLED
   if ( gVcDebugRead )
   {
       vc_dump_mem_16( "R:", vc_addr, host_addr, nbytes );
   }
#endif

   // reset the rbase register so that FIFO will be cleared more quickly for
   // the next call
   dummy = *vcrbase;

end:
#if VC_DEBUG_ENABLED && defined( KNLLOG )
   if ( gVcProfileTransfers )
   {
      KNLLOG("E: 0x%x %d\n", (int)host_addr, nbytes);
   }
#endif

   // release the lock
   vc_host_lock_release( channel );
   return rc;
}

/******************************************************************************
NAME
   vc_host_read_byteswapped

SYNOPSIS
   int vc_host_read_byteswapped (void *host_addr, uint32_t vc_addr, int nbytes,
                                 int channel)

FUNCTION
   Read nbytes consecutive bytes from VideoCore host port channel channel into
   the memory pointed to by host_addr. nbytes must be a multiple of 4, vc_addr
   must be aligned to 4 bytes and channel is 0 or 1. Returns non-zero for failure.

RETURNS
   int
******************************************************************************/

int vc_host_read_byteswapped_32(void *host_addr, uint32_t vc_addr, int nbytes,
                                int channel)
{
   volatile VC_Register_t *vcrbase, *vcdata, *vchcs;
   VC_Register_t        dummy;
   int                  nblocks, idx;
   int                  rc = 0;

   // sanity check on the data (vc_addr aligned to 4 bytes, nbytes is a
   // multiple of 4 and channel is 0 or 1)
   vc_assert( (vc_addr &  3) == 0 );
   vc_assert( (nbytes  &  3) == 0 );
   vc_assert( (channel & ~1) == 0 );

   // setup the pointers to reflect the selected channel registers
   vcrbase = &VC_REG_16( RBASE, channel );
   vcdata  = &VC_REG_16( DATA,  channel );
   vchcs   = &VC_REG_16( HCS,   channel );

   // calc number of 32 bit blocks to copy
   nblocks = nbytes >> 2;

   // acquire the channel host interface lock for exclusive access
   vc_host_lock_obtain( channel );

#if VC_DEBUG_ENABLED && defined( KNLLOG )
   if ( gVcProfileTransfers )
   {
      KNLLOG("S: 0x%x %d\n", (int)host_addr, nbytes);
   }
#endif

   *vchcs = (WLED | ( gVcHostPortWLED & 0x07 ));
   *vchcs = (RLED | ( gVcHostPortRLED & 0x07 ));

   LIMITED_WAIT_FOR(( *vcrbase & 1 ) != 0 );

   // set the base register for the read

   *vcrbase = (uint16_t)vc_addr;
   *vcrbase = (uint16_t)(vc_addr >> 16);

   // depending on the alignment of the host address do aligned or unaligned
   // reads
   if( ((uint32_t)host_addr)&1 ) {
      // host buffer is unaligned
      uint8_t *dest = (uint8_t *)host_addr;
      uint8_t *dummy_addr = (uint8_t *)&dummy;

      for(idx=0; idx<nblocks; idx++) {
         // wait until the read fifo contains data and then read 32 bits
         // unaligned into the host buffer

         LIMITED_WAIT_FOR(( *vchcs & RFD ) != 0 );

         if (dest != NULL)
         {
            dummy = *vcdata;
            dest[2] = dummy_addr[0];
            dest[3] = dummy_addr[1];
            dummy = *vcdata;
            dest[0] = dummy_addr[0];
            dest[1] = dummy_addr[1];

            dest += 4;
         }
      }
   }
   else
   {
#if VC_HOST_PORT_USE_DMA
      if ( gVcUseDma  && (((int)host_addr & 3) == 0) )
      {
         /* we need to data aligned to 32-bits for DMA to work */

         /* DMA the rest of the data */
         vcHostDmaRead( host_addr, vcdata, vchcs, nblocks << 1, 1 );
      }
      else
#endif
      if ( gVcUseFastXfer )
      {
         uint16_t *dest = (uint16_t *)host_addr;

         // 16 bit aligned buffer on the host

         if ( gVcUse32bitIO )
         {
            while ( nbytes >= 16 )
            {
               uint32_t dummy32;

               // wait until the fifo is full, then read the entire contents

               LIMITED_WAIT_FOR(( *vchcs & RFF ) != 0 );

               dummy32 = *(volatile uint32_t *)vcdata;
               *dest++ = (uint16_t)dummy32;
               *dest++ = (uint16_t)( dummy32 >> 16 );

               dummy32 = *(volatile uint32_t *)vcdata;
               *dest++ = (uint16_t)dummy32;
               *dest++ = (uint16_t)( dummy32 >> 16 );

               dummy32 = *(volatile uint32_t *)vcdata;
               *dest++ = (uint16_t)dummy32;
               *dest++ = (uint16_t)( dummy32 >> 16 );

               dummy32 = *(volatile uint32_t *)vcdata;
               *dest++ = (uint16_t)dummy32;
               *dest++ = (uint16_t)( dummy32 >> 16 );

               nbytes -= 16;
            }
         }
         else
         {
            while ( nbytes >= 16 )
            {
               // wait until the fifo is full, then read the entire contents

               LIMITED_WAIT_FOR(( *vchcs & RFF ) != 0 );

               dest[1] = *vcdata;
               dest[0] = *vcdata;
               dest[3] = *vcdata;
               dest[2] = *vcdata;
               dest[5] = *vcdata;
               dest[4] = *vcdata;
               dest[7] = *vcdata;
               dest[6] = *vcdata;

               dest += 8;
               nbytes -= 16;
            }
         }

         while ( nbytes > 0 )
         {
            LIMITED_WAIT_FOR(( *vchcs & RFD ) != 0 );

            dest[1] = *vcdata;
            dest[0] = *vcdata;

            dest += 2;
            nbytes -= 4;
         }
      }
      else
      {
         // 16 bit aligned buffer on the host
         uint16_t *dest = (uint16_t *)host_addr;

         for ( idx = 0; idx < nblocks; idx++ )
         {
            // wait until the read fifo contains data, then perform read

            LIMITED_WAIT_FOR(( *vchcs & RFD ) != 0 );

            // This function needs to present the data in the same byte order
            // as it is on the VC02. The caller is responsible for performing
            // 16 or 32 bit byte swaps as appropriate. Since our data bus is
            // connected to the VC02 data bus, we actually get the data as
            // 16 bit words, which we need to swap so that the byte order goes
            // back to what it was on the VC02.

            if (dest != NULL)
            {
               dest[1] = *vcdata;
               dest[0] = *vcdata;

               dest += 2;
            }
         }
      }
   }

#if VC_DEBUG_ENABLED
   if ( gVcDebugRead )
   {
       vc_dump_mem_32( "R:", vc_addr, host_addr, nbytes );
   }
#endif

   // reset the rbase register so that FIFO will be cleared more quickly for
   // the next call
   dummy = *vcrbase;

end:
#if VC_DEBUG_ENABLED && defined( KNLLOG )
   if ( gVcProfileTransfers )
   {
      KNLLOG("E: 0x%x %d\n", (int)host_addr, nbytes);
   }
#endif

   // release the lock
   vc_host_lock_release( channel );
   return rc;
}
#endif

/******************************************************************************
NAME
   vc_host_write_consecutive

SYNOPSIS
   int vc_host_write_consecutive (uint32_t vc_addr, void *host_addr, int nbytes,
                                  int channel)

FUNCTION
   Write nbytes consecutive bytes to VideoCore host port channel channel. nbytes
   must be a multiple of 4, vc_addr must be aligned to 4 bytes and channel is 0
   or 1. Returns non-zero for failure

RETURNS
   int
******************************************************************************/

int vc_host_write_consecutive (uint32_t vc_addr, void *host_addr, int nbytes,
                               int channel)
{
   volatile uint16_t    *vcwbase, *vcdata, *vchcs;
   uint16_t             dummy;
   int                  nblocks, idx;
   int                  rc = 0;

   // sanity check on the data (vc_addr aligned to 4 bytes, nbytes is a
   // multiple of 4 and channel is 0 or 1)
   vc_assert( (vc_addr &  3) == 0 );
   vc_assert( (nbytes  &  3) == 0 );
   vc_assert( (channel & ~1) == 0 );

   // setup the pointers to reflect the selected channel registers
   vcwbase = &VC_REG_16( WBASE, channel );
   vcdata  = &VC_REG_16( DATA,  channel );
   vchcs   = &VC_REG_16( HCS,   channel );

   // calc number of 32 bit blocks to copy
   nblocks = nbytes >> 2;

   // acquire channel's host-interface lock
   vc_host_lock_obtain( channel );

#if VC_DEBUG_ENABLED && defined( KNLLOG )
   if ( gVcProfileTransfers )
   {
      KNLLOG("S: 0x%x %d\n", (int)host_addr, nbytes);
   }
#endif

   *vchcs = (WLED | ( gVcHostPortWLED & 0x07 ));
   *vchcs = (RLED | ( gVcHostPortRLED & 0x07 ));

#if VC_DEBUG_ENABLED
   if ( gVcDebugWrite )
   {
       vc_dump_mem( "W:", vc_addr, host_addr, nbytes );
   }
#endif

   // set the write base address
   *vcwbase = (uint16_t)vc_addr;
   *vcwbase = (uint16_t)(vc_addr>>16);

   // depending on host alignment read/write 16bit or 8bit values to the bus

   if( ((uint32_t)host_addr)&1 ) {
      // host unaligned write
      uint8_t *dest = (uint8_t *)host_addr;
      uint8_t *dummy_addr = (uint8_t *)&dummy;

      for( idx=0; idx<nblocks; idx++ )
      {
         dummy_addr[0] = dest[1];
         dummy_addr[1] = dest[0];
         // wait until the write fifo is ready
         LIMITED_WAIT_FOR( *vchcs & WFD );
         *vcdata = dummy;

         dummy_addr[0] = dest[3];
         dummy_addr[1] = dest[2];
         *vcdata = dummy;

         dest += 4;
      }
   }
   else
   {
#if VC_HOST_PORT_USE_DMA
      if ( gVcUseDma  && (((int)host_addr & 3) == 0) )
      {
         /* DMA the rest of the data */
         vcHostDmaWrite( host_addr, vcdata, vchcs, nblocks << 1, 2 );
      }
      else
#endif
      if ( gVcUseFastXfer )
      {
         // host aligned write
         uint16_t *src = (uint16_t *)host_addr;

         if ( gVcUse32bitIO )
         {
            while ( nbytes >= 16 )
            {
               uint32_t dummy32;

               LIMITED_WAIT_FOR( *vchcs & WFE );

               dummy32 = ((uint32_t)VC_HTOV16(src[0]) << 16 ) | (uint32_t)VC_HTOV16(src[1]);
               *(volatile uint32_t *)vcdata = dummy32;

               dummy32 = ((uint32_t)VC_HTOV16(src[2]) << 16 ) | (uint32_t)VC_HTOV16(src[3]);
               *(volatile uint32_t *)vcdata = dummy32;

               dummy32 = ((uint32_t)VC_HTOV16(src[4]) << 16 ) | (uint32_t)VC_HTOV16(src[5]);
               *(volatile uint32_t *)vcdata = dummy32;

               dummy32 = ((uint32_t)VC_HTOV16(src[6]) << 16 ) | (uint32_t)VC_HTOV16(src[7]);
               *(volatile uint32_t *)vcdata = dummy32;

               src += 8;
               nbytes -= 16;
            }
         }
         else
         {
            while ( nbytes >= 16 )
            {
               LIMITED_WAIT_FOR( *vchcs & WFE );

               *vcdata = VC_HTOV16(src[0]);
               *vcdata = VC_HTOV16(src[1]);
               *vcdata = VC_HTOV16(src[2]);
               *vcdata = VC_HTOV16(src[3]);
               *vcdata = VC_HTOV16(src[4]);
               *vcdata = VC_HTOV16(src[5]);
               *vcdata = VC_HTOV16(src[6]);
               *vcdata = VC_HTOV16(src[7]);

               src += 8;
               nbytes -= 16;
            }
         }

         while ( nbytes > 0 )
         {
            LIMITED_WAIT_FOR( *vchcs & WFD );

            *vcdata = VC_HTOV16(src[0]);
            *vcdata = VC_HTOV16(src[1]);

            src += 2;
            nbytes -= 4;
         }
      }
      else
      {
         // host aligned write
         uint16_t *dest = (uint16_t *)host_addr;

         for( idx=0; idx<nblocks; idx++ )
         {
            LIMITED_WAIT_FOR( *vchcs & WFD );

            dummy = VC_HTOV16(dest[0]);
            *vcdata = dummy;
            dummy = VC_HTOV16(dest[1]);
            *vcdata = dummy;

            dest += 2;
         }
      }
   }

end:
#if VC_DEBUG_ENABLED && defined( KNLLOG )
   if ( gVcProfileTransfers )
   {
      KNLLOG("E: 0x%x %d\n", (int)host_addr, nbytes);
   }
#endif

   // release channel's host interface lock
   vc_host_lock_release( channel );
   return rc;
}

#if defined( VC_HOST_IS_BIG_ENDIAN )

/******************************************************************************
NAME
   vc_host_write_byteswapped

SYNOPSIS
   int vc_host_write_byteswapped (uint32_t vc_addr, void *host_addr, int nbytes,
                                  int channel)

FUNCTION
   Write nbytes consecutive bytes to VideoCore host port channel channel. nbytes
   must be a multiple of 4, vc_addr must be aligned to 4 bytes and channel is 0
   or 1. Returns non-zero for failure

RETURNS
   int
******************************************************************************/

int vc_host_write_byteswapped (uint32_t vc_addr, void *host_addr, int nbytes,
                               int channel)
{
   volatile uint16_t    *vcwbase, *vcdata, *vchcs;
   uint16_t             dummy;
   int                  nblocks, idx;
   int                  rc = 0;

   // sanity check on the data (vc_addr aligned to 4 bytes, nbytes is a
   // multiple of 4 and channel is 0 or 1)
   vc_assert( (vc_addr &  3) == 0 );
   vc_assert( (nbytes  &  3) == 0 );
   vc_assert( (channel & ~1) == 0 );

   // setup the pointers to reflect the selected channel registers
   vcwbase = &VC_REG_16( WBASE, channel );
   vcdata  = &VC_REG_16( DATA,  channel );
   vchcs   = &VC_REG_16( HCS,   channel );

   // calc number of 32 bit blocks to copy
   nblocks = nbytes >> 2;

   // acquire channel's host-interface lock
   vc_host_lock_obtain( channel );

#if VC_DEBUG_ENABLED && defined( KNLLOG )
   if ( gVcProfileTransfers )
   {
      KNLLOG("S: 0x%x %d\n", (int)host_addr, nbytes);
   }
#endif

   *vchcs = (WLED | ( gVcHostPortWLED & 0x07 ));
   *vchcs = (RLED | ( gVcHostPortRLED & 0x07 ));

#if VC_DEBUG_ENABLED
   if ( gVcDebugWrite )
   {
       vc_dump_mem_16( "W:", vc_addr, host_addr, nbytes );
   }
#endif

   // set the write base address
   *vcwbase = (uint16_t)vc_addr;
   *vcwbase = (uint16_t)(vc_addr>>16);

   // depending on host alignment read/write 16bit or 8bit values to the bus

   if( ((uint32_t)host_addr)&1 ) {
      // host unaligned write
      uint8_t *dest = (uint8_t *)host_addr;
      uint8_t *dummy_addr = (uint8_t *)&dummy;

      for( idx=0; idx<nblocks; idx++ )
      {
         dummy_addr[0] = dest[0];
         dummy_addr[1] = dest[1];
         // wait until the write fifo is ready
         LIMITED_WAIT_FOR( *vchcs & WFD );
         *vcdata = dummy;

         dummy_addr[0] = dest[2];
         dummy_addr[1] = dest[3];
         *vcdata = dummy;

         dest += 4;
      }
   }
   else
   {
#if VC_HOST_PORT_USE_DMA
      if ( gVcUseDma  && (((int)host_addr & 3) == 0) )
      {
         /* DMA the rest of the data */
         vcHostDmaWrite( host_addr, vcdata, vchcs, nblocks << 1, 0 );
      }
      else
#endif
      if ( gVcUseFastXfer )
      {
         // host 16 bit aligned write
         //
         // If we wait for the WFE bit (Write FIFO empty), then we can write
         // 8 x 16 bit words at a time.

         uint16_t *src;

         if ( gVcUse32bitIO )
         {
            uint32_t *src32 = (uint32_t *)host_addr;

            while ( nbytes >= 16 )
            {
               LIMITED_WAIT_FOR( *vchcs & WFE );

               *(volatile uint32_t *)vcdata = *src32++;
               *(volatile uint32_t *)vcdata = *src32++;
               *(volatile uint32_t *)vcdata = *src32++;
               *(volatile uint32_t *)vcdata = *src32++;

               nbytes -= 16;
            }
            src = (uint16_t *)src32;
         }
         else
         {
            src = (uint16_t *)host_addr;

            while ( nbytes >= 16 )
            {
               LIMITED_WAIT_FOR( *vchcs & WFE );

               *vcdata = *src++;
               *vcdata = *src++;
               *vcdata = *src++;
               *vcdata = *src++;
               *vcdata = *src++;
               *vcdata = *src++;
               *vcdata = *src++;
               *vcdata = *src++;

               nbytes -= 16;
            }
         }

         // Now write out the remainder

         while ( nbytes > 0 )
         {
            LIMITED_WAIT_FOR( *vchcs & WFD );

            *vcdata = *src++;
            *vcdata = *src++;

            nbytes -= 4;
         }
      }
      else
      {
         // host aligned write
         uint16_t *dest = (uint16_t *)host_addr;

         for( idx=0; idx<nblocks; idx++ )
         {
            LIMITED_WAIT_FOR( *vchcs & WFD );

            *vcdata = *dest++;
            *vcdata = *dest++;
         }

      }
   }

end:

#if VC_DEBUG_ENABLED && defined( KNLLOG )
   if ( gVcProfileTransfers )
   {
      KNLLOG("E: 0x%x %d\n", (int)host_addr, nbytes);
   }
#endif

   // release channel's host interface lock
   vc_host_lock_release( channel );
   return rc;
}

/******************************************************************************
NAME
   vc_host_write_byteswapped

SYNOPSIS
   int vc_host_write_byteswapped (uint32_t vc_addr, void *host_addr, int nbytes,
                                  int channel)

FUNCTION
   Write nbytes consecutive bytes to VideoCore host port channel channel. nbytes
   must be a multiple of 4, vc_addr must be aligned to 4 bytes and channel is 0
   or 1. Returns non-zero for failure

RETURNS
   int
******************************************************************************/

int vc_host_write_byteswapped_32 (uint32_t vc_addr, void *host_addr, int nbytes,
                               int channel)
{
   volatile uint16_t    *vcwbase, *vcdata, *vchcs;
   uint16_t             dummy;
   int                  nblocks, idx;
   int                  rc = 0;

   // sanity check on the data (vc_addr aligned to 4 bytes, nbytes is a
   // multiple of 4 and channel is 0 or 1)
   vc_assert( (vc_addr &  3) == 0 );
   vc_assert( (nbytes  &  3) == 0 );
   vc_assert( (channel & ~1) == 0 );

   // setup the pointers to reflect the selected channel registers
   vcwbase = &VC_REG_16( WBASE, channel );
   vcdata  = &VC_REG_16( DATA,  channel );
   vchcs   = &VC_REG_16( HCS,   channel );

   // calc number of 32 bit blocks to copy
   nblocks = nbytes >> 2;

   // acquire channel's host-interface lock
   vc_host_lock_obtain( channel );

#if VC_DEBUG_ENABLED && defined( KNLLOG )
   if ( gVcProfileTransfers )
   {
      KNLLOG("S: 0x%x %d\n", (int)host_addr, nbytes);
   }
#endif

   *vchcs = (WLED | ( gVcHostPortWLED & 0x07 ));
   *vchcs = (RLED | ( gVcHostPortRLED & 0x07 ));

#if VC_DEBUG_ENABLED
   if ( gVcDebugWrite )
   {
       vc_dump_mem_32( "W:", vc_addr, host_addr, nbytes );
   }
#endif

   // set the write base address
   *vcwbase = (uint16_t)vc_addr;
   *vcwbase = (uint16_t)(vc_addr>>16);

   // depending on host alignment read/write 16bit or 8bit values to the bus

   if( ((uint32_t)host_addr)&1 ) {
      // host unaligned write
      uint8_t *dest = (uint8_t *)host_addr;
      uint8_t *dummy_addr = (uint8_t *)&dummy;

      for( idx=0; idx<nblocks; idx++ )
      {
         dummy_addr[0] = dest[2];
         dummy_addr[1] = dest[3];
         // wait until the write fifo is ready
         LIMITED_WAIT_FOR( *vchcs & WFD );
         *vcdata = dummy;

         dummy_addr[0] = dest[0];
         dummy_addr[1] = dest[1];
         *vcdata = dummy;

         dest += 4;
      }
   }
   else
   {
#if VC_HOST_PORT_USE_DMA
      if ( gVcUseDma  && (((int)host_addr & 3) == 0) )
      {
         /* we need to data aligned to 32-bits for DMA to work */

         /* DMA the rest of the data */
         vcHostDmaWrite( host_addr, vcdata, vchcs, nblocks << 1, 1 );
      }
      else
#endif
      if ( gVcUseFastXfer )
      {
         // host aligned write
         //
         // If we wait for the WFE bit (Write FIFO empty), then we can write
         // 8 x 16 bit words at a time.

         uint16_t *src = (uint16_t *)host_addr;

         if ( gVcUse32bitIO )
         {
            while ( nbytes >= 16 )
            {
               LIMITED_WAIT_FOR( *vchcs & WFE );

               *(volatile uint32_t *)vcdata = ((uint32_t)src[ 1 ] << 16 ) | (uint32_t)src[ 0 ];
               *(volatile uint32_t *)vcdata = ((uint32_t)src[ 3 ] << 16 ) | (uint32_t)src[ 2 ];
               *(volatile uint32_t *)vcdata = ((uint32_t)src[ 5 ] << 16 ) | (uint32_t)src[ 4 ];
               *(volatile uint32_t *)vcdata = ((uint32_t)src[ 7 ] << 16 ) | (uint32_t)src[ 6 ];

               src += 8;
               nbytes -= 16;
            }
         }
         else
         {
            while ( nbytes >= 16 )
            {
               LIMITED_WAIT_FOR( *vchcs & WFE );

               *vcdata = src[ 1 ];
               *vcdata = src[ 0 ];
               *vcdata = src[ 3 ];
               *vcdata = src[ 2 ];
               *vcdata = src[ 5 ];
               *vcdata = src[ 4 ];
               *vcdata = src[ 7 ];
               *vcdata = src[ 6 ];

               src += 8;
               nbytes -= 16;
            }
         }

         // Now write out the remainder

         while ( nbytes > 0 )
         {
            LIMITED_WAIT_FOR( *vchcs & WFD );

            *vcdata = src[ 1 ];
            *vcdata = src[ 0 ];

            src += 2;
            nbytes -= 4;
         }
      }
      else
      {
         // host aligned write
         uint16_t *dest = (uint16_t *)host_addr;

         for( idx=0; idx<nblocks; idx++ )
         {
            LIMITED_WAIT_FOR( *vchcs & WFD );

            *vcdata = dest[ 1 ];
            *vcdata = dest[ 0 ];

            dest += 2;
         }
      }
   }

end:
#if VC_DEBUG_ENABLED && defined( KNLLOG )
   if ( gVcProfileTransfers )
   {
      KNLLOG("E: 0x%x %d\n", (int)host_addr, nbytes);
   }
#endif

   // release channel's host interface lock
   vc_host_lock_release( channel );
   return rc;
}

#endif  // VC_HOST_IS_BIG_ENDIAN

/******************************************************************************
NAME
   vc_host_send_interrupt

SYNOPSIS
   int vc_host_send_interrupt(int channel)

FUNCTION
   Cause the host to send VideoCore an interrupt. Non-zero means failure.

RETURNS
   int
******************************************************************************/

int vc_host_send_interrupt( int channel ) 
{
    vc_assert( (channel & ~1) == 0 );
    
    VC_DEBUG( MsgFifo, "channel: %d\n", channel );
    
    vc_host_lock_obtain( channel );
    
    if( channel == 0 )
        VCHCS0 = (INT | TRIGGER_INT);
    else
        VCHCS1 = (INT | TRIGGER_INT);
    
    vc_host_lock_release( channel );
    return 0;
}

uint16_t vc_host_read_reg( int reg, int channel )
{
    return VC_REG_16( reg,  channel );
}

void vc_host_write_reg( int reg, int channel, uint16_t val )
{
    VC_REG_16( reg,  channel ) = val;
}

/****************************************************************************
*
*  vc_dump_mem
*
*   Used to dumping memory bytes
*
***************************************************************************/

void vc_dump_mem( const char *label, uint32_t addr, const void *voidMem, size_t numBytes )
{
    const uint8_t  *mem = (uint8_t *)voidMem;
    size_t          offset;
    char            lineBuf[ 80 ];
    char           *s;

    while ( numBytes > 0 )
    {
        s = lineBuf;

        for ( offset = 0; offset < 16; offset++ )
        {
            if ( offset < numBytes )
            {
                s += sprintf( s, "%02x ", mem[ offset ]);
            }
            else
            {
                s += sprintf( s, "   " );
            }
        }

        for ( offset = 0; offset < 16; offset++ )
        {
            if ( offset < numBytes )
            {
                uint8_t ch = mem[ offset ];

                if (( ch < ' ' ) || ( ch > '~' ))
                {
                    ch = '.';
                }
                *s++ = (char)ch;
            }
        }
        *s++ = '\0';

        PRINT_INFO( "%s %08x: %s\n", label, addr, lineBuf );

        addr += 16;
        mem += 16;
        numBytes -= 16;
    }

} // vc_dump_mem

void vc_dump_mem_16( const char *label, uint32_t addr, const void *voidMem, size_t numBytes )
{
    const uint16_t *mem = (uint16_t *)voidMem;
    size_t          offset;
    char            lineBuf[ 80 ];
    char           *s;

    int numWords = numBytes >> 1;

    while ( numWords > 0 )
    {
        s = lineBuf;

        for ( offset = 0; offset < 8; offset ++ )
        {
            if ( offset < numBytes )
            {
                s += sprintf( s, "%04x ", mem[ offset ]);
            }
            else
            {
                s += sprintf( s, "   " );
            }
        }

        *s++ = '\0';

        PRINT_INFO( "%s %08x: %s\n", label, addr, lineBuf );

        addr += 16;
        mem += 8;
        numWords -= 8;
    }

} // vc_dump_mem_16

void vc_dump_mem_32( const char *label, uint32_t addr, const void *voidMem, size_t numBytes )
{
    const uint32_t *mem = (uint32_t *)voidMem;
    size_t          offset;
    char            lineBuf[ 80 ];
    char           *s;

    int numWords = numBytes >> 2;

    while ( numWords > 0 )
    {
        s = lineBuf;

        for ( offset = 0; offset < 4; offset ++ )
        {
            if ( offset < numBytes )
            {
                s += sprintf( s, "%08x ", mem[ offset ]);
            }
            else
            {
                s += sprintf( s, "   " );
            }
        }

        *s++ = '\0';

        PRINT_INFO( "%s %08x: %s\n", label, addr, lineBuf );

        addr += 16;
        mem += 4;
        numWords -= 4;
    }

} // vc_dump_mem_32

