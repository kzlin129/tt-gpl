/**************************************************************************x`***
* Copyright 2009 Broadcom Corporation.  All rights reserved.
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


#if defined( __KERNEL__ )

#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/broadcom/knllog.h>
#include <linux/crc32.h>
#include <linux/time.h>
#include <linux/broadcom/vc.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/broadcom/videocore_settings.h>
#include <csp/pifHw.h>
#include <linux/broadcom/vc03/vchost_config.h>
#include <linux/broadcom/vc03/vc03b0_hostport.h>
#include <linux/dma-mapping.h>
#include <linux/broadcom/gpio_irq.h>
#include <linux/broadcom/gpio_types.h>
#include <linux/broadcom/gpio.h>
#include <linux/broadcom/timer.h>
#include <asm/arch/dma.h>
#define DO_DMA    1

//#   define  PRINT_INFO( fmt, args... )  printk( fmt, ## args )
#   define  PRINT_INFO( fmt, args... ) 

#else

//#include <bcmtypes.h>
#include <kerneltypes.h>
   typedef UINT32 u32;
//#include <linux/autoconf.h>
#include <linux/broadcom/gpio.h>
#   undef __KERNEL__    // kerneltypes.h defines this and it confuses us
#include <string.h>
#include <printf.h>

#   define  PRINT_INFO( fmt, args... )  printf( fmt, ## args )

#define DO_DMA    0

#define vc_assert(expr) \
    if ( expr ) {} else \
    { \
        printf( "Assertion (%s) failed: FILE: %s LINE: %d\n", #expr, __FILE__, __LINE__ ); \
        while(1) {;} \
    }
#define VC_HTOV32(val) (val)
#define VC_HTOV16(val) (val)
#include <linux/elf.h>
#endif

#include <linux/broadcom/vc03/vc03b0_hostport.h>
#include <linux/broadcom/videocore_settings.h>
#include "cfg_global.h"

#if !defined(HW_VC03_RUN_GPIO)
#error define HW_VC03_RUN_GPIO 
#endif

#define LOCAL_HOST_BASE 0x1000

/*******
 *  *** this should go to a header file
 */
#define VC03_BASE_ADDR  HW_VC02_BASE_ADDR

#define PIF_DEV_SETTING_DIRECT_MODE 0
#define PIF_DEV_SETTING_B0_HS       3

// These should also come from a PIF header file

#define PIF_A4_A3_HIGH_LOW_CS0  (1<<4)     /* CS0 select */
#define PIF_A4_A3_LOW_HIGH_CS1  (1<<3)     /* CS1 select */
#define VC03_B0_MSG_STATUS_REG(ch)  ( (ch) ?  PIF_A4_A3_LOW_HIGH_CS1      : PIF_A4_A3_HIGH_LOW_CS0 )
#define VC03_B0_MSG_DATA_REG(ch)    ( (ch) ? (PIF_A4_A3_LOW_HIGH_CS1 + 1) : (PIF_A4_A3_HIGH_LOW_CS0 + 1) )

// Header at the start of all messages.  
// It is 32 bytes long and is followed by the message body
//
// Header fields:
//   magic        Magic number
//   seq          Sequence number, starts at 1 and is incremented in each message. 
//                The sequence numbers in each channel are independent.
//   length       Number of bytes in the body.  
//                Must be a multiple of 16 as required by MPHI.
//   
//   Checksums    A checksum is formed by treating the message body as an array
//                of 32-bit words and calculating their sum.
//   host_body_checksum
//                In messages from the host to VC, this is the checksum of the body.
//                In CTRL messages from VC to the host, this is the 
//                value of host_body_checksum in the most recent CTRL message from the host.
//                In DATA messages from VC to the host, this is the 
//                value of host_body_checksum in the most recent DATA message from the host.
//                Set to zero initially when there is no most recent message.
//   vc_body_checksum
//                In messages from VC to the host, this is the checksum of the body.
//                In CTRL messages from the host to VC this is the 
//                value of vc_body_checksum in the most recent CTRL message from VideoCore.
//                In DATA messages from the host to VC this is the 
//                value of vc_body_checksum in the most recent DATA message from VideoCore.
//                Set to zero initially when there is no most recent message.
//
//   padding      Padding words are not currently used and should be set to zero
typedef struct
{
   uint32_t    magic;            // magic number
   uint32_t    seq;              // sequence number, increment for each message on this channel
   uint32_t	   length;           // body length
   uint32_t    padding1;		   // -
   //
   uint32_t    host_body_checksum;    // checksum
   uint32_t    vc_body_checksum;    
   uint32_t    padding2;
   uint32_t    padding3;
} MPHI_HEADER_T;

#define  HOST_MESSAGE_MAGIC   0xA5AA2569

//int gVcDebugTrace = 1;
//int gVcDebugMsgFifo = 1;

#if ( DO_DMA == 1 )

DMA_MemMap_t        gDmaMemMap;
struct completion   gDmaDone;

#endif

typedef struct
{
    uint8_t    *headPtr;
    uint8_t    *bodyPtr;
    uint8_t    *tailPtr;

    size_t      headSize;
    size_t      bodySize;
    size_t      tailSize;

} AlignInfo_t;


uint32_t dma_chunk_size = 8000;

#if 0

/*
 *
 */
void vchost_gpio_pintype_set(int pin, int pintype, int interrupttype)
{
  // gpio controller
  // set pintype
  printk("vchost_gpio_pintype_set being called\n");

  if ( pintype == GPIO_PIN_TYPE_OUTPUT )
  {
      gpio_direction_output( pin, gpio_get_value( pin ));
  }
  else
  {
      gpio_direction_input( pin );
  }
}
void vchost_gpio_pinval_set(int pin, int pinval)
{
   gpio_set_value( pin, pinval );
}
#endif

#if !defined( __KERNEL__ )
int vc03_host_load_elf( char * img, uint32_t img_size, 
                         char ** second_stage_ptr, uint32_t * second_stage_size, 
                         char ** final_ptr, uint32_t * final_size );
#endif
/****************************************************************************
*
*  InitGPIO
*
*     Initializes the VC02 GPIO pins.
*
***************************************************************************/

int vchost_pininit( void )
{
    PRINT_INFO("vc_host_init_pins called\n");

    // We reserve the pins in the portconfig function since the pifHw_Init routine
    // requires that the pins already be reserved.

    return 0;

} // vchost_pininit

#if defined( __KERNEL__ )
/*
 * return value:
 *   1 if the driver will handle host port gpio irq 
 */
int vchost_gpioirq_handle(int irq,void *devId)
{  
  return 1;
}

static irq_handler_t    gVcIrqHandler;

static irqreturn_t vchost_pif_irq( int irq, void *devId )
{
    uint32_t    smics = PIFHW_READ_REG32( PIFHW_REG32_SMICS );
    uint32_t    smiei = PIFHW_READ_REG32( PIFHW_REG32_SMIEI );

    if ((( smiei & PIFHW_REG32_SMIEI_EIEN1 ) != 0 ) && (( smiei & PIFHW_REG32_SMIEI_EIS1 ) != 0 ))
    {
        VC_DEBUG( IrqTrace, "HAT1 fired" );

        // HAT 1 interrupt has occurred

        // Since the HAT interrupt is active high, and we don't have the ability to make it edge
        // senstive, we disable the interrupt here, and rely on it being re-enabled in the
        // vchost_msgavail function when no more messages are available

        PIFHW_CLEAR_REG32_BIT( PIFHW_REG32_SMIEI, PIFHW_REG32_SMIEI_EIEN1);     // Disable HAT1 interrupt

        // Call the original handler

        return gVcIrqHandler( irq, devId );
    }

    if ((( smiei & PIFHW_REG32_SMIEI_EIEN2 ) != 0 ) && (( smiei & PIFHW_REG32_SMIEI_EIS2 ) != 0 ))
    {
        VC_DEBUG( IrqTrace, "HAT2 fired" );

        // HAT 2 interrupt has occurred

        printk( KERN_ERR "%s: Unxpected HAT2 IRQ\n", __FUNCTION__ );

        // Disable the interrupt

        PIFHW_CLEAR_REG32_BIT( PIFHW_REG32_SMIEI, PIFHW_REG32_SMIEI_EIEN2 );
    }

    if ((( smics & PIFHW_REG32_SMICS_INTR ) != 0 ) && (( smics & PIFHW_REG32_SMICS_RXR ) != 0 ))
    {
        VC_DEBUG( IrqTrace, "RX FIFO needs reading" );

        printk( KERN_ERR "%s: Unexpected RX FIFO needs reading IRQ\n", __FUNCTION__ );

        // Disable the interrupt

        PIFHW_CLEAR_REG32_BIT( PIFHW_REG32_SMICS, PIFHW_REG32_SMICS_INTR );
    }

    if ((( smics & PIFHW_REG32_SMICS_INTT ) != 0 ) && (( smics & PIFHW_REG32_SMICS_TXW ) != 0 ))
    {
        VC_DEBUG( IrqTrace, "TX FIFO needs writing" );

        printk( KERN_ERR "%s: Unxpected TX FIFO needs writing IRQ\n", __FUNCTION__ );

        // Disable the interrupt

        PIFHW_CLEAR_REG32_BIT( PIFHW_REG32_SMICS, PIFHW_REG32_SMICS_INTT );
    }

    if ((( smics & PIFHW_REG32_SMICS_INTD ) != 0 ) && (( smics & PIFHW_REG32_SMICS_DONE ) != 0 ))
    {
        VC_DEBUG( IrqTrace, "Xfer DONE" );

        printk( KERN_ERR "%s: Unxpected DONE interript\n", __FUNCTION__ );

        // Disable the interrupt

        PIFHW_CLEAR_REG32_BIT(   PIFHW_REG32_SMICS, PIFHW_REG32_SMICS_INTD );
    }

    return IRQ_HANDLED;
}

int vchost_request_irq( irq_handler_t irqHandler, void *devId )
{
    int ret;

    // Notes about the HAT interrupt:
    //
    // The HAT interrupt can only be configured to be active high or active low, and it stays
    // active as long as there is data in the VideoCore FIFO.
    // 
    // The VideoCore driver interrupt handler doesn't actually read the data from the FIFO, it just 
    // signals a task to perform this operation. So we need to disable the HAT interrupt when
    // it fires, and then re-enable it again at a later point in time. We pick the "Transfer Done"
    // interrupt as this point in time.

    gVcIrqHandler = irqHandler;

    PIFHW_SET_REG32_BIT(   PIFHW_REG32_SMIEI, PIFHW_REG32_SMIEI_HAT1POL );  // Set HAT1 to be Active High
    PIFHW_SET_REG32_BIT(   PIFHW_REG32_SMIEI, PIFHW_REG32_SMIEI_HAT2POL );  // Set HAT2 to be Active High

    PIFHW_SET_REG32_BIT(   PIFHW_REG32_SMIEI, PIFHW_REG32_SMIEI_EIEN1);     // Enable HAT1 to generate interrupts
    PIFHW_CLEAR_REG32_BIT( PIFHW_REG32_SMIEI, PIFHW_REG32_SMIEI_EIEN2 );    // Disable HAT2 interrupt

    PIFHW_CLEAR_REG32_BIT( PIFHW_REG32_SMICS, PIFHW_REG32_SMICS_INTR );     // Disable RXR interrupt
    PIFHW_CLEAR_REG32_BIT( PIFHW_REG32_SMICS, PIFHW_REG32_SMICS_INTT );     // Disable TXW interrupt
    PIFHW_CLEAR_REG32_BIT( PIFHW_REG32_SMICS, PIFHW_REG32_SMICS_INTD );     // Disable DONE interrupt

    VC_DEBUG( IrqTrace, "irq: %d", IRQ_PIF );
    ret = request_irq( IRQ_PIF, vchost_pif_irq, IRQF_DISABLED, "pif_irq", devId );
    if (ret != 0)
    {
       printk( KERN_WARNING "VideoCore: failed to register isr=%d ret=%d\n", IRQ_PIF, ret);
       return -1;
    }   

    return 0;
}

void vchost_free_irq( void *devId )
{
    VC_DEBUG( IrqTrace, "" );

    free_irq( IRQ_PIF, devId );
}

static void vchost_dma_handler( DMA_Device_t dev, int reason, void *userData )
{
    (void)dev;
    (void)reason;
    (void)userData;

    complete( &gDmaDone );
}

#endif

#if !defined( __KERNEL__ )
int vc03_host_load_elf( char * img, uint32_t img_size, 
                         char ** second_stage_ptr, uint32_t * second_stage_size, 
                         char ** final_ptr, uint32_t * final_size )
{
   int i;
   Elf32_Ehdr  exeHdr;
   Elf32_Shdr  secHdr, strtabHdr;
   Elf32_Off shstrtab_offset;

   *second_stage_ptr = NULL;
   *final_ptr = NULL;
   *second_stage_size = 0;
   *final_size = 0;

   memcpy( &exeHdr, img, sizeof( exeHdr ));
   
   exeHdr.e_phnum     = VC_HTOV16( exeHdr.e_phnum );
   exeHdr.e_shnum     = VC_HTOV16( exeHdr.e_shnum );
   exeHdr.e_shoff     = VC_HTOV32( exeHdr.e_shoff );
   exeHdr.e_shstrndx  = VC_HTOV16( exeHdr.e_shstrndx );
   exeHdr.e_shentsize = VC_HTOV16( exeHdr.e_shentsize );

   /* we do not support elf files with program headers */
   vc_assert( (exeHdr.e_phnum == 0) && exeHdr.e_shnum );
   shstrtab_offset = exeHdr.e_shoff + ( exeHdr.e_shstrndx * exeHdr.e_shentsize );

   memcpy( &strtabHdr, &img[shstrtab_offset], sizeof(secHdr) );
   strtabHdr.sh_offset = VC_HTOV32( strtabHdr.sh_offset );

   PRINT_INFO( "vc03_host_load_elf, number of section header %d, offset 0x%x, shstrtab %d offset %d, sec hdr size %d\n",
               exeHdr.e_shnum, exeHdr.e_shoff, exeHdr.e_shstrndx,
               strtabHdr.sh_offset, exeHdr.e_shentsize );

   /* find out the section header str table */

   for( i=0; i < exeHdr.e_shnum; i++ )
   {
      uint8_t * strPtr;
      Elf32_Off secHdrOffset = exeHdr.e_shoff + ( i * exeHdr.e_shentsize );
      if( secHdrOffset >= img_size )
      {
         PRINT_INFO( "secHdrOffset %d is not within buffer of size %d\n", 
               secHdrOffset, img_size );
         return -1;
      }
      memcpy( &secHdr, &img[secHdrOffset], sizeof( secHdr ) );
      secHdr.sh_type  = VC_HTOV32( secHdr.sh_type );
      if( secHdr.sh_type == SHT_PROGBITS )
      {
         secHdr.sh_name  = VC_HTOV32( secHdr.sh_name );
         strPtr = (uint8_t *)(img + strtabHdr.sh_offset + secHdr.sh_name);
         if( (strPtr[0] == '.') &&
             (strPtr[1] == 'b') &&
             (strPtr[2] == 'o') &&
             (strPtr[3] == 'o') &&
             (strPtr[4] == 't') )
         {
            secHdr.sh_offset = VC_HTOV32( secHdr.sh_offset );
            secHdr.sh_size = VC_HTOV32( secHdr.sh_size );

            *second_stage_ptr = &img[secHdr.sh_offset];
            *second_stage_size = secHdr.sh_size;
            PRINT_INFO( "BOOT section found %d, size %d\n", i, *second_stage_size);
         }
         else if( (strPtr[0] == '.') &&
                  (strPtr[1] == 't') &&
                  (strPtr[2] == 'e') &&
                  (strPtr[3] == 'x') &&
                  (strPtr[4] == 't') )
         {
            secHdr.sh_offset = VC_HTOV32( secHdr.sh_offset );
            secHdr.sh_size = VC_HTOV32( secHdr.sh_size );

            *final_ptr = &img[secHdr.sh_offset];
            *final_size = secHdr.sh_size;
            
            PRINT_INFO( "FINAL_IMAGE_FOUND %d, sizeof %d\n", i, *final_size );
         }
      }           
      else
      {
         PRINT_INFO(" section %d (type %d) not part of consideration \n", i, secHdr.sh_type );
      }
   }
   

   return 0;
}
#endif

/****************************************************************************
*
*  vc_host_reset
*
*     Performs reset sequence on the VC02
*
***************************************************************************/

void vchost_reset( void )
{
    PRINT_INFO("vc_host_reset\n");
  
    vchost_portinit( NULL );

    vchost_pininit();

    PRINT_INFO( "Setting VideoCore RUN pin low (GPIO %d)\n", HW_VC03_RUN_GPIO  );
    gpio_direction_output( HW_VC03_RUN_GPIO, 0 );
    vchost_delay_msec(10);
    
    PRINT_INFO( "Setting VideoCore RUN pin high (GPIO %d)\n", HW_VC03_RUN_GPIO  );
    gpio_set_value( HW_VC03_RUN_GPIO, 1 );

    // Delay for 500 milliseconds to give the videocore a chance to initialize
    // Without this, if we try and depost a message into the MPHI buffers too soon
    // then the interface will hang.

    vchost_delay_msec( 500 );

} // vc_host_reset

typedef struct
{
  uint32_t header;
  uint32_t id_size;
  uint32_t signature[5];
  uint32_t footer;
  
} boot_message2_t;

#if !defined( __KERNEL__ )
void vc03_host_boot( void * img_ptr, uint32_t img_size )
{
   uint8_t *signature;
   uint32_t msg[8];
   int length;
   uint32_t *ptr;
   boot_message2_t msg2;
   int iter;
   int i;
   uint32_t checksum = 0;
   char * img = (char *)img_ptr;
   char * second_stage_ptr;
   char * final_ptr;
   uint32_t second_size, final_size;
   
#if defined( __KERNEL__ )
   uint32_t local_chunk = dma_chunk_size;
#endif
   PRINT_INFO( "vc03_host_boot called, img_size = %d\n", img_size );

   signature = (uint8_t *)img_ptr;

   if (( signature[0] == 0x7f )
         &&  ( signature[1] == 'E' )
         &&  ( signature[2] == 'L' )
         &&  ( signature[3] == 'F' ))
   {
      PRINT_INFO( "vc03 ELF image \n");
      vc03_host_load_elf( img, img_size, 
                          &second_stage_ptr, &second_size,
                          &final_ptr, &final_size );
   }
   else
   {
      PRINT_INFO( "ERROR!!! this is not an elf image\n");
      return;
   }
   //return;

  /* read a VC03 message to start booting from the host */
  {
     /* 2nd stage boot */
     memset( msg, 0, sizeof( msg ) );

     length = vchost_readmsg( (void *)msg, sizeof(msg), 0 );
     PRINT_INFO(" message from VC03 read (%d)\n", length);
     for( i=0; i < (int)(length/sizeof(uint32_t));i++)
     {
         PRINT_INFO( "0x%X\n", msg[i] );
     }

#if 0
#if defined( __KERNEL__ )
     if( dma_chunk_size != 8000 )
     {
         host_writemsg( trial, 120000, 3, 2 );
     }
#endif
#endif
     {
#if defined( __KERNEL__ )
        dma_chunk_size = 8000;
#endif
         host_writemsg( second_stage_ptr, second_size, 3, 2 );
#if defined( __KERNEL__ )
         dma_chunk_size = local_chunk;
#endif
     }

     memset( &msg2, 0, sizeof(msg2) );
     msg2.header = 0xAFE45251;   // VC03 magic
     msg2.id_size = (2 << 24) | (second_size & 0xffffff);
     msg2.footer = 0xF001BC4E;

     ptr = (uint32_t *)&msg2;
     PRINT_INFO("writing control message to VC03, len %d\n", sizeof(msg2));
     for( i=0; i < (int)(sizeof(msg2)/sizeof(uint32_t));i++)
     {
         PRINT_INFO( "0x%X\n", ptr[i] );
     }

     host_writemsg( &msg2, sizeof(msg2), 2, 1);

     /* write to address 0 to indicate end of message */
     host_write_vc03addr( 0, 0 );
     
  }
  {
     int gpio_val;
     iter = 500;
     do
     {
        gpio_val = gpio_get_value( HW_GPIO_VID_INT_PIN );
        vc_host_delay_msec( 1 );
        iter--;
     }while( (iter > 0) && (gpio_val == 0) );

     PRINT_INFO("trial = %d, result = %d\n", (500 - iter), gpio_val );
     if( gpio_val != 0 )
     {
        /* new message from VC03 ready, go to the final boot stage */
        memset( msg, 0, sizeof( msg ) );
        length = vchost_readmsg( (void *)msg, sizeof(msg), 0 );
        PRINT_INFO(" message from VC03 read (%d)\n", length);
        for( i=0; i < (int)(length/sizeof(uint32_t));i++)
        {
            PRINT_INFO( "0x%X\n", msg[i] );
        }

        memset( &msg2, 0, sizeof( msg2 ) );
        checksum = 0;
        for( i=0; i < (int)img_size; i++ )
        {
            checksum += (uint32_t)(img[i]);
        }
        host_writemsg( final_ptr, final_size, 3, 2 );

        msg2.id_size = (2 << 24) | (img_size & 0xffffff);
        msg2.header = 0xAFE45251;
        msg2.footer = 0xF001BC4E;
        ptr = (uint32_t *)&msg2;
        PRINT_INFO("writing control message to VC03, len %d checksum 0x%X\n", sizeof(msg2), checksum);
        for( i=0; i < (int)(sizeof(msg2)/sizeof(uint32_t));i++)
        {
            PRINT_INFO( "0x%X\n", ptr[i] );
        }

        host_writemsg( &msg2, sizeof(msg2), 2, 1);

        /* write to address 0 to indicate end of message */
        host_write_vc03addr( 0, 0 );
     }
     else
     {
        /* stop here */
        PRINT_INFO ("stop here !!!!\n");
     }
  }
  {
     int gpio_val;
     iter = 500;
     do
     {
        gpio_val = gpio_get_value( HW_GPIO_VID_INT_PIN );
        vc_host_delay_msec( 1 );
        iter --;
     } while( (iter > 0) && (gpio_val == 0) );
     PRINT_INFO("trial = %d, result = %d\n", (500 - iter), gpio_val );
     if( gpio_val != 0 )
     {
        /* new message from VC03 ready, go to the final boot stage */
        memset( msg, 0, sizeof( msg ) );
        length = vchost_readmsg( (void *)msg, sizeof(msg), 0 );
        PRINT_INFO(" message from VC03 read (%d)\n", length);
        for( i=0; i < (int)(length/sizeof(uint32_t));i++)
        {
            PRINT_INFO( "0x%X\n", msg[i] );
        }
     }
     else
     {
        PRINT_INFO("message not available, does that matter?\n");
     }
  }
   
}
#endif

/****************************************************************************
*
* ReadMessageStatus
*
*   Reads the status register to determine if messages are available
*   and how big the current unread message is.
*
*   The format of the returned status is
*
*   High 16 bits: 1 M N T UF OF FS L[23:15] 
*   Low  16 bits: 0 L[14:0]
*
*   M  = message available
*   N  = new message available
*   T  = message type: 0 for control message, 1 for data message
*   UF = underflow error flag
*   OF = overflow error flag
*   FS = fifo status bit (1 = fifo is at least half empty)
*   L  = message length
*
***************************************************************************/

static unsigned int ReadMessageStatus( uint32_t vcIdx )
{
    int         count;
    uint16_t    regL;
    uint16_t    regH;

    count = 0;
    do
    {
        pifHw_directReadFifoStart( VC03_B0_MSG_STATUS_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );  //upper half word
        regL = pifHw_DirectReadFifo();
    } while (( ++count < 4 ) && (( regL & 0x8000) == 0x8000 ));

    count = 0;
    do
    {
        pifHw_directReadFifoStart( VC03_B0_MSG_STATUS_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );  //upper half word
        regH = pifHw_DirectReadFifo();
    } while (( ++count < 4 ) && (( regH & 0x8000) == 0 ));

    return ((uint32_t)regH << 16 ) | regL;
}

/****************************************************************************
*
* ReadMessageStatus
*
*   Reads 32 bits of message data from the FIFO.
*
***************************************************************************/

static unsigned int ReadMessageData( uint32_t vcIdx )
{
    uint16_t    regL;
    uint16_t    regH;

    pifHw_directReadFifoStart( VC03_B0_MSG_DATA_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );  //upper half word
    regL = pifHw_DirectReadFifo();
    pifHw_directReadFifoStart( VC03_B0_MSG_DATA_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );  //upper half word
    regH = pifHw_DirectReadFifo();

    return ((uint32_t)regH << 16 ) | regL;
}


/****************************************************************************
*
* ReadMessageBlock
*
*   Reads a block of data from the FIFO.
*   Reads 32 bits of message data from the FIFO.
*
***************************************************************************/

static void ReadMessageBlock( uint32_t vcIdx, void *voidMem, size_t numBytes )
{
    uint16_t   *mem = voidMem;
    size_t      numWords = numBytes / sizeof( *mem );

    VC_DEBUG( VchiTrace, "0x%08lx %d bytes %d words", (unsigned long)voidMem, numBytes, numWords );

    while ( numWords >= 8 )
    {
        pifHw_directReadFifoStart( VC03_B0_MSG_DATA_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );
        *mem++ = pifHw_DirectReadFifo();
        
        pifHw_directReadFifoStart( VC03_B0_MSG_DATA_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );
        *mem++ = pifHw_DirectReadFifo();

        pifHw_directReadFifoStart( VC03_B0_MSG_DATA_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );
        *mem++ = pifHw_DirectReadFifo();

        pifHw_directReadFifoStart( VC03_B0_MSG_DATA_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );
        *mem++ = pifHw_DirectReadFifo();

        pifHw_directReadFifoStart( VC03_B0_MSG_DATA_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );
        *mem++ = pifHw_DirectReadFifo();

        pifHw_directReadFifoStart( VC03_B0_MSG_DATA_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );
        *mem++ = pifHw_DirectReadFifo();

        pifHw_directReadFifoStart( VC03_B0_MSG_DATA_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );
        *mem++ = pifHw_DirectReadFifo();

        pifHw_directReadFifoStart( VC03_B0_MSG_DATA_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );
        *mem++ = pifHw_DirectReadFifo();

        numWords -=8;
    }

    while ( numWords > 0 )
    {
        pifHw_directReadFifoStart( VC03_B0_MSG_DATA_REG( vcIdx ), PIF_DEV_SETTING_DIRECT_MODE );
        *mem++ = pifHw_DirectReadFifo();

        numWords--;
    }
}

/****************************************************************************
*
* WriteMessageBlock
*
*   Writes a block of data from the FIFO.
*   Writes 32 bits of message data from the FIFO.
*
***************************************************************************/

static void WriteMessageBlock( uint32_t vcIdx, void *voidMem, size_t numBytes )
{
    uint16_t   *mem = voidMem;
    size_t      numWords = numBytes / sizeof( *mem );

    VC_DEBUG( VchiTrace, "0x%08lx %d bytes %d words", (unsigned long)voidMem, numBytes, numWords );

    while ( numWords >= 8 )
    {
        pifHw_DirectWriteFifo( *mem++, VC03_B0_MSG_DATA_REG(vcIdx), PIF_DEV_SETTING_DIRECT_MODE );
        pifHw_DirectWriteFifo( *mem++, VC03_B0_MSG_DATA_REG(vcIdx), PIF_DEV_SETTING_DIRECT_MODE );
        pifHw_DirectWriteFifo( *mem++, VC03_B0_MSG_DATA_REG(vcIdx), PIF_DEV_SETTING_DIRECT_MODE );
        pifHw_DirectWriteFifo( *mem++, VC03_B0_MSG_DATA_REG(vcIdx), PIF_DEV_SETTING_DIRECT_MODE );
        pifHw_DirectWriteFifo( *mem++, VC03_B0_MSG_DATA_REG(vcIdx), PIF_DEV_SETTING_DIRECT_MODE );
        pifHw_DirectWriteFifo( *mem++, VC03_B0_MSG_DATA_REG(vcIdx), PIF_DEV_SETTING_DIRECT_MODE );
        pifHw_DirectWriteFifo( *mem++, VC03_B0_MSG_DATA_REG(vcIdx), PIF_DEV_SETTING_DIRECT_MODE );
        pifHw_DirectWriteFifo( *mem++, VC03_B0_MSG_DATA_REG(vcIdx), PIF_DEV_SETTING_DIRECT_MODE );

        numWords -=8;
    }

    while ( numWords > 0 )
    {
        pifHw_DirectWriteFifo( *mem++, VC03_B0_MSG_DATA_REG(vcIdx), PIF_DEV_SETTING_DIRECT_MODE );

        numWords--;
    }
}

/****************************************************************************
*
* AlignTransferSizes
*
*   DMA should always be performed on cachline boundaries. So this function
*   takes a pointer and lenght and converts it into 3 segments. The head 
*   portion is not cacheline aligned, and comes before the DMAable portion.
*   The body portion can be DMAed and the tail portion picks up the non-aligned
*   portion at the end.
*
***************************************************************************/

void AlignTransferSizes( void *mem, size_t numBytes, AlignInfo_t *ai )
{
    unsigned long   alignedPtr = ((unsigned long)mem + ( cache_line_size() - 1 )) 
                               & ~( cache_line_size() - 1 );
    size_t          alignedSize;

    ai->headPtr = ai->tailPtr = NULL;
    ai->headSize = ai->tailSize = NULL;
    ai->bodyPtr = mem;
    ai->bodySize = numBytes;

    // In the event that the receive buffer is not cacheline aligned, we'll
    // use direct I/O to transfer the portion upto a cacheline boundary.

    if ( gVcDebugAlignDma )
    {
        if ( (unsigned long)ai->bodyPtr != alignedPtr )
        {
            ai->headPtr = ai->bodyPtr;
            ai->bodyPtr = (uint8_t *)alignedPtr;

            ai->headSize = ai->bodyPtr - ai->headPtr;
            ai->bodySize -= ai->headSize;
        }

        alignedSize = ai->bodySize & ~( cache_line_size() - 1 );

        if ( ai->bodySize != alignedSize )
        {
            ai->tailSize = ai->bodySize - alignedSize;
            ai->bodySize = alignedSize;

            ai->tailPtr = ai->bodyPtr + ai->bodySize;
        }

        VC_DEBUG( VchiTrace, "head: 0x%08lx %d bytes", (unsigned long)ai->headPtr, ai->headSize );
        VC_DEBUG( VchiTrace, "body: 0x%08lx %d bytes", (unsigned long)ai->bodyPtr, ai->bodySize );
        VC_DEBUG( VchiTrace, "tail: 0x%08lx %d bytes", (unsigned long)ai->tailPtr, ai->tailSize );
    }
}

/****************************************************************************
*
* vchost_readmsg
*
*   Reads an entire message from the FIFO.
*
***************************************************************************/

int vchost_readmsg( void *msg, uint32_t maxlen_bytes, uint8_t *msgTypePtr )
{
#if ( DO_DMA == 1 )
    int                 doDma;
#endif
    int                 m_wait_count;
    uint32_t            reg_val;
    int                 msgLength = -1;
    uint8_t             msgType;
    timer_tick_count_t  xfer_start_tick = 0;
    timer_tick_count_t  xfer_dma_tick = 0;
    timer_tick_count_t  setup_time_ticks = 0;
    timer_tick_count_t  xfer_time_ticks = 0;
    const char         *xferTypeStr = "direct";

    VC_DEBUG( VchiTrace, "called, msg:0x%08lx maxlen=%d", (unsigned long)msg, maxlen_bytes );

    //PRINT_INFO("vchost_readmsg, max len = %d\n", maxlen_bytes);

    m_wait_count = 0;
    do
    {
        reg_val = ReadMessageStatus( 0 );
        m_wait_count++;
    
    } while( (m_wait_count < 300) && ((reg_val & VC_HOSTPORT_STATUS_M_FLAG) == 0) );
    
    if( (reg_val & VC_HOSTPORT_STATUS_N_FLAG) == 0 )
    {
        return -1;
    }
    
    // For whatever bizarre reason, the msgType returned from this function
    // is interpreted as 0 = data, 1 = control, which is inverted from the T
    // flag and the flag passed into vchost_writemsg
    
    msgType = (( reg_val & VC_HOSTPORT_STATUS_T_FLAG ) == 0 );
    
    if ( msgTypePtr != NULL )
    {
        *msgTypePtr = msgType;
    }
    
    msgLength = VC_HOSTPORT_STATUS_GET_LEN( reg_val );

    if( ((msgLength & 0xf) != 0) || (msgLength > (int)maxlen_bytes) )
    {
        printk( KERN_ERR "%s: msgLength:%d is not a multiple of 16 or exceeds %d\n",
                __FUNCTION__, msgLength, maxlen_bytes ); 
        return -1;
    }
    PRINT_INFO( "new msg (%d) type %d arrived, read count = %d\n", msgLength, 
                (reg_val & VC_HOSTPORT_STATUS_T_FLAG), m_wait_count );

    if ( gVcDebugRwPerf )
    {
        xfer_start_tick =  timer_get_tick_count(); 
    }

#if (DO_DMA == 1)

    doDma = !gVcDmaDisabled && ( msgLength >= gVcDmaMinSize );

    if ( doDma )
    {
        AlignInfo_t ai;

        AlignTransferSizes( msg, msgLength, &ai );

        // Try to map the memory. If that fails, then we'll
        // fallback and use the programmed I/O mode. One of the reasons that
        // this could fail is that we're using a type of memory that the dma_map_mem
        // function doesn't support

        if (( ai.bodySize > 0 ) && ( dma_map_mem( &gDmaMemMap, ai.bodyPtr, ai.bodySize, DMA_FROM_DEVICE ) == 0 ))
        {
            int                 rc;
            DMA_Handle_t        dmaHndl;
            pifHw_XFER_INFO_t   xferInfo;

            // We've mapped the memory. Now reserve a dma channel

            if (( dmaHndl = dma_request_channel( DMA_DEVICE_PIF_DEV_TO_MEM )) < 0 )
            {
                printk( KERN_ERR "%s: dma_request_channel failed: %d\n", __FUNCTION__, dmaHndl );
                dma_unmap_mem( &gDmaMemMap, 0 );
                return dmaHndl;
            }
            if ( ai.headSize > 0 )
            {
                // Read in the head portion using direct I/O
    
                ReadMessageBlock( 0, ai.headPtr, ai.headSize );
            }
            memset( &xferInfo, 0, sizeof( xferInfo ));

            xferInfo.lengths = ai.bodySize / sizeof( uint16_t );    // Number of 16-bit words
            xferInfo.xferDir = PIFHW_XFER_DIR_READ;
            xferInfo.settingIdx = PIF_DEV_SETTING_B0_HS;
            xferInfo.addr = VC03_B0_MSG_DATA_REG( 0 );
            xferInfo.intOption = 0;
            xferInfo.bDma = 1;
            xferInfo.bPacked = 1;
            xferInfo.highspeed = 0;

            if (( rc = pifHw_configTransfer( &xferInfo, NULL )) != 0 )
            {
                printk( KERN_ERR "%s: Call to pifHw_configTransfer failed: %d\n", __FUNCTION__, rc );
                return rc;
            }

            INIT_COMPLETION( gDmaDone );   // Mark as incomplete

            if ( gVcDebugRwPerf )
            {
                xferTypeStr = "dma";
                xfer_dma_tick = timer_get_tick_count(); 
            }

            if ((rc = dma_transfer_from_device( dmaHndl,
                                                MM_IO_VIRT_TO_PHYS( PIFHW_REG32_SMID ),
                                                gDmaMemMap.physAddr,
                                                ai.bodySize )) != 0 )
            {
                printk( KERN_ERR "%s DMA failed: %d\n", __FUNCTION__, rc );

                dma_free_channel( dmaHndl );
                dma_unmap_mem( &gDmaMemMap, 1 );

                return 0;
            }

            wait_for_completion( &gDmaDone );

            if ( ai.tailSize > 0 )
            {
                // Read in the tail portion using direct I/O

                ReadMessageBlock( 0, ai.tailPtr, ai.tailSize );
            }

            dma_free_channel( dmaHndl );
            dma_unmap_mem( &gDmaMemMap, 1 );
        }
        else
        {
            // dma_map_mem doesn't support the type of memory that we've tried to allocate.
            // In that case, we'll just back down to using the slow method.

            doDma = 0;
        }
    }

    if ( !doDma )
#endif
    {
        if ( gVcDebugRwPerf )
        {
            // This will usually be the same as xfer_start_time when not doing DMA.

            xfer_dma_tick = timer_get_tick_count(); 
        }
        if ( gVcDebugPifReadMsgBlock )
        {
            ReadMessageBlock( 0, msg, msgLength );
        }
        else
        {
            int         wordsRemaining = msgLength / sizeof( uint32_t );
            uint32_t   *msg32 = msg;

            while ( wordsRemaining > 0 )
    
            {
                if ( wordsRemaining >= 16 )
                {
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
                    *msg32++ = ReadMessageData( 0 );
    
                    wordsRemaining -= 16;
                }
                else
                {
                    *msg32++ = ReadMessageData( 0 );
    
                    wordsRemaining--;
                }
            }
        }
    }

    if ( gVcDebugRwPerf )
    {
        uint32_t  setup_time_usecs;
        uint32_t  xfer_time_usecs;

        xfer_time_ticks = ( timer_get_tick_count() - xfer_dma_tick );
        setup_time_ticks = xfer_dma_tick - xfer_start_tick;

        xfer_time_usecs  = xfer_time_ticks / ( timer_get_tick_rate() / 1000000 );
        setup_time_usecs = setup_time_ticks / ( timer_get_tick_rate() / 1000000 );

        if ( xfer_time_usecs == 0 )
        {
            xfer_time_usecs = 1;    // Prevent divide by zero
        }

        printk( KERN_INFO "%-16s: Setup: %3d.%03d msec Xfer: %3d.%03d msec (%5d kbytes/sec) Size: %5d bytes (%s)\n",
                __FUNCTION__,
                setup_time_usecs / 1000u, setup_time_usecs % 1000u,
                xfer_time_usecs / 1000u, xfer_time_usecs % 1000u,
                ((( msgLength * 1000 ) / xfer_time_usecs ) * 1000 ) / 1024,
                msgLength, xferTypeStr );
    }

    VC_DEBUG( VchiTrace, "done, actual len=%d", msgLength );
    if ( gVcDebugVchiTraceData )
    {
        printk( KERN_INFO "%s len=%d msgType:%s\n", __FUNCTION__, msgLength, msgType ? "Ctrl" : "Data" );
        vc_dump_mem( "R:", 0, msg, msgLength > gVcDebugVchiTraceDataSize ? gVcDebugVchiTraceDataSize : msgLength );
    }
    return msgLength;
}

/****************************************************************************
*
* vchost_msgavail
*
*   Determines if there are any messages available from the videocore.
*
*   Returns 1 if a data message is available
*   Returns 0 if a control message is available
*   Returns -1 if no messages are available
*
***************************************************************************/

int vchost_msgavail(void)
{
    uint32_t reg_val;
    
    // Do a dummy read. This ensures that the second read will get updated results.
    
    reg_val = ReadMessageStatus( 0 );
    reg_val = ReadMessageStatus( 0 );
    
    if( (reg_val & VC_HOSTPORT_STATUS_M_FLAG) == 0 )
    {
        // No more messages are available. This would be a good time to 
        // re-enable the HAT interrupt

        PIFHW_SET_REG32_BIT( PIFHW_REG32_SMIEI, PIFHW_REG32_SMIEI_EIEN1 );   // Enable HAT1 interrupt

        return -1;
    }
    return (reg_val & VC_HOSTPORT_STATUS_T_FLAG) ? 0 : 1;
}

/****************************************************************************
*
* vchost_writeparam
*
*   Writes a word of data to the message parameter register.
*
***************************************************************************/

void vchost_writeparam( uint16_t param )
{
    int count;

    pifHw_DirectWriteFifo( param, VC03_B0_MSG_STATUS_REG(0), PIF_DEV_SETTING_DIRECT_MODE );

    // The DONE bit should be set almost immediately. There are only 16 entries 
    // in the FIFO, and they're read out at 75 MHz

    count = 0;
    while ( PIFHW_TEST_REG32_BIT( PIFHW_REG32_SMIDCS, PIFHW_REG32_SMIDCS_DONE) == 0 )
    {
        if ( ++count >= 100 )
        {
            printk( KERN_ERR "%s: timeout waiting for DONE bit\n", __FUNCTION__ );
            break;
        }
    }
}

/****************************************************************************
*
* vchost_writedata
*
*   Writes a block of data to the videcore.
*
***************************************************************************/

int vchost_writedata( uint8_t *ptr, int numBytes )
{
#if ( DO_DMA == 1 )
    int                 doDma;
#endif
    timer_tick_count_t  xfer_start_tick = 0;
    timer_tick_count_t  xfer_dma_tick = 0;
    timer_tick_count_t  setup_time_ticks = 0;
    timer_tick_count_t  xfer_time_ticks = 0;
    const char         *xferTypeStr = "direct";

    if ( numBytes == 0 )
    {
        // Nothing to do

        return 0;
    }

    if (( numBytes & 0x0F ) != 0 )
    {
        printk( KERN_ERR "%s: numBytes:%d isn't a mulitple of 16\n", __FUNCTION__, numBytes );
        return -EINVAL;
    }

    if ( gVcDebugRwPerf )
    {
        xfer_start_tick = timer_get_tick_count(); 
    }

#if ( DO_DMA == 1 )

    doDma = !gVcDmaDisabled && ( numBytes >= gVcDmaMinSize );

    if ( doDma )
    {
        AlignInfo_t ai;

        AlignTransferSizes( ptr, numBytes, &ai );

        // Try to map the memory. If that fails, then we'll
        // fallback and use the programmed I/O mode. One of the reasons that
        // this could fail is that we're using a type of memory that the dma_map_mem
        // function doesn't support
    
        if (( ai.bodySize > 0 ) && ( dma_map_mem( &gDmaMemMap, ai.bodyPtr, ai.bodySize, DMA_TO_DEVICE ) == 0 ))
        {
            int             rc;
            DMA_Handle_t    dmaHndl;
            pifHw_XFER_INFO_t   xferInfo;

            // We've mapped the memory. Now reserve a dma channel
    
            if (( dmaHndl = dma_request_channel( DMA_DEVICE_PIF_MEM_TO_DEV )) < 0 )
            {
                printk( KERN_ERR "%s: dma_request_channel failed: %d\n", __FUNCTION__, dmaHndl );
                dma_unmap_mem( &gDmaMemMap, 0 );
                return dmaHndl;
            }

            if ( ai.headSize > 0 )
            {
                // Write out the head portion using direct I/O

                WriteMessageBlock( 0, ai.headPtr, ai.headSize );
            }

            memset( &xferInfo, 0, sizeof( xferInfo ));

            xferInfo.lengths = ai.bodySize / sizeof( uint16_t );    // Number of 16-bit words
            xferInfo.xferDir = PIFHW_XFER_DIR_WRITE;
            xferInfo.settingIdx = PIF_DEV_SETTING_B0_HS;
            xferInfo.addr = VC03_B0_MSG_DATA_REG( 0 );
            xferInfo.intOption = 0;
            xferInfo.bDma = 1;
            xferInfo.bPacked = 1;
            xferInfo.highspeed = 1;

            if (( rc = pifHw_configTransfer( &xferInfo, NULL )) != 0 )
            {
                printk( KERN_ERR "%s: Call to pifHw_configTransfer failed: %d\n", __FUNCTION__, rc );

                dma_free_channel( dmaHndl );
                dma_unmap_mem( &gDmaMemMap, 0 );
                return rc;
            }

            INIT_COMPLETION( gDmaDone );   // Mark as incomplete

            if ( gVcDebugRwPerf )
            {
                xferTypeStr = "dma";
                xfer_dma_tick = timer_get_tick_count(); 
            }
            if ((rc = dma_transfer_to_device( dmaHndl,
                                              gDmaMemMap.physAddr,
                                              MM_IO_VIRT_TO_PHYS( PIFHW_REG32_SMID ),
                                              ai.bodySize )) != 0 )
            {
                printk( KERN_ERR "%s DMA failed: %d\n", __FUNCTION__, rc );

                dma_free_channel( dmaHndl );
                dma_unmap_mem( &gDmaMemMap, 0 );

                return 0;
            }
            wait_for_completion( &gDmaDone );
    
            if ( ai.tailSize > 0 )
            {
                // Write out the tail portion using direct I/O

                WriteMessageBlock( 0, ai.tailPtr, ai.tailSize );
            }

            dma_free_channel( dmaHndl );
            dma_unmap_mem( &gDmaMemMap, 0 );
        }
        else
        {
            // dma_map_mem doesn't support the type of memory that we've tried to allocate.
            // In that case, we'll just back down to using the slow method.

            doDma = 0;
        }
    }
    if ( !doDma )
#endif
    {
        if ( gVcDebugRwPerf )
        {
            // This will usually be the same as xfer_start_time when not doing DMA.

            xfer_dma_tick = timer_get_tick_count(); 
        }
        if ( gVcDebugPifReadMsgBlock )
        {
            WriteMessageBlock( 0, ptr, numBytes );
        }
        else
        {
            uint16_t   *data = (uint16_t *)ptr;
            size_t  wordsRemaining = numBytes / sizeof( *data );

            while ( wordsRemaining > 0 )
            {
                int count;
    
                if ( wordsRemaining >= 16 )
                {
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
    
                    wordsRemaining -= 16;
                }
                else
                {
                    pifHw_DirectWriteFifo( *data++, VC03_B0_MSG_DATA_REG(0), PIF_DEV_SETTING_DIRECT_MODE );
                    wordsRemaining--;
                }
        
                // The DONE bit should be set almost immediately. There are only 16 entries 
                // in the FIFO, and they're read out at 75 MHz
        
                count = 0;
                while ( PIFHW_TEST_REG32_BIT( PIFHW_REG32_SMIDCS, PIFHW_REG32_SMIDCS_DONE) == 0 )
                {
                    if ( ++count >= 100 )
                    {
                        printk( KERN_ERR "%s: timeout waiting for DONE bit\n", __FUNCTION__ );
                        break;
                    }
                }
            }
        }
    }

    if ( gVcDebugRwPerf )
    {
        uint32_t  setup_time_usecs;
        uint32_t  xfer_time_usecs;

        xfer_time_ticks = ( timer_get_tick_count() - xfer_dma_tick );
        setup_time_ticks = xfer_dma_tick - xfer_start_tick;

        xfer_time_usecs  = xfer_time_ticks / ( timer_get_tick_rate() / 1000000 );
        setup_time_usecs = setup_time_ticks / ( timer_get_tick_rate() / 1000000 );

        if ( xfer_time_usecs == 0 )
        {
            xfer_time_usecs = 1;    // Prevent divide by zero
        }

        printk( KERN_INFO "%-16s: Setup: %3d.%03d msec Xfer: %3d.%03d msec (%5d kbytes/sec) Size: %5d bytes (%s)\n",
                __FUNCTION__,
                setup_time_usecs / 1000u, setup_time_usecs % 1000u,
                xfer_time_usecs / 1000u, xfer_time_usecs % 1000u,
                ((( numBytes * 1000 ) / xfer_time_usecs ) * 1000 ) / 1024,
                numBytes, xferTypeStr );
    }

    return numBytes;
}

/****************************************************************************
*
* vchost_writemsg
*
*   Writes a message to the videocore.
*
***************************************************************************/

int vchost_writemsg( void *msg, uint32_t numBytes, int8_t msgType )
{
    int rc;

    VC_DEBUG( VchiTrace, "called numBytes=%d msgType:%s", numBytes, ( msgType & 1 ) ? "Data" : "Ctrl" );

    vchost_writeparam( msgType );

    if ( gVcDebugVchiTraceData )
    {
        printk( KERN_INFO "%s len=%d msgType:%s %s\n", __FUNCTION__, numBytes,
                ( msgType & VC_HOSTPORT_PARAM_MSG_TYPE_DATA ) ? "Data" : "Ctrl",
                ( msgType & VC_HOSTPORT_PARAM_TERMINATE_DMA ) ? "Terminate DMA" : "" );

        vc_dump_mem( "W:", 0, msg, numBytes > gVcDebugVchiTraceDataSize ? gVcDebugVchiTraceDataSize : numBytes );
    }

    rc = vchost_writedata( msg, numBytes );

    VC_DEBUG( VchiTrace, "done" );
    return rc;
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
    char            lineBuf[ 100 ];
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

        printk( KERN_INFO "%s %08x: %s\n", label, addr, lineBuf );

        addr += 16;
        mem += 16;
        if ( numBytes > 16 )
        {
            numBytes -= 16;
        }
        else
        {
            numBytes = 0;
        }
    }

} // vc_dump_mem

/****************************************************************************
*
*  SetInterfaceTiming
*
*   Initializes the PIF timing registers used to talk to the videocore
*
***************************************************************************/

static void SetInterfaceTiming( int setupTime, int holdTime, int strobeTime, int paceTime, int mode )
{
    pifHw_DEVICE_SETTING_PAR_t  param;

    memset( &param, 0, sizeof( param ));

    param.dataWidth     = PIFHW_DWIDTH_16;
    param.mode          = PIFHW_MODE_80;
    param.swapMode      = PIFHW_DSWAP_NOSWAP;

    param.setupTime     = setupTime;
    param.holdTime      = holdTime;
    param.strobeTime    = strobeTime;
    param.paceTime      = paceTime;

    if ( mode == PIF_DEV_SETTING_B0_HS )
    {
        param.strobeTime++;
    }

    pifHw_configDeviceSetting( mode, PIFHW_XFER_DIR_READ, &param );

    param.strobeTime = strobeTime;

    pifHw_configDeviceSetting( mode, PIFHW_XFER_DIR_WRITE, &param );

    if ( mode == PIF_DEV_SETTING_B0_HS )
    {
        PIFHW_SET_REG32_BIT(PIFHW_REG32_SMIEI ,PIFHW_REG32_SMIEI_HIGHSPEED);
    }
    else
    {
        PIFHW_CLEAR_REG32_BIT(PIFHW_REG32_SMIEI ,PIFHW_REG32_SMIEI_HIGHSPEED);
    }
}

/****************************************************************************
*
*  vchost_set_interface_for_boot
*
*   During booting, the videocore is only driving the interface lines with 
*   4mA strength so we need to slow down.
*
***************************************************************************/

void vchost_set_interface_for_boot( void )
{
    SetInterfaceTiming( 2, 2, 10, 2, PIF_DEV_SETTING_DIRECT_MODE );
}

/****************************************************************************
*
*  vchost_set_interface_for_boot
*
*   Now that the firmware is running, it reconfigures the drive strength, 
*   and we can speed things back up again.
*
***************************************************************************/

void vchost_set_interface_for_run( void )
{
    SetInterfaceTiming( 1, 1, 1, 1, PIF_DEV_SETTING_DIRECT_MODE );
}


int vchost_portinit( void *portCfg )
{
    int                 rc;
    pifHw_INIT_DATA_t   pifInitData;

    (void)portCfg;

    if (( rc = gpio_request( HW_VC03_RUN_GPIO, "vc-run" )) != 0 )
    {
        printk( KERN_ERR "Unable to request videocore RUN pin: %d (Error %d)\n", HW_VC03_RUN_GPIO, rc );
        return rc;
    }
    gpio_direction_output( HW_VC03_RUN_GPIO, 0 );

    // Currently, pifHw_Init assumes that the gpio pins have already been reserved.

    memset( &pifInitData, 0, sizeof( pifInitData ));
    pifInitData.padCurrent = 12;
    pifInitData.gpioRun1 = HW_VC03_RUN_GPIO;
    pifHw_Init( &pifInitData );

    PIFHW_CLEAR_REG32_BIT(PIFHW_REG32_SMIDCS, PIFHW_REG32_SMIDCS_ENABLE); 
    PIFHW_CLEAR_REG32_BIT(PIFHW_REG32_SMICS, PIFHW_REG32_SMICS_ENABLE);  //to prevent setup err 

    SetInterfaceTiming( 1, 1, 1, 1, PIF_DEV_SETTING_DIRECT_MODE );
    SetInterfaceTiming( 0, 0, 0, 0, PIF_DEV_SETTING_B0_HS );

#if ( DO_DMA == 1 )
    init_completion( &gDmaDone );

    dma_init_mem_map( &gDmaMemMap );
    dma_set_device_handler( DMA_DEVICE_PIF_MEM_TO_DEV, vchost_dma_handler, NULL );
    dma_set_device_handler( DMA_DEVICE_PIF_DEV_TO_MEM, vchost_dma_handler, NULL );
#endif
    return 0;
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

int vc_host_read_byteswapped (void *host_addr, uint32_t vc_addr, int nbytes,
                              int channel)
{
    printk( KERN_ERR "***** %s ***** not implemented yet\n", __FUNCTION__ );
    return 0; 
}

int vc_host_read_consecutive (void *host_addr, uint32_t vc_addr, int nbytes,
                              int channel)
{
    printk( KERN_ERR "***** %s ***** not implemented yet\n", __FUNCTION__ );
    return 0; 
}
/******************************************************************************
NAME
   vc_host_read32

SYNOPSIS
   int vc_host_read32( uint32_t vc_addr, int channel)

FUNCTION
   Read 4 bytes consecutive bytes from VideoCore host port channel channel.

RETURNS
   int
*****************************************************************************/

uint32_t vc_host_read32(uint32_t vc_addr, int channel)
{
    printk( KERN_ERR "***** %s ***** not implemented yet\n", __FUNCTION__ );
    return 0; 
}

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
    printk( KERN_ERR "***** %s ***** not implemented yet\n", __FUNCTION__ );
    return 0; 
}

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
   (void)channel;
    return 0;
}

/*
 *
 */
uint16_t vc_host_read_reg( int reg, int channel )
{
   (void) reg;
   (void) channel;
  return 0;
}


/*
*
*/
void vc_host_write_reg( int reg, int channel, uint16_t val )
{
   (void) reg;
   (void) channel;
   (void) val;
}

