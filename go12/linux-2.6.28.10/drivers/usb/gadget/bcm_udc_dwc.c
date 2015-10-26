/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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


/****************************************************************************/
/**
*  @file    bcm_dwc_udc.c
*
*  @brief   Broadcom Linux driver for DWC USB 2.0 Device Controller (UDC)
*
*  This driver implements the Linux Gadget driver API as defined in usb_gadget.h
*
*  @note
*
*  This driver was written with the intent of being able to support any
*  variations on how this block is integrated into different Broadcom chips.
*
*  There is a requirement on how the DWC UDC is configured. In particular, this
*  driver requires that the following options be defined and enabled in the
*  UDC core.
*
*       UDC20AHB_CNAK_CLR_ENH_CC
*       UDC20AHB_STALL_SET_ENH_CC
*       UDC20AHB_SNAK_ENH_CC
*
*  Some other UDC attributes can be supported by setting compile time options
*  or with some minor modifications to the source code. Ideally these would
*  be run-time info that is provided by the device instance to the driver.
*  These attributes include the following.
*
*       BCM_UDC_EP_CNT
*       BCM_UDC_EP_MAX_PKT_SIZE
*       BCM_UDC_OUT_RX_FIFO_MEM_SIZE
*       BCM_UDC_IN_TX_FIFO_MEM_SIZE
*       BCM_UDC_IRQ
*       Type of each endpoint: Control, IN, OUT, or Bidirectional
*/
/****************************************************************************/

#define BCM_UDC_EP_CNT                  8
#define BCM_UDC_EP_MAX_PKT_SIZE         1024
#define BCM_UDC_OUT_RX_FIFO_MEM_SIZE    1024
#define BCM_UDC_IN_TX_FIFO_MEM_SIZE     4096

#define BCM_UDC_IRQ                     IRQ_USBHD2

/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/types.h>
#include <asm/arch/csp/chipcHw_inline.h>
#include <asm/arch/csp/chipcHw_def.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
    #include <linux/usb_ch9.h>
#else
    #include <linux/usb/ch9.h>
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
    #include <linux/usb_gadget.h>
#else
    #include <linux/usb/gadget.h>
#endif

/*
 * Name definitions for use with driver and devices. Device names will have
 * a digit appended. Note the correlation to the BCM_USBEHCI_HCD_MAX value
 * (max value is a single digit, hence the sizeof() + 1). Since this name
 * has to be the same in the device and the driver, really should have it
 * defined in some common include file used by both device and driver.
 */
#define BCM_UDC_NAME                "bcm-udc"

#ifndef BCM_UDC_KMARKER
#define BCM_UDC_KMARKER             BCM_UDC_NAME": "
#endif

#define BCM_KERROR(fmt, ...)        printk( KERN_ERR     BCM_UDC_KMARKER "ERROR: %s(): " fmt, __func__ , ## __VA_ARGS__ )
#define BCM_KINFO(fmt...)           printk( KERN_INFO    BCM_UDC_KMARKER fmt )
#define BCM_KPANIC(fmt, ...)        panic(               BCM_UDC_KMARKER "%s(): " fmt, __func__ , ## __VA_ARGS__ )
#define BCM_KWARN(fmt, ...)         printk( KERN_WARNING BCM_UDC_KMARKER "%s(): " fmt, __func__ , ## __VA_ARGS__  )

#ifdef DEBUG

    unsigned bcm_trace = 0x1;

    #define BCM_KTRACE(fmt, ...)        if (bcm_trace & 1) printk( BCM_UDC_KMARKER "%s(): " fmt, __func__ , ## __VA_ARGS__  )
    #define BCM_KTRACE2(fmt, ...)       if (bcm_trace & 2) printk( BCM_UDC_KMARKER "%s(): " fmt, __func__ , ## __VA_ARGS__  )
    #define BCM_KTRACE3(fmt, ...)       if (bcm_trace & 4) printk( BCM_UDC_KMARKER "%s(): " fmt, __func__ , ## __VA_ARGS__  )
    #define BCM_KTRACE_REQ(fmt, ...)    if (bcm_trace & 8) printk( BCM_UDC_KMARKER "%s(): " fmt, __func__ , ## __VA_ARGS__  )

#else

    #define BCM_KTRACE(fmt, ...)
    #define BCM_KTRACE2(fmt, ...)
    #define BCM_KTRACE3(fmt, ...)
    #define BCM_KTRACE_REQ(fmt, ...)

#endif

#include <asm/arch/irqs.h>
#include <asm/arch/csp/chipcHw_inline.h>

// Uncomment the following REG_DEBUG and REG_DEBUG_PRINT if it is desired to have
// the register modification routines defined in usbdevHw_def.h output debug info.
//
// #define usbDevHw_DEBUG_REG
// #define usbDevHw_DEBUG_REG_PRINT         printk
//
// Define and set DMA_BURST_LEN_32BIT if it is desired to use a DMA burst length
// other than the default which is 16 (INCR16). Set to 0 to disable burst mode
// and use INCR.
//
// #define usbDevHw_DMA_BURST_LEN_32BIT     0

#include <csp/usbDevHw.h>
#include <csp/usbPhyHw.h>

// Would be nice if DMA_ADDR_INVALID or similar was defined in dma-mapping.h
#define DMA_ADDR_INVALID                (~(dma_addr_t)0)

// Would be nice if ENOERROR or similar was defined in errno.h
#define ENOERROR                        0

// Would be nice if USB_DIR_MASK was defined in usb/ch9.h
#define USB_DIR_MASK                    USB_ENDPOINT_DIR_MASK
#define USB_DIR_UNKNOWN                 ~USB_DIR_MASK
#define USB_CONTROL_MAX_PACKET_SIZE     64

/* ---- External Variable Declarations ----------------------------------- */
/* ---- External Function Prototypes ------------------------------------- */
/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

#define BCM_UDC_MODULE_DESCRIPTION      "Broadcom USB Device Controller (UDC) driver"
#define BCM_UDC_MODULE_VERSION          "1.0.0"

#include "bcm_udc_dwc.h"

#define BCM_DRIVER_DESC                 BCM_UDC_MODULE_DESCRIPTION
#define BCM_DRIVER_VERSION              BCM_UDC_MODULE_VERSION"

/*
 * Definitions for the number of USB Device Controllers (UDC's) supported. Usually there's
 * just 1. Note that numbering is 0 .. BCM_UDC_MAX
 */
#define BCM_UDC_CNT_MAX                 9

#define SETUP_STATUS_OK                 0
#define SETUP_STATUS_ERROR              -1

/* ---- Private Function Prototypes -------------------------------------- */

/// @todo Rename dirn to dir to match USB_DIR_xxx def'ns
#define DIRN_STR(dirn)                  ((dirn) == USB_DIR_IN ? "IN" : "OUT")

static void CtrlEpSetupInit( BCM_UDC_EP_t *udcEpP, int status );
static void CtrlEpSetupRx( BCM_UDC_EP_t *udcEpP, struct usb_ctrlrequest *setup );


static void DmaEpInit( BCM_UDC_EP_t *udcEpP );

#ifdef DEBUG
static void DmaDump( BCM_UDC_t *udcP );
#endif
static void DmaDumpDesc( char *label, BCM_UDC_DMA_DESC_t *virt, BCM_UDC_DMA_DESC_t *phys );
static void DmaDumpEp( BCM_UDC_EP_t *udcEpP );

static void DmaDataInit( BCM_UDC_EP_t *udcEpP );
static void DmaDataFinis( BCM_UDC_EP_t *udcEpP );
static void DmaDataAddReady( BCM_UDC_EP_t *udcEpP );
static void DmaDataRemoveDone( BCM_UDC_EP_t *udcEpP );

static inline BCM_UDC_DMA_DESC_t *DmaDescChainAlloc( BCM_UDC_EP_t *udcEpP );
static inline int DmaDescChainEmpty( BCM_UDC_EP_t *udcEpP );
static inline void DmaDescChainFree( BCM_UDC_EP_t *udcEpP );
static inline int DmaDescChainFull( BCM_UDC_EP_t *udcEpP );
static inline BCM_UDC_DMA_DESC_t *DmaDescChainHead( BCM_UDC_EP_t *udcEpP );
static inline void DmaDescChainReset( BCM_UDC_EP_t *udcEpP );


static int GadgetDevAdd( BCM_UDC_t *udcP );
static void GadgetDevInit( struct platform_device *devP, BCM_UDC_t *udcP );
static void GadgetDevRelease( struct device *dev );
static int GadgetDevRemove( BCM_UDC_t *udcP );


static int GadgetEpEnable(struct usb_ep *ep, const struct usb_endpoint_descriptor *desc);
static int GadgetEpDisable(struct usb_ep *ep);
static struct usb_request *GadgetEpRequestAlloc(struct usb_ep *ep, unsigned gfp_flags);
static void GadgetEpRequestFree(struct usb_ep *ep, struct usb_request *req);
static int GadgetEpRequestQueue(struct usb_ep *ep, struct usb_request *req, unsigned gfp_flags);
static int GadgetEpRequestDequeue(struct usb_ep *ep, struct usb_request *req);
static int GadgetEpSetHalt(struct usb_ep *ep, int value);
static int GadgetEpFifoStatus(struct usb_ep *ep);
static void GadgetEpFifoFlush(struct usb_ep *ep);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
static void *GadgetEpBufferAlloc(struct usb_ep *ep, unsigned bytes, dma_addr_t *dma, unsigned gfp_flags);
static void GadgetEpBufferFree(struct usb_ep *_ep, void *buf, dma_addr_t dma, unsigned bytes);
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t  IrqUdc( int irq, void *context, struct pt_regs *pt_regs );
#else
static irqreturn_t  IrqUdc( int irq, void *context );
#endif

static void IrqDev( BCM_UDC_t *udcP, uint32_t irq );
static void IrqDevCfgSet( BCM_UDC_t *udcP );
static void IrqDevIntfSet( BCM_UDC_t *udcP );
static void IrqDevSpeedEnum( BCM_UDC_t *udcP );

static void IrqEp( BCM_UDC_t *udcP, uint32_t irqIn, uint32_t irqOut );
static void IrqEpInStatusCheck( BCM_UDC_EP_t *udcEpP );
static void IrqEpOutStatusCheck( BCM_UDC_EP_t *udcEpP );
static void IrqEpOutSetup( BCM_UDC_EP_t *udcEpP );


static int PlatformDriverProbe( struct platform_device *devP );
static int PlatformDriverRemove( struct platform_device *devP );
static int PlatformDmaAlloc( struct platform_device *platformDevP, BCM_UDC_t *udcP );
static void PlatformDmaFree( struct platform_device *platformDevP, BCM_UDC_t *udcP );


static void ProcFileCreate(void);
static void ProcFileRemove(void);


static void ReqQueueFlush( BCM_UDC_EP_t *udcEpP, int status );
static void ReqXferError( BCM_UDC_EP_t *udcEpP, int status );
static void ReqXferDone( BCM_UDC_EP_t *udcEpP, BCM_UDC_EP_REQ_t *udcEpReqP, int status );
static void ReqXferProcess( BCM_UDC_EP_t *udcEpP );
static void ReqXferStart( BCM_UDC_EP_t *udcEpP );


static void UdcOpsFinis( BCM_UDC_t *udcP );
static void UdcOpsInit( BCM_UDC_t *udcP );
static void UdcOpsShutdown( BCM_UDC_t *udcP );
static void UdcOpsStartup( BCM_UDC_t *udcP );

static void UdcEpInit( BCM_UDC_t *udcP, unsigned num, const char *name, unsigned dirn );
static int UdcEpCfg( BCM_UDC_EP_t *udcEpP, unsigned type, unsigned maxPktSize );


static void UdcFifoRamInit( BCM_UDC_t *udcP );
static int UdcFifoRamAlloc( BCM_UDC_EP_t *udcEpP, unsigned maxPktSize );
static void UdcFifoRamFree( BCM_UDC_EP_t *udcEpP );

/* ---- Private Variables ------------------------------------------------ */

/// @todo Can this not be a global?? Part of platform device??

BCM_UDC_t *bcmUdcP = NULL;

/*
 * Generic platform device driver definition.
 */
static struct platform_driver bcm_udc_PlatformDriver =
{
    .probe      = PlatformDriverProbe,
    .remove     = PlatformDriverRemove,

    #ifdef CONFIG_PM
    /// @todo .suspend =    PlatformDriverSuspend,
    /// @todo .resume =     PlatformDriverResume,
    #endif

    .driver =
    {
        .name   = BCM_UDC_NAME,
        .owner  = THIS_MODULE,
    },
};

static struct usb_gadget_ops bcm_udc_gadgetDevOps =
{
    .get_frame          = NULL,
    .wakeup             = NULL,
    .set_selfpowered    = NULL,
    .vbus_session       = NULL,
    .vbus_draw          = NULL,
    .pullup             = NULL,
    .ioctl              = NULL,
};

static struct usb_ep_ops bcm_udc_gadgetEpOps =
{
    .enable         = GadgetEpEnable,
    .disable        = GadgetEpDisable,

    #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
    /// @todo Are these needed? Seem to be BCM specials.
    .alloc_buffer   = GadgetEpBufferAlloc,
    .free_buffer    = GadgetEpBufferFree,
    #endif

    .alloc_request  = GadgetEpRequestAlloc,
    .free_request   = GadgetEpRequestFree,
    .queue          = GadgetEpRequestQueue,
    .dequeue        = GadgetEpRequestDequeue,

    #ifdef CONFIG_USB_GADGET_USE_DMA_MAP
    /// @todo Something does not seem right with how this config item is being handled in other
    /// device controller drivers. This is all that had to be done for things to work here.
    /// However, it is required that this config item be defined, otherwise there may be DMA
    /// alignment issues. This config item has no real effect within this driver, but it
    /// does within some gadget drivers like the ethernet gadget.
    ///
    .queue_dma      = GadgetEpRequestQueue,
    #endif

    .set_halt       = GadgetEpSetHalt,

    .fifo_status    = GadgetEpFifoStatus,
    .fifo_flush     = GadgetEpFifoFlush,
};


/*-------------------------------------------------------------------------*/

/* ==== Public Functions ================================================= */

//***************************************************************************
// Module level definitions use to load / unload the UDC driver
//***************************************************************************

static int __init bcm_udc_ModuleInit( void );
static void __exit bcm_udc_ModuleExit( void );

MODULE_DESCRIPTION( BCM_UDC_MODULE_DESCRIPTION );
MODULE_LICENSE( "GPL" );
MODULE_VERSION( BCM_UDC_MODULE_VERSION );

module_init (bcm_udc_ModuleInit);
module_exit (bcm_udc_ModuleExit);

static int __init bcm_udc_ModuleInit( void )
{
    BCM_KINFO( BCM_DRIVER_DESC " for Broadcom SOC\n" );

// Not sure if this is the right place... equivalent stuff done in bcmring_UsbEhci.c, but this is
// a host/dev hybrid module, and I don't see how the gadget.ko insmod gets this inserted?
// By the way, shouldn't this be in a CSP init/exit function?
// FIXME add the exit stuff
    chipcHw_setUsbDevice();
    chipcHw_setClockEnable( chipcHw_CLOCK_USB );
    chipcHw_powerUpUsbPhy();
    chipcHw_busInterfaceClockEnable( chipcHw_REG_BUS_CLOCK_USBD );
    return( platform_driver_register( &bcm_udc_PlatformDriver ) );
}

static void __exit bcm_udc_ModuleExit( void )
{
    platform_driver_unregister( &bcm_udc_PlatformDriver );
}

//***************************************************************************
// APIs used by a Gadget driver to attach / detach from the UDC driver.
//***************************************************************************

int usb_gadget_register_driver( struct usb_gadget_driver *gadget_driver )
{
    unsigned long flags;
    int err;


    BCM_KTRACE2( "enter\n" );

    if ( bcmUdcP == NULL )
    {
        BCM_KERROR( "UDC driver not initialized\n" );
        return( -ENODEV );
    }

    if ( !gadget_driver || !gadget_driver->bind || !gadget_driver->unbind || !gadget_driver->setup || (gadget_driver->speed < USB_SPEED_FULL) )
    {
        BCM_KERROR( "invalid gadget driver\n" );
        return( -EINVAL );
    }

    spin_lock_irqsave( &bcmUdcP->lock, flags );

    /// @todo somehow get rid of bcmUdcP global variable?
    if ( bcmUdcP->gadget_driver )
    {
        BCM_KWARN( "UDC driver busy\n" );
        spin_unlock_irqrestore( &bcmUdcP->lock, flags );
        return( -EBUSY );
    }

    // Hook up the gadget driver to the UDC controller driver
    //
    gadget_driver->driver.bus = NULL;
    bcmUdcP->gadget_driver = gadget_driver;
    bcmUdcP->gadget.dev.driver = &gadget_driver->driver;

    spin_unlock_irqrestore( &bcmUdcP->lock, flags  );

    BCM_KTRACE( "%s gadget_driver->bind()\n", gadget_driver->driver.name );

    err = gadget_driver->bind( &bcmUdcP->gadget );

    if ( err)
    {
        BCM_KERROR( "%s gadget_driver->bind() failed, err=%d\n", gadget_driver->driver.name, err );
        bcmUdcP->gadget.dev.driver = NULL;
        bcmUdcP->gadget_driver = NULL;
    }
    else
    {
        BCM_KINFO( "%s gadget_driver->bind() done\n", gadget_driver->driver.name );

        /// @todo Move the BUS connect/disconnect to CltrStartup/Shutdown??
        spin_lock_irqsave( &bcmUdcP->lock, flags );
        UdcOpsStartup( bcmUdcP );
        usbDevHw_DeviceBusConnect();
        spin_unlock_irqrestore( &bcmUdcP->lock, flags );
    }

    BCM_KTRACE2( "exit\n" );

    return( err );
}
EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver( struct usb_gadget_driver *gadget_driver )
{
    unsigned long flags;

    BCM_KTRACE2( "enter\n" );

    /// @todo somehow get rid of bcmUdcP global variable?
    if ( bcmUdcP == NULL )
    {
        BCM_KERROR( "UDC driver not initialized\n" );
        return( -ENODEV );
    }

    if ( !gadget_driver || (gadget_driver != bcmUdcP->gadget_driver) )
    {
        BCM_KERROR( "invalid gadget driver\n" );
        return( -EINVAL );
    }

    /// @todo Is this delay needed? If so, state why.
    usbDevHw_DeviceBusDisconnect();
    udelay(20);

    spin_lock_irqsave( &bcmUdcP->lock, flags );
    UdcOpsShutdown( bcmUdcP );
    spin_unlock_irqrestore( &bcmUdcP->lock, flags );

    BCM_KTRACE( "%s gadget_driver->disconnect()\n", gadget_driver->driver.name );
    bcmUdcP->gadget_driver->disconnect( &bcmUdcP->gadget );

    BCM_KTRACE( "%s gadget_driver->unbind()\n", gadget_driver->driver.name );
    gadget_driver->unbind( &bcmUdcP->gadget );

    bcmUdcP->gadget.dev.driver = NULL;
    bcmUdcP->gadget_driver = NULL;

    BCM_KINFO( "%s gadget_driver->unbind() done\n", gadget_driver->driver.name );

    return( ENOERROR );
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);


/* ==== Private Functions ================================================ */

//***************************************************************************
// APIs used to bind / unbind UDC devices to the UDC driver.
//***************************************************************************

int PlatformDriverProbe( struct platform_device *platformDevP )
{
    int err;


    BCM_KTRACE2( "enter\n" );

    /// @todo somehow get rid of bcmUdcP global variable? e.g. reference in platformDevP?
    if ( bcmUdcP != NULL )
    {
        BCM_KERROR( "device already attached\n" );
        return( -EBUSY );
    }

    if ( (bcmUdcP = kmalloc( sizeof(*bcmUdcP), GFP_KERNEL)) == NULL )
    {
        BCM_KERROR( "kmalloc() failed\n" );
        return( -ENOMEM );
    }

    /// @todo put these in UdcOpsInit()??
    memset( bcmUdcP, 0, sizeof(*bcmUdcP) );
    spin_lock_init( &bcmUdcP->lock );

    /// @todo Have to make sure all interrupts are off at the lower layers before requesting (enabling) at the top level.
    if ( (err = request_irq(BCM_UDC_IRQ, IrqUdc, 0, BCM_UDC_NAME, (void *)bcmUdcP)) < 0 )
    {
        BCM_KERROR( "request_irq() failed\n" );
        goto err1;
    }

    if ( (err = PlatformDmaAlloc(platformDevP, bcmUdcP)) < 0 )
    {
        BCM_KERROR( "PlatformDmaAlloc() failed\n" );
        goto err2;
    }

    /// @todo remove UdcOpsInit() ?? or is this called something else??
    UdcOpsInit( bcmUdcP );
    GadgetDevInit( platformDevP, bcmUdcP );

    if ( (err = GadgetDevAdd(bcmUdcP)) < 0 )
    {
        BCM_KERROR( "GadgetDevAdd() failed\n" );
        goto err3;
    }

    ProcFileCreate();

    #ifdef DEBUG
    usbDevHw_PrintInfo( printk );
    DmaDump( bcmUdcP );
    #endif

    BCM_KTRACE( "exit ok\n" );

    return( ENOERROR );

err3:
    PlatformDmaFree( platformDevP, bcmUdcP );
err2:
    free_irq( BCM_UDC_IRQ, bcmUdcP );
err1:
    kfree( bcmUdcP );
    bcmUdcP = NULL;

    BCM_KTRACE( "exit error\n" );

    return( err );
}

int PlatformDriverRemove( struct platform_device *platformDevP )
{
    BCM_KTRACE2( "enter\n" );

    if ( !bcmUdcP )
    {
        return( -ENODEV );
    }

    ProcFileRemove();
    GadgetDevRemove( bcmUdcP );
    UdcOpsFinis( bcmUdcP );

    PlatformDmaFree( platformDevP, bcmUdcP );
    free_irq( BCM_UDC_IRQ, bcmUdcP );

    kfree( bcmUdcP );
    bcmUdcP = NULL;

    BCM_KTRACE( "exit ok\n" );

    return( ENOERROR );
}

//***************************************************************************
// Platform device level alloc / free of memory used for DMA descriptors.
// A single block of memory static in size is used for DMA descriptors.
// Each endpoint has a small number of descriptors for its exclusive use.
// These are chained in a loop. See bcm_udc_dwc.h and DmaEpInit() for more
// details.
//***************************************************************************

int PlatformDmaAlloc( struct platform_device *platformDevP, BCM_UDC_t *udcP )
{
    udcP->dma.virtualAddr = dma_alloc_coherent( &platformDevP->dev, sizeof(BCM_UDC_DMA_t),
                                                (dma_addr_t *)&udcP->dma.physicalAddr, GFP_KERNEL );

    if ( !udcP->dma.virtualAddr )
    {
        BCM_KERROR("dma_alloc_coherent() failed\n");
        return( -ENOMEM );
    }

    return( ENOERROR );
}

void PlatformDmaFree( struct platform_device *platformDevP, BCM_UDC_t *udcP )
{
    unsigned num;


    dma_free_coherent( &platformDevP->dev, sizeof(BCM_UDC_DMA_t), udcP->dma.virtualAddr,
                        (dma_addr_t)udcP->dma.physicalAddr );

    for ( num = 0; num < BCM_UDC_EP_CNT; num ++ )
    {
        if ( udcP->ep[num].dma.alignedBuf )
        {
            dma_free_coherent( NULL, udcP->ep[num].dma.alignedLen, udcP->ep[num].dma.alignedBuf, udcP->ep[num].dma.alignedAddr );
            udcP->ep[num].dma.alignedBuf = NULL;
        }
    }
}

//***************************************************************************
// Linux Gadget device operations.
//***************************************************************************

void GadgetDevInit( struct platform_device *platformDevP, BCM_UDC_t *udcP )
{
    unsigned num;


    BCM_KTRACE2( "enter\n" );

    udcP->gadget.name = BCM_UDC_NAME;
    udcP->gadget.speed = USB_SPEED_UNKNOWN;
    udcP->gadget.ops = &bcm_udc_gadgetDevOps;
    udcP->gadget.is_dualspeed = 1;

    udcP->gadget.ep0 = &udcP->ep[0].usb_ep;
    INIT_LIST_HEAD(&udcP->gadget.ep_list);
    for ( num = 1; num < BCM_UDC_EP_CNT; num ++ )
    {
        list_add_tail( &udcP->ep[num].usb_ep.ep_list, &udcP->gadget.ep_list );
    }

    strcpy(udcP->gadget.dev.bus_id, "gadget");
    udcP->gadget.dev.release = GadgetDevRelease;
    udcP->gadget.dev.parent = &platformDevP->dev;

    BCM_KTRACE( "exit ok\n" );
}

int GadgetDevAdd( BCM_UDC_t *udcP )
{
    return( device_register(&udcP->gadget.dev) );
}

void GadgetDevRelease( struct device *dev )
{
    BCM_KTRACE2( "enter\n" );

    /// @todo Is this required here?? Anything else need to be done??
    // Shutdown the hardware operations
    //
    usbDevHw_OpsFinis();

    /// @todo remove global variable reference somehow?
    complete( bcmUdcP->devRelease );

    BCM_KTRACE( "exit ok\n" );
}

int GadgetDevRemove( BCM_UDC_t *udcP )
{
    DECLARE_COMPLETION(completion);

    BCM_KTRACE2( "enter\n" );

    udcP->devRelease = &completion;
    device_unregister( &udcP->gadget.dev );
    wait_for_completion( udcP->devRelease );

    BCM_KTRACE( "exit ok\n" );

    return( ENOERROR );
}

//***************************************************************************
// Linux Gadget endpoint operations. See usb_ep_ops in usb_gadget.h.
//***************************************************************************

int GadgetEpEnable( struct usb_ep *usb_ep, const struct usb_endpoint_descriptor *desc )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);
    BCM_UDC_t *udcP;
    unsigned long flags;
    unsigned maxPktSize;



    BCM_KTRACE2( "enter\n" );


    if ( !usb_ep || (udcEpP->bEndpointAddress != desc->bEndpointAddress) )
    {
        BCM_KERROR( "invalid endpoint (%p)\n", usb_ep );
        return( -EINVAL );
    }

    if ( !desc || udcEpP->desc || (desc->bDescriptorType != USB_DT_ENDPOINT) )
    {
        BCM_KERROR( "ep%d: invalid descriptor (%p)\n", udcEpP->num, desc );
        return( -EINVAL );
    }

    udcP = udcEpP->udcP;

    if ( !udcP->gadget_driver || (udcP->gadget.speed == USB_SPEED_UNKNOWN) )
    {
        BCM_KWARN( "%s: invalid device state\n", udcEpP->usb_ep.name );
        return( -ESHUTDOWN );
    }

    maxPktSize = le16_to_cpu (desc->wMaxPacketSize);
    if ( !maxPktSize || (maxPktSize > udcEpP->maxPktSize) || (maxPktSize & 0x3) )
    {
        // The FIFO depths are specified in 32-bit word units. Cannot handle partial units.
        //
        /// @todo partial units actually handled in s/w, not sure if OK at h/w level
        BCM_KERROR( "%s: invalid max pkt size\n", udcEpP->usb_ep.name );
        return( -ERANGE );
    }

    spin_lock_irqsave( &udcP->lock, flags );

    if ( UdcEpCfg( udcEpP, desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK, maxPktSize ) != ENOERROR )
    {
        spin_unlock_irqrestore( &udcP->lock, flags );
        BCM_KERROR( "%s: not enough FIFO space\n", udcEpP->usb_ep.name );
        return( -ENOSPC );
    }
    ///@todo Rework the UdcEpCfg() so it includes usbDevHw_EndptCfgSet() ...
    usbDevHw_EndptCfgSet( udcEpP->num, usbDevHw_DeviceCfgNum() );

    udcEpP->desc = desc;
    udcEpP->stopped = 0;
    udcEpP->usb_ep.maxpacket = maxPktSize;


    spin_unlock_irqrestore( &udcP->lock, flags );

    BCM_KTRACE( "%s: enabled\n", udcEpP->usb_ep.name );
    BCM_KTRACE2( "exit\n" );

    return( ENOERROR );
}

int GadgetEpDisable( struct usb_ep *usb_ep )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);
    BCM_UDC_t *udcP;
    unsigned long flags;


    BCM_KTRACE2( "enter\n" );

    if ( !usb_ep )
    {
        BCM_KERROR( "invalid endpoint\n" );
        return( -EINVAL );
    }

    if ( !udcEpP->desc )
    {
        BCM_KTRACE( "%s: already disabled\n", udcEpP->usb_ep.name );
        return( ENOERROR );
    }

    udcP = udcEpP->udcP;

    /// @todo Really need to do this around udcEpP->desc check
    spin_lock_irqsave(&udcP->lock, flags);

    ReqQueueFlush( udcEpP, -ESHUTDOWN );

    usbDevHw_EndptIrqDisable( udcEpP->num, udcEpP->dirn );
    udcEpP->desc = NULL;
    udcEpP->usb_ep.maxpacket = udcEpP->maxPktSize;
    UdcFifoRamFree( udcEpP );

    spin_unlock_irqrestore(&udcP->lock, flags);

    BCM_KTRACE("%s: disabled\n", udcEpP->usb_ep.name );

    return( ENOERROR );
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)

void *GadgetEpBufferAlloc( struct usb_ep *usb_ep, unsigned bytes, dma_addr_t *dma, unsigned gfp_flags )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);


    BCM_KTRACE( "%s: buf=0x%p flags=0x%x\n", usb_ep->name, buf, gfp_flags );

    return( dma_alloc_coherent(udcEpP->udcP->gadget.dev.parent, bytes, dma, gfp_flags) );
}

void GadgetEpBufferFree( struct usb_ep *usb_ep, void *buf, dma_addr_t dma, unsigned bytes )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);


    BCM_KTRACE( "%s: buf=0x%p\n", usb_ep->name, buf );

    dma_free_coherent(udcEpP->udcP->gadget.dev.parent, bytes, buf, dma);
}

#endif  // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)

struct usb_request * GadgetEpRequestAlloc( struct usb_ep *usb_ep, unsigned gfp_flags )
{
    BCM_UDC_EP_REQ_t *udcEpReqP;


    BCM_KTRACE2( "enter\n" );

    /// @todo Use usb_ep for sanity check??
    (void)usb_ep;

    if ( (udcEpReqP = kmalloc( sizeof(*udcEpReqP), gfp_flags )) != NULL )
    {
        // Set the usb_req.dma to DMA_ADDR_INVALID so it can be determined if the usb_req.buf needs
        // to be mapped when the request is subsequently queued.
        //
        memset( udcEpReqP, 0, sizeof(*udcEpReqP) );
        INIT_LIST_HEAD( &udcEpReqP->listNode );
        udcEpReqP->usb_req.dma = DMA_ADDR_INVALID;

        BCM_KTRACE3( "%s: req=0x%p flags=0x%x\n", usb_ep->name, udcEpReqP, gfp_flags );
        BCM_KTRACE2( "exit ok\n" );

        return( &udcEpReqP->usb_req );
    }

    /// @todo Replace this with a warning
    BCM_KTRACE2( "exit error\n" );

    return( NULL );
}


void  GadgetEpRequestFree( struct usb_ep *usb_ep, struct usb_request *usb_req )
{
    BCM_UDC_EP_REQ_t *udcEpReqP = container_of(usb_req, BCM_UDC_EP_REQ_t, usb_req);


    BCM_KTRACE2( "enter\n" );

    /// @todo Use usb_ep for sanity check??
    (void)usb_ep;

    if ( usb_req )
    {
        BCM_KTRACE3( "%s: req=0x%p\n", usb_ep->name, udcEpReqP );
        kfree( udcEpReqP );
    }

    BCM_KTRACE2( "exit\n" );
}

int GadgetEpRequestQueue( struct usb_ep *usb_ep, struct usb_request *usb_req, unsigned gfp_flags )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);
    BCM_UDC_EP_REQ_t *udcEpReqP = container_of(usb_req, BCM_UDC_EP_REQ_t, usb_req);
    BCM_UDC_t *udcP;
    unsigned long flags;


    BCM_KTRACE2( "enter\n" );

    if ( !usb_ep || !usb_req || !udcEpReqP->usb_req.complete || !udcEpReqP->usb_req.buf || !list_empty(&udcEpReqP->listNode) )
    {
        BCM_KERROR( "invalid request\n" );
        BCM_KTRACE( "usb_ep=0x%p udc_req=0x%p usb_req=0x%p usb_req.complete=0x%p usb_req.buf=0x%p\n",
                     usb_ep, udcEpReqP, usb_req, udcEpReqP->usb_req.complete, udcEpReqP->usb_req.buf );
        return( -EINVAL );
    }

    if ( !udcEpP->desc && (udcEpP->num != 0) )
    {
        BCM_KERROR( "%s: invalid EP state\n", udcEpP->usb_ep.name );
        return( -EFAULT );
    }

    if ( (udcEpP->type == USB_ENDPOINT_XFER_CONTROL) && !list_empty(&udcEpP->listQueue) )
    {
        BCM_KERROR( "%s: CTRL EP queue not empty\n", udcEpP->usb_ep.name );
        return( -EPERM );
    }

    if ( usb_req->length > 0xffff )
    {
        BCM_KERROR( "%s: request too big, length=%u\n", udcEpP->usb_ep.name, usb_req->length );
        return( -E2BIG );
    }

    udcP = udcEpP->udcP;

    if ( !udcP->gadget_driver || (udcP->gadget.speed == USB_SPEED_UNKNOWN) )
    {
        BCM_KWARN( "%s: invalid device state\n", udcEpP->usb_ep.name );
        return( -ESHUTDOWN );
    }

    if ( ((unsigned long)udcEpReqP->usb_req.buf) & 0x3UL )
    {
        BCM_KTRACE2( "%s: invalid buffer alignment: addr=0x%p\n", udcEpP->usb_ep.name, udcEpReqP->usb_req.buf );

        // The DMA buffer does not have the alignment required by the hardware. We keep an endpoint level
        // buffer available to handle this situation if it arises. If we don't currently have one available
        // for this purpose, or if the current one is not large enough, then allocate a new one. Since
        // we only have one buffer, we won't copy into the buffer until we are ready to do the DMA transfer.
        // Mark the request as needing this alignment (copy).

        if ( (udcEpP->dma.alignedBuf != NULL) && (udcEpP->dma.alignedLen < udcEpReqP->usb_req.length) )
        {
            BCM_KTRACE( "%s: dma_free_coherent(): addr=0x%x length=%u\n", udcEpP->usb_ep.name, udcEpP->dma.alignedAddr, udcEpP->dma.alignedLen );
            dma_free_coherent( NULL, udcEpP->dma.alignedLen, udcEpP->dma.alignedBuf, udcEpP->dma.alignedAddr );
            udcEpP->dma.alignedBuf = NULL;
        }

        if ( udcEpP->dma.alignedBuf == NULL )
        {
            udcEpP->dma.alignedLen = udcEpReqP->usb_req.length;
            udcEpP->dma.alignedBuf = dma_alloc_coherent( NULL, udcEpP->dma.alignedLen, &(udcEpP->dma.alignedAddr), GFP_KERNEL );
            BCM_KTRACE( "%s: dma_alloc_coherent(): addr=0x%x length=%u\n", udcEpP->usb_ep.name, udcEpP->dma.alignedAddr, udcEpP->dma.alignedLen );
        }

        if ( udcEpP->dma.alignedBuf == NULL )
        {
            BCM_KERROR( "%s: dma_alloc_coherent() failed, length=%u\n", udcEpP->usb_ep.name, usb_req->length );
            return( -ENOMEM );
        }

        udcEpReqP->dmaAligned = 1;
    }
    /// @todo Should not really have dma == 0. Unfortunately the ether_262317.c has been hacked to do this. See Trac #1318.
    else if ( (udcEpReqP->usb_req.dma == DMA_ADDR_INVALID) || (udcEpReqP->usb_req.dma == 0) )
    {
        // A physical address was not provided for the DMA buffer, so request it.
        //
        udcEpReqP->dmaMapped = 1;
        udcEpReqP->usb_req.dma = dma_map_single( udcEpP->udcP->gadget.dev.parent,
                                                    udcEpReqP->usb_req.buf,
                                                    udcEpReqP->usb_req.length,
                                                    (udcEpP->dirn == USB_DIR_IN ? DMA_TO_DEVICE : DMA_FROM_DEVICE) );
    }

    spin_lock_irqsave(&udcP->lock, flags);

    udcEpReqP->usb_req.status = -EINPROGRESS;
    udcEpReqP->usb_req.actual = 0;

    BCM_KTRACE( "%s: req=0x%p len=%d buf=0x%p dma=0x%x\n", udcEpP->usb_ep.name, usb_req, usb_req->length, usb_req->buf, udcEpReqP->usb_req.dma );

    if ( (udcEpP->type == USB_ENDPOINT_XFER_CONTROL) && (udcEpP->dirn == USB_DIR_OUT) && (udcEpReqP->usb_req.length == 0) )
    {
        // This might happen if gadget driver decides to send zero length packet (ZLP) during STATUS phase
        // of a control transfer. This may happen for the cases where there is not a DATA phase. Just consider
        // things complete. ZLP will be issued by hardware. See the handling of SETUP packets for more details
        // on control transfer processing.
        //
        ReqXferDone( udcEpP, udcEpReqP, ENOERROR );
    }
    else
    {
        list_add_tail( &udcEpReqP->listNode, &udcEpP->listQueue );
        ReqXferStart( udcEpP );
    }

    spin_unlock_irqrestore( &udcP->lock, flags );

    /// @todo add request info, e.g. address
    BCM_KTRACE2( "exit\n" );

    return( ENOERROR );
}

int GadgetEpRequestDequeue( struct usb_ep *usb_ep, struct usb_request *usb_req )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);
    BCM_UDC_EP_REQ_t *udcEpReqP = container_of(usb_req, BCM_UDC_EP_REQ_t, usb_req);
    unsigned long flags;


    BCM_KTRACE2( "enter\n" );

    if ( !usb_ep || !usb_req )
    {
        BCM_KERROR( "invalid request\n" );
        return( -EINVAL );
    }

    spin_lock_irqsave(&udcEpP->udcP->lock, flags);

    /* make sure it's actually queued on this endpoint */
    list_for_each_entry( udcEpReqP, &udcEpP->listQueue, listNode )
    {
        if (&udcEpReqP->usb_req == usb_req)
            break;
    }

    if (&udcEpReqP->usb_req != usb_req)
    {
        spin_unlock_irqrestore(&udcEpP->udcP->lock, flags);
        BCM_KWARN( "%s: request not queued\n", udcEpP->usb_ep.name );
        return( -ENOLINK );
    }

    /// @todo Handle case where the request is in progress, or completed but not dequeued

    ReqXferDone(udcEpP, udcEpReqP, -ECONNRESET);
    spin_unlock_irqrestore(&udcEpP->udcP->lock, flags);

    BCM_KTRACE( "%s: req=0x%p\n", udcEpP->usb_ep.name, usb_req );
    BCM_KTRACE2( "exit\n" );

    return( ENOERROR );
}


int GadgetEpSetHalt( struct usb_ep *usb_ep, int haltEnable )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);
    unsigned long flags;


    BCM_KTRACE2( "enter\n" );

    if ( !usb_ep )
    {
        BCM_KERROR( "invalid request\n" );
        return( -EINVAL );
    }

    if (udcEpP->type == USB_ENDPOINT_XFER_ISOC)
    {
        BCM_KWARN( "%s: ISO HALT operations not supported\n", udcEpP->usb_ep.name );
        return( -EOPNOTSUPP );
    }

    if ( haltEnable && (udcEpP->dirn == USB_DIR_IN) && !list_empty(&udcEpP->listQueue) )
    {
        // Only allow halt on an IN EP if its queue is empty
        //
        BCM_KWARN( "%s: IN queue not empty\n", udcEpP->usb_ep.name );
        return( -EAGAIN );
    }

    if ( !haltEnable && (udcEpP->type == USB_ENDPOINT_XFER_CONTROL) )
    {
        // Halt clear for a control EP should only be handled as part of the subsequent SETUP
        // exchange that occurs after the Halt was set.
        //
        BCM_KWARN( "%s: CTRL HALT clear\n", udcEpP->usb_ep.name );
        return( -EPROTO );
    }

    spin_lock_irqsave( &udcEpP->udcP->lock, flags );

    if ( !haltEnable )
    {
        usbDevHw_EndptStallDisable( udcEpP->num, udcEpP->dirn );
    }
    else if ( udcEpP->type != USB_ENDPOINT_XFER_CONTROL )
    {
        usbDevHw_EndptStallEnable( udcEpP->num, udcEpP->dirn );
    }
    else
    {
        usbDevHw_EndptStallEnable( udcEpP->num, USB_DIR_IN );
        usbDevHw_EndptStallEnable( udcEpP->num, USB_DIR_OUT );
    }

    spin_unlock_irqrestore( &udcEpP->udcP->lock, flags );

    BCM_KTRACE( "%s: HALT %s done\n", udcEpP->usb_ep.name, haltEnable ? "SET" : "CLR" );
    BCM_KTRACE2( "exit\n" );

    return( ENOERROR );
}

int GadgetEpFifoStatus( struct usb_ep *usb_ep )
{
    BCM_KTRACE( "enter/exit\n" );

    // The DWC UDC core doesn't have a mechanism for determining the number of bytes
    // currently in a FIFO. The best that can be done is determine whether or not a
    // FIFO is empty. However, for the situation where a single Rx FIFO is being
    // used for all endpoints, if cannot be determined which OUT and CTRL EP's are
    // affected if the Rx FIFO is not empty.

    return( -EOPNOTSUPP );
}


void GadgetEpFifoFlush( struct usb_ep *usb_ep )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);
    unsigned long flags;


    BCM_KTRACE2( "enter\n" );

    if ( !usb_ep )
    {
        BCM_KERROR( "invalid request\n" );
        return;
    }

    if ( udcEpP->type == USB_ENDPOINT_XFER_CONTROL )
    {
        // FIFO flush for a control EP does not make any sense. The SETUP protocol
        // should eliminate the need to flush.
        //
        BCM_KWARN( "%s: CTRL FIFO flush\n", udcEpP->usb_ep.name );
        return;
    }

    if ( usbDevHw_EndptFifoEmpty( udcEpP->num, udcEpP->dirn ) )
    {
        BCM_KTRACE( "%s: empty\n", udcEpP->usb_ep.name );
        return;
    }

    spin_lock_irqsave( &udcEpP->udcP->lock, flags );

    /// @todo There may be some issues for single Rx FIFO and subsequent EP0 operations
    /// @todo The UDC doc'n also mentions having to set DEVNAK bit and clearing it later.
    // FIFO flush will need to be disabled later on. E.g. when a EP request is queued.
    //
    usbDevHw_EndptFifoFlushEnable( udcEpP->num, udcEpP->dirn );

    spin_unlock_irqrestore( &udcEpP->udcP->lock, flags );

    BCM_KTRACE( "%s: FIFO flush enabled\n", udcEpP->usb_ep.name );
    BCM_KTRACE2( "exit\n" );
}

//***************************************************************************
// Routines for debug dump of DMA descriptors
//***************************************************************************

#ifdef DEBUG
void DmaDump( BCM_UDC_t *udcP )
{
    unsigned i;


    for ( i = 0; i < BCM_UDC_EP_CNT; i++ )
    {
        DmaDumpEp( &udcP->ep[i] );
    }
}
#endif

void DmaDumpDesc( char *label, BCM_UDC_DMA_DESC_t *virt, BCM_UDC_DMA_DESC_t *phys )
{
    printk( "%s virt=0x%p phys=0x%p:", label, virt, phys );
    printk( " 0x%08x", virt->status );
    printk( " 0x%08x", virt->reserved );
    printk( " 0x%08x", virt->bufAddr );
    printk( " 0x%08x", virt->nextDescAddr );
    printk( "\n" );
}

void DmaDumpEp( BCM_UDC_EP_t *udcEpP )
{
    unsigned i;


    printk(      "EP %d DMA\n", udcEpP->num );
    printk(      "   setup\n" );
    DmaDumpDesc( "       ", (BCM_UDC_DMA_DESC_t *)&udcEpP->dma.virtualAddr->setup, (BCM_UDC_DMA_DESC_t *)&udcEpP->dma.physicalAddr->setup );
    printk(      "   desc\n" );

    for ( i = 0; i < BCM_UDC_EP_DMA_DESC_CNT; i++ )
    {
        DmaDumpDesc( "       ", &udcEpP->dma.virtualAddr->desc[i], &udcEpP->dma.physicalAddr->desc[i] );
    }

    printk( "\n" );
}

//***************************************************************************
// Initialization of DMA descriptors at the endpoint level.
//***************************************************************************

void DmaEpInit( BCM_UDC_EP_t *udcEpP )
{
    unsigned i;


    BCM_KTRACE2( "enter: num=%u\n", udcEpP->num );

    /// @todo shorten names to virtAddr physAddr??
    udcEpP->dma.virtualAddr = &udcEpP->udcP->dma.virtualAddr->ep[ udcEpP->num ];
    udcEpP->dma.physicalAddr = &udcEpP->udcP->dma.physicalAddr->ep[ udcEpP->num ];

    // Control endpoints only do setup in the OUT direction, so only need to set the
    // buffer address for that direction. The buffer is set, even if not a control
    // endpoint, just to simplify things. There's no harm with this.
    //
    udcEpP->dma.virtualAddr->setup.status = usbDevHw_REG_DMA_STATUS_BUF_HOST_BUSY;
    wmb();
    usbDevHw_EndptDmaSetupBufAddrSet( udcEpP->num, USB_DIR_OUT, &udcEpP->dma.physicalAddr->setup );

    // Take ownership of the DMA descriptors, and chain them in a loop. This allows a small number
    // descriptors to be used for requests. Need to have the DWC DMA Descriptor Update option enabled
    // in the device control register in order to do this. When a transfer for a descriptor completes,
    // the descriptor will get re-used if there's still data left in a request to transfer. See the
    // DmaDataRemoveDone() and DmaDataAddReady() routines.
    /// @todo Put these in endpoint context??
    //
    for ( i = 0; i < BCM_UDC_EP_DMA_DESC_CNT; i++ )
    {
        udcEpP->dma.virtualAddr->desc[i].status = usbDevHw_REG_DMA_STATUS_BUF_HOST_BUSY;
        wmb();
        udcEpP->dma.virtualAddr->desc[i].nextDescAddr = (uint32_t)&udcEpP->dma.physicalAddr->desc[i+1];
    }
    udcEpP->dma.virtualAddr->desc[(BCM_UDC_EP_DMA_DESC_CNT - 1)].nextDescAddr = (uint32_t)&udcEpP->dma.physicalAddr->desc[0];

    // To simplify things, register the descriptor chain in both directions. Control endpoints are the
    // only type that will be transferring in both directions, but they will only be transferring in one
    // direction at a time, so should not be any issues with using the same descriptor set for both directions.
    // For single direction endpoints, the other direction will not be used.
    //
    usbDevHw_EndptDmaDataDescAddrSet( udcEpP->num, USB_DIR_OUT, &udcEpP->dma.physicalAddr->desc[0] );
    usbDevHw_EndptDmaDataDescAddrSet( udcEpP->num, USB_DIR_IN,  &udcEpP->dma.physicalAddr->desc[0] );

    BCM_KTRACE( "exit\n" );
}

//***************************************************************************
// DMA descriptor chain routines.
//
//  DmaDescChainReset - Initialize chain in preparation for transfer
//  DmaDescChainFull - Indicates if no descriptors in chain for available for use.
//  DmaDescChainAlloc - Get next free descriptor for use. Have to check if chain not full first.
//  DmaDescChainEmpty - Indicates if no descriptors in the chain are being used.
//  DmaDescChainHead - Pointer to 1st entry in chain. Have to check if chain not empty first.
//  DmaDescChainFree - Frees up 1st entry for use. Only do this if DMA for this descriptor has completed.
//
//***************************************************************************

inline BCM_UDC_DMA_DESC_t *DmaDescChainAlloc( BCM_UDC_EP_t *udcEpP )
{
    unsigned idx;

    idx = udcEpP->dma.addIndex++;

    return( &udcEpP->dma.virtualAddr->desc[ BCM_UDC_EP_DMA_DESC_IDX(idx) ] );
}

inline int DmaDescChainEmpty( BCM_UDC_EP_t *udcEpP )
{
    return( udcEpP->dma.addIndex == udcEpP->dma.removeIndex );
}

inline void DmaDescChainFree( BCM_UDC_EP_t *udcEpP )
{
    udcEpP->dma.removeIndex++;
}

inline int DmaDescChainFull( BCM_UDC_EP_t *udcEpP )
{
    return( !DmaDescChainEmpty(udcEpP) && (BCM_UDC_EP_DMA_DESC_IDX(udcEpP->dma.addIndex) == BCM_UDC_EP_DMA_DESC_IDX(udcEpP->dma.removeIndex)) );
}

inline BCM_UDC_DMA_DESC_t *DmaDescChainHead( BCM_UDC_EP_t *udcEpP )
{
    return( &udcEpP->dma.virtualAddr->desc[ BCM_UDC_EP_DMA_DESC_IDX(udcEpP->dma.removeIndex) ] );
}

inline void DmaDescChainReset( BCM_UDC_EP_t *udcEpP )
{
    udcEpP->dma.addIndex = 0;
    udcEpP->dma.removeIndex = 0;
}

//***************************************************************************
// DMA data routines.
//
// A gadget usb_request buf is used for the data. The entire buf contents may
// or may not fit into the descriptor chain at once. When the DMA transfer
// associated with a descriptor completes, the descriptor is re-used to add
// more segments of the usb_request to the chain as necessary.
//
//  DmaDataInit - Initialization in preparation for DMA of usb_request.
//  DmaDataAddReady - Adds usb_request segments into DMA chain until full or no segments left
//  DmaDataRemoveDone - Removes usb_request segments from DMA chain that have completed transfer
//  DmaDataFinis - Final stage of DMA of the usb_request
//
//***************************************************************************

void DmaDataInit( BCM_UDC_EP_t *udcEpP )
{
    BCM_UDC_EP_REQ_t *udcEpReqP;

    BCM_KTRACE2( "enter\n" );


    udcEpReqP = list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode );

    if ( udcEpReqP->dmaAligned )
    {
        // This buffer needs to be aligned in order to DMA. We do this by copying into a special buffer we
        // have for this purpose. Save the original DMA physical address so it can be restored later.
        // This may not be used, but we'll do it anyways. Then set the DMA address to the aligned buffer
        // address. Only the DMA physical address is used for the transfers, so the original buffer virtual
        // address does not need to be changed. Then copy the data into the aligned buffer.
        /// @todo Really only need to do the memcpy for IN data

        udcEpReqP->dmaAddrOrig = udcEpReqP->usb_req.dma;
        udcEpReqP->usb_req.dma = udcEpP->dma.alignedAddr;
        memcpy( udcEpP->dma.alignedBuf, udcEpReqP->usb_req.buf, udcEpReqP->usb_req.length );
    }

    udcEpP->dma.lengthDone = 0;
    udcEpP->dma.lengthToDo = udcEpP->dma.usb_req->length;
    udcEpP->dma.bufAddr = udcEpP->dma.usb_req->dma;
    udcEpP->dma.status = usbDevHw_REG_DMA_STATUS_RX_SUCCESS;

    if ( udcEpP->dirn == USB_DIR_IN )
    {
        // For IN transfers, do not need to segment the buffer into max packet portions
        // for the DMA descriptors. The hardware will automatically segment into max
        // packet sizes as necessary.
        //
        udcEpP->dma.lengthBufMax = udcEpP->dma.usb_req->length;

        // If the request is of zero length, then force the zero flag so DmaDataAddReady()
        // will queue the request. Conversely, if the gadget has set the zero flag, leave
        // it set only if it is needed (request length is a multiple of maxpacket)
        //
        if ( udcEpP->dma.usb_req->length == 0 )
        {
            udcEpP->dma.usb_req->zero = 1;
        }
        else if ( udcEpP->dma.usb_req->zero )
        {
            udcEpP->dma.usb_req->zero = (udcEpP->dma.usb_req->length % udcEpP->usb_ep.maxpacket) ? 0 : 1;
        }
    }
    else
    {
        udcEpP->dma.lengthBufMax = udcEpP->usb_ep.maxpacket;
    }

    DmaDescChainReset( udcEpP );

    BCM_KTRACE( "%s: todo=%d done=%d bufMax=%d buf=0x%x add=0x%x remove=0x%x\n",
                udcEpP->usb_ep.name, udcEpP->dma.lengthToDo, udcEpP->dma.lengthDone,
                udcEpP->dma.lengthBufMax, udcEpP->dma.bufAddr, udcEpP->dma.addIndex, udcEpP->dma.removeIndex );

    usbDevHw_EndptFifoFlushDisable( udcEpP->num, udcEpP->dirn );
    usbDevHw_EndptIrqEnable( udcEpP->num, udcEpP->dirn );

    BCM_KTRACE2( "exit\n" );
}

void DmaDataFinis( BCM_UDC_EP_t *udcEpP )
{
    BCM_UDC_EP_REQ_t *udcEpReqP;


    BCM_KTRACE2( "enter\n" );

    usbDevHw_EndptIrqDisable( udcEpP->num, udcEpP->dirn );
    usbDevHw_EndptDmaDisable( udcEpP->num, udcEpP->dirn );

    udcEpReqP = list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode );

    if ( udcEpReqP->dmaAligned )
    {
        // The original request buffer was not aligned properly, so a special buffer was used
        // for the transfer. Copy the aligned buffer contents into the original. Also restore
        // the original dma physical address.
        /// @todo Really only need to do the memcpy for OUT setup/data

        memcpy( udcEpReqP->usb_req.buf, udcEpP->dma.alignedBuf, udcEpReqP->usb_req.length );
        udcEpReqP->usb_req.dma = udcEpReqP->dmaAddrOrig;
    }

    BCM_KTRACE( "%s: exit\n", udcEpP->usb_ep.name );
}

void DmaDataAddReady( BCM_UDC_EP_t *udcEpP )
{
    volatile BCM_UDC_DMA_DESC_t *dmaDescP;
    uint32_t status;
    unsigned len;


    BCM_KTRACE2( "enter\n" );

    // Will only have one request in the chain at a time. Add request segments to the
    // chain until all parts of the request have been put in the chain or the chain
    // has no more room.
    //
    while ( !DmaDescChainFull( udcEpP ) && (udcEpP->dma.lengthToDo || udcEpP->dma.usb_req->zero) )
    {
        // Get the next descriptor in the chain, and then fill the descriptor contents as needed.
        // Do not set the descriptor buffer status to ready until last to ensure there's no
        // contention with the hardware.
        //
        dmaDescP = DmaDescChainAlloc( udcEpP );

        len = udcEpP->dma.lengthToDo < udcEpP->dma.lengthBufMax ? udcEpP->dma.lengthToDo : udcEpP->dma.lengthBufMax;
        udcEpP->dma.lengthToDo -= len;

        status = 0;

        if ( len < udcEpP->dma.lengthBufMax )
        {
            // If this segment is less than the max, then it is the last segment. There's no need to
            // send a closing ZLP, although this segment might be a ZLP. Regardless, clear the ZLP flag
            // to ensure that the processing of this request finishes. Also set the end of the descriptor
            // chain.
            //
            udcEpP->dma.usb_req->zero = 0;
            status |= usbDevHw_REG_DMA_STATUS_LAST_DESC;
        }
        else if ( (udcEpP->dma.lengthToDo == 0) && !udcEpP->dma.usb_req->zero )
        {
            // Segment is of the max length. Since there's nothing left, it has to also be the last
            // last segment. No closing ZLP segment requested, just set the end of the descriptor chain.
            //
            status |= usbDevHw_REG_DMA_STATUS_LAST_DESC;
        }

        /// @todo handle ISO PIDs and frame numbers

        dmaDescP->bufAddr = udcEpP->dma.bufAddr;
        status |= (len << usbDevHw_REG_DMA_STATUS_BYTE_CNT_SHIFT);
        dmaDescP->status = status | usbDevHw_REG_DMA_STATUS_BUF_HOST_READY;
        wmb();

        BCM_KTRACE( "%s: desc=0x%p status=0x%x buf=0x%x len=%d add=0x%x remove=0x%x\n",
                    udcEpP->usb_ep.name, dmaDescP, dmaDescP->status, dmaDescP->bufAddr,
                    len, udcEpP->dma.addIndex, udcEpP->dma.removeIndex );

        udcEpP->dma.bufAddr += len;
   }

    BCM_KTRACE2( "exit\n" );
}

void DmaDataRemoveDone( BCM_UDC_EP_t *udcEpP )
{
    volatile BCM_UDC_DMA_DESC_t *dmaDescP;
    uint32_t status;
    unsigned len;


    BCM_KTRACE2( "enter\n" );

    // Will only have one request in the chain at a time. Remove any completed
    // request segments from the chain so any segments awaiting transfer can
    // be put in the chain.
    //
    while ( !DmaDescChainEmpty( udcEpP ) )
    {
        // Examine the first entry in the chain. If its status is not done, then there's
        // nothing to remove.
        dmaDescP = DmaDescChainHead( udcEpP );
        if ( (dmaDescP->status & usbDevHw_REG_DMA_STATUS_BUF_MASK) != usbDevHw_REG_DMA_STATUS_BUF_DMA_DONE )
        {
            break;
        }

        // The transfer of this request segment has completed. Save the status info and then
        // take ownership of the descriptor. It is simpler to do this than modifying parts of
        // the descriptor in order to take ownership. Don't put the descriptor back in the chain
        // until all info affected by the status has been updated, just to be safe.
        //
        status = dmaDescP->status;
        dmaDescP->status = usbDevHw_REG_DMA_STATUS_BUF_HOST_BUSY;
        wmb();

        /// @todo handle ISO PIDs and frame numbers

        len = (status & usbDevHw_REG_DMA_STATUS_NON_ISO_BYTE_CNT_MASK) >> usbDevHw_REG_DMA_STATUS_NON_ISO_BYTE_CNT_SHIFT;
        udcEpP->dma.lengthDone += len;
        udcEpP->dma.usb_req->actual += len;

        if ( (status & usbDevHw_REG_DMA_STATUS_RX_MASK) != usbDevHw_REG_DMA_STATUS_RX_SUCCESS )
        {
            udcEpP->dma.status = status & usbDevHw_REG_DMA_STATUS_RX_MASK;
            udcEpP->dma.usb_req->status = -EIO;
            BCM_KWARN( "%s: DMA error: desc=0x%p status=0x%x len=%d add=0x%x remove=0x%x\n",
                        udcEpP->usb_ep.name, dmaDescP, status, len, udcEpP->dma.addIndex, udcEpP->dma.removeIndex );
        }
        else if ( (status & usbDevHw_REG_DMA_STATUS_LAST_DESC) && (udcEpP->dma.usb_req->status == -EINPROGRESS) )
        {
            udcEpP->dma.usb_req->status = ENOERROR;
        }

        DmaDescChainFree( udcEpP );

        BCM_KTRACE( "%s: desc=0x%p status=0x%x len=%d add=0x%x remove=0x%x\n",
                    udcEpP->usb_ep.name, dmaDescP, status, len, udcEpP->dma.addIndex, udcEpP->dma.removeIndex );
    }

    BCM_KTRACE2( "exit\n" );
}

//***************************************************************************
// UDC Operations routines.
//
//  UdcOpsInit - Initialization of the UDC in preparation for use by Gadget driver.
//  UdcOpsStartup - Start UDC operations. Happens after a Gadget driver attaches.
//  UdcOpsShutdown - Stop UDC operations. Happens after a Gadget driver detaches.
//  UdcOpsFinis - Finish / terminate all UDC operations
//
//***************************************************************************
static void UdcOpsFinis( BCM_UDC_t *udcP )
{
    /// @todo Anything need to be done here??
}

static void UdcOpsInit( BCM_UDC_t *udcP )
{
    BCM_KTRACE2( "enter\n" );
    BCM_KTRACE( "enter: dev: ctrl=0x%x stat=0x%x  irq mask: dev=0x%x ep=0x%x  irq status: dev=0x%x ep=0x%x  ep0: status=0x%x\n",
                 usbDevHw_REG_P->devCtrl, usbDevHw_REG_P->devStatus,
                 usbDevHw_REG_P->devIntrMask, usbDevHw_REG_P->eptIntrMask, usbDevHw_REG_P->devIntrStatus, usbDevHw_REG_P->eptIntrStatus,
                 usbDevHw_REG_P->eptFifoOut[0].status );

    usbDevHw_OpsInit();

    UdcFifoRamInit( udcP );

    // See usb/gadget/epautoconf.c for endpoint naming conventions.
    // Control endpoints are bi-directional, but initial transfer (SETUP stage) is always OUT.
    //
    /// @todo Really should make the non endpoint 0 init attributes configurable by the chip specific part
    /// of the driver, i.e. the device instantiation. The settings below are for a chip specific DWG UDC
    /// core configuration. Also should incorporate the DWG UDC endpoint type attribute as part of this,
    /// which can be control, IN, OUT, or bidirectional.
    //
    UdcEpInit( udcP, 0, "ep0",    USB_DIR_OUT );
    UdcEpInit( udcP, 1, "ep1in",  USB_DIR_IN );
    UdcEpInit( udcP, 2, "ep2in",  USB_DIR_IN );
    UdcEpInit( udcP, 3, "ep3out", USB_DIR_OUT );
    UdcEpInit( udcP, 4, "ep4in",  USB_DIR_IN );
    UdcEpInit( udcP, 5, "ep5out", USB_DIR_OUT );
    UdcEpInit( udcP, 6, "ep6in",  USB_DIR_IN );
    UdcEpInit( udcP, 7, "ep7out", USB_DIR_OUT );

    UdcEpCfg( &udcP->ep[0], USB_ENDPOINT_XFER_CONTROL, USB_CONTROL_MAX_PACKET_SIZE );

    usbDevHw_DeviceSelfPwrEnable();
    /// @todo usbDevHw_DeviceRemoteWakeupEnable();
    /// @todo usbDevHw_DeviceDmaBurstEnable()
    usbDevHw_DeviceSetDescriptorDisable();

    BCM_KTRACE( "exit: dev: ctrl=0x%x stat=0x%x  irq mask: dev=0x%x ep=0x%x  irq status: dev=0x%x ep=0x%x  ep0: status=0x%x\n",
                 usbDevHw_REG_P->devCtrl, usbDevHw_REG_P->devStatus,
                 usbDevHw_REG_P->devIntrMask, usbDevHw_REG_P->eptIntrMask, usbDevHw_REG_P->devIntrStatus, usbDevHw_REG_P->eptIntrStatus,
                 usbDevHw_REG_P->eptFifoOut[0].status );
   BCM_KTRACE( "exit ok\n" );
}

static void UdcOpsStartup( BCM_UDC_t *udcP )
{
    BCM_KTRACE2( "enter\n" );

    // Just enable interrupts for now. Endpoint 0 will get enabled once the speed enumeration
    // has completed. The Device DMA enable is global in scope. There's endpoint specific
    // DMA enables that will happen later.
    //
    usbDevHw_DeviceIrqEnable( usbDevHw_DEVICE_IRQ_SPEED_ENUM_DONE |
                              usbDevHw_DEVICE_IRQ_BUS_SUSPEND |
                              usbDevHw_DEVICE_IRQ_BUS_RESET |
                              usbDevHw_DEVICE_IRQ_SET_INTF |
                              usbDevHw_DEVICE_IRQ_SET_CFG
                            );
    usbDevHw_DeviceDmaEnable();

    BCM_KTRACE( "dev: ctrl=0x%x stat=0x%x  irq mask: dev=0x%x ep=0x%x  irq status: dev=0x%x ep=0x%x  ep0: status=0x%x\n",
                 usbDevHw_REG_P->devCtrl, usbDevHw_REG_P->devStatus,
                 usbDevHw_REG_P->devIntrMask, usbDevHw_REG_P->eptIntrMask, usbDevHw_REG_P->devIntrStatus, usbDevHw_REG_P->eptIntrStatus,
                 usbDevHw_REG_P->eptFifoOut[0].status );

    BCM_KTRACE( "exit ok\n" );
}

void UdcOpsShutdown( BCM_UDC_t *udcP )
{
    BCM_UDC_EP_t *udcEpP;


    BCM_KTRACE2( "enter\n" );

    usbDevHw_DeviceDmaDisable();
    usbDevHw_DeviceIrqDisable( usbDevHw_DEVICE_IRQ_ALL );
    usbDevHw_DeviceIrqClear( usbDevHw_DEVICE_IRQ_ALL );

    udcP->gadget.speed = USB_SPEED_UNKNOWN;

    ReqQueueFlush( &udcP->ep[0], -ESHUTDOWN );
    udcP->ep[0].desc = NULL;
    list_for_each_entry( udcEpP, &udcP->gadget.ep_list, usb_ep.ep_list )
    {
        ReqQueueFlush( udcEpP, -ESHUTDOWN );
        udcEpP->desc = NULL;
    }

    BCM_KTRACE( "exit\n" );
}

//***************************************************************************
// Control Endpoint SETUP related routines.
//
//  CtrlEpSetupInit - Prepares for next SETUP Rx. Status indicates if STALL req'd.
//  CtrlEpSetupRx - Handle Rx of a SETUP.
//
//***************************************************************************

void CtrlEpSetupInit( BCM_UDC_EP_t *udcEpP, int status )
{
    BCM_KTRACE2( "enter\n" );

    // Re-enable transfers to the SETUP buffer, clear IN and OUT NAKs, and re-enable OUT interrupts.
    //
    udcEpP->dma.virtualAddr->setup.status = usbDevHw_REG_DMA_STATUS_BUF_HOST_READY;
    udcEpP->dirn = USB_DIR_OUT;
    udcEpP->stopped = 0;

    if ( status == ENOERROR )
    {
        // Handling of previous SETUP was OK. Just clear any NAKs.
        //
        usbDevHw_EndptNakClear( udcEpP->num, USB_DIR_OUT );
        usbDevHw_EndptNakClear( udcEpP->num, USB_DIR_IN );
    }
    else
    {
        // Handling of previous SETUP failed. Set the STALL. This will get cleared
        // when the next SETUP is rx'd.
        //
        usbDevHw_EndptStallEnable( udcEpP->num, USB_DIR_IN );
        usbDevHw_EndptStallEnable( udcEpP->num, USB_DIR_OUT );
    }

    usbDevHw_EndptDmaEnable( udcEpP->num, USB_DIR_OUT );
    usbDevHw_EndptIrqEnable( udcEpP->num, USB_DIR_OUT );

    BCM_KTRACE( "%s: exit ok, status=%d\n", udcEpP->usb_ep.name, status );
}

/// @todo this only happens in the context of an irq. Might rename IrqCtrlEpSetupRx.
///
void CtrlEpSetupRx( BCM_UDC_EP_t *udcEpP, struct usb_ctrlrequest *setup )
{
    BCM_UDC_t *udcP;
    unsigned value;
    unsigned index;
    unsigned length;
    int status;


    BCM_KTRACE2( "enter\n" );

    // For a CTRL EP, the initial stage is a SETUP in the OUT direction. The direction of the subsequent DATA (optional)
    // and STATUS stages are dependant upon the SETUP. The DWC UDC will enable NAK for both directions upon the RX
    // of a SETUP. The order in which the NAK bits get cleared is dependant upon the direction of the subsequent
    // stages.
    //
    // To start, disable interrupts for the OUT direction. The appropriate direction interrupt will get set once
    // the subsequent transfer direction is determined.
    //
    usbDevHw_EndptIrqDisable( udcEpP->num, USB_DIR_OUT );

    value = le16_to_cpu( setup->wValue );
    index = le16_to_cpu( setup->wIndex );
    length = le16_to_cpu( setup->wLength );

    // Any SETUP packets appearing here need to be handled by the gadget driver. Some SETUPs may have
    // already been silently handled and acknowledged by the DWC UDC. The exceptions to this rule are the
    // USB_REQ_SET_CONFIGURATION and USB_REQ_SET_INTERFACE, which have been only partially handled with
    // the expectation that some additional software processing is required in order to complete these requests.
    // Thus, they have not been acknowledged by the DWC UDC. There is no DATA stage for these requests.
    //

    // Set the direction of the subsequent DATA stage of a control transfer. This is an
    // optional stage. It may not exist for all control transfers. If there is a DATA
    // stage, this info is used for DMA operations for any requests received from the
    // Gadget driver.
    //
    udcEpP->dirn = setup->bRequestType & USB_DIR_MASK;
    udcP = udcEpP->udcP;

    if ( udcEpP->num != 0 )
    {
        /// @todo Make changes here if the Linux USB gadget ever supports a control endpoint other
        /// than endpoint 0. The DWC UDC supports multiple control endpoints, and this driver has
        /// been written with this in mind. To make things work, really need to change the Gadget
        /// setup() callback parameters to provide an endpoint context, or add something similar
        /// to the usb_ep structure, or possibly use a usb_request to hold a setup data packet.
        //
        status = -EOPNOTSUPP;
    }
    else
    {
        // Forward the SETUP to the gadget driver for processing. The appropriate directional
        // interrupt and NAK clear will happen when the DATA stage request is queued.

        BCM_KTRACE( "%s: SETUP %02x.%02x value=%04x index=%04x len=%04x dir=%s\n",
                    udcEpP->usb_ep.name, setup->bRequestType, setup->bRequest, value, index, length, DIRN_STR(udcEpP->dirn) );

        spin_unlock(&udcP->lock);
        status = udcP->gadget_driver->setup (&udcP->gadget, setup);
        spin_lock(&udcP->lock);
    }

    if ( status < 0 )
    {
        // Error occurred during the processing of the SETUP, so enable STALL. This condition
        // can only be cleared with the RX of another SETUP, so prepare for that event.
        //
        BCM_KWARN( "%s: SETUP %02x.%02x STALL; status=%d\n",
                    udcEpP->usb_ep.name, setup->bRequestType, setup->bRequest, status );

        CtrlEpSetupInit( udcEpP, status );
    }
    else if ( length == 0 )
    {
        // No DATA stage. Just need to prepare for the next SETUP.
        //
        CtrlEpSetupInit( udcEpP, ENOERROR );
    }
    else
    {
        // The SETUP stage processing has completed OK, and there may or may not be a request queued
        // for the DATA stage. When the DATA stage completes, preparation for the RX of the next
        // SETUP will be done.
    }

    BCM_KTRACE2( "exit\n" );
}


//***************************************************************************
// IRQ routines.
//
//  IrqUdc - top level entry point.
//  IrqDev - top level device related interrupt handler
//  IrqDevCfgSet - device (endpoint 0) set config interrupt handler
//  IrqDevIntfSet - device (endpoint 0) set interface interrupt handler
//  IrqDevSpeedEnum - device speed enumeration done interrupt handler
//  IrqEp - top level endpoint related interrupt handler
//  IrqEpInStatusCheck - top level IN endpoint related interrupt handler
//  IrqEpOutStatusCheck -  top level OUT endpoint related interrupt handler
//  IrqEpOutSetup - Control endpoint SETUP Rx handler. This may get called
//                  directly as the result of an endpoint OUT interrupt, or
//                  indirectly as the result of device SET_CFG or SET_INTF.
//
//***************************************************************************

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
irqreturn_t IrqUdc(int irq, void *context, struct pt_regs *pt_regs)
#else
irqreturn_t IrqUdc(int irq, void *context)
#endif
{
    BCM_UDC_t *udcP;
    unsigned long flags;
    uint32_t irqDev;
    uint32_t irqEpIn;
    uint32_t irqEpOut;


    BCM_KTRACE2( "enter\n" );

    /// @todo sanity check irq
    (void)irq;
    #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
        (void)pt_regs;
    #endif

    udcP = (BCM_UDC_t *)context;

    spin_lock_irqsave( &udcP->lock, flags );

    if ( !udcP || !udcP->gadget_driver )
    {
        BCM_KERROR( "invalid context or no driver registered: irq dev=0x%x\n", usbDevHw_DeviceIrqActive() );

        usbDevHw_DeviceIrqClear( usbDevHw_DEVICE_IRQ_ALL );
        usbDevHw_EndptIrqListClear( USB_DIR_IN, ~0 );
        usbDevHw_EndptIrqListClear( USB_DIR_OUT, ~0 );

        spin_unlock_irqrestore( &udcP->lock, flags );

        return( IRQ_HANDLED );
    }

    /// @todo change Active to Pending??
    /// @todo merge usbDevHw EP IN/OUT routines?? Can only have 16 endpoints max due to a USB protocol restriction.

    irqDev = usbDevHw_DeviceIrqActive();
    irqEpIn = usbDevHw_EndptIrqListActive( USB_DIR_IN );
    irqEpOut = usbDevHw_EndptIrqListActive( USB_DIR_OUT );

    usbDevHw_DeviceIrqClear( irqDev );
    usbDevHw_EndptIrqListClear( USB_DIR_IN, irqEpIn );
    usbDevHw_EndptIrqListClear( USB_DIR_OUT, irqEpOut );

    BCM_KTRACE( "dev: ctrl=0x%x stat=0x%x  irq mask: dev=0x%x ep=0x%x  irq status: dev=0x%x epIn=0x%x epOut=0x%x  ep0: status=0x%x\n",
                 usbDevHw_REG_P->devCtrl, usbDevHw_REG_P->devStatus,
                 usbDevHw_REG_P->devIntrMask, usbDevHw_REG_P->eptIntrMask, irqDev, irqEpIn, irqEpOut,
                 usbDevHw_REG_P->eptFifoOut[0].status );

    // Handle the SET_CFG and SET_INTF interrupts after the endpoint and other device interrupts.
    // There can be some race conditions where we have an endpoint 0 interrupt pending for the
    // completion of a previous endpoint 0 transfer (e.g. a GET config) when a SETUP arrives
    // corresponding to the SET_CFG and SET_INTF. Need to complete the processing of the previous
    // transfer before handling the next one, i.e. the SET_CFG or SET_INTF.
    //
    IrqDev( udcP, irqDev & ~(usbDevHw_DEVICE_IRQ_SET_CFG | usbDevHw_DEVICE_IRQ_SET_INTF) );
    IrqEp( udcP, irqEpIn, irqEpOut );
    IrqDev( udcP, irqDev & (usbDevHw_DEVICE_IRQ_SET_CFG | usbDevHw_DEVICE_IRQ_SET_INTF) );

    spin_unlock_irqrestore( &udcP->lock, flags );

    BCM_KTRACE( "irq mask: dev=0x%x ep=0x%x: ep0 ctrl in=0x%x out=0x%x   dev: ctrl=0x%x stat=0x%x\n",
                usbDevHw_REG_P->devIntrMask, usbDevHw_REG_P->eptIntrMask,
                usbDevHw_REG_P->eptFifoIn[0].ctrl, usbDevHw_REG_P->eptFifoOut[0].ctrl,
                usbDevHw_REG_P->devCtrl, usbDevHw_REG_P->devStatus );
    BCM_KTRACE2( "exit\n" );

    return( (irqDev || irqEpIn || irqEpOut) ? IRQ_HANDLED : IRQ_NONE );
}

void IrqDev( BCM_UDC_t *udcP, uint32_t irq )
{
    if ( irq & usbDevHw_DEVICE_IRQ_BUS_RESET )
    {
        BCM_KINFO( "BUS reset\n" );
        ///  @todo: add support for reset
    }

    if ( irq & usbDevHw_DEVICE_IRQ_BUS_SUSPEND )
    {
        BCM_KTRACE( "BUS suspend\n" );
        ///  @todo: add support for suspend
    }

    if ( irq & usbDevHw_DEVICE_IRQ_BUS_IDLE )
    {
        BCM_KTRACE("BUS idle\n" );
        ///  @todo: add support for 3ms idle if needed.
    }

    if ( irq & usbDevHw_DEVICE_IRQ_SPEED_ENUM_DONE )
    {
        BCM_KTRACE2( "BUS speed enum done\n" );
        IrqDevSpeedEnum( udcP );
    }

    if ( irq & usbDevHw_DEVICE_IRQ_SET_CFG )
    {
        BCM_KTRACE2( "SET CFG\n" );
        IrqDevCfgSet( udcP );
    }

    if ( irq & usbDevHw_DEVICE_IRQ_SET_INTF )
    {
        BCM_KTRACE2( "SET INTF\n" );
        IrqDevIntfSet( udcP );
    }
}

void IrqDevCfgSet( BCM_UDC_t *udcP )
{
    struct usb_ctrlrequest setup;
    unsigned epNum;
    uint16_t cfg;


    BCM_KTRACE2( "enter\n" );


    // Device Configuration SETUP has been received. This is not placed in the SETUP
    // DMA buffer. The packet has to be re-created here so it can be forwarded to the
    // gadget driver to act upon.
    //
    cfg = (uint16_t) usbDevHw_DeviceCfgNum();

    setup.bRequestType = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
    setup.bRequest = USB_REQ_SET_CONFIGURATION;
    setup.wValue = cpu_to_le16(cfg);
    setup.wIndex = 0;
    setup.wLength = 0;

    // Setting the configuration number before the gadget responds is a bit presumptious, but should
    // not be fatal.
    /// @todo Do not set endpoint 0? Or is it a don't care?
    //
    for ( epNum = 0; epNum < BCM_UDC_EP_CNT; epNum++)
    {
        usbDevHw_EndptCfgSet( epNum, cfg );
    }

    BCM_KINFO( "SET CFG=%d\n", cfg );

    CtrlEpSetupRx( &udcP->ep[0], &setup );
    usbDevHw_DeviceSetupDone();

    BCM_KTRACE2( "exit\n" );
}

void IrqDevIntfSet( BCM_UDC_t *udcP )
{
    struct usb_ctrlrequest setup;
    unsigned epNum;
    uint16_t intf;
    uint16_t alt;


    BCM_KTRACE2( "enter\n" );

    // Device Interface SETUP has been received. This is not placed in the SETUP
    // DMA buffer. The packet has to be re-created here so it can be forwarded to the
    // gadget driver to act upon.
    //
    intf = (uint16_t) usbDevHw_DeviceIntfNum();
    alt =  (uint16_t) usbDevHw_DeviceAltNum();

    setup.bRequestType = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE;
    setup.bRequest = USB_REQ_SET_INTERFACE;
    setup.wValue = cpu_to_le16(alt);
    setup.wIndex = cpu_to_le16(intf);
    setup.wLength = 0;

    // Setting the interface numbers before the gadget responds is a bit presumptious, but should
    // not be fatal.
    /// @todo Do not set endpoint 0? Or is it a don't care?
    //
    for ( epNum = 0; epNum < BCM_UDC_EP_CNT; epNum++)
    {
        usbDevHw_EndptAltSet( epNum, alt );
        usbDevHw_EndptIntfSet( epNum, intf );
    }

    BCM_KINFO( "SET INTF=%d ALT=%d\n", intf, alt );

    CtrlEpSetupRx( &udcP->ep[0], &setup );
    usbDevHw_DeviceSetupDone();

    BCM_KTRACE2( "exit\n" );
}

void IrqDevSpeedEnum( BCM_UDC_t *udcP )
{
    unsigned prevSpeed;


    BCM_KTRACE2( "enter\n" );

    prevSpeed = udcP->gadget.speed;

    switch( usbDevHw_DeviceSpeedEnumerated() )
    {
        case usbDevHw_DEVICE_SPEED_HIGH:

            BCM_KINFO( "HIGH SPEED\n" );
            udcP->gadget.speed = USB_SPEED_HIGH;
            break;

        case usbDevHw_DEVICE_SPEED_FULL:

            BCM_KINFO( "FULL SPEED\n" );
            udcP->gadget.speed = USB_SPEED_FULL;
            break;

        case usbDevHw_DEVICE_SPEED_LOW:

            BCM_KWARN( "low speed not supported\n" );
            udcP->gadget.speed = USB_SPEED_LOW;
            break;

        default:

            BCM_KERROR( "unknown speed=0x%x\n", usbDevHw_DeviceSpeedEnumerated() );
            break;
    }

    if ( (prevSpeed == USB_SPEED_UNKNOWN) && (udcP->gadget.speed != USB_SPEED_UNKNOWN) )
    {
        // Speed has not been enumerated before, so now we can initialize transfers on endpoint 0.
        // Also have to disable the NAKs at a global level, which has been in place while waiting
        // for enumeration to complete.
        //
        BCM_KTRACE( "dev status=0x%08x: ep0 IN status=0x%08x OUT status=0x%08x\n",
                    usbDevHw_REG_P->devStatus, usbDevHw_REG_P->eptFifoIn[0].status, usbDevHw_REG_P->eptFifoOut[0].status );
        CtrlEpSetupInit( &udcP->ep[0], ENOERROR );
        usbDevHw_DeviceNakAllOutEptDisable();
    }

    BCM_KTRACE2( "exit\n" );
}

void IrqEp( BCM_UDC_t *udcP, uint32_t irqIn, uint32_t irqOut )
{
    uint32_t mask;
    unsigned num;


    mask = 1;
    for ( num = 0; num < BCM_UDC_EP_CNT; num++ )
    {
        if ( irqIn & mask )
        {
            IrqEpInStatusCheck( &udcP->ep[num] );
        }
        if ( irqOut & mask )
        {
            IrqEpOutStatusCheck( &udcP->ep[num] );
        }
        mask <<= 1;
    }

}

void IrqEpInStatusCheck( BCM_UDC_EP_t *udcEpP )
{
    uint32_t status;


    status = usbDevHw_EndptStatusActive( udcEpP->num, USB_DIR_IN );
    usbDevHw_EndptStatusClear( udcEpP->num, USB_DIR_IN, status );

    BCM_KTRACE( "%s: status=0x%x\n", udcEpP->usb_ep.name, status );

    if ( !status )
    {
        return;
    }

    /// @todo check might only be for direction...
    if ( (udcEpP->dirn != USB_DIR_IN) && (udcEpP->type != USB_ENDPOINT_XFER_CONTROL) )
    {
        BCM_KERROR( "%s: unexpected IN interrupt\n", udcEpP->usb_ep.name );
        return;
    }

    if ( udcEpP->dirn != USB_DIR_IN )
    {
        // This probably should not be happening
        BCM_KWARN( "%s: CTRL dirn OUT\n", udcEpP->usb_ep.name );
    }

    if ( status & usbDevHw_ENDPT_STATUS_IN_TOKEN_RX )
    {
        // If there's any IN requests, the DMA should be setup and ready to go. Nothing to do here.
        //
        /// @todo may need to use this event to start DMA, especially with 2820
        status &= ~usbDevHw_ENDPT_STATUS_IN_TOKEN_RX;
    }

    if ( status & usbDevHw_ENDPT_STATUS_IN_DMA_DONE )
    {
        // DMA has completed, but cannot start next transfer until usbDevHw_ENDPT_STATUS_IN_XFER_DONE.
        // To avoid race conditions and other issues, do not release the current transfer until then.
        //
        status &= ~usbDevHw_ENDPT_STATUS_IN_DMA_DONE;
    }

    if ( status & (usbDevHw_ENDPT_STATUS_IN_XFER_DONE | usbDevHw_ENDPT_STATUS_DMA_BUF_NOT_AVAIL) )
    {
        status &= ~(usbDevHw_ENDPT_STATUS_IN_XFER_DONE | usbDevHw_ENDPT_STATUS_DMA_BUF_NOT_AVAIL);
        ReqXferProcess( udcEpP );
    }

    if ( status & usbDevHw_ENDPT_STATUS_DMA_ERROR )
    {
        status &= ~usbDevHw_ENDPT_STATUS_DMA_ERROR;
        ReqXferError(udcEpP, -EIO);
    }

    if ( status )
    {
        BCM_KERROR( "%s: unknown status=0x%x\n", udcEpP->usb_ep.name, status );
    }
}

void IrqEpOutStatusCheck( BCM_UDC_EP_t *udcEpP )
{
    uint32_t status;


    status = usbDevHw_EndptStatusActive( udcEpP->num, USB_DIR_OUT );
    usbDevHw_EndptStatusClear( udcEpP->num, USB_DIR_OUT, status );

    BCM_KTRACE( "%s: status=0x%x\n", udcEpP->usb_ep.name, status );

    // Remove the Rx packet size field from the status. The datasheet states this field is not used
    // in DMA mode, but that is not true.
    //
    status &= usbDevHw_ENDPT_STATUS_ALL;

    if ( !status )
    {
        return;
    }

    if ( (udcEpP->dirn != USB_DIR_OUT) && (udcEpP->type != USB_ENDPOINT_XFER_CONTROL) )
    {
        BCM_KERROR( "%s: unexpected OUT interrupt\n", udcEpP->usb_ep.name );
        return;
    }

    if ( udcEpP->dirn != USB_DIR_OUT )
    {
        // This probably should not be happening
        BCM_KWARN( "%s: CTRL dirn IN\n", udcEpP->usb_ep.name );
    }

    if ( status & usbDevHw_ENDPT_STATUS_OUT_DMA_DATA_DONE )
    {
        status &= ~usbDevHw_ENDPT_STATUS_OUT_DMA_DATA_DONE;
        ReqXferProcess( udcEpP );
    }

    if ( status & usbDevHw_ENDPT_STATUS_OUT_DMA_SETUP_DONE )
    {
        status &= ~usbDevHw_ENDPT_STATUS_OUT_DMA_SETUP_DONE;
        IrqEpOutSetup(udcEpP);
    }

    if ( status & usbDevHw_ENDPT_STATUS_DMA_BUF_NOT_AVAIL )
    {
        /// @todo Verify under what situations this can happen. Should be when chain has emptied but last desc not reached
        /// @todo status for desc updates
        status &= ~usbDevHw_ENDPT_STATUS_DMA_BUF_NOT_AVAIL;
        ReqXferProcess( udcEpP );
    }

    if ( status & usbDevHw_ENDPT_STATUS_DMA_ERROR )
    {
        status &= ~usbDevHw_ENDPT_STATUS_DMA_ERROR;
        ///@todo merge XferError and XferProcess??
        ReqXferError(udcEpP, -EIO);
    }

    if ( status )
    {
        BCM_KERROR( "%s: unknown status=0x%x\n", udcEpP->usb_ep.name, status );
    }
}

void IrqEpOutSetup( BCM_UDC_EP_t *udcEpP )
{
    BCM_UDC_DMA_SETUP_t *dmaP;


    dmaP = &udcEpP->dma.virtualAddr->setup;

    if ( (dmaP->status & usbDevHw_REG_DMA_STATUS_BUF_MASK) != usbDevHw_REG_DMA_STATUS_BUF_DMA_DONE )
    {
        BCM_KERROR( "%s: unexpected DMA buf status=0x%x\n", udcEpP->usb_ep.name, (dmaP->status & usbDevHw_REG_DMA_STATUS_BUF_MASK) );
        /// @todo Make this use of DmaDumpEp() a BCM_KTRACE or DEBUG dependant??
        DmaDumpEp( udcEpP );
        CtrlEpSetupInit( udcEpP, ENOERROR );
    }
    else if ( (dmaP->status & usbDevHw_REG_DMA_STATUS_RX_MASK) != usbDevHw_REG_DMA_STATUS_RX_SUCCESS )
    {
        BCM_KERROR( "%s: unexpected DMA rx status=0x%x\n", udcEpP->usb_ep.name, (dmaP->status & usbDevHw_REG_DMA_STATUS_RX_MASK) );
        /// @todo Make this use of DmaDumpEp() a BCM_KTRACE or DEBUG dependant??
        DmaDumpEp( udcEpP );
        CtrlEpSetupInit( udcEpP, ENOERROR );
    }
    else
    {
        if ( udcEpP->num != 0 )
        {
            /// @todo Handle the cfg / intf / alt fields of the DMA status. This will only be any issue
            /// once the Linux Gadget driver framework supports control transfers on an endpoint other
            /// than 0.
            //
            BCM_KWARN( "%s: CTRL xfr support not complete\n", udcEpP->usb_ep.name );
        }
        // Take ownership of the descriptor while processing the request. Ownership will be released
        // when ready to Rx SETUP again.
        //
        dmaP->status = (dmaP->status & ~usbDevHw_REG_DMA_STATUS_BUF_MASK) | usbDevHw_REG_DMA_STATUS_BUF_HOST_BUSY;
        CtrlEpSetupRx( udcEpP, (struct usb_ctrlrequest *)&dmaP->data1 );
    }
}

//***************************************************************************
// UDC Endpoint routines.
//
//  UdcEpInit - Initialize endpoint structures
//  UdcEpCfg - Sets endpoint configuration in preparation for usage.
//
//***************************************************************************

static int UdcEpCfg( BCM_UDC_EP_t *udcEpP, unsigned type, unsigned maxPktSize )
{
    BCM_KTRACE2( "enter\n" );
    BCM_KTRACE( "%s: type=%u dirn=0x%x pkt=%u\n", udcEpP->usb_ep.name, type, udcEpP->dirn, maxPktSize );

    if ( UdcFifoRamAlloc( udcEpP, maxPktSize ) != ENOERROR )
    {
        return( -ENOSPC );
    }

    udcEpP->type = type;
    udcEpP->usb_ep.maxpacket = maxPktSize;
    usbDevHw_EndptOpsInit( udcEpP->num, udcEpP->type, udcEpP->dirn, maxPktSize );

    BCM_KTRACE( "%s: type=%u maxPktSize=%u\n", udcEpP->usb_ep.name, type, maxPktSize );

    BCM_KTRACE2( "exit\n" );

    return( ENOERROR );
}

static void UdcEpInit( BCM_UDC_t *udcP, unsigned num, const char *name, unsigned dirn )
{
    BCM_UDC_EP_t *udcEpP;


    BCM_KTRACE2( "%s: enter: num=%u dir=%s\n", name, num, DIRN_STR(dirn) );

    udcEpP = &udcP->ep[num];

    // Initialize the endpoint attribute / control structure. Note that the UDC max packet
    // size is an indication of the hardware capabilities, not what is necessarily
    // configured and used by the endpoint. In order to provide the most flexibility on
    // how the endpoints are used, this is set to the maximum possible. When the Linux
    // Gadget usb_ep_autoconfig() looks for a suitable endpoint, it *may* check to ensure
    // the max size is adequate. There may or may not be enough FIFO RAM left to support an
    // endpoint configuration, even though the max size indicates otherwise, due to FIFO RAM
    // consumption by other endpoints. If this condition exists, an error will be returned
    // when the gadget driver tries to enable the endpoint. It is felt that doing things in
    // this manner is much easier than trying to predict and accomodate all the endpoint
    // usage scenarios by various gadget drivers, both existing and yet to be developed.
    //
    udcEpP->udcP = udcP;
    udcEpP->num = num;
    udcEpP->dirn = dirn;
    udcEpP->bEndpointAddress = num | dirn;
    udcEpP->maxPktSize = BCM_UDC_EP_MAX_PKT_SIZE;
    udcEpP->usb_ep.maxpacket = udcEpP->maxPktSize;
    udcEpP->stopped = 0;
    INIT_LIST_HEAD(&udcEpP->listQueue);

    udcEpP->usb_ep.name = name;
    udcEpP->usb_ep.ops = &bcm_udc_gadgetEpOps;
    INIT_LIST_HEAD(&udcEpP->usb_ep.ep_list);

    DmaEpInit( udcEpP );

    BCM_KTRACE( "%s: exit\n", name );
}

//***************************************************************************
// UDC FIFO RAM management routines.
//
//  The are two FIFO RAMs, one for IN and one for OUT. Each is shared amongst
//  the endpoints and is dynamically allocated. In order to handle any excess
//  allocation issues, we need to keep track of consumption. These are used
//  as part of the Gadget endpoint enable / disable operations.
//
//  UdcFifoRamInit - Initializes the space available for allocation.
//  UdcFifoRamAlloc - Allocates space for endpoint.
//  UdcFifoRamFree - Fress space used by endpoint.
//
//***************************************************************************

static void UdcFifoRamInit( BCM_UDC_t *udcP )
{
    udcP->rxFifoSpace = BCM_UDC_OUT_RX_FIFO_MEM_SIZE;
    udcP->txFifoSpace = BCM_UDC_IN_TX_FIFO_MEM_SIZE;
}

static int UdcFifoRamAlloc( BCM_UDC_EP_t *udcEpP, unsigned maxPktSize )
{
    unsigned rxCnt;
    unsigned txCnt;


    #define EP_DIRN_TYPE(d,t)   (((d) << 8) | (t))

    /// @todo Move this FIFO space requirement calculation to CSP?
    switch ( EP_DIRN_TYPE(udcEpP->dirn, udcEpP->type) )
    {
        case EP_DIRN_TYPE(USB_DIR_OUT, USB_ENDPOINT_XFER_BULK):
        case EP_DIRN_TYPE(USB_DIR_OUT, USB_ENDPOINT_XFER_INT):
        case EP_DIRN_TYPE(USB_DIR_OUT, USB_ENDPOINT_XFER_ISOC):

            rxCnt = usbDevHw_FIFO_SIZE_UINT8(maxPktSize);
            txCnt = 0;
            break;

        case EP_DIRN_TYPE(USB_DIR_IN, USB_ENDPOINT_XFER_BULK):
        case EP_DIRN_TYPE(USB_DIR_IN, USB_ENDPOINT_XFER_INT):

            rxCnt = 0;
            txCnt = usbDevHw_FIFO_SIZE_UINT8(maxPktSize);
            break;

        case EP_DIRN_TYPE(USB_DIR_IN, USB_ENDPOINT_XFER_ISOC):

            // DWC UDC does double buffering for IN ISOC
            rxCnt = 0;
            txCnt = 2 * usbDevHw_FIFO_SIZE_UINT8(maxPktSize);
            break;

        case EP_DIRN_TYPE(USB_DIR_IN,  USB_ENDPOINT_XFER_CONTROL):
        case EP_DIRN_TYPE(USB_DIR_OUT, USB_ENDPOINT_XFER_CONTROL):

            rxCnt = usbDevHw_FIFO_SIZE_UINT8(maxPktSize);
            txCnt = rxCnt;
            break;

        default:

            BCM_KERROR(  "%s: invalid EP attributes\n", udcEpP->usb_ep.name );
            return( -ENODEV );
    }

    BCM_KTRACE( "rx req=%u free=%u: tx req=%u free=%u\n", rxCnt, udcEpP->udcP->rxFifoSpace, txCnt, udcEpP->udcP->txFifoSpace );

    /// @todo change FifoSpace to uint32 units??
    if ( (udcEpP->udcP->rxFifoSpace < rxCnt) || (udcEpP->udcP->txFifoSpace < txCnt) )
    {
        return( -ENOSPC );
    }

    udcEpP->rxFifoSize = rxCnt;
    udcEpP->txFifoSize = txCnt;

#if usbDevHw_REG_MULTI_RX_FIFO
    udcEpP->udcP->rxFifoSpace -= rxCnt;
#endif
    udcEpP->udcP->txFifoSpace -= txCnt;

    return( ENOERROR );
}

static void UdcFifoRamFree( BCM_UDC_EP_t *udcEpP )
{
#if usbDevHw_REG_MULTI_RX_FIFO
    udcEpP->udcP->rxFifoSpace += udcEpP->rxFifoSize;
#endif
    udcEpP->udcP->txFifoSpace += udcEpP->txFifoSize;

    udcEpP->rxFifoSize = 0;
    udcEpP->txFifoSize = 0;
}

//***************************************************************************
// Endpoint request operations
//***************************************************************************

void  ReqQueueFlush( BCM_UDC_EP_t *udcEpP, int status )
{
    BCM_UDC_EP_REQ_t *udcEpReqP;


    BCM_KTRACE2( "enter\n" );
    BCM_KTRACE( "%s\n", udcEpP->usb_ep.name );

    udcEpP->stopped = 1;
    usbDevHw_EndptOpsFinis( udcEpP->num );

    while ( !list_empty( &udcEpP->listQueue ) )
    {
        udcEpReqP = list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode );
        ReqXferDone( udcEpP, udcEpReqP, status );
    }
    udcEpP->dma.usb_req = NULL;

    BCM_KTRACE2( "exit\n" );
}

void ReqXferError( BCM_UDC_EP_t *udcEpP, int status )
{
    BCM_KTRACE2( "enter\n" );
    BCM_KTRACE( "%s\n", udcEpP->usb_ep.name );

    if ( !udcEpP->dma.usb_req )
    {
        BCM_KERROR( "%s: No request being transferred\n", udcEpP->usb_ep.name );
        return;
    }

    /// @todo abort current DMA, start next transfer if there is one.
    udcEpP->dma.usb_req->status = status;
    ReqXferProcess( udcEpP );

    BCM_KTRACE2( "exit\n" );
}

void ReqXferDone( BCM_UDC_EP_t *udcEpP, BCM_UDC_EP_REQ_t *udcEpReqP, int status)
{
    unsigned stopped;


    BCM_KTRACE2( "enter\n" );

    list_del_init( &udcEpReqP->listNode );

    if ( udcEpReqP->usb_req.status == -EINPROGRESS )
    {
        udcEpReqP->usb_req.status = status;
    }

    if ( udcEpReqP->dmaAligned )
    {
        udcEpReqP->dmaAligned = 0;
    }
    else if ( udcEpReqP->dmaMapped )
    {
        // A physical address was not provided for the DMA buffer. Release any resources
        // that were requested by the driver.
        //
        udcEpReqP->dmaMapped = 0;
        udcEpReqP->usb_req.dma = DMA_ADDR_INVALID;
        dma_unmap_single( udcEpP->udcP->gadget.dev.parent, udcEpReqP->usb_req.dma, udcEpReqP->usb_req.length,
                            (udcEpP->dirn == USB_DIR_IN ? DMA_TO_DEVICE : DMA_FROM_DEVICE) );
    }

    BCM_KTRACE_REQ( "%s: ready: req=0x%p buf=0x%p actual=%d\n", udcEpP->usb_ep.name, &udcEpReqP->usb_req, udcEpReqP->usb_req.buf, udcEpReqP->usb_req.actual );

    // Disable DMA operations during completion callback. The callback may cause requests to be
    // added to the queue, but we don't want to change the state of the queue head.
    //
    stopped = udcEpP->stopped;
    udcEpP->stopped = 1;
    spin_unlock( &udcEpP->udcP->lock );
    udcEpReqP->usb_req.complete( &udcEpP->usb_ep, &udcEpReqP->usb_req );
    spin_lock( &udcEpP->udcP->lock );
    udcEpP->stopped = stopped;

    /// @todo May not have valid access to request any longer it has been freed...
    BCM_KTRACE( "%s: complete: req=0x%p buf=0x%p\n", udcEpP->usb_ep.name, &udcEpReqP->usb_req, udcEpReqP->usb_req.buf );
    BCM_KTRACE2( "exit\n" );
}

void ReqXferProcess( BCM_UDC_EP_t *udcEpP )
{
    BCM_KTRACE2( "enter\n" );
    BCM_KTRACE( "%s\n", udcEpP->usb_ep.name );


    /// @todo Current transfer is always the queue head. Do we need a separate pointer? Maybe just a pointer to usb_request
    if ( !udcEpP->dma.usb_req )
    {
        BCM_KERROR( "%s: No request being transferred\n", udcEpP->usb_ep.name );
        return;
    }

    usbDevHw_EndptDmaDisable( udcEpP->num, udcEpP->dirn );
    DmaDataRemoveDone( udcEpP );

    if ( udcEpP->dma.usb_req->status != -EINPROGRESS )
    {
        // Current transfer stage has finished. This may or may not be with error.
        // Complete the transfer as needed before starting the next one, if any.
        //
        DmaDataFinis( udcEpP );

        if ( (udcEpP->type == USB_ENDPOINT_XFER_CONTROL) && (udcEpP->dirn == USB_DIR_IN) && (udcEpP->dma.usb_req->status == ENOERROR) )
        {
            // For the status phase of control IN transfers, the hardware requires that an OUT DMA transfer
            // actually takes place. This should be just an OUT ZLP, and we will re-use the IN buffer that
            // just completed transfer for this purpose. There should be no harm in doing this, even if the
            // OUT status is more than a ZLP.
            //
            udcEpP->dirn = USB_DIR_OUT;
            DmaDataInit( udcEpP );
        }
        else
        {
            // All transfer stages have completed. Return the request to the gadget driver, and then
            // setup for the next transfer.
            //
            ReqXferDone( udcEpP, list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode ), ENOERROR );

            if ( udcEpP->type == USB_ENDPOINT_XFER_CONTROL )
            {
                CtrlEpSetupInit( udcEpP, ENOERROR );
            }

            if ( list_empty( &udcEpP->listQueue ) )
            {
                udcEpP->dma.usb_req = NULL;
            }
            else
            {
                udcEpP->dma.usb_req = &(list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode ))->usb_req;
                DmaDataInit( udcEpP );
            }
        }
    }


    if ( udcEpP->dma.usb_req != NULL )
    {
        DmaDataAddReady( udcEpP );
        usbDevHw_EndptNakClear( udcEpP->num, udcEpP->dirn );
        usbDevHw_EndptDmaEnable( udcEpP->num, udcEpP->dirn );
    }

    BCM_KTRACE2( "exit\n" );
}

void ReqXferStart( BCM_UDC_EP_t *udcEpP )
{
    BCM_KTRACE2( "enter\n" );
    BCM_KTRACE( "%s: %s\n", udcEpP->usb_ep.name, DIRN_STR(udcEpP->dirn) );

    /// @todo Is this necessary?? Stopped happens as a result of a halt, complete(), dequeue(), nuke().
    /// nuke() is called when ep disabled, during setup processing, and by udc_queisce(). The latter is
    /// called during vbus state change (cable insert/remove), USB reset interrupt, and gadget deregister.
    if ( udcEpP->stopped )
    {
        BCM_KTRACE( "exit: stopped\n" );
        return;
    }

    /// @todo Current transfer is always the queue head. Do we need a separate pointer? Maybe just a pointer to usb_request
    /// need to know if the queue head has already been loaded. Maybe that's the point of the "stopped".
    if ( udcEpP->dma.usb_req )
    {
        BCM_KTRACE( "%s: busy\n", udcEpP->usb_ep.name );
    }
    else if ( !list_empty( &udcEpP->listQueue ) )
    {
        udcEpP->dma.usb_req = &(list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode ))->usb_req;
        DmaDataInit( udcEpP );
        DmaDataAddReady( udcEpP );
        usbDevHw_EndptNakClear( udcEpP->num, udcEpP->dirn );
        usbDevHw_EndptDmaEnable( udcEpP->num, udcEpP->dirn );
    }

    BCM_KTRACE2( "exit\n" );
}

//***************************************************************************
// Linux proc file system functions
//***************************************************************************

#ifdef CONFIG_USB_GADGET_DEBUG_FILES

#include <linux/seq_file.h>

static const char bcm_udc_procFileName[] = "driver/" BCM_UDC_NAME;

static int ProcFileShow(struct seq_file *s, void *_)
{
    return( 0 );
}

static int ProcFileOpen(struct inode *inode, struct file *file)
{
    return( single_open(file, ProcFileShow, NULL) );
}

static struct file_operations bcm_udc_procFileOps =
{
    .open       = ProcFileOpen,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static void ProcFileCreate(void)
{
    struct proc_dir_entry *pde;

    pde = create_proc_entry (bcm_udc_procFileName, 0, NULL);
    if (pde)
    {
        pde->proc_fops = &bcm_udc_procFileOps;
    }
}

static void ProcFileRemove(void)
{
    remove_proc_entry(bcm_udc_procFileName, NULL);
}

#else

static void ProcFileCreate(void) {}
static void ProcFileRemove(void) {}

#endif
