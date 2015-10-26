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



/* ---- Include Files ---------------------------------------------------- */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
    #include <linux/usb_gadget.h>
#else
    #include <linux/usb/gadget.h>
#endif

/* ---- Public Constants and Types --------------------------------------- */

// Some unsigned number trickery for indexing into DMA descriptor chain. If the
// decriptor count is some power of 2, then we can use the mask to extract
// an index and not worry about wrap around as the unsigned variables are
// incremented. E.g. in following, IDX(0), IDX(4), IDX(8), ..., IDX(0xffffc)
// all produce the same result, i.e. 0.
//
#define BCM_UDC_EP_DMA_DESC_CNT             4
#define BCM_UDC_EP_DMA_DESC_IDX_MASK        (BCM_UDC_EP_DMA_DESC_CNT - 1)
#define BCM_UDC_EP_DMA_DESC_IDX(num)        ((num) & BCM_UDC_EP_DMA_DESC_IDX_MASK)

// Some DWC UDC DMA descriptor layout definitions. See datasheet for details.
//
typedef struct
{
    uint32_t status;
    uint32_t reserved;
    uint32_t data1;
    uint32_t data2;
}
BCM_UDC_DMA_SETUP_t;

typedef struct
{
    uint32_t status;
    uint32_t reserved;
    uint32_t bufAddr;
    uint32_t nextDescAddr;
}
BCM_UDC_DMA_DESC_t;

/// @todo Abstract the DMA descriptor layout and DMA operations in usbDevHw.h???

// Common DMA descriptor layout used for all endpoints. Only control endpoints
// need the setup descriptor, but in order to simply things it is defined for
// all. It may be possible to omit this altogether, and just use one of data
// descriptors for setup instead. The control transfer protocol should allow
// this to be done.
//
typedef struct
{
    BCM_UDC_DMA_SETUP_t setup;
    BCM_UDC_DMA_DESC_t  desc[BCM_UDC_EP_DMA_DESC_CNT];
}
BCM_UDC_EP_DMA_t;

// Structure used for DMA descriptor allocation. Not really necessary but convenient.
//
typedef struct
{
    BCM_UDC_EP_DMA_t ep[BCM_UDC_EP_CNT];
}
BCM_UDC_DMA_t;

typedef struct BCM_UDC_s BCM_UDC_t;
typedef struct BCM_UDC_EP_s BCM_UDC_EP_t;

// Structure used to hold endpoint specific information. There's one of these for
// each endpoint.
//
// The Rx/Tx FIFO sizes are used for RAM allocation purposes. Each transfer
// direction has its own RAM that is used for all the FIFOs in that direction.
// The RAM gets segmented (allocated) as each endpoint gets enabled. This dynamic
// allocation FIFO sizes gives flexibility, and does not require that an
// endpoint's size be fixed at run-time or during compilation. If there's not
// enough FIFO RAM as required by a gadget's endpoint definitions, then an
// error will occur for the enabling of any endpoints after the FIFO RAM has
// become exhausted.
//
// The DMA virtual address is used for all descriptor operations. The DMA
// physical address is for convenience (setting hardware registers, obtaining
// addresses for descriptor chaining, etc.). The DMA descriptors are not
// allocated on a per-endpoint basis. These are just pointers into the
// large block that was allocated for all endpoints.
//
struct BCM_UDC_EP_s
{
    struct usb_ep usb_ep;                           // usb_gadget.h
    const struct usb_endpoint_descriptor *desc;     // usb/ch9.h
    struct list_head listQueue;                     // active BCM_UDC_EP_REQ's for the endpoint
    BCM_UDC_t *udcP;                                // endpoint owner (UDC controller)
    unsigned num;
    unsigned dirn;                                  // USB_DIR_xxx (direction)
    unsigned type;                                  // USB_ENDPOINT_XFER_xxx
    unsigned bEndpointAddress;                      // dirn | type
    unsigned maxPktSize;
    unsigned rxFifoSize;                            // Rx FIFO ram allocated
    unsigned txFifoSize;                            // Tx FIFO ram allocated
    unsigned stopped : 1;
    struct
    {
        BCM_UDC_EP_DMA_t *virtualAddr;
        BCM_UDC_EP_DMA_t *physicalAddr;
        struct usb_request *usb_req;                // Current request being DMA'd
        /// @todo Some of the below are duplicates of usb_request elements. Use usb_request instead.
        unsigned lengthBufMax;                      // Max buffer length to use with a descriptor
        unsigned lengthDone;                        // Length of request DMA'd so far
        unsigned lengthToDo;                        // Length of request left to DMA
        unsigned addIndex;                          // descriptor chain index
        unsigned removeIndex;                       // descriptor chain index
        uint32_t bufAddr;                           // Location in request to DMA
        uint32_t status;
        void *alignedBuf;                           // Aligned buffer. Only used if usb_req buffer not aligned properly.
        dma_addr_t alignedAddr;                     // Aligned buffer physical address
        unsigned alignedLen;                        // Aligned buffer length
    }
    dma;
};

// Structure used to hold controller information. There should be one of these
// for each controller. Most likely there's only one.
//
// The Rx/Tx FIFO space are used for RAM allocation purposes. These track how
// much RAM is available for use as a FIFO. When an endpoint is enabled, these
// are check to see if there's enough RAM for a FIFO of the desired length as
// implied by the max packet size.
//
struct BCM_UDC_s
{
    struct usb_gadget gadget;                       // usb_gadget.h
    struct usb_gadget_driver *gadget_driver;        // usb_gadget.h
    struct completion *devRelease;                  // Used for coordination during device removal
    spinlock_t lock;
    unsigned irqNum;
    unsigned rxFifoSpace;
    unsigned txFifoSpace;
    BCM_UDC_EP_t ep[ BCM_UDC_EP_CNT ];
    struct
    {
        BCM_UDC_DMA_t *virtualAddr;
        BCM_UDC_DMA_t *physicalAddr;
    }
    dma;
};

// Structure used to hold an endpoint transfer request. Can be any number of
// these for an endpoint.
//
typedef struct BCM_UDC_EP_REQ_s
{
    struct usb_request usb_req;                     // usb_gadget.h
    struct list_head listNode;                      // For linking in the BCM_UDC_EP request queue
    dma_addr_t dmaAddrOrig;                         // Original buffer DMA address (physical).
    unsigned dmaMapped : 1;                         // Indicates if address mapping req'd. See usb_gadget.h
    unsigned dmaAligned : 1;                        // Indicates if buffer duplication done for alignment.
}
BCM_UDC_EP_REQ_t;

/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */
/* ---- Inline Function Definitions -------------------------------------- */
