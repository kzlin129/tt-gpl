/*****************************************************************************
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
/**
*
*  @file    amixer.h
*
*****************************************************************************/
#if !defined( AMIXER_H )
#define AMIXER_H

/* ---- Include Files ---------------------------------------------------- */
#if defined( __KERNEL__ )
#include <linux/types.h>               /* Needed for standard types */
#else
#include <stdint.h>
#endif

/* ---- Constants and Types ---------------------------------------------- */

/**
*  Mixer port IDs. 
*/
typedef void * AMIXER_PORT_ID;

/**
*  Audio mixer port information.
*
*  A mixer port is the conduit for ferrying samples in two directions, 
*  and thus a port provides both source and destination data info.
*/
typedef struct amixer_port_info
{
   char           name[32];            /* Name string */
   int            dst_hz;              /* Current destination sampling frequency in Hz */
   int            dst_chans;           /* Number of channels, i.e. mono = 1 */
   int            dst_bytes;           /* destination frame size in bytes */
   int            dst_cnxs;            /* Number of destination connections */
   int            src_hz;              /* Current source sampling frequency in Hz */
   int            src_chans;           /* Number of channels, i.e. mono = 1 */
   int            src_bytes;           /* Source frame size in bytes */
   int            src_cnxs;            /* Number of source connections */
}
AMIXER_PORT_INFO;

/**
*  Audio mixer high level information
*/
typedef struct amixer_info
{
   int ports;                          /* Number of registered ports */
   int cnxs;                           /* Number of connections */
   int clients;                        /* Number of clients */
}
AMIXER_INFO;

/* Connection types */
typedef enum amixer_connect_type
{
   AMIXER_CONNECT_STRAIGHT_THROUGH,    /* No interleave and de-interleave needed */
   AMIXER_CONNECT_MULTI_TO_MONO_CONV,  /* Convert multi-channel into single mono channel */
   AMIXER_CONNECT_MULTI_DEINTERLEAVE,  /* Multi-channel de-interleave needed */
   AMIXER_CONNECT_MONO_INTERLEAVE,     /* Interleave of mono channels needed */
   AMIXER_CONNECT_MONO_TO_MULT_DUP,    /* Duplicate mono channel into multi-channel output */
}
AMIXER_CONNECT_TYPE;

/**
* Mixer connection descriptor is formed by the following bit fields:
* 
*     type  - connection type (AMIXER_CONNECT_TYPE)
*     chan  - sub-channel index, when used with 
*             AMIXER_CONNECT_MULTI_DEINTERLEAVE
*             or AMIXER_CONNECT_MONO_INTERLEAVE
*           - number of channels, when used with 
*             AMIXER_CONNECT_MULTI_TO_MONO_CONV or
*             AMIXER_CONNECT_STRAIGHT_THROUGH
*
* | reserved (31-8) | chan (7-4) | type (3-0) |
*
*/
typedef uint32_t AMIXER_CONNECT_DESC;

#define AMIXER_CONNECT_TYPE_SHIFT      0
#define AMIXER_CONNECT_CHAN_SHIFT      4

#define AMIXER_CONNECT_TYPE_BMASK      0x0f
#define AMIXER_CONNECT_CHAN_BMASK      0xf0

/* Helper macros to create connection descriptors */
#define AMIXER_CREATE_DESC( type, chan ) \
   ((((type) << AMIXER_CONNECT_TYPE_SHIFT ) & AMIXER_CONNECT_TYPE_BMASK) | \
    (((chan) << AMIXER_CONNECT_CHAN_SHIFT ) & AMIXER_CONNECT_CHAN_BMASK))

/* Helper macros to parse connection descriptor */
#define AMIXER_CONNECT_GET_TYPE( d )   (((d) & AMIXER_CONNECT_TYPE_BMASK) >> AMIXER_CONNECT_TYPE_SHIFT)
#define AMIXER_CONNECT_GET_CHAN( d )   (((d) & AMIXER_CONNECT_CHAN_BMASK) >> AMIXER_CONNECT_CHAN_SHIFT)

/* The following descriptors are defined for the common connection descriptors */
#define AMIXER_CONNECT_MONO2MONO       AMIXER_CREATE_DESC( AMIXER_CONNECT_STRAIGHT_THROUGH, 1 )
#define AMIXER_CONNECT_STEREO2STEREO   AMIXER_CREATE_DESC( AMIXER_CONNECT_STRAIGHT_THROUGH, 2 )
#define AMIXER_CONNECT_STEREO2MONO     AMIXER_CREATE_DESC( AMIXER_CONNECT_MULTI_TO_MONO_CONV, 2 )
#define AMIXER_CONNECT_STEREO_SPLITL   AMIXER_CREATE_DESC( AMIXER_CONNECT_MULTI_DEINTERLEAVE, 0 /* Left is always first sub-channel */ )
#define AMIXER_CONNECT_STEREO_SPLITR   AMIXER_CREATE_DESC( AMIXER_CONNECT_MULTI_DEINTERLEAVE, 1 /* Right is always second sub-channel */ )
#define AMIXER_CONNECT_MONO2STEREOL    AMIXER_CREATE_DESC( AMIXER_CONNECT_MONO_INTERLEAVE, 0 )
#define AMIXER_CONNECT_MONO2STEREOR    AMIXER_CREATE_DESC( AMIXER_CONNECT_MONO_INTERLEAVE, 1 )
#define AMIXER_CONNECT_MONO_DUPSTEREO  AMIXER_CREATE_DESC( AMIXER_CONNECT_MONO_TO_MULT_DUP, 0 )

/* Mixer client handle */
#if defined( __KERNEL__ )
typedef void * AMIXER_HDL;
#else
typedef int    AMIXER_HDL;
#endif

#if defined( __KERNEL__ )

/**
*  Mixer port callbacks. Each port can register up to four
*  callbacks. Two callbacks are used by the mixer to obtain destination and
*  source data pointers, and the other two callbacks are used to indicate
*  when the mixer has finished its business with the pointers. All data
*  pointers point to 16-bit signed data.
*
*  Destination and source directions are relative to the port. A destination
*  pointer is used to write data into the port, where a source point is 
*  used to read data from the port.
*/
typedef struct amixer_port_cb
{
   /* Triggered when mixer needs the source data pointer */
   int16_t *(*getsrc)(
      int   bytes,                     /*<< (i) size of the buffer in bytes */
      void *privdata                   /*<< (i) user supplied data */
   );

   /* Triggered when mixer done with source pointer */
   void  (*srcdone)(
      int   bytes,                     /*<< (i) size of the buffer in bytes */
      void *privdata                   /*<< (i) user supplied data */
   );

   /* Triggered when mixer needs the destination data pointer */
   int16_t *(*getdst)(
      int   bytes,                     /*<< (i) size of the buffer in bytes */
      void *privdata                   /*<< (i) user supplied data */
   );

   /* Triggered when mixer done with destination pointer */
   void  (*dstdone)(
      int   bytes,                     /*<< (i) size of the buffer in bytes */
      void *privdata                   /*<< (i) user supplied data */
   );
} 
AMIXER_PORT_CB;

/***************************************************************************/
/**
*  Generic resampler. Provided an appropriate filter is designed, 
*  this resampler can accommodate any resampling ratio of 
*  interpfac/decimfac.
*/
typedef void (*AMIXER_RESAMP_FNC)( 
   int16_t       *insamp,           /*<< (i) Ptr to input samples */
   int16_t       *outsamp,          /*<< (o) Ptr to output samples */
   int16_t        numsamp,          /*<< (i) Number of samples to generate */
   const int16_t *filtcoeff,        /*<< (i) Ptr to filter coefficients */
   int            filtlen,          /*<< (i) Filter length */
   int            interpfac,        /*<< (i) Interpolation factor */
   int            decimfac          /*<< (i) Decimation factor */
);

typedef void (*AMIXER_ADD_FNC)( 
   int16_t       *dstp,             /*<< (o) Ptr to vector sum */
   const int16_t *src1p,            /*<< (i) Ptr to vector summand 1 */
   const int16_t *src2p,            /*<< (i) Ptr to vector summand 2 */
   int            numsamp           /*<< (i) Number of samples to add */
);

typedef void (*AMIXER_MPYQ16_FNC)( 
   int16_t       *dstp,             /*<< (o) Ptr to output samples */
   const int16_t *srcp,             /*<< (i) Ptr to input samples */
   int            numsamp,          /*<< (i) Number of samples to add */
   uint16_t       q16gain           /*<< (i) Q16 linear gain value to multiply with */
);

typedef void (*AMIXER_MACQ16_FNC)( 
   int16_t       *dstp,             /*<< (o) Ptr to output samples */
   const int16_t *srcp,             /*<< (i) Ptr to input samples */
   int            numsamp,          /*<< (i) Number of samples to add */
   uint16_t       q16gain           /*<< (i) Q16 linear gain value to multiply with */
);


#endif   /* __KERNEL__ */

/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */

/***************************************************************************/
/**
*  Allocates a client handle to allow the client to make use of the 
*  audio mixer resources.
*
*  @return
*     >= 0        On success, valid client handle
*     -ve         On general failure
*/
AMIXER_HDL amixerAllocateClient( void );

/***************************************************************************/
/**
*  Frees client handle and performs cleanup including deleting connections
*  owned by client.
*
*  @return
*     0           On success
*     -ve         On general failure
*/
int amixerFreeClient( 
   AMIXER_HDL hdl                   /*<< (i) client handle */
);

/***************************************************************************/
/**
*  Queries for port ID by name string. 
*
*  @return
*     0     On success
*     -1    Not found
*     -ve   Other errors
*/
int amixerQueryPortByName(
   AMIXER_HDL      hdl,             /*<< (i) Mixer client handle */
   const char     *name,            /*<< (i) Name of port */
   AMIXER_PORT_ID *id               /*<< (o) Ptr to store found port ID */
);

/***************************************************************************/
/**
*  Obtain audio mixer port information
*
*  @return
*     0     On success
*     -ve   Failure code
*/
int amixerGetPortInfo(
   AMIXER_HDL        hdl,           /*<< (i) Mixer client handle */
   AMIXER_PORT_ID    port,          /*<< (i) Port id */
   AMIXER_PORT_INFO *info           /*<< (o) Ptr to port info structure */
);

/***************************************************************************/
/**
*  Obtain audio mixer port information by name
*
*  @return
*     0     On success
*     -ve   Failure code
*/
static inline int amixerGetPortInfoByName(
   AMIXER_HDL        hdl,           /*<< (i) Mixer client handle */
   const char       *name,          /*<< (i) Port name string */
   AMIXER_PORT_INFO *info           /*<< (o) Ptr to port info structure */
)
{
   AMIXER_PORT_ID port;
   int            err;
   err = amixerQueryPortByName( hdl, name, &port );
   if ( err )
   {
      return err;
   }
   return amixerGetPortInfo( hdl, port, info );
}

/***************************************************************************/
/**
*  Obtain high level audio mixer information, such as the number of
*  registered users.
*
*  @return
*     0     On success
*     -ve   Failure code
*/
int amixerGetInfo(
   AMIXER_HDL     hdl,              /*<< (i) Mixer client handle */
   AMIXER_INFO   *info              /*<< (o) Ptr to info structure */
);

/***************************************************************************/
/**
*  Set connection loss to attenuate samples between source and destination
*  ports. Loss range is from 0 db to mute in 1db increments. 
*
*  @return
*     0           On success
*     -EINVAL     No such connection found
*     -ve         Other errors
*/
int amixerSetCnxLoss(
   AMIXER_HDL           hdl,        /*<< (i) client handle */
   AMIXER_PORT_ID       src_port,   /*<< (i) source port id */
   AMIXER_PORT_ID       dst_port,   /*<< (i) destination port id */
   unsigned int         db          /*<< (i) Loss amount in dB */
);

/***************************************************************************/
/**
*  Set connection loss by name string
*
*  @return
*     0           On success
*     -EINVAL     No such connection found
*     -ve         Other errors
*/
static inline int amixerSetCnxLossByName(
   AMIXER_HDL           hdl,        /*<< (i) client handle */
   const char          *src_name,   /*<< (i) source port name */
   const char          *dst_name,   /*<< (i) destination port name */
   unsigned int         db          /*<< (i) Loss amount in dB */
)
{
   AMIXER_PORT_ID src_port, dst_port;   
   int            err;
   err = amixerQueryPortByName( hdl, src_name, &src_port );
   if ( err )
   {
      return err;
   }
   err = amixerQueryPortByName( hdl, dst_name, &dst_port );
   if ( err )
   {
      return err;
   }
   return amixerSetCnxLoss( hdl, src_port, dst_port, db );
}

/***************************************************************************/
/**
*  Read connection loss amount in dB.
*
*  @return
*     0           On success
*     -EINVAL     No such connection found
*     -ve         Other errors
*/
int amixerGetCnxLoss(
   AMIXER_HDL           hdl,        /*<< (i) client handle */
   AMIXER_PORT_ID       src_port,   /*<< (i) source port id */
   AMIXER_PORT_ID       dst_port,   /*<< (i) destination port id */
   unsigned int        *db          /*<< (o) Pointer to store attenuation amount */
);

/***************************************************************************/
/**
*  Read connection loss amount in dB by name string.
*
*  @return
*     0           On success
*     -EINVAL     No such connection found
*     -ve         Other errors
*/
static inline int amixerGetCnxLossByName(
   AMIXER_HDL           hdl,        /*<< (i) client handle */
   const char          *src_name,   /*<< (i) source port name */
   const char          *dst_name,   /*<< (i) destination port name */
   unsigned int        *db          /*<< (o) Pointer to store attenuation amount */
)
{
   AMIXER_PORT_ID src_port, dst_port;   
   int            err;
   err = amixerQueryPortByName( hdl, src_name, &src_port );
   if ( err )
   {
      return err;
   }
   err = amixerQueryPortByName( hdl, dst_name, &dst_port );
   if ( err )
   {
      return err;
   }
   return amixerGetCnxLoss( hdl, src_port, dst_port, db );
}

/***************************************************************************/
/**
*  Make a simplex connection from source to destination ports.
*
*  @return
*     0           On success
*     -ve         On general failure
*/
int amixerConnect(
   AMIXER_HDL           hdl,        /*<< (i) client handle */
   AMIXER_PORT_ID       src_port,   /*<< (i) source port id */
   AMIXER_PORT_ID       dst_port,   /*<< (i) destination port id */
   AMIXER_CONNECT_DESC  desc        /*<< (i) Connection descriptor */
);

/***************************************************************************/
/**
*  Make a simplex connection from source to destination ports by name.
*
*  @return
*     0           On success
*     -ve         On general failure
*/
static inline int amixerConnectByName(
   AMIXER_HDL           hdl,        /*<< (i) client handle */
   const char          *src_name,   /*<< (i) source port name */
   const char          *dst_name,   /*<< (i) destination port name */
   AMIXER_CONNECT_DESC  desc        /*<< (i) Connection descriptor */
)
{
   AMIXER_PORT_ID src_port, dst_port;   
   int            err;
   err = amixerQueryPortByName( hdl, src_name, &src_port );
   if ( err )
   {
      return err;
   }
   err = amixerQueryPortByName( hdl, dst_name, &dst_port );
   if ( err )
   {
      return err;
   }
   return amixerConnect( hdl, src_port, dst_port, desc );
}

/***************************************************************************/
/**
*  Make a duplex connection between source and destination ports.
*
*  @return
*     0           On success
*     -ve         On general failure
*/
static inline int amixerConnectAll(
   AMIXER_HDL           hdl,        /*<< (i) Mixer client handle */
   AMIXER_PORT_ID       port1,      /*<< (i) First port id */
   AMIXER_PORT_ID       port2,      /*<< (i) Second port id */
   AMIXER_CONNECT_DESC  desc        /*<< (i) Connection descriptor */
)
{
   int err;
   err = amixerConnect( hdl, port1, port2, desc );
   if ( !err )
   {
      err = amixerConnect( hdl, port2, port1, desc );
   }
   return err;
}

/***************************************************************************/
/**
*  Make a duplex connection between source and destination ports by name.
*
*  @return
*     0           On success
*     -ve         On general failure
*/
static inline int amixerConnectAllByName(
   AMIXER_HDL           hdl,        /*<< (i) Mixer client handle */
   const char          *port1_name, /*<< (i) First port name */
   const char          *port2_name, /*<< (i) Second port name */
   AMIXER_CONNECT_DESC  desc        /*<< (i) Connection descriptor */
)
{
   AMIXER_PORT_ID port1, port2;   
   int            err;
   err = amixerQueryPortByName( hdl, port1_name, &port1 );
   if ( err )
   {
      return err;
   }
   err = amixerQueryPortByName( hdl, port2_name, &port2 );
   if ( err )
   {
      return err;
   }
   return amixerConnectAll( hdl, port1, port2, desc );
}

/***************************************************************************/
/**
*  Remove simplex connections between source and destination ports.
*
*  @return
*     0           On success
*     -ve         On general failure
*/
int amixerDisconnect(
   AMIXER_HDL           hdl,        /*<< (i) Mixer client handle */
   AMIXER_PORT_ID       src_port,   /*<< (i) source port id */
   AMIXER_PORT_ID       dst_port    /*<< (i) destination port id */
);

/***************************************************************************/
/**
*  Remove simplex connections between source and destination ports 
*  by name.
*
*  @return
*     0           On success
*     -ve         On general failure
*/
static inline int amixerDisconnectByName(
   AMIXER_HDL           hdl,        /*<< (i) Mixer client handle */
   const char          *src_name,   /*<< (i) source port name */
   const char          *dst_name    /*<< (i) destination port name */
)
{
   AMIXER_PORT_ID src_port, dst_port;   
   int            err;
   err = amixerQueryPortByName( hdl, src_name, &src_port );
   if ( err )
   {
      return err;
   }
   err = amixerQueryPortByName( hdl, dst_name, &dst_port );
   if ( err )
   {
      return err;
   }
   return amixerDisconnect( hdl, src_port, dst_port );
}

/***************************************************************************/
/**
*  Remove all connections between source and destination ports.
*
*  @return
*     0           On success
*     -ve         On general failure
*/
static inline int amixerDisconnectAll(
   AMIXER_HDL           hdl,        /*<< (i) Mixer client handle */
   AMIXER_PORT_ID       port1,      /*<< (i) First port id */
   AMIXER_PORT_ID       port2       /*<< (i) Second port id */
)
{
   int err;
   err = amixerDisconnect( hdl, port1, port2 );
   if ( !err )
   {
      err = amixerDisconnect( hdl, port2, port1 );
   }
   return err;
}

/***************************************************************************/
/**
*  Remove all connections between source and destination ports by name
*
*  @return
*     0           On success
*     -ve         On general failure
*/
static inline int amixerDisconnectAllByName(
   AMIXER_HDL           hdl,        /*<< (i) Mixer client handle */
   const char          *port1_name, /*<< (i) First port name */
   const char          *port2_name  /*<< (i) Second port name */
)
{
   AMIXER_PORT_ID port1, port2;   
   int            err;
   err = amixerQueryPortByName( hdl, port1_name, &port1 );
   if ( err )
   {
      return err;
   }
   err = amixerQueryPortByName( hdl, port2_name, &port2 );
   if ( err )
   {
      return err;
   }
   return amixerDisconnectAll( hdl, port1, port2 );
}

#if defined( __KERNEL__ )

/***************************************************************************/
/**
*  Create a mixer port.
*
*  @return
*     ptr      On success, valid pointer to a mixer port. Test with IS_ERR()
*     err      On failure, error code return. Retrieve with PTR_ERR().
*/
AMIXER_PORT_ID amixerCreatePort(
   const char       *name,          /*<< (i) Name string */
   AMIXER_PORT_CB   *cb,            /*<< (i) Callbacks */
   void             *privdata,      /*<< (i) Private data passed back to callbacks */
   int               dst_hz,        /*<< (i) Current destination sampling frequency in Hz */
   int               dst_chans,     /*<< (i) Number of channels, i.e. mono = 1 */
   int               dst_bytes,     /*<< (i) Destination frame size in bytes */
   int               src_hz,        /*<< (i) Current source sampling frequency in Hz */
   int               src_chans,     /*<< (i) Number of channels, i.e. mono = 1 */
   int               src_bytes      /*<< (i) Source frame size in bytes */
);

/***************************************************************************/
/**
*  Remove an existing mixer port. All connections to this port will also
*  be removed.
*
*  @return
*     0           On success
*     -ve         On general failure
*
*  @remarks
*     Typically ports are not removed unless the entire user is removed.
*     Although it is conceivable that the API supports dynamic creation and
*     deletion of ports, it is not practical for most applications since
*     applications will have to constantly query what ports still exists 
*     and keep track of valid port IDs. It is simpler to assume that the 
*     port IDs are 0-indexed and the IDs count up to the number of ports 
*     a mixer user has (subtract one).
*/
int amixerRemovePort(
   AMIXER_PORT_ID port              /*<< (i) Port to remove */
);

/***************************************************************************/
/**
*  Set port sampling frequency and expected frame sizes. Source and 
*  destination sampling frequencies may be different, as with their
*  frame sizes.
*
*  @return
*     0           On success
*     -ve         On general failure
*/
int amixerSetPortFreq(
   AMIXER_PORT_ID portid,           /*<< (i) source port id */
   int            dst_hz,           /*<< (i) Destination sampling frequency in Hz */
   int            dst_bytes,        /*<< (i) Destination frame size in bytes */
   int            src_hz,           /*<< (i) Source sampling frequency in Hz */
   int            src_bytes         /*<< (i) Source frame size in bytes */
);

/***************************************************************************/
/**
*  This routine indicates to the mixer the amount of time elapsed and 
*  schedules it to execute. Typically, this routine is called by HAL Audio,
*  which is clocked by hardware.
*
*/
void amixerElapsedTime( 
   int elapsed_usec                 /*<< (i) Amount of time elapsed since last call in usec */
);

void amixerSetResampler(
   AMIXER_RESAMP_FNC funcp          /*<< (i) Ptr to user supplied resampler */
);
void amixerSetAdder( 
   AMIXER_ADD_FNC funcp             /*<< (i) Ptr to user supplied adder */
);

#endif   /* __KERNEL__ */

#endif   /* AMIXER_H */


