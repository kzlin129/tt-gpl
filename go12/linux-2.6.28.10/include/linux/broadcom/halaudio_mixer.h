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




/**
*
*  @file    halaudio_mixer.h
*
*****************************************************************************/
#if !defined( HALMIXER_H )
#define HALMIXER_H

/* ---- Include Files ---------------------------------------------------- */
#include <asm/types.h>

/* ---- Constants and Types ---------------------------------------------- */

/* current users to the HAL that will register itself with mixer ports */
typedef enum
{
   HAL_MIXER_USER_HAL,
   HAL_MIXER_USER_EPT,
   HAL_MIXER_USER_EPT_SP,
   HAL_MIXER_USER_ALSA,
   HAL_MIXER_USER_VC02,
   HAL_MIXER_USER_NONE,

} HAL_MIXER_USER_TYPE;

/* signal sampling frequency supported by the HAL audio mixer */
#define HAL_MIXER_SIGTYPE_8K         0x0001  // 8kHz PCM
#define HAL_MIXER_SIGTYPE_16K        0x0002  // 16kHz PCM
#define HAL_MIXER_SIGTYPE_22K        0x0004  // 22.05kHz PCM
#define HAL_MIXER_SIGTYPE_32K        0x0008  // 32kHz PCM
#define HAL_MIXER_SIGTYPE_40K        0x0010  // 40kHz PCM
#define HAL_MIXER_SIGTYPE_44K        0x0020  // 44.1kHz PCM
#define HAL_MIXER_SIGTYPE_48K        0x0040  // 48kHz PCM

/* maximum number of ports per user type */
#define HAL_MIXER_MAXNUM_PTYPE    10

/* callback functions from the mixer users that can be called when switchboard is being serviced */
typedef struct
{
   char * (*halmixer_get_outputptr)( int portIdx, int numBytes );
   char * (*halmixer_get_inputptr)( int portIdx, int numBytes );
   void (*halmixer_inputReady)( int portIdx, int numBytes );
   void (*halmixer_outputRead)( int portIdx, int numBytes );

} HAL_MIXER_CALLBACK_T;

/* information associated with each registered port */
typedef struct
{
   HAL_MIXER_USER_TYPE usrid;
   unsigned int portIdx;      /* this port Idx it to allow multiple ports for each user */
   unsigned int input_capability;
   unsigned int output_capability;
   int stereo_partner;        /* this is the port number showing the opposite channel in stereo playout
                               * value of -1 in this field indicates that this port is not capable of stereo playout */
   HAL_MIXER_CALLBACK_T callback;

} HAL_MIXER_PORT_INFO;

/* structure used for HALMIXER_CMD_QUERYPORT command */ 
typedef struct
{
   unsigned userType;
   int      numPorts;
   short    list[HAL_MIXER_MAXNUM_PTYPE]; /* the return list containing the port number that belong to the
                                           * requested mixer type */

} HALMIXER_QueryPort_t;

/* return the information associated with a specified port */
typedef struct
{
   int portNum;
   HAL_MIXER_PORT_INFO  portInfo;

} HALMIXER_QueryPortInfo_t;

/* structure used for HALMIXER_CMD_QUERYPORT command */ 
typedef struct
{
   int src;          /* target port where output samples needs to be attenuated */
   int dB;           /* amount attenuation that needs to be applied */
   int resultingdB;  /* accumulated attenuation applied that will be returned to the caller */

} HALMIXER_Attenuate_t;

#define HALMIXER_SWBARRAY_LEN 30

/* connection type that is available today */
typedef enum
{
   HALMIXER_CONNECT_MONO,
   HALMIXER_CONNECT_STEREO_SPLIT,
   HALMIXER_CONNECT_STEREO_INTERLEAVE,

} HALMIXER_CONNECTION_TYPE;

/* structure used when the user needs to add a switchboard connection
 * this structure defines the connection to be made */
typedef struct
{
   int src;       /* the source can play out either mono / stereo (interleaved, left channel first) */
   int dst_1;     /* the left (1) and right(2) channel in a stereo connection
                   * note that in a mono connection, the user should connect to the left channel (dst_1) */
   int dst_2;     /* this field should be -1 in a mono connection */
   int duplex;    /* note that we do not expect duplex type connection in stereo case */
   HALMIXER_CONNECTION_TYPE connType;  /* connection type */

} HALMIXER_Connection_t;

/* definition of a destination group in the switchboard connection list */
#define HALMIXER_NUMDSTP_PERGP   3
typedef struct
{
   void * resampler_ptr;
   int dst_lst[HALMIXER_NUMDSTP_PERGP]; /* list containing the port numbers that belongs to this destination group */

} HALMIXER_DST_GROUP;

/* a component within the switchboard connection list */
typedef struct
{
   union
   {
      int srcPortId;
      HALMIXER_DST_GROUP  dst_grp;

   } share;
   int connType;
   int numConn;   /* when this is a dst port group, this specifies the number of dst ports in the list
                   * when this is a source port, this specifies the number of destinations it is connected to */
} HALMIXER_SWB_port;

/* halaudio mixer switchboard connection list */
typedef struct
{
   int   portsUsed; /* number of ports used in the connection list */
   HALMIXER_SWB_port  list[HALMIXER_SWBARRAY_LEN];   /* connection list */

} HALMIXER_SWB_Connection_List;


/* ---- Variable Externs ------------------------------------------------- */
/* ---- Function Prototypes ---------------------------------------------- */
/* the C model of these functions are being provided by the mixer */
/* users of the mixer can register a more efficient version of the code */
/* for example, hand assembled code of the function */
typedef void (*RESAMPFUNC)( short *insamp, short *outsamp, short numsamp,
                           short *filtcoeff, unsigned short filtlen,
                            unsigned short interpfac, unsigned short decimfac );
typedef void (*ADDERFUNC)( short *dstp, short* src1p, short *src2p, int numSamp );
typedef void (*ATTENUFUNC)( short *dstp, short * srcp, short numSamp, unsigned int gainVal );

#define HALMIXER_MAGIC              'M'

#define HALMIXER_CMD_FIRST          0x80
#define HALMIXER_CMD_QUERYPORT      0x81
#define HALMIXER_CMD_ADDCONNECT     0x82
#define HALMIXER_CMD_REMOVECONNECT  0x83
#define HALMIXER_CMD_ATTENUATE      0x84
#define HALMIXER_CMD_QUERYSWB       0x85
#define HALMIXER_CMD_QUERYPORTINFO  0x86

#define HALMIXER_CMD_LAST           0x86

#define HALMIXER_IOCTL_QUERYPORT       _IOWR( HALMIXER_MAGIC, HALMIXER_CMD_QUERYPORT, HALMIXER_QueryPort_t )
#define HALMIXER_IOCTL_ADDCONNECT      _IOWR( HALMIXER_MAGIC, HALMIXER_CMD_ADDCONNECT, HALMIXER_Connection_t )
#define HALMIXER_IOCTL_REMOVECONNECT   _IOWR( HALMIXER_MAGIC, HALMIXER_CMD_REMOVECONNECT, HALMIXER_Connection_t )
#define HALMIXER_IOCTL_ATTENUATE       _IOWR( HALMIXER_MAGIC, HALMIXER_CMD_ATTENUATE, HALMIXER_Attenuate_t )
#define HALMIXER_IOCTL_QUERYSWB        _IOWR( HALMIXER_MAGIC, HALMIXER_CMD_QUERYSWB, HALMIXER_SWB_Connection_List )
#define HALMIXER_IOCTL_QUERYPORTINFO   _IOWR( HALMIXER_MAGIC, HALMIXER_CMD_QUERYPORTINFO, HALMIXER_QueryPortInfo_t )

int halMixer_registerPort( HAL_MIXER_PORT_INFO *info  );
int halMixer_addConnect( HALMIXER_Connection_t * connectp );
void halMixer_runSwitchboard( int numBytes, short frequency );
int halMixer_removeConnect( HALMIXER_Connection_t * connectp );
int halMixer_deregisterPort( int portId, HAL_MIXER_PORT_INFO *info );

void halMixer_changeSampFreq( int id, unsigned int inFreq, unsigned int outFreq );
void halMixer_registerResampler( RESAMPFUNC funcp );
void halMixer_deregisterResampler( RESAMPFUNC funcp );
void halMixer_registerAdder( ADDERFUNC funcp );
void halMixer_deregisterAdder( ADDERFUNC funcp );
void halMixer_registerAttenuator( ATTENUFUNC funcp );
void halMixer_deregisterAttenuator( ATTENUFUNC funcp );
int halMixer_queryPort( unsigned id, short *portList );
int halMixer_queryPortInfo( int id, HAL_MIXER_PORT_INFO * info );
int halMixer_registerStereoPartner( int self, int partner );

#endif /* HALMIXER_H */


