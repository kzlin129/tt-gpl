/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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
*  @file    vc.h
*
*  @brief   This file provides the public API to the VC02 driver.
*
*****************************************************************************/


#if !defined( LINUX_VC_H )
#define LINUX_VC_H   ///< To prevent multiple includes of vc.h

/* ---- Include Files ---------------------------------------------------- */

#if defined( __KERNEL__ ) || defined( BUILDING_BOOTLOADER )
#   include <linux/types.h>
#else
#   include <inttypes.h>
#endif

#include <linux/ioctl.h>

/* ---- Constants and Types ---------------------------------------------- */

#define VC_GENCMD_MAX_LENGTH            256  ///< Maximum length of a gencmd (matches GENCMD_MAX_LENGTH in vcgencmd.c)
#define VC_GENCMD_RESPONSE_MAX_LENGTH  1024  ///< Maximum length of response returned by gencmd

#define NO_VC02_BOOT       0                 ///< Conditional compilation flag.  When set certain VC02-related operations are omitted

#define VC_MAX_VC02_CORE_FREQ   150          ///< Maximum VC02 frequeny (MHz)
#define VC_MIN_VC02_CORE_FREQ   5            ///< Minimum VC02 frequeny (MHz)

/** @addtogroup ioctlIface */
/** @{ */

/**
*  @brief   Data structure used with #VC_IOCTL_GENCMD
*
*  In order to issue a gencmd to the VC02 the user fills in the
*  cmd field of this structure and calls #VC_IOCTL_GENCMD.  This
*  is a blocking call and the response status from the VC02 is
*  returned in the err field with the response string in the str
*  field.
*/
typedef struct
{
    char    cmd[ VC_GENCMD_MAX_LENGTH ];              ///< The user puts the gencmd string in here

    struct
    {
        int     err;                                  ///< Response status
        char    str[ VC_GENCMD_RESPONSE_MAX_LENGTH ]; ///< Response string

    }   response;                                     ///< Structure containing the response from the VC02

} VC_GenCmd_t;

/**
*  @brief   Data structure used with #VC_IOCTL_HOST_READ,
*           #VC_IOCTL_HOST_READ_SWAPPED, #VC_IOCTL_HOST_WRITE
*           and #VC_IOCTL_HOST_WRITE_SWAPPED
*
*  In order to allow read/write of data from/to the VC02 from
*  user-mode, the user fills in this structure and calls the
*  relevant VC_IOCTL_HOST_... command.  The SWAPPED ioctls flip
*  the order of the bytes in words.
*
*  @note    The VC_IOCTL_HOST_... commands primarily exist to
*           allow the Fuse user-mode file system to interface
*           with the Host-VC02 driver and thus access the SD
*           card connected to the VC02.
*/
typedef struct
{
    uint32_t    vc_addr;      ///< Ptr to the VC02 data (cast to an uint32_t)
    void       *buffer;       ///< Ptr to the host buffer where the VC02 data should be read from/written to
    int         numBytes;     ///< Number of bytes of data to read/write
    int         channel;      ///< The Host-VC02 channel that should be read/written
    int         result;       ///< The result of the read/write operation MORE

} VC_HostIO_t;

/**
*  @brief   Data structure used with #VC_IOCTL_INTF_REG_EVENT_INT
*
*  In order register (from user-mode) with the Host-VC02
*  interface interrupt handler an event to be signalled when a
*  vc-to-host interrupt occurs, the user fills in this structure
*  and calls the VC_IOCTL_INTF_REG_EVENT_INT command.
*
*  @note    The VC_IOCTL_INTF_REG_EVENT_INT command primarily
*           exists to allow the Fuse user-mode file system to
*           interface with the Host-VC02 driver and thus access
*           the SD card connected to the VC02.
*/
typedef struct
{
    void       *event;     ///< Ptr to event object
    int         mask;      ///< Bits in this mask specify the services for which signalling on interrupt is required
    int         result;    ///< The result of the ioctl call

} VC_RegEventInt_t;

/**
*  @brief   Data structure used with #VC_IOCTL_LOCK_CREATE,
*           #VC_IOCTL_LOCK_OBTAIN and #VC_IOCTL_LOCK_RELEASE
*
*  In order to access the Host-VC02 interface lock support from
*  user-mode, the user fills in this structure and calls the
*  relevant VC_IOCTL_LOCK_... command.
*
*  @note    The VC_IOCTL_LOCK_... commands primarily exist to
*           allow the Fuse user-mode file system to interface
*           with the Host-VC02 driver and thus access the SD
*           card connected to the VC02.
*/
typedef struct
{
    void       *lock;      ///< Ptr to the lock object

    const char *fileName;  ///< Ptr to file name in which ioctl call is made
    int         lineNum;   ///< Line number in file at which ioctl call is made

} VC_Lock_t;

/**
*  @brief   Data structure used with #VC_IOCTL_EVENT_CREATE,
*           #VC_IOCTL_EVENT_WAIT, #VC_IOCTL_EVENT_STATUS,
*           #VC_IOCTL_EVENT_CLEAR and #VC_IOCTL_EVENT_SET
*
*  In order to access driver event support from user-mode, the
*  user fills in this structure and calls the relevant
*  VC_IOCTL_EVENT_... command.
*
*  @note    The VC_IOCTL_EVENT_... commands primarily exist to
*           allow the Fuse user-mode file system to interface
*           with the Host-VC02 driver and thus access the SD
*           card connected to the VC02.
*/
typedef struct
{
    void       *event;     ///< Ptr to the event object
    int         status;    ///< The event status (1 if set, 0 if clear)

    const char *fileName;  ///< Ptr to file name in which ioctl call is made
    int         lineNum;   ///< Line number in file at which ioctl call is made

} VC_Event_t;

/**
*  @brief   Data structure used with #VC_IOCTL_BOOT_VC02
*
*  In order to boot the VC02 to run some specified firmware the
*  user needs to fill in this structure and then call
*  #VC_IOCTL_BOOT_VC02  Note that the firmware is assumed to be
*  in 16-bit words.
*/
typedef struct
{
   void    *firmware;   ///< Ptr to firmware to load
   size_t   len;        ///< Length of firmware (in bytes)

} VC_Boot_t;

/**
*  @brief   Data structure used with #VC_IOCTL_START_AUDIO and
*           #VC_IOCTL_STOP_AUDIO
*
*  In order to set up the routing of audio samples to/from a
*  VC02 stream and a Host audio mixer port the user needs to
*  fill in this structure then call the relevant ioctl.
*/
typedef struct
{
   int record;          ///< 0 for playback, 1 for record
   int record_freq;     ///< Sampling frequency for record samples (one of the defined quantities in halaudio_mixer.h)
   int stream_num;      ///< The Host-VC02 stream number to which audio samples must be routed
   int audioMixerPort_left;   ///< The audio mixer port to which audio samples must be routed (left channel) in stereo playout
   int audioMixerPort_right;  ///< The audio mixer port to which audio samples must be routed (right channel) in stereo playout

} VC_Audio_t;

/**
*  @brief   Data structure used with #VC_IOCTL_REDIRECT_AUDIO
*
*  In order to change the Host audio mixer port of a
*  currently-configured audio routing the user needs to fill in
*  this structure and then call #VC_IOCTL_REDIRECT_AUDIO
*/
typedef struct
{
   int currMixerPort_left;   ///< The current Host audio mixer port via which audio is being routed
   int currMixerPort_right;  ///< The current Host audio mixer port via which audio is being routed
   int newMixerPort_left;    ///< The new Host audio mixer port
   int newMixerPort_right;   ///< The new Host audio mixer port

} VC_Audio_reDirect_t;

/**
*  @brief   Data structure used with #VC_IOCTL_HOST_REG
*
*  The #VC_IOCTL_HOST_REG command, which is intended for low-level debugging
*  of the physical Host-VC02 interface, allows values to be read from/written
*  to the Host interface registers on the VC02.  The operation to be performed
*  (read or write) and the register to be targeted are specified by the data
*  in this structure.
*
*  @note    The #VC_IOCTL_HOST_REG command is only intended for low-level
*           debugging of the physical Host-VC02 inteface.  As it offers no
*           protection against misuse it should really only be used when
*           no code is running on the VC02 (if it is used during normal
*           operation it will almost certainly mess up the Host-VC02 interface
*           to the extent that a reboot will be required to return to normal
*           functionality.
*/
typedef struct
{
   int op;	   ///< Specifies the operation required (0 for read, 1 for write)
   int reg;	   ///< Specifies the register to be read/written (0-7)
   int val;	   ///< Specifies the 16-bit value to be written to the register

} VC_HostReg_t;


// VC03 IOCTLs

/*
 *  MSGIO IOCTLS
 */

#if 0 // no longer in use.
typedef struct
{
  int inum;
  void *host_addr;
  int nbytes;
  unsigned int *event;
} msgio_write_blocking_t;

typedef struct
{
  int inum;
  int cmd_code;
  int ext_length;
  void *data;
  unsigned int *event;
} msgio_send_cmd_blocking_t;

typedef struct
{
  int inum;
  void *host_addr;
  int nbytes;
  unsigned int *event;
} msgio_read_blocking_t;

typedef struct
{
  int inum;
  void *header;
  unsigned int *event;
} msgio_read_header_t;

#else

typedef struct
{
  void     *handle;
  void     *data;
  uint32_t data_size;
  uint32_t flags;
  void     *msg_handle;
} msg_queue_t;

typedef struct
{
  void     *handle;
  void     *vector;
  uint32_t count;
  uint32_t flags;
  void     *msg_handle;
} msg_queuev_t;

typedef struct
{
  void     *handle;
  void     *data;
  uint32_t max_data_size_to_read;
  uint32_t actual_msg_size;
} msg_dequeue_t;

typedef struct
{
  void     *handle;
  void     *data_dst;
  uint32_t data_size;
  uint32_t flags;
  void     *bulk_handle;
} bulk_queue_receive_t;

typedef struct
{
  void     *handle;
  void     *data_src;
  uint32_t data_size;
  uint32_t flags;
  void     *bulk_handle;
} bulk_queue_transmit_t;
#endif


typedef struct msg_buffer
{
   struct msg_buffer *next_buffer; // Member of kernel driver's buffer list.
   unsigned int	total_size;	// Size of this kernel memory block (bytes).
   unsigned int	reserved;	// Reserved for use by the kernel driver.
   unsigned int	data_size;	// Number of data bytes to follow.
   unsigned int msg_data[0];	// Start of data area.

} msg_buffer_t;

/*
 *  OMX IOCTLs
 */
typedef struct
{
  int func;
  void *data;
  int len;
  void *resp;
  int resplen;
} vcilcs_func_out_t;

typedef struct
{
  void* host_addr;
  void* vc_addr;
  int   len;
  int channel;
} vc_hostreq_memaccess;

/*
 *  OMX memory management
 */
typedef struct
{
  int   len;    // length
  void* addr;   // user addr
  void* kaddr;  // virt. kernel addr
  int   key;    // majic number
} vc_malloc_t;


#define VC_ILCSINB_MAX 320

typedef struct
{
  int cmd;
  uint32_t xid;
  unsigned char msg[VC_ILCSINB_MAX];
  int msglen;
  char resp[VC_ILCSINB_MAX];
  int  resplen;
} __attribute__ ((packed)) vc_ilcsinb_t;


#define VC_HOSTREG_OP_READ	   0  ///< #VC_HostReg_t op field value for read
#define VC_HOSTREG_OP_WRITE	   1  ///< #VC_HostReg_t op field value for write

#define VC_MAGIC	'V'   ///< ioctl cmd type for Host-VC02 driver cmds

#define VC_CMD_FIRST                0x80     ///< First ioctl cmd number

#define VC_CMD_INIT                 0x80     ///< Cmd number for #VC_IOCTL_INIT
#define VC_CMD_DUMP                 0x81     ///< Cmd number for #VC_IOCTL_DUMP
#define VC_CMD_GENCMD               0x82     ///< Cmd number for #VC_IOCTL_GENCMD
#define VC_CMD_START_FRAME_LOOPBACK 0x83     ///< Cmd number for #VC_IOCTL_START_FRAME_LOOPBACK
#define VC_CMD_STOP_FRAME_LOOPBACK  0x84     ///< Cmd number for #VC_IOCTL_STOP_FRAME_LOOPBACK
#define VC_CMD_INIT_GPIO            0x85     ///< Cmd number for #VC_IOCTL_INIT_GPIO
#define VC_CMD_RUN                  0x86     ///< Cmd number for #VC_IOCTL_RUN
#define VC_CMD_HIB                  0x87     ///< Cmd number for #VC_IOCTL_HIB
#define VC_CMD_RESET                0x88     ///< Cmd number for #VC_IOCTL_RESET
#define VC_CMD_BOOT                 0x89     ///< Cmd number for #VC_IOCTL_BOOT
#define VC_CMD_GET_STC              0x8c     ///< Cmd number for #VC_IOCTL_GET_STC
#define VC_CMD_LOCK_CREATE          0x8d     ///< Cmd number for #VC_IOCTL_LOCK_CREATE
#define VC_CMD_LOCK_OBTAIN          0x8e     ///< Cmd number for #VC_IOCTL_LOCK_OBTAIN
#define VC_CMD_LOCK_RELEASE         0x8f     ///< Cmd number for #VC_IOCTL_LOCK_RELEASE
#define VC_CMD_EVENT_CREATE         0x90     ///< Cmd number for #VC_IOCTL_EVENT_CREATE
#define VC_CMD_EVENT_WAIT           0x91     ///< Cmd number for #VC_IOCTL_EVENT_WAIT
#define VC_CMD_EVENT_STATUS         0x92     ///< Cmd number for #VC_IOCTL_EVENT_STATUS
#define VC_CMD_EVENT_CLEAR          0x93     ///< Cmd number for #VC_IOCTL_EVENT_CLEAR
#define VC_CMD_EVENT_SET            0x94     ///< Cmd number for #VC_IOCTL_EVENT_SET
#define VC_CMD_HOST_READ            0x95     ///< Cmd number for #VC_IOCTL_HOST_READ
#define VC_CMD_HOST_READ_SWAPPED    0x96     ///< Cmd number for #VC_IOCTL_HOST_READ_SWAPPED
#define VC_CMD_HOST_WRITE           0x97     ///< Cmd number for #VC_IOCTL_HOST_WRITE
#define VC_CMD_HOST_WRITE_SWAPPED   0x98     ///< Cmd number for #VC_IOCTL_HOST_WRITE_SWAPPED
#define VC_CMD_HOST_SEND_INT        0x99     ///< Cmd number for #VC_IOCTL_HOST_SEND_INT
#define VC_CMD_INTF_SEND_INT        0x9A     ///< Cmd number for #VC_IOCTL_INTF_SEND_INT
#define VC_CMD_INTF_REG_EVENT_INT   0x9B     ///< Cmd number for #VC_IOCTL_INTF_REG_EVENT_INT
#define VC_CMD_SLEEP_MODE           0x9C     ///< Cmd number for #VC_IOCTL_SLEEP_MODE
#define VC_CMD_BOOT_VC02            0x9D     ///< Cmd number for #VC_IOCTL_BOOT_VC02
#define VC_CMD_INIT_SDRAM           0x9E     ///< Cmd number for #VC_IOCTL_INIT_SDRAM
#define VC_CMD_INIT_GENCMD          0x9F     ///< Cmd number for #VC_IOCTL_INIT_GENCMD
#define VC_CMD_START_AUDIO          0xA0     ///< Cmd number for #VC_IOCTL_START_AUDIO
#define VC_CMD_STOP_AUDIO           0xA1     ///< Cmd number for #VC_IOCTL_STOP_AUDIO
#define VC_CMD_SET_GMTOFF           0xA2     ///< Cmd number for #VC_IOCTL_SET_GMTOFF
#define VC_CMD_REDIRECT_AUDIO       0xA3     ///< Cmd number for #VC_IOCTL_REDIRECT_AUDIO
#define VC_CMD_HOST_REG             0xA4     ///< Cmd number for #VC_IOCTL_HOST_REG

#define VC_CMD_ILCS_EXECUTE_FUNC    0xA5
#define VC_CMD_ILCS_COMPONENT_LOCK  0xA6
#define VC_CMD_HOSTREQ_WRITEMEM     0xA7
#define VC_CMD_MALLOC               0xA8
#define VC_CMD_FREE                 0xA9
#define VC_CMD_ILCSINB              0xAA

#if 0 // for msgfifo interface, no longer in use.
#define VC_CMD_MSGIO_INIT              0xAB
#define VC_CMD_MSGIO_WRITE_BLOCKING    0xAC
#define VC_CMD_MSGIO_SEND_CMD_BLOCKING 0xAD
#define VC_CMD_MSGIO_WRITE_FLUSH       0xAE
#define VC_CMD_MSGIO_INPUT_BYTES_AVAIL 0xAF
#define VC_CMD_MSGIO_READ_BLOCKING     0xB0
#define VC_CMD_MSGIO_READ_HEADER       0xB1
#define VC_CMD_MSGIO_READ_FLUSH        0xB2
#else // use these for vchi interface
#define VC_CMD_RESERVED0               0xAB	// XXX temporary, for compatibility with older stuff
#define VC_CMD_MSG_QUEUE               0xAC
#define VC_CMD_MSG_QUEUEV              0xBA
#define VC_CMD_MSG_DEQUEUE             0xAD
#define VC_CMD_BULK_QUEUE_RECEIVE      0xAE
#define VC_CMD_BULK_QUEUE_TRANSMIT     0xAF
#define VC_CMD_RESERVED1               0xB0	// XXX temporary, for compatibility with older stuff
#define VC_CMD_RESERVED2               0xB1
#define VC_CMD_RESERVED3               0xB2
#endif
#define VC_CMD_INIT_SERVICE            0xB3
#define VC_CMD_CREATE_GFX_RESOURCE     0xB4
#define VC_CMD_DESTROY_GFX_RESOURCE    0xB5
#define VC_CMD_WRITE_GFX_MSG           0xB6
#define VC_CMD_ALLOC_BUFFER            0xB7
#define VC_CMD_FREE_BUFFER             0xB8
#define VC_CMD_ILCSINBRESP             0xB9


#define VC_CMD_LAST                    0xBA     ///< Last ioctl cmd number

#define VC_IOCTL_INIT                  _IO(   VC_MAGIC, VC_CMD_INIT )                                 ///< Initialize the Host VC02 interface (does not take any data)
#define VC_IOCTL_DUMP                  _IO(   VC_MAGIC, VC_CMD_DUMP )                                 ///< Print some information about the VC02 interface to the console (does not take any data)
#define VC_IOCTL_GENCMD                _IOWR( VC_MAGIC, VC_CMD_GENCMD, VC_GenCmd_t )                  ///< Send a gencmd to the VC02.  See #VC_GenCmd_t for more details.
#define VC_IOCTL_START_FRAME_LOOPBACK  _IO(   VC_MAGIC, VC_CMD_START_FRAME_LOOPBACK )                 ///< Loop frames received from the VC02 encoder back to the VC02 decoder.  Data is an int specifying the decoder stream number.
#define VC_IOCTL_STOP_FRAME_LOOPBACK   _IO(   VC_MAGIC, VC_CMD_STOP_FRAME_LOOPBACK )                  ///< Stop frame loopback operation (does not take any data)
#define VC_IOCTL_INIT_GPIO             _IO(   VC_MAGIC, VC_CMD_INIT_GPIO )                            ///< Initialize the VC02 GPIO pins (does not take any data)
#define VC_IOCTL_RUN                   _IO(   VC_MAGIC, VC_CMD_RUN )                                  ///< Set the state of the VC02 run pin.  Data is an int specifying the pin state (0 or 1).
#define VC_IOCTL_HIB                   _IO(   VC_MAGIC, VC_CMD_HIB )                                  ///< Set the state of the VC02 hibernate pin.  Data is an int specifying the pin state (0 or 1).
#define VC_IOCTL_RESET                 _IO(   VC_MAGIC, VC_CMD_RESET )                                ///< Set the state of the VC02 reset pin.  Data is an int specifying the pin state (0 or 1).
#define VC_IOCTL_BOOT                  _IO(   VC_MAGIC, VC_CMD_BOOT )                                 ///< Load the default image into the VC02 and run it (does not take any data)
#define VC_IOCTL_GET_STC               _IO(   VC_MAGIC, VC_CMD_GET_STC )                              ///< Read the VC02 STC register and print it to the console (does not take any data)
#define VC_IOCTL_LOCK_CREATE           _IOWR( VC_MAGIC, VC_CMD_LOCK_CREATE, VC_Lock_t )               ///< Create a lock.  See #VC_Lock_t for more details.
#define VC_IOCTL_LOCK_OBTAIN           _IOW(  VC_MAGIC, VC_CMD_LOCK_OBTAIN, VC_Lock_t )               ///< Obtain a lock.  See #VC_Lock_t for more details.
#define VC_IOCTL_LOCK_RELEASE          _IOW(  VC_MAGIC, VC_CMD_LOCK_RELEASE, VC_Lock_t )              ///< Release a lock.  See #VC_Lock_t for more details.
#define VC_IOCTL_EVENT_CREATE          _IOWR( VC_MAGIC, VC_CMD_EVENT_CREATE, VC_Event_t )             ///< Create an event.  See #VC_Event_t for more details.
#define VC_IOCTL_EVENT_WAIT            _IOW(  VC_MAGIC, VC_CMD_EVENT_WAIT, VC_Event_t )               ///< Wait on an event.  See #VC_Event_t for more details.
#define VC_IOCTL_EVENT_STATUS          _IOWR( VC_MAGIC, VC_CMD_EVENT_STATUS, VC_Event_t )             ///< Get event status.  See #VC_Event_t for more details.
#define VC_IOCTL_EVENT_CLEAR           _IOW(  VC_MAGIC, VC_CMD_EVENT_CLEAR, VC_Event_t )              ///< Clear an event.  See #VC_Event_t for more details.
#define VC_IOCTL_EVENT_SET             _IOW(  VC_MAGIC, VC_CMD_EVENT_SET, VC_Event_t )                ///< Set an event.  See #VC_Event_t for more details.
#define VC_IOCTL_HOST_READ             _IOWR( VC_MAGIC, VC_CMD_HOST_READ, VC_HostIO_t )               ///< Read data from the VC02.  See #VC_HostIO_t for more details.
#define VC_IOCTL_HOST_READ_SWAPPED     _IOWR( VC_MAGIC, VC_CMD_HOST_READ_SWAPPED, VC_HostIO_t )       ///< Read data (with byte swap) from the VC02.  See #VC_HostIO_t for more details.
#define VC_IOCTL_HOST_WRITE            _IOWR( VC_MAGIC, VC_CMD_HOST_WRITE, VC_HostIO_t )              ///< Write data to the VC02.  See #VC_HostIO_t for more details.
#define VC_IOCTL_HOST_WRITE_SWAPPED    _IOWR( VC_MAGIC, VC_CMD_HOST_WRITE_SWAPPED, VC_HostIO_t )      ///< Write data (with byte swap) to the VC02.  See #VC_HostIO_t for more details.
#define VC_IOCTL_HOST_SEND_INT         _IO(   VC_MAGIC, VC_CMD_HOST_SEND_INT )                        ///< Send the VC02 an interrupt.  Data is an int specifying the Host-VC02 channnel.
#define VC_IOCTL_INTF_SEND_INT         _IO(   VC_MAGIC, VC_CMD_INTF_SEND_INT )                        ///< Request an interrupt from the specified services.  Data is an int whose set bits specify the services.
#define VC_IOCTL_INTF_REG_EVENT_INT    _IOWR( VC_MAGIC, VC_CMD_INTF_REG_EVENT_INT, VC_RegEventInt_t ) ///< Request an event from the specified services.  See #VC_RegEventInt_t for more details.
#define VC_IOCTL_SLEEP_MODE            _IO(   VC_MAGIC, VC_CMD_SLEEP_MODE )                           ///< Change the VC02 clock frequency.  Data is an int (1 for sleep, 0 otherwise)
#define VC_IOCTL_BOOT_VC02             _IOW(  VC_MAGIC, VC_CMD_BOOT_VC02, VC_Lock_t )                 ///< Load a specified image into the VC02 and run it.  See #VC_Boot_t for more details.
#define VC_IOCTL_INIT_SDRAM            _IO(   VC_MAGIC, VC_CMD_INIT_SDRAM )                           ///< Initialize the VC02 external SDRAM (does not take any data)
#define VC_IOCTL_INIT_GENCMD           _IO(   VC_MAGIC, VC_CMD_INIT_GENCMD )                          ///< Initialize the VC02 gencmd interface (does not take any data)
#define VC_IOCTL_START_AUDIO           _IOWR(   VC_MAGIC, VC_CMD_START_AUDIO, VC_Audio_t )            ///< Set up an audio connection between the VC02 and a Host mixer port.  See #VC_Audio_t for more details.
#define VC_IOCTL_STOP_AUDIO            _IOWR(   VC_MAGIC, VC_CMD_STOP_AUDIO, VC_Audio_t )             ///< Tear down the audio connection between the VC02 and a Host mixer port.  See #VC_Audio_t for more details.
#define VC_IOCTL_SET_GMTOFF            _IO(   VC_MAGIC, VC_CMD_SET_GMTOFF )                           ///< Set the GMT offset for VC02 timestamps.  Data is an int specifying the offset (in secs).
#define VC_IOCTL_REDIRECT_AUDIO        _IOWR(   VC_MAGIC, VC_CMD_REDIRECT_AUDIO, VC_Audio_reDirect_t )///< Change the Host mixer port of the current audio connection.  See #VC_Audio_reDirect_t for more details.
#define VC_IOCTL_HOST_REG              _IOWR(   VC_MAGIC, VC_CMD_HOST_REG, VC_HostReg_t )             ///< Read from/write to VC02 host interface register.  See #VC_HostReg_t for more details.

#define VC_IOCTL_ILCS_EXECUTE_FUNC     _IOWR(VC_MAGIC, VC_CMD_ILCS_EXECUTE_FUNC, vcilcs_func_out_t)
#define VC_IOCTL_ILCS_COMPONENT_LOCK   _IOWR(VC_MAGIC, VC_CMD_ILCS_COMPONENT_LOCK, int)
#define VC_IOCTL_HOSTREQ_WRITEMEM      _IOWR(VC_MAGIC, VC_CMD_HOSTREQ_WRITEMEM, vc_hostreq_memaccess)
#define VC_IOCTL_MALLOC                _IOWR(VC_MAGIC, VC_CMD_MALLOC, vc_malloc_t)
#define VC_IOCTL_FREE                  _IO(VC_MAGIC, VC_CMD_FREE)
#define VC_IOCTL_ILCSINB               _IOWR(VC_MAGIC, VC_CMD_ILCSINB, vc_ilcsinb_t)
#define VC_IOCTL_ILCSINBRESP           _IOWR(VC_MAGIC, VC_CMD_ILCSINBRESP, vc_ilcsinb_t)

#if 0 // no longer in use.
#define VC_IOCTL_MSGIO_INIT              _IOWR(VC_MAGIC, VC_CMD_MSGIO_INIT, int)
#define VC_IOCTL_MSGIO_WRITE_BLOCKING    _IOWR(VC_MAGIC, VC_CMD_MSGIO_WRITE_BLOCKING, msgio_write_blocking_t)
#define VC_IOCTL_MSGIO_SEND_CMD_BLOCKING _IOWR(VC_MAGIC, VC_CMD_MSGIO_SEND_CMD_BLOCKING, msgio_send_cmd_blocking_t)
#define VC_IOCTL_MSGIO_WRITE_FLUSH       _IOWR(VC_MAGIC, VC_CMD_MSGIO_WRITE_FLUSH, int)
#define VC_IOCTL_MSGIO_INPUT_BYTES_AVAIL _IOWR(VC_MAGIC, VC_CMD_MSGIO_INPUT_BYTES_AVAIL, int)
#define VC_IOCTL_MSGIO_READ_BLOCKING     _IOWR(VC_MAGIC, VC_CMD_MSGIO_READ_BLOCKING, msgio_read_blocking_t)
#define VC_IOCTL_MSGIO_READ_HEADER       _IOWR(VC_MAGIC, VC_CMD_MSGIO_READ_HEADER, msgio_read_header_t)
#define VC_IOCTL_MSGIO_READ_FLUSH        _IOWR(VC_MAGIC, VC_CMD_MSGIO_READ_FLUSH, int)
#else
#define VC_IOCTL_HI_INIT               _IOWR(VC_MAGIC, VC_CMD_HI_INIT, int)
#define VC_IOCTL_MSG_QUEUE             _IOWR(VC_MAGIC, VC_CMD_MSG_QUEUE, msg_queue_t)
#define VC_IOCTL_MSG_QUEUEV            _IOWR(VC_MAGIC, VC_CMD_MSG_QUEUEV, msg_queuev_t)
#define VC_IOCTL_MSG_DEQUEUE           _IOWR(VC_MAGIC, VC_CMD_MSG_DEQUEUE, msg_dequeue_t)
#define VC_IOCTL_BULK_QUEUE_RECEIVE    _IOWR(VC_MAGIC, VC_CMD_BULK_QUEUE_RECEIVE, bulk_queue_receive_t)
#define VC_IOCTL_BULK_QUEUE_TRANSMIT   _IOWR(VC_MAGIC, VC_CMD_BULK_QUEUE_TRANSMIT, bulk_queue_transmit_t)
#endif
#define VC_IOCTL_INIT_SERVICE          _IOWR(VC_MAGIC, VC_CMD_INIT_SERVICE, int)
#define VC_IOCTL_CREATE_GFX_RESOURCE   _IOWR(VC_MAGIC, VC_CMD_CREATE_GFX_RESOURCE, int)
#define VC_IOCTL_DESTROY_GFX_RESOURCE  _IOWR(VC_MAGIC, VC_CMD_DESTROY_GFX_RESOURCE, int)
#define VC_IOCTL_WRITE_GFX_MSG         _IO(VC_MAGIC, VC_CMD_WRITE_GFX_MSG)
#define VC_IOCTL_ALLOC_BUFFER          _IOWR(VC_MAGIC, VC_CMD_ALLOC_BUFFER, int)
#define VC_IOCTL_FREE_BUFFER           _IO(VC_MAGIC, VC_CMD_FREE_BUFFER)

/** @} */

/* ---- Variable Externs ------------------------------------------------- */

#if defined( __KERNEL__ )
#   define VC_DEBUG_ENABLED    1       ///< Debug flag for kernel mode
#else
#   define VC_DEBUG_ENABLED    1       ///< Debug flag for user mode
#endif

#if VC_DEBUG_ENABLED

extern  int gVcDebugRead;              ///< Data associated with /proc/sys/vc/debug/read
extern  int gVcDebugWrite;             ///< Data associated with /proc/sys/vc/debug/write
extern  int gVcDebugTrace;             ///< Data associated with /proc/sys/vc/debug/trace
extern  int gVcDebugInfo;              ///< Data associated with /proc/sys/vc/debug/info
extern  int gVcDebugMsgFifo;           ///< Data associated with /proc/sys/vc/debug/msgfifo
extern  int gVcDebugFrameLoopback;     ///< Data associated with /proc/sys/vc/debug/frame-loopback
extern  int gVcDebugHostIrq;           ///< Data associated with /proc/sys/vc/debug/host-irq
extern  int gVcDebugVerbose;           ///< Data associated with /proc/sys/vc/debug/verbose
extern  int gVcDebugDisplayRegion;     ///< Data associated with /proc/sys/vc/debug/display-region
extern  int gVcDebugVchiTrace;         ///< Data associated with /proc/sys/vc/debug/vchi-trace
extern  int gVcDebugVchiTraceData;     ///< Data associated with /proc/sys/vc/debug/vchi-trace-data
extern  int gVcDebugRwPerf;            ///< Data associated with /proc/sys/vc/debug/rw-perf

/** @addtogroup funcIface */
/** @{ */

#   if defined( __KERNEL__ )
        /**
         *  @brief Debug print macro.  Printing is conditional based on
         *         the 'flag' argument which in turn is linked to a
         *         debug-... proc entry--see \ref procIface
         */
#       define VC_DEBUG(flag, fmt, args...) if ( gVcDebug##flag ) printk( "%s: " fmt, __FUNCTION__ , ## args ) // for kernel mode
#       define VC_DEBUG2(flag, fmt, args...) if ( gVcDebug##flag ) printk(fmt, ##args ) // for kernel mode

#       define VC_KNLLOG(flag, fmt, args...) if ( gVcDebug##flag ) KNLLOG( fmt, ## args ) // for kernel mode

#   else
        /**
         *  @brief Debug print macro.  Printing is conditional based on
         *         the 'flag' argument which in turn is linked to a
         *         debug-... proc entry--see \ref procIface
         */
#       define VC_DEBUG(flag, fmt, args...) if ( gVcDebug##flag ) printf( "%s: " fmt, __FUNCTION__ , ## args ) // for user mode
#       define VC_DEBUG2(flag, fmt, args...) if ( gVcDebug##flag ) printf(fmt, ##args ) // for user mode

#       define VC_KNLLOG(flag, fmt, args...) if ( gVcDebug##flag ) printf( fmt, ## args ) // for user mode

#   endif
#else

#   define VC_DEBUG(flag, fmt, args...)
#   define VC_DEBUG2(flag, fmt, args...)

#endif // VC_DEBUG_ENABLED


#if defined( __KERNEL__ )
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#include <linux/broadcom/vc_frmfwd_defs.h>

/**
*  @brief   Structure used to store clock data.
*
*  For video playout during a video call the Host timestamps
*  data packets sent to the VC02 with a playout time referenced
*  to the VC02 realtime clock (this enables the Host to maintain
*  synchronization between playout of the video and audio
*  streams). In order to do this the Host must have the ability
*  to synchronize its sense of playout time with the VC02 sense
*  of time.  It achieves this by making use of data in this
*  clock structure which it can access via calls to the
*  #vc_host_get_clock and #vc_host_set_clock functions.
*/
typedef struct
{
   unsigned int            dspclock;      ///< VC02 STC free-running timer value
   unsigned int            hostclock;     ///< Host free-running timer value
   unsigned int            time_s;        ///< Real time (in secs)
   unsigned int            time_frac;     ///< Real time (microseconds between secs)
   unsigned int            last_stc;      ///< VC02 STC value on last update
   unsigned int            stcfreq;       ///< Frequency of the VC02 STC (in Hz)
   unsigned int            speed;         ///< VC02 clock speed factor

} VC_Clock_t;


/**
*  @brief   Header for the frame buffer #VC_FrameBuffer_t used
*           to pass data between the Host-VC02 interface and the
*           user
*/
typedef struct
{
    struct list_head         node;     ///< MORE
    FRMFWD_FRAME_INFO_T      frame;    ///< Frame header

} VC_FrameBufferHdr_t;

/**
*  @brief   Frame buffer used to pass data between the Host-VC02
*           interface and the user
*
*  See #VC_HOSTCALLBACK_T for examples of where the frame
*  buffers are used.
*/
typedef struct
{
    VC_FrameBufferHdr_t hdr;  ///< Data header
    uint8_t data[4];          ///< The actual data (the '4' is dummy as the array is actually big enough to contain all the data)

} VC_FrameBuffer_t;

/**
*  @brief   List of queued frame buffers
*/
typedef struct
{
    struct list_head    list;       ///< Frame buffer queue
    spinlock_t          lock;       ///< Spinlock
    struct semaphore    availSem;   ///< Semaphore

} VC_FrameBufferList_t;

/**
*  @brief   Structure containing ptrs to functions that need to
*           be called by the Host-VC02 interface in order to
*           transfer data frames to/from the VC02
*
*  This structure contains the data that gets passed to the
*  function #vc_host_set_cmd_callback
*/
typedef struct
{
   /**
    * @brief   Ptr to function called by the interface to get
    *     a ptr to a buffer into which it can copy data from a
    *     frame received from the VC02
    *
    * @param   stream_num        (in) The number of the stream on
    *                            which the data was received
    * @param   length_in_bytes   (in) The length of the received
    *                            data
    *
    * @return  Ptr to an allocated buffer into which received data
    *          can be copied
    */
   VC_FrameBuffer_t * (* vc_get_framebuf)(unsigned int stream_num, int length_in_bytes);

   /**
    * @brief   Ptr to function called by the interface in order to
    *          output data received from the VC02
    *
    * @param   frame          (in) Ptr to a buffer containing a
    *                         frame of data received from the VC02
    *                         (the recipient is responsible for
    *                         freeing the buffer)
    * @param   lengthinbyte   (in) The total length of the buffer
    *                         containing the frame of data received
    *                         from the VC02 (length of the data +
    *                         sizeof( FRMFWD_FRAME_INFO_T ) )
    * @param   stream_num     (in) The number of the stream on
    *                         which the data was received
    */
   void (* vc_enqueue_framebuf)(VC_FrameBuffer_t *frame, unsigned int lengthinbyte, int stream_num);

   /**
    * @brief   Ptr to function called by the interface to get a
    *          buffer containing a frame of data that needs to be
    *          sent to the VC02
    *
    * @param   stream_num  (in) The number of the stream on which
    *                      the data is to be sent
    *
    * @return  Ptr to an allocated buffer containing the frame to
    *          be sent
    */
   VC_FrameBuffer_t * (* vc_dequeue_framebuf)(int stream_num);

   /**
    * @brief Ptr to function called by the interface to free a
    *        buffer of data that has been sent to the VC02
    *
    * @param   frame (in) Ptr to buffer of data that has been sent
    *                to the VC02
    */
   void (* vc_free_framebuf)(VC_FrameBuffer_t *frame);

} VC_HOSTCALLBACK_T;

/**
*  @brief   Stuctured used to store interface global data
*/
typedef struct
{
   VC_HOSTCALLBACK_T   *callback;   ///< Ptr to structure of interface ptrs

} VC_HOSTGLOBALS_T;

/**
* @brief   Set interface callback functions
*
*  Call this function to set the callback functions that need to
*  be called by the Host-VC02 interface in order to transfer
*  data frames to/from the VC02
*
*  @param   callback (in) Ptr to structure containing ptrs to
*                    callback functions
*
*  @return  Ptr to previous structure of callback ptrs (allows
*           caller to restore previous setup)
*/
VC_HOSTCALLBACK_T *vc_host_set_cmd_callback(VC_HOSTCALLBACK_T *callback);

/**
*  @brief   Dequeue a frame buffer from the Host-to-VC02 frame
*           buffer queue and send it to the VC02.
*
*  @param   devhdl   (in) Device handle specifying the frame
*                    buffer queue
*/
void vc_sendframe(int devhdl);

/**
*  @brief   Get the clock structure.
*
*  See #VC_Clock_t for more details.
*
*  @param   clock (out) Ptr to clock data structure
*/
void vc_host_get_clock( VC_Clock_t *clock );

/**
*  @brief   Set the clock stucture.
*
*  See #VC_Clock_t for more details.
*
*  @param   clock (in) Ptr to clock data structure
*/
void vc_host_set_clock( VC_Clock_t *clock );

/**
*  @brief   Query whether the Host-VC02 interface has been
*           initialized.
*
*  @return  0 if not initialized, else 1
*/
int vc_is_initialized( void );

/**
*  @brief   Convenience function to send a command to the VC02
*           and receive the response
*
*  This function takes a format specifier and variable number of
*  arguments (similar to printf), composes the corresponding
*  gencmd, sends it to the VC02 and waits for the response which
*  is then returned to the caller.
*
*  @param   response Ptr to buffer into which response should be
*                    copied
*  @param   maxlen   Length of the response buffer
*  @param   format   Ptr to format string (as per printf)
*
*  @return  0 if successful, else not 0
*/
int vc_gencmd(char *response, int maxlen, const char *format, ...);

/**
*  @brief   Dump Host-VC02 interface status information to the
*           console
*/
void show_vc02( void );

/**
*  @brief   Dump bytes to the console
*
*  Performs a hex dump of bytes to the console.  The dump is
*  done in rows of 16 bytes.  Each row starts with a preface
*  string (specified by the "label" parameter) and ends with the
*  ASCII equivalents ('.' is used for non-printing bytes)
*
*  @param   label    Ptr to string that prefaces each row
*  @param   addr     Initial value of a incrementing address
*                    included in the row preface (use 0 if you
*                    don't know what to put here)
*  @param   mem      Ptr to start of data to dump
*  @param   numBytes Number of bytes of data to dump
*/
void vc_dump_mem( const char *label, uint32_t addr, const void *mem, size_t numBytes );

/**
*  @brief   Dump 16-bit words to the console
*
*  Similar to #vc_dump_mem although there are only 8 words per
*  row and no ASCII equilavents are printed.
*
*  @param   label    Ptr to string that prefaces each row
*  @param   addr     Initial value of a incrementing address
*                    included in the row preface (use 0 if you
*                    don't know what to put here)
*  @param   mem      Ptr to start of data to dump
*  @param   numBytes Number of bytes of data to dump
*/
void vc_dump_mem_16( const char *label, uint32_t addr, const void *mem, size_t numBytes );

/**
*  @brief   Dump 32-bit words to the console
*
*  Similar to #vc_dump_mem although there are only 4 words per
*  row and no ASCII equilavents are printed.
*
*  @param   label    Ptr to string that prefaces each row
*  @param   addr     Initial value of a incrementing address
*                    included in the row preface (use 0 if you
*                    don't know what to put here)
*  @param   mem      Ptr to start of data to dump
*  @param   numBytes Number of bytes of data to dump
*/
void vc_dump_mem_32( const char *label, uint32_t addr, const void *mem, size_t numBytes );


#endif  /* __KERNEL__ */

/** @} */

/* ---- Function Prototypes ---------------------------------------------- */

#endif  /* LINUX_VC_H */
