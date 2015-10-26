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
*  @file    vc03_frmfwd_defs.h
*
*  @brief   frame forward service enumeration & misc types & defines
*
****************************************************************************/
#ifndef BCM_VC03_FRMFWD_DEFS_H
#define BCM_VC03_FRMFWD_DEFS_H

#if defined( __KERNEL__ )
#   include <linux/types.h>
#  include <linux/list.h>
#  include <linux/spinlock.h>
#  include <linux/version.h>
#  if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26)
#     include <linux/semaphore.h>
#  else
#     include <asm/semaphore.h>
#  endif
#else
#   include <inttypes.h>
#endif
#include <linux/broadcom/omx/OMX_Video.h>



// Types and constants common between the application and the driver, video conference call related
typedef enum
{
   VC03_VIDEOCONF_ADD_DEC = 0,
   VC03_VIDEOCONF_DEL_DEC,
   VC03_VIDEOCONF_DEC_DISPLAY_CONFIG,

   VC03_VIDEOCONF_ADD_ENC,
   VC03_VIDEOCONF_DEL_ENC,
   VC03_VIDEOCONF_ENC_DISPLAY_CONFIG,

   VC03_VIDEOCONF_CMD_MAX

} VC03_VIDEOCONF_CMD_HEADER;

typedef enum
{
   VC03_DISPLAY_LCD = 0,
   VC03_DISPLAY_HDMI = 2,

} VC03_DISPLAY_DEVICE;

/* configuration parameters for stream display */
typedef struct
{
   unsigned short x_offset;
   unsigned short y_offset;
   unsigned short width;
   unsigned short height;
   int            layer;       // display layer
   VC03_DISPLAY_DEVICE  device_num;  // LCD or HDMI output

} VC03_DISPLAY_ARG;

#if 0
/* configuration parameters for decoder streams */
/* Note, currently decoding process is hardcoded to H.264 and SPS and PPS will be received inband */
typedef struct
{

} VC03_DEC_STRM_ARG;
#endif

/* configuration parameters for the encoder stream */
typedef struct
{
   OMX_VIDEO_CODINGTYPE codecType;
   unsigned int resolutionWidth;
   unsigned int resolutionHeight;
   unsigned int frameRate;
   unsigned int bitRate;

} VC03_ENC_STRM_ARG;

typedef struct
{
   OMX_VIDEO_CODINGTYPE decType;

} VC03_DEC_STRM_ARG;

typedef struct
{
   /* union of parameters being passed from the host to the VC03 for different configurations */
   union
   {
      VC03_DISPLAY_ARG display;
      VC03_ENC_STRM_ARG encParam;
      VC03_DEC_STRM_ARG decParam;

   } arg;

} VC03_VIDEOCONF_CMD_ARG;

/* command structure, command + config parameters from the host to the VC03 */
typedef struct
{
   VC03_VIDEOCONF_CMD_HEADER cmd;
   VC03_VIDEOCONF_CMD_ARG * arg;

} VC03_VIDEOCONF_CMD;

typedef struct {
   uint8_t stream_num;  // might be useful later on for multi-stream support
   uint8_t flags;       // bit 2: end_of_stream, bit 1: marker, bit 0: key_frame
   uint8_t offset;      // byte offset from end of INFO structure to start of payload
   uint8_t reserved1;
   uint16_t seq_num;
   uint16_t reserved2;
   uint32_t timestamp;
   uint32_t data_len;  // amount of video data

} VC03_FRAME_INFO_T;  // try to make it to 16 byte structure for better alignment

/**
*  @brief   Header for the frame buffer #VC03_FrameBuffer_t used
*           to pass data between the Host-VC03 interface and the
*           user
*/
typedef struct
{
    struct list_head         node;     ///< MORE
    VC03_FRAME_INFO_T        frame;    ///< Frame header

} VC03_FrameBufferHdr_t;

/**
*  @brief   Frame buffer used to pass data between the Host-VC03
*           interface and the user
*
*  See #VC03_HOSTCALLBACK_T for examples of where the frame
*  buffers are used.
*/
typedef struct
{
    VC03_FrameBufferHdr_t hdr;  ///< Data header
    uint8_t data[4];          ///< The actual data (the '4' is dummy as the array is actually big enough to contain all the data)

} VC03_FrameBuffer_t;

/**
*  @brief   List of queued frame buffers to be sent from the host to the VC03
*/
typedef struct
{
   struct list_head list;     ///< Frame buffer queue
   spinlock_t lock;           ///< Spinlock (FIXME : should use mutex instead?)
   struct semaphore availSem; ///< Semaphore

} VC03_FrameBufferList_t;

/**
*  @brief   Structure containing ptrs to functions that need to
*           be called by the Host kernel driver that handles OMX buffers
*           Network packets will be transformed into OMX buffer messages by the
*           host kernel driver and communicated to the VC03
*
*  This structure contains the data that gets passed to the
*  function #vc03_host_set_cmd_callback
*/
typedef struct
{
   /**
    * @brief   Ptr to function called by the interface to get
    *     a ptr to a buffer into which it can copy data from a
    *     frame received from the VC03
    *
    * @param   stream_num        (in) The number of the stream on
    *                            which the data was received
    * @param   length_in_bytes   (in) The length of the received
    *                            data
    *
    * @return  Ptr to an allocated buffer into which received data
    *          can be copied
    */
   VC03_FrameBuffer_t * (* vc_get_framebuf)(unsigned int stream_num, int length_in_bytes);

   /**
    * @brief   Ptr to function called by the interface in order to
    *          output data received from the VC03
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
   void (* vc_enqueue_framebuf)(VC03_FrameBuffer_t *frame, unsigned int lengthinbyte, int stream_num);
 

   /**
    * @brief   Ptr to function called by the interface to get a
    *          buffer containing a frame of data that needs to be
    *          sent to the VC03
    *
    * @param   stream_num  (in) The number of the stream on which
    *                      the data is to be sent
    *
    * @return  Ptr to an allocated buffer containing the frame to
    *          be sent
    */
   VC03_FrameBuffer_t * (* vc_dequeue_framebuf)(int stream_num);

   /**
    * @brief Ptr to function called by the interface to free a
    *        buffer of data that has been sent to the VC03
    *
    * @param   frame (in) Ptr to buffer of data that has been sent
    *                to the VC03
    */
   void (* vc_free_framebuf)(VC03_FrameBuffer_t *frame);

} VC03_HOSTCALLBACK_T;


/**
* @brief   Set interface callback functions
*
*  Call this function to set the callback functions that need to
*  be called by the Host-VC03 interface in order to transfer
*  data frames to/from the VC03
*
*  @param   callback (in) Ptr to structure containing ptrs to
*                    callback functions
*
*  @return  Ptr to previous structure of callback ptrs (allows
*           caller to restore previous setup)
*/
VC03_HOSTCALLBACK_T *vc03_host_set_cmd_callback(VC03_HOSTCALLBACK_T *callback);

/**
* @brief   send frames from the host to the VC03
*
*  Call this function to send the frames associated with the video stream handle
*  to the VC03.
*
*  @param   devhdl (in) stream handle
*
*  @return  
*/
void vc03_sendframe( int devhdl );

/**
* @brief   start the frame processing threads
*
*  Call this function to start off the packet processing threads
*
*  @param
*
*  @return  
*/
void vc03_startfrmfwd_thds( void );

/**
* @brief   vc03 commands associated with video conferencing
*
*  Call this function (with appropriate arguments) when we need to perform the
*  following tasks for video conferencing:
*   a) start a video decoder path
*   b) start a video encoder path
*   c) change the display region
*   d) etc
*
*  @param
*
*  @return  
*/
void vc03_confcmd( VC03_VIDEOCONF_CMD * cmdp );



#endif // BCM_VC03_FRMFWD_DEFS_H
