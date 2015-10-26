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
*  @file    vc_vidconftest.c
*
*  @brief   test code for video conference API
*           Note this test code is developed to verify the video conference API
*
****************************************************************************/
#include <linux/broadcom/omx/OMX_Broadcom.h>
#include <linux/broadcom/vc03/host_ilcore.h>
#include <linux/broadcom/vc03/vcilclient.h>
#include <linux/broadcom/vc03/vcilplatform.h>
#include <linux/broadcom/vc03/vcos.h>

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/vmalloc.h>
#include <linux/syscalls.h>

#include <stdarg.h>

#include <linux/broadcom/vc03/vc03_frmfwd_defs.h>
#include <linux/broadcom/knllog.h>

#define MAX_DEC_PKT_SIZE   60000 /* maximum size of the packet we can handle */


/******************************************************************************
Static data.
******************************************************************************/

static struct file *write_hdl;
static struct file *read_hdl;
int read_file = 0;

static  VC03_FrameBufferList_t   gVCTest_encFramesList;
static  VC03_FrameBufferList_t   gVCTest_decFramesList;

static int enc_quitnow = 0;
static int dec_quitnow = 0;
static int enc_saverawonly = 0;
static int enc_saveoutput = 1;

static int dec_framerate = 0;

/******************************************************************************
Static functions.
******************************************************************************/
static int dec_filereadThread( void *unused );
static int enc_filewriteThread( void *unused );

static VC03_FrameBuffer_t *allocateFrameBuffer_cb( unsigned int stream_Num, int length_in_bytes );
static void enqueueFrameBuffer_cb( VC03_FrameBuffer_t *frame, unsigned int lengthinbyte, int streamNum );
static void releaseFrameBuffer_cb( VC03_FrameBuffer_t *frame );
static VC03_FrameBuffer_t * dequeueFrameBuffer_cb( int halhdl );


void initFrameBufferList( VC03_FrameBufferList_t *fList );
VC03_FrameBuffer_t *internalDequeueFrameBuffer( VC03_FrameBufferList_t *fList );
void enqueueFrameBuffer( VC03_FrameBufferList_t *fList, VC03_FrameBuffer_t *frameBuf, int wakeup );
VC03_FrameBuffer_t *dequeueFrameBuffer( VC03_FrameBufferList_t *fList );

static VC03_HOSTCALLBACK_T gTest_HostCallBack = 
{
   allocateFrameBuffer_cb,
   enqueueFrameBuffer_cb,
   dequeueFrameBuffer_cb,
   releaseFrameBuffer_cb,
};

/*---------------------------------------------------------------------------*/
/***************************************************************************/
/**
*  start encoder test for video conference.
*
*  @param1
*
*  @return
*
*  @remarks
*    the encoder parameter are pass within a string in the following manner
*    <encoder type> <width> <height> <framerate> <bitrate> <capture file>
*
*/
void vc_start_enctest( char * test_str )
{
   VC03_VIDEOCONF_CMD cmd;
   VC03_VIDEOCONF_CMD_ARG arg;
   int framerate;
   int bitrate;
   int width;
   int height;
   char * path;
   char * path_end;
   int threadPid;
   mm_segment_t old_fs;

   /* setup a thread for writing the encoder packets to a file */
   enc_quitnow = 0;
   initFrameBufferList( &gVCTest_encFramesList );
   threadPid = kthread_run( enc_filewriteThread, NULL, "encTest" );

   /* set the list of callback for packet handling */
   vc03_host_set_cmd_callback( &gTest_HostCallBack ); 
    
   cmd.arg = &arg;

   path = test_str;

   /*determine the encoder type for this call */
   if( 0 == strncmp( path, "h264", 4 ) )
   {
      cmd.arg->arg.encParam.codecType = OMX_VIDEO_CodingAVC;
      printk("encoder type = h264\n" );
   }
   else if( 0 == strncmp( path, "h263", 4 ) )
   {
      cmd.arg->arg.encParam.codecType = OMX_VIDEO_CodingH263;
      printk("encoder type = h263\n" );
   }
   else if( 0 == strncmp( path, "mpeg4", 5 ) )
   {
      cmd.arg->arg.encParam.codecType = OMX_VIDEO_CodingMPEG4;
      printk("encoder type = mpeg4\n");
   }
   path = strchr( path, ' ');
   path++;

   enc_saveoutput = 1;
   if( 0 == strncmp( path, "raw", 3 ) )
   {
      enc_saverawonly = 1;
      path = strchr( path, ' ');
      path++;
   }
   else if( 0 == strncmp( path, "nosave", 6 ) )
   {
      enc_saveoutput = 0;
      path = strchr( path, ' ');
      path++;
   }

   if(1 != sscanf(path, "%d", &width))
   {
      width = 1280;
   }
   path = strchr( path, ' ');
   path++;

   if(1 != sscanf(path, "%d", &height))
   {
      height = 720;
   }
   path = strchr( path, ' ');
   path++;
   
   if(1 != sscanf(path, "%d", &framerate))
   {
      framerate = 30;
   }
   path = strchr( path, ' ');
   path++;
   if(1 != sscanf(path, "%d", &bitrate))
   {
      bitrate = 512000;
   }
  
   write_hdl = NULL;

   if( enc_saveoutput == 1 )
   {
      path = strchr( path, ' ');
      path++;
      path_end = path + strlen(path) - 1;
 
      if(*path_end == '\n')
         *path_end = 0;

      old_fs = get_fs();
      set_fs ( get_ds() );
      write_hdl = filp_open( path, O_TRUNC | O_WRONLY | O_CREAT, 0644 );
      set_fs( old_fs );

      if( !write_hdl || !write_hdl->f_op || !write_hdl->f_op->write )
      {
         printk("failed to open file for storing bitstream\n");
         return;
      
      }
   }

   /* set the resolution, framerate and bit rate parameters */
   cmd.arg->arg.encParam.frameRate = framerate;
   cmd.arg->arg.encParam.bitRate = bitrate;

   cmd.arg->arg.encParam.resolutionWidth = width;
   cmd.arg->arg.encParam.resolutionHeight = height;

   /* specify the command type */
   cmd.cmd = VC03_VIDEOCONF_ADD_ENC;

   printk("width %d height %d framerate %d, bitrate %d filename %s\n", 
         width, height, framerate, bitrate, path );

   /* queue the command with the video conference client */
   vc03_confcmd( &cmd );

   return;
}
/***************************************************************************/
/**
*  stop encoder test for video conference.
*
*  @param1
*
*  @return
*
*  @remarks
*
*/

void vc_stop_enctest( void )
{
   VC03_VIDEOCONF_CMD cmd;
   mm_segment_t old_fs;
   char * ptr;

   cmd.cmd = VC03_VIDEOCONF_DEL_ENC;
   /* this command needs no argument */
   cmd.arg = NULL;
   vc03_confcmd( &cmd );

   if( write_hdl != NULL )
   {
      old_fs = get_fs();
      set_fs (get_ds());
      filp_close( write_hdl, current->files );
      set_fs (old_fs);
      write_hdl =  NULL;
   }

   /* do a dummy message to quit the encoder process thread */
   enc_quitnow = 1;
   ptr = vmalloc( 10 );
   enqueueFrameBuffer( &gVCTest_encFramesList, (VC03_FrameBuffer_t *)ptr, 1 );
   
   /* un-register the set of callback for packet handling */
   vc03_host_set_cmd_callback( NULL ); 
}

/***************************************************************************/
/**
*  start decoder test for video conference.
*
*  @param1
*
*  @return
*
*  @remarks
*    the encoder parameter are pass within a string in the following manner
*    <encoder type> <width> <height> <framerate> <bitrate> <capture file>
*
*/
void vc_start_dectest( char * test_str )
{
   VC03_VIDEOCONF_CMD cmd;
   VC03_VIDEOCONF_CMD_ARG arg;
   char * path;
   char * path_end;
   mm_segment_t old_fs;
   int flags, threadPid;

   dec_quitnow = 0;

   initFrameBufferList( &gVCTest_decFramesList );

   /* set the list of callback for packet handling */
   vc03_host_set_cmd_callback( &gTest_HostCallBack ); 
    
   cmd.arg = &arg;

   path = test_str;

   /*determine the encoder type for this call */
   if( 0 == strncmp( path, "h264", 4 ) )
   {
      cmd.arg->arg.decParam.decType = OMX_VIDEO_CodingAVC;
      printk("decoder type = h264\n" );
   }
   else if( 0 == strncmp( path, "h263", 4 ) )
   {
      cmd.arg->arg.decParam.decType = OMX_VIDEO_CodingH263;
      printk("decoder type = h263\n" );
   }
   else if( 0 == strncmp( path, "mpeg4", 5 ) )
   {
      cmd.arg->arg.decParam.decType = OMX_VIDEO_CodingMPEG4;
      printk("decoder type = mpeg4\n");
   }
   path = strchr( path, ' ');
   path++;

   if(1 != sscanf(path, "%d", &dec_framerate))
   {
      dec_framerate = 30;
   }

   path = strchr( path, ' ');
   path++;
   path_end = path + strlen(path) - 1;
 
   if(*path_end == '\n')
      *path_end = 0;

   printk( "decoder framerate %d filename = %s\n", dec_framerate, path );

   read_hdl = NULL;
   old_fs = get_fs();
   set_fs ( get_ds() );
   read_hdl = filp_open( path, O_RDONLY, 0644 );
   set_fs( old_fs );

   if( !read_hdl || !read_hdl->f_op || !read_hdl->f_op->read )
   {
      printk("failed to open file for reading bitstream\n");
      return;
   }

   cmd.cmd = VC03_VIDEOCONF_ADD_DEC;
   vc03_confcmd( &cmd );

   threadPid = kthread_run( dec_filereadThread, NULL, "decTest" );
}
/***************************************************************************/
/**
*  stop decoder test for video conference.
*
*  @param1
*
*  @return
*
*  @remarks
*
*/
void vc_stop_dectest( void )
{
   VC03_VIDEOCONF_CMD cmd;
   mm_segment_t old_fs;

   dec_quitnow = 1;

   cmd.cmd = VC03_VIDEOCONF_DEL_DEC;
   /* this command needs no argument */
   cmd.arg = NULL;
   vc03_confcmd( &cmd );

   old_fs = get_fs();
   set_fs (get_ds());
   //sys_close( read_file );

           
   filp_close( read_hdl, current->files );
 
   set_fs (old_fs);
   read_hdl =  NULL;
   
   /* un-register the set of callback for packet handling */
   vc03_host_set_cmd_callback( NULL ); 
}
/***************************************************************************/
/**
*  configure the encoder camera preview window
*
*  @param1
*
*  @return
*
*  @remarks
*    the encoder parameter are pass within a string in the following manner
*    <ori_x> <ori_y> <width> <height> <z-order layer> <device number>
*
*/

void vc_set_encdisp( char * test_str )
{
   VC03_VIDEOCONF_CMD cmd;
   VC03_VIDEOCONF_CMD_ARG arg;
   int ori_x;
   int ori_y;
   int width;
   int height;
   int layer;
   int device;
   char * path;
    
   cmd.arg = &arg;

   path = test_str;
   if(1 != sscanf(path, "%d", &ori_x))
   {
      ori_x = 0;
   }
   path = strchr( path, ' ');
   path++;

   if(1 != sscanf(path, "%d", &ori_y))
   {
      ori_y = 0;
   }
   path = strchr( path, ' ');
   path++;
   if(1 != sscanf(path, "%d", &width))
   {
      width = 800;
   }
   path = strchr( path, ' ');
   path++;
   if(1 != sscanf(path, "%d", &height))
   {
      height = 480;
   }
   path = strchr( path, ' ');
   path++;
   if(1 != sscanf(path, "%d", &layer))
   {
      layer = 800;
   }
   path = strchr( path, ' ');
   path++;
   if(1 != sscanf(path, "%d", &device))
   {
      device = 0;
   }

   cmd.arg->arg.display.x_offset = ori_x;
   cmd.arg->arg.display.y_offset = ori_y;
   cmd.arg->arg.display.width = width;
   cmd.arg->arg.display.height = height;
   cmd.arg->arg.display.layer = layer;
   cmd.arg->arg.display.device_num = device;
 
   cmd.cmd = VC03_VIDEOCONF_ENC_DISPLAY_CONFIG;
   vc03_confcmd( &cmd );
   return;

}

/***************************************************************************/
/**
*  configure the encoder camera preview window
*
*  @param1
*
*  @return
*
*  @remarks
*    the encoder parameter are pass within a string in the following manner
*    <ori_x> <ori_y> <width> <height> <z-order layer> <device number>
*
*/

void vc_set_decdisp( char * test_str )
{
   VC03_VIDEOCONF_CMD cmd;
   VC03_VIDEOCONF_CMD_ARG arg;
   int ori_x;
   int ori_y;
   int width;
   int height;
   int layer;
   int device;
   char * path;
    
   cmd.arg = &arg;

   path = test_str;
   if(1 != sscanf(path, "%d", &ori_x))
   {
      ori_x = 0;
   }
   path = strchr( path, ' ');
   path++;

   if(1 != sscanf(path, "%d", &ori_y))
   {
      ori_y = 0;
   }
   path = strchr( path, ' ');
   path++;
   if(1 != sscanf(path, "%d", &width))
   {
      width = 800;
   }
   path = strchr( path, ' ');
   path++;
   if(1 != sscanf(path, "%d", &height))
   {
      height = 480;
   }
   path = strchr( path, ' ');
   path++;
   if(1 != sscanf(path, "%d", &layer))
   {
      layer = 800;
   }
   path = strchr( path, ' ');
   path++;
   if(1 != sscanf(path, "%d", &device))
   {
      device = 0;
   }

   cmd.arg->arg.display.x_offset = ori_x;
   cmd.arg->arg.display.y_offset = ori_y;
   cmd.arg->arg.display.width = width;
   cmd.arg->arg.display.height = height;
   cmd.arg->arg.display.layer = layer;
   cmd.arg->arg.display.device_num = device;
 
   cmd.cmd = VC03_VIDEOCONF_DEC_DISPLAY_CONFIG;
   vc03_confcmd( &cmd );
   return;

}

/*-------------- callback function for packet handling ------------------ */
/***************************************************************************/
/**
*  allocate a frame buffer so the encoded bits can be copied to
*
*  @param1
*
*  @return
*
*  @remarks
*
*/
static VC03_FrameBuffer_t *allocateFrameBuffer_cb( unsigned int stream_Num, int length_in_bytes )
{
   (void) stream_Num;
   char * ptr = vmalloc( length_in_bytes );
   return( VC03_FrameBuffer_t *)ptr;
}

static void enqueueFrameBuffer_cb( VC03_FrameBuffer_t *frame, unsigned int lengthinbyte, int streamNum )
{
   (void) lengthinbyte;
   (void) streamNum;
   //vfree( frame );
   enqueueFrameBuffer( &gVCTest_encFramesList, frame, 1 );
}

static void releaseFrameBuffer_cb( VC03_FrameBuffer_t *frame )
{
   vfree( frame );
}
static VC03_FrameBuffer_t * dequeueFrameBuffer_cb( int hdl )
{
   (void) hdl;
   return internalDequeueFrameBuffer( &gVCTest_decFramesList );
}

/*--------------- functions to maintain queues of frame buffers ------------------------ */
/****************************************************************************
*
*  enqueueFrameBuffer
*
*   Adds a buffer to the queue.
*
***************************************************************************/
void enqueueFrameBuffer( VC03_FrameBufferList_t *fList, VC03_FrameBuffer_t *frameBuf, int wakeup )
{
    //VC_DEBUG( Trace, "frameBuf: 0x%08x\n", (uint32_t)frameBuf );

    // Add the frame to the end of the list
    {
        unsigned long   flags;

        spin_lock_irqsave( &fList->lock, flags );

        list_add_tail( &frameBuf->hdr.node, &fList->list );

        spin_unlock_irqrestore( &fList->lock, flags );
    }

    if( wakeup )
    {
      // Wakeup anybody waiting for it.
      up( &fList->availSem );
    }

} // enqueueFrameBuffer

/****************************************************************************
*
*  VC_DequeueFrameBuffer
*
*   Removes a buffer from the queue. If the queue is empty, then it will
*   block.
*
***************************************************************************/
VC03_FrameBuffer_t *dequeueFrameBuffer( VC03_FrameBufferList_t *fList )
{
    // First of wait for a buffer to be available

    if ( down_interruptible( &fList->availSem ) != 0 )
    {
        return NULL;
    }

    return internalDequeueFrameBuffer( fList );

} // dequeueFrameBuffer


/****************************************************************************
*
*  initFrameBufferList
*
*   Initialize a list of frame buffers.
*
***************************************************************************/
void initFrameBufferList( VC03_FrameBufferList_t *fList )
{
    INIT_LIST_HEAD( &fList->list );

    spin_lock_init(  &fList->lock );
    sema_init( &fList->availSem, 0 );

} // initFrameBufferList

/****************************************************************************
*
*  internalDequeueFrameBuffer
*
*   Removes a buffer from the queue. This function assumes that the caller
*   has already downed the semaphore.
*
***************************************************************************/
VC03_FrameBuffer_t *internalDequeueFrameBuffer( VC03_FrameBufferList_t *fList )
{
    VC03_FrameBuffer_t    *frameBuf;
    VC03_FrameBufferHdr_t    *frameBufHdr;

    {
        unsigned long   flags;

        spin_lock_irqsave( &fList->lock, flags );

        if ( list_empty( &fList->list ))
        {
            frameBuf = NULL;
        }
        else
        {
            struct list_head *deqNode = fList->list.next;
            list_del_init( deqNode );

            frameBufHdr = list_entry( deqNode, VC03_FrameBufferHdr_t, node );
            frameBuf = list_entry( frameBufHdr, VC03_FrameBuffer_t, hdr );
        }
        spin_unlock_irqrestore( &fList->lock, flags );
    }

    //VC_DEBUG( Trace, "returning frameBuf: 0x%08x\n", (uint32_t)frameBuf );

    return frameBuf;

} // internalDequeueFrameBuffer


/*--------------- file read and write threads ------------------------ */
static int enc_filewriteThread( void * unused )
{
   VC03_FrameBuffer_t   *frameBuf;
   mm_segment_t old_fs;
   
   while( (frameBuf = dequeueFrameBuffer( &gVCTest_encFramesList ) ) != NULL )
   {
      if( enc_quitnow == 1 )
      {
         vfree( frameBuf );
         break;
      }
      if( write_hdl )
      {
         old_fs = get_fs();
         set_fs( get_ds() );

         /* write the size of this frame, take this line out if you just want the raw bit stream */
         /* you will need the size in case you want the decoder side of this test to work */
         if( enc_saverawonly == 0 )
         {
            write_hdl->f_op->write( write_hdl, &frameBuf->hdr.frame.data_len, sizeof(int), &write_hdl->f_pos ); 
         }
         write_hdl->f_op->write( write_hdl, frameBuf->data, frameBuf->hdr.frame.data_len, &write_hdl->f_pos ); 

         set_fs( old_fs );
      }

      vfree( frameBuf );
   }
   printk("reach the end of the enc test thread\n");
   enc_quitnow = 0;
}

static int dec_filereadThread( void * unused )
{
   VC03_FrameBuffer_t  * frameBuf;
   mm_segment_t old_fs;
   int nbytes = 0;
   int seqNum = 0;
   int framesize = 0;
   char * framesizep = vmalloc( 10 );

   while( dec_quitnow == 0 )
   {
      if( read_hdl )
      {
         framesize = 0;

         old_fs = get_fs();
         set_fs( get_ds() );

         /* write the size of this frame, take this line out if you just want the raw bit stream */
         /* you will need the size in case you want the decoder side of this test to work */
         read_hdl->f_op->read( read_hdl, (void *)&framesize, sizeof(int), &read_hdl->f_pos ); 

         KNLLOG("framesize=%d\n", framesize);
         if( framesize <= 0 )
         {
            printk("framesize is incorrect %d\n", framesize );
            set_fs(old_fs);
            break;
         }
         frameBuf = (VC03_FrameBuffer_t *)vmalloc( framesize + sizeof( VC03_FrameBufferHdr_t ) );
         
         read_hdl->f_op->read( read_hdl, frameBuf->data, framesize, &read_hdl->f_pos ); 

         set_fs( old_fs );
         /* timestamp is a don't care*/
         frameBuf->hdr.frame.seq_num = seqNum++;
         frameBuf->hdr.frame.timestamp = 0;
         frameBuf->hdr.frame.flags = 1;
         /* data length is the one that matters */
         frameBuf->hdr.frame.data_len = framesize;

         enqueueFrameBuffer( &gVCTest_decFramesList, frameBuf, 0 );

         /* tell the VC03 that there are frames available for decode */
         vc03_sendframe( 0 );
      }

      /* sleep for the amount of time requested */
      set_current_state(  TASK_INTERRUPTIBLE );
      schedule_timeout( ((HZ)/dec_framerate) );
   }
   printk("file done\n");
   
}
