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




/*=============================================================================

Project  :  VMCS Host Apps
Module   :  Demo Apps - Camcorder
File     :  $RCSfile: camcorder.c,v $
Revision :  $Revision: #4 $


Camcorder Host Application.
=============================================================================*/

#if defined( __KERNEL__ )

#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#else // __KERNEL__

#include <string.h>
#include <stdio.h>
#include "omxctl.h"
#endif

#include <stdarg.h>
#include <linux/broadcom/vc.h>
#include <linux/broadcom/vc03/vcilcs.h>
#include <linux/broadcom/vc03/vcilcs_intern.h>
#include <linux/broadcom/omx/omx.h>
#include "vc_fileservice_defs.h"
#include "vcilclient.h"
#include "vcilplatform.h"

#define INIT_CONTENTURI_MAXLEN 256

#if defined(__KERNEL__)
#endif

#if !defined(__KERNEL__)
#define vc_gencmd libomx_gencmd
#endif 

#define ilclient_debug_output OMX_DEBUG

/*
 *
 */

typedef struct
{
   char *format_name;
   char *filename;
   char *extension;
   int res_width;
   int res_height;
   char *video_format;
   int video_bitrate;
   char *audio_format;
   int audio_bitrate;
   int filesize;
   int framerate;
} CAMCORDER_FORMAT_T;

typedef enum {
   RECORD_IDLE,
   RECORD_STARTING,
   RECORD_RUNNING,
   RECORD_STOPPING,
   RECORD_VIEWFINDER,
   RECORD_STOPPING_VIEWFINDER,
   RECORD_INVALID,
} RECORD_RECORDSTATE_T;

typedef enum {
   PAUSE_CHANGE = 1,
   STATE_CHANGE  = 2,
   CREATE_STATE  = 4,
   REGION_CHANGE = 8,
   CAMERA_SETUP  = 0x10,
   ZOOM_CHANGE = 0x20,
   STABILISE_CHANGE = 0x40,
   DESTROY_STATE  = 0x80,
   EXIT_APP       = 0x100,
} RECORD_EVENTS_T;

#define CONTENTURI_MAXLEN 256
#define FORMAT_MAXLEN 32

typedef struct {
   OMX_PARAM_CONTENTURITYPE uri;
   OMX_U8 uri_data[CONTENTURI_MAXLEN];
} RECORD_OMX_CONTENTURI_TYPE_T;


typedef struct {

   RECORD_RECORDSTATE_T state;
   RECORD_OMX_CONTENTURI_TYPE_T uri;
   char error_msg[64];
   OMX_CONFIG_DISPLAYREGIONTYPE region;
   int mode;

   // internal state
   OMX_VIDEO_CODINGTYPE video_format;
   OMX_AUDIO_CODINGTYPE audio_format;
   int capture_width;
   int capture_height;
   int audio_bitrate;
   int video_bitrate;
   int zoom;
   int audio_stream_present;
   int video_stream_present;
   int max_files_size;
   int framerate;
   int paused;
   int components_created;
   int viewfinder;
   int stabilise;
   signed long zoom_x;
   signed long zoom_y;
   OMX_U32 camplus_id;
   char cdi_id[16];
   int cam_pools_allocated;

   // pointers to OpenMAX elements
   ILCLIENT_T *client;
   COMPONENT_T audio_record;
   COMPONENT_T audio_encode;
   COMPONENT_T camera;
   COMPONENT_T video_encode;
   COMPONENT_T write_media;
   COMPONENT_T display;
   COMPONENT_T clock;

   COMPONENT_T *list[8];
   COMPONENT_T *sub_list[8];
   COMPONENT_T *ulist[8];

   TUNNEL_T ca_di_tunnel[2];
   TUNNEL_T ca_ve_tunnel[2];
   TUNNEL_T ve_wm_tunnel[2];
   TUNNEL_T rc_ae_tunnel[2];
   TUNNEL_T ae_wm_tunnel[2];

} RECORD_STATE_T;

RECORD_STATE_T recordstate;

#define CAMCORDER_FMT_MAX  2
CAMCORDER_FORMAT_T camcorder_formats[CAMCORDER_FMT_MAX] =
  {
    //   { "H264:720p",     "/mfs/sd/MediaFiles/camcorder_h264_720p.mp4",         1280, 720, "mp_recordvideo2 %s VideoFormat=h264 VideoBitRate=8000000 AudioFormat=aac  AudioBitRate=96000" },
    //{ "H264:720p",     "/mfs/sd/MediaFiles/camcorder_h264_720p.mp4", 1280, 720, "h264", 8000000, "aac", 96000,0,0},
    { "H264:720p",     
      "/mfs/sd/MediaFiles/camcorder_h264_720p",".mp4", 
      1280, 720, 
      "h264", 8000000, "aac", 96000,0,0},
    //
    { "H264:VGA",     
      "/mfs/sd/MediaFiles/camcorder_h264_vga", ".mp4",         
      640, 480, 
      "h264", 1500000, "aac", 96000, 0, 0},
   //   { "H264:720p",     "/mfs/sd/MediaFiles/camcorder_h264_720p.mp4",         1280, 720, "mp_recordvideo2 %s VideoFormat=h264 VideoBitRate=8000000 AudioFormat=mp3  AudioBitRate=96000" },
   //   { "MPEG4:VGA",     "/mfs/sd/MediaFiles/camcorder_mpeg4_vga.mp4",         640, 480, "mp_recordvideo2 %s VideoFormat=mpg4 VideoBitRate=2000000 AudioFormat=aac  AudioBitRate=96000" },
   //   { "MPEG4:720p",     "/mfs/sd/MediaFiles/camcorder_mpeg4_720p.mp4",         1280, 720, "mp_recordvideo2 %s VideoFormat=mpg4 VideoBitRate=2000000 AudioFormat=aac  AudioBitRate=96000" },
   //   { "H263:CIF",     "/mfs/sd/MediaFiles/camcorder_h263_cif.mp4",         352, 288, "mp_recordvideo2 %s VideoFormat=h263 VideoBitRate=2000000 AudioFormat=aac  AudioBitRate=96000" },
};

static struct record_set_transform_value {
   char *string;
   OMX_DISPLAYTRANSFORMTYPE transform;
} transforms[] = {
   {"rot0", OMX_DISPLAY_ROT0},
   {"rot90", OMX_DISPLAY_ROT90},
   {"rot180", OMX_DISPLAY_ROT180},
   {"rot270", OMX_DISPLAY_ROT270},
   {"mirror_rot0", OMX_DISPLAY_MIRROR_ROT0},
   {"mirror_rot90", OMX_DISPLAY_MIRROR_ROT90},
   {"mirror_rot180", OMX_DISPLAY_MIRROR_ROT180},
   {"mirror_rot270", OMX_DISPLAY_MIRROR_ROT270},
   {0, 0},
};

#define CMDSTR_MAX 20

/*
 *
 */
#if !defined( __KERNEL__ )
int  vc_do_recorder3(void);
#endif
int vc_do_recorder2(char* cmd);
int vc_do_recorder(char* cmd, char* param);
int recorder_setup(void);
int recorder_start(char* fname);
int recorder_stop(void);
int recorder_quit(void);
int recorder_mode(char* modestr);
int setup_components(RECORD_STATE_T *st);
int32_t recorder_set_region(void *handle, uint32_t display, uint32_t fullscreen,
			    uint32_t x, uint32_t y, uint32_t width, uint32_t height,
			    const char *fill_mode, const char *transform, int animate);
int32_t recorder_set_resolution(void *handle, int width, int height);
int     recorder_set_zoom(void *handle, int newzoom);
int32_t recorder_prepare_camera(void *handle, int camplus_id, const char *cdi_id);
int32_t recorder_viewfinder_enable(void *handle);
int     recorder_configure_camera(RECORD_STATE_T *st, OMX_BOOL want_display);
int32_t recorder_open_file(void *handle, const char *filename,
                         const char *video_format, int video_bitrate,
                         const char *audio_format, int audio_bitrate,
			   int filesize, int framerate);
int32_t open_file(RECORD_STATE_T *st);
OMX_AUDIO_CODINGTYPE parse_audio_type(const char* type_string);
OMX_VIDEO_CODINGTYPE parse_video_type(const char* type_string);

#if defined(__KERNEL__)
int recorder_testwrite(char* path);
int recorder_testwrite2(char* path);
#endif

/*
 *
 */
#if !defined( __KERNEL__ )

int vc_do_recorder3(void)
{
  char cmdstr[CMDSTR_MAX + FILENAME_MAX + 1];
  
  vc_do_recorder("on", NULL);
  do
    {
      putchar('>');
      cmdstr[0] = 0;
      gets(cmdstr);     
      vc_do_recorder2(cmdstr);
    }
  while(0 != strncmp(cmdstr, "off", 4));

  return 0;
}

#endif // __KERNEL__

/*
 *
 */
int vc_do_recorder2(char* cmdstr)
{
  char* param = strchr(cmdstr, ' ');
  if(param)
    {
      *param = 0;
      return vc_do_recorder(cmdstr, param + 1); 
    }
  
  return vc_do_recorder(cmdstr, NULL); 
}

/*
 *
 */
int vc_do_recorder(char* cmd, char* param)
{
  
  if(0 == strncmp(cmd, "on", CMDSTR_MAX))
    {
      if(0 != recorder_setup())
	{
	  return -1;
	}
      
      recorder_viewfinder_enable(&recordstate);      
    } 
  else if(0 == strncmp(cmd, "rec", CMDSTR_MAX) ||
     0 == strncmp(cmd, "record", CMDSTR_MAX))
    {
      if(0 != recorder_start(param))
	{
	  return -1;
	}
      
    }   
  else if(0 == strncmp(cmd, "mode", CMDSTR_MAX))
    {
      if(0 != recorder_mode(param))
	{
	  return -1;
	}
      
    } 
  else if(0 == strncmp(cmd, "stop", CMDSTR_MAX))
    {
      if(0 != recorder_stop())
	{
	  return -1;
	}
      
      recorder_viewfinder_enable(&recordstate);      
    } 
  else if(0 == strncmp(cmd, "off", CMDSTR_MAX))
    {
      if(0 != recorder_quit())
	{
	  return -1;
	}
    }
#if defined(__KERNEL__)
  else if(0 == strncmp(cmd, "testfs", CMDSTR_MAX))
    {
      recorder_testwrite(param);
      return 0;
    }
  else if(0 == strncmp(cmd, "testfs2", CMDSTR_MAX))
    {
      recorder_testwrite2(param);
      return 0;
    }
#endif  
  else
    {
      ilclient_debug_output("invalid command: %s\n", cmd);
      return -1;
    }

  return 0;
}  

/*
 *
 */
int recorder_start(char* fname)
{
  int ret, recmode;
  static char* fname_def = "/mfs/sd/720p.mp4";

  if(NULL == fname || 0 == *fname)
    {
      fname = fname_def;
    }
  
  recmode = recordstate.mode;
  
  ret = recorder_open_file(&recordstate,
			   fname,
			   camcorder_formats[recmode].video_format,
			   camcorder_formats[recmode].video_bitrate,
			   camcorder_formats[recmode].audio_format,
			   camcorder_formats[recmode].audio_bitrate,
			   camcorder_formats[recmode].filesize,
			   camcorder_formats[recmode].framerate);
  return ret;
}

/*
 *
 */
int recorder_mode(char* modestr)
{
   RECORD_STATE_T *st = (RECORD_STATE_T *) &recordstate;
   int mode;

   if(1 != sscanf(modestr, "%d", &mode))
     {
       return -1;       
     }
   
   st->mode = mode % CAMCORDER_FMT_MAX;
   ilclient_debug_output("new mode=%d\n", mode % CAMCORDER_FMT_MAX);
   return 0;
}

/*
 * Transition all components back to loaded. Destroy tunnels.
 *
 */
int recorder_stop()
{
   RECORD_STATE_T *st = (RECORD_STATE_T *) &recordstate;
   OMX_ERRORTYPE error;
   OMX_PORT_PARAM_TYPE param;
   int status;
   COMPONENT_T *vlist[] = {&st->camera, &st->clock, &st->video_encode, &st->write_media,NULL};
   OMX_TIME_CONFIG_CLOCKSTATETYPE cstate;
   COMPONENT_T *alist[] = {&st->audio_record,&st->audio_encode, &st->write_media, &st->clock, NULL};
   COMPONENT_T *list[] = {&st->audio_record, &st->camera, &st->clock, &st->video_encode, &st->audio_encode, &st->write_media, NULL};
      
   st->state = RECORD_STOPPING;
   
   param.nSize = sizeof(OMX_PORT_PARAM_TYPE);
   param.nVersion.nVersion = OMX_VERSION;
   error = OMX_GetParameter(st->clock.comp, OMX_IndexParamOtherInit, &param);
   vc_assert(error == OMX_ErrorNone);

   {
      //Stop the clock - audio_record and camera will stop emitting data
     memset(&cstate, 0, sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE));
     cstate.nSize = sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE);
     cstate.nVersion.nVersion = OMX_VERSION;
     cstate.eState = OMX_TIME_ClockStateStopped;
     cstate.nStartTime = 0;
     cstate.nOffset = 0;
     cstate.nWaitMask = 0;
     error = OMX_SetConfig(st->clock.comp, OMX_IndexConfigTimeClockState, &cstate);
     vc_assert(error == OMX_ErrorNone);
   }

   //Wait for write media to recieve EOS on both of its ports.

   if (!st->paused){


      if (st->audio_bitrate)
      {
         status = ilclient_wait_for_event(&st->write_media, OMX_EventBufferFlag,
                                          170, 0, 1, 0, ILCLIENT_BUFFER_FLAG_EOS, 3000);
         vc_assert(status==0);
      }

      if (st->video_bitrate)
      {

         status = ilclient_wait_for_event(&st->write_media, OMX_EventBufferFlag,
                                          171, 0, 1, 0, ILCLIENT_BUFFER_FLAG_EOS, 3000);
         vc_assert(status==0);
      }
   }


   //transition pipeline specific components
   if (st->audio_bitrate && !st->video_bitrate){

      // transition the remaining components to idle and then loaded
      ilclient_state_transition(alist, OMX_StateIdle);
      ilclient_state_transition(alist, OMX_StateLoaded);

      // teardown tunnels
      ilclient_teardown_tunnels(st->rc_ae_tunnel);
      ilclient_teardown_tunnels(st->ae_wm_tunnel);

      {// Destroy clock tunnel
         error = OMX_SetupTunnel(st->clock.comp, param.nStartPortNumber+1, NULL, 0);
         vc_assert(error == OMX_ErrorNone);

         error = OMX_SetupTunnel(st->audio_record.comp, 181, NULL, 0);
         vc_assert(error == OMX_ErrorNone);
      }
   }
   else if (!st->audio_bitrate && st->video_bitrate){
      if (st->viewfinder){
         ilclient_change_component_state(&st->display, OMX_StateIdle);
         ilclient_change_component_state(&st->display, OMX_StateLoaded);
      }

      ilclient_state_transition(vlist, OMX_StateIdle);
      ilclient_state_transition(vlist, OMX_StateLoaded);

      // teardown tunnels
      if (st->viewfinder){
         ilclient_teardown_tunnels(st->ca_di_tunnel);
      }

      ilclient_teardown_tunnels(st->ca_ve_tunnel);
      ilclient_teardown_tunnels(st->ve_wm_tunnel);

      {// Destroy clock tunnel
         error = OMX_SetupTunnel(st->clock.comp, param.nStartPortNumber, NULL, 0);
         vc_assert(error == OMX_ErrorNone);

         error = OMX_SetupTunnel(st->camera.comp, 72, NULL, 0);
         vc_assert(error == OMX_ErrorNone);
      }

      ilclient_return_events(&st->camera);
   }
   else if (st->audio_bitrate && st->video_bitrate){
      if (st->viewfinder){
         ilclient_change_component_state(&st->display, OMX_StateIdle);
         ilclient_change_component_state(&st->display, OMX_StateLoaded);
      }

      // transition the remaining components to loaded
      ilclient_state_transition(list, OMX_StateIdle);
      ilclient_state_transition(list, OMX_StateLoaded);

      // teardown tunnels
      if (st->viewfinder){
         ilclient_teardown_tunnels(st->ca_di_tunnel);
      }

      ilclient_teardown_tunnels(st->rc_ae_tunnel);
      ilclient_teardown_tunnels(st->ae_wm_tunnel);
      ilclient_teardown_tunnels(st->ca_ve_tunnel);
      ilclient_teardown_tunnels(st->ve_wm_tunnel);

      {// Destroy clock tunnels
         error = OMX_SetupTunnel(st->clock.comp, param.nStartPortNumber+1, NULL, 0);
         vc_assert(error == OMX_ErrorNone);

         error = OMX_SetupTunnel(st->audio_record.comp, 181, NULL, 0);
         vc_assert(error == OMX_ErrorNone);

         error = OMX_SetupTunnel(st->clock.comp, param.nStartPortNumber+0, NULL, 0);
         vc_assert(error == OMX_ErrorNone);

         error = OMX_SetupTunnel(st->camera.comp, 72, NULL, 0);
         vc_assert(error == OMX_ErrorNone);
      }

      ilclient_return_events(&st->camera);
   }

   st->viewfinder=0;
   st->paused = 0;
   st->state = RECORD_IDLE;

   return 0;
}

/*
 *
 */
int recorder_quit(void)
{
  RECORD_STATE_T *st = (RECORD_STATE_T *) &recordstate;  
  OMX_ERRORTYPE error;
  COMPONENT_T *list[] = 
    {
      &st->camera, &st->video_encode, &st->display, &st->audio_record, &st->audio_encode, &st->write_media, &st->clock, NULL,
    };
  
  ilclient_cleanup_components(list);
  
  error = OMX_SetupDisptask(0, 0, 0);
  vc_assert(error == OMX_ErrorNone);
  
  if (st->cam_pools_allocated)
    {
      error = OMX_SetupCamPools(st->camplus_id,
                                0, 0, 0,    //Hi res pools
                                0, 0, 0,     //Lo res pools
                                0, 0, 0
				);
      vc_assert(error == OMX_ErrorNone);
      st->cam_pools_allocated = 0;
    }
  
  error = OMX_Deinit();
  vc_assert(error == OMX_ErrorNone);
  
  ilclient_destroy(st->client);

  return 0;
}

/***********************************************************
* Name: record_open_file
*
* Description: opens a given media file and starts record if
* start_recording is true.  If start_recording false we initialise
* in paused mode.
*
* video_format specifies the format in which to record the video track,
* likewise audio_format specifies the format for the audio track.
* If video_format is NULL we record only audio, if audio_format
* is NULL we record only video.
*
* XXX_bitrate specify the bitrate of
* the relevant track. If the format for a track has not been set the
* bitrate field for that track will be ignored.
*
* filesize specified the maximum filesize in bytes. A value of 0 means no filesize limit.
*
* framerate specifieds the maximum framerate. A value of 0 means no framerate limiting.
*
*
***********************************************************/
int32_t recorder_open_file(void *handle,
                         const char *filename,
                         const char *video_format,
                         int video_bitrate,
                         const char *audio_format,
                         int audio_bitrate,
                         int filesize,
                         int framerate)
{

   RECORD_STATE_T *st = (RECORD_STATE_T *) handle;
   int32_t success = -1;

   ilclient_debug_output("[recorder_open_file](%s,v:%s,a:%s)\n", filename, video_format ? "Y" : "N", audio_format ? "Y" : "N");

   if (st->state == RECORD_IDLE || st->state == RECORD_VIEWFINDER)
   {
      if (strlen(filename) >= CONTENTURI_MAXLEN)
      {
         strcpy(st->error_msg, "Filename too long");
      }
      else
      {
         st->error_msg[0] = 0;
         st->uri.uri.nVersion.nVersion = OMX_VERSION;
         strcpy((char *) st->uri.uri.contentURI, filename);
         st->uri.uri.nSize = sizeof(OMX_PARAM_CONTENTURITYPE) + strlen(filename);

         //Take copies of the supplied formats
         if (audio_bitrate){
            st->audio_format = parse_audio_type(audio_format);
            st->audio_bitrate = audio_bitrate;
         }

         if (video_bitrate){
            st->video_format = parse_video_type(video_format);
            st->video_bitrate = video_bitrate;
         }

         st->max_files_size = filesize;
         st->framerate = framerate;

         st->state = RECORD_STARTING;

         if (st->audio_format == OMX_AUDIO_CodingUnused || st->audio_format == OMX_AUDIO_CodingUnused)
            success = -1;
         else
            success = 0;

	 if (open_file(st) >= 0)
	   {
	     //we succeed in opening the file & we haven't been transitioned to stopped then transition to running
	     st->state = RECORD_RUNNING;
	   }
	 else
	   {  //we failed to open the file, go back to idle
	     vc_assert(0);
	     recorder_stop();
	     st->state = RECORD_IDLE;
	   }
	 
      }
   }

   return success;
}


/*
 *
 */
int recorder_setup(void)
{
  char response[128];
  int error;

  // initialize OpenMax
  error = OMX_Init();
  vc_assert(error == OMX_ErrorNone);

  // 
  vc_gencmd(response, sizeof(response), "ap_output_control output=0 power=1");
  vc_gencmd(response, sizeof(response), "display_control 0 power=2 backlight=2" );
  vc_gencmd(response, sizeof(response), "display_control 2 power=2 mode=0 svideo=1" );
  memset(&recordstate, 0, sizeof(recordstate));
  
  //
  recordstate.client = ilclient_init();
  setup_components(&recordstate);
  recorder_set_region(&recordstate, /*uint32_t display*/ 0 , /*uint32_t fullscreen*/ 1,
		      /*uint32_t x*/0, /*uint32_t y*/0, /*uint32_t width*/0, /*uint32_t height*/0,
		      /*char *fill_mode*/"fill", /*char *transform*/"mirror_rot0", /*animate*/ 0);
  recorder_set_resolution(&recordstate, 
			camcorder_formats[0].res_width, 
			camcorder_formats[0 ].res_height);
  recorder_set_zoom(&recordstate, 256); // initial zoom
  recorder_prepare_camera(&recordstate, (uint32_t)'CAMC', "CCP2");
  
  return 0;
}

/*
 *
 */
int setup_components(RECORD_STATE_T *st)
{
   OMX_ERRORTYPE error;
   int load_error;
   OMX_PARAM_U32TYPE omx_camplus_id;

   vc_assert(!st->components_created);

   // now start everything up
   //   error = OMX_Init();
   // vc_assert(error == OMX_ErrorNone);

   error = OMX_SetupDisptask(2, 1280, 720);
   vc_assert(error == OMX_ErrorNone);

   set_tunnel(st->rc_ae_tunnel, &st->audio_record, 180, &st->audio_encode, 160);
   set_tunnel(st->ae_wm_tunnel, &st->audio_encode, 161, &st->write_media, 170);
   set_tunnel(st->ca_di_tunnel, &st->camera, 70, &st->display, 10);
   set_tunnel(st->ca_ve_tunnel, &st->camera, 71, &st->video_encode, 200);
   set_tunnel(st->ve_wm_tunnel, &st->video_encode, 201, &st->write_media, 171);

   load_error=0;

   /* Setup audio_record component */
   {
      load_error += ilclient_create_component(st->client, &st->audio_record, "audio_record", 0, 0, 0);
   }

   /* Setup audio_encode component */
   {
      load_error += ilclient_create_component(st->client, &st->audio_encode, "audio_encode", 0, 0, 0);
   }

   /*Setup video_encode component*/
   {
      load_error += ilclient_create_component(st->client, &st->video_encode, "video_encode", 0, 0, 0);
   }

   /*Setup Camera*/
   {
      load_error += ilclient_create_component(st->client, &st->camera, "camera", 0, 0, 0);

      //Set camplus ID to zero. Expect it to be set later.
      omx_camplus_id.nSize = sizeof(OMX_PARAM_U32TYPE);
      omx_camplus_id.nVersion.nVersion = OMX_VERSION;
      omx_camplus_id.nPortIndex = OMX_ALL;
      omx_camplus_id.nU32 = 0;

      error += OMX_SetParameter(st->camera.comp, OMX_IndexParamCameraCamplusId, &omx_camplus_id);

      st->cam_pools_allocated = 0;
   }

   /*Setup display*/
   {
      load_error += ilclient_create_component(st->client, &st->display, "display", 0, 0, 0);
   }

   /* Setup write media component (required for any rec case)*/
   {
      load_error += ilclient_create_component(st->client, &st->write_media, "write_media", 0, 0, 0);
   }

   /* Setup clock component (required for any rec case)*/
   {
      load_error += ilclient_create_component(st->client, &st->clock, "clock", 0, 0, 0);
   }

   if (load_error != 0)
   {
      strcpy(st->error_msg, "Failed to load components");
      return -1;
   }

   vc_assert(error == OMX_ErrorNone);
   if (error == OMX_ErrorNone){
      st->components_created = OMX_TRUE;
      return OMX_ErrorNone;
   }
   else{
      return OMX_ErrorUndefined;
   }
}


/***********************************************************
* Name: recorder_set_region
*
* Description: sets the current region for viewfinder mode with given
* display number, transform and fill mode.
* If fullscreen is false, x, y, width, and height are used as the
* destination rectangle.
*
* Returns: 0 on success, -1 on failure
***********************************************************/
int32_t recorder_set_region(void *handle, uint32_t display, uint32_t fullscreen,
			    uint32_t x, uint32_t y, uint32_t width, uint32_t height,
			    const char *fill_mode,
			    const char *transform,
			    int animate)
{
  //   RECORD_STATE_T *st = (RECORD_STATE_T *) handle;
   OMX_CONFIG_DISPLAYREGIONTYPE region;
   int32_t i, success = 0;

   (void)handle;
   (void)animate;

   region.nSize = sizeof(OMX_CONFIG_DISPLAYREGIONTYPE);
   region.nVersion.nVersion = OMX_VERSION;
   region.set = OMX_DISPLAY_SET_NUM | OMX_DISPLAY_SET_FULLSCREEN |
                OMX_DISPLAY_SET_TRANSFORM | OMX_DISPLAY_SET_MODE;
   region.num = display;

   region.fullscreen = fullscreen;
   if (!fullscreen)
   {
      region.set |= OMX_DISPLAY_SET_DEST_RECT;
      region.dest_rect.x_offset = x;
      region.dest_rect.y_offset = y;
      region.dest_rect.width = width;
      region.dest_rect.height = height;
   }

   i=0;
   while (transforms[i].string && 0 == strcmp(transforms[i].string, transform))
      i++;

   if (!transforms[i].string)
      success = -1;

   region.transform = transforms[i].transform;

   if (0 == strcmp(fill_mode, "letterbox"))
      region.mode = OMX_DISPLAY_MODE_LETTERBOX;
   else if (0 == strcmp(fill_mode, "fill"))
      region.mode = OMX_DISPLAY_MODE_FILL;
   else
      success = -1;

   return success;
}


/***********************************************************
* Name: recorder_set_resolution
*
* Description: Sets the image size that recorder still or video images will have.
* The width and height values must be multiples of 16 up to 720P.
*
* Returns: 0 on success, -1 on failure
***********************************************************/
int32_t recorder_set_resolution(void *handle,
                              int width,
                              int height)
{
  RECORD_STATE_T *st = (RECORD_STATE_T *) handle;
  int32_t success = -1;
  
  if (st->state == RECORD_IDLE)
    {
      st->capture_width = width;
      st->capture_height = height;
      success = 0;
    }
  
  return success;
}

/***********************************************************
* Name: recorder_set_cam_zoom
*
* Description: sets the zoom properties for the camera
*
* Returns: 0 on success, -1 on failure
***********************************************************/
int recorder_set_zoom(void *handle, int newzoom)
{
   RECORD_STATE_T *st = (RECORD_STATE_T *) handle;
   int success = -1;
   OMX_S32 zoom_x=0;
   OMX_S32 zoom_y=0;
   OMX_ERRORTYPE error;
   OMX_CONFIG_SCALEFACTORTYPE zoom_param;

   st->zoom  = newzoom;
   zoom_x = (st->zoom << 8);
   zoom_y = (st->zoom << 8);
   vc_assert(zoom_x == zoom_y);
   zoom_param.nPortIndex = OMX_ALL;
   zoom_param.nSize = sizeof(OMX_TIME_CONFIG_TIMESTAMPTYPE);
   zoom_param.nVersion.nVersion = OMX_VERSION;
   zoom_param.xWidth = zoom_x;
   zoom_param.xHeight = zoom_y;
   error = OMX_SetParameter(st->camera.comp, OMX_IndexConfigCommonDigitalZoom, &zoom_param);
   vc_assert(error == OMX_ErrorNone);
   if (error == OMX_ErrorNone)
     {
       success = 0;
     }

   return success;

}

/***********************************************************
* Name: recorder_prepare_camera
*
* Description: prepare the camera.
*
* Returns 0 on success, -1 on failure
***********************************************************/
int32_t recorder_prepare_camera(void *handle, int camplus_id, const char *cdi_id)
{
   RECORD_STATE_T *st = (RECORD_STATE_T *) handle;
   OMX_ERRORTYPE error;
   OMX_PARAM_U32TYPE omx_camplus_id;
   OMX_PARAM_CAMERACDIIDTYPE camplus_cdi_id;
  
   st->camplus_id = camplus_id;
   strcpy(st->cdi_id, cdi_id);

   vc_assert(st->components_created);
   vc_assert(!st->cam_pools_allocated);

   omx_camplus_id.nSize = sizeof(OMX_PARAM_U32TYPE);
   omx_camplus_id.nVersion.nVersion = OMX_VERSION;
   omx_camplus_id.nPortIndex = OMX_ALL;
   omx_camplus_id.nU32 = st->camplus_id;

   camplus_cdi_id.nSize = sizeof(OMX_PARAM_U32TYPE);
   camplus_cdi_id.nVersion.nVersion = OMX_VERSION;
   strcpy((char *) camplus_cdi_id.id, st->cdi_id);

   error = OMX_SetParameter(st->camera.comp, OMX_IndexParamCameraCamplusId, &omx_camplus_id);
   if (error != OMX_ErrorNone)
      return -1;
   
   error = OMX_SetParameter(st->camera.comp, OMX_IndexParamCameraCdiId, &camplus_cdi_id);
   if (error != OMX_ErrorNone)
      return -1;
   else
      return 0;
   
}

/***********************************************************
* Name: recorder_viewfinder_enable
*
* Description: enable the viewfinder.
* Disable viewfinder by transitioning to MP_STOP.
*
* Returns: 0 on success, -1 on failure
***********************************************************/
int32_t recorder_viewfinder_enable(void *handle)
{
  
  RECORD_STATE_T *st = (RECORD_STATE_T *) handle;
  int32_t ret, success = -1;
  
  if (st->state == RECORD_IDLE)
    {
      st->state = RECORD_VIEWFINDER;
      success = 0;
    }
  
   //Configure the camera
   ret = recorder_configure_camera(st, 1);

   st->viewfinder = OMX_TRUE;

   return ret;
}

int recorder_configure_camera(RECORD_STATE_T *st, OMX_BOOL want_display)
{

   OMX_CONFIG_DISPLAYREGIONTYPE region;
   OMX_PARAM_U32TYPE camplus_id;
   OMX_ERRORTYPE error;
   OMX_CONFIG_BOOLEANTYPE param;
   OMX_PARAM_CAMERAMODETYPE camera_mode;
   COMPONENT_T *list[] = {&st->camera, &st->display, NULL};
   TUNNEL_T tunnel[] = {
     {&st->camera, 70, &st->display, 10},
     {NULL, 0, NULL, 0}
   };

   vc_assert(st->camera.comp);
   vc_assert(st->capture_width && st->capture_height);


   ilclient_disable_port(&st->camera, 70); // disable the preview port
   ilclient_disable_port(&st->camera, 71); // disable the capture port
   ilclient_disable_port(&st->camera, 72); // disable the time port

   ilclient_disable_port(&st->display, 10);
   ilclient_disable_port(&st->display, 11);

   if (!st->cam_pools_allocated){
      // setup camera pool

      camplus_id.nSize = sizeof(OMX_PARAM_U32TYPE);
      camplus_id.nVersion.nVersion = OMX_VERSION;
      camplus_id.nPortIndex = OMX_ALL;
      //Get camplus ID for this instance (we could set it and use that name instead,
      //but we only have one instance, so use the default)
      error = OMX_GetParameter(st->camera.comp, OMX_IndexParamCameraCamplusId, &camplus_id);
      vc_assert(camplus_id.nU32);
      vc_assert(error == OMX_ErrorNone);
      st->camplus_id = camplus_id.nU32;

      error = OMX_SetupCamPools(camplus_id.nU32,
                                8, st->capture_width, st->capture_height,    //Hi res pools
                                0, 0,    0,     //Lo res pools
                                2, 1600, 900
                               );
      vc_assert(error == OMX_ErrorNone);
      st->cam_pools_allocated = 1;

   }

   //Setup component to use ILCamPool image pool
   {
      param.nSize = sizeof(OMX_CONFIG_BOOLEANTYPE);
      param.nVersion.nVersion = OMX_VERSION;
      param.bEnabled = OMX_TRUE;
      error = OMX_SetParameter(st->camera.comp, OMX_IndexParamUseCameraPool, &param);
      vc_assert(error == OMX_ErrorNone);
   }

   // select video mode from CamPlus (enables proprietary tunneling in YUV_UV if
   // tunnel sink requests it, sets low res image at a scaling factor of 2, and
   // duplicates hi_res image out of both viewfinder and capture ports)
   {
      memset(&camera_mode, 0, sizeof(OMX_PARAM_CAMERAMODETYPE));
      camera_mode.nSize = sizeof(OMX_PARAM_CAMERAMODETYPE);
      camera_mode.nVersion.nVersion = OMX_VERSION;
      camera_mode.eMode = OMX_CAMERAMODETYPE_VIDEO;
      error = OMX_SetParameter(st->camera.comp, OMX_IndexParamCameraMode, &camera_mode);
      vc_assert(error == OMX_ErrorNone);
   }

   // setup the output format
   {
     OMX_PARAM_PORTDEFINITIONTYPE param;
      memset(&param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
      param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
      param.nVersion.nVersion = OMX_VERSION;
      param.nPortIndex = 71;
      param.eDir = OMX_DirOutput;
      param.eDomain = OMX_PortDomainVideo;
      param.format.video.eCompressionFormat = OMX_IMAGE_CodingUnused;
      param.format.video.eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;
      param.format.video.nFrameWidth = st->capture_width;
      param.format.video.nFrameHeight = st->capture_height;
      param.format.video.nStride = st->capture_width;
      param.format.video.nSliceHeight = st->capture_height;
      param.format.video.xFramerate = 30 << 16; /* 30fps to effectively request max frame rate */
      param.nBufferCountActual = 1;

      error = OMX_SetParameter(st->camera.comp, OMX_IndexParamPortDefinition, &param);
      vc_assert(error == OMX_ErrorNone);

      param.nPortIndex = 70;
      param.format.video.nSliceHeight = 16;
      error = OMX_SetParameter(st->camera.comp, OMX_IndexParamPortDefinition, &param);
      vc_assert(error == OMX_ErrorNone);
   }
   if (want_display){
      // setup display component
      {
         region.nSize = sizeof(OMX_CONFIG_DISPLAYREGIONTYPE);
         region.nVersion.nVersion = OMX_VERSION;
         region.set = OMX_DISPLAY_SET_NUM | OMX_DISPLAY_SET_FULLSCREEN | OMX_DISPLAY_SET_MODE | OMX_DISPLAY_SET_TRANSFORM;
         region.num = 0;
         region.fullscreen = 1;
         region.mode = OMX_DISPLAY_MODE_LETTERBOX;
         region.transform = OMX_DISPLAY_ROT0;
         error = OMX_SetConfig(st->display.comp, OMX_IndexConfigDisplayRegion, &region);
         vc_assert(error == OMX_ErrorNone);
      }

      //create the tunnel between camera and display

      if (ilclient_setup_tunnel(tunnel, 0, PLATFORM_EVENTGROUP_SUSPEND) != 0)
         vc_assert(0);

      // transition to executing
      ilclient_state_transition(list, OMX_StateIdle);

      // transition to executing
      ilclient_state_transition(list, OMX_StateExecuting);
   }

   if (error == OMX_ErrorNone){
      return 0;
   }
   else{
      strcpy(st->error_msg, "Failed to load components");
      return -1;
   }
}

OMX_VIDEO_CODINGTYPE parse_video_type(const char* type_string){

   OMX_VIDEO_CODINGTYPE coding = OMX_VIDEO_CodingUnused;

   if (!strncmp(type_string, "h264", sizeof(type_string)))
      coding = OMX_VIDEO_CodingAVC;
   else if (!strncmp(type_string, "263", sizeof(type_string)))
      coding = OMX_VIDEO_CodingH263;

   return coding;
}

OMX_AUDIO_CODINGTYPE parse_audio_type(const char* type_string){

   OMX_AUDIO_CODINGTYPE coding = OMX_AUDIO_CodingUnused;

   if (!strncmp(type_string, "aac", sizeof(type_string)))
      coding = OMX_AUDIO_CodingAAC;
   else if (!strncmp(type_string, "mp3", sizeof(type_string)))
      coding = OMX_AUDIO_CodingMP3;

   return coding;
}

int32_t open_file(RECORD_STATE_T *st)
{
   OMX_ERRORTYPE error;
   vc_assert(st->components_created);

   //Setup the components
   {
      if (st->audio_bitrate){
         //Configure audio_record
         {
            OMX_PARAM_PORTDEFINITIONTYPE param;
            memset(&param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
            param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
            param.nVersion.nVersion = OMX_VERSION;
            param.nPortIndex = 180;

            error = OMX_GetParameter(st->audio_record.comp, OMX_IndexParamPortDefinition, &param);

            param.nBufferSize = 1<<13;

            error = OMX_SetParameter(st->audio_record.comp, OMX_IndexParamPortDefinition, &param);
            vc_assert(error == OMX_ErrorNone);
         }

         // Configure audio_encode
         {
            OMX_PARAM_PORTDEFINITIONTYPE encode_output_param;
            memset(&encode_output_param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
            encode_output_param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
            encode_output_param.nVersion.nVersion = OMX_VERSION;
            encode_output_param.nPortIndex = 161;

            error = OMX_GetParameter(st->audio_encode.comp, OMX_IndexParamPortDefinition, &encode_output_param);
            vc_assert(error == OMX_ErrorNone);

            encode_output_param.eDir = OMX_DirOutput;
            encode_output_param.eDomain = OMX_PortDomainAudio;
            encode_output_param.format.audio.eEncoding = st->audio_format;
            encode_output_param.nBufferSize = 2000;

            error = OMX_SetParameter(st->audio_encode.comp, OMX_IndexParamPortDefinition, &encode_output_param);
            vc_assert(error == OMX_ErrorNone);

            /* setup output encode format */

            switch (st->audio_format){
            case OMX_AUDIO_CodingAAC :
            {
               OMX_AUDIO_PARAM_AACPROFILETYPE encoded_format;
               encoded_format.nSize = sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE);
               encoded_format.nVersion.nVersion = OMX_VERSION;
               encoded_format.nChannels=2;
               encoded_format.nBitRate=st->audio_bitrate;//can i get away with this
               encoded_format.nSampleRate=16000;
               encoded_format.nPortIndex = 161;
               error = OMX_SetParameter(st->audio_encode.comp, OMX_IndexParamAudioAac , &encoded_format);
            }
            break;
            case OMX_AUDIO_CodingMP3 :
            {
               OMX_AUDIO_PARAM_MP3TYPE encoded_format;
               encoded_format.nSize = sizeof(OMX_AUDIO_PARAM_MP3TYPE);
               encoded_format.nVersion.nVersion = OMX_VERSION;
               encoded_format.nChannels=2;
               encoded_format.nBitRate=st->audio_bitrate;//can i get away with this
               encoded_format.nSampleRate=16000;
               encoded_format.nPortIndex = 161;
               error = OMX_SetParameter(st->audio_encode.comp, OMX_IndexParamAudioMp3 , &encoded_format);
            }
            break;
            default: vc_assert(0);
            }

            vc_assert(error == OMX_ErrorNone);
         }
      }

      if (st->video_bitrate){
         // Configure video_encode
         {
            OMX_PARAM_PORTDEFINITIONTYPE encode_output_param;
            memset(&encode_output_param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
            encode_output_param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
            encode_output_param.nVersion.nVersion = OMX_VERSION;
            encode_output_param.nPortIndex = 201;


            error = OMX_GetParameter(st->video_encode.comp, OMX_IndexParamPortDefinition, &encode_output_param);
            vc_assert(error == OMX_ErrorNone);

            encode_output_param.eDir = OMX_DirOutput;
            encode_output_param.eDomain = OMX_PortDomainVideo;
            encode_output_param.format.video.eCompressionFormat = st->video_format;
            encode_output_param.format.video.nBitrate = st->video_bitrate;
	    //            encode_output_param.format.video.nBitrate = 1<<20;
            encode_output_param.nBufferSize = (400*1024);

            error = OMX_SetParameter(st->video_encode.comp, OMX_IndexParamPortDefinition, &encode_output_param);
            vc_assert(error == OMX_ErrorNone);
         }

         if (!st->viewfinder){
            if (recorder_configure_camera(st, 0) <0){
               strcpy(st->error_msg, "Failed to load components");
               return -1;
            }
         }
      }
      // Configure write_media
      {
         //specify the content by URI
         error = OMX_SetParameter(st->write_media.comp, OMX_IndexParamContentURI, &st->uri);
         vc_assert(error == OMX_ErrorNone);
      }

      // Configure clock
      {
         //disable ports on audio_record component
         ilclient_disable_port(&st->audio_record, 180); // disable the audio port
         ilclient_disable_port(&st->audio_record, 181); // disable the clock port

         //disable capture and clock ports on camrea component
         ilclient_disable_port(&st->camera, 71); // disable the capture port
         ilclient_disable_port(&st->camera, 72); // disable the clock port

         // disable all ports apart from the first two
         OMX_PORT_PARAM_TYPE param;
         OMX_TIME_CONFIG_ACTIVEREFCLOCKTYPE refclock;
         uint32_t i;

         param.nSize = sizeof(OMX_PORT_PARAM_TYPE);
         param.nVersion.nVersion = OMX_VERSION;
         error = OMX_GetParameter(st->clock.comp, OMX_IndexParamOtherInit, &param);
         vc_assert(error == OMX_ErrorNone);
         vc_assert(param.nPorts >= 2);

         for (i=param.nStartPortNumber; i<param.nStartPortNumber+param.nPorts; i++)
            ilclient_disable_port(&st->clock, i);

         refclock.nSize = sizeof(OMX_TIME_CONFIG_ACTIVEREFCLOCKTYPE);
         refclock.nVersion.nVersion = OMX_VERSION;
         refclock.eClock = st->audio_bitrate ? OMX_TIME_RefClockAudio : OMX_TIME_RefClockVideo;
         error = OMX_SetConfig(st->clock.comp, OMX_IndexConfigTimeActiveRefClock, &refclock);
         vc_assert(error == OMX_ErrorNone);

         ilclient_change_component_state(&st->clock, OMX_StateIdle);

         if (st->video_bitrate){
            // can connect up camera to clock - no data dependencies
            error = OMX_SetupTunnel(st->clock.comp, param.nStartPortNumber, st->camera.comp, 72);
            vc_assert(error == OMX_ErrorNone);

            error = OMX_SendCommand(st->clock.comp, OMX_CommandPortEnable, param.nStartPortNumber, NULL);
            vc_assert(error == OMX_ErrorNone);

            error = OMX_SendCommand(st->camera.comp, OMX_CommandPortEnable, 72, NULL);
            vc_assert(error == OMX_ErrorNone);

            ilclient_wait_for_event(&st->camera, OMX_EventCmdComplete,
                                    OMX_CommandPortEnable, 0, 72, 0,
                                    ILCLIENT_PORT_ENABLED, PLATFORM_EVENTGROUP_SUSPEND);
            ilclient_wait_for_event(&st->clock, OMX_EventCmdComplete,
                                    OMX_CommandPortEnable, 0, param.nStartPortNumber, 0,
                                    ILCLIENT_PORT_ENABLED, PLATFORM_EVENTGROUP_SUSPEND);
         }

         if (st->audio_bitrate){
            // can connect up audio record to clock - no data dependencies
            error = OMX_SetupTunnel(st->clock.comp, param.nStartPortNumber+1, st->audio_record.comp, 181);
            vc_assert(error == OMX_ErrorNone);

            error = OMX_SendCommand(st->clock.comp, OMX_CommandPortEnable, param.nStartPortNumber+1, NULL);
            vc_assert(error == OMX_ErrorNone);

            error = OMX_SendCommand(st->audio_record.comp, OMX_CommandPortEnable, 181, NULL);
            vc_assert(error == OMX_ErrorNone);

            ilclient_change_component_state(&st->audio_record, OMX_StateIdle);

            ilclient_wait_for_event(&st->audio_record, OMX_EventCmdComplete,
                                    OMX_CommandPortEnable, 0, 181, 0,
                                    ILCLIENT_PORT_ENABLED, PLATFORM_EVENTGROUP_SUSPEND);
            ilclient_wait_for_event(&st->clock, OMX_EventCmdComplete,
                                    OMX_CommandPortEnable, 0, param.nStartPortNumber+1, 0,
                                    ILCLIENT_PORT_ENABLED, PLATFORM_EVENTGROUP_SUSPEND);
         }

         {
            OMX_TIME_CONFIG_CLOCKSTATETYPE cstate;

            memset(&cstate, 0, sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE));
            cstate.nSize = sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE);
            cstate.nVersion.nVersion = OMX_VERSION;
            error = OMX_GetConfig(st->clock.comp, OMX_IndexConfigTimeClockState, &cstate);
            vc_assert(error == OMX_ErrorNone && cstate.eState == OMX_TIME_ClockStateStopped);
         }
      }

      //We have already setup the components, now connect up accordingly

      // disable all ports on write media component
      ilclient_disable_port(&st->write_media, 170); // disable the WM audio port
      ilclient_disable_port(&st->write_media, 171); // disable the WM video port

      //If we have an audio pipeline then set that up first
      if (st->audio_bitrate){
         ilclient_enable_port(&st->write_media, 170); // enable the audio port

         //Setup the audio record to audio encode pipeline.
         if (ilclient_setup_tunnel(st->rc_ae_tunnel, 0, PLATFORM_EVENTGROUP_SUSPEND) != 0)
            vc_assert(0);

         COMPONENT_T *alist[] = {&st->audio_record, &st->audio_encode, NULL};

         //Transition audio record and encode to executing. Clock not yet started so won't output data.
         ilclient_state_transition(alist, OMX_StateIdle);
         ilclient_state_transition(alist, OMX_StateExecuting);
      }

      //Setup the video pipeline
      if (st->video_bitrate){
         /*
         * At this stage either the camera and display components are tunneled and
         * executing or both are in the loaded state. The camera will not be capturing
         * in either version.
         */

         // setup capture port to capture
         {
            OMX_CONFIG_BOOLEANTYPE capturing;
            memset(&capturing, 0, sizeof(OMX_CONFIG_BOOLEANTYPE));
            capturing.nSize = sizeof(OMX_CONFIG_BOOLEANTYPE);
            capturing.nVersion.nVersion = OMX_VERSION;
            capturing.bEnabled = OMX_TRUE;
            //capturing.bEnabled = OMX_FALSE;
            error = OMX_SetParameter(st->camera.comp, OMX_IndexConfigCapturing, &capturing);
            vc_assert(error == OMX_ErrorNone);
         }

         if (!st->viewfinder){
            // setup camera component
            ilclient_disable_port(&st->camera, 70); // disable the preview port as we don't use the display
         }

         // set up tunnel between camera and video encode
         if (ilclient_setup_tunnel(st->ca_ve_tunnel, 0, PLATFORM_EVENTGROUP_SUSPEND) != 0)
            vc_assert(0);

         //Ensure both camera and video encode are executing
         if (st->viewfinder){
            ilclient_change_component_state(&st->video_encode, OMX_StateExecuting);
         }
         else{
            COMPONENT_T *list[] = {&st->camera, &st->video_encode, NULL};
            ilclient_state_transition(list, OMX_StateExecuting);
         }
      }

      //Transition clock to executing, tell it to move to WaitForStartTime
      ilclient_change_component_state(&st->clock, OMX_StateExecuting);

      {
         OMX_TIME_CONFIG_CLOCKSTATETYPE cstate;

         memset(&cstate, 0, sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE));
         cstate.nSize = sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE);
         cstate.nVersion.nVersion = OMX_VERSION;
         cstate.eState = st->audio_bitrate ? OMX_TIME_ClockStateWaitingForStartTime : OMX_TIME_ClockStateRunning;
         cstate.nStartTime = 0;
         cstate.nOffset = 0;
         cstate.nWaitMask = OMX_CLOCKPORT1;
         error = OMX_SetConfig(st->clock.comp, OMX_IndexConfigTimeClockState, &cstate);
         vc_assert(error == OMX_ErrorNone);
      }

      //Clock will now recieve start time from audio record and source components will begin to output data.
      //Connect encodes to write media, transition write media to executing to finalise its sources.

      if (st->audio_bitrate){
         if (ilclient_setup_tunnel(st->ae_wm_tunnel, 0, PLATFORM_EVENTGROUP_SUSPEND) != 0)
            vc_assert(0);
      }

      if (st->video_bitrate){
         if (ilclient_setup_tunnel(st->ve_wm_tunnel, 0, PLATFORM_EVENTGROUP_SUSPEND) != 0)
            vc_assert(0);
      }

      ilclient_change_component_state(&st->write_media, OMX_StateExecuting);

      return 0;
   }
}


#if defined(__KERNEL__)
/*
 *   
 */
int vc_hostfs_open(const char *inPath, int vc_oflag);
int vc_hostfs_close(int fildes);
int vc_hostfs_write(int fd, const void *buf, unsigned int count);

#define FILEPATH_MAX 255

typedef struct 
{
  char path[FILEPATH_MAX + 1];
} testwrite_param_t;

static testwrite_param_t testwrite_param;

int recorder_testwrite_task(void* data)
{
  unsigned long st, et, dt;
  const int block_size = 4096;
  const int file_size = 1024 * 1024 * 32;
  // const int file_size = 4096;
  int count = file_size / block_size;
  int i, fd;
  char* buf;
  testwrite_param_t* param;
  
  param = (testwrite_param_t*) data;

  printk("recorder_testwrite_task starting. path=%s\n", param->path);
  //
  buf = vmalloc(block_size);
  for(i = 0; i < block_size; ++i)
    {						
      buf[i] = i;
    }

  // 
  st = jiffies_to_msecs(jiffies);

  //
  fd = vc_hostfs_open(param->path, VC_O_WRONLY | VC_O_CREAT | VC_O_TRUNC);

  if(fd < 0)
    {
      printk("recorder_testwrite: unable to open %s\n", param->path);
      vfree(buf);
      return -1;
    }
	  //
  while(count)
    {
      vc_hostfs_write(fd, buf, block_size);
      --count;
    }
  
  //
  vc_hostfs_close(fd);
  et = jiffies_to_msecs(jiffies);
  dt = (et > st) ? (et - st) : (~0 - st + et);
  printk("recorder_testwrite_task finished, st=%lu et=%lu dt=%lu wrote %d MByte, %lu byte/msec path=%s\n", 
	 st, et, dt, 
	 file_size/(1024*1024),
	 file_size/dt,
	 param->path);
  vfree(buf);
  
  return 0;
}

int recorder_testwrite_task2(void* data)
{
  unsigned long st, et, dt, ddt;;
#define  BLOCK_MAX 4096
  const int file_size = 1024 * 1024 * 32;
  int block_size;
  int iter = file_size / (BLOCK_MAX + BLOCK_MAX/3);
  int i, fd, nbytes, pos;
  char* buf;
  testwrite_param_t* param;
  
  param = (testwrite_param_t*) data;
  printk("recorder_testwrite_task2 starting, path=%s\n", param->path);

  //
  buf = vmalloc(BLOCK_MAX);
  nbytes = 0;
  ddt = 0;
  fd = vc_hostfs_open(param->path, VC_O_WRONLY | VC_O_CREAT | VC_O_TRUNC);
  if(fd < 0)
    {
      printk("reorder_testwrite2: unable to open %s\n", param->path);
      vfree(buf);
      return -1;
    }

  //
  block_size = BLOCK_MAX;
  for(i = 0; i < block_size; ++i)
    {						
      buf[i] = (uint8_t)i;
    }
  //
  st = jiffies_to_msecs(jiffies);
  while(iter)
    {
      block_size = BLOCK_MAX;
      block_size = vc_hostfs_write(fd, buf, block_size);
      nbytes += block_size;

      block_size = BLOCK_MAX / 3;
      block_size = vc_hostfs_write(fd, buf, block_size);
      nbytes += block_size;      
      pos = vc_hostfs_lseek64(fd, 0, 1);
      vc_hostfs_lseek64(fd, nbytes - 177, 0);
      vc_hostfs_write(fd, &iter, 4);
      nbytes += 4;
      pos = vc_hostfs_lseek64(fd, pos, 0);      
      --iter;
    }
  
  //
  vc_hostfs_close(fd);
  et = jiffies_to_msecs(jiffies);
  dt = (et > st) ? (et - st) : (~0 - st + et);
  ddt += dt;
  printk("recorder_testwrite_task finished, st=%lu et=%lu dt=%lu wrote %d MByte, %d byte/msec path=%s\n", 
	 st, et, ddt, 
	 nbytes/(1024*1024),
	 nbytes/ ddt,
	 param->path);
  vfree(buf);
  
  return 0;
}

int recorder_testwrite(char* path)
{
  int ret;

  strncpy(testwrite_param.path, path, FILEPATH_MAX);
  testwrite_param.path[FILEPATH_MAX] = 0;
  ret = kthread_run(recorder_testwrite_task, &testwrite_param, "recorder_testwrite_task");
  if(IS_ERR(ret))
    {
      printk("error starting recorder_testwrite_task ret=%d\n", ret);
    }

  return 0;
}

int recorder_testwrite2(char* path)
{
  int ret;

  strncpy(testwrite_param.path, path, FILEPATH_MAX);
  testwrite_param.path[FILEPATH_MAX] = 0;
  ret = kthread_run(recorder_testwrite_task2, &testwrite_param, "recorder_testwrite_task");
  if(IS_ERR(ret))
    {
      printk("error starting recorder_testwrite_task ret=%d\n", ret);
    }

  return 0;
}

#endif // __KERNEL__
