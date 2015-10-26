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
Module   :  IL Demos - Play
File     :  $RCSfile: play.c,v $
Revision :  $Revision: 1.1.2.10 $

FILE DESCRIPTION
Play demo
=============================================================================*/

#define PLAYING_LENGTH_MSEC      10000
#define CAMERA_PREVIEW_RENDERER  1


#include <stdarg.h>
#include <linux/broadcom/vc03/vcilcs_intern.h>
#include <linux/broadcom/vc03/vcomx.h>
#include <linux/broadcom/vc03/vcilclient.h>
#include <linux/broadcom/vc03/vcilplatform.h>
#include <linux/broadcom/vc03/dispmanx_types.h>

#if !defined(__KERNEL__)

#define malloc libomx_malloc
#define free   libomx_free
#include <string.h>
#include <stdio.h>
#include "omxctl.h"

#define local_printf printf

#else

#define local_printf ilclient_debug_output

#endif


#define OMX_VERSION 0x10101


/* This component will handle URI's up to this maximum length */
#define ILTEST_CONTENTURI_MAXLEN 256

typedef struct {
   char error_msg[64];
   OMX_S32 scale;

   // pointers to OpenMAX elements
   ILCLIENT_T *client;
   COMPONENT_T *clock;

#if CAMERA_PREVIEW_RENDERER
   COMPONENT_T *list[4];
//   TUNNEL_T tunnels[9];
   TUNNEL_T ctunnels[2];

   COMPONENT_T *camera;
   COMPONENT_T *video_render2;

   OMX_CONFIG_DISPLAYREGIONTYPE region2;
   TUNNEL_T ca_di_tunnel[2];

   OMX_U32 camplus_id;
   int camera_num;
   char isp_tuner[32];
   OMX_IMAGE_FOCUSCONTROLTYPE focus_mode;
   OMX_U32 focus_position;
   int capture_width;
   int capture_height;

#else
   COMPONENT_T *list[11];
   TUNNEL_T tunnels[8];
   TUNNEL_T ctunnels[6];


#endif

} CAM_STATE_T;


#define ilclient_debug_output OMX_DEBUG

#define DISPMANX_FRAMEBUF_SCREEN_DISPLAY     0
CAM_STATE_T cst;


#define RECORD_IL_MAX_FRAMERATE             30
#define RECORD_IL_IP_WIDTH                1600
#define RECORD_IL_IP_HEIGHT                900
#define RECORD_IL_NUM_HI_RES_POOL_FRAMES     2


static int configure_camera(CAM_STATE_T *st)//, OMX_BOOL want_display)
{
   OMX_ERRORTYPE error;

   ilclient_disable_port(st->camera, 70); // disable the preview port
   ilclient_disable_port(st->camera, 71); // disable the capture port
   ilclient_disable_port(st->camera, 72); // disable the time port

   ilclient_disable_port(st->video_render2, 90);

   {
      OMX_PARAM_CAMERAIMAGEPOOLTYPE camera_pools;

      // setup camera pool
      camera_pools.nSize = sizeof(OMX_PARAM_CAMERAIMAGEPOOLTYPE);
      camera_pools.nVersion.nVersion = OMX_VERSION;
      camera_pools.num_hi_res_frames = RECORD_IL_NUM_HI_RES_POOL_FRAMES;
      camera_pools.hi_res_width = st->capture_width;
      camera_pools.hi_res_height = st->capture_height;
      camera_pools.hi_res_type = 0;

      camera_pools.num_low_res_frames = camera_pools.num_hi_res_frames;
      camera_pools.low_res_width = st->capture_width >>1;
      camera_pools.low_res_height = st->capture_height >>1;
      camera_pools.low_res_type = 0;

      camera_pools.num_input_frames = 2;
      camera_pools.input_width = RECORD_IL_IP_WIDTH;
      camera_pools.input_height = RECORD_IL_IP_HEIGHT;
      camera_pools.input_type = 0;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamCameraImagePool, &camera_pools);
   }

   // setup the output format
   {
      //Only set the viewfinder port settings here - the capture port will be the same,
      //except for the framerate, which we don't know yet.
      OMX_PARAM_PORTDEFINITIONTYPE param;
      memset(&param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
      param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
      param.nVersion.nVersion = OMX_VERSION;
      param.nPortIndex = 70;
      param.eDir = OMX_DirOutput;
      param.eDomain = OMX_PortDomainVideo;
      param.format.video.eCompressionFormat = OMX_IMAGE_CodingUnused;
      param.format.video.eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;
      param.format.video.nFrameWidth = st->capture_width;
      param.format.video.nFrameHeight = st->capture_height;
      param.format.video.nStride = (st->capture_width+15)&~15;
      param.format.video.nSliceHeight = 16;
      param.format.video.xFramerate = RECORD_IL_MAX_FRAMERATE << 16;
      param.nBufferCountActual = 1;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamPortDefinition, &param);
   }
   
   {
      //Only set the viewfinder port settings here - the capture port will be the same,
      //except for the framerate, which we don't know yet.
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
      param.format.video.nStride = (st->capture_width+15)&~15;
      param.format.video.nSliceHeight = 16;
      param.format.video.xFramerate = RECORD_IL_MAX_FRAMERATE << 16;
      param.nBufferCountActual = 1;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamPortDefinition, &param);
      vc_assert(error == OMX_ErrorNone);
   }

   // setup capture port to capture
   {
      OMX_CONFIG_BOOLEANTYPE capturing;
      memset(&capturing, 0, sizeof(OMX_CONFIG_BOOLEANTYPE));
      capturing.nSize = sizeof(OMX_CONFIG_BOOLEANTYPE);
      capturing.nVersion.nVersion = OMX_VERSION;
      capturing.bEnabled = OMX_TRUE;
      //capturing.bEnabled = OMX_FALSE;
      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexConfigCapturing, &capturing);
      vc_assert(error == OMX_ErrorNone);
   }

   {
      OMX_IMAGE_FOCUSCONTROLTYPE focus_mode;
      OMX_U32 focus_position;
      OMX_ERRORTYPE error;
      OMX_IMAGE_CONFIG_FOCUSCONTROLTYPE focus_control;

      focus_mode = st->focus_mode;
      focus_position = st->focus_position;

      memset(&focus_control, 0, sizeof(OMX_IMAGE_CONFIG_FOCUSCONTROLTYPE));
      focus_control.nSize = sizeof(OMX_IMAGE_CONFIG_FOCUSCONTROLTYPE);
      focus_control.nVersion.nVersion = OMX_VERSION;
      focus_control.nPortIndex = OMX_ALL;
      error = OMX_GetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexConfigFocusControl, &focus_control);

      focus_control.eFocusControl = focus_mode;
      focus_control.nFocusStepIndex = focus_position;
      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexConfigFocusControl, &focus_control);
   }

   {
      OMX_PARAM_U32TYPE cameras_present;
      cameras_present.nSize = sizeof(OMX_PARAM_U32TYPE);
      cameras_present.nVersion.nVersion = OMX_VERSION;
      cameras_present.nPortIndex = OMX_ALL;

      error = OMX_GetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamCameraDevicesPresent, &cameras_present);

      if (error != OMX_ErrorNone || !(cameras_present.nU32&(1<<st->camera_num)))
         return -1;
   }

   {
      OMX_PARAM_U32TYPE omx_camplus_id;
      omx_camplus_id.nSize = sizeof(OMX_PARAM_U32TYPE);
      omx_camplus_id.nVersion.nVersion = OMX_VERSION;
      omx_camplus_id.nPortIndex = OMX_ALL;
      omx_camplus_id.nU32 = st->camplus_id;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamCameraCamplusId, &omx_camplus_id);
      if (error != OMX_ErrorNone)
         return -1;
   }

   {
      OMX_PARAM_U32TYPE camera_number;
      camera_number.nSize = sizeof(OMX_PARAM_U32TYPE);
      camera_number.nVersion.nVersion = OMX_VERSION;
      camera_number.nPortIndex = OMX_ALL;
      camera_number.nU32 = st->camera_num;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamCameraDeviceNumber, &camera_number);
      if (error != OMX_ErrorNone)
         return -1;
   }
#if 0
   {
      OMX_PARAM_CAMERAISPTUNERTYPE isp_tuner;
      isp_tuner.nSize = sizeof(OMX_PARAM_CAMERAISPTUNERTYPE);
      isp_tuner.nVersion.nVersion = OMX_VERSION;
      strcpy((char*)isp_tuner.tuner_name, st->isp_tuner);
      
      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamISPTunerName, &isp_tuner);

      if (error != OMX_ErrorNone)
         return -1;
   }
#endif
   {
      COMPONENT_T *list[] = {st->camera, st->video_render2, NULL};

      ilclient_setup_tunnel(st->ca_di_tunnel, 0, PLATFORM_EVENTGROUP_SUSPEND);
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

static int32_t open_cam(CAM_STATE_T *st)
{
   OMX_CONFIG_DISPLAYREGIONTYPE region2;
//   OMX_CONFIG_VISUALISATIONTYPE vistype;
   OMX_ERRORTYPE error;
//   TUNNEL_T *tunnel;
   COMPONENT_T **comp;
   int i;
   //, streams;

   region2 = st->region2;

//   vistype = st->vistype;

   comp = st->list;
//   tunnel = st->tunnels;

   {
      OMX_PARAM_U32TYPE omx_camplus_id;

      ilclient_create_component(st->client, &st->camera, "camera", 0, 0, 0);

      *comp++ = st->camera;

      //Set camplus ID to zero. Expect it to be set later.
      omx_camplus_id.nSize = sizeof(OMX_PARAM_U32TYPE);
      omx_camplus_id.nVersion.nVersion = OMX_VERSION;
      omx_camplus_id.nPortIndex = OMX_ALL;
      omx_camplus_id.nU32 = 0;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->camera), OMX_IndexParamCameraCamplusId, &omx_camplus_id);
   }

   ilclient_create_component(st->client, &st->video_render2, "video_render", 0, 1, 0);
   *comp++ = st->video_render2;
   ilclient_disable_port(st->video_render2, 90);

   // setup display region if already set
   if (region2.set)
   {
      error = OMX_SetConfig(ILC_GET_HANDLE(st->video_render2), OMX_IndexConfigDisplayRegion, &region2);
      vc_assert(error == OMX_ErrorNone);
   }

   ilclient_create_component(st->client, &st->clock, "clock", 0, 0, 0);
   *comp++ = st->clock;

   // setup clock
   {
      OMX_PORT_PARAM_TYPE param;
      OMX_TIME_CONFIG_ACTIVEREFCLOCKTYPE refclock;
      OMX_TIME_CONFIG_CLOCKSTATETYPE cstate;
      OMX_TIME_CONFIG_SCALETYPE sparam;
      TUNNEL_T *ctunnel;
      int port;

      param.nSize = sizeof(OMX_PORT_PARAM_TYPE);
      param.nVersion.nVersion = OMX_VERSION;
      error = OMX_GetParameter(ILC_GET_HANDLE(st->clock), OMX_IndexParamOtherInit, &param);
      vc_assert(error == OMX_ErrorNone);
      vc_assert(param.nPorts >= 4);

      refclock.nSize = sizeof(OMX_TIME_CONFIG_ACTIVEREFCLOCKTYPE);
      refclock.nVersion.nVersion = OMX_VERSION;
      refclock.eClock = OMX_TIME_RefClockVideo;
      error = OMX_SetConfig(ILC_GET_HANDLE(st->clock), OMX_IndexConfigTimeActiveRefClock, &refclock);
      vc_assert(error == OMX_ErrorNone);

      cstate.nSize = sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE);
      cstate.nVersion.nVersion = OMX_VERSION;
      cstate.eState = OMX_TIME_ClockStateWaitingForStartTime;
      cstate.nStartTime = 0;
      cstate.nOffset = -1000 * 200;
      cstate.nWaitMask = OMX_CLOCKPORT0;//(st->audio_stream_present && st->video_stream_present ? OMX_CLOCKPORT0|OMX_CLOCKPORT1 : OMX_CLOCKPORT0);
      error = OMX_SetConfig(ILC_GET_HANDLE(st->clock), OMX_IndexConfigTimeClockState, &cstate);
      vc_assert(error == OMX_ErrorNone);

      sparam.nSize = sizeof(OMX_TIME_CONFIG_SCALETYPE);
      sparam.nVersion.nVersion = OMX_VERSION;
      sparam.xScale = st->scale;
      error = OMX_SetConfig(ILC_GET_HANDLE(st->clock), OMX_IndexConfigTimeScale, &sparam);
      vc_assert(error == OMX_ErrorNone);

      ctunnel = st->ctunnels;
      port = param.nStartPortNumber;

      set_tunnel(ctunnel, st->clock, port++, st->camera, 72);
      ilclient_disable_tunnel(ctunnel++);

      // disable unused clock ports
      for (i=port; i<param.nStartPortNumber+param.nPorts; i++)
         ilclient_disable_port(st->clock, i);

      // connect up tunnels - no data dependencies
      ctunnel = st->ctunnels;
      while(ctunnel->sink)
      {
         error = OMX_SetupTunnel(ILC_GET_HANDLE(ctunnel->source), ctunnel->source_port, ILC_GET_HANDLE(ctunnel->sink), ctunnel->sink_port);
         ctunnel++;
      }
   }

   set_tunnel(st->ca_di_tunnel, st->camera, 70, st->video_render2, 90);

   configure_camera(st);


      {
         OMX_CONFIG_DISPLAYREGIONTYPE current_region;
         
         memset( &current_region, 0, sizeof( OMX_CONFIG_DISPLAYREGIONTYPE ) );
         current_region.nSize = sizeof( OMX_CONFIG_DISPLAYREGIONTYPE );
         current_region.nVersion.nVersion = OMX_VERSION;
         current_region.nPortIndex = 90;
         error = OMX_GetConfig( ILC_GET_HANDLE( st->video_render2 ), OMX_IndexConfigDisplayRegion, 
               &current_region);
         vc_assert( error == OMX_ErrorNone );

         printk("--- Display Regsion---\n" );
         printk("port idx = %u\n", (unsigned int)current_region.nPortIndex );
         printk("VC03 size = %u (%d)\n", (unsigned int)current_region.nSize, sizeof( OMX_CONFIG_DISPLAYREGIONTYPE ) );
         printk("display_num = %u\n", (unsigned int)current_region.num );
         printk("fullscreen = %d\n", current_region.fullscreen );
         printk("dest_rect = %d, %d, %d, %d\n", current_region.dest_rect.x_offset, 
               current_region.dest_rect.y_offset, current_region.dest_rect.width, current_region.dest_rect.height);
         printk("src_rect = %d, %d, %d, %d\n", current_region.src_rect.x_offset, 
               current_region.src_rect.y_offset, current_region.src_rect.width, current_region.src_rect.height);
         printk("noaspect = %d\n", current_region.noaspect );
         printk("mode = %d\n", current_region.mode );
         printk("pixel_x = %u, pixel_y = %u\n", (unsigned int)current_region.pixel_x, 
               (unsigned int)current_region.pixel_y );
         printk("layer = %d\n", (int)current_region.layer );
      }   

   local_printf("playing video/audio... \n");
   local_printf("hit return to terminate...\n");
   //if (st->video_stream_present ||st->audio_stream_present  )
#if 0
   {

#if defined( __KERNEL__ )
	mdelay(PLAYING_LENGTH_MSEC);
#else
	getchar();
#endif

  	ilclient_debug_output("terminate playing\n");
	 // now transition all components to idle and then loaded
	ilclient_state_transition(st->list,OMX_StateIdle);
	ilclient_state_transition(st->list,OMX_StateLoaded);
	ilclient_debug_output("ilclient_state_transition\n");

	// teardown tunnels
#if CAMERA_PREVIEW_RENDERER
	ilclient_teardown_tunnels(st->ca_di_tunnel);
#endif

//	ilclient_teardown_tunnels(st->tunnels);
	ilclient_teardown_tunnels(st->ctunnels);
	ilclient_debug_output("ilclient_teardown_tunnels\n");

	// delete components
	ilclient_cleanup_components(st->list);
	ilclient_debug_output( "ilclient_cleanup_components\n");
	
	error = OMX_Deinit();
	ilclient_debug_output( "OMX_Deinit() %d\n",error);

	ilclient_destroy(st->client);
 	local_printf("finish playing\n");
	
  }
#endif  
   return 0;
}

static int32_t stop_cam(CAM_STATE_T *st)
{
   OMX_ERRORTYPE error;

  	ilclient_debug_output("terminate playing\n");
	 // now transition all components to idle and then loaded
	ilclient_state_transition(st->list,OMX_StateIdle);
	ilclient_state_transition(st->list,OMX_StateLoaded);
	ilclient_debug_output("ilclient_state_transition\n");

	// teardown tunnels
#if CAMERA_PREVIEW_RENDERER
	ilclient_teardown_tunnels(st->ca_di_tunnel);
#endif

//	ilclient_teardown_tunnels(st->tunnels);
	ilclient_teardown_tunnels(st->ctunnels);
	ilclient_debug_output("ilclient_teardown_tunnels\n");

	// delete components
	ilclient_cleanup_components(st->list);
	ilclient_debug_output( "ilclient_cleanup_components\n");
	
	error = OMX_Deinit();
	ilclient_debug_output( "OMX_Deinit() %d\n",error);

	ilclient_destroy(st->client);
 	local_printf("finish playing\n");

   return 0;
   
}

void vc_do_cam(int display, int start)
{
  OMX_ERRORTYPE error;

  if( start == 1 )
  {
      memset(&cst, 0, sizeof(cst));

      cst.region2.nVersion.nVersion = OMX_VERSION;
      cst.region2.nSize = sizeof(cst.region2);
      cst.region2.set = OMX_DISPLAY_SET_NUM | OMX_DISPLAY_SET_FULLSCREEN | 
                        OMX_DISPLAY_SET_TRANSFORM | OMX_DISPLAY_SET_LAYER | 
                        OMX_DISPLAY_SET_DEST_RECT | OMX_DISPLAY_SET_MODE;
      cst.region2.num = display;
      cst.region2.nPortIndex = 90;
      cst.region2.transform = OMX_DISPLAY_ROT0;
      cst.region2.fullscreen = 0;
      cst.region2.dest_rect.x_offset = 400;
      cst.region2.dest_rect.y_offset = 0;
      cst.region2.dest_rect.width = 400;
      cst.region2.dest_rect.height = 240;
      cst.region2.mode = OMX_DISPLAY_MODE_FILL;
      cst.region2.layer = 3;

      cst.focus_mode = OMX_IMAGE_FocusControlHyperfocal;
      cst.focus_position = 215;  // this is a "normal" hyperfccal position.
      cst.capture_width = 320;//320;
      cst.capture_height = 240;//240;
      cst.camplus_id = 'CAMC';
      cst.camera_num = 0;
      //strcpy(cst.isp_tuner, "vb6850");

//   cst.newscale = 
      cst.scale = 1 << 16;
//   cst.newplaymode = cst.playmode = OMX_PLAYMODE_NORMAL;

      error = OMX_Init();
      cst.client = ilclient_init();
      open_cam(&cst);
  }
  else
  {
     /* tear down */
     stop_cam( &cst );
  }
#if defined(__KERNEL__)
 // os_delay(10000);
#endif
}
