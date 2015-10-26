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

FILE DESCRIPTION
Camcorder Host Application.
=============================================================================*/

#if defined( __KERNEL__ )
#include <linux/string.h>
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
#include "vcilclient.h"
#include "vcilplatform.h"

#define INIT_CONTENTURI_MAXLEN 256

#if defined(__KERNEL__)
#define FILENAME_MAX 255
#else
#define vc_gencmd libomx_gencmd
#endif 

#define CMDSTR_MAX 20

typedef struct {
   OMX_PARAM_CONTENTURITYPE uri;
   OMX_U8 uri_data[INIT_CONTENTURI_MAXLEN];
} __attribute__ ((packed)) INIT_OMX_CONTENTURI_TYPE_T;

typedef enum 
  {
    digicam_start = 0,
    digicam_viewfinder = 1,
    digicam_capture = 2,
    digicam_stabilzation = 3,
    digicam_display = 4,
    digicam_zoom = 5,
    digicam_end = 6,

    digicam_max_op,

  } digicam_op_t;
  
int  vc_do_digicam(char* cmd, char* param);
int  vc_do_digicam2(char* cmdstr);
#if !defined( __KERNEL__ )
int  vc_do_digicam3(void);
#endif
int  digicam_setup(void);
int  digicam_init(COMPONENT_T *list[]);
void digicam_view(TUNNEL_T tunnel1[]);
void digicam_view_change_zoom(TUNNEL_T tunnel[], int32_t zoom);
void digicam_view_toggle_stabilization(COMPONENT_T *camera, OMX_BOOL state);
void digicam_view_activate_capture(TUNNEL_T tunnel[], char* filename);

#define ilclient_debug_output OMX_DEBUG

/*
 *
 */
ILCLIENT_T *client = NULL;
COMPONENT_T write_still,  camera, display, read_still, resize;
int initialized = 0;
OMX_PARAM_U32TYPE camplus_id;
OMX_BOOL stabilization = OMX_FALSE;

TUNNEL_T tunnel1[] = 
  { 
    {&camera, 70, &display, 10},
    {&camera, 71, &write_still, 30},
    {NULL, 0, NULL, 0}
  };
COMPONENT_T *list[] = {&camera, &write_still, &display, &read_still, &resize, NULL};

/*
 * 
 */
int vc_do_digicam2(char* cmdstr)
{
  // simple parsing, assume one space delimited "command_parameter" format
  char* param = strchr(cmdstr, ' ');
  if(param)
    {
      *param = 0;
      return vc_do_digicam(cmdstr, param + 1); 
    }

  return vc_do_digicam(cmdstr, NULL); 
}

/*
 * 
 */
#if !defined( __KERNEL__ )

int vc_do_digicam3(void)
{
  char cmdstr[CMDSTR_MAX + FILENAME_MAX + 1];
  
  vc_do_digicam("start", NULL);
  do
    {
      putchar('>');
      cmdstr[0] = 0;
      gets(cmdstr);     
      vc_do_digicam2(cmdstr);
    }
  while(0 != strncmp(cmdstr, "stop", 4));

  return 0;
}

#endif // __KERNEL__

/*
 *
 */
int vc_do_digicam(char* cmd, char* param)
{
  int error;
  
  if(0 == strncmp(cmd, "start", CMDSTR_MAX) ||
     0 == strncmp(cmd, "on", CMDSTR_MAX))
    {
      if(0 != digicam_setup())
	{
	  return -1;
	}
      
      digicam_view(tunnel1);
      
    } 
  else if(0 == strncmp(cmd, "view", CMDSTR_MAX))
    {
      digicam_view(tunnel1);
    }
  else if(0 == strncmp(cmd, "stab", CMDSTR_MAX))
    {
      stabilization = !stabilization;
      digicam_view_toggle_stabilization(&camera, stabilization);      
    }
  else if(0 == strncmp(cmd, "zoom", CMDSTR_MAX))
    {
      int zoom;
      if(NULL==param || 
	 1 != sscanf(param, "%d", &zoom))
	{
	  ilclient_debug_output("invalid command parameters:  %s %s\n", cmd, param);
	  return -1;
	}
      digicam_view_change_zoom(tunnel1, zoom);
    }
  else if((0 == strncmp(cmd, "capture", CMDSTR_MAX)) ||
	  (0 == strncmp(cmd, "cap", CMDSTR_MAX))
	  )
    {      
      if(NULL == param)						
	{
	  ilclient_debug_output("invalid command: %s\n", cmd);
	}
      else	
	{
	  digicam_view_activate_capture(tunnel1, param);
	}
    }
  else if(0 == strncmp(cmd, "stop", CMDSTR_MAX))
    {
      initialized = 0;
      
	// tear down tunnels
      ilclient_teardown_tunnels(tunnel1);
      //ilclient_teardown_tunnels(tunnel2);
      
      // delete components
      ilclient_cleanup_components(list);
      
      error = OMX_SetupDisptask(0, 0, 0);
      vc_assert(error == OMX_ErrorNone);
      
      error = OMX_SetupCamPools(camplus_id.nU32,
				0, 0, 0,    //Hi res pools
				0, 0, 0,    //Lo res pools
				0, 0, 0
				);
      vc_assert(error == OMX_ErrorNone);
      
      error = OMX_Deinit();
      vc_assert(error == OMX_ErrorNone);
      
      ilclient_destroy(client);
      client = NULL;
    }
  else
    {
      ilclient_debug_output("invalid command: digicam %s\n", cmd);
    }
      
  
  return 0;
}

/*
 *
 */ 
int digicam_setup(void)
{
  OMX_ERRORTYPE error;
  //TUNNEL_T tunnel2[] = {{&camera, 71, &write_still, 30},
  //                     {NULL, 0, NULL, 0}};
  //  TUNNEL_T tunnel3[] = {{&read_still, 40, &resize, 60},
  //			{&resize, 61, &display, 10},
  //			{NULL, 0, NULL, 0}
  //};
  char response[128];

  if(initialized)
    {
      ilclient_debug_output("error: digicam already initialized.\n");
      return -1;
    }

  // initialize OpenMax
  error = OMX_Init();
  vc_assert(error == OMX_ErrorNone);

  // enable the display output
  vc_gencmd(response, sizeof(response), "display_control 0 power=2 backlight=2");
  vc_gencmd(response, sizeof(response), "display_control 2 power=2 mode=0 svideo=1 width=320 height=240");

  //
  //  vc_gencmd( response, 128, "set_vll_dir /mfs/digicam_il" );  
  
  client = ilclient_init();
  
  error = OMX_SetupDisptask(2, 1280, 720);
  vc_assert(error == OMX_ErrorNone);
  
  if(0 != digicam_init(list))
    {
      return -1;
    }
  
  // setup camera pool
  camplus_id.nSize = sizeof(OMX_PARAM_U32TYPE);
  camplus_id.nVersion.nVersion = OMX_VERSION;
  camplus_id.nPortIndex = OMX_ALL;
  //Get camplus ID for this instance (we could set it and use that name instead,
  //but we only have one instance, so use the default)
  error = OMX_GetParameter(camera.comp, OMX_IndexParamCameraCamplusId, &camplus_id);
  vc_assert(error == OMX_ErrorNone);
  
  error = OMX_SetupCamPools(camplus_id.nU32,
			    3, 2000, 1500, //Hi res pools
			    0, 0,    0,    //Lo res pools - use hi res pools
			    2, 2000, 1500
			    );
  vc_assert(error == OMX_ErrorNone);
  
  initialized = 1;

  return 0;
}

int digicam_init(COMPONENT_T *list[])
{
   INIT_OMX_CONTENTURI_TYPE_T loc;
   char filename[]="/mfs/sd/captured.jpg";
   OMX_ERRORTYPE error;
   OMX_CONFIG_DISPLAYREGIONTYPE region;
   OMX_PARAM_RESIZETYPE conf;
   
   COMPONENT_T *camera=list[0];
   COMPONENT_T *write_still=list[1];
   COMPONENT_T *display=list[2];
   COMPONENT_T *read_still=list[3];
   COMPONENT_T *resize=list[4];

   // setup write_still component
   if(0 != ilclient_create_component(client, write_still, "write_still", 0, 0, 0))
     {
       ilclient_debug_output("digicam_init error\n");
       return -1;
     }

   memset(&loc, 0, sizeof(INIT_OMX_CONTENTURI_TYPE_T));
   loc.uri.nVersion.nVersion = OMX_VERSION;
   strcpy((char *) loc.uri.contentURI, filename);
   loc.uri.nSize = sizeof(OMX_PARAM_CONTENTURITYPE) + strlen(filename);

   error = OMX_SetParameter(write_still->comp, OMX_IndexParamContentURI, &loc);
   vc_assert(error == OMX_ErrorNone);

   // setup display component
   if(0 != ilclient_create_component(client, display, "display", 0, 0, 0))
     {
     }

   region.nSize = sizeof(OMX_CONFIG_DISPLAYREGIONTYPE);
   region.nVersion.nVersion = OMX_VERSION;
   region.set = OMX_DISPLAY_SET_NUM | OMX_DISPLAY_SET_FULLSCREEN | OMX_DISPLAY_SET_MODE | OMX_DISPLAY_SET_TRANSFORM;
   region.num = 0;
   region.fullscreen = 1;
   region.mode = OMX_DISPLAY_MODE_LETTERBOX;
   region.transform = OMX_DISPLAY_ROT0;
   error = OMX_SetConfig(display->comp, OMX_IndexConfigDisplayRegion, &region);
   vc_assert(error == OMX_ErrorNone);

   // setup camera component
   ilclient_create_component(client, camera, "camera", 0, 0, 0);

   //Setup component to use ILCamPool image pool
   {
      OMX_CONFIG_BOOLEANTYPE param;
      param.nSize = sizeof(OMX_CONFIG_BOOLEANTYPE);
      param.nVersion.nVersion = OMX_VERSION;
      param.bEnabled = OMX_TRUE;
      error = OMX_SetParameter(camera->comp, OMX_IndexParamUseCameraPool, &param);
      vc_assert(error == OMX_ErrorNone);
   }

   // select stills mode from CamPlus
   {
      OMX_PARAM_CAMERAMODETYPE camera_mode;
      memset(&camera_mode, 0, sizeof(OMX_PARAM_CAMERAMODETYPE));
      camera_mode.nSize = sizeof(OMX_PARAM_CAMERAMODETYPE);
      camera_mode.nVersion.nVersion = OMX_VERSION;
      camera_mode.eMode = OMX_CAMERAMODETYPE_STILLS;
      error = OMX_SetParameter(camera->comp, OMX_IndexParamCameraMode, &camera_mode);
      vc_assert(error == OMX_ErrorNone);
   }

   // setup read_still component
   ilclient_create_component(client, read_still, "read_still", 0, 0, 0);

   // setup resize component
   ilclient_create_component(client, resize, "resize", 0, 0, 0);

   memset(&conf, 0, sizeof(OMX_PARAM_RESIZETYPE));
   conf.nSize = sizeof(OMX_PARAM_RESIZETYPE);
   conf.nVersion.nVersion = OMX_VERSION;
   conf.nPortIndex = 61;
   conf.eMode = OMX_RESIZE_DISPLAY;
   conf.bPreserveAspectRatio = OMX_TRUE;
   conf.bAllowUpscaling = OMX_FALSE;

   error = OMX_SetParameter(resize->comp, OMX_IndexParamResize, &conf);
   vc_assert(error == OMX_ErrorNone);

   //Disable all ports
   ilclient_disable_port(display, 10); // video input port
   ilclient_disable_port(display, 11); // clock port
   ilclient_disable_port(camera, 70); // viewfinder port
   ilclient_disable_port(camera, 71); // capture port
   ilclient_disable_port(camera, 72); // clock port
   ilclient_disable_port(write_still, 30); // encoder input
   ilclient_disable_port(read_still, 40); // decoder input
   ilclient_disable_port(resize, 60); // resize input
   ilclient_disable_port(resize, 61); // resize output
   
   return 0;
}

void digicam_view(TUNNEL_T tunnel1[])
{
   OMX_ERRORTYPE error;
   int ret;
   OMX_PARAM_PORTDEFINITIONTYPE param;
   COMPONENT_T *camera = tunnel1[0].source;
   COMPONENT_T *display = tunnel1[0].sink;
   COMPONENT_T *write_still = tunnel1[1].sink;
   COMPONENT_T *list[] = {camera, display, write_still, NULL};
   TUNNEL_T *tunnel_ptr;

   memset(&param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
   param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
   param.nVersion.nVersion = OMX_VERSION;
   param.nPortIndex = 70;
   param.eDir = OMX_DirOutput;
   param.eDomain = OMX_PortDomainImage;
   param.format.video.eCompressionFormat = OMX_IMAGE_CodingUnused;
   param.format.video.eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;
   param.format.video.nFrameWidth = param.format.video.nStride = 800;
   param.format.video.nFrameHeight = 480;
   param.format.video.nSliceHeight = 16;
   param.nBufferCountActual = 1;

   error = OMX_SetParameter(camera->comp, OMX_IndexParamPortDefinition, &param);
   vc_assert(error == OMX_ErrorNone);

   param.nPortIndex = 71;
   param.format.video.nFrameWidth = param.format.video.nStride = 1600;
   param.format.video.nFrameHeight = 960;
   param.format.video.nSliceHeight = 16;
   param.nBufferCountActual = 1;
   error = OMX_SetParameter(camera->comp, OMX_IndexParamPortDefinition, &param);
   vc_assert(error == OMX_ErrorNone);

   {
      OMX_PARAM_SENSORMODETYPE sensor_mode;

      //need to set mode when both camera outputs are disabled
      memset(&sensor_mode, 0, sizeof(OMX_PARAM_SENSORMODETYPE));
      sensor_mode.nSize = sizeof(OMX_PARAM_SENSORMODETYPE);
      sensor_mode.nVersion.nVersion = OMX_VERSION;
      sensor_mode.nFrameRate=0;
      sensor_mode.nPortIndex=OMX_ALL;
      sensor_mode.bOneShot=OMX_TRUE;
      error = OMX_SetParameter(camera->comp, OMX_IndexParamCommonSensorMode, &sensor_mode);
      vc_assert(error == OMX_ErrorNone);
   }

   // setup capture port to capture
   {
      OMX_CONFIG_BOOLEANTYPE capturing;
      memset(&capturing, 0, sizeof(OMX_CONFIG_BOOLEANTYPE));
      capturing.nSize = sizeof(OMX_CONFIG_BOOLEANTYPE);
      capturing.nVersion.nVersion = OMX_VERSION;
      capturing.bEnabled = OMX_FALSE;
      error = OMX_SetParameter(camera->comp, OMX_IndexConfigCapturing, &capturing);
      vc_assert(error == OMX_ErrorNone);
   }

   // create tunnels between components
   ilclient_disable_port(camera, 70); // viewfinder port
   ilclient_disable_port(camera, 71); // capture port
   ilclient_disable_port(camera, 72); // clock port

   ret = 0;
   tunnel_ptr = tunnel1;
   while (tunnel_ptr->source!=NULL)
   {
      ret |= ilclient_setup_tunnel(tunnel_ptr, 0, PLATFORM_EVENTGROUP_SUSPEND);
      tunnel_ptr++;
   }
   vc_assert(!ret);

   if (!ret)
   {
      // transition to executing
      ilclient_state_transition(list, OMX_StateExecuting);
   }

}

#define ZOOM_STEP 0x2000 //step size of +/- x0.125
void digicam_view_change_zoom(TUNNEL_T tunnel[], int32_t param1)
{
   OMX_CONFIG_SCALEFACTORTYPE zoom_params;
   OMX_S32 zoom_factor;
   OMX_ERRORTYPE err;
// static OMX_S32 zoom_value=0x10000;

   zoom_factor = (param1 > 0) ? -ZOOM_STEP : ZOOM_STEP;

   memset(&zoom_params, 0, sizeof(OMX_CONFIG_SCALEFACTORTYPE));
   zoom_params.nSize = sizeof(OMX_CONFIG_SCALEFACTORTYPE);
   zoom_params.nVersion.nVersion = OMX_VERSION;
   zoom_params.nPortIndex = OMX_ALL;

   //Could read value, but there are rounding errors involved, so use local copy.
   err = OMX_GetConfig(tunnel[0].source->comp, OMX_IndexConfigCommonDigitalZoom, &zoom_params);
   vc_assert(err==OMX_ErrorNone);

   //zoom_value += zoom_factor;

   zoom_params.xWidth += zoom_factor;

   //Could recompute for xHeight, but it's going to be the same as xWidth (must be for Camplus)
   zoom_params.xHeight = zoom_params.xWidth;

   err = OMX_SetConfig(tunnel[0].source->comp, OMX_IndexConfigCommonDigitalZoom, &zoom_params);
// if(err==OMX_ErrorBadParameter)
// { //Invalid value, so reread to get the current one
//  err = OMX_GetConfig(tunnel[0].source->comp, OMX_IndexConfigCommonDigitalZoom, &zoom_params);
//  zoom_value = zoom_params.xWidth;
// }
   vc_assert(err==OMX_ErrorNone || err==OMX_ErrorBadParameter);
}

void digicam_view_toggle_stabilization(COMPONENT_T *camera, OMX_BOOL state)
{
   OMX_ERRORTYPE error;
   OMX_CONFIG_FRAMESTABTYPE stabilization;

   memset(&stabilization, 0, sizeof(OMX_CONFIG_FRAMESTABTYPE));
   stabilization.nSize = sizeof(OMX_CONFIG_FRAMESTABTYPE);
   stabilization.nVersion.nVersion = OMX_VERSION;
   stabilization.nPortIndex = OMX_ALL;
   stabilization.bStab = state;
   error = OMX_SetParameter(camera->comp, OMX_IndexConfigCommonFrameStabilisation, &stabilization);
   vc_assert(error == OMX_ErrorNone);
}

void digicam_view_activate_capture(TUNNEL_T tunnel[], char* filename)
{
   COMPONENT_T *camera = tunnel[0].source;
   COMPONENT_T *write_still = tunnel[1].sink;
   int success;
   OMX_ERRORTYPE error;
   INIT_OMX_CONTENTURI_TYPE_T loc;
   COMPONENT_T *list[] = { write_still, NULL};

   // set capture file name
   ilclient_disable_tunnel(tunnel + 1);

   memset(&loc, 0, sizeof(INIT_OMX_CONTENTURI_TYPE_T));
   loc.uri.nVersion.nVersion = OMX_VERSION;
   strcpy((char *) loc.uri.contentURI, filename);
   loc.uri.nSize = sizeof(OMX_PARAM_CONTENTURITYPE) + strlen(filename);
   error = OMX_SetParameter(write_still->comp, OMX_IndexParamContentURI, &loc);
   vc_assert(error == OMX_ErrorNone);

   error = ilclient_enable_tunnel(tunnel  + 1);
   vc_assert(error == OMX_ErrorNone);
   ilclient_state_transition(list, OMX_StateExecuting);

   // setup capture port to capture
   OMX_CONFIG_BOOLEANTYPE capturing;
   memset(&capturing, 0, sizeof(OMX_CONFIG_BOOLEANTYPE));
   capturing.nSize = sizeof(OMX_CONFIG_BOOLEANTYPE);
   capturing.nVersion.nVersion = OMX_VERSION;
   capturing.bEnabled = OMX_TRUE;
   error = OMX_SetParameter(camera->comp, OMX_IndexConfigCapturing, &capturing);
   vc_assert(error == OMX_ErrorNone);
   
   success = ilclient_wait_for_event(write_still, OMX_EventBufferFlag,
                                     tunnel[1].sink_port, 0, 1, 0, ILCLIENT_BUFFER_FLAG_EOS, 5000);
   vc_assert(success == 0);

}
