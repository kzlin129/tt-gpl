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

#include <stdarg.h>
//#include <linux/broadcom/vc.h>
#include <linux/broadcom/vc03/vcilcs_intern.h>
#include <linux/broadcom/vc03/vcomx.h>
#include <linux/broadcom/vc03/vcilclient.h>
#include <linux/broadcom/vc03/vcilplatform.h>
#include <linux/broadcom/vc03/dispmanx_types.h>
//#include <linux/broadcom/vc03/vc_dispmanx.h>


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
   OMX_PARAM_CONTENTURITYPE uri;
   OMX_U8 uri_data[ILTEST_CONTENTURI_MAXLEN];
} __attribute__((packed)) PLAYBACK_OMX_CONTENTURI_TYPE_T;


typedef struct {
   void *text_userdata;
#define MAX_LYRICS_LENGTH 128
   uint8_t lyrics_text[MAX_LYRICS_LENGTH+1];
   void *notify_userdata;
   void *metadata_userdata;
   void* egl_callback;
   void* egl_userdata;
   void* egl_display;
   int* egl_surfaces;
   uint32_t egl_num_surfaces;

   // fields protected by sema
   PLAYBACK_OMX_CONTENTURI_TYPE_T uri;
   char error_msg[64];
   OMX_CONFIG_VISUALISATIONTYPE vistype;
   OMX_CONFIG_DISPLAYREGIONTYPE region;
   OMX_PLAYMODETYPE newplaymode;
   OMX_PARAM_IMAGEPOOLSIZETYPE image_pool;
   OMX_S32 newscale;
   uint32_t single_steps;
   OMX_TICKS newposition;
   int32_t sync_offset;
   OMX_BUFFERHEADERTYPE *egl_empty_list;
   OMX_BUFFERHEADERTYPE *egl_pending_list;   

   // internal state 
   OMX_PLAYMODETYPE playmode;
   OMX_S32 scale;
   int eos;
   int eos_required;
   int audio_stream_present;
   int video_stream_present;
   int text_stream_present;
   int loop_count;
   int audio_track;
   int seekable;
   int vr_port;

   // pointers to OpenMAX elements
   ILCLIENT_T *client;
   COMPONENT_T *read_media;
   COMPONENT_T *audio_decode;
   COMPONENT_T *audio_render;
   COMPONENT_T *video_decode;
   COMPONENT_T *video_scheduler;
   COMPONENT_T *video_render;
   COMPONENT_T *clock;
   COMPONENT_T *visualisation;
   COMPONENT_T *text_scheduler;
   COMPONENT_T *image_fx;

   COMPONENT_T *list[11];
   TUNNEL_T tunnels[8];
   TUNNEL_T ctunnels[6];

} PLAYBACK_STATE_T;




#define ilclient_debug_output OMX_DEBUG

static int32_t stop_play(PLAYBACK_STATE_T *st)
{
   OMX_ERRORTYPE error;

   if (st->video_stream_present ||st->audio_stream_present  )
   {
      ilclient_debug_output("terminate playing\n");
      // now transition all components to idle and then loaded
      ilclient_state_transition(st->list,OMX_StateIdle);
      ilclient_state_transition(st->list,OMX_StateLoaded);
      ilclient_debug_output("ilclient_state_transition\n");

      // teardown tunnels
      ilclient_teardown_tunnels(st->tunnels);
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

   return 0;
}


static int32_t open_file(PLAYBACK_STATE_T *st)
{
   OMX_CONFIG_DISPLAYREGIONTYPE region;
   OMX_CONFIG_VISUALISATIONTYPE vistype;
   OMX_ERRORTYPE error;
   TUNNEL_T *tunnel;
   COMPONENT_T **comp;
   int i, streams;

   region = st->region;
   vistype = st->vistype;

   if(ilclient_create_component(st->client, &st->read_media, "read_media", 0, 0, 0) < 0)
   {
      strcpy(st->error_msg, "Failed to load components");
      return -1;
   }

   st->list[0] = st->read_media;

   comp = st->list+1;
   tunnel = st->tunnels;

   // set file location
   // change state to idle
   // wait for an audio stream to be advertised
   error = OMX_SetParameter(ILC_GET_HANDLE(st->read_media), OMX_IndexParamContentURI, &st->uri);
   if(error != OMX_ErrorNone)
   {
      // error case  
      strcpy(st->error_msg, "Unreadable file");
      ilclient_change_component_state(st->read_media, OMX_StateLoaded);
      return -1;
   }

   if(ilclient_change_component_state(st->read_media, OMX_StateIdle) != 0)
   {
      ilclient_change_component_state(st->read_media, OMX_StateLoaded);
      return -1;
   }

   if(ilclient_wait_for_event(st->read_media, OMX_EventPortSettingsChanged, 110, 0, -1, 1,
      ILCLIENT_PARAMETER_CHANGED | ILCLIENT_EVENT_ERROR, 
      PLATFORM_EVENTGROUP_SUSPEND) != 0)
   {
      ilclient_change_component_state(st->read_media, OMX_StateLoaded);
      return -1;
   }

   // create audio decode
   ilclient_create_component(st->client, &st->audio_decode, "audio_decode", 0, 0, 0);
   *comp = st->audio_decode;
   ilclient_disable_port(st->audio_decode, 121);
   set_tunnel(tunnel, st->read_media, 110, st->audio_decode, 120);
   // if this fails, then bad codec or no audio stream is present
   if(ilclient_setup_tunnel(tunnel, st->audio_track, 0) != 0)
   {
      // delete audio_decode component
      ilclient_cleanup_components(comp);
      st->audio_decode = NULL; 
   }
   else
   {
      st->audio_stream_present = 1;
      tunnel++;
      comp++;
   }

   // we know that all port parameter changed messages are sent at once
   // create video decode
   ilclient_create_component(st->client, &st->video_decode, "video_decode", 0, 0, 0);
   *comp = st->video_decode;
   ilclient_disable_port(st->video_decode, 131);

   // configure decoder image pool size, if required
   if ( st->image_pool.nSize != 0 )
   {
      error = OMX_SetConfig(ILC_GET_HANDLE(st->video_decode), OMX_IndexParamImagePoolSize, &st->image_pool);
      vc_assert(error == OMX_ErrorNone);
   }

   set_tunnel(tunnel, st->read_media, 111, st->video_decode, 130);
   // if this fails, then either bad codec, or no video stream available
   if(ilclient_setup_tunnel(tunnel, 0, PLATFORM_EVENTGROUP_SUSPEND) != 0)
   {
      // delete video_decode component
      ilclient_cleanup_components(comp);
      st->video_decode = NULL;
   }
   else
   {
      st->video_stream_present = 1;
      tunnel++;
      comp++;
   }

   if (!st->video_stream_present && !st->audio_stream_present)
   {
      // we appear to have nothing we can play
      ilclient_change_component_state(st->read_media, OMX_StateLoaded);
      strcpy(st->error_msg, "No valid streams present");
      return -1;
   }

   // create text scheduler
   ilclient_create_component(st->client, &st->text_scheduler, "text_scheduler", 0, 1, 0);
   *comp = st->text_scheduler;
   ilclient_disable_port(st->text_scheduler, 151);
   ilclient_disable_port(st->text_scheduler, 152);
   set_tunnel(tunnel, st->read_media, 112, st->text_scheduler, 150);
   // if this fails, then either bad text type, or no text stream available
   if(ilclient_setup_tunnel(tunnel, 0, PLATFORM_EVENTGROUP_SUSPEND) != 0)
   {
      // delete text scheduler component
      ilclient_cleanup_components(comp);
      st->text_scheduler = NULL;
   }
   else
   {
      st->text_stream_present = 1;
      tunnel++;
      comp++;
   }

   st->eos_required = st->audio_stream_present + st->video_stream_present + st->text_stream_present;

   // find out whether we can do seeks or ff/rew
   {
      OMX_TIME_CONFIG_SEEKMODETYPE param;
      param.nSize = sizeof(OMX_TIME_CONFIG_SEEKMODETYPE);
      param.nVersion.nVersion = OMX_VERSION;
      param.eType = OMX_TIME_SeekModeFast;
      error = OMX_SetParameter(ILC_GET_HANDLE(st->read_media), OMX_IndexConfigTimeSeekMode, &param);
      if (error == OMX_ErrorBadParameter)
         st->seekable = 0;
      else if (error == OMX_ErrorNone)
         st->seekable = 1;
      else
         vc_assert(0);
   }

   // create remaining components, depending on what streams we have available
   if(!st->video_stream_present)
   {
      ilclient_create_component(st->client, &st->visualisation, "visualisation", 0, 0, 0);
      *comp++ = st->visualisation;
      ilclient_disable_port(st->visualisation, 140);
      ilclient_disable_port(st->visualisation, 141);
   }

   if(st->audio_stream_present)
   {
      ilclient_create_component(st->client, &st->audio_render, "audio_render", 0, 0, 0);
      *comp++ = st->audio_render;
      ilclient_disable_port(st->audio_render, 100);
   }

   ilclient_create_component(st->client, &st->video_scheduler, "video_scheduler", 0, 0, 0);
   *comp++ = st->video_scheduler;
   ilclient_disable_port(st->video_scheduler, 10);
   ilclient_disable_port(st->video_scheduler, 11);

   if( st->egl_num_surfaces )
   {
      st->vr_port = 220;
   }
   else
   {
      st->vr_port = 90;
   }

   ilclient_create_component(st->client, &st->video_render, st->egl_num_surfaces ? "video_egl_render" : "video_render" , 0, 1, 0);
   *comp++ = st->video_render;
   ilclient_disable_port(st->video_render, st->vr_port );

   if( st->egl_num_surfaces )
   {
      ilclient_disable_port(st->video_render, st->vr_port + 1);
   }

   ilclient_create_component(st->client, &st->clock, "clock", 0, 0, 0);
   *comp++ = st->clock;

   // setup display region if already set
   if (region.set)
   {
      error = OMX_SetConfig(ILC_GET_HANDLE(st->video_render), OMX_IndexConfigDisplayRegion, &region);
      vc_assert(error == OMX_ErrorNone);
   }

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
      cstate.nWaitMask = (st->audio_stream_present && st->video_stream_present ? OMX_CLOCKPORT0|OMX_CLOCKPORT1 : OMX_CLOCKPORT0);
      error = OMX_SetConfig(ILC_GET_HANDLE(st->clock), OMX_IndexConfigTimeClockState, &cstate);
      vc_assert(error == OMX_ErrorNone);

      sparam.nSize = sizeof(OMX_TIME_CONFIG_SCALETYPE);
      sparam.nVersion.nVersion = OMX_VERSION;
      sparam.xScale = st->scale;
      error = OMX_SetConfig(ILC_GET_HANDLE(st->clock), OMX_IndexConfigTimeScale, &sparam);
      vc_assert(error == OMX_ErrorNone);

      ctunnel = st->ctunnels;
      port = param.nStartPortNumber;

      if(st->audio_stream_present)
      {
         set_tunnel(ctunnel, st->clock, port++, st->audio_render, 101);
         ilclient_disable_tunnel(ctunnel++);
      }

      set_tunnel(ctunnel, st->clock, port++, st->video_scheduler, 12);
      ilclient_disable_tunnel(ctunnel++);

      if(!st->video_stream_present)
      {
         set_tunnel(ctunnel, st->clock, port++, st->visualisation, 143);
         ilclient_disable_tunnel(ctunnel++);
      }

      if(st->text_stream_present)
      {
         set_tunnel(ctunnel, st->clock, port++, st->text_scheduler, 152);
         ilclient_disable_tunnel(ctunnel++);
      }

      // disable unused clock ports
      for (i=port; i<param.nStartPortNumber+param.nPorts; i++)
         ilclient_disable_port(st->clock, i);

      // connect up tunnels - no data dependencies
      ctunnel = st->ctunnels;
      while(ctunnel->sink)
      {
         error = OMX_SetupTunnel(ILC_GET_HANDLE(ctunnel->source), ctunnel->source_port, ILC_GET_HANDLE(ctunnel->sink), ctunnel->sink_port);
         vc_assert(error == OMX_ErrorNone);
         ctunnel++;
      }
   }

   if (st->text_stream_present)
   {
      //Pass text_scheduler a buffer
      OMX_PARAM_PORTDEFINITIONTYPE port_settings;
      OMX_BUFFERHEADERTYPE *pBuffer;
      unsigned char *buf;

      memset(&port_settings, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
      port_settings.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
      port_settings.nVersion.nVersion = OMX_VERSION;
      port_settings.nPortIndex = 151;
      port_settings.eDomain = OMX_PortDomainOther;
      port_settings.eDir = OMX_DirOutput;
      port_settings.format.other.eFormat = OMX_OTHER_FormatText;
      port_settings.nBufferCountActual = 1;
      port_settings.nBufferSize = MAX_LYRICS_LENGTH;

      error = OMX_SetParameter(ILC_GET_HANDLE(st->text_scheduler), OMX_IndexParamPortDefinition, &port_settings);
      vc_assert(error == OMX_ErrorNone);
      error = OMX_GetParameter(ILC_GET_HANDLE(st->text_scheduler), OMX_IndexParamPortDefinition, &port_settings);
      vc_assert(error == OMX_ErrorNone);

      error = OMX_SendCommand(ILC_GET_HANDLE(st->text_scheduler), OMX_CommandPortEnable, 151, NULL);
      vc_assert(error == OMX_ErrorNone);

      buf = platform_malloc(MAX_LYRICS_LENGTH, __LINE__, __FILE__);
      error = OMX_UseBuffer(ILC_GET_HANDLE(st->text_scheduler), &pBuffer, 151, NULL, MAX_LYRICS_LENGTH, buf);
      assert(error == OMX_ErrorNone);

      ilclient_wait_for_event(st->text_scheduler, OMX_EventCmdComplete, OMX_CommandPortEnable, 0, 151, 0,
         ILCLIENT_PORT_ENABLED, PLATFORM_EVENTGROUP_SUSPEND);

      ilclient_change_component_state(st->text_scheduler, OMX_StateExecuting);

      ilclient_debug_output("playback_il: FillThisBuffer %p", pBuffer);

      pBuffer->pAppPrivate = NULL;
      pBuffer->pInputPortPrivate = NULL;
      error = OMX_FillThisBuffer(ILC_GET_HANDLE(st->text_scheduler), pBuffer);
      vc_assert(error == OMX_ErrorNone);
   }

   ilclient_change_component_state(st->read_media, OMX_StateExecuting);

   // to get the components to pass a buffer, they both need to be in executing state
   if (st->audio_stream_present)
      ilclient_change_component_state(st->audio_decode, OMX_StateExecuting);
   if (st->video_stream_present)
      ilclient_change_component_state(st->video_decode, OMX_StateExecuting);

   // setup the tunnels to be used
   if(!st->video_stream_present)
   {
      set_tunnel(tunnel++, st->audio_decode, 121, st->visualisation, 140);
      set_tunnel(tunnel++, st->visualisation, 141, st->audio_render, 100);
      set_tunnel(tunnel++, st->visualisation, 142, st->video_scheduler, 10);
   }
   else
   {
      if(st->audio_stream_present)
         set_tunnel(tunnel++, st->audio_decode, 121, st->audio_render, 100);
      set_tunnel(tunnel++, st->video_decode, 131, st->video_scheduler, 10);
   }

   set_tunnel(tunnel, st->video_scheduler, 11, st->video_render, st->vr_port );

   streams = st->video_stream_present + st->audio_stream_present + st->text_stream_present;

   // and wait for the port parameter changed from the decode output ports, then connect up to st->audio_render and st->video_scheduler
   if(st->audio_stream_present)
   {
      if(ilclient_setup_tunnel(st->tunnels+streams, 0, 2000) < 0)
      {
         strcpy(st->error_msg, "Audio codec error");
         return -1;
      }
   }

   if(st->video_stream_present)
   {
      // check if we have interlaced content
      OMX_CONFIG_INTERLACETYPE interlace;

      if(ilclient_wait_for_event(st->video_decode, OMX_EventPortSettingsChanged,
         131, 0, -1, 1, ILCLIENT_PARAMETER_CHANGED | ILCLIENT_EVENT_ERROR, 2000) < 0)
      {
         strcpy(st->error_msg, "Video codec error");
         return -1;
      }

      // insert an image_fx component between video_decode and video_scheduler if we spot interlaced
      // content   
      memset(&interlace, 0, sizeof(OMX_CONFIG_INTERLACETYPE));
      interlace.nSize = sizeof(OMX_CONFIG_INTERLACETYPE);
      interlace.nVersion.nVersion = OMX_VERSION;
      interlace.nPortIndex = 131;
      error = OMX_GetConfig(ILC_GET_HANDLE(st->video_decode), OMX_IndexConfigCommonInterlace, &interlace);
      if(error == OMX_ErrorNone && interlace.eMode != OMX_InterlaceProgressive &&
         ilclient_create_component(st->client, &st->image_fx, "image_fx", 0, 0, 0) >= 0)
      {
#if 1
         OMX_CONFIG_IMAGEFILTERPARAMSTYPE filter;
         // for top field first clips
         memset(&filter, 0, sizeof(OMX_CONFIG_IMAGEFILTERPARAMSTYPE));
         filter.nSize = sizeof(OMX_CONFIG_IMAGEFILTERPARAMSTYPE);
         filter.nVersion.nVersion = OMX_VERSION;
         filter.nPortIndex = 191;
         filter.nNumParams = 1;
         filter.nParams[0] = 3;     
         filter.eImageFilter = OMX_ImageFilterDeInterlaceAdvanced; // OMX_ImageFilterDeInterlaceLineDouble;     
         error = OMX_SetConfig(ILC_GET_HANDLE(st->image_fx), OMX_IndexConfigCommonImageFilterParameters, &filter);
#else
         OMX_CONFIG_IMAGEFILTERTYPE filter;
         memset(&filter, 0, sizeof(OMX_CONFIG_IMAGEFILTERTYPE));
         filter.nSize = sizeof(OMX_CONFIG_IMAGEFILTERTYPE);
         filter.nVersion.nVersion = OMX_VERSION;
         filter.nPortIndex = 191;
         filter.eImageFilter = OMX_ImageFilterDeInterlaceAdvanced;
         error = OMX_SetConfig(ILC_GET_HANDLE(st->image_fx), OMX_IndexConfigCommonImageFilter, &filter);
#endif
         *comp = st->image_fx;

         if(error == OMX_ErrorNone)
         {
            OMX_PARAM_PORTDEFINITIONTYPE portdef;
            OMX_PARAM_IMAGEPOOLSIZETYPE poolsize;
            OMX_TIME_CONFIG_TIMESTAMPTYPE time;

            // we need to make the video decode image pool smaller, so there is space for the
            // image effects component.  To do this, we need to disable the video_decoder, so
            // we also need to seek back to the start of the clip.
            // So stop read_media, flush decoders, seek, disable video_decode, change pool size,
            // then get back to where we were.
            ilclient_change_component_state(st->read_media, OMX_StatePause);
            ilclient_flush_tunnels(st->tunnels, st->audio_stream_present + st->video_stream_present);

            // send the seek back to the start - all handlers should support
            // this, even if they don't support seeking to other parts of the clip.
            memset(&time, 0, sizeof(OMX_TIME_CONFIG_TIMESTAMPTYPE));
            time.nSize = sizeof(OMX_TIME_CONFIG_TIMESTAMPTYPE);
            time.nVersion.nVersion = OMX_VERSION;
            time.nTimestamp = 0;
            error = OMX_SetConfig(ILC_GET_HANDLE(st->read_media), OMX_IndexConfigTimePosition, &time);
            vc_assert(error == OMX_ErrorNone);

            // get the default number of pages
            memset(&poolsize, 0, sizeof(OMX_PARAM_IMAGEPOOLSIZETYPE));
            poolsize.nSize = sizeof(OMX_PARAM_IMAGEPOOLSIZETYPE);
            poolsize.nVersion.nVersion = OMX_VERSION;
            if(OMX_GetParameter(ILC_GET_HANDLE(st->video_decode), OMX_IndexParamImagePoolSize, &poolsize) != OMX_ErrorNone)
               poolsize.num_pages = 12;

            // get the actual video size
            memset(&portdef, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
            portdef.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
            portdef.nVersion.nVersion = OMX_VERSION;
            portdef.nPortIndex = 131;
            if(OMX_GetParameter(ILC_GET_HANDLE(st->video_decode), OMX_IndexParamPortDefinition, &portdef) != OMX_ErrorNone)
            {
               poolsize.width = portdef.format.video.nFrameWidth;
               poolsize.height = portdef.format.video.nFrameHeight;
            }
            else
            {
               // the get parameter call really shouldn't fail, but if so then we assume SD resolution, 720i
               poolsize.width = 720;
               poolsize.height = 576;
            }

            ilclient_disable_tunnel(st->tunnels + st->audio_stream_present);
            ilclient_change_component_state(st->video_decode, OMX_StateIdle);
            ilclient_change_component_state(st->video_decode, OMX_StateLoaded);
            if(OMX_SetParameter(ILC_GET_HANDLE(st->video_decode), OMX_IndexParamImagePoolSize, &poolsize) != OMX_ErrorNone)
               vc_assert(0);

            ilclient_change_component_state(st->video_decode, OMX_StateIdle);
            ilclient_enable_tunnel(st->tunnels + st->audio_stream_present);
            ilclient_change_component_state(st->video_decode, OMX_StateExecuting);
            ilclient_change_component_state(st->read_media, OMX_StateExecuting);

            // now insert image_fx between video_decode and video_scheduler
            set_tunnel(st->tunnels+streams+st->audio_stream_present, st->video_decode, 131, st->image_fx, 190);
            set_tunnel(tunnel+1, st->image_fx, 191, st->video_scheduler, 10);
            ilclient_disable_port(st->image_fx, 191);
         }
         else
         {
            // give up with image_fx, doesn't support deinterlacing
            ilclient_cleanup_components(comp);
         }
      }

      // connect video_decode->video_scheduler/image_fx
      if(ilclient_setup_tunnel(st->tunnels+streams+st->audio_stream_present, 0, 0) < 0)
      {
         strcpy(st->error_msg, "Video codec error");
         return -1;
      }

      // connect image_fx->video_scheduler
      if(st->image_fx && ilclient_setup_tunnel(tunnel+1, 0, 2000) < 0)
      {
         strcpy(st->error_msg, "Image fx error");
         return -1;
      }

      // connect video_scheduler->video_render
      if(ilclient_setup_tunnel(tunnel, 0, 2000) < 0)
      {
         strcpy(st->error_msg, "Bad video display format");
         return -1;
      }
   }
   else
   {
      // connect visualisation->audio_render
      if(ilclient_setup_tunnel(st->tunnels+streams+1, 0, 2000) < 0)
      {
         strcpy(st->error_msg, "Bad audio format");
         return -1;
      }
   }

   // set the visualisation if we have one
   if (st->visualisation && vistype.name[0])
   {
      vistype.nPortIndex = 142;
      error = OMX_SetConfig(ILC_GET_HANDLE(st->visualisation), OMX_IndexConfigVisualisation, &vistype);
      vc_assert(error == OMX_ErrorNone);
   }

   {
      COMPONENT_T *ilist[] = {st->clock, st->video_scheduler, st->video_render, st->image_fx, NULL};
      ilclient_state_transition(ilist, OMX_StateIdle);

      if( st->egl_num_surfaces )
      {
         //Pass egl_render a buffer
         OMX_PARAM_PORTDEFINITIONTYPE port_settings;
         OMX_BUFFERHEADERTYPE *empty_list = NULL;         
         uint32_t count = 0;

         memset(&port_settings, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
         port_settings.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
         port_settings.nVersion.nVersion = OMX_VERSION;
         port_settings.nPortIndex = 221;

         error = OMX_GetParameter(ILC_GET_HANDLE(st->video_render), OMX_IndexParamPortDefinition, &port_settings);
         vc_assert(error == OMX_ErrorNone);

         port_settings.nBufferCountActual = st->egl_num_surfaces;
         port_settings.format.video.pNativeWindow = st->egl_display;

         error = OMX_SetParameter(ILC_GET_HANDLE(st->video_render), OMX_IndexParamPortDefinition, &port_settings);
         vc_assert(error == OMX_ErrorNone);         

         error = OMX_SendCommand(ILC_GET_HANDLE(st->video_render), OMX_CommandPortEnable, 221, NULL);
         vc_assert(error == OMX_ErrorNone);

         ilclient_wait_for_event(st->video_render, OMX_EventCmdComplete, OMX_CommandPortEnable, 0, 221, 0,
            ILCLIENT_PORT_ENABLED, PLATFORM_EVENTGROUP_SUSPEND);

         empty_list = st->egl_empty_list;

         while( empty_list )
         {
            empty_list = empty_list->pAppPrivate;
         }
      }

      ilclient_state_transition(ilist, OMX_StateExecuting);
   }

   if (st->text_stream_present)
   {
      // enable clock->text_render
      ilclient_enable_tunnel(st->ctunnels+2+st->audio_stream_present-st->video_stream_present);
   }

   // enable clock->video_scheduler
   ilclient_enable_tunnel(st->ctunnels+st->audio_stream_present);

   if (st->audio_stream_present)
   {
      // enable clock->audio_render
      ilclient_enable_tunnel(st->ctunnels);
      ilclient_change_component_state(st->audio_render, OMX_StateExecuting);
   }

   if (!st->video_stream_present)
   {
      // enable clock->visualisation
      ilclient_enable_tunnel(st->ctunnels+2);
      ilclient_change_component_state(st->visualisation, OMX_StateExecuting);
   }

   local_printf("playing video/audio... \n");
   
#if defined( __KERNEL__ )
   local_printf("type echo play stop > ... to terminate\n");
#else
   local_printf("hit return to terminate...\n");
   if (st->video_stream_present ||st->audio_stream_present  )
   {

	getchar();
    stop_play(st);
    }
#endif

   return 0;
}

#define DISPMANX_FRAMEBUF_SCREEN_DISPLAY     0
PLAYBACK_STATE_T st;

void vc_do_play(char* filename,int display)
{
   OMX_ERRORTYPE error;

   if(0	==	strncmp(filename,	"stop", 4))
   {
      stop_play( &st );
   }
   else
   {
      char* cptr;
      memset(&st, 0, sizeof(st));
      st.uri.uri.nVersion.nVersion = OMX_VERSION;
      strncpy((char*)st.uri.uri.contentURI, filename, ILTEST_CONTENTURI_MAXLEN-1);
      OMX_DEBUG("sz=%d %d\n", sizeof(OMX_PARAM_CONTENTURITYPE), sizeof(OMX_VERSIONTYPE));
      st.uri.uri.nSize = sizeof(OMX_PARAM_CONTENTURITYPE) + strlen(filename) ;
      st.loop_count = 1;
      st.audio_track = 0;
      cptr = (char*)st.uri.uri.contentURI;
      ilclient_debug_output("playing %s\n", cptr);


      st.region.nVersion.nVersion = OMX_VERSION;
      st.region.nSize = sizeof(st.region);
      st.region.set = OMX_DISPLAY_SET_NUM | OMX_DISPLAY_SET_FULLSCREEN | 
         OMX_DISPLAY_SET_TRANSFORM | OMX_DISPLAY_SET_LAYER;
      st.region.num = display;
      st.region.nPortIndex = 90;
      st.region.transform = OMX_DISPLAY_ROT0;
      st.region.fullscreen = 1;
      st.region.layer = 2;

      st.newscale = st.scale = 1 << 16;
      st.newplaymode = st.playmode = OMX_PLAYMODE_NORMAL;

      error = OMX_Init();
      vc_assert(error == OMX_ErrorNone);
      st.client = ilclient_init();
      open_file(&st);
   }
}
