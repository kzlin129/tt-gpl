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




#include <linux/broadcom/omx/OMX_Broadcom.h>
#include <linux/broadcom/vc03/host_ilcore.h>
#include <linux/broadcom/vc03/vcilclient.h>
#include <linux/broadcom/vc03/vcilplatform.h>
#include <linux/broadcom/vc03/vcos.h>

#if !defined __KERNEL__
#include <linux/broadcom/vc03/vcomx.h>
#define malloc libomx_malloc
#define free   libomx_free
#endif

#define INPUT_BUFFER_ARRIVED 1

static void input_buffer_callback(void *data, COMPONENT_T *comp)
{
   platform_eventgroup_set(data, INPUT_BUFFER_ARRIVED, PLATFORM_EVENTGROUP_OPERATION_OR);
}

void vc_do_stripes(void)
{
   OMX_ERRORTYPE error;
   OMX_PARAM_PORTDEFINITIONTYPE param;
   OMX_CONFIG_DISPLAYREGIONTYPE region;
   int n, j;
   uint32_t i, set;
   COMPONENT_T *st, *list[] = {NULL, NULL};
   ILCLIENT_T *client = ilclient_init();
   OMX_BUFFERHEADERTYPE *buffer, *buffer_list = NULL;
   PLATFORM_EVENTGROUP_T events;

   n = platform_eventgroup_create(&events);
   vc_assert(n == 0);

   ilclient_set_empty_buffer_done_callback(client, input_buffer_callback, &events);

   error = OMX_Init();
   vc_assert(error == OMX_ErrorNone);

   ilclient_create_component(client, &st, "video_render", 0, 1, 1);

   list[0] = st;
   region.nSize = sizeof(OMX_CONFIG_DISPLAYREGIONTYPE);
   region.nVersion.nVersion = OMX_VERSION;
   region.nPortIndex = 90;
   region.set = OMX_DISPLAY_SET_NUM | OMX_DISPLAY_SET_FULLSCREEN | OMX_DISPLAY_SET_TRANSFORM | OMX_DISPLAY_SET_MODE;
   region.num = 0;
   region.fullscreen = 1;
   region.transform = OMX_DISPLAY_ROT0;
   region.mode = OMX_DISPLAY_MODE_LETTERBOX;
   error = OMX_SetConfig(ILC_GET_HANDLE(st), OMX_IndexConfigDisplayRegion, &region);
   vc_assert(error == OMX_ErrorNone);

   memset(&param, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
   param.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
   param.nVersion.nVersion = OMX_VERSION;
   param.nPortIndex = 90;
   param.eDomain = OMX_PortDomainImage;
   param.eDir = OMX_DirInput;
   param.format.image.eCompressionFormat = OMX_IMAGE_CodingUnused;
   param.format.image.eColorFormat = OMX_COLOR_Format16bitRGB565;
   param.format.image.nFrameWidth = 320;
   param.format.image.nFrameHeight = 240;
   param.format.image.nStride = ((param.format.image.nFrameWidth + 15) & ~15)   * 2;
   param.format.image.nSliceHeight = 240;
   param.nBufferCountActual = 2;
   param.nBufferSize = param.format.image.nSliceHeight * param.format.image.nStride;
  
   error = OMX_SetParameter(ILC_GET_HANDLE(st), OMX_IndexParamPortDefinition, &param);
   vc_assert(error == OMX_ErrorNone);

   error = OMX_SendCommand(ILC_GET_HANDLE(st), OMX_CommandStateSet, OMX_StateIdle, NULL);
   vc_assert(error == OMX_ErrorNone);

   for (i=0; i<param.nBufferCountActual; i++)
   {
      unsigned char *buf = malloc(param.nBufferSize);
      printk("buf[%d]=0x%08X\n", i, (int)buf);
      error = OMX_UseBuffer(ILC_GET_HANDLE(st), &buffer_list, 90, buffer_list, param.nBufferSize, buf);
   }

   ilclient_wait_for_event(st, OMX_EventCmdComplete, OMX_CommandStateSet, 0, OMX_StateIdle, 0,
                           ILCLIENT_STATE_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);

   ilclient_change_component_state(st, OMX_StateExecuting);

   for (n=0; n<20; n++)
     {
       unsigned short *p;
       
       if(buffer_list)
	 {
	   buffer = buffer_list;
	   buffer_list = buffer_list->pAppPrivate;
	 }
       else
	 {
	   while((buffer = ilclient_get_input_buffer(st, 90)) == NULL)
	     platform_eventgroup_get(&events, INPUT_BUFFER_ARRIVED, PLATFORM_EVENTGROUP_OPERATION_OR_CONSUME, -1, &set);
	 }
       
       p = (unsigned short *) buffer->pBuffer;
       
       for(i=0; i<(param.format.image.nFrameHeight+15)/16; i++)
	 {
#          define R (0x1f << 11)
#          define G (0x3f << 6)
#          define B (0x1f << 0)
	   int m = i+n;
	   unsigned short col =
	     R * ((m >> 2) & 1) |
	     G * ((m >> 1) & 1) |
	     B * ((m >> 0) & 1) ;
	   
	   for (j=0; j<16*param.format.image.nStride>>1; j++)
	     {
	       *p++ = col;
	     }
	   
	 }
       
       buffer->nFilledLen = param.nBufferSize;
       buffer->nOffset = 0;
       buffer->pAppPrivate = NULL;
       
       error = OMX_EmptyThisBuffer(ILC_GET_HANDLE(st), buffer);
       vc_assert(error == OMX_ErrorNone);
     }

   // cleanup
   ilclient_change_component_state(st, OMX_StateIdle);

   error = OMX_SendCommand(ILC_GET_HANDLE(st), OMX_CommandStateSet, OMX_StateLoaded, NULL);
   vc_assert(error == OMX_ErrorNone);

   while((buffer = ilclient_get_input_buffer(st, 90)) != NULL)
     {
       unsigned char *buf = buffer->pBuffer;
       
       error = OMX_FreeBuffer(ILC_GET_HANDLE(st), 90, buffer);
       vc_assert(error == OMX_ErrorNone);
       
       free(buf);
     }

   ilclient_wait_for_event(st, OMX_EventCmdComplete, OMX_CommandStateSet, 0, OMX_StateLoaded, 0,
                           ILCLIENT_STATE_CHANGED, PLATFORM_EVENTGROUP_SUSPEND);

   ilclient_cleanup_components(list);

   error = OMX_Deinit();
   vc_assert(error == OMX_ErrorNone);

   n = platform_eventgroup_delete(&events);
   vc_assert(n == 0);

   ilclient_destroy(client);
}
