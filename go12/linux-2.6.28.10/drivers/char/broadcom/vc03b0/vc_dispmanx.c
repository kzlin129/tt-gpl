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

Project  :  VideoCore Software Host Interface (Host-side functions)
Module   :  Host Request Service (host-side)
File     :  $RCSfile: $
Revision :  $Revision: $

FILE DESCRIPTION
Display manager service API.
=============================================================================*/

#if defined(__KERNEL__)

#include <linux/string.h>

#else

#endif

#include "vchost_config.h"
#include "vchost.h"
#include "vcinterface.h"

#include "vciface.h"
#include "vcmsgfifo.h"
//#include "vcutil.h"

#include "vc_dispservice_x_defs.h"
#include "vc_dispmanx.h"
#include "vchi/host_msgfifo_wrapper.h" 


/******************************************************************************
Global data.
******************************************************************************/

/******************************************************************************
Local types and defines.
******************************************************************************/
typedef struct {
   VC_MSGFIFO_RESP_HEADER_T base;
   uint32_t read_response;
   uint32_t extra_param;   // if any extra parameter passed back
   uint32_t extra_param2;  // if any extra parameter passed back
   uint32_t extra_param3;  // if any extra parameter passed back
} VC_DISPMANX_TB_RESP_T;

typedef struct {
   VC_MSGFIFO_RESP_HEADER_T base;
   DISPMANX_DISPLAY_GET_INFO_RESP_T resp;
} VC_DISPMANX_DISPLAY_GET_INFO_RESP_T;

typedef struct {
   VC_MSGFIFO_RESP_HEADER_T base;
   DISPMANX_GET_DEVICES_RESP_T resp;
} VC_DISPMANX_GET_DEVICES_RESP_T;

#define VC_NUM_HOST_RESOURCES 64
typedef struct {
   uint32_t vc_handle;
   uint32_t pitch;
   uint32_t image_data;
   uint32_t native_image_handle;
   uint32_t opacity;
} HOST_DISPMANX_RESOURCE_T;
HOST_DISPMANX_RESOURCE_T host_resources[VC_NUM_HOST_RESOURCES];

// Mask to OR in to one of our handles to ensure zero means failure.
#define HANDLE_MASK (1<<31)

/******************************************************************************
Static data.
******************************************************************************/

static int dispmanx_inum = -1;
static void *host_dispmanx_lock=NULL;
static void *host_update_lock=NULL;
static void *dispmanx_ievent;
static int dispmanx_current_handle = 0;
static char dispmanx_devices[DISPMANX_MAX_HOST_DEVICES][DISPMANX_MAX_DEVICE_NAME_LEN];

/******************************************************************************
Static functions.
******************************************************************************/

static void initialise_host_handles(void)
{
   int i;
   for (i = 0; i < VC_NUM_HOST_RESOURCES; i++)
      host_resources[i].vc_handle = 0;
}

static int allocate_host_handle(void)
{
   int i;
   for (i = 0; i < VC_NUM_HOST_RESOURCES; i++, dispmanx_current_handle++) {
      if (dispmanx_current_handle == VC_NUM_HOST_RESOURCES)
         dispmanx_current_handle = 0;
      if (host_resources[dispmanx_current_handle].vc_handle == 0) {
         return dispmanx_current_handle;
      }
   }
   assert(0);   // no more host-side objects (increase VC_NUM_HOST_RESOURCES?)
   return -1;
}

static void free_host_handle(int handle)
{
   assert(host_resources[handle].vc_handle);  // freeing an unused resource would be strange
   host_resources[handle].vc_handle = 0;
}


/***********************************************************
 * Name: vc_dispmanx_init
 *
 * Arguments:
 *       -
 *
 * Description: Initialise the Host side part of dispmanx
 *
 * Returns: The service number (-ve indicates failure)
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_init( void )
{
   int status;
   int i;
   VC_GENERIC_INTERFACE_T generic_interface;

   if( host_dispmanx_lock == NULL )
      host_dispmanx_lock = vc_lock_create();

   if( host_update_lock == NULL )
      host_update_lock = vc_lock_create(); /* Initialise as available */

   vc_lock_obtain(host_dispmanx_lock);

   if( dispmanx_ievent == NULL )
      dispmanx_ievent = vc_event_create();

   // We simply loop through every interface that there is and look for one
   // that claims to be a display manager service.
   for (i = 0; i < VC_NUM_INTERFACES; i++)
   {
      if (vc_sharedmem_header.iface[i])
      {
         uint16_t stype;
         vc_host_read_consecutive(&generic_interface, vc_interface_base + vc_sharedmem_header.iface[i], sizeof(VC_GENERIC_INTERFACE_T), 0);
         stype = VC_VTOH16(generic_interface.stype);
         if (stype == VC_STYPE_DISPMAN)
         {
            // Gotcha!
            dispmanx_inum = i;
            vc_interface_register_event_int(dispmanx_ievent, (1<<dispmanx_inum));
            status = vc_msgfifo_init(i);
            vc_assert(status == 0);
            break;
         }
      }
   }

   initialise_host_handles();

   // Reset all the device names.
   for (i = 0; i < DISPMANX_MAX_HOST_DEVICES; i++)
      dispmanx_devices[i][0] = 0;

   vc_lock_release(host_dispmanx_lock);
   return dispmanx_inum;
}

VCHPRE_ int VCHPOST_ vc_dispman_init( void )
{
   return vc_dispmanx_init();
}

/***********************************************************
 * Name: vc_dispmanx_stop
 *
 * Arguments:
 *       -
 *
 * Description: Stops the Host side part of dispmanx
 *
 * Returns: -
 *
 ***********************************************************/
VCHPRE_ void VCHPOST_ vc_dispmanx_stop( void )
{
   // Wait for the current lock-holder to finish before zapping dispmanx.
   vc_lock_obtain(host_dispmanx_lock);
   dispmanx_inum = -1;
   vc_interface_register_event_int(dispmanx_ievent, 0);
   vc_lock_release(host_dispmanx_lock);
}


/***********************************************************
 * Name: vc_dispmanx_rect_set
 *
 * Arguments:
 *       VC_RECT_T *rect
 *       uint32_t x_offset
 *       uint32_t y_offset
 *       uint32_t width
 *       uint32_t height
 *
 * Description: Fills in the fields of the supplied VC_RECT_T structure
 *
 * Returns: 0 or failure
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_rect_set( VC_RECT_T *rect, uint32_t x_offset, uint32_t y_offset, uint32_t width, uint32_t height )
{
   rect->x = x_offset;
   rect->y = y_offset;
   rect->width = width;
   rect->height = height;
   return 0;
}



/***********************************************************
 * Name: vc_dispmanx_get_devices
 *
 * Arguments:
 *       uint32_t * ndevices
 *
 * Description: Get the number of devices
 *
 * Returns: 0 or failure
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_get_devices( uint32_t * ndevices )
{
   VC_DISPMANX_GET_DEVICES_RESP_T response_buffer;
   DISPMANX_GET_DEVICES_RESP_T *resp = &response_buffer.resp;
   int32_t response;
   if (dispmanx_inum < 0)
      return -1;
   vc_lock_obtain(host_dispmanx_lock);
   // Send the command
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_GET_DEVICES, 0, NULL, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(response_buffer), dispmanx_ievent);

   memcpy(dispmanx_devices, resp->names, sizeof(dispmanx_devices));
   *ndevices = VC_VTOH32(resp->ndevices);
   response = VC_VTOH32(resp->response);

   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return response;
}


/***********************************************************
 * Name: vc_dispmanx_get_device_name
 *
 * Arguments:
 *       uint32_t device_number
 *       uint32_t max_length
 *       char  * pname
 *
 * Description: get the device name. Need to call vc_dispmanx_get_devices first.
 *              Note that some devices may return a NULL string.
 *
 * Returns: 0 or failure
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_get_device_name( uint32_t device_number, uint32_t max_length, char  * pname )
{
   if (device_number >= DISPMANX_MAX_HOST_DEVICES) {
      assert(0);
      return -1;
   }
   strncpy(pname, dispmanx_devices[device_number], max_length);
   pname[max_length-1] = 0;
   return 0;
}


/***********************************************************
 * Name: vc_dispmanx_get_modes
 *
 * Arguments:
 *       uint32_t device
 *       uint32_t * nmodes
 *
 * Description: Reports the number of modes a device has
 *
 * Returns: 0 or failure
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_get_modes( uint32_t device, uint32_t * nmodes )
{
   DISPMANX_GET_MODES_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   int32_t response;
   if (dispmanx_inum < 0)
      return -1;
   params.device = VC_HTOV32(device);
   vc_lock_obtain(host_dispmanx_lock);
   // Send the command
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_GET_MODES, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(response_buffer), dispmanx_ievent);

   *nmodes = VC_VTOH32(response_buffer.extra_param);
   response = VC_VTOH32(response_buffer.read_response);

   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return response;
}



/***********************************************************
 * Name: vc_dispmanx_get_mode_info
 *
 * Arguments:
 *       uint32_t device
 *       uint32_t mode
 *       DISPMANX_MODEINFO_T  * pmodes
 *
 * Description: Fills in the structure with useful information
 *
 * Returns: 0 or failure
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_get_mode_info( uint32_t display, uint32_t mode, DISPMANX_MODEINFO_T  * pinfo )
{
   DISPMANX_GET_MODE_INFO_PARAM_T params;
   VC_DISPMANX_DISPLAY_GET_INFO_RESP_T response_buffer;  // returns same info
   int response;

   if (dispmanx_inum < 0)
      return -1;
   params.display = VC_VTOH32(display);
   params.mode = VC_VTOH32(mode);

   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_GET_MODE_INFO, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(response_buffer), dispmanx_ievent);
   response = VC_VTOH32(response_buffer.resp.read_response);
   pinfo->width = VC_VTOH32(response_buffer.resp.width);
   pinfo->height = VC_VTOH32(response_buffer.resp.height);
   pinfo->aspect_pixwidth = VC_VTOH32(response_buffer.resp.aspect_pixwidth);
   pinfo->aspect_pixheight = VC_VTOH32(response_buffer.resp.aspect_pixheight);
   pinfo->fieldrate_num = VC_VTOH32(response_buffer.resp.fieldrate_num);
   pinfo->fieldrate_denom = VC_VTOH32(response_buffer.resp.fieldrate_denom);
   pinfo->fields_per_frame = VC_VTOH32(response_buffer.resp.fields_per_frame);
   pinfo->transform = (VC_IMAGE_TRANSFORM_T)VC_VTOH32(response_buffer.resp.transform);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return response;
}

/******************************************************************************
NAME
   vc_dispmanx_query_image_formats

PARAMS
   uint32_t *support_formats - the returned supported image formats

FUNCTION
   Returns the support image formats from the VMCS host

RETURNS
   Success: 0
   Otherwise non-zero
******************************************************************************/
VCHPRE_ int  VCHPOST_ vc_dispmanx_query_image_formats( uint32_t *supported_formats )
{
   VC_DISPMANX_TB_RESP_T response_buffer;

   if (dispmanx_inum < 0)
      return -1;

   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_DISPLAY_QUERY_IMAGE_FORMATS, 0, NULL, dispmanx_ievent );
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   *supported_formats = VC_VTOH32(response_buffer.read_response);

   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);

   return 0;
}


/***********************************************************
 * Name: vc_dispmanx_resource_create
 *
 * Arguments:
 *       VC_IMAGE_TYPE_T type
 *       uint32_t width
 *       uint32_t height
 *
 * Description: Create a new resource (in Videocore memory)
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ DISPMANX_RESOURCE_HANDLE_T VCHPOST_ vc_dispmanx_resource_create( VC_IMAGE_TYPE_T type, uint32_t width, uint32_t height, uint32_t *native_image_handle )
{
   DISPMANX_RESOURCE_CREATE_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   int handle = -1;

   params.type = VC_HTOV32(type);
   params.width = VC_HTOV32(width);
   params.height = VC_HTOV32(height);

   if (dispmanx_inum < 0)
      return -1;
   vc_lock_obtain(host_dispmanx_lock);

   // Check we have a host-side resource to keep this info.
   handle = allocate_host_handle();
   if (handle < 0) {
      vc_lock_release(host_dispmanx_lock);
      return (DISPMANX_RESOURCE_HANDLE_T)handle;
   }

   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_RESOURCE_CREATE,
                                    PAD16(sizeof(DISPMANX_RESOURCE_CREATE_PARAM_T)),
                                    &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   if (response_buffer.read_response == 0) {
      free_host_handle(handle);
      vc_msgfifo_read_flush(dispmanx_inum);
      vc_lock_release(host_dispmanx_lock);
      return (DISPMANX_RESOURCE_HANDLE_T)0;
   }

   // All appears well. Store the info locally.
   host_resources[handle].vc_handle = VC_VTOH32(response_buffer.read_response);
   host_resources[handle].pitch = VC_VTOH32(response_buffer.extra_param);
   host_resources[handle].native_image_handle = VC_VTOH32(response_buffer.extra_param2);
   host_resources[handle].image_data = VC_VTOH32(response_buffer.extra_param3);
   host_resources[handle].opacity = 255;  // fully opaque by default

   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);

   *native_image_handle = host_resources[handle].native_image_handle;

   // OR with mask before letting it out
   return (DISPMANX_RESOURCE_HANDLE_T)(handle|HANDLE_MASK);
}


/***********************************************************
 * Name: vc_dispmanx_resource_set_opacity
 *
 * Arguments:
 *       DISPMANX_RESOURCE_HANDLE_T res
 *
 * Description:
 *
 * Returns: 0 or failure
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_resource_set_opacity( DISPMANX_RESOURCE_HANDLE_T res, uint32_t opacity )
{
   int handle = (int)res;
   assert(handle);
   host_resources[handle&~HANDLE_MASK].opacity = opacity;
   return 0;
}


/***********************************************************
 * Name: vc_dispmanx_resource_delete
 *
 * Arguments:
 *       DISPMANX_RESOURCE_HANDLE_T res
 *
 * Description:
 *
 * Returns: 0 or failure
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_resource_delete( DISPMANX_RESOURCE_HANDLE_T res )
{

   DISPMANX_RESOURCE_DELETE_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   int handle = (int)res;
   int32_t response;

   if (dispmanx_inum < 0)
      return -1;
   // We ORed a mask on before letting it out!
   handle &= ~HANDLE_MASK;
   assert(host_resources[handle].vc_handle); // it should be
   params.handle = VC_VTOH32((uint32_t)host_resources[handle].vc_handle);
   vc_lock_obtain(host_dispmanx_lock);
   free_host_handle(handle);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_RESOURCE_DELETE, sizeof(DISPMANX_RESOURCE_DELETE_PARAM_T), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return response;
}


/***********************************************************
 * Name: vc_dispmanx_resource_write_data
 *
 * Arguments:
 *       DISPMANX_RESOURCE_HANDLE_T res
 *       int src_pitch
 *       void * src_address
 *       const VC_RECT_T * rect
 *
 * Description: Copy the bitmap data to VideoCore memory
 *
 * Returns: 0 or failure
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_resource_write_data(DISPMANX_RESOURCE_HANDLE_T handle, VC_IMAGE_TYPE_T src_type, 
						     int src_pitch, void * src_address, const VC_RECT_T * rect )
{
   // This is going to have to send a message to vc to queue a bulk receive, and then
   // queue a bulk transmit back here.
   HOST_DISPMANX_RESOURCE_T *res;
   uint8_t *host_start = (uint8_t *)src_address + src_pitch * rect->y;
   uint32_t vc_start;
   int bulk_len;

   assert( handle );
   handle &= ~HANDLE_MASK;
   res = &host_resources[handle];

   bulk_len = src_pitch * rect->height;
   vc_start = res->image_data + res->pitch*rect->y;

   host_vchi_msgfifo_wrapper_bulk_write( vc_start, host_start, bulk_len );

   return 0;
}

/***********************************************************
 * Name: vc_dispmanx_display_open
 *
 * Arguments:
 *       uint32_t device
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ DISPMANX_DISPLAY_HANDLE_T VCHPOST_ vc_dispmanx_display_open( uint32_t device )
{
   DISPMANX_DISPLAY_OPEN_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   DISPMANX_DISPLAY_HANDLE_T display_h;
   if (dispmanx_inum < 0)
      return -1;
   params.device = VC_HTOV32(device);

   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_DISPLAY_OPEN, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   display_h = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return display_h;
}


/***********************************************************
 * Name: vc_dispmanx_display_open_mode
 *
 * Arguments:
 *       uint32_t device
 *       uint32_t mode
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ DISPMANX_DISPLAY_HANDLE_T VCHPOST_ vc_dispmanx_display_open_mode( uint32_t device, uint32_t mode )
{
   DISPMANX_DISPLAY_OPEN_MODE_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   DISPMANX_DISPLAY_HANDLE_T display_h;
   if (dispmanx_inum < 0)
      return -1;
   params.device = VC_HTOV32(device);
   params.mode = VC_HTOV32(mode);

   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_DISPLAY_OPEN_MODE, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   display_h = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return display_h;
}


/***********************************************************
 * Name: vc_dispmanx_display_open_offscreen
 *
 * Arguments:
 *       DISPMANX_RESOURCE_HANDLE_T dest
 *       VC_IMAGE_TRANSFORM_T orientation
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ DISPMANX_DISPLAY_HANDLE_T VCHPOST_ vc_dispmanx_display_open_offscreen( DISPMANX_RESOURCE_HANDLE_T dest, VC_IMAGE_TRANSFORM_T orientation )
{
   DISPMANX_DISPLAY_OPEN_OFFSCREEN_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   DISPMANX_DISPLAY_HANDLE_T display_h;
   uint32_t dest_handle;
   if (dispmanx_inum < 0)
      return -1;
   assert(dest);
   dest_handle = dest&~HANDLE_MASK;
   dest = host_resources[dest_handle].vc_handle;
   params.dest = VC_HTOV32(dest);
   params.orientation = VC_HTOV32(((uint32_t)orientation));

   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_DISPLAY_OPEN_OFFSCREEN, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   display_h = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return display_h;
}


/***********************************************************
 * Name: vc_dispmanx_display_reconfigure
 *
 * Arguments:
 *       DISPMANX_DISPLAY_HANDLE_T display
 *       uint32_t mode
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_display_reconfigure( DISPMANX_DISPLAY_HANDLE_T device, uint32_t mode )
{
   DISPMANX_DISPLAY_OPEN_MODE_PARAM_T params;  // these are the correct params
   VC_DISPMANX_TB_RESP_T response_buffer;
   int response;
   if (dispmanx_inum < 0)
      return -1;
   params.device = VC_HTOV32(device);
   params.mode = VC_HTOV32(mode);

   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_DISPLAY_RECONFIGURE, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return response;
}


/***********************************************************
 * Name: vc_dispmanx_display_set_destination
 *
 * Arguments:
 *       DISPMANX_DISPLAY_HANDLE_T display
 *       DISPMANX_RESOURCE_HANDLE_T dest
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_display_set_destination( DISPMANX_DISPLAY_HANDLE_T display, DISPMANX_RESOURCE_HANDLE_T dest )
{
   DISPMANX_DISPLAY_SET_DESTINATION_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   uint32_t dest_handle;
   int response;
   if (dispmanx_inum < 0)
      return -1;
   assert(dest);
   dest_handle = dest&~HANDLE_MASK;
   dest = host_resources[dest_handle].vc_handle;
   params.dest = VC_HTOV32(dest);
   params.display = VC_HTOV32(display);

   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_DISPLAY_SET_DESTINATION, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return response;
}


/***********************************************************
 * Name: vc_dispmanx_display_set_background
 *
 * Arguments:
 *       DISPMANX_UPDATE_HANDLE_T update
 *       DISPMANX_DISPLAY_HANDLE_T display
 *       uint8_t red
 *       uint8_t green
 *       uint8_t blue
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_display_set_background( DISPMANX_UPDATE_HANDLE_T update, DISPMANX_DISPLAY_HANDLE_T display,
                                                                       uint8_t red, uint8_t green, uint8_t blue )
{
   DISPMANX_DISPLAY_SET_BACKGROUND_PARAM_T params;  // these are the correct params
   VC_DISPMANX_TB_RESP_T response_buffer;
   int response;
   uint32_t colour = (red<<16)|(green<<8)|blue;
   if (dispmanx_inum < 0)
      return -1;
   params.display = VC_HTOV32(display);
   params.update = VC_HTOV32(update);
   params.colour = VC_HTOV32(colour);

   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_DISPLAY_SET_BACKGROUND, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return response;
}


/***********************************************************
 * Name: vc_dispmanx_display_get_info
 *
 * Arguments:
 *       DISPMANX_DISPLAY_HANDLE_T display
 *       DISPMANX_MODEINFO_T * pinfo
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_display_get_info( DISPMANX_DISPLAY_HANDLE_T display, DISPMANX_MODEINFO_T * pinfo )
{
   DISPMANX_DISPLAY_GET_INFO_PARAM_T params;
   VC_DISPMANX_DISPLAY_GET_INFO_RESP_T response_buffer;
   int response;

   if (dispmanx_inum < 0)
      return -1;
   params.display = VC_VTOH32(display);

   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_DISPLAY_GET_INFO, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(response_buffer), dispmanx_ievent);
   response = VC_VTOH32(response_buffer.resp.read_response);
   pinfo->width = VC_VTOH32(response_buffer.resp.width);
   pinfo->height = VC_VTOH32(response_buffer.resp.height);
   pinfo->aspect_pixwidth = VC_VTOH32(response_buffer.resp.aspect_pixwidth);
   pinfo->aspect_pixheight = VC_VTOH32(response_buffer.resp.aspect_pixheight);
   pinfo->fieldrate_num = VC_VTOH32(response_buffer.resp.fieldrate_num);
   pinfo->fieldrate_denom = VC_VTOH32(response_buffer.resp.fieldrate_denom);
   pinfo->fields_per_frame = VC_VTOH32(response_buffer.resp.fields_per_frame);
   pinfo->transform = (VC_IMAGE_TRANSFORM_T)VC_VTOH32(response_buffer.resp.transform);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return response;
}


/***********************************************************
 * Name: vc_dispmanx_display_close
 *
 * Arguments:
 *       DISPMANX_DISPLAY_HANDLE_T display
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_display_close( DISPMANX_DISPLAY_HANDLE_T display )
{
   DISPMANX_DISPLAY_OPEN_PARAM_T params;  // this will do
   VC_DISPMANX_TB_RESP_T response_buffer;
   int response;
   if (dispmanx_inum < 0)
      return -1;
   params.device = VC_HTOV32(display);

   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_DISPLAY_CLOSE, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return response;
}


/***********************************************************
 * Name: vc_dispmanx_update_start
 *
 * Arguments:
 *       int32_t priority
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ DISPMANX_UPDATE_HANDLE_T VCHPOST_ vc_dispmanx_update_start( int32_t priority )
{
   DISPMANX_UPDATE_START_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   DISPMANX_UPDATE_HANDLE_T update_h;
   if (dispmanx_inum < 0)
      return -1;
   params.priority = VC_VTOH32(priority);

   vc_lock_obtain(host_update_lock); /* Only one update at once - do we still need this? */

   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_UPDATE_START, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   update_h = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return update_h;
}


/***********************************************************
 * Name: vc_dispmanx_element_add
 *
 * Arguments:
 *       DISPMANX_UPDATE_HANDLE_T update
 *       DISPMANX_DISPLAY_HANDLE_T display
 *       int32_t layer
 *       const VC_RECT_T *dest_rect
 *       DISPMANX_RESOURCE_HANDLE_T src
 *       const VC_RECT_T *src_rect
 *       DISPMANX_FLAGS_T flags
 *       uint8_t opacity
 *       DISPMANX_RESOURCE_HANDLE_T mask
 *       VC_IMAGE_TRANSFORM_T transform
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ DISPMANX_ELEMENT_HANDLE_T VCHPOST_ vc_dispmanx_element_add ( DISPMANX_UPDATE_HANDLE_T update,
                                                                     DISPMANX_DISPLAY_HANDLE_T display,
                                                                     int32_t layer,
                                                                     const VC_RECT_T *dest_rect,
                                                                     DISPMANX_RESOURCE_HANDLE_T src,
                                                                     const VC_RECT_T *src_rect,
                                                                     DISPMANX_FLAGS_T flags,
                                                                     uint32_t opacity,
                                                                     DISPMANX_RESOURCE_HANDLE_T mask,
                                                                     VC_IMAGE_TRANSFORM_T transform )
{
   DISPMANX_ELEMENT_ADD_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   DISPMANX_ELEMENT_HANDLE_T element_h;
   uint16_t dest_x_offset = dest_rect->x, dest_y_offset = dest_rect->y;
   uint16_t dest_width = dest_rect->width, dest_height = dest_rect->height;
   uint16_t src_x_offset = src_rect->x, src_y_offset = src_rect->y;
   uint16_t src_width = src_rect->width, src_height = src_rect->height;
   int src_handle;
   params.update = VC_HTOV32(update);
   params.display = VC_HTOV32(display);
   params.layer = VC_HTOV32(layer);
   params.dest_x = VC_HTOV16(dest_x_offset);
   params.dest_y = VC_HTOV16(dest_y_offset);
   params.dest_width = VC_HTOV16(dest_width);
   params.dest_height = VC_HTOV16(dest_height);
   params.src_x = VC_HTOV16(src_x_offset);
   params.src_y = VC_HTOV16(src_y_offset);
   params.src_width = VC_HTOV16(src_width);
   params.src_height = VC_HTOV16(src_height);
   params.flags = VC_HTOV32((uint32_t)flags);
   assert(src);
   src_handle = src&~HANDLE_MASK;
   src = host_resources[src_handle].vc_handle;
   params.src_resource = VC_HTOV32(src);
   params.transform = VC_HTOV32(transform);
   params.mask_resource = 0;
   if (mask) {
      int tmp = host_resources[mask&~HANDLE_MASK].vc_handle;
      params.mask_resource = VC_HTOV32(tmp);
   }
   if (opacity == (uint32_t)-1)
      opacity = host_resources[src_handle].opacity;
   params.opacity = VC_VTOH32(opacity);
   if (dispmanx_inum < 0)
      return -1;
   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_ELEMENT_ADD, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   element_h = (DISPMANX_ELEMENT_HANDLE_T)VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return element_h;
}


/***********************************************************
 * Name: vc_dispmanx_element_change_source
 *
 * Arguments:
 *       DISPMANX_UPDATE_HANDLE_T update
 *       DISPMANX_ELEMENT_HANDLE_T element
 *       DISPMANX_RESOURCE_HANDLE_T src
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_element_change_source( DISPMANX_UPDATE_HANDLE_T update, DISPMANX_ELEMENT_HANDLE_T element,
                                                        DISPMANX_RESOURCE_HANDLE_T src )
{
   DISPMANX_ELEMENT_CHANGE_SOURCE_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   int src_handle;
   int response;
   params.update = VC_HTOV32(update);
   assert(src);
   src_handle = src&~HANDLE_MASK;
   src = host_resources[src_handle].vc_handle;
   params.src_resource = VC_HTOV32(src);
   if (dispmanx_inum < 0)
      return -1;
   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_ELEMENT_ADD, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return response;
}


/***********************************************************
 * Name: vc_dispmanx_element_modified
 *
 * Arguments:
 *       DISPMANX_UPDATE_HANDLE_T update
 *       DISPMANX_ELEMENT_HANDLE_T element
 *       const VC_RECT_T * rect
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_element_modified( DISPMANX_UPDATE_HANDLE_T update, DISPMANX_ELEMENT_HANDLE_T element, const VC_RECT_T * rect )
{
   DISPMANX_ELEMENT_MODIFIED_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   int response;

   params.element = VC_VTOH32(element);
   params.update = VC_VTOH32(update);
   params.x = VC_VTOH16(rect->x);
   params.y = VC_VTOH16(rect->y);
   params.width = VC_VTOH16(rect->width);
   params.height = VC_VTOH16(rect->height);

   if (dispmanx_inum < 0)
      return -1;
   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_ELEMENT_MODIFIED, sizeof(DISPMANX_ELEMENT_MODIFIED_PARAM_T), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return response;
}


/***********************************************************
 * Name: vc_dispmanx_element_remove
 *
 * Arguments:
 *       DISPMANX_UPDATE_HANDLE_T update
 *       DISPMANX_ELEMENT_HANDLE_T element
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_element_remove( DISPMANX_UPDATE_HANDLE_T update, DISPMANX_ELEMENT_HANDLE_T element )
{
   DISPMANX_ELEMENT_REMOVE_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   int response;

   params.element = VC_VTOH32(element);
   params.update = VC_VTOH32(update);

   if (dispmanx_inum < 0)
      return -1;
   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_ELEMENT_REMOVE, sizeof(DISPMANX_ELEMENT_REMOVE_PARAM_T), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);
   return response;
}


/***********************************************************
 * Name: vc_dispmanx_update_submit
 *
 * Arguments:
 *       DISPMANX_UPDATE_HANDLE_T update
 *       DISPMANX_CALLBACK_FUNC_T cb_func
 *       void *cb_arg
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_update_submit( DISPMANX_UPDATE_HANDLE_T update, DISPMANX_CALLBACK_FUNC_T cb_func, void *cb_arg )
{
   assert(0); // can you use vc_dispmanx_update_submit_sync instead?
   return 0;
}


/***********************************************************
 * Name: vc_dispmanx_update_submit_sync
 *
 * Arguments:
 *       DISPMANX_UPDATE_HANDLE_T update
 *
 * Description:
 *
 * Returns:
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_update_submit_sync( DISPMANX_UPDATE_HANDLE_T update )
{
   DISPMANX_UPDATE_SUBMIT_SYNC_PARAM_T params;
   VC_DISPMANX_TB_RESP_T response_buffer;
   int32_t response;
   
   if (dispmanx_inum < 0)
      return -1;
   params.update = VC_VTOH32(update);
   vc_lock_obtain(host_dispmanx_lock);
   vc_msgfifo_send_command_blocking(dispmanx_inum, VC_DISPMANX_UPDATE_SUBMIT_SYNC, sizeof(params), &params, dispmanx_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispmanx_inum, &response_buffer, sizeof(VC_DISPMANX_TB_RESP_T), dispmanx_ievent);
   response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispmanx_inum);
   vc_lock_release(host_dispmanx_lock);

   vc_lock_release(host_update_lock); /* Only one update at once  - do we still need this? */

   return 0;
}


/***********************************************************
 * Name: vc_dispmanx_inum
 *
 * Arguments:
 *       -
 *
 * Description: Return the interface number
 *
 * Returns: interface number (negative is error)
 *
 ***********************************************************/
VCHPRE_ int VCHPOST_ vc_dispmanx_inum ( void )
{
   return dispmanx_inum;
}
