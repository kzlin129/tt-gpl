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




#include <linux/string.h>

#include "vchost_config.h"
#include "vchost.h"
#include "vcinterface.h"

#include "vciface.h"
#include "vcmsgfifo.h"
#include "vcutil.h"

#include "vc_dispservice_defs.h"
#include "vc_dispman2.h"

/******************************************************************************
Global data.
******************************************************************************/

/******************************************************************************
Local types and defines.
******************************************************************************/

/******************************************************************************
Static data.
******************************************************************************/

int dispman_inum = -1;
void *host_dispman_lock=NULL;
void *host_update_lock=NULL;
void *dispman_ievent;

/******************************************************************************
Static functions.
******************************************************************************/

/******************************************************************************
NAME
   vc_dispman_init

SYNOPSIS
   int vc_dispman_init()

FUNCTION
   Initialise the display manager service for use. A negative return value
   indicates failure (which may mean it has not been started on VideoCore).

RETURNS
   int
******************************************************************************/

int vc_dispman_init (void) {
   int status;
   int i;
   VC_GENERIC_INTERFACE_T generic_interface;

   if (host_dispman_lock == NULL)
      host_dispman_lock = vc_lock_create();

   if (host_update_lock == NULL)
      host_update_lock = vc_lock_create(); /* Initialise as available */

   vc_lock_obtain(host_dispman_lock);

   if (dispman_ievent == NULL)
      dispman_ievent = vc_event_create();

   // We simply loop through every interface that there is and look for one
   // that claims to be a display manager service.
   for (i = 0; i < VC_NUM_INTERFACES; i++) {
      if (vc_sharedmem_header.iface[i]) {
         uint16_t stype;
         vc_host_read_consecutive(&generic_interface, vc_interface_base + vc_sharedmem_header.iface[i], sizeof(VC_GENERIC_INTERFACE_T), 0);
         stype = VC_VTOH16(generic_interface.stype);
         if (stype == VC_STYPE_DISPMAN) {
            // Gotcha!
            dispman_inum = i;
            vc_interface_register_event_int(dispman_ievent, (1<<dispman_inum));
            status = vc_msgfifo_init(i);
            vc_assert(status == 0);
            break;
         }
      }
   }

   vc_lock_release(host_dispman_lock);
   return dispman_inum;
}

/******************************************************************************
NAME
   vc_dispman_stop

SYNOPSIS
   void vc_dispman_stop()

FUNCTION
   This tells us that the display manager service has stopped, thereby preventing
   any of the functions from doing anything.

RETURNS
   void
******************************************************************************/

void vc_dispman_stop () {
   // Wait for the current lock-holder to finish before zapping dispman.
   vc_lock_obtain(host_dispman_lock);
   dispman_inum = -1;
   vc_interface_register_event_int(dispman_ievent, 0);
   vc_lock_release(host_dispman_lock);
}

/******************************************************************************
NAME
   vc_dispman_inum

SYNOPSIS
   int vc_dispman_inum()

FUNCTION
   Return the dispman service number (-1 if not running).

RETURNS
   int
******************************************************************************/

int vc_dispman_inum () {
   return dispman_inum;
}

typedef struct {
   VC_MSGFIFO_RESP_HEADER_T base;
   uint32_t read_response;
   uint32_t extra_param;   // if any extra parameter passed back
   uint32_t pad[2];
} VC_DISPMAN_TB_RESP_T;

/******************************************************************************
NAME
   vc_dispman_lock

SYNOPSIS
   int vc_dispman_lock(uint32_t state, int32_t *response)

FUNCTION
   Send a VC_DISPMAN_DISPLAY_LOCK command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/

int vc_dispman_lock(uint32_t state, int32_t *response) {
   DISPMAN_LOCK_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;
   if (dispman_inum < 0)
      return -1;
   params.state = VC_HTOV32(state);
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_LOCK, sizeof(params), &params, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}

/******************************************************************************
NAME
   vc_dispman_unlock

SYNOPSIS
   int vc_dispman_unlock(uint32_t *state)

FUNCTION
   Send a VC_DISPMAN_DISPLAY_UNLOCK command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/

int vc_dispman_unlock(uint32_t *state) {
   VC_DISPMAN_TB_RESP_T response_buffer;
   if (dispman_inum < 0)
      return -1;
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_UNLOCK, 0, NULL, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *state = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}

/******************************************************************************
NAME
   vc_dispman_display_set_destination

SYNOPSIS
   int vc_dispman_display_set_destination(uint32_t display, uint32_t resource, uint32_t *response)

FUNCTION
   Send a VC_DISPMAN_DISPLAY_SET_DESTINATION command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/

int vc_dispman_display_set_destination(uint32_t display, uint32_t resource, int32_t *response) {
   DISPMAN_SET_DEST_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;
   params.display = display;
   params.resource = resource;
   if (dispman_inum < 0)
      return -1;
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_SET_DESTINATION, sizeof(DISPMAN_SET_DEST_PARAM_T), &params, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}

/******************************************************************************
NAME
   vc_dispman_display_snapshot

SYNOPSIS
   int vc_dispman_display_snapshot(uint32_t display, uint32_t resource, uint32_t *response)

FUNCTION
   Send a VC_DISPMAN_DISPLAY_SNAPSHOT command. Return once the resource buffer is filled.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/

int vc_dispman_display_snapshot(uint32_t display, uint32_t resource, int32_t *response) {
   DISPMAN_DISPLAY_SNAPSHOT_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;
   params.display = display;
   params.resource = resource;
   if (dispman_inum < 0)
      return -1;
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_SNAPSHOT, sizeof(DISPMAN_DISPLAY_SNAPSHOT_PARAM_T), &params, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}

/******************************************************************************
NAME
   vc_dispman_display_change_resolution

SYNOPSIS
   int vc_dispman_display_change_resolution(uint32_t display,
                               uint16_t x_offset, uint16_t y_offset, uint16_t width, uint16_t height,
                               uint16_t max_width, uint16_t max_height, int32_t *response)

FUNCTION
   Change the apparent resolution of a display. Cannot make it bigger than its initial setting.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/

int vc_dispman_display_reconfigure(uint32_t display,
                                   uint16_t width, uint16_t height, int32_t *response) {
   DISPMAN_RECONFIGURE_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;
   params.display = VC_HTOV32(display);
   params.width = VC_HTOV16(width);
   params.height = VC_HTOV16(height);
   if (dispman_inum < 0)
      return -1;
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_RECONFIGURE,
                                    sizeof(DISPMAN_RECONFIGURE_PARAM_T), &params, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}

/******************************************************************************
NAME
   vc_dispman_display_get_composite

SYNOPSIS
   int vc_dispman_display_get_composite(uint32_t display, uint32_t *composite_addr)

FUNCTION
   Return the address on VideoCore of the composite buffer associate with a
   particular display.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/

int vc_dispman_display_get_composite(uint32_t display, uint32_t *composite_addr) {
   DISPMAN_GET_COMPOSITE_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;
   params.display = display;
   if (dispman_inum < 0)
      return -1;
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_GET_COMPOSITE, sizeof(DISPMAN_GET_COMPOSITE_PARAM_T), &params, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *composite_addr = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}


/******************************************************************************
NAME
   vc_dispman_display_apply_effects_instance

SYNOPSIS
   int vc_dispman_display_apply_effects_instance(uint32_t display, const uint32_t effects_instance, int32_t *response)

FUNCTION
   Send a VC_DISPMAN_DISPLAY_APPLY_EFFECT command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/
int vc_dispman_display_apply_effects_instance(uint32_t display, const uint32_t effects_instance, int32_t *response)
{
   int success = -1;
   DISPMAN_APPLY_EFFECTS_INSTANCE_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;

   //store the params
   params.display = display;
   params.effects_instance = effects_instance;

   if (dispman_inum >= 0)
   {
      //send the cmd
      success = 0;
      vc_lock_obtain(host_dispman_lock);
      vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_APPLY_EFFECT_INSTANCE, sizeof(DISPMAN_APPLY_EFFECTS_INSTANCE_PARAM_T), &params, dispman_ievent);

      // Read the response.
      vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
      *response = VC_VTOH32(response_buffer.read_response);

      // Flush the input pointer.
      vc_msgfifo_read_flush(dispman_inum);
      vc_lock_release(host_dispman_lock);
   }

   return success;
}

typedef struct
{
   VC_MSGFIFO_RESP_HEADER_T base;
   DISPMAN_CREATE_EFFECTS_INSTANCE_RESPONSE_T create_effects_response;
   uint32_t pad[2]; //pad to a multiple of 16
} VC_DISPMAN_CREATE_EFFECTS_RESP_T;

/******************************************************************************
NAME
   vc_dispman_display_create_effects_instance

SYNOPSIS
   int vc_dispman_display_create_effects_instance( uint32_t *effects_instance, int32_t *response)

FUNCTION
   Send a VC_DISPMAN_DISPLAY_CREATE_EFFECTS_INSTANCE command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/
int vc_dispman_create_effects_instance( uint32_t *effects_instance, int32_t *response)
{
   int success = -1;
   VC_DISPMAN_CREATE_EFFECTS_RESP_T response_buffer;

   if( ( NULL != effects_instance) && (NULL != response) )
   {
      if (dispman_inum >= 0)
      {
         //send the cmd
         success = 0;

         vc_lock_obtain(host_dispman_lock);
         vc_msgfifo_send_command_blocking(dispman_inum,
                                          VC_DISPMAN_DISPLAY_CREATE_EFFECTS_INSTANCE,
                                          0,
                                          NULL,
                                          dispman_ievent);

         // Read the response.
         vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_CREATE_EFFECTS_RESP_T), dispman_ievent);
         *response = VC_VTOH32(response_buffer.create_effects_response.read_response);

         //set the effects instance
         *effects_instance = VC_VTOH32(response_buffer.create_effects_response.effects_instance);

         // Flush the input pointer.
         vc_msgfifo_read_flush(dispman_inum);
         vc_lock_release(host_dispman_lock);
      }
   }

   return success;
}


/******************************************************************************
NAME
   vc_dispman_display_delete_effects_instance

SYNOPSIS
   int vc_dispman_delete_effects_instance( const uint32_t effects_instance, int32_t *response)

FUNCTION
   Send a VC_DISPMAN_DISPLAY_DELETE_EFFECTS_INSTANCE command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/
int vc_dispman_delete_effects_instance( const uint32_t effects_instance, int32_t *response)
{
   int success = -1;
   DISPMAN_DELETE_EFFECTS_INSTANCE_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;

   if (dispman_inum >= 0)
   {
      //store the params
      params.effects_instance = effects_instance;

      //send the cmd
      success = 0;
      vc_lock_obtain(host_dispman_lock);
      vc_msgfifo_send_command_blocking(dispman_inum,
                                       VC_DISPMAN_DISPLAY_DELETE_EFFECTS_INSTANCE,
                                       sizeof(DISPMAN_DELETE_EFFECTS_INSTANCE_PARAM_T),
                                       &params,
                                       dispman_ievent);

      // Read the response.
      vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);

      // Flush the input pointer.
      vc_msgfifo_read_flush(dispman_inum);
      vc_lock_release(host_dispman_lock);
   }

   return success;
}


/******************************************************************************
NAME
   vc_dispman_set_effect

SYNOPSIS
   int vc_dispman_set_effect( const uint32_t effect_instance, const char *effect_name, int32_t *response)

FUNCTION
   Send a VC_DISPMAN_DISPLAY_APPLY_EFFECT command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/
int vc_dispman_set_effect( const uint32_t effects_instance, const char *effect_name, int32_t *response)
{
   int success = -1;
   DISPMAN_SET_EFFECT_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;

   //store the params
   params.effects_instance = effects_instance;

   /* Note - strncpy will no append a null terminator if the
      strings length exactly matches the max buffer size */
   memset( params.effect_name, 0, DISPMAN_MAX_EFFECT_NAME);
   strncpy( params.effect_name, effect_name, DISPMAN_MAX_EFFECT_NAME - 1 );

   if (dispman_inum >= 0)
   {
      //send the cmd
      success = 0;
      vc_lock_obtain(host_dispman_lock);
      vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_SET_EFFECT, sizeof(DISPMAN_SET_EFFECT_PARAM_T), &params, dispman_ievent);

      // Read the response.
      vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
      *response = VC_VTOH32(response_buffer.read_response);

      // Flush the input pointer.
      vc_msgfifo_read_flush(dispman_inum);
      vc_lock_release(host_dispman_lock);
   }

   return success;
}



/******************************************************************************
NAME
   vc_dispman_update_start

SYNOPSIS
   int vc_dispman_update_start(uint32_t update, int32_t *response)

FUNCTION
   Send a VC_DISPMAN_DISPLAY_UPDATE_START command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/

int vc_dispman_update_start(int32_t *response) {
   VC_DISPMAN_TB_RESP_T response_buffer;
   if (dispman_inum < 0)
      return -1;

   vc_lock_obtain(host_update_lock); /* Only one update at once */

   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_UPDATE_START, 0, NULL, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}

/******************************************************************************
NAME
   vc_dispman_update_end

SYNOPSIS
   int vc_dispman_update_end(uint32_t update, int32_t *response)

FUNCTION
   Send a VC_DISPMAN_DISPLAY_UPDATE_END command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/

int vc_dispman_update_end(int32_t *response) {
   VC_DISPMAN_TB_RESP_T response_buffer;
   if (dispman_inum < 0)
      return -1;
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_UPDATE_END, 0, NULL, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);

   vc_lock_release(host_update_lock); /* Only one update at once */

   return 0;
}

/******************************************************************************
NAME
   vc_dispman_object_add

SYNOPSIS
   int vc_dispman_object_add(uint32_t update, uint32_t object,
     uint32_t display, int32_t layer, uint32_t dest_x, uint32_t dest_y,
     uint32_t src_x, uint32_t src_y, uint32_t width, uint32_t height,
     uint32_t resource, VC_DISPMAN_TRANSFORM_T transform)

FUNCTION
   Send a VC_DISPMAN_OBJECT_ADD command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/

int vc_dispman_object_add(uint32_t *object,
     uint32_t display, int32_t layer, uint16_t dest_x_offset, uint16_t dest_y_offset,
     uint16_t width, uint16_t height, uint32_t resource,
     uint16_t src_x_offset, uint16_t src_y_offset, VC_DISPMAN_TRANSFORM_T transform) {
   DISPMAN_OBJECT_ADD_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;
   //params.object = object;
   params.display = VC_HTOV32(display);
   params.layer = VC_HTOV32(layer);
   params.dest_x = VC_HTOV16(dest_x_offset);
   params.dest_y = VC_HTOV16(dest_y_offset);
   params.src_x = VC_HTOV16(src_x_offset);
   params.src_y = VC_HTOV16(src_y_offset);
   params.width = VC_HTOV16(width);
   params.height = VC_HTOV16(height);
   params.resource = VC_HTOV32(resource);
   params.transform = VC_HTOV32(transform);
   if (dispman_inum < 0)
      return -1;
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_OBJECT_ADD, sizeof(DISPMAN_OBJECT_ADD_PARAM_T), &params, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *object = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}

/******************************************************************************
NAME
   vc_dispman_object_remove

SYNOPSIS
   int vc_dispman_object_remove(uint32_t object, int32_t *response)

FUNCTION
   Send a VC_DISPMAN_DISPLAY_OBJECT_REMOVE command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/
int vc_dispman_object_remove(uint32_t object, int32_t *response) {

   DISPMAN_OBJECT_REMOVE_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;

   params.object = VC_HTOV32(object);

   if (dispman_inum < 0)
      return -1;
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_OBJECT_REMOVE, sizeof(DISPMAN_OBJECT_REMOVE_PARAM_T), &params, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}

/******************************************************************************
NAME
   vc_dispman_object_modify

SYNOPSIS
   int vc_dispman_object_remove(uint32_t object, uint16_t x_offset, uint16_t y_offset,
                                uint16_t width, uint16_t height, int32_t *response)

FUNCTION
   Send a VC_DISPMAN_DISPLAY_OBJECT_MODIFY command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/
int vc_dispman_object_modify(uint32_t object,
                             uint16_t x_offset,
                             uint16_t y_offset,
                             uint16_t width,
                             uint16_t height,
                             int32_t *response) {
   DISPMAN_OBJECT_MODIFY_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;

   params.object = VC_HTOV32(object);
   params.src_x = VC_VTOH16(x_offset);
   params.src_y = VC_VTOH16(y_offset);
   params.width = VC_VTOH16(width);
   params.height = VC_VTOH16(height);

   if (dispman_inum < 0)
      return -1;
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_OBJECT_MODIFY, sizeof(DISPMAN_OBJECT_MODIFY_PARAM_T), &params, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}

/******************************************************************************
NAME
   vc_dispman_resource_create

SYNOPSIS
   int vc_dispman_resource_create(uint32_t *resource,
     VC_IMAGE_PARAM_T *image, VC_RESOURCE_TYPE_T type)

FUNCTION
   Send a VC_DISPMAN_RESOURCE_CREATE command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
******************************************************************************/
int vc_dispman_resource_create(uint32_t *resource,
    VC_IMAGE_PARAM_T *image, VC_RESOURCE_TYPE_T type) {

   DISPMAN_RESOURCE_CREATE_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;

   if (image == NULL) {
      return -1;
   }

   params.resource = resource;
   params.image.type = image->type;
   params.image.width = VC_HTOV32(image->width);
   params.image.height = VC_HTOV32(image->height);
   params.image.pitch = VC_HTOV32(image->pitch);
   params.image.size = VC_HTOV32(image->size);
   params.image.pointer = VC_HTOV32(image->pointer);
   params.type = type;

   if (dispman_inum < 0)
      return -1;
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_RESOURCE_CREATE,
      PAD16(sizeof(DISPMAN_RESOURCE_CREATE_PARAM_T)),
      &params, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *resource = VC_VTOH32(response_buffer.read_response);

   // VideoCore now sends back the image_data pointer in VideoCore memory.
   if (image->pointer == 0)
   {
      image->pointer = VC_VTOH32( response_buffer.extra_param ); /* Get image buffer pointer */
   }

   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}

/******************************************************************************
NAME
   vc_dispman_resource_delete

SYNOPSIS
   int vc_dispman_resource_delete(uint32_t resource, int32_t *response)

FUNCTION
   Send a VC_DISPMAN_RESOURCE_DELETE command. This will return immediately, though possibly with
   a VC_DISPMAN_FIFO_FULL return code if there is insufficient space in the fifo.
   In this case the message is not sent.

RETURNS
   Command sent: 0
   Otherwise non-zero
   Response from command
******************************************************************************/
int vc_dispman_resource_delete(uint32_t resource, int32_t *response) {

   DISPMAN_RESOURCE_DELETE_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;

   params.resource = VC_HTOV32(resource);

   if (dispman_inum < 0)
      return -1;
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_RESOURCE_DELETE, sizeof(DISPMAN_RESOURCE_DELETE_PARAM_T), &params, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}


int vc_dispman_resource_set_alpha(uint32_t resource, uint32_t alpha, int32_t *response) {

   DISPMAN_RESOURCE_SET_ALPHA_PARAM_T params;
   VC_DISPMAN_TB_RESP_T response_buffer;

   params.resource = resource;
   params.alpha    = VC_HTOV32(alpha);

   if (dispman_inum < 0)
      return -1;
   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_RESOURCE_SET_ALPHA, sizeof(DISPMAN_RESOURCE_SET_ALPHA_PARAM_T), &params, dispman_ievent);
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *response = VC_VTOH32(response_buffer.read_response);
   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);
   return 0;
}


/******************************************************************************
NAME
   vc_dispman_image_create

SYNOPSIS
   int vc_dispman_image_create(VC_IMAGE_PARAM_T *image, VC_IMAGE_FORMAT_T type,
      uint32_t width, uint32_t height, uint32_t pitch, void *pointer)

FUNCTION
   Initialise the image.

RETURNS
   Success: 0
   Otherwise non-zero
******************************************************************************/
int vc_dispman_image_create(VC_IMAGE_PARAM_T *image, VC_IMAGE_FORMAT_T type,
   uint32_t width, uint32_t height, uint32_t pitch, void *pointer) {
   //Could check upper limits on image size here, if known

   if(width == 0 || height == 0) return -1;

   if (pitch) {
      // Pitch must be 4-byte aligned.
      vc_assert((pitch&3) == 0);
      image->pitch = pitch;
   }
   else {
      switch(type) {
      case VC_FORMAT_RGB565:
      case VC_FORMAT_RGBA565:
      case VC_FORMAT_RGBA16:
         image->pitch = (width*2 + HOST_PITCH_ALIGNMENT-1)&~(HOST_PITCH_ALIGNMENT-1);
         break;
      case VC_FORMAT_RGB888:
         image->pitch = (width*3 + HOST_PITCH_ALIGNMENT-1)&~(HOST_PITCH_ALIGNMENT-1);
         break;
      case VC_FORMAT_RGBA32:
         image->pitch = (width*4 + HOST_PITCH_ALIGNMENT-1)&~(HOST_PITCH_ALIGNMENT-1);
         break;
      default:
         //Illegal type
         vc_assert(0);
         return -1;
      }
   }
   image->type = type;
   image->width = width;
   image->height = height;
   image->pointer = (uint32_t)pointer;

   //no need to height up to multiple of 16
   image->size = image->pitch * height;

   return 0;
}

/******************************************************************************
NAME
   vc_dispman_query_image_formats

PARAMS
   uint32_t *support_formats - the returned supported image formats
      
FUNCTION
   Returns the support image formats from the VMCS host

RETURNS
   Success: 0
   Otherwise non-zero
******************************************************************************/
int vc_dispman_query_image_formats( uint32_t *supported_formats )
{     
   VC_DISPMAN_TB_RESP_T response_buffer;

   if (dispman_inum < 0)
      return -1;

   vc_lock_obtain(host_dispman_lock);
   vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_QUERY_IMAGE_FORMATS, 0, NULL, dispman_ievent );
   // Read the response.
   vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(VC_DISPMAN_TB_RESP_T), dispman_ievent);
   *supported_formats = VC_VTOH32(response_buffer.read_response);

   // Flush the input pointer.
   vc_msgfifo_read_flush(dispman_inum);
   vc_lock_release(host_dispman_lock);

   return 0;
}

/******************************************************************************
NAME
   vc_dispman_copy_dirty_rows

PARAMS
   VC_IMAGE_PARAM_T *srcImage   - Pointer to source image
   VC_IMAGE_PARAM_T *dstImage   - Pointer to destination image
   int          numRows         - total number of rows in dirty row bits
   uint32_t    *dirtyRowBits    - Pointer to array of dirty row bits
      
FUNCTION
   Copies the indicated rows from one image (framebuffer) to another.

RETURNS
   Success: 0
   Otherwise non-zero
******************************************************************************/

#define ALIGN_UP(value, alignment) (((value)+((alignment)-1)) & ~((alignment)-1))
#define BITS_TO_UINT32S( bits )  ( ALIGN_UP( bits, 32 ) / 32 )
#define BITS_TO_BYTES( bits )    ( BITS_TO_UINT32S( bits ) * sizeof( uint32_t ))

int vc_dispman_copy_dirty_rows
(
    VC_IMAGE_PARAM_T   *srcImage, 
    VC_IMAGE_PARAM_T   *dstImage,
    int                 numRows,
    uint32_t           *dirtyRowBits,
    int32_t            *response
)
{
    VC_DISPMAN_TB_RESP_T        response_buffer;
    VC_COPY_DIRTY_ROWS_PARAM_T  cmd;
    int                         i;

    vc_assert( srcImage->pitch == dstImage->pitch );
    vc_assert( srcImage->height == dstImage->height );
    vc_assert( BITS_TO_BYTES( numRows ) <= sizeof( cmd.dirtyRowBits ));

    if ( dispman_inum < 0 )
    {
        return -1;
    }

    memset( &cmd, 0, sizeof( cmd ));

    cmd.src_ptr = VC_HTOV32( srcImage->pointer );
    cmd.dst_ptr = VC_HTOV32( dstImage->pointer );
    cmd.pitch   = VC_HTOV32( srcImage->pitch );
    cmd.numRows = VC_HTOV32( srcImage->height );

    for ( i = 0; i < BITS_TO_UINT32S(  numRows ); i++ ) 
    {
        cmd.dirtyRowBits[ i ] = VC_HTOV32( dirtyRowBits[ i ]);
    }

    vc_lock_obtain(host_dispman_lock);
    vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_COPY_DIRTY_ROWS, PAD16( sizeof( cmd )), &cmd, dispman_ievent );

    // Read the response.

    vc_msgfifo_read_blocking(dispman_inum, &response_buffer, sizeof(response_buffer), dispman_ievent);
    *response = VC_VTOH32(response_buffer.read_response);

    // Flush the input pointer.

    vc_msgfifo_read_flush(dispman_inum);
    vc_lock_release(host_dispman_lock);

    return 0;
}

/******************************************************************************
NAME
   vc_dispman_copyarea

PARAMS
   VC_IMAGE_PARAM_T   *image           - Pointer to the image where copyarea is
                                         performed
   uint32_t            bytes_per_line  - Bytes per line (pitch)
   uint32_t            bytes_per_pixel - Bytes per pixel
   uint32_t            max_width       - Maximum width in pixels
   uint32_t            max_height      - Maximum height in lines
   uint32_t            sx              - Copy area source X
   uint32_t            sy              - Copy area source Y
   uint32_t            dx              - Copy area destination X
   uint32_t            dy              - Copy area destination Y
   uint32_t            width           - Copy area width
   uint32_t            height          - Copy area height
   int32_t            *response        - Pointer to the returned msg
      
FUNCTION
   Copies the indicated area on the specified image (framebuffer)

RETURNS
   Success: 0
   Otherwise non-zero
******************************************************************************/
int vc_dispman_copyarea
(
   VC_IMAGE_PARAM_T    *image,
   uint32_t             bytes_per_line,
   uint32_t             bytes_per_pixel,
   uint32_t             max_width,
   uint32_t             max_height,
   uint32_t             sx,
   uint32_t             sy,
   uint32_t             dx,
   uint32_t             dy,
   uint32_t             width,
   uint32_t             height,
   int32_t             *response
)
{
    VC_DISPMAN_TB_RESP_T  response_buffer;
    VC_COPYAREA_PARAM_T   cmd;

    if ( dispman_inum < 0 )
    {
        return -1;
    }

    memset( &cmd, 0, sizeof( cmd ));

    cmd.img_ptr         = VC_HTOV32( image->pointer );
    cmd.bytes_per_line  = VC_HTOV32( bytes_per_line );
    cmd.bytes_per_pixel = VC_HTOV32( bytes_per_pixel );
    cmd.max_width       = VC_HTOV32( max_width );
    cmd.max_height      = VC_HTOV32( max_height );
    cmd.sx              = VC_HTOV32( sx );
    cmd.sy              = VC_HTOV32( sy );
    cmd.dx              = VC_HTOV32( dx );
    cmd.dy              = VC_HTOV32( dy );
    cmd.width           = VC_HTOV32( width );
    cmd.height          = VC_HTOV32( height );

    /* send the command */
    vc_lock_obtain(host_dispman_lock);
    vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_COPYAREA,
          sizeof( cmd ), &cmd, dispman_ievent );

    /* read the response */
    vc_msgfifo_read_blocking(dispman_inum, &response_buffer,
          sizeof(response_buffer), dispman_ievent);
    *response = VC_VTOH32(response_buffer.read_response);

    /* flush the input pointer */
    vc_msgfifo_read_flush(dispman_inum);
    vc_lock_release(host_dispman_lock);
    return 0;
}

/******************************************************************************
NAME
   vc_dispman_fillrect

PARAMS
   VC_IMAGE_PARAM_T   *image           - Pointer to the image where fillrect is
                                         performed
   uint32_t            bytes_per_line  - Bytes per line (pitch)
   uint32_t            bytes_per_pixel - Bytes per pixel
   uint32_t            max_width       - Maximum width in pixels
   uint32_t            max_height      - Maximum height in lines
   uint32_t            dx              - Fill rect destination X
   uint32_t            dy              - Fill rect destination Y
   uint32_t            width           - Fill rect width
   uint32_t            height          - Fill rect height
   uint32_t            color           - Fill rect color
   int32_t            *response        - Pointer to the returned msg
      
FUNCTION
   Fill color in the indicated area on the specified image (framebuffer)

RETURNS
   Success: 0
   Otherwise non-zero
******************************************************************************/
int vc_dispman_fillrect
(
   VC_IMAGE_PARAM_T    *image,
   uint32_t             bytes_per_line,
   uint32_t             bytes_per_pixel,
   uint32_t             max_width,
   uint32_t             max_height,
   uint32_t             dx,
   uint32_t             dy,
   uint32_t             width,
   uint32_t             height,
   uint32_t             color,
   int32_t             *response
)
{
    VC_DISPMAN_TB_RESP_T  response_buffer;
    VC_FILLRECT_PARAM_T   cmd;

    if ( dispman_inum < 0 )
    {
        return -1;
    }

    memset( &cmd, 0, sizeof( cmd ));

    cmd.img_ptr         = VC_HTOV32( image->pointer );
    cmd.bytes_per_line  = VC_HTOV32( bytes_per_line );
    cmd.bytes_per_pixel = VC_HTOV32( bytes_per_pixel );
    cmd.max_width       = VC_HTOV32( max_width );
    cmd.max_height      = VC_HTOV32( max_height );
    cmd.dx              = VC_HTOV32( dx );
    cmd.dy              = VC_HTOV32( dy );
    cmd.width           = VC_HTOV32( width );
    cmd.height          = VC_HTOV32( height );
    cmd.color           = VC_HTOV32( color );

    /* send the command */
    vc_lock_obtain(host_dispman_lock);
    vc_msgfifo_send_command_blocking(dispman_inum, VC_DISPMAN_DISPLAY_FILLRECT,
          sizeof( cmd ), &cmd, dispman_ievent );

    /* read the response */
    vc_msgfifo_read_blocking(dispman_inum, &response_buffer,
          sizeof(response_buffer), dispman_ievent);
    *response = VC_VTOH32(response_buffer.read_response);

    /* flush the input pointer */
    vc_msgfifo_read_flush(dispman_inum);
    vc_lock_release(host_dispman_lock);
    return 0;
}
