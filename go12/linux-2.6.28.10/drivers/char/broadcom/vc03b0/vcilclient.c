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


Project  :  OpenMAX IL
Module   :
File     :  $RCSfile: $
Revision :  $Revision: $

FILE DESCRIPTION
OpenMAX IL - Library for writing IL clients
=============================================================================*/

#if defined( __KERNEL__ )

#include <linux/string.h>

#else

#include <stdio.h>
#include <string.h>

#endif

#include <linux/broadcom/omx/OMX_Broadcom.h>
#include <linux/broadcom/vc03/vcilclient.h>
#include <linux/broadcom/vc03/vcos.h>
#include <linux/broadcom/vc03/vcilplatform.h>
#include <linux/broadcom/vc03/vcomx.h>

OMX_API OMX_ERRORTYPE OMX_APIENTRY vc_OMX_GetHandle(
   OMX_OUT OMX_HANDLETYPE* pHandle,
   OMX_IN  OMX_STRING cComponentName,
   OMX_IN  OMX_PTR pAppData,
   OMX_IN  OMX_CALLBACKTYPE* pCallBacks);
OMX_API OMX_ERRORTYPE OMX_APIENTRY vc_OMX_FreeHandle(
   OMX_IN  OMX_HANDLETYPE hComponent);

#define ilclienbt_debug_output(arg)

/******************************************************************************
Static data and types used only in this file.
******************************************************************************/

struct _ILEVENT_T {
   OMX_EVENTTYPE eEvent;
   OMX_U32 nData1;
   OMX_U32 nData2;
   OMX_PTR pEventData;
   struct _ILEVENT_T *next;
};

#define NUM_EVENTS 100
struct _ILCLIENT_T {
   ILEVENT_T *event_list;
   PLATFORM_SEMAPHORE_T event_sema;
   ILEVENT_T event_rep[NUM_EVENTS];

   ILCLIENT_CALLBACK_T port_settings_callback;
   void *port_settings_callback_data;
   ILCLIENT_CALLBACK_T eos_callback;
   void *eos_callback_data;
   ILCLIENT_CALLBACK_T error_callback;
   void *error_callback_data;
   ILCLIENT_BUFFER_CALLBACK_T fill_buffer_done_callback;
   void *fill_buffer_done_callback_data;
   ILCLIENT_BUFFER_CALLBACK_T empty_buffer_done_callback;
   void *empty_buffer_done_callback_data;
   ILCLIENT_CALLBACK_T facesdetected_callback;
   void *facesdetected_callback_data;
};

struct _COMPONENT_T {
   OMX_HANDLETYPE comp;
   int local_comp;
   PLATFORM_SEMAPHORE_T sema;
   PLATFORM_EVENTGROUP_T event;
   OMX_BUFFERHEADERTYPE *out_list;
   OMX_BUFFERHEADERTYPE *in_list;
   char name[32];
   unsigned int error_mask;
   unsigned int private;
   ILEVENT_T *list;
   ILCLIENT_T *client;
};

#define random_wait()
static char *states[] = {"Invalid", "Loaded", "Idle", "Executing", "Pause", "WaitingForResources"};

typedef enum {
   ILCLIENT_ERROR_UNPOPULATED  = 0x1,
   ILCLIENT_ERROR_SAMESTATE    = 0x2,
   ILCLIENT_ERROR_BADPARAMETER = 0x4,
} ILERROR_MASK_T;

/******************************************************************************
Static functions.
******************************************************************************/

static OMX_ERRORTYPE ilclient_empty_buffer_done(OMX_IN OMX_HANDLETYPE hComponent,
      OMX_IN OMX_PTR pAppData,
      OMX_IN OMX_BUFFERHEADERTYPE* pBuffer);
static OMX_ERRORTYPE ilclient_empty_buffer_done_error(OMX_IN OMX_HANDLETYPE hComponent,
      OMX_IN OMX_PTR pAppData,
      OMX_IN OMX_BUFFERHEADERTYPE* pBuffer);
static OMX_ERRORTYPE ilclient_fill_buffer_done(OMX_OUT OMX_HANDLETYPE hComponent,
      OMX_OUT OMX_PTR pAppData,
      OMX_OUT OMX_BUFFERHEADERTYPE* pBuffer);
static OMX_ERRORTYPE ilclient_fill_buffer_done_error(OMX_OUT OMX_HANDLETYPE hComponent,
      OMX_OUT OMX_PTR pAppData,
      OMX_OUT OMX_BUFFERHEADERTYPE* pBuffer);
static OMX_ERRORTYPE ilclient_event_handler(OMX_IN OMX_HANDLETYPE hComponent,
      OMX_IN OMX_PTR pAppData,
      OMX_IN OMX_EVENTTYPE eEvent,
      OMX_IN OMX_U32 nData1,
      OMX_IN OMX_U32 nData2,
      OMX_IN OMX_PTR pEventData);
static void ilclient_lock_events(ILCLIENT_T *st);
static void ilclient_unlock_events(ILCLIENT_T *st);

#define ilclient_debug_output OMX_DEBUG

/******************************************************************************
Global functions
******************************************************************************/

/***********************************************************
 * Name: ilclient_init
 *
 * Description: Creates ilclient pointer
 *
 * Returns: pointer to client structure
 ***********************************************************/
ILCLIENT_T *ilclient_init()
{
   ILCLIENT_T *st = platform_malloc(sizeof(ILCLIENT_T), __LINE__, __FILE__);
   int i;
   memset(st, 0, sizeof(ILCLIENT_T));

   i = platform_semaphore_create(&st->event_sema, 1);
   vc_assert(i == 0);

   ilclient_lock_events(st);
   st->event_list = NULL;
   for (i=0; i<NUM_EVENTS; i++)
   {
      st->event_rep[i].eEvent = -1; // mark as unused
      st->event_rep[i].next = st->event_list;
      st->event_list = st->event_rep+i;
   }
   ilclient_unlock_events(st);
   return st;
}

/***********************************************************
 * Name: ilclient_destroy
 *
 * Description: frees client state
 *
 * Returns: void
 ***********************************************************/
void ilclient_destroy(ILCLIENT_T *st)
{
   int i;

   i = platform_semaphore_delete(&st->event_sema);
   platform_free(st);
}

/***********************************************************
 * Name: ilclient_set_port_settings_callback
 *
 * Description: sets the callback used when receiving port settings
 * changed messages.  The data field in the callback function will be
 * the port index reporting the message.
 *
 * Returns: void
 ***********************************************************/
void ilclient_set_port_settings_callback(ILCLIENT_T *st, ILCLIENT_CALLBACK_T func, void *userdata)
{
   st->port_settings_callback = func;
   st->port_settings_callback_data = userdata;
}

/***********************************************************
 * Name: ilclient_set_eos_callback
 *
 * Description: sets the callback used when receiving eos flags.  The
 * data parameter in the callback function will be the port index
 * reporting an eos flag.
 *
 * Returns: void
 ***********************************************************/
void ilclient_set_eos_callback(ILCLIENT_T *st, ILCLIENT_CALLBACK_T func, void *userdata)
{
   st->eos_callback = func;
   st->eos_callback_data = userdata;
}

/***********************************************************
 * Name: ilclient_set_error_callback
 *
 * Description: sets the callback used when receiving error events.
 * The data parameter in the callback function will be the error code
 * being reported.
 *
 * Returns: void
 ***********************************************************/
void ilclient_set_error_callback(ILCLIENT_T *st, ILCLIENT_CALLBACK_T func, void *userdata)
{
   st->error_callback = func;
   st->error_callback_data = userdata;
}

/***********************************************************
 * Name: ilclient_set_fill_buffer_done_callback
 *
 * Description: sets the callback used when receiving
 * fill_buffer_done event
 *
 * Returns: void
 ***********************************************************/
void ilclient_set_fill_buffer_done_callback(ILCLIENT_T *st, ILCLIENT_BUFFER_CALLBACK_T func, void *userdata)
{
   st->fill_buffer_done_callback = func;
   st->fill_buffer_done_callback_data = userdata;
}

/***********************************************************
 * Name: ilclient_set_empty_buffer_done_callback
 *
 * Description: sets the callback used when receiving
 * empty_buffer_done event
 *
 * Returns: void
 ***********************************************************/
void ilclient_set_empty_buffer_done_callback(ILCLIENT_T *st, ILCLIENT_BUFFER_CALLBACK_T func, void *userdata)
{
   st->empty_buffer_done_callback = func;
   st->empty_buffer_done_callback_data = userdata;
}

/***********************************************************
 * Name: ilclient_set_facesdetected_callback
 *
 * Description: sets the callback used when face detection
 * events are received
 *
 * Returns: void
 ***********************************************************/
void ilclient_set_facesdetected_callback(ILCLIENT_T *st, ILCLIENT_CALLBACK_T func, void *userdata)
{
   st->facesdetected_callback = func;
   st->facesdetected_callback_data = userdata;
}

/***********************************************************
 * Name: ilclient_create_component
 *
 * Description: initialises a component state structure and creates
 * the IL component.  If local_comp is 1, creates a component local to
 * the client (rather than via the IL component service).  client_fill
 * and client_empty are flags indicating whether this component will
 * use client communication for filling and emptying buffers.
 *
 * Returns: 0 on success, -1 on failure
 ***********************************************************/
int ilclient_create_component(ILCLIENT_T *client, COMPONENT_T **comp, char *name,
                              int local_comp, int client_fill, int client_empty)
{
   OMX_CALLBACKTYPE callbacks;
   OMX_ERRORTYPE error;
   char component_name[128];
   int32_t status;

   *comp = platform_malloc(sizeof(COMPONENT_T), __LINE__, __FILE__);
   if(!*comp)
      return -1;

   memset(*comp, 0, sizeof(COMPONENT_T));

   status = platform_eventgroup_create(&(*comp)->event);
   vc_assert(status == 0);
   status = platform_semaphore_create(&(*comp)->sema, 1);
   vc_assert(status == 0);
   (*comp)->client = client;

   strcpy((*comp)->name, "cl:");
   strcat((*comp)->name, name);

   strcpy(component_name, "OMX.broadcom.");
   strcat(component_name, name);

   (*comp)->local_comp = local_comp;

   callbacks.EventHandler = ilclient_event_handler;
   callbacks.EmptyBufferDone = client_empty ? ilclient_empty_buffer_done : ilclient_empty_buffer_done_error;
   callbacks.FillBufferDone = client_fill ? ilclient_fill_buffer_done : ilclient_fill_buffer_done_error;

   //   if (local_comp)
   //     error = vc_OMX_GetHandle(&(*comp)->comp, component_name, *comp, &callbacks);
   // else
   error = OMX_GetHandle(&(*comp)->comp, component_name, *comp, &callbacks);
   
   if (error == OMX_ErrorNone)
      return 0;
   else
      return -1;
}

/***********************************************************
 * Name: ilclient_remove_event
 *
 * Description: Removes an event from a component event list.  ignore1
 * and ignore2 are flags indicating whether to not match on nData1 and
 * nData2 respectively.
 *
 * Returns: 0 if the event was removed.  -1 if no matching event was
 * found.
 ***********************************************************/
int ilclient_remove_event(COMPONENT_T *st, OMX_EVENTTYPE eEvent,
                          OMX_U32 nData1, int ignore1, OMX_IN OMX_U32 nData2, int ignore2)
{
   ILEVENT_T *cur, *prev;
   uint32_t set;
   ilclient_lock_events(st->client);

   cur = st->list;
   prev = NULL;

   while (cur && !(cur->eEvent == eEvent && (ignore1 || cur->nData1 == nData1) && (ignore2 || cur->nData2 == nData2)))
   {
      prev = cur;
      cur = cur->next;
   }

   if (cur == NULL)
   {
      ilclient_unlock_events(st->client);
      return -1;
   }

   if (prev == NULL)
      st->list = cur->next;
   else
      prev->next = cur->next;

   // add back into spare list
   cur->next = st->client->event_list;
   st->client->event_list = cur;
   cur->eEvent = -1; // mark as unused

   // if we're removing an error event, then clear the error bit from the eventgroup,
   // since the user might have been notified through the error callback, and then 
   // can't clear the event bit - this will then cause problems the next time they
   // wait for an error.
   if(eEvent == OMX_EventError)
      platform_eventgroup_get(&st->event, ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_OPERATION_OR_CONSUME, 0, &set);

   ilclient_unlock_events(st->client);
   return 0;
}

/***********************************************************
 * Name: ilclient_state_transition
 *
 * Description: Transitions a null terminated list of IL components to
 * a given state.  All components are told to transition in a random
 * order before any are checked for transition completion.
 *
 * Returns: void
 ***********************************************************/
void ilclient_state_transition(COMPONENT_T *list[], OMX_STATETYPE state)
{
   OMX_ERRORTYPE error;
   int i;
   int num, ret;
   uint32_t set;
   num=0;
   while (list[num])
      num++;

   // if we transition the supplier port first, it will call freebuffer on the non
   // supplier, which will correctly signal a port unpopulated error.  We want to
   // ignore these errors.
   if (state == OMX_StateLoaded)
      for (i=0; i<num; i++)
         list[i]->error_mask |= ILCLIENT_ERROR_UNPOPULATED;
   for (i=0; i<num; i++)
      list[i]->private = ((rand() >> 13) & 0xff)+1;

   for (i=0; i<num; i++)
   {
      // transition the components in a random order
      int j, min = -1;
      for (j=0; j<num; j++)
         if (list[j]->private && (min == -1 || list[min]->private > list[j]->private))
            min = j;

      list[min]->private = 0;

      random_wait();
      //Clear error event for this component
      platform_eventgroup_get(&list[min]->event, ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_OPERATION_OR_CONSUME, 0, &set);

      error = OMX_SendCommand(list[min]->comp, OMX_CommandStateSet, state, NULL);
      vc_assert(error == OMX_ErrorNone);
   }

   random_wait();

   for (i=0; i<num; i++)
   {
      ret = ilclient_wait_for_event(list[i], OMX_EventCmdComplete,
                                    OMX_CommandStateSet, 0, state, 0, ILCLIENT_STATE_CHANGED|ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_SUSPEND);
      if (ret != 0 && ilclient_remove_event(list[i], OMX_EventError, OMX_ErrorSameState, 0, 0, 1) < 0)
         vc_assert(0);
   }

   if (state == OMX_StateLoaded)
      for (i=0; i<num; i++)
         list[i]->error_mask &= ~ILCLIENT_ERROR_UNPOPULATED;
}

/***********************************************************
 * Name: ilclient_teardown_tunnels
 *
 * Description: tears down a null terminated list of tunnels.
 *
 * Returns: void
 ***********************************************************/
void ilclient_teardown_tunnels(TUNNEL_T *tunnel)
{
   int i;
   OMX_ERRORTYPE error;

   i=0;;
   while (tunnel[i].source)
   {
      error = OMX_SetupTunnel(tunnel[i].source->comp, tunnel[i].source_port, NULL, 0);
      vc_assert(error == OMX_ErrorNone);

      error = OMX_SetupTunnel(tunnel[i].sink->comp, tunnel[i].sink_port, NULL, 0);
      vc_assert(error == OMX_ErrorNone);
      i++;
   }
}

/***********************************************************
 * Name: ilclient_disable_tunnel
 *
 * Description: disables a tunnel by disabling the ports.  Allows
 * ports to signal same state error if they were already disabled.
 *
 * Returns: void
 ***********************************************************/
void ilclient_disable_tunnel(TUNNEL_T *tunnel)
{
   OMX_ERRORTYPE error;
   int ret;
   
   if(tunnel->source == 0 || tunnel->sink == 0)
      return;

   tunnel->source->error_mask |= ILCLIENT_ERROR_UNPOPULATED;
   tunnel->sink->error_mask |= ILCLIENT_ERROR_UNPOPULATED;

   error = OMX_SendCommand(tunnel->source->comp, OMX_CommandPortDisable, tunnel->source_port, NULL);
   vc_assert(error == OMX_ErrorNone);

   error = OMX_SendCommand(tunnel->sink->comp, OMX_CommandPortDisable, tunnel->sink_port, NULL);
   vc_assert(error == OMX_ErrorNone);

   ret = ilclient_wait_for_event(tunnel->source, OMX_EventCmdComplete,
                                 OMX_CommandPortDisable, 0, tunnel->source_port, 0,
                                 ILCLIENT_PORT_DISABLED | ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_SUSPEND);
   if (ret != 0 && ilclient_remove_event(tunnel->source, OMX_EventError, OMX_ErrorSameState, 0, 0, 1) < 0)
      vc_assert(0);

   ret = ilclient_wait_for_event(tunnel->sink, OMX_EventCmdComplete,
                                 OMX_CommandPortDisable, 0, tunnel->sink_port, 0,
                                 ILCLIENT_PORT_DISABLED | ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_SUSPEND);
   if (ret != 0 && ilclient_remove_event(tunnel->sink, OMX_EventError, OMX_ErrorSameState, 0, 0, 1) < 0)
      vc_assert(0);

   tunnel->source->error_mask &= ~ILCLIENT_ERROR_UNPOPULATED;
   tunnel->sink->error_mask &= ~ILCLIENT_ERROR_UNPOPULATED;
}

/***********************************************************
 * Name: ilclient_enable_tunnel
 *
 * Description: enables a tunnel by enabling the ports
 *
 * Returns: 0 on success, -1 on failure
 ***********************************************************/
int ilclient_enable_tunnel(TUNNEL_T *tunnel)
{
   OMX_STATETYPE state;
   OMX_ERRORTYPE error;

   error = OMX_SendCommand(tunnel->source->comp, OMX_CommandPortEnable, tunnel->source_port, NULL);
   vc_assert(error == OMX_ErrorNone);

   error = OMX_SendCommand(tunnel->sink->comp, OMX_CommandPortEnable, tunnel->sink_port, NULL);
   vc_assert(error == OMX_ErrorNone);

   // to complete, the sink component can't be in loaded state
   error = OMX_GetState(tunnel->sink->comp, &state);
   vc_assert(error == OMX_ErrorNone);
   if (state == OMX_StateLoaded)
   {
      ilclient_wait_for_event(tunnel->sink, OMX_EventCmdComplete,
                              OMX_CommandPortEnable, 0, tunnel->sink_port, 0,
                              ILCLIENT_PORT_ENABLED, PLATFORM_EVENTGROUP_SUSPEND);

      if (ilclient_change_component_state(tunnel->sink, OMX_StateIdle) < 0)
      {
         ilclient_disable_port(tunnel->source, tunnel->source_port);
         return -1;
      }
   }
   else
   {
      ilclient_wait_for_event(tunnel->sink, OMX_EventCmdComplete,
                              OMX_CommandPortEnable, 0, tunnel->sink_port, 0,
                              ILCLIENT_PORT_ENABLED, PLATFORM_EVENTGROUP_SUSPEND);
   }

   ilclient_wait_for_event(tunnel->source, OMX_EventCmdComplete,
                           OMX_CommandPortEnable, 0, tunnel->source_port, 0,
                           ILCLIENT_PORT_ENABLED, PLATFORM_EVENTGROUP_SUSPEND);

   return 0;
}


/***********************************************************
 * Name: ilclient_flush_tunnels
 *
 * Description: flushes all ports used in a null terminated list of
 * tunnels.  max specifies the maximum number of tunnels to flush from
 * the list, where max=0 means all tunnels.
 *
 * Returns: void
 ***********************************************************/
void ilclient_flush_tunnels(TUNNEL_T *tunnel, int max)
{
   OMX_ERRORTYPE error;
   int i;

   i=0;
   while (tunnel[i].source && (max == 0 || i < max))
   {
      error = OMX_SendCommand(tunnel[i].source->comp, OMX_CommandFlush, tunnel[i].source_port, NULL);
      vc_assert(error == OMX_ErrorNone);

      error = OMX_SendCommand(tunnel[i].sink->comp, OMX_CommandFlush, tunnel[i].sink_port, NULL);
      vc_assert(error == OMX_ErrorNone);

      ilclient_wait_for_event(tunnel[i].source, OMX_EventCmdComplete,
                              OMX_CommandFlush, 0, tunnel[i].source_port, 0,
                              ILCLIENT_PORT_FLUSH, PLATFORM_EVENTGROUP_SUSPEND);
      ilclient_wait_for_event(tunnel[i].sink, OMX_EventCmdComplete,
                              OMX_CommandFlush, 0, tunnel[i].sink_port, 0,
                              ILCLIENT_PORT_FLUSH, PLATFORM_EVENTGROUP_SUSPEND);
      i++;
   }
}


/***********************************************************
 * Name: ilclient_return_events
 *
 * Description: Returns all events from a component event list to the
 * list of unused event structures.
 *
 * Returns: void
 ***********************************************************/
void ilclient_return_events(COMPONENT_T *comp)
{
   ilclient_lock_events(comp->client);
   while (comp->list)
   {
      ILEVENT_T *next = comp->list->next;
      comp->list->next = comp->client->event_list;
      comp->client->event_list = comp->list;
      comp->list = next;
   }
   ilclient_unlock_events(comp->client);
}

/***********************************************************
 * Name: ilclient_cleanup_components
 *
 * Description: frees all components from a null terminated list and
 * deletes resources used in component state structure.
 *
 * Returns: void
 ***********************************************************/
void ilclient_cleanup_components(COMPONENT_T *list[])
{
   int i;
   OMX_ERRORTYPE error;

   i=0;
   while (list[i])
   {
      ilclient_return_events(list[i]);
      if (list[i]->comp)
      {
	//         if (list[i]->local_comp)
	//     error = vc_OMX_FreeHandle(list[i]->comp);
        // else
            error = OMX_FreeHandle(list[i]->comp);

         vc_assert(error == OMX_ErrorNone);
      }
      i++;
   }

   i=0;
   while (list[i])
   {
      int32_t ret;
      ret = platform_eventgroup_delete(&list[i]->event);
      vc_assert(ret == 0);
      ret = platform_semaphore_delete(&list[i]->sema);
      vc_assert(ret == 0);
      platform_free(list[i]);
      list[i] = NULL;
      i++;
   }
}

/***********************************************************
 * Name: ilclient_change_component_state
 *
 * Description: changes the state of a single component.  Note: this
 * may not be suitable if the component is tunnelled and requires
 * connected components to also change state.
 *
 * Returns: 0 on success, -1 on failure (note - trying to change to
 * the same state which causes a OMX_ErrorSameState is treated as
 * success)
 ***********************************************************/
int ilclient_change_component_state(COMPONENT_T *comp, OMX_STATETYPE state)
{
   OMX_ERRORTYPE error;
   int ret;
   error = OMX_SendCommand(comp->comp, OMX_CommandStateSet, state, NULL);
   vc_assert(error == OMX_ErrorNone);
   ret = ilclient_wait_for_event(comp, OMX_EventCmdComplete,
                                 OMX_CommandStateSet, 0, state, 0,
                                 ILCLIENT_STATE_CHANGED | ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_SUSPEND);
   // only allowed to fail with a same state error
   if (ret != 0 && ilclient_remove_event(comp, OMX_EventError, OMX_ErrorSameState, 0, 0, 1) < 0)
   {
      ilclient_remove_event(comp, OMX_EventError, 0, 1, 0, 1);
      return -1;
   }
   return 0;
}

/***********************************************************
 * Name: ilclient_disable_port
 *
 * Description: disables a port on a given component.
 *
 * Returns: void
 ***********************************************************/
void ilclient_disable_port(COMPONENT_T *comp, int portIndex)
{
   OMX_ERRORTYPE error;
   int ret;
   error = OMX_SendCommand(comp->comp, OMX_CommandPortDisable, portIndex, NULL);
   vc_assert(error == OMX_ErrorNone);
   ret = ilclient_wait_for_event(comp, OMX_EventCmdComplete,
                                 OMX_CommandPortDisable, 0, portIndex, 0,
                                 ILCLIENT_PORT_DISABLED | ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_SUSPEND);
   // only allowed to fail with a same state error
   if (ret != 0 && ilclient_remove_event(comp, OMX_EventError, OMX_ErrorSameState, 0, 0, 1) < 0)
      vc_assert(0);
}

/***********************************************************
 * Name: ilclient_enabled_port
 *
 * Description: enables a port on a given component.
 *
 * Returns: void
 ***********************************************************/
void ilclient_enable_port(COMPONENT_T *comp, int portIndex)
{
   OMX_ERRORTYPE error;
   int ret;
   error = OMX_SendCommand(comp->comp, OMX_CommandPortEnable, portIndex, NULL);
   vc_assert(error == OMX_ErrorNone);
   ret = ilclient_wait_for_event(comp, OMX_EventCmdComplete,
                                 OMX_CommandPortEnable, 0, portIndex, 0,
                                 ILCLIENT_PORT_ENABLED | ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_SUSPEND);
   // only allowed to fail with a same state error
   if (ret != 0 && ilclient_remove_event(comp, OMX_EventError, OMX_ErrorSameState, 0, 0, 1) < 0)
      vc_assert(0);
}

/***********************************************************
 * Name: ilclient_setup_tunnel
 *
 * Description: creates a tunnel between components that require that
 * ports be inititially disabled, then enabled after tunnel setup.  If
 * timeout is non-zero, it will initially wait until a port settings
 * changes message has been received by the output port.  If port
 * streams are supported by the output port, the requested port stream
 * will be selected.
 *
 * Returns: 0 indicates success, negative indicates failure.
 * -1: a timeout waiting for the parameter changed
 * -2: an error was returned instead of parameter changed
 * -3: no streams are available from this port
 * -4: requested stream is not available from this port
 * -5: the data format was not acceptable to the sink
 ***********************************************************/
int ilclient_setup_tunnel(TUNNEL_T *tunnel, unsigned int portStream, int timeout)
{
   OMX_ERRORTYPE error;
   OMX_PARAM_U32TYPE param;
   OMX_STATETYPE state;
   int32_t status;

   // source component must at least be idle, not loaded
   error = OMX_GetState(tunnel->source->comp, &state);
   vc_assert(error == OMX_ErrorNone);
   if (state == OMX_StateLoaded && ilclient_change_component_state(tunnel->source, OMX_StateIdle) < 0)
      return -2;

   // wait for the port parameter changed from the source port
   if(timeout)
   {
      status = ilclient_wait_for_event(tunnel->source, OMX_EventPortSettingsChanged,
                                       tunnel->source_port, 0, -1, 1,
                                       ILCLIENT_PARAMETER_CHANGED | ILCLIENT_EVENT_ERROR, timeout);
      
      if (status < 0)
         return status;
   }

   // disable ports
   ilclient_disable_tunnel(tunnel);

   // if this source port uses port streams, we need to select one of them before proceeding
   // if getparameter causes an error that's fine, nothing needs selecting
   param.nSize = sizeof(OMX_PARAM_U32TYPE);
   param.nVersion.nVersion = OMX_VERSION;
   param.nPortIndex = tunnel->source_port;
   if (OMX_GetParameter(tunnel->source->comp, OMX_IndexParamNumAvailableStreams, &param) == OMX_ErrorNone)
   {
      if (param.nU32 == 0)
      {
         // no streams available
         // leave the source port disabled, and return a failure
         return -3;
      }
      if (param.nU32 <= portStream)
      {
         // requested stream not available
         // no streams available
         // leave the source port disabled, and return a failure
         return -4;
      }

      param.nU32 = portStream;
      error = OMX_SetParameter(tunnel->source->comp, OMX_IndexParamActiveStream, &param);
      vc_assert(error == OMX_ErrorNone);
   }

   // now create the tunnel
   error = OMX_SetupTunnel(tunnel->source->comp, tunnel->source_port, tunnel->sink->comp, tunnel->sink_port);

   if (error != OMX_ErrorNone || ilclient_enable_tunnel(tunnel) < 0)
   {
      // probably format not compatible
      error = OMX_SetupTunnel(tunnel->source->comp, tunnel->source_port, NULL, 0);
      vc_assert(error == OMX_ErrorNone);
      error = OMX_SetupTunnel(tunnel->sink->comp, tunnel->sink_port, NULL, 0);
      vc_assert(error == OMX_ErrorNone);

      return -5;
   }

   return 0;
}

/***********************************************************
 * Name: ilclient_wait_for_event
 *
 * Description: waits for a given event to appear on a component event
 * list.  If not immediately present, will wait on that components
 * event group for the given event flag.
 *
 * Returns: 0 indicates success, negative indicates failure.
 * -1: a timeout was received.
 * -2: an error event was received.
 ***********************************************************/
int ilclient_wait_for_event(COMPONENT_T *comp, OMX_EVENTTYPE event,
                            OMX_U32 nData1, int ignore1, OMX_IN OMX_U32 nData2, int ignore2,
                            int event_flag, int suspend)
{
   int32_t status;
   uint32_t set;

   while (ilclient_remove_event(comp, event, nData1, ignore1, nData2, ignore2) < 0)
   {
      status = platform_eventgroup_get(&comp->event, event_flag, PLATFORM_EVENTGROUP_OPERATION_OR_CONSUME, suspend, &set);
      if (status != 0)
         return -1;
      if (set & ILCLIENT_EVENT_ERROR)
         return -2;
   }
   return 0;
}

/***********************************************************
 * Name: ilclient_get_output_buffer
 *
 * Description: Returns an output buffer returned from a component
 * using the OMX_FillBufferDone callback from the output list for the
 * given component and port index.
 *
 * Returns: pointer to buffer if available, otherwise NULL
 ***********************************************************/
OMX_BUFFERHEADERTYPE *ilclient_get_output_buffer(COMPONENT_T *comp, int portIndex)
{
   OMX_BUFFERHEADERTYPE *ret = NULL, *prev = NULL;

   platform_semaphore_wait(&comp->sema);
   ret = comp->out_list;
   while(ret != NULL && (int)ret->nOutputPortIndex != portIndex)
   {
      prev = ret;
      ret = ret->pPlatformPrivate;
   }

   if(ret)
   {
      if(prev == NULL)
         comp->out_list = ret->pPlatformPrivate;
      else
         prev->pPlatformPrivate = ret->pPlatformPrivate;

      ret->pPlatformPrivate = NULL;
   }
   platform_semaphore_post(&comp->sema);

   return ret;
}

/***********************************************************
 * Name: ilclient_get_input_buffer
 *
 * Description: Returns an input buffer return from a component using
 * the OMX_EmptyBufferDone callback from the output list for the given
 * component and port index.
 *
 * Returns: pointer to buffer if available, otherwise NULL
 ***********************************************************/
OMX_BUFFERHEADERTYPE *ilclient_get_input_buffer(COMPONENT_T *comp, int portIndex)
{
   OMX_BUFFERHEADERTYPE *ret = NULL, *prev = NULL;

   platform_semaphore_wait(&comp->sema);
   ret = comp->in_list;
   while(ret != NULL && ret->nInputPortIndex != portIndex)
   {
      prev = ret;
      ret = ret->pPlatformPrivate;
   }

   if(ret)
   {
      if(prev == NULL)
         comp->in_list = ret->pPlatformPrivate;
      else
         prev->pPlatformPrivate = ret->pPlatformPrivate;

      ret->pPlatformPrivate = NULL;
   }
   platform_semaphore_post(&comp->sema);

   return ret;
}


/******************************************************************************
Static functions
******************************************************************************/

/***********************************************************
 * Name: ilclient_lock_events
 *
 * Description: locks the client event structure
 *
 * Returns: void
 ***********************************************************/
static void ilclient_lock_events(ILCLIENT_T *st)
{
   int i;
   i = platform_semaphore_wait(&st->event_sema);
   vc_assert(i == 0);
}

/***********************************************************
 * Name: ilclient_unlock_events
 *
 * Description: unlocks the client event structure
 *
 * Returns: void
 ***********************************************************/
static void ilclient_unlock_events(ILCLIENT_T *st)
{
   int i;
   i = platform_semaphore_post(&st->event_sema);
   vc_assert(i == 0);
}

/***********************************************************
 * Name: ilclient_event_handler
 *
 * Description: event handler passed to core to use as component
 * callback
 *
 * Returns: success
 ***********************************************************/
static OMX_ERRORTYPE ilclient_event_handler(OMX_IN OMX_HANDLETYPE hComponent,
      OMX_IN OMX_PTR pAppData,
      OMX_IN OMX_EVENTTYPE eEvent,
      OMX_IN OMX_U32 nData1,
      OMX_IN OMX_U32 nData2,
      OMX_IN OMX_PTR pEventData)
{
   COMPONENT_T *st = (COMPONENT_T *) pAppData;
   ILEVENT_T *event;

   ilclient_lock_events(st->client);
   vc_assert(st->client->event_list);
   event = st->client->event_list;

   switch (eEvent) {
   case OMX_EventCmdComplete:
      switch (nData1) {
      case OMX_CommandStateSet:
	      ilclient_debug_output("%s: callback state changed (%s)\n", st->name, states[nData2]);
         platform_eventgroup_set(&st->event, ILCLIENT_STATE_CHANGED, PLATFORM_EVENTGROUP_OPERATION_OR);
         break;
      case OMX_CommandPortDisable:
	      ilclient_debug_output("%s: callback port disable %d\n", st->name, (int)nData2);
         platform_eventgroup_set(&st->event, ILCLIENT_PORT_DISABLED, PLATFORM_EVENTGROUP_OPERATION_OR);
         break;
      case OMX_CommandPortEnable:
	      ilclient_debug_output("%s: callback port enable %d\n", st->name, (int)nData2);
         platform_eventgroup_set(&st->event, ILCLIENT_PORT_ENABLED, PLATFORM_EVENTGROUP_OPERATION_OR);
         break;
      case OMX_CommandFlush:
	      ilclient_debug_output("%s: callback port flush %d\n", st->name, (int)nData2);
         platform_eventgroup_set(&st->event, ILCLIENT_PORT_FLUSH, PLATFORM_EVENTGROUP_OPERATION_OR);
         break;
      case OMX_CommandMarkBuffer:
	      ilclient_debug_output("%s: callback mark buffer %d\n", st->name, (int)nData2);
         platform_eventgroup_set(&st->event, ILCLIENT_MARKED_BUFFER, PLATFORM_EVENTGROUP_OPERATION_OR);
         break;
      default:
         vc_assert(0);
      }
      break;
   case OMX_EventError:
   {
      OMX_ERRORTYPE error = nData1;
      switch (error) {
      case OMX_ErrorPortUnpopulated:
         if (st->error_mask & ILCLIENT_ERROR_UNPOPULATED)
         {
            ilclient_debug_output("%s: ignore error: port unpopulated\n", st->name);
            event = NULL;
            break;
         }
         ilclient_debug_output("%s: port unpopulated\n", st->name);
         platform_eventgroup_set(&st->event, ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_OPERATION_OR);
         break;
      case OMX_ErrorSameState:
         if (st->error_mask & ILCLIENT_ERROR_SAMESTATE)
         {
            ilclient_debug_output("%s: ignore error: same state\n", st->name);
            event = NULL;
            break;
         }
         ilclient_debug_output("%s: same state\n", st->name);
         platform_eventgroup_set(&st->event, ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_OPERATION_OR);
         break;
      case OMX_ErrorBadParameter:
         if (st->error_mask & ILCLIENT_ERROR_BADPARAMETER)
         {
            ilclient_debug_output("%s: ignore error: bad parameter\n", st->name);
            event = NULL;
            break;
         }
         ilclient_debug_output("%s: bad parameter\n", st->name);
         platform_eventgroup_set(&st->event, ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_OPERATION_OR);
         break;
      case OMX_ErrorIncorrectStateTransition:
      case OMX_ErrorBadPortIndex:
      case OMX_ErrorStreamCorrupt:
      case OMX_ErrorInsufficientResources:
      case OMX_ErrorUnsupportedSetting:
         ilclient_debug_output("%s: error %x\n", st->name, error);
         platform_eventgroup_set(&st->event, ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_OPERATION_OR);
         break;
      default:
         vc_assert(0);
         ilclient_debug_output("%s: unexpected error %x\n", st->name, error);
         platform_eventgroup_set(&st->event, ILCLIENT_EVENT_ERROR, PLATFORM_EVENTGROUP_OPERATION_OR);
         break;
      }
      
      if(event && st->client->error_callback)
         st->client->error_callback(st->client->error_callback_data, st, nData1);
   }
   break;
   case OMX_EventBufferFlag:
     ilclient_debug_output("%s: buffer flag %d/%x\n", st->name, (int)nData1, (int)nData2);
      if (nData2 & OMX_BUFFERFLAG_EOS)
      {
         platform_eventgroup_set(&st->event, ILCLIENT_BUFFER_FLAG_EOS, PLATFORM_EVENTGROUP_OPERATION_OR);
         nData2 = OMX_BUFFERFLAG_EOS;
         if (st->client->eos_callback)
            st->client->eos_callback(st->client->eos_callback_data, st, nData1);
      }
      else
         vc_assert(0);
      break;
   case OMX_EventPortSettingsChanged:
     ilclient_debug_output("%s: port settings changed %d\n", st->name, (int)nData1);
      platform_eventgroup_set(&st->event, ILCLIENT_PARAMETER_CHANGED, PLATFORM_EVENTGROUP_OPERATION_OR);
      if (st->client->port_settings_callback)
         st->client->port_settings_callback(st->client->port_settings_callback_data, st, nData1);
      break;
   case OMX_EventMark:
      ilclient_debug_output("%s: buffer mark %p\n", st->name, pEventData);
      platform_eventgroup_set(&st->event, ILCLIENT_BUFFER_MARK, PLATFORM_EVENTGROUP_OPERATION_OR);
      break;
   case OMX_EventFaceDetection:
      ilclient_debug_output("%s: faces detected port %u, faces %u\n", st->name, (uint32_t)nData1, (uint32_t)nData2);
      if (st->client->facesdetected_callback)
         st->client->facesdetected_callback(st->client->facesdetected_callback_data, st, nData2);
      break;
   default:
      vc_assert(0);
      break;
   }

   if (event)
   {
      // fill in details
      event->eEvent = eEvent;
      event->nData1 = nData1;
      event->nData2 = nData2;
      event->pEventData = pEventData;

      // remove from top of spare list
      st->client->event_list = st->client->event_list->next;

      // put at head of component event queue
      event->next = st->list;
      st->list = event;
   }
   ilclient_unlock_events(st->client);

   return OMX_ErrorNone;
}

/***********************************************************
 * Name: ilclient_empty_buffer_done
 *
 * Description: passed to core to use as component callback, puts
 * buffer on list
 *
 * Returns:
 ***********************************************************/
static OMX_ERRORTYPE ilclient_empty_buffer_done(OMX_IN OMX_HANDLETYPE hComponent,
      OMX_IN OMX_PTR pAppData,
      OMX_IN OMX_BUFFERHEADERTYPE* pBuffer)
{
   COMPONENT_T *st = (COMPONENT_T *) pAppData;

   //ilclient_debug_output("%s: empty buffer done %p\n", st->name, pBuffer);

   platform_semaphore_wait(&st->sema);
   pBuffer->pPlatformPrivate = st->in_list;
   st->in_list = pBuffer;
   platform_semaphore_post(&st->sema);

   platform_eventgroup_set(&st->event, ILCLIENT_EMPTY_BUFFER_DONE, PLATFORM_EVENTGROUP_OPERATION_OR);

   if (st->client->empty_buffer_done_callback)
      st->client->empty_buffer_done_callback(st->client->empty_buffer_done_callback_data, st);

   return OMX_ErrorNone;
}

/***********************************************************
 * Name: ilclient_empty_buffer_done_error
 *
 * Description: passed to core to use as component callback, asserts
 * on use as client not expecting component to use this callback.
 *
 * Returns:
 ***********************************************************/
static OMX_ERRORTYPE ilclient_empty_buffer_done_error(OMX_IN OMX_HANDLETYPE hComponent,
      OMX_IN OMX_PTR pAppData,
      OMX_IN OMX_BUFFERHEADERTYPE* pBuffer)
{
   vc_assert(0);
   return OMX_ErrorNone;
}

/***********************************************************
 * Name: ilclient_fill_buffer_done
 *
 * Description: passed to core to use as component callback, puts
 * buffer on list
 *
 * Returns:
 ***********************************************************/
static OMX_ERRORTYPE ilclient_fill_buffer_done(OMX_OUT OMX_HANDLETYPE hComponent,
      OMX_OUT OMX_PTR pAppData,
      OMX_OUT OMX_BUFFERHEADERTYPE* pBuffer)
{
   COMPONENT_T *st = (COMPONENT_T *) pAppData;
   OMX_BUFFERHEADERTYPE *list;

   //ilclient_debug_output("%s: fill buffer done %p\n", st->name, pBuffer);

   platform_semaphore_wait(&st->sema);
   // insert at end of the list, so we process buffers in
   // the correct order
   list = st->out_list;
   while(list && list->pPlatformPrivate)
      list = list->pPlatformPrivate;

   if(!list)
      st->out_list = pBuffer;
   else
      list->pPlatformPrivate = pBuffer;
      
   pBuffer->pPlatformPrivate = NULL;
   platform_semaphore_post(&st->sema);

   platform_eventgroup_set(&st->event, ILCLIENT_FILL_BUFFER_DONE, PLATFORM_EVENTGROUP_OPERATION_OR);

   if (st->client->fill_buffer_done_callback)
      st->client->fill_buffer_done_callback(st->client->fill_buffer_done_callback_data, st);

   return OMX_ErrorNone;
}

/***********************************************************
 * Name: ilclient_fill_buffer_done_error
 *
 * Description: passed to core to use as component callback, asserts
 * on use as client not expecting component to use this callback.
 *
 * Returns:
 ***********************************************************/
static OMX_ERRORTYPE ilclient_fill_buffer_done_error(OMX_OUT OMX_HANDLETYPE hComponent,
      OMX_OUT OMX_PTR pAppData,
      OMX_OUT OMX_BUFFERHEADERTYPE* pBuffer)
{
   vc_assert(0);
   return OMX_ErrorNone;
}

OMX_HANDLETYPE ilclient_get_handle(COMPONENT_T *comp)
{
   return comp->comp;
}

#if defined( __KERNEL__ )

#include <linux/module.h>

EXPORT_SYMBOL (ilclient_init);
EXPORT_SYMBOL (ilclient_create_component);
EXPORT_SYMBOL (ilclient_cleanup_components);
EXPORT_SYMBOL (ilclient_change_component_state);
EXPORT_SYMBOL (ilclient_destroy);
EXPORT_SYMBOL (ilclient_wait_for_event);
EXPORT_SYMBOL (ilclient_disable_port);
EXPORT_SYMBOL (ilclient_set_empty_buffer_done_callback);
EXPORT_SYMBOL (ilclient_set_fill_buffer_done_callback);
EXPORT_SYMBOL (ilclient_setup_tunnel);
EXPORT_SYMBOL (ilclient_disable_tunnel);
EXPORT_SYMBOL (ilclient_enable_tunnel);
#endif


