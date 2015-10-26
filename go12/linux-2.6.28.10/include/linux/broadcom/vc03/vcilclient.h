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

/**
 * \file
 *
 * OpenMAX IL defines an integration layer that gives the client a
 * number of components.  These can be plugged together to create a
 * graph of interacting components to achieve a use-case such as
 * playing back a video or capturing a still image.  The IL Client
 * controls these components, creating and connecting them,
 * configuring how each will operate, as well as responding to error
 * conditions.
 *
 * In the course of developing multiple clients it becomes obvious
 * that there is a great deal of shared code between all clients.
 * This can include changing the state of a component, setting up a
 * tunnel, or the implementation of an EventHandler callback.  This
 * common code is included as part of VMCS-X since it will most likely
 * be of benefit for all IL client code using the VMCS-X OpenMAX IL
 * implementation.  It is completely optional, since it uses just base
 * primitive functions documented in the OpenMAX IL specification,
 * however provides a good starting point for developing clients using
 * the VMCS-X OpenMAX IL implementation.
 *
 * <b>insert diagram here</b>
 *
 * <b>IL Client Type interactions</b>
 *
 * In the application using the IL Client API there is typically no
 * need to know the internal structure of the type shown above, so
 * they are defined as opaque structures.
 *
 * \section example Example Use
 *
 * The following example IL client is taken from the iltest host
 * application.  It creates two components, <DFN>audio_capture</DFN> and
 * <DFN>audio_render</DFN>. These are first configured, and then tunnelled
 * together.  They are transitioned to executing, before the client
 * then sleeps for 10 seconds.  The components are then transitioned
 * to loaded state, before the tunnel is torn down and components, IL
 * client and IL core are all deinitialised.
 *
 * \code
 * void do_audio_capture_render_test()
 * {
 *    OMX_ERRORTYPE error;
 *    COMPONENT_T *audio_capture, *audio_render;
 *    COMPONENT_T *list[3] = {NULL, NULL, NULL};
 *    TUNNEL_T tunnel[2];
 *    ILCLIENT_T *client = ilclient_init();
 *    OMX_AUDIO_CONFIG_VOLUMETYPE vol;
 * 
 *    // initalize core
 *    error = OMX_Init();
 *    vc_assert(error == OMX_ErrorNone);
 * 
 *    // setup audio_capture component
 *    ilclient_create_component(client, &audio_capture, "audio_capture", 0, 0, 0);
 *    list[0] = audio_capture;
 * 
 *    // setup audio_render component
 *    ilclient_create_component(client, &audio_render, "audio_render", 0, 0, 0);
 *    list[1] = audio_render;
 * 
 *    vol.nSize = sizeof(OMX_AUDIO_CONFIG_VOLUMETYPE);
 *    vol.nVersion.nVersion = OMX_VERSION;
 *    vol.nPortIndex = 100;
 *    vol.bLinear = OMX_TRUE;
 *    vol.sVolume.nMin = 50;
 *    vol.sVolume.nValue = 50;
 *    vol.sVolume.nMax = 50;
 * 
 *    error = OMX_SetParameter(ILC_GET_HANDLE(audio_render), 
 *                             OMX_IndexConfigAudioVolume, &vol);
 *    vc_assert(error == OMX_ErrorNone);
 * 
 *    // Disable ports
 *    ilclient_disable_port(audio_render, 100);
 *    ilclient_disable_port(audio_render, 101);
 *    ilclient_disable_port(audio_capture, 180);
 *    ilclient_disable_port(audio_capture, 181);
 * 
 *    set_tunnel(tunnel, audio_capture, 180, audio_render, 100);
 *    set_tunnel(tunnel+1, NULL, 0, NULL, 0);
 * 
 *    if (ilclient_setup_tunnel(tunnel, 0, PLATFORM_EVENTGROUP_SUSPEND) != 0)
 *       vc_assert(0);
 * 
 *    // transition both component to executing - setting up the tunnel will
 *    // have transitioned both to idle already
 *    ilclient_state_transition(list, OMX_StateExecuting);
 * 
 *    platform_sleep(10000);
 * 
 *    // transition both components to idle
 *    ilclient_state_transition(list, OMX_StateIdle);
 * 
 *    // transition both components to loaded
 *    ilclient_state_transition(list, OMX_StateLoaded);
 * 
 *    // tear down the tunnel
 *    ilclient_teardown_tunnels(tunnel);
 * 
 *    // delete components
 *    ilclient_cleanup_components(list);
 * 
 *     // deinit core
 *    error = OMX_Deinit();
 *    assert(error == OMX_ErrorNone);
 * 
 *    ilclient_destroy(client);
 * }
 * \endcode
 */

#ifndef _IL_CLIENT_H
#define _IL_CLIENT_H

#include <linux/broadcom/omx/OMX_Broadcom.h>
#include "host_ilcore.h"


/**
 * The <DFN>ILCLIENT_T</DFN> structure encapsulates the state needed for the IL
 * Client API.  It contains a set of callback functions used to
 * communicate with the user.  It also includes a linked list of free
 * event structures.
 ***********************************************************/
typedef struct _ILCLIENT_T ILCLIENT_T;


/**
 * Each <DFN>ILEVENT_T</DFN> structure stores the result of an <DFN>EventHandler</DFN>
 * callback from a component, storing the event message type and any
 * parameters returned.
 ***********************************************************/
typedef struct _ILEVENT_T ILEVENT_T;


/**
 * The <DFN>COMPONENT_T</DFN> structure represents an IL component, together with
 * the necessary extra information required by the IL Client API.
 * This structure stores the handle to the OMX component, as well as
 * the event list containing all events sent by this component.  The
 * component state structure also holds a pair of buffer queues, for
 * input and output buffers returned to the client by the
 * <DFN>FillBufferDone</DFN> and <DFN>EmptyBufferDone</DFN> callbacks.  As some operations
 * result in error callbacks that can be ignored, an error mask is
 * maintained to allow some errors to be ignored.  A pointer to the
 * client state structure is also added.
 ***********************************************************/
struct _COMPONENT_T;
typedef struct _COMPONENT_T COMPONENT_T;


/**
 * The generic callback function is used for communicating events from
 * a particular component to the user.
 *
 * @param userdata The data returned from when the callback was registered.
 *
 * @param comp The component structure representing the component that
 * originated this event.
 *
 * @param data The relevant data field from the event.
 *
 * @return Void.
 ***********************************************************/
typedef void (*ILCLIENT_CALLBACK_T)(void *userdata, struct _COMPONENT_T *comp, OMX_U32 data);


/**
 * The buffer callback function is used for indicating that a
 * component has returned a buffer on a port using client buffer
 * communication.
 *
 * @param data The data returned from when the callback was registered.
 *
 * @param comp The component from which the buffer originated.
 *
 * @return Void.
 ***********************************************************/
typedef void (*ILCLIENT_BUFFER_CALLBACK_T)(void *data, struct _COMPONENT_T *comp);


/**
 * The event mask enumeration describes the possible events that the
 * user can ask to wait for when waiting for a particular event.
 ***********************************************************/
typedef enum {
   ILCLIENT_EMPTY_BUFFER_DONE  = 0x1,   /**< Set when a buffer is
                                           returned from an input
                                           port */

   ILCLIENT_FILL_BUFFER_DONE   = 0x2,   /**< Set when a buffer is
                                           returned from an output
                                           port */

   ILCLIENT_PORT_DISABLED      = 0x4,   /**< Set when a port indicates
                                           it has completed a disable
                                           command. */

   ILCLIENT_PORT_ENABLED       = 0x8,   /**< Set when a port indicates
                                           is has completed an enable
                                           command. */

   ILCLIENT_STATE_CHANGED      = 0x10,  /**< Set when a component
                                           indicates it has completed
                                           a state change command. */

   ILCLIENT_BUFFER_FLAG_EOS    = 0x20,  /**< Set when a port signals
                                           an EOS event. */

   ILCLIENT_PARAMETER_CHANGED  = 0x40,  /**< Set when a port signals a
                                           port settings changed
                                           event. */

   ILCLIENT_EVENT_ERROR        = 0x80,  /**< Set when a component
                                           indicates an error. */

   ILCLIENT_PORT_FLUSH         = 0x100, /**< Set when a port indicates
                                           is has completed a flush
                                           command. */

   ILCLIENT_MARKED_BUFFER      = 0x200, /**< Set when a port indicates
                                           it has marked a buffer. */

   ILCLIENT_BUFFER_MARK        = 0x400, /**< Set when a port indicates
                                           it has received a buffer
                                           mark. */
} ILEVENT_MASK_T;


/**
 * Some operations in this API act on a tunnel, so the tunnel state
 * structure (<DFN>TUNNEL_T</DFN>) is a convenient store of the source and sink
 * of the tunnel.  For each, a pointer to the relevant component state
 * structure and the port index is stored.
 ***********************************************************/
typedef struct {
   COMPONENT_T *source;  /**< The source component */
   int source_port;      /**< The output port index on the source component */
   COMPONENT_T *sink;    /**< The sink component */
   int sink_port;        /**< The input port index on the sink component */
} TUNNEL_T;


/**
 * The <DFN>set_tunnel</DFN> macro is a useful function that initialises a
 * <DFN>TUNNEL_T</DFN> structure.
 ***********************************************************/
#define set_tunnel(t,a,b,c,d) {TUNNEL_T *_ilct = (t); \
                               _ilct->source = (a); _ilct->source_port = (b); \
                               _ilct->sink = (c); _ilct->sink_port = (d);}

/**
 * For calling OpenMAX IL methods directory, we need to access the
 * <DFN>OMX_HANDLETYPE</DFN> corresponding to the <DFN>COMPONENT_T</DFN> structure.  This
 * macro enables this while keeping the <DFN>COMPONENT_T</DFN> structure opaque.
 * The parameter <DFN>x</DFN> should be of the type <DFN>*COMPONENT_T</DFN>.
 ***********************************************************/
#define ILC_GET_HANDLE(x) (*((OMX_HANDLETYPE *) (x)))


/**
 * For convenience we define <DFN>OMX_VERSION</DFN> to refer to the current
 * version of OpenMAX IL supported by VMCS-X, for initialised
 * OpenMAX IL structures.
 ***********************************************************/
#define OMX_VERSION 0x10101


/**
 * An IL Client structure is created by the <DFN>ilclient_init</DFN> method.
 * This structure is used when creating components, but otherwise is
 * not needed in other API functions.
 *
 * @return pointer to client structure
 ***********************************************************/
extern ILCLIENT_T *ilclient_init(void);

/**
 * When all components have been deleted, the IL Client structure can
 * be destroyed by calling the <DFN>ilclient_destroy</DFN> function.
 *
 * @param handle The client handle.  After calling this function, this
 * handle should not be used.
 *
 * @return void
 ***********************************************************/
extern void ilclient_destroy(ILCLIENT_T *handle);

/**
 * The <DFN>ilclient_set_port_settings_callback</DFN> function registers a
 * callback to be used when the <DFN>OMX_EventPortSettingsChanged</DFN> event is
 * received.  When the event is received, a pointer to the component
 * structure and port index is returned by the callback.
 *
 * @param handle The client handle
 *
 * @param func The callback function to use.  Calling this function
 * with a <DFN>NULL</DFN> function pointer will deregister any existing
 * registered callback.
 *
 * @param userdata Data to be passed back when calling the callback
 * function.
 *
 * @return void
 ***********************************************************/
extern void ilclient_set_port_settings_callback(ILCLIENT_T *handle,
                                                ILCLIENT_CALLBACK_T func,
                                                void *userdata);

/**
 * The <DFN>ilclient_set_eos_callback</DFN> function registers a callback to be
 * used when the <DFN>OMX_EventBufferFlag</DFN> is received with the
 * <DFN>OMX_BUFFERFLAG_EOS</DFN> flag set. When the event is received, a pointer
 * to the component structure and port index is returned by the
 * callback.
 *
 * @param handle The client handle
 *
 * @param func The callback function to use.  Calling this function
 * with a <DFN>NULL</DFN> function pointer will deregister any existing
 * registered callback.
 *
 * @param userdata Data to be passed back when calling the callback
 * function.
 *
 * @return void
 ***********************************************************/
extern void ilclient_set_eos_callback(ILCLIENT_T *handle,
                                      ILCLIENT_CALLBACK_T func,
                                      void *userdata);

/**
 * The <DFN>ilclient_set_error_callback</DFN> function registers a callback to be
 * used when the <DFN>OMX_EventError</DFN> is received from a component.  When
 * the event is received, a pointer to the component structure and the
 * error code are reported by the callback.
 *
 * @param handle The client handle
 *
 * @param func The callback function to use.  Calling this function
 * with a <DFN>NULL</DFN> function pointer will deregister any existing
 * registered callback.
 *
 * @param userdata Data to be passed back when calling the callback
 * function.
 *
 * @return void
 ***********************************************************/
extern void ilclient_set_error_callback(ILCLIENT_T *handle,
                                        ILCLIENT_CALLBACK_T func,
                                        void *userdata);

/**
 * The <DFN>ilclient_set_fill_buffer_done_callback</DFN> function registers a
 * callback to be used when a buffer passed to an output port using the
 * <DFN>OMX_FillBuffer</DFN> call is returned with the <DFN>OMX_FillBufferDone</DFN>
 * callback.  When the event is received, a pointer to the component
 * structure is returned by the callback.  The user may then use the
 * <DFN>ilclient_get_output_buffer</DFN> function to retrieve the buffer.
 *
 * @param handle The client handle
 *
 * @param func The callback function to use.  Calling this function
 * with a <DFN>NULL</DFN> function pointer will deregister any existing
 * registered callback.
 *
 * @param userdata Data to be passed back when calling the callback
 * function.
 *
 * @return void
 ***********************************************************/
extern void ilclient_set_fill_buffer_done_callback(ILCLIENT_T *handle,
                                                   ILCLIENT_BUFFER_CALLBACK_T func,
                                                   void *userdata);

/**
 * The <DFN>ilclient_set_empty_buffer_done_callback</DFN> function registers a
 * callback to be used when a buffer passed to an input port using the
 * <DFN>OMX_EmptyBuffer</DFN> call is returned with the <DFN>OMX_EmptyBufferDone</DFN>
 * callback.  When the event is received, a pointer to the component
 * structure is returned by the callback.  The user may then use the
 * <DFN>ilclient_get_input_buffer</DFN> function to retrieve the buffer.
 *
 * @param handle The client handle
 *
 * @param func The callback function to use.  Calling this function
 * with a <DFN>NULL</DFN> function pointer will deregister any existing
 * registered callback.
 *
 * @param userdata Data to be passed back when calling the callback
 * function.
 *
 * @return void
 ***********************************************************/
extern void ilclient_set_empty_buffer_done_callback(ILCLIENT_T *handle,
                                                    ILCLIENT_BUFFER_CALLBACK_T func,
                                                    void *userdata);

/**
 * The <DFN>ilclient_set_facesdetected_callback</DFN> function registers a
 * callback to be used when an OMX_EventFaceDetection event occurs.
 * When the event is received, a pointer to the component
 * structure is returned by the callback.  The user may then use an 
 * <DFN>OMX_GetConfig</DFN> call with <DFN>OMX_IndexConfigCommonFaceDetectionRegion</DFN>
*  to retrieve the face information.
 *
 * @param handle The client handle
 *
 * @param func The callback function to use.  Calling this function
 * with a <DFN>NULL</DFN> function pointer will deregister any existing
 * registered callback.
 *
 * @param userdata Data to be passed back when calling the callback
 * function.
 *
 * @return void
 ***********************************************************/
extern void ilclient_set_facesdetected_callback(ILCLIENT_T *handle,
                                                ILCLIENT_CALLBACK_T func,
                                                void *userdata);
                                                    
                                                    
/**
 * Components are created using the <DFN>ilclient_create_component</DFN>
 * function.  
 *
 * @param handle The client handle
 *
 * @param comp On successful creation, the component structure pointer
 * will be written back into <DFN>comp</DFN>.
 *
 * @param name The name of the component to be created.  Component
 * names are automatically prefixed with <DFN>"OMX.broadcom."</DFN>.  This name
 * will also be used in debugging messages added about this component.
 *
 * @param local_comp When the host application is being run natively
 * on VideoCore, we can set <DFN>local_comp</DFN> to be non-zero to indicate that
 * this component should be created natively, rather than through the
 * IL Component service.  To support this fully, the platform-dependent 
 * IL core needs to be integrated into the implementation of
 * IL Client.  This functionality is only of use when testing
 * tunneling via the IL component service, so this argument should normally be set to 0.
 *
 * @param client_fill This indicates whether the component will ever
 * return buffers from an output port to the client.
 *
 * @param client_empty This indicates whether the component will ever
 * return buffers from an input port to the client.
 *
 * @return 0 on success, -1 on failure
 ***********************************************************/
extern int  ilclient_create_component(ILCLIENT_T *handle, COMPONENT_T **comp, char *name,
                                      int local_comp, int client_fill, int client_empty);

/**
 * The <DFN>ilclient_cleanup_components</DFN> function deallocates all state associated with components and
 * frees the OpenMAX component handles. All tunnels connecting
 * components should have been torn down explicitly, and all components
 * must be in loaded state.
 *
 * @param list A null-terminated list of component pointers to be
 * deallocated.
 * 
 * @return void
 ***********************************************************/
extern void ilclient_cleanup_components(COMPONENT_T *list[]);


/**
 * The <DFN>ilclient_change_component_state</DFN> function changes the state of an individual component.
 * This will trigger the state change, and also wait for that
 * state change to be completed.  It should not be called if this
 * state change has dependencies on other components also changing
 * states.  Trying to change to the same state is treated as success.
 *
 * @param comp The component to change.
 *
 * @param state The new state to which to transition.
 *
 * @return 0 on success, -1 on failure.
 ***********************************************************/
extern int  ilclient_change_component_state(COMPONENT_T *comp, OMX_STATETYPE state);


/**
 * The <DFN>ilclient_state_transition</DFN> function transitions a set of
 * components that need to perform a simultaneous state transition; 
 * for example, when two components are tunnelled and the buffer
 * supplier port needs to allocate and pass buffers to a non-supplier
 * port.  All components are sent a command to change state, then the
 * function will wait for all components to signal that they have
 * changed state.
 *
 * @param list A null-terminated list of component pointers.
 *
 * @param state The new state to which to transition all components.
 *
 * @return void
 ***********************************************************/
extern void ilclient_state_transition(COMPONENT_T *list[], OMX_STATETYPE state);


/**
 * The <DFN>ilclient_disable_port</DFN> function disables a port on a given
 * component.  This function sends the disable port message to the
 * component and waits for the component to signal that this has taken
 * place.  The error that the port is already disabled is not treated
 * as a fatal error. 
 *
 * @param comp The component containing the port to disable.
 *
 * @param portIndex The port index of the port to disable.  This must
 * be a named port index, rather than a <DFN>OMX_ALL</DFN> value.
 *
 * @return void
 ***********************************************************/
extern void ilclient_disable_port(COMPONENT_T *comp, int portIndex);


/**
 * The <DFN>ilclient_enable_port</DFN> function enables a port on a given
 * component.  This function sends the enable port message to the
 * component and waits for the component to signal that this has taken
 * place.  The error that the port is already enabled is not treated
 * as a fatal error.
 *
 * @param comp The component containing the port to enable.
 *
 * @param portIndex The port index of the port to enable.  This must
 * be a named port index, rather than a <DFN>OMX_ALL</DFN> value.

 * @return void
 ***********************************************************/
extern void ilclient_enable_port(COMPONENT_T *comp, int portIndex);


/**
 * With a populated tunnel structure, the <DFN>ilclient_setup_tunnel</DFN>
 * function connects the tunnel.  It first transitions the source
 * component to idle if currently in loaded state, and then optionally
 * checks the source event list for a port settings changed event from
 * the source port.  If this event is not in the event queue then this
 * function optionally waits for it to arrive.
 *
 * Both ports are then disabled, and the source port is inspected for
 * a port streams parameter.  If this is supported, then the
 * <DFN>portStream</DFN> argument is used to select which port stream to use.
 * The two ports are then tunnelled using the <DFN>OMX_SetupTunnel</DFN>
 * function.  If this is successful, then both ports are enabled.
 * Note that for disabling and enabling the tunnelled ports, the
 * function <DFN>ilclient_disable_tunnel</DFN> and <DFN>ilclient_enable_tunnel</DFN> are
 * used, so the relevant documentation for those functions applies
 * here.
 *
 * @param tunnel The tunnel structure representing the tunnel to
 * set up.
 *
 * @param portStream If port streams are supported on the output port
 * of the tunnel, then this parameter indicates the port stream to
 * select on this port.
 *
 * @param timeout The time duration in milliseconds to wait for the
 * output port to signal a port settings changed event before
 * returning a timeout failure.  If this is 0, then we do not check
 * for a port settings changed before setting up the tunnel.
 *
 * @return 0 indicates success, negative indicates failure.
 *  - -1: a timeout waiting for the parameter changed
 *  - -2: an error was returned instead of parameter changed
 *  - -3: no streams are available from this port
 *  - -4: requested stream is not available from this port
 *  - -5: the data format was not acceptable to the sink
 ***********************************************************/
extern int  ilclient_setup_tunnel(TUNNEL_T *tunnel, unsigned int portStream, int timeout);


/**
 * The <DFN>ilclient_disable_tunnel</DFN> function disables both ports listed in
 * the tunnel structure.  It will send a port disable command to each
 * port, then waits for both to indicate they have completed the
 * transition.  The errors <DFN>OMX_ErrorPortUnpopulated</DFN> and
 * <DFN>OMX_ErrorSameState</DFN> are both ignored by this function; the former
 * since the first port to disable may deallocate buffers before the
 * second port has been disabled, leading to the second port reporting
 * the unpopulated error.
 *
 * @param tunnel The tunnel to disable.
 *
 * @return void
 ***********************************************************/
extern void ilclient_disable_tunnel(TUNNEL_T *tunnel);


/**
 * The <DFN>ilclient_enable_tunnel</DFN> function enables both ports listed in
 * the tunnel structure.  It will first send a port enable command to
 * each port.  It then checks whether the sink component is not in
 * loaded state - if so, the function waits for both ports to complete
 * the requested port enable.  If the sink component was in loaded
 * state, then this component is transitioned to idle to allow the
 * ports to exchange buffers and enable the ports.  This is since
 * typically this function is used when creating a tunnel between two
 * components, where the source component is processing data to enable
 * it to report the port settings changed event, and the sink port has
 * yet to be used.  Before transitioning the sink component to idle,
 * this function waits for the sink port to be enabled - since the
 * component is in loaded state, this will happen quickly.  If the
 * transition to idle fails, the sink component is transitioned back
 * to loaded and the source port disabled.  If the transition
 * succeeds, the function then waits for the source port to complete
 * the requested port enable.
 *
 * @param tunnel The tunnel to enable.
 *
 * @return 0 on success, -1 on failure.
 ***********************************************************/
extern int ilclient_enable_tunnel(TUNNEL_T *tunnel);


/**
 * The <DFN>ilclient_flush_tunnels</DFN> function will flush a number of tunnels
 * from the list of tunnels presented.  For each tunnel that is to be
 * flushed, both source and sink ports are sent a flush command.  The
 * function then waits for both ports to report they have completed
 * the flush operation.
 *
 * @param tunnel List of tunnels.  The list must be terminated with a
 * tunnel structure with <DFN>NULL</DFN> component entries.
 *
 * @param max The maximum number of tunnels to flush from the list.
 * A value of 0 indicates that all tunnels in the list are flushed.
 *
 * @return void
 ***********************************************************/
extern void ilclient_flush_tunnels(TUNNEL_T *tunnel, int max);


/**
 * The <DFN>ilclient_teardown_tunnels</DFN> function tears down all tunnels in
 * the list of tunnels presented.  For each tunnel in the list, the
 * <DFN>OMX_SetupTunnel</DFN> is called on the source port and on the sink port,
 * where for both calls the destination component is <DFN>NULL</DFN> and the
 * destination port is zero.  The VMCS-X IL implementation requires
 * that all tunnels are torn down in this manner before components are
 * freed. 
 *
 * @param tunnels List of tunnels to teardown.  The list must be
 * terminated with a tunnel structure with <DFN>NULL</DFN> component entries.
 *
 * @return void
 ***********************************************************/
extern void ilclient_teardown_tunnels(TUNNEL_T *tunnels);


/**
 * The <DFN>ilclient_get_output_buffer</DFN> function returns an output buffer
 * that was returned from a component using the <DFN>OMX_FillBufferDone</DFN>
 * callback.
 *
 * @param comp The component that returned the buffer.
 *
 * @param portIndex The port index on the component that the buffer
 * was returned from.
 *
 * @return Pointer to buffer if available, otherwise <DFN>NULL</DFN>.
 ***********************************************************/
extern OMX_BUFFERHEADERTYPE *ilclient_get_output_buffer(COMPONENT_T *comp, int portIndex);


/**
 * The <DFN>ilclient_get_input_buffer</DFN> function returns an input buffer
 * that was returned from a component using the <DFN>OMX_EmptyBufferDone</DFN>
 * callback.
 *
 * @param comp The component that returned the buffer.
 *
 * @param portIndex The port index on the component from which the buffer
 * was returned.
 *
 * @return pointer to buffer if available, otherwise <DFN>NULL</DFN>
 ***********************************************************/
extern OMX_BUFFERHEADERTYPE *ilclient_get_input_buffer(COMPONENT_T *comp, int portIndex);


/**
 * The <DFN>ilclient_remove_event</DFN> function queries the event list for the
 * given component, matching against the given criteria.  If a matching
 * event is found, it is removed and added to the free event list.
 *
 * @param comp The component that returned the matching event.
 *
 * @param event The event type of the matching event.
 *
 * @param nData1 The <DFN>nData1</DFN> field of the matching event.
 *
 * @param ignore1 Whether to ignore the <DFN>nData1</DFN> field when finding a
 * matching event.  A value of 0 indicates that <DFN>nData1</DFN> must match, a
 * value of 1 indicates that <DFN>nData1</DFN> does not have to match.
 *
 * @param nData2 The <DFN>nData2</DFN> field of the matching event.
 *
 * @param ignore2 Whether to ignore the <DFN>nData2</DFN> field when finding a
 * matching event.  A value of 0 indicates that <DFN>nData2</DFN> must match, a
 * value of 1 indicates that <DFN>nData2</DFN> does not have to match.
 *
 * @return 0 if the event was removed.  -1 if no matching event was
 * found.
 ***********************************************************/
extern int  ilclient_remove_event(COMPONENT_T *comp, OMX_EVENTTYPE event,
                                     OMX_U32 nData1, int ignore1,
                                     OMX_U32 nData2, int ignore2);


/**
 * The <DFN>ilclient_return_events</DFN> function removes all events from a
 * component event list and adds them to the IL client free event
 * list.  This function is typically never needs to be called, and is
 * automatically called when components are freed.
 *
 * @param comp The component from which all events should be moved to
 * the free list.
 *
 * @return void
 ***********************************************************/
extern void ilclient_return_events(COMPONENT_T *comp);


/**
 * The <DFN>ilclient_wait_for_event</DFN> function is similar to
 * <DFN>ilclient_remove_event</DFN>, but allows the caller to block until that
 * event arrives.
 *
 * @param comp The component that returned the matching event.
 *
 * @param event The event type of the matching event.
 *
 * @param nData1 The <DFN>nData1</DFN> field of the matching event.
 *
 * @param ignore1 Whether to ignore the <DFN>nData1</DFN> field when finding a
 * matching event.  A value of 0 indicates that <DFN>nData1</DFN> must match, a
 * value of 1 indicates that <DFN>nData1</DFN> does not have to match.
 *
 * @param nData2 The <DFN>nData2</DFN> field of the matching event.
 *
 * @param ignore2 Whether to ignore the <DFN>nData2</DFN> field when finding a
 * matching event.  A value of 0 indicates that <DFN>nData2</DFN> must match, a
 * value of 1 indicates that <DFN>nData2</DFN> does not have to match.
 *
 * @param event_flag Specifies a bitfield of IL client events to wait
 * for, given in <DFN>ILEVENT_MASK_T</DFN>.  If any of these events are signalled
 * by the component, the event list is then re-checked for a matching
 * event.  If the <DFN>ILCLIENT_EVENT_ERROR</DFN> bit is included, and this bit
 * is signalled by the component, then the function will return.
 *
 * @param timeout Specifies how long to block for in milliseconds
 * before returning a failure.
 *
 * @return 0 indicates success, negative indicates failure.
 *  - -1: a timeout was received.
 *  - -2: an error event was received.
 ***********************************************************/
extern int  ilclient_wait_for_event(COMPONENT_T *comp, OMX_EVENTTYPE event,
                                       OMX_U32 nData1, int ignore1,
                                       OMX_U32 nData2, int ignore2,
                                       int event_flag, int timeout);

/**
 * The <DFN>ilclient_debug_output</DFN> function adds a message to a 
 * host-specific debug display.  For a local VideoCore host the message is
 * added to the internal message log.  For a Win32 host the message is
 * printed to the debug display.  This function should be customised
 * when IL client is ported to another host platform.
 * 
 * @param format A message to add, together with the variable
 * argument list similar to <DFN>printf</DFN> and other standard C functions.
 *
 * @return void
 ***********************************************************/
extern void ilclient_debug_output(char *format, ...);

/**
  * Return the underlying OMX component held by an IL client handle.
  *
  * @param client  ILC client handle
  */
extern OMX_HANDLETYPE ilclient_get_handle(COMPONENT_T *comp);

/**
  * Macros to manipulate OMX_TICKS values.
  */

#ifndef OMX_SKIP64BIT

#define ilclient_ticks_from_s64(s) (s)
#define ilclient_ticks_to_s64(t)   (t)

#else

static inline OMX_TICKS ilclient_ticks_from_s64(int64_t s) {
   OMX_TICKS ret;
   ret.nLowPart = s;
   ret.nHighPart = s>>32;
   return ret;
}
static inline int64_t ilclient_ticks_to_s64(OMX_TICKS t) {
   uint64_t u = t.nLowPart | ((uint64_t)t.nHighPart << 32);
   return u;
}


#endif /* OMX_SKIP64BIT */


#endif /* ILCLIENT_H */
