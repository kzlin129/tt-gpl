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



//#include <stdio.h>

#include "vchost.h"
#include "vcinterface.h"
#include "vciface.h"
#include "vcgencmd.h"

#include "vc_dispservice_defs.h"
#include "vc_dispman2.h"
#if defined(USE_FILESYS) && USE_FILESYS
#include "vcfilesys.h"
#endif
#if defined(USE_HOSTREQ) && USE_HOSTREQ
#include "vchostreq_int.h"
#endif
//#include "vcdsos.h"       // platforms with VOD
#include "vcstate_intern.h"

/******************************************************************************
Global data.
******************************************************************************/

VC_STATE_T vc_state;

/******************************************************************************
Local types and defines.
******************************************************************************/
// Define NO_VARGS if the vargs mechanism used for vc_gencmd does not work on the host platform
//#define NO_VARGS

/******************************************************************************
Static data.
******************************************************************************/

static const char *app_names[] =
{
  "error",                        // ringer is not loadable
  "filemgr.vll",
  "mplayer.vll",
  "mrecorder.vll",
  "download.vll",
  "pcmplayer.vll",
  "usbmgr.vll",
  "editor.vll"
};

/******************************************************************************
Static functions.
******************************************************************************/

static void start_services (void) {
   vc_interface_init();
   vc_gencmd_init();
   vc_dispman_init();
#if defined(USE_FILESYS) && USE_FILESYS
   vc_filesys_init();
#endif
#if defined(USE_HOSTREQ) && USE_HOSTREQ
   vc_hostreq_init();
#endif
   //vc_dserv_init();    // for platforms that have this
}

static void stop_services (VC_POWER_STATE_T power_state) {
   vc_dispman_stop();
#if defined(USE_FILESYS) && USE_FILESYS
   vc_filesys_stop();
#endif
#if defined(USE_HOSTREQ) && USE_HOSTREQ
   vc_hostreq_stop();
#endif
   //vc_dserv_stop()    // for platforms that have this
   // For gencmd, we prepare VideoCore for power-down or hibernation. There is no response.
   vc_gencmd_send("power_control videocore %d", power_state==VC_POWER_STATE_OFF ? 0 : 1);
   vc_gencmd_stop();
}

/******************************************************************************
NAME
   vc_state_init

SYNOPSIS
   int vc_state_init()

FUNCTION
   Initial vc_state data. Should be called prior to first booting VideoCore.

RETURNS
   int - non-zero for error
******************************************************************************/

int vc_state_init () {
   int retval = 0;
   // Memory should be all zero when we start up, so power_state should be VC_POWER_STATE_OFF.
   if (vc_state.power_state != VC_POWER_STATE_OFF) {
     vc_assert(0);
     retval = -1;
   }
   vc_state.power_state = VC_POWER_STATE_OFF;
   vc_state.num_loaded_apps = 0;
   return retval;
}

/******************************************************************************
NAME
   vc_state_set_power_state

SYNOPSIS
   int vc_state_set_power_state(VC_POWER_STATE_T state)

FUNCTION
   Used to power VideoCore up, or down, or put it into hibernation. Relevant
   services are started/stopped as necessary.

RETURNS
   int - non-zero for error
******************************************************************************/

int vc_state_set_power_state (VC_POWER_STATE_T new_state) {
   char response[32];     // length is actually immaterial
   int retval = 0;
   switch (vc_state.power_state) {
   case VC_POWER_STATE_OFF:
     // We are currently off.
     switch (new_state) {
     case VC_POWER_STATE_OFF:
       // OK, we already are.
       break;
     case VC_POWER_STATE_HIBERNATE:  // can't do this, must be booted first
     case VC_POWER_STATE_RUNNING:    // or this, must be booted first
     default:                                                // weird error
       vc_assert(0);
       retval = -1;
       break;
     }
     break;
   case VC_POWER_STATE_HIBERNATE:
     // We are currently in hibernation.
     switch (new_state) {
     case VC_POWER_STATE_OFF:
       // The baseband can just power us off. Services *should* be stopped already.
       vc_state.power_state = new_state;
       break;
     case VC_POWER_STATE_HIBERNATE:
       // we already are
       break;
     case VC_POWER_STATE_RUNNING:
       // We must tell VMCS to restore itself to its pre-hibernation state.
       start_services();
       vc_gencmd(response, sizeof(response), "power_control videocore 2");
       vc_state.power_state = new_state;
       break;
     default:
       vc_assert(0);
       retval = -1;
     }
     break;
   case VC_POWER_STATE_RUNNING:
     // We are currently running.
     switch (new_state) {
     case VC_POWER_STATE_OFF:
     case VC_POWER_STATE_HIBERNATE:
       // Stop services and tell VideoCore to prepare for power-down/hibernation.
       stop_services(new_state);
       vc_state.power_state = new_state;
       vc_state.num_loaded_apps = 0;
       break;
     case VC_POWER_STATE_RUNNING:
       // Already are. Start services in case they weren't.
       start_services();
       break;
     default:
       vc_assert(0);
       retval = -1;
     }
     break;
   default:
     vc_assert(0);
     retval = -1;
   }
   return retval;
}

/******************************************************************************
NAME
   vc_state_get_power_state

SYNOPSIS
   VC_POWER_STATE_T vc_state_get_power_state()

FUNCTION
   Return the current power state of VideoCore.

RETURNS
   VC_POWER_STATE_T
******************************************************************************/

VC_POWER_STATE_T vc_state_get_power_state (void) {
   return vc_state.power_state;
}

/******************************************************************************
NAME
   vc_state_service_running

SYNOPSIS
   int vc_state_service_running(VC_SERVICE_T service)

FUNCTION
   Return the service number of the service if it is running, else -1.

RETURNS
   int
******************************************************************************/

int vc_state_service_running (VC_SERVICE_T service) {
   int retval = -1;
   vc_assert(service < VC_SERVICE_MAXNUM);
   switch (service) {
   case VC_SERVICE_GENCMD:
     retval = vc_gencmd_inum();
     break;
   case VC_SERVICE_DISPMAN:
     retval = vc_dispman_inum();
     break;
#if defined(USE_FILESYS) && USE_FILESYS
   case VC_SERVICE_FILESYS:
     retval = vc_filesys_inum();
     break;
#endif
     /*
       case VC_SERVICE_DSERV:
       retval = vc_dserv_inum();
       break;
     */

#if defined(USE_HOSTREQ) && USE_HOSTREQ
   case VC_SERVICE_HOSTREQ:
     retval = vc_hostreq_inum();
     break;
#endif
        default:
            break;
   }
   return retval;
}

/******************************************************************************
NAME
   vc_state_application_name

SYNOPSIS
   const char *vc_state_application_name(VC_APPLICATION_T app)

FUNCTION
   Return the name of the given application.

RETURNS
   const char *
******************************************************************************/

// Return name to use to load an application.
const char *vc_state_application_name (VC_APPLICATION_T app) {
   vc_assert(app < VC_APPLICATION_MAXNUM);

   return app_names[(int)app];
}

/******************************************************************************
NAME
   vc_state_load_application

SYNOPSIS
   int vc_state_load_application(VC_APPLICATION_T app)

FUNCTION
   Load an application and return its task_id. Returns < 0 if an error occurs.

RETURNS
   int - non-zero for error
******************************************************************************/

int vc_state_load_application (VC_APPLICATION_T app) {
   char response[48];
   int task_id;
   if (vc_state.num_loaded_apps < VC_MAX_LOADED_APPS) {

#ifdef NO_VARGS
     sprintf(response,"load_application %s", vc_state_application_name(app) );
     vc_gencmd(response, sizeof(response), response );
#else
     vc_gencmd(response, sizeof(response), "load_application %s", vc_state_application_name(app));
#endif
     if (!vc_gencmd_number_property(response, (char*)"task", &task_id))
       task_id = -1;   // failed to load
     else {
       // add to list of loaded apps
       vc_state.loaded_apps[vc_state.num_loaded_apps].app = app;
       vc_state.loaded_apps[vc_state.num_loaded_apps].task_id = task_id;
       vc_state.num_loaded_apps++;
     }
   }
   return task_id;
}

/******************************************************************************
NAME
   vc_state_end_application

SYNOPSIS
   int vc_state_end_application(int task_id)

FUNCTION
   End the application with the given task_id. Return non-zero if an error occurs.

RETURNS
   int - non-zero for error
******************************************************************************/

// End an application. Return non-zero for an error.
int vc_state_end_application (int task_id) {
   int i;
   int retval = -1;
   for (i = 0; i < vc_state.num_loaded_apps; i++) {
     if (task_id == vc_state.loaded_apps[i].task_id) {
       // We end this app and remove from the list.
       char response[48];

#ifdef NO_VARGS
       sprintf(response,"end_application %d", task_id );
       vc_gencmd(response, sizeof(response), response);  // can't go wrong?
#else
       vc_gencmd(response, sizeof(response), "end_application %d", task_id);  // can't go wrong?
#endif
       vc_state.num_loaded_apps--;
       vc_state.loaded_apps[i].app = vc_state.loaded_apps[vc_state.num_loaded_apps].app;
       vc_state.loaded_apps[i].task_id = vc_state.loaded_apps[vc_state.num_loaded_apps].task_id;
       retval = 0;
       break;
     }
   }
   return retval;
}

/******************************************************************************
NAME
   vc_state_application_task_id

SYNOPSIS
   int vc_state_application_task_id(VC_APPLICATION_T app)

FUNCTION
   Return task_id of application if it is running, or -1 if it is not.

RETURNS
   int
******************************************************************************/

int vc_state_application_task_id (VC_APPLICATION_T app) {
   int i;
   for (i = 0; i < vc_state.num_loaded_apps; i++) {
     if (app == vc_state.loaded_apps[i].app)
       return vc_state.loaded_apps[i].task_id;
   }
   return -1;
}

/******************************************************************************
NAME
   vc_state_application_running

SYNOPSIS
   int vc_state_application_running(int task_id)

FUNCTION
   Return non-zero if the application is still running.

RETURNS
   int - non-zero for error
******************************************************************************/

int vc_state_application_running (int task_id) {
   int i;
   for (i = 0; i < vc_state.num_loaded_apps; i++) {
     if (task_id == vc_state.loaded_apps[i].task_id)
       return 1;
   }
   return 0;
}

