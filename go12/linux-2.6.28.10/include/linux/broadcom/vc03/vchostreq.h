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




#ifndef VCHOSTREQ_H
#define VCHOSTREQ_H

#include "vc_hostreq_defs.h"
#include <linux/time.h>

extern int vc_hostreq_init (void);

extern void vc_hostreq_stop (void);

/* Supplies current key settings (preferably when there is a change) */
extern void vc_hostreq_keychange(keys_t keys);

/* Sets a user notify function to be called for the given notify event */
extern int32_t vc_hostreq_set_notify( const VC_HRNOTIFY_T notify_event, VC_HRNOTIFY_CALLBACK_T notifyfunc );

/*---------------------------------------------------------------------------*/
/*** The following require a host/application specific implementation ***/

/* Change key capture settings (might be ignored) */
extern void vc_hostreq_capturekeys(keys_t keymask);


/* Play a zero terminated sequence on the vibrator */
extern void vc_hostreq_vibratorplay(const int *sequence);


/* Stop the vibrator sequence immediately */
extern void vc_hostreq_vibratorstop(void);


/* Switch backlight on or off. */
extern void vc_hostreq_keylight(int state);


/* Set LEDs to specific brightness and colour levels. */
extern void vc_hostreq_setleds(led_t ledvalues);


/* Returns seconds since midnight (00:00.00) Jan 1 1970, local time. */
extern time_t vc_hostreq_time(void);


/* Send an event with the given eventcode to the host. */
extern void vc_hostreq_notify(int eventcode, int param);

/* Receive config data for subsequent hostreq_rendertext events. */
extern void vc_hostreq_configtext(char *config_data, int len);

/* Render a text string as a bitmap and then return it to VideoCore. */
extern void vc_hostreq_rendertext(char *text, int len);

/* Receive link information from a media file. */
extern void vc_hostreq_linkdata(char *link_data, int len);

/* Receive DMB FIC data */
extern void vc_hostreq_dmb_fic(char *data, int len);

/* Receive DMB PAD data */
extern void vc_hostreq_dmb_pad(char *data, int len);

/* Receive DMB DATA data */
extern void vc_hostreq_dmb_data(char *data, int len);

/* Request keypress info (from RTSP server). */
extern void vc_hostreq_keyin(void);


/*---------------------------------------------------------------------------*/
/***  Define a table of functions to be filled in for a specific platform  ***/

typedef struct {
   void (*fn_capturekeys)(keys_t keymask);
   void (*fn_vibratorplay)(const int *sequence);
   void (*fn_vibratorstop)(void);
   void (*fn_keylight)(int state);
   void (*fn_setleds)(led_t ledvalues);
   time_t (*fn_time)(void);
   void (*fn_notify)(int eventcode, int param);
   void (*fn_configtext)(char *config_data, int len);
   void (*fn_rendertext)(char *text, int len);
   void (*fn_linkdata)(char *link_data, int len);
   void (*fn_dmb_fic)(char *data, int len);
   void (*fn_dmb_pad)(char *data, int len);
   void (*fn_dmb_data)(char *data, int len);
   void (*fn_keyin)(void);
} VC_HOSTREQ_FUNCS_T;

extern VC_HOSTREQ_FUNCS_T vc_hostreq_functable;

#endif
