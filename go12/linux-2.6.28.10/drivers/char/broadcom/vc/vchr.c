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



/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

       V I D E O C O R E   H O S T   R E Q E U E S T   M E S S A G E
                       H A N D L I N G   T A S K


GENERAL DESCRIPTION
  This is the top-level interface task that watches for VideoCore file system
  requests and responses.
  It acts on them and kicks all the other tasks that may be waiting on
  particular services.

INITIALIZATION AND SEQUENCING REQUIREMENTS
  This task should be started up with ????????

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*===========================================================================

                        EDIT HISTORY FOR MODULE

$Header: /projects/Videocore/repository/vmcs_host/vc02_generic/vchr.c,v 1.4 2005/02/08 15:32:52 rao Exp $

when       who     what, where, why
--------   ---     ----------------------------------------------------------



===========================================================================*/

/*===========================================================================

                     INCLUDE FILES FOR MODULE

===========================================================================*/

#include "vchostreq.h"
#include "vchostreq_int.h"
#include "vchost.h"
#include "vchost_int.h"

int vc_keymask = 0;

void vc_hrmh_task( void )
{
  void *event;

  event = vc_hostreq_read_event();

  vc_event_blocking();

  while (1)
  {
    vc_event_wait(event);
    vc_hostreq_poll_message_fifo();
  }
}

/* Stubs that must be filled in for host. */

/* Change key capture settings (might be ignored) */
void vc_hostreq_capturekeys(keys_t keymask) {
   vc_keymask = keymask;
}

/* Play a zero terminated sequence on the vibrator */
void vc_hostreq_vibratorplay(const int *sequence) {
  UNUSED_PARAMETER(sequence);
}

/* Stop the vibrator sequence immediately */
void vc_hostreq_vibratorstop(void) {
}

/* Switch backlight on or off. */
void vc_hostreq_keylight(int state) {
  UNUSED_PARAMETER(state);
}

/* Set LEDs to specific brightness and colour levels. */
void vc_hostreq_setleds(led_t ledvalues) {
  UNUSED_PARAMETER(ledvalues);
}

/* Returns seconds since midnight (00:00.00) Jan 1 1970, local time. */
time_t vc_hostreq_time(void) 
{
   struct   timeval  tv;

   // On the ARM, boot times start out in 1969. On the MIPS, it seems to default
   // to the year 2000. So what we do, is that if the year is less than 2001
   // then we assume that the time hasn't been set by the NTP server
   // and we tell the VC02 that the time is 0.
   //

   do_gettimeofday( &tv );

   // There are rougly 365 * 24 * 60 * 60 seconds in a year

   if (( tv.tv_sec > 0 ) && ( tv.tv_sec < (( 2001 - 1970 ) * 365 * 24 * 60 * 60 )))
   {
      // We assume that the time hasn't been set, so we just report 0 to the VC02

      return 0;
   }

   // gVcGmtOffset gets sent down by vc-fuse to give us the correct offset from 
   // GMT.

   return tv.tv_sec + gVcGmtOffset;
}

/* Send an event with the given eventcode to the host. */
void vc_hostreq_notify(int eventcode, int param) {
  UNUSED_PARAMETER(eventcode);
  UNUSED_PARAMETER(param);
}

/* Receive config string for any subsequent text data. */
void vc_hostreq_configtext(char *config_data, int len) {
  UNUSED_PARAMETER(config_data);
  UNUSED_PARAMETER(len);
}

/* Render text as a bitmap and return to VideoCore. */
void vc_hostreq_rendertext(char *text, int len) {
  UNUSED_PARAMETER(text);
  UNUSED_PARAMETER(len);
}

/* Receive link information from a media file. */
void vc_hostreq_linkdata(char *link_data, int len) {
  UNUSED_PARAMETER(link_data);
  UNUSED_PARAMETER(len);
}

/* Receive DMB FIC data */
void vc_hostreq_dmb_fic(char *data, int len) {
  UNUSED_PARAMETER(data);
  UNUSED_PARAMETER(len);
}

/* Receive DMB PAD data */
void vc_hostreq_dmb_pad(char *data, int len) {
  UNUSED_PARAMETER(data);
  UNUSED_PARAMETER(len);
}

/* Receive DMB DATA data */
void vc_hostreq_dmb_data(char *data, int len) {
  UNUSED_PARAMETER(data);
  UNUSED_PARAMETER(len);
}

/* Request keypress info (from RTSP server). */
void vc_hostreq_keyin(void) {
}
