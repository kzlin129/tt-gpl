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




#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "vchost.h"
#include "vcgencmd.h"
#include "vchostreq.h"
#include "vcringtone.h"
#include "vcmelody.h"
#include "vcerrors.h"

#define MDEBUG1
#ifdef MDEBUG1
int tprintf(const char *format, ...);
#endif

/******************************************************************************
Global data.
******************************************************************************/

/******************************************************************************
Local types and defines.
******************************************************************************/

#define VC_MAXMIDISIZE  900000

#define FORCEGETPOSN    (1<<8)


typedef struct
{
   volatile int playing_new;
   volatile int playing_old;
   volatile int loaddoneid;
   int playstate;
   int baseposn;
   int basetime;
   int loopct;
   int vol;
   int pan;
   int key;
   int tempo;
   int(* volatile cbfunc_old)(int);
   int(* volatile cbfunc)(int);
} VC_ONERGCHAN_T;

typedef struct
{
   void *ringer_lock;
   void *load_sema;
   int notifyok;
   volatile int multichan;
   volatile int blink_en;
   volatile int motor_en;
   VC_ONERGCHAN_T ch[5];
} VC_RGGLOBALS_T;

/******************************************************************************
Static data.
******************************************************************************/

static VC_RGGLOBALS_T vc_rgg = {NULL, NULL, 0,0,0,0, {
                                 {0,0,0, 0,0,0,0,0,0,0,0, NULL},
                                 {0,0,0, 0,0,0,0,0,0,0,0, NULL},
                                 {0,0,0, 0,0,0,0,0,0,0,0, NULL},
                                 {0,0,0, 0,0,0,0,0,0,0,0, NULL},
                                 {0,0,0, 0,0,0,0,0,0,0,0, NULL} } };

/******************************************************************************
Static functions.
******************************************************************************/

static int vc_ringer_play(int ch, char *fstr, int flen, int memfile, int loopct, int(* cbfunc)(int id));
static int vc_ringer_load(int ch, char *fstr, int flen, int memfile, int(* cbfunc)(int id));
static int vc_ringer_end(int ch);
static void vc_ringer_setctrl(int ch, char ctrlv, int val);
static int vc_ringer_getctrl(int ch, char ctrlv);
static int vc_ringer_getposition(int ch);
static int vc_ringer_getduration(int ch);
static void vc_ringer_enablink(int on);
static void vc_ringer_enamotor(int on);
static void vc_ringtone_notify_func(int id);
static int vc_ringer_mode(int mode);

static int force_ch(int ch);
static int get_melodysize(uint8_t *faddr);
static uint32_t rd_uint32(uint8_t *fptr);
static uint32_t rd_uint16(uint8_t *fptr);


/*---------------------------------------------------------------------------*/

/******************************************************************************
NAME
   vc_ringtone_init

SYNOPSIS
   int vc_ringtone_init(void)

FUNCTION
   Initialises the ringtone tune API.
   (Not essential)

RETURNS
   void
******************************************************************************/

void vc_ringtone_init(void)
{
   int i;
   vc_rgg.multichan = 0;
   for (i = 0; i < 5; i++)
   {
      vc_rgg.ch[i].cbfunc = NULL;
      vc_rgg.ch[i].cbfunc_old = NULL;
      vc_rgg.ch[i].playing_new = 0;
      vc_rgg.ch[i].playing_old = 0;
      vc_rgg.ch[i].loaddoneid = 0;
      vc_rgg.ch[i].loopct = 0;
      vc_rgg.ch[i].vol = 0;
      vc_rgg.ch[i].pan = 0;
      vc_rgg.ch[i].key = 0;
      vc_rgg.ch[i].tempo = 0;
   }
   vc_rgg.blink_en = 0;
   vc_rgg.motor_en = 0;
   vc_rgg.notifyok = (vc_hostreq_set_notify(VC_HRNOTIFY_RINGTONE_END,
                                            vc_ringtone_notify_func) == 0);
   if (vc_rgg.ringer_lock == NULL)
      vc_rgg.ringer_lock = vc_lock_create();
   if (vc_rgg.load_sema == NULL)
   {
      vc_rgg.load_sema = vc_lock_create();
      vc_lock_obtain(vc_rgg.load_sema);
   }
}

/*=============================================================================
   Functions to play MIDI and SMAF files from host memory
=============================================================================*/

/******************************************************************************
NAME
   vc_melody_play

SYNOPSIS
   int vc_melody_play(int mmf_addr, int loopct, int(* cbfunc)(int id))

FUNCTION
   Function to play SMAF and MIDI files from the host memory.
   Loopct is number of times to play, or 0 to loop continuously.
   If the function successfully starts playing the file, cbfunc, if not NULL,
   is called when the playing of a file ends.  The id returned by the cbfunc
   contains a termination code in bits 7:0, with one of the following values:
      127: means playing ended normally
      100: means playing aborted early (if user issues an rg_seek gencmd)
      101: means an abnormal abort, due to a file or other error
   NOTE: vc_melody_end does NOT need to be called once the end of a tone has
   been indicated via the callback.

   Uses the rg_load general command.
   The size of the file is determined from the file contents, but midi files
   are truncated to a max of 900000 bytes.

RETURNS
   0 on success, non-zero on failure
******************************************************************************/
int vc_melody_play(int mmf_addr, int loopct, int(* cbfunc)(int id))
{
   /* Check file type and size */
   int flen = get_melodysize((uint8_t *)mmf_addr);
   if (flen <= 0)
      return -1;
   return (vc_ringer_play(0, (char *)mmf_addr, flen, 1, loopct, cbfunc));
}

/******************************************************************************
NAME
   vc_melody_end

SYNOPSIS
   int vc_melody_end(void)

FUNCTION
   Stops the playing of any currently playing SMAF or MIDI file.

RETURNS
   0
******************************************************************************/
int vc_melody_end(void)
{
   return (vc_ringer_end(0));
}

/******************************************************************************
NAME
   vc_melody_playing

SYNOPSIS
   int vc_melody_playing(void)

FUNCTION
   Determines if a melody (or keytone or ringtone) is still playing

RETURNS
   1 if playing, 0 if stopped
******************************************************************************/
int vc_melody_playing(void)
{
   return (vc_rgg.ch[0].playing_new != 0);
}

/******************************************************************************
NAME
   vc_melody_getvol

SYNOPSIS
   int vc_melody_getvol(void)

FUNCTION
   Reads the current volume for the playing melody

RETURNS
   Current volume
******************************************************************************/
int vc_melody_getvol(void)
{
   return (vc_ringer_getctrl(0, 'v'));
}

/******************************************************************************
NAME
   vc_melody_setvol

SYNOPSIS
   void vc_melody_setvol(int vol)

FUNCTION
   Sets the volume for the playing melody

RETURNS
   -
******************************************************************************/
void vc_melody_setvol(int vol)
{
   vc_ringer_setctrl(0, 'v', vol);
}

/******************************************************************************
NAME
   vc_melody_enablink

SYNOPSIS
   void vc_melody_enablink(int on)

FUNCTION
   Enables/Disables Blinking of LED in synch with tune (for next tune)

RETURNS
   -
******************************************************************************/
void vc_melody_enablink(int on)
{
   vc_ringer_enablink(on);
}

/******************************************************************************
NAME
   vc_melody_enamotor

SYNOPSIS
   void vc_melody_enamotor(int on)

FUNCTION
   Enables/Disables Vibration motor control in synch with tune (for next tune)

RETURNS
   -
******************************************************************************/
void vc_melody_enamotor(int on)
{
   vc_ringer_enamotor(on);
}

/*=============================================================================
   Functions to play multi-channel phrase files
=============================================================================*/

/******************************************************************************
NAME
   vc_phrase_play

SYNOPSIS
   int vc_phrase_play(int ch, int mmf_addr, int loopct, int(* cbfunc)(int id))

FUNCTION
   Function to play multi-channel phrase files from the host memory.
   Ch is the phrase channel number (0-3).
   Loopct is number of times to play, or 0 to loop continuously.
   If the function successfully starts playing the file, cbfunc, if not NULL,
   is called when the playing of a file ends.  The id returned by the cbfunc
   contains the phrase channel in bits 10:8 and a termination code in bits 7:0.
   The termination codes returned are:
      127: means playing ended normally
      100: means playing aborted early (if user issues an rg_seek gencmd)
      101: means an abnormal abort, due to a file or other error
   NOTE: vc_phrase_end does NOT need to be called once the end of a tone has
   been indicated via the callback.

   Uses the rg_play general command.

RETURNS
   0 on success, non-zero on failure
******************************************************************************/
int vc_phrase_play(int ch, int mmf_addr, int loopct, int(* cbfunc)(int id))
{
   /* Check file type and size */
   int flen = get_melodysize((uint8_t *)mmf_addr);
   if (flen <= 0)
      return -1;
   return (vc_ringer_play((ch & 3)+1, (char *)mmf_addr, flen, 1, loopct, cbfunc));
}

/******************************************************************************
NAME
   vc_phrase_end

SYNOPSIS
   int vc_phrase_end(int ch)

FUNCTION
   Stops playing any phrase file currently playing on the given channel.
   Ch is channel in the range 0-3

RETURNS
   0
******************************************************************************/
int vc_phrase_end(int ch)
{
   return (vc_ringer_end(ch+1));
}

/******************************************************************************
NAME
   vc_phrase_playing

SYNOPSIS
   int vc_phrase_playing(int chbits)

FUNCTION
   Determines if any SMAF Phrase files are still playing
   Returns the logical AND of chbits with the playing flag bits for each
   phrase channel.  Phrase channels 0-3 correpsond to bits 0-3.

RETURNS
   AND of chbits with channel playing bits, 0 if all stopped
******************************************************************************/
int vc_phrase_playing(int chbits)
{
   int i;
   int playing = 0;

   for (i = 0; i < 4; i++)
      if (vc_rgg.ch[i+1].playing_new)
         playing |= (1 << i);

   return (playing & chbits);
}

/******************************************************************************
NAME
   vc_phrase_getvol

SYNOPSIS
   int vc_phrase_getvol(int ch)

FUNCTION
   Reads the current volume for the given phrase channel

RETURNS
   Current volume
******************************************************************************/
int vc_phrase_getvol(int ch)
{
   return (vc_ringer_getctrl(ch+1, 'v'));
}

/******************************************************************************
NAME
   vc_phrase_setvol

SYNOPSIS
   void vc_phrase_setvol(int ch, int vol)

FUNCTION
   Sets the volume for the given phrase channel

RETURNS
   -
******************************************************************************/
void vc_phrase_setvol(int ch, int vol)
{
   vc_ringer_setctrl(ch+1, 'v', vol);
}


/*=============================================================================
   Functions for playing files from VideoCore file system
   NOT IN USE NOW
=============================================================================*/

/******************************************************************************
NAME
   vc_ringtone_play

SYNOPSIS
   int vc_ringtone_play(char *fname, int loopct, int(* cbfunc)(int id))

FUNCTION
   Function to play SMAF and MIDI file from a filesystem file.
   Loopct is number of times to play, or 0 to loop continuously.
   If the function successfully starts playing the file, cbfunc, if not NULL,
   is called when the playing of a file ends.  The id returned by the cbfunc
   contains a termination code in bits 7:0, with one of the following values:
      127: means playing ended normally
      100: means playing aborted early (if user issues an rg_seek gencmd)
      101: means an abnormal abort, due to a file or other error
   NOTE: vc_ringtone_end does NOT need to be called once the end of a tone has
   been indicated via the callback.
   Uses VideoCore rg_load general command.

RETURNS
   0 on success, non-zero on failure
******************************************************************************/
int vc_ringtone_play(char *fname, int loopct, int(* cbfunc)(int id))
{
   return (vc_ringer_play(0, fname, 0, 0, loopct, cbfunc));
}

/******************************************************************************
NAME
   vc_ringtone_end

SYNOPSIS
   int vc_ringtone_end(void)

FUNCTION
   Stops the playing of any currently playing SMAF or MIDI file.

RETURNS
   0
******************************************************************************/
int vc_ringtone_end(void)
{
   return (vc_ringer_end(0));
}

/*=============================================================================
   Functions for KEYTONE playing
   NOT IN USE NOW
=============================================================================*/

/******************************************************************************
NAME
   vc_keytone_play

SYNOPSIS
   int vc_keytone_play(int mmf_addr, int loopct, int(* cbfunc)(int id))

FUNCTION
   Identical to vc_melody_play
   THIS FUNCTION IS NOT REQUIRED NOW

RETURNS
   0 on success, non-zero on failure
******************************************************************************/
int vc_keytone_play(int mmf_addr, int loopct, int(* cbfunc)(int id))
{
   return (vc_melody_play(mmf_addr, loopct, cbfunc));
}

/******************************************************************************
NAME
   vc_keytone_end

SYNOPSIS
   int vc_keytone_end(void)

FUNCTION
   Identical to vc_melody_end
   THIS FUNCTION IS NOT REQUIRED NOW

RETURNS
   0
******************************************************************************/
int vc_keytone_end(void)
{
   return (vc_ringer_end(0));
}

/*=============================================================================
   Functions to support Music and Sound API emulation
=============================================================================*/

/******************************************************************************
NAME
   vc_mfile_load

SYNOPSIS
   int vc_mfile_load(int ch, int mmf_addr, int(* cbfunc)(int id))

FUNCTION
   Function to just load a file from host memory, ready to be played.
   Uses VideoCore rg_load general command, via vc_ringer_load().

RETURNS
   0 on success, non-zero on failure
******************************************************************************/
int vc_mfile_load(int ch, int mmf_addr, int(* cbfunc)(int id))
{
   /* Check file type and size */
   int flen = get_melodysize((uint8_t *)mmf_addr);
   if (flen <= 0)
      return -1;
   return (vc_ringer_load(ch, (char *)mmf_addr, flen, 1, cbfunc));
}

/******************************************************************************
NAME
   vc_mfile_seek

SYNOPSIS
   int vc_mfile_seek(int ch, int seekms)

FUNCTION
   Seeks to the given position in the current smaf/midi file.

RETURNS
   0 on success, non-zero on failure
******************************************************************************/
int vc_mfile_seek(int ch, int seekms)
{
   char s[128];
   int retval = 0;

   /* Force get position to use rg_get_status */
   vc_rgg.ch[ch].playstate = 1 + FORCEGETPOSN;

   vc_gencmd_send("rg_seek %d %d", ch, seekms);

   /* Get and check response for command failure */
   vc_gencmd_read_response(s, 128);
   vc_gencmd_number_property(s, "error", &retval);

   return retval;
}

/******************************************************************************
NAME
   vc_mfile_start

SYNOPSIS
   int vc_mfile_start(int ch)

FUNCTION
   Starts playing the current music file from the current position

RETURNS
   0 on success, non-zero on failure
******************************************************************************/
int vc_mfile_start(int ch)
{
   char s[128];
   int retval = 0;

   /* Force get position to use rg_get_status */
   vc_rgg.ch[ch].playstate = 2 + FORCEGETPOSN;
   vc_rgg.ch[ch].playing_new = 1;

   vc_gencmd_send("rg_play %d", ch);

   /* Get and check response for command failure */
   vc_gencmd_read_response(s, 128);
   vc_gencmd_number_property(s, "error", &retval);

   return retval;
}

/******************************************************************************
NAME
   vc_mfile_stop

SYNOPSIS
   int vc_mfile_stop(int ch)

FUNCTION
   Stops playing the current music file.

RETURNS
   0 on success, non-zero on failure
******************************************************************************/
int vc_mfile_stop(int ch)
{
   char s[128];
   int retval = 0;

   /* Force get position to use rg_get_status */
   /*
      ### As it takes a bit of time to stop, next call to ctrl_get_position may
      ### not be the final resting position.
   */
   vc_rgg.ch[ch].playstate = 1 + FORCEGETPOSN;

   vc_gencmd_send("rg_stop %d", ch);

   /* Get and check response for command failure */
   vc_gencmd_read_response(s, 128);
   vc_gencmd_number_property(s, "error", &retval);

   return retval;
}

/******************************************************************************
NAME
   vc_mfile_unload

SYNOPSIS
   int vc_mfile_unload(int ch)

FUNCTION
   Unloads the current music file.

RETURNS
   0 on success, non-zero on failure
******************************************************************************/
int vc_mfile_unload(int ch)
{
   char s[128];
   int retval = 0;

   /* Clear the call-back function now */
   vc_lock_obtain(vc_rgg.ringer_lock);  /* Need lock to do this */
   vc_rgg.ch[ch].cbfunc = NULL;
   vc_lock_release(vc_rgg.ringer_lock);

   vc_rgg.ch[ch].playstate = 0;

   vc_gencmd_send("rg_load %d", ch);

   /* Get and check response for command failure */
   vc_gencmd_read_response(s, 128);
   vc_gencmd_number_property(s, "error", &retval);

   return retval;
}

/******************************************************************************
NAME
   vc_mfile_ctrl

SYNOPSIS
   int vc_mfile_ctrl(int ch, int ctrl_num, void *prm)

FUNCTION
   Miscellaneous music file control functions

RETURNS
   0 on success, <0 on failure
******************************************************************************/
int vc_mfile_ctrl(int ch, int ctrl_num, void *prm)
{
   int retval = 0;

   switch (ctrl_num)
   {
      case VC_MFILE_CTRL_GET_LENGTH:
         /* ### Really should determine duration locally */
         retval = vc_ringer_getduration(ch);
         break;
      case VC_MFILE_CTRL_SET_HVVOLUME:
         break;
      case VC_MFILE_CTRL_GET_HVVOLUME:
         break;
      case VC_MFILE_CTRL_SET_STMVOLUME:
         break;
      case VC_MFILE_CTRL_GET_STMVOLUME:
         break;
      case VC_MFILE_CTRL_SET_CHVOLUME:
         break;
      case VC_MFILE_CTRL_GET_CHVOLUME:
         break;
      case VC_MFILE_CTRL_SET_STOPTIME:
         /* Set time to stop at - to generate a stoptime callback when there */
         break;
      case VC_MFILE_CTRL_GET_STOPTIME:
         break;
      case VC_MFILE_CTRL_SET_SPEED:
         vc_ringer_setctrl(ch, 's', *(int *)prm);
         break;
      case VC_MFILE_CTRL_GET_SPEED:
         *(int *)prm = vc_ringer_getctrl(ch, 's');
         break;
      case VC_MFILE_CTRL_GET_POSITION:
         if (vc_rgg.ch[ch].playstate == 0)
            break;
         /* Bit 8 of playstate is flag to force rg_get_status */
         if ((vc_rgg.ch[ch].playstate & FORCEGETPOSN) == 0)
         {
            int tmpt = vc_millitime() - vc_rgg.ch[ch].basetime;
            if (tmpt > 500 || tmpt < 0)
               vc_rgg.ch[ch].playstate |= FORCEGETPOSN;
            else
            {
               /* If its < 1/2 s since last rg_get_status, extrapolate posn using time */
               int dposn = 0;
               if (vc_rgg.ch[ch].playstate > 1)
                  dposn = (tmpt * vc_rgg.ch[ch].tempo)/100;
               retval = vc_rgg.ch[ch].baseposn + dposn;
            }
         }
         if ((vc_rgg.ch[ch].playstate & FORCEGETPOSN) != 0)
         {
            vc_rgg.ch[ch].playstate &= ~FORCEGETPOSN;
            retval = vc_ringer_getposition(ch);
            vc_rgg.ch[ch].basetime = vc_millitime();
            vc_rgg.ch[ch].baseposn = retval;
         }
         break;
      case VC_MFILE_CTRL_SET_VOLUME:
         vc_ringer_setctrl(ch, 'v', *(int *)prm);
         break;
      case VC_MFILE_CTRL_GET_VOLUME:
         *(int *)prm = vc_ringer_getctrl(ch, 'v');
         break;
      case VC_MFILE_CTRL_SET_PANPOT:
         vc_ringer_setctrl(ch, 'p', *(int *)prm);
         break;
      case VC_MFILE_CTRL_GET_PANPOT:
         *(int *)prm = vc_ringer_getctrl(ch, 'p');
         break;
      case VC_MFILE_CTRL_SET_KEYCONTROL:
         vc_ringer_setctrl(ch, 'k', *(int *)prm);
         break;
      case VC_MFILE_CTRL_GET_KEYCONTROL:
         *(int *)prm = vc_ringer_getctrl(ch, 'k');
         break;
      case VC_MFILE_CTRL_SET_EVENTNOTE:
         break;
      case VC_MFILE_CTRL_GET_CONTROL_VAL:
         break;
      case VC_MFILE_CTRL_SET_LOOPCOUNT:
         vc_ringer_setctrl(ch, 'n', *(int *)prm);
         break;
      case VC_MFILE_CTRL_SET_ENDPOINT:
         /* Set time to stop at */
         break;
      case VC_MFILE_CTRL_SEND_SHORT_MSG:
         break;
      case VC_MFILE_CTRL_SEND_LONG_MSG:
         break;
      case VC_MFILE_CTRL_SET_TIMER:
         /* For timing manual midi messages */
         break;

      default:
         retval = -1;
   }

   return retval;
}

/******************************************************************************
NAME
   vc_mfile_mode

SYNOPSIS
   int vc_mfile_mode(int mode)

FUNCTION
   Sets the mode: 0 = melody, 1 = phrase.
   Anything currently loaded/playing is unloaded if the mode changes

RETURNS
   0 if no change, 1 if mode changed
******************************************************************************/
int vc_mfile_mode(int mode)
{
   return (vc_ringer_mode(mode));
}


/*---------------------------------------------------------------------------*/
/* Static functions */
/******************************************************************************
NAME
   vc_ringer_play

SYNOPSIS
   int vc_ringer_play(int ch, char *fstr, int flen, int memfile, int loopct, int(* cbfunc)(int id))

FUNCTION
   Common function to play SMAF and MIDI files from either host memory file or
   file-system file.
   Loopct is number of times to play, or 0 to loop continuously.
   If the function successfully starts playing the file, cbfunc, if not NULL,
   is called with id = 127 when the playing a tone reaches its natural end.
   NOTE: vc_ringer_end does NOT need to be called once the end of a tone has
   been indicated.
   Uses VideoCore rg_play general command.

RETURNS
   0 on success, non-zero on failure
******************************************************************************/
static int vc_ringer_play(int ch, char *fstr, int flen, int memfile, int loopct, int(* cbfunc)(int id))
{
   char s[128];
   int retval = 0;

   /* Create a lock if we don't yet have one... */
   if (vc_rgg.ringer_lock == NULL)
      vc_rgg.ringer_lock = vc_lock_create();

   /* Create a load event if we don't yet have one... */
   if (vc_rgg.load_sema == NULL)
   {
      vc_rgg.load_sema = vc_lock_create();
      vc_lock_obtain(vc_rgg.load_sema);
   }

   /* If we haven't successfully set the notify function, do it now */
   if (!vc_rgg.notifyok)
      vc_rgg.notifyok = (vc_hostreq_set_notify(VC_HRNOTIFY_RINGTONE_END,
                                               vc_ringtone_notify_func) == 0);

   ch = force_ch(ch);   /* Ensure ch in range 0-4 */

   /* Set default ctrl values */
   vc_rgg.ch[ch].loopct = loopct;
   vc_rgg.ch[ch].vol = 100;
   vc_rgg.ch[ch].pan = 64;
   vc_rgg.ch[ch].key = 0;
   vc_rgg.ch[ch].tempo = 100;

   /*
      Set the new call-back function, transfer any playing_new to playing_old
      and set playing_new now, assuming gencmd is going to succeed
   */
   vc_lock_obtain(vc_rgg.ringer_lock);  /* Need a lock */
   vc_rgg.ch[ch].cbfunc_old = cbfunc;
   if (vc_rgg.ch[ch].playing_new)
      vc_rgg.ch[ch].playing_old++;
   vc_rgg.ch[ch].playing_new = 1;
   vc_lock_release(vc_rgg.ringer_lock);

   /* Switch between split (multich) and full mode if necessary */
   vc_ringer_mode(ch > 0);

   /* Use in-built Ringtone application */
   if (memfile)
      vc_gencmd_send("rg_load %d a %d l %d g n %d b %d m %d",
                        ch, (int)(void *)fstr, flen, loopct, vc_rgg.blink_en, vc_rgg.motor_en);
   else
      vc_gencmd_send("rg_load %d f %s g n %d b %d m %d",
                        ch, fstr, loopct, vc_rgg.blink_en, vc_rgg.motor_en);

   /* Get and check response for command failure */
   vc_gencmd_read_response(s, 128);
   vc_gencmd_number_property(s, "error", &retval);

   /* Wait for load to have occurred */
   vc_lock_obtain(vc_rgg.load_sema);

   /* Clear playing_new again and clear cbfunc if command failed */
   if (retval)
   {
      vc_lock_obtain(vc_rgg.ringer_lock);  /* Need a lock */
      vc_rgg.ch[ch].cbfunc_old = NULL;
      vc_rgg.ch[ch].playing_new = 0;
      vc_lock_release(vc_rgg.ringer_lock);
   }

   return retval;
}

/******************************************************************************
NAME
   vc_ringer_load

SYNOPSIS
   int vc_ringer_load(int ch, char *fstr, int flen, int memfile, int(* cbfunc)(int id))

FUNCTION
   Function to just load a file, ready to be played.
   Uses VideoCore rg_load general command.

RETURNS
   0 on success, non-zero on failure
******************************************************************************/
static int vc_ringer_load(int ch, char *fstr, int flen, int memfile, int(* cbfunc)(int id))
{
   char s[128];
   int retval = 0;

   /* Create a lock if we don't yet have one... */
   if (vc_rgg.ringer_lock == NULL)
      vc_rgg.ringer_lock = vc_lock_create();

   /* Create a load event if we don't yet have one... */
   if (vc_rgg.load_sema == NULL)
   {
      vc_rgg.load_sema = vc_lock_create();
      vc_lock_obtain(vc_rgg.load_sema);
   }

   /* If we haven't successfully set the notify function, do it now */
   if (!vc_rgg.notifyok)
      vc_rgg.notifyok = (vc_hostreq_set_notify(VC_HRNOTIFY_RINGTONE_END,
                                               vc_ringtone_notify_func) == 0);

   ch = force_ch(ch);   /* Ensure ch in range 0-4 */

   /* Set default ctrl values (matching defaults from rg_load) */
   vc_rgg.ch[ch].loopct = 1;
   vc_rgg.ch[ch].vol = 100;
   vc_rgg.ch[ch].pan = 64;
   vc_rgg.ch[ch].key = 0;
   vc_rgg.ch[ch].tempo = 100;

   /* Force get position to use rg_get_status */
   vc_rgg.ch[ch].playstate = 1 + FORCEGETPOSN;

   /*
      Set the new call-back function, transfer any playing_new to playing_old
      and clear playing_new for now
   */
   vc_lock_obtain(vc_rgg.ringer_lock);  /* Need a lock */
   vc_rgg.ch[ch].cbfunc = cbfunc;
   vc_rgg.ch[ch].playing_old += vc_rgg.ch[ch].playing_new;
   vc_rgg.ch[ch].playing_new = 0;
   vc_rgg.ch[ch].loaddoneid = 0;
   vc_lock_release(vc_rgg.ringer_lock);

   /* Switch between split (multich) and full mode if necessary */
   vc_ringer_mode(ch > 0);

   /* Use in-built Ringtone application */
   /* ### Load with d(ur) option - this is really to slow for quick beeps */
   if (memfile)
      vc_gencmd_send("rg_load %d a %d l %d d",
                        ch, (int)(void *)fstr, flen);
   else
      vc_gencmd_send("rg_load %d f %s d",
                        ch, fstr);

   /* Get and check response for command failure */
   vc_gencmd_read_response(s, 128);
   vc_gencmd_number_property(s, "error", &retval);

   /* Wait for load to have occurred */
   vc_lock_obtain(vc_rgg.load_sema);

   /* Fail if load did not succeed */
   if (vc_rgg.ch[ch].loaddoneid != 1)
      retval = VC_ERR_BAD_SOURCE;

   /* Clear cbfunc if command failed */
   if (retval)
   {
      vc_lock_obtain(vc_rgg.ringer_lock);  /* Need a lock */
      vc_rgg.ch[ch].cbfunc = NULL;
      vc_lock_release(vc_rgg.ringer_lock);
   }

   return retval;
}

/******************************************************************************
NAME
   vc_ringer_end

SYNOPSIS
   int vc_ringer_end(ch)

FUNCTION
   Stops the playing of any currently playing SMAF or MIDI file.

RETURNS
   0
******************************************************************************/
static int vc_ringer_end(int ch)
{
   char s[128];
   int retval = 0;

   /* Create a lock if we don't yet have one... */
   if (vc_rgg.ringer_lock == NULL)
      vc_rgg.ringer_lock = vc_lock_create();

   ch = force_ch(ch);   /* Ensure ch in range 0-4 */

   /* Clear the call-back function now */
   vc_lock_obtain(vc_rgg.ringer_lock);  /* Need lock to do this */
   vc_rgg.ch[ch].cbfunc_old = NULL;
   vc_lock_release(vc_rgg.ringer_lock);

   vc_gencmd_send("rg_stop %d", ch);
   vc_gencmd_read_response(s, 128);


   return retval;
}

/******************************************************************************
NAME
   vc_ringer_getctrl

SYNOPSIS
   int vc_ringer_getctrl(int ch, char ctrlv)

FUNCTION
   Reads the current ctrl value for the given channel
   ctrlv is one of n(num), v(ol), p(an), k(ey), s(peed)

RETURNS
   Control value, or 0 if ctrlv is invalid
******************************************************************************/
static int vc_ringer_getctrl(int ch, char ctrlv)
{
   int retval = 0;
   ch = force_ch(ch);   /* Ensure ch in range 0-4 */
   switch (ctrlv)
   {
      case 'n': retval = vc_rgg.ch[ch].loopct;  break;
      case 'v': retval = vc_rgg.ch[ch].vol;     break;
      case 'p': retval = vc_rgg.ch[ch].pan;     break;
      case 'k': retval = vc_rgg.ch[ch].key;     break;
      case 's': retval = vc_rgg.ch[ch].tempo;   break;
      default: retval = 0;
   }
   return retval;
}

/******************************************************************************
NAME
   vc_ringer_setctrl

SYNOPSIS
   void vc_ringer_setctrl(int ch, char ctrlv, int val)

FUNCTION
   Sets the current ctrl value for the given channel
   ctrlv is one of n(num), v(ol), p(an), k(ey), s(peed)
   If ctrlv is an invalid control, the call is silently ignored

RETURNS
   -
******************************************************************************/
static void vc_ringer_setctrl(int ch, char ctrlv, int val)
{
   int valid = 1;
   char s[128];
   ch = force_ch(ch);   /* Ensure ch in range 0-4 */

   /* Keep a copy */
   switch (ctrlv)
   {
      case 'n': vc_rgg.ch[ch].loopct = val;  break;
      case 'v': vc_rgg.ch[ch].vol = val;     break;
      case 'p': vc_rgg.ch[ch].pan = val;     break;
      case 'k': vc_rgg.ch[ch].key = val;     break;
      case 's': vc_rgg.ch[ch].tempo = val;   break;
      default:
         valid = 0;
   }

   if (valid)
   {
      /* Set the ctrl using rg_ctrl */
      vc_gencmd_send("rg_ctrl %d %c %d", ch, ctrlv, val);
      vc_gencmd_read_response(s, 128);
   }
}

/******************************************************************************
NAME
   vc_ringer_getposition

SYNOPSIS
   int vc_ringer_getposition(int ch)

FUNCTION
   Reads the current position using rg_get_status

RETURNS
   Current position, or -1 on error
******************************************************************************/
static int vc_ringer_getposition(int ch)
{
   char s[128];
   int retval = 0;
   ch = force_ch(ch);   /* Ensure ch in range 0-4 */

   vc_gencmd_send("rg_get_status %d", ch);

   /* Get and check response for command failure */
   vc_gencmd_read_response(s, 128);
   vc_gencmd_number_property(s, "error", &retval);
   if (retval)
      return -1;
   vc_gencmd_number_property(s, "position", &retval);

   return retval;
}

/******************************************************************************
NAME
   vc_ringer_getduration

SYNOPSIS
   int vc_ringer_getduration(int ch)

FUNCTION
   Reads the file duration using rg_get_status

RETURNS
   Current duration, or -1 on error
******************************************************************************/
static int vc_ringer_getduration(int ch)
{
   char s[128];
   int retval = 0;
   ch = force_ch(ch);   /* Ensure ch in range 0-4 */

   vc_gencmd_send("rg_get_status %d", ch);

   /* Get and check response for command failure */
   vc_gencmd_read_response(s, 128);
   vc_gencmd_number_property(s, "error", &retval);
   if (retval)
      return -1;
   vc_gencmd_number_property(s, "duration", &retval);

   return retval;
}

/******************************************************************************
NAME
   vc_ringer_enablink

SYNOPSIS
   void vc_ringer_enablink(int on)

FUNCTION
   Enables/Disables Blinking of LED in synch with tune (for next tune)

RETURNS
   -
******************************************************************************/
static void vc_ringer_enablink(int on)
{
   vc_rgg.blink_en = on;
}

/******************************************************************************
NAME
   vc_ringer_enamotor

SYNOPSIS
   void vc_ringer_enamotor(int on)

FUNCTION
   Enables/Disables Vibration motor control in synch with tune (for next tune)

RETURNS
   -
******************************************************************************/
static void vc_ringer_enamotor(int on)
{
   vc_rgg.motor_en = on;
}

/******************************************************************************
NAME
   vc_ringtone_notify_func

SYNOPSIS
   void vc_ringtone_notify_func(int id)

FUNCTION
   Notify function to be called for VC_HRNOTIFY_RINGTONE_END notify events

RETURNS
   void
******************************************************************************/
static void vc_ringtone_notify_func(int id)
{
static int maprenotify[4] = {0, 127, 100, 101};  /* for compatibility */
   int ringendid;
   int loaddoneid;
   int miscid;
   int ch;
   /* Use a lock as notify function runs in a different thread */
   vc_lock_obtain(vc_rgg.ringer_lock);

   /* chan in bits 8-10 of id */
   ch = (id>>8) & 7;
   if (ch < 0 || ch > 4)
      return;

   /* Separate ringtone end and load notify events */
   ringendid = (id >> 2) & 3;
   loaddoneid = id & 3;
   /* Misc id is for REPEAT, USER, STOPTIME, JOBDONE etc */
   miscid = (id >> 4) & 15;

   /* Set event when a load has happened or failed */
   if (loaddoneid)
   {
      vc_rgg.ch[ch].loaddoneid = loaddoneid;
      vc_lock_release(vc_rgg.load_sema);
   }

   if (ringendid)
   {
      /* Map phrase channels from 1-4 to 0-3 */
      if (ch > 0)
         ringendid += ((ch-1)<<8);

      /*
         If playing_old is set just clear that, otherwise if playing_new is set,
         clear it and call any call-back function.
         It is an error if neither playing_old or playing_new is set.
      */
      if (vc_rgg.ch[ch].playing_old > 0)
         vc_rgg.ch[ch].playing_old -= 1;
      else if (vc_rgg.ch[ch].playing_new)
      {
         vc_rgg.ch[ch].playing_new = 0;

         if (vc_rgg.ch[ch].cbfunc_old)
         {
            /* Remap old format calback parameter */
            (*vc_rgg.ch[ch].cbfunc_old)(maprenotify[ringendid]);
            vc_rgg.ch[ch].cbfunc_old = NULL;
         }

         if (vc_rgg.ch[ch].cbfunc_old)
         {
            /* New format callback is ok as is */
            (*vc_rgg.ch[ch].cbfunc_old)(ringendid);
         }
      }
   }

   if (miscid && vc_rgg.ch[ch].cbfunc)
   {
      /* Map phrase channels from 1-4 to 0-3 */
      if (ch > 0)
         miscid += ((ch-1)<<8);

      (*vc_rgg.ch[ch].cbfunc)(miscid+3);
   }

   vc_lock_release(vc_rgg.ringer_lock);
}

/******************************************************************************
NAME
   vc_ringer_mode

SYNOPSIS
   int vc_ringer_mode(int mode)

FUNCTION
   Sets the mode: 0 = melody, 1 = phrase.
   Anything currently loaded/playing is unloaded if the mode changes

RETURNS
   0 if no change, 1 if mode changed
******************************************************************************/
static int vc_ringer_mode(int mode)
{
   char s[128];
   int retval;
   mode = (mode != 0);
   retval = mode != vc_rgg.multichan;
   if (retval)
   {
      vc_rgg.multichan = mode;
      vc_gencmd_send("rg_mode %s", mode ? "s" : "f");
      vc_gencmd_read_response(s, 128);
   }

   return retval;
}

/******************************************************************************
NAME
   force_ch

SYNOPSIS
   int force_ch(int ch);

FUNCTION
   Forces ch into range 0-4.
   Values <= 0 become 0.  Values > 0 wrap within range 1-4.

RETURNS
   Limited ch number
******************************************************************************/
static int force_ch(int ch)
{
   /* Force ch into reange 0-4 */
   if (ch > 0)
      ch = ((ch-1) & 3)+1;
   else
      ch = 0;
   return ch;
}

#define SMAF_ID      0x4D4D4D44
#define MIDI_ID      0x4D546864
#define MTRK_ID      0x4D54726B

/******************************************************************************
NAME
   get_melodysize

SYNOPSIS
   int get_melodysize(uint8_t *faddr)

FUNCTION
   Determines the size of the given SMAF or MIDI memory file

RETURNS
   Size of file
******************************************************************************/
static int get_melodysize(uint8_t *faddr)
{
   uint8_t *fptr = faddr;
   int len = 0;
   int olen;
   int maxtrk,ntrk;
   uint32_t fid;
   uint32_t id;

   /* Check file magic id */
   fid = rd_uint32(fptr);

   if (fid == SMAF_ID)
   {
      /* SMAF is easy... */
      len = 8 + rd_uint32(fptr+4);
   }
   else if (fid == MIDI_ID)
   {
      /* For MIDI we need to read the header, then all the tracks */
      /* Skip MThd - don't care about type really */
      len = 8 + rd_uint32(fptr+4);
      maxtrk = rd_uint16(fptr+10);
      fptr = faddr+len;
      olen = 0;
      ntrk = 0;
      /* Skip as many MTrk's as we find, subject to max size */
      id = rd_uint32(fptr);
      while (id == MTRK_ID && ntrk < maxtrk && len < VC_MAXMIDISIZE)
      {
         olen = len;
         len += 8 + rd_uint32(fptr+4);
         fptr = faddr+len;
         ntrk++;
         if (ntrk < maxtrk)
            id = rd_uint32(fptr);
      }
   }

   return len;
}

/******************************************************************************
NAME
   rd_uint32

SYNOPSIS
   uint32_t rd_uint32(uint8_t *fptr)

FUNCTION
   Converts 4 bytes to uint32 big-endian

RETURNS
   void
******************************************************************************/
static uint32_t rd_uint32(uint8_t *fptr)
{
   uint32_t num = (*fptr++);
   num = (num<<8) + (*fptr++);
   num = (num<<8) + (*fptr++);
   num = (num<<8) + (*fptr++);
   return num;
}

/******************************************************************************
NAME
   rd_uint16

SYNOPSIS
   int rd_uint16(uint8_t *fptr)

FUNCTION
   Converts 2 bytes to int big-endian

RETURNS
   void
******************************************************************************/
static uint32_t rd_uint16(uint8_t *fptr)
{
   int num = (*fptr++);
   num = (num<<8) + (*fptr++);
   return num;
}

