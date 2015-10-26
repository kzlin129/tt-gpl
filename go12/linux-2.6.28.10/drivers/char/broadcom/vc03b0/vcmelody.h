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



#ifndef VCMELODY_H
#define VCMELODY_H


/******************************************************************************
   Enumerate control codes
******************************************************************************/
#define VC_MFILE_CTRL_GET_LENGTH          2
#define VC_MFILE_CTRL_SET_HVVOLUME        5
#define VC_MFILE_CTRL_GET_HVVOLUME        6
#define VC_MFILE_CTRL_SET_STMVOLUME       7
#define VC_MFILE_CTRL_GET_STMVOLUME       8
#define VC_MFILE_CTRL_SET_CHVOLUME        9
#define VC_MFILE_CTRL_GET_CHVOLUME       10
#define VC_MFILE_CTRL_SET_STOPTIME       11
#define VC_MFILE_CTRL_GET_STOPTIME       12
#define VC_MFILE_CTRL_SET_SPEED          13
#define VC_MFILE_CTRL_GET_SPEED          14
#define VC_MFILE_CTRL_GET_POSITION       15
#define VC_MFILE_CTRL_SET_VOLUME         16
#define VC_MFILE_CTRL_GET_VOLUME         17
#define VC_MFILE_CTRL_SET_PANPOT         18
#define VC_MFILE_CTRL_GET_PANPOT         19
#define VC_MFILE_CTRL_SET_KEYCONTROL     20
#define VC_MFILE_CTRL_GET_KEYCONTROL     21
#define VC_MFILE_CTRL_SET_EVENTNOTE      22
#define VC_MFILE_CTRL_GET_CONTROL_VAL    23
#define VC_MFILE_CTRL_SET_LOOPCOUNT      24
#define VC_MFILE_CTRL_SET_ENDPOINT       25
#define VC_MFILE_CTRL_SEND_SHORT_MSG     26
#define VC_MFILE_CTRL_SEND_LONG_MSG      27
#define VC_MFILE_CTRL_SET_TIMER          28

/******************************************************************************
   Codes passed to callback function (in ls byte)
******************************************************************************/
#define VC_MFILE_CB_NATURALEND            1
#define VC_MFILE_CB_ABORTEND              2
#define VC_MFILE_CB_ERROREND              3
#define VC_MFILE_CB_REPEAT                4

/******************************************************************************
   Initialises the melody tune API.
******************************************************************************/

/******************************************************************************
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
   Returns: 0 on success, non-zero on failure
******************************************************************************/
extern int vc_melody_play(int mmf_addr, int loopct, int(* cbfunc)(int id));

/******************************************************************************
   Stops the playing of any currently playing SMAF or MIDI file
   Returns: 0
******************************************************************************/
extern int vc_melody_end(void);

/******************************************************************************
   Determines if a melody (or keytone or ringtone) is still playing
   Returns 1 if playing, 0 if stopped
******************************************************************************/
extern int vc_melody_playing(void);

/******************************************************************************
   Returns the current volume for the playing melody
******************************************************************************/
extern int vc_melody_getvol(void);

/******************************************************************************
   Sets the volume for the playing melody
******************************************************************************/
extern void vc_melody_setvol(int vol);

/******************************************************************************
   Enables/Disables Blinking of LED in synch with tune (for next tune)
******************************************************************************/
extern void vc_melody_enablink(int on);

/******************************************************************************
   Enables/Disables Vibration motor control in synch with tune (for next tune)
******************************************************************************/
extern void vc_melody_enamotor(int on);

/******************************************************************************
   Function to play SMAF phrase files from the host memory.
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
   Returns: 0 on success, non-zero on failure
******************************************************************************/
extern int vc_phrase_play(int ch, int mmf_addr, int loopct, int(* cbfunc)(int id));

/******************************************************************************
   Stops the playing of any currently playing SMAF Phrase file, on given chan
   Returns: 0
******************************************************************************/
extern int vc_phrase_end(int ch);

/******************************************************************************
   Determines if any SMAF Phrase files are still playing
   Returns the logical AND of chbits with the playing flag bits for each
   phrase channel.  Phrase channels 0-3 correpsond to bits 0-3.
******************************************************************************/
extern int vc_phrase_playing(int chbits);

/******************************************************************************
   Returns the current volume for the given phrase channel
******************************************************************************/
extern int vc_phrase_getvol(int ch);

/******************************************************************************
   Sets the volume for the given phrase channel
******************************************************************************/
extern void vc_phrase_setvol(int ch, int vol);


/******************************************************************************
   Set of lower level functions to control playing of SMAF and MIDI files
   from host memory.  These are intended for internal use by Music / Sound
   middleware emulation layer.
   Return: 0 on success, non-zero on failure
******************************************************************************/
extern int vc_mfile_load(int ch, int mmf_addr, int(* cbfunc)(int id));

extern int vc_mfile_seek(int ch, int seekms);

extern int vc_mfile_start(int ch);

extern int vc_mfile_stop(int ch);

extern int vc_mfile_unload(int ch);

extern int vc_mfile_ctrl(int ch, int ctrl_num, void *prm);

extern int vc_mfile_mode(int mode);


/*===== Deprecated functions ======*/

/******************************************************************************
   These functions are identical to vc_melody_xxx
   THESE FUNCTIONS ARE NOT REQUIRED NOW
******************************************************************************/
extern int vc_keytone_play(int mmf_addr, int loopct, int(* cbfunc)(int id));
extern int vc_keytone_end(void);


#endif
