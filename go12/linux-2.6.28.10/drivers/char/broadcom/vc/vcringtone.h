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



#ifndef VCRINGTONE_H
#define VCRINGTONE_H

/******************************************************************************
   Initialises the ringtone tune API.
******************************************************************************/
extern void vc_ringtone_init(void);

/******************************************************************************
   Function to play SMAF and MIDI file from a filesystem file.
   Loopct is number of times to play, or 0 to loop continuously.
   If the function successfully starts playing the file, cbfunc, if not NULL,
   is called with id = 127 when the playing of a one-shot tone ends.
   It is safe, but unnecessary, to call vc_melody_end once the end of a
   one-shot tone has been indicated.
   Uses VideoCore rg_play application.
   Returns: 0 on success, 1 if note has already finished, < 0 on failure
******************************************************************************/
extern int vc_ringtone_play(char *fname, int loopct, int(* cbfunc)(int id));

/******************************************************************************
   Stops the playing of any currently playing SMAF or MIDI file.
   Returns: 0 if there was something really playing,
            1 if it stopped a tentative play,
            < 0 otherwise
******************************************************************************/
extern int vc_ringtone_end(void);


#endif
