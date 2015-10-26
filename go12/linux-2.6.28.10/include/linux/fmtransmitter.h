/*
 * Generic FM tranmitter include file
 *
 *	This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 * Author:		Rogier Stam, <Rogier.Stam@tomtom.com>
 * Adjusted from si4710.h, by Moore Mao, <Moore.Mao@tomtom.com> 
 */

#ifndef __INCLUDE_LINUX_FMTRANSMITTER_H
#define ____INCLUDE_LINUX_FMTRANSMITTER_H	__FILE__

#include <linux/ioctl.h>
#include <linux/major.h>

#define FMTRANSMITTER_MAJOR    		(176)
#define FMTRANSMITTER_MINOR    		(0)

#define FMTRANSMITTER_DRIVER_MAGIC     'S'

#define IOW_SET_FM_FREQUENCY		_IOW(FMTRANSMITTER_DRIVER_MAGIC,1,unsigned int)
#define IOW_ENABLE			_IOW(FMTRANSMITTER_DRIVER_MAGIC,2,unsigned int)

/*
 * Set the FM transmitter in mono or stereo mode.
 * @preconditions: none
 * @param: either FMTRX_MONO or FMTRX_STEREO
 * @return: 0 on success, error code otherwise.
 */
#define IOW_FMTRX_SET_MONO_STEREO		_IOW(FMTRANSMITTER_DRIVER_MAGIC,3,unsigned int)

/*
 * Turn on FM transmitter silent mode.
 * @precondition: none
 * @param: either 0 to disable, or non-zero to enable silent mode.
 * @return: 0 on success, error code otherwise.
 */
#define IOW_FMTRX_SILENT_MODE_ENABLE		_IOW(FMTRANSMITTER_DRIVER_MAGIC,4,unsigned int)

/*
 * Turn on Transmit Audio Dynamic Range Control.
 * @precondition: none
 * @param: either 0 to disable, or non-zero to enable TADRC.
 * @return: 0 on success, error code otherwise.
 */
#define IOW_FMTRX_TADRC_ENABLE			_IOW(FMTRANSMITTER_DRIVER_MAGIC,5,unsigned int)

/*
 * Get the current state of the FM transmitter. 
 * @precondition: none
 * @param: Pointer to enum to store data in.
 * @return: see fmtrx_state enum.
 */
#define IOR_FM_GET_STATE			_IOR(FMTRANSMITTER_DRIVER_MAGIC,6,enum fmtrx_state)

/*
 * Set the Audio Dynamic Range Control enabled or disabled.
 * @preconditions: none
 * @param: either 0 to disable, or non-zero to enable silent mode.
 * @return: 0 on success, error code otherwise.
 */
#define IOW_FMTRX_SET_ADRC			_IOW(FMTRANSMITTER_DRIVER_MAGIC,7,unsigned int)


/*
 * Set the FM transmitter's power mode.
 * @param: power in dBuV
 * @return: 0 on success, error code otherwise.
 */ 
#define IOW_FMTRX_SET_POWER			_IOW(FMTRANSMITTER_DRIVER_MAGIC,8,unsigned char)

#endif /* __INCLUDE_LINUX_FMTRANSMITTER_H */


