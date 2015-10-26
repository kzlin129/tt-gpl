/*
 *  Local dock detect driver include file.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 * Author:      Laurent Gregoire <laurent.gregoire@tomtom.com>
 */

#ifndef __DOCK_DETECT_H
#define __DOCK_DETECT_H      __FILE__

typedef int (*dock_state_handler)(void);
extern void dock_set_dock_state_handler(dock_state_handler hnd);

#endif
