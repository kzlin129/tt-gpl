/* kxr94.h
 *
 * Control driver for KXR94 accelerometer
 *
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Authors: Vincent Dejouy <vincent.dejouy@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef INCLUDE_LINUX_KXR94_H
#define INCLUDE_LINUX_KXR94_H

typedef struct
{
	void (*init)(void);
}  kxr94_pdata_t;

#define KXR94_DEVNAME			"tomtom-kxr94-accelerometer"
#endif
