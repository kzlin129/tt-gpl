/* alc5628-i2c.h
 *
 * Control driver for Realtek ALC5628 audio codec
 *
 * Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 * Authors: Ard Biesheuvel <ard.biesheuvel@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define ALC5628_I2C_SLAVE_ADDR	(0x18)

int alc5628_i2c_init(void);

