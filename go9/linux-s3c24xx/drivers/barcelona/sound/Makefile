# drivers/barcelona/sound/Makefile
#
# Makefile for the Barcelona sound driver.
#
# Copyright (C) 2004 TomTom BV.
# Author: Sebastian Sjoberg <sebastian.sjoberg@tomtom.com>
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#EXTRA_CFLAGS = -DSDEBUG_T

obj-$(CONFIG_BARCELONA_SOUND)			+= sound.o
obj-$(CONFIG_BARCELONA_SOUND_FILTER)	+= ats5500_stub.lib
obj-$(CONFIG_BARCELONA_SOUND_ALC5628)	+= alc5628.o alc5628-i2c.o

sound-objs := dma.o spi.o wm8711.o wm8750.o wm8971.o iis.o iis-s3c2412.o iis-s3c2443.o iis-s3c2450.o driver.o codec.o


#ats5500_ARM9.lib

# EOF
