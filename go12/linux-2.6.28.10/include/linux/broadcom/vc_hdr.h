/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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



#if !defined( VC_HDR_H )
#define VC_HDR_H

#include <linux/broadcom/bcmtypes.h>

// This file contains the definition for the header which describes the
// video core firmware.

#define VC_FIRMWARE_TYPE_ELF        0   // Uncompressed ELF file
#define VC_FIRMWARE_TYPE_GZIP_ELF   1   // gzip compressed ELF file

// All fields are to be stored in big-endian (i.e. network) order.

typedef struct
{
    char        signature[ 4 ]; // 'VCFW'
    uint32_t      areaSize;       // Size of area which contains the firmware
                                // areaSize is expected to be a multiple of the flash block size.
    uint32_t      firmwareType;   // Format of the firmware
    uint32_t      firmwareSize;   // Size of the firmware (in bytes).
    uint32_t      firmwareOffset; // Offset from beginning of header to beginning of firmware.

    uint8_t       reserved[ 44 ]; // Pad out VC_FIRMWARE_HEADER structure to 64 bytes.

} VC_FIRMWARE_HEADER;

#endif  // VC_HDR_H

