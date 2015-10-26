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


#if defined( __KERNEL__ ) || defined( BUILDING_BOOTLOADER )
#   include <linux/types.h>
#else
#   include <inttypes.h>
#endif


#ifndef VCINTERFACE_H
#define VCINTERFACE_H

typedef struct
{
   uint16_t iface[8];
   uint8_t  vin_int_req;
   uint8_t  vout_int_ack;
   uint8_t  dummy1[14];
   uint8_t  vin_int_ack;
   uint8_t  vout_int_req;
   uint8_t  dummy2[14];
} VC_SHAREDMEM_HEADER_T;


/*---------------------------------------------------------------------------*/
/* Interface types */

typedef enum
{
   VC_ITYPE_MSGFIFO = 1,
   VC_ITYPE_REGBLOCK = 2
} VC_ITYPE_T;

typedef struct
{
   uint16_t itype;
   uint16_t sver;
   uint16_t stype;
   uint8_t  dummy1[10];
} VC_GENERIC_INTERFACE_T;

typedef struct
{
   uint16_t itype;
   uint16_t sver;
   uint16_t stype;
   uint16_t vin_fmin;
   uint16_t vin_fmax;
   uint16_t vout_fmin;
   uint16_t vout_fmax;
   uint8_t dummy[2];
   uint16_t vin_frptr;
   uint8_t  dummy1[14];
   uint16_t vin_fwptr;
   uint8_t  dummy2[14];
   uint16_t vout_frptr;
   uint8_t  dummy3[14];
   uint16_t vout_fwptr;
   uint8_t  dummy4[14];
} VC_MSGFIFO_INTERFACE_T;

typedef struct
{
   uint16_t itype;
   uint16_t sver;
   uint16_t stype;
   uint16_t blk_size;
   uint8_t  dummy1[8];
} VC_REGBLOCK_INTERFACE_T;

/*---------------------------------------------------------------------------*/
/* Some generic response codes */

typedef enum
{
   VC_RESP_OK,
   VC_RESP_ERROR,
   VC_RESP_HDR_AT_END      // actual header is found after a null byte in the
                           // gencmd - not valid for anything else
} VC_RESP_CODE_T;

/*---------------------------------------------------------------------------*/
/* Services */

typedef enum
{
   VC_STYPE_GENCMD = 1,
   VC_STYPE_DISPMAN = 2,
   VC_STYPE_TOUCHSCREEN = 3,
   VC_STYPE_DATASERVICE = 4,
   VC_STYPE_FILESERVICE = 5,
   VC_STYPE_HOSTREQ = 6,
   VC_STYPE_NOTA = 7,
   VC_STYPE_AVSERV = 8,
   VC_STYPE_OGLESSVC = 9,      /* OpenGL/ES proxy service */
   VC_STYPE_VVB = 10,
   VC_STYPE_VSIF = 11,
   VC_STYPE_ILCS = 12,
   VC_STYPE_FRMFWD = 13
} VC_STYPE_T;

/*---------------------------------------------------------------------------*/
/* Constant interface parameters */

#define VC_NUM_INTERFACES 8
#define VC_INTERFACE_AREA_SIZE 65536
#define VC_INTERFACE_BLOCK_SIZE 16
#define VC_CMD_SYNC 0xF1A55A1F



/*---------------------------------------------------------------------------*/
/* 'magic' locations in VC01/02/05 */
// GLS
/* Address where VideoCore will store the "id" of the running application.*/
#define VC_APP_ADDRESS      0x180000b8 // MULTICORE_SYNC_MBOX_6
/* Address where VideoCore can reference the pointer to the shared memory area */
#define VC_SHAREDPTR_ADDR   0x180000bc // MULTICORE_SYNC_MBOX_7

/*---------------------------------------------------------------------------*/
/* The command codes. */

enum {
   // Gencmd commands.
   VC_GENCMD_VERSION,
   VC_GENCMD_EXECUTE,

   // frame forwarding service
   VC_FRMFWD_SEND_FRAME,
   VC_FRMFWD_ACK_FRAME,
   VC_FRMFWD_OUT_OF_BUFFERS,
};


// Touch service parameter block.
typedef struct {
   unsigned short xpos;
   unsigned short ypos;
   unsigned short touch;
   uint8_t dummy1[2];
   unsigned int raw;
   uint8_t dummy2[4];
} VC_TOUCH_PARAMS_T;

/*---------------------------------------------------------------------------*/
/* Enum that describes what kind of VideoCore application is running. */

typedef enum {
   VC_INTERFACE_APP_NONE = 0,    // No application running
   VC_INTERFACE_APP_VMCS,        // Standard VMCS running
   VC_INTERFACE_APP_VMCS_GAME,    // "Game" with minimal VMCS running.
   VC_INTERFACE_APP_VMCS_TCL,        // Standard VMCS used from TCL
   VC_INTERFACE_APP_VMCS_GAME_TCL    // "Game" to be debugged using TCL
} VC_INTERFACE_APP_T;

#endif
