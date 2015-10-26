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




/*=============================================================================

Project  :  Inter-processor communication
Module   :  FIFO
File     :  $RCSfile:  $
Revision :  $Revision: $

FILE DESCRIPTION
=============================================================================*/

#ifndef SOFTWARE_FIFO_H_
#define SOFTWARE_FIFO_H_

#include <linux/broadcom/vc03/vcos.h>

typedef struct {
   uint8_t        *malloc_address;
   uint8_t        *base_address;
   uint8_t        *read_ptr;
   uint8_t        *write_ptr;
   uint32_t       size;
   uint32_t       bytes_read;
   uint32_t       bytes_written;
   // semaphore to protect this structure
   OS_SEMAPHORE_T software_fifo_semaphore;
} SOFTWARE_FIFO_HANDLE_T;


int32_t software_fifo_create( const uint32_t size, int alignment, SOFTWARE_FIFO_HANDLE_T *handle );
int32_t software_fifo_destroy( SOFTWARE_FIFO_HANDLE_T *handle );

int32_t software_fifo_read( SOFTWARE_FIFO_HANDLE_T *handle, void *data, const uint32_t data_size );
int32_t software_fifo_write( SOFTWARE_FIFO_HANDLE_T *handle, const void *data, const uint32_t data_size );

int32_t software_fifo_data_available( SOFTWARE_FIFO_HANDLE_T *handle );
int32_t software_fifo_room_available( SOFTWARE_FIFO_HANDLE_T *handle );

int32_t software_fifo_peek( SOFTWARE_FIFO_HANDLE_T *handle, void *data, const uint32_t data_size );
int32_t software_fifo_remove( SOFTWARE_FIFO_HANDLE_T *handle, const uint32_t data_size );

int32_t software_fifo_protect( SOFTWARE_FIFO_HANDLE_T *handle );
int32_t software_fifo_unprotect( SOFTWARE_FIFO_HANDLE_T *handle );

#endif // SOFTWARE_FIFO_H_
