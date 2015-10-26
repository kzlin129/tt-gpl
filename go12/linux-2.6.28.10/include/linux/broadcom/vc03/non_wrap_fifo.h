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
Module   :  non wrap FIFO
File     :  $RCSfile:  $
Revision :  $Revision: $

FILE DESCRIPTION
=============================================================================*/

#ifndef _VCHI_NON_WRAP_FIFO_H_
#define _VCHI_NON_WRAP_FIFO_H_

#include "vchios.h"

typedef struct opaque_vchi_mqueue_t VCHI_NWFIFO_T;


VCHI_NWFIFO_T *non_wrap_fifo_create( const char *name, const uint32_t size, int alignment, int no_of_slots );
int32_t non_wrap_fifo_destroy( VCHI_NWFIFO_T *fifo );
int32_t non_wrap_fifo_read( VCHI_NWFIFO_T *fifo, void *data, const uint32_t max_data_size );
int32_t non_wrap_fifo_write( VCHI_NWFIFO_T *fifo, const void *data, const uint32_t data_size );
int32_t non_wrap_fifo_request_write_address( VCHI_NWFIFO_T *fifo, void **address, uint32_t size );
int32_t non_wrap_fifo_write_complete( VCHI_NWFIFO_T *fifo, void *address );
int32_t non_wrap_fifo_request_read_address( VCHI_NWFIFO_T *fifo, void **address, uint32_t *length );
int32_t non_wrap_fifo_read_complete( VCHI_NWFIFO_T *fifo, void *address );
int32_t non_wrap_fifo_remove( VCHI_NWFIFO_T *fifo, void *address );
void    non_wrap_fifo_debug( VCHI_NWFIFO_T *fifo );


#endif // _VCHI_NON_WRAP_FIFO_H_

/********************************** End of file ******************************************/
