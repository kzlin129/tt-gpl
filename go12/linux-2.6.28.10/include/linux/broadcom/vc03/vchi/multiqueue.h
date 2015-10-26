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

Project  :  VCHI
Module   :  Routines to manipulate linked lists

FILE DESCRIPTION

=============================================================================*/

#ifndef _VCHI_MULTIQUEUE_H_
#define _VCHI_MULTIQUEUE_H_

#include <linux/broadcom/vc03/vcos.h>

// The user's elements must contain a next pointer at the start of the structure,
// followed by a prev pointer, if VCHI_MQUEUE_DOUBLY_LINKED is defined.
#undef VCHI_MQUEUE_DOUBLY_LINKED

typedef struct opaque_vchi_mqueue_t VCHI_MQUEUE_T;

// Create a pool and some queues
VCHI_MQUEUE_T *vchi_mqueue_create( const char *name, int nqueues, int nelements, int element_size );

// Destroy the pool
int32_t vchi_mqueue_destroy( VCHI_MQUEUE_T *mq );

// Get the address of the first element in the 'from' queue
void *vchi_mqueue_peek( VCHI_MQUEUE_T *mq, int from, int block );

// Get the address of the last element in the 'from' queue
void *vchi_mqueue_peek_tail( VCHI_MQUEUE_T *mq, int from, int block );

// Get the address of both the first and last elements in the 'from' queue, atomically
void vchi_mqueue_limits( VCHI_MQUEUE_T *mq, int from, void **head, void **tail, int block );

// return pointer to head of 'from' (or NULL if empty)
void *vchi_mqueue_get( VCHI_MQUEUE_T *mq, int from, int block );

// push 'element' onto the end of 'to'
void vchi_mqueue_put( VCHI_MQUEUE_T *mq, int to, void *element );

// Move the first element from 'from' and add it to the end of 'to'.  Return pointer to the moved element.
void *vchi_mqueue_move( VCHI_MQUEUE_T *mq, int from, int to );

// Find the given element on the 'from' queue and remove it
int32_t vchi_mqueue_element_get( VCHI_MQUEUE_T *mq, int from, void *element );

// Move the given element from the 'from' queue and add it to the end of 'to'
int32_t vchi_mqueue_element_move( VCHI_MQUEUE_T *mq, int from, int to, void *element );

// dump debug info
void vchi_mqueue_debug( VCHI_MQUEUE_T *mq, void(*func)( int idx, void *elem ));


#endif /* _VCHI_MULTIQUEUE_H_ */

/********************************** End of file ******************************************/
