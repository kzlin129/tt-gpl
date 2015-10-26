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



#include <linux/string.h>
//#include <stdio.h>
#include <linux/ctype.h>

#include "vchost.h"
#include "vcinterface.h"

#include "vciface.h"
#include "vcmsgfifo.h"
#include "vcutil.h"

#include "vc_hostreq_defs.h"
#include "vchostreq.h"
#include "vchostreq_int.h"
#include "vchostmem.h"

#if 0
# define DEBUG_TRACE(x) tprintf x
# include "../../../Tcl/include/tcl.h" // temp :debug only
#else
# define DEBUG_TRACE(x)
#endif

/******************************************************************************
Global data.
******************************************************************************/

/******************************************************************************
Local types and defines.
******************************************************************************/

typedef struct hostmem_block_s
{
  int len; /* 0; free or -1; taken but not head; otherwise  head & remaining no blocks  */
} HOSTMEM_BLOCK_T;


#define NUM_BLOCKS 128
#define BLOCK_SIZE 1024


const int BLOCK_FREE = 0;
const int BLOCK_USED = -1;

/******************************************************************************
Static data.
******************************************************************************/
#ifndef HOST_MALLOC_EXISTS
static unsigned char   memory_blocks[NUM_BLOCKS][BLOCK_SIZE];
static HOSTMEM_BLOCK_T block_state[NUM_BLOCKS];
#endif

/******************************************************************************
Static functions
******************************************************************************/



/*---------------------------------------------------------------------------*/


/******************************************************************************
NAME
   vchostreq_meminit

SYNOPSIS
   void vchostreq_meminit( void )

FUNCTION
   Initialisations

RETURNS
   -
******************************************************************************/
void vchostreq_meminit( void )
{
#ifndef HOST_MALLOC_EXISTS
  int i;

  DEBUG_TRACE(("<vchostreq_meminit() %d blocks of %d bytes each from &0x%08x\n",
          NUM_BLOCKS, BLOCK_SIZE, memory_blocks));
  for( i=0; i < NUM_BLOCKS; i++ )
  {
    block_state[i].len = BLOCK_FREE;
  }
#endif
}

/******************************************************************************
NAME
   vchostreq_writemem

SYNOPSIS
   VC_RESP_CODE_T vchostreq_writemem( void* host_addr, void *vc_addr, int len, int channel )

FUNCTION
   Writes a block of data from Videocore into the host
   memory area specified by host_addr

RETURNS
   Success  : VC_RESP_OK     ; all the data was copied
   Fail     : VC_RESP_ERROR  ; error in input parameters or operation
******************************************************************************/
VC_RESP_CODE_T vchostreq_writemem( void* host_addr, void *vc_addr, int len, int channel )
{
  char *end_addr;
  int   overlap;
  VC_RESP_CODE_T  retval = VC_RESP_OK;
  int   ael;
  char tempbuf[16];
  int block_len;

  if( (host_addr == NULL)
      || (vc_addr == NULL)
      || ( len == 0 ) )
  {
    retval = VC_RESP_ERROR; /* VC never gets response ??? */
  }
  else
  {
     if (channel == -1)
        channel = 0;     // default channel

    /* Can only transfer in multiples of 16 bytes. Potentially
    ** copy more bytes than requested & preserve the original
    ** contents of the overlapping region */
    ael = (len + 15) & ~15;

    end_addr = (char*)host_addr + len;
    overlap  = ael - len;

    DEBUG_TRACE((" ael %d, end_addr %x, overlap %d\n", ael, end_addr, overlap ));
    memcpy( tempbuf, (void*)end_addr, overlap );

#define HRMAX_WRITEMEM 32768
    /* Split large reads over the host-bus
    ** to avoid hogging it */
    while( ael > 0 )
    {
      DEBUG_TRACE((" ael %d\n", ael));
      block_len = ael;

      if( block_len > HRMAX_WRITEMEM ) block_len = HRMAX_WRITEMEM;

      if( vc_host_read_consecutive( host_addr, (uint32_t)vc_addr, block_len, channel ) != 0 )
      {
        retval = VC_RESP_ERROR;
      }

      ael -= block_len;
      host_addr = (void*)((uint32_t)host_addr + block_len);
      vc_addr = (void*)((uint32_t)vc_addr + block_len);
    }

    memcpy( (void*)end_addr, tempbuf, overlap );

    DEBUG_TRACE((" \"%s\"\n", host_addr ));
  }

  return( retval );
}

/******************************************************************************
NAME
   vchostreq_readmem

SYNOPSIS
   VC_RESP_CODE_T vchostreq_readmem( void* host_addr, void *vc_addr, int len )

FUNCTION
   Allows Videocore to read a block of memory from a
   memory area on the host specified by host_addr

RETURNS
   Success  : VC_RESP_OK     ; all the data was copied
   Fail     : VC_RESP_ERROR  ; error in input parameters or operation
******************************************************************************/
VC_RESP_CODE_T vchostreq_readmem( void* host_addr, void *vc_addr, int len )
{
  VC_RESP_CODE_T retval = VC_RESP_OK;
  int tmplen = (len + 15) & ~15;

  if( (host_addr == NULL)
      || (vc_addr == NULL)
      || ( len == 0 ) )
  {
    retval = VC_RESP_ERROR; /* VC never gets response ??? */
  }
  else
  {
    DEBUG_TRACE(("\n vc_host_write vc:%x host:%x %d\n", vc_addr, host_addr, tmplen ));

    {
      int i;
      for( i = 0; i < 6; i++ ) DEBUG_TRACE((" %02x ", *((char*)host_addr + i) ));
    }

    /* Write directly to user buffer on videocore */
    vc_host_write_consecutive((uint32_t)vc_addr, host_addr, tmplen, 0);
  }

  return( retval );
}

/******************************************************************************
NAME
   vchostreq_malloc

SYNOPSIS
   void*  vchostreq_malloc( size_t bytes)

FUNCTION
   Allocates a block of memory of size "bytes" on the host

RETURNS
   The "address" of the memory block on host. Returns NULL if
   unsucccessful.
******************************************************************************/
void*  vchostreq_malloc( size_t bytes)
{
#ifdef HOST_MALLOC_EXISTS
  return (void*)malloc( bytes );
#else
  int i,j;
  int num_blocks = (BLOCK_SIZE + bytes -1) / BLOCK_SIZE;
  int start_block;

  DEBUG_TRACE(("<> vchostreq_malloc() for %d bytes\n", bytes));

  for( i=0; i < NUM_BLOCKS; i++ )
  {
    DEBUG_TRACE(("<> Try block %d : len %d\n", i, block_state[i].len ));
    if(block_state[i].len == BLOCK_FREE)
    {
      if (i+num_blocks>NUM_BLOCKS)
      {
        DEBUG_TRACE(("<> Error - ran out of blocks\n"));
        return (NULL) ;
      }
      for( j = 0; j < num_blocks; j++ )
      {
        DEBUG_TRACE(("<> Get contiguous block %d of %d\n", i+j, num_blocks ));
        if(block_state[i+j].len != BLOCK_FREE) break;
      }

      if( j == num_blocks )
      {
        DEBUG_TRACE(("<> Used %d blocks for request of %d bytes -> %d : &0x%08x\n",
                num_blocks, bytes, i, &memory_blocks[i] ));
        start_block = i;
        block_state[i].len = num_blocks;
        for(j=0;j<num_blocks;j++)
          block_state[i+j].len = BLOCK_USED;

        return(memory_blocks[i]);
      }
      i+=j;
    }
  }

  return( NULL );
#endif
}

/******************************************************************************
NAME
   vchostreq_free

SYNOPSIS
   void  vchostreq_free( void *host_addr )

FUNCTION
   Frees a block of host memory previously allocated by vchostreq_malloc

RETURNS
   Success  : VC_RESP_OK     ; memory block freed
   Fail     : VC_RESP_ERROR  ; memory block was not valid
******************************************************************************/
void vchostreq_free( void *host_addr )
{
#ifdef HOST_MALLOC_EXISTS
  if( host_addr ) free( host_addr );
#else
  int addr=(int)host_addr;
  int startaddr=(int)&memory_blocks[0][0];
  int blocknum=(addr-startaddr)/BLOCK_SIZE;
  int num_blocks;
  int i;

  DEBUG_TRACE(("<> vchostreq_free() for address &0x%08x\n", host_addr ));

  if ((blocknum<0) || (blocknum>=NUM_BLOCKS) || (addr!=startaddr+BLOCK_SIZE*blocknum))
  {
    DEBUG_TRACE(("<> ERROR - invalid block\n"));
    return;
  }
  num_blocks=block_state[blocknum].len;
  if (num_blocks<=0)
  {
    DEBUG_TRACE(("<> ERROR - pointer is not start of block\n"));
    return;
  }

  DEBUG_TRACE(("<> Free block %d\n", blocknum));
  block_state[blocknum].len=BLOCK_FREE;
  for(i=blocknum+1;i<blocknum+num_blocks;i++)
  {
    DEBUG_TRACE(("<> Free block %d\n", i));
    if (block_state[i].len!=BLOCK_USED)
    {
      return;
    }
    block_state[i].len=BLOCK_FREE;
  }
#endif
}


/******************************************************************************
NAME
   vchostreq_memove

SYNOPSIS
   VC_RESP_CODE_T vc_hostreq_memmove( void  *dest_addr, void *src_addr, int len )

FUNCTION
   Moves memory block on host to another location on the host

RETURNS
   Success  : VC_RESP_OK     ; all the data was copied
   Fail     : VC_RESP_ERROR  ; error in input parameters or operation
******************************************************************************/
VC_RESP_CODE_T vchostreq_memmove( void  *dest_addr, void *src_addr, int len )
{
  memmove( dest_addr, src_addr, len );
  return( VC_RESP_OK);
}

/******************************************************************************
NAME
   vchostreq_readmem_3d

SYNOPSIS
   VC_RESP_CODE_T vchostreq_readmem_3d

FUNCTION
   Moves memory block on host to another location on the host

RETURNS
   Success  : VC_RESP_OK     ; all the data was copied
   Fail     : VC_RESP_ERROR  ; error in input parameters or operation
******************************************************************************/

VC_RESP_CODE_T vchostreq_readmem_3d(uint32_t dest, unsigned int tile_width, unsigned int tile_height, unsigned int num_tiles,
                                    unsigned int tile_d_pitch, unsigned int d_pitch, void *src, int tile_s_pitch, int s_pitch,
                                    uint32_t frac_offset, uint32_t frac_int, int channel) {
   // Put a generic memcpy style implementation in here.
   // dest is the address on VideoCore, src is the address here on the host.
   unsigned int ntiles, height;
   char *s = (char *)src;
   for (ntiles = 0; ntiles < num_tiles; ntiles++) {
      uint32_t dest1 = dest;
      char *s1 = s;
      if (tile_width == tile_d_pitch && tile_width == tile_s_pitch && frac_int == 1<<16)

         // This function is only used by the VideoCore display manager to
         // pull host frame-buffer data into it's stripe buffer (to be sent
         // to the LCD). Data sent across the host-VC interface must be in
         // little-endian order. Therefore, if the host is big-endian, we
         // need to byte-swap the frame-buffer data as we write it to the
         // interface...Note that this logic will fall apart if this function
         // is used for purposes other than reading frame-buffer data, or if
         // the frame-buffer doesn't contain 16 bit-per-pixel data.

#ifdef VC_HOST_IS_BIG_ENDIAN
         vc_host_write_byteswapped(dest1, s1, tile_width*tile_height, channel);
#else
         vc_host_write_consecutive(dest1, s1, tile_width*tile_height, channel);
#endif
      else {
         for (height = 0; height < tile_height; height++) {
#ifdef VC_HOST_IS_BIG_ENDIAN
            vc_host_write_byteswapped(dest1, s1+(frac_offset>>16)*tile_s_pitch, tile_width, channel);
#else
            vc_host_write_consecutive(dest1, s1+(frac_offset>>16)*tile_s_pitch, tile_width, channel);
#endif
            dest1 += tile_d_pitch;
            frac_offset += frac_int;
         }
      }

      dest += d_pitch;
      s += s_pitch;
   }
   return VC_RESP_OK;
}
