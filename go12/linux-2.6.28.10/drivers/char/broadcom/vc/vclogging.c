/*****************************************************************************
* Copyright 2002 - 2008 Broadcom Corporation.  All rights reserved.
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



/**
*
*  @file    vclogging.c
*
*  @brief   VC02 log retrieve functions
*
****************************************************************************/


#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/errno.h>




#include "vchost.h"
#include "vcutil.h"

#include "vclogging.h"

/******************************************************************************
Global data.
******************************************************************************/
extern int gErrorPerPeriod;
extern int gErrorPeriodLenSec;
extern int gErrorPrtEnable;
/******************************************************************************
Local types and defines.
******************************************************************************/
// Align a pointer by rounding up:
#define ALIGN_UP(ptr, alignment) (((unsigned long)(ptr)+(alignment-1))&~((alignment)-1))
// Align a pointer by rounding down:
#define ALIGN_DOWN(ptr, alignment) (((unsigned long)(ptr))&~((alignment)-1))

/******************************************************************************
Static data.
******************************************************************************/
static int assert_retrieve_locked = 0;
static logging_fifo_log_t assert_log_header;

/******************************************************************************
Static functions.
******************************************************************************/
static unsigned char *log_ptr (logging_fifo_log_t *log, unsigned char *ptr);
static int log_peek (logging_fifo_log_t *log, unsigned char *ptr, void *buffer, int nbytes);
static int vc_flush_dcache(void);
static void vc_retrieve_log_header(logging_header_t *log_header, uint32_t log_address);
static void vc_retrieve_assert_log_header(logging_fifo_log_t *fifo_log_header, uint32_t log_address);

/*---------------------------------------------------------------------------*/

/***************************************************************************/
/**
*  Flush the VC02 data cache
*
*  @param1
*
*  @return  1 if OK, 0 if timed out
*
*  @remarks
*
*/
#define MAX_CACHE_FLUSH_RDY_WAIT    100     // in ticks
#define CACHE_FLUSH_WAIT            5       // in ticks
#define MAX_SLEEPCNT                (MAX_CACHE_FLUSH_RDY_WAIT/CACHE_FLUSH_WAIT)

int vc_flush_dcache(void)
{
   uint32_t mccs;
   int sleepCnt = 0;

   // now flush the data cache to make sure that data memory content is current

   // first save cache MCCS register value
   vc_host_read_consecutive(&mccs, 0x10000C00, 4, 0);
   mccs |= VC_HTOV32(0x40000000);     // set DCF bit
   vc_host_write_consecutive(0x10000C00, &mccs, 4, 0);

   // wait until the caches are flushed (DCE [bit 0] is set in MCCS)
   vc_host_read_consecutive(&mccs, 0x10000C00, 4, 0);
   while ( !(VC_VTOH32(mccs) & 0x1) && (sleepCnt < MAX_SLEEPCNT) ) {
      set_current_state(  TASK_INTERRUPTIBLE );

      schedule_timeout(CACHE_FLUSH_WAIT);
      sleepCnt++;
      vc_host_read_consecutive(&mccs, 0x10000C00, 4, 0);
   }

   // reset DCF
   mccs &= ~VC_HTOV32(0x40000000);     // set DCF bit
   vc_host_write_consecutive(0x10000C00, &mccs, 4, 0);

   if (sleepCnt == MAX_SLEEPCNT) return 0;
   return 1;
}

/***************************************************************************/
/**
*  retrieve the vclib log header
*
*  @param1   log_header         (out) pointer to where vclib header log header is to be copied
*  @param2   log_address        (in)  start address of vclib header log within VC02
*
*  @return
*
*  @remarks
*
*/
void vc_retrieve_log_header(logging_header_t *log_header, uint32_t log_address)
{
   int i;

   if (!vc_flush_dcache()) vc_assert(0);

   // grab log out of VC02
   vc_host_read_consecutive(log_header, log_address, sizeof(logging_header_t), 0);

   // handle endianness assuming log header consists only of 32-bit word items
   for (i=0; i<(sizeof(logging_header_t)/sizeof(uint32_t)); i++) {
      ((uint32_t *)log_header)[i] = VC_VTOH32(((uint32_t *)log_header)[i]);
   }
}

/***************************************************************************/
/**
*  retrieve the assert log header
*
*  @param1   fifo_log_header    (out) pointer to where assert log header is to be copied
*  @param2   log_address        (in)  start address of vclib log within VC02
*
*  @return
*
*  @remarks
*
*/
void vc_retrieve_assert_log_header(logging_fifo_log_t *fifo_log_header, uint32_t log_address)
{
   logging_header_t log_header;
   logging_fifo_log_t *fifo_log_address;

   vc_retrieve_log_header(&log_header, log_address);
   vc_assert(log_header.assertion_log.type == LOGGING_ASSERTION_LOG);
   fifo_log_address = (logging_fifo_log_t *)((uint32_t)log_header.assertion_log.log);

   // retrieve FIFO log header and take care endianness
   vc_host_read_consecutive(fifo_log_header, (uint32_t)fifo_log_address, sizeof(logging_fifo_log_t), 0);
   fifo_log_header->start = (char *)VC_VTOH32((uint32_t)fifo_log_header->start);
   fifo_log_header->end = (char *)VC_VTOH32((uint32_t)fifo_log_header->end);
   fifo_log_header->ptr = (char *)VC_VTOH32((uint32_t)fifo_log_header->ptr);
   fifo_log_header->next_msg = (char *)VC_VTOH32((uint32_t)fifo_log_header->next_msg);
}


/***************************************************************************/
/**
*  retrieve the next assert log message
*
*  @param1   log_address    (in)  start address of vclib log within VC02
*  @param2   time           (out) time stamp (in usec)
*  @param3   filename       (out) filename where assert occurred
*  @param4   lineno         (out) line number where assert occurred
*  @param5   cond           (out) assert condition string
*
*  @return 1 if log message found, 0 if none
*
*  @remarks
*
*/
#define LOGSIZE 1024
int vc_retrieve_assert_log
(
   uint32_t          log_address,
   uint32_t         *time,
   unsigned short   *seqNum,
   char             *filename,
   uint32_t          maxFileNameLen,
   uint32_t         *lineno,
   char             *cond,
   uint32_t          maxCondLen
)
{
   logging_fifo_log_msg_header_t header;
   unsigned char *next_msg_addr;
   char *log;
   int cond_start_pos;
   int msg_size;
   uint32_t localLineno;
   logging_header_t log_header;
   logging_fifo_log_t *logging_fifo_log_t_addr;
   uint32_t next_msg_addr_as_int;
   size_t maxLen;
   int peekHeaderErr, peekMsgErr;

   if (assert_retrieve_locked)
   {
      // something has gone haywire, so exit
      return 0;
   }

   log = kmalloc(LOGSIZE, GFP_KERNEL);
   vc_retrieve_assert_log_header(&assert_log_header, log_address);

   // Use do-while(0) so that we can break out on errors.
   do
   {
      if (assert_log_header.ptr != assert_log_header.next_msg)
      {
         next_msg_addr = assert_log_header.next_msg;

         // first read message header making sure endianness is taken care of
         peekHeaderErr = log_peek(&assert_log_header, next_msg_addr, &header, sizeof(header));
         header.time = VC_VTOH32(header.time);
         header.seq_num = VC_VTOH16(header.seq_num);
         header.size = VC_VTOH16(header.size);


         *seqNum = header.seq_num;

         // read message content
         next_msg_addr = log_ptr(&assert_log_header, next_msg_addr+sizeof(header));
         msg_size = header.size-sizeof(header);
         msg_size = msg_size > LOGSIZE-12 ? LOGSIZE-12 : msg_size;
         peekMsgErr = log_peek(&assert_log_header, next_msg_addr, log, msg_size);

         // update next message address
         next_msg_addr = log_ptr(&assert_log_header, next_msg_addr+header.size-sizeof(header));

         if ( ( msg_size != (header.size-sizeof(header) ) ) ||
              ( next_msg_addr > assert_log_header.end )     ||
              ( next_msg_addr < assert_log_header.start )   ||
              peekHeaderErr                                 ||
              peekMsgErr                                    )
         {
            vc_assert(0);
            assert_retrieve_locked = 1;
            printk( KERN_ERR "vc_retrieve_assert_log : start = 0x%08x : end = 0x%08x : ptr = 0x%08x : next_msg = 0x%08x : next_msg_addr = 0x%08x\n",
                    (int)assert_log_header.start, (int)assert_log_header.end, (int)assert_log_header.ptr, (int)assert_log_header.next_msg, (int)next_msg_addr );
            printk( KERN_ERR "vc_retrieve_assert_log : time = %ld : seq_num = %d : size = %d\n",
                    header.time, header.seq_num, header.size );
            vc_dump_mem( "assert_hdr", 0, &header, sizeof( header ) );
            vc_dump_mem( "assert_log", 0, log, 128 );
            kfree(log);
            return 0;
         }

         // parse log message
         *time = header.time;

         // strncpy doesn't guarantee that the dst buffer will be NULL terminated.
         // Need to terminate it manually, in case the src buffer is longer than
         // the dst buffer.
         strncpy( filename, log, maxFileNameLen );
         filename[ maxFileNameLen - 1 ] = '\0';

         memcpy( &localLineno, &log[strlen(log)+1], sizeof( localLineno ));
         *lineno = VC_VTOH32(localLineno);

         cond_start_pos = strlen(log)+1+sizeof(uint32_t);
         maxLen = min( maxCondLen, msg_size - cond_start_pos + 1 );
         memcpy(cond, &log[cond_start_pos], maxLen);

         // condition string isn't null terminated
         cond[ maxLen - 1] = 0;

         //
         // Update read ptr for the benefit of both the VC02 and the host
         //
         // first we need to get the address of the VC02 assert logging fifo control structure
         vc_retrieve_log_header( &log_header, log_address );
         vc_assert( log_header.assertion_log.type == LOGGING_ASSERTION_LOG );
         logging_fifo_log_t_addr = (logging_fifo_log_t *)((uint32_t)log_header.assertion_log.log);

         // now convert the read ptr to the appropriate endianess and write it to the VC02 control structure
         next_msg_addr_as_int = VC_HTOV32( (uint32_t)next_msg_addr );
         vc_host_write_consecutive( (uint32_t)(&logging_fifo_log_t_addr->next_msg), &next_msg_addr_as_int, 4, 0 );

         kfree(log);
         return 1;
      }
   }
   while ( 0 );

   kfree(log);
   return 0;
}
/***************************************************************************/
/**
*  function to print out the errors observed by the VC02 host driver.
*
*  @param1   msg      (in)       string to be printed out for the error
*  @param2   errorLog (in/out)   struction containing the error counters
*
*  @remarks
*     this function prints out VC02 host driver errors.  There are 3 configuration
*     flags that control the behaviour:
*        1) gErrorPrtEnable - enable / disable the print out of errors
*        2) gErrorPeriodLenSec - length of each tracking period in second
*        3) gErrorPerPeriod - the number of errors to be printed out in each tracking period
*                             e.g. the first 3 errors in each period will be printed out if this is set to 3
*
*/

#define JIFFIES_PERSEC  128
void vc_hostdrv_errlog( char * msg, VC_ERRORlog_t * errorLog )
{
   int timediff;
   unsigned int currentTime = jiffies;

   if( errorLog->errorCount == 0 )
   {
      /* first error after boot up */
      errorLog->periodExpiresJiffies = currentTime + (gErrorPeriodLenSec * JIFFIES_PERSEC);
   }
   errorLog->errorCount++;

   if( !gErrorPrtEnable )
   {
      /* print out of error is not enabled, return from here */
      return;
   }

   timediff = currentTime - errorLog->periodExpiresJiffies;
   if( timediff > (gErrorPeriodLenSec * JIFFIES_PERSEC) )
   {
      /* error has not occur for > 1 period time, reset the period */
      errorLog->periodExpiresJiffies = currentTime + (gErrorPeriodLenSec * JIFFIES_PERSEC);
      errorLog->countThisPeriod = 0;
   }
   if( (timediff > 0) && (timediff < 50) )
   {
      /* this period has just end, start another tracking period */
      /* this condition is normally hit when the error keeps on being hit frequently */
      errorLog->periodExpiresJiffies = currentTime + (gErrorPeriodLenSec * JIFFIES_PERSEC);
      errorLog->countThisPeriod = 0;
   }

   if( errorLog->countThisPeriod < gErrorPerPeriod )
   {
      printk( KERN_ERR "time %u %s %d\n", currentTime, msg, errorLog->errorCount );
   }
   errorLog->countThisPeriod++;

}
/******************************************************************************
Static functions.
******************************************************************************/

// Return a valid fifo log pointer (takes care of wraparound at either end).

static unsigned char *log_ptr (logging_fifo_log_t *log, unsigned char *ptr) {
   if (ptr >= log->end) return log->start + (ptr - log->end);
   else if (ptr < log->start) return log->end - (log->start - ptr);
   else return ptr;
}

// Read from the fifo (does not move the fifo pointers).
//
// Return 0 for success, else error.

static int log_peek (logging_fifo_log_t *log, unsigned char *ptr, void *buffer, int nbytes) {
   char *local_buffer;
   uint32_t aligned_ptr;
   int avail = log->end - ptr;

   if ( ( avail < 0 ) || ( ptr < log->start ) )
   {
      // Error.
      return ( 1 );
   }

   local_buffer = kmalloc(LOGSIZE, GFP_KERNEL);
   vc_assert( LOGSIZE > nbytes+8);
   aligned_ptr = (uint32_t)ALIGN_DOWN(ptr, 4);
   if (avail >= nbytes) {
      // vc_host_read_consecutive can only read word aligned addresses from VC02
      vc_host_read_consecutive(local_buffer, aligned_ptr, ALIGN_UP(nbytes+(uint32_t)ptr-aligned_ptr, 4), 0);
      memcpy(buffer, &local_buffer[(uint32_t)ptr-aligned_ptr], nbytes);
   } else {
      vc_host_read_consecutive(local_buffer, aligned_ptr, ALIGN_UP(avail+(uint32_t)ptr-aligned_ptr, 4), 0);
      memcpy(buffer, &local_buffer[(uint32_t)ptr-aligned_ptr], avail);
      vc_host_read_consecutive(local_buffer, (uint32_t)log->start, ALIGN_UP(nbytes-avail, 4), 0);
      memcpy((unsigned char *)buffer+avail, local_buffer, nbytes-avail);
   }
   kfree(local_buffer);

   return ( 0 );
}


