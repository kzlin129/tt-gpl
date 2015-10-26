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


#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/broadcom/vc03/vcos.h>
#include <linux/dma-mapping.h>

struct DMA_Mem_s
{
    struct list_head    list;   // List of DMA allocated buffers
    
    void       *virtAddr;
    dma_addr_t  physAddr;
    size_t      numBytes;
};

typedef struct DMA_Mem_s DMA_Mem_t;

static  LIST_HEAD( gDmaMemList );

static  DECLARE_MUTEX( gMemLock );

/*******************************************************************
 *
 *  Kernel mode OS layer
 *
 *      os_thread_start tasks have a prototype of
 *
 *          void thread_func( unsigned int argc, void **argv )
 *
 *      while kernel functions have a prototype of:
 *
 *          int thread_func( void *data )
 *
 *  So this function is the glue to put these two together.
 *
 *******************************************************************/
typedef void (*OS_TASK_FUNCPTR)(unsigned int argc, char **argv );

typedef struct
{
   OS_TASK_FUNCPTR func;
   int arg;
} FUNC_INFO_T;


int os_thread_start_helper( FUNC_INFO_T *info )
{
    FUNC_INFO_T local;

    local.arg = info->arg;
    local.func= info->func;
    
    os_free(info);
    (*local.func)(0, local.arg);

    return 0;
}

void *vcos_alloc( size_t size, const char *name, const char *function, const char *fileName, int lineNum )
{
    void        *mem;
    const char  *s;
    const char  *allocStr = "";

    if (( s = strrchr( fileName, '/' )) == NULL )
    {
        s = fileName;
    }
    else
    {
        s++;
    }

    if ( strcmp( name, "MPHI_BUFFER" ) == 0 )
    {
        DMA_Mem_t   *dmaMem = kmalloc( sizeof( *dmaMem ), GFP_KERNEL );

        allocStr = "dma_alloc";

        dmaMem->numBytes = size;

        // Use DMA coherent buffer

        dmaMem->virtAddr = dma_alloc_coherent( NULL, size, &dmaMem->physAddr, GFP_KERNEL );

        if (( mem = dmaMem->virtAddr ) == NULL )
        {
            kfree( dmaMem );
        }
        else
        {
            down( &gMemLock );
            list_add_tail( &dmaMem->list, &gDmaMemList );
            up( &gMemLock );
        }
    }
    else
    {
        // We use kmalloc since we can DMA from kmalloc'd buffers, but we can't DMA from vmalloc'd buffers
    
        if ( gVcOsKmalloc )
        {
            allocStr = "kmalloc";

            mem = kmalloc( size, GFP_KERNEL );
        }
        else
        {
            allocStr = "vmalloc";

            mem = vmalloc( size );
        }
    }

    VC_DEBUG( MemAllocTrace, "%s (%s, %d) allocating %d bytes for %s using %s", function, s, lineNum, size, name, allocStr );


    if ( mem == NULL )
    {
        printk( KERN_ERR "%s: Unable to kmalloc %d bytes (func: %s file: %s line: %d\n", 
                __FUNCTION__, size, function, fileName, lineNum );
    }

    return mem;
}

void  vcos_free( void *mem )
{
    if ((( (unsigned long)mem >= VMALLOC_START ) ) && ( (unsigned long)mem < VMALLOC_END ))
    {
        vfree( mem );
    }
    else
    if ( (unsigned long)mem >= VMALLOC_END )
    {
        struct  list_head  *dmaMemEntry;

        // This was DMA memory

        down( &gMemLock );

        list_for_each( dmaMemEntry, &gDmaMemList )
        {
            DMA_Mem_t   *dmaMem = list_entry( dmaMemEntry, DMA_Mem_t, list );

            if ( dmaMem->virtAddr == mem )
            {
                dma_free_coherent( NULL, dmaMem->numBytes, dmaMem->virtAddr, dmaMem->physAddr );
                list_del( &dmaMem->list );

                kfree( dmaMem );
                break;
            }
        }
        up( &gMemLock );
    }
    else
    {
        kfree( mem );
    }
}

int os_thread_start(OS_THREAD_T *threadptr, void *task, int arg, int stack_size, char *name)  {    FUNC_INFO_T *info = os_malloc(sizeof(FUNC_INFO_T),0,"os_thread_start");    info->func = task;    info->arg  = arg;        *(threadptr) = kthread_run(os_thread_start_helper, (void *)info, name);	     return  0;                                                    }int os_thread_start_param(OS_THREAD_T *threadptr, void *task, char *name, void *param){							     *(threadptr) = kthread_run(task, param, name);	    return 0;}int os_thread_stop(OS_THREAD_T *threadptr) {				     kthread_stop(threadptr);    return 0;}