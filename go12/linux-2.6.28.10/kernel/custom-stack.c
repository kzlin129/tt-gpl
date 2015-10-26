/****************************************************************************
*
*  Copyright (C) 2007 Dave Hylands (Broadcom Corporation)
*  Copyright (C) 1996-2000 Russell King - Converted to ARM.
*  Original Copyright (C) 1995  Linus Torvalds
*
*  This file is subject to the terms and conditions of the GNU General Public
*  License.  See the file "COPYING" in the main directory of this archive
*  for more details.
*
*****************************************************************************
*
*  custom-stack.c
*
*  PURPOSE:
*
*     Allows customized stack sizes to be used on a per-thread basis.
*
*  NOTES:
*
*     This code was taken from the arch/arm/kernel/process.c and generalized
*     to allow 16k and 8k stacks, and be used on the ARM and MIPS
*
****************************************************************************/

#include <linux/config.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>

/* ---- Include Files ---------------------------------------------------- */
/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

#define DEBUG_ENABLED   0

#if DEBUG_ENABLED
#   define CS_DEBUG( fmt, args... ) printk( "%s: " fmt, __FUNCTION__ , ## args )
#else
#   define CS_DEBUG( fmt, args... )
#endif  

/*
 * Task structure and kernel stack allocation.
 */

struct stack_list 
{
   unsigned long *head;
	unsigned int nr;
};

/* ---- Private Variables ------------------------------------------------ */

#define  SIMPLE_ALLOC   0

#if SIMPLE_ALLOC == 0

static DEFINE_PER_CPU(struct stack_list, stack_list) = { NULL, 0 };

#define CACHED_STACKS	4

#define CUSTOM_STACK_DEFAULT_ALIGN  ( THREAD_BIG_STACK_SIZE - THREAD_DEFAULT_STACK_SIZE )

#endif

/* ---- Functions -------------------------------------------------------- */

struct thread_info *alloc_thread_info(struct task_struct *task, unsigned long flags)
{
   unsigned long *stack = NULL;

#if SIMPLE_ALLOC
   stack = (unsigned long *)
         __get_free_pages(GFP_KERNEL, CONFIG_CUSTOM_STACK_DEFAULT_ORDER);

#ifdef CONFIG_DEBUG_STACK_USAGE
   	/*
   	 * The stack must be cleared if you want SYSRQ-T to
   	 * give sensible stack usage information
   	 */
   	if (stack)
   		memset(stack, 0, THREAD_DEFAULT_STACK_SIZE);
#endif

#else
	if ((CACHED_STACKS) && ((flags & CLONE_CUSTOM_STACK) == 0)) {
		struct stack_list *sl = &get_cpu_var(stack_list);
		unsigned long *p = sl->head;

		if (p) {
			sl->head = (unsigned long *)p[0];
			sl->nr -= 1;
		}
		put_cpu_var(stack_list);

		stack = p;

      if ( stack )
      {
         CS_DEBUG( "Pulled default stack from cache @ 0x%08lx\n", (unsigned long)stack );
      }

#ifdef CONFIG_DEBUG_STACK_USAGE
      /*
       * The stack must be cleared if you want SYSRQ-T to
       * give sensible stack usage information
       */
      if (stack)
         memset(stack, 0, THREAD_DEFAULT_STACK_SIZE);
#endif
   }

	if (!stack)
   {
      if (flags & CLONE_CUSTOM_STACK)
      {
         /* The custom stack is aligned on its own size, so we can just use
          * the custom order.
          */

         stack = (unsigned long *)
               __get_free_pages(GFP_KERNEL, CONFIG_CUSTOM_STACK_BIG_ORDER);

         printk( "alloc_thread_info: Allocated BIG stack @ 0x%08lx\n", (unsigned long)stack );

#ifdef CONFIG_DEBUG_STACK_USAGE
      	/*
      	 * The stack must be cleared if you want SYSRQ-T to
      	 * give sensible stack usage information
      	 */
      	if (stack)
      		memset(stack, 0, THREAD_BIG_STACK_SIZE);
#endif
      }
      else
      {
         unsigned long *wrong_alignment = NULL;

         /* For regular sized stacks, we need them to be aligned properly,
          * so we keep allocating until we get one on the proper alignment.
          */

         do
         {
            stack = (unsigned long *)
                  __get_free_pages(GFP_KERNEL, CONFIG_CUSTOM_STACK_DEFAULT_ORDER);
            if (!stack)
            {
               break;   // Out of memory
            }

            CS_DEBUG( "Allocated default stack @ 0x%08lx\n", (unsigned long)stack );

            if ( CUSTOM_STACK_DEFAULT_ALIGN )
            {
               if (( (unsigned long)stack & ( THREAD_BIG_STACK_SIZE - 1uL )) != CUSTOM_STACK_DEFAULT_ALIGN )
               {
                  CS_DEBUG( "Allocated 0x%08lx, wrong alignment\n", (unsigned long)stack );

                  *(unsigned long **)stack = wrong_alignment;
                  wrong_alignment = stack;
                  stack = NULL;
               }
            }
         } while (stack == NULL);

         /* Free up the extra wrongly aligned stacks that we allocated
          */

         while (wrong_alignment)
         {
            unsigned long *next_stack = *(unsigned long **)wrong_alignment;
            CS_DEBUG( "Freeing 0x%08lx, wrong mask\n", (unsigned long)wrong_alignment );

            free_pages((unsigned long)wrong_alignment, CONFIG_CUSTOM_STACK_DEFAULT_ORDER);

            wrong_alignment = next_stack;
         }

         CS_DEBUG( "Allocated default stack @ 0x%08lx\n", (unsigned long)stack );

#ifdef CONFIG_DEBUG_STACK_USAGE
   	/*
   	 * The stack must be cleared if you want SYSRQ-T to
   	 * give sensible stack usage information
   	 */
   	if (stack)
   		memset(stack, 0, THREAD_DEFAULT_STACK_SIZE);
#endif
      }
   }
#endif

   CS_DEBUG( "Have a stack @ 0x%08lx\n", (unsigned long)stack );

   if ( stack )
   {
      struct thread_info *thread;

      /* NOTE: The following expression works regardless of the stack size
       *       that was actually allocated.
       */
       
      thread = (struct thread_info *)
         (((unsigned long)stack | ( THREAD_BIG_STACK_SIZE - 1 )) + 1 - sizeof( struct thread_info ));

      CS_DEBUG( "thread_info @ 0x%08lx\n", (unsigned long)thread );

      return thread;
   }

   return NULL;
}

void free_thread_info(struct thread_info *thread)
{
   unsigned long stack = (unsigned long)&thread[ 1 ];

   CS_DEBUG( "0x%08lx\n", (unsigned long)thread );

#if SIMPLE_ALLOC

   stack -= THREAD_DEFAULT_STACK_SIZE;

   CS_DEBUG( "default stack @ 0x%08lx\n", stack );

   free_pages(stack, CONFIG_CUSTOM_STACK_DEFAULT_ORDER);
#else
   if (thread->task->flags & CLONE_CUSTOM_STACK)
   {
      stack -= THREAD_BIG_STACK_SIZE;

      CS_DEBUG( "big stack @ 0x%08lx\n", stack );

      free_pages(stack, CONFIG_CUSTOM_STACK_BIG_ORDER);
   }
   else
   {
      stack -= THREAD_DEFAULT_STACK_SIZE;

      if (CACHED_STACKS) {
         struct stack_list *sl = &get_cpu_var(stack_list);
         if (sl->nr < CACHED_STACKS) {

            unsigned long *p = (unsigned long *)stack;

            CS_DEBUG( "caching default stack @ 0x%08lx\n", stack );

            p[0] = (unsigned long)sl->head;
            sl->head = p;
            sl->nr += 1;
            put_cpu_var(stack_list);
            return;
         }
         put_cpu_var(stack_list);
      }

      CS_DEBUG( "default stack @ 0x%08lx\n", stack );

      free_pages(stack, CONFIG_CUSTOM_STACK_DEFAULT_ORDER);
   }
#endif
}


