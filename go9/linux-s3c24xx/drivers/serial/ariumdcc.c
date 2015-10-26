/*
 *  drivers/char/ariumdcc.c
 *
 *  Copyright (C) 2003 American Arium
 *
 * Emulate a serial device using the DCC registers available on most ARM9
 * implementations.
 *
 * This version is for use with Linux 2.6 kernels
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#include <linux/config.h>
#else
#include <linux/autoconf.h>
#endif
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/console.h>
#include <linux/completion.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

//#define XY_INLINE __inline   
#define XY_INLINE	 			//for debugging - make it easier to single step in the driver

#ifdef CONFIG_MODVERSIONS
#include <linux/modversions.h>
#endif

#include <linux/workqueue.h>
#include <linux/kernel.h>

#define CONFIG_DISABLE_HLT defined(CONFIG_PXA27x)  // prevent PXA27x from sleeping at idle

#ifdef CONFIG_DISABLE_HLT
void disable_hlt(void);
void enable_hlt(void);
#endif


#ifdef CONFIG_DEVFS_FS /* only if enabled, to avoid errors in 2.0 */
#include <linux/devfs_fs_kernel.h>
#endif /* CONFIG_DEVFS_FS */

#define  CONFIG_TIMESYS

#define CONFIG_ARIUMDCC_CONSOLE         /* Add console support? #undef if no. */
#define CONFIG_ARIUM_NR_PORTS       8   /* Used to be NR_PORTS. The number of ports. */
#define CONFIG_ARIUM_TIMEOUT_SECS   2   /* Max nbr of seconds to wait for SourcePoint. */
#define CONFIG_ARIUMDCC_MINOR      72   /* Starting minor number. used to be just ARIUMDCC_MINOR. */

#define USE_FLIP_FUNCTIONS  LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)

/* FIXME: I don't know exactly when these macros changed, so this should
 * be fixed to be more precise.
 *  BCL - apparently 2.6 has a revised approach kdev_t is obsolete.
 */
typedef u32 kdev_t;    // BCL

#if 0 // LINUX_VERSION_CODE > KERNEL_VERSION(2,4,99)
#define ARIUM_MAJOR(n) major(n)
#define ARIUM_MINOR(n) minor(n)
#define ARIUM_MK_KDEV(n,m) mk_kdev(n,m)
#else
#define ARIUM_MAJOR(n) MAJOR(n)
#define ARIUM_MINOR(n) MINOR(n)
#define ARIUM_MK_KDEV(n,m) MKDEV(n,m)
#endif

/* #undef ARIUMDCC_DEBUG */

#ifdef CONFIG_CPU_XSCALE
#define ARIUMDCC_VERSION  3  /* Version number of the protocol used: uses rx acks. */
#else
#define ARIUMDCC_VERSION  2  /* Version number of the protocol used. */
#endif
#define ARIUMDCC_MINOR_VERSION  7 /* Updated on each change. */


#define ARIUMDCC_OPTION_BUFFER 1 /* Do we want to use fragment flag? */
#define ARIUMDCC_TRAILER_ENABLED 0

typedef unsigned int dcc_status_t;


#define MAX_POLL_LOOPS           10000 /* number of tries to check dcc status */
#define ARIUMDCC_RING_BUFFER_SIZE 1024  /* number of 32bit words, not bytes */
#define DCC_RESET_SEQUENCE       0x1b1b1b1b /* ASCII code for ESC, 4 times */
#define DCC_RESET_RESPONSE       ~0x1b /* inverted ASCII code for ESC */


static dcc_status_t dcc_status;
static DECLARE_WAIT_QUEUE_HEAD(writeq);
static DECLARE_MUTEX(readsem); /* prevent multiple simultaneous read()s */
static DECLARE_MUTEX(writesem); /* prevent multiple simultaneous write()s */
static atomic_t timer_isr_enabled;
static struct completion timer_isr_done;
static int first_time_reset = 0;

/* The following variable could be used to throttle output if there is */
/* no emulator to talk to.  But, for now, it is only used to check     */
/* to see if the console should be registered or not...                */
static int good_reset = 0;

#ifndef CONFIG_TIMESYS
static struct work_struct dcctq;
#else  /* !CONFIG_TIMESYS */
static struct timer_list timer;
#endif /* !CONFIG_TIMESYS */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void dcc_packet_read(void*);
DECLARE_WORK(dcc_packet_read_task, dcc_packet_read, 0);
#else
static void dcc_packet_read(struct work_struct *work);
DECLARE_WORK(dcc_packet_read_task, dcc_packet_read);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void dcc_poll(void*);
DECLARE_WORK(dcc_poll_task, dcc_poll, 0);
#else
static void dcc_poll(struct work_struct *work);
DECLARE_WORK(dcc_poll_task, dcc_poll);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void console_write(void*);
#else
static void console_write(struct work_struct *work);
#endif

//DECLARE_WORK(console_write_task, console_write, 0);

static struct workqueue_struct* ariumdcc_wq = 0;

static int do_dcc_readwrite( void );

#ifdef CONFIG_DEVFS_FS
/* An empty file_operations struct. We don't do anything, yet. */
struct file_operations ariumdcc_fops;
#endif /* CONFIG_DEVFS_FS */

// module loading hooks
static int enable_arium_module_hook(void);
static int disable_arium_module_hook(void);

void arium_module_hook( struct module * mod );

/*
 * I/O buffer layer, including timer ISR
 */

#define ARIUMDCC_MAX_BODY_SIZE (1024)  /* Max we allow right now.        */
#define ARIUMDCC_MAX_CHANNEL   (32767) /* 15 bits == 32767 bytes maximum */

typedef unsigned int dcc_header;

#define GET_DCC_HEADER_LEN(i)        (((i) & 0x00007fff))
#define GET_DCC_HEADER_MOREFRAG(i)   (((i) & 0x80000000) >> 31)
#define GET_DCC_HEADER_CHAN(i)       (((i) & 0x7fff0000) >> 16)
#if 0 /* Currently, special single byte transfer is not enabled. */
#define GET_DCC_HEADER_SINGLEFLAG(i) (((i) & 0x00008000) >> 15)
#define GET_DCC_HEADER_DATABYTE(i)   (((i) & 0x000000ff))
#endif

#define SET_DCC_HEADER_LEN(i, v)        ((((unsigned int)v) & 0x00007fff) | \
                                                (i & 0xffff8000))
#define SET_DCC_HEADER_MOREFRAG(i, v)   (((((unsigned int)v) << 31) & 0x80000000) | \
                                                (i & 0xffff7fff))
#define SET_DCC_HEADER_CHAN(i, v)       (((((unsigned int)v) << 16) & 0x7fff0000) | \
                                                (i & 0x8000ffff))
#if 0 /* Currently, special single byte transfer is not enabled. */
#define SET_DCC_HEADER_SINGLEFLAG(i, v) (((((unsigned int)v) << 15) & 0x00008000) | \
                                                (i & 0xffff7fff))
#define SET_DCC_HEADER_DATABYTE(i, v)   (((((unsigned int)v)      ) & 0x000000ff) | \
                                                (i & 0xffffff00))
#endif


typedef unsigned int dcc_word;


struct ring_buffer {
    unsigned int head;
    unsigned int tail;
    int len;
    unsigned int buf[ARIUMDCC_RING_BUFFER_SIZE];
};

static struct ring_buffer inring;
static struct ring_buffer outring;
static DECLARE_MUTEX(inring_sem);   /* serialize inring access */
static DECLARE_MUTEX(outring_sem);  /* prevent multiple simultaneous write()s */
static spinlock_t outringlock = SPIN_LOCK_UNLOCKED;

/* just a magic pseudo-random number */
static const dcc_word dcc_init_word = 0xa1370535;

/* Escape word (ASCII for 4 ESC characters) */
static dcc_word dcc_escape_word = 0x1b1b1b1b;

#ifdef CONFIG_CPU_XSCALE
/* Acknowledge word */
static const dcc_word dcc_ack_word = 0x8c6310a5;
#endif

#define ARIUMDCC_RESPONSE_TOKEN 0x42
#define ARIUMDCC_RESPONSE_SYSTEM 0x01    /* 1 == Linux     */
#define ARIUMDCC_RESPONSE_VERSION   ARIUMDCC_VERSION

static dcc_word dcc_response_word =
    ( ((ARIUMDCC_RESPONSE_TOKEN   & 0xff) << 24)
    | ((ARIUMDCC_RESPONSE_SYSTEM  & 0xff) << 16)
    |  (ARIUMDCC_RESPONSE_VERSION & 0xffff));

/*****************************************************************
 * Ring Buffer Routines
 *****************************************************************/

static XY_INLINE int ring_buffer_len(struct ring_buffer *rb)
{
    return rb->len;
}

static XY_INLINE int ring_buffer_len_ex(struct ring_buffer *rb, struct semaphore* sem)
{
    int d;
    down(sem);
    d = rb->len;
    up(sem);
    return d;
}

static XY_INLINE int is_ring_buffer_empty(struct ring_buffer *rb)
{
    return rb->len <= 0;
}

static XY_INLINE int is_ring_buffer_empty_ex(struct ring_buffer *rb,
                                            struct semaphore* sem)
{
    int d;
    down(sem);
    d = (rb->len <= 0);
    up(sem);
    return d;
}


static XY_INLINE int is_ring_buffer_full(struct ring_buffer *rb)
{
    return rb->len >= ARIUMDCC_RING_BUFFER_SIZE;
}

static XY_INLINE void ring_buffer_push(struct ring_buffer *rb, unsigned int d)
{
    rb->buf[rb->tail++] = d;
    if (rb->tail >= ARIUMDCC_RING_BUFFER_SIZE)
        rb->tail = 0;
    rb->len++;
}

static XY_INLINE void ring_buffer_push_ex(struct ring_buffer *rb,
                                         struct semaphore* sem,
					 unsigned int d)
{
	BUG_ON( in_interrupt() );
    down(sem);
    rb->buf[rb->tail++] = d;
    if (rb->tail >= ARIUMDCC_RING_BUFFER_SIZE)
        rb->tail = 0;
    rb->len++;
    up(sem);
}


static XY_INLINE unsigned int ring_buffer_pop(struct ring_buffer *rb)
{
    unsigned int d = rb->buf[rb->head++];
    if (rb->head >= ARIUMDCC_RING_BUFFER_SIZE)
        rb->head = 0;
    rb->len--;
    return d;
}

#if 0
static XY_INLINE unsigned int ring_buffer_pop_ex(struct ring_buffer *rb,
                                                struct semaphore* sem)
{
    unsigned int d;
    down(sem);
    d = rb->buf[rb->head++];
    if (rb->head >= ARIUMDCC_RING_BUFFER_SIZE)
        rb->head = 0;
    rb->len--;
    up(sem);
    return d;
}
#endif

static XY_INLINE unsigned int ring_buffer_peek(struct ring_buffer *rb)
{
    return rb->buf[rb->head];
}

#if 0
static XY_INLINE unsigned int ring_buffer_peek_ex(struct ring_buffer *rb,
                                                 struct semaphore* sem)
{
    unsigned int d;
    down(sem);
    d = rb->buf[rb->head];
    up(sem);
    return d;
}
#endif
static XY_INLINE void ring_buffer_clear(struct ring_buffer *rb)
{
    rb->head = rb->tail = rb->len = 0;
}

#if 0
static XY_INLINE unsigned int ring_buffer_space(struct ring_buffer *rb)
{
    return ARIUMDCC_RING_BUFFER_SIZE - rb->len;
}
#endif

static XY_INLINE unsigned int ring_buffer_space_ex(struct ring_buffer *rb,
                                                  struct semaphore* sem)
{
    unsigned int d;
    down(sem);
    d = ARIUMDCC_RING_BUFFER_SIZE - rb->len;
    up(sem);
    return d;
}

/*
 * Bitbanging functions for dealing with the DCC registers.
 */

static XY_INLINE unsigned int read_dcc_status( void )
{
    unsigned int val = 0;

#ifdef CONFIG_CPU_XSCALE
    __asm__ __volatile__ (
        "mov %0, #0\n\tmrc p14, 0, R15, C14, C0, 0\n\torrmi %0, %0, #1\n\torrvs %0, %0, #2\n\t"
        : "=r" (val)
    );
#else
#ifdef CONFIG_CPU_V6
    __asm__ __volatile__ (
        "mov %0, #0\n\tmrc p14, 0, R15, C0, C1, 0 \n\torreq %0, %0, #1\n\torrcs %0, %0, #2\n\t"
        : "=r" (val)
    );
#else /* ARM7/9 processor */
    __asm__ __volatile__ (
        "mrc p14, 0, %0, C0, C0\n\t"
        : "=r" (val)
    );
#endif /* CONFIG_CPU_V6 */
#endif /* CONFIG_CPU_XSCALE */

    return val;
}

static XY_INLINE unsigned int read_dcc_data( void )
{
    unsigned int val = 0;

#ifdef CONFIG_CPU_XSCALE
    __asm__ __volatile__ (
        "mrc p14, 0, %0, C9, C0, 0\n\t"
        : "=r" (val)
    );
#else
#ifdef CONFIG_CPU_V6  /* ARM11 processor */
    __asm__ __volatile__ (
        "mrc p14, 0, %0, C0, C5\n\t"
        : "=r" (val)
    );
#else /* ARM 7/9 processor */
    __asm__ __volatile__ (
        "mrc p14, 0, %0, C1, C0\n\t"
        : "=r" (val)
    );
#endif /* CONFIG_CPU_V6 */
#endif /* CONFIG_CPU_XSCALE */

    return val;
}


static XY_INLINE void write_dcc_data(unsigned int d)
{
//	BUG_ON( in_interrupt() );
#ifdef CONFIG_CPU_XSCALE
    __asm__ __volatile__ (
        "mcr p14, 0, %0, C8, C0, 0\n\t"
        :: "r" (d));
#else
#ifdef CONFIG_CPU_V6  /* ARM11 processor */
    __asm__ __volatile__ (
        "mcr p14, 0, %0, C0, C5\n\t"
        :: "r" (d));
#else /* ARM 7/9 processor */
    __asm__ __volatile__ (
        "mcr p14, 0, %0, C1, C0\n\t"
        :: "r" (d));
#endif /* CONFIG_CPU_V6 */
#endif /* CONFIG_CPU_XSCALE */
}


static int last_read_was_escape;
static int last_read_normal_word;
static int last_word_was_escape; /* a word we are escaping */

/*
 * Called while the DCC ISR is disabled to initialize
 * all the state involved in DCC packet transfers.
 * Intended to be called after the handshake.
 */
static void initialize_dcc( void )
{
    /* clear the ring buffers */
    ring_buffer_clear(&inring);
    ring_buffer_clear(&outring);

    /* clear the packet reading state */
    last_read_was_escape = 0;
    last_read_normal_word = 0;
    last_word_was_escape = 0;
}

#define GET_DCC_STATUS_WRITEBIT(i) (((i) & 0x2) == 0x2)
#define GET_DCC_STATUS_READBIT(i)  (((i) & 0x1) == 0x1)

/*
 * Performs the DCC initialization handshake. Should only

 * be called while the DCC timer is disabled.
 * Warning: will not return until the DCC protocol
 * initialization handshake is complete.
 */
static int reset_dcc( void )
{
    unsigned int d;
    unsigned long timeout;

    timeout = jiffies + (HZ * CONFIG_ARIUM_TIMEOUT_SECS );  // Wait up to this many seconds.

    dcc_status = read_dcc_status();

    /* If this is the first time after reset, always send INIT. Otherwise, */
    /* only send the INIT and wait for the INIT back if we don't see an    */
    /* INIT already sitting in the read register...                        */
    if ( !GET_DCC_STATUS_READBIT(dcc_status) ||
         dcc_init_word != read_dcc_data() ||
	     first_time_reset++ == 0 ) {

        /* Wait until we get room to write */
        while ( GET_DCC_STATUS_WRITEBIT( dcc_status = read_dcc_status()) ) {
           if( time_before(timeout, jiffies)) {
               printk(KERN_INFO "ariumdcc: Waiting to connect to SourcePoint emulator\n");
               return 0;
           }
        } 

        write_dcc_data(dcc_init_word);

        d = 0;    // Init this to something other than dcc_init_word for test in following loop.

        do {
            /* Try to read a word, if it is available... */
            if ( GET_DCC_STATUS_READBIT( dcc_status = read_dcc_status()) ) {
              d = read_dcc_data();  /* Ok, something available, now read it. */
            }

            /* If we haven't received dcc_init_word back in timeout time, then... */
            if( time_before(timeout, jiffies)) {
                printk(KERN_INFO "ariumdcc: Waiting to connect to SourcePoint emulator\n");
                return 0;
            }

        /* If not the init word returned to us, keep waiting... */
        } while ( d != dcc_init_word );
    }

    return 1;   /* Reset was successful. */
}

/*
 * Sends the RESPONSE magic word. Supposed to be called after
 * we receive an INIT.
 */
static void send_init_response( void )
{
    /* Write the RESPONSE magic word */
    /* Wait until the write bit clears (wait forever) */
    while ( GET_DCC_STATUS_WRITEBIT( dcc_status = read_dcc_status()));
    write_dcc_data(dcc_response_word);
#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: sent RESPONSE magic word\n");
#endif
}


/*
 * Read from the DCC and puts it into the input buffer. Keeps
 * reading (one word at a time) as long as there is room in the
 * input buffer and there is data to read from the DCC.

 * Returns 0 if successful, non-zero if we got the init sequence
 * and should stop interacting with the DCC immediately.
 * Guaranteed to read at most 1 word, and to place at most 1 word
 * into the ring buffer per call.
 */
//static XY_INLINE void do_dcc_read_word( void )
void do_dcc_read_word( void )
{
    unsigned int d;

    if (last_read_normal_word) {
        /* this happens if we get an escape word followed by a normal
         * word. We don't want to forget about the normal word, but
         * we don't want to add two words in one call. */
        d = last_read_normal_word;
        last_read_normal_word = 0;
    } else {
        d = read_dcc_data();
    }

#ifdef CONFIG_CPU_XSCALE
    // Tell ECM that we've read rx.
    while ( GET_DCC_STATUS_WRITEBIT( dcc_status = read_dcc_status()));
    write_dcc_data(dcc_ack_word);
    while ( GET_DCC_STATUS_WRITEBIT( dcc_status = read_dcc_status()));
#endif

    if (last_read_was_escape) {
        if (d == dcc_init_word) {
            ring_buffer_push(&inring, d);
            last_read_was_escape = 0;
        } else {
            /* got a bogie, stick the last esc into the ring and stick
             * the current word into the last_read word */
            ring_buffer_push(&inring, d);
            last_read_normal_word = d;
            last_read_was_escape = 0;
        }
    } else if (d == dcc_escape_word) {
        last_read_was_escape = 1;
    } else if (d == dcc_init_word) {
#ifdef ARIUMDCC_DEBUG
        printk(KERN_INFO "ariumdcc: received INIT command\n");
#endif
        /* Try doing the init dance (send and init back, then wait for an init. */
        good_reset = reset_dcc();
        /* We got the INIT sequence, so send the response. */
        send_init_response();
        /* Now clear the buffers and just keep going. */
        initialize_dcc();
    } else { /* just a normal character */
        ring_buffer_push(&inring, d);
    }
}
/*
    spin_unlock(&outringlock);
    spin_unlock(&inringlock);
*/

/*
 * Decides if the given word needs to be escaped for outbound data.
 */
static XY_INLINE int should_escape_dcc_write_word(unsigned int d)
{
    /* NOTE: We can never escape a NULL (0x0) word. */

    /* Escape the INIT word */
    if (d == dcc_init_word) {
        return 1;
    }
    /* Escape the ESCAPE word */
    else if (d == dcc_escape_word) {
        return 1;
    }

    /* If we're XScale, escape the DEBUG_HANDLER_OK (0x60) word.
     * Otherwise the ICE may treat the word as a command from the
     * debug kernel. */
#define ARIUMDCC_DEBUG_HANDLER 0x00000060
#define ARIUMDCC_DEBUG_HANDLER_MANGLED 0x00006060
    else if (d == ARIUMDCC_DEBUG_HANDLER) {
        return 1;
    }
    /* Escape the Acknowledge word */
    else if (d == dcc_escape_word) {
        return 1;
    }

    return 0;
}

/*
 * Transforms special outbound words that we can't send across
 * the DCC channel for one reason or another into another form
 * that is safe to transmit.
 */
static XY_INLINE int transform_dcc_write_word(unsigned int d)
{

    /* Mangle an 0x60 to be 0x6060 */
    if (d == ARIUMDCC_DEBUG_HANDLER) {
        return ARIUMDCC_DEBUG_HANDLER_MANGLED;
    }

    return d; /* Leave all others as they were */
}


/*
 * Writes to the DCC from the output buffer. Keeps writing (one
 * word at a time) as long as there are words in the output buffer
 * and the write register of the DCC is clear.
 */
static XY_INLINE void do_dcc_write_word( void )
{
    unsigned int d;

    if (last_word_was_escape) {
        /* Pop the remaining word (we're guaranteed that the ringbuffer
         * contains a word when last_word_was_escape is true). Also
         * transform that word in case the word we're escaping can't
         * be transfered across directly for some reason. */
        d = ring_buffer_pop(&outring);
        d = transform_dcc_write_word(d);
        write_dcc_data(d);
        last_word_was_escape = 0;
    } else {
        /* Just peek in case it's a word we want to escape, in which case
         * we need to be called again to completely flush the buffer. */
        d = ring_buffer_peek(&outring);
        if (should_escape_dcc_write_word(d)) {
            write_dcc_data(dcc_escape_word);
            last_word_was_escape = 1;
        } else {/*
    spin_unlock(&outringlock);
    spin_unlock(&inringlock);
*/

            (void) ring_buffer_pop(&outring);
            write_dcc_data(d);
        }
    }
}

static int do_dcc_readwrite( void )
{
    unsigned int output_buffer_len_begin;
//    int retrytimes = MAX_POLL_LOOPS;
    int retrytimes = 1;
    int productive = 0;


    down(&outring_sem);
    down(&inring_sem);

    output_buffer_len_begin = ring_buffer_len(&outring);
    while (retrytimes-- > 0) {

        dcc_status = read_dcc_status();
        if (GET_DCC_STATUS_READBIT(dcc_status)
                && !is_ring_buffer_full(&inring)) {
            do_dcc_read_word();
            productive = 1;
        }
        if (!GET_DCC_STATUS_WRITEBIT(dcc_status)
                && !is_ring_buffer_empty(&outring)) {
            do_dcc_write_word();
            productive = 1;
         }
      if (productive)
         break;
    }

    up(&outring_sem);
    up(&inring_sem);

    return productive;
}


#ifdef CONFIG_ARIUMDCC_CONSOLE
/* Flush the output ringbuffer.
 * Must ONLY be called from w/in an interrupt context.
 * Must NOT be called while any other routines are dealing
 * with the input or output ringbuffers.
 */
static void flush_dcc_write( void )
{
    unsigned long flags;

    spin_lock_irqsave(&outringlock, flags);

    /* Don't bother locking the input or output ring buffers, since
     * this must only be called from w/in an interrupt context while
     * interrupts are disabled. */
    while (!is_ring_buffer_empty(&outring)) {
        dcc_status = read_dcc_status();
        if (!GET_DCC_STATUS_WRITEBIT(dcc_status)) {
            do_dcc_write_word();
        }
    }

    spin_unlock_irqrestore(&outringlock, flags);
}
#endif /* CONFIG_ARIUMDCC_CONSOLE */


/*
 * Called from the timer ISR. Checks the DCC control register
 * (aka the status register) to see if incoming data is waiting
 * or there is room for outgoing data (and that there is data to
 * send). Wakes up the appropriate sub-tasks when those conditions
 * are true. Continues to add itself to the timer queue as long
 * as the driver wants it.
 */
#ifndef CONFIG_TIMESYS
static void do_check_dcc_status(void *data)
#else   /* !CONFIG_TIMESYS */
static void do_check_dcc_status(unsigned long data)
#endif  /* !CONFIG_TIMESYS */
{
    int timer_isr_status = atomic_read(&timer_isr_enabled);

    if (!timer_isr_status) {
        /* ping a completion port when we aren't going to
         * add ourselves anymore, so that the routine that shut
         * us down can synchronize. */
        complete(&timer_isr_done);
    } else {
        queue_work(ariumdcc_wq, &dcc_poll_task);

#ifdef CONFIG_TIMESYS
        timer.expires  = jiffies + HZ/100;
        add_timer(&timer);
#endif /* CONFIG_TIMESYS */
    }
}

struct channel_tree {
    unsigned int channel;
    struct tty_struct *tty;
    struct channel_tree *left, *right;
    struct channel_tree *next;
};

static struct channel_tree *chantree;
static spinlock_t chantreelock = SPIN_LOCK_UNLOCKED;


static struct channel_tree *find_channel(struct channel_tree *tree,
                                                  unsigned int channel)
{
    if (!tree)
        return NULL;
    if (channel == tree->channel)
        return tree;
    else if (channel < tree->channel)
        return find_channel(tree->left, channel);
    else
        return find_channel(tree->right, channel);
}


static void add_channel(struct channel_tree **tree,
                                 struct channel_tree *node)
{
    if (!*tree) {
        *tree = node;
        return;
    }
    else if (node->channel < (*tree)->channel)
        add_channel(&((*tree)->left), node);
    else if (node->channel > (*tree)->channel)
        add_channel(&((*tree)->right), node);
    else if (node->tty == (*tree)->tty) {
        node->next = (*tree)->next;
        (*tree)->next = node;
    } else
        /* This shouldn't happen */
        ;
}


static struct channel_tree *remove_channel(struct channel_tree **tree,
                                                    unsigned int channel)
{
    if (!*tree)
        return NULL;
    else if (channel == (*tree)->channel) {
        if ((*tree)->next) {
            struct channel_tree *temp = (*tree)->next;
            (*tree)->next = temp->next;
            temp->next = NULL;
            return temp;
        } else {
            struct channel_tree *found = (*tree);
            struct channel_tree *left = (*tree)->left;
            struct channel_tree *right = (*tree)->right;
            found->left = found->right = NULL;
            *tree = left;
            if (right)
               add_channel(tree, right);
            return found;
        }
    }
    else if (channel < (*tree)->channel)
        return remove_channel(&(*tree)->left, channel);
    else
        return remove_channel(&(*tree)->right, channel);
}


/*  Wake up any channels that were waiting because we were
    out of space in the ring buffer.                        */

void  wake_channels(struct channel_tree *tree )
{
    struct channel_tree *next;

    if (!tree)
        return;
    
    tty_wakeup(tree->tty);
    /* wake_up_interruptible(&(tree->tty->write_wait)); */

    next = tree->next;
    while( next ) {
       tty_wakeup(next->tty);
       /* wake_up_interruptible(&(next->tty->write_wait)); */
       next = next->next;
    }

    if (tree->left)
        wake_channels(tree->left);
    if (tree->right)
        wake_channels(tree->right);

}



/*
 * Enable the DCC ISR by hooking it to the timer interrupt.
 */
static void enable_timer_isr( void )
{
    /* initialize the completion port so we can kill the timer ISR later */
    init_completion(&timer_isr_done);

    /* initialize the enabled flag so we can disable it later */
    atomic_set(&timer_isr_enabled, 1);

    /* start the do_check_dcc_status() timer ISR */
#ifndef CONFIG_TIMESYS
    dcctq.routine = do_check_dcc_status;
    queue_task(&dcctq, &tq_timer); /* Start up the perpetual timer ISR */
#else  /* !CONFIG_TIMESYS */
	init_timer(&timer);
	timer.function = do_check_dcc_status;
    timer.expires  = jiffies + HZ/100;
	timer.data = 0;
    add_timer(&timer);
#endif /* !CONFIG_TIMESYS */
}

/*
 * Disable the DCC ISR.
 */
static void disable_timer_isr( void )
{
    int timer_isr_status;
    timer_isr_status = atomic_read(&timer_isr_enabled);

    /* only try to disable it if it's enabled */
    if (timer_isr_status) {
        /* Shut off the do_check_dcc_status() timer ISR */
        atomic_set(&timer_isr_enabled, 0);

        /* wait for it to shut off */
        wait_for_completion(&timer_isr_done);
    }
}

static XY_INLINE void push_char_onto_flip(struct tty_struct *tty,
                                         unsigned char c)
{
#if USE_FLIP_FUNCTIONS
    tty_insert_flip_char(tty, c, 0);	
#else
    /* Swap the buffers right away because we have more to push
     * and we can't hold on to it until later. */
    if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
        /* We're setting this always for now
        tty->low_latency = 1;
        */
        tty_flip_buffer_push(tty);
        /*
        tty->low_latency = 0;
        */
    }

    *tty->flip.char_buf_ptr = c;
    *tty->flip.flag_buf_ptr = 0;
    tty->flip.flag_buf_ptr++;
    tty->flip.char_buf_ptr++;
    tty->flip.count++;
#endif
}

/*poll_task
 * This tasklet drives the DCC polling prcedure.

 TT10731
 The value of POLL_DELAY_REPS is optimized for cogent 637 at 166Mhz
 Putfile/Getfile are affected by this parameter. If slower targets are used,
 may need to increase this value. Faster targets could decrease to reduce
 driver resources.

 */

#define POLL_DELAY_USECS 30   /* usecs between poll attempts */
#define POLL_DELAY_REPS 8     /* number attempts for successful pool */


struct ariumdcc_poll_struct {
   int poll_delay_usecs;
   int poll_delay_reps;
   int task_invocations;
   int task_successes;
   int poll_attempts;
   int poll_successes;
};

struct ariumdcc_poll_struct ariumdcc_poll_stats = {
   POLL_DELAY_USECS,
   POLL_DELAY_REPS,
   0,
   0,
   0
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void dcc_poll(void* ___nothing)
#else
static void dcc_poll(struct work_struct *___nothing)
#endif
{
   int success = 0;
   int reps;

   ariumdcc_poll_stats.task_invocations++;

   for (reps = ariumdcc_poll_stats.poll_delay_reps; reps > 0; --reps  ) {
      ariumdcc_poll_stats.poll_attempts++;
      if ( do_dcc_readwrite() ) {
         reps = ariumdcc_poll_stats.poll_delay_reps;    /* reset reps */
         ariumdcc_poll_stats.poll_successes++;
         success = 1;
      }
      udelay(ariumdcc_poll_stats.poll_delay_usecs);
   }

   if ( success )
   {
	   ariumdcc_poll_stats.task_successes++;
	   wake_up_interruptible(&writeq);

	   if (!is_ring_buffer_empty(&inring))
		   queue_work(ariumdcc_wq, &dcc_packet_read_task);

      wake_channels( chantree );
   }
}

/*
 * Try and parse a single DCC packet. Once we get a whole packet,
 * look in the list of open chscheduleannels and find out if this one is
 * open. If it is, put it in that channel's flip buffer. If it
 * isn't open then just discard it (and post an error).
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void dcc_packet_read(void* ___nothing)
#else
static void dcc_packet_read(struct work_struct *___nothing)
#endif
{
    dcc_header header;
    struct channel_tree *chan = NULL;
    unsigned int d;
    size_t totalbytes;
    size_t bodybytesleft;
    int nwords;
    int nwordsleft;
    unsigned int nstillstuffed;
    char c;

#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: running in read tasklet\n");
#endif

    down(&inring_sem);

tryanotherpacket:

    /* don't bother if the input ring is empty */
    if (is_ring_buffer_empty(&inring))
        goto out;

    header = ring_buffer_peek(&inring);

    spin_lock(&chantreelock);
    chan = find_channel(chantree, GET_DCC_HEADER_CHAN(header));
    spin_unlock(&chantreelock);
    if (!chan)
#ifdef ARIUMDCC_DEBUG
        printk(KERN_INFO "ariumdcc: Failed to find channel node (for ch#%d)!\n",
               GET_DCC_HEADER_CHAN(header));
#else
        ;
#endif /* ARIUMDCC_DEBUG */
    else
        chan->tty->low_latency = 1;

#if ARIUMDCC_TRAILER_ENABLED
    totalbytes = GET_DCC_HEADER_LEN(header) + 6; /* 4 for the header, 2 trailer */
#else
    totalbytes = GET_DCC_HEADER_LEN(header) + 4; /* 4 for the header */
#endif

    bodybytesleft = GET_DCC_HEADER_LEN(header);
    nwords = (totalbytes / 4) + ((totalbytes % 4) ? 1 : 0);

    /* we're waiting for a whole packet to show up in
     * the ring buffer */
    if (ring_buffer_len(&inring) < nwords)
        goto out;

    /* pop the header (we already peeked at it */
    (void) ring_buffer_pop(&inring);

    /* read each word, pull out the bytes, stick them in the
     * given buffer.                                      */
    for (nwordsleft = nwords-1; nwordsleft; nwordsleft--) {
        d = ring_buffer_pop(&inring);
        nstillstuffed = 4;

#if ARIUMDCC_TRAILER_ENABLED
	/* We may read the trailer word (if the trailer is */
    /* alone in a word) and then ignore it. This is by design. */
#endif
        while (bodybytesleft > 0 && nstillstuffed > 0) {
            c = 0xff & (d >> 24);
            if (chan)
                push_char_onto_flip(chan->tty, c);
            /* else we just discard the data */

            d <<= 8;
            nstillstuffed--;
            bodybytesleft--;
        }

        /* Ignore t             ariumdcc_outring_relieved_count = 0;
he trailer for now (we don't use it) */
    }

    if (chan)
        tty_flip_buffer_push(chan->tty);

    goto tryanotherpacket;

out:
    up(&inring_sem);
}

int ariumdcc_outring_full_count = 0;
int ariumdcc_outring_relieved_count = 0;

static int write_dcc_fragment(const unsigned char *body, size_t len,
                              int more_fragment, unsigned int channel,
                              int from_console_write)
{
    int rv = 0;
    dcc_header header = 0;
    size_t lenleft = len;
    const unsigned char *p = body;
    int nwords;

#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: trying to write fragment of len %d\n", len);
#endif

    if (channel > ARIUMDCC_MAX_CHANNEL) {
        return -EINVAL; /* channel out of range */
    }

    if (len > ARIUMDCC_MAX_BODY_SIZE || len == 0) {
        return -EINVAL; /* length too big or too small */
    }

    header = SET_DCC_HEADER_LEN(header, len);
    rv = len;
    header = SET_DCC_HEADER_MOREFRAG(header, (more_fragment ? 1 : 0));
    header = SET_DCC_HEADER_CHAN(header, channel);
#if 0 /* Currently, we do not put first byte in header. */
    databyte = *p++;
    header = SET_DCC_HEADER_DATABYTE(header, databyte);
    lenleft--;
#endif

#if 0 /* Special single byte transfer is not enabled. */
    if (len == 1) { /* special case */
        nwords = 1;
    } else {
#else
    {
#endif
#if ARIUMDCC_TRAILER_ENABLED
        int nbytes = len + 6; /* 4 for the header, 2 for the trailer */
#else
        int nbytes = len + 4; /* 4 for the header */
#endif
        nwords = (nbytes / 4) + ((nbytes % 4) ? 1 : 0);
    }

    /* Don't block if we're already in the kernel or interrupt context!! */
    /* WARNING: We assume here that if we are in the interrupt context,
     * then the output buffer has already been flushed and we have enough
     * room to write this packet. */
    /* wait until the output buffer has enough space for this packet */
    while (!from_console_write
           && ring_buffer_space_ex(&outring, &outring_sem) < nwords) {
//           && ring_buffer_space(&outring) < nwords) {
             ariumdcc_outring_full_count++;

        if (wait_event_interruptible(writeq,
                ring_buffer_space_ex(&outring, &outring_sem) >= nwords)) {
//                ring_buffer_space(&outring) >= nwords)) {
            rv = -ERESTARTSYS;
            goto out;
        }
        ariumdcc_outring_relieved_count++;
    }

#if 0 /* Special single byte transfer is not enabled. */
    if (len == 1) { /* special case, body of length 1 */
        ring_buffer_push_irq(&outring, &outringlock, header);
    } else {
#else
    {
#endif
        int nstuffed = 0;
#if ARIUMDCC_TRAILER_ENABLED
        int ntrailer = 2;
#endif
        unsigned int d = 0;

        ring_buffer_push_ex(&outring, &outring_sem, header);
        for (;;) {
            if (nstuffed == 4) {
                ring_buffer_push_ex(&outring, &outring_sem, d);
                nstuffed = 0;
                d = 0;
            }
            if (lenleft > 0) {
                d <<= 8;
                d |= *p++;
                nstuffed++;
                lenleft--;
            }
#if ARIUMDCC_TRAILER_ENABLED
            else if (ntrailer > 0) {
                d <<= 8;
                d |= 0xff; /* XXX: if we had a trailer, it would go here */
                nstuffed++;
                ntrailer--;
            }
#endif
            else {
                break;
            }

        }
        d <<= ((4 - nstuffed) * 8); /* shift over the last little bit */
        if (nstuffed) {
            ring_buffer_push_ex(&outring, &outring_sem, d);
        }
    }

#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: write_dcc_fragment(len=%d) returning %d\n",
           len, rv);
#endif

out:
    return rv;
}

static ssize_t ariumdcc_write_single_char(unsigned char ch, unsigned int channel)
{
    dcc_header header = 0;

    if (channel > ARIUMDCC_MAX_CHANNEL) { /* 15 bits for the channel field */
        return -EINVAL; /* channel out of range */
    }

    header = SET_DCC_HEADER_LEN(header, 1);
    header = SET_DCC_HEADER_MOREFRAG(header, 1);
    header = SET_DCC_HEADER_CHAN(header, channel);
#if 0 /* special single byte transfers are not currently supported. */
    header = SET_DCC_HEADER_DATABYTE(header, ch);
#endif


    spin_lock(&outringlock);
    /* Is there enough room in buffer for two more words? */
    if (ring_buffer_len(&outring) + 2 >= ARIUMDCC_RING_BUFFER_SIZE) {
        spin_unlock(&outringlock);
        return -EAGAIN;
    }

    ring_buffer_push(&outring, header);
    header = (ch << 24) | 0x00ffffff;
    ring_buffer_push(&outring, header);
    spin_unlock(&outringlock);

    return 1;
}

/*
 * Write the given string of arbitrary length to the dcc.
 * If "buffered" is true, the other side is instructed to buffer
 * up the whole contents of this write before flushing to the higher
 * layer.

 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
static ssize_t ariumdcc_write(const char *buf, size_t nbytes,
                              int buffered, unsigned int channel,
                              int from_user)
{
#else
static ssize_t ariumdcc_write(const char *buf, size_t nbytes,
                              int buffered, unsigned int channel )
{
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
   char tbuf[ARIUMDCC_MAX_BODY_SIZE];
#endif
    ssize_t ret = 0;
    size_t bytesleft = nbytes;

#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: continuing with ariumdcc_write w/ len %d\n",
           nbytes);
#endif

    while (bytesleft > 0) {
        size_t chunklen;
        int more_fragment;   /* Indicates that additional packets follow this one. */

        if (bytesleft > ARIUMDCC_MAX_BODY_SIZE) {
            more_fragment = (buffered ? 1 : 0); /* only bother setting
                                                   the more_fragment if
                                                   we want buffered behavior */
            chunklen = ARIUMDCC_MAX_BODY_SIZE;
        } else {
            more_fragment = 0;
            chunklen = bytesleft;
        }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
        if (from_user) {
            if (copy_from_user(tbuf, buf, chunklen)) {
                ret = -EFAULT;
                goto out;
            }
        }
#endif

        /* only write one fragment at a time, even if someone's sleeping */
        if (down_interruptible(&writesem)) {
            ret = -ERESTARTSYS;
            goto out;
        }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
        if (from_user)
            ret = write_dcc_fragment(tbuf, chunklen, more_fragment, channel, 0);
        else
#endif
            ret = write_dcc_fragment(buf,  chunklen, more_fragment, channel, 0);

        up(&writesem);

        if (ret < 0)
            goto out;
        bytesleft -= chunklen;
    	buf       += chunklen;
    }
    if (bytesleft == 0)
        ret = nbytes; /* we wrote everything */

out:
#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: ariumdcc_write returning %d\n", ret);
#endif
    return ret;
}

#ifdef CONFIG_ARIUMDCC_CONSOLE

#if 1

struct console_work_struct {
   struct work_struct ws;
   unsigned int count;
   char message[1];
};

// console output work queue handler
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)

static void console_write(void* data)
{
    struct console_work_struct* cws = (struct console_work_struct*) data;

#else

static void console_write(struct work_struct *work)
{
    struct console_work_struct* cws =
        container_of(work, struct console_work_struct, ws);
#endif

    ssize_t bytesleft = cws->count;
    const char* s = cws->message;

    while (bytesleft > 0) {
        ssize_t chunklen;
        ssize_t ret;

        if (bytesleft > ARIUMDCC_MAX_BODY_SIZE) {
            chunklen = ARIUMDCC_MAX_BODY_SIZE;
        } else {
            chunklen = bytesleft;
        }

        /* Flush the buffer before we write the data, in case the
         * buffer is full. */
        flush_dcc_write();

        /* The console always writes to channel 0 */

        ret = write_dcc_fragment(s, chunklen,
                                 0 /* always last fragment (don't buffer) */,
                                 0 /* channel */,
                                 0 /* from console write */);

        /* Something failed, but we have no way to recover from that and we
         * can't call printk() since we're already inside printk(), so oh well.
         */
        if (ret < 0)
            break;

        s += ret; /* Move the s pointer forward by the amount of bytes
                   * we ended up writing */

        /* Flush the buffer after we write the data, since it might
         * be critial information like a panic or other crash info. */
        flush_dcc_write();

        bytesleft -= ret;
    }

    kfree((void *)cws);
}


static void ariumdcc_console_write(struct console *co, const char *s,
                                   unsigned int count)
{
    void *data = kmalloc(sizeof(struct console_work_struct) + count, GFP_ATOMIC);
    if (data != 0)
    {
       struct console_work_struct* cws = (struct console_work_struct*) data;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
       INIT_WORK( &cws->ws, console_write, cws );
#else
       INIT_WORK( &cws->ws, console_write);
#endif
       cws->count = count;
       memcpy( cws->message, s, count );
       queue_work(ariumdcc_wq, &cws->ws);
    }
}

#else

static void ariumdcc_console_write(struct console *co, const char *s,
                                   unsigned int count)
{
    ssize_t ret = 0;
    ssize_t bytesleft = count;

    while (bytesleft > 0) {
        ssize_t chunklen;

        if (bytesleft > ARIUMDCC_MAX_BODY_SIZE) {
            chunklen = ARIUMDCC_MAX_BODY_SIZE;
        } else {
            chunklen = bytesleft;
        }


        /* The kernel console writes always happen while interrupts are
         * disabled, so we can assume that the data is not coming from
         * userspace. */

        /* Flush the buffer before we write the data, in case the
         * buffer is full. */
        flush_dcc_write();

        /* The console always writes to channel 0 */

        ret = write_dcc_fragment(s, chunklen,
                                 0 /* always last fragment (don't buffer) */,
                                 0 /* channel */,
                                 1 /* from console write */);

        /* Something failed, but we have no way to recover from that and we
         * can't call printk() since we're already inside printk(), so oh well.
         */
        if (ret < 0)
            goto out;

        s += ret; /* Move the s pointer forward by the amount of bytes
                   * we ended up writing */

        /* Flush the buffer after we write the data, since it might
         * be critial information like a panic or other crash info. */
        flush_dcc_write();

        bytesleft -= ret;
    }
out:
    return;
}
#endif


static struct tty_driver serial_driver;
//static int serial_refcount;  // BCL not used in 2.6
static struct tty_struct *serial_table[CONFIG_ARIUM_NR_PORTS];
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static struct termios *serial_termios[CONFIG_ARIUM_NR_PORTS];
static struct termios *serial_termios_locked[CONFIG_ARIUM_NR_PORTS];
#else
static struct ktermios *serial_termios[CONFIG_ARIUM_NR_PORTS];
static struct ktermios *serial_termios_locked[CONFIG_ARIUM_NR_PORTS];
#endif

static int __init ariumdcc_console_setup(struct console *co, char *options)
{
    return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
static kdev_t ariumdcc_console_device(struct console *c)
{   
    return ARIUM_MK_KDEV(TTY_MAJOR, CONFIG_ARIUMDCC_MINOR + c->index);
}
#else

static struct tty_driver* ariumdcc_console_device(struct console *c, int *index)
{
	*index = 0;
	return &serial_driver;
}
#endif

#define ARIUMDCC_CONSOLE_NAME "ttyDCC"

static struct console ariumdcc_console = {
    .name = ARIUMDCC_CONSOLE_NAME,
    .write = ariumdcc_console_write,
    .flags = CON_PRINTBUFFER,

    .setup = ariumdcc_console_setup,
    .device = ariumdcc_console_device,
    .index = -1,
};
#endif /* CONFIG_ARIUMDCC_CONSOLE */

/*
 * Called whenever there is a state change on a channel. This
 * routine implements any action that should be taken when that
 * happens, like sending a message on the control channel signifying
 * the state change.
 * channel should only occupy the lower 15 bits.
 * state should be wither ARIUMDCC_STATE_OPEN or ARIUMDCC_STATE_CLOSE.
 */
#define ARIUMDCC_CONTROL_CHANNEL (0x7fff) /* the highest channel possible */
#define ARIUMDCC_MSG_STATECHANGE (10)
#define ARIUMDCC_STATE_OPEN      (1)
#define ARIUMDCC_STATE_CLOSE     (2)
static int ariumdcc_channel_state(unsigned int channel, int state)
{
    char msg[5];
    /* State Change messages are defined as follows:
     * [ msg type ] (8 bits)
     * [ unused ] (1 bit)
     * [ channel ] (15 bits)
     * [ new state ] (8 bits)
     */
    msg[0] = ARIUMDCC_MSG_STATECHANGE;
    msg[1] = (channel >> 8) & 0x7f;
    msg[2] = channel & 0xff;
    msg[3] = state & 0xff;
    msg[4] = '\0';
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
    return ariumdcc_write(msg, sizeof(msg), 0 /* not buffered */,
                          ARIUMDCC_CONTROL_CHANNEL, 0 /* not from user */);
#else
    return ariumdcc_write(msg, sizeof(msg), 0 /* not buffered */,
                          ARIUMDCC_CONTROL_CHANNEL);
#endif
}

static int rs_open(struct tty_struct *tty, struct file *filp)
{
    int rv = 0, line;
    struct channel_tree *chan;
    unsigned long flags;

    line = tty->index;
    if ((line < 0) || (line >= CONFIG_ARIUM_NR_PORTS)) {
        rv = -ENODEV;
        goto out;
    }

#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: opening tty channel %d\n", line);
#endif

    chan = kmalloc(sizeof(*chan), GFP_KERNEL);
    if (!chan)
        return -ENOMEM;


    memset(chan, 0, sizeof(*chan));
    chan->channel = line;
    chan->tty = tty;
/*	
	tty->flip.buf_num = 0;
	tty->flip.char_buf_ptr = tty->flip.char_buf;
	tty->flip.flag_buf_ptr = tty->flip.flag_buf;
*/
    spin_lock_irqsave(&chantreelock, flags);
    add_channel(&chantree, chan);
    spin_unlock_irqrestore(&chantreelock, flags);

    spin_lock_irqsave(&chantreelock, flags);
    chan = find_channel(chantree, line);
    spin_unlock_irqrestore(&chantreelock, flags);
    if (chan && chan->tty != tty) {
        /* already open, sorry! */
        printk(KERN_INFO "ariumdcc: channel %d already open\n", line);
        return -EBUSY;
    }

    /* Send message on control channel to signify channel opening */
    (void) ariumdcc_channel_state(line, ARIUMDCC_STATE_OPEN);
out:
    return rv;
}

static void rs_close(struct tty_struct *tty, struct file *filp)
{
    unsigned long flags;
    int line;
    struct channel_tree *chan;

    line = tty->index;
    if ((line < 0) || (line >= CONFIG_ARIUM_NR_PORTS)) {
        /* FIXME: panic? */
        goto out;
    }

#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: closing  channel %d\n", line);
#endif

    spin_lock_irqsave(&chantreelock, flags);
    /* remove ourselves from the flip list */
    chan = remove_channel(&chantree, line);
    spin_unlock_irqrestore(&chantreelock, flags);

    if (!chan)
        printk(KERN_INFO "ariumdcc: channel %d wasn't open\n", line);
    else
	{
        kfree(chan);
		/* Send message on control channel to signify channel closing */
		(void) ariumdcc_channel_state(line, ARIUMDCC_STATE_CLOSE);
	}

out:
    return;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
static int rs_write(struct tty_struct *tty, int from_user,
                    const unsigned char *buf, int count)
#else
static int rs_write(struct tty_struct *tty,
                    const unsigned char *buf, int count)
#endif
{
    int line;

    /* Figure out what "line" we're on and translate it
     * to a channel for DCC. */
    line = tty->index;
    if ((line < 0) || (line >= CONFIG_ARIUM_NR_PORTS)) {
        /* FIXME: panic? */
        return -ENODEV;
    }

    if (in_interrupt()) {
        printk(KERN_WARNING "ariumdcc: calling rs_write from within "
               "an interrupt context, BAD! unable to write\n");
        return -EINVAL;
    }
    else {
        /* write directly to the dcc packet generator */
#ifdef ARIUMDCC_DEBUG
        printk(KERN_INFO "ariumdcc: calling ariumdcc_write from rs_write"
               " and from user == %d\n", from_user);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
        return ariumdcc_write(buf, count, ARIUMDCC_OPTION_BUFFER, line, from_user);
#else
        return ariumdcc_write(buf, count, ARIUMDCC_OPTION_BUFFER, line);
#endif

    }
}

static void rs_put_char(struct tty_struct *tty, unsigned char ch)
{
    int line = tty->index;
    if ((line < 0) || (line >= CONFIG_ARIUM_NR_PORTS)) {
        /* FIXME: panic? */
        goto out;
    }

    if (in_interrupt()) {
        int rv;

        /* Since we're in an interrupt, we can't go to sleep. We call
         * this routine because it doesn't go to sleep, but if we are
         * full then we lose this character. */
        rv = ariumdcc_write_single_char(ch, line);
        if (rv < 0) {
            printk(KERN_WARNING "ariumdcc: failed to write single character "
                   "while in interrupt context! (ch = %x, error = %d)\n",
                   ch, rv);
        }
    }
    else {
#ifdef ARIUMDCC_DEBUG
        printk(KERN_INFO "ariumdcc: calling ariumdcc_write from rs_put_char\n");
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
        (void) ariumdcc_write(&ch, 1, 0, line, 0);
#else
        (void) ariumdcc_write(&ch, 1, 0, line);
#endif
    }
out:
    return;
}

static void rs_flush_chars(struct tty_struct *tty)
{
#ifndef CONFIG_TIMESYS
    if (in_interrupt()) {
#else  /* !CONFIG_TIMESYS */
    if (1) {   /* There is a conflict with softirqd_CPUn and softirqd_lown. */
#endif /* !CONFIG_TIMESYS */
        int retrytimes = MAX_POLL_LOOPS;

        spin_lock(&outringlock);
        while (retrytimes-- > 0) {
            if (is_ring_buffer_empty(&outring))
                break;
            dcc_status = read_dcc_status();
            if (!GET_DCC_STATUS_WRITEBIT(dcc_status))
                do_dcc_write_word();
        }
#ifdef ARIUMDCC_DEBUG
        if (!is_ring_buffer_empty(&outring)) {
            printk(KERN_WARNING "ariumdcc: ECM failed to read %d words from "
                   "the output buffer when trying to flush.\n",
                   ring_buffer_len(&outring));
        }
#endif
        spin_unlock(&outringlock);
    } else {
#ifdef ARIUMDCC_DEBUG
        printk(KERN_INFO "ariumdcc: calling rs_flush_chars\n");
#endif
        while (!is_ring_buffer_empty_ex(&outring, &outring_sem)) {
            wait_event_interruptible(writeq,
                    is_ring_buffer_empty_ex(&outring, &outring_sem));
        }
    }
}

static int rs_write_room(struct tty_struct *tty)
{
    /* We return the worst case, which is when a stream
     * of single-character packets are written to the ring buffer,
     * meaning we only have space for nwords characters. */
    return ring_buffer_space_ex(&outring, &outring_sem);
}

static int rs_chars_in_buffer(struct tty_struct *tty)
{
    /* Assume each word written to the DCC only contains
     * one character of data, to be conservative in our estimate. */
    return ring_buffer_len_ex(&outring, &outring_sem);
}

#if 0
static int rs_ioctl(struct tty_struct *tty, struct file * file,
                    unsigned int cmd, unsigned long arg)
{
}

static void rs_set_termios(struct tty_struct *tty, struct termios * old)
{
}
#endif

static void rs_throttle(struct tty_struct * tty)
{
    /* FIXME: throttle the kernel thread from feeding the flip buffer */
}

static void rs_unthrottle(struct tty_struct * tty)
{
    /* FIXME: unthrottle the kernel thread from feeding the flip buffer */
}

static void rs_stop(struct tty_struct *tty)
{
    /* FIXME: remove ourselves from the flip list */
}

static void rs_start(struct tty_struct *tty)
{
    /* FIXME: add ourselves to the flip list */
}

static void rs_hangup(struct tty_struct *tty)
{
    /* FIXME: not sure what to do here, maybe nothing */
}

#if 0
static void rs_break_ctl(struct tty_struct *tty, int state)
{
}

static void rs_flush_buffer(struct tty_struct *tty)
{
}

static void rs_wait_until_sent(struct tty_struct *tty, int timeout)
{
}

static void rs_send_xchar(struct tty_struct *tty, char ch)
{
}

static int rs_read_proc(char *page, char **start, off_t off,
                     int count, int *eof, void *data)
{
}
#endif

static void do_ariumdcc_initialization(void)
{

    atomic_set(&timer_isr_enabled, 0);

#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: calling disable_timer_isr()\n");
#endif
    disable_timer_isr();
#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: calling reset_dcc()\n");
#endif
    good_reset = reset_dcc(); /* perform the handshake */
#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: sending the handshake response\n");
#endif
    if(good_reset) {
        send_init_response(); /* return the response */
    }
#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: calling initialize_dcc()\n");
#endif
    if(good_reset) {
        initialize_dcc();
    }
#ifdef ARIUMDCC_DEBUG
    printk(KERN_INFO "ariumdcc: calling enable_timer_isr()\n");
#endif
    enable_timer_isr();

#ifdef CONFIG_ARIUMDCC_CONSOLE
    if(good_reset) {
        register_console(&ariumdcc_console);
    }
#endif

}

#ifdef CONFIG_ARIUMDCC_CONSOLE
void __init ariumdcc_console_init(void)
{
    do_ariumdcc_initialization();
}
#endif


int __init ariumdcc_init(void)
{
    int rv;

#ifdef CONFIG_DISABLE_HLT
	// prevent pm sleep mode during idle on xscale pxa27x 
	disable_hlt();
#endif

    // activate arium kernel module debugging hook
    enable_arium_module_hook();

    // create our kernel thread
    ariumdcc_wq = create_workqueue("ariumdcc");

    if (!atomic_read(&timer_isr_enabled)) {
        do_ariumdcc_initialization();
    }

    memset(&serial_driver, 0, sizeof(serial_driver));
    serial_driver.magic = TTY_DRIVER_MAGIC;
    serial_driver.owner = THIS_MODULE;
    serial_driver.driver_name = "ariumdcc";
#ifndef CONFIG_DEVFS_FS
    serial_driver.name = "ttyDCC";
#else
//  BCL  serial_driver.name      = "ttyDCC%d";
    serial_driver.name      = "ttyDCC";
    serial_driver.devfs_name = "ttyDCC";
	serial_driver.name_base = 0;
#endif /* CONFIG_DEVFS_FS */
    serial_driver.major = TTY_MAJOR;

    serial_driver.minor_start = CONFIG_ARIUMDCC_MINOR;
    serial_driver.num = CONFIG_ARIUM_NR_PORTS;
    serial_driver.type = TTY_DRIVER_TYPE_SERIAL;
	  serial_driver.subtype = SERIAL_TYPE_NORMAL;
    serial_driver.init_termios = tty_std_termios;

    serial_driver.init_termios.c_cflag =
        B38400 | CS8 | CREAD | HUPCL | CLOCAL;
//    serial_driver.refcount = &serial_refcount;  // apparently by value in 2.6
    serial_driver.ttys = serial_table;
    serial_driver.termios = serial_termios;
    serial_driver.termios_locked = serial_termios_locked;

    serial_driver.open = rs_open;
    serial_driver.close = rs_close;
    serial_driver.write = rs_write;
    serial_driver.put_char = rs_put_char;
    serial_driver.flush_chars = rs_flush_chars;
    serial_driver.write_room = rs_write_room;
    serial_driver.chars_in_buffer = rs_chars_in_buffer;
    serial_driver.throttle = rs_throttle;
    serial_driver.unthrottle = rs_unthrottle;
    serial_driver.stop = rs_stop;
    serial_driver.start = rs_start;
    serial_driver.hangup = rs_hangup;

    rv = tty_register_driver(&serial_driver);
    if (rv) {
        printk(KERN_WARNING "ariumdcc: failed to register dcc tty driver\n");
        return rv;
    }

    printk(KERN_INFO "%s: version %d.%d for %s - Maj/Min=%d,%d NR_PORTS=%d\n",
           serial_driver.driver_name,
           ARIUMDCC_VERSION,
           ARIUMDCC_MINOR_VERSION,
           serial_driver.name,
           serial_driver.major,
           serial_driver.minor_start,
           serial_driver.num);

    return 0;
}

static void __exit ariumdcc_cleanup(void)
{
    disable_timer_isr();

#ifdef ARIUMDCC_DEBUG
    printk("<1>ariumdcc: unregistered major %d\n", ariumdcc_major);
#endif

    // deactivate arium kernel module debugging hook
    disable_arium_module_hook();

#ifdef CONFIG_DISABLE_HLT
	enable_hlt();
#endif
}

// hook used by SourcePoint to intercept module loads
void arium_module_hook( struct module* mod )
{
  printk(KERN_DEBUG "Arium module debugging hook entered\n");
}

EXPORT_SYMBOL(arium_module_hook);

static int arium_module_notifier(struct notifier_block *self, unsigned long val, void *v)
{
  struct module* mod = (struct module*) v;

#if 0
  struct module_sect_attr* sattr;
	for( sattr = mod->sect_attrs->attrs; sattr->attr.owner == mod; sattr++)
    printk(KERN_DEBUG "ariumdcc: %s %8.8x %s\n", mod->name, sattr->address, sattr->name);
#endif

  arium_module_hook(mod);
  return NOTIFY_DONE;
}

// we use this block to observe the module notifier
static struct notifier_block nb_module = {
	.notifier_call = arium_module_notifier,
	.priority = 10
};

static int enable_arium_module_hook(void)
{
	return register_module_notifier(&nb_module);
}

static int disable_arium_module_hook(void)
{
	return unregister_module_notifier(&nb_module);
}

module_init(ariumdcc_init);
module_exit(ariumdcc_cleanup);

MODULE_AUTHOR("American Arium");
MODULE_DESCRIPTION("ARM9/XScale DCC/DBG register support");
MODULE_SUPPORTED_DEVICE("/dev/ttyDCC");
MODULE_LICENSE("GPL");
