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





/*
 * Kernel driver support for msgio ioctls.
 *
 * These functions can sleep, so the code should be re-enterable in order
 * to support multiple simultaneous clients.
 */


// XXX How much of this do we really need?
#include <stdarg.h>

#if defined( __KERNEL__ )

#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/kthread.h>
#include <linux/broadcom/vc.h>
#include <asm/uaccess.h>
#include <linux/mm.h>		// for kmalloc() and friends
#include <linux/vmalloc.h>	// for vmalloc() and friends
#include <linux/jiffies.h>
#else

#include <stdio.h>

#endif

#include <linux/broadcom/omx/omx.h>
#include "vchost.h"
#include "vchost_int.h"		// for Event_t
#include "vcinterface.h"
#include "vciface.h"
#include "vcmsgfifo.h"
// XXX How much of this do we really need?


#undef TRACE_ENABLED

#ifdef TRACE_ENABLED
extern int     gVcDebugTrace;
#endif // TRACE_ENABLED



// XXX WARNING: only one process at a time can use an interface.
Event_t *events[VC_NUM_INTERFACES];

// Initialization


/*
 * Initialize a service, based on its inum.
 */
int vc_msgio_init (int inum)
{
	// Allocate an event struct for this interface.
	if (events[inum] == NULL)
	{
		events[inum] = vc_event_create();
	}
	vc_interface_register_event_int(events[inum], (1<<inum));

	// Current implementation always returns 0.
	vc_msgfifo_init(inum);

	return 0;
}


/*
 * Search for a VideoCore service by service ID, and if found, initialize it
 * and return its inum.
 */
int vc_init_service(int serviceID)
{
	int i;
	VC_GENERIC_INTERFACE_T generic_interface;

	// Loop through every interface that there is and look for one that
	// matches the requested service ID.
	for (i = 0; i < VC_NUM_INTERFACES; i++) {
		if (vc_sharedmem_header.iface[i]) {
			uint16_t stype;
			vc_host_read_consecutive(&generic_interface,
				vc_interface_base + vc_sharedmem_header.iface[i],
				sizeof(VC_GENERIC_INTERFACE_T), 0);
			stype = VC_VTOH16(generic_interface.stype);
			if ((int)stype == serviceID) {
				vc_msgio_init(i);
				return i;
			}
		}
	}
	return -1;
}



// Write / Output functions


// Don't know what the limit is, but 65536 is definitely too big.
#define MAX_COPY_FROM_USER 4096

/*
 * Called from vc_drv.c: vc_ioctl() to process VC_IOCTL_GFX_WRITE_BLOCKING.
 */
int vc_msgio_write_blocking(msgio_write_blocking_t* arg)
{
#ifdef TRACE_ENABLED
	int save_trace = gVcDebugTrace;
#endif // TRACE_ENABLED
	msgio_write_blocking_t wr_desc;
	void* data;
	void* host_addr;
	unsigned int bufsize;

//	gVcDebugTrace = 1;
	// Driver may sleep here while faulting in the ioctl arg block.
	if (copy_from_user(&wr_desc, (void *)arg, sizeof(wr_desc)) != 0)
	{
#ifdef TRACE_ENABLED
		VC_DEBUG(Trace,
			"ioctl failed to access msgio write desc, at addr=0x%08x\n",
									(int)arg);
		gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
		return -EFAULT;
	}

	bufsize = wr_desc.nbytes;
	if (bufsize > MAX_COPY_FROM_USER)
		bufsize = MAX_COPY_FROM_USER;

	data = kmalloc(bufsize, GFP_KERNEL);
	if (data == NULL)
	{
#ifdef TRACE_ENABLED
		VC_DEBUG(Trace,
			"ioctl unable to allocate vc_msgfifo buffer, size=%d\n",
									wr_desc.nbytes);
		gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
		return -EFAULT;
	}

	host_addr = wr_desc.host_addr;

	while(wr_desc.nbytes > 0)
	{
#if 0
		printk("write_blocking copy_from_user addr 0x%08x to knl buffer 0x&08x nb %d\n",
			host_addr, (unsigned int)data, bufsize);
#endif
		// Driver may sleep here while faulting in the user pages.
		if (copy_from_user(data, host_addr, bufsize) != 0)
		{
#ifdef TRACE_ENABLED
			VC_DEBUG(Trace,
			"ioctl failed to copy from user buffer at 0x%08x\n", (int)arg);
			gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
			kfree(data);
			return -EFAULT;
		}

#ifdef TRACE_ENABLED
		VC_DEBUG(Trace,
			"write_blocking: inum %d  addr 0x%08x  size %d\n",
				wr_desc.inum, (unsigned int)data, bufsize);
#endif // TRACE_ENABLED
		// Driver may sleep here while feeding the fifo.
		vc_msgfifo_write_blocking(wr_desc.inum, data, bufsize,
		       				events[wr_desc.inum]);
		wr_desc.nbytes -= bufsize;
		host_addr += bufsize;

		// Last block: send only the bytes remaining.
		if (wr_desc.nbytes < bufsize)
       			bufsize = wr_desc.nbytes;
	}

#if 0
	// wr_desc.event access already checked by get_user() above.
	__put_user(event, wr_desc.event);
#endif

	kfree(data);
#ifdef TRACE_ENABLED
	gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
	return 0;
}


/*
 * Called from vc_drv.c: vc_ioctl() to process VC_IOCTL_GFX_SEND_CMD_BLOCKING.
 */
int vc_msgio_send_command_blocking(msgio_send_cmd_blocking_t* arg)
{
#ifdef TRACE_ENABLED
	int save_trace = gVcDebugTrace;
#endif // TRACE_ENABLED
	msgio_send_cmd_blocking_t cmd_desc;
	void* data = NULL;

#ifdef TRACE_ENABLED
//	gVcDebugTrace = 1;
#endif // TRACE_ENABLED

	// Driver may sleep here while faulting in the ioctl arg block.
	if (copy_from_user(&cmd_desc, (void *)arg, sizeof(cmd_desc)) != 0)
	{
#ifdef TRACE_ENABLED
		VC_DEBUG(Trace,
			"ioctl failed to access send command desc, at addr=0x%08x\n",
									(int)arg);
#endif // TRACE_ENABLED
		return -EFAULT;
	}

#if 0
	// cmd_desc.event is user space address of where to post the event completion.
	// Fetch the current value.
	if (0 != get_user(event, cmd_desc.event))
	{
		VC_DEBUG(Trace,
			"ioctl failed to access cmd_desc.event, at addr=0x%08x\n",
							(int)cmd_desc.event);
		return -EFAULT;
	}
#endif

	if ((cmd_desc.ext_length > 0) && (cmd_desc.data != NULL))
	{
		// Using vmalloc here, rather than fiddling with the maximum allocation size.
		data = vmalloc(cmd_desc.ext_length);
		if (data == NULL)
		{
#ifdef TRACE_ENABLED
			VC_DEBUG(Trace,
				"ioctl unable to allocate vc_msgfifo buffer, size=%d\n",
									cmd_desc.ext_length);
#endif // TRACE_ENABLED
			return -EFAULT;
		}

#if 0
		printk("send_command_blocking copy_from_user addr 0x%08x nb %d\n",
			cmd_desc.data, cmd_desc.ext_length);
#endif
		// Driver may sleep here while faulting in the user pages.
		if (copy_from_user(data, cmd_desc.data, cmd_desc.ext_length) != 0)
		{
#ifdef TRACE_ENABLED
			VC_DEBUG(Trace,
				"ioctl failed to copy from user buffer at 0x%08x\n", (int)arg);
#endif // TRACE_ENABLED
			vfree(data);
			return -EFAULT;
		}
	}

#ifdef TRACE_ENABLED
	VC_DEBUG(Trace,
		"send_command_blocking: inum %d  cmd %d  size %d  addr 0x%08x\n",
		cmd_desc.inum, cmd_desc.cmd_code, cmd_desc.ext_length, (unsigned int)data);
#endif // TRACE_ENABLED
	// Driver may sleep here while feeding the fifo.
	vc_msgfifo_send_command_blocking(cmd_desc.inum, cmd_desc.cmd_code,
				cmd_desc.ext_length, data, events[cmd_desc.inum]);

#if 0 // XXX NO! we don't try to sync event flags in user space! XXX
	// cmd_desc.event access already checked by get_user() above.
	__put_user(event, cmd_desc.event);
#endif

	vfree(data);
#ifdef TRACE_ENABLED
	gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
	return 0;
}


/*
 * Called from vc_drv.c: vc_ioctl() to process VC_IOCTL_GFX_WRITE_FLUSH.
 */
int vc_msgio_write_flush(int inum)
{
#ifdef TRACE_ENABLED
	int save_trace = gVcDebugTrace;
//	gVcDebugTrace = 1;
	VC_DEBUG(Trace, "write_flush: inum %d\n", inum);
#endif // TRACE_ENABLED
	// Driver may sleep here while flushing the fifo.
	// Current implementation always returns 0.
	vc_msgfifo_write_flush(inum);
#ifdef TRACE_ENABLED
	gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
	return 0;
}



// Read / Input functions

/*
 * Called from vc_drv.c: vc_ioctl() to process VC_IOCTL_GFX_INPUT_BYTES_AVAIL.
 */
int vc_msgio_input_bytes_available(int inum)
{
	// Returns non-negative count of bytes available.
	// It should not be necessary to check validity here. 
	return vc_msgfifo_input_bytes_available(inum);
}


/*
 * Called from vc_drv.c: vc_ioctl() to process VC_IOCTL_GFX_READ_BLOCKING.
 */
int vc_msgio_read_blocking(msgio_read_blocking_t* arg)
{
#ifdef TRACE_ENABLED
	int save_trace = gVcDebugTrace;
#endif // TRACE_ENABLED
	msgio_read_blocking_t rd_desc;
	unsigned int* data;

#ifdef TRACE_ENABLED
//	gVcDebugTrace = 1;
#endif // TRACE_ENABLED

#if 0
	printk("vc_msgio_read_blocking copy args from user\n");
#endif

	// Driver may sleep here while faulting in the ioctl arg block.
	if (copy_from_user(&rd_desc, (void *)arg, sizeof(rd_desc)) != 0)
	{
#ifdef TRACE_ENABLED
		VC_DEBUG(Trace,
			"ioctl failed to access msgio read desc, at addr=0x%08x\n",
									(int)arg);
#endif // TRACE_ENABLED
		return -EFAULT;
	}

	// Using vmalloc here, rather than fiddling with the maximum allocation size.
	// Round up input buffer size to next 16 byte boundary.  See XXX below.
	data = vmalloc(sizeof((rd_desc.nbytes + 0xf) & ~0xf));
	if (data == NULL)
	{
#ifdef TRACE_ENABLED
		VC_DEBUG(Trace,
			"ioctl unable to allocate vc_msgfifo buffer, size=%d\n",
									rd_desc.nbytes);
#endif // TRACE_ENABLED
#ifdef TRACE_ENABLED
		gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
		return -EFAULT;
	}

#ifdef TRACE_ENABLED
	VC_DEBUG(Trace,
		"read_blocking: inum %d  addr 0x%08x  size %d\n",
			rd_desc.inum, (unsigned int)data, rd_desc.nbytes);
#endif // TRACE_ENABLED

//	vc_host_delay_msec(200);		// XXX Hack

	// Driver may sleep here while reading the fifo.
	// Round up transfer size to next 16 byte boundary, because VC3 transfers multiples of 16 bytes.
	vc_msgfifo_read_blocking(rd_desc.inum, data, (rd_desc.nbytes + 0xf) & ~0xf,
							events[rd_desc.inum]);
#if 0
	printk("vc_msgio_read_blocking copy results to user\n");
	printk(" %08x %08x %08x %08x\n", data[0], data[1], data[2], data[3]);
#endif

	// Driver may sleep here while faulting in the user pages.
	if (copy_to_user(rd_desc.host_addr, data, rd_desc.nbytes) != 0)
	{
#ifdef TRACE_ENABLED
		VC_DEBUG(Trace,
			"ioctl failed to copy to user buffer at 0x%08X\n", (int)arg);
		gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
		vfree(data);
		return -EFAULT;
	}

#if 0
	// rd_desc.event access already checked by get_user() above.
	__put_user(event, rd_desc.event);
#endif

	vfree(data);

#if 0
	printk("vc_msgio_read_blocking returning\n");
#endif

#ifdef TRACE_ENABLED
	gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
	return 0;
}


/*
 * Called from vc_drv.c: vc_ioctl() to process VC_IOCTL_GFX_READ_HEADER.
 */
int vc_msgio_read_header(msgio_read_header_t* arg)
{
#ifdef TRACE_ENABLED
	int save_trace = gVcDebugTrace;
#endif // TRACE_ENABLED
	msgio_read_header_t rh_desc;
	void* data;

#ifdef TRACE_ENABLED
//	gVcDebugTrace = 1;
#endif // TRACE_ENABLED

	// Driver may sleep here while faulting in the ioctl arg block.
	if (copy_from_user(&rh_desc, (void *)arg, sizeof(rh_desc)) != 0)
	{
#ifdef TRACE_ENABLED
		VC_DEBUG(Trace,
			"ioctl failed to access msgio read desc, at addr=0x%08x\n",
									(int)arg);
		gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
		return -EFAULT;
	}

	// Using kmalloc here, because the size is small and fixed.
	data = kmalloc(sizeof(VC_MSGFIFO_CMD_HEADER_T), GFP_KERNEL);
	if (data == NULL)
	{
#ifdef TRACE_ENABLED
//		gVcDebugTrace = 1;
		VC_DEBUG(Trace,
			"ioctl unable to allocate vc_msgfifo buffer, size=%d\n",
							sizeof(VC_MSGFIFO_CMD_HEADER_T));
		gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
		return -EFAULT;
	}

#ifdef TRACE_ENABLED
	VC_DEBUG(Trace,
		"read_header: inum %d  addr 0x%08x\n",
			rh_desc.inum, (unsigned int)data);
#endif // TRACE_ENABLED
	// Driver may sleep here while reading the fifo.
	vc_msgfifo_read_header(rh_desc.inum, data, events[rh_desc.inum]);

	// Driver may sleep here while faulting in the user pages.
	if (copy_to_user(rh_desc.header, data, sizeof(VC_MSGFIFO_CMD_HEADER_T)) != 0)
	{
#ifdef TRACE_ENABLED
//		gVcDebugTrace = 1;
		VC_DEBUG( Trace,
			"ioctl failed to copy to user buffer at 0x%08x\n", (int)arg);
		gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
		kfree(data);
		return -EFAULT;
	}

#if 0
	// rd_desc.event access already checked by get_user() above.
	__put_user(event, rh_desc.event);
#endif

	kfree(data);
#ifdef TRACE_ENABLED
	gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
	return 0;
}


/*
 * Called from vc_drv.c: vc_ioctl() to process VC_IOCTL_GFX_READ_FLUSH.
 */
int vc_msgio_read_flush(int inum)
{
#ifdef TRACE_ENABLED
	int save_trace = gVcDebugTrace;

//	gVcDebugTrace = 1;

	VC_DEBUG(Trace, "read_flush: inum %d\n", inum);
#endif // TRACE_ENABLED
	// Driver may sleep here while flushing the fifo.
	// Current implementation always returns 0.
	vc_msgfifo_read_flush(inum);
#ifdef TRACE_ENABLED
	gVcDebugTrace = save_trace;
#endif // TRACE_ENABLED
	return 0;
}
