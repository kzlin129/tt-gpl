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




// Linux user space driver for accessing the VideoCore FIFO.
//
// Provides a vcmsgfifo style interface by issuing ioctls to the kernel driver.
//
// Suitable for supporting OpenGL ES 2.0, OpenVG, and EGL.



#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>	// for open flags
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/broadcom/vc.h>
//#include "vc.h"
//
#define REQUIRE_VCOGLES	// needed to make vcmsgfifo.h define vc_msgfifo_read_header()
#include "vcmsgfifo.h"


#define DEVPATH  "/dev/vc03"


#define PRINT_INFO
#undef PRINT_DEBUG

#ifdef PRINT_DEBUG
#define MSGIO_DEBUG(a) fprintf(stderr, (a))
#define MSGIO_DEBUG2(a, b) fprintf(stderr, (a), (b))
#define MSGIO_DEBUG3(a, b, c) fprintf(stderr, (a), (b), (c))
#define MSGIO_DEBUG4(a, b, c, d) fprintf(stderr, (a), (b), (c), (d))
#define MSGIO_DEBUG7(a, b, c, d, e, f, g) fprintf(stderr, (a), (b), (c), (d), (e), (f), (g))
#else
#define MSGIO_DEBUG(a)
#define MSGIO_DEBUG2(a, b)
#define MSGIO_DEBUG3(a, b, c)
#define MSGIO_DEBUG4(a, b, c, d)
#define MSGIO_DEBUG7(a, b, c, d, e, f, g)
#endif

#ifdef PRINT_INFO
#define MSGIO_INFO(a) fprintf(stderr, (a))
#define MSGIO_INFO2(a, b) fprintf(stderr, (a), (b))
#define MSGIO_INFO3(a, b, c) fprintf(stderr, (a), (b), (c))
#define MSGIO_INFO4(a, b, c, d) fprintf(stderr, (a), (b), (c), (d))
#else
#define MSGIO_INFO(a)
#define MSGIO_INFO2(a, b)
#define MSGIO_INFO3(a, b, c)
#define MSGIO_INFO4(a, b, c, d)
#endif

// Static data: one instance per process.
// This is OK because OpenGL supports only one rendering thread per process.

typedef struct
{
	int fd;
	int openfailed;

} msgio_state_t;

static msgio_state_t msgio_state = 
{
	-1,
	0,
};



// Initialization / Device access functions

/*
 * Initialize userland library and device support for OpenGL ES, OpenVG, and EGL.
 */
static int msgio_init(void)
{
	if (msgio_state.fd != -1)
	{
		MSGIO_DEBUG("Error: msgio already initialized\n");
		return -1;
	}
  
	msgio_state.fd = open(DEVPATH, O_RDWR);

	if (msgio_state.fd < 0)
	{
		MSGIO_INFO2("Error: msgio unable to open %s\n", DEVPATH);
		msgio_state.fd = -1;
		return -1;
	}

	MSGIO_DEBUG2("Opened file %d for vc msg i/o\n", msgio_state.fd);
	return 0;
}


/*
 * Uninitialize userland library and device support for OpenGL ES, OpenVG, and EGL.
 */
#if 0	// XXX currently unused XXX
static int msgio_uninit(void)
{
	if (msgio_state.fd != -1)
	{	  
		close(msgio_state.fd);
		msgio_state.fd = -1;
		msgio_state.openfailed = 0;
	}

	return 0;
}
#endif // XXX currently unused XXX


/*
 * Check if userland library and device support for OpenGL ES, OpenVG, and EGL
 * is already initialized.
 */
static int msgio_initialized(void)
{
	return (msgio_state.fd != -1);
}




// Initialization


/* Initialise a msgfifo interface for use. Non-zero return indicates failure. */

int vc_msgfifo_init (int inum)
{
	int ret;

	// If inum == -1, just get a file descriptor for future IOCTLs.
	if (inum == -1)
	{
		msgio_init();
		return 0;
	}

	if (-1 == msgio_state.fd)
	{
		msgio_init();
	}

	if ((ret = ioctl(msgio_state.fd, VC_IOCTL_MSGIO_INIT, inum)) != 0 )
	{
		MSGIO_INFO3("Error: msgio init failed for inum %d, returned %d\n", inum, ret);
		return -1;
	}
	MSGIO_DEBUG("msgio init succeeded\n");
	return 0;
}


/* Find a service by service ID, and initialize it for use with the msgfifo driver. */

int vc_service_init(int serviceID)
{
	int ret;

	if (-1 == msgio_state.fd)
	{
		msgio_init();
	}

	if ((ret = ioctl(msgio_state.fd, VC_IOCTL_INIT_SERVICE, serviceID)) < 0 )
	{
		MSGIO_INFO3("Error: vc_service_init init failed for service %d, returned %d\n", serviceID, ret);
		return -1;
	}
	MSGIO_DEBUG("msgio init succeeded\n");
	return ret;
}


// Write / Output functions

/*
 * Issue a system call to cause the kernel VC driver to write nbytes to the fifo.
 */
void vc_msgfifo_write_blocking (int inum, void *host_addr, int nbytes, void *event)
{
	msgio_write_blocking_t wr_desc;
	int ret;

	if (!msgio_initialized())
	{
		// Try to open the device; give up if it fails to open.
		if (msgio_state.openfailed)
		{
			MSGIO_DEBUG("Error: failed to open msgio device.\n");
			return;
		}

		MSGIO_DEBUG("Warning: writing fifo when msgio not initialized\n");
		if (0 != msgio_init())
		{
			msgio_state.openfailed = 1;
			return;
		}
	}
	
	wr_desc.inum = inum;
	wr_desc.host_addr = host_addr;
	wr_desc.nbytes = nbytes;
	wr_desc.event = event;

	MSGIO_DEBUG4("msgio write blocking ioctl issued, inum %d  addr 0x%08x  nb %d\n",
		       	inum, (unsigned int)host_addr, nbytes);
	if ((ret = ioctl(msgio_state.fd, VC_IOCTL_MSGIO_WRITE_BLOCKING, &wr_desc)) != 0 )
	{
		MSGIO_INFO2("Error: msgio write ioctl blocking failed, returned %d\n", ret);
		return;
	}
	MSGIO_DEBUG("msgio write blocking ioctl succeeded\n");
}


/*
 * Issue a system call to cause the kernel VC driver to send a command to the fifo.
 * vcmsgfifo.h defines this as returning void, so we can't return any error info.
 */

void vc_msgfifo_send_command_blocking (int inum, int cmd_code, int ext_length, void *data, void *event)
{
	msgio_send_cmd_blocking_t cmd_desc;
	int ret;

	if (!msgio_initialized())
	{
		// Try to open the device; give up if it fails to open.
		if (msgio_state.openfailed)
		{
			MSGIO_DEBUG("Error: failed to open msgio device.\n");
			return;
		}

		MSGIO_DEBUG("Warning: sending msgio command when VC not initialized\n");
		if (0 != msgio_init())
		{
			msgio_state.openfailed = 1;
			return;
		}
	}
	
	cmd_desc.inum = inum;
	cmd_desc.cmd_code = cmd_code;
	cmd_desc.ext_length = ext_length;
	cmd_desc.data = data;
	cmd_desc.event = event;

MSGIO_DEBUG7("snd cmd blocking: fd %d  inum %d  cmdcode %d  length %d data 0x%x  event 0x%x\n",
		msgio_state.fd, inum, cmd_code, ext_length, (unsigned int)data, event);
	if ((ret = ioctl(msgio_state.fd, VC_IOCTL_MSGIO_SEND_CMD_BLOCKING, &cmd_desc)) != 0 )
	{
		MSGIO_INFO2("Error: msgio send command blocking failed, returned %d\n", ret);
		return;
	}
	MSGIO_DEBUG("msgio send command blocking ioctl succeeded\n");
	return;
}


/*
 * Issue a system call to cause the kernel VC driver to flush the write fifo.
 */
int vc_msgfifo_write_flush(int inum)
{
	int ret;

	if (!msgio_initialized())
	{
		return 0;
	}

	if ((ret = ioctl(msgio_state.fd, VC_IOCTL_MSGIO_WRITE_FLUSH, inum)) != 0 )
	{
		MSGIO_INFO2("Error: msgio write flush failed, returned %d\n", ret);
		return -1;
	}

	MSGIO_DEBUG("msgio write flush ioctl succeeded\n");
	return 0;
}



// Read / Input functions

/*
 * Issue a system call to cause the kernel VC driver to return the number
 * of bytes of input available from the fifo.
 */
int vc_msgfifo_input_bytes_available (int inum)
{
	if (!msgio_initialized())
	{
		return 0;
	}

	// ioctl returns non-negative count of bytes available.
	return ioctl(msgio_state.fd, VC_IOCTL_MSGIO_INPUT_BYTES_AVAIL, inum);
}


/*
 * Issue a system call to cause the kernel VC driver to read the next
 * block of bytes from the input fifo as a msg header.
 */
void vc_msgfifo_read_header(int inum, VC_MSGFIFO_CMD_HEADER_T * header, void *event)
{
	msgio_read_header_t rh_desc;
	int ret;

	if (!msgio_initialized())
	{
		// Try to open the device; give up if it fails to open.
		if (msgio_state.openfailed)
		{
			MSGIO_DEBUG("Error: failed to open VC device.\n");
			return;
		}

		MSGIO_DEBUG("Warning: reading header when VC not initialized\n");
		if (0 != msgio_init())
		{
			msgio_state.openfailed = 1;
			return;
		}
	}
	
	rh_desc.inum = inum;
	rh_desc.header = header;
	rh_desc.event = event;

	if ((ret = ioctl(msgio_state.fd, VC_IOCTL_MSGIO_READ_HEADER, &rh_desc)) != 0 )
	{
		MSGIO_INFO2("Error: msgio read header ioctl failed, returned %d\n", ret);
		return;
	}
	MSGIO_DEBUG("msgio read header ioctl succeeded\n");
}


/*
 * Issue a system call to cause the kernel VC driver to read nbytes from the fifo.
 */
void vc_msgfifo_read_blocking (int inum, void *host_addr, int nbytes, void *event)
{
	msgio_read_blocking_t rd_desc;
	int ret;

	if (!msgio_initialized())
	{
		// Try to open the device; give up if it fails to open.
		if (msgio_state.openfailed)
		{
			MSGIO_INFO("Error: failed to open VC device.\n");
			return;
		}

		MSGIO_INFO("Warning: reading fifo when VC not initialized\n");
		if (0 != msgio_init())
		{
			msgio_state.openfailed = 1;
			return;
		}
	}
	
	rd_desc.inum = inum;
	rd_desc.host_addr = host_addr;
	rd_desc.nbytes = nbytes;
	rd_desc.event = event;

	MSGIO_DEBUG4("msgio read blocking ioctl issued, inum %d  addr 0x%08x  nb %d\n",
		rd_desc.inum, (unsigned int)rd_desc.host_addr, rd_desc.nbytes);

	if ((ret = ioctl(msgio_state.fd, VC_IOCTL_MSGIO_READ_BLOCKING, &rd_desc)) != 0 )
	{
		MSGIO_INFO2("Error: msgio read blocking ioctl failed, returned %d\n", ret);
		return;
	}
	MSGIO_DEBUG("msgio read blocking ioctl succeeded\n");
}


/*
 * Issue a system call to cause the kernel VC driver to flush the read fifo.
 */
int vc_msgfifo_read_flush (int inum)
{
	int ret;

	if (!msgio_initialized())
	{
		return 0;
	}

	if ((ret = ioctl(msgio_state.fd, VC_IOCTL_MSGIO_READ_FLUSH, inum)) != 0 )
	{
		MSGIO_INFO2("Error: msgio read flush failed, returned %d\n", ret);
		return -1;
	}

	MSGIO_DEBUG("msgio read flush ioctl succeeded\n");
	return 0;
}


#if 1
/////////////// DUMMIES FOR ALL THE OTHER MESSAGE FIFO ROUTINES XXXXXXXXXXXXXXX


/* Return the number of bytes present and unread by the VideoCore in our output fifo. */

int vc_msgfifo_output_bytes_available(int inum)
{
	MSGIO_INFO("msgio output bytes available not implemented\n");
	return -1;
}

/* Return the number of bytes space available in our output fifo. */

int vc_msgfifo_output_space_available(int inum)
{
	MSGIO_INFO("msgio output space available not implemented\n");
	return -1;
}

/* Reload our input fifo pointer that VideoCore sets (vout_fwptr). */

int vc_msgfifo_read_refresh(int inum)
{
	MSGIO_INFO("msgio read refresh not implemented\n");
	return -1;
}

/* Reload our output fifo pointer that VideoCore sets (vin_frptr). */

int vc_msgfifo_write_refresh(int inum)
{
	MSGIO_INFO("msgio write refresh not implemented\n");
	return -1;
}

/* Read bytes from our input fifo. Returns the number read (which may be zero). */

int vc_msgfifo_read_consecutive(int inum, void *host_addr, int nbytes)
{
	MSGIO_INFO("msgio read consecutive not implemented\n");
	return -1;
}

/* Write bytes to our output fifo. Returns the number written (which may be zero). */

int vc_msgfifo_write_consecutive(int inum, void *host_addr, int nbytes)
{
	MSGIO_INFO("msgio write consecutive not implemented\n");
	return -1;
}

/* Send a command and ext_length bytes of parameters to the fifo. If insufficient
   space in the fifo, writes nothing at all, and returns VC_MSGFIFO_FULL. USE NOW DEPRECATED. */

int vc_msgfifo_send_command(int inum, int cmd_code, int ext_length, void *data)
{
	MSGIO_INFO("msgio send command not implemented\n");
	return -1;
}

#endif
