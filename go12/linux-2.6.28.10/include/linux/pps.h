/*
 * PPS API kernel header
 *
 *
 * Copyright (C) 2005-2009   Rodolfo Giometti <giometti@linux.it>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#ifndef _PPS_H_
#define _PPS_H_

#define PPS_VERSION		"5.3.5"
#define PPS_MAX_SOURCES		16		/* should be enough... */

/* Implementation note: the logical states ``assert'' and ``clear''
 * are implemented in terms of the chip register, i.e. ``assert''
 * means the bit is set.  */

/*
 * 3.2 New data structures
 */

#define PPS_API_VERS_1		1
#define PPS_API_VERS		PPS_API_VERS_1	/* we use API version 1 */
#define PPS_MAX_NAME_LEN	32

/* 32-bit vs. 64-bit compatibility.
 *
 * 0n i386, the alignment of a uint64_t is only 4 bytes, while on most other
 * architectures it's 8 bytes. On i386, there will be no padding between the
 * two consecutive 'struct pps_ktime' members of struct pps_kinfo and struct
 * pps_kparams. But on most platforms there will be padding to ensure correct
 * alignment.
 *
 * The simple fix is probably to add an explicit padding.
 *					 		[David Woodhouse]
 */
struct pps_ktime {
	__s64 sec;
	__s32 nsec;
	__u32 flags;
};
#define PPS_TIME_INVALID	(1<<0)	/* used to specify timeout==NULL */

struct pps_kinfo {
	__u32 assert_sequence;		/* seq. num. of assert event */
	__u32 clear_sequence; 		/* seq. num. of clear event */
	struct pps_ktime assert_tu;	/* time of assert event */
	struct pps_ktime clear_tu;	/* time of clear event */
	int current_mode;		/* current mode bits */
};

struct pps_kparams {
	int api_version;		/* API version # */
	int mode;			/* mode bits */
	struct pps_ktime assert_off_tu;	/* offset compensation for assert */
	struct pps_ktime clear_off_tu;	/* offset compensation for clear */
};

/*
 * 3.3 Mode bit definitions
 */

/* Device/implementation parameters */
#define PPS_CAPTUREASSERT	0x01	/* capture assert events */
#define PPS_CAPTURECLEAR	0x02	/* capture clear events */
#define PPS_CAPTUREBOTH		0x03	/* capture assert and clear events */

#define PPS_OFFSETASSERT	0x10	/* apply compensation for assert ev. */
#define PPS_OFFSETCLEAR		0x20	/* apply compensation for clear ev. */

#define PPS_CANWAIT		0x100	/* can we wait for an event? */
#define PPS_CANPOLL		0x200	/* bit reserved for future use */

/* Kernel actions */
#define PPS_ECHOASSERT		0x40	/* feed back assert event to output */
#define PPS_ECHOCLEAR		0x80	/* feed back clear event to output */

/* Timestamp formats */
#define PPS_TSFMT_TSPEC		0x1000	/* select timespec format */
#define PPS_TSFMT_NTPFP		0x2000	/* select NTP format */

/*
 * 3.4.4 New functions: disciplining the kernel timebase
 */

/* Kernel consumers */
#define PPS_KC_HARDPPS		0	/* hardpps() (or equivalent) */
#define PPS_KC_HARDPPS_PLL	1	/* hardpps() constrained to
					   use a phase-locked loop */
#define PPS_KC_HARDPPS_FLL	2	/* hardpps() constrained to
					   use a frequency-locked loop */
/*
 * Here begins the implementation-specific part!
 */

struct pps_fdata {
	struct pps_kinfo info;
	struct pps_ktime timeout;
};

#include <linux/ioctl.h>

#define PPS_GETPARAMS		_IOR('p', 0xa1, struct pps_kparams *)
#define PPS_SETPARAMS		_IOW('p', 0xa2, struct pps_kparams *)
#define PPS_GETCAP		_IOR('p', 0xa3, int *)
#define PPS_FETCH		_IOWR('p', 0xa4, struct pps_fdata *)

#ifdef __KERNEL__

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/time.h>

/*
 * Global defines
 */

/* The specific PPS source info */
struct pps_source_info {
	char name[PPS_MAX_NAME_LEN];		/* simbolic name */
	char path[PPS_MAX_NAME_LEN];		/* path of connected device */
	int mode;				/* PPS's allowed mode */

	void (*echo)(int source, int event, void *data); /* PPS echo function */

	struct module *owner;
	struct device *dev;
};

/* The main struct */
struct pps_device {
	struct pps_source_info info;		/* PSS source info */

	struct pps_kparams params;		/* PPS's current params */

	__u32 assert_sequence;			/* PPS' assert event seq # */
	__u32 clear_sequence;			/* PPS' clear event seq # */
	struct pps_ktime assert_tu;
	struct pps_ktime clear_tu;
	int current_mode;			/* PPS mode at event time */

	int go;					/* PPS event is arrived? */
	wait_queue_head_t queue;		/* PPS event queue */

	unsigned int id;			/* PPS source unique ID */
	struct cdev cdev;
	struct device *dev;
	int devno;
	struct fasync_struct *async_queue;	/* fasync method */
	spinlock_t lock;

	atomic_t usage;				/* usage count */
};

/*
 * Global variables
 */

extern spinlock_t pps_idr_lock;
extern struct idr pps_idr;
extern struct timespec pps_irq_ts[];

extern struct device_attribute pps_attrs[];

/*
 * Exported functions
 */

struct pps_device *pps_get_source(int source);
extern void pps_put_source(struct pps_device *pps);
extern int pps_register_source(struct pps_source_info *info,
				int default_params);
extern void pps_unregister_source(int source);
extern int pps_register_cdev(struct pps_device *pps);
extern void pps_unregister_cdev(struct pps_device *pps);
extern void pps_event(int source, struct pps_ktime *ts, int event, void *data);

#endif /* __KERNEL__ */

#endif /* _PPS_H_ */
