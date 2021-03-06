/*
 * pps-ldisc.c -- PPS line discipline
 *
 *
 * Copyright (C) 2008	Rodolfo Giometti <giometti@linux.it>
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

#include <linux/module.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/pps.h>

#define PPS_TTY_MAGIC		0x0001

static void pps_tty_dcd_change(struct tty_struct *tty, unsigned int status,
				struct timespec *ts)
{
	long id = (long) tty->disc_data;
	struct timespec __ts;
	struct pps_ktime pps_ts;

	/* First of all we get the time stamp... */
	getnstimeofday(&__ts);

	/* Does caller give us a timestamp? */
	if (ts) {	/* Yes. Let's use it! */
		pps_ts.sec = ts->tv_sec;
		pps_ts.nsec = ts->tv_nsec;
	} else {	/* No. Do it ourself! */
		pps_ts.sec = __ts.tv_sec;
		pps_ts.nsec = __ts.tv_nsec;
	}

	/* Now do the PPS event report */
	pps_event(id, &pps_ts, status ? PPS_CAPTUREASSERT : PPS_CAPTURECLEAR,
			NULL);

	pr_debug("PPS %s at %lu on source #%d\n",
			status ? "assert" : "clear", jiffies, (int) id);
}

static int pps_tty_open(struct tty_struct *tty)
{
	struct pps_source_info info;
	struct tty_driver *drv = tty->driver;
	int index = tty->index + drv->name_base;
	long ret;

	info.owner = THIS_MODULE;
	info.dev = NULL;
	snprintf(info.name, PPS_MAX_NAME_LEN, "%s%d", drv->driver_name, index);
	snprintf(info.path, PPS_MAX_NAME_LEN, "/dev/%s%d", drv->name, index);
	info.mode = PPS_CAPTUREBOTH | \
			PPS_OFFSETASSERT | PPS_OFFSETCLEAR | \
			PPS_CANWAIT | PPS_TSFMT_TSPEC;

	ret = pps_register_source(&info, PPS_CAPTUREBOTH | \
				PPS_OFFSETASSERT | PPS_OFFSETCLEAR);
	if (ret < 0) {
		pr_err("cannot register PPS source \"%s\"\n", info.path);
		return ret;
	}
	tty->disc_data = (void *) ret;

	/* Should open N_TTY ldisc too */
	ret = n_tty_open(tty);
	if (ret < 0)
		pps_unregister_source((long) tty->disc_data);

	pr_info("PPS source #%d \"%s\" added\n", (int) ret, info.path);

	return 0;
}

static void pps_tty_close(struct tty_struct *tty)
{
	long id = (long) tty->disc_data;

	pps_unregister_source(id);
	n_tty_close(tty);

	pr_info("PPS source #%d removed\n", (int) id);
}

struct tty_ldisc_ops pps_ldisc_ops = {
	.owner		= THIS_MODULE,
	.magic		= PPS_TTY_MAGIC,
	.name		= "pps_tty",
	.dcd_change	= pps_tty_dcd_change,
	.open		= pps_tty_open,
	.close		= pps_tty_close,

	/* Now we should use N_TTY ldisc methods in order to have
	 * normal tty behaviour
	 */
	.flush_buffer	= n_tty_flush_buffer,
	.chars_in_buffer = n_tty_chars_in_buffer,
	.read		= n_tty_read,
	.write		= n_tty_write,
	.ioctl		= n_tty_ioctl_helper,
	.set_termios	= n_tty_set_termios,
	.poll		= n_tty_poll,
	.receive_buf	= n_tty_receive_buf,
	.write_wakeup	= n_tty_write_wakeup
};

/*
 * Module stuff
 */

static int __init pps_tty_init(void)
{
	int err;

	err = tty_register_ldisc(N_PPS, &pps_ldisc_ops);
	if (err)
		pr_err("can't register PPS line discipline\n");
	else
		pr_info("PPS line discipline registered\n");

	return err;
}

static void __exit pps_tty_cleanup(void)
{
	int err;

	err = tty_unregister_ldisc(N_PPS);
	if (err)
		pr_err("can't unregister PPS line discipline\n");
	else
		pr_info("PPS line discipline removed\n");
}

module_init(pps_tty_init);
module_exit(pps_tty_cleanup);

MODULE_ALIAS_LDISC(N_PPS);
MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("PPS TTY device driver");
MODULE_LICENSE("GPL");
