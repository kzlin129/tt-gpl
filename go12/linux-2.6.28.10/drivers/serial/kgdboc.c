/*
 * Based on the same principle as kgdboe using the NETPOLL api, this
 * driver uses a console polling api to implement a gdb serial inteface
 * which is multiplexed on a console port.
 *
 * Maintainer: Jason Wessel <jason.wessel@windriver.com>
 *
 * 2007-2008 (c) Jason Wessel - Wind River Systems, Inc.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/kgdb.h>
#include <linux/tty.h>

#define MAX_CONFIG_LEN		40
#define MAX_CHAR_RING		512

static struct kgdb_io		kgdboc_io_ops;

/* -1 = init not run yet, 0 = unconfigured, 1 = configured. */
static int configured		= -1;

static char config[MAX_CONFIG_LEN];
static struct kparam_string kps = {
	.string			= config,
	.maxlen			= MAX_CONFIG_LEN,
};

static struct tty_driver	*kgdb_tty_driver;
static int			kgdb_tty_line;
static struct file		*kgdb_filp;

static int kgdboc_option_setup(char *opt)
{
	if (strlen(opt) > MAX_CONFIG_LEN) {
		printk(KERN_ERR "kgdboc: config string too long\n");
		return -ENOSPC;
	}
	strcpy(config, opt);

	return 0;
}

static int buffered_char = -1;
static u8 ch_ring[MAX_CHAR_RING];
static int ch_head;
static int ch_tail;
static u8 break_char;
static int no_polled_breaks;
static int schedule_breakpoints;

/* Return 1 if a the next layer up should discard the character,
 * else return 0
 */
static int kgdboc_rx_callback(u8 c)
{
	if (likely(atomic_read(&kgdb_active) == -1)) {
		if (no_polled_breaks)
			return 0;
		if (c != break_char)
			buffered_char = c;
		if (c == break_char ||
		    (c == '$' && !kgdb_connected && break_char == 0x03)) {
			if (schedule_breakpoints)
				kgdb_schedule_breakpoint();
			else
				kgdb_breakpoint();
			return 1;
		}
		return 0;
	}
	/* Assume the debugger is active and store the characters in an
	 * array which will be decremented at a later point.
	 */
	ch_ring[ch_tail] = c;
	ch_tail++;
	if (ch_tail >= MAX_CHAR_RING)
		ch_tail = 0;
	if (ch_tail == ch_head)
		printk(KERN_CRIT "kgdboc: ERROR rx buffer overflow\n");

	return 1;
}

__setup("kgdboc=", kgdboc_option_setup);

static void release_kgdboc_tty(void)
{
	if (kgdb_tty_driver)
		kgdb_tty_driver->ops->poll_init(kgdb_tty_driver, kgdb_tty_line,
						NULL, (void *)-1);
	if (kgdb_filp)
		tty_console_poll_close(&kgdb_filp);
	kgdb_tty_driver = NULL;
}

static int configure_kgdboc(void)
{
	struct tty_driver *p;
	int tty_line = 0;
	int err;
	char *str;

	err = kgdboc_option_setup(config);
	if (err || !strlen(config) || isspace(config[0]))
		goto noconfig;

	err = -ENODEV;
	/* If a driver was previously configured remove it now */
	release_kgdboc_tty();

	p = tty_find_polling_driver(config, &tty_line);
	if (!p)
		goto noconfig;

	kgdb_tty_driver = p;
	kgdb_tty_line = tty_line;
	/* Set defaults and parse optional configuration information */
	no_polled_breaks = 0;
	schedule_breakpoints = 1;
	break_char = 0x03;
	if (strstr(config, ",n"))
		no_polled_breaks = 1;
	if (strstr(config, ",B"))
		schedule_breakpoints = 0;
	str = strstr(config, ",c");
	if (str)
		break_char = simple_strtoul(str+2, &str, 10);
	str = strrchr(config, ','); /* pointer to baud for init callback */
	if (str) {
		str++;
		if (!(*str >= '0' && *str <= '9'))
			str = NULL;
	}
	/* Initialize the HW level driver for polling */
	if (p->ops->poll_init(p, tty_line, str, kgdboc_rx_callback))
		goto noconfig;

	/* Open the port and obtain a tty which call low level driver startup */
	if (tty_console_poll_open(kgdb_tty_driver, &kgdb_filp,
				  kgdb_tty_line) != 0)
		goto noconfig;

	err = kgdb_register_io_module(&kgdboc_io_ops);
	if (err)
		goto noconfig;

	configured = 1;

	return 0;

noconfig:
	release_kgdboc_tty();
	config[0] = 0;
	configured = 0;

	return err;
}

static int __init init_kgdboc(void)
{
	/* Already configured? */
	if (configured == 1)
		return 0;

	return configure_kgdboc();
}

static void cleanup_kgdboc(void)
{
	if (configured == 1)
		kgdb_unregister_io_module(&kgdboc_io_ops);
}

static int kgdboc_get_char(void)
{
	int ret;

	if (buffered_char >= 0)
		return xchg(&buffered_char, -1);

	do {
		ret = kgdb_tty_driver->ops->poll_get_char(kgdb_tty_driver,
						kgdb_tty_line);
		if (ret != -2)
			return ret;

		/* A return of -2 means use the poll character ring */
		if (ch_head != ch_tail) {
			ret = ch_ring[ch_head];
			ch_head++;
			if (ch_head >= MAX_CHAR_RING)
				ch_head = 0;
			return ret;
		}
	} while (ret == -2);

	return -1;
}

static void kgdboc_put_char(u8 chr)
{
	kgdb_tty_driver->ops->poll_put_char(kgdb_tty_driver,
					kgdb_tty_line, chr);
}

static int param_set_kgdboc_var(const char *kmessage, struct kernel_param *kp)
{
	int len = strlen(kmessage);

	if (len >= MAX_CONFIG_LEN) {
		printk(KERN_ERR "kgdboc: config string too long\n");
		return -ENOSPC;
	}

	/* Only copy in the string if the init function has not run yet */
	if (configured < 0) {
		strcpy(config, kmessage);
		return 0;
	}

	if (kgdb_connected) {
		printk(KERN_ERR
		       "kgdboc: Cannot reconfigure while KGDB is connected.\n");

		return -EBUSY;
	}

	strcpy(config, kmessage);
	/* Chop out \n char as a result of echo */
	if (config[len - 1] == '\n')
		config[len - 1] = '\0';

	if (configured == 1)
		cleanup_kgdboc();

	/* Go and configure with the new params. */
	return configure_kgdboc();
}

static void kgdboc_pre_exp_handler(void)
{
	/* Increment the module count when the debugger is active */
	if (!kgdb_connected)
		try_module_get(THIS_MODULE);
	ch_head = 0;
	ch_tail = 0;
}

static void kgdboc_post_exp_handler(void)
{
	/* decrement the module count when the debugger detaches */
	if (!kgdb_connected)
		module_put(THIS_MODULE);
}

static struct kgdb_io kgdboc_io_ops = {
	.name			= "kgdboc",
	.read_char		= kgdboc_get_char,
	.write_char		= kgdboc_put_char,
	.pre_exception		= kgdboc_pre_exp_handler,
	.post_exception		= kgdboc_post_exp_handler,
};

module_init(init_kgdboc);
module_exit(cleanup_kgdboc);
module_param_call(kgdboc, param_set_kgdboc_var, param_get_string, &kps, 0644);
/* The optional paramters to the config string are:
 * ,n == no monitoring the port for a break char
 * ,B == monitor the port for a break char and issue a breakpoint in line
 * ,c### == Use an alternate break character 1-255 instead of ^C (3)
 * The baud parameter must always be last, if used
 * ,baud == A baud rate parameter IE: 115200n81
 */
MODULE_PARM_DESC(kgdboc, "<serial_device>[,n][,B][,c###][,baud]");
MODULE_DESCRIPTION("KGDB Console TTY Driver");
MODULE_LICENSE("GPL");
