/*
 * linux/drivers/serial/s3c2410.c
 *
 * Driver for onboard UARTs on the Samsung S3C24XX
 *
 * Based on drivers/char/serial.c and drivers/char/21285.c
 *
 * Ben Dooks, (c) 2003-2005 Simtec Electronics
 *	http://www.simtec.co.uk/products/SWLINUX/
 *
 * Changelog:
 *
 * 22-Jul-2004  BJD  Finished off device rewrite
 *
 * 21-Jul-2004  BJD  Thanks to <herbet@13thfloor.at> for pointing out
 *                   problems with baud rate and loss of IR settings. Update
 *                   to add configuration via platform_device structure
 *
 * 28-Sep-2004  BJD  Re-write for the following items
 *		     - S3C2410 and S3C2440 serial support
 *		     - Power Management support
 *		     - Fix console via IrDA devices
 *		     - SysReq (Herbert Pötzl)
 *		     - Break character handling (Herbert Pötzl)
 *		     - spin-lock initialisation (Dimitry Andric)
 *		     - added clock control
 *		     - updated init code to use platform_device info
 *
 * 06-Mar-2005  BJD  Add s3c2440 fclk clock source
 *
 * 09-Mar-2005  BJD  Add s3c2400 support
 *
 * 10-Mar-2005  LCVR Changed S3C2410_VA_UART to S3C24XX_VA_UART
*/

/* Note on 2440 fclk clock source handling
 *
 * Whilst it is possible to use the fclk as clock source, the method
 * of properly switching too/from this is currently un-implemented, so
 * whichever way is configured at startup is the one that will be used.
*/

/* Hote on 2410 error handling
 *
 * The s3c2410 manual has a love/hate affair with the contents of the
 * UERSTAT register in the UART blocks, and keeps marking some of the
 * error bits as reserved. Having checked with the s3c2410x01,
 * it copes with BREAKs properly, so I am happy to ignore the RESERVED
 * feature from the latter versions of the manual.
 *
 * If it becomes aparrent that latter versions of the 2410 remove these
 * bits, then action will have to be taken to differentiate the versions
 * and change the policy on BREAK
 *
 * BJD, 04-Nov-2004
*/

#include <linux/config.h>

#if defined(CONFIG_SERIAL_S3C2410_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <asm/hardware.h>
#include <asm/hardware/clock.h>

#include <asm/arch/regs-serial.h>
#include <asm/arch/regs-gpio.h>

#include <asm/mach-types.h>
#include <linux/cpufreq.h>

#include <barcelona/gopins.h>
/* todo: move this to include/ */
#include "../arch/arm/mach-s3c2410/tomtomgo-iopins.h"

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <asm/arch-s3c2410/regs-clock.h>
#include <barcelona/cpufreq_order.h>

static struct notifier_block	freq_transition;
static struct notifier_block	freq_policy;
#endif

/* structures */

struct s3c24xx_uart_info {
	char			*name;
	unsigned int		type;
	unsigned int		fifosize;
	unsigned long		rx_fifomask;
	unsigned long		rx_fifoshift;
	unsigned long		rx_fifofull;
	unsigned long		tx_fifomask;
	unsigned long		tx_fifoshift;
	unsigned long		tx_fifofull;
	int			has_slot;

	/* clock source control */

	int (*get_clksrc)(struct uart_port *, struct s3c24xx_uart_clksrc *clk);
	int (*set_clksrc)(struct uart_port *, struct s3c24xx_uart_clksrc *clk);

	/* uart controls */
	int (*reset_port)(struct uart_port *, struct s3c2410_uartcfg *);
};

struct s3c24xx_uart_port {
	unsigned char			rx_claimed;
	unsigned char			tx_claimed;

	struct s3c24xx_uart_info	*info;
	struct s3c24xx_uart_clksrc	*clksrc;
	struct clk			*clk;
	struct clk			*baudclk;
	struct uart_port		port;
	struct timer_list		cts_poll_timer;
};


/* configuration defines */

#if 0
#if 1
/* send debug to the low-level output routines */

extern void printascii(const char *);

static void
s3c24xx_serial_dbg(const char *fmt, ...)
{
	va_list va;
	char buff[256];

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

	printascii(buff);
}

#define dbg(x...) s3c24xx_serial_dbg(x)

#else
#define dbg(format, arg...) printk(KERN_DEBUG __FILE__ ": " format, ## arg)
#endif
#else /* no debug */
#define dbg(x...) do {} while(0)
#endif

/* UART name and device definitions */

#define S3C24XX_SERIAL_NAME	"ttySAC"
#define S3C24XX_SERIAL_DEVFS    "tts/"
#define S3C24XX_SERIAL_MAJOR	204
#define S3C24XX_SERIAL_MINOR	64


/* conversion functions */

#define s3c24xx_dev_to_port(__dev) (struct uart_port *)dev_get_drvdata(__dev)
#define s3c24xx_dev_to_cfg(__dev) (struct s3c2410_uartcfg *)((__dev)->platform_data)

/* we can support 4 uarts, but not always use them */

#define NR_PORTS 4

/* port irq numbers */

#define TX_IRQ(port) ((port)->irq + 1)
#define RX_IRQ(port) ((port)->irq)

/* register access controls */

#define portaddr(port, reg) ((port)->membase + (reg))

#define rd_regb(port, reg) (__raw_readb(portaddr(port, reg)))
#define rd_regl(port, reg) (__raw_readl(portaddr(port, reg)))

#define wr_regb(port, reg, val) \
  do { __raw_writeb(val, portaddr(port, reg)); } while(0)

#define wr_regl(port, reg, val) \
  do { __raw_writel(val, portaddr(port, reg)); } while(0)

/* macros to change one thing to another */

#define tx_enabled(port) ((port)->unused[0])
#define rx_enabled(port) ((port)->unused[1])

/* flag to ignore all characters comming in */
#define RXSTAT_DUMMY_READ (0x10000000)

/*
 * This determines how often we check the modem status signals
 * for any change.  They generally aren't connected to an IRQ
 * so we have to poll them.  We also check immediately before
 * filling the TX fifo in case CTS has been dropped.
 */
#define S3C24XX_TIMEOUT	(250*HZ/1000)

/*
 * Checks modem status via CTS line.
 */
static void check_modem_status(struct uart_port *port)
{
	unsigned int umstat = rd_regb(port,S3C2410_UMSTAT);
 
	if (umstat & S3C2410_UMSTAT_DeltaCTS)
	{
		uart_handle_cts_change(port, umstat & S3C2410_UMSTAT_CTS);
		dbg("CTS line of UART %d changed its state. New state %d\n", 
			port->info->tty->index, umstat & S3C2410_UMSTAT_CTS);
	}

	wake_up_interruptible(&port->info->delta_msr_wait);
}

/*
 * This is our per-port timeout handler, for checking the
 * modem status signals.
 */
static void s3c24xx_timeout(unsigned long data)
{
	struct s3c24xx_uart_port *ourport = (struct s3c24xx_uart_port *)data;
	struct uart_port         *port    = &ourport->port;
	unsigned long flags;
  
	if (port->info) 
	{
		spin_lock_irqsave(&port->lock, flags);
		check_modem_status(port);
		spin_unlock_irqrestore(&port->lock, flags);
		mod_timer(&ourport->cts_poll_timer, jiffies + S3C24XX_TIMEOUT);
	}
}

static inline struct s3c24xx_uart_port *to_ourport(struct uart_port *port)
{
	return container_of(port, struct s3c24xx_uart_port, port);
}

/* translate a port to the device name */

static inline const char *s3c24xx_serial_portname(struct uart_port *port)
{
	return to_platform_device(port->dev)->name;
}

static int s3c24xx_serial_txempty_nofifo(struct uart_port *port)
{
	return (rd_regl(port, S3C2410_UTRSTAT) & S3C2410_UTRSTAT_TXE);
}

static void s3c24xx_serial_rx_enable(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ucon, ufcon;
	int count = 10000;

	spin_lock_irqsave(&port->lock, flags);

	while (--count && !s3c24xx_serial_txempty_nofifo(port))
		udelay(100);

	ufcon = rd_regl(port, S3C2410_UFCON);
	ufcon &= ~(S3C2440_UFCON_RXTRIGMASK);
	switch (port->type) {
		case PORT_S3C2412:
		case PORT_S3C2440: /* and S3C2442 */ 
		case PORT_S3C2443:
			/* 64 byte fifo */
			ufcon |= S3C2440_UFCON_RXTRIG32;
			break;
		case PORT_S3C2410:
			/* 16 byte fifo, and other levels/flags */
			ufcon |= S3C2410_UFCON_RXTRIG8;
			break;
	}
	wr_regl(port, S3C2410_UFCON, ufcon | S3C2410_UFCON_RESETRX);
	wr_regl(port, S3C2410_UFCON, ufcon);

	ucon = rd_regl(port, S3C2410_UCON);
	ucon &= ~(S3C2410_UCON_RXIRQMASK);
	ucon |= S3C2410_UCON_RXIRQMODE | S3C2410_UCON_RXFIFO_TOI;
	wr_regl(port, S3C2410_UCON, ucon);

	rx_enabled(port) = 1;
	spin_unlock_irqrestore(&port->lock, flags);
}

static void s3c24xx_serial_rx_disable(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ucon;

	spin_lock_irqsave(&port->lock, flags);

	ucon = rd_regl(port, S3C2410_UCON);
	ucon &= ~S3C2410_UCON_RXIRQMODE;
	wr_regl(port, S3C2410_UCON, ucon);

	rx_enabled(port) = 0;
	spin_unlock_irqrestore(&port->lock, flags);
}

static void s3c24xx_serial_stop_tx(struct uart_port *port)
{
	if (tx_enabled(port)) {
		disable_irq(TX_IRQ(port));
		tx_enabled(port) = 0;
		if (port->flags & UPF_CONS_FLOW)
			s3c24xx_serial_rx_enable(port);
	}
}

static void s3c24xx_serial_start_tx(struct uart_port *port)
{
	if (!tx_enabled(port)) {
		if (port->flags & UPF_CONS_FLOW)
			s3c24xx_serial_rx_disable(port);

		enable_irq(TX_IRQ(port));
		tx_enabled(port) = 1;
	}
}


static void s3c24xx_serial_stop_rx(struct uart_port *port)
{
	if (rx_enabled(port)) {
		dbg("s3c24xx_serial_stop_rx: port=%p\n", port);
		disable_irq(RX_IRQ(port));
		rx_enabled(port) = 0;
	}
}

static void s3c24xx_serial_enable_ms(struct uart_port *port)
{
	struct s3c24xx_uart_port *ourport = to_ourport(port);

	mod_timer(&ourport->cts_poll_timer, jiffies);
}

static inline struct s3c24xx_uart_info *s3c24xx_port_to_info(struct uart_port *port)
{
	return to_ourport(port)->info;
}

static inline struct s3c2410_uartcfg *s3c24xx_port_to_cfg(struct uart_port *port)
{
	if (port->dev == NULL)
		return NULL;

	return (struct s3c2410_uartcfg *)port->dev->platform_data;
}

static int s3c24xx_serial_rx_fifocnt(struct s3c24xx_uart_port *ourport,
				     unsigned long ufstat)
{
	struct s3c24xx_uart_info *info = ourport->info;

	if (ufstat & info->rx_fifofull)
		return info->fifosize;

	return (ufstat & info->rx_fifomask) >> info->rx_fifoshift;
}

/* ? - where has parity gone?? */
#define S3C2410_UERSTAT_PARITY (0x1000)

static irqreturn_t 
s3c24xx_serial_rx_chars(int irq, void *dev_id, struct pt_regs *regs)
{
	struct s3c24xx_uart_port *ourport = dev_id;
	struct uart_port *port = &ourport->port;
	struct tty_struct *tty = port->info->tty;
	unsigned int ufcon, ch, flag, ufstat, uerstat;
	int max_count = 64;

	while (max_count-- > 0) {
		ufcon = rd_regl(port, S3C2410_UFCON);
		ufstat = rd_regl(port, S3C2410_UFSTAT);

		if (s3c24xx_serial_rx_fifocnt(ourport, ufstat) == 0)
			break;

		/* Pushing forced in interrupt context */
		if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
			int low_latency = tty->low_latency;
			tty->low_latency = 1;
			tty_flip_buffer_push(tty);
			tty->low_latency = low_latency;
		}

		uerstat = rd_regl(port, S3C2410_UERSTAT);
		ch = rd_regb(port, S3C2410_URXH);

		if (port->flags & UPF_CONS_FLOW) {
			int txe = s3c24xx_serial_txempty_nofifo(port);

			if (rx_enabled(port)) {
				if (!txe) {
					rx_enabled(port) = 0;
					continue;
				}
			} else {
				if (txe) {
					ufcon |= S3C2410_UFCON_RESETRX;
					wr_regl(port, S3C2410_UFCON, ufcon);
					rx_enabled(port) = 1;
					goto out;
				}
				continue;
			}
		}

		/* insert the character into the buffer */

		flag = TTY_NORMAL;
		port->icount.rx++;

		if (unlikely(uerstat & S3C2410_UERSTAT_ANY)) {
			dbg("rxerr: port line=%d ch=0x%02x, rxs=0x%08x\n", port->line, 
			    ch, uerstat);

			/* check for break */
			if (uerstat & S3C2410_UERSTAT_BREAK) {
				dbg("break!\n");
				port->icount.brk++;
				if (uart_handle_break(port))
				    goto ignore_char;
			}

			if (uerstat & S3C2410_UERSTAT_FRAME)
				port->icount.frame++;
			if (uerstat & S3C2410_UERSTAT_OVERRUN)
				port->icount.overrun++;

			uerstat &= port->read_status_mask;

			if (uerstat & S3C2410_UERSTAT_BREAK)
				flag = TTY_BREAK;
			else if (uerstat & S3C2410_UERSTAT_PARITY)
				flag = TTY_PARITY;
			else if (uerstat & ( S3C2410_UERSTAT_FRAME | S3C2410_UERSTAT_OVERRUN))
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(port, ch))
			goto ignore_char;

		if ((uerstat & port->ignore_status_mask & ~S3C2410_UERSTAT_OVERRUN) == 0) {
			tty_insert_flip_char(tty, ch, flag);
		}

		/*
		 * Overrun is special.  Since it's reported immediately,
		 * it doesn't affect the current character.
		 */
		if (uerstat & ~port->ignore_status_mask & S3C2410_UERSTAT_OVERRUN)
			tty_insert_flip_char(tty, 0, TTY_OVERRUN);

	ignore_char:
		continue;
	}
	tty_flip_buffer_push(tty);

 out:
	return IRQ_HANDLED;
}

static irqreturn_t 
s3c24xx_serial_tx_chars(int irq, void *id, struct pt_regs *regs)
{
	struct s3c24xx_uart_port *ourport = id;
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->info->xmit;
	int count = 256;

	if (port->x_char) {
		wr_regb(port, S3C2410_UTXH, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		goto out;
	}

	/* if there isnt anything more to transmit, or the uart is now
	 * stopped, disable the uart and exit
	*/

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		s3c24xx_serial_stop_tx(port);
		goto out;
	}

	/* try and drain the buffer... */

	while (!uart_circ_empty(xmit) && count-- > 0) {
		if (rd_regl(port, S3C2410_UFSTAT) & ourport->info->tx_fifofull)
			break;

		wr_regb(port, S3C2410_UTXH, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

 out:
	return IRQ_HANDLED;
}

static unsigned int s3c24xx_serial_tx_empty(struct uart_port *port)
{
	struct s3c24xx_uart_info *info = s3c24xx_port_to_info(port);
	unsigned long ufstat = rd_regl(port, S3C2410_UFSTAT);
	unsigned long ufcon = rd_regl(port, S3C2410_UFCON);

	if (ufcon & S3C2410_UFCON_FIFOMODE) {
		if ((ufstat & info->tx_fifomask) != 0 ||
		    (ufstat & info->tx_fifofull))
			return 0;

		return 1;
	}

	return s3c24xx_serial_txempty_nofifo(port);
}

/* no modem control lines */
static unsigned int s3c24xx_serial_get_mctrl(struct uart_port *port)
{
	unsigned int umstat = rd_regb(port,S3C2410_UMSTAT);

	if (umstat & S3C2410_UMSTAT_CTS)
		return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
	else
		return TIOCM_CAR | TIOCM_DSR;
}

static void s3c24xx_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* if no AFC selected, do manual RTS */
	unsigned int mcr = rd_regb(port, S3C2410_UMCON);

	if (mctrl & TIOCM_RTS)
		mcr |= S3C2410_UMCOM_RTS_LOW;

	/* TODO: control other TIOCM lines (DTR/DSR etc), might be mappped to other GPIO pins */
	wr_regb(port, S3C2410_UMCON, mcr);
}

static void s3c24xx_serial_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;
	unsigned int ucon;

	spin_lock_irqsave(&port->lock, flags);

	ucon = rd_regl(port, S3C2410_UCON);

	if (break_state)
		ucon |= S3C2410_UCON_SBREAK;
	else
		ucon &= ~S3C2410_UCON_SBREAK;

	wr_regl(port, S3C2410_UCON, ucon);

	spin_unlock_irqrestore(&port->lock, flags);
}

static void s3c24xx_serial_shutdown(struct uart_port *port)
{
	struct s3c24xx_uart_port *ourport = to_ourport(port);

	/*
	 * Stop our timer.
	 */
	del_timer_sync(&ourport->cts_poll_timer);

	if (ourport->tx_claimed) {
		free_irq(TX_IRQ(port), ourport);
		tx_enabled(port) = 0;
		ourport->tx_claimed = 0;
	}

	if (ourport->rx_claimed) {
		free_irq(RX_IRQ(port), ourport);
		ourport->rx_claimed = 0;
		rx_enabled(port) = 0;
	}
}


static int s3c24xx_serial_startup(struct uart_port *port)
{
	struct s3c24xx_uart_port *ourport = to_ourport(port);
	unsigned long flags;
	int ret;

	dbg("s3c24xx_serial_startup: port=%p (%08lx,%p)\n",
	    port->mapbase, port->membase);

	rx_enabled(port) = 1;

	ret = request_irq(RX_IRQ(port),
			  s3c24xx_serial_rx_chars, 0,
			  s3c24xx_serial_portname(port), ourport);

	if (ret != 0) {
		printk(KERN_ERR "cannot get irq %d\n", RX_IRQ(port));
		return ret;
	}

	ourport->rx_claimed = 1;

	dbg("requesting tx irq...\n");

	tx_enabled(port) = 1;

	ret = request_irq(TX_IRQ(port),
			  s3c24xx_serial_tx_chars, 0,
			  s3c24xx_serial_portname(port), ourport);

	if (ret) {
		printk(KERN_ERR "cannot get irq %d\n", TX_IRQ(port));
		goto err;
	}

	ourport->tx_claimed = 1;

	/*
	 * Enable modem status interrupts
	 */
	spin_lock_irqsave(&port->lock, flags);
	s3c24xx_serial_enable_ms(port);
	spin_unlock_irqrestore(&port->lock,flags);

	dbg("s3c24xx_serial_startup ok\n");

	/* the port reset code should have done the correct
	 * register setup for the port controls */

	return ret;

 err:
	s3c24xx_serial_shutdown(port);
	return ret;
}

/* power power management control */

static void s3c24xx_serial_pm(struct uart_port *port, unsigned int level,
			      unsigned int old)
{
	struct s3c24xx_uart_port *ourport = to_ourport(port);

	switch (level) {
	case 3:
#ifdef CONFIG_S3C2410_PM_DEBUG
		printk(KERN_INFO "s3c24xx_serial: ignoring request to powerdown\n");
#else /* CONFIG_S3C2410_PM_DEBUG */
		if (!IS_ERR(ourport->baudclk) && ourport->baudclk != NULL)
			clk_disable(ourport->baudclk);

		clk_disable(ourport->clk);
#endif /* CONFIG_S3C2410_PM_DEBUG */
		break;

	case 0:
		clk_enable(ourport->clk);

		if (!IS_ERR(ourport->baudclk) && ourport->baudclk != NULL)
			clk_enable(ourport->baudclk);

		break;
	default:
		printk(KERN_ERR "s3c24xx_serial: unknown pm %d\n", level);
	}
}

/* baud rate calculation
 *
 * The UARTs on the S3C2410/S3C2440 can take their clocks from a number
 * of different sources, including the peripheral clock ("pclk") and an
 * external clock ("uextclk" in modern manuals, UCLK in 2410/40 manuals). 
 * The S3C2440 also adds the core clock ("fclk") with a programmable extra divisor.
 *
 * The following code goes through the clock sources, and calculates the
 * baud clocks (and the resultant actual baud rates) and then tries to
 * pick the closest one and select that.
 *
*/


#define MAX_CLKS (8)

static struct s3c24xx_uart_clksrc tmp_clksrc = {
	.name		= "pclk",
	.min_baud	= 0,
	.max_baud	= 0,
	.rate		= 0,
	.divisor	= 1,
};

static inline int
s3c24xx_serial_getsource(struct uart_port *port, struct s3c24xx_uart_clksrc *c)
{
	struct s3c24xx_uart_info *info = s3c24xx_port_to_info(port);

	return (info->get_clksrc)(port, c);
}

static inline int
s3c24xx_serial_setsource(struct uart_port *port, struct s3c24xx_uart_clksrc *c)
{
	struct s3c24xx_uart_info *info = s3c24xx_port_to_info(port);

	return (info->set_clksrc)(port, c);
}

struct baud_calc {
	struct s3c24xx_uart_clksrc	*clksrc;
	unsigned int			 calc;
	unsigned int			 quot;
	unsigned int			 slot;
	struct clk			*src;
};

static int s3c24xx_serial_calcbaud(struct baud_calc *calc,
				   struct uart_port *port,
				   struct s3c24xx_uart_clksrc *clksrc,
				   unsigned int baud)
{
	unsigned long rate;
	unsigned long targ_rate;

	calc->src = clk_get(port->dev, clksrc->name);
	if (calc->src == NULL || IS_ERR(calc->src))
		return 0;

	/* Clock rate can't be zero. Make sure there is a rate. */
	if( clksrc->rate == 0 )
		clksrc->rate=clk_get_rate( calc->src );

	rate = clksrc->rate/clksrc->divisor;
	calc->clksrc = clksrc;

	if( s3c24xx_port_to_info( port )->has_slot )
	{
		targ_rate=(rate+(baud/2))/baud;
		calc->quot=targ_rate/16;
		calc->slot=targ_rate%16;
		calc->calc=rate/(16*calc->quot + calc->slot); 
	}
	else
	{
		calc->quot=(rate + (8 * baud)) / (16 * baud);
		calc->calc=(rate / (calc->quot * 16));
		calc->slot=0;
	}
		
	calc->quot--;
	return 1;
}

static unsigned int s3c24xx_serial_getclk(struct uart_port *port,
					  struct s3c24xx_uart_clksrc **clksrc,
					  struct clk **clk,
					  unsigned int *slot,
					  unsigned int baud)
{
	struct s3c2410_uartcfg *cfg = s3c24xx_port_to_cfg(port);
	struct s3c24xx_uart_clksrc *clkp;
	struct baud_calc res[MAX_CLKS];
	struct baud_calc *resptr, *best, *sptr;
	int i;

	clkp = cfg->clocks;
	best = NULL;

	if (cfg->clocks_size < 2) {
		if (cfg->clocks_size == 0)
			clkp = &tmp_clksrc;

		/* check to see if we're sourcing fclk, and if so we're
		 * going to have to update the clock source
		 */

		if (strcmp(clkp->name, "fclk") == 0) {
			struct s3c24xx_uart_clksrc src;

			s3c24xx_serial_getsource(port, &src);

			/* check that the port already using fclk, and if
			 * not, then re-select fclk
			 */

			if (strcmp(src.name, clkp->name) == 0) {
				s3c24xx_serial_setsource(port, clkp);
				s3c24xx_serial_getsource(port, &src);
			}

			clkp->divisor = src.divisor;
		}

		s3c24xx_serial_calcbaud(res, port, clkp, baud);
		best = res;
		resptr = best + 1;
	} else {
		resptr = res;

		for (i = 0; i < cfg->clocks_size; i++, clkp++) {
			if (s3c24xx_serial_calcbaud(resptr, port, clkp, baud))
				resptr++;
		}
	}

	/* ok, we now need to select the best clock we found */

	if (!best) {
		unsigned int deviation = (1<<30)|((1<<30)-1);
		int calc_deviation;

		for (sptr = res; sptr < resptr; sptr++) {
			printk(KERN_DEBUG
			       "found clk %p (%s) quot %d, calc %d\n",
			       sptr->clksrc, sptr->clksrc->name,
			       sptr->quot, sptr->calc);

			calc_deviation = baud - sptr->calc;
			if (calc_deviation < 0)
				calc_deviation = -calc_deviation;

			if (calc_deviation < deviation) {
				best = sptr;
				deviation = calc_deviation;
			}
		}

		printk(KERN_DEBUG "best %p (deviation %d)\n", best, deviation);
	}

	printk(KERN_DEBUG "selected clock %p (%s) quot %d, calc %d\n",
	       best->clksrc, best->clksrc->name, best->quot, best->calc);

	/* store results to pass back */

	*clksrc = best->clksrc;
	*clk    = best->src;
	*slot   = best->slot;

	return best->quot;
}

static inline unsigned divslot2num( unsigned int divslot )
{
	unsigned int count=0;

	while( divslot != 0 )
	{
		if( divslot & 0x0001 ) count+=1; 
		divslot>>=1;
	}
	return count;
}

static inline unsigned num2divslot( unsigned int num )
{
	/*
	 * From the S3C2443X manual, table 5.2, "Recommended value
	 * table of DIVSLOTn register"
	 */
	static const unsigned values[] = {
		0x0000, 0x0008, 0x0808, 0x0888, 0x2222, 0x4924, 0x4A52, 0x54AA,
		0x5555, 0xD555, 0xD5D5, 0xDDD5, 0xDDDD, 0xDFDD, 0xDFDF, 0xFFDF,
	};
	BUG_ON(num >= ARRAY_SIZE(values));
	return values[num];
}

static const char *s3c24xx_serial_type(struct uart_port *port)
{
	switch (port->type) {
	case PORT_S3C2400:
		return "S3C2400";
	case PORT_S3C2410:
		return "S3C2410";
	case PORT_S3C2412:
		return "S3C2412";
	case PORT_S3C2440:
		return "S3C2440";
	case PORT_S3C2443:
		return "S3C2443";
	default:
		return NULL;
	}
}

static void s3c24xx_serial_set_rtscts(struct uart_port *port, gopin_t rtspin, gopin_t ctspin)
{
	gopin_t expected_rtspin[2] = {0,0};
	gopin_t expected_ctspin[2] = {0,0};
	int	expected_func[2]   = {0,0};
	int	expected_pins	   = 0;
	int	option;

	switch (port->type) {
		case PORT_S3C2443:
			switch (port->line) {
				case 0:
					expected_rtspin[0] = PIN_GPH9;
					expected_ctspin[0] = PIN_GPH8;
					expected_func[0]   = 0;
					expected_pins = 1;
					break;
				case 1:
					expected_rtspin[0] = PIN_GPH11;
					expected_ctspin[0] = PIN_GPH10;
					expected_func[0]   = 0;
					expected_pins = 1;
					break;
				case 2:
					expected_rtspin[0] = PIN_GPH6;
					expected_ctspin[0] = PIN_GPH7;
					expected_func[0]   = 1;
					expected_pins = 1;
					break;
				default:
					break;
			}
			break;
		case PORT_S3C2412:
		case PORT_S3C2440:
                        /* S3C2442 is equivalent to S3C2440 */
			switch (port->line) {
				case 0:
					expected_rtspin[0] = PIN_GPH1;
					expected_ctspin[0] = PIN_GPH0;
					expected_func[0]   = 0;
					expected_pins 	   = 1;
					break;
				case 1:
					expected_rtspin[0] = PIN_GPG9;
					expected_ctspin[0] = PIN_GPG10;
					expected_func[0]   = 1;
					expected_rtspin[1] = PIN_GPH6;
					expected_ctspin[1] = PIN_GPH7;
					expected_func[1]   = 1;
					expected_pins      = 2;
					break;
				default:
					break;
			}
			break;
		case PORT_S3C2410:
			switch (port->line) {
				case 0:
					expected_rtspin[0] = PIN_GPH1;
					expected_ctspin[0] = PIN_GPH0;
					expected_func[1]   = 0;
					expected_pins      = 1;
					break;
				case 1:
					expected_rtspin[1] = PIN_GPH6;
					expected_ctspin[1] = PIN_GPH7;
					expected_func[1]   = 1;
					expected_pins      = 1;
					break;
				default:
					break;
			}
			break;
	}
	if (!expected_pins) {
		printk(KERN_ERR "PORT_%s rts/cts AFC not supported for port %d\n", s3c24xx_serial_type(port), port->line);
		goto exit;
	}

	/* check RTS location */
	for (option=0; option<expected_pins; option++)
		if (IO_GetPinLocation(expected_rtspin[option]) == IO_GetPinLocation(rtspin))
			goto found_rts;
	printk(KERN_ERR "PORT_%s impossible RTS gpio mapping for port %d\n", s3c24xx_serial_type(port), port->line);
	goto exit;
found_rts:
	IOP_SetFunc(expected_rtspin[option], expected_func[option]);

	/* check CTS location */
	for (option=0; option<expected_pins; option++)
		if (IO_GetPinLocation(expected_ctspin[option]) == IO_GetPinLocation(ctspin)) 
			goto found_cts;
	printk(KERN_ERR "PORT_%s impossible CTS gpio mapping for port %d\n", s3c24xx_serial_type(port), port->line);
	goto exit;
found_cts:
	IOP_SetFunc(expected_ctspin[option], expected_func[option]);

	printk(KERN_ERR "PORT_%s AFC RTS/CTS enabled for port %d\n", s3c24xx_serial_type(port), port->line);
exit:
	return;
}

static void s3c24xx_serial_set_termios(struct uart_port *port,
				       struct termios *termios,
				       struct termios *old)
{
	struct s3c2410_uartcfg *cfg = s3c24xx_port_to_cfg(port);
	struct s3c24xx_uart_port *ourport = to_ourport(port);
	struct s3c24xx_uart_clksrc *clksrc = NULL;
	struct clk *clk = NULL;
	unsigned long flags;
	unsigned int baud, quot;
	unsigned int slot;
	unsigned int ulcon;
	unsigned int umcon;

	/*
	 * We don't support modem control lines.
	 */
	termios->c_cflag &= ~(HUPCL | CMSPAR);
	termios->c_cflag |= CLOCAL;

	/*
	 * Ask the core to calculate the divisor for us.
	 */

	baud = uart_get_baud_rate(port, termios, old, 0, 115200*8);

	if (baud == 38400 && (port->flags & UPF_SPD_MASK) == UPF_SPD_CUST)
	{
		if( port->flags & UPF_MAGIC_MULTIPLIER )
		{
			quot = port->custom_divisor / 16 - 1;
			slot = port->custom_divisor % 16;
		}
		else
		{
			quot = port->custom_divisor - 1;
			slot = 0;
		}
	}
	else
		quot = s3c24xx_serial_getclk(port, &clksrc, &clk, &slot, baud);

	/* check to see if we need  to change clock source */

	if (ourport->clksrc != clksrc || ourport->baudclk != clk) {
		s3c24xx_serial_setsource(port, clksrc);

		if (ourport->baudclk != NULL && !IS_ERR(ourport->baudclk)) {
			clk_disable(ourport->baudclk);
			clk_unuse(ourport->baudclk);
			ourport->baudclk  = NULL;
		}

		clk_use(clk);
		clk_enable(clk);

		ourport->clksrc = clksrc;
		ourport->baudclk = clk;
	}

	/* Set right clock rate in port struct. */
	if( port->flags & UPF_MAGIC_MULTIPLIER )
	{
		ourport->port.uartclk=16*clk_get_rate( ourport->baudclk );
		ourport->port.custom_divisor=(quot+1)*16 + slot;
	}
	else
	{
		ourport->port.uartclk=clk_get_rate( ourport->baudclk );
		ourport->port.custom_divisor=quot + 1;
	}

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		dbg("config: 5bits/char\n");
		ulcon = S3C2410_LCON_CS5;
		break;
	case CS6:
		dbg("config: 6bits/char\n");
		ulcon = S3C2410_LCON_CS6;
		break;
	case CS7:
		dbg("config: 7bits/char\n");
		ulcon = S3C2410_LCON_CS7;
		break;
	case CS8:
	default:
		dbg("config: 8bits/char\n");
		ulcon = S3C2410_LCON_CS8;
		break;
	}

	/* preserve original lcon IR settings */
	ulcon |= (cfg->ulcon & S3C2410_LCON_IRM);

	if (termios->c_cflag & CSTOPB)
		ulcon |= S3C2410_LCON_STOPB;

	umcon = 0;
	if (termios->c_cflag & CRTSCTS) {
		gopin_t rtspin = 0, ctspin = 0;

		umcon |= S3C2410_UMCOM_AFC;
		umcon &= ~(S3C24XX_UMCON_RTSTRIGMASK);

		switch (port->type) {
			case PORT_S3C2440:
			case PORT_S3C2443:
				/* trigger the rts/cts handshake when 64-byte fifo half full (sender might be slow ?) */
				umcon |= S3C24XX_UMCON_RTSTRIG32;
		}
#ifdef CONFIG_MACH_TOMTOMGO
		/* check tomtom uart gpio mappings */
		if (port->line == IO_GetGpsUartNr()) {
			rtspin = IO_Pin(RTS_GPS);
			ctspin = IO_Pin(CTS_GPS);
		}
		if (port->line == IO_GetBluetoothUartNr()) {
			rtspin = IO_Pin(RTS_BT);
			ctspin = IO_Pin(CTS_BT);
		}
		if (port->line == IO_GetGprsUartNr()) {
			rtspin = IO_Pin(GSM_RTS);
			ctspin = IO_Pin(GSM_CTS);
		}
#else
		/* TODO: pick a RTS/CTS pin from the options, depending on platform definition */
#endif
		if (rtspin && ctspin) {
			s3c24xx_serial_set_rtscts(port, rtspin, ctspin);
		}
	}

	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			ulcon |= S3C2410_LCON_PODD;
		else
			ulcon |= S3C2410_LCON_PEVEN;
	} else {
		ulcon |= S3C2410_LCON_PNONE;
	}

	spin_lock_irqsave(&port->lock, flags);

	if (baud == 921600) {
		unsigned int ufcon;
		ufcon = rd_regl(port, S3C2410_UFCON);
		ufcon &= ~S3C2440_UFCON_RXTRIGMASK;
		ufcon |=  S3C2440_UFCON_RXTRIG8;
		wr_regl(port, S3C2410_UFCON, ufcon);
		dbg("AC%d: lowering IRQ fifo level to 8 byte for %d baud\n", port->line, baud);
	}

	dbg("setting ulcon to %08x, brddiv to %d\n", ulcon, quot);

	wr_regl(port, S3C2410_ULCON, ulcon);
	wr_regl(port, S3C2410_UBRDIV, quot);
	if( ourport->info->has_slot )
		wr_regl( port, S3C2412_UDIVSLOT, num2divslot( slot ) );
	wr_regl(port, S3C2410_UMCON, umcon);

	dbg("uart: ulcon = 0x%08x, ucon = 0x%08x, ufcon = 0x%08x\n",
	    rd_regl(port, S3C2410_ULCON),
	    rd_regl(port, S3C2410_UCON),
	    rd_regl(port, S3C2410_UFCON));

	del_timer_sync(&ourport->cts_poll_timer);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * Which character status flags are we interested in?
	 */
	port->read_status_mask = S3C2410_UERSTAT_OVERRUN;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= S3C2410_UERSTAT_FRAME | S3C2410_UERSTAT_PARITY;

	/*
	 * Which character status flags should we ignore?
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= S3C2410_UERSTAT_OVERRUN;
	if (termios->c_iflag & IGNBRK && termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= S3C2410_UERSTAT_FRAME;

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= RXSTAT_DUMMY_READ;

	if (UART_ENABLE_MS(port, termios->c_cflag))
		s3c24xx_serial_enable_ms(port);

	spin_unlock_irqrestore(&port->lock, flags);
}

#define MAP_SIZE (0x100)

static void s3c24xx_serial_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, MAP_SIZE);
}

static int s3c24xx_serial_request_port(struct uart_port *port)
{
	const char *name = s3c24xx_serial_portname(port);
	return request_mem_region(port->mapbase, MAP_SIZE, name) ? 0 : -EBUSY;
}

static void s3c24xx_serial_config_port(struct uart_port *port, int flags)
{
	struct s3c24xx_uart_info *info = s3c24xx_port_to_info(port);

	if (flags & UART_CONFIG_TYPE &&
	    s3c24xx_serial_request_port(port) == 0)
		port->type = info->type;
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int
s3c24xx_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct s3c24xx_uart_info *info = s3c24xx_port_to_info(port);
	struct s3c24xx_uart_port *ourport = to_ourport(port);

	if (ser->type != PORT_UNKNOWN && ser->type != info->type)
		return -EINVAL;

	/* Ensure we have the right baud_base. */
	if( port->flags & UPF_MAGIC_MULTIPLIER )
	{
		if( port->uartclk != clk_get_rate( ourport->baudclk ) * 16 ) return -EINVAL;
	}
	else
	{
		if( port->uartclk != clk_get_rate( ourport->baudclk ) ) return -EINVAL;
	}

	if( ser->baud_base != port->uartclk/16 )
		return -EINVAL;

	return 0;
}


#ifdef CONFIG_SERIAL_S3C2410_CONSOLE

static struct console s3c24xx_serial_console;

#define S3C24XX_SERIAL_CONSOLE &s3c24xx_serial_console
#else
#define S3C24XX_SERIAL_CONSOLE NULL
#endif

static struct uart_ops s3c24xx_serial_ops = {
	.pm		= s3c24xx_serial_pm,
	.tx_empty	= s3c24xx_serial_tx_empty,
	.get_mctrl	= s3c24xx_serial_get_mctrl,
	.set_mctrl	= s3c24xx_serial_set_mctrl,
	.stop_tx	= s3c24xx_serial_stop_tx,
	.start_tx	= s3c24xx_serial_start_tx,
	.stop_rx	= s3c24xx_serial_stop_rx,
	.enable_ms	= s3c24xx_serial_enable_ms,
	.break_ctl	= s3c24xx_serial_break_ctl,
	.startup	= s3c24xx_serial_startup,
	.shutdown	= s3c24xx_serial_shutdown,
	.set_termios	= s3c24xx_serial_set_termios,
	.type		= s3c24xx_serial_type,
	.release_port	= s3c24xx_serial_release_port,
	.request_port	= s3c24xx_serial_request_port,
	.config_port	= s3c24xx_serial_config_port,
	.verify_port	= s3c24xx_serial_verify_port,
};


static struct uart_driver s3c24xx_uart_drv = {
	.owner		= THIS_MODULE,
	.dev_name	= "s3c2410_serial",
	.nr		= NR_PORTS,
	.cons		= S3C24XX_SERIAL_CONSOLE,
	.driver_name	= S3C24XX_SERIAL_NAME,
	.devfs_name	= S3C24XX_SERIAL_DEVFS,
	.major		= S3C24XX_SERIAL_MAJOR,
	.minor		= S3C24XX_SERIAL_MINOR,
};

static struct s3c24xx_uart_port s3c24xx_serial_ports[NR_PORTS] = {
	[0] = {
		.port = {
			.lock		= SPIN_LOCK_UNLOCKED,
			.iotype		= UPIO_MEM,
			.irq		= 0,
			.uartclk	= 0,
			.custom_divisor	= 1,
			.fifosize	= 16,
			.ops		= &s3c24xx_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 0,
		}
	},
	[1] = {
		.port = {
			.lock		= SPIN_LOCK_UNLOCKED,
			.iotype		= UPIO_MEM,
			.irq		= 0,
			.uartclk	= 0,
			.custom_divisor	= 1,
			.fifosize	= 16,
			.ops		= &s3c24xx_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 1,
		}
	},
#if NR_PORTS > 2
	[2] = {
		.port = {
			.lock		= SPIN_LOCK_UNLOCKED,
			.iotype		= UPIO_MEM,
			.irq		= 0,
			.uartclk	= 0,
			.custom_divisor	= 1,
			.fifosize	= 16,
			.ops		= &s3c24xx_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 2,
		}
	},
#endif
#if NR_PORTS > 3
	[3] = {
		.port = {
			.lock		= SPIN_LOCK_UNLOCKED,
			.iotype		= UPIO_MEM,
			.irq		= 0,
			.uartclk	= 0,
			.custom_divisor	= 1,
			.fifosize	= 16,
			.ops		= &s3c24xx_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 3,
		}
	},
#endif
};

/* s3c24xx_serial_resetport
 *
 * wrapper to call the specific reset for this port (reset the fifos
 * and the settings)
*/

static inline int s3c24xx_serial_resetport(struct uart_port * port,
					   struct s3c2410_uartcfg *cfg)
{
	struct s3c24xx_uart_info *info = s3c24xx_port_to_info(port);

	return (info->reset_port)(port, cfg);
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
static int s3c24xx_port_in_use( struct s3c24xx_uart_port *port )
{
	int	index=(int) ((((unsigned long int) port) - ((unsigned long int) s3c24xx_serial_ports))/sizeof( *port ));

	/* Non-existing ports are never in use. */
	if( index > s3c24xx_uart_count )
		return 0;

	/* Current port is console? */
	if( s3c24xx_serial_console.index != index )
	{
		/* tx_claimed & rx_claimed -> port is in use. */
		if( port->tx_claimed && port->rx_claimed )
			return 1;
		else
			return 0;
	}
	else
	{
		/* Console port is ALWAYS in use. */
		return 1;
	}
}

unsigned long get_nearest_baud( unsigned long baud )
{
	unsigned long	maxbaud=921600;
	unsigned long	index=0;
	unsigned long	low_baud_limit, high_baud_limit;

	while( (maxbaud >> index) > 0 )
	{
		high_baud_limit=((maxbaud >> index) * 163)/160;
		low_baud_limit=((maxbaud >> index) * 160)/163;
		if( (baud <= high_baud_limit) && (baud >= low_baud_limit) ) return (maxbaud >> index);
		else index+=1;
	}
	return baud;
}

static unsigned long int s3c24xx_transition_disable_rts( struct s3c24xx_uart_port *ourport )
{
	unsigned long int	umcon=rd_regl( &(ourport->port), S3C2410_UMCON );

	/* disable AFC, deactivate nRTS (H) */
	wr_regl( &(ourport->port), S3C2410_UMCON, (umcon & ~(S3C2410_UMCOM_AFC | S3C2410_UMCOM_RTS_LOW)) );
	return umcon;
}

static void s3c24xx_transition_restore_rts( struct s3c24xx_uart_port *ourport, unsigned long int umcon )
{
	/* restore AFC, old nRTS state */
	wr_regl( &(ourport->port), S3C2410_UMCON, umcon );
	return;
}

static void s3c24xx_transition_drain_txfifo( struct s3c24xx_uart_port *ourport )
{
	unsigned long int	ufcon=rd_regl(&(ourport->port), S3C2410_UFCON);
	int			count=0;

	/* Wait while the port drains. */
	if (ufcon & S3C2410_UFCON_FIFOMODE)
	{
		while( ((rd_regl( &(ourport->port), S3C2410_UFSTAT ) & ourport->info->tx_fifomask) != 0) &&
			(count < 0xFFFF) )
		{
			count++;
			udelay( 5 );
		}
	}
	else
	{
		while( ((rd_regl( &(ourport->port), S3C2410_UTRSTAT ) & 0x02) != 0x02) && (count < 0xFFFF) )
		{
			count++;
			udelay( 5 );
		}
	}
	return;
}

static void s3c24xx_transition_drain_rxfifo( struct s3c24xx_uart_port *ourport )
{
	unsigned long int	ufcon=rd_regl(&(ourport->port), S3C2410_UFCON);
	int			count=0;

	/* Wait while the port drains. */
	if (ufcon & S3C2410_UFCON_FIFOMODE)
	{
		while( ((rd_regl( &(ourport->port), S3C2410_UFSTAT ) & ourport->info->rx_fifomask) != 0) &&
			(count < 0xFFFF) )
		{
			count++;
			udelay( 5 );
		}
	}
	else
	{
		while( ((rd_regl( &(ourport->port), S3C2410_UTRSTAT ) & 0x01) != 0x01) && (count < 0xFFFF) )
		{
			count++;
			udelay( 5 );
		}
	}
	return;
}

/*
 * CPU clock speed change handler. Change uart divisors, or refuse policy if ports in use with too high baudrate.
 */
void s3c24xx_transition_set_div( struct s3c24xx_uart_port *ourport, unsigned long int pclk_new,
				 unsigned long int curr_clk, int port_idx )
{
	unsigned long			baud;
	unsigned short			quot;
	struct s3c24xx_uart_clksrc	*clksrc=ourport->clksrc; 
	struct clk			*clk=ourport->baudclk;
	unsigned long int		new_baud=0;
	unsigned long int		result;
	unsigned long int		ubrdiv;
	unsigned long int		umcon;
	unsigned long int		ucon;
	unsigned int			slot;

	/* Check if info is NULL. If so, then the port is not initialized. Use 115k2 as speed. */
	if( (ourport->port.info == NULL) || (ourport->port.info->tty == 0) ||
	    (ourport->port.info->tty->termios == NULL) )
	{
		ubrdiv=rd_regl( &(ourport->port), S3C2410_UBRDIV ) + 1;
		if( ourport->info->has_slot )
		{
			slot=divslot2num( rd_regl( &(ourport->port), S3C2412_UDIVSLOT ) ); 
			baud=get_nearest_baud( curr_clk/(ubrdiv * 16 + slot) );
		}
		else
		{
			baud=get_nearest_baud( curr_clk/(ubrdiv * 16) );
		}
	}
	else
		baud = uart_get_baud_rate( &(ourport->port),
					   ourport->port.info->tty->termios,
					   NULL, 0, 115200*8 );

	/* Ensure that if this is pclk, the new pclk is set when using the serial_getclk routine. */
	/* Otherwise it will calculate using the OLD rate. */
	if( !strcmp( clksrc->name, "pclk" ) )
		clksrc->rate=pclk_new * 1000;

	/* Now it's safe to modify the baud rate. */
	slot=0;

	/* First determine the new custom divisor. */
	if( (baud == 38400) && ((ourport->port.flags & UPF_SPD_MASK) == UPF_SPD_CUST) )
	{
		ourport->port.custom_divisor=pclk_new * ourport->port.custom_divisor / curr_clk;
		quot=ourport->port.custom_divisor/16 - 1;
		if( ourport->port.flags & UPF_MAGIC_MULTIPLIER )
			slot=ourport->port.custom_divisor % 16;
		else
			slot=0;
	}
	else    
	{
		quot = s3c24xx_serial_getclk(&(ourport->port), &clksrc, &clk, &slot, baud);
		if( ourport->port.flags & UPF_MAGIC_MULTIPLIER )
			ourport->port.custom_divisor=(quot+1) * 16 + slot;
		else
			ourport->port.custom_divisor=quot+1;
	}

	/* check to see if we need  to change clock source */       
	if( ourport->clksrc != clksrc || ourport->baudclk != clk )
	{
		/* check to see if we need  to change clock source */       
		if( ourport->baudclk != NULL && !IS_ERR(ourport->baudclk) )
		{
			clk_unuse( ourport->baudclk );
			ourport->baudclk  = NULL;
		}

		clk_use(clk);

		ourport->clksrc = clksrc;
		ourport->baudclk = clk;
	}

	/* Print warning if we deviate too much. */
	if( !strcmp( clksrc->name, "pclk" ) )
	{
		/* Check if the clock deviates too much. Should not happen. */
		if( ourport->info->has_slot )
			new_baud=1000*pclk_new/(16 * (((unsigned long int) quot) + 1) + slot);
		else
			new_baud=1000*pclk_new/(16 * (((unsigned long int) quot) + 1));

		/* Max deviation is 3/160. */
		if( baud < new_baud ) result=new_baud - baud;
		else result=baud-new_baud;
		if( (160 * result/baud) >= 3 )
			printk( "WARNING! Serial clock for port %i deviates too much!\n", port_idx );
	}

	/* No IRQ now. Need to wait for transition. */
	disable_irq( TX_IRQ( &(ourport->port) ) );
	disable_irq( RX_IRQ( &(ourport->port) ) );

	/* Drain the fifo's and disable RTS before switching. */
	s3c24xx_transition_drain_rxfifo( ourport );
	umcon=s3c24xx_transition_disable_rts( ourport );
	s3c24xx_transition_drain_txfifo( ourport );

	/* Prepare for switching to another clocksource. */
	ucon=rd_regl( &(ourport->port), S3C2410_UCON );
	if( curr_clk > (1000 * pclk_new) )
	{
		s3c24xx_serial_setsource( &(ourport->port), clksrc );
		wr_regl(&(ourport->port), S3C2410_UBRDIV, quot );
		if( ourport->info->has_slot )
			wr_regl( &(ourport->port), S3C2412_UDIVSLOT, num2divslot( slot ) );
	}
	else
	{
		wr_regl(&(ourport->port), S3C2410_UBRDIV, quot );
		if( ourport->info->has_slot )
			wr_regl( &(ourport->port), S3C2412_UDIVSLOT, num2divslot( slot ) );
		s3c24xx_serial_setsource( &(ourport->port), clksrc );
	}

	/* Restore IRQ now. Need to wait for transition. */
	enable_irq( RX_IRQ( &(ourport->port) ) );
	enable_irq( TX_IRQ( &(ourport->port) ) );

	/* Set right clock rate in port struct. */
	if( ourport->port.flags & UPF_MAGIC_MULTIPLIER )
		ourport->port.uartclk=16*clk_get_rate( ourport->baudclk );
	else
		ourport->port.uartclk=clk_get_rate( ourport->baudclk );

	s3c24xx_transition_restore_rts( ourport, umcon );

	/* Done. */
	return;
}

static int s3c2412_set_temporary_clock( unsigned long int *transmask, unsigned long int pclk_old )
{
	struct clk			*uartclk=NULL;
	unsigned long			baud;
	unsigned long			ubrdiv;
	unsigned long			result;
	struct s3c24xx_uart_port	*ourport;
	unsigned long			ucon;
	int				count=0;
	unsigned long			umcon;
	unsigned long			newbaud;
	unsigned long int		flags;
	unsigned long			slot;

	/* Clear transmask. Anything that goes ok through this temporary transition is flagged. */
	*transmask=0;

	/* Cycle past all uarts, and try to switch them to the constant UARTCLK. */
	for( count=0; count < s3c24xx_uart_count; count++ )
	{
		ourport = &(s3c24xx_serial_ports[count]);

		spin_lock_irqsave( &(ourport->port.lock), flags );

		/* Only change if the port is in use. */
		if( !s3c24xx_port_in_use( ourport ) || strcmp( ourport->clksrc->name, "pclk" ) )
		{
			spin_unlock_irqrestore( &(ourport->port.lock), flags );
			continue;
		}

		/* Get the uartclk. This will be our temporary source during switchover. */
		uartclk=clk_get( ourport->port.dev, "uartclk" );
		if( IS_ERR( uartclk ) )
		{
			spin_unlock_irqrestore( &(ourport->port.lock), flags );
			return -1;
		}

		/* Ensure UARTCLK is enabled. */
		clk_enable( uartclk );

		/* Check if info is NULL. If so, then the port is not initialized. Use 115k2 as speed. */
		if( (ourport->port.info == NULL) || (ourport->port.info->tty == 0) ||
		    (ourport->port.info->tty->termios == NULL) )
		{
			ubrdiv=rd_regl( &(ourport->port), S3C2410_UBRDIV ) + 1;
			if( ourport->info->has_slot )
			{
				slot=divslot2num( rd_regl( &(ourport->port), S3C2412_UDIVSLOT ) ); 
				baud=get_nearest_baud( (pclk_old * 1000)/(ubrdiv * 16 + slot) );
			}
			else
			{
				baud=get_nearest_baud( (pclk_old * 1000)/(ubrdiv * 16) );
			}
		}
		else
			baud = uart_get_baud_rate( &(ourport->port),
						   ourport->port.info->tty->termios,
						   NULL, 0, 115200*8 );

		/* Calculate the nearest divider from 12 MHz. */
		slot=0;
		newbaud=(clk_get_rate( uartclk ) + baud/2)/baud;
		ubrdiv=newbaud/16;
		if( ubrdiv < 1 ) ubrdiv=1;
		if( ourport->info->has_slot )
		{
			slot=newbaud%16;
			newbaud=clk_get_rate( uartclk )/(16 * ubrdiv + slot);
		}
		else
		{
			newbaud=clk_get_rate( uartclk )/(16 * ubrdiv);
		}

		if( newbaud > baud ) result=newbaud - baud;
		else result=baud - newbaud;
		if( (160 * result/baud) >= 3 )
		{
			printk( "WARNING! Can't set temporary clock for port %i! Trying to disable it during transition. Expect artifacts!\n", count + 1 );
			/* Disable TX and RX. */
			uart_suspend_port(&s3c24xx_uart_drv, &(ourport->port));

			/* Disable the clocks. */
			if (!IS_ERR(ourport->baudclk) && ourport->baudclk != NULL)
				clk_disable(ourport->baudclk);

			clk_disable(ourport->clk);

			/* Flag as not enabled during transition. */
			*transmask&=~(1 << count);
		}
		else
		{
			/* Flag as enabled during transtition. */
			*transmask|=1 << count;

			/* No IRQ now. Need to wait for transition. */
			disable_irq( TX_IRQ( &(ourport->port) ) );
			disable_irq( RX_IRQ( &(ourport->port) ) );

			/* Temporary disable the port. */
			s3c24xx_transition_drain_rxfifo( ourport );
			umcon=s3c24xx_transition_disable_rts( ourport );
			s3c24xx_transition_drain_txfifo( ourport );

			/* Get the current state in UCON. */
			ucon=rd_regl( &(ourport->port), S3C2410_UCON );

			/* Go from higher to lower rate? Then from higher divider to lower divider. */
			if( (1000 * pclk_old) > clk_get_rate( uartclk ) )
			{
				/* Select the new (temporary) clocksource. */
				wr_regl( &(ourport->port), S3C2410_UCON, (ucon & ~S3C2412_UCON_CLKMASK) | S3C2412_UCON_UARTCLK );

				/* Set the new (temporary) divider. */
				wr_regl( &(ourport->port), S3C2410_UBRDIV, ubrdiv - 1 );

				if( ourport->info->has_slot )
					wr_regl( &(ourport->port), S3C2412_UDIVSLOT, num2divslot( slot ) );
			}
			else
			{
				/* Set the new (temporary) divider. */
				wr_regl( &(ourport->port), S3C2410_UBRDIV, ubrdiv - 1 );

				if( ourport->info->has_slot )
					wr_regl( &(ourport->port), S3C2412_UDIVSLOT, num2divslot( slot ) );

				/* Select the new (temporary) clocksource. */
				wr_regl( &(ourport->port), S3C2410_UCON, (ucon & ~S3C2412_UCON_CLKMASK) | S3C2412_UCON_UARTCLK );
			}

			/* Restore IRQ now. Need to wait for transition. */
			enable_irq( RX_IRQ( &(ourport->port) ) );
			enable_irq( TX_IRQ( &(ourport->port) ) );

			/* Restore RTS state. */
			s3c24xx_transition_restore_rts( ourport, umcon );

			/* Set right clock rate in port struct. */
			if( ourport->port.flags & UPF_MAGIC_MULTIPLIER )
			{
				ourport->port.custom_divisor=ubrdiv * 16 + slot;
				ourport->port.uartclk=16*clk_get_rate( uartclk );
			}
			else
			{
				ourport->port.custom_divisor=ubrdiv;
				ourport->port.uartclk=clk_get_rate( uartclk );
			}
		}

		/* Release the clock. */
		clk_put( uartclk );

		spin_unlock_irqrestore( &(ourport->port.lock), flags );
	}
	return 0;
}

static int s3c2412_restore_original_clock( unsigned long int transmask, unsigned long int pclk_new )
{
	struct clk			*uartclk=NULL;
	struct s3c24xx_uart_port	*ourport;
	int				count;
	unsigned long int		flags;

	/* Cycle through all serialports and reenable. */
	for( count=0; count < s3c24xx_uart_count; count++ )
	{
		ourport = &(s3c24xx_serial_ports[count]);

		spin_lock_irqsave( &(ourport->port.lock), flags );

		/* Only change if the port is in use. */
		if( !s3c24xx_port_in_use( ourport ) || strcmp( ourport->clksrc->name, "pclk" ) )
		{
			spin_unlock_irqrestore( &(ourport->port.lock), flags );
			continue;
		}

		/* Get the uartclk. This will be our temporary source during switchover. */
		uartclk=clk_get( ourport->port.dev, "uartclk" );
		if( IS_ERR( uartclk ) )
		{
			spin_unlock_irqrestore( &(ourport->port.lock), flags );
			return -1;
		}

		if( !(transmask & (1 << count)) )
		{
			uart_resume_port(&s3c24xx_uart_drv, &(ourport->port));
			printk( "Reenabling disabled port %i\n", count + 1 );
		}

		s3c24xx_transition_set_div( ourport, pclk_new, clk_get_rate( uartclk ), count + 1 );

		if( !(transmask & (1 << count)) )
		{
			/* Re-enable the clocks. */
			if (!IS_ERR(ourport->baudclk) && ourport->baudclk != NULL)
				clk_enable(ourport->baudclk);

			clk_enable( ourport->clk );
		}

		/* Release used clock. */
		clk_put( uartclk );

		spin_unlock_irqrestore( &(ourport->port.lock), flags );
	}
	return 0;
}

static int
s3c24xx_serial_freq_transition(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs		*f = data;
	unsigned long int		pclk_old;
	unsigned long int		pclk_new;
	static unsigned long int	transmask;

	f->trans2pclk( f, &pclk_old, &pclk_new );

	switch (val) {
	case CPUFREQ_PRECHANGE:
		/* Use temp clock. We'll shift back later. */
		s3c2412_set_temporary_clock( &transmask, pclk_old );
		break;

	case CPUFREQ_POSTCHANGE:
		/* Restore and set new. */
		s3c2412_restore_original_clock( transmask, pclk_new );
		break;
	}
	return 0;
}

static int
s3c24xx_serial_freq_policy(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_policy	*policy = data;
	unsigned long int	min_pclk=0;
	unsigned long int	curr_pclk;
	unsigned long long int	pclk_freq;
	unsigned long int	max_pclk=0xFFFFFFFF;
	unsigned long int	baud;
	unsigned long int	low_pclk;
	unsigned long int	high_pclk;
	int			count;

	policy->policy2pclk( policy, &low_pclk, &high_pclk );

	/* Determine the highest minimum and lowest maximum required by all ports. */
	for( count=0; count < s3c24xx_uart_count; count++ )
	{
		struct s3c24xx_uart_clksrc	tmpclk;

		/* First determine if this clock is derived from HCLK (PCLK) */
		s3c24xx_serial_ports[count].info->get_clksrc( &(s3c24xx_serial_ports[count].port), &tmpclk );

		if( strcmp( tmpclk.name, "pclk" ) ) continue;

		/* Check if the port is in use. If not, continue. */
		if( !s3c24xx_port_in_use( &(s3c24xx_serial_ports[count]) ) ) continue;

		/* Now get the baud rate. This determines what PCLK (and consequently HCLK) is supported. */
		/* Check if info is NULL. If so, then the port is not initialized. Use 115k2 as speed. */
		if( (s3c24xx_serial_ports[count].port.info == NULL) || (s3c24xx_serial_ports[count].port.info->tty == 0) ||
		    (s3c24xx_serial_ports[count].port.info->tty->termios == NULL) )
			baud=115200;
		else
			baud = uart_get_baud_rate( &(s3c24xx_serial_ports[count].port),
						   s3c24xx_serial_ports[count].port.info->tty->termios,
						   NULL, 0, 115200*8 );

		/* Find out what the required pclk for this port is. */
		pclk_freq=((unsigned long long int) baud) * 16 * 1;
		if( pclk_freq > 0xFFFFFFFF ) 
			curr_pclk=0xFFFFFFFF;
		else curr_pclk=(unsigned long int) pclk_freq;

		if( curr_pclk >= min_pclk ) min_pclk=curr_pclk;

		pclk_freq=((unsigned long long int) baud) * 16 * 65535;
		if( pclk_freq > 0xFFFFFFFF ) 
			curr_pclk=0xFFFFFFFF;
		else curr_pclk=(unsigned long int) pclk_freq;

		if( curr_pclk <= max_pclk ) max_pclk=curr_pclk;
	}

	/* min_pclk and max_pclk now contain the min/max values we can allow. */
	if( min_pclk == 0 )
		min_pclk=low_pclk;
	else
		min_pclk/=1000;

	if( max_pclk == 0xFFFFFFFF )
		max_pclk=high_pclk;
	else
		max_pclk/=1000;

	switch (val) {
	case CPUFREQ_ADJUST:
		policy->pclk2policy( policy, min_pclk, max_pclk );
		break;

	case CPUFREQ_INCOMPATIBLE:
		if( (low_pclk >= min_pclk) || (low_pclk <= max_pclk) )
			min_pclk=low_pclk;

		if( (high_pclk >= min_pclk) || (high_pclk <= max_pclk) )
			max_pclk=high_pclk;

		if( (high_pclk != max_pclk) || (low_pclk != min_pclk) )
			policy->pclk2policy( policy, min_pclk, max_pclk );
		break;
	case CPUFREQ_NOTIFY:
		/* Do nothing. We can't disable the serial port, just live with it. */
		break;
	}
	return 0;
}
#endif

/* s3c24xx_serial_init_port
 *
 * initialise a single serial port from the platform device given
 */

static int s3c24xx_serial_init_port(struct s3c24xx_uart_port *ourport,
				    struct s3c24xx_uart_info *info,
				    struct platform_device *platdev)
{
	struct uart_port *port = &ourport->port;
	struct s3c2410_uartcfg *cfg;
	struct resource *res;

	dbg("s3c24xx_serial_init_port: port=%p, platdev=%p\n", port, platdev);

	if (platdev == NULL)
		return -ENODEV;

	cfg = s3c24xx_dev_to_cfg(&platdev->dev);

	if (port->mapbase != 0)
		return 0;

	/* XXX: hwport should get its max from somewhere else */
	if (cfg->hwport >= NR_PORTS)
		return -EINVAL;

	/* setup info for port */
	port->dev	= &platdev->dev;
	ourport->info	= info;

	/* copy the info in from provided structure */
	ourport->port.fifosize = info->fifosize;

	dbg("s3c24xx_serial_init_port: %p (hw %d)...\n", port, cfg->hwport);

	port->uartclk = 1;
	port->custom_divisor=1;

	/* If we support the slotdiv trick, emulate we have magic multipliers. */
	if( info->has_slot )
		port->flags|=UPF_MAGIC_MULTIPLIER;

	if (cfg->uart_flags & UPF_CONS_FLOW) {
		dbg("s3c24xx_serial_init_port: enabling flow control\n");
		port->flags |= UPF_CONS_FLOW;
	}

	/* sort our the physical and virtual addresses for each UART */

	res = platform_get_resource(platdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR "failed to find memory resource for uart\n");
		return -EINVAL;
	}

	dbg("resource %p (%lx..%lx)\n", res, res->start, res->end);

	port->mapbase	= res->start;
	port->membase	= S3C24XX_VA_UART + (res->start - S3C2410_PA_UART);
	port->irq	= platform_get_irq(platdev, 0);

	ourport->clk	= clk_get(&platdev->dev, "uart");

	if (ourport->clk != NULL && !IS_ERR(ourport->clk))
		clk_use(ourport->clk);

	dbg("port: map=%08x, mem=%08x, irq=%d, clock=%ld\n",
	    port->mapbase, port->membase, port->irq, port->uartclk);

	/* set timer for modem */
	init_timer(&ourport->cts_poll_timer);
	ourport->cts_poll_timer.function = s3c24xx_timeout;
	ourport->cts_poll_timer.data     = (unsigned long) ourport;
	
	/* reset the fifos (and setup the uart) */
	s3c24xx_serial_resetport(port, cfg);
	return 0;
}

/* Device driver serial port probe */

static int probe_index = 0;

int s3c24xx_serial_probe(struct device *_dev,
			 struct s3c24xx_uart_info *info)
{
	struct s3c24xx_uart_port *ourport;
	struct platform_device *dev = to_platform_device(_dev);
	int ret;

	dbg("s3c24xx_serial_probe(%p, %p) %d\n", _dev, info, probe_index);

	ourport = &s3c24xx_serial_ports[probe_index];
	probe_index++;

	dbg("%s: initialising port %p...\n", __FUNCTION__, ourport);

	ret = s3c24xx_serial_init_port(ourport, info, dev);
	if (ret < 0)
		goto probe_err;

	dbg("%s: adding port\n", __FUNCTION__);
	uart_add_one_port(&s3c24xx_uart_drv, &ourport->port);
	dev_set_drvdata(_dev, &ourport->port);

	return 0;

 probe_err:
	return ret;
}

int s3c24xx_serial_remove(struct device *_dev)
{
	struct uart_port *port = s3c24xx_dev_to_port(_dev);

	if (port)
		uart_remove_one_port(&s3c24xx_uart_drv, port);

	return 0;
}

static void s3c24xx_serial_shutdown_drv(struct device *dev)
{
	struct uart_port *port = s3c24xx_dev_to_port(dev);

	dbg("Shutting down port %p\n", port);
	if (port)
		uart_suspend_port(&s3c24xx_uart_drv, port);
}

/* UART power management code */

#ifdef CONFIG_PM

int s3c24xx_serial_suspend(struct device *dev, pm_message_t state, u32 level)
{
	struct uart_port *port = s3c24xx_dev_to_port(dev);

	if (port && level == SUSPEND_DISABLE)
		uart_suspend_port(&s3c24xx_uart_drv, port);

	return 0;
}

int s3c24xx_serial_resume(struct device *dev, u32 level)
{
	struct uart_port *port = s3c24xx_dev_to_port(dev);
	struct s3c24xx_uart_port *ourport = to_ourport(port);

	if (port && level == RESUME_ENABLE) {
		clk_enable(ourport->clk);
		s3c24xx_serial_resetport(port, s3c24xx_port_to_cfg(port));
		clk_disable(ourport->clk);

		uart_resume_port(&s3c24xx_uart_drv, port);
	}

	return 0;
}

#else
#define s3c24xx_serial_suspend NULL
#define s3c24xx_serial_resume  NULL
#endif

int s3c24xx_serial_init(struct device_driver *drv,
			struct s3c24xx_uart_info *info)
{
	dbg("s3c24xx_serial_init(%p,%p)\n", drv, info);
	return driver_register(drv);
}


/* now comes the code to initialise either the s3c2410 or s3c2440 serial
 * port information
*/

/* cpu specific variations on the serial port support */

#ifdef CONFIG_CPU_S3C2400

static int s3c2400_serial_getsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	struct clk	*clock;
	clk->divisor = 1;
	clk->name = "pclk";
	clock=clk_get( port->dev, clksrc->name );
	if( clock == NULL || IS_ERR( clock ) )
		return -1;
 
	clk->rate=clk_get_rate( clock );
	clk_put( clock );

	return 0;
}

static int s3c2400_serial_setsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	return 0;
}

static int s3c2400_serial_resetport(struct uart_port *port,
				    struct s3c2410_uartcfg *cfg)
{
	unsigned long int	ucon;

	dbg("s3c2400_serial_resetport: port=%p (%08lx), cfg=%p\n",
	    port, port->mapbase, cfg);

	ucon=rd_regl( port, S3C2410_UCON );

	/* Ensure the state of the clock selection is restored. */
	wr_regl(port, S3C2410_UCON,  (cfg->ucon & ~S3C2410_UCON_UEXTCLK) | (ucon & S3C2410_UCON_UEXTCLK) );
	wr_regl(port, S3C2410_ULCON, cfg->ulcon);

	/* reset both fifos */

	wr_regl(port, S3C2410_UFCON, cfg->ufcon | S3C2410_UFCON_RESETBOTH);
	wr_regl(port, S3C2410_UFCON, cfg->ufcon);

	return 0;
}

static struct s3c24xx_uart_info s3c2400_uart_inf = {
	.name		= "Samsung S3C2400 UART",
	.type		= PORT_S3C2400,
	.fifosize	= 16,
	.rx_fifomask	= S3C2410_UFSTAT_RXMASK,
	.rx_fifoshift	= S3C2410_UFSTAT_RXSHIFT,
	.rx_fifofull	= S3C2410_UFSTAT_RXFULL,
	.tx_fifofull	= S3C2410_UFSTAT_TXFULL,
	.tx_fifomask	= S3C2410_UFSTAT_TXMASK,
	.tx_fifoshift	= S3C2410_UFSTAT_TXSHIFT,
	.has_slot	= 0;
	.get_clksrc	= s3c2400_serial_getsource,
	.set_clksrc	= s3c2400_serial_setsource,
	.reset_port	= s3c2400_serial_resetport,
};

static int s3c2400_serial_probe(struct device *dev)
{
	return s3c24xx_serial_probe(dev, &s3c2400_uart_inf);
}

static struct device_driver s3c2400_serial_drv = {
	.name		= "s3c2400-uart",
	.bus		= &platform_bus_type,
	.probe		= s3c2400_serial_probe,
	.remove		= s3c24xx_serial_remove,
	.suspend	= s3c24xx_serial_suspend,
	.resume		= s3c24xx_serial_resume,
};

static inline int s3c2400_serial_init(void)
{
	return s3c24xx_serial_init(&s3c2400_serial_drv, &s3c2400_uart_inf);
}

static inline void s3c2400_serial_exit(void)
{
	driver_unregister(&s3c2400_serial_drv);
}

#define s3c2400_uart_inf_at &s3c2400_uart_inf
#else

static inline int s3c2400_serial_init(void)
{
	return 0;
}

static inline void s3c2400_serial_exit(void)
{
}

#define s3c2400_uart_inf_at NULL

#endif /* CONFIG_CPU_S3C2400 */

/* S3C2410 support */

#ifdef CONFIG_CPU_S3C2410

static int s3c2410_serial_setsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);

	if (strcmp(clk->name, "uextclk") == 0)
		ucon |= S3C2410_UCON_UEXTCLK;
	else
		ucon &= ~S3C2410_UCON_UEXTCLK;

	wr_regl(port, S3C2410_UCON, ucon);
	return 0;
}

static int s3c2410_serial_getsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long	ucon = rd_regl(port, S3C2410_UCON);
	struct clk	*clock;

	clk->divisor = 1;
	clk->name = (ucon & S3C2410_UCON_UEXTCLK) ? "uextclk" : "pclk";

	clock=clk_get( port->dev, clk->name );
	if( clock == NULL || IS_ERR( clock ) )
		return -1;
 
	clk->rate=clk_get_rate( clock );
	clk_put( clock );

	return 0;
}

static int s3c2410_serial_resetport(struct uart_port *port,
				    struct s3c2410_uartcfg *cfg)
{
	unsigned long int	ucon;
	dbg("s3c2410_serial_resetport: port=%p (%08lx), cfg=%p\n",
	    port, port->mapbase, cfg);

	ucon=rd_regl( port, S3C2410_UCON );

	/* Ensure the state of the clock selection is restored. */
	wr_regl(port, S3C2410_UCON,  (cfg->ucon & ~S3C2410_UCON_UEXTCLK) | (ucon & S3C2410_UCON_UEXTCLK) );
	wr_regl(port, S3C2410_ULCON, cfg->ulcon);

	/* reset both fifos */

	wr_regl(port, S3C2410_UFCON, cfg->ufcon | S3C2410_UFCON_RESETBOTH);
	wr_regl(port, S3C2410_UFCON, cfg->ufcon);

	return 0;
}

static struct s3c24xx_uart_info s3c2410_uart_inf = {
	.name		= "Samsung S3C2410 UART",
	.type		= PORT_S3C2410,
	.fifosize	= 16,
	.rx_fifomask	= S3C2410_UFSTAT_RXMASK,
	.rx_fifoshift	= S3C2410_UFSTAT_RXSHIFT,
	.rx_fifofull	= S3C2410_UFSTAT_RXFULL,
	.tx_fifofull	= S3C2410_UFSTAT_TXFULL,
	.tx_fifomask	= S3C2410_UFSTAT_TXMASK,
	.tx_fifoshift	= S3C2410_UFSTAT_TXSHIFT,
	.has_slot	= 0,
	.get_clksrc	= s3c2410_serial_getsource,
	.set_clksrc	= s3c2410_serial_setsource,
	.reset_port	= s3c2410_serial_resetport,
};

/* device management */

static int s3c2410_serial_probe(struct device *dev)
{
	return s3c24xx_serial_probe(dev, &s3c2410_uart_inf);
}

static struct device_driver s3c2410_serial_drv = {
	.name		= "s3c2410-uart",
	.bus		= &platform_bus_type,
	.probe		= s3c2410_serial_probe,
	.remove		= s3c24xx_serial_remove,
	.shutdown	= s3c24xx_serial_shutdown_drv,
	.suspend	= s3c24xx_serial_suspend,
	.resume		= s3c24xx_serial_resume,
};

static inline int s3c2410_serial_init(void)
{
	return s3c24xx_serial_init(&s3c2410_serial_drv, &s3c2410_uart_inf);
}

static inline void s3c2410_serial_exit(void)
{
	driver_unregister(&s3c2410_serial_drv);
}

#define s3c2410_uart_inf_at &s3c2410_uart_inf
#else

static inline int s3c2410_serial_init(void)
{
	return 0;
}

static inline void s3c2410_serial_exit(void)
{
}

#define s3c2410_uart_inf_at NULL

#endif /* CONFIG_CPU_S3C2410 */

#ifdef CONFIG_CPU_S3C2412

static int s3c2412_serial_setsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);

	ucon &= ~S3C2412_UCON_CLKMASK;

	if (strcmp(clk->name, "uextclk") == 0)
		ucon |= S3C2412_UCON_UEXTCLK;
	else if (strcmp(clk->name, "pclk") == 0)
		ucon |= S3C2412_UCON_PCLK2;
	else if (strcmp(clk->name, "uartclk") == 0) // XXX TODO
		ucon |= S3C2412_UCON_UARTCLK;
	else {
		printk(KERN_ERR "unknown clock source %s\n", clk->name);
		return -EINVAL;
	}

	wr_regl(port, S3C2410_UCON, ucon);
	return 0;
}


static int s3c2412_serial_getsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long	ucon = rd_regl(port, S3C2410_UCON);
	struct clk	*clock;

	switch (ucon & S3C2412_UCON_CLKMASK) {
	case S3C2412_UCON_UEXTCLK:
		clk->divisor = 1;
		clk->name = "uextclk";
		break;

	case S3C2412_UCON_PCLK:
	case S3C2412_UCON_PCLK2:
		clk->divisor = 1;
		clk->name = "pclk";
		break;

	case S3C2412_UCON_UARTCLK: // XXX TODO
		clk->divisor = 1;
		clk->name = "uartclk";
		break;
	}

	clock=clk_get( port->dev, clk->name );
	if( clock == NULL || IS_ERR( clock ) )
		return -1;
 
	clk->rate=clk_get_rate( clock );
	clk_put( clock );

	return 0;
}

static int s3c2412_serial_resetport(struct uart_port *port,
				    struct s3c2410_uartcfg *cfg)
{
	unsigned long int	ucon;

	dbg("s3c2412_serial_resetport: port=%p (%08lx), cfg=%p\n",
	    port, port->mapbase, cfg);

	ucon=rd_regl( port, S3C2410_UCON );

	/* Ensure the state of the clock selection is restored. */
	wr_regl(port, S3C2410_UCON,  (cfg->ucon & ~S3C2412_UCON_CLKMASK) | (ucon & S3C2412_UCON_CLKMASK) );
	wr_regl(port, S3C2410_ULCON, cfg->ulcon);

	/* reset both fifos */

	wr_regl(port, S3C2410_UFCON, cfg->ufcon | S3C2410_UFCON_RESETBOTH);
	wr_regl(port, S3C2410_UFCON, cfg->ufcon);

	return 0;
}

static struct s3c24xx_uart_info s3c2412_uart_inf = {
	.name		= "Samsung S3C2412/3 UART",
	.type		= PORT_S3C2412,
	.fifosize	= 64,
	.rx_fifomask	= S3C2440_UFSTAT_RXMASK,
	.rx_fifoshift	= S3C2440_UFSTAT_RXSHIFT,
	.rx_fifofull	= S3C2440_UFSTAT_RXFULL,
	.tx_fifofull	= S3C2440_UFSTAT_TXFULL,
	.tx_fifomask	= S3C2440_UFSTAT_TXMASK,
	.tx_fifoshift	= S3C2440_UFSTAT_TXSHIFT,
	.has_slot	= 1,
	.get_clksrc	= s3c2412_serial_getsource,
	.set_clksrc	= s3c2412_serial_setsource,
	.reset_port	= s3c2412_serial_resetport,
};

/* device management */

static int s3c2412_serial_probe(struct device *dev)
{
	dbg("s3c2412_serial_probe: dev=%p\n", dev);
	return s3c24xx_serial_probe(dev, &s3c2412_uart_inf);
}

static struct device_driver s3c2412_serial_drv = {
	.name		= "s3c2412-uart",
	.bus		= &platform_bus_type,
	.probe		= s3c2412_serial_probe,
	.remove		= s3c24xx_serial_remove,
	.shutdown	= s3c24xx_serial_shutdown_drv,
	.suspend	= s3c24xx_serial_suspend,
	.resume		= s3c24xx_serial_resume,
};


static inline int s3c2412_serial_init(void)
{
	return s3c24xx_serial_init(&s3c2412_serial_drv, &s3c2412_uart_inf);
}

static inline void s3c2412_serial_exit(void)
{
	driver_unregister(&s3c2412_serial_drv);
}

#define s3c2412_uart_inf_at &s3c2412_uart_inf
#else

static inline int s3c2412_serial_init(void)
{
	return 0;
}

static inline void s3c2412_serial_exit(void)
{
}

#define s3c2412_uart_inf_at NULL
#endif /* CONFIG_CPU_S3C2412 */

#ifdef CONFIG_CPU_S3C2440

static int s3c2440_serial_setsource(struct uart_port *port,
				     struct s3c24xx_uart_clksrc *clk)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);

	// todo - proper fclk<>nonfclk switch //

	ucon &= ~S3C2440_UCON_CLKMASK;

	if (strcmp(clk->name, "uextclk") == 0)
		ucon |= S3C2440_UCON_UEXTCLK;
	else if (strcmp(clk->name, "pclk") == 0)
		ucon |= S3C2440_UCON_PCLK;
	else if (strcmp(clk->name, "fclk") == 0)
		ucon |= S3C2440_UCON_FCLK;
	else {
		printk(KERN_ERR "unknown clock source %s\n", clk->name);
		return -EINVAL;
	}

	wr_regl(port, S3C2410_UCON, ucon);
	return 0;
}


static int s3c2440_serial_getsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long	ucon = rd_regl(port, S3C2410_UCON);
	unsigned long	ucon0, ucon1, ucon2;
	struct clk	*clock;

	switch (ucon & S3C2440_UCON_CLKMASK) {
	case S3C2440_UCON_UEXTCLK:
		clk->divisor = 1;
		clk->name = "uextclk";
		break;

	case S3C2440_UCON_PCLK:
	case S3C2440_UCON_PCLK2:
		clk->divisor = 1;
		clk->name = "pclk";
		break;

	case S3C2440_UCON_FCLK:
		/* the fun of calculating the uart divisors on
		 * the s3c2440 */

		ucon0 = __raw_readl(S3C24XX_VA_UART0 + S3C2410_UCON);
		ucon1 = __raw_readl(S3C24XX_VA_UART1 + S3C2410_UCON);
		ucon2 = __raw_readl(S3C24XX_VA_UART2 + S3C2410_UCON);

		printk("ucons: %08lx, %08lx, %08lx\n", ucon0, ucon1, ucon2);

		ucon0 &= S3C2440_UCON0_DIVMASK;
		ucon1 &= S3C2440_UCON1_DIVMASK;
		ucon2 &= S3C2440_UCON2_DIVMASK;

		if (ucon0 != 0) {
			clk->divisor = ucon0 >> S3C2440_UCON_DIVSHIFT;
			clk->divisor += 6;
		} else if (ucon1 != 0) {
			clk->divisor = ucon1 >> S3C2440_UCON_DIVSHIFT;
			clk->divisor += 21;
		} else if (ucon2 != 0) {
			clk->divisor = ucon2 >> S3C2440_UCON_DIVSHIFT;
			clk->divisor += 36;
		} else {
			/* manual calims 44, seems to be 9 */
			clk->divisor = 9;
		}

		clk->name = "fclk";
		break;
	}

	clock=clk_get( port->dev, clk->name );
	if( clock == NULL || IS_ERR( clock ) )
		return -1;
 
	clk->rate=clk_get_rate( clock );
	clk_put( clock );
	return 0;
}

static int s3c2440_serial_resetport(struct uart_port *port,
				    struct s3c2410_uartcfg *cfg)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);

	dbg("s3c2440_serial_resetport: port=%p (%08lx), cfg=%p\n",
	    port, port->mapbase, cfg);

	/* ensure we don't change the clock settings... */

	ucon &= (S3C2440_UCON0_DIVMASK | (3<<10));

	wr_regl(port, S3C2410_UCON,  ucon | cfg->ucon);
	wr_regl(port, S3C2410_ULCON, cfg->ulcon);

	/* reset both fifos */

	wr_regl(port, S3C2410_UFCON, cfg->ufcon | S3C2410_UFCON_RESETBOTH);
	wr_regl(port, S3C2410_UFCON, cfg->ufcon);

	return 0;
}

static struct s3c24xx_uart_info s3c2440_uart_inf = {
	.name		= "Samsung S3C2440 UART",
	.type		= PORT_S3C2440,
	.fifosize	= 64,
	.rx_fifomask	= S3C2440_UFSTAT_RXMASK,
	.rx_fifoshift	= S3C2440_UFSTAT_RXSHIFT,
	.rx_fifofull	= S3C2440_UFSTAT_RXFULL,
	.tx_fifofull	= S3C2440_UFSTAT_TXFULL,
	.tx_fifomask	= S3C2440_UFSTAT_TXMASK,
	.tx_fifoshift	= S3C2440_UFSTAT_TXSHIFT,
	.has_slot	= 0,
	.get_clksrc	= s3c2440_serial_getsource,
	.set_clksrc	= s3c2440_serial_setsource,
	.reset_port	= s3c2440_serial_resetport,
};

/* device management */

static int s3c2440_serial_probe(struct device *dev)
{
	dbg("s3c2440_serial_probe: dev=%p\n", dev);
	return s3c24xx_serial_probe(dev, &s3c2440_uart_inf);
}

static struct device_driver s3c2440_serial_drv = {
	.name		= "s3c2440-uart",
	.bus		= &platform_bus_type,
	.probe		= s3c2440_serial_probe,
	.remove		= s3c24xx_serial_remove,
	.shutdown	= s3c24xx_serial_shutdown_drv,
	.suspend	= s3c24xx_serial_suspend,
	.resume		= s3c24xx_serial_resume,
};


static inline int s3c2440_serial_init(void)
{
	return s3c24xx_serial_init(&s3c2440_serial_drv, &s3c2440_uart_inf);
}

static inline void s3c2440_serial_exit(void)
{
	driver_unregister(&s3c2440_serial_drv);
}

#define s3c2440_uart_inf_at &s3c2440_uart_inf
#else

static inline int s3c2440_serial_init(void)
{
	return 0;
}

static inline void s3c2440_serial_exit(void)
{
}

#define s3c2440_uart_inf_at NULL
#endif /* CONFIG_CPU_S3C2440 */

#ifdef CONFIG_CPU_S3C2443

static int s3c2443_serial_setsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);

	ucon &= ~S3C2443_UCON_CLKMASK;

	if (strcmp(clk->name, "pclk") == 0)
		ucon |= S3C2443_UCON_PCLK;
	else if (strcmp(clk->name, "uextclk") == 0)
		ucon |= S3C2443_UCON_UEXTCLK;
	else if (strcmp(clk->name, "epllclk") == 0)
		ucon |= S3C2443_UCON_EPLLCLK;
	else {
		printk(KERN_ERR "unknown clock source %s\n", clk->name);
		return -EINVAL;
	}

	wr_regl(port, S3C2410_UCON, ucon);
	return 0;
}


static int s3c2443_serial_getsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long	ucon = rd_regl(port, S3C2410_UCON);
	struct clk	*clock;

	switch (ucon & S3C2443_UCON_CLKMASK) {
	case S3C2443_UCON_PCLK:
	case S3C2443_UCON_PCLK2:
		clk->divisor = 1;
		clk->name = "pclk";
		break;

	case S3C2443_UCON_UEXTCLK:
		clk->divisor = 1;
		clk->name = "uextclk";
		break;

	case S3C2443_UCON_EPLLCLK:
		clk->divisor = 1;
		clk->name = "epllclk";
		break;
	}

	clock=clk_get( port->dev, clk->name );
	if( clock == NULL || IS_ERR( clock ) )
		return -1;
 
	clk->rate=clk_get_rate( clock );
	clk_put( clock );

	return 0;
}

static int s3c2443_serial_resetport(struct uart_port *port,
				    struct s3c2410_uartcfg *cfg)
{
	unsigned long int	ucon;

	dbg("s3c2443_serial_resetport: port=%p (%08lx), cfg=%p\n",
	    port, port->mapbase, cfg);

	ucon=rd_regl( port, S3C2410_UCON );

	/* Ensure the state of the clock selection is restored. */
	wr_regl(port, S3C2410_UCON,  (cfg->ucon & ~S3C2440_UCON_CLKMASK) | (ucon & S3C2440_UCON_CLKMASK) );
	wr_regl(port, S3C2410_ULCON, cfg->ulcon);

	/* reset both fifos */

	wr_regl(port, S3C2410_UFCON, cfg->ufcon | S3C2410_UFCON_RESETBOTH);
	wr_regl(port, S3C2410_UFCON, cfg->ufcon);

	return 0;
}

static struct s3c24xx_uart_info s3c2443_uart_inf = {
	.name		= "Samsung S3C2443 UART",
	.type		= PORT_S3C2443,
	.fifosize	= 64,
	.rx_fifomask	= S3C2440_UFSTAT_RXMASK,
	.rx_fifoshift	= S3C2440_UFSTAT_RXSHIFT,
	.rx_fifofull	= S3C2440_UFSTAT_RXFULL,
	.tx_fifofull	= S3C2440_UFSTAT_TXFULL,
	.tx_fifomask	= S3C2440_UFSTAT_TXMASK,
	.tx_fifoshift	= S3C2440_UFSTAT_TXSHIFT,
	.has_slot	= 1,
	.get_clksrc	= s3c2443_serial_getsource,
	.set_clksrc	= s3c2443_serial_setsource,
	.reset_port	= s3c2443_serial_resetport,
};

/* device management */

static int s3c2443_serial_probe(struct device *dev)
{
	dbg("s3c2443_serial_probe: dev=%p\n", dev);
	return s3c24xx_serial_probe(dev, &s3c2443_uart_inf);
}

static struct device_driver s3c2443_serial_drv = {
	.name		= "s3c2443-uart",
	.bus		= &platform_bus_type,
	.probe		= s3c2443_serial_probe,
	.remove		= s3c24xx_serial_remove,
	.shutdown	= s3c24xx_serial_shutdown_drv,
	.suspend	= s3c24xx_serial_suspend,
	.resume		= s3c24xx_serial_resume,
};


static inline int s3c2443_serial_init(void)
{
	return s3c24xx_serial_init(&s3c2443_serial_drv, &s3c2443_uart_inf);
}

static inline void s3c2443_serial_exit(void)
{
	driver_unregister(&s3c2443_serial_drv);
}

#define s3c2443_uart_inf_at &s3c2443_uart_inf
#else

static inline int s3c2443_serial_init(void)
{
	return 0;
}

static inline void s3c2443_serial_exit(void)
{
}

#define s3c2443_uart_inf_at NULL
#endif /* CONFIG_CPU_S3C2443 */

/* module initialisation code */

static int __init s3c24xx_serial_modinit(void)
{
	int ret;

	ret = uart_register_driver(&s3c24xx_uart_drv);
	if (ret < 0) {
		printk(KERN_ERR "failed to register UART driver\n");
		return -1;
	}

	s3c2400_serial_init();
	s3c2410_serial_init();
	s3c2412_serial_init();
	s3c2440_serial_init();
	s3c2443_serial_init();

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	freq_transition.notifier_call = s3c24xx_serial_freq_transition;
	freq_transition.priority=CPUFREQ_ORDER_S3C24XX_SERIAL_PRIO;
	freq_policy.notifier_call = s3c24xx_serial_freq_policy;
	cpufreq_register_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	return 0;
}

static void __exit s3c24xx_serial_modexit(void)
{
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	cpufreq_unregister_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	s3c2400_serial_exit();
	s3c2410_serial_exit();
	s3c2412_serial_exit();
	s3c2440_serial_exit();
	s3c2443_serial_exit();

	uart_unregister_driver(&s3c24xx_uart_drv);
}


module_init(s3c24xx_serial_modinit);
module_exit(s3c24xx_serial_modexit);

/* Console code */

#ifdef CONFIG_SERIAL_S3C2410_CONSOLE

static struct uart_port *cons_uart;

static int
s3c24xx_serial_console_txrdy(struct uart_port *port, unsigned int ufcon)
{
	struct s3c24xx_uart_info *info = s3c24xx_port_to_info(port);
	unsigned long ufstat, utrstat;

	if (ufcon & S3C2410_UFCON_FIFOMODE) {
		/* fifo mode - check amount of data in fifo registers... */
		ufstat = rd_regl(port, S3C2410_UFSTAT);
		return (ufstat & info->tx_fifofull) ? 0 : 1;
	}

	/* in non-fifo mode, we go and use the tx buffer empty */

	utrstat = rd_regl(port, S3C2410_UTRSTAT);
	return (utrstat & S3C2410_UTRSTAT_TXE) ? 1 : 0;
}

static void
s3c24xx_serial_console_write(struct console *co, const char *s,
			     unsigned int count)
{
	int i;
	unsigned int ufcon = rd_regl(cons_uart, S3C2410_UFCON);

	for (i = 0; i < count; i++) {
		while (!s3c24xx_serial_console_txrdy(cons_uart, ufcon))
			barrier();

		wr_regb(cons_uart, S3C2410_UTXH, s[i]);

		if (s[i] == '\n') {
			while (!s3c24xx_serial_console_txrdy(cons_uart, ufcon))
				barrier();

			wr_regb(cons_uart, S3C2410_UTXH, '\r');
		}
	}
}

static void __init
s3c24xx_serial_get_options(struct uart_port *port, int *baud,
			   int *parity, int *bits)
{
	struct s3c24xx_uart_clksrc clksrc;
	struct clk *clk;
	unsigned int ulcon;
	unsigned int ucon;
	unsigned int ubrdiv;
	unsigned long rate;
	unsigned int udivslot;

	ulcon  = rd_regl(port, S3C2410_ULCON);
	ucon   = rd_regl(port, S3C2410_UCON);
	ubrdiv = rd_regl(port, S3C2410_UBRDIV);

	dbg("s3c24xx_serial_get_options: port=%p\n"
	    "registers: ulcon=%08x, ucon=%08x, ubdriv=%08x\n",
	    port, ulcon, ucon, ubrdiv);

	if ((ucon & 0xf) != 0) {
		/* consider the serial port configured if the tx/rx mode set */

		switch (ulcon & S3C2410_LCON_CSMASK) {
		case S3C2410_LCON_CS5:
			*bits = 5;
			break;
		case S3C2410_LCON_CS6:
			*bits = 6;
			break;
		case S3C2410_LCON_CS7:
			*bits = 7;
			break;
		default:
		case S3C2410_LCON_CS8:
			*bits = 8;
			break;
		}

		switch (ulcon & S3C2410_LCON_PMASK) {
		case S3C2410_LCON_PEVEN:
			*parity = 'e';
			break;

		case S3C2410_LCON_PODD:
			*parity = 'o';
			break;

		case S3C2410_LCON_PNONE:
		default:
			*parity = 'n';
		}

		/* now calculate the baud rate */

		s3c24xx_serial_getsource(port, &clksrc);

		clk = clk_get(port->dev, clksrc.name);
		if (!IS_ERR(clk) && clk != NULL)
			rate = clk_get_rate(clk) / clksrc.divisor;
		else
			rate = 1;

		if( s3c24xx_port_to_info(port)->has_slot )
		{
			udivslot = rd_regl( port, S3C2412_UDIVSLOT );
			*baud = rate / ( 16 * (ubrdiv + 1) + divslot2num( udivslot ) );
		}
		else
		{
			*baud = rate / ( 16 * (ubrdiv + 1));
		}
		dbg("calculated baud %d\n", *baud);
	}

}

/* s3c24xx_serial_init_ports
 *
 * initialise the serial ports from the machine provided initialisation
 * data.
*/

static int s3c24xx_serial_init_ports(struct s3c24xx_uart_info *info)
{
	struct s3c24xx_uart_port *ptr = s3c24xx_serial_ports;
	struct platform_device **platdev_ptr;
	int i;

	dbg("s3c24xx_serial_init_ports: initialising ports...\n");

	platdev_ptr = s3c24xx_uart_devs;

	for (i = 0; i < s3c24xx_uart_count; i++, ptr++, platdev_ptr++) {
		s3c24xx_serial_init_port(ptr, info, *platdev_ptr);
	}

	return 0;
}

static int __init
s3c24xx_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	dbg("s3c24xx_serial_console_setup: co=%p (%d), %s\n",
	    co, co->index, options);

	/* is this a valid port */

	if (co->index == -1 || co->index >= NR_PORTS)
		co->index = 0;

	port = &s3c24xx_serial_ports[co->index].port;

	/* is the port configured? */

	if (port->mapbase == 0x0) {
		co->index = 0;
		port = &s3c24xx_serial_ports[co->index].port;
	}

	cons_uart = port;

	dbg("s3c24xx_serial_console_setup: port=%p (%d)\n", port, co->index);

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		s3c24xx_serial_get_options(port, &baud, &parity, &bits);

	dbg("s3c24xx_serial_console_setup: baud %d\n", baud);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

/* s3c24xx_serial_initconsole
 *
 * initialise the console from one of the uart drivers
*/

static struct console s3c24xx_serial_console =
{
	.name		= S3C24XX_SERIAL_NAME,
	.device		= uart_console_device,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.write		= s3c24xx_serial_console_write,
	.setup		= s3c24xx_serial_console_setup
};

static int s3c24xx_serial_initconsole(void)
{
	struct s3c24xx_uart_info *info;
	struct platform_device *dev = s3c24xx_uart_devs[0];
	struct clk	*clock;

	dbg("s3c24xx_serial_initconsole\n");

	/* select driver based on the cpu */

	if (dev == NULL) {
		printk(KERN_ERR "s3c24xx: no devices for console init\n");
		return 0;
	}

	if (strcmp(dev->name, "s3c2400-uart") == 0) {
		info = s3c2400_uart_inf_at;
	} else if (strcmp(dev->name, "s3c2410-uart") == 0) {
		info = s3c2410_uart_inf_at;
	} else if (strcmp(dev->name, "s3c2412-uart") == 0) {
		info = s3c2412_uart_inf_at;
	} else if (strcmp(dev->name, "s3c2440-uart") == 0) {
		info = s3c2440_uart_inf_at;
	} else if (strcmp(dev->name, "s3c2443-uart") == 0) {
		info = s3c2443_uart_inf_at;
	} else {
		printk(KERN_ERR "s3c24xx: no driver for %s\n", dev->name);
		return 0;
	}

	if (info == NULL) {
		printk(KERN_ERR "s3c24xx: no driver for console\n");
		return 0;
	}

	/* Ensure (for console) that the struct for serial console is initialized. */
	clock=clk_get( &(dev->dev), tmp_clksrc.name );
	if( clock != NULL && !IS_ERR( clock ) )
	{
		tmp_clksrc.rate=clk_get_rate( clock );
		clk_put( clock );
	}

	s3c24xx_serial_console.data = &s3c24xx_uart_drv;
	s3c24xx_serial_init_ports(info);

	register_console(&s3c24xx_serial_console);
	return 0;
}

console_initcall(s3c24xx_serial_initconsole);

#endif /* CONFIG_SERIAL_S3C2410_CONSOLE */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>");
MODULE_DESCRIPTION("Samsung S3C24xx Serial port driver");
