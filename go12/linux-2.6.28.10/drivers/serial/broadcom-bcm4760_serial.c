/*
 *  linux/drivers/serial/broadcom-bcm4760_serial.c
 *
 *  Driver for Broadcom BCM4760 serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright 1999 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
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
 *
 * This is a generic driver for ARM AMBA-type serial ports as implemented
 * in the BCM4760. The hardware functionality is pretty much identica.
 * The functionality provided by this driver is however unique and is
 * not provided by the generic PL011 driver. These UART devices
 * have a lot of 16550-like features, but are not register compatible.
 * Note that although they do have CTS, DCD and DSR inputs, they do
 * not have an RI input, nor do they have DTR or RTS outputs.  If
 * required, these have to be supplied via some other means (eg, GPIO)
 * and hooked into this driver.
 *
 * Changelog:
 *
 * 19-Feb-2009  KOM  Support for GPS driver added and bugs fixed for refs #811:
 *                 FIXED::
 *                    - 01.001: Setting interrupt loop AMBA_ISR_PASS_LIMIT to 10 instead of 256 in bcm4760_uart_int()
 *                    - 01.002: Checking if TX interrupt is already started added
 *                    - 01.003: Stop sending added if TX FIFO is full in interrupt handler bcm4760_uart_tx_chars()
 *                    - 01.004: Reading RIS and mask this value from MIS instead of reading MIS 
 *                              because MIS has two masks RTIS & RXIS always and it means "if (1)" 
 *                              and "while (1)" in interrupt handler.
 *                    - 01.005: Setting hardware flow control added in bcm4760_uart_set_termios().
 *                    - 01.006: Init fifo size to 256 instead of 16 in bcm4760_uart_probe()
 *                 ADDED::    
 *                    - 02.001: Calculating RX/TX Statistic added.  
 *                    - 02.002: Calculating RX/TX Statistic commented out  
 *                              because "icount.dcd" is using for UART01x_FR_RXFE instead of UART01x_FR_DCD
 *                    - 02.003: Print statistic information to buffer added into bcm4760_uart_serial_get_port_info().
 *                              This function is called from GPS driver (/dev/gps)in gps.c  
 *                 FIXME:: 
 *                    - 03.001: Use 64 instead of 256 in interrupt handler bcm4760_uart_rx_chars() ? 
 *                    - 03.002: Need to debug maximum bytes (256) for sending to TX FIFO in TX interrupt handler.
 *                    - 03.003: Is this usage correct in bcm4760_uart_startup()? 
 *                              RX int FIFO & RTS level >= 1/2 full and TX int FIFO level <= 1/8 full ? 
 *                    - 03.004: Need to usage STANDBY instead of RESET ASIC in bcm4760_uart_set_termios()
 *                 DEBUG:: 
 *                    - DEBUG_IO: Debug IO TX/RX data added
 *                    - DEBUG_RTS_DTR_LBE_OUT1: Turn on/off pins RTS, DTR, LBE, OUT1
 *
 * 22-Feb-2009  KOM  GPS RESET removed in bcm4760_uart_set_termios() and moved to GPS driver
 *                      See FIXME 03.004 below.
 *
 * 25-Mar-2009  KOM  FIXME: Temporary fix. Need fix Startup bug in amba-pl011.c. 
 *                   - Apparently optimization is wrong therefore I am using external IO functions __read...() and __write...() from gps.c.  
 *                     I will debug it.
 *                   - restored AMBA_ISR_PASS_LIMIT to 256, not 10
 *                   - restored reading reg UART011_MIS in bcm4760_uart_int only
 *                       status = ___readw(uap->port.membase + UART011_MIS);
 *                   - debug printing added in bcm4760_uart_shutdown()
 *
 * 16-Jul-2009  JLH  Removed ___readw and ___writew as they shouldn't be needed.
 *                   Modified TX startup sequence to not use loopback mode as that was causing TX glitching
 *                   resulting in failed or unreliable communication with BT and perhaps GPS.
 *
 * 12-Aug-2009  JLH  Added suspend/resume functions.
 *
 */

#if defined(CONFIG_SERIAL_BROADCOM_BCM4760_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/amba/bus.h>
#include <linux/amba/serial.h>
#include <linux/clk.h>
#include <linux/delay.h>       
#include <asm/arch/gps.h>      

#include <asm/io.h>
#include <asm/sizes.h>

#define BCM4760_UART_MOD_DESCRIPTION	"BCM4760 UART Driver"
#define BCM4760_UART_MOD_VERSION		2.0

#define DEBUG_IO                0   // Debug IO TX/RX data
#define DEBUG_RTS_DTR_LBE_OUT1  0   // Turn on/off pins RTS, DTR, LBE, OUT1


#ifdef CONFIG_ARCH_BCMRING
#define UART_NR			2
#elif defined(CONFIG_PLAT_BCM476X)
#define UART_NR			4
#else
#define UART_NR			14
#endif


#if defined(CONFIG_ARCH_BCMRING) || defined(CONFIG_PLAT_BCM476X)
#define SERIAL_AMBA_MAJOR	4
#else
#define SERIAL_AMBA_MAJOR	204
#endif
#define SERIAL_AMBA_MINOR	64
#define SERIAL_AMBA_NR		UART_NR

//KOM FIXED 01.001:: Setting interrupt loop to 10 instead of 256 in irqreturn_t bcm4760_uart_int()
#define AMBA_ISR_PASS_LIMIT	    256

#define UART_DR_ERROR		(UART011_DR_OE|UART011_DR_BE|UART011_DR_PE|UART011_DR_FE)
#define UART_DUMMY_DR_RX	(1 << 16)

/*
 * Data structure to hold read/write configuration registers while
 * we're into suspend to RAM. Part of uart_amba_port structure and
 * not used separately.
 */
struct bcm4760_uart_suspend_data
{
	uint16_t	uart_ibrd;
	uint16_t	uart_fbrd;
	uint8_t		uart_lcr;
	uint8_t		uart_ifls;
	uint16_t	uart_cr;
	uint16_t	uart_imsc;
	uint16_t	uart_dmacr;
};

/*
 * We wrap our port structure around the generic uart_port.
 */
struct uart_amba_port {
	struct uart_port	port;
	struct clk		*clk;
	unsigned int		im;	/* interrupt mask */
	unsigned int		old_status;
	struct bcm4760_uart_suspend_data str_state;	/* register data to be preserved during suspend to RAM */
};

/* Defines */
#define PFX "amba-pl011: "
#define GPS_PORT 3

unsigned long port_total_icount0s[UART_NR];

//#define dbg(x...) printk(KERN_EMERG PFX x)
#define dbg(x...) do {} while (0)
//#define dbg(fmt, arg...) printk( KERN_DEBUG "%s: " fmt, __FUNCTION__ ,##arg); 

extern void gps_power(unsigned on);
extern void gps_reset(void);

static void bcm4760_uart_tx_chars(struct uart_amba_port *uap);


/*=====================================================================*/
/*========================= Serial Driver Functions ===================*/
/*=====================================================================*/


static void bcm4760_uart_stop_tx(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

#if DEBUG_IO
	if ( port->line == GPS_PORT )
	{
		dbg("CR=%04X\n", readw(uap->port.membase + UART011_CR));
	}
#endif
	uap->im &= ~UART011_TXIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
}

static void bcm4760_uart_start_tx(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

#if DEBUG_IO
	if ( port->line == GPS_PORT )
	{
		dbg("port=%d, CR=%04X, FR=%04X, RIS=%04X\n",
		    port->line,
		    readw(uap->port.membase + UART011_CR),
		    readw(uap->port.membase + UART01x_FR),
		    readw(uap->port.membase + UART011_RIS));
	}
#endif
	if ( (!(readw(uap->port.membase + UART011_CR) & UART011_CR_TXE)) ||
	     ((!(readw(uap->port.membase + UART01x_FR) & UART01x_FR_TXFF)) &&
	      (!(readw(uap->port.membase + UART011_RIS) & UART011_TXIS))) )
	{
#if DEBUG_IO
	    if ( port->line == GPS_PORT )
		{
			dbg("Kicking TX.\n");
		}
#endif
		uap->im &= ~UART011_TXIM;
		writew(uap->im, uap->port.membase + UART011_IMSC);
		writew(readw(uap->port.membase + UART011_CR) | UART011_CR_TXE,
				  uap->port.membase + UART011_CR);
		bcm4760_uart_tx_chars(uap);
	}
//	else
//	//KOM FIXED 01.002:: Checking if TX interrupt is already started added
//	if ( (uap->im & UART011_TXIM) == 0 ) 
	{ 
		uap->im |= UART011_TXIM;
		writew(uap->im, uap->port.membase + UART011_IMSC);
	} 
}

static void bcm4760_uart_stop_rx(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

	uap->im &= ~(UART011_RXIM|UART011_RTIM|UART011_FEIM|
		     UART011_PEIM|UART011_BEIM|UART011_OEIM);
	writew(uap->im, uap->port.membase + UART011_IMSC);
}

static void bcm4760_uart_enable_ms(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

	uap->im |= UART011_RIMIM|UART011_CTSMIM|UART011_DCDMIM|UART011_DSRMIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
}

static void bcm4760_uart_rx_chars(struct uart_amba_port *uap)
{
	struct tty_struct *tty = uap->port.info->port.tty;
	//KOM FIXME 03.001:: use 64 instead of 256 ?
	unsigned int status, ch, flag, max_count = 256;

#if DEBUG_IO
	char acBuf[1024],*pos; 
	struct uart_port *port = &uap->port;
	if ( port->line == GPS_PORT )
	{
	  pos = (char *)&acBuf[0];
      pos += sprintf(pos,"%s","BCM:H<-A ");
	}
#endif

	status = readw(uap->port.membase + UART01x_FR);
	//while ((status & UART01x_FR_RXFE) == 0 && max_count--) {
    while ( 1 ) {

	    //KOM ADDED 02.001:: Calculating statistic when RX FIFO is empty
		if ( status & UART01x_FR_RXFE) {
           uap->port.icount.dcd++;
		   break; 
		}
        max_count--;
		if ( max_count==0 ) {
           //KOM ADDED 02.001:: Calculating statistic when max count of bytes received
           uap->port.icount.dsr++;
		   break; 
		} 

		ch = readw(uap->port.membase + UART01x_DR) | UART_DUMMY_DR_RX;
		flag = TTY_NORMAL;
		uap->port.icount.rx++;

		/*
		 * Note that the error handling code is
		 * out of the main execution path
		 */
		if (unlikely(ch & UART_DR_ERROR)) {
			if (ch & UART011_DR_BE) {
				ch &= ~(UART011_DR_FE | UART011_DR_PE);
				uap->port.icount.brk++;
				if (uart_handle_break(&uap->port))
					goto ignore_char;
			} else if (ch & UART011_DR_PE)
				uap->port.icount.parity++;
			else if (ch & UART011_DR_FE)
				uap->port.icount.frame++;
			if (ch & UART011_DR_OE)
				uap->port.icount.overrun++;

			ch &= uap->port.read_status_mask;

			if (ch & UART011_DR_BE)
				flag = TTY_BREAK;
			else if (ch & UART011_DR_PE)
				flag = TTY_PARITY;
			else if (ch & UART011_DR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&uap->port, ch & 255))
			goto ignore_char;

#if DEBUG_IO
	    if ( port->line == GPS_PORT )
	    {
	       pos += sprintf(pos,"%04x ",ch);
	    } 
#endif

		uart_insert_char(&uap->port, ch, UART011_DR_OE, ch, flag);

	ignore_char:
		status = readw(uap->port.membase + UART01x_FR);
	}

#if DEBUG_IO
	if ( port->line == GPS_PORT )
	{
      *pos++ = '\n'; 
      *pos = 0; 
	  dbg("%s", acBuf);
	}
#endif

	spin_unlock(&uap->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&uap->port.lock);
}

static void bcm4760_uart_tx_chars(struct uart_amba_port *uap)
{
	struct circ_buf *xmit = &uap->port.info->xmit;
	int count;
	struct uart_port *port = &uap->port;
	//unsigned int ch;
#if DEBUG_IO
	char acBuf[1024],*pos; 
	if ( port->line == GPS_PORT )
	{
	  pos = (char *)&acBuf[0];
      pos += sprintf(pos,"%s","BCM:H->A ");
	}
#endif

	if (uap->port.x_char) {
		writew(uap->port.x_char, uap->port.membase + UART01x_DR);
		uap->port.icount.tx++;
		uap->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&uap->port)) {
		bcm4760_uart_stop_tx(&uap->port);
		return;
	}

	//KOM FIXME 03.002:: Need to debug maximum bytes (256) for sending to TX FIFO in TX interrupt handler
	// count = uap->port.fifosize;
	count = uap->port.fifosize >> 1;

	do {
		//KOM FIXED 01.003:: Stop sending added if TX FIFO is full 
		//    ADDED 02.001:: Calculating statistic when TX FIFO is full
        unsigned int status = readw(port->membase + UART01x_FR);
        if (status & UART01x_FR_TXFF )
	    {
	      port->icount.buf_overrun++;
	      break;
	    }  

		//KOM ADDED 02.001:: Calculating statistic when nUARTCTS is high
    	if ( (status & UART01x_FR_CTS) == 0 )
            uap->port.icount.cts++;

		//KOM ADDED 02.002:: Calculating statistic when nUARTDCD is high commented out
        //if ( (status & UART01x_FR_DCD) == 0 )
        //  uap->port.icount.dcd++;

		writew(xmit->buf[xmit->tail], uap->port.membase + UART01x_DR);

#if DEBUG_IO
		if ( port->line == GPS_PORT )
	    {
	      pos += sprintf(pos,"%02x ",xmit->buf[xmit->tail]);
	    } 
#endif
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		uap->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

#if DEBUG_IO
    if ( port->line == GPS_PORT )
	{
      *pos++ = '\n'; 
      *pos = 0; 
	  dbg("%s", acBuf);
	}
#endif

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&uap->port);

	if (uart_circ_empty(xmit))
		bcm4760_uart_stop_tx(&uap->port);
}

static void bcm4760_uart_modem_status(struct uart_amba_port *uap)
{
	unsigned int status, delta;

	status = readw(uap->port.membase + UART01x_FR) & UART01x_FR_MODEM_ANY;

	delta = status ^ uap->old_status;
	uap->old_status = status;

	if (!delta)
		return;

	if (delta & UART01x_FR_DCD)
		uart_handle_dcd_change(&uap->port, status & UART01x_FR_DCD);

	if (delta & UART01x_FR_CTS)
		uart_handle_cts_change(&uap->port, status & UART01x_FR_CTS);

	wake_up_interruptible(&uap->port.info->delta_msr_wait);
}

static irqreturn_t bcm4760_uart_int(int irq, void *dev_id)
{
	struct uart_amba_port *uap = dev_id;
	unsigned int status, pass_counter = AMBA_ISR_PASS_LIMIT;
	int handled = 0;

	spin_lock(&uap->port.lock);

    //KOM FIXED 01.004::
	//      reading UART011_RIS and mask this value from UART011_MIS instead of 
	//      reading UART011_MIS because UART011_MIS has two masks UART011_RTIS & UART011_RXIS and 
	//      it means "if (1)" and "while (1)"
	//      commented out: status = readw(uap->port.membase + UART011_MIS);
	//status = readw(uap->port.membase + UART011_RIS);
	status = readw(uap->port.membase + UART011_MIS);
	if (status) { 
		do {
			writew(status & ~(UART011_TXIS|UART011_RTIS|
					  UART011_RXIS),
			       uap->port.membase + UART011_ICR);

			if (status & (UART011_RTIS|UART011_RXIS))
			{
				bcm4760_uart_rx_chars(uap);
			}
			if (status & (UART011_DSRMIS|UART011_DCDMIS|
				      UART011_CTSMIS|UART011_RIMIS))
			{  
				bcm4760_uart_modem_status(uap);
			} 
			if (status & UART011_TXIS)
			{ 
				//KOM ADDED 02.001:: Calculating statistic when bit TXIS is high in UART011_RIS
                uap->port.icount.rng++;

				bcm4760_uart_tx_chars(uap);
			}

			if (pass_counter-- == 0)
			{
				break;
			}

			//KOM FIXED 01.004:: See above
	        status = readw(uap->port.membase + UART011_MIS);
	        //status = readw(uap->port.membase + UART011_RIS);
			//status &= readw(uap->port.membase + UART011_MIS);   
		} while (status != 0);
		handled = 1;
	}

	spin_unlock(&uap->port.lock);

	return IRQ_RETVAL(handled);
}

static unsigned int pl01x_tx_empty(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned int status = readw(uap->port.membase + UART01x_FR);
	return status & (UART01x_FR_BUSY|UART01x_FR_TXFF) ? 0 : TIOCSER_TEMT;
}

static unsigned int pl01x_get_mctrl(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned int result = 0;
	unsigned int status = readw(uap->port.membase + UART01x_FR);

#define TIOCMBIT(uartbit, tiocmbit)	\
	if (status & uartbit)		\
		result |= tiocmbit

	TIOCMBIT(UART01x_FR_DCD, TIOCM_CAR);
	TIOCMBIT(UART01x_FR_DSR, TIOCM_DSR);
	TIOCMBIT(UART01x_FR_CTS, TIOCM_CTS);
	TIOCMBIT(UART011_FR_RI, TIOCM_RNG);
#undef TIOCMBIT
	return result;
}

static void bcm4760_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned int cr;

	cr = readw(uap->port.membase + UART011_CR);

#define	TIOCMBIT(tiocmbit, uartbit)		\
	if (mctrl & tiocmbit)		\
		cr |= uartbit;		\
	else				\
		cr &= ~uartbit

	TIOCMBIT(TIOCM_RTS, UART011_CR_RTS);
	TIOCMBIT(TIOCM_DTR, UART011_CR_DTR);
	TIOCMBIT(TIOCM_OUT1, UART011_CR_OUT1);
	TIOCMBIT(TIOCM_OUT2, UART011_CR_OUT2);
	TIOCMBIT(TIOCM_LOOP, UART011_CR_LBE);
#undef TIOCMBIT

	writew(cr, uap->port.membase + UART011_CR);
}

static void bcm4760_uart_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned long flags;
	unsigned int lcr_h;

	spin_lock_irqsave(&uap->port.lock, flags);
	lcr_h = readw(uap->port.membase + UART011_LCRH);
	if (break_state == -1)
		lcr_h |= UART01x_LCRH_BRK;
	else
		lcr_h &= ~UART01x_LCRH_BRK;
	writew(lcr_h, uap->port.membase + UART011_LCRH);
	spin_unlock_irqrestore(&uap->port.lock, flags);
}

#ifdef CONFIG_CONSOLE_POLL
static int pl010_get_poll_char(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned int status;

	do {
		status = readw(uap->port.membase + UART01x_FR);
	} while (status & UART01x_FR_RXFE);

	return readw(uap->port.membase + UART01x_DR);
}

static void pl010_put_poll_char(struct uart_port *port,
			 unsigned char ch)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

	while (readw(uap->port.membase + UART01x_FR) & UART01x_FR_TXFF)
		barrier();

	writew(ch, uap->port.membase + UART01x_DR);
}

#endif /* CONFIG_CONSOLE_POLL */

static int bcm4760_uart_startup(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned int cr;
	int retval;

	dbg("bcm4760_uart_startup: port=%p (%08lx,%p), line=%u\n",
	    port,port->mapbase, port->membase, port->line);

	/*
	 * Try to enable the clock producer.
	 */
	retval = clk_enable(uap->clk);
	if (retval)
		goto out;

	uap->port.uartclk = clk_get_rate(uap->clk);

	dbg("requesting irq...clock=%ld\n",uap->port.uartclk);

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(uap->port.irq, bcm4760_uart_int, 0, "uart-pl011", uap);
	if (retval) {
		printk(KERN_ERR "cannot get irq %d\n", uap->port.irq);
		goto clk_dis;
	}

	//KOM FIXME 03.003:: Is this usage correct?
	//     RX interrupt FIFO level & RTS level >= 1/2 full
	//     TX interrupt FIFO level <= 1/8 full
	//     Original values: UART011_IFLS_RX4_8|UART011_IFLS_TX4_8
	writew(UART011_IFLS_RX4_8|UART011_IFLS_TX2_8,
	       uap->port.membase + UART011_IFLS);

	/*
	 * Provoke TX FIFO interrupt into asserting.
	 */
#if 0
	{ 
	  cr = UART01x_CR_UARTEN | UART011_CR_TXE | UART011_CR_LBE;
	  writew(cr, uap->port.membase + UART011_CR);
	  writew(0, uap->port.membase + UART011_FBRD);
	  writew(1, uap->port.membase + UART011_IBRD);
	  writew(0, uap->port.membase + UART011_LCRH);
	
	  writew(0, uap->port.membase + UART01x_DR);
	  while (readw(uap->port.membase + UART01x_FR) & UART01x_FR_BUSY)
		 barrier();
	}
#endif
//	cr = UART01x_CR_UARTEN | UART011_CR_RXE | UART011_CR_TXE;
	cr = UART01x_CR_UARTEN | UART011_CR_RXE;
	writew(cr, uap->port.membase + UART011_CR);

	/*
	 * initialise the old status of the modem signals
	 */
	uap->old_status = readw(uap->port.membase + UART01x_FR) & UART01x_FR_MODEM_ANY;

	/*
	 * Finally, enable interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = UART011_RXIM | UART011_RTIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);

	dbg("bcm4760_uart_startup ok, IM=%08lx CR=%08lx\n",uap->im,cr);

	spin_unlock_irq(&uap->port.lock);

	return 0;

 clk_dis:
	clk_disable(uap->clk);
 out:
	return retval;
}


static void bcm4760_uart_shutdown(struct uart_port *port)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;
	unsigned long val;

	val = readw(uap->port.membase + UART011_IMSC);
	dbg("bcm4760_uart_shutdown: port=%p (%08lx,%p),IMSC=%08lx im=%08lx line=%u\n",
	    port,port->mapbase, port->membase, val,uap->im, port->line);

	/*
	 * disable all interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = 0;
	writew(uap->im, uap->port.membase + UART011_IMSC);
	writew(0xffff, uap->port.membase + UART011_ICR);
	spin_unlock_irq(&uap->port.lock);

	/*
	 * Free the interrupt
	 */
	free_irq(uap->port.irq, uap);

	/*
	 * disable the port
	 */
	writew(UART01x_CR_UARTEN | UART011_CR_TXE, uap->port.membase + UART011_CR);

	/*
	 * disable break condition and fifos
	 */
	val = readw(uap->port.membase + UART011_LCRH);
	val &= ~(UART01x_LCRH_BRK | UART01x_LCRH_FEN);
	writew(val, uap->port.membase + UART011_LCRH);

	/*
	 * Shut down the clock producer
	 */
	clk_disable(uap->clk);
}

#if DEBUG_RTS_DTR_LBE_OUT1
#define DBG_CR_LBE     0x08000000    //0x001
//#define DBG_LCRH_FEN   0x10000000  //removed
#define DBG_TIOCM_DTR  0x20000000    //0x002
#define DBG_TIOCM_RTS  0x40000000    //0x004
#define DBG_TIOCM_OUT1 0x80000000    //0x2000 
#endif

static void
bcm4760_uart_set_termios(struct uart_port *port, struct ktermios *termios,
		     struct ktermios *old)
{
	unsigned int lcr_h, old_cr;
	unsigned long flags;
	unsigned int baud, quot;

	dbg("bcm4760_uart_set_termios: port=%d\n",port->line);

	//KOM FIXME 03.004:: Need to usage STANDBY instead of RESET ASIC
    //if ( port->line == GPS_PORT )
	//{ 
    //  gps_reset();
	//} 

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = port->uartclk * 4 / baud;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		dbg("config: 5bits/char\n");
		lcr_h = UART01x_LCRH_WLEN_5;
		break;
	case CS6:
        dbg("config: 6bits/char\n");
		lcr_h = UART01x_LCRH_WLEN_6;
		break;
	case CS7:
		dbg("config: 7bits/char\n");
		lcr_h = UART01x_LCRH_WLEN_7;
		break;
	default: // CS8
		dbg("config: 8bits/char\n");
		lcr_h = UART01x_LCRH_WLEN_8;
		break;
	}
	if (termios->c_cflag & CSTOPB)
		lcr_h |= UART01x_LCRH_STP2;
	if (termios->c_cflag & PARENB) {
		lcr_h |= UART01x_LCRH_PEN;
		if (!(termios->c_cflag & PARODD))
			lcr_h |= UART01x_LCRH_EPS;
	}
	if (port->fifosize > 1) {
	   lcr_h |= UART01x_LCRH_FEN;
	}

	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART011_DR_OE | 255;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART011_DR_FE | UART011_DR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART011_DR_BE;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART011_DR_FE | UART011_DR_PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART011_DR_BE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART011_DR_OE;
	}

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_DUMMY_DR_RX;

	if (UART_ENABLE_MS(port, termios->c_cflag))
		bcm4760_uart_enable_ms(port);

	/* first, disable everything */
	old_cr = readw(port->membase + UART011_CR);
	old_cr &= ~UART011_CR_TXE;

	dbg("bcm4760_uart_set_termios: c_cflag=0x%08x, CRTSCTS=0x%08x old_cr=0x%08x\n",termios->c_cflag,termios->c_cflag & CRTSCTS,old_cr);

    //if ( port->line == GPS_PORT )
	{
//       old_cr |= UART011_CR_TXE | UART011_CR_RXE | UART01x_CR_UARTEN;
       old_cr |= UART011_CR_RXE | UART01x_CR_UARTEN;
       old_cr |=  ( UART011_CR_RTS | UART011_CR_DTR );

	   //KOM FIXED 01.005:: Setting hardware flow control added.
	   //old_cr |= (termios->c_cflag & CRTSCTS) ? (UART011_CR_CTSEN | UART011_CR_RTSEN ) : 0;
	   if ( termios->c_cflag & CRTSCTS )
	   {
		  old_cr |=  ( UART011_CR_CTSEN | UART011_CR_RTSEN );
	   } 
#if DEBUG_RTS_DTR_LBE_OUT1
	   else {

          old_cr &= ~( UART011_CR_CTSEN | UART011_CR_RTSEN ); 

          //DEBUG only
		  old_cr &= ~( UART011_CR_RTS | UART011_CR_DTR );

          //DEBUG only
   	      if ( termios->c_oflag & DBG_TIOCM_RTS )
            old_cr |=  UART011_CR_RTS; 
	      else 
            old_cr &= ~( UART011_CR_RTS);

		  dbg("bcm4760_uart_set_termios.1: c_oflag=0x%08x, TIOCM_RTS=0x%08x old_cr=0x%08x\n",termios->c_oflag,termios->c_oflag & DBG_TIOCM_RTS,old_cr);

   	      if ( termios->c_oflag & DBG_TIOCM_DTR )
            old_cr |=  UART011_CR_DTR; 
	      else 
            old_cr &= ~( UART011_CR_DTR);

          dbg("bcm4760_uart_set_termios.2: c_oflag=0x%08x, TIOCM_DTR=0x%08x old_cr=0x%08x\n",termios->c_oflag,termios->c_oflag & DBG_TIOCM_DTR,old_cr);

   	      if ( termios->c_oflag & DBG_CR_LBE )
            old_cr |=  UART011_CR_LBE; 
	      else 
            old_cr &= ~( UART011_CR_LBE);

          dbg("bcm4760_uart_set_termios.3: c_oflag=0x%08x, CR_LBE=0x%08x old_cr=0x%08x\n",termios->c_oflag,termios->c_oflag & DBG_CR_LBE,old_cr);
	   }

	   if ( termios->c_oflag & DBG_TIOCM_OUT1 )
         old_cr |=  UART011_CR_OUT1; 
	   else 
         old_cr &= ~( UART011_CR_OUT1);

       dbg("bcm4760_uart_set_termios.4: c_oflag=0x%08x, TIOCM_OUT1=0x%08x old_cr=0x%08x\n",termios->c_oflag,termios->c_oflag & DBG_TIOCM_OUT1,old_cr);

#endif  //DEBUG_RTS_DTR_LBE_OUT1

	   //old_cr |= (UART011_CR_CTSEN | UART011_CR_RTSEN );
	}

	writew(0, port->membase + UART011_CR);

	/* Set baud rate */
	writew(quot & 0x3f, port->membase + UART011_FBRD);
	writew(quot >> 6, port->membase + UART011_IBRD);

	/*
	 * ----------v----------v----------v----------v-----
	 * NOTE: MUST BE WRITTEN AFTER UARTLCR_M & UARTLCR_L
	 * ----------^----------^----------^----------^-----
	 */
	writew(lcr_h, port->membase + UART011_LCRH);
	writew(old_cr, port->membase + UART011_CR);

	port_total_icount0s[port->line] = 0xffffffff;
	memset(&port->icount,0,sizeof(port->icount)); //clear statistic

	dbg("setting LCRH to %08x, baud %d, brddiv to FBRD=%08x, IBRD=%08x\n",
		 lcr_h, baud, quot & 0x3f,quot >> 6);

	dbg("bcm4760_uart_set_termios: uart: LCRH = 0x%08x, CR = 0x%08x, IFLS = 0x%08x, IMSC = 0x%08x\n",
		 readw(port->membase + UART011_LCRH),
		 readw(port->membase + UART011_CR),
	     readw(port->membase + UART011_IFLS),
		 readw(port->membase + UART011_IMSC));

	spin_unlock_irqrestore(&port->lock, flags);
}


static const char *bcm4760_uart_type(struct uart_port *port)
{
	return port->type == PORT_AMBA ? "AMBA/PL011" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'
 */
static void pl010_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, SZ_4K);
}

/*
 * Request the memory region(s) being used by 'port'
 */
static int pl010_request_port(struct uart_port *port)
{
	return request_mem_region(port->mapbase, SZ_4K, "uart-pl011")
			!= NULL ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void pl010_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_AMBA;
		pl010_request_port(port);
	}
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int pl010_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_AMBA)
		ret = -EINVAL;
	if (ser->irq < 0 || ser->irq >= nr_irqs)
		ret = -EINVAL;
	if (ser->baud_base < 9600)
		ret = -EINVAL;
	return ret;
}


static struct uart_ops amba_bcm4760_uart_pops = {
	.tx_empty	= pl01x_tx_empty,
	.set_mctrl	= bcm4760_uart_set_mctrl,
	.get_mctrl	= pl01x_get_mctrl,
	.stop_tx	= bcm4760_uart_stop_tx,
	.start_tx	= bcm4760_uart_start_tx,
	.stop_rx	= bcm4760_uart_stop_rx,
	.enable_ms	= bcm4760_uart_enable_ms,
	.break_ctl	= bcm4760_uart_break_ctl,
	.startup	= bcm4760_uart_startup,
	.shutdown	= bcm4760_uart_shutdown,
	.set_termios	= bcm4760_uart_set_termios,
	.type		= bcm4760_uart_type,
	.release_port	= pl010_release_port,
	.request_port	= pl010_request_port,
	.config_port	= pl010_config_port,
	.verify_port	= pl010_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char = pl010_get_poll_char,
	.poll_put_char = pl010_put_poll_char,
#endif
};

static struct uart_amba_port *amba_ports[UART_NR];

/* KOM ADDED 02.003:: 
 *     Print statistic information to buffer. This function is called from GPS driver (/dev/gps)in gps.c 
 */
int bcm4760_uart_serial_get_port_info(int line,char *szBuf)
{
    register char *pos;
	struct uart_port *port = &amba_ports[line]->port;
	unsigned long total_icount0 = port_total_icount0s[line];

    if ( szBuf == NULL ) return 0;
    pos = (char *)szBuf;
    if ( port )
    {
        unsigned long current_icount = (unsigned long)port->icount.brk +
			                           (unsigned long)port->icount.parity + 
									   (unsigned long)port->icount.frame +
                                       (unsigned long)port->icount.overrun +
									   (unsigned long)port->icount.tx +
                                       (unsigned long)port->icount.cts +
									   (unsigned long)port->icount.dsr +
		                               (unsigned long)port->icount.dcd +
		                               (unsigned long)port->icount.buf_overrun;

		if ( current_icount != total_icount0 )
		{ 
           pos += sprintf(pos,"bcm4760_uart_serial_get_port_info: port->line=%d\n",port->line);
           pos += sprintf(pos,"RX: icnt=%lu be=%lu pe=%lu fe=%lu oe=%lu\n",
		   (unsigned long)port->icount.rx,
		   (unsigned long)port->icount.brk,
		   (unsigned long)port->icount.parity,
		   (unsigned long)port->icount.frame,
		   (unsigned long)port->icount.overrun);  
           pos += sprintf(pos,"TX: icnt=%lu cts=%lu rxmax=%lu rxfe=%lu ff=%lu txis=%lu\n",
		   (unsigned long)port->icount.tx,
		   (unsigned long)port->icount.cts,
		   (unsigned long)port->icount.dsr,
		   (unsigned long)port->icount.dcd,
		   (unsigned long)port->icount.buf_overrun,
		   (unsigned long)port->icount.rng);

		   port_total_icount0s[line] = current_icount;
		}
	} 

    return pos - szBuf;
}	

#ifdef CONFIG_SERIAL_BROADCOM_BCM4760_CONSOLE

static void bcm4760_uart_console_putchar(struct uart_port *port, int ch)
{
	struct uart_amba_port *uap = (struct uart_amba_port *)port;

	while (readw(uap->port.membase + UART01x_FR) & UART01x_FR_TXFF)
		barrier();
	writew(ch, uap->port.membase + UART01x_DR);
}

static void
bcm4760_uart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_amba_port *uap = amba_ports[co->index];
	unsigned int status, old_cr, new_cr;

	clk_enable(uap->clk);

	/*
	 *	First save the CR then disable the interrupts
	 */
	old_cr = readw(uap->port.membase + UART011_CR);
	new_cr = old_cr & ~UART011_CR_CTSEN;
	new_cr |= UART01x_CR_UARTEN | UART011_CR_TXE;
	writew(new_cr, uap->port.membase + UART011_CR);

	uart_console_write(&uap->port, s, count, bcm4760_uart_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the TCR
	 */
	do {
		status = readw(uap->port.membase + UART01x_FR);
	} while (status & UART01x_FR_BUSY);
	writew(old_cr, uap->port.membase + UART011_CR);

	clk_disable(uap->clk);
}

static void __init
bcm4760_uart_console_get_options(struct uart_amba_port *uap, int *baud,
			     int *parity, int *bits)
{
	if (readw(uap->port.membase + UART011_CR) & UART01x_CR_UARTEN) {
		unsigned int lcr_h, ibrd, fbrd;

		lcr_h = readw(uap->port.membase + UART011_LCRH);

		*parity = 'n';
		if (lcr_h & UART01x_LCRH_PEN) {
			if (lcr_h & UART01x_LCRH_EPS)
				*parity = 'e';
			else
				*parity = 'o';
		}

		if ((lcr_h & 0x60) == UART01x_LCRH_WLEN_7)
			*bits = 7;
		else
			*bits = 8;

		ibrd = readw(uap->port.membase + UART011_IBRD);
		fbrd = readw(uap->port.membase + UART011_FBRD);

		*baud = uap->port.uartclk * 4 / (64 * ibrd + fbrd);
	}
}

static int __init bcm4760_uart_console_setup(struct console *co, char *options)
{
	struct uart_amba_port *uap;
	int baud = 38400;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	dbg("bcm4760_uart_console_setup: co=%p (%d), %s\n",co, co->index, options);

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= UART_NR)
		co->index = 0;
	uap = amba_ports[co->index];
	if (!uap)
		return -ENODEV;

	uap->port.uartclk = clk_get_rate(uap->clk);

	dbg("bcm4760_uart_console_setup: port=%p (%d)\n", &uap->port, co->index);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		bcm4760_uart_console_get_options(uap, &baud, &parity, &bits);

	dbg("bcm4760_uart_console_setup: baud %d\n", baud);

	return uart_set_options(&uap->port, co, baud, parity, bits, flow);
}

static struct uart_driver amba_reg;
static struct console amba_console = {
#if defined(CONFIG_ARCH_BCMRING) || defined(CONFIG_PLAT_BCM476X)
	.name		= "ttyS",
#else
	.name		= "ttyAMA",
#endif
	.write		= bcm4760_uart_console_write,
	.device		= uart_console_device,
	.setup		= bcm4760_uart_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &amba_reg,
};

#define AMBA_CONSOLE	(&amba_console)
#else
#define AMBA_CONSOLE	NULL
#endif

static struct uart_driver amba_reg = {
	.owner			= THIS_MODULE,
#if defined(CONFIG_ARCH_BCMRING) || defined(CONFIG_PLAT_BCM476X)
	.driver_name		= "ttyS",
	.dev_name		= "ttyS",
#else
	.driver_name		= "ttyAMA",
	.dev_name		= "ttyAMA",
#endif
	.major			= SERIAL_AMBA_MAJOR,
	.minor			= SERIAL_AMBA_MINOR,
	.nr			= UART_NR,
	.cons			= AMBA_CONSOLE,
};


static int bcm4760_uart_probe(struct amba_device *dev, void *id)
{
	struct uart_amba_port *uap;
	void __iomem *base;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(amba_ports); i++)
		if (amba_ports[i] == NULL)
			break;

	if (i == ARRAY_SIZE(amba_ports)) {
		ret = -EBUSY;
		goto out;
	}

	uap = kzalloc(sizeof(struct uart_amba_port), GFP_KERNEL);
	if (uap == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	base = ioremap(dev->res.start, PAGE_SIZE);
	if (!base) {
		ret = -ENOMEM;
		goto free;
	}

	uap->clk = clk_get(&dev->dev, "UARTCLK");
	if (IS_ERR(uap->clk)) {
		ret = PTR_ERR(uap->clk);
		goto unmap;
	}

	uap->port.dev = &dev->dev;
	uap->port.mapbase = dev->res.start;
	uap->port.membase = base;
	uap->port.iotype = UPIO_MEM;
	uap->port.irq = dev->irq[0];
	uap->port.fifosize = 256;         //KOM FIXED 01.006:: Init fifo size to 256 instead of 16
	uap->port.ops = &amba_bcm4760_uart_pops;
	uap->port.flags = UPF_BOOT_AUTOCONF;
	uap->port.line = i;

	amba_ports[i] = uap;

	{ 
	  struct uart_port *port = &uap->port;
	  dbg("bcm4760_uart_probe: port=%p, id=%p\n", port, id);

	  dbg("resource %p (%lx..%lx)\n", &dev->res, dev->res.start, dev->res.end);

	  dbg("bcm4760_uart_probe: line=%u map=%08x, mem=%08x, irq=%d, clock=%ld\n",
	       port->line, port->mapbase, port->membase, port->irq, port->uartclk);
	  port_total_icount0s[port->line] = 0;
	}

	amba_set_drvdata(dev, uap);
	ret = uart_add_one_port(&amba_reg, &uap->port);
	if (ret) {
		amba_set_drvdata(dev, NULL);
		amba_ports[i] = NULL;
		clk_put(uap->clk);

 unmap:
		iounmap(base);
 free:
		kfree(uap);
	}
 out:
	return ret;
}

static int bcm4760_uart_remove(struct amba_device *dev)
{
	struct uart_amba_port *uap = amba_get_drvdata(dev);
	int i;

	amba_set_drvdata(dev, NULL);

	uart_remove_one_port(&amba_reg, &uap->port);

	for (i = 0; i < ARRAY_SIZE(amba_ports); i++)
		if (amba_ports[i] == uap)
			amba_ports[i] = NULL;

	iounmap(uap->port.membase);
	clk_put(uap->clk);
	kfree(uap);
	return 0;
}

/* @brief bcm4760_uart_suspend
 *
 * The suspend function unrolls all UART configuration registers
 * into a RAM based structure. For this data we've extended the
 * uart_amba_port structure to include the register values we
 * need to preserve since this structure is allocation kzalloced.
 */
int bcm4760_uart_suspend(struct amba_device *dev, pm_message_t pm)
{
	struct uart_amba_port *uap = amba_get_drvdata(dev);

	dbg("Suspend processed for %s.\n", pdev->name); 

	uap->str_state.uart_ibrd = readw(uap->port.membase + UART011_IBRD);
	uap->str_state.uart_fbrd = readw(uap->port.membase + UART011_FBRD);
	uap->str_state.uart_lcr = readb(uap->port.membase + UART011_LCRH);
	uap->str_state.uart_ifls = readb(uap->port.membase + UART011_IFLS);
	uap->str_state.uart_cr = readw(uap->port.membase + UART011_CR);
	uap->str_state.uart_imsc = readw(uap->port.membase + UART011_IMSC);
	uap->str_state.uart_dmacr = readw(uap->port.membase + UART011_DMACR);

	return 0;
} /* bcm4760_uart_suspend */

/* @brief bcm4760_uart_resume
 *
 * The resume function loads the values preserved in the RAM based
 * structure back into the UART registers to reconfigure the device
 * back to the correct operational state.
 */
int	bcm4760_uart_resume(struct amba_device *dev)
{
	struct uart_amba_port *uap = amba_get_drvdata(dev);

	dbg("Suspend processed for %s.\n", pdev->name); 

	writew(uap->str_state.uart_ibrd, uap->port.membase + UART011_IBRD);
	writew(uap->str_state.uart_fbrd, uap->port.membase + UART011_FBRD);
	writeb(uap->str_state.uart_lcr, uap->port.membase + UART011_LCRH);
	writeb(uap->str_state.uart_ifls, uap->port.membase + UART011_IFLS);
	writew(uap->str_state.uart_cr, uap->port.membase + UART011_CR);
	writew(uap->str_state.uart_imsc, uap->port.membase + UART011_IMSC);
	writew(uap->str_state.uart_dmacr, uap->port.membase + UART011_DMACR);

	return 0;
} /* bcm4760_uart_resume */

static struct amba_id bcm4760_uart_ids[] __initdata = {
	{
		.id	= 0x00041011,
		.mask	= 0x000fffff,
	},
	{ 0, 0 },
};

static struct amba_driver bcm4760_uart_driver = {
	.drv = {
		.name	= "uart-bcm4760",
	},
	.id_table	= bcm4760_uart_ids,
	.probe		= bcm4760_uart_probe,
	.remove		= bcm4760_uart_remove,
#ifdef CONFIG_PM
	.suspend    = bcm4760_uart_suspend,
	.resume     = bcm4760_uart_resume,
#endif
};

static int __init bcm4760_uart_init(void)
{
	int ret;
	printk(KERN_INFO "Serial: BCM4760 UART driver\n");

	ret = uart_register_driver(&amba_reg);
	if (ret == 0) {
		ret = amba_driver_register(&bcm4760_uart_driver);
		if (ret)
			uart_unregister_driver(&amba_reg);
	}
	return ret;
}

static void __exit bcm4760_uart_exit(void)
{
	amba_driver_unregister(&bcm4760_uart_driver);
	uart_unregister_driver(&amba_reg);
}

module_init(bcm4760_uart_init);
module_exit(bcm4760_uart_exit);

MODULE_AUTHOR("Broadcom/ARM Ltd/Deep Blue Solutions Ltd");
MODULE_VERSION(BCM4760_UART_MOD_VERSION);
MODULE_DESCRIPTION(BCM4760_UART_MOD_DESCRIPTION);
MODULE_LICENSE("GPL");
