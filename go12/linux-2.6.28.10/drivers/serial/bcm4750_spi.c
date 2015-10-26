/*
 * linux/drivers/serial/bcm4750.c
 *
 * Driver for BCM4750 SPI UART Emulation Interface
 *
 * Rogier Stam, (c) 2008 TomTom International B.V
 *	http://www.tomtom.com
 *
 * Changelog:
 *
 * Driver comments
 * This driver uses spi_sync. Unfortunately this uses the complete( ) function to wait for completion, something
 * which takes roughly 10us when CONFIG_PREEMPT is not enabled. Therefore, this driver should be partially
 * optimized by using spi_async, and use its complete callback function to fire the work to be done/deal with status.
 * in the IRQ handler we could do an spi_async call and use the callback function for doing the work now done in
 * txrx_work. Because of these large delays we cannot have the maximum bandwidth.
*/
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
#include <linux/spi/spi.h>
#include <linux/kthread.h>
#include <linux/circ_buf.h>
#include <linux/bcm4750.h>

#include <asm/io.h>
#include <asm/irq.h>

#if 0
#include <asm/hardware.h>
#include <asm/mach-types.h>
#endif
#include <mach/gpio.h>

#ifdef CONFIG_SERIAL_BCM4750_EMU_DMA
#include <linux/dma-mapping.h>
#endif

struct bcm4750_message
{
	unsigned char			cmd_stat;
	unsigned char			data[18];
} __attribute__((__packed__));

/* structures */
struct bcm4750_uart_port
{
	int				rx_enabled;
	int				tx_enabled;
	struct ktermios			termios;
	spinlock_t			lock;
	struct uart_port		port;
	struct work_struct		rxtx_work;
	struct work_struct		start_tx;
	struct work_struct		stop_rx;
	struct work_struct		stop_tx;
	struct workqueue_struct		*serial_wq;
	atomic_t			bh_scheduled;

        struct spi_transfer             master_transfer;
};

/* UART name and device definitions */
#define BCM4750_SERIAL_NAME	"ttyBCM"
#define BCM4750_SERIAL_MAJOR	205
#define BCM4750_SERIAL_MINOR	0

struct uart_driver		bcm4750_uart_driver=
{
	.owner			= THIS_MODULE,
	.driver_name		= BCM4750_SERIAL_NAME,
	.dev_name		= BCM4750_SERIAL_NAME,
	.nr			= 1,
	.major			= BCM4750_SERIAL_MAJOR,
	.minor			= BCM4750_SERIAL_MINOR,
	.cons			= NULL,
};

static struct spi_driver	bcm4750_spi_driver;
static struct uart_ops		bcm4750_serial_ops;
static unsigned char bcm4750_read_status( struct bcm4750_uart_port *bcm_port );
static unsigned char bcm4750_write_control( struct bcm4750_uart_port *bcm_port, unsigned char control );
static unsigned char bcm4750_write_receive( struct bcm4750_uart_port *bcm_port, int length );
static unsigned char bcm4750_read_transmit( struct bcm4750_uart_port *bcm_port, int *length );
static irqreturn_t bcm4750_serial_irq( int irq, void *pdata );

static inline struct spi_device *bcm4750_uart_to_spidevice( struct bcm4750_uart_port *bcm_port )
{
	return to_spi_device( bcm_port->port.dev );
}

static inline unsigned char *bcm4750_get_write_buf( struct bcm4750_uart_port *bcm_port )
{
	struct bcm4750_message	*mesg=(struct bcm4750_message *) bcm_port->master_transfer.tx_buf;

	return mesg->data;
}

static inline unsigned char *bcm4750_get_read_buf( struct bcm4750_uart_port *bcm_port )
{
	struct bcm4750_message	*mesg=(struct bcm4750_message *) bcm_port->master_transfer.rx_buf;

	return mesg->data;
}

static unsigned int bcm4750_get_mctrl( struct uart_port *port )
{
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void bcm4750_set_mctrl( struct uart_port *port, unsigned int mctrl )
{
	return;
}

static unsigned int bcm4750_tx_empty( struct uart_port *port )
{
	if( uart_circ_chars_pending( &port->info->xmit ) == 0 )
		return TIOCSER_TEMT;
	else
		return 0;
}

static void bcm4750_stop_tx_work( struct work_struct *work )
{
	struct bcm4750_uart_port	*bport=container_of( work, struct bcm4750_uart_port, stop_tx );
	unsigned long int		flags;
	unsigned char			control=0;
	unsigned char			status;
	
	status=bcm4750_read_status( bport );

	spin_lock_irqsave( &(bport->lock), flags );

	if( bport->tx_enabled )
	{
		/* To prevent constant interrupts, disable the IRQ once no longer in use. */
		if( !bport->rx_enabled && !atomic_read( &(bport->bh_scheduled) ) )
			disable_irq( bport->port.irq );

		bport->tx_enabled=0;
	}

	if( bport->rx_enabled ) control |= 0x10; 
	spin_unlock_irqrestore( &(bport->lock), flags );

	/* Write back to the control register. */
	bcm4750_write_control( bport, (status & 0xC0) | control );
	bcm4750_write_control( bport, control );
	return;
}

static void bcm4750_stop_tx( struct uart_port *port )
{
	struct bcm4750_uart_port	*bport=container_of( port, struct bcm4750_uart_port, port );

	queue_work( bport->serial_wq, &(bport->stop_tx) );
	return;
}

static void bcm4750_start_tx_work( struct work_struct *work )
{
	struct bcm4750_uart_port	*bport=container_of( work, struct bcm4750_uart_port, start_tx );
	unsigned long int		flags;
	unsigned char			control=0x20;
	unsigned char			status;

	/* Initialize the device. */
	status=bcm4750_read_status( bport );

	spin_lock_irqsave( &(bport->lock), flags );
	if( bport->rx_enabled ) control |= 0x10; 

	/* We're starting up. Enable the IRQ. If RX was disabled, this means that the IRQ was also. Otherwise, don't enable the IRQ. */
	if( !bport->tx_enabled )
	{
		bport->tx_enabled=1;
		if( !bport->rx_enabled && !atomic_read( &(bport->bh_scheduled) ) )
			enable_irq( bport->port.irq );
	}
	spin_unlock_irqrestore( &(bport->lock), flags );

	/* Write back to the control register. */
	bcm4750_write_control( bport, (status & 0xC0) | control );
	bcm4750_write_control( bport, control );
	return;
}

static void bcm4750_start_tx( struct uart_port *port )
{
	struct bcm4750_uart_port	*bport=container_of( port, struct bcm4750_uart_port, port );

	queue_work( bport->serial_wq, &(bport->start_tx) );

	return;
}

static void bcm4750_stop_rx_work( struct work_struct *work )
{
	struct bcm4750_uart_port	*bport=container_of( work, struct bcm4750_uart_port, stop_rx );
	unsigned long int		flags;
	unsigned char			control=0;
	unsigned char			status;

	status=bcm4750_read_status( bport );

	spin_lock_irqsave( &(bport->lock), flags );
	/* To prevent constant interrupts, disable the IRQ once no longer in use. */
	if( bport->rx_enabled )
	{
		if( !bport->tx_enabled && !atomic_read( &(bport->bh_scheduled) ) )
			disable_irq( bport->port.irq );
		bport->rx_enabled=0;
	}
	if( bport->tx_enabled ) control |= 0x20; 

	spin_unlock_irqrestore( &(bport->lock), flags );

	/* Write back to the control register. */
	bcm4750_write_control( bport, (status & 0xC0) | control );
	bcm4750_write_control( bport, control );
	return;
}

static void bcm4750_stop_rx( struct uart_port *port )
{
	struct bcm4750_uart_port	*bport=container_of( port, struct bcm4750_uart_port, port );

	queue_work( bport->serial_wq, &(bport->stop_rx) );
	return;
}

static void bcm4750_enable_ms(struct uart_port *port)
{
	return;
}

static void bcm4750_break_ctl(struct uart_port *port, int break_state)
{
	return;
}

static int bcm4750_startup( struct uart_port *port )
{
	struct bcm4750_uart_port	*bport=container_of( port, struct bcm4750_uart_port, port );
	unsigned long int		flags;

	spin_lock_irqsave( &(bport->lock), flags );
	atomic_set( &(bport->bh_scheduled), 0 );
	bport->rx_enabled=1;
	bport->tx_enabled=0;
	enable_irq( bport->port.irq );
	spin_unlock_irqrestore( &(bport->lock), flags );
	return 0;
}

static void bcm4750_shutdown( struct uart_port *port )
{
	struct bcm4750_uart_port	*bport=container_of( port, struct bcm4750_uart_port, port );
	unsigned long int		flags;

	spin_lock_irqsave( &(bport->lock), flags );
	if( bport->rx_enabled )
		bcm4750_stop_rx( port );

	if( bport->tx_enabled )
		bcm4750_stop_tx( port );

	spin_unlock_irqrestore( &(bport->lock), flags );
	flush_workqueue( bport->serial_wq );
	return;
}

static void bcm4750_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old )
{
	struct bcm4750_uart_port	*bport=container_of( port, struct bcm4750_uart_port, port );

	/* Termios field is fake. It simply accepts whatever is set, then ignores it. */
	memcpy( &(bport->termios), termios, sizeof( bport->termios ) );

	/* Set the same timeout as for a 115200 port. It doesn't matter. As long as it doesnt hang us up. */
	uart_update_timeout( port, CS8, 115200 );
	return;
}

static const char *bcm4750_type(struct uart_port *port)
{
	return "BCM4750_EMU";
}

static void bcm4750_release_port(struct uart_port *port)
{
	return;
}

static int bcm4750_request_port(struct uart_port *port)
{
	return 0;
}

static void bcm4750_config_port(struct uart_port *port, int flags)
{
	return;
}

static int bcm4750_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if( port->type == PORT_16550A )
		return 0;
	else
		return -EINVAL;
}

/* returns status. */
static unsigned char bcm4750_transfer( struct bcm4750_uart_port *bport, unsigned char addr, int length, int use_dma )
{
	struct bcm4750_message		*bcm4750_write_msg=(struct bcm4750_message *) (bport->master_transfer.tx_buf);
	struct bcm4750_message		*bcm4750_read_msg=(struct bcm4750_message *) (bport->master_transfer.rx_buf);
        struct spi_message              msg;
        struct spi_transfer             transfer;

	if( length > sizeof( bcm4750_write_msg->data ) )
		return 0xFF;

        /* Prepare the data. */
        memcpy( &transfer, &(bport->master_transfer), sizeof( transfer ) );
        spi_message_init( &msg );

	if( use_dma )
	{
        	msg.is_dma_mapped=1;
	}
	else
	{
		transfer.tx_dma=0;
		transfer.rx_dma=0;
        	msg.is_dma_mapped=0;
	}

	bcm4750_write_msg->cmd_stat=addr;
	transfer.len=length + 1;
	spi_message_add_tail( &transfer, &msg );
        msg.spi=bcm4750_uart_to_spidevice( bport );
	if( spi_sync( msg.spi, &msg ) )
		return 0xFF;
	else
		return bcm4750_read_msg->cmd_stat;
}

static unsigned char bcm4750_read( struct bcm4750_uart_port *bcm_port, unsigned char addr, int length )
{
	unsigned char	*write_buf=bcm4750_get_write_buf( bcm_port );

	memset( write_buf, 0, length );
#ifdef CONFIG_SERIAL_BCM4750_EMU_DMA
	return bcm4750_transfer( bcm_port, addr, length, (length > 1 ? 1 : 0) );
#else
	return bcm4750_transfer( bcm_port, addr, length, 0 );
#endif
}

static unsigned char bcm4750_write( struct bcm4750_uart_port *bcm_port, unsigned char addr, int length )
{
#ifdef CONFIG_SERIAL_BCM4750_EMU_DMA
	return bcm4750_transfer( bcm_port, addr, length, (length > 1 ? 1 : 0) );
#else
	return bcm4750_transfer( bcm_port, addr, length, 0 );
#endif
}

static unsigned char bcm4750_write_control( struct bcm4750_uart_port *bcm_port, unsigned char control )
{
	unsigned char	*write_buf=bcm4750_get_write_buf( bcm_port );

	write_buf[0]=control;
	return bcm4750_write( bcm_port, 0x00, 1 );
}

static unsigned char bcm4750_read_status( struct bcm4750_uart_port *bcm_port )
{
	return bcm4750_read( bcm_port, 0x80, 0 );
}

static unsigned char bcm4750_read_transmit( struct bcm4750_uart_port *bcm_port, int *length )
{
	unsigned char	status=bcm4750_read_status( bcm_port );
	unsigned char	retval;

	*length=status & 0x1F;

	if( *length != 0 )
		retval=bcm4750_read( bcm_port, 0xc0, *length );
	else
		retval=status;
	return status;
}

static unsigned char bcm4750_write_receive( struct bcm4750_uart_port *bcm_port, int length )
{
	return bcm4750_write( bcm_port, 0x40, length );
}

static irqreturn_t bcm4750_serial_irq( int irq, void *pdata )
{
	struct bcm4750_uart_port	*bport=(struct bcm4750_uart_port *) pdata;

	/* Can't do SPI in interrupt :( */
	/* No spinlock for accessing rx_enabled/tx_enabled here! Don't want to hang in an interrupt. */
	if( bport->rx_enabled || bport->tx_enabled )
	{
		atomic_inc( &(bport->bh_scheduled) );
		queue_work( bport->serial_wq, &(bport->rxtx_work) );
	}

	/* Not neat, but the level IRQ won't stop until the SPI transaction is finished. And the transaction won't finish until */
	/* the worker thread has started. To prevent endless looping disable the IRQ here, enable it later. */
	disable_irq( bport->port.irq );
	return IRQ_HANDLED;
}

static void bcm4750_rxtx_work( struct work_struct *work )
{
	struct bcm4750_uart_port	*bport=container_of( work, struct bcm4750_uart_port, rxtx_work );
	struct uart_port		*port=&(bport->port);
	unsigned char			control;
	int				length;
	int				pending;
	int				count;
	unsigned char			status;
	unsigned long int		flags;

	/* Ensure the bh_scheduled counter is decreased */
	atomic_dec( &(bport->bh_scheduled) );

	/* Drain the TX/RX buffer in a loop equally. */
	do
	{
		status=bcm4750_read_transmit( bport, &length );
		if( status & 0x1F )
		{
			if( status == 0xFF )
			{
				printk( "bcm4750_read_transmit buffer error!\n" );
				spin_lock_irqsave( &(bport->lock), flags );
				control=(bport->rx_enabled ? 0x10 : 0x00) | (bport->tx_enabled ? 0x20 : 0x00);
				spin_unlock_irqrestore( &(bport->lock), flags );
				bcm4750_write_control( bport, 0xC0 | control );
				bcm4750_write_control( bport, control );
				spin_lock_irqsave( &(bport->lock), flags );
				if( bport->rx_enabled || bport->tx_enabled )
					enable_irq( port->irq );
				spin_unlock_irqrestore( &(bport->lock), flags );
				return;
			}

			/* If we're closing off, or this IRQ is handled after the port is closed off, TTY doesn't exist anymore. */
			/* In that case we don't push the bytes up. They don't matter any longer. */
			if( port->info->port.tty != NULL )
			{
				count=tty_insert_flip_string( port->info->port.tty, bcm4750_get_read_buf( bport ), length );
				if( count < length )
				{
					printk( KERN_WARNING "BCM4750: Input overrun by %i bytes!\n", length - count );
					port->icount.buf_overrun+=length - count;
				}
				port->icount.rx+=count;
				tty_flip_buffer_push( port->info->port.tty );
			}
		}

		pending=uart_circ_chars_pending( &port->info->xmit );
		if( (status & 0x20) && (pending != 0) )
		{
			/* We need to update the buffer first, as it can change while we work with it. */
			spin_lock_irqsave(&port->lock, flags);
			if( pending > 18 ) pending=18;
			count=CIRC_CNT_TO_END( port->info->xmit.head, port->info->xmit.tail, UART_XMIT_SIZE );
			if( pending > count ) pending=count;

			/* Copy the data. */
			memcpy( bcm4750_get_write_buf( bport ), port->info->xmit.buf + port->info->xmit.tail, pending );

			/* Update the circular buffer. */
			port->info->xmit.tail=(port->info->xmit.tail + pending) & (UART_XMIT_SIZE - 1);
			spin_unlock_irqrestore(&port->lock, flags);

			/* Write the data. */
			if( (status=bcm4750_write_receive( bport, pending ) & 0x80) == 0 )
			{
				/* Transmission successfull. Update the TX count. */
				port->icount.tx+=pending;
			}
			else
			{
				/* Very unlikely that we get here.... */
				port->icount.overrun+=pending;
			}
		}
	}
	while( (status & 0x1F) || (((pending=uart_circ_chars_pending( &port->info->xmit )) != 0) && (status & 0x20)) );

	/* Inform upstream we need more data, if the buffer is getting empty. Also, to prevent excessive IRQs */
	/* disable TX if the buffer is empty. Upstream will do a start_tx upon a write anyway. */
	pending=uart_circ_chars_pending( &port->info->xmit );
	if( (pending < WAKEUP_CHARS) && (port->info->port.tty != NULL) )
		tty_wakeup( port->info->port.tty );

	/* Any access to tx_enabled or rx_enabled under spinlock! */
	spin_lock_irqsave( &(bport->lock), flags );
	if( bport->tx_enabled && (pending == 0) )
		bport->tx_enabled=0;

	control=(bport->rx_enabled ? 0x10 : 0x00) | (bport->tx_enabled ? 0x20 : 0x00);
	spin_unlock_irqrestore( &(bport->lock), flags );

	/* Need to clear the interrupt flag before the next IRQ. Otherwise it */
	/* does not fire. Also check if any data needs to be written. And clear */
	/* any error flags. */
	status=bcm4750_write_control( bport, control );

	/* Reenable the IRQ now. The buffer was emptied, so we can wait for the IRQ once more. */
	spin_lock_irqsave( &(bport->lock), flags );
	if( bport->rx_enabled || bport->tx_enabled )
		enable_irq( port->irq );
	spin_unlock_irqrestore( &(bport->lock), flags );
	return;
}

static int bcm4750_do_identify( struct bcm4750_uart_port *bcm_port )
{
	const char			ident_write[]={0xFF, 0x00, 0xFD, 0x00, 0x00, 0xFC};
	const char			ident_read[]={0xFE, 0x00, 0xFD, 0x00, 0x00, 0xF1, 0x87, 0x0D, 0x20, 0xFC};
	char				buffer[18];
	int				length;
	unsigned char			status;

	/* Do identify. */
	memcpy( bcm4750_get_write_buf( bcm_port ), ident_write, sizeof( ident_write ) );
	if( (status=bcm4750_write_receive( bcm_port, sizeof( ident_write ) )) != 0x20 )
	{
		printk( "BCM4750: Error writing identify! Status %.2X\n", status );
		return -ENODEV;
	}
	mdelay( 10 );
	if( (status=bcm4750_read_transmit( bcm_port, &length )) != 0x2A )
	{
		printk( "BCM4750: Error receiving identify result! Status %.2X\n", status );
		return -ENODEV;
	}
	memcpy( buffer, bcm4750_get_read_buf( bcm_port ), length );

	if( (length != sizeof( ident_read )) || memcmp( ident_read, buffer, length ) )
	{
		printk( "BCM4750: Ident data received is not correct!\n" );
		for( length=0; length < sizeof( ident_read ); length++ )
			printk( "Read[%i]: %.2X, Expect: %.2X\n", length, buffer[length], ident_read[length] );
		printk( "Length received: %i, expected %u\n", length, sizeof( ident_read ) );
		return -ENODEV;
	}

	printk( "BCM4750: Identify success! Received:\n" );
	for( length=0; length < sizeof( ident_read ); length++ )
		printk( "%.2X ", buffer[length] );
	printk( "\nStatus: %.2X\n", status );
	return 0;
}

static int bcm4750_spi_probe( struct spi_device *spi )
{
	struct bcm4750_uart_port	*bcm4750_uart_port;
	int				retval;

	/* Signon. */
	printk( "TomTom GO BCM4750 SPI TTY Emulation Driver, (c) 2008 TomTom B.V\n" );

	/* Allocate memory for the private driver data. */
	bcm4750_uart_port=(struct bcm4750_uart_port *) kmalloc( sizeof( struct bcm4750_uart_port ), GFP_ATOMIC );
	if( bcm4750_uart_port == NULL )
	{
		printk( KERN_ERR "Failed to allocate memory for the BCM4750 SPI TTY Emulation driver.\n" );
		return -ENOMEM;
	}
	else memset( bcm4750_uart_port, 0, sizeof( struct bcm4750_uart_port ) );

	/* Initialize the structure. */
	spin_lock_init( &(bcm4750_uart_port->port.lock) );
	spin_lock_init( &(bcm4750_uart_port->lock) );
	bcm4750_uart_port->port.iotype=UPIO_MEM;
	bcm4750_uart_port->port.irq=spi->irq;
	bcm4750_uart_port->port.custom_divisor=1;
	bcm4750_uart_port->port.fifosize=18;
	bcm4750_uart_port->port.ops=&bcm4750_serial_ops;
	bcm4750_uart_port->port.flags=UPF_BOOT_AUTOCONF;
	bcm4750_uart_port->port.type=PORT_16550A;
	bcm4750_uart_port->port.dev=&(spi->dev);

	/* Allocate memory for the transfer buffer. */
#ifdef CONFIG_SERIAL_BCM4750_EMU_DMA
	spi->dev.coherent_dma_mask = 0xffffffffUL;

	bcm4750_uart_port->master_transfer.rx_buf=dma_alloc_coherent( &(spi->dev), sizeof( struct bcm4750_message ),
								      &(bcm4750_uart_port->master_transfer.rx_dma), GFP_KERNEL | GFP_DMA );
	bcm4750_uart_port->master_transfer.tx_buf=dma_alloc_coherent( &(spi->dev), sizeof( struct bcm4750_message ),
								      &(bcm4750_uart_port->master_transfer.tx_dma), GFP_KERNEL | GFP_DMA );
#else
	bcm4750_uart_port->master_transfer.tx_buf=kmalloc( sizeof( struct bcm4750_message ), GFP_KERNEL );	
	bcm4750_uart_port->master_transfer.rx_buf=kmalloc( sizeof( struct bcm4750_message ), GFP_KERNEL );	
	bcm4750_uart_port->master_transfer.tx_dma=0;
	bcm4750_uart_port->master_transfer.rx_dma=0;
#endif
	if( !bcm4750_uart_port->master_transfer.tx_buf || !bcm4750_uart_port->master_transfer.rx_buf )
	{
		printk( KERN_ERR "Failed to allocate transfer buffer memory.\n" );
		if( bcm4750_uart_port->master_transfer.tx_buf )
#ifdef CONFIG_SERIAL_BCM4750_EMU_DMA
			dma_free_coherent( &(spi->dev), sizeof( struct bcm4750_message ), (void *) bcm4750_uart_port->master_transfer.tx_buf,
					   bcm4750_uart_port->master_transfer.tx_dma );
#else
			kfree( bcm4750_uart_port->master_transfer.tx_buf );
#endif
		if( bcm4750_uart_port->master_transfer.rx_buf )
#ifdef CONFIG_SERIAL_BCM4750_EMU_DMA
			dma_free_coherent( &(spi->dev), sizeof( struct bcm4750_message ), (void *) bcm4750_uart_port->master_transfer.rx_buf,
					   bcm4750_uart_port->master_transfer.rx_dma );
#else
			kfree( bcm4750_uart_port->master_transfer.rx_buf );
#endif
			kfree( bcm4750_uart_port );
			return -ENOMEM;
	}
	bcm4750_uart_port->master_transfer.speed_hz=0;
	bcm4750_uart_port->master_transfer.bits_per_word=8;
	bcm4750_uart_port->master_transfer.delay_usecs=0;

	/* Detect. */
	if( bcm4750_do_identify( bcm4750_uart_port ) < 0 )
	{
		printk( KERN_ERR "Failed to identify BCM4750 chip! (Is the chip powered?)\n" );
#ifdef CONFIG_SERIAL_BCM4750_EMU_DMA
		dma_free_coherent( &(spi->dev), sizeof( struct bcm4750_message ), (void *) bcm4750_uart_port->master_transfer.tx_buf,
				   bcm4750_uart_port->master_transfer.tx_dma );
		dma_free_coherent( &(spi->dev), sizeof( struct bcm4750_message ), (void *) bcm4750_uart_port->master_transfer.rx_buf,
				   bcm4750_uart_port->master_transfer.rx_dma );
#else
		kfree( bcm4750_uart_port->master_transfer.tx_buf );
		kfree( bcm4750_uart_port->master_transfer.rx_buf );
#endif
		kfree( bcm4750_uart_port );
		return -ENODEV;
	}

	/* Register the serial driver. */
	retval=uart_register_driver( &bcm4750_uart_driver );
	if( retval )
	{
		printk( KERN_ERR "Failed to register BCM4750 SPI TTY Emulation driver.\n" );
#ifdef CONFIG_SERIAL_BCM4750_EMU_DMA
		dma_free_coherent( &(spi->dev), sizeof( struct bcm4750_message ), (void *) bcm4750_uart_port->master_transfer.tx_buf,
				   bcm4750_uart_port->master_transfer.tx_dma ); 
		dma_free_coherent( &(spi->dev), sizeof( struct bcm4750_message ), (void *) bcm4750_uart_port->master_transfer.rx_buf,
				   bcm4750_uart_port->master_transfer.rx_dma ); 
#else
		kfree( bcm4750_uart_port->master_transfer.tx_buf );
		kfree( bcm4750_uart_port->master_transfer.rx_buf );
#endif
		kfree( bcm4750_uart_port );
		return retval;
	}

	/* Register the one TTY device. */
	uart_add_one_port( &bcm4750_uart_driver, &(bcm4750_uart_port->port) );

	/* Request the interrupt. */
	retval=request_irq( bcm4750_uart_port->port.irq, bcm4750_serial_irq, IRQF_TRIGGER_LOW, bcm4750_uart_driver.driver_name, bcm4750_uart_port );
	if( retval )
	{
		printk( KERN_ERR "Failed to register BCM4750 SPI TTY IRQ.\n" );
		uart_remove_one_port( &bcm4750_uart_driver, &(bcm4750_uart_port->port) );
		uart_unregister_driver( &bcm4750_uart_driver );
#ifdef CONFIG_SERIAL_BCM4750_EMU_DMA
		dma_free_coherent( &(spi->dev), sizeof( struct bcm4750_message ), (void *) bcm4750_uart_port->master_transfer.tx_buf,
				   bcm4750_uart_port->master_transfer.tx_dma ); 
		dma_free_coherent( &(spi->dev), sizeof( struct bcm4750_message ), (void *) bcm4750_uart_port->master_transfer.rx_buf,
				   bcm4750_uart_port->master_transfer.rx_dma ); 
#else
		kfree( bcm4750_uart_port->master_transfer.tx_buf );
		kfree( bcm4750_uart_port->master_transfer.rx_buf );
#endif
		kfree( bcm4750_uart_port );
		return retval;
	}

	/* Set the private info. */
	spi_set_drvdata( spi, bcm4750_uart_port );

	/* Setup the workqueue. */
	INIT_WORK( &(bcm4750_uart_port->rxtx_work), bcm4750_rxtx_work );
	INIT_WORK( &(bcm4750_uart_port->start_tx), bcm4750_start_tx_work );
	INIT_WORK( &(bcm4750_uart_port->stop_rx), bcm4750_stop_rx_work );
	INIT_WORK( &(bcm4750_uart_port->stop_tx), bcm4750_stop_tx_work );
	atomic_set( &(bcm4750_uart_port->bh_scheduled), 0 );
	bcm4750_uart_port->serial_wq=create_singlethread_workqueue( "bcm4750_wq" );

	/* Disable interrupts, clear underrun/overrun condition. */
	printk( "BCM4751: State after init: %.2X\n", bcm4750_write_control( bcm4750_uart_port, 0xC0 ) );
	disable_irq( bcm4750_uart_port->port.irq );
	return 0;
}

static int bcm4750_spi_remove( struct spi_device *spi )
{
	struct bcm4750_uart_port	*bport=(struct bcm4750_uart_port *) spi_get_drvdata( spi );

	flush_workqueue( bport->serial_wq );
	destroy_workqueue( bport->serial_wq );
	uart_remove_one_port( &bcm4750_uart_driver, &(bport->port) );
	uart_unregister_driver( &bcm4750_uart_driver );
#ifdef CONFIG_SERIAL_BCM4750_EMU_DMA
	dma_free_coherent( &(spi->dev), sizeof( struct bcm4750_message ), (void *) bport->master_transfer.rx_buf,
			   bport->master_transfer.rx_dma ); 
	dma_free_coherent( &(spi->dev), sizeof( struct bcm4750_message ), (void *) bport->master_transfer.tx_buf,
			   bport->master_transfer.tx_dma ); 
#else
	kfree( bport->master_transfer.rx_buf );
	kfree( bport->master_transfer.tx_buf );
#endif
	kfree( bport );
	free_irq( spi->irq, bport );
	return 0;
}

static struct uart_ops	bcm4750_serial_ops=
{
        .tx_empty       = bcm4750_tx_empty,
        .get_mctrl      = bcm4750_get_mctrl,
        .set_mctrl      = bcm4750_set_mctrl,
        .stop_tx        = bcm4750_stop_tx,
        .start_tx       = bcm4750_start_tx,
        .stop_rx        = bcm4750_stop_rx,
        .enable_ms      = bcm4750_enable_ms,
        .break_ctl      = bcm4750_break_ctl,
        .startup        = bcm4750_startup,
        .shutdown       = bcm4750_shutdown,
        .set_termios    = bcm4750_set_termios,
        .type           = bcm4750_type,
        .release_port   = bcm4750_release_port,
        .request_port   = bcm4750_request_port,
        .config_port    = bcm4750_config_port,
        .verify_port    = bcm4750_verify_port,
};

static int bcm4750_suspend( struct spi_device *spi, pm_message_t state )
{
	struct bcm4750_uart_port	*bport=(struct bcm4750_uart_port *) spi_get_drvdata( spi );
	struct bcm4750_platform_data 	*pdata=spi->dev.platform_data;

	/* No need to suspend if nothing is running. */
	if( bport->rx_enabled || bport->tx_enabled )
		uart_suspend_port( &bcm4750_uart_driver, &(bport->port) );

	if (pdata->suspend)
		pdata->suspend();

        return 0;
}

static int bcm4750_resume( struct spi_device *spi )
{
	struct bcm4750_uart_port	*bport=(struct bcm4750_uart_port *) spi_get_drvdata( spi );
	struct bcm4750_platform_data	*pdata=spi->dev.platform_data;

	if (pdata->resume)
		pdata->resume();

	/* If the port was suspended, resume now. */
	if( bport->port.suspended )
		uart_resume_port( &bcm4750_uart_driver, &(bport->port) );

	return 0;
}

static struct spi_driver	bcm4750_spi_driver=
{
	.driver=
	{
		.name		= "bcm4750_spi_tty",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},
	.probe			= bcm4750_spi_probe,
	.remove			= bcm4750_spi_remove,
	.suspend		= bcm4750_suspend,
	.resume			= bcm4750_resume,
};

static int __init bcm4750_spi_init( void )
{
	return spi_register_driver( &bcm4750_spi_driver );
}

static void __exit bcm4750_spi_exit( void )
{
	spi_unregister_driver( &bcm4750_spi_driver );
	return;
}

module_init( bcm4750_spi_init );
module_exit( bcm4750_spi_exit );

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rogier Stam <rogier.stam@tomtom.com>");
MODULE_DESCRIPTION("BCM4750 TTY Emulation Driver");
