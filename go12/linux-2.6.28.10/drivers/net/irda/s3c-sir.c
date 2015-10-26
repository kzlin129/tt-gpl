/*
 * drivers/net/irda/s3c-sir.c
 * Samsung Infra-red driver for the S3C embedded microprocessor 
 * 
 * Copyright (C) 2009 for Samsung Electronics
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
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/rtnetlink.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <net/irda/irda.h>
#include <net/irda/wrapper.h>
#include <net/irda/irda_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/irda.h>

#include <mach/map.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <plat/regs-serial.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#if defined(CONFIG_PLAT_S5PC1XX)
#include <plat/gpio-bank-a1.h>
#elif defined(CONFIG_PLAT_S3C64XX)
#include <plat/gpio-bank-b.h>
#endif

#include <asm/dma.h>
#include <plat/dma.h>
#include <mach/irqs.h>

/* #define S3C_IRDA_DEBUG */

#ifdef S3C_IRDA_DEBUG
#define DBG(x...)       printk(PFX x)
#else
#define DBG(x...)       do { } while (0)
#endif

#define DRIVER_NAME "s3c-irda"
#define PFX DRIVER_NAME ": "

static int max_rate = 115200;

struct s3c_irda {
        unsigned char           hscr0;
        unsigned char           utcr4;
        unsigned char           power;
        unsigned char           open;

        int                     speed;
        int                     newspeed;

        struct sk_buff          *txskb;
        struct sk_buff          *rxskb;

        struct net_device_stats stats;
        struct device           *dev;
        struct irda_platform_data *pdata;
        struct irlap_cb         *irlap;
        struct qos_info         qos;

        iobuff_t                tx_buff;
        iobuff_t                rx_buff;
        unsigned int		sir_irq_rx;
        unsigned int		sir_irq_tx;
        
	struct resource         *sir_mem;
        struct clk              *sir_clk;
        void __iomem            *sir_base;
        int                     dma;
};


#define RESSIZE(ressource)	(((ressource)->end - (ressource)->start)+1)
#define S3C_SIR_MAX_RXLEN	2047

static const unsigned int nSlotTable[16] = {0x0000,0x0080,0x0808,0x8888,0x2222,0x4924,0x4a52,0x54aa,
				     0x5555,0xd555,0xd5d5,0xddd5,0xdddd,0xdfdd,0xdfdf,0xffdf};

#if defined(CONFIG_CPU_S3C6400) || defined(CONFIG_CPU_S3C6410) || defined(CONFIG_CPU_S5PC100)
/* UART3 port has been reserved for Irda */
#define sir_writereg(val,reg) writel(val, (S3C24XX_VA_UART3 + reg))
#define sir_readreg(reg)  readl(S3C24XX_VA_UART3 + reg)
#else
/* UART2 port has been reserved for Irda */
#define sir_writereg(val,reg) writel(val, (S3C24XX_VA_UART2 + reg))
#define sir_readreg(reg)  readl(S3C24XX_VA_UART2 + reg)
#endif


extern int clk_enable(struct clk *clk);
extern unsigned long clk_get_rate(struct clk *clk);
extern struct clk *clk_get(struct device *dev, const char *id);
extern void clk_put(struct clk *clk);
extern void clk_disable(struct clk *clk);


static void  s3c_irda_gpio_conf(void)
{
#if defined(CONFIG_CPU_S3C6400) || defined(CONFIG_CPU_S3C6410)
	s3c_gpio_cfgpin(S3C64XX_GPB(2), S3C64XX_GPB2_UART_RXD3);
	s3c_gpio_cfgpin(S3C64XX_GPB(3), S3C64XX_GPB3_UART_TXD3);
#elif defined(CONFIG_CPU_S5PC100)
	s3c_gpio_cfgpin(S5PC1XX_GPA1(2), S5PC1XX_GPA1_2_UART_3_RXD);
	s3c_gpio_cfgpin(S5PC1XX_GPA1(3), S5PC1XX_GPA1_3_UART_3_TXD);
#endif
}


static int s3c_irda_sir_init(struct s3c_irda *si)
{
        u32 ucon, ulcon, ufcon;

        DBG("%s\r\n", __FUNCTION__);

        /* Enable uart clock */
        clk_enable(si->sir_clk);

	/* Setup the GPIOs */
	s3c_irda_gpio_conf();

	ulcon = S3C_LCON_IRM | S3C_LCON_PNONE | S3C_LCON_CS8;
	ucon =  S3C_UCON_PCLK | S3C_UCON_TXILEVEL | S3C_UCON_RXILEVEL | \
	        S3C_UCON_RXFIFO_TOI| S3C_UCON_RX_ESIE | \
	        S3C_UCON_LOOP_OPERATION | S3C_UCON_NO_SBS | \
		S3C_UCON_RXIRQMODE;
	ufcon = S3C_UFCON_TXTRIG16 | S3C_UFCON_RXTRIG32 | \
		S3C_UFCON_RESETBOTH | S3C_UFCON_FIFO_ENABLE;

        sir_writereg(ulcon, S3C_ULCON);
        sir_writereg(ufcon, S3C_UFCON);
        sir_writereg(ucon, S3C_UCON);

        return 0;
}


static int s3c_irda_sir_stop(struct s3c_irda *si)
{
        DBG("%s\r\n", __FUNCTION__);

	disable_irq(si->sir_irq_rx);
	disable_irq(si->sir_irq_tx);

        sir_writereg(0, S3C_ULCON);
        sir_writereg(0, S3C_UFCON);
        sir_writereg(0, S3C_UCON);

        /* Disable the uart clock */
        clk_disable(si->sir_clk);

        return 0;
}


static int s3c_irda_sir_setspeed(struct s3c_irda *si, u32 speed)
{
        u32 ubrdiv, pclk;
	int slot = -1;

        DBG("%s\r\n", __FUNCTION__);

        pclk = clk_get_rate(si->sir_clk);
        ubrdiv = (int) (pclk/16/speed) - 1;

        DBG("sir : pclk %d speed %d ubrdiv %d \r\n", pclk, speed, ubrdiv);

        sir_writereg(ubrdiv, S3C_UBRDIV);

	if(slot >= 0) {
		sir_writereg(nSlotTable[slot], S3C_UDIVSLOT);
	}
        return 0;
}


/*
 * Set the IrDA communications speed.
 */
static int s3c_irda_set_speed(struct s3c_irda *si, int speed)
{
        unsigned long flags;
        int ret = -EINVAL;

        DBG("%s\r\n", __FUNCTION__);

        switch (speed) {
        case 9600:      case 19200:     case 38400:
        case 57600:     case 115200:

                local_irq_save(flags);
                s3c_irda_sir_setspeed(si, speed);
                si->speed = speed;
                local_irq_restore(flags);
                ret = 0;
                break;

        default:
		printk(KERN_ERR "Invalid speed requested\n");
                break;
        }

        return ret;

}


/*
 * Control the power state of the IrDA transmitter.
 * State:
 *  0 - off
 *  1 - short range, lowest power
 *  2 - medium range, medium power
 *  3 - maximum range, high power
 *
 * Currently, only assabet is known to support this.
 */
static int
__s3c_irda_set_power(struct s3c_irda *si, unsigned int state)
{
        int ret = 0;
        DBG("%s\r\n", __FUNCTION__);

        if(si->pdata->set_power)
                ret = si->pdata->set_power(si->dev, state);
        return ret;
}


static inline int s3c_set_power(struct s3c_irda *si, unsigned int state)
{
        int ret;
        DBG("%s\r\n", __FUNCTION__);

        ret = __s3c_irda_set_power(si, state);
        if(ret == 0)
                si->power = state;

        return ret;
}

static int s3c_irda_startup(struct s3c_irda *si)
{
        int ret;
        
        DBG("%s\r\n", __FUNCTION__);

        /* Ensure that the ports for this device are setup correctly */
#if defined(CONFIG_CPU_S3C6400) || defined(CONFIG_CPU_S3C6410)
        s3c_gpio_setpull(S3C64XX_GPB(2), S3C_GPIO_PULL_UP);
        s3c_gpio_setpull(S3C64XX_GPB(3), S3C_GPIO_PULL_UP);
#elif defined(CONFIG_CPU_S5PC100)
        s3c_gpio_setpull(S5PC1XX_GPA1(2), S3C_GPIO_PULL_UP);
        s3c_gpio_setpull(S5PC1XX_GPA1(3), S3C_GPIO_PULL_UP);
#endif

        ret = s3c_irda_sir_init(si);
        if(ret) {
                printk("Irda Startup failed\r\n");
                return ret;
        }

        ret = s3c_irda_set_speed(si, si->speed = 9600);
        if(ret) 
                s3c_irda_sir_stop(si);

        return ret;
}

static void s3c_irda_shutdown(struct s3c_irda *si)
{
        DBG("%s\r\n", __FUNCTION__);
        s3c_irda_sir_stop(si);
}


#if defined(CONFIG_PM)
/*
 * Suspend the IrDA interface.
 */
static int s3c_irda_suspend(struct platform_device *pdev, pm_message_t state)
{
        struct net_device *dev = platform_get_drvdata(pdev);
        struct s3c_irda *si;

        DBG("%s\r\n", __FUNCTION__);

        if(!dev)
                return 0;

        si = dev->priv;
        if(si->open) {
                /* Stop the transmit queue */
                netif_device_detach(dev);
                s3c_irda_shutdown(si);
                __s3c_irda_set_power(si, 0);
        }

        return 0;
}


/*
 * Resume the IrDA interface.
 */
static int s3c_irda_resume(struct platform_device *pdev)
{
        struct net_device *dev = platform_get_drvdata(pdev);
        struct s3c_irda *si;

        DBG("%s\r\n", __FUNCTION__);

        if(!dev)
                return 0;

        si = dev->priv;
        if(si->open) {
                /*
                 * If we missed a speed change, initialise at the new speed
                 * directly.  It is debatable whether this is actually
                 * required, but in the interests of continuing from where
                 * we left off it is desireable.  The converse argument is
                 * that we should re-negotiate at 9600 baud again.
                 */
                if(si->newspeed) {
                        si->speed = si->newspeed;
                        si->newspeed = 0;
                }

                s3c_irda_startup(si);
                __s3c_irda_set_power(si, si->power);

                
                /* This automatically wakes up the queue */
                netif_device_attach(dev);
        }

        return 0;
}
#else
#define s3c_irda_suspend    NULL
#define s3c_irda_resume     NULL
#endif


/*
 * SIR format interrupt service routines.
 */
static irqreturn_t  s3c_irda_sir_irq(int irq, void *dev_id)
{

        struct net_device *dev = dev_id;
        struct s3c_irda *si = dev->priv;
        int err_status;

        u8 data;
	u32 ucon, ufcon, ufstat;


        err_status = sir_readreg(S3C_UERSTAT);

        if(err_status) {
                printk("Error : 0x%x\n", sir_readreg(S3C_UERSTAT));
                data = sir_readreg(S3C_URXH);
                si->stats.rx_errors++;
                si->stats.rx_frame_errors++;
        }

	if(irq == si->sir_irq_rx){
		DBG("Rx intr : 0x%1x\n", intpnd);

                while ((sir_readreg(S3C_UFSTAT) & 0X3F) > 0) {
                        data = sir_readreg(S3C_URXH);
                        async_unwrap_char(dev, &si->stats, &si->rx_buff, data);
                }
                dev->last_rx = jiffies;

		/* Clear FIFO */
		ufcon = sir_readreg(S3C_UFCON);
		ufcon |= 3;
		sir_writereg(ufcon, S3C_UFCON);
        }

	if(irq == si->sir_irq_tx) {
		DBG("Tx intr : 0x%1x\n", intpnd);
		
		if(si->tx_buff.len > 0) {
               		ufstat = sir_readreg(S3C_UFSTAT);
                
		        /* Transmitter FIFO is not full */
        	       	while (!(ufstat & (1 << 14)) ) {
				while(!(sir_readreg(S3C_UTRSTAT) & 0x02));
				sir_writereg(*si->tx_buff.data++, S3C_UTXH);
	              	 	if(si->tx_buff.len == 0)
        	                	break;
         		        si->tx_buff.len -= 1;
                        	rmb();
	                        ufstat = sir_readreg(S3C_UFSTAT);
				}

                	if(si->tx_buff.len == 0) {
                       		si->stats.tx_packets++;
	               		si->stats.tx_bytes += si->tx_buff.data -
        	  			              si->tx_buff.head;
	
        	     	 /* We need to ensure that transmit has finished */
                	 do {
                 		rmb();
	                       	ufstat = sir_readreg(S3C_UFSTAT);
        	       	} while (((ufstat >> 8) & 0x3f) > 0);

               
              		/* Transmission complete. Now enable the receiver.  
	 		 * Sometimes we get a receive IRQ immediately 
	 		 * after a transmit 
 		         */
              
	               	ufcon = sir_readreg(S3C_UFCON);
        	       	ufcon |= 7;
               		sir_writereg(ufcon, S3C_UFCON);

	               	ucon = sir_readreg(S3C_UCON);
        	      	ucon &= ~( 3 << 2);
               		sir_writereg(ucon, S3C_UCON);

	               	if(si->newspeed) {
        	               	s3c_irda_set_speed(si, si->newspeed);
                	       	si->newspeed = 0;
               		}

      	 		if(1) {
                      		ucon |= 1;
	                      	sir_writereg(ucon, S3C_UCON);
        	             	}

               		netif_wake_queue(dev);
			}
		}
	}
        return IRQ_HANDLED;
}

static int s3c_irda_hard_xmit(struct sk_buff *skb, struct net_device *dev)
{
        struct s3c_irda *si = dev->priv;
        int speed = irda_get_next_speed(skb);

        DBG("%s\r\n", __FUNCTION__);

        /*
         * Does this packet contain a request to change the interface
         * speed?  If so, remember it until we complete the transmission
         * of this frame.
         */
        if(speed != si->speed && speed != -1) {
                DBG("Irda  New Speed %d bps\r\n", speed);
                si->newspeed = speed;
        }
        
        /* If this is an empty frame, we can bypass a lot */
        if(skb->len == 0) {
                if(si->newspeed) {
                        si->newspeed = 0;
                        s3c_irda_set_speed(si, speed);
                }
                dev_kfree_skb(skb);
                return 0;
        }

        {
                u32 ucon, ufcon, len;
                u8 *cp;

                netif_stop_queue(dev);

                cp = si->tx_buff.data = si->tx_buff.head;
                len = si->tx_buff.len  = async_wrap_skb(skb, si->tx_buff.data,
                                                  si->tx_buff.truesize);

                /*
                 * Set the transmit interrupt enable.  This will fire
                 * off an interrupt immediately.  Note that we disable
                 * the receiver so we won't get spurious characteres
                 * received.
                 */

                /* Stop Rx 
		 * UCON  : Receive Mode Disable 
		 */
                ucon = sir_readreg(S3C_UCON);
                ucon &= ~( 3); 
                sir_writereg(ucon, S3C_UCON);

                /* Clear FIFO */
                ufcon = sir_readreg(S3C_UFCON);
                ufcon |= 7;
                sir_writereg(ufcon, S3C_UFCON);

		/* UCON : Transmit Mode - 01 - Interrupt request or polling mode */
                ucon |= (1 << 2 );
                sir_writereg(ucon, S3C_UCON);

                dev_kfree_skb(skb);
        }

        dev->trans_start = jiffies;
        return 0;
}


static int s3c_irda_ioctl(struct net_device *dev, struct ifreq *ifreq, int cmd)
{
        struct if_irda_req *rq = (struct if_irda_req *)ifreq;
        struct s3c_irda *si = dev->priv;
        int ret = -EOPNOTSUPP;

        DBG("%s\r\n", __FUNCTION__);

        switch (cmd) {
        case SIOCSBANDWIDTH:

                if(capable(CAP_NET_ADMIN)) {
                        
                        /* We are unable to set the speed if the device is not running */
                        if(si->open) 
                                ret = s3c_irda_set_speed(si,
                                                rq->ifr_baudrate);
                        else {
                                DBG("s3c_irda_ioctl: SIOCSBANDWIDTH: !netif_running\n");
				ret = 0;
                        }
                }
                break;

        case SIOCSMEDIABUSY:
                ret = -EPERM;
                if(capable(CAP_NET_ADMIN)) {
                        irda_device_set_media_busy(dev, TRUE);
                        ret = 0;
                }
                break;

        case SIOCGRECEIVING:
                rq->ifr_receiving =  si->rx_buff.state != OUTSIDE_FRAME;
                ret = 0;
                break;

        default:
                break;
        }

        return ret;
}


static struct net_device_stats *s3c_irda_stats(struct net_device *dev)
{
        struct s3c_irda *si = dev->priv;

        DBG("%s\r\n", __FUNCTION__);
        return &si->stats;
}

static int s3c_irda_start(struct net_device *dev)
{
        struct s3c_irda *si = dev->priv;
        int err;

        DBG("%s\r\n", __FUNCTION__);

        si->speed = 9600;
	
	err = request_irq(si->sir_irq_rx, s3c_irda_sir_irq, 0, dev->name, dev);
	if(err)
                goto err_irq1;

        err = request_irq(si->sir_irq_tx, s3c_irda_sir_irq, 0, dev->name, dev);
        if(err)                                                     
                  goto err_irq1; 

        /* The interrupt must remain disabled for now */
	disable_irq(si->sir_irq_rx);
        disable_irq(si->sir_irq_tx);

        
        /* Setup the serial port for the specified speed */
        err = s3c_irda_startup(si);
        if(err)
                goto err_irq2;

        
        /* Open a new IrLAP layer instance */
        si->irlap = irlap_open(dev, &si->qos, "s3c");

        err = -ENOMEM;
        if(!si->irlap)
                goto err_irlap;

        
        /* Now enable the interrupt and start the queue */
	enable_irq(si->sir_irq_rx);
	enable_irq(si->sir_irq_tx);
        si->open = 1;
        netif_start_queue(dev);
	sir_writereg(0, S3C_UINTMSK); 
        
	return 0;

err_irlap:
        si->open = 0;
        s3c_irda_shutdown(si);
err_irq2:
        free_irq(si->sir_irq_rx, dev);
        free_irq(si->sir_irq_tx, dev);
err_irq1:
        return err;
}

static int s3c_irda_stop(struct net_device *dev)
{
        struct s3c_irda *si = dev->priv;

        DBG("%s\r\n", __FUNCTION__);

	disable_irq(si->sir_irq_rx);
        disable_irq(si->sir_irq_tx);

        s3c_irda_shutdown(si);

        /* Clean up */
        if(si->rxskb) {
                dev_kfree_skb(si->rxskb);
                si->rxskb = NULL;
        }

        /* Stop IrLAP */
        if(si->irlap) {
                irlap_close(si->irlap);
                si->irlap = NULL;
        }

        netif_stop_queue(dev);
        si->open = 0;

        return 0;
}

static int s3c_irda_init_iobuf(iobuff_t *io, int size)
{
        DBG("%s\r\n", __FUNCTION__);

	io->head = kmalloc(size, GFP_KERNEL | GFP_DMA);

        if(io->head != NULL) {
                io->truesize = size;
                io->in_frame = FALSE;
                io->state    = OUTSIDE_FRAME;
                io->data     = io->head;
        }

        return io->head ? 0 : -ENOMEM;
}


static int s3c_irda_init_mem( struct s3c_irda *si,
                struct platform_device *pdev)
{
        int ret = 0;

        DBG("%s\r\n", __FUNCTION__);

        si->sir_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

        if(!si->sir_mem) {
                printk("failed to get io sir_memory region resouce.\n");
                return -ENOENT;
        }

        if(NULL == request_mem_region(si->sir_mem->start,
                RESSIZE(si->sir_mem), pdev->name)) {
                printk("failed to request io sir memory region.\n");
                ret =  -ENOENT;
        }

        return ret;
}


static int s3c_irda_free_mem( struct s3c_irda *si)
{
        DBG("%s\r\n", __FUNCTION__);

        release_mem_region(si->sir_mem->start, RESSIZE(si->sir_mem));
        return 0;
}

static int s3c_irda_init_clk(struct device *dev,
				struct s3c_irda *si)
{
        int ret;
        DBG("%s   \r\n", __FUNCTION__);

        si->sir_clk = clk_get(dev, "uart");

        if(IS_ERR(si->sir_clk)) {
                DBG(KERN_INFO PFX "failed to find sir clock source.\n");
                ret = PTR_ERR(si->sir_clk);
                si->sir_clk = NULL;
                goto sir_free_si;
        }

        if((ret = clk_enable(si->sir_clk))) {
                printk("failed to use sir clock source.\n");
		ret =-ENODEV;
                goto sir_clk_free;
        }

        return 0;

sir_clk_free:
        clk_put(si->sir_clk);
sir_free_si:
        return ret;
}


static int s3c_irda_stop_clk( struct s3c_irda *si)
{
        DBG("%s\r\n", __FUNCTION__);

        clk_disable(si->sir_clk);
        clk_put(si->sir_clk);
        return 0;
}


static int s3c_irda_probe(struct platform_device *pdev)
{
        struct net_device *dev;
        struct s3c_irda *si;
        unsigned int baudrate_mask;
        int err;

        DBG("%s\r\n", __FUNCTION__);

        dev = alloc_irdadev(sizeof(struct s3c_irda));
        if(!dev){
        	printk("alloc_irdadev Error! \r\n");
                return -ENOMEM;
        }

        si = dev->priv;
        si->dev = &pdev->dev;
        si->pdata = pdev->dev.platform_data;

        if((err = s3c_irda_init_mem(si, pdev)) != 0)
                goto err_mem;


        err = s3c_irda_init_iobuf(&si->rx_buff, 14384);
        if(err)
                goto err_iobuf_rx;

        err = s3c_irda_init_iobuf(&si->tx_buff, 14384);
        if(err)
                goto err_iobuf_tx;

        dev->hard_start_xmit    = s3c_irda_hard_xmit;
        dev->open               = s3c_irda_start;
        dev->stop               = s3c_irda_stop;
        dev->do_ioctl           = s3c_irda_ioctl;
        dev->get_stats          = s3c_irda_stats;
        
	si->sir_irq_rx = platform_get_irq(pdev, 0);

        if(si->sir_irq_rx == 0) {
                printk("failed to get rx interrupt resource.\n");
                goto err_irq;
        }

	si->sir_irq_tx = platform_get_irq(pdev, 1);                              
                                                           
        if(si->sir_irq_tx == 0) {                        
                printk("failed to get tx interrupt resource.\n");   
                goto err_irq;                                 
        }                      

        if(s3c_irda_init_clk(&pdev->dev, si)!= 0)
                    goto err_irq;

        irda_init_max_qos_capabilies(&si->qos);

        baudrate_mask = IR_9600 | IR_19200 | IR_38400 | IR_57600 | IR_115200;

        switch (max_rate) {
                case 115200:            baudrate_mask |= IR_115200;
                case 57600:             baudrate_mask |= IR_57600;
                case 38400:             baudrate_mask |= IR_38400;
                case 19200:             baudrate_mask |= IR_19200;
        }

        si->qos.baud_rate.bits &= baudrate_mask;
        si->qos.min_turn_time.bits = 7;

        irda_qos_bits_to_value(&si->qos);

        sir_writereg(0, S3C_UCON);
        sir_writereg(0, S3C_ULCON);

        err = register_netdev(dev);
        if(err == 0) {
                platform_set_drvdata(pdev, dev);
        	DBG("%s success \r\n", __FUNCTION__);
	}

        if(err) {
                s3c_irda_stop_clk(si);
 err_irq:
                kfree(si->tx_buff.head);
 err_iobuf_tx:
                kfree(si->rx_buff.head);
 err_iobuf_rx:
                s3c_irda_free_mem(si);
 err_mem:
                free_netdev(dev);
        }
        return err;
}


static int s3c_irda_remove(struct platform_device *pdev)
{
        struct net_device *dev = platform_get_drvdata(pdev);

        DBG("%s\r\n", __FUNCTION__);

        if(dev) {
                struct s3c_irda *si = dev->priv;
                unregister_netdev(dev);
                kfree(si->tx_buff.head);
                kfree(si->rx_buff.head);
                s3c_irda_free_mem(si);
                s3c_irda_stop_clk(si);
                free_netdev(dev);
        }

        return 0;
}


static struct platform_driver s3c_irda_driver = {
        .probe          = s3c_irda_probe,
        .remove         = s3c_irda_remove,
        .suspend        = s3c_irda_suspend,
        .resume         = s3c_irda_resume,
	.driver         = {
		.name   = "s3c-irda",
	},
};

static char banner[] = KERN_INFO "S3C IrDA driver, (c) 2009 Samsung Electronics\n";

static int __init s3c_irda_init(void)
{
        printk(banner);
        return platform_driver_register(&s3c_irda_driver);

}


static void __exit s3c_irda_exit(void)
{
        platform_driver_unregister(&s3c_irda_driver);
}


module_init(s3c_irda_init);
module_exit(s3c_irda_exit);

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("S3C IrDA driver");
MODULE_LICENSE("GPL");

