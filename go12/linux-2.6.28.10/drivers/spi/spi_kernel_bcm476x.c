/*
 * Broadcom bcm476x SPI master controller (kernel mode)
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/broadcom/gpio.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/arch/hardware.h>
#include <asm/delay.h>
#include <asm/arch/hardware.h>
#include <asm/arch/spi.h>
#include <asm/arch/spi_ioctl.h>
#include <asm/arch/dma.h>
#include <asm/arch/reg_gpio.h>
#include <asm/arch/bcm4760_reg.h>

//#define USE_TASKLET_TRANSFER

//#define PIN_NUM             16  /*interrupt pin GPIO 16*/

#define CS_ASSERT           1
#define CS_DEASSERT         0

#define MAX_BUSES           3

#define DMA_INT_MASK        (DCSR_ENDINTR | DCSR_STARTINTR | DCSR_BUSERR)
#define RESET_DMA_CHANNEL   (DCSR_NODESC | DMA_INT_MASK)
#define IS_DMA_ALIGNED(x)   (((u32)(x)&0x07)==0)


#define START_STATE         ((void*)0)
#define RUNNING_STATE       ((void*)1)
#define DONE_STATE          ((void*)2)
#define ERROR_STATE         ((void*)-1)

#define QUEUE_RUNNING         0
#define QUEUE_STOPPED         1


// SPI registers
#define MMAP_SPI0_BASE          IO_ADDRESS (SPI0_REG_BASE_ADDR)     /*  SPI 0 */
#define MMAP_SPI1_BASE          IO_ADDRESS (SPI1_REG_BASE_ADDR)     /*  SPI 1 */

#define SPI0_CLOCK_MGR_ADDR     IO_ADDRESS ((CMU_REG_BASE_ADDR + 0x58))
#define SPI1_CLOCK_MGR_ADDR     IO_ADDRESS ((CMU_REG_BASE_ADDR + 0x58))
#define SPI0_CLOCK_DIV_MGR_ADDR IO_ADDRESS ((CMU_REG_BASE_ADDR + 0x84))
#define SPI1_CLOCK_DIV_MGR_ADDR IO_ADDRESS ((CMU_REG_BASE_ADDR + 0x84))

#define SPI0_CLOCK_EN_BIT       17
#define SPI1_CLOCK_EN_BIT       18

// SELECTION OF SPI PORT
#define MMAP_SPI_BASE           (drv_data->master->bus_num ? MMAP_SPI1_BASE : MMAP_SPI0_BASE)
#define SPI_CLOCK_EN_BIT        (drv_data->master->bus_num ? SPI1_CLOCK_EN_BIT : SPI0_CLOCK_EN_BIT)
#define SPI_CLOCK_MGR_ADDR      (drv_data->master->bus_num ? SPI1_CLOCK_MGR_ADDR : SPI0_CLOCK_MGR_ADDR)

// SPI1 GPIO Pin Share
#define	SPI_CLK_1_N_GPIO	   8       // func 1
#define	SPI_RX_1_N_GPIO	       9       // func 1
#define	SPI_TX_1_N_GPIO		   10      // func 1

// SPI1 GPIO Chip Select
#define SPI_FSS1_0_N_GPIO      11      // SPI1 CS dev 0 (func 1)
#define SPI_FSS1_1_N_GPIO      19      // SPI1 CS dev 1 (func 2)
#define SPI_FSS1_2_N_GPIO      20      // SPI1 CS dev 2 (func 2)
#define SPI_FSS1_3_N_GPIO      21      // SPI1 CS dev 3 (func 2)

// SPI0 GPIO Chip Select
#define SPI_FSS0_0_N_GPIO      67      // SPIO CS dev 0 (func gpio) 
#define SPI_FSS0_3_N_GPIO      25      // SPI0 CS dev 1 (func 1)
#define SPI_FSS0_2_N_GPIO      26      // SPI0 CS dev 2 (func 1)
#define SPI_FSS0_1_N_GPIO      27      // SPI0 CS dev 3 (func 1)

// END OF SPI PORT SELECTION
#define MMAP_CHIP_MGR_BASE      IO_ADDRESS (CMU_REG_BASE_ADDR)
#define GPIO_PINMUX_BASE        (MMAP_CHIP_MGR_BASE + 0x414)

#define SPI_SSPCR0              ((MMAP_SPI_BASE) 	+ 0x00)
#define SPI_SSPCR1              ((MMAP_SPI_BASE) 	+ 0x04)
#define SPI_SSPDR               ((MMAP_SPI_BASE) 	+ 0x08)
#define SPI_SSPSR               ((MMAP_SPI_BASE) 	+ 0x0c)
#define SPI_SSPCPSR             ((MMAP_SPI_BASE) 	+ 0x10)
#define SPI_SSPIMSC             ((MMAP_SPI_BASE) 	+ 0x14)
#define SPI_SSPRIS              ((MMAP_SPI_BASE) 	+ 0x18)
#define SPI_SSPMIS              ((MMAP_SPI_BASE) 	+ 0x1c)
#define SPI_SSPICR              ((MMAP_SPI_BASE) 	+ 0x20)
#define SPI_SSPDMACR            ((MMAP_SPI_BASE) 	+ 0x24)

/* Clock Input Configuration */
#define BLOCK_INPUT_CLK		12000000	/* 12 MHz */
#define SSPCPSR_PRESCALE_RATE	2		/* prescale divisor */ 

/**
 * Default Control registers settings.
 */
#define SSPCR1_DEF	0	/* SPI Master, no loopback. */


typedef enum 
{
	e4BITS = 0x3,
	e5BITS,
	e6BITS,
	e7BITS,
	e8BITS,
	e9BITS,
	e10BITS,
	e11BITS,
	e12BITS,
	e13BITS,
	e14BITS,
	e15BITS,
	e16BITS
} bits_per_word_t;

enum pinmux_fsel_t {
    GPIO_ALT0        = 0, /* default */
    GPIO_ALT1        = 1,
    GPIO_ALT2        = 2,
    GPIO_ALT3        = 3
};

struct driver_data {
    /* Driver model hookup */
    struct  platform_device *pdev;

    /* SPI framework hookup */
    struct  spi_master *master;

    /* DMA setup stuff */
    int     rx_channel;
    int     tx_channel;
    u32     *null_dma_buf;

    /* Driver message queue */
    struct  workqueue_struct    *workqueue;
    struct  work_struct pump_messages;
    spinlock_t  lock;
    struct  list_head queue;
    int     busy;
    int     run;
    int     cs_gpio;     // chip select gpio

#ifdef USE_TASKLET_TRANSFER
    /* Message Transfer pump */
    struct  tasklet_struct pump_transfers;
#endif

    /* Current message transfer state info */
    struct  spi_message* cur_msg;
    struct  spi_transfer* cur_transfer;
    struct  chip_data *cur_chip;
    size_t  len;
    void    *tx;
    void    *tx_end;
    void    *rx;
    void    *rx_end;
    int     dma_mapped;
    dma_addr_t  rx_dma;
    dma_addr_t  tx_dma;
    size_t  rx_map_len;
    size_t  tx_map_len;
    u8      n_bytes;
    u32     dma_width;
    int     cs_change;
    void    (*write)(struct driver_data *drv_data);
    void    (*read)(struct driver_data *drv_data);
    irqreturn_t (*transfer_handler)(struct driver_data *drv_data);
    void    (*cs_control)(struct driver_data *drv_data, u32 command);
};

struct chip_data {
    u32     cr0;
    u32     cr1;
    u32     to;
    u32     psp;
    u32     timeout;
    u8      n_bytes;
    u32     dma_width;
    u32     dma_burst_size;
    u32     threshold;
    u32     dma_threshold;
    u8      enable_dma;
    u8      bits_per_word;
    u32     speed_hz;
    void    (*write)(struct driver_data *drv_data);
    void    (*read)(struct driver_data *drv_data);
    void    (*cs_control)(struct driver_data *drv_data, u32 command);
};

static void bcm476x_pump_messages(struct work_struct *data);

/*
static irqreturn_t bcm476x_spi_irq(int irq, void *data)
{
    unsigned int status, gpio;

    disable_irq(IRQ_GPIO_0);
    
    gpio = PIN_NUM;
    status = gpio_get_interrupt_status(gpio);
    if (status) {
        gpio_clear_interrupt(gpio);
    }
    
    enable_irq(IRQ_GPIO_0);
    return IRQ_HANDLED;
}
*/

static int flush(struct driver_data *drv_data)
{
    return loops_per_jiffy << 1;
}

static void restore_state(struct driver_data *drv_data)
{
}

static void null_cs_control(struct driver_data *drv_data, u32 command)
{
}

static void null_writer(struct driver_data *drv_data)
{
}

static void null_reader(struct driver_data *drv_data)
{
}

/* caller already set message->status; dma and pio irqs are blocked */
static void giveback(struct driver_data *drv_data)
{
    struct spi_transfer* last_transfer;
    struct spi_message *msg;

#ifdef USE_TASKLET_TRANSFER  
    unsigned long flags;
    spin_lock_irqsave(&drv_data->lock, flags);
#endif
    
    msg = drv_data->cur_msg;
    drv_data->cur_msg = NULL;
    drv_data->cur_transfer = NULL;
    drv_data->cur_chip = NULL;
    queue_work(drv_data->workqueue, &drv_data->pump_messages);
#ifdef USE_TASKLET_TRANSFER
    spin_unlock_irqrestore(&drv_data->lock, flags);
#endif

    last_transfer = list_entry(msg->transfers.prev,
                    struct spi_transfer,
                    transfer_list);

	if (!last_transfer->cs_change)
		drv_data->cs_control(drv_data, CS_DEASSERT);

    msg->state = NULL;
    if (msg->complete)
        msg->complete(msg->context);
}

static void spi_flush (struct driver_data *drv_data)
{
	while (!(inb(SPI_SSPSR) & SPI_SSPSR_TFE)) {
		outw (SSPCR1_DEF |  SPI_SSPSCR1_SSE, SPI_SSPCR1);
		outw (SSPCR1_DEF & ~SPI_SSPSCR1_SSE, SPI_SSPCR1);
	}

	while (inb(SPI_SSPSR) & SPI_SSPSR_RNE)
		inw (SPI_SSPDR);
}


static void    gpio_pinmux_fsel    (int32_t gpio, enum pinmux_fsel_t func)
{
    uint32_t regval;
    uint32_t regaddr;
    uint32_t shift;

    regaddr = GPIO_PINMUX_BASE;
    shift = (gpio - 8) * 3;
    regval = inl (regaddr);
    regval &= ~(0x3 << shift);
    regval |= (func << shift);
    outl (regval, regaddr);
}


static void *next_transfer(struct driver_data *drv_data)
{
    struct spi_message *msg = drv_data->cur_msg;
    struct spi_transfer *trans = drv_data->cur_transfer;

    /* Move to next transfer */
    if (trans->transfer_list.next != &msg->transfers) {
        drv_data->cur_transfer =
            list_entry(trans->transfer_list.next,
                    struct spi_transfer,
                    transfer_list);
        return RUNNING_STATE;
    }
    else
        return DONE_STATE;
}

// Chip select map index by bus number and chip-select index
struct gpio_func_map { 
        u8 gpio;
        u8 func;
};

static struct gpio_func_map cs_map[2][4] = {
    /* SPI0-map */
    {
    {SPI_FSS0_0_N_GPIO, GPIO_ALT0},
    {SPI_FSS0_1_N_GPIO, GPIO_ALT0},
    {SPI_FSS0_2_N_GPIO, GPIO_ALT0},
    {SPI_FSS0_3_N_GPIO, GPIO_ALT0},
    },
    /* SPI1-map */
    {
    {SPI_FSS1_0_N_GPIO, GPIO_ALT0},
    {SPI_FSS1_1_N_GPIO, GPIO_ALT0},
    {SPI_FSS1_2_N_GPIO, GPIO_ALT0},
    {SPI_FSS1_3_N_GPIO, GPIO_ALT0},
    }
};

//static 
void spi_dev_select(int bus, int cs)
{
	struct gpio_func_map *gpio_map = &cs_map[bus][cs];

	gpio_pinmux_fsel ( gpio_map->gpio, gpio_map->func );
	/* Take CS low */
	gpio_direction_output( gpio_map->gpio, 0 );
}

//static 
void spi_dev_deselect(int bus, int cs)
{
    struct gpio_func_map *gpio_map = &cs_map[bus][cs];
   
    /* Take CS high */
    gpio_set_value( gpio_map->gpio, 1 );
}

/* Set up spi pinmuxing */
//static
int spi_pinmuxing_init(struct spi_device	*spi)
{
    int cs = spi->chip_select; 

	if (cs > spi->master->num_chipselect) {
        return -EINVAL;
    }

    if (spi->master->bus_num) {
        gpio_pinmux_fsel ( SPI_TX_1_N_GPIO, GPIO_ALT1 );
        gpio_pinmux_fsel ( SPI_RX_1_N_GPIO, GPIO_ALT1 );
        gpio_pinmux_fsel ( SPI_CLK_1_N_GPIO, GPIO_ALT1);
    }

   if (cs >= 0) 
       spi_dev_select(spi->master->bus_num, cs);
   return 0;
}

static void spi_pinmuxing_deinit(struct spi_device	*spi)
{
    int cs = spi->chip_select; 

	if (cs > spi->master->num_chipselect) {
        return;
    }

    if (cs >= 0) {
        spi_dev_deselect(spi->master->bus_num, cs);
        gpio_free(cs);
    }
}

bits_per_word_t bcm476x_get_bits_per_word(u8 bits_per_word)
{
	switch(bits_per_word) {
		case 4: 
			return e4BITS;
		case 5: 
			return e5BITS;
		case 6: 
			return e6BITS;
		case 7: 
			return e7BITS;
		case 8: 
			return e8BITS;
		case 9: 
			return e9BITS;
		case 10: 
			return e10BITS;
		case 11: 
			return e11BITS;
		case 12: 
			return e12BITS;
		case 13: 
			return e13BITS;
		case 14: 
			return e14BITS;
		case 15: 
			return e15BITS;
		case 16:
			return e16BITS; 
		default:
			return e8BITS; 
	}
}

void bcm476x_config_transfer(struct spi_device *spi, struct driver_data *drv_data)
{
	bits_per_word_t		bits_per_word;
	u32			speed_hz;
	u32			clock_rate;
	u32			val_sspcr0;
	struct spi_transfer	*transfer = drv_data->cur_transfer;

	if (transfer->bits_per_word != 0)
		bits_per_word = bcm476x_get_bits_per_word(transfer->bits_per_word);
	else
		bits_per_word = bcm476x_get_bits_per_word(spi->bits_per_word);	/* use default. */

	if (transfer->speed_hz != 0)
		speed_hz = transfer->speed_hz;
	else
		speed_hz = spi->max_speed_hz;	/* use default. */

	clock_rate  = (BLOCK_INPUT_CLK - speed_hz) / speed_hz; 
	clock_rate  = (clock_rate & SPI_SSPCR0_SCR_MASK) << SPI_SSPCR0_SCR_SHIFT;

	val_sspcr0  = inw(SPI_SSPCR0);
	val_sspcr0  &= ~(SPI0_F_SCR_MASK|SPI0_F_SPH_MASK|SPI0_F_SPO_MASK|SPI0_F_DSS_MASK);
	val_sspcr0  |= clock_rate | bits_per_word;
	if(spi->mode&SPI_CPHA)
		val_sspcr0 |= SPI0_F_SPH_MASK;
	if(spi->mode&SPI_CPOL)
		val_sspcr0 |= SPI0_F_SPO_MASK;
	
	outw(val_sspcr0, SPI_SSPCR0);	/* set freq, SPI phase & polarity, word size. */
        outw(SSPCR1_DEF | SPI_SSPSCR1_SSE, SPI_SSPCR1);	/* enable SSP */
}

void bcm476x_handle_transfers(struct spi_device *spi, struct driver_data *drv_data)
{
	do {
		struct spi_transfer *transfer = drv_data->cur_transfer;
		int i;
		int fifo_transfer_len = transfer->len; /* number of fifo transfers */

		bcm476x_config_transfer(spi, drv_data);
		spi_flush(drv_data);

		if ( transfer->bits_per_word > 8 ) {
			fifo_transfer_len >>= 1;
		}
		for (i = 0 ; i < fifo_transfer_len ; i++) {
			if (transfer->tx_buf)
			{
				if (transfer->bits_per_word > 8)
					outw (((u16)((u16*)transfer->tx_buf)[i]), SPI_SSPDR);
				else
					outw (((u16)((u8*)transfer->tx_buf)[i]), SPI_SSPDR);
			}
			else
                		outw (0x0000, SPI_SSPDR); /* sent dummy during read cycle */
			
			while (inb(SPI_SSPSR) & SPI_SSPSR_BSY);
			
			if (transfer->rx_buf)
			{
				if (transfer->bits_per_word > 8)
					((u16*)transfer->rx_buf)[i] = inw(SPI_SSPDR);
				else
					((u8*)transfer->rx_buf)[i] = (u8)inw(SPI_SSPDR);
			}
		}
		drv_data->cur_msg->actual_length += transfer->len; 
	} while((drv_data->cur_msg->state = next_transfer(drv_data)) != DONE_STATE);
}

void bcm476x_pump_transfers(unsigned long data)
{
    struct driver_data *drv_data = (struct driver_data *)data;
    struct spi_message *message = NULL;
    struct spi_transfer *transfer = NULL;
    struct spi_transfer *previous = NULL;
    struct chip_data *chip = NULL;
    struct spi_device	*spi;

    //TEST
    drv_data->cs_control = null_cs_control;

    /* Get current state information */
    message = drv_data->cur_msg;
    transfer = drv_data->cur_transfer;
    chip = drv_data->cur_chip;

    /* Handle for abort */
    if (message->state == ERROR_STATE) {
        message->status = -EIO;
        giveback(drv_data);
        return;
    }

    /* Handle end of message */
    if (message->state == DONE_STATE) {
        message->status = 0;
        giveback(drv_data);
        return;
    }

    /* Delay if requested at end of transfer*/
    if (message->state == RUNNING_STATE) {
        previous = list_entry(transfer->transfer_list.prev,
                    struct spi_transfer,
                    transfer_list);
        if (previous->delay_usecs)
            udelay(previous->delay_usecs);
    }

    /* Setup the transfer state based on the type of transfer */
    if (flush(drv_data) == 0) {
        //dev_err(&drv_data->pdev->dev, "pump_transfers: flush failed\n");
        message->status = -EIO;
        giveback(drv_data);
        return;
    }
    drv_data->n_bytes = chip->n_bytes;
    drv_data->dma_width = chip->dma_width;
    drv_data->cs_control = chip->cs_control;
    drv_data->tx = (void *)transfer->tx_buf;
    drv_data->tx_end = drv_data->tx + transfer->len;
    drv_data->rx = transfer->rx_buf;
    drv_data->rx_end = drv_data->rx + transfer->len;
    drv_data->rx_dma = transfer->rx_dma;
    drv_data->tx_dma = transfer->tx_dma;
    drv_data->len = transfer->len;
    drv_data->write = drv_data->tx ? chip->write : null_writer;
    drv_data->read = drv_data->rx ? chip->read : null_reader;
    drv_data->cs_change = transfer->cs_change;

    /* Setup pinmuxing */
    spi = message->spi;
    spi_pinmuxing_init(spi);

    outw (SSPCPSR_PRESCALE_RATE, SPI_SSPCPSR);	/* Prescale Divisor. */
    outw (0, SPI_SSPIMSC);			/* don't use interrupts */

    bcm476x_handle_transfers(spi, drv_data);
    spi_pinmuxing_deinit(spi);

    giveback(drv_data);
    message->status = 0;

    message->state = RUNNING_STATE;
}

static void bcm476x_pump_messages(struct work_struct *work)
{
    //struct driver_data *drv_data = data;
    struct driver_data *drv_data =
                container_of(work, struct driver_data, pump_messages);
    unsigned long flags;
    
    /* Lock queue and check for queue work */
    spin_lock_irqsave(&drv_data->lock, flags);
    if (list_empty(&drv_data->queue) || drv_data->run == QUEUE_STOPPED) {
        drv_data->busy = 0;
        spin_unlock_irqrestore(&drv_data->lock, flags);
        return;
    }

    /* Make sure we are not already running a message */
    if (drv_data->cur_msg) {
        spin_unlock_irqrestore(&drv_data->lock, flags);
        return;
    }

    /* Extract head of queue */
    drv_data->cur_msg = list_entry(drv_data->queue.next,
                    struct spi_message, queue);
    list_del_init(&drv_data->cur_msg->queue);

    /* Initial message state*/
    drv_data->cur_msg->state = START_STATE;

    drv_data->cur_transfer = list_entry(drv_data->cur_msg->transfers.next,
                        struct spi_transfer,
                        transfer_list);

    /* Setup the SSP using the per chip configuration */
    drv_data->cur_chip = spi_get_ctldata(drv_data->cur_msg->spi);
    restore_state(drv_data);

#ifdef USE_TASKLET_TRANSFER
    /* Mark as busy and launch transfers */
    tasklet_schedule(&drv_data->pump_transfers);
#else
    bcm476x_pump_transfers ((unsigned long) (drv_data) );
#endif

    drv_data->busy = 1;
    spin_unlock_irqrestore(&drv_data->lock, flags);
}

static int transfer(struct spi_device *spi, struct spi_message *msg)
{
    struct driver_data *drv_data = spi_master_get_devdata(spi->master);
    unsigned long flags;

    spin_lock_irqsave(&drv_data->lock, flags);

    if (drv_data->run == QUEUE_STOPPED) {
        spin_unlock_irqrestore(&drv_data->lock, flags);
        return -ESHUTDOWN;
    }

    msg->actual_length = 0;
    msg->status = -EINPROGRESS;
    msg->state = START_STATE;

    list_add_tail(&msg->queue, &drv_data->queue);

    if (drv_data->run == QUEUE_RUNNING && !drv_data->busy)
        queue_work(drv_data->workqueue, &drv_data->pump_messages);

    spin_unlock_irqrestore(&drv_data->lock, flags);

    return 0;
}

static int setup(struct spi_device *spi)
{
    struct chip_data *chip;

    if (!spi->bits_per_word)
        spi->bits_per_word = 8;

    /* Only destroy_queue (or use chip_info) on first setup */
    chip = spi_get_ctldata(spi);
    if (chip == NULL) {
        chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
        if (!chip)
            return -ENOMEM;

        chip->cs_control = null_cs_control;
        chip->enable_dma = 0;
    }

    chip->speed_hz = spi->max_speed_hz;

    chip->bits_per_word = spi->bits_per_word;

    spi_set_ctldata(spi, chip);

    return 0;
}

static void cleanup(struct spi_device *spi)
{
    struct chip_data *chip = spi_get_ctldata((struct spi_device *)spi);

    kfree(chip);
}

static int init_queue(struct driver_data *drv_data)
{
    INIT_LIST_HEAD(&drv_data->queue);
    spin_lock_init(&drv_data->lock);

    drv_data->run = QUEUE_STOPPED;
    drv_data->busy = 0;

#ifdef USE_TASKLET_TRANSFER
    tasklet_init(&drv_data->pump_transfers,
            bcm476x_pump_transfers,    (unsigned long)drv_data);
#endif

    INIT_WORK(&drv_data->pump_messages, bcm476x_pump_messages);
    drv_data->workqueue = create_singlethread_workqueue("SPI-drv");
    if (drv_data->workqueue == NULL)
        return -EBUSY;

    return 0;
}

static int start_queue(struct driver_data *drv_data)
{
    unsigned long flags;

    spin_lock_irqsave(&drv_data->lock, flags);

    if (drv_data->run == QUEUE_RUNNING || drv_data->busy) {
        spin_unlock_irqrestore(&drv_data->lock, flags);
        return -EBUSY;
    }

    drv_data->run = QUEUE_RUNNING;
    drv_data->cur_msg = NULL;
    drv_data->cur_transfer = NULL;
    drv_data->cur_chip = NULL;
    spin_unlock_irqrestore(&drv_data->lock, flags);

    queue_work(drv_data->workqueue, &drv_data->pump_messages);

    return 0;
}

static int stop_queue(struct driver_data *drv_data)
{
    unsigned long flags;
    unsigned limit = 500;
    int status = 0;

    spin_lock_irqsave(&drv_data->lock, flags);

    /* This is a bit lame, but is optimized for the common execution path.
     * A wait_queue on the drv_data->busy could be used, but then the common
     * execution path (pump_messages) would be required to call wake_up or
     * friends on every SPI message. Do this instead */
    drv_data->run = QUEUE_STOPPED;
    while (!list_empty(&drv_data->queue) && drv_data->busy && limit--) {
        spin_unlock_irqrestore(&drv_data->lock, flags);
        msleep(10);
        spin_lock_irqsave(&drv_data->lock, flags);
    }

    if (!list_empty(&drv_data->queue) || drv_data->busy)
        status = -EBUSY;

    spin_unlock_irqrestore(&drv_data->lock, flags);

    return status;
}

static int destroy_queue(struct driver_data *drv_data)
{
    int status;

    status = stop_queue(drv_data);
    if (status != 0)
        return status;

    destroy_workqueue(drv_data->workqueue);

    return 0;
}

static int bcm476x_spi_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct bcm476x_spi_master *platform_info;
    struct spi_master *master;
    struct driver_data *drv_data = 0;
    int status = 0;

    platform_info = dev->platform_data;

    /* Allocate master with space for drv_data and null dma buffer */
    master = spi_alloc_master(dev, sizeof(struct driver_data) + 16);
    if (!master) {
        dev_err( dev, "can not alloc spi_master\n");
        return -ENOMEM;
    }

    drv_data                = spi_master_get_devdata(master);
    drv_data->master        = master;

    master->bus_num         = to_platform_device(dev)->id;
    master->num_chipselect  = platform_info->num_chipselect;
    master->cleanup         = cleanup;
    master->setup           = setup;
    master->transfer        = transfer;

    drv_data->null_dma_buf = (u32 *)ALIGN((u32)(drv_data +
                        sizeof(struct driver_data)), 8);


    /* Setup DMA if requested */
    drv_data->tx_channel    = -1;
    drv_data->rx_channel    = -1;

    /* Initial and start queue */
    status = init_queue(drv_data);
    if (status != 0) {
        dev_err( dev, "problem initializing queue\n");
        goto out_error_clock_enabled;
    }

    status = start_queue(drv_data);
    if (status != 0) {
        dev_err( dev, "problem starting queue\n");
        goto out_error_clock_enabled;
    }

        /* Register with the SPI framework */
    platform_set_drvdata( pdev, drv_data );

    status = spi_register_master(master);
    if (status != 0) {
        dev_err( dev, "problem registering spi master\n");
        goto out_error_queue_alloc;
    }
/*
    gpio_set_interrupt_trigger_on_falling_edge   ( PIN_NUM, 1 );
    gpio_set_interrupt_debounce_enable           ( PIN_NUM, 0 );
    gpio_set_interrupt_mask                      ( PIN_NUM, 1 );
    gpio_set_type_control                        ( PIN_NUM, GPIO_TC_INPUT_WITH_INTERRUPT );
    gpio_set_pull_up_down_enable                 ( PIN_NUM, 1 );
    gpio_set_pull_up_down_is_up                  ( PIN_NUM, 1 );

    int ret = request_irq(IRQ_GPIO_0, bcm476x_spi_irq, IRQF_DISABLED|IRQF_SHARED,
            "enc28j60", NULL);
    
    if (ret < 0) {
        //dev_err (dev, "error int\n");
    }
*/
    return status;

out_error_queue_alloc:
    destroy_queue(drv_data);

out_error_clock_enabled:
      return status;
}

static int bcm476x_spi_remove(struct platform_device *pdev)
{
    struct driver_data *drv_data = platform_get_drvdata(pdev);
    int status = 0;

    if (!drv_data)
        return 0;

    /* Remove the queue */
    status = destroy_queue(drv_data);
    if (status != 0)
        return status;

    spi_unregister_master(drv_data->master);

    platform_set_drvdata(pdev, NULL);

    return 0;
}

static void bcm476x_spi_shutdown(struct platform_device *pdev)
{
    int status = 0;

    if ((status = bcm476x_spi_remove(pdev)) != 0)
        dev_err(&pdev->dev, "shutdown failed with %d\n", status);

}

#ifdef CONFIG_PM
static int bcm476x_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
    return 0;
}

static int bcm476x_spi_resume(struct platform_device *pdev)
{
    return 0;
}
#else
#define bcm476x_spi_suspend     NULL
#define bcm476x_spi_resume      NULL
#endif

static struct platform_driver bcm476x_driver = {
    .driver = {
        .name   = "spi",
        .owner  = THIS_MODULE,
    },
    .probe      = bcm476x_spi_probe,
    .remove     = bcm476x_spi_remove,
    .shutdown   = bcm476x_spi_shutdown,
    .suspend    = bcm476x_spi_suspend,
    .resume     = bcm476x_spi_resume
};


static int __init bcm476x_spi_init(void)
{
    printk(KERN_INFO "bcm476x_spi_init\n");
    return platform_driver_probe (&bcm476x_driver, bcm476x_spi_probe);
}

module_init(bcm476x_spi_init);

static void __exit bcm476x_spi_exit(void)
{
    printk(KERN_INFO "bcm476x_spi_exit\n");
    platform_driver_unregister (&bcm476x_driver);
}

module_exit(bcm476x_spi_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("bcm476x SPI Contoller");
MODULE_LICENSE("GPL");

