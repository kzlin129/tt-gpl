/* si4703.c
 *
 * Control driver for Silicon Lab 4703 chip.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Xander Hover <Xander.Hover@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/fmreceiver.h>
#include <linux/si4703.h>
#include <linux/device.h>
#include <barcelona/gopins.h>
#include <linux/delay.h>
#include <asm/semaphore.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <asm/irq.h>
#include <asm/arch/irqs.h>
#include <linux/wait.h>   /* needed to implement blocking read and wait queues */
#include <linux/poll.h>   /* needed for poll_wait() call */
#include <linux/kfifo.h>  /* needed for the kfifo ringbuffer */
#include <barcelona/gopins.h>   /* needed for tomtomgo series HAL */
#include "si470x_bit_fields.h"

#define PFX "SI4703: "
#define PK_DBG(fmt, arg...)    printk(KERN_DEBUG PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_ERR(fmt, arg...)    printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)

//#define LOGGING
#ifdef LOGGING
#define PK_X(fmt, arg...)     printk(fmt, ##arg);
#else
#define PK_X(fmt, arg...)
#endif

#define USE_CRYSTAL  (1)      // 0 = external RCLK, 1 = crystal oscillator
#define POWERUP_TIME (110)    // Powerup delay in milliseconds
#define CRYSTAL_TIME (500)  // Crystal stabilization delay in milliseconds

#define TUNE_TIMEOUT (10)
#define SEEK_TIMEOUT (15)

struct si4703dev {
  wait_queue_head_t wq; /* wait queue for read */
  struct cdev cdev;
  struct i2c_client *client;
  spinlock_t sp_lock;
  struct kfifo *fifo;
};

static atomic_t si4703_available = ATOMIC_INIT(1); /* there shall only be one user of the device */
static gopin_t tmcRX_dock_i2c_en_pin; /* not available in palermo  */
static gopin_t tmcRX_reset_pin; /* reset is active low */
static gopin_t tmcRX_sen_pin; /* sen is active low */
static dev_t tmcRX_devno;
static unsigned int tmcRX_irq=0;  /* the interrupt number of the device */
static u16 s_device_id = 0xFFFF;  /* value when device not recognized */
static u32 rds_bad=0; /* counter for corrupt rds blocks */
static u32 rds_good=0; /* counter for valid rds blocks */

//#define RDSQ_LOGGING
#ifdef RDSQ_LOGGING
static u32 prev_rds_good=0;
static u32 rdsq=0;
static unsigned long rds_timestamp_ms=0;
#endif

/* Spec says SI4703 sits on address 0x10 */
static unsigned short normal_i2c[] = { SI4703_I2C_SLAVE_ADDR, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static void si4703_workqueue_handler(void *arg);
static int si470x_initialize(void);
static int si470x_disable(void);
static int si470x_reset(void);
static int si470x_powerup(void);
static int si470x_enable_i2c(void);
static int si470x_disable_i2c(void);
static int si470x_enable_rds(void);
static int si470x_disable_rds(void);
static int si470x_reg_write(u8 number_of_reg);
static int si470x_reg_read(u8 number_of_reg, u16 *plreg);
static int si470x_set_band(__u16 band);
static int si470x_tune(__u16 freq);
static int si470x_set_frequency(__u16 freq);
static int si470x_seek(struct fmr_seek_param_type seek_param);
static int si470x_seek_next(u8 seekup, u8 seekmode);
static int si470x_set_volume(u16 volume);
static u16 si470x_get_current_frequency(void);
static u8  si470x_get_current_rssi(void);

static int si4703_fifo_alloc(struct si4703dev *dev, int size);
static void si4703_fifo_free(struct si4703dev *dev);
static void si4703_fifo_purge(struct si4703dev *dev);
static int si4703_fifo_len(struct si4703dev *dev);
static int si4703_fifo_put(struct si4703dev *dev, unsigned char *data, int len);
static int si4703_fifo_get(struct si4703dev *dev, unsigned char *data, int len);

static u16 chanToFreq(u16 channel);
static u16 freqToChan(u16 freq);
#ifdef LOGGING
static void si470x_reg_dump(u16 *lreg, int startreg, int endreg);
#endif

static int si4703_attach(struct i2c_adapter *adapter);
static int si4703_detach(struct i2c_client *client);
static int si4703_detect_client(struct i2c_adapter *adapter, int address, int kind);
static int si4703_core_probe(struct device *dev);
static int si4703_core_suspend(struct device *dev, pm_message_t state, u32 level);
static int si4703_core_resume(struct device *dev, u32 level);

static int si4703_open(struct inode *nodep, struct file *filep);
static int si4703_release(struct inode *nodep, struct file *filep);
static int si4703_ioctl(struct inode *nodep, struct file *filep, unsigned int cmd, unsigned long arg);
static unsigned int si4703_poll(struct file *filep, struct poll_table_struct *pq);
static ssize_t si4703_read(struct file *filep, char __user * buf, size_t count, loff_t * pos);

static struct i2c_driver driver = {
  .id = I2C_DRIVERID_SI4703,
  .owner = THIS_MODULE,
  .name = "si4703_i2c",
  .flags = I2C_DF_NOTIFY, /* I2C_DF_NOTIFY is gone in recent kernels */
  .attach_adapter = si4703_attach,
  .detach_client = si4703_detach,
  .driver = {
              .name = SI4703_DEVNAME,
              .owner = THIS_MODULE,
              .probe = si4703_core_probe,
              .suspend = si4703_core_suspend,
              .resume = si4703_core_resume,
              .bus = &platform_bus_type,
            }
};

static struct i2c_client client_template = {
  .name = "si4703_i2c_client",
  .flags = I2C_CLIENT_ALLOW_USE,
  .usage_count = 0,
  .driver = &driver,
  .addr = SI4703_I2C_SLAVE_ADDR,
};

struct file_operations si4703_fops = {
  .owner = THIS_MODULE,
  .open = si4703_open,
  .release = si4703_release,
  .ioctl = si4703_ioctl,
  .poll = si4703_poll,
  .read = si4703_read,
};

struct si4703_registers {
  u16 deviceid;
  u16 chipid;
  u16 powercfg;
  u16 channel;
  u16 sysconfig1;
  u16 sysconfig2;
  u16 sysconfig3;
  u16 test1;
  u16 test2;
  u16 bootconfig;
  u16 statusrssi;
  u16 readchan;
  u16 rdsa;
  u16 rdsb;
  u16 rdsc;
  u16 rdsd;
} __attribute__ ((__packed__));


#define NR_OF_REGISTERS   (16)
#define NR_OF_WRITE_REGS   (6)

typedef struct si4703_registers si4703_registers_t;

static u16 si470x_shadow[NR_OF_REGISTERS];
static si4703_registers_t *preg_shadow = (si4703_registers_t*) si470x_shadow;
static volatile u8 WaitSTCInterrupt = 0;
static struct si4703dev *gdev = NULL;  /* global pointer used everywhere */
static int si4703_suspended = 0;

DECLARE_WORK(rds_data_grabber, si4703_workqueue_handler, (void *) NULL);

static void si4703_workqueue_handler(void *arg)
{
  u16 lreg[NR_OF_REGISTERS];
  si4703_registers_t *plreg = (si4703_registers_t*) lreg;

  if (atomic_read(&si4703_available) != 0)
  {
    PK_X("si4703_workqueue_handler: si4703_available.counter(%d)\n", si4703_available.counter);
    return;
  }
  if (si4703_suspended)
  {
    PK_X("si4703_workqueue_handler: si4703_suspended(%d)\n", si4703_suspended);
    return;
  }

  // This interrupt can fire for RDS or STC. Determine which one it is by
  // checking the shadow status of TUNE and SEEK
  if (!((preg_shadow->powercfg & SI47_SEEK) || (preg_shadow->channel & SI47_TUNE)))
  {
    struct fmr_rds_type rdsoutdata;
    u8 rdssync;

    if (si470x_reg_read(6, lreg) != 0)
    {
      PK_ERR("ERROR @ %d, si470x_reg_read\n", __LINE__);
      return;
    }
#ifdef LOGGING
#ifdef DUMP_RDS_DATA
    PK_X("si4703_workqueue_handler : ");
    si470x_reg_dump(lreg, 0x0A, 0x0F);
#endif
#endif

    rdsoutdata.sf_bandlimit = (unsigned char) ((plreg->statusrssi & SI47_SFBL) >> 13);
    rdsoutdata.rssi = (unsigned char) si470x_get_current_rssi();
    rdsoutdata.freq = (unsigned short int) si470x_get_current_frequency();

    rdssync = (plreg->statusrssi & SI47_RDSS) ? 1 : 0;

    if (rdssync)
    {      
      u8 bler[4];
      u8 errors = 0;

      // Gather the latest BLER info
      bler[0] = (plreg->statusrssi & SI47_BLERA) >> 9;
      bler[1] = (plreg->readchan   & SI47_BLERB) >> 14;
      bler[2] = (plreg->readchan   & SI47_BLERC) >> 12;
      bler[3] = (plreg->readchan   & SI47_BLERD) >> 10;

      errors = bler[0] + bler[1] + bler[2] + bler[3];
      if (errors > 0)
      {
        rds_bad++;
        PK_X("-");
      }
      else
      {
        rds_good++;
        PK_X("+");

        if (si4703_fifo_len(gdev) < (54 * sizeof(struct fmr_rds_type)))
        { /* put groups in fifo if it still fits */
          rdsoutdata.blocka = plreg->rdsa;
          rdsoutdata.blockb = plreg->rdsb;
          rdsoutdata.blockc = plreg->rdsc;
          rdsoutdata.blockd = plreg->rdsd;

          si4703_fifo_put(gdev, (u8 *) &rdsoutdata, sizeof(struct fmr_rds_type));
          wake_up_interruptible(&gdev->wq);
        }
      }
#ifdef RDSQ_LOGGING
      if (((rds_bad+rds_good)%10) == 0)
      {
        unsigned long now_ms = jiffies_to_msecs(jiffies);
        unsigned long time_interval_ms = now_ms - rds_timestamp_ms;
        unsigned int rds_count;
        unsigned int rdsq;
        int rds_bad_ratio = 0;

        rds_count = rds_good - prev_rds_good;
        prev_rds_good = rds_good;

        //reset rds timestamp
        rds_timestamp_ms = now_ms;

        rdsq = 100 * (rds_count*100000) / (time_interval_ms * 1131);
        rds_bad_ratio = (1000*rds_bad)/(rds_good+rds_bad);

        printk("si4703: f(%d) rssi(%d) #bad(%d) #good(%d) bad(%d.%d) t(%d) rdsq(%d)\n",
               rdsoutdata.freq,
               rdsoutdata.rssi,
               rds_bad,
               rds_good,
               rds_bad_ratio/10,
               rds_bad_ratio%10,
               (int)time_interval_ms,
               rdsq);
      }
#endif
    }
    else
    {
      //no rds sync
      PK_X("?");
    }
  }
  else
  {
    //TUNE or SEEK finished
    PK_X("STC\n");

    WaitSTCInterrupt = 0;
    wake_up_interruptible(&gdev->wq);
  }
}

static irqreturn_t si4703_interrupthandler(int irq, void *dev_id, struct pt_regs *regs)
{
  schedule_work(&rds_data_grabber);
  return IRQ_HANDLED;
}

static int si4703_fifo_alloc(struct si4703dev *dev, int size)
{
  int ret = 0;
  dev->fifo = kfifo_alloc(size, GFP_KERNEL, &dev->sp_lock);
  
  if (dev->fifo == NULL)
  {
    ret = -1;
  }    

  PK_X("si4703_fifo_alloc return %d\n", ret);
  return ret;
}

static void si4703_fifo_free(struct si4703dev *dev)
{
  kfifo_free(dev->fifo);
}

static void si4703_fifo_purge(struct si4703dev *dev)
{
  kfifo_reset(dev->fifo);
}

static int si4703_fifo_len(struct si4703dev *dev)
{
  return kfifo_len(dev->fifo);
}

static int si4703_fifo_put(struct si4703dev *dev, unsigned char *data, int len)
{
  return kfifo_put(dev->fifo, data, len);
}

static int si4703_fifo_get(struct si4703dev *dev, unsigned char *data, int len)
{
  return kfifo_get(dev->fifo, data, len);
}

static int si4703_open(struct inode *nodep, struct file *filep)
{
  int rc = 0;
  int retries = 0;

  PK_X("si4703_open enter\n");

  /* Only one client may open the device */
  if (!atomic_dec_and_test(&si4703_available))
  {
    atomic_inc(&si4703_available);
    PK_X("si4703_open return -EBUSY\n");
    return -EBUSY;
  }

  PK_X("si4703_open i2c_add_driver(&driver)\n");
  rc = i2c_add_driver(&driver);
  if (rc) 
    goto client_unusable;

  filep->private_data = gdev; /* for easy reference */
  si4703_fifo_purge(gdev);

  do {
    rc = si470x_initialize();
    PK_X("si4703_open: retry(%d) si470x_initialize return %d\n", retries, rc);
    retries++;
    msleep(100);
  } while ((rc < 0) && (retries < 3));  
  if (rc)
    goto client_unusable;

  PK_X("si4703_open i2c_use_client\n");
  rc = i2c_use_client(gdev->client);
  if (rc)
    goto client_unusable;

  enable_irq(tmcRX_irq);
  
  PK_X("si4703_open return\n");
  return 0;

client_unusable:
  PK_ERR("si4703_open return ERROR(%d)\n", rc);
  i2c_del_driver(&driver);

  atomic_inc(&si4703_available);
  return -ENXIO;
}

static int si4703_release(struct inode *nodep, struct file *filep)
{
  PK_X("si4703_release enter\n");  
  
  disable_irq(tmcRX_irq);
  si470x_disable_i2c();
  
  if (filep && filep->private_data) 
  {
    filep->private_data = NULL;
  }

  PK_X("si4703_release:i2c_release_client\n"); 
  if ( i2c_release_client(gdev->client) )
  {
    PK_ERR("Could not release client!\n");
    /* release allways succeeds */
  }

  PK_X("si4703_release i2c_del_driver(&driver)\n");
  i2c_del_driver(&driver);

  /* make device available for others to open */
  atomic_inc(&si4703_available);

  PK_X("si4703_release return\n");
  return 0;
}

static int si4703_ioctl(struct inode *nodep, struct file *filep, unsigned int cmd, unsigned long arg)
{
  int ret = 0;

  if (_IOC_TYPE(cmd) != FMRECEIVER_DRIVER_MAGIC)
  {
    PK_ERR("Wrong IOC type! Failing command\n");
    ret = -ENOTTY;
  }

  switch (cmd) {
  case TMCIOC_RDS_START:
    {
      PK_X("si4703_ioctl TMCIOC_RDS_START\n");
      ret = si470x_enable_rds();
    }
    break;
  case TMCIOC_RDS_STOP:
    {
      PK_X("si4703_ioctl TMCIOC_RDS_STOP\n");
      ret = si470x_disable_rds();
    }
    break;
  case TMCIOC_G_FREQUENCY:
    {
      __u16 freq;

      PK_X("si4703_ioctl TMCIOC_G_FREQUENCY\n");
      freq = si470x_get_current_frequency();
      if (put_user(freq, (__u16 __user *) arg))
      {
        ret = -EFAULT;
      }
    }
    break;
  case TMCIOC_S_FREQUENCY:
    {
      __u16 freq;

      PK_X("si4703_ioctl TMCIOC_S_FREQUENCY\n");
      if (get_user(freq, (__u16 __user *) arg))
      {
        ret = -EFAULT;
      }
      else
      {
        ret = si470x_set_frequency(freq);
      }
    }
    break;
  case TMCIOC_S_BAND:
    {
      __u16 band;

      PK_X("si4703_ioctl TMCIOC_S_BAND\n");
      if (get_user(band, (__u16 __user *) arg))
      {
        ret = -EFAULT;
      }
      else
      {
        ret = si470x_set_band(band);
      }
    }
    break;
  case TMCIOC_S_VOLUME:
    {
      __u16 volume;

      PK_X("si4703_ioctl TMCIOC_S_VOLUME\n");
      if (get_user(volume, (__u16 __user *) arg))
      {
        ret = -EFAULT;
      }
      else
      {
        ret = si470x_set_volume(volume);
      }
    }
    break;
  case TMCIOC_SEEK:
    {
      struct fmr_seek_param_type param;

      PK_X("si4703_ioctl TMCIOC_SEEK\n");
      if (copy_from_user(&param, (void __user *) arg, sizeof(struct fmr_seek_param_type)))
      {
        ret = -EFAULT;
      }
      else
      {
        ret = si470x_seek(param);
      }
    }
    break;
  case TMCIOC_QUERY_DEVICE_ID:
    {
      __u16 devid = s_device_id;
      PK_X("si4703_ioctl TMCIOC_QUERY_DEVICE_ID\n");

      if (put_user(devid, (__u16 __user *) arg))
      {
        ret = -EFAULT;
      }
    }
    break;
  case TMCIOC_Q_RSSI:
    {
      __u16 rssi = 0;

      PK_X("si4703_ioctl TMCIOC_Q_RSSI\n");
      //update shadow, to get up to date rssi
      if (si470x_reg_read(1, si470x_shadow) != 0)
      {
        PK_ERR("ERROR @ %d, si470x_reg_read\n", __LINE__);
        ret = -1;
      }
      else
      {
        rssi = si470x_get_current_rssi();
        if (put_user(rssi, (__u16 __user *) arg))
        {
          ret = -EFAULT;
        }
      }
    }
    break;
  default:
    {
      /* other commands not supported by this device */
      ret = -ENOTTY;
    }
    break;
  }
  PK_X("si4703_ioctl return %d\n", ret);
  return ret;
}

/* we are implementing poll to support the select/poll/epoll system calls
 * This is done to allow the userspace app to check whether it is possible
 * to read non-blocking */
static unsigned int si4703_poll(struct file *filep, struct poll_table_struct *pq)
{
  unsigned int mask = 0;

  if (si4703_fifo_len(gdev) > 0)
    mask |= POLLIN | POLLRDNORM;  /* readable */
  else 
  {
    poll_wait(filep, &gdev->wq, pq);
    if (si4703_fifo_len(gdev) > 0)
      mask |= POLLIN | POLLRDNORM;  /* readable */
  }

  return mask;
}

/* read blocks when there is no data, as it should */
static ssize_t si4703_read(struct file *filep, char __user * buf, size_t count, loff_t * pos)
{
  struct fmr_rds_type rdsdata;

  count = sizeof(struct fmr_rds_type);

  while (!(si4703_fifo_len(gdev) > 0)) 
  {
    if (filep->f_flags & O_NONBLOCK)
      goto read_again_error;

    if (wait_event_interruptible(gdev->wq, (si4703_fifo_len(gdev) > 0)))
      goto read_restartsys_error;

  }
  /* ok RDS data is available */
  /* unless, in the mean time, a successful seek purged the fifo. */
  /* then we need to tell the caller to restart the read */
  if (sizeof(struct fmr_rds_type) != si4703_fifo_get(gdev, (u8 *) &rdsdata, sizeof(struct fmr_rds_type)))
  {
    goto read_again_error;
  }

  if (copy_to_user(buf, &rdsdata, count))
  {
    goto read_fault_error;
  }

  return count;

read_fault_error:
  PK_X("si4703_read return result = -EFAULT\n");
  return -EFAULT;
read_again_error:
  PK_X("si4703_read return result = -EAGAIN\n");
  return -EAGAIN;
read_restartsys_error:
  PK_X("si4703_read return result = -ERESTARTSYS\n");
  return -ERESTARTSYS;
}

static int si4703_detect_client(struct i2c_adapter *adapter, int address, int kind)
{
  struct i2c_client *c;
  int rc = 0;

  PK_X("si4703_detect_client enter\n");

  c = kmalloc(sizeof *c, GFP_KERNEL);
  if (!c) 
  {
    return -ENOMEM;
  }
  memcpy(c, &client_template, sizeof *c);
  c->adapter = adapter;

  strcpy(c->name, /*"Silicon Labs 4703" */ SI4703_DEVNAME);
  
  PK_X("si4703_detect_client: i2c_set_clientdata\n"); 
  i2c_set_clientdata(c, gdev);

  PK_X("si4703_detect_client: i2c_attach_client\n"); 
  i2c_attach_client(c);

  gdev->client = c;

  /* what kind of device do we have here? */
  PK_X("si4703_detect_client: Detecting device on adapter: %s, at I2C address: 0x%02X\n", adapter->name, address);

  return rc;
}

static int si4703_attach(struct i2c_adapter *adap)
{
  int ret = 0;
  int i;
  PK_X("si4703_attach enter\n");
  
  // Zero shadow registers to start
  for ( i = 0; i < NR_OF_REGISTERS; i++ )
  {
    si470x_shadow[i] = 0;
  } 

  if (tmcRX_sen_pin) 
  {
    IOP_Deactivate(tmcRX_sen_pin);
    udelay(500);
  }
  si470x_disable_i2c();
  si470x_reset();
  si470x_enable_i2c();

  PK_X("si4703_attach: i2c_probe\n");
  ret = i2c_probe(adap, &addr_data, si4703_detect_client);

  PK_X("si4703_attach: return id=0x%x, name=%s, ret=%d \n", adap->id, adap->algo->name, ret);

  return ret;
}

static int si4703_detach(struct i2c_client *c)
{
  PK_X("si4703_detach enter\n");

  if (c) 
  {
    PK_X("si4703_detach i2c_detach_client(c)\n");    
    i2c_detach_client(c);
  }
  
  PK_X("si4703_detach return\n");
  return 0;
}

static int __init si4703_init(void)
{
  int result = 0 ;

  PK_X("si4703_init enter\n");
  
  PK_X("si4703_init driver_register(&driver.driver)\n");
  result = driver_register(&driver.driver);
  if (result) 
  {
    PK_ERR("Unable to register driver %d\n", result);
    return result;
  }

  gdev = kmalloc(sizeof *gdev, GFP_KERNEL);
  if (!gdev) 
  {
    PK_ERR("si4703_init kmalloc ERROR\n");
    return -ENOMEM;
  }
  memset(gdev, 0, sizeof *gdev);

  init_waitqueue_head(&gdev->wq); /* initialize waitqueue for readers */  
  spin_lock_init(&gdev->sp_lock);
  result = si4703_fifo_alloc(gdev, 55 * sizeof(struct fmr_rds_type));
  if (result) 
  {
    return result;
  }

  /* Now setup our character device */
  PK_X("si4703_init cdev_init\n");
  cdev_init(&gdev->cdev, &si4703_fops);
  gdev->cdev.owner = THIS_MODULE;
  gdev->cdev.ops = &si4703_fops;

  PK_X("si4703_init cdev_add\n");
  result = cdev_add(&gdev->cdev, tmcRX_devno, 1);
  if (result) 
  {
    PK_ERR("Unable to add character device\n");
    return result;
  }

  PK_X("si4703_init request_irq\n");
  result = request_irq(tmcRX_irq, si4703_interrupthandler, SA_INTERRUPT, "si4703", (void *) gdev);
  if (result)
  {
    PK_X("si4703_init: Cannot register interrupt handler for irq %d.\n", tmcRX_irq);
    return result;
  } 
  else 
  {
    PK_X("si4703_init: Interrupt handler registered for irq %d.\n", tmcRX_irq);
  }
  disable_irq(tmcRX_irq);

  PK_X("si4703_init return 0x%x\n", result);
  return result;
}

static void __exit si4703_exit(void)
{
  PK_X("si4703_exit enter\n");

  if (tmcRX_irq != 0)
  {
    PK_X("si4703_exit free_irq\n");
    free_irq(tmcRX_irq, (void *) gdev);
    tmcRX_irq = 0;
  }

  PK_X("si4703_exit driver_unregister(&driver.driver)\n");
  driver_unregister(&driver.driver);

  if (gdev->fifo != NULL) 
  {
    PK_X("si4703_exit si4703_fifo_purge(gdev)\n");
    si4703_fifo_purge(gdev);
    PK_X("si4703_exit si4703_fifo_free(gdev)\n");
    si4703_fifo_free(gdev);
    gdev->fifo = NULL;
  }

  PK_X("si4703_exit cdev_del(&gdev->cdev)\n");
  cdev_del(&gdev->cdev);
  
  if ( gdev )
  {
    PK_X("si4703_exit kfree(gdev)\n");
    kfree(gdev);
    gdev = NULL;
  }
  
  PK_X("si4703_exit return\n");
}

static int si470x_set_band(__u16 band)
{
  u16 spacing=0;
  u16 deemphasis=0;

  PK_X("si470x_set_band enter\n");

  switch (band) 
  {
  case 0U:
    {
      /* Europe, 87.5 - 108.0 MHz, 100 kHz grid */
      band = 0x0000;
      spacing = 0x0010;
      deemphasis = 0x0800;
    }
    break;
  case 1U:
    {
      /* USA, 87.5 - 108.0 MHz, 200 kHz grid */
      band = 0x0000;
      spacing = 0x0000;
      deemphasis = 0x0000;
    }
    break;
  case 2U:
    {
      /* Japan, 76.0 - 90.0 MHz, 100 kHz grid */
      band = 0x0080;
      spacing = 0x0010;
      deemphasis = 0x0800;
    }
    break;
  default:
      return -1;
  }
  preg_shadow->sysconfig2 = (preg_shadow->sysconfig2 & ~0x00FF) | band | spacing | 0x000F;
  preg_shadow->sysconfig1 = (preg_shadow->sysconfig1 & ~0x0800) | deemphasis;
  if (si470x_reg_write(4) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_write\n", __LINE__);
    return -1;
  }

  PK_X("si4703_set_band return\n");
  return 0;
}


static int si470x_set_frequency(__u16 freq)
{
  struct fmr_rds_type rdsoutdata;

  si4703_fifo_purge(gdev);
  
  if (si470x_tune(freq) != 0)
  {
    PK_ERR("ERROR tune failed\n");
    return -1;
  }
  else
  {
    rdsoutdata.sf_bandlimit = (unsigned char) ((preg_shadow->statusrssi & SI47_SFBL) >> 13);
    rdsoutdata.rssi = (unsigned char) si470x_get_current_rssi();
    rdsoutdata.freq = (unsigned short int) si470x_get_current_frequency();
    rdsoutdata.blocka = (unsigned short int) 0x00FF;
    rdsoutdata.blockb = (unsigned short int) 0x0000;
    rdsoutdata.blockc = (unsigned short int) 0x0000;
    rdsoutdata.blockd = (unsigned short int) 0x0000;

    PK_X("si470x_set_frequency: fifo_put f(%d) rssi(%d)\n", rdsoutdata.freq, rdsoutdata.rssi);
    si4703_fifo_put(gdev, (u8 *) &rdsoutdata, sizeof(struct fmr_rds_type));
    wake_up_interruptible(&gdev->wq);
  }

  return 0;
}

static int si470x_tune(__u16 freq)
{
  int ret = 0;
  u16 channel= 0;
  long timeout=0;
  int retry = 0;

  PK_X("si470x_tune %d enter\n", freq);

  rds_bad = 0;
  rds_good = 0;
#ifdef RDSQ_LOGGING
  prev_rds_good=0;
  rdsq = 0;
#endif
  channel = freqToChan(freq);

  // Write channel number to register 0x03 and set tune bit
  preg_shadow->channel = channel | SI47_TUNE;
#ifdef LOGGING
  PK_X("si470x_tune: ");
  si470x_reg_dump(si470x_shadow, 0x00, 0x0F);
#endif

  WaitSTCInterrupt = 1;
  PK_X("si470x_tune: set TUNE bit\n");
  if (si470x_reg_write(2) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_write\n", __LINE__);
    return -1;
  }

  // Wait for stc bit to be set
  timeout = wait_event_interruptible_timeout(gdev->wq, (WaitSTCInterrupt == 0), TUNE_TIMEOUT * HZ);

  if (timeout == 0)
  {
    PK_ERR("ERROR STC interrupt missed\n");
#ifdef LOGGING
    //timeout, read back all registers
    if (si470x_reg_read(NR_OF_REGISTERS, si470x_shadow) != 0)
    {
      PK_ERR("ERROR @ %d, si470x_reg_read\n", __LINE__);
      return -1;
    }
    PK_X("si470x_tune: ");
    si470x_reg_dump(si470x_shadow, 0x00, 0x0F);
#endif
    ret = -1;
  }
  else
  {
    PK_X("si470x_tune: STC interrupt after %d s\n", (int)(TUNE_TIMEOUT - (timeout/HZ)));
  }
  
  // Write address 0x03 to clear tune bit
  preg_shadow->channel &= ~SI47_TUNE;
  PK_X("si470x_tune: clear TUNE bit, retry(%d)\n", retry);
  if (si470x_reg_write(2) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_write\n", __LINE__);
    return -1;
  }

  // Wait for stc bit to be cleared.  This step is very important. If it
  // is ignored and another seek or tune occurs too quickly, the tuner
  // will not set the STC bit properly.
  retry=0;
  if (si470x_reg_read(2, si470x_shadow) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_read\n", __LINE__);
    return -1;
  }
  while (((preg_shadow->statusrssi & SI47_STC) != 0) && (retry < 100))
  {
      if (si470x_reg_read(2, si470x_shadow) != 0)
      {
        PK_ERR("ERROR @ %d, si470x_reg_read\n", __LINE__);
        return -1;
      }
      msleep(5);
      retry++;
  }

  PK_X("si470x_tune return retry(%d) f(%d) rssi(%d) afcrl(%d)\n", 
         retry, 
         si470x_get_current_frequency(), 
         (u8)(preg_shadow->statusrssi & SI47_RSSI), 
         (u8)(preg_shadow->statusrssi & SI47_AFCRL));

  return ret;
}

static int si470x_seek(struct fmr_seek_param_type seek_param)
{
  struct fmr_rds_type rdsoutdata;

  PK_X("si470x_seek enter freq(%d) threshold(%d) up(%d) wrap(%d)\n",
         seek_param.freq, seek_param.threshold, seek_param.up, seek_param.wrap);

  si4703_fifo_purge(gdev);
  
  if (si470x_tune(seek_param.freq) != 0)
  {
    PK_ERR("ERROR tune failed\n");
    return -1;
  }
  else
  {
    //seek config:
    preg_shadow->sysconfig2 &= ~SI47_SEEKTH; // Clear the field first
    preg_shadow->sysconfig2 |= ((u16) (seek_param.threshold << 8) & 0xFF00); // Set the SEEKTH
    preg_shadow->sysconfig3 &= ~SI47_SKSNR;  // Clear the field first
    preg_shadow->sysconfig3 &= ~SI47_SKCNT;  // Clear the field first
    preg_shadow->sysconfig3 |= 0xA << 4;     // Set the SKSNR to 0xA
    preg_shadow->sysconfig3 |= 4;            // Set the SKCNT to 4

    if (si470x_seek_next(seek_param.up, (seek_param.wrap+1)%2) < 0)
    {
      PK_ERR("ERROR seek_next failed\n");
      return -1;
    }
    else
    {
      //preg_shadow (STATUSRSSI and READCHAN) up to date
      rdsoutdata.sf_bandlimit = (unsigned char) ((preg_shadow->statusrssi & SI47_SFBL) >> 13);
      rdsoutdata.rssi = (unsigned char) si470x_get_current_rssi();
      rdsoutdata.freq = (unsigned short int) si470x_get_current_frequency();
      rdsoutdata.blocka = (unsigned short int) 0x00FF;
      rdsoutdata.blockb = (unsigned short int) 0x0000;
      rdsoutdata.blockc = (unsigned short int) 0x0000;
      rdsoutdata.blockd = (unsigned short int) 0x0000;

      PK_X("si470x_seek: fifo_put f(%d) rssi(%d) bandlimit(%d)\n", rdsoutdata.freq, rdsoutdata.rssi, rdsoutdata.sf_bandlimit);
      si4703_fifo_put(gdev, (u8 *) &rdsoutdata, sizeof(struct fmr_rds_type));
      wake_up_interruptible(&gdev->wq);
    }
  }

  PK_X("si470x_seek return\n");
  return 0;
}

//-----------------------------------------------------------------------------
// Inputs:
//      seekup:  0 = seek down
//               1 = seek up
//      seekmode: 0 = wrap at band limits
//                1 = stop at band limits
// Outputs:
//      zero = seek found a station
//      nonzero = seek did not find a station
//-----------------------------------------------------------------------------
static int si470x_seek_next(u8 seekup, u8 seekmode)
{
  int ret=0;
  long timeout=0;
  int retry=0;

  PK_X("si470x_seek_next enter up(%d) mode(%d)\n", seekup, seekmode);

  // Set or clear seekmode bit in address 0x02
  if (seekmode)
  {
    preg_shadow->powercfg |= SI47_SKMODE;
  }
  else
  {
    preg_shadow->powercfg &= ~SI47_SKMODE;
  }

  // Set or clear seekup bit in address 0x02
  if(seekup)
  {
    preg_shadow->powercfg |= SI47_SEEKUP;
  }
  else
  {
    preg_shadow->powercfg &= ~SI47_SEEKUP;
  }

  // Set seek bit in address 0x02
  preg_shadow->powercfg |= SI47_SEEK;

#ifdef LOGGING
  PK_X("si470x_seek_next: ");
  si470x_reg_dump(si470x_shadow, 0x00, 0x0F);
#endif

  WaitSTCInterrupt = 1;
  PK_X("si470x_seek_next: set SEEK bit\n");
  // write seek config upto sysconfig3
  if (si470x_reg_write(5) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_write\n", __LINE__);
    return -1;
  }

  // Wait for stc bit to be set
  timeout = wait_event_interruptible_timeout(gdev->wq, (WaitSTCInterrupt == 0), SEEK_TIMEOUT * HZ);

  if (timeout == 0)
  {
    PK_ERR("ERROR STC interrupt missed\n");
#ifdef LOGGING
    //timeout, read back all registers
    if (si470x_reg_read(NR_OF_REGISTERS, si470x_shadow) != 0)
    {
      PK_ERR("ERROR @ %d, si470x_reg_read\n", __LINE__);
      return -1;
    }
    PK_X("si470x_seek_next: ");
    si470x_reg_dump(si470x_shadow, 0x00, 0x0F);
#endif    
    ret = -1;
  }
  else
  {
    PK_X("si470x_seek_next: STC interrupt after %d s\n", (int)(SEEK_TIMEOUT - (timeout/HZ)));
  }

    // Clear seek bit in address 0x02
  preg_shadow->powercfg &= ~SI47_SEEK;
  if (si470x_reg_write(1) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_write\n", __LINE__);
    return -1;
  }

  // Wait for stc bit to be cleared.  This step is very important. If it
  // is ignored and another seek or tune occurs too quickly, the tuner
  // will not set the STC bit properly.
  retry=0;
  if (si470x_reg_read(2, si470x_shadow) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_read\n", __LINE__);
    return -1;
  }
  while (((preg_shadow->statusrssi & SI47_STC) != 0)  && (retry < 100))
  {
    if (si470x_reg_read(2, si470x_shadow) != 0)
    {
      PK_ERR("ERROR @ %d, si470x_reg_read\n", __LINE__);
      return -1;
    }
    msleep(5);
    retry++;
  }

  // The tuner is now set to the newly found channel if one was available
  // as indicated by the seek-fail bit.
  PK_X("si470x_seek_next return retry(%d) f(%d) rssi(%d) afcrl(%d)\n", 
         retry, 
         si470x_get_current_frequency(), 
         (u8)(preg_shadow->statusrssi & SI47_RSSI),
         (u8)(preg_shadow->statusrssi & SI47_AFCRL));
  return ret;
}

static int si470x_set_volume(u16 volume)
{
  volume &= 0x000F;
  preg_shadow->sysconfig2 = (preg_shadow->sysconfig2 & ~SI47_VOLUME) | volume;

  if (si470x_reg_write(4))
  {
      return -1;
  }

  return 0;
}

//-----------------------------------------------------------------------------
// Get current frequency fromn shadow registers
//-----------------------------------------------------------------------------
static u16 si470x_get_current_frequency(void)
{
  return (chanToFreq(preg_shadow->readchan & SI47_READCHAN));
}

//-----------------------------------------------------------------------------
// Get current rssi from shadow registers
//-----------------------------------------------------------------------------
static u8 si470x_get_current_rssi(void)
{
  if ((preg_shadow->statusrssi & SI47_AFCRL) == 1)
  {
    PK_X("si470x_get_current_rssi: AFCRL=1 invalid channel f(%d)\n", si470x_get_current_frequency());
    preg_shadow->statusrssi &= ~SI47_RSSI;
  }

  return ((u8)(preg_shadow->statusrssi & SI47_RSSI));
}

//-----------------------------------------------------------------------------
// Converts from a channel number to a frequency value
//
// Inputs:
//  Channel number using current channel spacing
//
// Output:
//  Frequency in 10kHz steps
//-----------------------------------------------------------------------------
static u16 chanToFreq(u16 channel)
{
  u8 channelSpace, delta;
  u16 bottomOfBand;

  channelSpace = (preg_shadow->sysconfig2 & SI47_SPACE) >> 4;

  // calculate frequency

  if((preg_shadow->sysconfig2 & SI47_BAND) == SI47_BAND_USA_EUR) 
  {
    bottomOfBand = 8750;
  } 
  else 
  {
    bottomOfBand = 7600;
  }

  switch (channelSpace) {
  case 0x0:
    delta = 20;
    break;
  case 0x1:
    delta = 10;
    break;
  case 0x2:
    delta = 5;
    break;
  default:
    //ERROR
    delta = 0;
    break;
  }

  return (bottomOfBand + channel * delta);
}

static u16 freqToChan(u16 freq)
{
  u8 channelSpace, delta;
  u16 bottomOfBand;

  channelSpace = (preg_shadow->sysconfig2 & SI47_SPACE) >> 4;

  if((preg_shadow->sysconfig2 & SI47_BAND) == SI47_BAND_USA_EUR) 
  {
    bottomOfBand = 8750;
  } 
  else 
  {
    bottomOfBand = 7600;
  }

  switch (channelSpace) {
  case 0x0:
    delta = 20;
    break;
  case 0x1:
    delta = 10;
    break;
  case 0x2:
    delta = 5;
    break;
  default:
    //ERROR
    delta = 1;
    break;
  }

  return ((freq - bottomOfBand) / delta);
}

static int si470x_reg_write(u8 number_of_reg)
{
  u16 lreg[NR_OF_WRITE_REGS];
  int retval=0, ret=0, i=0;

  //Note: write starts at register 0x02h (POWERCFG), register 0x00, 0x01, 0x0A .. 0x0F are readonly
  //Note: we never write to reg 0x08 (TEST2) and 0x09 (BOOTCONFIG)
  if ((gdev->client != NULL) && (number_of_reg <= NR_OF_WRITE_REGS))
  {
    for (i = 0; i < number_of_reg; i++)
    {
      lreg[i] = cpu_to_be16(si470x_shadow[i+2]);
    }
  
    ret = i2c_master_send(gdev->client, (u8 *) lreg, (2 * number_of_reg));
    
    if (ret != (2 * number_of_reg))
    {
      retval = -EBUSY;
    }
  }

  return retval;
}

static int si470x_reg_read(u8 number_of_reg, u16 *plreg)
{
  u16 lreg[NR_OF_REGISTERS];
  int retval=0, ret=0, i=0;  

  // Note: read starts at register 0Ah (STATUSRSSI) and wraps back around till it reaches 09h
  if ((gdev->client != NULL) && (number_of_reg <= NR_OF_REGISTERS))
  {    
    ret = i2c_master_recv(gdev->client, (u8 *) lreg, (2 * number_of_reg));    
    
    if (ret == (2 * number_of_reg))
    {
      if ((0x0A + number_of_reg) < NR_OF_REGISTERS)
      {
        for (i = 0x0A; i < (0x0A + number_of_reg); i++)
        {
          plreg[i] = be16_to_cpu(lreg[i - 0x0A]);
        }
      }
      else
      {
        for (i = 0x0A; i <= 0x0F; i++)
        {
          plreg[i] = be16_to_cpu(lreg[i - 0x0A]);
        }
        for (i=0; i < (number_of_reg-6); i++)
        {
          plreg[i] = be16_to_cpu(lreg[i+6]);
        }
      }
    }
    else
    {
      retval = -EBUSY;
    }
  }

  return retval;
}


//-----------------------------------------------------------------------------
// Resets the part and initializes registers to the point of being ready for
// the first tune or seek.
//-----------------------------------------------------------------------------
static int si470x_initialize(void)
{
  int rc=0;

  PK_X("si470x_initialize enter\n");

  si470x_powerup();

  // Initialize shadow registers
  if (si470x_reg_read(NR_OF_REGISTERS, si470x_shadow) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_read\n", __LINE__);
    return -1;
  }

  /* set RDS verbose mode */
  preg_shadow->powercfg |= SI47_RDSM;

  /* set RDS enable, RDSI enable, STCI enable, GPIO2 == RDS/STC Interrupt */
  preg_shadow->sysconfig1 |= /*SI47_RDS | SI47_RDSIEN |*/ SI47_STCIEN | SI47_GPIO2_INT;

  /* set RDS high performance mode */
  preg_shadow->sysconfig3 |= SI47_RDSPRF;

  //write (POWERCFG...SYSCONFIG3)
  if (si470x_reg_write(5) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_write\n", __LINE__);
    return -1;
  }

#ifdef LOGGING
  PK_X("si470x_initialize: ");
  si470x_reg_dump(si470x_shadow, 0x00, 0x0F);
#endif

  PK_X("si470x_initialize: deviceid(0x%04x) chipid(0x%04x) test1(0x%04x)\n",
         preg_shadow->deviceid, preg_shadow->chipid, preg_shadow->test1);

  if (preg_shadow->deviceid != 0x1242)
  {
    PK_X("si470x_initialize: invalid device id 0x%x\n", preg_shadow->deviceid);
    s_device_id = 0xFFFF;
    rc = -1;
  }
  else
  {
    u16 dev = 0U;

    dev = (preg_shadow->chipid >> 6) & 0x000F;

    switch (dev)
    {
    case 1U:
      s_device_id = 4702U;
      break;
    case 8U:
      PK_X("si470x_initialize ERROR\n");
      rc = -1;
    case 9U:
      s_device_id = 4703U;
      break;
    default:
      PK_X("si470x_initialize ERROR dev(%d)\n", dev);
      rc = -1;
      break;
    }
  }

  PK_X("si470x_initialize return\n");
  return rc;
}
//-----------------------------------------------------------------------------
// Reset the Si470x
//-----------------------------------------------------------------------------
static int si470x_reset(void)
{
  PK_X("si470x_reset\n");

  IOP_Activate(tmcRX_reset_pin);
  msleep(100);
  IOP_Deactivate(tmcRX_reset_pin);
  msleep(100);

  return 0;
}
//-----------------------------------------------------------------------------
// Take the Si470x out of powerdown mode.
//-----------------------------------------------------------------------------
static int si470x_powerup(void)
{
  PK_X("si470x_powerup enter\n");

#if USE_CRYSTAL
   // Check if the device is already powered up. Only initialize shadow
    // registers if it is.  This isn't necessary if not using the crystal
    // or if it can be guaranteed that this function isn't called while
    // the device is already poweredup. This check prevents register 7
    // from being written with the wrong value. If the device is already
    // powered up, the XOSCEN bit should be or'd with 0x3C04 instead of 0x0100
  if(preg_shadow->powercfg & SI47_ENABLE)
    preg_shadow->test1 = SI47_XOSCEN | 0x3C04;
  else
    preg_shadow->test1 = SI47_XOSCEN | 0x0100;

#ifdef LOGGING
  PK_X("si470x_powerup write(6): ");
  si470x_reg_dump(si470x_shadow, 0x02, 0x07);
#endif

  if (si470x_reg_write(6) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_write\n", __LINE__);
    return -1;
  }
  msleep(CRYSTAL_TIME); // wait for crystal frequency to stabilize
#endif

  // Powerup the device
  preg_shadow->powercfg |= SI47_ENABLE;
  preg_shadow->powercfg &= ~SI47_DISABLE;
  if (si470x_reg_write(1) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_write\n", __LINE__);
    return -1;
  }
  msleep(POWERUP_TIME); // wait for si470x to powerup

  PK_X("si470x_powerup return\n");
  return 0;
}
//-----------------------------------------------------------------------------
// Configures the device for normal RDS operation
//-----------------------------------------------------------------------------
static int si470x_enable_rds(void)
{
  PK_X("si470x_enable_rds enter\n");
#ifdef RDSQ_LOGGING
  rds_timestamp_ms = jiffies_to_msecs(jiffies);
#endif

  preg_shadow->sysconfig1 |= SI47_RDSIEN; /* set RDSI enable */
  preg_shadow->sysconfig1 |= SI47_RDS;    /* set RDS enable  */
  if (si470x_reg_write(3) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_write\n", __LINE__);
    return -1;
  }

  PK_X("si470x_enable_rds return\n");

  return 0;
}

static int si470x_disable_rds(void)
{
  PK_X("si470x_disable_rds enter\n");

  preg_shadow->sysconfig1 &= ~SI47_RDSIEN;  /* unset RDSI enable */
  preg_shadow->sysconfig1 &= ~SI47_RDS;     /* unset RDS enable  */
  if (si470x_reg_write(3) != 0)
  {
    PK_ERR("ERROR @ %d, si470x_reg_write\n", __LINE__);
    return -1;
  }

  si4703_fifo_purge(gdev);

  PK_X("si470x_disable_rds return\n");

  return 0;
}

#ifdef LOGGING
static void si470x_reg_dump(u16 *lreg, int startreg, int endreg)
{
  int i;

  for (i=startreg; i<=endreg; i++)
  {
    PK_X("%04x ", lreg[i]);
  }
  PK_X("\n");
}
#endif

static int si470x_enable_i2c(void)
{
  if (tmcRX_dock_i2c_en_pin)
  {
    PK_X("si470x_enable_i2c\n");
    IOP_Activate(tmcRX_dock_i2c_en_pin);
    udelay(500);    
  }
  return 0;
}

static int si470x_disable_i2c()
{
  if (tmcRX_dock_i2c_en_pin)
  {
    PK_X("si470x_disable_i2c\n");
    IOP_Deactivate(tmcRX_dock_i2c_en_pin);
    udelay(500);    
  }
  return 0;
}

static int si4703_core_probe(struct device *dev)
{
  struct platform_device* pdev = to_platform_device(dev);

  PK_X("si4703_core_probe\n");

  if ( pdev != NULL ) 
  {
    tmcRX_irq = platform_get_irq(pdev, 0);
    tmcRX_dock_i2c_en_pin = ((struct fm_receiver_info*) (pdev->dev.platform_data))->fm_dock_i2c_en_pin ;
    tmcRX_reset_pin = ((struct fm_receiver_info*) (pdev->dev.platform_data))->fm_reset_pin ;
    tmcRX_sen_pin   = ((struct fm_receiver_info*) (pdev->dev.platform_data))->fm_sen_pin ;
    tmcRX_devno   = ((struct fm_receiver_info*) (pdev->dev.platform_data))->device_nr ;

    return 0;
  }
  return -1;
}

static int si470x_disable()
{
  int retry=0;

  if (preg_shadow->powercfg & SI47_ENABLE) 
  {
    // First, disable RDS      
    PK_X("si470x_disable: disable RDS\n");
    preg_shadow->sysconfig1 &= ~SI47_RDSIEN;  /* unset RDSI enable */
    preg_shadow->sysconfig1 &= ~SI47_RDS;     /* unset RDS enable  */
    if (si470x_reg_write(3) != 0)
    {
      PK_ERR("ERROR @ %d, si470x_reg_write\n", __LINE__);
    }

    // Set DISABLE=1
    preg_shadow->powercfg |= SI47_DISABLE;
    PK_X("si470x_disable: Set DISABLE=1 powercfg(0x%x)\n", preg_shadow->powercfg);
    si470x_reg_write(1);
  }

  // Wait for indication that powerdown was successful: ENABLE=0
  retry=0;
  if (si470x_reg_read(9, si470x_shadow) != 0) // Note: read starts from register 0xA
  {
    PK_ERR("ERROR @ %d, si470x_reg_read\n", __LINE__);
  }
  while (((preg_shadow->powercfg & SI47_ENABLE) != 0)  && (retry < 10))
  {
    if (si470x_reg_read(9, si470x_shadow) != 0)
    {
      PK_ERR("ERROR @ %d, si470x_reg_read\n", __LINE__);
    }      
    msleep(10);
    retry++;      
  }
  PK_X("si470x_disable: Low Power, Bus Accessible, powercfg(0x%x) retry(%d)\n", preg_shadow->powercfg, retry);
  
  return 0;
}

static int si4703_core_suspend(struct device *dev, pm_message_t state, u32 level)
{  
  PK_X("si4703_core_suspend enter\n");

  if (si4703_suspended == 0)
  { 
    si4703_suspended = 1;          

    if (atomic_read(&si4703_available) == 0)
    {
      PK_X("si4703_core_suspend si470x_disable()\n");
      si470x_disable();      
      si470x_disable_i2c();    
      
      PK_X("si4703_core_suspend: Activate(tmcRX_reset_pin)\n");
      IOP_Activate(tmcRX_reset_pin);
    }
  }

  PK_X("si4703_core_suspend return\n"); 
  return 0;
}

static int si4703_core_resume(struct device *dev, u32 level)
{
  int rc = 0;
  int retries = 0;

  PK_X("si4703_core_resume enter\n");

  if (si4703_suspended == 1)
  {
    if (atomic_read(&si4703_available) == 0)
    {
      if (tmcRX_sen_pin)
      {
        PK_X("si4703_core_resume : IOP_Deactivate tmcRX_sen_pin(0x%x)\n", tmcRX_sen_pin);
        IOP_Deactivate(tmcRX_sen_pin);
        udelay(500);
      }
  
      si470x_disable_i2c();
      si470x_reset();
      si470x_enable_i2c();
  
      do {
        rc = si470x_initialize();
        PK_X("si4703_core_resume: retry(%d) si470x_initialize return %d\n", retries, rc);
        retries++;
        msleep(100);
      } while ((rc < 0) && (retries < 3));
    }

    si4703_suspended = 0;
  }
  
  PK_X("si4703_core_resume return\n");
  return rc;
}


MODULE_AUTHOR("RDS-TMC dev team ehv");
MODULE_DESCRIPTION("Driver for I2C connected Silicon Labs 4703 FM/RDS/TMC Receiver");
MODULE_LICENSE("GPL");  /* needed for access to some linux api functions */

module_init(si4703_init);
module_exit(si4703_exit);

