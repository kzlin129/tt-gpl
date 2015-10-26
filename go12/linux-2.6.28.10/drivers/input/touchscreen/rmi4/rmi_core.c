/**
 * \file
 * Synaptics Register Mapped Interface (RMI4) Data Layer Driver.
 * Copyright (C) 2008 - 2009, Synaptics Incorporated
 *
 *
 * This protocol is layered as follows.
 * <pre>
 *  +----------------------------------------+
 *  |                                        |
 *  |              Application               |
 *  |                                        |
 *  +----------------------------------------+
 *  |                                        |
 *  |               RMI Driver               | Data Layer (THIS DRIVER)
 *  |              (this file)               |
 *  +-----+-----+-------+----------+---------+
 *  | I2C | SPI | SMBus |        etc.        | Physical Layer
 *  +-----+-----+-------+----------+---------+
 *</pre>
 *  Each of the physical layer drivers is contained in a file called
 *  rmi_phys_xxx.c.  Someone compiling the kernel enables CONFIG_RMI and then
 *  one or more CONFIG_RMI_xxx options in the .config file.  For example, when
 *  CONFIG_RMI_I2C=m is enabled, a rmi.ko and a rmi_phys_i2c.ko will be
 *  compiled.  rmi_phys_i2c.ko will depend on rmi.ko, so when rmi_phys_i2c.ko
 *  is loaded, rmi.ko will automatically be loaded.  Each of the physical
 *  layer drivers is a platform_driver that may handle suspend/resume, etc.,
 *  so this driver does not do so.
 *
 *  The register paradigm of RMI is a "pull" rather than "push" data flow.
 *  As such, it is the application driver that needs to implement either
 *  polling or interrupt driven, and the physical driver merely handles
 *  the register accesses.  For interrupt driven, the application registers
 *  an "attention" function that may be called in interrupt context by the
 *  physical driver if an attention interrupt is available.  The physical
 *  driver notifies the application through the polling_required variable,
 *  and the application driver must do one or the other based on this variable.
 *
 *  At this point in time, there can only be one application driver per
 *  physical driver.
 *
 */

static const char drvname[] = "rmi";

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#include "rmi.h"
#include "rmi_core.h"
    
/* we need these to control and query interrupts */
unsigned short fn01QueryBaseAddr = 0;     /* RMI4 device control */
EXPORT_SYMBOL(fn01QueryBaseAddr);
unsigned short fn01ControlBaseAddr = 0;
EXPORT_SYMBOL(fn01ControlBaseAddr);

unsigned short fn11DataBaseAddr = 0;      /* 2D capacitive sensor */
EXPORT_SYMBOL(fn11DataBaseAddr);
unsigned short fn11ControlBaseAddr = 0;
EXPORT_SYMBOL(fn11ControlBaseAddr);
unsigned short fn11FingersSupported = 0;
EXPORT_SYMBOL(fn11FingersSupported);
unsigned int   fn11DataRegBlockSize = 0;
EXPORT_SYMBOL(fn11DataRegBlockSize);
unsigned char  fn11InterruptMask = 0;
EXPORT_SYMBOL(fn11InterruptMask);
unsigned char  fn11InterruptRegister = 0;
EXPORT_SYMBOL(fn11InterruptRegister);
bool fn11HasRel = false;
EXPORT_SYMBOL(fn11HasRel);
bool fn11HasGestures = false;
EXPORT_SYMBOL(fn11HasGestures);
bool fn11HasPinch = false;
EXPORT_SYMBOL(fn11HasPinch);
bool fn11HasFlick = false;
EXPORT_SYMBOL(fn11HasFlick);
bool fn11HasTap = false;
EXPORT_SYMBOL(fn11HasTap);
bool fn11HasTapAndHold = false;
EXPORT_SYMBOL(fn11HasTapAndHold);
bool fn11HasDoubleTap = false;
EXPORT_SYMBOL(fn11HasDoubleTap);
bool fn11HasEarlyTap = false;
EXPORT_SYMBOL(fn11HasEarlyTap);
bool fn11HasPress = false;
EXPORT_SYMBOL(fn11HasPress);
bool fn11HasPalmDetect = false;
EXPORT_SYMBOL(fn11HasPalmDetect);

unsigned short fn19DataBaseAddr = 0;      /* capacitive buttons */
EXPORT_SYMBOL(fn19DataBaseAddr);
unsigned short fn19ButtonsSupported = 0;
EXPORT_SYMBOL(fn19ButtonsSupported);
unsigned char  fn19InterruptMask = 0;
EXPORT_SYMBOL(fn19InterruptMask);
unsigned char  fn19InterruptRegister = 0;
EXPORT_SYMBOL(fn19InterruptRegister);

unsigned int interruptRegisterCount = 0;
EXPORT_SYMBOL(interruptRegisterCount);

static LIST_HEAD(phys_drivers);
static DEFINE_MUTEX(phys_drivers_mutex);
static LIST_HEAD(app_drivers);
static DEFINE_MUTEX(app_drivers_mutex);

struct rmi_device_settings {
  signed char sensitivity;
};

static struct rmi_device_settings rmi_settings = {
  .sensitivity = 0,
};

#define RMI_SET_SENSITIVITY 1
#define RMI_GET_SENSITIVITY 2

static int
rmi_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
  printk("RMI IOCTL\n");

  switch (cmd) {
    case RMI_SET_SENSITIVITY:
      if (copy_from_user(&(rmi_settings.sensitivity), (signed char *)arg, sizeof(signed char)))
        return -EFAULT;
      else
      {
        struct rmi_application *app;
        list_for_each_entry(app, &app_drivers, apps) {			
          if (app->rpd) {
            struct rmi_function_info *rfi;
            list_for_each_entry(rfi, &app->rpd->rmi.functions, link) {
              if (rfi->functionNum == 0x11) {
                app->rpd->write(app->rpd, rfi->funcDescriptor.controlBaseAddr+13, rmi_settings.sensitivity);
              }
            }
          }
        }
      }
      break;
	
    case RMI_GET_SENSITIVITY:
      {
        /* get current setting and return that */
        struct rmi_application *app;
        list_for_each_entry(app, &app_drivers, apps) {			
          if (app->rpd) {
            struct rmi_function_info *rfi;
            list_for_each_entry(rfi, &app->rpd->rmi.functions, link) {
              if (rfi->functionNum == 0x11) {
                app->rpd->read(app->rpd, rfi->funcDescriptor.controlBaseAddr+13, &rmi_settings.sensitivity);
              }
            }
          }
        }

        if (copy_to_user((signed char *)arg, &(rmi_settings.sensitivity), sizeof(signed char)))
          return -EFAULT;
      }
      break;
		
    default:
      return -EINVAL;
  }

  return 0;
}

static struct file_operations rmi_fops = {
  .owner = THIS_MODULE,
  .ioctl = rmi_ioctl,	
};

static struct miscdevice rmi_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = "rmi",
  .fops = &rmi_fops,
};

#if RMI_ALLOC_STATS
  int appallocsrmi;
  EXPORT_SYMBOL(appallocsrmi);
#endif

int rmi_read(struct rmi_application *app, unsigned short address, char *dest)
{
  struct rmi_phys_driver *rpd = app->rpd;
  return rpd->read(rpd, address, dest);
}
EXPORT_SYMBOL(rmi_read);

int rmi_write(struct rmi_application *app, unsigned short address, unsigned char data)
{
  struct rmi_phys_driver *rpd = app->rpd;
  return rpd->write(rpd, address, data);
}
EXPORT_SYMBOL(rmi_write);

int rmi_read_multiple(struct rmi_application *app, unsigned short address, void *dest, int length)
{
  struct rmi_phys_driver *rpd = app->rpd;
  if(!app->rpd) return -ENODEV;
  return rpd->read_multiple(rpd, address, dest, length);
}
EXPORT_SYMBOL(rmi_read_multiple);

int rmi_write_multiple(struct rmi_application *app, unsigned short address, void *data, int length)
{
  struct rmi_phys_driver *rpd = app->rpd;
  return rpd->write_multiple(rpd, address, data, length);
}
EXPORT_SYMBOL(rmi_write_multiple);

static void match(struct rmi_application *app, struct rmi_phys_driver *rpd)
{
  app->polling_required = rpd->polling_required;
  if(app->probe(app, &rpd->rmi)) {
    /* Found a match, bind them together. */
    /** The try_module_get() makes sure that the physical
     * driver cannot be unloaded while a app driver is
     * using it.
     */
    if(try_module_get(rpd->module)) {
      app->rpd = rpd;
      rpd->app = app;
      printk(KERN_INFO "%s: %s bound to %s\n", drvname, app->name, rpd->name);
      rpd->attention = app->attention;
      app->config(app);
    }
  } else {
    app->polling_required = 0;
  }
}

int rmi_polling_required(struct rmi_application *app)
{
  return app->polling_required;
}
EXPORT_SYMBOL(rmi_polling_required);

int rmi_get_attn(struct rmi_application *app)
{
  return app->rpd->get_attention(app->rpd);
}
EXPORT_SYMBOL(rmi_get_attn);

int rmi_register_phys_driver(struct rmi_phys_driver *rpd)
{
  struct rmi_application *app;
  int retval;

  if(!rpd->name) {
    printk(KERN_ERR "%s: Physical driver must specify a name\n", drvname);
    return -EINVAL;
  }
  if(!rpd->write) {
    printk(KERN_ERR "%s: Physical driver %s must specify a writer.\n",  drvname, rpd->name);
    return -EINVAL;
  }
  if(!rpd->read) {
    printk(KERN_ERR "%s: Physical driver %s must specify a reader.\n", drvname, rpd->name);
    return -EINVAL;
  }
  if(!rpd->write_multiple) {
    printk(KERN_ERR "%s: Physical driver %s must specify a multiple writer.\n",  drvname, rpd->name);
    return -EINVAL;
  }
  if(!rpd->read_multiple) {
    printk(KERN_ERR "%s: Physical driver %s must specify a multiple reader.\n", drvname, rpd->name);
    return -EINVAL;
  }
  if(!rpd->module) {
    printk(KERN_ERR "%s: Physical driver %s must specify a module.\n",  drvname, rpd->name);
    return -EINVAL;
  }

  pr_debug("%s: Registering phys driver %s\n", drvname, rpd->name);

  rpd->attention = 0;

  /* Get some information from the device */
  {
    int i;
    unsigned char value;
    unsigned char std_queries[21];
    int funcCnt = 0;
    unsigned char fn11Queries[9];
    unsigned char fn19Queries[2];
    unsigned char numDataPoints = 0;
    unsigned char fn11InterruptOffset = 0;
    unsigned char fn19InterruptOffset = 0;
    unsigned char interruptCount = 0;

    struct rmi_function_descriptor rmi_fd;

    /* init the physical drivers RMI module info list of functions */
    INIT_LIST_HEAD(&rpd->rmi.functions);

    printk(KERN_INFO "%s: Functions:\n", drvname);

    /* Read the Page Descriptor Table to determine what functions are present */
    for(i = 0x00E9; i > 0x000a; i -= 6 )
    {
      if (!(retval = rpd->read_multiple(rpd, i, &rmi_fd, sizeof(rmi_fd))))
      {
        if(rmi_fd.functionNum)
        {
          printk(KERN_INFO "%s: ", drvname);

          switch(rmi_fd.functionNum & 0xff)
          {
            case 0x01:
              printk("   RMI Device Control\n");
              /* Save Fn $01 query and control base addresses since
                 we'll need them later to get/set properties and check interrupts. */
              fn01QueryBaseAddr = rmi_fd.queryBaseAddr;
              fn01ControlBaseAddr = rmi_fd.controlBaseAddr;
              break;

            case 0x11:
              printk("   2-D Sensor\n");
              /* need to get number of fingers supported -
                 to be used when getting data since the number of registers to
                 read depends on the number of fingers supported */
              rpd->read_multiple(rpd, rmi_fd.queryBaseAddr, &fn11Queries, sizeof(fn11Queries));
              numDataPoints = (fn11Queries[1] & 0x7) + 1; /* add one since zero based */
              fn11FingersSupported = numDataPoints;
              /* Need to get interrupt info to be used later when handling interrupts. */
              fn11InterruptOffset = interruptCount % 8;
              fn11InterruptRegister = interruptCount/8;
              for (i = fn11InterruptOffset; i < ((rmi_fd.interruptSrcCnt & 0x7) + fn11InterruptOffset); i++)
              {
                fn11InterruptMask |= 1 << i;
              }
              /* Store data address - used in ISR to read finger data. */
              fn11DataBaseAddr = rmi_fd.dataBaseAddr;

              /* need to determine the size of data to read - this depends on conditions such as
                 whether Relative data is reported and if Gesture data is reported. */
	      {
                unsigned char fn11AbsDataSize = fn11Queries[5] & 0x03;
                fn11HasRel = fn11Queries[1] & 0x08;
                fn11HasGestures = fn11Queries[1] & 0x20;
                /* if we have gesture support determine which gestures we support */
                if (fn11HasGestures) {
                  fn11HasPinch = fn11Queries[6] & 0x40;
                  fn11HasFlick = fn11Queries[6] & 0x10;
                  fn11HasTap = fn11Queries[6] & 0x01;
                  fn11HasTapAndHold = fn11Queries[6] & 0x02;
                  fn11HasDoubleTap = fn11Queries[6] & 0x04;
                  fn11HasEarlyTap = fn11Queries[6] & 0x08;
                  fn11HasPress = fn11Queries[6] & 0x20;
                  fn11HasPalmDetect = fn11Queries[7] & 0x01;
                }

                fn11DataRegBlockSize = 2 /* One register each for X and Y coarse absolute position */
                + 3 * (fn11AbsDataSize == 0) /* if AbsDataSize is 0, one for LSB for X & Y, one for W, one for Z */
                + 2 * fn11HasRel; /* if has Relative reporting, one register each for Delta X, Delta Y */
	      }

              break;

            case 0x08:
              printk("   BIST\n");
              break;

            case 0x19: 
              printk("   Buttons\n");
              /* Need to get the number of buttons supported  - used when reading data registers
                 since the number of registers to read depends on the number of buttons - 
                 Number of Data Registers = trunc((ButtonCount + 7) / 8)*/
              rpd->read_multiple(rpd, rmi_fd.queryBaseAddr, &fn19Queries, sizeof(fn19Queries));
              numDataPoints = fn19Queries[1] & 0x1f;
              fn19ButtonsSupported = numDataPoints;
              /* Store interrupt info - used later when handling interrupts. */
              fn19InterruptOffset = interruptCount % 8;
              fn19InterruptRegister = interruptCount/8;
              for (i = fn19InterruptOffset; i < ((rmi_fd.interruptSrcCnt & 0x7) + fn19InterruptOffset); i++)
              {
                fn19InterruptMask |= 1 << i;
              }
              /* Store data address - used in ISR to read button status. */
              fn19DataBaseAddr = rmi_fd.dataBaseAddr;
              break;

            case 0x30:
              printk("   LED/GPIOs\n");
              break;

            case 0x32:
              printk("   Timer\n");
              break;

            case 0x34:
              printk("   Flash Update\n");
              break;

            default:
              printk("   Unknown Function %02x\n", i&0xff);
              break;
          }

          /* bump interrupt count for next iteration */
          interruptCount += (rmi_fd.interruptSrcCnt & 0x7);

          /* We only want to add functions 0x11 and 0x019 as these correspond
             to 2D sensor and buttons (note: you can add other supported sources for your device)*/
          if (((rmi_fd.functionNum & 0xff) == 0x11) ||
            ((rmi_fd.functionNum & 0xff) == 0x19))
          {  
            struct rmi_function_info *rfi;

            rfi = kmalloc(sizeof(*rfi), GFP_KERNEL);

            if(!rfi) {
              printk(KERN_ERR "%s: could not allocate memory for function %x\n",
			          drvname, rmi_fd.functionNum & 0xff);
			  continue;
            }

            rfi->functionCnt  = funcCnt++;
            rfi->functionNum  = rmi_fd.functionNum & 0xff; /* function number - used later in ISR to access data */
            rfi->numSources = rmi_fd.interruptSrcCnt;
            rfi->numDataPoints = numDataPoints; /* number of fingers, number of buttons, etc. */
            if ((rmi_fd.functionNum & 0xff) == 0x11) 
            {
              rfi->interruptRegister = fn11InterruptRegister;
              rfi->interruptMask = fn11InterruptMask;
            } else if ((rmi_fd.functionNum & 0xff) == 0x19) {
              rfi->interruptRegister = fn19InterruptRegister;
              rfi->interruptMask = fn19InterruptMask;
            }

            /* Store base addresses for the function */
            rfi->funcDescriptor.queryBaseAddr = rmi_fd.queryBaseAddr;
            rfi->funcDescriptor.commandBaseAddr = rmi_fd.commandBaseAddr;
            rfi->funcDescriptor.controlBaseAddr = rmi_fd.controlBaseAddr;
            rfi->funcDescriptor.dataBaseAddr = rmi_fd.dataBaseAddr;
            rfi->funcDescriptor.interruptSrcCnt = rmi_fd.interruptSrcCnt;
            rfi->funcDescriptor.functionNum = rmi_fd.functionNum;
			   
            printk(KERN_INFO "%s:  Sources: %d \n", drvname, rfi->numSources);

            /* link this function info to the RMI module infos list of functions */
            list_add_tail(&rfi->link, &rpd->rmi.functions);
          }
        } else {
          /* A zero in the function number signals the end of the PDT */ 
          break;
        }
      } else {
        /* failed to read next PDT entry - end PDT scan - this may result in an incomplete
           set of recognized functions - should probably return an error but the driver
           may still be viable for diagnostics and debugging so let's let it continue. */
        break;
      }
    }

    interruptRegisterCount = (interruptCount + 7) / 8;

    /* Function $01 will be used to query the product properties, and product ID
     * so we had to read the PDT above first to get the Fn $01 query address and
     * control address prior to filling in the product info. */

    /* Disable Interrupts. It is up to the Application Driver to
     * turn them on when it's ready for them. */
    if(fn11InterruptRegister && !(retval = rpd->write(rpd, fn01ControlBaseAddr + 1 + fn11InterruptRegister, 0))) {
      printk("Function $11 Interrupt Disable Fail: %d\n", retval);
    }
    if(fn19InterruptRegister && !(retval = rpd->write(rpd, fn01ControlBaseAddr + 1 + fn19InterruptRegister, 0))) {
      printk("Function $19 Interrupt Disable Fail: %d\n", retval);
    }

    /* Load up the standard queries and get the RMI module info */
    if((retval = rpd->read_multiple(rpd, fn01QueryBaseAddr, std_queries, sizeof(std_queries)))) {
      printk(KERN_ERR "%s: Fail reading queries\n", drvname);
      retval = -EIO;
      goto exit_fail;
    }

    rpd->rmi.rmi_maj_ver  = 4;
    rpd->rmi.rmi_min_ver  = 0;

    /* get manufacturer id, properties, product info, date code, tester id, serial num and product id (name) */
    rpd->rmi.mfgid        = std_queries[0];
    
    rpd->rmi.properties   = std_queries[1];
	
    rpd->rmi.prod_info[0] = std_queries[2];
    rpd->rmi.prod_info[1] = std_queries[3];

    rpd->rmi.date_code[0] = std_queries[4] & 0x1f; /* year - 2001-2032 */
    rpd->rmi.date_code[1] = std_queries[5] & 0x0f; /* month - 1-12 */
    rpd->rmi.date_code[2] = std_queries[6] & 0x1f; /* day - 1-31 */

    rpd->rmi.tester_id    = ((std_queries[7] & 0x7f) << 8) | (std_queries[8] & 0x7f);

    rpd->rmi.serial_num   = ((std_queries[9] & 0x7f) << 8) | (std_queries[10] & 0x7f);

    for (i = 0; i < 10; i++)
    {
      rpd->rmi.prod_id[i] = std_queries[11+i];
    }
		
    rpd->rmi.prod_id[10] = 0;

    printk(KERN_INFO "%s: RMI Protocol: %d.%d\n",
      drvname, rpd->rmi.rmi_maj_ver, rpd->rmi.rmi_min_ver);
    printk(KERN_INFO "%s: Manufacturer: %d", drvname,
      rpd->rmi.mfgid);
	
    if(rpd->rmi.mfgid == 1) {
      printk(" (Synaptics)");
    }
    printk("\n");

    printk(KERN_INFO "%s: Properties: %x \n", drvname, rpd->rmi.properties);
   
    printk(KERN_INFO "%s: Product Info: %x %x \n", drvname, rpd->rmi.prod_info[0], rpd->rmi.prod_info[1]);

    printk(KERN_INFO "%s: Date Code: Year : %d Month: %d Day: %d\n", drvname, rpd->rmi.date_code[0],
      rpd->rmi.date_code[1], rpd->rmi.date_code[2]);

    printk(KERN_INFO "%s: Tester ID: %d \n", drvname, rpd->rmi.tester_id);

    printk(KERN_INFO "%s: Serial Number: %x \n", drvname, rpd->rmi.serial_num);

    printk(KERN_INFO "%s: Product ID: %s\n", drvname, rpd->rmi.prod_id);
  }

  mutex_lock(&phys_drivers_mutex);
  list_add_tail(&rpd->drivers, &phys_drivers);
  mutex_unlock(&phys_drivers_mutex);

  /* Do a probe for any applications that are registered */
  list_for_each_entry(app, &app_drivers, apps) {
    /* Only check apps that are not already bound */
    if(!app->rpd) {
      match(app, rpd);
    }
  }

  pr_debug("Registered phys driver %s\n", rpd->name);	
  return 0;

exit_fail:
  return retval;
}
EXPORT_SYMBOL(rmi_register_phys_driver);

int rmi_unregister_phys_driver(struct rmi_phys_driver *rpd)
{
  if(rpd->app) {
    printk(KERN_WARNING "%s: WARNING: unregister of %s while %s still attached\n",
      drvname, rpd->name, rpd->app->name);
  }
	
  pr_debug("Unregistering phys driver %s\n", rpd->name);
  mutex_lock(&phys_drivers_mutex);
  list_del(&rpd->drivers);
  mutex_unlock(&phys_drivers_mutex);
	
  return 0;
}
EXPORT_SYMBOL(rmi_unregister_phys_driver);

struct rmi_application * rmi_register_application(const char *name,
  void (*attention)(struct rmi_phys_driver *pd, int instance),
  int (*probe)(struct rmi_application *app,
  const struct rmi_module_info *rmi),
  void (*config)(struct rmi_application *app))
{
  struct rmi_application *app;
  struct rmi_phys_driver *rpd;

  if(!name) {
    printk(KERN_ERR "%s: Application driver must specify a name\n", drvname);
    return 0;
  }
  
  if(!attention) {
    printk(KERN_ERR "%s: Application driver %s must specify attention notifier.\n",
      drvname, name);
    return 0;
  }
	
  if(!probe) {
    printk(KERN_ERR "%s: Application driver %s must specify a probe function.\n",
      drvname, name);
    return 0;
  }

  if(!config) {
    printk(KERN_ERR "%s: Application driver %s must specify a config function.\n",
      drvname, name);
    return 0;
  }

  pr_debug("Registering app driver %s\n", name);

  app = kmalloc(sizeof(*app), GFP_KERNEL);
  if(!app) {
    printk(KERN_ERR "%s: Out of memory\n", drvname);
    return 0;
  }
  INC_ALLOC_STAT(app);

  app->name      = name;
  app->attention = attention;
  app->probe     = probe;
  app->config    = config;
  app->rpd       = 0;

  mutex_lock(&app_drivers_mutex);
  list_add_tail(&app->apps, &app_drivers);
  mutex_unlock(&app_drivers_mutex);

  /* Probe for any matches with physical drivers */
  list_for_each_entry(rpd, &phys_drivers, drivers) {
    if(!rpd->app) {
      match(app, rpd);
    }
  }

  pr_debug("Registered app driver %s (%p)\n", name, app);
  
  return app;
}
EXPORT_SYMBOL(rmi_register_application);

void rmi_unregister_application(struct rmi_application *app)
{
  struct rmi_application *tmp;
  int found = 0;

  if(!app) {
    return;
  }

  pr_debug("Unregistering app driver %s (%p)\n", app->name, app);

  list_for_each_entry(tmp, &app_drivers, apps) {
    if(tmp == app) {
      found = 1;
      break;
    }
  }
	
  if(!found) {
    printk(KERN_ERR "%s: Removing rmi application %s: not found\n",
      drvname, app->name);
    return;
  }
	
  if(app->rpd) {
    /* Release the phys driver so it can be unloaded. */
    module_put(app->rpd->module);
    app->rpd->app = 0;
  }
	
  list_del(&app->apps);
  kfree(app);
  DEC_ALLOC_STAT(app);

  pr_debug("Unregistered app driver %p\n", app);
}
EXPORT_SYMBOL(rmi_unregister_application);

static int __init mod_init(void)
{
  printk(KERN_INFO "Register Mapped Interface Data Layer Driver\n");
  return misc_register(&rmi_misc);
}

static void __exit mod_exit(void)
{
  struct rmi_application *app, *apptmp;

  /* These lists should be empty, but just in case . . . */
  mutex_lock(&app_drivers_mutex);
  list_for_each_entry_safe(app, apptmp, &app_drivers, apps) {
    list_del(&app->apps);
    kfree(app);
    DEC_ALLOC_STAT(app);
  }
  mutex_unlock(&app_drivers_mutex);

  misc_deregister(&rmi_misc);

  CHECK_ALLOC_STAT(app);
}

/** Specifies to the kernel that the mod_init() function should be called when
 * the module is loaded.
 * \see mod_init()
 */
module_init(mod_init);
/** Specifies to the kernel that the mod_exit() function should be called when
 * the module is unloaded.
 * \see mod_exit()
 */
module_exit(mod_exit);

/** Standard driver module information - the author of the module.
 */
MODULE_AUTHOR("Synaptics, Inc.");
/** Standard driver module information - a summary description of this module.
 */
MODULE_DESCRIPTION("RMI4 Core Driver");
/** Standard driver module information - the license under which this module
 * is included in the kernel.
 */
MODULE_LICENSE("");

/* vim600: set noexpandtab sw=8 ts=8 : */
