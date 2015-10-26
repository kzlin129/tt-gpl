/*
 * pmu_bus.c - pmu bus 
 *
 * Derived from platform.c
 *
 * This file is released under the GPLv2
 *
 */

#include <linux/pmu_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/bootmem.h>
#include <linux/err.h>
#include <linux/slab.h>

#include "base.h"

#define PFX "pmu_bus: "

typedef enum {e_read, e_write} e_transfer_t;

#define to_pmu_driver(drv)	(container_of((drv), struct pmu_driver, \
				 driver))

/* core_lock protects core_adapter and guarantees that device detection, 
   deletion of detected devices, attach_adapter and detach_adapter 
   calls are serialized */
static DEFINE_MUTEX(core_lock);
static struct pmu_adapter *core_adapter = NULL;

struct bus_type pmu_bus_type;

struct device pmu_bus = {
	.init_name	= "pmu",
	.bus_id		= "pmu",
};
EXPORT_SYMBOL(pmu_bus);

static void pmu_client_dev_release(struct device *dev)
{
	kfree(to_pmu_client(dev));
}


/**
 * pmu_register_device - register a pmu device
 * @adap: the adapter managing the device
 * @info: describes one pmu device
 */
struct pmu_client *
pmu_register_device(struct pmu_board_info const *info)
{
	struct pmu_client	*client;
	int                  status;

	if (!core_adapter) {
		printk(KERN_INFO PFX "cannot register board [%s]\n", info->name);
		return NULL;
	}

	client = kzalloc(sizeof *client, GFP_KERNEL);
	if (!client)
		return NULL;

	client->adapter = core_adapter;

	client->dev.platform_data = info->platform_data;

	strlcpy(client->name, info->name, sizeof(client->name));

	client->dev.parent = &client->adapter->dev;
	client->dev.bus = &pmu_bus_type;
	client->dev.release = pmu_client_dev_release;

	snprintf(client->dev.bus_id,
			 BUS_ID_SIZE,
			 "%s-%s",
			 client->adapter->name,
			 client->name);

	dev_set_name(&client->dev, "%s-%s",client->adapter->name,
					client->name);
	status = device_register(&client->dev);
	if(status) {
		printk(KERN_ERR PFX "Failed to register pmu client %s"
				" [status = %d]\n",	client->name, status);
		kfree(client);
		return NULL;
	}

	printk(KERN_INFO PFX "client [%s] registered with pmu bus %s\n",
		   client->name, dev_name(&client->dev));

	return client;
}
EXPORT_SYMBOL(pmu_register_device);

/**
 * pmu_unregister_device - reverse effect of pmu_register_device()
 * @client: value returned from pmu_ergister_device()
 */
void pmu_unregister_device(struct pmu_client *client)
{
	device_unregister(&client->dev);
}
EXPORT_SYMBOL(pmu_unregister_device);

/**
 * pmu_register_driver - register pmu driver
 * @driver: the driver being registered
 */
int pmu_register_driver(struct pmu_driver *driver)
{
	int res;

	/* Can't register until after driver model init */
	if (unlikely(WARN_ON(!pmu_bus_type.p)))
		return -EAGAIN;

	/* add the driver to the list of pmu drivers in the driver core */
//	driver->driver.owner = owner;
	driver->driver.bus = &pmu_bus_type;

	/* When registration returns, the driver core
	 * will have called probe() for all matching-but-unbound devices.
	 */
	res = driver_register(&driver->driver);
	if (res)
		return res;

	printk(KERN_INFO PFX "pmu-core: driver [%s] registered\n", driver->driver.name);

	INIT_LIST_HEAD(&driver->clients);

	return 0;
}
EXPORT_SYMBOL(pmu_register_driver);


/**
 * pmu_unregister_driver - unregister pmu driver
 * @driver: the driver being unregistered
 */
void pmu_unregister_driver(struct pmu_driver *driver)
{
	driver_unregister(&driver->driver);
	printk(KERN_INFO PFX "pmu-core: driver [%s] unregistered\n", driver->driver.name);
}
EXPORT_SYMBOL(pmu_unregister_driver);


/* modalias support enables more hands-off userspace setup:
 * (a) environment variable lets new-style hotplug events work once system is
 *     fully running:  "modprobe $MODALIAS"
 * (b) sysfs attribute lets new-style coldplug recover from hotplug events
 *     mishandled before system is fully running:  "modprobe $(cat modalias)"
 */
static ssize_t modalias_show(struct device *dev, struct device_attribute *a,
			     char *buf)
{
	struct pmu_device	*device = to_pmu_device(dev);
	int len = snprintf(buf, PAGE_SIZE, "pmu:%s\n", device->name);

	return (len >= PAGE_SIZE) ? (PAGE_SIZE - 1) : len;
}

static struct device_attribute pmu_dev_attrs[] = {
	__ATTR_RO(modalias),
	__ATTR_NULL,
};

static const struct pmu_device_id *pmu_match_id(const struct pmu_device_id *id,
												const struct pmu_client *client)
{
	while (id->name[0]) {
		if (strcmp(client->name, id->name) == 0)
			return id;
		id++;
	}
	return NULL;
}

static int pmu_device_match(struct device *dev, struct device_driver *drv)
{
	struct pmu_client *client = to_pmu_client(dev);
	struct pmu_driver *driver = to_pmu_driver(drv);

//	printk(KERN_INFO PFX "match driver [%s] and device [%s]\n", 
//			driver->driver.name, client->name);

	/* match against the id table first */
	if (driver->id_table)
		return pmu_match_id(driver->id_table, client) != NULL;

	/* fall-back to driver name match */
	return (strcmp(client->name, driver->driver.name) == 0);
}

static int pmu_device_probe(struct device *dev)
{
	struct pmu_client       *client = to_pmu_client(dev);
	struct pmu_driver       *driver = to_pmu_driver(dev->driver);
	int status;

	if (!driver->probe || !driver->id_table)
		return -ENODEV;

	client->driver = driver;

//	printk(KERN_INFO PFX "probe driver [%s]\n", driver->driver.name);

	status = driver->probe(client, pmu_match_id(driver->id_table, client));
	if (status)
		client->driver = NULL;

	return status;
}

static int pmu_device_remove(struct device *dev)
{
	struct pmu_client       *client = to_pmu_client(dev);
	struct pmu_driver       *driver;
	int                     status;

	if (!dev->driver)
		return 0;

	driver = to_pmu_driver(dev->driver);
	if (driver->remove) {
//		printk(KERN_INFO PFX "remove driver [%s]\n", driver->driver.name);
		status = driver->remove(client);
	} else {
		dev->driver = NULL;
		status = 0;
	}
	if (status == 0)
		client->driver = NULL;

	return status;
}

static void pmu_device_shutdown(struct device *dev)
{
	struct pmu_driver *driver;

	if (!dev->driver)
		return;

	driver = to_pmu_driver(dev->driver);
	if (driver->shutdown) {
		printk(KERN_INFO PFX "shutdown driver [%s]\n", driver->driver.name);
		driver->shutdown(to_pmu_client(dev));
	}
}


static int pmu_device_suspend(struct device *dev, pm_message_t mesg)
{
	struct pmu_driver *driver;

	if (!dev->driver)
		return 0;

	driver = to_pmu_driver(dev->driver);
	if (!driver->suspend)
		return 0;

	printk(KERN_INFO PFX "suspend driver [%s]\n", driver->driver.name);

	return driver->suspend(to_pmu_client(dev), mesg);
}

static int pmu_device_resume(struct device *dev)
{
	struct pmu_driver *driver;

	if (!dev->driver)
		return 0;

	driver = to_pmu_driver(dev->driver);
	if (!driver->resume)
		return 0;

	printk(KERN_INFO PFX "resume driver [%s]\n", driver->driver.name);

	return driver->resume(to_pmu_client(dev));
}

struct bus_type pmu_bus_type = {
	.name           = "pmu",
	.dev_attrs      = pmu_dev_attrs,
	.match          = pmu_device_match,
	.probe          = pmu_device_probe,
	.remove         = pmu_device_remove,
	.shutdown       = pmu_device_shutdown,
	.suspend        = pmu_device_suspend,
	.resume         = pmu_device_resume,
};


/**
 * pmu_verify_client - return parameter as pmu_client, or NULL
 * When traversing the driver model tree, use this function to 
 * avoid oopses caused by wrongly treating some non-PMU device
 * as a pmu_client.
 */
struct pmu_client *pmu_verify_client(struct device *dev)
{
	return (dev->bus == &pmu_bus_type)
			? to_pmu_client(dev)
			: NULL;
}
EXPORT_SYMBOL(pmu_verify_client);

static ssize_t pmu_show_adapter_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pmu_adapter *adap = to_pmu_adapter(dev);
	return sprintf(buf, "%s\n", adap->name);
}

static struct device_attribute pmu_adapter_attrs[] = {
	__ATTR(name, S_IRUGO, pmu_show_adapter_name, NULL),
	{ },
};

static struct class pmu_adapter_class = {
	.owner		= THIS_MODULE,
	.name		= "pmu-adapter",
	.dev_attrs	= pmu_adapter_attrs,
};


static int _pmu_bus_lock(struct pmu_adapter *adap)
{
	// irqs are not only disabled when an interrupt happens...
	if (in_atomic() || irqs_disabled()) {
		return mutex_trylock(&adap->bus_lock);
	} else {
		mutex_lock_nested(&adap->bus_lock, adap->level);
		return 1;
	}
}

static int _pmu_bus_transfer(struct pmu_client *client, int addr, void *data,
							int len, e_transfer_t transfer)
{
	struct pmu_adapter *adap = client->adapter;
	int ret;

	switch(transfer) {
	  case e_read:
		if (adap->algo->read) {
			ret = adap->algo->read(adap, addr, data, len);
		} else {
			printk(KERN_WARNING PFX "PMU read transfers not supported\n");
			ret = -EOPNOTSUPP;
		}
		break;
	  case e_write:
		if (adap->algo->write) {
			ret = adap->algo->write(adap, addr, data, len);
		} else {
			printk(KERN_WARNING PFX "PMU write transfers not supported\n");
			ret = -EOPNOTSUPP;
		}
		break;
	  default:
		printk(KERN_WARNING PFX "PMU transfer not supported [%d]\n", transfer);
		ret = -EOPNOTSUPP;
		break;
	}
	return ret;
}

static int pmu_bus_transfer(struct pmu_client *client, int addr, void *data,
							int len, e_transfer_t transfer)
{
	struct pmu_adapter *adap = client->adapter;
	int ret;

	ret = _pmu_bus_lock(adap);
	if (!ret) {
		printk (KERN_WARNING "pmu_bus: PMU activity is ongoing ... retry later (%d, %d)\n", in_atomic(), irqs_disabled());
		/* PMU activity is ongoing. */
		ret = -EAGAIN;
		goto no_mutex;
	}

	ret = _pmu_bus_transfer(client, addr, data, len, transfer);

	mutex_unlock(&adap->bus_lock);

no_mutex:
	return ret;
}

/* ------------------------------------------------------------------------- */

/* For sequential reading/writing on the pmu bus.    */
/* client  : pmu client                              */                     
/* addr    : address on device                       */ 
/* data    : data to be transferred.                 */ 
/* len     : number of bytes to transfer.            */
int pmu_bus_seqread(struct pmu_client *client, int addr, void *data, int len)
{
	return pmu_bus_transfer(client, addr, data, len, e_read);
}
EXPORT_SYMBOL(pmu_bus_seqread);

int pmu_bus_seqwrite(struct pmu_client *client, int addr, void *data, int len)
{
	return pmu_bus_transfer(client, addr, data, len, e_write);
}
EXPORT_SYMBOL(pmu_bus_seqwrite);

#define PMU_BUS_OPERATION(TEST_OP, OPERATION) \
do { \
	struct pmu_adapter *adap = client->adapter; \
	int ret; \
	unsigned long old = 0, new = 0; \
 \
	if (WARN_ON(nr > 7)) { \
		ret = -EINVAL; \
		goto no_mutex; \
	} \
	if (WARN_ON(nr < 0)) { \
		ret = -EINVAL; \
		goto no_mutex; \
	} \
 \
	ret = _pmu_bus_lock(adap); \
	if (!ret) { \
		printk (KERN_WARNING "pmu_bus: PMU activity is ongoing ... retry later (%d, %d)\n", in_atomic(), irqs_disabled()); \
		/* PMU activity is ongoing. */ \
		ret = -EAGAIN; \
		goto no_mutex; \
	} \
 \
	/* if we get 0 from the transfer something fishy is going on... */ \
	ret = _pmu_bus_transfer(client, addr, &old, 1, e_read); \
	WARN_ON(ret == 0); \
	if (ret == 1) { \
		new = old; \
		OPERATION(nr, &new); \
		ret = _pmu_bus_transfer(client, addr, &new, 1, e_write); \
		WARN_ON(ret == 0); \
		/* printk("%s: addr 0x%x, nr %d, read 0x%lx, written 0x%lx\n", __FUNCTION__, addr, nr, old, new); */ \
	} \
 \
	mutex_unlock(&adap->bus_lock); \
 \
no_mutex: \
	if (ret >= 0) { \
		if (TEST_OP) \
			ret = (old >> nr) & 1; \
		else \
			ret = 0; \
	} \
	return ret; \
} while(0)

int pmu_bus_set_bit(struct pmu_client *client, int addr, int nr)
{
	PMU_BUS_OPERATION( 0, __set_bit );
}
EXPORT_SYMBOL(pmu_bus_set_bit);

int pmu_bus_clear_bit(struct pmu_client *client, int addr, int nr)
{
	PMU_BUS_OPERATION( 0, __clear_bit );
}
EXPORT_SYMBOL(pmu_bus_clear_bit);

int pmu_bus_change_bit(struct pmu_client *client, int addr, int nr)
{
	PMU_BUS_OPERATION( 0, __change_bit );
}
EXPORT_SYMBOL(pmu_bus_change_bit);

int pmu_bus_test_and_set_bit(struct pmu_client *client, int addr, int nr)
{
	PMU_BUS_OPERATION( 1, __set_bit );
}
EXPORT_SYMBOL(pmu_bus_test_and_set_bit);

int pmu_bus_test_and_clear_bit(struct pmu_client *client, int addr, int nr)
{
	PMU_BUS_OPERATION( 1, __clear_bit );
}
EXPORT_SYMBOL(pmu_bus_test_and_clear_bit);

int pmu_bus_test_and_change_bit(struct pmu_client *client, int addr, int nr)
{
	PMU_BUS_OPERATION( 1, __change_bit );
}
EXPORT_SYMBOL(pmu_bus_test_and_change_bit);


/* ------------------------------------------------------------------------- */
/* Bus Backend Registration. */

static void pmu_adapter_dev_release(struct device *dev)
{
	struct pmu_adapter *adap = NULL;

	BUG_ON(!dev);

	adap = to_pmu_adapter(dev);
	BUG_ON(!adap);

	complete(&adap->dev_released);
}

static int pmu_do_del_adapter(struct device_driver *d, void *data)
{
	struct pmu_driver  *driver  = to_pmu_driver(d);
	struct pmu_adapter *adapter = data;
	struct pmu_client  *client, *_n;

	/* Remove the devices we created ourselves as the result of hardware
	 * probing (using a driver's detect method) */
	list_for_each_entry_safe(client, _n, &driver->clients, detected) {
		if (client->adapter == adapter) {
			printk(KERN_INFO PFX "Removing %s\n", client->name);
			list_del(&client->detected);
			pmu_unregister_device(client);
		}
	}

	return 0;
}

static int pmu_do_add_adapter(struct device_driver *d, void *data)
{
//	struct pmu_driver *driver = to_pmu_driver(d);
//	struct pmu_adapter *adap = data;

	/* Detect already registered devices on that bus, and instantiate them */
	// Not yet implemented.

	return 0;
}

static int pmu_register_adapter(struct pmu_adapter *adap)
{
	int res = 0;

	/* Can't register until after driver model init */
	if (unlikely(WARN_ON(!pmu_bus_type.p))) {
		return -EAGAIN;
	}

	mutex_init(&adap->bus_lock);

	dev_set_name(&adap->dev, "pmu-%s", adap->name);
	adap->dev.release = &pmu_adapter_dev_release;
	adap->dev.class   = &pmu_adapter_class;

	res = device_register(&adap->dev);
	if (res) {
		printk(KERN_ERR PFX "could NOT register adapter [%s]\n", adap->name);
		
		mutex_lock(&core_lock);
		core_adapter = NULL;
		mutex_unlock(&core_lock);
		return res;
	}

	printk(KERN_INFO PFX "adapter [%s] registered\n", adap->name);

	core_adapter = adap;

	/* Notify drivers */
	mutex_lock(&core_lock);
	res = bus_for_each_drv(&pmu_bus_type, NULL, adap,
							pmu_do_add_adapter);
	mutex_unlock(&core_lock);

	return 0;
}

int pmu_add_adapter(struct pmu_adapter *adap)
{
	int res = 0;

	mutex_lock(&core_lock);
	if (NULL != core_adapter) {
		/* an adapter is already registered. */
		printk(KERN_WARNING PFX "could NOT add adapter [%s]\n", adap->name);
		res = -EBUSY;
	}
	mutex_unlock(&core_lock);

	if (0 == res)
		res = pmu_register_adapter(adap);

	return res;
}
EXPORT_SYMBOL(pmu_add_adapter);

static int pmu_unregister_client(struct device *dev, void *dummy)
{
	struct pmu_client *client = pmu_verify_client(dev);
	if (client)
		pmu_unregister_device(client);
	return 0;
}

int pmu_del_adapter(struct pmu_adapter *adap)
{
	int res = 0;

	/* First make sure that this adapter was ever added */
	if (core_adapter != adap) {
		printk(KERN_WARNING PFX "pmu-core: attempting to delete unregistered "
								"adapter [%s]\n", adap->name);
		return -EINVAL;
	}

	/* Tell drivers about this removal */
	mutex_lock(&core_lock);
	res = bus_for_each_drv(&pmu_bus_type, NULL, adap,
							pmu_do_del_adapter);
	mutex_unlock(&core_lock);
	if (res)
		return res;

	/* Detach any active clients. This can't fail, thus we do not
		checking the returned value. */
	res = device_for_each_child(&adap->dev, NULL, pmu_unregister_client);

	/* clean up the sysfs representation */
	init_completion(&adap->dev_released);
	device_unregister(&adap->dev);

	/* wait for sysfs to drop all references */
	wait_for_completion(&adap->dev_released);

    printk(KERN_INFO PFX "adapter [%s] unregistered\n", adap->name);

	/* Clear the device structure in case this adapter is ever going to be
		added again */
	memset(&adap->dev, 0, sizeof(adap->dev));
	return 0;
}
EXPORT_SYMBOL(pmu_del_adapter);

int __init pmu_init(void)
{
	int res;

	printk(KERN_INFO PFX "pmu-core init\n");

	res = device_register(&pmu_bus);
	if (res) {
		printk(KERN_ERR PFX "Device Registration Failure !\n");
		return res;
	}

	res = bus_register(&pmu_bus_type);
	if (res) {
		printk(KERN_ERR PFX	"Bus Registration Failure !\n");
		device_unregister(&pmu_bus);
		return res;
	}

	res = class_register(&pmu_adapter_class);
	if (res) {
		bus_unregister(&pmu_bus_type);
		device_unregister(&pmu_bus);
		return res;
	}

	printk(KERN_INFO PFX "Core Registered\n");

	return 0;
}

static void __exit pmu_exit(void)
{
	class_unregister(&pmu_adapter_class);
	bus_unregister(&pmu_bus_type);
	device_unregister(&pmu_bus);
}

//postcore_initcall(pmu_init);
module_exit(pmu_exit);



