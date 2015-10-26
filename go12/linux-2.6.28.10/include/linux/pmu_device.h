/*
 * pmu_device.h - generic, centralized driver model
 *
 *
 * This file is released under the GPLv2
 *
 */

#ifndef _pmu_device_H_
#define _pmu_device_H_

#include <linux/device.h>
#include <linux/mod_devicetable.h>

#define PMU_NAME_SIZE 20

#define PMU_MODULE_PREFIX  "pmu:"

extern struct bus_type pmu_bus_type;

struct pmu_device_id {
	char name[PMU_NAME_SIZE];
	kernel_ulong_t driver_data
		__attribute__((aligned(sizeof(kernel_ulong_t))));
};

struct pmu_board_info {
	char  name[PMU_NAME_SIZE];
	void *platform_data;
};


struct pmu_device {
	const char	* name;
	int		id;
	struct device	dev;
	u32		num_resources;
	struct resource	* resource;
};

#define to_pmu_device(x) container_of((x), struct pmu_device, dev)

/**
 * pmu_register_device - register a pmu device
 * @adap: the adapter managing the device
 * @info: describes one pmu device
 */
extern struct pmu_client *
pmu_register_device(struct pmu_board_info const *info);

/**
 * pmu_unregister_device - reverse effect of pmu_register_device()
 * @client: value returned from pmu_ergister_device()
 */
extern void pmu_unregister_device(struct pmu_client *client);

#define pmu_get_drvdata(_client)	dev_get_drvdata(&(_client)->dev)
#define pmu_set_drvdata(_client,data)	dev_set_drvdata(&(_client)->dev, (data))

struct pmu_adapter;
struct pmu_client;

struct pmu_driver {
	int (*probe)(struct pmu_client *, const struct pmu_device_id *);
	int (*remove)(struct pmu_client *);
	void (*shutdown)(struct pmu_client *);
	int (*suspend)(struct pmu_client *, pm_message_t state);
	int (*resume)(struct pmu_client *);

	struct device_driver driver;

	struct pmu_device_id *id_table;

	struct list_head clients;
};

int pmu_register_driver(struct pmu_driver *driver);
extern void pmu_unregister_driver(struct pmu_driver *);

/**
 * An pmu_client identifies a single pmu device integrated to the pmu chip.
 */
struct pmu_client {
	char name[PMU_NAME_SIZE];
	struct pmu_adapter *adapter;    /* the adapter we sit on        */
	struct pmu_driver *driver;      /* and our access routines      */
	struct device dev;              /* the device structure         */

	struct list_head detected;
};
#define to_pmu_client(d) container_of(d, struct pmu_client, dev)

extern struct pmu_client *pmu_verify_client(struct device *dev);

/* -------------------------------------------------- */

struct pmu_algorithm {
	int (*read)  (struct pmu_adapter *adap, int address, 
					void *data, int length);
	int (*write) (struct pmu_adapter *adap, int address, 
					void *data, int length);
};

/*
 * pmu_adapter is the structure used to identify a physical pmu bus along
 * with the access algorithms necessary to access it.
 */
struct pmu_adapter {
	struct module *owner;
	const struct pmu_algorithm *algo; /* the algorithm to access the bus */
	void *algo_data;

	/* data fields that are valid for all devices   */
	u8 level;                       /* nesting level for bus lock */
	struct mutex bus_lock;
	struct device dev;              /* the adapter device */
	char name[48];
	struct completion dev_released;
};

#define to_pmu_adapter(d) container_of(d, struct pmu_adapter, dev)
static inline void *pmu_get_adapdata(const struct pmu_adapter *dev)
{
	return dev_get_drvdata(&dev->dev);
}

static inline void pmu_set_adapdata(struct pmu_adapter *dev, void *data)
{
	dev_set_drvdata(&dev->dev, data);
}


/* -------------------------------------------------- */

/* For sequential reading/writing on the pmu bus.     */
/* client  : pmu client                               */
/* addr    : address on device                        */ 
/* data    : data to be transferred.                  */
/* len     : number of bytes to transfer.             */
/* return  : the number of bytes written or -ve error code */
extern int pmu_bus_seqread (struct pmu_client *client, int addr, void *data, int len);
extern int pmu_bus_seqwrite(struct pmu_client *client, int addr, void *data, int len);

/* For atomic (on the bus) bit operations.
 * These operations ensure that a read-modify-write operation (for example masking an
 * interrupt) is atomic. So two users at the same time won't mess each others changes.
 *
 * Note that these operations are also atomic with respect to the seqread() and
 * seqwrite() above.
 *
 * These operations return either a -ve error value or the result of the
 * operation.
 */
extern int pmu_bus_set_bit(struct pmu_client *client, int addr, int nr);
extern int pmu_bus_clear_bit(struct pmu_client *client, int addr, int nr);
extern int pmu_bus_change_bit(struct pmu_client *client, int addr, int nr);
extern int pmu_bus_test_and_set_bit(struct pmu_client *client, int addr, int nr);
extern int pmu_bus_test_and_clear_bit(struct pmu_client *client, int addr, int nr);
extern int pmu_bus_test_and_change_bit(struct pmu_client *client, int addr, int nr);

/* For registering bus backend implementations. */
extern int pmu_add_adapter(struct pmu_adapter *);
extern int pmu_del_adapter(struct pmu_adapter *);
#endif /* _pmu_device_H_ */
