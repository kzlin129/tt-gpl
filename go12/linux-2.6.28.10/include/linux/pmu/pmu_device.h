/*
 * pmu_device.h - Generic, centralized driver model for Power Management Unit Devices
 *
 * Copied from platform_device.h
 * Copyright (c) 2001-2003 Patrick Mochel <mochel@osdl.org>
 *
 * This file is released under the GPLv2
 *
 */

#ifndef _PMU_DEVICE_H_
#define _PMU_DEVICE_H_

#include <linux/device.h>
#include <linux/irq.h>

typedef int (*pmu_device_ioctl_callback)(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

/**  pmu_device_regulator_map - Describes the configuration and capabilities of regulators in the PMU.
 * @available: boolean flag indicating availability of regulator
 * @programmable: boolean flag indicating programmability of regulator
 * @reg_addr: address of regulator control register for mode control
 * @reg_addr_volt: address of control register to change voltage
 * @min_mV: minimum voltage in mV
 * @max_mV: maximum voltage in mV
 * @mV_step: programmable voltage step size in mV
 * @vout_mask: Mask of bits in register
 * @vout_shift: Bit shift in register
 * @vout_to_mV_map: Map for converting register voltage to register value
 * @map_size: Size of register map
 *
 * There must be one entry of this structure for each switching regulator or LDO in
 * the PMU device.
 */

typedef struct pmu_device_regulator_map
{
	int available; 
	int programmable; 
	u8  reg_addr; 
	u8  reg_addr_volt; 
	u32 min_mV; 
	u32 max_mV;
	u32 mV_step; 
	u32 vout_mask; 
	u32 vout_shift; 
	u32 *vout_to_mV_map; 
	u32 map_size; 
} pmu_device_regulator_map_t;


/** struct pmu_bus_ops - Describes valid PMU Bus operations.
 * @read:  This callback performs reads of PMU device registers and
 *         returns the data. Data in PMU registers is assumed to be
 *         byte aligned unless the pmu_register_alignment setting in
 *         the pmu_device_platform_info struct, if present, indicates
 *         an alignment requirement of 16/32-bits. The length is in the
 *         units defined by the pmu_register_min_size setting in the
 *         pmu_device_platform_info structure. The PMU device platform
 *         driver detects erroneous alignments of PMU register address
 *         and length by the pmu_bus_seqread function and reports errors
 *         if they don't match the capability of the PMU device.
 * @write: This callback performs writes of PMU device registers. Data in
 *         PMU registers is assumed to be byte aligned unless the
 *         pmu_register_alignment setting in the pmu_device_platform_info
 *         struct, if present, indicates an alignment requirement of
 *         16/32-bits. The length is in the units defined by the
 *         pmu_register_min_size setting in the pmu_device_platform_info
 *         structure. The PMU device platform driver detects erroneous
 *         alignments of PMU register address and length by the
 *         pmu_bus_seqwrite function and reports errors if they don't
 *         match the capability of the PMU device.
 *
 * The pmu_bus_ops structure contains pointers to callbacks for handling
 * the PMU bus read and write operations for a specific PMU. The PMU
 * device must register a bus driver that is then used by any subdevices
 * (child devices) to perform any I/O transactions with the PMU. The actual
 * bus interface is therefore hidden from the subdevices and can be
 * changed without impact to subdevice implementations. 
 */
struct pmu_bus_ops
{
	int (*read)( int address, void *data, int length );
	int (*write)( int address, void *data, int length );
};

/** struct pmu_bus_device - PMU Bus device description
 * @dev:    Pointer to device structure for PMU Bus device.
 * @ops:    Pointer to pmu_bus_ops structure that provides
 *          the required read/write callbacks to implement
 *          communication with this PMU device.
 */
struct pmu_bus_device
{
	struct device		*dev;
	struct pmu_bus_ops	*ops;
};

/** struct pmu_device - Definition of PMU device
 * @name:                   name of PMU device
 * @id:                     ID number for PMU device
 * @dev:                    Pointer to device structure
 * @num_resources:          Number of resource structures
 *                          pointed to be @resource
 * @resource:               List of resources
 * @bus_device:             Bus device associated with this
 *                          PMU device.
 * @ioctl_callback_chain:   List of IOCTL callback functions
 *                          registered with this PMU device
 *                          by PMU subdevice drivers.
 *
 * There is one of these structures for each PMU device in
 * the system. Typically there will only be a single PMU in
 * a system but it is possible that a very complex system
 * could contain multiple PMU devices even of different types.
 */
struct pmu_device
{
	const char	* name;
	int		id;
	struct device	dev;
	u32		num_resources;
	struct resource	* resource;
	struct pmu_bus_device * bus_device;
	struct list_head ioctl_callback_chain;
};

/** struct pmu_device_platform_into - Platform data for Power Management Unit devices
 * @init:   This callback is typically used by the PMU device to perform
 *          initialization that is required before it is ready to provide
 *          services or to initialize hardware that must be ready prior
 *          to PMU subsystem drivers beginning to initialize themselves.
 * @remove: This callback is used to perform any cleanup or hardware
 *          shutdown when a PMU device is being remove, during
 *          system shutdown or when it is being removed from service for
 *          any reason.
 * @register_alignment: This variable defines the alignment requirements of
 *          accesses to the PMU device registers allowable via the read and
 *          write functions. If this value is 0 or 1 that indicates byte
 *          alignment. Values of 2 and 4 indicate 16-bit and 32-bit alignment
 *          requirements, respectively.
 * @register_min_size: This variable defines the alignment requirements of
 *          accesses to the PMU device registers allowable via the read and
 *          write functions. If this value is 0 or 1 that indicates accesses
 *          may be in units of bytes. Values of 2 and 4 indicate 16-bit and
 *          32-bit units for size, respectively.
 * @num_of_subdevices: This is a count of the number of sub or child
 *          devices in the table referenced by @subdevices. This set of
 *          subdevices will be register by the pmu_device framework after
 *          the parent device is registered. No additional calls to
 *          pmu_device_register are required to register these subdevices.
 * @subdevices:        This points to a list of registers sub or child
 *          devices of this PMU device. As mentioned for @num_of_subdevices
 *          this is a list of one or more subdevices that will be
 *          registered by the PMU device framework after the parent
 *          PMU device is registered.
 *
 * When calling pmu_device_register this structure may be
 * passed with the dev structure referenced via the platform_data
 * pointer. If this structure is present the pmu_device framework
 * calls back the function references appropriately.
 */

struct pmu_device_platform_info
{

	int (*init)(struct pmu_device *pmu_device);
	void (*remove)(struct pmu_device *pmu_device);
	int register_alignment;
	int register_min_size;
	int num_of_pmu_subdevices;
	struct pmu_device *subdevices;
};



#define to_pmu_device(x) container_of((x), struct pmu_device, dev)

/** struct ioctl_callback_entry - Definition for an entry in the IOCTL callback chain.
 * @list:           list head structure for list management
 * @low_bound:      Value for low end of IOCTL range reservation.
 * @high_bound:     Value for high end of IOCTL range reservation.
 * @ioctl_fnc:		Pointer to IOCTL handler function.
 *
 * Subdevices register IOCTL function ranges they want to handle
 * via the @pmu_ioctl_register function which adds the registrations
 * to this list in order sorted by the IOCTL function ranges. This
 * list is run when an IOCTL is received and the @process_ioctl_chain
 * function is called, typically by the main PMU driver (usually named
 * the "core" driver. It receives all the original IOCTl functions and
 * calls @pmu_run_ioctl_chain to let pmu_device.c "run" the chain and
 * give each subdevice driver its chance to handle the IOCTL function.
 */

struct ioctl_callback_entry
{
	struct list_head list;
	int low_bound;
	int high_bound;
	pmu_device_ioctl_callback ioctl_fnc;
};


/** pmu_device_register - Register this PMU device with the framework.
 * @dev:    PMU device to register with PMU device framework.
 *
 * Returns a standard Linux error value (0 for successful).
 */
extern int pmu_device_register(struct pmu_device *dev);

/** pmu_device_unregister - Unregister this PMU device with the framework.
 * @dev:    PMU device to unregister with PMU device framework.
 *
 * Returns a standard Linux error value (0 for successful).
 */
extern void pmu_device_unregister(struct pmu_device *dev);

extern struct bus_type pmu_bus_type;
extern struct pmu_bus_device pmu_bus;

/** pmu_get_resource - Get a resource associated with this PMU device.
 * @dev:    PMU device to obtain resource for.
 * @type:   Type of resource to search for such as I/O, memory, IRQ or
 *          DMA.
 * @num:    Index value into list of resources. 0 means first resource
 *          in list of type specified by @type. 1 means second resource
 *          in list of type specified by @type  and so forth.
 *
 * Returns pointer to resource structure or NULL if it couldn't be found.
 */
extern struct resource *pmu_get_resource(struct pmu_device *dev, unsigned int type, unsigned int num);

/** pmu_get_irq - get an IRQ for a device
 * @dev:    pmu device
 * @num:    IRQ number index
 *
 * This function is basically a @pmu_get_resource with the type forced
 * to be IORESOURCE_IRQ.
 */
extern int pmu_get_irq(struct pmu_device *dev, unsigned int num);

/** pmu_get_resource_byname - Get a resource associated with this PMU device, searching by name.
 * @dev:    PMU device to obtain resource for.
 * @type:   Type of resource to search for such as I/O, memory, IRQ or
 *          DMA.
 * @name:   Name of resource to search for with list of resources of
 *          type specified by @type.
 *
 * Returns pointer to resource structure or NULL if it couldn't be found.
 */
extern struct resource *pmu_get_resource_byname(struct pmu_device *dev, unsigned int type, char *name);

/**
 * pmu_get_irq - get an IRQ for a device
 * @dev:    PMU device to find IRQ for.
 * @name:   IRQ name to search resource list for.
 *
 * This function is basically a @pmu_get_resource_byname with the type forced
 * to be IORESOURCE_IRQ.
 */
extern int pmu_get_irq_byname(struct pmu_device *dev, char *name);

/** pmu_add_devices - add a number of PMU devices
 * @devs: array of PMU devices to add
 * @num: number of PMU devices in array
 */
extern int pmu_add_devices(struct pmu_device **devs, int num);

/**
 * pmu_device_put
 * @pdev:   PMU device to free
 *
 * Free all memory associated with a PMU device.  This function must
 * _only_ be externally called in error cases.  All other usage is a bug.
 */
extern void pmu_device_put(struct pmu_device *pdev);

/** pmu_device_alloc - Create/Allocate and PMU device object
 * @name:   base name of the device we're adding
 * @id:     instance id
 *
 * Create a PMU device object which can have other objects attached
 * to it, and which will have attached objects freed when it is released.
 */
extern struct pmu_device *pmu_device_alloc(const char *name, int id);

/** pmu_device_add_resources - Add resource definitions to a PMU device
 * @pdev:   platform device allocated by pmu_device_alloc to add resources to
 * @res:    set of resources that needs to be allocated for the device
 * @num:    number of resources
 *
 * Add a copy of the resources to the platform device.  The memory
 * associated with the resources will be freed when the platform device is
 * released.
 */
extern int pmu_device_add_resources(struct pmu_device *pdev,
				  					struct resource *res,
									unsigned int num);

/** pmu_device_add_data - Add platform data to a PMU device.
 * @pdev:   PMU device allocated by pmu_device_alloc to add data to
 * @data:   Platform specific data for this PMU device
 * @size:   Size of platform specific data
 *
 * Add a copy of platform specific data to the PMU device's
 * platform_data pointer.  The memory associated with the platform data
 * will be freed when the platform device is released.
 */
extern int pmu_device_add_data(struct pmu_device *pdev,
							   const void *data,
			     			   size_t size);

/** pmu_device_add - add a platform device to device hierarchy
 * @pdev:   PMU device we're adding
 *
 * This is part 2 of pmu_device_register(), though may be called
 * separately _iff_ pdev was allocated by pmu_device_alloc().
 */
extern int pmu_device_add(struct pmu_device *pdev);

/** pmu_device_del - remove a platform-level device
 * @pdev:   PMU device we're removing
 *
 * Note that this function will also release all memory- and port-based
 * resources owned by the device (@dev->resource).  This function must
 * _only_ be externally called in error cases.  All other usage is a bug.
 */
extern void pmu_device_del(struct pmu_device *pdev);

/** pmu_device_register - add a PMU-level device
 * @pdev:   PMU device we're adding
 */
extern int pmu_device_register(struct pmu_device *pdev);

/** pmu_device_unregister - unregister a PMU-level device
 * @pdev:   PMU device we're unregistering
 *
 * Unregistration is done in 2 steps. First we release all resources
 * and remove it from the subsystem, then we drop reference count by
 * calling pmu_device_put().
 */
extern void pmu_device_unregister(struct pmu_device *pdev);

/** pmu_ioctl_register - Register a range of IOCTl function code to be routed to the specified callback function.
 * @pmu:    PMU device to register IOCTL callback to.
 * @low_bound: Lower boundary of IOCTl function number range to register IOCTL
 *          callback function for.
 * @high_bound: Upper boundary of IOCTl function number range to register IOCTL
 *          callback function for.
 * @ioctl_fnc: Address of IOCTL callback function.
 *
 * IOCTLs are used by the pmctl utility to perform low-level testing of the
 * driver and hardware functions. Since the IOCTLs are specific to different
 * functional blocks of the PMU to IOCTL handlers need to be spread across
 * several different drivers. This function allows those drivers to register
 * themselves to receive a range of IOCTL function values from the main PMU
 * driver. This allows higher level applications to call the main PMU driver
 * regardless of the function to be performed. It will be routed to the
 * correct driver based upon the IOCTL function value ranges that each driver
 * registers with the PMU device framework. If the registration is valid
 * successful status (zero) is returned otherwise -EINVAL is returned.
 */
extern int pmu_ioctl_register(struct pmu_device *pmu,
							  int low_bound,
							  int high_bound,
							  pmu_device_ioctl_callback ioctl_fnc);

/** pmu_ioctl_unregister - Unregister a range of IOCTl function codes.
 * @pmu:    PMU device to unregister IOCTL callback from.
 * @low_bound: Lower boundary of IOCTl function number range to register IOCTL
 *          callback function for.
 * @high_bound: Upper boundary of IOCTl function number range to register IOCTL
 *          callback function for.
 * @ioctl_fnc: Address of IOCTL callback function.
 *
 * This function allows a previous registration of a range of IOCTL values
 * to be withdrawn. The range (@low_bound-@high_bound) and @ioctl_fnc must
 * match an existing registration. If a matching registration is found it
 * is removed from the routing table and successful (zero) status is returned,
 * otherwise -EINVAL is returned.
 */
extern int pmu_ioctl_unregister(struct pmu_device *pmu,
								int low_bound,
								int high_bound,
								pmu_device_ioctl_callback ioctl_fnc);

/** pmu_run_ioctl_chain - Run the registered IOCTLs to process a received IOCTL function.
 * @pmu:    PMU device to run IOCTL chain for.
 *
 * This function will scan the IOCTL chain to find an entry that the IOCTL function
 * code value fits within the low/high bound of and, if found, call the IOCTL callback
 * function associated with that entry in the list.
 */
extern int pmu_run_ioctl_chain(struct pmu_device *pmu,
							   struct inode *inode,
							   struct file *file,
							   unsigned int cmd,
							   unsigned long arg);

/** struct pmu_driver - PMU Driver Structure definition
 * @probe:      Callback from PMU device framework that detects presence of
 *              correct PMU device and performs necessary configuration and
 *              setup of both hardware and software associated with the PMU
 *              device.
 * @remove:     Callback from PMU device framework called at device removal
 *              time. It places the hardware into a state commensurate with
 *              that fact that it won't be in use until probe is called again.
 *              That state is probably as similar as possible to the default
 *              power on condition/state.
 * @shutdown:   Callback from PMU device framework when system is shutting
 *              down. Device hardware should be placed into a state that
 *              is appropriate for an imminent power loss.
 * @suspend:    Legacy style suspend callback from PMU device framework.
 * @suspend_late: Legacy style suspend_late callback from PMU device framework.
 * @resume:     Legacy style resume callback from PMU device framework.
 * @resume_early: Legacy style resume_early callback from PMU device framework.
 * @ioctl_callback: ioctl callback from PMU device framework. This callback is
 *              used to provide a unified ioctl interface to the PMU device
 *              as a whole with individual ranges of ioctl function values
 *              logically mapped to individual PMU subdevice drivers.
 * @pm:         Structure for describing power management device handling
 *              under the new Linux PM infrastructure.
 * @driver:     Pointer to driver for this PMU device.
 *
 * This contains the callbacks that are provided by PMU subsystem drivers.
 * Subsystem drivers implement the subsets of functionality integrated within
 * most PMU devices. For example, many PMU devices include regulators, LDOs,
 * chargers and battery monitoring. Some PMU devices integrate additional
 * features such as Real-time Clocks, GPIOs, LEDs and other features. These
 * are all considered subsystems within the context of the PMU device/driver
 * framework. Each PMU subsystem driver must register via the PMU device/driver
 * framework. Each driver must create and initialize a pmu_driver structure and
 * then register it with the PMU framework via the pmu_driver_register function.
 *
 * The PMU framework is responsible for routing communications to the proper
 * PMU bus driver. PMU bus drivers must register with the PMU framework also.
 * The the PMU framework acts as a "middle-man" routing traffic from the
 * PMU subsystem drivers to the proper PMU bus driver.
 *
 * Suspend/resume and other such events are also handled by the PMU framework.
 * They are forwarded to each subsystem driver in the order in which
 * they are registered with the PMU framework driver. This ensures an orderly
 * entry and exit from suspend states eliminating race and contention
 * between subsystem drivers. It also ensure that subsystem drivers are
 * permitted to properly perform any PMU device configuration prior
 * to the PMU driver shutting down the bus (backend) driver interface.
 *
 */

struct pmu_driver
{
	int (*probe)(struct pmu_device *);
	int (*remove)(struct pmu_device *);
	void (*shutdown)(struct pmu_device *);
	int (*suspend)(struct pmu_device *, pm_message_t state);
	int (*suspend_late)(struct pmu_device *, pm_message_t state);
	int (*resume_early)(struct pmu_device *);
	int (*resume)(struct pmu_device *);
	int (*ioctl_callback)(struct pmu_device *, struct inode *inode,
						  struct file *file, unsigned int cmd,
						  unsigned long arg);
	struct pm_ext_ops *pm;
	struct device_driver driver;
};

/** pmu_driver_register - Register a PMU driver into the framework.
 * @drv:    Pointer to pmu_driver structure that describes driver to be
 *          registered into PMU framework.
 */
extern int pmu_driver_register(struct pmu_driver *drv);

/** pmu_driver_unregister - Unregister a PMU driver into the framework.
 * @drv:    Pointer to pmu_driver structure that describes driver to be
 *          unregistered from PMU framework.
 *
 * This function returns zero for successful read operations and one of
 * the valid Linux error return codes otherwise.
 */
extern void pmu_driver_unregister(struct pmu_driver *drv);

struct pmu_driver_evt_desc;

typedef void (*pmu_evt_handler_t)(int evt, void *evt_data);

typedef void (*pmu_evt_flow_handler_t)(struct pmu_device *pdev,
									   struct pmu_driver_evt_desc *pdesc);

/** struct pmu_evt_action - Defines action entry for PMU events
 * @next:       Pointer to next action entry in list for a particular event.
 * @handler:    Pointer to event handler for the action.
 * @flags:      Status flags reflecting condition and operation of this action.
 * @name:       Name of action.
 *
 * Actions are arranged in linked lists of one or more attached to a particular
 * event. Handlers are registered via the @pmu_add_handler function. The PMU
 * device framework and the core (or primary) driver of a particular PMU device
 * will typically work together to translate the hardware interrupts and the
 * status associated with those interrupts into events that are passed to the
 * PMU subdevice drivers via handlers they've registered via this mechanism.
 * The callbacks via these handlers occur in the context of a thread rather
 * than from interrupt context. Since some PMU devices communicate over I2C
 * the interrupt status/condition cannot be determined from interrupt context
 * since I2C access is not possible during interrupt context execution. Thus
 * the subdevice drivers are isolated from this.
 */
typedef struct pmu_evt_action
{
	struct pmu_evt_action		*next;
	pmu_evt_handler_t			handler;
	uint32_t					flags;
	const char					*name;
} pmu_evt_action_t;

/** struct pmu_driver_evt_desc - Defines a PMU event descriptor
 * @next:           Pointer to next event descriptor.
 * @flow_handler:   Pointer to function to handle flow/translation for this event.
 * @action:         Pointer to action list.
 * @status:         Status flags associated with this event.
 * @name:           Name of this event.
 *
 * Each PMU device may define one or more events. Each event is typically
 * describing a related group of possible conditions. The PMU device framework
 * and the core (or primary) driver for a particular PMU device work together
 * to translate interrupts and status read from the PMU device into one or
 * more events. Each event represents one or more interrupt conditions
 * decoded from PMU interrupts and PMU register information. For example,
 * several related interrupt conditions relating to the charger might be
 * translated into a single event. Several other conditions relating to
 * operation of the ADCs in a PMU might be another event. This logical
 * grouping of multiple interrupt conditions into a single event simplifies
 * the registration process for PMU subdevice drivers. Instead of having
 * to register for several individual interrupts they can register for,
 * in many cases, one event and then further decode the event within a
 * single handler function from information provided by the PMU device
 * framework.
 */

typedef struct pmu_driver_evt_desc
{
	struct pmu_driver_evt_desc	*next;
	pmu_evt_flow_handler_t		*flow_handler;
	pmu_evt_action_t			*action;
	uint32_t					status;
	const char					*name;
} pmu_driver_evt_desc_t;

#define pmu_get_drvdata(_dev)		dev_get_drvdata(&(_dev)->dev)
#define pmu_set_drvdata(_dev,data)	dev_set_drvdata(&(_dev)->dev, (data))

/** pmu_bus_seqread - Read data from a sequential set of PMU device registers.
 * @dev:        PMU device to communicate with.
 * @address:    PMU device register starting address.
 * @data:       Buffer to store read data from PMU registers into.
 * @length:     Number of bytes to transfer starting from @address.
 *
 * This function returns zero for successful write operations and one of
 * the valid Linux error return codes otherwise.
 */
extern int pmu_bus_seqread( struct pmu_device *dev, int address, void *data, int length );

/** pmu_bus_seqwrite - Write data to a sequential set of PMU device registers.
 * @dev:        PMU device to communicate with.
 * @address:    PMU device register starting address.
 * @data:       Buffer holding data to write into PMU registers.
 * @length:     Number of bytes to transfer starting from @address.
 */
extern int pmu_bus_seqwrite( struct pmu_device *dev, int address, void *data, int length );


/** pmu_bus_register - Register a PMU BUS device with the PMU framework.
 * @bus:    Bus device to register.
 *
 * Each PMU should have a bus device (and only a single bus device per PMU
 * device is allowed) register to provide communications services. The purpose
 * of the bus driver is to provide isolation from the actual physical bus
 * interface of the PMU device from the PMU drivers.
 *
 * Until a bus device is registered and initialized for a PMU device the
 * @pmu_bus_seqread and @pmu_bus_seqwrite functions will be rejected (failed)
 * by the PMU framework.
 */
extern int pmu_bus_register( struct pmu_bus_device *bus );

/** pmu_bus_unregister - Unregister a PMU BUS device with the PMU framework.
 * @bus:    Bus device to unregister.
 *
 * Once a bus device is unregistered from a PMU device the @pmu_bus_seqread
 * and @pmu_bus_seqwrite functions will be rejected (failed) by the PMU
 * framework.
 */
extern int pmu_bus_unregister( struct pmu_bus_device *bus );

#endif /* _PMU_DEVICE_H_ */
