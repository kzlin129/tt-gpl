/**
 * \file
 * Synaptics Register Mapped Interface (RMI4) Data Layer Core Header.
 * Copyright (c) 2008-2009 Synaptics Incorporated.
 *
 * \note YOU SHOULD NOT NEED TO INCLUDE THIS FILE FOR PHYSICAL OR APPLICATION
 * LAYERS.
 */

struct rmi_application {
  const char *name;
  void (*attention)(struct rmi_phys_driver *pd, int instance);
  /** Probe Function
   *  This function is called to give the application layer an
   *  opportunity to claim an RMI device.  The application layer cannot
   *  read RMI registers at this point.  Defer that to the config
   *  function call which occurs immediately after a successful probe.
   */
  int (*probe)(struct rmi_application *app, const struct rmi_module_info *rmi);
  /** Config Function
   *  This function is called after a successful probe.  It gives the
   *  application driver an opportunity to query and/or configure an RMI
   *  device before data starts flowing.
   */
  void (*config)(struct rmi_application *app);
  /** Standard kernel linked list implementation.
   *  Documentation on how to use it can be found at
   *  http://isis.poly.edu/kulesh/stuff/src/klist/.
   */
  struct list_head apps;
  struct rmi_phys_driver *rpd;
  int polling_required;
};

/* vim600: set noexpandtab sw=8 ts=8 : */
