/**
 * \file
 * Synaptics Register Mapped Interface (RMI) Header File.
 * Copyright (c) 2008-2009 Synaptics Incorporated
 *
 */

/** RMI4 Protocol Support
 */

/** For each function present on the RMI device, we need to get the RMI4 Function
 *  Descriptor info from the Page Descriptor Table. This will give us the
 *  addresses for Query, Command, Control, Data and the Source Count (number
 *  of sources for this function) and the function id.
 */
struct rmi_function_descriptor {
  unsigned char queryBaseAddr;
  unsigned char commandBaseAddr;
  unsigned char controlBaseAddr;
  unsigned char dataBaseAddr;
  unsigned char interruptSrcCnt;
  unsigned char functionNum;
};

/** For each function present on the RMI device, there will be a corresponding
 * entry in the functions list of the rmi_module_info structure.  This entry
 * gives information about the number of data sources and the number of data
 * registers associated with the function.
 * \see rmi_module_info
 */
struct rmi_function_info {
  unsigned int  functionCnt;
  unsigned char functionNum;

  /** This is the number of data sources associated with the function.
   * \note This is not the total number of data registers associated with
   * this function!
   * \see data_regs
   */
  unsigned char numSources;
  
  /** This is the number of data points supported - for example, for
   *  function $11 (2D sensor) the number of data points is equal to the number
   *  of fingers - for function $19 (buttons)it is eqaul to the number of buttons
   */
  unsigned char numDataPoints; 

  /** This is the interrupt register and mask - needed for enabling the interrupts
   *  and for checking what source had caused the attention line interrupt.
   */
  unsigned char interruptRegister;
  unsigned char interruptMask;

  /** This is the RMI function descriptor associated with this function.
   *  It contains the Base addresses for the functions query, command, 
   *  control, and data registers.
   */
  struct rmi_function_descriptor funcDescriptor;

  /** Standard kernel linked list implementation.
   * Documentation on how to use it can be found at
   * http://isis.poly.edu/kulesh/stuff/src/klist/.
   */
  struct list_head link;
};

/** This encapsulates the information found using the RMI4 Function $01 
 *  query registers.
 *  Assuming appropriate endian-ness, you can populate most of this
 *  structure by reading query registers starting at the query base address
 *  that was obtained from RMI4 function 0x01 function descriptor info read
 *  from the Page Descriptor Table. 
 *
 *  Specific register information is provided in the comments for each field.
 *  For further reference, please see the <i>Synaptics RMI Interfacing
 *  Guide</i> document.
 */
struct rmi_module_info {
  /** The Protocol Major Version number.
   */
  unsigned rmi_maj_ver;

  /** The Protocol Minor Version number.
   */
  unsigned rmi_min_ver;

  /** The manufacturer identification byte.
   */
  unsigned char mfgid;

  /** The Product Properties information.
   */
  unsigned char properties;

  /** The product info bytes.
   * You can build a product info string using the following printf
   * format and args:
   * \code printf("%i %02i", productInfo[0], productInfo[1]); \endcode
   */
  unsigned char prod_info[2];

   /** Date Code - Year, Month, Day.
    */
  unsigned char date_code[3];

  /** Tester ID (14 bits).
   */
  unsigned short tester_id;

  /** Serial Number (14 bits).
   */
  unsigned short serial_num;

  /** A null-terminated string that identifies this particular product.
   */
  char prod_id[10];

  /** A list of the function presence queries.
   * This list uses the standard kernel linked list implementation.
   * Documentation on on how to use it can be found at
   * http://isis.poly.edu/kulesh/stuff/src/klist/.
   * \see rmi_function_info
   */
  struct list_head functions;
};

struct rmi_phys_driver {
  char *name;
  int  (*write)(struct rmi_phys_driver *pd, unsigned short address, char data);
  int  (*read)(struct rmi_phys_driver *pd, unsigned short address, char *buffer);
  int  (*write_multiple)(struct rmi_phys_driver *pd, unsigned short address, void *buffer, int length);
  int  (*read_multiple)(struct rmi_phys_driver *pd, unsigned short address, void *buffer, int length);
  void (*attention)(struct rmi_phys_driver *pd, int instance);
  int  (*get_attention)(struct rmi_phys_driver *pd);
  int  polling_required;
  /* Private elements of the structure */
  /** Standard kernel linked list implementation.
   * Documentation on how to use it can be found at
   * http://isis.poly.edu/kulesh/stuff/src/klist/.
   */
  struct list_head drivers;
  struct rmi_application *app;
  struct rmi_module_info rmi;
  struct module *module;
};

int rmi_read(struct rmi_application *app, unsigned short address, char *dest);
int rmi_write(struct rmi_application *app, unsigned short address, unsigned char data);
int rmi_read_multiple(struct rmi_application *app, unsigned short address, void *dest, int length);
int rmi_write_multiple(struct rmi_application *app, unsigned short address, void *data, int length);
int rmi_register_phys_driver(struct rmi_phys_driver *rpd);
int rmi_unregister_phys_driver(struct rmi_phys_driver *rpd);

struct rmi_application *rmi_register_application(const char *name,
  void (*attention)(struct rmi_phys_driver *pd, int instance),
  int (*probe)(struct rmi_application *app, const struct rmi_module_info *rmi),
  void (*config)(struct rmi_application *app));

void rmi_unregister_application(struct rmi_application *app);
int  rmi_polling_required(struct rmi_application *app);
int  rmi_get_attn(struct rmi_application *app);

/** Set this to 1 to turn on code used in detecting buffer leaks. */
#define RMI_ALLOC_STATS 1

#if RMI_ALLOC_STATS
extern int appallocsrmi;
#   define INC_ALLOC_STAT(X)   X##allocsrmi++
#   define DEC_ALLOC_STAT(X)   \
	do { if(X##allocsrmi) X##allocsrmi--; \
	     else printk("Too many " #X " frees\n"); } while(0)
#   define CHECK_ALLOC_STAT(X) \
	do { if(X##allocsrmi) printk("Left over " #X " buffers: %d\n", \
		X##allocsrmi); } while(0)
#else
#   define INC_ALLOC_STAT(X)   do { } while(0)
#   define DEC_ALLOC_STAT(X)   do { } while(0)
#   define CHECK_ALLOC_STAT(X) do { } while(0)
#endif

/* vim600: set noexpandtab sw=8 ts=8 :*/
