
/**
 * \file
 * Synaptics Register Mapped Interface (RMI4) TouchPad Application Layer Driver.
 * Copyright (c) 2008-2009 Synaptics Incorporated
 *
 *
 * This code implements a polling mechanism with backoff as well as
 * interrupt-driven sampling.  For polling, the module has two parameters:
 * polljif (Poll Jiffies) and hspolljif (High Speed Poll Jiffies).  The driver
 * polls at polljif until it sees a press, and then it polls at hspolljif.
 * When there is no longer a touch, it reverts to the polljif period.
 *
 * The polljif parameter can be changed during run time like this:\code
 *   echo 100 > /sys/module/rmi_app_touchpad/parameters/polljif
 * \endcode
 *
 * That will change the pollrate to 1 per second (assuming HZ == 100).  The
 * same is true for hspolljif.
 *
 * The parameters have no effect for the interrupt-driven case.
 *
 * Note that it is the lower-level drivers that determine whether this driver
 * has to do polling or interrupt-driven.  Polling can always be done, but if
 * we have an interrupt connected to the attention (ATTN) line, then it is
 * better to be interrupt driven.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/input.h>

#include "rmi.h"
#include "rmi_core.h"

#if RMI_ALLOC_STATS
  static int pollallocsrmi;
#endif

/** The number of jiffies between polls without touches */
static int polljif = HZ/60;
/** The number of jiffies between polls with touches */
static int hspolljif = HZ/60;
module_param(polljif, int, 0644);
module_param(hspolljif, int, 0644);
MODULE_PARM_DESC(polljif,   "How many jiffies between low speed polls.");
MODULE_PARM_DESC(hspolljif, "How many jiffies between high speed polls.");

static struct rmi_application *app;
static struct completion touch_completion;
static struct completion thread_comp;
struct task_struct *kthread;
static int time_to_quit;
static struct input_dev *input;

/* RMI4 function addresses read from PDT in rmi_core.c */
extern unsigned short fn01QueryBaseAddr;      /* RMI4 device control */
extern unsigned short fn01ControlBaseAddr;
extern unsigned int   interruptRegisterCount;

extern unsigned short fn11DataBaseAddr;       /* 2D capacitive sensor */
extern unsigned short fn11ControlBaseAddr;
extern unsigned short fn11FingersSupported;
extern unsigned int   fn11DataRegBlockSize;
extern unsigned char  fn11InterruptRegister;
extern unsigned char  fn11InterruptMask;
extern bool fn11HasGestures;
extern bool fn11HasPinch;
extern bool fn11HasFlick;
extern bool fn11HasTap;
extern bool fn11HasTapAndHold;
extern bool fn11HasDoubleTap;
extern bool fn11HasEarlyTap;
extern bool fn11HasPress;
extern bool fn11HasPalmDetect;

extern unsigned short fn19DataBaseAddr;      /* capacitive buttons */
extern unsigned short fn19ButtonsSupported;
extern unsigned char  fn19InterruptRegister;
extern unsigned char  fn19InterruptMask;

int last_touch=0;

#define BTN_EVENT(x)  (BTN_0 + x)

/**
 * This is the function we pass to the RMI4 subsystem so we can be notified
 * when attention is required.  It may be called in interrupt context.
 */
static void attention(struct rmi_phys_driver *rpd, int instance)
{
  /* All we have to do is wake up the kernel sampling thread. */
  complete(&touch_completion);
}

/**
 * This is the meat of the driver.  It reads in a sample and reports it (if
 * appropriate) to the input subsystem.  It is used for both polling and
 * interrupt driven operation.
 */
int gather_abs_report(struct rmi_application *app)
{
  unsigned char values[2] = {0,0};
  unsigned char data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
  unsigned char interruptStatus[4] = {0,0,0,0};
  int touch = 0; /* number of touch points - fingers or buttons */
  int finger;

  static int X[5] = {0,0,0,0,0}; /* support up to 5 fingers */
  static int Y[5] = {0,0,0,0,0};
  static int Z[5] = {0,0,0,0,0};
  static int W[5] = {0,0,0,0,0};
  static int Button[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; /* up to 12 buttons */

  /* Get the interrupt status from the function $01 data register+1 to find 
     which source was interrupting so we can read the data from that 
     source (2D sensor, buttons, etc.). */
  if(rmi_read_multiple(app, fn01ControlBaseAddr + 1, interruptStatus, interruptRegisterCount)) {
      return 0;
     } 

  /* If 2D sensor interrupted then read the appropriate sensor data */
  if (interruptStatus[fn11InterruptRegister] & fn11InterruptMask)
  {
    
    /* get 2D sensor finger data */
    /* First get the finger status field - the size of the finger status field is 
       determined by the number of finger supporte - 2 bits per finger, so the number
       of registers to read is : registerCount = ciel(numberOfFingers/4).
       Read the required number of registers and check each 2 bit field to determine
       if a finger is down (00 = finger not present, 01 = finger present and data accurate,
       10 = finger present but data may not be accurate, 11 = reserved for product use).
     */
    int fn11FingerRegisters = (fn11FingersSupported + 3)/4;

    if (rmi_read_multiple(app, fn11DataBaseAddr, values, fn11FingerRegisters)) {  
      return 0;
    }

    /* For each finger present, read the proper number of registers to get absolute data,
       relative data (if supported) and gesture data (if supported). By default, only the
       absolute data interrupt is enabled. */
    for (finger = 0; finger < fn11FingersSupported; finger++)
    {
      int reg = finger/4; /* determine which data byte the finger status is in */
      int fingerShift = (finger %4) *2; /* determine bit shift to get that fingers status */
      int fingerStatus = (values[reg] >> fingerShift) & 3;

      if (fingerStatus == 1 || fingerStatus == 2)
      {
        touch++; /* number of active touch points not same as number of supported fingers */

	//printk(KERN_INFO "read finger position at address [0x%x] for [%d] registers\n",  fn11DataBaseAddr + ((finger  * fn11DataRegBlockSize) + fn11FingerRegisters), fn11DataRegBlockSize);

        /* Read the finger data */
        if(rmi_read_multiple(app, fn11DataBaseAddr + 
          ((finger  * fn11DataRegBlockSize) + fn11FingerRegisters), data, fn11DataRegBlockSize))
        {
          return 0;
        } else {
            X[touch - 1] = data[0];
	    X[touch - 1] = (X[touch - 1] << 4) & 0x0FF0;
            X[touch - 1] |= data[2] & 0xf;
            Y[touch - 1] = data[1];
	    Y[touch - 1] = (Y[touch - 1] << 4) & 0x0FF0;
            Y[touch - 1] |= (data[2] >> 4) & 0xf;
            W[touch - 1] = data[3]; /* upper 4 bits are Wy, lower 4 are Wx, but reported together */
            Z[touch - 1] = data[4];
	    if (Z[touch - 1] != 0)
		Z[touch - 1] = 1;
        }
      }
    }

    /* if gestures are supported then read the gesture data registers at the end of the finger data */
    if (fn11HasGestures) {
      int gestureDataRegisterCount = ((fn11HasTap || fn11HasTapAndHold || fn11HasDoubleTap || fn11HasEarlyTap || fn11HasFlick || fn11HasPress || fn11HasPinch) + fn11HasPalmDetect + (fn11HasPinch || fn11HasFlick) + 2*fn11HasFlick);

      //printk (KERN_INFO "read gesture at adress: [0x%x] for [%d] registers\n", fn11DataBaseAddr + ((finger  * fn11DataRegBlockSize) + fn11FingerRegisters), gestureDataRegisterCount);

      if(rmi_read_multiple(app, fn11DataBaseAddr + 
        ((finger  * fn11DataRegBlockSize) + fn11FingerRegisters), data, gestureDataRegisterCount))
      {
        return 0;
      } else {
        /* send events for gestures */
        if (data[0]) {
	  //printk(KERN_INFO "send events for gesture data[0]=[%d] fingers=[%d]\n", data[0], ((data[1] & 0xe0) >> 5) + 1);
          input_report_abs(input, ABS_GESTURE, data[0] & 0x7F); /* send gesture */	  
          input_report_abs(input, ABS_FINGERS, (((data[1] & 0xe0) >> 5) + 1)); /* send number of fingers associated with gesture */
      
          /* for flick or pinch we need to send the flick X and Y distance and flick time, or the pinch motion */
          if (data[0] & 0x10) {
	    int misc_info=0;
	    // For flick motion, misc info is: bits[0:7]=X flick distance, bits[8:15]=Y flick distance, bits[16:24]= flick time
	    misc_info=data[2]; /* X flick distance */
	    misc_info=misc_info<<8;
	    misc_info|=data[3]; /* Y flick distance */
	    misc_info=misc_info<<8;
	    misc_info|=data[4]; /* flick time */
	  
            input_report_abs(input, ABS_MISC, misc_info);
          } else if (data[0] & 0x40) {
	    // For pinch motion, misc info is the pinch motion itself
	    int misc_info=0;
	    misc_info=data[2]; /* pinch motion */
            input_report_abs(input, ABS_MISC, misc_info); /* pinch motion */
          }
        } else if (data[1] & 0x01) {
          input_report_abs(input, ABS_GESTURE, (data[1] & 0x01)); /* palm detect - no number of fingers */
        }
	else {
	  input_report_abs(input, ABS_GESTURE, data[0]);
	}
      }
    }

    /* for each touch point report the X,Y,Z,W as ABS events */
    if (touch) 
    {	
      if(touch > 1) {
        /* Multifinger */
        	
	input_report_abs(input, ABS_FINGERS, touch);

        input_report_abs(input, ABS_X, X[0]);
        input_report_abs(input, ABS_Y, Y[0]);
        input_report_abs(input, ABS_TOOL_WIDTH, W[0]);
        input_report_abs(input, ABS_PRESSURE, Z[0]);

        input_report_abs(input, ABS_FINGER2_X, X[1]);
        input_report_abs(input, ABS_FINGER2_Y, Y[1]);
        input_report_abs(input, ABS_FINGER2_TOOL_WIDTH, W[1]);
        input_report_abs(input, ABS_FINGER2_PRESSURE, Z[1]);

        /* third finger ? */
        if (touch > 2)
        {
          input_report_abs(input, ABS_FINGER3_X, X[2]);
          input_report_abs(input, ABS_FINGER3_Y, Y[2]);
          input_report_abs(input, ABS_FINGER3_TOOL_WIDTH, W[2]);
          input_report_abs(input, ABS_FINGER3_PRESSURE, Z[2]);
        }  
         
        /* fourth finger ? */
        if (touch > 3)
        {
          input_report_abs(input, ABS_FINGER4_X, X[3]);
          input_report_abs(input, ABS_FINGER4_Y, Y[3]);
          input_report_abs(input, ABS_FINGER4_TOOL_WIDTH, W[3]);
          input_report_abs(input, ABS_FINGER4_PRESSURE, Z[3]);
        }
        
        /* fifth finger ? */
        if (touch > 4)
        {
          input_report_abs(input, ABS_FINGER5_X, X[4]);
          input_report_abs(input, ABS_FINGER5_Y, Y[4]);
          input_report_abs(input, ABS_FINGER5_TOOL_WIDTH, W[4]);
          input_report_abs(input, ABS_FINGER5_PRESSURE, Z[4]);
        }
      } else {
	//printk(KERN_INFO "big if touch if, branch else x[0]=[%d] y[0]=[%d] w[0]=[%d] z[0]=[%d]\n", X[0], Y[0], W[0], Z[0]);
        /* Single finger */
        input_report_abs(input, ABS_FINGERS, touch);
        
        input_report_abs(input, ABS_X, X[0]);
        input_report_abs(input, ABS_Y, Y[0]);
        input_report_abs(input, ABS_TOOL_WIDTH, W[0]);
        input_report_abs(input, ABS_PRESSURE, Z[0]);
      }
      input_sync(input); /* sync after groups of events */
    }
    else if ( !touch && last_touch ) {	

	Z[0]=0;

	//printk(KERN_INFO "new else if branch, touch=[%d] x[0]=[%d] y[0]=[%d] w[0]=[%d] z[0]=[%d]\n", touch, X[0], Y[0], W[0], Z[0]);

        /* Single finger */
        input_report_abs(input, ABS_FINGERS, touch);
        
        input_report_abs(input, ABS_X, X[0]);
        input_report_abs(input, ABS_Y, Y[0]);
        input_report_abs(input, ABS_TOOL_WIDTH, W[0]);
        input_report_abs(input, ABS_PRESSURE, Z[0]);
        input_sync(input); /* sync after groups of events */
    }
  }
  else if (interruptStatus[fn19InterruptRegister] & fn19InterruptMask) {
    /* get button data */
    int button;
    /* First get the button status field - the size of the button status field is 
       determined by the number of buttons supported - 1 bit per button, so the number
       of registers to read is : registerCount = ciel(numberOfButtons/8).
       Read the required number of registers and check each 1 bit field to determine
       if a button is down (1 = button down, 0 = button not down).
     */


    int fn19ButtonsRegisters = (fn19ButtonsSupported + 7)/8;
    if (rmi_read_multiple(app, fn19DataBaseAddr, values, fn19ButtonsRegisters)) {
      return 0;
    }

    /* For each button present, read the proper bit in the proper register to get the button data */
    /* and send a key event for the button if pressed or released */
    for (button = 0; button < fn19ButtonsSupported; button++)
    {
      int reg = button/7; /* determine which data byte the button status is in */
      int buttonShift = (button %7); /* determine bit shift to get that buttons status */
      int buttonStatus = (values[reg] >> buttonShift) & 1;

      if (buttonStatus == 1)
      {
        touch++; /* number of active touch points not same as number of supported buttons */

        /* only send a pressed key event if the button was previously unpressed */ 
        if (Button[button] == 0) {
          input_report_key(input, BTN_EVENT(button), 1); /* press */
        }
        Button[button] = 1;

      } else {

        /* only send a released key event if the button was previously pressed */
        if (Button[button] == 1) 
        {
          input_report_key(input, BTN_EVENT(button), 0); /* release */
        }
        Button[button] = 0;

      }
    }

    input_sync(input); /* sync after groups of events */
  }

  last_touch=touch;

  /* return the number of touch points - fingers down or buttons pressed */
  return touch;
}


/**
 * This is a kernel thread that processes packets when we receive them.  It is
 * only used for the interrupt-driven case.  Polling may also be done in this
 * driver and doesn't use this thread (although I suppose it could be modified
 * to do polling, too, instead of using timers).
 */
int rmitouchd(void *param)
{
  struct rmi_application *app = param;

  daemonize("rmitouchd");
	
  while (!kthread_should_stop()) {
    if(time_to_quit) break;
		
    /* wait for interrupt from ATTN line */
    wait_for_completion_interruptible(&touch_completion);
		
    if (time_to_quit) break;
	
    try_to_freeze();
		
    do {
      gather_abs_report(app);
    } while(rmi_get_attn(app));
  }

  complete_and_exit(&thread_comp, 0);
}

/* Head of the list to keep track of who is polling.  This is so we can
 * properly shut down the timers on exit. */
static struct list_head pollers;

/* Simple structure to keep track of things */
struct poll_instance {
  struct delayed_work dw;
  struct rmi_application *app;
  struct list_head link;
};

/* The main routine for the polling case. */
static void poll_work_callback(struct work_struct *data)
{
  struct delayed_work *dw = container_of(data, struct delayed_work, work);
  struct poll_instance *pi = container_of(dw, struct poll_instance, dw);
  static int calls_with_no_data = 0;
  int touch = 0;

  if(time_to_quit) return;

  touch = gather_abs_report(pi->app);

  /* This code implements a backoff.  If we have a call with data being
   * received, we decrease the time between polls to hspolljif.  If
   * there are several calls at that faster rate that do not have data,
   * we go back to slower polling.
   */
  if(touch) calls_with_no_data = 0;
  if(!time_to_quit) {	/* Don't schedule if it's time to quit. */
    if(calls_with_no_data > 5) {
      schedule_delayed_work(dw, polljif);
    } else {
      if(!touch) calls_with_no_data++;
      schedule_delayed_work(dw, hspolljif);
    }
  }
}

/**
 * This is the probe function passed to the RMI4 subsystem that gives us a
 * chance to recognize an RMI4 device.  In this case, we're looking for
 * Synaptics 2D devices.
 */
static int probe(struct rmi_application *app, const struct rmi_module_info *rmi)
{
  struct rmi_function_info *rfi;
  int retval = 0;

  if(!rmi) {
    printk(KERN_ERR "rmi_app_touchpad.probe: "
      "Invalid module info: %p\n", rmi);
    return 0;
  }

  if(rmi->mfgid != 1) { /* Synaptics */
    return 0;
  }

  list_for_each_entry(rfi, &rmi->functions, link) {
    if(rfi->functionNum == 0x11) {
      retval = 1;
      /* We have detected a 2D Sensor. */

      /* if polling required then set up polling call back and worker thread */
      if(0 < rmi_polling_required(app)) {
        struct poll_instance *pi;
        pi = kmalloc(sizeof(*pi), GFP_KERNEL);
        if(!pi) {
          printk(KERN_ERR "rmi_app_touchpad: "
            "Out of memory claiming %s\n",
            rmi->prod_id);
          continue;
        }
        INC_ALLOC_STAT(poll);
        INIT_DELAYED_WORK(&pi->dw, poll_work_callback);
        
        pi->app     = app;
        list_add_tail(&pi->link, &pollers);
        schedule_delayed_work(&pi->dw, polljif);
      } else {
        /* Interrupts enabled in the config stage */
      }
    }
  }

  return retval;
}

static void config(struct rmi_application *app)
{
  /* Check if we had detected a 2D sensor or buttons. */
  unsigned char data[14];
  struct rmi_function_info *rfi;
  struct rmi_phys_driver *rpd;
  struct rmi_module_info *rmi; /* module info has list of functions */

  rpd = app->rpd;
  rmi = &(rpd->rmi);

  list_for_each_entry(rfi, &rmi->functions, link) {
    if(rfi->functionNum == 0x11) {
      /*
       * To Query 2D devices we need to read from the address obtained
       * from the function descriptor stored in the RMI function info.
       */
      if(rmi_read_multiple(app, rfi->funcDescriptor.queryBaseAddr, data, 9)) {
        printk("Could not read function query registers %x\n", rfi->funcDescriptor.queryBaseAddr);
        return;
      } else {

        printk("  Number of Fingers:   %d\n", data[1] & 7);
        printk("  Is Configurable:     %d\n", data[1] & (1 << 7) ? 1 : 0);
        printk("  Has Gestures:        %d\n", data[1] & (1 << 5) ? 1 : 0);
        printk("  Has Absolute:        %d\n", data[1] & (1 << 4) ? 1 : 0);
        printk("  Has Relative:        %d\n", data[1] & (1 << 3) ? 1 : 0);

        printk("  Number X Electrodes: %d\n", data[2] & 0x1f);
        printk("  Number Y Electrodes: %d\n", data[3] & 0x1f);
        printk("  Maximum Electrodes:  %d\n", data[4] & 0x1f);

        printk("  Absolute Data Size:  %d\n", data[5] & 3);

        printk("  Has XY Dist:         %d\n", data[6] & (1 << 7) ? 1 : 0);
        printk("  Has Pinch:           %d\n", data[6] & (1 << 6) ? 1 : 0);
        printk("  Has Press:           %d\n", data[6] & (1 << 5) ? 1 : 0);
        printk("  Has Flick:           %d\n", data[6] & (1 << 4) ? 1 : 0);
        printk("  Has Early Tap:       %d\n", data[6] & (1 << 3) ? 1 : 0);
        printk("  Has Double Tap:      %d\n", data[6] & (1 << 2) ? 1 : 0);
        printk("  Has Tap and Hold:    %d\n", data[6] & (1 << 1) ? 1 : 0);
        printk("  Has Tap:             %d\n", data[6] & 1 ? 1 : 0);
        printk("  Has Palm Detect:     %d\n", data[7] & 1 ? 1 : 0);
 
        if(rmi_read_multiple(app, rfi->funcDescriptor.controlBaseAddr, data, 14)) {
          printk("Could not read function control registers %x\n", rfi->funcDescriptor.controlBaseAddr);
          return;
        }

        printk("  Sensor Max X:  %d\n", ((data[6] & 0x1f) << 8) | ((data[7] & 0xff) << 0));
        printk("  Sensor Max Y:  %d\n", ((data[8] & 0x1f) << 8) | ((data[9] & 0xff) << 0));
      }
    } else if (rfi->functionNum == 0x19) {
      /*
       * Query Buttons RMI function info to get number of  buttons and if configurable.
       */
      if(rmi_read_multiple(app, rfi->funcDescriptor.queryBaseAddr, data, 2)) {
        printk("Could not read function query registers %x\n", rfi->funcDescriptor.queryBaseAddr);
        return;
      } else {
        printk("  Number of Buttons:   %d\n", data[1] & 0x1f);
        printk("  Is Configurable:     %d\n", data[0] & 1);
      }      
    }

    if (!rmi_polling_required(app) && ((rfi->functionNum == 0x11) || (rfi->functionNum == 0x19))) {
      /* Turn on interrupts for this function - for now only the absolute source. */
      rmi_write(app, fn01ControlBaseAddr + 1 + rfi->interruptRegister, rfi->interruptMask);
      printk("  Interrupt Driven - turning on interrupst for function %x\n", rfi->functionNum);
      
      /* If function is $11 (2D sensor) and gestures are supported need to enable the interrupt per gesture */
      if ((rfi->functionNum == 0x11) && fn11HasGestures) {
        unsigned char gestureIntMask1 = 0;
        unsigned char gestureIntMask2 = 0;
        if (fn11HasTap)
          gestureIntMask1 |= 0x01;
        if (fn11HasTapAndHold)
          gestureIntMask1 |= 0x02;
        if (fn11HasDoubleTap)
          gestureIntMask1 |= 0x04;
        if (fn11HasEarlyTap)
          gestureIntMask1 |= 0x08;
        if (fn11HasFlick)
          gestureIntMask1 |= 0x10;
        if (fn11HasPress)
          gestureIntMask1 |= 0x20;
        if (fn11HasPinch)
          gestureIntMask1 |= 0x40;
        if (fn11HasPalmDetect)
          gestureIntMask2 = 0x01;

        if (gestureIntMask1 || gestureIntMask2) {
          rmi_write(app, fn11ControlBaseAddr + 10, gestureIntMask1);
          rmi_write(app, fn11ControlBaseAddr + 11, gestureIntMask2);

          printk("  Interrupt Driven - turning on Enhanced Gesture Interrupts for function %x\n", rfi->functionNum);
        }
      }
    }
  }

  if(!rmi_polling_required(app)) {
    /* We're interrupt driven, so turn on the thread. */
    kthread = kthread_run(&rmitouchd, app, "rmitouchd");

    if(HZ < 500) {
      /* The default polling frequency of 80 times per
       * second is too fast (the Linux time slice for
       * sub-GHz processors is only 100 times per second).
       * So program it to 40.
       */
      rmi_write(app, fn01ControlBaseAddr, (1 << 6));
    }
  }
}

/**
 * The module initialization function in which we register as a RMI
 * application driver.  We also register with the input subsystem so we can
 * pass coordinates to it.
 */
static int __init mod_init(void)
{
  int err;
  printk(KERN_INFO "RMI4 TouchPad Driver\n");

  INIT_LIST_HEAD(&pollers);

  time_to_quit = 0;
  init_completion(&touch_completion);
  init_completion(&thread_comp);

  input = input_allocate_device();
  if (input == NULL) {
    goto exit;
  }

  input->name = "RMI4 Touchpad";
  input->phys = "rmi_app_touchpad";
  input->evbit[0] = BIT(EV_ABS);
  input_set_abs_params(input, ABS_X, 0, 8191, 0, 0);
  input_set_abs_params(input, ABS_Y, 0, 8191, 0, 0);
  input_set_abs_params(input, ABS_PRESSURE, 0, 255, 0, 0);
  input_set_abs_params(input, ABS_TOOL_WIDTH, 0, 15, 0, 0);
  input_set_abs_params(input, ABS_FINGERS, 0, 3, 0, 0);
  input_set_abs_params(input, ABS_FINGER2_X, 0, 8191, 0, 0);
  input_set_abs_params(input, ABS_FINGER3_X, 0, 8191, 0, 0);
  input_set_abs_params(input, ABS_FINGER4_X, 0, 8191, 0, 0);
  input_set_abs_params(input, ABS_FINGER5_X, 0, 8191, 0, 0);
  input_set_abs_params(input, ABS_FINGER2_Y, 0, 8191, 0, 0);
  input_set_abs_params(input, ABS_FINGER3_Y, 0, 8191, 0, 0);
  input_set_abs_params(input, ABS_FINGER4_Y, 0, 8191, 0, 0);
  input_set_abs_params(input, ABS_FINGER5_Y, 0, 8191, 0, 0);
  input_set_abs_params(input, ABS_FINGER2_PRESSURE, 0, 255, 0, 0);
  input_set_abs_params(input, ABS_FINGER2_TOOL_WIDTH, 0, 15, 0, 0);
  input_set_abs_params(input, ABS_FINGER3_PRESSURE, 0, 255, 0, 0);
  input_set_abs_params(input, ABS_FINGER3_TOOL_WIDTH, 0, 15, 0, 0);
  input_set_abs_params(input, ABS_FINGER4_PRESSURE, 0, 255, 0, 0);
  input_set_abs_params(input, ABS_FINGER4_TOOL_WIDTH, 0, 15, 0, 0);
  input_set_abs_params(input, ABS_FINGER5_PRESSURE, 0, 255, 0, 0);
  input_set_abs_params(input, ABS_FINGER5_TOOL_WIDTH, 0, 15, 0, 0);
  input_set_abs_params(input, ABS_MISC, 0, 8191, 0, 0);
  input_set_abs_params(input, ABS_GESTURE, 0, 128, 0, 0);

  if ((err = input_register_device(input)))
	return err;

  app = rmi_register_application("rmi_touchpad", attention, probe, config);

  if(!app) {
    return -ENODEV;
  }

  return 0;
exit:
  rmi_unregister_application(app);
  return -ENOMEM;
}

/**
 * The module exit function.
 */
static void __exit mod_exit(void)
{
  struct poll_instance *pi, *pi_tmp;
  time_to_quit = 1;

  complete(&touch_completion); /* Kick the thread awake */
  list_for_each_entry(pi, &pollers, link) {
  cancel_delayed_work(&pi->dw);	/* Cancel any pending polls */
  }
  flush_scheduled_work();	/* Make sure all pollers are stopped */
  if(kthread) wait_for_completion(&thread_comp);

  /* Unregister everything */
  rmi_unregister_application(app);
  input_unregister_device(input);

  /* Free up the structures */
  list_for_each_entry_safe(pi, pi_tmp, &pollers, link) {
    list_del(&pi->link);
    kfree(pi);
    DEC_ALLOC_STAT(poll);
  }

  CHECK_ALLOC_STAT(poll);
}

/** Standard driver module information - the author of the module.
 */
MODULE_AUTHOR("Synaptics, Inc.");
/** Standard driver module information - a summary description of this module.
 */
MODULE_DESCRIPTION("RMI4 Touchpad Application Driver");
/** Standard driver module information - the license under which this module
 * is included in the kernel.
 */
MODULE_LICENSE("");

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

/* vim600: set noexpandtab sw=8 ts=8 :*/
