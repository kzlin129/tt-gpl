/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/




/*=============================================================================

Project  :  vcfw
Module   :  chip driver
File     :  $RCSfile: driver.h,v $
Revision :  $Revision: 1.6 $

FILE DESCRIPTION
xxx
=============================================================================*/

#ifndef DRIVER_H_
#define DRIVER_H_

#include <linux/broadcom/vc03/vcos.h>

/******************************************************************************
 Global Macros
 *****************************************************************************/

// build the driver function table names through macro magic
#define DRIVER_CONCATENATE(a,b) a##b

//concat the driver name and the _get_func_table extension
#define DRIVER_NAME(x) DRIVER_CONCATENATE( x, _get_func_table )

/******************************************************************************
 Global Defines
 *****************************************************************************/

typedef enum
{
   DRIVER_FLAGS_SUPPORTS_CORE_FREQ_CHANGE = 0x1,
   DRIVER_FLAGS_SUPPORTS_RUN_DOMAIN_CHANGE = 0x2

} DRIVER_FLAGS_T;

/******************************************************************************
 Function defines
 *****************************************************************************/

//Generic handle passed used by all drivers
typedef struct opaque_driver_handle_t *DRIVER_HANDLE_T;

//Routine to initialise a driver
typedef int32_t (*DRIVER_INIT_T)( void );

//Routine to shutdown a driver
typedef int32_t (*DRIVER_EXIT_T)( void );


//Routine to return a drivers info (name, version etc..)
typedef int32_t (*DRIVER_INFO_T)(const char **driver_name,
                                 uint32_t *version_major,
                                 uint32_t *version_minor,
                                 DRIVER_FLAGS_T *flags );

//Routine to open a driver.
typedef int32_t (*DRIVER_OPEN_T)(const void *params,
                                 DRIVER_HANDLE_T *handle );

//Routine to close a driver
typedef int32_t (*DRIVER_CLOSE_T)( const DRIVER_HANDLE_T handle );


//Optional routine to handle clock change messages
typedef int32_t (*DRIVER_CORE_FREQ_CHANGE_T)(const uint32_t core_freq_in_hz,
                                             const uint32_t pending );

//Optional routine to handle power domain requests
typedef int32_t (*DRIVER_RUN_DOMAIN_CHANGE_T)(  const uint32_t new_run_domain_active_state,
                                                const uint32_t pending );

/******************************************************************************
 Driver struct definition
 *****************************************************************************/

/* Basic driver structure, as implemented by all device drivers */
#define COMMON_DRIVER_API(param1) \
   /*Used to be param1... but no drivers were using multiple params and MSVC doesn't like vararg macros*/ \
   /*Function to initialize the driver. This is used at the start of day to //initialize the driver*/ \
   DRIVER_INIT_T   init; \
   \
   /*Routine to shutdown a driver*/ \
   DRIVER_EXIT_T   exit; \
   \
   /*Function to get the driver name + version*/ \
   DRIVER_INFO_T   info; \
   \
   /*Function to open an instance of the driver. Takes in option parameters, //defined per driver.*/ \
   /*Returns a handle to the open driver and a success code*/ \
   int32_t (*open)(param1, \
                   DRIVER_HANDLE_T *handle ); \
   \
   /*Function to close a driver instance*/ \
   /*Returns success code*/ \
   DRIVER_CLOSE_T   close;


/* Extended driver structure - optional per driver */
#define COMMON_DRIVER_EXT_API \
   /*Function used to tell a driver that the core freq is about to change*/ \
   /*Returns success code (0 if its ok to change the clock)*/ \
   DRIVER_CORE_FREQ_CHANGE_T   core_freq_change;\
   \
   /*Function used to tell a driver if a power domain is about to change*/ \
   /*Returns success code (0 if its ok to change the power domains)*/ \
   DRIVER_RUN_DOMAIN_CHANGE_T   run_domain_change;


typedef struct
{
   //just include the basic driver api
   COMMON_DRIVER_API(void const *unused)

} DRIVER_T;


typedef struct
{
   //just include the basic driver api
   COMMON_DRIVER_API(void const *unused)

   //also include the extended api
   COMMON_DRIVER_EXT_API

} DRIVER_EXT_T;


#endif // DRIVER_H_
