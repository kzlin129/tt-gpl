/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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




/****************************************************************************/
/**
*  @file    rtcHw.h
*
*  @brief   API definitions for low level RTC & BBL driver
*
*/
/****************************************************************************/
#ifndef _RTCHW_H
#define _RTCHW_H

#include <linux/rtc.h>

/****************************************************************************/
/****************************************************************************
*  NOTE: Since RTC block is programmed using one address and one data
*         register, simultaneous access to these registers using the
*         following APIs are not safe. 
*
*         Proper protection is required before calling these functions.
*/
/****************************************************************************/
typedef uint32_t rtcHw_TIME_t;              /* Time in second */

typedef enum
{
  rtcHw_INTERVAL_125ms     = 0x00000001,    /* Time interval 125ms */
  rtcHw_INTERVAL_250ms     = 0x00000002,    /* Time interval 250ms */
  rtcHw_INTERVAL_500ms     = 0x00000004,    /* Time interval 500ms */
  rtcHw_INTERVAL_1000ms    = 0x00000008,    /* Time interval 1 sec */
  rtcHw_INTERVAL_2000ms    = 0x00000010,    /* Time interval 2 sec */
  rtcHw_INTERVAL_4000ms    = 0x00000020,    /* Time interval 4 sec */
  rtcHw_INTERVAL_8000ms    = 0x00000040,    /* Time interval 8 sec */
  rtcHw_INTERVAL_16000ms   = 0x00000080,    /* Time interval 16 sec */
  rtcHw_INTERVAL_32000ms   = 0x00000100,    /* Time interval 32 sec */
  rtcHw_INTERVAL_64000ms   = 0x00000200,    /* Time interval 64 sec */
  rtcHw_INTERVAL_128000ms  = 0x00000400     /* Time interval 128 sec */
}rtcHw_INTERVAL_e;


typedef enum
{
  rtcHw_INTERRUPT_STATUS_NONE,              /* No interrupt occured */
  rtcHw_INTERRUPT_STATUS_TAMPER,            /* Tamper interrupt occured */
  rtcHw_INTERRUPT_STATUS_ONESHOT,           /* Oneshot interrupt occured */
  rtcHw_INTERRUPT_STATUS_PERIODIC           /* Periodic interrupt occured */
}rtcHw_INTERRUPT_STATUS_e;

typedef enum {e_irq_one_shot, e_irq_periodic} e_rtc_hw_irq_type;

/****************************************************************************/
/**
*  @brief   A common function to read bbl register
*
*/
/****************************************************************************/
uint32_t rtcHw_readReg ( uint32_t regAddr );

/****************************************************************************/
/**
*  @brief   A common function to write bbl register
*
*/
/****************************************************************************/
void rtcHw_writeReg ( uint32_t regAddr, uint32_t val );

/****************************************************************************/
/**
*  @brief   Starts the RTC timer
* 
*  This function starts the RTC timer
*/
/****************************************************************************/
void rtcHw_startTimer ( void );



/****************************************************************************/
/**
*  @brief   Stops the RTC timer
* 
*  This function stops the RTC timer
*/
/****************************************************************************/
void rtcHw_stopTimer ( void );


/****************************************************************************/
/**
*  @brief   Gets the current time counts in seconds
* 
*  This function gets the current time counts
*
*  @return  Current time count since it started
*/
/****************************************************************************/
rtcHw_TIME_t rtcHw_getTime ( void );



/****************************************************************************/
/**
*  @brief   Sets the current time counts in seconds
* 
*  This function sets the current time counts
*
*/
/****************************************************************************/
void rtcHw_setTime ( rtcHw_TIME_t time );




/****************************************************************************/
/**
*  @brief   Configures a periodic timer to generate timer interrupt after 
*           certain time interval
* 
*  This function initializes a periodic timer to generate timer interrupt
*  after every time interval.
*
*  @return   On success: Effective interval set in mili-second (resolution 125ms)
*            On failure: 0
*
*/
/****************************************************************************/
void rtcHw_setPeriodicTimerInterval
( 
   rtcHw_INTERVAL_e   interval       //<  [ IN ] Periodic time intervals 
);



/****************************************************************************/
/**
*  @brief   Start the periodic timer
*
*  This function starts the periodic timer
*
*/
/****************************************************************************/
void rtcHw_startPeriodicTimer ( void );



/****************************************************************************/
/**
*  @brief   Stops the periodic timer
* 
*  This function stops the periodic timer
*
*/
/****************************************************************************/
void rtcHw_stopPeriodicTimer ( void );



/****************************************************************************/
/**
*  @brief   Clears the periodic timer interrupt
* 
*  This function clears the periodic timer interrupt
*
*/
/****************************************************************************/
void rtcHw_clearPeriodicTimerInterrupt ( void );


/****************************************************************************/
/**
*  @brief   Clears the tamper interrupt
* 
*  This function clears the tamper interrupt
*
*/
/****************************************************************************/
void rtcHw_clearTamperInterrupt ( void );



/****************************************************************************/
/**
*  @brief   Get interrupt status
* 
*  This function gets interrupt status
*
*  @return rtcHw_INTERRUPT_STATUS_NONE      : On no interrupt
*
*          rtcHw_INTERRUPT_STATUS_TAMPER    : On interrupt
*          rtcHw_INTERRUPT_STATUS_PERIODIC
*          rtcHw_INTERRUPT_STATUS_ONESHOT
*/
/****************************************************************************/
rtcHw_INTERRUPT_STATUS_e rtcHw_getInterruptStatus ( void );


/****************************************************************************/
/**
*  @brief   Enables the tamper interrupt 
* 
*  This function enables the tamper interrupt 
*
*/
/****************************************************************************/
void rtcHw_enableTamperInterrupt ( void );


/****************************************************************************/
/**
*  @brief   Disables the tamper interrupt 
* 
*  This function disables the tamper interrupt 
*
*/
/****************************************************************************/
void rtcHw_disableTamperInterrupt ( void );


/****************************************************************************/
/**
*  @brief   Configures the real time timer to generate an oneshot interrupt
* 
*  This function initializes the timer to generate one shot interrupt
*  after certain time period in second
*
*  @return   On success: Effective period set in seconds
*            On failure: 0
*
*/
/****************************************************************************/
rtcHw_TIME_t rtcHw_setOneshotTimer
( 
   rtcHw_TIME_t   sec               //<  [ IN ] Period in second 
);


/****************************************************************************/
/**
*  @brief   Enable oneshot interrupt
* 
*/
/****************************************************************************/
void rtcHw_enableOneshotInt ( void );


/****************************************************************************/
/**
*  @brief   Disable oneshot interrupt
* 
*/
/****************************************************************************/
void rtcHw_disableOneshotInt ( void );

/****************************************************************************/
/**
*  @brief   Gets the real time timer to generate an oneshot interrupt
* 
*  This function gets the timer to generate one shot interrupt
*  after certain time period in second
*
*  @return   timer value
*
*/
/****************************************************************************/
rtcHw_TIME_t rtcHw_getOneshotTimer ( void );

/****************************************************************************/
/**
*  @brief   Clears the one shot timer interrupt
* 
*  This function clears the one shot timer interrupt
*
*/
/****************************************************************************/
void rtcHw_clearOneshotTimerInterrupt ( void );


/****************************************************************************/
/**
*  @brief   Check if oneshot interrupt is enabled/pending
* 
*  This function check status of oneshot interrupt enable/pending
*
*/
/****************************************************************************/
int rtcHw_isOneshotEnabled ( void );


/****************************************************************************/
/**
*  @brief   Read from BBL RAM
* 
*  This function gets contents from the BBL RAM
*
*  @return  Data from the specified RAM location
*/
/****************************************************************************/
unsigned char rtcHw_bblRamRead 
( 
   uint32_t address     ///< BBL Ram address from 0x00 to 0xFF
);


/****************************************************************************/
/**
*  @brief   Write into BBL RAM
* 
*  This function writes into the BBL RAM
*
*  @return  0 : on success
*          -1 : on failure
*/
/****************************************************************************/
int rtcHw_bblRamWrite
( 
   uint32_t address,       ///< BBL Ram address from 0x00 to 0xFF
   unsigned char value     ///< Data to be written at the address
);

#endif /* _RTCHW_H */








