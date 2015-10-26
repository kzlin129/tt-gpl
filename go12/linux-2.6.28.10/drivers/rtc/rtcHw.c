/*****************************************************************************
* Copyright 2003 - 2008 Broadcom Corporation.  All rights reserved.
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

/* ---- Include Files ----------------------------------------------------- */

#include <asm/arch/rtcHw.h>
#include <asm/arch/rtcHw_reg.h>
#include <asm/arch/rtc_cpuapi4760.h>

/* ---- Private Constants and Types ---------------------------------------- */
#define rtcHw_RTC_STOP                0     /* Stop RTC */
#define rtcHw_RTC_START               1     /* Start RTC */
#define rtcHw_RTC_INT_ENABLE          3     /* Enable RTC timer interrupt */
#define rtcHw_RTC_INT_DISABLE         4     /* Disable RTC timer interrupt */
#define rtcHw_PER_INT_ENABLE          5     /* Enable RTC periodic timer interrupt */
#define rtcHw_PER_INT_DISABLE         6     /* Disable RTC periodic timer interrupt */

#define RTC_POLL_FOR_READY_TIMEOUT 1			// Timeout in HZ, 1/HZ of a second in this case

/* ---- Public Constants and Types ----------------------------------------- */
/* ---- Public Variable Externs -------------------------------------------- */

/* ---- Private Function Prototypes ----------------------------------------- */

int32_t readyForCommand ( unsigned long delay )
{
   unsigned long timeout;
	 
  //while ( ! rtchw_REG_IS_COMMAND_COMPLETE() ) udelay ( 1 );
   // Wait until RTC is not busy
   timeout = jiffies + (delay*HZ);
   while ( ! rtchw_REG_IS_COMMAND_COMPLETE() )
   {
      if (time_after(jiffies, timeout))
         return 1;
   }
   return 0;
}

/* ---- Public Function Prototypes ----------------------------------------- */


/****************************************************************************/
/**
*  @brief   A common function to read bbl register
*
*/
/****************************************************************************/
uint32_t rtcHw_readReg ( uint32_t regAddr )
{
   uint32_t val;

   /* Make sure RTC is ready to accept commands */
   readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
   rtchw_REG_SET_RTC_ADDRESS ( regAddr );
   rtchw_REG_COMMAND_READ();
   /* Wait until command is processed */
   //while ( ! rtchw_REG_IS_COMMAND_COMPLETE() ) udelay ( 1 );
	 readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);
   /* Read the data */
   val = rtchw_REG_GET_DATA ();
   return val;
}
/****************************************************************************/
/**
*  @brief   A common function to write bbl register
*
*/
/****************************************************************************/
void rtcHw_writeReg ( uint32_t regAddr, uint32_t val )
{
   /* Make sure RTC is ready to accept commands */
   readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
   rtchw_REG_SET_RTC_ADDRESS ( regAddr );
   rtchw_REG_SET_DATA ( val );
   rtchw_REG_COMMAND_WRITE ();
}

/****************************************************************************/
/**
*  @brief   A common function to update BBL control register 
* 
*/
/****************************************************************************/
void updateBblControl ( uint32_t command )
{
   uint32_t val=0;

   /* Make sure RTC is ready to accept commands */
   readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);

   switch ( command )
   {
	 
      case rtcHw_RTC_STOP:
				rtchw_REG_SET_RTC_ADDRESS ( rtchw_RSRST_ADDR );
				rtchw_REG_COMMAND_READ();
				/* Wait until command is processed */
				readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);
				/* Read the data */
				val = rtchw_REG_GET_DATA ();
	 /* Set the RTC stop bit */
				val |= rtchw_CMD_BBL_RTC_STOP;
        break;
      case rtcHw_RTC_START:
				rtchw_REG_SET_RTC_ADDRESS ( rtchw_RSRST_ADDR );
				rtchw_REG_COMMAND_READ();
				/* Wait until command is processed */
				readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);
				/* Read the data */
				val = rtchw_REG_GET_DATA ();
         /* Clear RTC stop bit */
				val &= ~ rtchw_CMD_BBL_RTC_STOP;
        break;
				 
      case rtcHw_RTC_INT_ENABLE:
				rtchw_REG_SET_RTC_ADDRESS ( rtchw_CONTROL_ADDR );
				rtchw_REG_COMMAND_READ();
				/* Wait until command is processed */
				readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);
				/* Read the data */
				val = rtchw_REG_GET_DATA ();
         /* Set RTC interrupt enable bit */
         val |= rtchw_CMD_RTC_INT_ENABLE;
         break;
      case rtcHw_RTC_INT_DISABLE:
				rtchw_REG_SET_RTC_ADDRESS ( rtchw_CONTROL_ADDR );
				rtchw_REG_COMMAND_READ();
				/* Wait until command is processed */
				readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);
				/* Read the data */
				val = rtchw_REG_GET_DATA ();
         /* Clear RTC interrupt enable bit */
         val &= ~ rtchw_CMD_RTC_INT_ENABLE;
         break;
      case rtcHw_PER_INT_ENABLE:
				rtchw_REG_SET_RTC_ADDRESS ( rtchw_CONTROL_ADDR );
				rtchw_REG_COMMAND_READ();
				/* Wait until command is processed */
				readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);
				/* Read the data */
				val = rtchw_REG_GET_DATA ();
         /* Set Periodic interrupt enable bit */
         val |= rtchw_CMD_PER_INT_ENABLE;          
         break;
      case rtcHw_PER_INT_DISABLE:
				rtchw_REG_SET_RTC_ADDRESS ( rtchw_CONTROL_ADDR );
				rtchw_REG_COMMAND_READ();
				/* Wait until command is processed */
				readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);
				/* Read the data */
				val = rtchw_REG_GET_DATA ();
         /* Clear Periodic interrupt enable bit */
         val &= ~ rtchw_CMD_PER_INT_ENABLE;          
         break;
   }
   /* Set the updated value into the control register */
   rtchw_REG_SET_DATA ( val );
   rtchw_REG_COMMAND_WRITE ();
}


/****************************************************************************/
/**
*  @brief   Starts running the timer
* 
*  This function starts the timer
*/
/****************************************************************************/
void rtcHw_startTimer ( void )
{
   updateBblControl ( rtcHw_RTC_START );
}


/****************************************************************************/
/**
*  @brief   Stops the RTC timer
* 
*  This function stops the RTC timer
*/
/****************************************************************************/
void rtcHw_stopTimer ( void )
{
   updateBblControl ( rtcHw_RTC_STOP );
}


/****************************************************************************/
/**
*  @brief   Gets the current time counts in seconds
* 
*  This function gets the current time counts
*
*  @return  Current time count since it started
*/
/****************************************************************************/
rtcHw_TIME_t rtcHw_getTime ( void )
{
   /* Make sure RTC is ready to accept commands */
   readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
   rtchw_REG_SET_RTC_ADDRESS ( rtchw_CURRENT_TIME_ADDR );
   rtchw_REG_COMMAND_READ();
   /* Wait until command is processed */
   //while ( ! rtchw_REG_IS_COMMAND_COMPLETE() ) udelay ( 1 );
	 	 readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);
   /* Read the data */
   return rtchw_REG_GET_DATA ();
}

/****************************************************************************/
/**
*  @brief   Sets the current time counts in seconds
* 
*  This function sets the current time counts
*
*/
/****************************************************************************/
void rtcHw_setTime ( rtcHw_TIME_t time )
{
   /* Make sure RTC is ready to accept commands */
   readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
   rtchw_REG_SET_RTC_ADDRESS ( rtchw_RTC_REGISTER_ADDR );
   rtchw_REG_SET_DATA ( time );
   rtchw_REG_COMMAND_WRITE ();
   /* Wait until command is processed */
   //while ( ! rtchw_REG_IS_COMMAND_COMPLETE() ) udelay ( 1 );
	 	 readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);

}


/****************************************************************************/
/**
*  @brief   Configures a periodic timer to generate timer interrupt after 
*           certain time interval
* 
*  This function initializes a periodic timer to generate timer interrupt
*  after every time interval.
*
*/
/****************************************************************************/
void rtcHw_setPeriodicTimerInterval ( rtcHw_INTERVAL_e interval )
{
   /* Make sure RTC is ready to accept commands */
   readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
   rtchw_REG_SET_RTC_ADDRESS ( rtchw_PERIODIC_TIMER_ADDR );
   rtchw_REG_SET_DATA ( interval );
   rtchw_REG_COMMAND_WRITE ();
}


/****************************************************************************/
/**
*  @brief   Configures a periodic timer to generate timer interrupt after
*           certain time interval
*
*  This function initializes a periodic timer to generate timer interrupt
*  after every time interval.
*
*/
/****************************************************************************/
void rtcHw_startPeriodicTimer ( void )
{
   /* Clear any pending interrupt */
   rtcHw_clearPeriodicTimerInterrupt (  );
   /* Enable periodic timer interrupt */
   updateBblControl ( rtcHw_PER_INT_ENABLE );
}


/****************************************************************************/
/**
*  @brief   Stops the periodic timer
* 
*  This function stops the periodic timer
*
*/
/****************************************************************************/
void rtcHw_stopPeriodicTimer ( void )
{
   updateBblControl ( rtcHw_PER_INT_DISABLE );
}



/****************************************************************************/
/**
*  @brief   Clears the periodic timer interrupt
* 
*  This function clears the periodic timer interrupt
*
*/
/****************************************************************************/
void rtcHw_clearPeriodicTimerInterrupt ( void )
{
   /* Make sure RTC is ready to accept commands */
   readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
   rtchw_REG_SET_RTC_ADDRESS ( rtchw_CLEAR_INTR_ADDR );
   /* Set periodic clear interrupt event */
   rtchw_REG_SET_DATA ( rtchw_CMD_PER_INTR_CLEAR );
   rtchw_REG_COMMAND_WRITE ();
}

#if (0)
/****************************************************************************/
/**
*  @brief   Clears the tamper interrupt
* 
*  This function clears the tamper interrupt
*
*/
/****************************************************************************/
void rtcHw_clearTamperInterrupt ( void )
{
   if ( pRtcHw->tamper & rtchw_REG_TAMPER_INTERRUPT_STATUS )
   {
      pRtcHw->tamper |= rtchw_REG_TAMPER_INTERRUPT_STATUS;
   }
}

/****************************************************************************/
/**
*  @brief   Enables the tamper interrupt 
* 
*  This function enables the tamper interrupt 
*
*/
/****************************************************************************/
void rtcHw_enableTamperInterrupt ( void )
{
   pRtcHw->tamper |= rtchw_REG_TAMPER_INTERRUPT_ENABLE;
}


/****************************************************************************/
/**
*  @brief   Disables the tamper interrupt 
* 
*  This function disables the tamper interrupt 
*
*/
/****************************************************************************/
void rtcHw_disableTamperInterrupt ( void )
{
   pRtcHw->tamper &= ~rtchw_REG_TAMPER_INTERRUPT_ENABLE;
}

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
rtcHw_INTERRUPT_STATUS_e rtcHw_getInterruptStatus ( void )
{
   uint32_t status;
   uint32_t enable;   

   if ( pRtcHw->tamper & rtchw_REG_TAMPER_INTERRUPT_STATUS )
   {
      return rtcHw_INTERRUPT_STATUS_TAMPER;
   }

   /* Read the interrupt status */
   /* Make sure RTC is ready to accept commands */
   readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
   rtchw_REG_SET_RTC_ADDRESS ( rtchw_INTERRUPT_STATUS_ADDR );
   rtchw_REG_COMMAND_READ();
   /* Wait until command is processed */
   //while ( ! rtchw_REG_IS_COMMAND_COMPLETE() ) udelay ( 1 );
	 readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);

   /* Read the data */
   status = rtchw_REG_GET_DATA ();
   

   /* Read the interrupt enable bits */
   /* Make sure RTC is ready to accept commands */
   readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
   rtchw_REG_SET_RTC_ADDRESS ( rtchw_CONTROL_ADDR );
   rtchw_REG_COMMAND_READ();
   /* Wait until command is processed */
   //while ( ! rtchw_REG_IS_COMMAND_COMPLETE() ) udelay ( 1 );
	 	 readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);

   /* Read the data */
   enable = rtchw_REG_GET_DATA ();

   /* Identify the cause of interrupt */
   if ( ( status & rtchw_CMD_PERIODIC_INTERRUPT_STATUS ) && 
        ( enable & rtchw_CMD_PER_INT_ENABLE )
        )
   {
      return rtcHw_INTERRUPT_STATUS_PERIODIC;
   }
   else if ( ( status & rtchw_CMD_ONESHOT_INTERRUPT_STATUS ) &&
             ( enable & rtchw_CMD_RTC_INT_ENABLE )
             )
   {
      return rtcHw_INTERRUPT_STATUS_ONESHOT;
   }

   return rtcHw_INTERRUPT_STATUS_NONE;
}
#endif

/****************************************************************************/
/**
*  @brief   Configures the real time timer to generate an oneshot interrupt
* 
*  This function initializes the timer to generate one shot interrupt
*  after certain time period in seconds
*
*  @return   On success: Effective period set in seconds
*            On failure: 0
*
*/
/****************************************************************************/
rtcHw_TIME_t rtcHw_setOneshotTimer ( rtcHw_TIME_t sec )
{
   /* Clear any pending interrupt */
   rtcHw_clearOneshotTimerInterrupt (  );
   /* Make sure RTC is ready to accept commands */
   readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
   rtchw_REG_SET_RTC_ADDRESS ( rtchw_MATCH_REGISTER_ADDR );

   rtchw_REG_SET_DATA ( sec & 0xffff );
   rtchw_REG_COMMAND_WRITE ();
   return sec;
}

/****************************************************************************/
/**
*  @brief   Enable oneshot interrupt
* 
*/
/****************************************************************************/
void rtcHw_enableOneshotInt ( void )
{
   /* Enable RTC timer interrupt */
   updateBblControl ( rtcHw_RTC_INT_ENABLE );
}


/****************************************************************************/
/**
*  @brief   Disable oneshot interrupt
* 
*/
/****************************************************************************/
void rtcHw_disableOneshotInt ( void )
{
   /* Disable RTC timer interrupt */
   updateBblControl ( rtcHw_RTC_INT_DISABLE );
}

/****************************************************************************/
/**
*  @brief   Check if oneshot interrupt is enabled/pending
*
*/
/****************************************************************************/
int rtcHw_isOneshotEnabled ( void )
{
   uint32_t ctrlreg = rtcHw_readReg ( rtchw_CONTROL_ADDR );

   return (ctrlreg & rtchw_CMD_RTC_INT_ENABLE) ? 1 : 0;
}

/****************************************************************************/
/**
*  @brief   Gets the real time timer to generate an oneshot interrupt
* 
*  This function initializes the timer to generate one shot interrupt
*  after certain time period in seconds
*
*  @return   On success: Effective period set in seconds
*            On failure: 0
*
*/
/****************************************************************************/
rtcHw_TIME_t rtcHw_getOneshotTimer ( void )
{
   /* Make sure RTC is ready to accept commands */
   readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
   rtchw_REG_SET_RTC_ADDRESS ( rtchw_MATCH_REGISTER_ADDR );

   rtchw_REG_COMMAND_READ();
   /* Wait until command is processed */
   //while ( ! rtchw_REG_IS_COMMAND_COMPLETE() ) udelay ( 1 );
	 readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);

   /* Read the data */
   return rtchw_REG_GET_DATA ();
}

/****************************************************************************/
/**
*  @brief   Clears the one shot timer interrupt
* 
*  This function clears the one shot timer interrupt
*
*/
/****************************************************************************/
void rtcHw_clearOneshotTimerInterrupt ( void )
{
   /* Make sure RTC is ready to accept commands */
   readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
   rtchw_REG_SET_RTC_ADDRESS ( rtchw_CLEAR_INTR_ADDR );
   /* Set RTC clear interrupt event */
   rtchw_REG_SET_DATA ( rtchw_CMD_RTC_INTR_CLEAR );
   rtchw_REG_COMMAND_WRITE ();
}

#if (0)
/****************************************************************************/
/**
*  @brief   Read from BBL RAM
* 
*  This function gets contents from the BBL RAM
*
*  @return  Data from the specified RAM location
*/
/****************************************************************************/
unsigned char rtcHw_bblRamRead ( uint32_t address )
{
   if ( address <= 0x000000FF )
   {
      /* Make sure RTC is ready to accept commands */
      readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
      rtchw_REG_SET_BBL_ADDRESS ( address );
      rtchw_REG_COMMAND_READ();
      /* Wait until command is processed */
      //while ( ! rtchw_REG_IS_COMMAND_COMPLETE() ) udelay ( 1 );
			readyForCommand(RTC_POLL_FOR_READY_TIMEOUT);

      /* Read the data */
      return (unsigned char) rtchw_REG_GET_DATA ();
   }
   else
   {
      return 0;
   }
}

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
int rtcHw_bblRamWrite ( uint32_t address, unsigned char value )
{
   if ( address <= 0x000000FF )
   {
      rtchw_REG_SET_DATA ( value );
      /* Make sure RTC is ready to accept commands */
      readyForCommand (RTC_POLL_FOR_READY_TIMEOUT);
      rtchw_REG_SET_BBL_ADDRESS ( address );
      rtchw_REG_COMMAND_WRITE ();
      return 0;
   }
   else
   {
      return -1;
   }
}

#endif

