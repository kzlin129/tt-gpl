/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
/** 
 * 
 *   @file   tahoe_stat.h 
 * 
 *   @brief  Error codes.
 * 
 ****************************************************************************/

#ifndef _BCM_OPMODE_STAT_H_
#define _BCM_OPMODE_STAT_H_

/**
     @enum  tahoe_stat_t

     @ingroup TAHOE_OPMODEIFC

     Opmode return codes, this list will grow as development progresses
     Using a different naming convention for these enums to differentiate.
 */
typedef enum _tahoe_stat_t
{
	/* NOTE: This enum list must be kept in sync with the stat_str_array defined
	 * in tahoe_stat_str.c file 
	 */
   OpStatFirst               = 0x0000,
   OpOk                      = OpStatFirst,     /* opmode API successful                     */
   OpBadParam                = 0x0001,          /* No valid op can be derived from inputs    */
   OpUnknownErr              = 0x0002,          /* Unknown error                             */
   OpPllEnableFailed         = 0x0003,          /* Failed to enable a PLL                    */
   OpPllSetFreqFailed        = 0x0004,          /* Failed to set PLL frequency               */
   OpCrystalEnableFailed     = 0x0005,          /* Failed to enable the crystal              */
   OpClkSrcEnableFailed      = 0x0006,          /* Failed to enable the clock source         */
   OpNoSuitableClkSrc        = 0x0007,          /* No suitable clock source                  */
   OpClkSrcCannotBchanged    = 0x0008,          /* Preallocated clock source is in use and cannot be changed      */

   OpPeripConfigFailed       = 0x0009,          /* Peripheral config failed                  */
   OpArm11ClkSwitchFailed    = 0x000A,          /* Switching ARM11 frequency failed          */
   OpArm11SetClkFailed       = 0x000B,          /* Setting ARM11 frequency failed            */
   OpVcSysGetFreqFailed      = 0x000C,          /* Getting VCSYS [matrix-vc-ahb] frequency failed                 */
   OpClkSwitchFailed    = 0x000D,          /* Switching ARM11 frequency failed          */
   OpVcSysSetFreqFailed      = 0x000E,          /* Setting VCSYS [matrix-vc-ahb] frequency failed                 */

   OpDesiredFreqTooHigh      = 0x000F,          /* Desired Frequency is too high             */
   OpDesiredFreqTooLow       = 0x0010,          /* Desired Frequency is too Low              */

   OpClkSrcDisableFailed     = 0x0011,          /* Failed to enable the clock source         */
   OpPeripSetClkFailed       = 0x0012,          /* Setting peripheral frequency failed            */
   OpEmiAsyncModeNotSetup    = 0x0013,		    /* During clock switch to Async Mode, 
                                                   the CMDASYNC register needs to be setup first*/
   OpInvalidClockRatios	     = 0x0014,	        /* Clock ratios are invalid (for synchronizer) */
   OpInvalidClockMode        = 0x0015,          /* Synchronizer clock mode restrictions        */

   OpInvalidChipMgrPllConfig = 0x0016,		    /* chipmgr pll config is invalid */
   OpDramInvalidMode		 = 0x0017,
   OpNoAsyncClkSide 		 = 0x0018,          /* core does not have an async clock side */
   OpInvalidCsrc     		 = 0x0019,          /* clock source is invalid */
   OpInvalidDiv     		 = 0x001a,          /* divider is invalid */
   OpCsrcInUse       		 = 0x001b,          /* clock source is in-use */
   OpFreqLessThanMinFreq     = 0x001c,          /* Desired frequency is less than the minimum freq supported */
   OpMultiCoresBeingEnabled  = 0x001d,          /* Attempting to enable multiple cores, disable core sharing async-clk before enabling this core */
   OpInvalidCmSysSettings	 = 0x001e,
   OpInvalidCmAhbSettings	 = 0x001f,
   OpCmAhbGetFreqFailed		 = 0x0020,
   OpNoPinShareInfoForCore   = 0x0021,          /* There is no pin share information for the given CORE */
   OpPinShareOperationFailed = 0x0022,          /* Pin share related operation failed */
   OpPeripCustomLogicFailed  = 0x0023,          /* Peripheral custom logic Handler failed */

   OpIoctlError		         = 0x0030,
   OpDriverOpenError         = 0x0031,
   OpOutOfMemory	         = 0x0032,


   OpPmuWrongInputVoltageVal   = 0x0060,
   OpPmuWrongi2cCommandType = 0x0061,

   OpVoltDesiredFreqNotInTable = 0x0070,
   OpVoltDesiredFreqNotInARMTable = 0x0071,
   OpVoltDesiredFreqNotInVCTable = 0x0072,
   OpVoltDesiredFreqArmSideGreaterThanMax = 0x0073, 
   OpVoltDesiredFreqVcSideGreaterThanMax = 0x0074 ,
   OpVoltGetArmFreqFailure = 0x0075 ,
   OpVoltGetArmVoltageFailure = 0x0076 ,
   OpVoltGetVcVoltageFailure = 0x0077 ,
   OpVoltGetVcFreqFailure = 0x0078 ,

   // OpPwrSeqWrongInputVoltageVal = 0x0080,
   OpPwrSeqClearRegFile = 0x0081,
   OpPwrSeqRun2SleepTimerSetFailure = 0x0082, 
   OpPwrSeqSleep2HibTimerSetFailure = 0x0083, 
   OpPwrSeqRun2HibTimerSetFailure = 0x0084, 
   OpPwrSeqTimersSetFailure = 0x0085, 
   OpPwrSeqGetCmdsForSetVoltageFailure = 0x0086,
   OpPwrSeqStartSchemeFailure = 0x0087, 
   OpPwrSeqStopSchemeFailure = 0x0088, 
   OpPwrSeqAddCmdFailure = 0x0089,
   OpPwrSeqProgRegFileRunToSleepFailure = 0x008a , 
   OpPwrSeqProgRegFileSleepToHibernateFailure = 0x008b ,
   OpPwrSeqProgRegFileSleepToRunFailure = 0x008c ,
   OpPwrSeqProgRegFileHibernateToRunFailure = 0x008c ,
   OpPwrSeqProgRegFileRunToHibernateFailure = 0x008d ,
   OpPwrSeqEnablingWakeupEventsFailure = 0x008e ,
   OpPwrSeqClearRegFileFailure = 0x008f,
   OpPwrSeqTimerSetRunToSleepFailure = 0x0090,  
   OpPwrSeqTimerSetSleepToHibernateFailure = 0x0091,
   OpPwrSeqIntrSetRunToRunFailure = 0x0092,
   OpPwrSeqIntrSetRunToHibernateFailure = 0x0093,
   OpPwrSeqIntrSetRunToSleepFailure = 0x0094,
   OpPwrSeqIntrSetSleepToHibernateFailure = 0x0095, 
   OpPwrSeqProgRegFileFailure = 0x0096,
   OpPwrSeqSetPowerSourceFailure = 0x0097, 
   OpPwrSeqEnableEventsRunToSleepFailure = 0x0099,
   OpPwrSeqSetOscillatorBitsFailure = 0x009a,
   OpPwrSeqStartTransFailure = 0x009b,
   OpPwrSeqSaveIntrFailure = 0x009c ,
   OpPwrSeqDisIntrFailure = 0x009d ,
   OpPwrSeqPwrMgrIntrEnableFailure = 0x009e ,

   OpTransFromRunToSleepFailure = 0x009f,
   OpPwrSeqEnaIntrFailure = 0x0100 ,
   OpPwrSeqCheckAndClearPMIntrStatus = 0x0101 ,



//Add new return codes above this line                                             
   OpInvalidOpMode        = 0xFFFE,          /* Invalid operational mode specified        */
   OpNotImplemented       = 0xFFFF,          /* Functionality has not been implemented    */
   OpStatLast             = OpNotImplemented,
   OpStatMaxEntries
}tahoe_stat_t;


/* defined in tahoe_stat_str.c */
tahoe_stat_t tahoe_get_stat_str_size(tahoe_stat_t status_code, int *buf_size);

/* defined in tahoe_stat_str.c */
tahoe_stat_t tahoe_get_stat_str(tahoe_stat_t status_code, char *status_str_buf, int buf_size);

#endif /* _BCM_OPMODE_STAT_H_ */
