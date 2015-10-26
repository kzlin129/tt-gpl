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
 *   @file   armbusmtx.h 
 * 
 *   @brief  ARM Bus Matrix low level API 
 * 
 ****************************************************************************/

#ifndef _ARMBUSMTX_IFC_H_
#define _ARMBUSMTX_IFC_H_

#include "bcm_os_support.h"
#include "bcm_privtypes.h"

//--------------------------------------------------
/**
    @enum armbusmtx_synctop_mode_t

   Sychronizer Operational mode.

    @ingroup  ARMBUSMTX
*/
//--------------------------------------------------
typedef enum _armbusmtx_synctop_mode_t
{
	ARMBUSMTX_SYNCTOP_MODE_BYPASS     = 0, /**< 0 */  //SHOULD BE USED ONLY FOR TESTING!!!
    ARMBUSMTX_SYNCTOP_MODE_BANDWIDTH  = 1, /**< 1 */
    ARMBUSMTX_SYNCTOP_MODE_LATENCY    = 2, /**< 2 */   
} armbusmtx_synctop_mode_t;


//--------------------------------------------------
/**
    @enum armbusmtx_synctop_clkmode_t

   Sychronizer clock mode.

    @ingroup  ARMBUSMTX
*/
//--------------------------------------------------
typedef enum _armbusmtx_synctop_clkmode_t
{
	ARMBUSMTX_SYNCTOP_CLKMODE_1to1    = 0, 
	ARMBUSMTX_SYNCTOP_CLKMODE_Nto1    = 1, /***/  //Master N times faster than the slave
	ARMBUSMTX_SYNCTOP_CLKMODE_1toN    = 2, /***/  //Master N times slower than the slave
}armbusmtx_synctop_clkmode_t;

//--------------------------------------------------
/**
    @enum armbusmtx_synctop_t

    USB mode

    @ingroup  ARMBUSMTX
*/
//--------------------------------------------------
typedef enum _armbusmtx_synctop_t
{
    ARMBUSMTX_SYNC_AI  = 0,     /**< 0 */
    ARMBUSMTX_SYNC_AO  = 1,     /**< 1 */
    ARMBUSMTX_SYNC_PI  = 2,     /**< 2 */
    ARMBUSMTX_SYNC_VI  = 3,     /**< 3 */
    ARMBUSMTX_SYNC_VO  = 4,     /**< 4 */
    ARMBUSMTX_SYNC_INVALID, 
} armbusmtx_synctop_t;


//Recommended watermark values from 2820 datasheet
#define ARMBUSMTX_SYNCTOP_DEFAULT_LOW_WATER_MARK   0x2
#define ARMBUSMTX_SYNCTOP_DEFAULT_HIGH_WATER_MARK  0xD

/**
    @struct armbusmtx_synctop_info_t

    @ingroup  ARMBUSMTX
*/
typedef struct _armbusmtx_synctop_info_t
{
    armbusmtx_synctop_mode_t    mode;
    bool_t                    clk_change;    //TRUE - update involves clk change.
                                             //FALSE - update does not involve clk change.
    /**
        Note: System requirements from 2820 Datasheet
              For 5 sync_tops, system requires the following: 
              Everyone can be 1 to 1 mode. 2 can be in N to 1 mode and the other 3 can be in 1:N mode. 

              The caller has to ensure these requirements are met as this layer
              does not keep tab on the different synctop clk modes.
    */
    armbusmtx_synctop_clkmode_t clkmode;       

    //If the caller does not know what the best watermarks values, 
    //the defaults specified above should be used.
    uint32_t                  high_watermark;//used only in case of latency mode
    uint32_t                  low_watermark; //used only in case of latency mode
    uint32_t                  prefetch_len;
}armbusmtx_synctop_info_t;


//--------------------------------------------------
/**
    Initialize ARM Bus Matrix.

    @param  Block virtual address

    @special
        This initialization routine currently initializes the base 
        address for the arm bus matrix block. 

    @retval   None

    @ingroup  ARMBUSMTX
*/
//--------------------------------------------------
void armbusmtx_api_init(void);


//--------------------------------------------------
/**
    Set Sync top mode.

    @param  synctop       - which synctop?
    @param  psynctop_info  - mode and watermark info if mode is latency

    @special
        This function modifies a couple of different fields in the sync-
    cntrl registers. This is required most of the times as all the fields
    have to be set at the same time. For ex: indicating a clock change is
    associated with an update. 

    At times, the caller may want to just modify say prefetch length or the 
    watermarks. In such cases, the api's that allow modifying single fields 
    can be used. 
    
    Under other circumstances, when the current mode has to be changed leaving
    the prefetch length unchanged, the caller invoke armbusmtx_get_synctop()
    to get the current settings and modify the required changes before calling
    this API i.e. perform a read-modify-write.


    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  ARMBUSMTX
*/
//--------------------------------------------------
uint32_t armbusmtx_set_synctop(armbusmtx_synctop_t synctop, const armbusmtx_synctop_info_t *psynctop_info);


//--------------------------------------------------
/**
    Get synctop modes.

    @param  synctop       - which synctop?
    @param  synctop_info  - mode and watermark info if mode is latency

    @special
        The watermark fields in the synctop_info struct make sense only
        when the mode is latency mode.

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  ARMBUSMTX
*/
//--------------------------------------------------
uint32_t armbusmtx_get_synctop(armbusmtx_synctop_t synctop, armbusmtx_synctop_info_t *psynctop_info);


//--------------------------------------------------
/**
    Set prefetch length.

    @param  synctop - which synctop?
    @param  prefetch_len       

    @special
        None.

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  ARMBUSMTX
*/
//--------------------------------------------------
uint32_t armbuxmtx_synctop_set_prefetch_len(armbusmtx_synctop_t synctop, uint32_t prefetch_len);


//--------------------------------------------------
/**
    Set low/high watermarks.

    @param  synctop - which synctop?
    @param  low_watermark       
    @param  high_watermark

    @special
        The watermark fields make sense only when the mode is latency mode.

    @retval   BCM_OK - Success
    @retval   BCM_ERROR - Failed

    @ingroup  ARMBUSMTX
*/
//--------------------------------------------------
uint32_t armbuxmtx_synctop_set_watermarks(armbusmtx_synctop_t synctop, uint32_t low_watermark, uint32_t high_watermark);

#endif // _ARMBUSMTX_IFC_H_

