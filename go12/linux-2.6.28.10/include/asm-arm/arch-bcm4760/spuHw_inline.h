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
*  @file    spuHw_inline.h
*
*  @brief   Inline API implementation for low level Security Processor Unit module
*
*/
/****************************************************************************/
#ifndef SPUHW_INLINE_H
#define SPUHW_INLINE_H

#include <asm/arch/spuHw.h>

#define SECURE_ACCESS               0x00000001
#define SECURE_WRITE_ACCESS         0x00000002
#define OPEN_ACCESS                 0x00000003
#define READ_VIOLATION              0x00000001

#define GENERATE_ERROR_INTERRUPT    0x00000003
#define DISABLE_ERROR_INTERRUPT     0x00000000
#define GENERATE_SLAVE_ERROR        0x00000001
#define ERROR_INTERRUPT_STATUS      0x40000000
#define DISABLE_DMA_INTERRUPT       0x00000000

#define DMA_INTERFACE_CLEAR         0x00000008
#define DMA_OPERATION_DONE          0x00000004
#define DMA_INTERFACE_BUSY          0x00000002
#define DMA_INTERFACE_ENABLE        0x00000001

#define CLEAR_INTERRUPT             0x00000000
#define IN_FLOW_CONTROLLER_DMA      0x00000001    /* DMA as input flow controller */
#define OUT_FLOW_CONTROLLER_DMA     0x00000002    /* DMA as output flow controller */

#define SPU_SOFT_RESET              0x00000002    /* Reset SPU core logic */

/****************************************************************************/
/**
*  @brief   Sets misc. key table configuration options
*
*  
*  @return    -1 - On failure
*              0 - On success
*  @note
*     - Host CPU must be in secure mode to allow this operation
*     - Please see spuHw_reg.h for available key table configuration options
*/
/****************************************************************************/
static inline int spuHw_configKeyTable 
( 
   uint32_t options  ///< Key table configuration options
)
{
   pSpuHw_APB_REG->KekControl = options;
   return 0;
}


/****************************************************************************/
/**
*  @brief   Intializes the SPU device
*
*  
*  @return  void
*
*  @note
*     - Host CPU must be in secure mode to allow this operation
*/
/****************************************************************************/
static inline void spuHw_initDevice ( void )
{
   /* Initialize the device in open mode */
   pSpuHw_APB_REG->Control = spuHw_DEV_CONFIG_OPEN | SPU_SOFT_RESET;
   /* Wait until soft reset is completed */
   while ( pSpuHw_APB_REG->Control & SPU_SOFT_RESET );
   /* Clear DMA interface */
   pSpuHw_AXI_REG->DmaStatus = DMA_INTERFACE_CLEAR;
   pSpuHw_AXI_REG->DmaStatus = 0;
   pSpuHw_AXI_REG->Reaction = GENERATE_SLAVE_ERROR;
   pSpuHw_AXI_REG->InterruptStatus = CLEAR_INTERRUPT;
   pSpuHw_AXI_REG->InterruptEnable = DISABLE_DMA_INTERRUPT;
   pSpuHw_AXI_REG->AccessPermission = OPEN_ACCESS;
   pSpuHw_AXI_REG->DmaControl = IN_FLOW_CONTROLLER_DMA | OUT_FLOW_CONTROLLER_DMA;
   pSpuHw_AXI_REG->DmaSize = 0x00000000;
   /* Configure the device as secure device */
   pSpuHw_APB_REG->Control = spuHw_DEV_CONFIG_OPEN;
}


/****************************************************************************/
/**
*  @brief   Sets misc. SPU device configuration options
*
*  
*  @return    -1 - On failure
*              0 - On success
*  @note
*           Please see spuHw_reg.h for available options ( spuHw_DEV_CONFIG_XXXXXX )
*/
/****************************************************************************/
static inline int spuHw_configDevice 
( 
   uint32_t options    ///< [IN] SPU device configuration options.
)
{
   pSpuHw_APB_REG->Control = options;
   pSpuHw_APB_REG->KekBase = spuHw_KEK_BASE_OFFSET;

   return 0;
}

/****************************************************************************/
/**
*  @brief   Set secure data access to SPU blcok
*
*  
*  @return    -1 - On failure
*              0 - On success
*/
/****************************************************************************/
static inline int spuHw_setSecureAccess ( void )
{
/* 476x only support Open mode */
   pSpuHw_AXI_REG->AccessPermission = OPEN_ACCESS;
   return 0;
}

/****************************************************************************/
/**
*  @brief   Set only the secure write access to SPU blcok
*
*  
*  @return    -1 - On failure
*              0 - On success
*/
/****************************************************************************/
static inline int spuHw_setSecureWriteAccess ( void )
{
/* 476x only support Open mode */
   pSpuHw_AXI_REG->AccessPermission = OPEN_ACCESS;
   return 0;
}

/****************************************************************************/
/**
*  @brief   Set both secure and non-secure data access to SPU blcok
*
*  
*  @return    -1 - On failure
*              0 - On success
*/
/****************************************************************************/
static inline int spuHw_setOpenAccess ( void )
{
   pSpuHw_AXI_REG->AccessPermission = OPEN_ACCESS;
   return 0;
}

/****************************************************************************/
/**
*  @brief   Get SPU input FIFO address 
*
*  
*  @return    0  - On failure
*             !0 - On success
*
*  @note
*            Input FIFO address should be used to send packet into SPU block 
*/
/****************************************************************************/
static inline uint32_t spuHw_getInputFifoAddress ( void )
{

  return (uint32_t) spuHw_IN_FIFO_ADDR;
}

/****************************************************************************/
/**
*  @brief   Get SPU output FIFO address 
*
*  
*  @return     0  - On failure
*              !0 - On success
*  @note
*            Output FIFO address should be used to receive packet from SPU block 
*/
/****************************************************************************/
static inline uint32_t spuHw_getOutputFifoAddress ( void )
{

  return (uint32_t) spuHw_OUT_FIFO_ADDR;
}

/****************************************************************************/
/**
*  @brief   Sets input and output packet length to begin SPU processing.
*
*  
*  @return  -1 : If fails the set the data length (channel is busy)
*            0 : Successful
*  @note
* 
*     Output SPU Packet length (outLen) must be known before hand
*
*     Input SPU Packet:
*
*     |<--CommandLen--->|    |<-------dataLen-------------------->|
*     +-----------------+....+------------------------------------+.......
*     | SPU Command     |    |   Data                             |       
*     +-----------------+....+------------------------------------+.......
*     ^                      ^
*     |                      |
*     command                data
*
*     Output SPU Packet:
*
*     |<--CommandLen--->|<-------dataLen-------------------->|<-Hash->|
*     +-----------------+------------------------------------+--------+-+.......
*     | SPU Command     |   Data                             |        | |
*     +-----------------+------------------------------------+--------+-+......
*     ^                 ^                                             ^
*     |                 |                                             |
*     command           data                                          Status
*
*     outLen = CommandLen (spuHw_OUTPUT_HEADER_LEN) + dataLen + Hash len + 
*              Status len (spuHw_OUTPUT_STATUS_LEN)
*
*     Once input and output packet lengths are set, use external DMA or 
*     spuHw_performCryptoOperation to transfer the Input SPU Packet to SPU block.
*
*/
/****************************************************************************/
static inline int spuHw_setPacketLength 
( 
   size_t inLen,          ///< [IN]  Input SPU packet length (command + payload) in bytes
   size_t outLen          ///< [IN]  Expected output (PTM) packet length in bytes
)
{
   if (pSpuHw_AXI_REG->DmaStatus & DMA_INTERFACE_BUSY )
   {
     return -1;
   }
   pSpuHw_AXI_REG->DmaStatus = 0;
   /* Set the length of the DMA transfer */
   /* Adding extra 1 word to the input data ( inLen + 1 ) to fullfill 
      the requirement for having an extra status word at the end of
      input data
   */
   inLen = ( ( inLen + 3 ) / sizeof (uint32_t) ) + 1;
   outLen = ( outLen + 3 ) / sizeof (uint32_t);
   pSpuHw_AXI_REG->DmaSize = ( ( outLen & 0x0000FFFF ) << 16 ) | ( inLen & 0x0000FFFF );
   return 0;
}

/****************************************************************************/
/**
*  @brief   Initiates DMA operation
*
*  @note
*           This function should be called after setting up the DMA channel
*           using external DMA device
*/
/****************************************************************************/
static inline void spuHw_initiateDma ( void )
{
   pSpuHw_AXI_REG->DmaStatus =  DMA_INTERFACE_ENABLE;
} 

/****************************************************************************/
/**
*  @brief    Get context id within SPU packet received from SPU block
*
*  
*  @return   Context ID
*
*  @note
*
*     Output SPU Packet:
*
*     +-----------------+------------------------------------+-----.......
*     | SPU Command     |   Data                             |       
*     +-----------------+------------------------------------+-----.......
*     ^                 ^                                    ^
*     |                 |                                    |
*     outPacket         Begining of the data                 Begining of the authentication data
*
*
*/
/****************************************************************************/
static inline spuHw_CONTEXT_ID_t spuHw_getContextId 
(
   spuHw_PACKET_t outPacket      ///<  [IN] SPU packet received from SPU block
)
{
   return ((uint32_t*)outPacket)[1];
}

/****************************************************************************/
/**
*  @brief    Get current processing status
*
*  
*  @return   1 - Data processing complete
*            0 - Data processing on going
*
*  @note
*      Data processing starts after DMAing input packet to SPU block 
*      at input FIFO address. 
*
*      SPU processed data can be retrieved DMAing output packet from
*      output FIFO address.
*/
/****************************************************************************/
static inline int spuHw_processingDone ( void )
{
   return pSpuHw_AXI_REG->DmaStatus & DMA_OPERATION_DONE;
}

/****************************************************************************/
/**
*  @brief    Get most recently completed SPU data processing status
*
*  
*  @return   status of type spuHw_STATUS_e
*
*            
*/
/****************************************************************************/
static inline spuHw_STATUS_e spuHw_getProcessingStatus ( void )
{
   uint32_t status = pSpuHw_AXI_REG->SecFailedStatus;

   /* Check if any error interrupt happened */
   if ( status & ERROR_INTERRUPT_STATUS )
   {
      if ( status & READ_VIOLATION )
      {
         return spuHw_STATUS_READ_ACCESS_ERROR;
      }
      else
      {
         return spuHw_STATUS_WRITE_ACCESS_ERROR;
      }
   }

   return spuHw_STATUS_SUCCESS;
}

/****************************************************************************/
/**
*  @brief   Enable interrupt for SPU error notification
*
*  
*  @return  void
*/
/****************************************************************************/
static inline void spuHw_setErrorInterruptEnable ( void )
{
   /* Generate interrupt upon error */
   pSpuHw_AXI_REG->Reaction = GENERATE_ERROR_INTERRUPT;
}

/****************************************************************************/
/**
*  @brief   Disable interrupt for SPU error notification
*
*  
*  @return void
*/
/****************************************************************************/
static inline void spuHw_setErrorInterruptDisable ( void )
{
   /* Do not report access violation */
   pSpuHw_AXI_REG->Reaction = DISABLE_ERROR_INTERRUPT;
}

/****************************************************************************/
/**
*  @brief   Clear interrupt for SPU error notification
*
*  
*  @return  void
*/
/****************************************************************************/
static inline void spuHw_clearErrorInterrupt ( void )
{
   /* Clear any un-serviced interrupt */
   pSpuHw_AXI_REG->InterruptStatus = CLEAR_INTERRUPT;
}

/****************************************************************************/
/**
*  @brief   Identify the staus after crypto processing
*
*  
*  @return  spuHw_CRYPTO_STATUS_e
*
*   @note
*    
*     Output SPU Packet:
*
*     +-----------------+------------------------------------+-+.....
*     | SPU Command     |   Data                             | |     
*     +-----------------+------------------------------------+-+.....
*     ^                                                      ^
*     |                                                      |
*     outPacket                                              Status
*
*     |<---------------------packetLen------------------------>|
*/
/****************************************************************************/
static inline spuHw_CRYPTO_STATUS_e spuHw_getCryptoStatus
(
   spuHw_PACKET_t outPacket,      ///<  [IN] SPU packet received from SPU block
   int            packetLen       ///<  [IN] Total length of the SPU packet received from SPU block in bytes
)
{
   uint32_t status = ( (uint32_t *)outPacket ) [ ((packetLen + 3) / sizeof (uint32_t) ) - 1];

   if ( status & spuHw_CMD_CRYPTO_STATUS_ERROR  )
   {
      switch ( status & spuHw_CMD_CRYPTO_STATUS_MASK )
      {
         case spuHw_CMD_CRYPTO_STATUS_SUCCESS: 
         case spuHw_CMD_CRYPTO_STATUS_INVALID_KEY_HANDLE:
         case spuHw_CMD_CRYPTO_STATUS_INVALID_ICV:
         case spuHw_CMD_CRYPTO_STATUS_INVALID_KEY:
            return (spuHw_CRYPTO_STATUS_e)(status & spuHw_CMD_CRYPTO_STATUS_MASK);
      }
      return spuHw_CRYPTO_STATUS_UNKNOWN ;
   }
   return spuHw_CRYPTO_STATUS_SUCCESS;
}


#endif /* SPUHW_INLINE_H */
