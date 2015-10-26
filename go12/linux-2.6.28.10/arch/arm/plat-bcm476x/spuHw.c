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

#include <linux/types.h>
#include <stddef.h>
#include <asm/arch/spuHw.h>
#include <linux/module.h>
#include <asm/arch/spuHw_inline.h>

/****************************************************************************/
/**
*  @file    spuHw.c
*
*  @brief   API implementation for low level Security Processor Unit module
*
*/
/****************************************************************************/
/* Misc. defines for command buffer */
#define INVALID_KEY_OFFSET               0x00000FFF     /* Invalid offset to the cache key */

#define SCTX_PRESENT                     0x80000000     /* Security context field is present in the command */
#define BDESC_PRESENT                    0x20000000     /* Buffer data descriptor is present in the command */
#define BD_PRESENT                       0x08000000     /* Buffer data is present in the command */
#define OUT_ALL_DATA                     0x00010000     /* Output all data */
#define HASH_PRESENT                     0x04000000     /* Hash data is present */

#define SCTX_TYPE_GENERIC                0x00000000
#define SCTX_SIZE_MASK                   0x000000FF
#define SCTX_SIZE_MIN                    3
#define SCTX_AUTH_FIRST                  0x40000000     /* Authentication is performed first. Encryption is done afterwards */

typedef struct
{
   uint32_t firstWord;
   uint32_t secondWord;
   uint32_t thirdWord;
   uint32_t extended;
}sctx_HDR_t;

#ifndef BOOT0_BUILD
static uint32_t spuDdrCache[spuHw_KEK_CACHE_SIZE];
#endif

#ifndef BOOT0_BUILD
/****************************************************************************/
/**
*  @brief   Initializes the table containing cryptographic key
*
*  
*  @return     -1 - On failure
*               0 - On success
*  @note
*     SPU must be in secure mode to allow this operation
*/
/****************************************************************************/
int spuHw_initKeyTable 
(
   uint32_t size                 ///<  [IN] Number of key entry into the table
)
{
   uint32_t *cache = (uint32_t *) ( spuHw_APB_BASE_ADDRESS + spuHw_KEK_BASE_OFFSET ); 
   uint32_t i;

   /* check cache availability */
   if ( size > ( spuHw_KEK_CACHE_SIZE - 1 ) / 2 )
   {
      return -1;
   }

   /* Initialize the entire cache space */
   for ( i = 0; i < spuHw_KEK_CACHE_SIZE; i++ )
   {
      spuDdrCache[i] = cache[i] = INVALID_KEY_OFFSET;
   }

   /* Set the offset pointing to the first key entry */
   spuDdrCache[0] = cache[0] = ( size + 1 ) * 4;

   return 0;
}
EXPORT_SYMBOL(spuHw_initKeyTable);
#endif /* BOOT0_BUILD */




#ifndef BOOT0_BUILD
/****************************************************************************/
/**
*  @brief   Adds cryptographic key into the key table
*
*  
*  @return    -1 - On failure
*              0 - On success
*  @note
*         0             = first entry
*        TableSize - 1  = last entry 
*
*        - TableSize is set using spuHw_initKeyTable()
*        - Host CPU must be in secure mode to allow this operation
*/       
/****************************************************************************/
int spuHw_setKey 
( 
   spuHw_KEY_INDEX_t  index,             ///< [IN] Key position (index) in the table 
   spuHw_KEY_t        *pKey              ///< [IN] Cryptographic key
)
{
   uint32_t i;
   uint32_t offset = 0;
   uint32_t tableSize = 0;
   uint32_t keyAttrib = 0;
   uint32_t keyBase = 0;
   uint32_t endAddr;
   uint32_t *cache = (uint32_t *) ( spuHw_APB_BASE_ADDRESS + spuHw_KEK_BASE_OFFSET ); 

   /* Find the total number of key to be configured */
   tableSize = ( spuDdrCache[0] / 4 ) - 1;

   /* Exit, if table is not initialized or index too large */
   if ( tableSize == 0 || index >= tableSize )
   {
      return -1;
   }
   /* Read the desired key attribute */
   keyAttrib = spuDdrCache[index + 1];

   /* Check if this is a new key (offset not defined yet) */
   if ( ( offset = keyAttrib & spuHw_CMD_KEY_OFFSET_MASK ) == INVALID_KEY_OFFSET )
   {
      uint32_t prevEntry = spuDdrCache[0];

      /* Look backward for previous offset */
      i = index;
      while ( i && ( ( prevEntry = spuDdrCache[i] ) & spuHw_CMD_KEY_OFFSET_MASK ) == INVALID_KEY_OFFSET ) i--;
      if ( i == 0 )
      {
	 if ( ( prevEntry & spuHw_CMD_KEY_OFFSET_MASK ) == INVALID_KEY_OFFSET )
         {
 	    return -1;
         }
         else
         {
            offset = 0;
         }
      }
      else
      {
         /* Calculate word aligned new offset taking offset and size from previous key entry */
         offset = ( ( prevEntry & spuHw_CMD_KEY_OFFSET_MASK ) + ( ( prevEntry & spuHw_CMD_KEY_SIZE_MASK ) >> spuHw_CMD_KEY_SIZE_SHIFT ) + 3 ) & 0xFFFFFFFC;
      }
   }

   keyBase = spuDdrCache[0] & spuHw_CMD_KEY_OFFSET_MASK;

   /* Must have  a valid offset */
   if ( ( offset + keyBase ) < ( spuHw_KEK_CACHE_SIZE * 4 ) )
   {
      /* Identify the end address of the key */
      endAddr = keyBase + offset + ( pKey->size * sizeof (uint32_t) );

      /* Exit, if key is too big to fit into the table */
      if ( endAddr > ( spuHw_KEK_CACHE_SIZE * 4 ) )
      {
         return -1;
      } 
      /* Make sure the new key doensot overwrite any valid key */
      for ( i = index + 1; i < tableSize; i++ )
      {
         uint32_t nextEntry = spuDdrCache[ i + 1 ];
         uint32_t nextOffset = keyBase + ( nextEntry & spuHw_CMD_KEY_OFFSET_MASK );

         if (  ( nextOffset != INVALID_KEY_OFFSET ) && ( endAddr > nextOffset ) )
         {
            /* Check the validity of the next entry */
            if ( nextEntry & spuHw_CMD_KEY_VALID )
            {
               /* can not overwrite a valid entry */
               return -1;
            }
            else 
            {
               /* remove invalid entry */
               spuDdrCache[ i + 1 ] = cache[ i + 1 ] = 0;
            }
         }
      }
      /* Load the key value */
      for ( i = 0; i < pKey->size; i++ )
      {
         ((uint32_t* )(spuHw_APB_BASE_ADDRESS + spuHw_KEK_BASE_OFFSET + keyBase + offset))[i] =  pKey->key[i];
      }
      /* Set new key attribute */
      keyAttrib = 0;
      /* Set attribute for valid key */
      if ( pKey->valid )
      {
         keyAttrib |= spuHw_CMD_KEY_VALLID;
      }
      /* Set attribute for secure key */
      keyAttrib |= pKey->mode;
      /* Set attribute for key type */
      keyAttrib |= pKey->type;
      /* Set attribute for key size */
      keyAttrib |= ( pKey->size * sizeof (uint32_t) ) << spuHw_CMD_KEY_SIZE_SHIFT;
      /* Set attribute for key offset */
      keyAttrib |= offset;
      /* Update key attribute */
      spuDdrCache[index + 1] = cache [index + 1] = keyAttrib;
   }
   else
   {
      return -1;
   }

   /* Key installed successfully */
   return 0;
}
EXPORT_SYMBOL(spuHw_setKey);
#endif


/****************************************************************************/
/**
*  @brief   Creates command for crypto processing 
*
*  
*  @return   -1                           - On failure
*            Length of command in bytes   - On success 
*
*  @note
*            Command lengh must be used to estimate the SPU packet (command + data ) length
*
*     SPU Command:
*
*     |<--CommandLen--->|   
*     +-----------------+...
*     | SPU Command     |   
*     +-----------------+...
*     ^                     
*     |                     
*     command               
*
*     The command buffer (pCommand) must begin on 32 bit boundary
*/
/****************************************************************************/
int spuHw_createCryptoCommand 
( 
   spuHw_CONTEXT_t  *pCtx,         ///< [IN]  Context for crypto operation
   void             *pCommand      ///< [OUT] Command buffer
)
{
   uint32_t   *pCmd = (uint32_t*)pCommand;
   sctx_HDR_t *pSctxHdr;
   uint32_t   *pDataAttr;
   uint32_t   idx = 0;
   uint32_t   len = 0;      
   uint32_t   i = 0;      

   pCmd[0] =  (SCTX_PRESENT | BDESC_PRESENT | BD_PRESENT | OUT_ALL_DATA);
   /* Set extended message header with the context id */
   pCmd[1] =  pCtx->contextId;
   /* Set SCTX header */
   pSctxHdr = ( sctx_HDR_t* ) & pCmd[2];

   pSctxHdr->secondWord =  pCtx->authOrder  | 
                           pCtx->operation  | 
                           pCtx->cryptoAlgo | 
                           pCtx->cryptoMode | 
                           pCtx->cryptoType | 
                           pCtx->authAlgo   |
                           pCtx->authMode   |
                           pCtx->authType   |
                           pCtx->keyUpdate;   

   if ( pCtx->icvLen )
   {
      pSctxHdr->thirdWord = ( pCtx->icvLen << spuHw_CMD_SCTX_ICV_SIZE_SHIFT ) & spuHw_CMD_SCTX_ICV_SIZE_MASK  ;
   }
   else
   {
      pSctxHdr->thirdWord = 0;
   }

   if ( pCtx->keyType == spuHw_KEY_PROTECTED )
   {
      pCtx->authKeyLen = 0;
      pCtx->cryptoKeyLen = 0;
      pSctxHdr->thirdWord |= spuHw_CMD_SCTX_KEY_PROTECT |
                            ( ( ( ( pCtx->keyHandle + 1 ) * 4 ) << spuHw_CMD_SCTX_KEY_HANDLE_SHIFT ) & spuHw_CMD_SCTX_KEY_HANDLE_MASK );
   }
   else
   {
      /* Set Authentication key */
      len = pCtx->authKeyLen;
      for ( idx = 0; idx < len; idx++ )
      {
         ((uint32_t*)&pSctxHdr->extended)[idx] = ((uint32_t*)pCtx->authKey)[idx];
      }

      /* Set crypto key */
      len = pCtx->authKeyLen + pCtx->cryptoKeyLen;
      for ( i = 0; idx < len; idx++, i++ )
      {
         ((uint32_t*)&pSctxHdr->extended)[idx] = ((uint32_t*)pCtx->cryptoKey)[i];
      }
   }

   if ( pCtx->initVectorLen )
   {
      pSctxHdr->thirdWord |= spuHw_CMD_SCTX_IV_CONTEXT;
      /* Set initialize vector */
      len = pCtx->authKeyLen + pCtx->cryptoKeyLen + pCtx->initVectorLen;
      for ( i = 0 ; idx < len; idx++, i++ )
      {
         ((uint32_t*)&pSctxHdr->extended)[idx] = ((uint32_t*)pCtx->initVector)[i];
      }
   }

   pSctxHdr->firstWord =   SCTX_TYPE_GENERIC | ( SCTX_SIZE_MASK & ( SCTX_SIZE_MIN + pCtx->authKeyLen +  pCtx->cryptoKeyLen + pCtx->initVectorLen));
   /* Include data info */
   pDataAttr = (uint32_t*) ((void*)pSctxHdr + ( SCTX_SIZE_MIN + pCtx->authKeyLen + pCtx->cryptoKeyLen + pCtx->initVectorLen ) * sizeof (uint32_t));
   /* Updating Buffer Descriptor header */
   pDataAttr[0] = (pCtx->dataAttribute.macOffset << 16) | pCtx->dataAttribute.macLength;
   pDataAttr[1] = (pCtx->dataAttribute.cryptoOffset << 16) | pCtx->dataAttribute.cryptoLength;
   if ( pCtx->dataAttribute.aesGcmAadLength )
   {
      /* Use offset IV to store aesGcmAadLength */
      pDataAttr[2] = pCtx->dataAttribute.aesGcmAadLength;
   }
   else
   {
      pDataAttr[2] = 0;  /* Overwriting Offset ICV and IV */
   }
   /* Updating Buffer Data header */
   pDataAttr[3] = pCtx->dataAttribute.dataLength << 16; 

   return ((void*)pDataAttr + ( sizeof (uint32_t) * 4 )) - pCommand;
}
EXPORT_SYMBOL(spuHw_createCryptoCommand);


/****************************************************************************/
/**
*  @brief   Performs authentication operation in non-DMA mode
*
*  
*  @return  On Error   : -1
*           On Success :  0
*
*  @note
*
*     Input parameters:
*
*     |<--CommandLen--->|   
*     +-----------------+...+------------------------------------+.......
*     | SPU Command     |   | Data                               |       
*     +-----------------+...+------------------------------------+.......
*     ^                     ^
*     |                     |
*     pCommand              pInputData (begining of the data)
*
*                           |<-------dataLen-------------------->|
*
*
*     Authentication data:
*
*     +------------------------------------+.......
*     |   Authentication Data              |       
*     +------------------------------------+.......
*                       
*     All the buffers must be 32 bit aligned
*  
*  @Important
*     This function assumes spuHw_setPacketLength() is called prior to 
*     calling this function
*/
/****************************************************************************/
int spuHw_performAuthenticationOperation
(    
   void             *pCommand,      ///< [IN] Command buffer
   int              commandLen,     ///< [IN] Length of the command buffer in bytes
   void*            *pInputData,    ///< [IN] Input data 
   void*            authOutput,     ///< [OUT] Authentication data
   int              authLen         ///< [IN] Expected authentication length in bytes
)
{
   int i, j, authIndex;
   int inputDataLen = ( (  pSpuHw_AXI_REG->DmaSize & 0x0000FFFF ) - 1 /* Status */) * sizeof (uint32_t) - commandLen;
   int outputDataLen = ( pSpuHw_AXI_REG->DmaSize & 0xFFFF0000 ) >> 16 ;
   uint32_t skip = 0;

   if ( ( pSpuHw_AXI_REG->DmaStatus & DMA_INTERFACE_BUSY ) || 
        ( inputDataLen == 0 ) || 
        ( outputDataLen == 0 ) || 
        ( authLen == 0 ) )
   {
      return -1;
   }

   /* Convert authentication length */
   authLen = ( ( authLen + 3 ) / sizeof (uint32_t) ) + 1 /* Status word */;

   authIndex = outputDataLen  - authLen;

   /* Activate the FIFO interface */
   pSpuHw_AXI_REG->DmaStatus = DMA_INTERFACE_ENABLE;

   /* Push the command into the input FIFO */
   commandLen = commandLen / sizeof (uint32_t);
   for ( i = 0, j = 0; i < commandLen; i++ )
   {
      while(!(pSpuHw_AXI_REG->FifoStatus) & 0x00001 );
      pSpuHw_AXI_REG->FifoInReg = ((uint32_t*)pCommand)[i];
      if( pSpuHw_AXI_REG->FifoStatus & 0x10000 )
      {
         if ( j >= authIndex )
         {
            ((uint32_t*)authOutput)[j - authIndex] = pSpuHw_AXI_REG->FifoOutReg;
         }
         else
         {
            skip = pSpuHw_AXI_REG->FifoOutReg;
         }
         j++;
      }
   }
   inputDataLen = ( inputDataLen + 3 ) / sizeof (uint32_t);
   /* Push the input data into the input FIFO */   
   for ( i = 0; i < inputDataLen; i++ )
   {
      while( !(pSpuHw_AXI_REG->FifoStatus) & 0x00001 );
      pSpuHw_AXI_REG->FifoInReg = ((uint32_t*)pInputData)[i];
      if( pSpuHw_AXI_REG->FifoStatus & 0x10000 )
      {
         if ( j >= authIndex )
         {
            ((uint32_t*)authOutput)[j - authIndex] = pSpuHw_AXI_REG->FifoOutReg;
         }
         else
         {
            skip = pSpuHw_AXI_REG->FifoOutReg;
         }
         j++;
      }
   }

   /* Push extra one word for status */
   while(!(pSpuHw_AXI_REG->FifoStatus) & 0x00001 );   
   pSpuHw_AXI_REG->FifoInReg = 0x00000000;

   /* Keep continue reading from the output FIFO */
   while( j < outputDataLen )
   {
      while(!(pSpuHw_AXI_REG->FifoStatus & 0x10000) );
      if ( ( j >= authIndex ) && ( j <  (outputDataLen - 1) ) )
      {
         ((uint32_t*)authOutput)[j - authIndex] = pSpuHw_AXI_REG->FifoOutReg;
      }
      else
      {
         skip = pSpuHw_AXI_REG->FifoOutReg;
      }
      j++;
   }

   /* Clear the interface */
   pSpuHw_AXI_REG->DmaStatus |= DMA_INTERFACE_CLEAR;
   pSpuHw_AXI_REG->DmaStatus &= ~DMA_INTERFACE_CLEAR;

   /* Check Error bit [17] in the output status word  */
   return ( skip & spuHw_CMD_CRYPTO_STATUS_ERROR ) ? -1 : 0;
}



#ifndef SPU_OCDE_REDUCED
/****************************************************************************/
/**
*  @brief   Performs crypto operation in non-DMA mode
*
*  
*  @return  On Error   : -1
*           On Success :  0
*
*  @note
*
*     Input parameters:
*
*     |<--CommandLen--->|   
*     +-----------------+...+------------------------------------+.......
*     | SPU Command     |   | Data                               |       
*     +-----------------+...+------------------------------------+.......
*     ^                     ^
*     |                     |
*     pCommand              pInputData (begining of the data)
*
*                           |<-------dataLen-------------------->|
*
*
*     Output SPU Packet:
*
*     +-----------------+------------------------------------+.......
*     | SPU Command     |   Data                             |       
*     +-----------------+------------------------------------+.......
*     ^                 
*     |                 
*     outPacket         
*
*                       
*     All the buffers must be 32 bit aligned
*/
/****************************************************************************/
int spuHw_performCryptoOperation
(    
   void             *pCommand,      ///< [IN] Command buffer
   int              commandLen,     ///< [IN] Length of the command buffer in bytes
   void*            *pInputData,    ///< [IN] Input data 
   int              inputDataLen,   ///< [IN] Input data length in bytes
   spuHw_PACKET_t   outPacket,      ///< [OUT] SPU packet received from SPU block   
   int              outputDataLen   ///< [IN] Expected SPU packet length in bytes
)
{
   int i, j;

   if (pSpuHw_AXI_REG->DmaStatus & DMA_INTERFACE_BUSY )
   {
     return -1;
   }
   /* Activate the FIFO interface */
   pSpuHw_AXI_REG->DmaStatus =  DMA_INTERFACE_ENABLE;

   /* Push the command into the input FIFO */
   commandLen = commandLen / sizeof (uint32_t);
   for ( i = 0, j = 0; i < commandLen; i++ )
   {
      while(!(pSpuHw_AXI_REG->FifoStatus) & 0x00001 );
      pSpuHw_AXI_REG->FifoInReg = ((uint32_t*)pCommand)[i];
      if( pSpuHw_AXI_REG->FifoStatus & 0x10000 )
      {
         ((uint32_t*)outPacket)[j] = pSpuHw_AXI_REG->FifoOutReg;
         j++;
      }
   }
   inputDataLen = ( inputDataLen + 3 ) / sizeof (uint32_t);
   /* Push the input data into the input FIFO */   
   for ( i = 0; i < inputDataLen; i++ )
   {
      while(!(pSpuHw_AXI_REG->FifoStatus) & 0x00001 );
      pSpuHw_AXI_REG->FifoInReg = ((uint32_t*)pInputData)[i];
      if( pSpuHw_AXI_REG->FifoStatus & 0x10000 )
      {
         ((uint32_t*)outPacket)[j] = pSpuHw_AXI_REG->FifoOutReg;
         j++;
      }
   }

   /* Push extra one word for status */
   while(!(pSpuHw_AXI_REG->FifoStatus) & 0x00001 );   
   pSpuHw_AXI_REG->FifoInReg = 0x00000000;

   /* Keep continue reading from the output FIFO */
   outputDataLen = ( outputDataLen + 3 ) / sizeof (uint32_t);
   while( j < outputDataLen )
   {
      while(!(pSpuHw_AXI_REG->FifoStatus & 0x10000) );
      ((uint32_t*)outPacket)[j] = pSpuHw_AXI_REG->FifoOutReg;
      j++;
   }

   /* Clear the interface */
   pSpuHw_AXI_REG->DmaStatus |= DMA_INTERFACE_CLEAR;
   pSpuHw_AXI_REG->DmaStatus &= ~DMA_INTERFACE_CLEAR;

   /* Check Error bit [17] in the output status word */
   return ( ( (uint32_t*)outPacket)[j-1] & spuHw_CMD_CRYPTO_STATUS_ERROR ) ? -1 : 0;
}
EXPORT_SYMBOL(spuHw_performCryptoOperation);
#endif



#ifndef BOOT0_BUILD
/****************************************************************************/
/**
*  @brief   Locate data within SPU packet received from SPU block
*
*  
*  @return   NULL                 - On failure
*            Begining of the data - On success 
*
*   @note
*    
*     Output SPU Packet:
*
*     +-----------------+------------------------------------+.......
*     | SPU Command     |   Data                             |       
*     +-----------------+------------------------------------+.......
*     ^                 ^
*     |                 |
*     outPacket         Begining of the data
*
*                       |<-------pDataLen------------------->|
*/
/****************************************************************************/
void* spuHw_getData 
( 
   spuHw_PACKET_t outPacket,      ///<  [IN] SPU packet received from SPU block
   size_t         *pDataLen       ///<  [OUT] Length of data in bytes
)
{
   uint32_t *ptr = (uint32_t*)outPacket;
   *pDataLen = 0;
   if ( ptr[0] & BD_PRESENT )
   {
      *pDataLen = ptr[2] >> 16;
      return (void*) &ptr[3];
   }
   else
   {
      return NULL;
   }
}
EXPORT_SYMBOL(spuHw_getData);
#endif

/****************************************************************************/
/**
*  @brief   Locate authentication result within SPU packet received from SPU block
*
*  
*  @return   NULL                                - On failure
*            Begining of the authentication data - On success 
*
*   @note
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
void* spuHw_getAuthenticationResult 
(
   spuHw_PACKET_t outPacket      ///<  [IN] SPU packet received from SPU block
)
{
   uint32_t *ptr = (uint32_t*)outPacket;
   if ( ( ( (ptr[0])) & (HASH_PRESENT | BD_PRESENT ) ) == (HASH_PRESENT | BD_PRESENT) )
   {
      int idx = ( ( ptr[2] >> 16 ) + 3 ) / 4;
      
      return (void*) &ptr[(spuHw_OUTPUT_HEADER_LEN / sizeof(uint32_t)) + idx];
   }
   else
   {
      return NULL;
   }
}

#ifndef BOOT0_BUILD
/****************************************************************************/
/**
*  @brief   Prints out the key table
*
*  
*  @return    -1 - On failure
*              0 - On success
*  @note
*     For debug purpose only
*     Host CPU must be in secure mode to allow this operation
*/
/****************************************************************************/
void spuHw_printKeyTable 
( 
   int (*fpPrint) ( const char *, ... )    ///< [ IN ] Print callback function
)
{
   uint32_t *cache = (uint32_t *) ( spuHw_APB_BASE_ADDRESS + spuHw_KEK_BASE_OFFSET ); 
   uint32_t tableSize = ( cache[0] / 4 ) - 1;
   uint32_t i;

   if (tableSize == 0 )
   {
      return;
   }

   (*fpPrint) ("Key table entries(%d):\n", tableSize);   
   (*fpPrint) ("Key buffer offset 0x%X\n", cache[0] & spuHw_CMD_KEY_OFFSET_MASK);
   for ( i = 1; i <= tableSize; i++ )
   {
      uint32_t entry = cache[i];
      /* Print offset */
     (*fpPrint) ("0x%03X: ", i * 4);
     /* Print valid */
     if ( entry & spuHw_CMD_KEY_VALLID )
     {
        (*fpPrint) ("V ");
     }
     else
     {
        (*fpPrint) ("  ");
     }
     /* Print secure */
     if ( entry & spuHw_CMD_KEY_SECURE )
     {
        (*fpPrint) ("K ");
     }
     else
     {
        (*fpPrint) ("  ");
     }
     /* Print type */
     (*fpPrint) ("0x%03X ", ( entry & spuHw_CMD_KEY_MASK ) >> spuHw_CMD_KEY_SHIFT );
     /* Print size */
     (*fpPrint) ("0x%03X ", ( entry & spuHw_CMD_KEY_SIZE_MASK) >> spuHw_CMD_KEY_SIZE_SHIFT );
     /* Print key offset */
     (*fpPrint) ("0x%03X \n", ( entry & spuHw_CMD_KEY_OFFSET_MASK) );
   }

   (*fpPrint) ("\nKey buffer:\n"); 
   for ( i = tableSize + 1; i < spuHw_KEK_CACHE_SIZE; i++ )
   {
      /* Print offset */
      (*fpPrint) ("0x%03X: 0x%08X\n", i * 4, cache[i] );
   }   
}
#endif

