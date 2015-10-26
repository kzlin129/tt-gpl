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
*  @file    spuHw.h
*
*  @brief   API definitions for low level Security Processor Unit module
*
*/
/****************************************************************************/
#ifndef SPUHW_H
#define SPUHW_H

#include <asm/arch/spuHw_reg.h>

typedef void     *spuHw_PACKET_t;     /* SPU packet (SPU command + data) */
typedef uint32_t spuHw_KEY_INDEX_t;   /* Key index into the key table */
typedef uint32_t spuHw_CONTEXT_ID_t;  /* Context ID of a packet */

typedef enum
{
  spuHw_CRYPTO_OPERATION_ENCRYPTION = spuHw_CMD_CRYPTO_ENCRYPTION,  /* Perform encryption operation */
  spuHw_CRYPTO_OPERATION_DECRYPTION = spuHw_CMD_CRYPTO_DECRYPTION   /* Perform decryption operation */
}spuHw_CRYPTO_OPERTION_e;


typedef enum
{
  spuHw_CRYPTO_ALGO_NULL          = spuHw_CMD_CRYPTO_NULL,   /* Perform NULL encryption algorithm */
  spuHw_CRYPTO_ALGO_RC4           = spuHw_CMD_CRYPTO_RC4 ,   /* Perform RC4 encryption algorithm */
  spuHw_CRYPTO_ALGO_DES           = spuHw_CMD_CRYPTO_DES,    /* Perform DES encryption algorithm */
  spuHw_CRYPTO_ALGO_3DES          = spuHw_CMD_CRYPTO_3DES,   /* Perform 3DES encryption algorithm */
  spuHw_CRYPTO_ALGO_AES           = spuHw_CMD_CRYPTO_AES     /* Perform AES encryption algorithm */
}spuHw_CRYPTO_ALGO_e;


typedef enum
{
  spuHw_CRYPTO_MODE_ECB           = spuHw_CMD_CMODE_ECB,
  spuHw_CRYPTO_MODE_CBC           = spuHw_CMD_CMODE_CBC,
  spuHw_CRYPTO_MODE_CTR           = spuHw_CMD_CMODE_CTR,
  spuHw_CRYPTO_MODE_GCM           = spuHw_CMD_CMODE_GCM,
}spuHw_CRYPTO_MODE_e;

typedef enum
{
  spuHw_CRYPTO_TYPE_NULL_NONE     = spuHw_CMD_COPTYPE_INIT,
  spuHw_CRYPTO_TYPE_RC4_INIT      = spuHw_CMD_COPTYPE_INIT,
  spuHw_CRYPTO_TYPE_RC4_UPDATE    = spuHw_CMD_COPTYPE_UPDATE,
  spuHw_CRYPTO_TYPE_DES_K56       = spuHw_CMD_COPTYPE_K56,
  spuHw_CRYPTO_TYPE_3DES_K168     = spuHw_CMD_COPTYPE_K168,
  spuHw_CRYPTO_TYPE_AES_K128      = spuHw_CMD_COPTYPE_K128,
  spuHw_CRYPTO_TYPE_AES_K192      = spuHw_CMD_COPTYPE_K192,
  spuHw_CRYPTO_TYPE_AES_K256      = spuHw_CMD_COPTYPE_K256
}spuHw_CRYPTO_TYPE_e; 


typedef enum
{
  spuHw_AUTH_ALGO_NULL            = spuHw_CMD_AUTH_NULL,     /* Perform NULL authentication/hash algorithm*/
  spuHw_AUTH_ALGO_MD5             = spuHw_CMD_AUTH_MD5,      /* Perform MD5 authentication/hash algorithm*/
  spuHw_AUTH_ALGO_SHA1            = spuHw_CMD_AUTH_SHA1,     /* Perform SHA1 authentication/hash algorithm*/
  spuHw_AUTH_ALGO_SHA224          = spuHw_CMD_AUTH_SHA224,   /* Perform SHA224 authentication/hash algorithm*/
  spuHw_AUTH_ALGO_SHA256          = spuHw_CMD_AUTH_SHA256,   /* Perform SHA256 authentication/hash algorithm*/
  spuHw_AUTH_ALGO_AES             = spuHw_CMD_AUTH_AES       /* Perform AES authentication/hash algorithm for AES-GCM */
}spuHw_AUTH_ALGO_e;


typedef enum
{
  spuHw_AUTH_MODE_HASH           = spuHw_CMD_AMODE_HASH,
  spuHw_AUTH_MODE_CTXT           = spuHw_CMD_AMODE_CTXT,
  spuHw_AUTH_MODE_HMAC           = spuHw_CMD_AMODE_HMAC,
  spuHw_AUTH_MODE_FHMAC          = spuHw_CMD_AMODE_FHMAC,
  spuHw_AUTH_MODE_GCM            = spuHw_CMD_AMODE_GCM,
}spuHw_AUTH_MODE_e;


typedef enum
{
  spuHw_AUTH_TYPE_NULL_NONE       = spuHw_CMD_AOPTYPE_NONE,
  spuHw_AUTH_TYPE_MD5_FULL        = spuHw_CMD_AOPTYPE_FULL,
  spuHw_AUTH_TYPE_MD5_INIT        = spuHw_CMD_AOPTYPE_INIT,
  spuHw_AUTH_TYPE_MD5_UPDATE      = spuHw_CMD_AOPTYPE_UPDATE,
  spuHw_AUTH_TYPE_MD5_FINAL       = spuHw_CMD_AOPTYPE_FIN,
  spuHw_AUTH_TYPE_SHA1_FULL       = spuHw_CMD_AOPTYPE_FULL,
  spuHw_AUTH_TYPE_SHA1_INIT       = spuHw_CMD_AOPTYPE_INIT,
  spuHw_AUTH_TYPE_SHA1_UPDATE     = spuHw_CMD_AOPTYPE_UPDATE,
  spuHw_AUTH_TYPE_SHA1_FINAL      = spuHw_CMD_AOPTYPE_FIN,
  spuHw_AUTH_TYPE_SHA224_FULL     = spuHw_CMD_AOPTYPE_FULL,
  spuHw_AUTH_TYPE_SHA256_FULL     = spuHw_CMD_AOPTYPE_FULL,
  spuHw_AUTH_TYPE_AES_K128        = spuHw_CMD_AOPTYPE_AES_K128
}spuHw_AUTH_TYPE_e; 


typedef enum
{
  spuHw_AUTH_ORDER_FIRST          = spuHw_CMD_AUTH_FIRST,    /* Authentication is performed first. Encryption is done afterwards  */
  spuHw_AUTH_ORDER_LAST           = spuHw_CMD_AUTH_LAST      /* Encryption is performed first. Authentication is done over the encrypted data */
}spuHw_AUTH_ORDER_e;


typedef enum
{
  spuHw_KEY_TYPE_DES_CBC          = spuHw_CMD_KEY_DES_CBC, 
  spuHw_KEY_TYPE_DES_ECB          = spuHw_CMD_KEY_DES_CBC, 
  spuHw_KEY_TYPE_3DES_CBC         = spuHw_CMD_KEY_3DES_CBC,
  spuHw_KEY_TYPE_3DES_ECB         = spuHw_CMD_KEY_3DES_CBC,
  spuHw_KEY_TYPE_AES128_CBC       = spuHw_CMD_KEY_AES128_CBC, 
  spuHw_KEY_TYPE_AES128_ECB       = spuHw_CMD_KEY_AES128_ECB,
  spuHw_KEY_TYPE_AES192_CBC       = spuHw_CMD_KEY_AES192_CBC, 
  spuHw_KEY_TYPE_AES192_ECB       = spuHw_CMD_KEY_AES192_ECB,
  spuHw_KEY_TYPE_AES256_CBC       = spuHw_CMD_KEY_AES256_CBC,
  spuHw_KEY_TYPE_AES256_ECB       = spuHw_CMD_KEY_AES256_ECB,
  spuHw_KEY_TYPE_AES128_CTR       = spuHw_CMD_KEY_AES128_CTR,
  spuHw_KEY_TYPE_AES192_CTR       = spuHw_CMD_KEY_AES192_CTR,
  spuHw_KEY_TYPE_AES256_CTR       = spuHw_CMD_KEY_AES256_CTR,
  spuHw_KEY_TYPE_HMAC_SHA1        = spuHw_CMD_KEY_HMAC_SHA1,
  spuHw_KEY_TYPE_HMAC_MD5         = spuHw_CMD_KEY_HMAC_MD5,
  spuHw_KEY_TYPE_HMAC_SHA224      = spuHw_CMD_KEY_HMAC_SHA224,
  spuHw_KEY_TYPE_HMAC_SHA256      = spuHw_CMD_KEY_HMAC_SHA256,
  spuHw_KEY_TYPE_FHMAC_SHA1       = spuHw_CMD_KEY_FHMAC_SHA1,
  spuHw_KEY_TYPE_FHMAC_MD5        = spuHw_CMD_KEY_FHMAC_MD5,
  spuHw_KEY_TYPE_CTXT_SHA1        = spuHw_CMD_KEY_CTXT_SHA1,
  spuHw_KEY_TYPE_CTXT_MD5         = spuHw_CMD_KEY_CTXT_MD5,
  spuHw_KEY_TYPE_RC4              = spuHw_CMD_KEY_RC4,
  spuHw_KEY_TYPE_HASH_UPDT_SHA1   = spuHw_CMD_KEY_HASH_UPDT_SHA1,
  spuHw_KEY_TYPE_HASH_UPDT_MD5    = spuHw_CMD_KEY_HASH_UPDT_MD5,
  spuHw_KEY_TYPE_HASH_UPDT_SHA224 = spuHw_CMD_KEY_HASH_UPDT_SHA224,
  spuHw_KEY_TYPE_HASH_UPDT_SHA256 = spuHw_CMD_KEY_HASH_UPDT_SHA256,
  spuHw_KEY_TYPE_HASH_FIN_SHA1    = spuHw_CMD_KEY_HASH_FIN_SHA1,
  spuHw_KEY_TYPE_HASH_FIN_MD5     = spuHw_CMD_KEY_HASH_FIN_MD5,
  spuHw_KEY_TYPE_HASH_FIN_SHA224  = spuHw_CMD_KEY_HASH_FIN_SHA224,
  spuHw_KEY_TYPE_HASH_FIN_SHA256  = spuHw_CMD_KEY_HASH_FIN_SHA256
}spuHw_KEY_TYPE_e;


typedef enum
{
  spuHw_KEY_MODE_SECURE           = spuHw_CMD_KEY_SECURE,    /* Key to be used when SPU (not host CPU) is in secure mode */
  spuHw_KEY_MODE_OPEN             = spuHw_CMD_KEY_OPEN       /* Key to be used when SPU (not host CPU) is in open mode */
}spuHw_KEY_MODE_e;

typedef enum
{
  spuHw_STATUS_READ_ACCESS_ERROR,         /* Read access permission failure */
  spuHw_STATUS_WRITE_ACCESS_ERROR,        /* Write access permission failure */
  spuHw_STATUS_SUCCESS                    /* Crypto processing is done successfully */
}spuHw_STATUS_e;

typedef enum
{
  spuHw_CRYPTO_STATUS_SUCCESS            = spuHw_CMD_CRYPTO_STATUS_SUCCESS,             /* Error free status */
  spuHw_CRYPTO_STATUS_INVALID_KEY_HANDLE = spuHw_CMD_CRYPTO_STATUS_INVALID_KEY_HANDLE,  /* The key handle does not match */
  spuHw_CRYPTO_STATUS_INVALID_ICV        = spuHw_CMD_CRYPTO_STATUS_INVALID_ICV,         /* ICV comparison failed */
  spuHw_CRYPTO_STATUS_INVALID_KEY        = spuHw_CMD_CRYPTO_STATUS_INVALID_KEY,         /* The key type requested doesn't match */
  spuHw_CRYPTO_STATUS_UNKNOWN            = spuHw_CMD_CRYPTO_STATUS_UNKNOWN              /* Unknown error */
}spuHw_CRYPTO_STATUS_e;


typedef enum
{
  spuHw_KEY_OPEN      = 0,
  spuHw_KEY_PROTECTED = spuHw_CMD_SCTX_KEY_PROTECT
}spuHw_KEY_e;


/* Data type for spuHw_KEY_t defining chip specific cryptographic key */
typedef struct
{
  spuHw_KEY_TYPE_e    type;                /* Key type */
  spuHw_KEY_MODE_e    mode;                /* Key to be used when SPU (not host CPU) is in secure/open mode */
  uint32_t            *key;                /* Cryptographic Key */
  size_t              size;                /* Key size in words */
  char                valid;               /* Key entry valid(1)/invalid(0) */
}spuHw_KEY_t;


/* Data type describing data */
typedef struct
{
   unsigned short  dataLength;             /* Total data length in bytes */   
   unsigned short  macOffset;              /* Authentication operation to be performed at */
   unsigned short  macLength;              /* Length of the authentication data in bytes */
   unsigned short  cryptoOffset;           /* Crypto operation to be performed at */
   unsigned short  cryptoLength;           /* Length of the crypto data in bytes */
   unsigned short  aesGcmAadLength;        /* AES-GCM Authentication only data length in bytes

                                              |<-------------128 bit aligned ---------------------------->|<------ 128 bit aligned -------->| 
                                              |<--------- aesGcmAadLength ----------------->|<--. pad..-->|<--... Payload ...-->|<-- pad -->|
                                                         |<----- SecTAG-------------------->|
                                              +-----+----+------------+-----+----+----+-----+
                                              | DA  | SA | Ether type | TCI | SL | PN | SCI | .....       |<-- cryptoLength  -->|
                                              +-----+----+------------+-----+----+----+-----+             
                                              macOffset = 0
                                              |------------ cryptoOffset -------------------------------->|
                                              |<-------------------------- macLength ------------------------------------------------------>|
                                              |<-------------------------- dataLength ----------------------------------------------------->|

                                            */
}spuHw_DATA_ATTRIB_t;


/* Data type for spuHw_CONTEXT_t defining chip specific crypto context */
typedef struct
{
  spuHw_CONTEXT_ID_t       contextId;       /* Packet context id */
  spuHw_CRYPTO_OPERTION_e  operation;       /* Cryptographic operation */
  spuHw_CRYPTO_ALGO_e      cryptoAlgo;      /* Cryptographic algorithm */
  spuHw_CRYPTO_MODE_e      cryptoMode;      /* Cryptographic mode */
  spuHw_CRYPTO_TYPE_e      cryptoType;      /* Cryptographic operation type */
  spuHw_AUTH_ALGO_e        authAlgo;        /* Authentication/Hash algorithm */
  spuHw_AUTH_MODE_e        authMode;        /* Authentication/Hash mode */
  spuHw_AUTH_TYPE_e        authType;        /* Authentication/Hash type */
  spuHw_AUTH_ORDER_e       authOrder;       /* Authentication order */

  spuHw_KEY_e              keyType;         /* Protected or open key */
  spuHw_KEY_INDEX_t        keyHandle;       /* Key handle (Key entry), when "keyType" is spuHw_KEY_PROTECTED */
  void*                    authKey;         /* Authentication key (MAC key), when "keyType" is spuHw_KEY_OPEN */
  uint32_t                 authKeyLen;      /* Authentication key length in words, when "keyType" is spuHw_KEY_OPEN */
  void*                    cryptoKey;       /* Crypto key (Cihper key), when "keyType" is spuHw_KEY_OPEN */
  uint32_t                 cryptoKeyLen;    /* Crypto key length in words, when "keyType" is spuHw_KEY_OPEN */

  uint32_t                 keyUpdate;       /* Offset within the key to update the state */

  void*                    initVector;      /* Initialize vector */
  uint32_t                 initVectorLen;   /* Initialize vector length in words */
  
  uint32_t                 icvLen;          /* ICV (Integrity Check Value ~ HASH) length in word */
  spuHw_DATA_ATTRIB_t      dataAttribute;   /* Data attribute */
}spuHw_CONTEXT_t;


#ifdef __cplusplus
extern "C"
{
#endif

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
);

/****************************************************************************/
/**
*  @brief   Sets misc. key table configuration options
*
*  
*  @return    -1 - On failure
*              0 - On success
*  @note
*     - Host CPU must be in secure mode to allow this operation
*     - Please see spuHw_reg.h for available key table configuration 
*       options (spuHw_KEY_CONFIG_XXXXX)
*/
/****************************************************************************/
static inline int spuHw_configKeyTable 
( 
   uint32_t options  ///< Key table configuration options
);

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
*        Host CPU must be in secure mode to allow this operation
*/
/****************************************************************************/
int spuHw_setKey 
( 
   spuHw_KEY_INDEX_t  index,             ///< [IN] Key position (index) in the table 
   spuHw_KEY_t        *pKey              ///< [IN] Cryptographic key
);

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
);

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
static inline void spuHw_initDevice ( void );



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
);

/****************************************************************************/
/**
*  @brief   Set secure data access to SPU blcok
*
*  
*  @return    -1 - On failure
*              0 - On success
*/
/****************************************************************************/
static inline int spuHw_setSecureAccess ( void );

/****************************************************************************/
/**
*  @brief   Set only the secure write access to SPU blcok
*
*  
*  @return    -1 - On failure
*              0 - On success
*/
/****************************************************************************/
static inline int spuHw_setSecureWriteAccess ( void );

/****************************************************************************/
/**
*  @brief   Set both secure and non-secure data access to SPU blcok
*
*  
*  @return    -1 - On failure
*              0 - On success
*/
/****************************************************************************/
static inline int spuHw_setOpenAccess ( void );

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
);


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
);


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
*                           |<------- inputDataLen ------------->|
*
*
*     Output SPU Packet:
*     
*     |<-------------------- outputDataLen ----------------->|
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
   int              dataLen,        ///< [IN] Input data length in bytes
   spuHw_PACKET_t   outPacket,      ///< [OUT] SPU packet received from SPU block   
   int              outputDataLen   ///< [IN] Expected SPU packet length in bytes
);
#endif

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
static inline uint32_t spuHw_getInputFifoAddress ( void );

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
static inline uint32_t spuHw_getOutputFifoAddress ( void );

/****************************************************************************/
/**
*  @brief   Sets input and output packet length to begin SPU processing.
*
*  
*  @return  -1 : If fails the set the data length (channel is busy)
*            0 : Successful
*
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
   size_t inLen,          ///< [IN]  Input SPU packet length in bytes
   size_t outLen          ///< [IN]  Expected output (PTM) packet length in bytes
);

/****************************************************************************/
/**
*  @brief   Initiates DMA operation
*
*  @note
*           This function should be called after setting up the DMA channel
*           using external DMA device
*/
/****************************************************************************/
static inline void spuHw_initiateDma ( void );

/****************************************************************************/
/**
*  @brief   Clears DMA operation
*
*  @note
*           This function should be called before setting up the DMA channel
*           using external DMA device
*/
/****************************************************************************/
//static inline void spuHw_clearDma ( void );


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
static inline int spuHw_processingDone ( void );

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
static inline spuHw_STATUS_e spuHw_getProcessingStatus ( void );

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
);

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
);



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
);

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
*/
/****************************************************************************/
static inline spuHw_CONTEXT_ID_t spuHw_getContextId 
(
   spuHw_PACKET_t outPacket      ///<  [IN] SPU packet received from SPU block
);

/****************************************************************************/
/**
*  @brief   Enable interrupt for SPU error notification
*
*  
*  @return  void
*/
/****************************************************************************/
static inline void spuHw_setErrorInterruptEnable ( void );

/****************************************************************************/
/**
*  @brief   Disable interrupt for SPU error notification
*
*  
*  @return void
*/
/****************************************************************************/
static inline void spuHw_setErrorInterruptDisable ( void );

/****************************************************************************/
/**
*  @brief   Clear interrupt for SPU error notification
*
*  
*  @return  void
*/
/****************************************************************************/
static inline void spuHw_clearErrorInterrupt ( void );


#ifdef __cplusplus
extern "C"
}
#endif

#endif /* SPUHW_H */
















