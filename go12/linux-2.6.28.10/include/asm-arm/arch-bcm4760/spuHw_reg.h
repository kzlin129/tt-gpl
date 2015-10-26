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
*  @file    spuHw_reg.h
*
*  @brief   API definitions for low level Security Processor Unit registers
*
*/
/****************************************************************************/
#ifndef SPUHW_REG_H
#define SPUHW_REG_H

//#include <mach/csp/mm_io.h> 
#include <asm/arch/hardware.h>
#include <asm/arch/bcm4760_reg.h>


typedef struct
{
  uint32_t Reaction;             /* AXI Reaction */
  uint32_t InterruptStatus;      /* SPUM Interrupt Status */
  uint32_t InterruptEnable;      /* SPUM Interrupt Enable */
  uint32_t SecFailedAddress;     /* Security Failed Address */
  uint32_t SecFailedStatus;      /* Info of the failed access */
  uint32_t AccessPermission;     /* Setting access permission */
  uint32_t PrivDebug1;           /* Private register for Debug Only */
  uint32_t PrivDebug2;           /* Private register for Debug Only */
  uint32_t DmaControl;           /* Setting DMA Flow Control */
  uint32_t DmaStatus;            /* DMA Status */
  uint32_t DmaSize;              /* Setting DMA transfer size */
  uint32_t FifoStatus;           /* IN/OUT FIFO Status */
  uint32_t FifoInReg;            /* FIFO IN Register */
  uint32_t FifoOutReg;           /* FIFO OUT Register */
}spuhw_AXI_REG_t;

typedef struct
{
  uint32_t Control;               /* SPUM Control */
  uint32_t reserved1;             /* Reserved */
  uint32_t reserved2;             /* Reserved */
  uint32_t reserved3;             /* Reserved */
  uint32_t KekControl;            /* KEK Cache Control */
  uint32_t KekBase;             	/* KEK cache memory base address */ 
  uint32_t reserved4;             /* Reserved */
  uint32_t reserved5;             /* Reserved */
  uint32_t DataProcess;           /* Data Processing Control */ 
}spuhw_APB_REG_t;


//#define spuHw_AXI_BASE_ADDRESS             MM_IO_BASE_SPUM
#define spuHw_AXI_BASE_ADDRESS             IO_ADDRESS(SPM_AHB_REG_BASE_ADDR)
//#define spuHw_APB_BASE_ADDRESS             MM_IO_BASE_SPUMP
#define spuHw_APB_BASE_ADDRESS             IO_ADDRESS(SPM_APB_REG_BASE_ADDR)
#define spuHw_IN_FIFO_ADDR                 spuHw_AXI_BASE_ADDRESS + 0x00008000    /* SPU Input FIFO address for external DMA to use */
#define spuHw_IN_FIFO_PHYS_ADDR            SPM_AHB_REG_BASE_ADDR + 0x00008000     /* SPU Input FIFO phys address for external DMA to use */
#define spuHw_OUT_FIFO_ADDR                spuHw_AXI_BASE_ADDRESS + 0x0000C000    /* SPU Output FIFO address for external DMA to use */
#define spuHw_OUT_FIFO_PHYS_ADDR           SPM_AHB_REG_BASE_ADDR + 0x0000C000     /* SPU Output FIFO phys address for external DMA to use */

#define spuHw_KEK_BASE_OFFSET              0x00000800
#define spuHw_KEK_CACHE_SIZE               128           /* Number of words in KEK cache table */


/* Misc. SPU device configuration options for spuHw_configDevice() */
#define spuHw_DEV_CONFIG_OPEN              0xC0000000    /* SPU is in open mode */
#define spuHw_DEV_CONFIG_SECURE            0x80000000    /* SPU is in secure mode */
#define spuHw_DEV_CONFIG_OUTPUT_LITTLE     0x00001000    /* Output data to little endian format */
#define spuHw_DEV_CONFIG_INPUT_LITTLE      0x00000800    /* Input data is little endian format */


#define spuHw_DEV_CONFIG_HASH_DISABLE      0x00000400    /* Disable HAS engine */
#define spuHw_DEV_CONFIG_DES_DISABLE       0x00000200    /* Disable DES engine */
#define spuHw_DEV_CONFIG_AES_DISABLE       0x00000100    /* Disable AES engine */
#define spuHw_DEV_CONFIG_RC4_DISABLE       0x00000080    /* Disable RC4 engine */
#define spuHw_DEV_CONFIG_SPU_DISABLE       0x00000001    /* Disable entire SPU  */
 
/* Misc. cryptographic key configuration options for spuHw_configKeyTable() */
#define spuHw_KEY_CONFIG_ENABLE            0x00800000    /* Enable key block */
#define spuHw_KEY_CONFIG_READ_DISABLE      0x00400000    /* Disable reading key from key table */
 

#define spuHw_CMD_CRYPTO_ENCRYPTION        0x00000000
#define spuHw_CMD_CRYPTO_DECRYPTION        0x80000000

/* Misc. key entry defines */
#define spuHw_CMD_KEY_SIZE_SHIFT           12
#define spuHw_CMD_KEY_SIZE_MASK            ( 0x000001FF << spuHw_CMD_KEY_SIZE_SHIFT )
#define spuHw_CMD_KEY_VALID                0x80000000 
#define spuHw_CMD_KEY_OFFSET_MASK          0x00000FFF

/* Misc. key type defines (cache key) */    
#define spuHw_CMD_KEY_SHIFT                21
#define spuHw_CMD_KEY_MASK                 ( 0x000000FF << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_DES_CBC              ( 0x00000000 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_3DES_CBC             ( 0x00000001 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_AES128_CBC           ( 0x00000002 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_AES128_ECB           ( 0x00000002 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_AES192_CBC           ( 0x00000003 << spuHw_CMD_KEY_SHIFT ) 
#define spuHw_CMD_KEY_AES192_ECB           ( 0x00000003 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_AES256_CBC           ( 0x00000004 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_AES256_ECB           ( 0x00000004 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_AES128_CTR           ( 0x00000005 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_AES192_CTR           ( 0x00000006 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_AES256_CTR           ( 0x00000007 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_HMAC_SHA1            ( 0x00000040 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_HMAC_MD5             ( 0x00000041 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_HMAC_SHA224          ( 0x00000042 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_HMAC_SHA256          ( 0x00000043 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_FHMAC_SHA1           ( 0x00000044 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_FHMAC_MD5            ( 0x00000045 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_CTXT_SHA1            ( 0x00000048 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_CTXT_MD5             ( 0x00000049 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_RC4                  ( 0x00000050 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_HASH_UPDT_SHA1       ( 0x00000052 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_HASH_UPDT_MD5        ( 0x00000053 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_HASH_UPDT_SHA224     ( 0x00000054 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_HASH_UPDT_SHA256     ( 0x00000055 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_HASH_FIN_SHA1        ( 0x00000056 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_HASH_FIN_MD5         ( 0x00000057 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_HASH_FIN_SHA224      ( 0x00000058 << spuHw_CMD_KEY_SHIFT )
#define spuHw_CMD_KEY_HASH_FIN_SHA256      ( 0x00000059 << spuHw_CMD_KEY_SHIFT )

/* Misc. key mode defines (cache key) */
#define spuHw_CMD_KEY_SECURE               0x40000000
#define spuHw_CMD_KEY_OPEN                 0x00000000
#define spuHw_CMD_KEY_VALLID               0x80000000

/* Misc. crypto algorithms */
#define spuHw_CMD_CRYPTO_SHIFT             21
#define spuHw_CMD_CRYPTO_MASK              ( 0x00000007 << spuHw_CMD_CRYPTO_SHIFT )
#define spuHw_CMD_CRYPTO_NULL              ( 0x00000000 << spuHw_CMD_CRYPTO_SHIFT )
#define spuHw_CMD_CRYPTO_RC4               ( 0x00000001 << spuHw_CMD_CRYPTO_SHIFT ) 
#define spuHw_CMD_CRYPTO_DES               ( 0x00000002 << spuHw_CMD_CRYPTO_SHIFT )
#define spuHw_CMD_CRYPTO_3DES              ( 0x00000003 << spuHw_CMD_CRYPTO_SHIFT )
#define spuHw_CMD_CRYPTO_AES               ( 0x00000004 << spuHw_CMD_CRYPTO_SHIFT )

/* Misc. crypto modes */
#define spuHw_CMD_CMODE_SHIFT              18
#define spuHw_CMD_CMODE_MASK               ( 0x00000007 << spuHw_CMD_CMODE_SHIFT )
#define spuHw_CMD_CMODE_ECB                ( 0x00000000 << spuHw_CMD_CMODE_SHIFT )
#define spuHw_CMD_CMODE_CBC                ( 0x00000001 << spuHw_CMD_CMODE_SHIFT )
#define spuHw_CMD_CMODE_CTR                ( 0x00000004 << spuHw_CMD_CMODE_SHIFT )
#define spuHw_CMD_CMODE_GCM                ( 0x00000006 << spuHw_CMD_CMODE_SHIFT )


/* Misc. crypto operation types */
#define spuHw_CMD_COPTYPE_SHIFT            16
#define spuHw_CMD_COPTYPE_MASK             ( 0x00000003 << spuHw_CMD_COPTYPE_SHIFT )
#define spuHw_CMD_COPTYPE_INIT             ( 0x00000000 << spuHw_CMD_COPTYPE_SHIFT )
#define spuHw_CMD_COPTYPE_K56              ( 0x00000000 << spuHw_CMD_COPTYPE_SHIFT )
#define spuHw_CMD_COPTYPE_K168             ( 0x00000000 << spuHw_CMD_COPTYPE_SHIFT )
#define spuHw_CMD_COPTYPE_K128             ( 0x00000000 << spuHw_CMD_COPTYPE_SHIFT )
#define spuHw_CMD_COPTYPE_UPDATE           ( 0x00000001 << spuHw_CMD_COPTYPE_SHIFT )
#define spuHw_CMD_COPTYPE_K192             ( 0x00000001 << spuHw_CMD_COPTYPE_SHIFT )
#define spuHw_CMD_COPTYPE_K256             ( 0x00000002 << spuHw_CMD_COPTYPE_SHIFT )

#define spuHw_CMD_AUTH_FIRST               0x40000000
#define spuHw_CMD_AUTH_LAST                0x00000000

/* Misc. authentication algorithms */
#define spuHw_CMD_AUTH_SHIFT               13
#define spuHw_CMD_AUTH_MASK                ( 0x00000007 << spuHw_CMD_AUTH_SHIFT )
#define spuHw_CMD_AUTH_NULL                ( 0x00000000 << spuHw_CMD_AUTH_SHIFT )
#define spuHw_CMD_AUTH_MD5                 ( 0x00000001 << spuHw_CMD_AUTH_SHIFT )
#define spuHw_CMD_AUTH_SHA1                ( 0x00000002 << spuHw_CMD_AUTH_SHIFT )
#define spuHw_CMD_AUTH_SHA224              ( 0x00000003 << spuHw_CMD_AUTH_SHIFT )
#define spuHw_CMD_AUTH_SHA256              ( 0x00000004 << spuHw_CMD_AUTH_SHIFT )
#define spuHw_CMD_AUTH_AES                 ( 0x00000005 << spuHw_CMD_AUTH_SHIFT )


/* Misc. authetication modes */
#define spuHw_CMD_AMODE_SHIFT              10
#define spuHw_CMD_AMODE_MASK               ( 0x00000007 << spuHw_CMD_AMODE_SHIFT )
#define spuHw_CMD_AMODE_HASH               ( 0x00000000 << spuHw_CMD_AMODE_SHIFT )
#define spuHw_CMD_AMODE_CTXT               ( 0x00000001 << spuHw_CMD_AMODE_SHIFT )
#define spuHw_CMD_AMODE_HMAC               ( 0x00000002 << spuHw_CMD_AMODE_SHIFT )
#define spuHw_CMD_AMODE_FHMAC              ( 0x00000006 << spuHw_CMD_AMODE_SHIFT )
#define spuHw_CMD_AMODE_GCM                ( 0x00000006 << spuHw_CMD_AMODE_SHIFT )

/* Misc. authetication operation types */
#define spuHw_CMD_AOPTYPE_SHIFT            8
#define spuHw_CMD_AOPTYPE_MASK             ( 0x00000003 << spuHw_CMD_AOPTYPE_SHIFT )
#define spuHw_CMD_AOPTYPE_NONE             ( 0x00000000 << spuHw_CMD_AOPTYPE_SHIFT )
#define spuHw_CMD_AOPTYPE_FULL             ( 0x00000000 << spuHw_CMD_AOPTYPE_SHIFT )
#define spuHw_CMD_AOPTYPE_INIT             ( 0x00000001 << spuHw_CMD_AOPTYPE_SHIFT )
#define spuHw_CMD_AOPTYPE_UPDATE           ( 0x00000002 << spuHw_CMD_AOPTYPE_SHIFT )
#define spuHw_CMD_AOPTYPE_FIN              ( 0x00000003 << spuHw_CMD_AOPTYPE_SHIFT )
#define spuHw_CMD_AOPTYPE_AES_K128         ( 0x00000000 << spuHw_CMD_AOPTYPE_SHIFT )

#define spuHw_CMD_SCTX_KEY_PROTECT         0x80000000

#define spuHw_CMD_SCTX_KEY_HANDLE_SHIFT    20
#define spuHw_CMD_SCTX_KEY_HANDLE_MASK     ( 0x000001FF << spuHw_CMD_SCTX_KEY_HANDLE_SHIFT ) 

#define spuHw_CMD_SCTX_ICV_INSERT          0x00002000
#define spuHw_CMD_SCTX_ICV_CHECK           0x00001000
#define spuHw_CMD_SCTX_ICV_SIZE_SHIFT      8
#define spuHw_CMD_SCTX_ICV_SIZE_MASK       ( 0x0000000F << spuHw_CMD_SCTX_ICV_SIZE_SHIFT )

#define spuHw_CMD_SCTX_IV_CONTEXT          0x00000080
#define spuHw_CMD_SCTX_IV_EXPLICIT         0x00000040
#define spuHw_CMD_SCTX_IV_GENERATE         0x00000020

#define spuHw_CMD_SCTX_IV_OFFSET_SHIFT     3
#define spuHw_CMD_SCTX_IV_OFFSET_MASK      ( 0x00000003 << spuHw_CMD_SCTX_IV_SHIFT ) 

#define spuHw_CMD_SCTX_IV_EXPLICIT_SIZE    0x00000007

#define spuHw_CMD_AUTH_RESULT_ENABLE       0x04000000  /* Generate authentication result  */   
#define spuHw_CMD_AUTH_RESULT_DISABLE      0x00000000  /* Do not generate authentication result  */   


#define spuHw_CMD_CRYPTO_STATUS_MASK                  0x0000FF00
#define spuHw_CMD_CRYPTO_STATUS_SUCCESS               0x00000000
#define spuHw_CMD_CRYPTO_STATUS_INVALID_KEY_HANDLE    0x00000300
#define spuHw_CMD_CRYPTO_STATUS_INVALID_ICV           0x00000400
#define spuHw_CMD_CRYPTO_STATUS_INVALID_KEY           0x00000200
#define spuHw_CMD_CRYPTO_STATUS_UNKNOWN               0x0000FF00       
#define spuHw_CMD_CRYPTO_STATUS_ERROR                 0x00020000


#define spuHw_OUTPUT_HEADER_LEN            12          /* Total SPU output header in bytes */
#define spuHw_OUTPUT_STATUS_LEN            4           /* Out status length in bytes */

#define pSpuHw_AXI_REG ((volatile spuhw_AXI_REG_t *) spuHw_AXI_BASE_ADDRESS)
#define pSpuHw_APB_REG ((volatile spuhw_APB_REG_t *) spuHw_APB_BASE_ADDRESS)

#define pSpuHw_DMA_FIFO_IN                  (spuHw_AXI_BASE_ADDRESS + 0x8000) 
#define pSpuHw_DMA_FIFO_OUT                 (spuHw_AXI_BASE_ADDRESS + 0xC000) 


#endif /* SPUHW_REG_H */

