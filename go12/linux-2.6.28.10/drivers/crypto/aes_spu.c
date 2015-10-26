/*****************************************************************************
*  Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
*
*  Unless you and Broadcom execute a separate written software license
*  agreement governing use of this software, this software is licensed to you
*  under the terms of the GNU General Public License version 2, available at
*  http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
*  Notwithstanding the above, under no circumstances may you combine this
*  software in any way with any other Broadcom software provided under a
*  license other than the GPL, without Broadcom's express prior written
*  consent.
*
*****************************************************************************/

/* 
 * Cryptographic API.
 *
 * AES Cipher Algorithm.
 *
 * Based on Brian Gladman's code.
 *
 * Linux developers:
 *  Alexander Kjeldaas <astor@fast.no>
 *  Herbert Valerio Riedel <hvr@hvrlab.org>
 *  Kyle McMartin <kyle@debian.org>
 *  Adam J. Richter <adam@yggdrasil.com> (conversion to 2.5 API).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * ---------------------------------------------------------------------------
 * Copyright (c) 2002, Dr Brian Gladman <brg@gladman.me.uk>, Worcester, UK.
 * All rights reserved.
 *
 * LICENSE TERMS
 *
 * The free distribution and use of this software in both source and binary
 * form is allowed (with or without changes) provided that:
 *
 *   1. distributions of this source code include the above copyright
 *      notice, this list of conditions and the following disclaimer;
 *
 *   2. distributions in binary form include the above copyright
 *      notice, this list of conditions and the following disclaimer
 *      in the documentation and/or other associated materials;
 *
 *   3. the copyright holder's name is not used to endorse products
 *      built using this software without specific written permission.
 *
 * ALTERNATIVELY, provided that this notice is retained in full, this product
 * may be distributed under the terms of the GNU General Public License (GPL),
 * in which case the provisions of the GPL apply INSTEAD OF those given above.
 *
 * DISCLAIMER
 *
 * This software is provided 'as is' with no explicit or implied warranties
 * in respect of its properties, including, but not limited to, correctness
 * and/or fitness for purpose.
 * ---------------------------------------------------------------------------
 */

#include <crypto/aes.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/crypto.h>
#include <asm/byteorder.h>

#include <linux/broadcom/bcm_spu.h>
#include <asm/arch/spuHw.h>
#include <asm/arch/spuHw_inline.h>
//#include <mach/csp/cap.h>

static spu_dma_context  crypto_dma;   /* DMA context for SPU */
static volatile atomic_t spu_dma_initialized;

static int initialize_dma(void)
{
   if ( !atomic_read(&spu_dma_initialized) )
   {
      /* Initialize DMA context settings */
      if ( spu_dma_context_init( &crypto_dma ) )
      {
         return 1;
      }   
  
      /* Perform DMA buffer allocation */
      if( spu_dma_alloc( &crypto_dma) )
      {
         return 1;
      }

	  atomic_set(&spu_dma_initialized, 1);
   }

   /* Associate DMA context to device handlers */
   if ( spu_dma_set_device_handlers( &crypto_dma ) )
   {
      return 1;
   }

   return 0;
}

/**
 * crypto_aes_spu_set_key - Set the AES key.
 * @tfm: The %crypto_tfm that is used in the context.
 * @in_key: The input key.
 * @key_len: The size of the key.
 *
 * Returns 0 on success, on failure the %CRYPTO_TFM_RES_BAD_KEY_LEN flag in tfm
 * is set.  Key expansion occurs on SPU.
 */
int crypto_aes_spu_set_key(struct crypto_tfm *tfm, const u8 *in_key, unsigned int key_len)
{
   struct crypto_aes_ctx *ctx = crypto_tfm_ctx(tfm);
   const __le32 *key = (const __le32 *)in_key;
   u32 *flags = &tfm->crt_flags;

   switch (key_len) {
      case AES_KEYSIZE_128:
         /* Copy 128 bits from key buffer */
         ctx->key_enc[0] = key[0];
         ctx->key_enc[1] = key[1];
         ctx->key_enc[2] = key[2];
         ctx->key_enc[3] = key[3];
         break;
      case AES_KEYSIZE_192:
         /* Copy 192 bits from key buffer */
         ctx->key_enc[0] = key[0];
         ctx->key_enc[1] = key[1];
         ctx->key_enc[2] = key[2];
         ctx->key_enc[3] = key[3];
         ctx->key_enc[4] = key[4];
         ctx->key_enc[5] = key[5];
         break;
      case AES_KEYSIZE_256:
         /* Copy 256 bits from key buffer */
         ctx->key_enc[0] = key[0];
         ctx->key_enc[1] = key[1];
         ctx->key_enc[2] = key[2];
         ctx->key_enc[3] = key[3];
         ctx->key_enc[4] = key[4];
         ctx->key_enc[5] = key[5];
         ctx->key_enc[6] = key[6];
         ctx->key_enc[7] = key[7];
         break;
      default:
         *flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
         return -EINVAL;
         break;
   }
   ctx->key_length = key_len;
   
   return 0;
}
EXPORT_SYMBOL_GPL(crypto_aes_spu_set_key);

static void aes_spu_encrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
   const struct crypto_aes_ctx *ctx = crypto_tfm_ctx(tfm);
   const unsigned int key_len = ctx->key_length;
   const __le32 *src = (const __le32 *)in;
   __le32 *dst = (__le32 *)out;

   int cmd_len_bytes;
   int output_len_bytes;

   u32 *cmd_buffer; 
   u32 *tmp_in_bufp;
   u32 *tmp_out_bufp;

   spuHw_CONTEXT_t aes_spu_context;

 /* Reserve channels for DMA transfer */
   spu_dma_reserve( &crypto_dma );

   /* Initialize DMA context and allocate memory */
   if( initialize_dma() != 0 )
	   return;

   /* Assign buffers for DMA */     
   cmd_buffer    = crypto_dma.crypto_cmd.virt; 
   tmp_in_bufp   = crypto_dma.crypto_in.virt;
   tmp_out_bufp  = crypto_dma.crypto_out.virt;
   
   memset ( (void*)&aes_spu_context, 0, sizeof(aes_spu_context) );   
  
   aes_spu_context.cryptoAlgo      = spuHw_CRYPTO_ALGO_AES;
   aes_spu_context.cryptoMode      = spuHw_CRYPTO_MODE_ECB;     /* Use ECB for atomic encrypt/decypt operations */
   aes_spu_context.authAlgo        = spuHw_CMD_AUTH_NULL;       /* Do not use authorization algorithm */
   aes_spu_context.dataAttribute.cryptoOffset = 0;
   aes_spu_context.dataAttribute.cryptoLength = AES_BLOCK_SIZE; /* Perform crypto on entire data block */
   aes_spu_context.dataAttribute.dataLength = (((AES_BLOCK_SIZE + 3) / sizeof(u32)) * sizeof(u32));

   switch (key_len) {
      case AES_KEYSIZE_128:
         aes_spu_context.cryptoType = spuHw_CRYPTO_TYPE_AES_K128;
         break;
      case AES_KEYSIZE_192:
         aes_spu_context.cryptoType = spuHw_CRYPTO_TYPE_AES_K192;
         break;
      case AES_KEYSIZE_256:
         aes_spu_context.cryptoType = spuHw_CRYPTO_TYPE_AES_K256;         
         break;
      default:
         /* Bail out.  Key length is validated with crypto_aes_spu_set_key() */
         return;
   }   

   /* Set operation to encrypt */
   aes_spu_context.operation         = spuHw_CRYPTO_OPERATION_ENCRYPTION;
   aes_spu_context.keyType           = spuHw_KEY_OPEN;
   aes_spu_context.cryptoKey         = (void *)ctx->key_enc;
   aes_spu_context.cryptoKeyLen      = (ctx->key_length / sizeof(u32));

   spu_request( 0 );

   cmd_len_bytes = spuHw_createCryptoCommand( &aes_spu_context, cmd_buffer );

   output_len_bytes =   spuHw_OUTPUT_HEADER_LEN + 
                        aes_spu_context.dataAttribute.dataLength + 
                        (aes_spu_context.icvLen * sizeof(u32)) +
                        spuHw_OUTPUT_STATUS_LEN;

   spuHw_setPacketLength( cmd_len_bytes + aes_spu_context.dataAttribute.dataLength, output_len_bytes );

   memset( tmp_in_bufp + (aes_spu_context.dataAttribute.dataLength / sizeof(u32)), 0, sizeof(u32) );
   memcpy( tmp_in_bufp, (void*)src, aes_spu_context.dataAttribute.dataLength );

 /* Reserve channels for DMA transfer */
//   spu_dma_reserve( &crypto_dma );

   /* configure the SPU DMA */   
   spu_dma_config(&crypto_dma, cmd_len_bytes, aes_spu_context.dataAttribute.dataLength, output_len_bytes);

   /* ignite the SPU FIFO for DMA */
   spuHw_initiateDma();

   /* Wait for DMA transfer to complete */
   spu_dma_wait(&crypto_dma);

   /* Free aquired DMA channels */
   spu_dma_free( &crypto_dma );   

   if (spuHw_getCryptoStatus( (spuHw_PACKET_t)tmp_out_bufp, output_len_bytes) == spuHw_CRYPTO_STATUS_SUCCESS )
   {
      /* Copy over output payload to destination */
      memcpy( dst, (tmp_out_bufp + (spuHw_OUTPUT_HEADER_LEN / sizeof(u32))), aes_spu_context.dataAttribute.dataLength );
   }

   spu_release();   
}

static void aes_spu_decrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
   const struct crypto_aes_ctx *ctx = crypto_tfm_ctx(tfm);
   const unsigned int key_len = ctx->key_length;
   const __le32 *src = (const __le32 *)in;
   __le32 *dst = (__le32 *)out;

   int cmd_len_bytes;
   int output_len_bytes;

   u32 *cmd_buffer;
   u32 *tmp_in_bufp;
   u32 *tmp_out_bufp;

   spuHw_CONTEXT_t aes_spu_context;

   /* Reserve channels for DMA transfer */
   spu_dma_reserve( &crypto_dma );

   /* Initialize DMA context and allocate memory */
   if( initialize_dma() != 0 )
	   return;

   /* Assign buffers for DMA */     
   cmd_buffer    = crypto_dma.crypto_cmd.virt; 
   tmp_in_bufp   = crypto_dma.crypto_in.virt;
   tmp_out_bufp  = crypto_dma.crypto_out.virt;

   memset ( (void*)&aes_spu_context, 0, sizeof(aes_spu_context) );   
  
   aes_spu_context.cryptoAlgo      = spuHw_CRYPTO_ALGO_AES;
   aes_spu_context.cryptoMode      = spuHw_CRYPTO_MODE_ECB;     /* Use ECB for atomic encrypt/decypt operations */
   aes_spu_context.authAlgo        = spuHw_CMD_AUTH_NULL;       /* Do not use authorization algorithm */
   aes_spu_context.dataAttribute.cryptoOffset = 0;
   aes_spu_context.dataAttribute.cryptoLength = AES_BLOCK_SIZE; /* Perform crypto on entire data block */
   aes_spu_context.dataAttribute.dataLength = (((AES_BLOCK_SIZE + 3) / sizeof(u32)) * sizeof(u32));

   switch (key_len) {
      case AES_KEYSIZE_128:
         aes_spu_context.cryptoType = spuHw_CRYPTO_TYPE_AES_K128;
         break;
      case AES_KEYSIZE_192:
         aes_spu_context.cryptoType = spuHw_CRYPTO_TYPE_AES_K192;
         break;
      case AES_KEYSIZE_256:
         aes_spu_context.cryptoType = spuHw_CRYPTO_TYPE_AES_K256;         
         break;
      default:
         /* Bail out.  Key length is validated with crypto_aes_spu_set_key() */
         return;
   }   

   /* Set operation to decrypt */
   aes_spu_context.operation         = spuHw_CRYPTO_OPERATION_DECRYPTION;
   aes_spu_context.keyType           = spuHw_KEY_OPEN;
   aes_spu_context.cryptoKey         = (void *)ctx->key_enc;
   aes_spu_context.cryptoKeyLen      = (ctx->key_length / sizeof(u32));

   spu_request( 0 );   

   cmd_len_bytes = spuHw_createCryptoCommand( &aes_spu_context, cmd_buffer );

   output_len_bytes = spuHw_OUTPUT_HEADER_LEN + 
                        aes_spu_context.dataAttribute.dataLength + 
                        (aes_spu_context.icvLen * sizeof(u32)) +
                        spuHw_OUTPUT_STATUS_LEN;

   spuHw_setPacketLength( cmd_len_bytes + aes_spu_context.dataAttribute.dataLength, output_len_bytes );

   memset( tmp_in_bufp + (aes_spu_context.dataAttribute.dataLength / sizeof(u32)), 0, sizeof(u32) );
   memcpy( tmp_in_bufp, (void*)src, aes_spu_context.dataAttribute.dataLength );

   /* Reserve channels for DMA transfer */
   //spu_dma_reserve( &crypto_dma );

   /* configure the SPU DMA */   
   spu_dma_config(&crypto_dma, cmd_len_bytes, aes_spu_context.dataAttribute.dataLength, output_len_bytes);

   /* ignite the SPU FIFO for DMA */
   spuHw_initiateDma();

   /* Wait for DMA transfer to complete */
   spu_dma_wait(&crypto_dma);

   /* Free aquired DMA channels */
   spu_dma_free( &crypto_dma );   

   if (spuHw_getCryptoStatus( (spuHw_PACKET_t)tmp_out_bufp, output_len_bytes) == spuHw_CRYPTO_STATUS_SUCCESS )
   {
      /* Copy over output payload to destination */
      memcpy( dst, (tmp_out_bufp + (spuHw_OUTPUT_HEADER_LEN / sizeof(u32))), aes_spu_context.dataAttribute.dataLength );
   }

   spu_release();   
}

static void aes_spu_hardware_init( void )
{
   /* Request and release will ensure hardware is initialized */
   spu_request( 0 );
   spu_release();

   atomic_set(&spu_dma_initialized, 0);
}

static struct crypto_alg aes_spu_alg = {
	.cra_name	   	=	"aes",
	.cra_driver_name	=	"aes-spu",
	.cra_priority		=	101,
	.cra_flags		   =	CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct crypto_aes_ctx),
	.cra_alignmask		=	3,
	.cra_module		   =	THIS_MODULE,
	.cra_list		   =	LIST_HEAD_INIT(aes_spu_alg.cra_list),
	.cra_u =	{
		.cipher = {
			.cia_min_keysize	=	AES_MIN_KEY_SIZE,
			.cia_max_keysize	=	AES_MAX_KEY_SIZE,
			.cia_setkey		   =	crypto_aes_spu_set_key,
			.cia_encrypt		=	aes_spu_encrypt,
			.cia_decrypt		=	aes_spu_decrypt
		}
	}
};

static int __init aes_spu_init(void)
{  
#if 0 /* 4760-OCF Port: */
   if (cap_isPresent(CAP_SPU,0) == CAP_NOT_PRESENT ) {
      printk (KERN_WARNING "AES SPU is not supported\n");
      return -EFAULT;
   }
#endif

   aes_spu_hardware_init();    
   return crypto_register_alg(&aes_spu_alg);
}

static void __exit aes_spu_fini(void)
{
   crypto_unregister_alg(&aes_spu_alg);

   /* Release DMA allocated buffers */
   spu_dma_dealloc( &crypto_dma );
   atomic_set(&spu_dma_initialized, 0);
}

subsys_initcall(aes_spu_init);
module_exit(aes_spu_fini);

MODULE_DESCRIPTION("Broadcom SPU-M Hardware Accelerated (AES) Cipher Algorithm");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("aes_spu");
