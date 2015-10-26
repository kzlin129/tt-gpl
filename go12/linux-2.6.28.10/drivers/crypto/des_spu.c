/*****************************************************************************
*  Copyright 2001 - 2009 Broadcom Corporation.  All rights reserved.
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
 * DES & Triple DES EDE Cipher Algorithms.
 *
 * Copyright (c) 2005 Dag Arne Osvik <da@osvik.no>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <asm/byteorder.h>
#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/crypto.h>
#include <linux/types.h>

#include <crypto/des.h>

#include <linux/broadcom/bcm_spu.h>
#include <asm/arch/spuHw.h>
#include <asm/arch/spuHw_inline.h>
//#include <mach/csp/cap.h>

#define DES_KEY_SIZE_WORDS                ((DES_KEY_SIZE + 3) / sizeof(u32))
#define DES3_KEY_SIZE_WORDS               ((DES3_KEY_SIZE + 3) / sizeof(u32))

static spu_dma_context  crypto_dma;   /* DMA context for SPU */
static volatile atomic_t spu_dma_initialized;

struct des_spu_ctx {
	u32 key[DES_KEY_SIZE];
};

struct des3_spu_ede_ctx {
	u32 key[DES3_EDE_KEY_SIZE];
};

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

static int des_spu_setkey(struct crypto_tfm *tfm, const u8 *key,
		      unsigned int keylen)
{
	struct des_spu_ctx *dctx = crypto_tfm_ctx(tfm);

   /* Copy to output */
   memcpy(dctx->key, key, keylen);
	return 0;
}

static void des_spu_encrypt(struct crypto_tfm *tfm, u8 *dst, const u8 *src)
{
	struct des_spu_ctx *dctx = crypto_tfm_ctx(tfm);
	const __le32 *s = (const __le32 *)src;
	__le32 *d = (__le32 *)dst;
   int cmd_len_bytes;
   int output_len_bytes;

   u32 *cmd_buffer;
   u32 *tmp_in_bufp;
   u32 *tmp_out_bufp;

   spuHw_CONTEXT_t des_spu_context;


   /* Reserve channels for DMA transfer */
   spu_dma_reserve( &crypto_dma );

   /* Initialize DMA context and allocate memory */
   if( initialize_dma() != 0 )
	   return;

   /* Assign buffers for DMA */
   cmd_buffer    = crypto_dma.crypto_cmd.virt;
   tmp_in_bufp   = crypto_dma.crypto_in.virt;
   tmp_out_bufp  = crypto_dma.crypto_out.virt;

   memset ( (void*)&des_spu_context, 0, sizeof(des_spu_context) );

   des_spu_context.cryptoAlgo      = spuHw_CRYPTO_ALGO_DES;
   des_spu_context.cryptoMode      = spuHw_CRYPTO_MODE_ECB;     /* Use ECB for atomic encrypt/decypt operations */
   des_spu_context.cryptoType      = spuHw_CRYPTO_TYPE_DES_K56;
   des_spu_context.authAlgo        = spuHw_CMD_AUTH_NULL;       /* Do not use authorization algorithm */
   des_spu_context.dataAttribute.cryptoOffset = 0;
   des_spu_context.dataAttribute.cryptoLength = DES_BLOCK_SIZE; /* Perform crypto on entire data block */
   des_spu_context.dataAttribute.dataLength = (((DES_BLOCK_SIZE + 3) / sizeof(u32)) * sizeof(u32));

   /* Set operation to encrypt */
   des_spu_context.operation         = spuHw_CRYPTO_OPERATION_ENCRYPTION;
   des_spu_context.keyType           = spuHw_KEY_OPEN;
   des_spu_context.cryptoKey         = (void *)dctx->key;
   des_spu_context.cryptoKeyLen      = (DES_KEY_SIZE / sizeof(u32));

   spu_request( 0 );

   cmd_len_bytes = spuHw_createCryptoCommand( &des_spu_context, cmd_buffer );

   output_len_bytes =   spuHw_OUTPUT_HEADER_LEN +
                        des_spu_context.dataAttribute.dataLength +
                        (des_spu_context.icvLen * sizeof(u32)) +
                        spuHw_OUTPUT_STATUS_LEN;


   spuHw_setPacketLength( cmd_len_bytes + des_spu_context.dataAttribute.dataLength, output_len_bytes );

   memset( tmp_in_bufp + (des_spu_context.dataAttribute.dataLength / sizeof(u32)), 0, sizeof(u32) );
   memcpy( tmp_in_bufp, (void*)s, des_spu_context.dataAttribute.dataLength );

   /* Reserve channels for DMA transfer */
   //spu_dma_reserve( &crypto_dma );

   /* configure the SPU DMA */
   spu_dma_config(&crypto_dma, cmd_len_bytes, des_spu_context.dataAttribute.dataLength, output_len_bytes);

   /* ignite the SPU FIFO for DMA */
   spuHw_initiateDma();

   /* Wait for DMA transfer to complete */
   spu_dma_wait(&crypto_dma);

   /* Free aquired DMA channels */
   spu_dma_free( &crypto_dma );

   if (spuHw_getCryptoStatus( (spuHw_PACKET_t)tmp_out_bufp, output_len_bytes) == spuHw_CRYPTO_STATUS_SUCCESS )
   {
      /* Copy over output payload to destination */
      memcpy( d, (tmp_out_bufp + (spuHw_OUTPUT_HEADER_LEN / sizeof(u32))), des_spu_context.dataAttribute.dataLength );
   }

   spu_release();

}

static void des_spu_decrypt(struct crypto_tfm *tfm, u8 *dst, const u8 *src)
{
	struct des_spu_ctx *dctx = crypto_tfm_ctx(tfm);
	const __le32 *s = (const __le32 *)src;
	__le32 *d = (__le32 *)dst;
   int cmd_len_bytes;
   int output_len_bytes;

   u32 *cmd_buffer;
   u32 *tmp_in_bufp;
   u32 *tmp_out_bufp;

   spuHw_CONTEXT_t des_spu_context;

   /* Reserve channels for DMA transfer */
   spu_dma_reserve( &crypto_dma );

   /* Initialize DMA context and allocate memory */
   if( initialize_dma() != 0 )
	   return;

   /* Assign buffers for DMA */
   cmd_buffer    = crypto_dma.crypto_cmd.virt;
   tmp_in_bufp   = crypto_dma.crypto_in.virt;
   tmp_out_bufp  = crypto_dma.crypto_out.virt;

   memset ( (void*)&des_spu_context, 0, sizeof(des_spu_context) );

   des_spu_context.cryptoAlgo      = spuHw_CRYPTO_ALGO_DES;
   des_spu_context.cryptoMode      = spuHw_CRYPTO_MODE_ECB;     /* Use ECB for atomic encrypt/decypt operations */
   des_spu_context.cryptoType      = spuHw_CRYPTO_TYPE_DES_K56;
   des_spu_context.authAlgo        = spuHw_CMD_AUTH_NULL;       /* Do not use authorization algorithm */
   des_spu_context.dataAttribute.cryptoOffset = 0;
   des_spu_context.dataAttribute.cryptoLength = DES_BLOCK_SIZE; /* Perform crypto on entire data block */
   des_spu_context.dataAttribute.dataLength = (((DES_BLOCK_SIZE + 3) / sizeof(u32)) * sizeof(u32));

   /* Set operation to encrypt */
   des_spu_context.operation         = spuHw_CRYPTO_OPERATION_DECRYPTION;
   des_spu_context.keyType           = spuHw_KEY_OPEN;
   des_spu_context.cryptoKey         = (void *)dctx->key;
   des_spu_context.cryptoKeyLen      = (DES_KEY_SIZE / sizeof(u32));

   spu_request( 0 );

   cmd_len_bytes = spuHw_createCryptoCommand( &des_spu_context, cmd_buffer );

   output_len_bytes =   spuHw_OUTPUT_HEADER_LEN +
                        des_spu_context.dataAttribute.dataLength +
                        (des_spu_context.icvLen * sizeof(u32)) +
                        spuHw_OUTPUT_STATUS_LEN;

   spuHw_setPacketLength( cmd_len_bytes + des_spu_context.dataAttribute.dataLength, output_len_bytes );

   memset( tmp_in_bufp + (des_spu_context.dataAttribute.dataLength / sizeof(u32)), 0, sizeof(u32) );
   memcpy( tmp_in_bufp, (void*)s, des_spu_context.dataAttribute.dataLength );

   /* Reserve channels for DMA transfer */
   //spu_dma_reserve( &crypto_dma );

   /* configure the SPU DMA */
   spu_dma_config(&crypto_dma, cmd_len_bytes, des_spu_context.dataAttribute.dataLength, output_len_bytes);

   /* ignite the SPU FIFO for DMA */
   spuHw_initiateDma();

   /* Wait for DMA transfer to complete */
   spu_dma_wait(&crypto_dma);

   /* Free aquired DMA channels */
   spu_dma_free( &crypto_dma );

   if (spuHw_getCryptoStatus( (spuHw_PACKET_t)tmp_out_bufp, output_len_bytes) == spuHw_CRYPTO_STATUS_SUCCESS )
   {
      /* Copy over output payload to destination */
      memcpy( d, (tmp_out_bufp + (spuHw_OUTPUT_HEADER_LEN / sizeof(u32))), des_spu_context.dataAttribute.dataLength );
   }

   spu_release();
}

/*
 * RFC2451:
 *
 *   For DES-EDE3, there is no known need to reject weak or
 *   complementation keys.  Any weakness is obviated by the use of
 *   multiple keys.
 *
 *   However, if the first two or last two independent 64-bit keys are
 *   equal (k1 == k2 or k2 == k3), then the DES3 operation is simply the
 *   same as DES.  Implementers MUST reject keys that exhibit this
 *   property.
 *
 */
static int des3_spu_ede_setkey(struct crypto_tfm *tfm, const u8 *key,
			   unsigned int keylen)
{
	const u32 *K = (const u32 *)key;
	struct des3_spu_ede_ctx *dctx = crypto_tfm_ctx(tfm);
	u32 *ctx_key = dctx->key;
	u32 *flags = &tfm->crt_flags;

	if (unlikely(!((K[0] ^ K[2]) | (K[1] ^ K[3])) ||
		     !((K[2] ^ K[4]) | (K[3] ^ K[5]))))
	{
		*flags |= CRYPTO_TFM_RES_BAD_KEY_SCHED;
		return -EINVAL;
	}
   memcpy( ctx_key, key, keylen );
	return 0;
}

static void des3_spu_ede_encrypt(struct crypto_tfm *tfm, u8 *dst, const u8 *src)
{
	struct des3_spu_ede_ctx *dctx = crypto_tfm_ctx(tfm);
	const __le32 *s = (const __le32 *)src;
	__le32 *d = (__le32 *)dst;
   int cmd_len_bytes;
   int output_len_bytes;

   u32 *cmd_buffer;
   u32 *tmp_in_bufp;
   u32 *tmp_out_bufp;

   spuHw_CONTEXT_t des3_spu_context;

   /* Reserve channels for DMA transfer */
   spu_dma_reserve( &crypto_dma );

   /* Initialize DMA context and allocate memory */
   if( initialize_dma() != 0 )
	   return;

   /* Assign buffers for DMA */
   cmd_buffer    = crypto_dma.crypto_cmd.virt;
   tmp_in_bufp   = crypto_dma.crypto_in.virt;
   tmp_out_bufp  = crypto_dma.crypto_out.virt;

   memset ( (void*)&des3_spu_context, 0, sizeof(des3_spu_context) );

   des3_spu_context.cryptoAlgo      = spuHw_CRYPTO_ALGO_3DES;
   des3_spu_context.cryptoMode      = spuHw_CRYPTO_MODE_ECB;     /* Use ECB for atomic encrypt/decypt operations */
   des3_spu_context.cryptoType      = spuHw_CRYPTO_TYPE_3DES_K168;
   des3_spu_context.authAlgo        = spuHw_CMD_AUTH_NULL;       /* Do not use authorization algorithm */
   des3_spu_context.dataAttribute.cryptoOffset = 0;
   des3_spu_context.dataAttribute.cryptoLength = DES3_EDE_BLOCK_SIZE; /* Perform crypto on entire data block */
   des3_spu_context.dataAttribute.dataLength = (((DES3_EDE_BLOCK_SIZE + 3) / sizeof(u32)) * sizeof(u32));

   /* Set operation to encrypt */
   des3_spu_context.operation         = spuHw_CRYPTO_OPERATION_ENCRYPTION;
   des3_spu_context.keyType           = spuHw_KEY_OPEN;
   des3_spu_context.cryptoKey         = (void *)dctx->key;
   des3_spu_context.cryptoKeyLen      = (DES3_EDE_KEY_SIZE / sizeof(u32));

   spu_request( 0 );

   cmd_len_bytes = spuHw_createCryptoCommand( &des3_spu_context, cmd_buffer );

  output_len_bytes =   spuHw_OUTPUT_HEADER_LEN +
                        des3_spu_context.dataAttribute.dataLength +
                        (des3_spu_context.icvLen * sizeof(u32)) +
                        spuHw_OUTPUT_STATUS_LEN;

   spuHw_setPacketLength( cmd_len_bytes + des3_spu_context.dataAttribute.dataLength, output_len_bytes );

   memset( tmp_in_bufp + (des3_spu_context.dataAttribute.dataLength / sizeof(u32)), 0, sizeof(u32) );
   memcpy( tmp_in_bufp, (void*)s, des3_spu_context.dataAttribute.dataLength );

   /* Reserve channels for DMA transfer */
   //spu_dma_reserve( &crypto_dma );

   /* configure the SPU DMA */
   spu_dma_config(&crypto_dma, cmd_len_bytes, des3_spu_context.dataAttribute.dataLength, output_len_bytes);

   /* ignite the SPU FIFO for DMA */
   spuHw_initiateDma();

   /* Wait for DMA transfer to complete */
   spu_dma_wait(&crypto_dma);

   /* Free aquired DMA channels */
   spu_dma_free( &crypto_dma );

   if (spuHw_getCryptoStatus( (spuHw_PACKET_t)tmp_out_bufp, output_len_bytes) == spuHw_CRYPTO_STATUS_SUCCESS )
   {
      /* Copy over output payload to destination */
      memcpy( d, (tmp_out_bufp + (spuHw_OUTPUT_HEADER_LEN / sizeof(u32))), des3_spu_context.dataAttribute.dataLength );
   }

   spu_release();
}

static void des3_spu_ede_decrypt(struct crypto_tfm *tfm, u8 *dst, const u8 *src)
{
	struct des3_spu_ede_ctx *dctx = crypto_tfm_ctx(tfm);
	const __le32 *s = (const __le32 *)src;
	__le32 *d = (__le32 *)dst;

   int cmd_len_bytes;
   int output_len_bytes;

   u32 *cmd_buffer;
   u32 *tmp_in_bufp;
   u32 *tmp_out_bufp;

   spuHw_CONTEXT_t des3_spu_context;

   /* Reserve channels for DMA transfer */
   spu_dma_reserve( &crypto_dma );

   /* Initialize DMA context and allocate memory */
   if( initialize_dma() != 0 )
	   return;

   /* Assign buffers for DMA */
   cmd_buffer    = crypto_dma.crypto_cmd.virt;
   tmp_in_bufp   = crypto_dma.crypto_in.virt;
   tmp_out_bufp  = crypto_dma.crypto_out.virt;

   memset ( (void*)&des3_spu_context, 0, sizeof(des3_spu_context) );

   des3_spu_context.cryptoAlgo      = spuHw_CRYPTO_ALGO_3DES;
   des3_spu_context.cryptoMode      = spuHw_CRYPTO_MODE_ECB;     /* Use ECB for atomic encrypt/decypt operations */
   des3_spu_context.cryptoType      = spuHw_CRYPTO_TYPE_3DES_K168;
   des3_spu_context.authAlgo        = spuHw_CMD_AUTH_NULL;       /* Do not use authorization algorithm */
   des3_spu_context.dataAttribute.cryptoOffset = 0;
   des3_spu_context.dataAttribute.cryptoLength = DES3_EDE_BLOCK_SIZE; /* Perform crypto on entire data block */
   des3_spu_context.dataAttribute.dataLength = (((DES3_EDE_BLOCK_SIZE + 3) / sizeof(u32)) * sizeof(u32));

   /* Set operation to encrypt */
   des3_spu_context.operation         = spuHw_CRYPTO_OPERATION_DECRYPTION;
   des3_spu_context.keyType           = spuHw_KEY_OPEN;
   des3_spu_context.cryptoKey         = (void *)dctx->key;
   des3_spu_context.cryptoKeyLen      = (DES3_EDE_KEY_SIZE / sizeof(u32));

   spu_request( 0 );

   cmd_len_bytes = spuHw_createCryptoCommand( &des3_spu_context, cmd_buffer );

  output_len_bytes =   spuHw_OUTPUT_HEADER_LEN +
                        des3_spu_context.dataAttribute.dataLength +
                        (des3_spu_context.icvLen * sizeof(u32)) +
                        spuHw_OUTPUT_STATUS_LEN;

   spuHw_setPacketLength( cmd_len_bytes + des3_spu_context.dataAttribute.dataLength, output_len_bytes );

   memset( tmp_in_bufp + (des3_spu_context.dataAttribute.dataLength / sizeof(u32)), 0, sizeof(u32) );
   memcpy( tmp_in_bufp, (void*)s, des3_spu_context.dataAttribute.dataLength );

   /* Reserve channels for DMA transfer */
   //spu_dma_reserve( &crypto_dma );

   /* configure the SPU DMA */
   spu_dma_config(&crypto_dma, cmd_len_bytes, des3_spu_context.dataAttribute.dataLength, output_len_bytes);

   /* ignite the SPU FIFO for DMA */
   spuHw_initiateDma();

   /* Wait for DMA transfer to complete */
   spu_dma_wait(&crypto_dma);

   /* Free aquired DMA channels */
   spu_dma_free( &crypto_dma );

   if (spuHw_getCryptoStatus( (spuHw_PACKET_t)tmp_out_bufp, output_len_bytes) == spuHw_CRYPTO_STATUS_SUCCESS )
   {
      /* Copy over output payload to destination */
      memcpy( d, (tmp_out_bufp + (spuHw_OUTPUT_HEADER_LEN / sizeof(u32))), des3_spu_context.dataAttribute.dataLength );
   }

   spu_release();
}

static struct crypto_alg des_spu_alg = {
	.cra_name		=	"des",
	.cra_driver_name	=	"des-spu",
	.cra_flags		=	CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		=	DES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct des_spu_ctx),
	.cra_module		=	THIS_MODULE,
	.cra_alignmask		=	3,
	.cra_list		=	LIST_HEAD_INIT(des_spu_alg.cra_list),
	.cra_u			=	{ .cipher = {
	.cia_min_keysize	=	DES_KEY_SIZE,
	.cia_max_keysize	=	DES_KEY_SIZE,
	.cia_setkey		=	des_spu_setkey,
	.cia_encrypt		=	des_spu_encrypt,
	.cia_decrypt		=	des_spu_decrypt } }
};

static struct crypto_alg des3_spu_ede_alg = {
	.cra_name		=	"des3_ede",
	.cra_driver_name	=	"des3_ede-spu",
	.cra_flags		=	CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		=	DES3_EDE_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct des3_spu_ede_ctx),
	.cra_module		=	THIS_MODULE,
	.cra_alignmask		=	3,
	.cra_list		=	LIST_HEAD_INIT(des3_spu_ede_alg.cra_list),
	.cra_u			=	{ .cipher = {
	.cia_min_keysize	=	DES3_EDE_KEY_SIZE,
	.cia_max_keysize	=	DES3_EDE_KEY_SIZE,
	.cia_setkey		=	des3_spu_ede_setkey,
	.cia_encrypt		=	des3_spu_ede_encrypt,
	.cia_decrypt		=	des3_spu_ede_decrypt } }
};

static int __init des_spu_mod_init(void)
{
   int ret;
#if 0 /* 4760-OCF Port: */
   if (cap_isPresent(CAP_SPU,0) == CAP_NOT_PRESENT ) {
      printk (KERN_WARNING "DES SPU is not supported\n");
      return -EFAULT;
   }
#endif
	 
	ret = 0;

	ret = crypto_register_alg(&des_spu_alg);
	if (ret < 0)
		goto out;

	ret = crypto_register_alg(&des3_spu_ede_alg);
	if (ret < 0)
		crypto_unregister_alg(&des_spu_alg);

    atomic_set(&spu_dma_initialized, 0);

out:
	return ret;
}

static void __exit des_spu_mod_fini(void)
{
	crypto_unregister_alg(&des3_spu_ede_alg);
	crypto_unregister_alg(&des_spu_alg);

	/* Release DMA allocated buffers */
    spu_dma_dealloc( &crypto_dma );
    atomic_set(&spu_dma_initialized, 0);
}

subsys_initcall(des_spu_mod_init);
module_exit(des_spu_mod_fini);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Broadcom SPU-M Hardware accelerated DES & Triple DES EDE Cipher Algorithms");
MODULE_ALIAS("des_spu");
