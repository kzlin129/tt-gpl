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
 * MD5 Message Digest Algorithm (RFC1321).
 *
 * Derived from cryptoapi implementation, originally based on the
 * public domain implementation written by Colin Plumb in 1993.
 *
 * Copyright (c) Cryptoapi developers.
 * Copyright (c) 2002 James Morris <jmorris@intercode.com.au>
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/crypto.h>
#include <linux/types.h>
#include <asm/byteorder.h>

#include <linux/broadcom/bcm_spu.h>
#include <asm/arch/spuHw.h>
#include <asm/arch/spuHw_inline.h>
//#include <mach/csp/cap.h>

#define MD5_DIGEST_SIZE		   16
#define MD5_HMAC_BLOCK_SIZE	64
#define MD5_BLOCK_WORDS		   16
#define MD5_HASH_WORDS		   4

static spu_dma_context  crypto_dma;   /* DMA context for SPU */
static volatile atomic_t spu_dma_initialized;

struct md5_spu_ctx {
	u32 hash[MD5_HASH_WORDS];
	u32 block[MD5_BLOCK_WORDS];
	u64 byte_count;
   u32 init_performed;
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

static void md5_spu_transform_init(u32 *digest, const char *data)
{
   int cmd_len_bytes;
   int output_len_bytes;

   u32 *cmd_buffer;
   u32 *tmp_in_bufp;
   u32 *tmp_out_bufp;

   spuHw_CONTEXT_t md5_spu_context;

   /* Reserve channels for DMA transfer */
   spu_dma_reserve( &crypto_dma );
	 
   /* Initialize DMA context and allocate memory */
   if( initialize_dma() != 0 )
	   return;

   /* Assign buffers for DMA */     
   cmd_buffer    = crypto_dma.crypto_cmd.virt; 
   tmp_in_bufp   = crypto_dma.crypto_in.virt;
   tmp_out_bufp  = crypto_dma.crypto_out.virt;

   memset ( (void*)&md5_spu_context, 0, sizeof(md5_spu_context) );
   
   md5_spu_context.operation                = spuHw_CRYPTO_OPERATION_ENCRYPTION;
   md5_spu_context.cryptoAlgo               = spuHw_CRYPTO_ALGO_NULL;
   md5_spu_context.authAlgo                 = spuHw_AUTH_ALGO_MD5;
   md5_spu_context.authMode                 = spuHw_AUTH_MODE_HASH;
   md5_spu_context.authType                 = spuHw_AUTH_TYPE_MD5_INIT;
   md5_spu_context.authOrder                = spuHw_CMD_AUTH_FIRST;
   md5_spu_context.keyType                  = spuHw_KEY_OPEN;
   md5_spu_context.icvLen                   = MD5_HASH_WORDS;

   md5_spu_context.dataAttribute.macOffset  = 0;
   md5_spu_context.dataAttribute.macLength  = MD5_HMAC_BLOCK_SIZE;   
   md5_spu_context.dataAttribute.dataLength = (((MD5_HMAC_BLOCK_SIZE + 3) / sizeof(u32)) * sizeof(u32));

   spu_request( 0 );

   cmd_len_bytes = spuHw_createCryptoCommand( &md5_spu_context, cmd_buffer );

   output_len_bytes =   spuHw_OUTPUT_HEADER_LEN +
                        md5_spu_context.dataAttribute.dataLength +
                        (md5_spu_context.icvLen * sizeof(u32)) +
                        spuHw_OUTPUT_STATUS_LEN;

   spuHw_setPacketLength( cmd_len_bytes + md5_spu_context.dataAttribute.dataLength, output_len_bytes );

   memset( tmp_in_bufp + (md5_spu_context.dataAttribute.dataLength / sizeof(u32)), 0, sizeof(u32) );
   memcpy( tmp_in_bufp, (void*)data, md5_spu_context.dataAttribute.dataLength );

   /* Reserve channels for DMA transfer */
   //spu_dma_reserve( &crypto_dma );

   /* configure the SPU DMA */   
   spu_dma_config(&crypto_dma, cmd_len_bytes, md5_spu_context.dataAttribute.dataLength, output_len_bytes);

   /* ignite the SPU FIFO for DMA */
   spuHw_initiateDma();

   /* Wait for DMA transfer to complete */
   spu_dma_wait(&crypto_dma);

   /* Free aquired DMA channels */
   spu_dma_free( &crypto_dma );   

   if (spuHw_getCryptoStatus( (spuHw_PACKET_t)tmp_out_bufp, output_len_bytes) == spuHw_CRYPTO_STATUS_SUCCESS )
   {
      /* Copy over output payload to destination */
      memcpy(  digest, 
               tmp_out_bufp + ((spuHw_OUTPUT_HEADER_LEN + md5_spu_context.dataAttribute.dataLength + 3 ) / sizeof(u32)),
               md5_spu_context.dataAttribute.dataLength );        
   }

   spu_release();
}

static void md5_spu_transform_update(u32 *digest, const char *data)
{
   int cmd_len_bytes;
   int output_len_bytes;

   u32 *cmd_buffer;
   u32 *tmp_in_bufp;
   u32 *tmp_out_bufp;

   spuHw_CONTEXT_t md5_spu_context;

   /* Reserve channels for DMA transfer */
   spu_dma_reserve( &crypto_dma );

   /* Initialize DMA context and allocate memory */
   if( initialize_dma() != 0 )
	   return;

   /* Assign buffers for DMA */     
   cmd_buffer    = crypto_dma.crypto_cmd.virt; 
   tmp_in_bufp   = crypto_dma.crypto_in.virt;
   tmp_out_bufp  = crypto_dma.crypto_out.virt;

   memset ( (void*)&md5_spu_context, 0, sizeof(md5_spu_context) );

   md5_spu_context.operation                = spuHw_CRYPTO_OPERATION_ENCRYPTION;
   md5_spu_context.cryptoAlgo               = spuHw_CRYPTO_ALGO_NULL;
   md5_spu_context.authAlgo                 = spuHw_AUTH_ALGO_MD5;
   md5_spu_context.authMode                 = spuHw_AUTH_MODE_HASH;
   md5_spu_context.authType                 = spuHw_AUTH_TYPE_MD5_UPDATE;
   md5_spu_context.authOrder                = spuHw_CMD_AUTH_FIRST;
   md5_spu_context.keyType                  = spuHw_KEY_OPEN;

   md5_spu_context.authKey                  = (void *)digest;
   md5_spu_context.authKeyLen               = MD5_HASH_WORDS;
   
   md5_spu_context.icvLen                   = MD5_HASH_WORDS;
   md5_spu_context.dataAttribute.macOffset  = 0;
   md5_spu_context.dataAttribute.macLength  = MD5_HMAC_BLOCK_SIZE;
   md5_spu_context.dataAttribute.dataLength = (((MD5_HMAC_BLOCK_SIZE + 3) / sizeof(u32)) * sizeof(u32));

   spu_request( 0 );

   cmd_len_bytes = spuHw_createCryptoCommand( &md5_spu_context, cmd_buffer );

   output_len_bytes =   spuHw_OUTPUT_HEADER_LEN +
                        md5_spu_context.dataAttribute.dataLength +
                        (md5_spu_context.icvLen * sizeof(u32)) +
                        spuHw_OUTPUT_STATUS_LEN;

   spuHw_setPacketLength( cmd_len_bytes + md5_spu_context.dataAttribute.dataLength, output_len_bytes );

   memset( tmp_in_bufp + (md5_spu_context.dataAttribute.dataLength / sizeof(u32)), 0, sizeof(u32) );
   memcpy( tmp_in_bufp, (void*)data, md5_spu_context.dataAttribute.dataLength );

   /* Reserve channels for DMA transfer */
   //spu_dma_reserve( &crypto_dma );

   /* configure the SPU DMA */   
   spu_dma_config(&crypto_dma, cmd_len_bytes, md5_spu_context.dataAttribute.dataLength, output_len_bytes);

   /* ignite the SPU FIFO for DMA */
   spuHw_initiateDma();

   /* Wait for DMA transfer to complete */
   spu_dma_wait(&crypto_dma);

   /* Free aquired DMA channels */
   spu_dma_free( &crypto_dma );   

   if (spuHw_getCryptoStatus( (spuHw_PACKET_t)tmp_out_bufp, output_len_bytes) == spuHw_CRYPTO_STATUS_SUCCESS )
   {
      /* Copy over output payload to destination */
      memcpy(  digest, 
               tmp_out_bufp + ((spuHw_OUTPUT_HEADER_LEN + md5_spu_context.dataAttribute.dataLength + 3 ) / sizeof(u32)),
               md5_spu_context.dataAttribute.dataLength );        
   }

   spu_release();
}

static void md5_spu_transform(struct md5_spu_ctx *ctx)
{
   if( ctx->init_performed == 0 )
   {
      md5_spu_transform_init( ctx->hash, (char *)ctx->block );
      ctx->init_performed = 1;
   }
   else
   {
      md5_spu_transform_update( ctx->hash, (char *)ctx->block );
   }
}

static void md5_spu_init(struct crypto_tfm *tfm)
{
	struct md5_spu_ctx *mctx = crypto_tfm_ctx(tfm);

	mctx->hash[0] = 0x67452301;
	mctx->hash[1] = 0xefcdab89;
	mctx->hash[2] = 0x98badcfe;
	mctx->hash[3] = 0x10325476;
	mctx->byte_count = 0;
   mctx->init_performed = 0;
}

static void md5_spu_update(struct crypto_tfm *tfm, const u8 *data, unsigned int len)
{
	struct md5_spu_ctx *mctx = crypto_tfm_ctx(tfm);
	const u32 avail = sizeof(mctx->block) - (mctx->byte_count & 0x3f);

	mctx->byte_count += len;

	if (avail > len) {
		memcpy((char *)mctx->block + (sizeof(mctx->block) - avail),
		       data, len);
		return;
	}

	memcpy((char *)mctx->block + (sizeof(mctx->block) - avail),
	       data, avail);
   md5_spu_transform( mctx );
	data += avail;
	len -= avail;

	while (len >= sizeof(mctx->block)) {
		memcpy(mctx->block, data, sizeof(mctx->block));
      md5_spu_transform_update( mctx->hash, (char *)mctx->block );
		data += sizeof(mctx->block);
		len -= sizeof(mctx->block);
	}
	memcpy(mctx->block, data, len);
}

static void md5_spu_final(struct crypto_tfm *tfm, u8 *out)
{
	struct md5_spu_ctx *mctx = crypto_tfm_ctx(tfm);
	const unsigned int offset = mctx->byte_count & 0x3f;
	char *p = (char *)mctx->block + offset;
	int padding = 56 - (offset + 1);
   int i;

	*p++ = 0x80;
	if (padding < 0) {
		memset(p, 0x00, padding + sizeof (u64));
      md5_spu_transform( mctx );
      
		p = (char *)mctx->block;
		padding = 56;
	}

	memset(p, 0, padding);
	mctx->block[14] = mctx->byte_count << 3;
	mctx->block[15] = mctx->byte_count >> 29;
   md5_spu_transform( mctx );

   for ( i = 0; i < MD5_HASH_WORDS; i++ )
   {
      mctx->hash[i] = be32_to_cpu( mctx->hash[i]);
   }
   
	memcpy(out, mctx->hash, sizeof(mctx->hash));
	memset(mctx, 0, sizeof(*mctx));
}

static void md5_spu_hardware_init( void )
{
   spu_request( 0 );
   spu_release();

   atomic_set(&spu_dma_initialized, 0);
}

static struct crypto_alg alg = {
	.cra_name	      =	"md5",
	.cra_driver_name	=	"md5-spu",
   .cra_priority     =  101,
	.cra_flags	      =	CRYPTO_ALG_TYPE_DIGEST,
	.cra_blocksize	   =	MD5_HMAC_BLOCK_SIZE,
	.cra_ctxsize	   =	sizeof(struct md5_spu_ctx),
	.cra_module	      =	THIS_MODULE,
	.cra_list	      =	LIST_HEAD_INIT(alg.cra_list),
	.cra_u =	{
      .digest = {
	      .dia_digestsize	=	MD5_DIGEST_SIZE,
	      .dia_init   	   = 	md5_spu_init,
	      .dia_update 	   =	md5_spu_update,
	      .dia_final  	   =	md5_spu_final 
      } 
   }
};

static int __init md5_mod_init(void)
{
#if 0 /* 4760-OCF Port: */
   if (cap_isPresent(CAP_SPU,0) == CAP_NOT_PRESENT ) {
      printk (KERN_WARNING "MD5 SPU is not supported\n");
      return -EFAULT;
   }
#endif

  md5_spu_hardware_init();
	return crypto_register_alg(&alg);
}

static void __exit md5_mod_fini(void)
{
	crypto_unregister_alg(&alg);

	/* Release DMA allocated buffers */
    spu_dma_dealloc( &crypto_dma );
    atomic_set(&spu_dma_initialized, 0);
}

subsys_initcall(md5_mod_init);
module_exit(md5_mod_fini);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Broadcom SPU-M Accelerated MD5 Message Digest Algorithm");
