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
 * SHA1 Secure Hash Algorithm.
 *
 * Derived from cryptoapi implementation, adapted for in-place
 * scatterlist interface.
 *
 * Copyright (c) Alan Smithee.
 * Copyright (c) Andrew McDonald <andrew@mcdonald.org.uk>
 * Copyright (c) Jean-Francois Dive <jef@linuxbe.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/crypto.h>
#include <linux/cryptohash.h>
#include <linux/types.h>
#include <crypto/sha.h>
#include <asm/byteorder.h>

#include <linux/broadcom/bcm_spu.h>
#include <asm/arch/spuHw.h>
#include <asm/arch/spuHw_inline.h>
//#include <mach/csp/cap.h>


static spu_dma_context  crypto_dma;   /* DMA context for SPU */
static volatile atomic_t spu_dma_initialized;

struct sha1_spu_ctx {
        u64 count;
        u32 state[5];
        u8  buffer[64];
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

static void sha1_spu_transform_init( u32 *digest, const char *data, unsigned int len )
{
   int cmd_len_bytes;
   int output_len_bytes;

   u32 *cmd_buffer;
   u32 *tmp_in_bufp;
   u32 *tmp_out_bufp;

   spuHw_CONTEXT_t sha1_spu_context;

   /* Reserve channels for DMA transfer */
   spu_dma_reserve( &crypto_dma );

   /* Initialize DMA context and allocate memory */
   if( initialize_dma() != 0 )
	   return;

   /* Assign buffers for DMA */     
   cmd_buffer    = crypto_dma.crypto_cmd.virt; 
   tmp_in_bufp   = crypto_dma.crypto_in.virt;
   tmp_out_bufp  = crypto_dma.crypto_out.virt;

   memset ( (void*)&sha1_spu_context, 0, sizeof(sha1_spu_context) );
   
   sha1_spu_context.operation                = spuHw_CRYPTO_OPERATION_ENCRYPTION;
   sha1_spu_context.cryptoAlgo               = spuHw_CRYPTO_ALGO_NULL;
   sha1_spu_context.authAlgo                 = spuHw_AUTH_ALGO_SHA1;
   sha1_spu_context.authMode                 = spuHw_AUTH_MODE_HASH;
   sha1_spu_context.authType                 = spuHw_AUTH_TYPE_SHA1_INIT;
   sha1_spu_context.authOrder                = spuHw_CMD_AUTH_FIRST;
   sha1_spu_context.keyType                  = spuHw_KEY_OPEN;
   sha1_spu_context.icvLen                   = (SHA1_DIGEST_SIZE / sizeof(u32));

   sha1_spu_context.dataAttribute.macOffset  = 0;
   sha1_spu_context.dataAttribute.macLength  = len;   
   sha1_spu_context.dataAttribute.dataLength = (((len + 3) / sizeof(u32)) * sizeof(u32));

   spu_request( 0 );

   cmd_len_bytes = spuHw_createCryptoCommand( &sha1_spu_context, cmd_buffer );
   output_len_bytes =   spuHw_OUTPUT_HEADER_LEN +
                        sha1_spu_context.dataAttribute.dataLength +
                        (sha1_spu_context.icvLen * sizeof(u32)) +
                        spuHw_OUTPUT_STATUS_LEN;

   spuHw_setPacketLength( cmd_len_bytes + sha1_spu_context.dataAttribute.dataLength, output_len_bytes );

   memset( tmp_in_bufp + (sha1_spu_context.dataAttribute.dataLength / sizeof(u32)), 0, sizeof(u32) );
   memcpy( tmp_in_bufp, (void*)data, sha1_spu_context.dataAttribute.dataLength );

   /* Reserve channels for DMA transfer */
   //spu_dma_reserve( &crypto_dma );

   /* configure the SPU DMA */   
   spu_dma_config(&crypto_dma, cmd_len_bytes, sha1_spu_context.dataAttribute.dataLength, output_len_bytes);

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
               tmp_out_bufp + ((spuHw_OUTPUT_HEADER_LEN + sha1_spu_context.dataAttribute.dataLength + 3 ) / sizeof(u32)),
               sha1_spu_context.dataAttribute.dataLength );        
   }

   spu_release();

}

static void sha1_spu_transform_update(u32 *digest, const char *data)
{

   int cmd_len_bytes;
   int output_len_bytes;

   u32 *cmd_buffer;
   u32 *tmp_in_bufp;
   u32 *tmp_out_bufp;

   spuHw_CONTEXT_t sha1_spu_context;

   /* Reserve channels for DMA transfer */
   spu_dma_reserve( &crypto_dma );

   /* Initialize DMA context and allocate memory */
   if( initialize_dma() != 0 )
	   return;

   /* Assign buffers for DMA */     
   cmd_buffer    = crypto_dma.crypto_cmd.virt; 
   tmp_in_bufp   = crypto_dma.crypto_in.virt;
   tmp_out_bufp  = crypto_dma.crypto_out.virt;

   memset ( (void*)&sha1_spu_context, 0, sizeof(sha1_spu_context) );
   sha1_spu_context.operation                = spuHw_CRYPTO_OPERATION_ENCRYPTION;
   sha1_spu_context.cryptoAlgo               = spuHw_CRYPTO_ALGO_NULL;
   sha1_spu_context.authAlgo                 = spuHw_AUTH_ALGO_SHA1;
   sha1_spu_context.authMode                 = spuHw_AUTH_MODE_HASH;
   sha1_spu_context.authType                 = spuHw_AUTH_TYPE_SHA1_UPDATE;
   sha1_spu_context.authOrder                = spuHw_CMD_AUTH_FIRST;
   sha1_spu_context.keyType                  = spuHw_KEY_OPEN;

   sha1_spu_context.authKey                  = (void *)digest;
   sha1_spu_context.authKeyLen               = (SHA1_DIGEST_SIZE / sizeof(u32));
   
   sha1_spu_context.icvLen                   = (SHA1_DIGEST_SIZE / sizeof(u32));
   sha1_spu_context.dataAttribute.macOffset  = 0;
   sha1_spu_context.dataAttribute.macLength  = SHA1_BLOCK_SIZE;
   sha1_spu_context.dataAttribute.dataLength = SHA1_BLOCK_SIZE;

   spu_request( 0 );

   cmd_len_bytes = spuHw_createCryptoCommand( &sha1_spu_context, cmd_buffer );

   output_len_bytes =   spuHw_OUTPUT_HEADER_LEN +
                        sha1_spu_context.dataAttribute.dataLength +
                        (sha1_spu_context.icvLen * sizeof(u32)) +
                        spuHw_OUTPUT_STATUS_LEN;

   spuHw_setPacketLength( cmd_len_bytes + sha1_spu_context.dataAttribute.dataLength, output_len_bytes );

   memset( tmp_in_bufp + (sha1_spu_context.dataAttribute.dataLength / sizeof(u32)), 0, sizeof(u32) );
   memcpy( tmp_in_bufp, (void*)data, sha1_spu_context.dataAttribute.dataLength );

   /* Reserve channels for DMA transfer */
   //spu_dma_reserve( &crypto_dma );

   /* configure the SPU DMA */   
   spu_dma_config(&crypto_dma, cmd_len_bytes, sha1_spu_context.dataAttribute.dataLength, output_len_bytes);

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
               tmp_out_bufp + ((spuHw_OUTPUT_HEADER_LEN + sha1_spu_context.dataAttribute.dataLength + 3 ) / sizeof(u32)),
               sha1_spu_context.dataAttribute.dataLength );      
   }

   spu_release();

}

static void sha1_init(struct crypto_tfm *tfm)
{
	struct sha1_spu_ctx *sctx = crypto_tfm_ctx(tfm);
	static const struct sha1_spu_ctx initstate = {
	  0,
	  { SHA1_H0, SHA1_H1, SHA1_H2, SHA1_H3, SHA1_H4 },
	  { 0, },
     0
	};
  	*sctx = initstate;
}

static void sha1_update(struct crypto_tfm *tfm, const u8 *data,
			unsigned int len)
{
	struct sha1_spu_ctx *sctx = crypto_tfm_ctx(tfm);
	unsigned int partial, done;
	const u8 *src;
  
	partial = sctx->count & 0x3f;
	sctx->count += len;
	done = 0;
	src = data;

	if ((partial + len) > 63) {

		if (partial) {
			done = -partial;
			memcpy(sctx->buffer + partial, data, done + SHA1_BLOCK_SIZE);
			src = sctx->buffer;
		}

		do {
         if ( sctx->init_performed == 0 )
         {
            sha1_spu_transform_init(sctx->state, src, SHA1_BLOCK_SIZE);
            sctx->init_performed = 1;
         }
         else
         {
            sha1_spu_transform_update( sctx->state, src );
         }
			done += SHA1_BLOCK_SIZE;
			src = data + done;
		} while (done + 63 < len);

		partial = 0;
	}
	memcpy(sctx->buffer + partial, src, len - done);
}


/* Add padding and return the message digest. */
static void sha1_final(struct crypto_tfm *tfm, u8 *out)
{
	struct sha1_spu_ctx *sctx = crypto_tfm_ctx(tfm);
	__be32 *dst = (__be32 *)out;
	u32 i, index, padlen;
	__be64 bits;
	static const u8 padding[SHA1_BLOCK_SIZE] = { 0x80, };

	bits = cpu_to_be64(sctx->count << 3);

	/* Pad out to 56 mod 64 */
	index = sctx->count & 0x3f;
	padlen = (index < 56) ? (56 - index) : ((64+56) - index);
	sha1_update(tfm, padding, padlen);

	/* Append length */
	sha1_update(tfm, (const u8 *)&bits, sizeof(bits));
   
	for (i = 0; i < (SHA1_DIGEST_SIZE / sizeof(u32)); i++)
		dst[i] = (sctx->state[i]);

	/* Wipe context */
	memset(sctx, 0, sizeof *sctx);
}

static void sha1_spu_hardware_init( void )
{
   /* Request and release will ensure hardware is initialized */   
   spu_request( 0 );
   spu_release();

   atomic_set(&spu_dma_initialized, 0);
}

static struct crypto_alg alg = {
	.cra_name	      = "sha1",
	.cra_driver_name  = "sha1-spu",
   .cra_priority     = 101,
	.cra_flags	      = CRYPTO_ALG_TYPE_DIGEST,
	.cra_blocksize	   = SHA1_BLOCK_SIZE,
	.cra_ctxsize	   = sizeof(struct sha1_spu_ctx),
	.cra_module	      = THIS_MODULE,
	.cra_alignmask	   = 3,
	.cra_list         = LIST_HEAD_INIT(alg.cra_list),
	.cra_u = { 
      .digest = {
	      .dia_digestsize	= SHA1_DIGEST_SIZE,
	      .dia_init   	   = sha1_init,
	      .dia_update 	   = sha1_update,
	      .dia_final  	   = sha1_final 
      } 
   }
};

static int __init sha1_generic_mod_init(void)
{
#if 0 /* 4760-OCF Port: */
   if (cap_isPresent(CAP_SPU,0) == CAP_NOT_PRESENT ) {
      printk (KERN_WARNING "SHA1 SPU is not supported\n");
      return -EFAULT;
   }
#endif

  sha1_spu_hardware_init();
	return crypto_register_alg(&alg);
}

static void __exit sha1_generic_mod_fini(void)
{
	crypto_unregister_alg(&alg);

    /* Release DMA allocated buffers */
    spu_dma_dealloc( &crypto_dma );
    atomic_set(&spu_dma_initialized, 0);
}

subsys_initcall(sha1_generic_mod_init);
module_exit(sha1_generic_mod_fini);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Broadcom SPU-M Accelerated SHA1 Secure Hash Algorithm");

MODULE_ALIAS("sha1_spu");
