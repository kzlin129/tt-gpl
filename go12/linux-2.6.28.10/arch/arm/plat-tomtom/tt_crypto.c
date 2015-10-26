/*
 * tt_crypto.c - tt specific crypto using crypto context passed by the bootloader
 *
 * 2010-03-08 Ard Biesheuvel <ard.biesheuvel@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/scatterlist.h>
#include <linux/crypto/ttcrypto_ctx.h>
#include <linux/crypto/tt_crypto.h>
#include <plat/factorydata.h>
#include <asm/uaccess.h>
#include <crypto/aes.h>

#define TTCRYPTO_MAJOR		178

extern struct factorydata_buffer_info_t fdt_buffer_info;

static struct ttcrypto_ctx ctx;

static int ttcrypto_ss_decrypt(struct ttcrypto_ss_crypt *in)
{
	struct blkcipher_desc desc = { .flags = CRYPTO_TFM_REQ_MAY_SLEEP };
	struct scatterlist sg;
	unsigned int s;
	unsigned char *buf;
	int rc = 0;

	if ( IS_ERR(desc.tfm = crypto_alloc_blkcipher("ecb(aes)", 0, CRYPTO_ALG_ASYNC))) {
		printk(KERN_ERR "ttcrypto: required aes cipher not available\n");
		return -ENOENT;
	}

	/* use the leading 128 bits of the shared secret as AES key */
	if (0 > crypto_blkcipher_setkey(desc.tfm, ctx.shared_secret, AES_KEYSIZE_128)) {
		printk(KERN_ERR "ttcrypto: failed to set aes key\n");
		rc = -ENOENT;
		goto freecipher;
	}
	
	if (copy_from_user((void*)&s, (void*)&in->size, sizeof(s))) {
		rc = -EINVAL;
		goto freecipher;
	}

	if ( !(buf = kmalloc(s, GFP_KERNEL))) {
		rc = -ENOMEM;
		goto freecipher;
	}
	
	if (copy_from_user(buf, (void*)&in->data, s)) {
		rc = -ENOMEM;
		goto freemem;
	}

	/* set up a scatterlist containing the whole data area */
	sg_init_one(&sg, buf, s);

	if (0 != (rc = crypto_blkcipher_decrypt(&desc, &sg, &sg, s)))
		goto freemem;

	if (copy_to_user((void*)&in->data, buf, s))
		rc = -ENOMEM;

freemem:
	kfree(buf);
freecipher:
	crypto_free_blkcipher(desc.tfm);
	return rc;
}

static int ttcrypto_ss_decrypt_cbc(struct ttcrypto_ss_crypt *in)
{
	struct blkcipher_desc desc = { .flags = CRYPTO_TFM_REQ_MAY_SLEEP };
	struct scatterlist sg;
	unsigned int s;
	unsigned char *buf;
	int rc = 0;

	if ( IS_ERR(desc.tfm = crypto_alloc_blkcipher("cbc(aes)", 0, CRYPTO_ALG_ASYNC))) {
		printk(KERN_ERR "ttcrypto: required aes cipher not available\n");
		return -ENOENT;
	}

	/* use the leading 128 bits of the shared secret as AES key */
	if (0 > crypto_blkcipher_setkey(desc.tfm, ctx.shared_secret, AES_KEYSIZE_128)) {
		printk(KERN_ERR "ttcrypto: failed to set aes key\n");
		rc = -ENOENT;
		goto freecipher;
	}
	
	if (copy_from_user((void*)&s, (void*)&in->size, sizeof(s))) {
		rc = -EINVAL;
		goto freecipher;
	}

	if ( !(buf = kmalloc(s, GFP_KERNEL))) {
		rc = -ENOMEM;
		goto freecipher;
	}
	
	if (copy_from_user(buf, (void*)&in->data, s)) {
		rc = -ENOMEM;
		goto freemem;
	}

	/* set up a scatterlist containing the whole data area */
	sg_init_one(&sg, buf, s);

	if (0 != (rc = crypto_blkcipher_decrypt(&desc, &sg, &sg, s)))
		goto freemem;

	if (copy_to_user((void*)&in->data, buf, s))
		rc = -ENOMEM;

freemem:
	kfree(buf);
freecipher:
	crypto_free_blkcipher(desc.tfm);
	return rc;
}

static int ttcrypto_ss_encrypt_cbc(struct ttcrypto_ss_crypt *in)
{
	struct blkcipher_desc desc = { .flags = CRYPTO_TFM_REQ_MAY_SLEEP };
	struct scatterlist sg;
	unsigned int s;
	unsigned char *buf;
	int rc = 0;

	if ( IS_ERR(desc.tfm = crypto_alloc_blkcipher("cbc(aes)", 0, CRYPTO_ALG_ASYNC))) {
		printk(KERN_ERR "ttcrypto: required aes cipher not available\n");
		return -ENOENT;
	}

	/* use the leading 128 bits of the shared secret as AES key */
	if (0 > crypto_blkcipher_setkey(desc.tfm, ctx.shared_secret, AES_KEYSIZE_128)) {
		printk(KERN_ERR "ttcrypto: failed to set aes key\n");
		rc = -ENOENT;
		goto freecipher;
	}
	
	if (copy_from_user((void*)&s, (void*)&in->size, sizeof(s))) {
		rc = -EINVAL;
		goto freecipher;
	}

	if ( !(buf = kmalloc(s, GFP_KERNEL))) {
		rc = -ENOMEM;
		goto freecipher;
	}
	
	if (copy_from_user(buf, (void*)&in->data, s)) {
		rc = -ENOMEM;
		goto freemem;
	}

	/* set up a scatterlist containing the whole data area */
	sg_init_one(&sg, buf, s);

	if (0 != (rc = crypto_blkcipher_encrypt(&desc, &sg, &sg, s)))
		goto freemem;

	if (copy_to_user((void*)&in->data, buf, s))
		rc = -ENOMEM;

freemem:
	kfree(buf);
freecipher:
	crypto_free_blkcipher(desc.tfm);
	return rc;
}

static int ttcrypto_ss_hmac(struct ttcrypto_ss_hmac *in)
{
	return 0;
}

static int ttcrypto_ioctl(struct inode *i, struct file *fp, unsigned int cmd, unsigned long arg)
{
	if (!ctx.version)
		return -ENOENT;

	switch (cmd) {
		default:			return -EINVAL;
		case TTCRYPTO_SS_DECRYPT:    	return ttcrypto_ss_decrypt((void*)arg);
		case TTCRYPTO_SS_ENCRYPT_CBC: return ttcrypto_ss_encrypt_cbc((void*)arg);
		case TTCRYPTO_SS_DECRYPT_CBC: return ttcrypto_ss_decrypt_cbc((void*)arg);
		case TTCRYPTO_SS_HMAC:      	return ttcrypto_ss_hmac((void*)arg);
	}
	return 0;
}

static int ttcrypto_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ttcrypto_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations ttcrypto_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= ttcrypto_ioctl,
	.open		= ttcrypto_open,
	.release	= ttcrypto_release
};

static int __init tomtom_crypto_init(void)
{
	struct ttcrypto_ctx *c = (fdt_buffer_info.address + fdt_buffer_info.size) & ~TTCRYPTO_ALIGN_MASK;
	int rc;

	/* look for the correct magic */
	if (c->magic != TTCRYPTO_CTX_MAGIC)
		printk(KERN_ERR "ttcrypto: incorrect magic, no crypto context found\n");
	else if (0 > (rc = register_chrdev(TTCRYPTO_MAJOR, "ttcrypto", &ttcrypto_fops)))
		printk(KERN_ERR "ttcrypto: can't create char device: %d\n", rc);
	else {
		printk(KERN_WARNING "ttcrypto: found version %d tt_crypto context\n", c->version);

		memcpy(&ctx, c, sizeof(struct ttcrypto_ctx));
		memzero(c, sizeof(struct ttcrypto_ctx));
	}

	return 0;
}

late_initcall(tomtom_crypto_init);

