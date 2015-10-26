/* fs/ngffs/sysfile.c
 *
 * NGFFS, next generation flash file system
 *
 * Copyright (C) 2004-2007 TomTom BV <http://www.tomtom.com/>
 * Author: Koen Martens <kmartens@sonologic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/blkdev.h>
#include <linux/mtd/mtd.h>
#include <linux/time.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <barcelona/debug.h>
#include <linux/ngffs.h>
#include "kern.h"
#include "chunk.h"
#include "file.h"
#include "flash.h"
#include "garbage.h"

#define NGFFS_SYSFILE

#include "sysfile.h"
#include "sysfiles.h"

/* Defines */
#define PFX "ngffs: "
#define PK_DBG PK_DBG_FUNC

int ngffs_sysfile_readpage (struct file *filp, struct page *pg);

struct file_operations ngffs_sysfile_operations = {
	.llseek = generic_file_llseek,
	.open   = generic_file_open,
	.read   = generic_file_read,
	.write  = generic_file_write,
//	.ioctl  = ngffs_ioctl,
	.mmap   = generic_file_mmap,
//	.fsync  = ngffs_null_fsync
};

/* ngffs_file_inode_operations */

struct inode_operations ngffs_sysfile_inode_operations = {
//	.setattr = ngffs_setattr
};


struct address_space_operations ngffs_sysfile_address_operations = {
	.readpage      = ngffs_sysfile_readpage,
//	.prepare_write = ngffs_prepare_write,
//	.commit_write  = ngffs_commit_write
};

int ngffs_sysfile_do_readpage_nolock(struct inode *inode, struct page *pg)
{
	struct ngffs_info *ngsb=NGFFS_INFO(inode->i_sb);
	int i;
	int rv=0;
	__u32 offset;

	i=inode->i_ino-3;
	PK_DBG("sysfile found at %i\n",i);

	PK_DBG("sysfile read\n");
	if (!PageLocked(pg)) {
		/* PLEASECHECK Koen: PAGE_BUG has been removed as of 2.6.12 or so,
		 * no idea what it should be. */
		printk("page BUG for page at %p\n", pg);
		BUG();
	}

	if(i>=NGFFS_SYSFILES) {
		PK_WARN("sysfile id out of range!\n");
		goto readpage_fail;
	}

	/* Determine offset */
	offset = ngffs_sysfiles[i].ofs;
	if ( ngsb->mtd->size >= 0x200000 ) /* >= 2MB */
	{
		/* factory data stored in upper half of flash */
		if ( offset < 0x4000 ) /* factory data */
		{
			offset += 0x100000; /* 1MB offset */
		}
	}

	printk("[kwwo] reading abs addr 0x%x\n", offset);
	rv=ngffs_absolute_read(ngsb->mtd,offset,(u_char *)page_address(pg),ngffs_sysfiles[i].length);
	if(rv) goto readpage_fail;

	//  if (!strcmp(ngffs_sysfiles[i].name,"id")) memcpy((u_char *)page_address(pg),"AAAAAAAAAAAA",12);

	SetPageUptodate(pg);
	ClearPageError(pg);
	flush_dcache_page(pg);
	kunmap(pg);
	return 0;

readpage_fail:
	ClearPageUptodate(pg);
	SetPageError(pg);
	kunmap(pg);
	return rv;

}

int ngffs_sysfile_readpage (struct file *filp, struct page *pg)
{

	PK_DBG("ngffs_sysfile_readpage()\n");
	// do lock stuff?
	ngffs_sysfile_do_readpage_nolock(filp->f_dentry->d_inode,pg);
	unlock_page(pg);
	// unlock
	return 0;
}

/* EOF */
