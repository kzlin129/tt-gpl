/*
 * linux/drivers/userdma.c
 *
 * Broadcom BCM476X user dma driver header file
 *
 * Copyright (C) 2008-2009 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/arch/dma.h>
#include <asm/dma-mapping.h>
#include <linux/broadcom/userdma.h>

#ifdef CONFIG_PLAT_BCM476X
#include "bcm476x_userdma.h"
#endif

#define DRIVER_VERSION "Version 0.1"
#define DRIVER_DESC    "BCM476X userdma driver."
#define DRIVER_INFO    DRIVER_DESC " " DRIVER_VERSION 

#define USERDMA_DEV_NAME	"userdma"

#define USERDMA_MAJOR MISC_MAJOR
#define USERDMA_MINOR 161
	/*
	 *    Global variables
	 */
static int userdma_loaded = 0;
static int userdma_major_nr;
static u64 userdma_mask;
struct miscdevice g_userdma_dev;

int bcm476x_userdma_open(void)
{
	if (!userdma_loaded)
		return -1;


	return 0;
}

static int userdma_open(struct inode *inode, struct file *file)
{
	if (MAJOR(inode->i_rdev) != userdma_major_nr)
	{
		BUG();
		return -1;
	}

	return (bcm476x_userdma_open());
}

int bcm476x_userdma_close(void)
{
	return 0;
}

static int userdma_release(struct inode *inode, struct file *file)
{
	return (bcm476x_userdma_close());
}

int userdma_alloc_buf(void *arg)
{
    int rc = 0;
    userdma_mem_t dma_mem;
    copy_from_user(&dma_mem, arg, sizeof(userdma_mem_t));

#if 1
    dma_mem.virt = dma_alloc_writecombine(g_userdma_dev.this_device, dma_mem.alloc_size, (dma_addr_t *)&dma_mem.phys, GFP_DMA);
    if (!dma_mem.virt) {
        printk(KERN_ERR "userdma_alloc_buf failed\n");
        rc = -ENOMEM;
    }
#else
    dma_mem.virt = kmalloc(dma_mem.alloc_size, GFP_DMA);
    if (!dma_mem.virt) {
        printk(KERN_ERR "userdma_alloc_buf failed\n");
        rc = -ENOMEM;
    }
    else
        dma_mem.phys = (void*) virt_to_phys(dma_mem.virt);
#endif

    //printk("userdma_alloc_buf: virt %p, phys %p\n", dma_mem.virt, dma_mem.phys);
    copy_to_user(arg, &dma_mem, sizeof(userdma_mem_t));
    return rc;    
}

int  userdma_release_buf(void *arg)
{
    userdma_mem_t dma_mem;

    copy_from_user(&dma_mem, arg, sizeof(userdma_mem_t));
#if 1
    dma_free_writecombine(g_userdma_dev.this_device, dma_mem.alloc_size, dma_mem.virt, (dma_addr_t) dma_mem.phys); 
#else
    kfree(dma_mem.virt);
#endif
    return 0;
}

static int userdma_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int err = 0;

    if (!capable(CAP_SYS_ADMIN)) {
        return -EACCES;
    }

    if (MAJOR(inode->i_rdev) != userdma_major_nr){
        printk (
                KERN_ERR "userdma_ioctl(): wrong major number %d != %d\n",
                MAJOR(inode->i_rdev),
                userdma_major_nr
                );

        return -EINVAL;
    }

    switch(cmd) {
        case USERDMA_IOCTL_ALLOC_CHANNEL:
            err = userdma_alloc_channel((int *) arg );
            break;

        case USERDMA_IOCTL_RELEASE_CHANNEL:
            err = userdma_release_channel((int) arg );
            break;

        case USERDMA_IOCTL_ALLOC_BUF:
            err = userdma_alloc_buf( (void *) arg );
            break;

        case USERDMA_IOCTL_RELEASE_BUF:
            err = userdma_release_buf( (void *) arg );
            break;

        case USERDMA_IOCTL_DMA:
            err = userdma_do_dma((userdma_ioctl_t *)arg);
            break;

        //case USERDMA_IOCTL_MEMCP:
            //err = userdma_do_memcp(arg);
        //    break;

        default:
            err = -0xf;
            printk("userdma ioctl doesnt exist.\n");
            break;
    }
    return err;
}

static struct file_operations userdma_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= userdma_ioctl,
    .open		= userdma_open,
	.release	= userdma_release,
};

struct miscdevice g_userdma_dev = {
	.minor		= USERDMA_MINOR,
	.name		= "userdma",
	.fops		= &userdma_fops,
};

static int __init userdma_init(void)
{
   int ret;

	ret = misc_register(&g_userdma_dev);
	if (ret) {
		printk(KERN_ERR "cannot register miscdev on minor=%d (err=%d)\n",
			USERDMA_MINOR, ret);
		return ret;
	}
    g_userdma_dev.this_device->dma_mask = &userdma_mask;
	g_userdma_dev.this_device->coherent_dma_mask = userdma_mask = DMA_32BIT_MASK;
    userdma_major_nr = USERDMA_MAJOR;
    userdma_loaded = 1;
	printk("userdma device loaded\n");

	return 0;

}

static void __exit userdma_exit(void)
{
	dprintk(KERN_INFO "Removing Broadcom bcm476x userdma device driver...\n");

	userdma_loaded = 0;
	misc_deregister(&g_userdma_dev);
}


#ifdef MODULE
 MODULE_DESCRIPTION (DRIVER_INFO);
#endif

module_init(userdma_init);
module_exit(userdma_exit);

MODULE_AUTHOR("BROADCOM");
MODULE_DESCRIPTION("BROADCOM BCM476X UserDMA");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(USERDMA_MINOR);


