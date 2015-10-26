/* 
 *  Copyright (C) 2009 TomTom BV <http://www.tomtom.com/>
 *
 *  Export the device-tree buffer to user-land through a char device.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/limits.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/mm.h>

#include <asm/uaccess.h>

/* Defines */
#define PFX "fdtexport:"
#define PK_DBG PK_DBG_FUNC

#define FDT_DEV_MAJOR	177

/* Safe as everybody uses the same device-tree */
static struct resource *tdb_buffer;
	
static int fdt_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int fdt_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t fdt_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t retval = 0;
	void *from = (void*)tdb_buffer->start;
	size_t fdt_size = tdb_buffer->end - tdb_buffer->start;
	if (*f_pos < fdt_size) {
		if (count > fdt_size - *f_pos) {
			count = fdt_size - *f_pos;
		}

		from += *f_pos;
		
		if (!copy_to_user(buf, from, count)) {
			*f_pos += count;
			retval = count;
		} else {		
			retval = -EFAULT;
		}
	} else {
		retval = 0;
	}

	return retval;
}

loff_t fdt_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos = 0;

	switch(whence) {
	  case SEEK_SET:
		newpos = off;
		break;
	  case SEEK_CUR:
		newpos = filp->f_pos + off; 
		break;
	  case SEEK_END:
		newpos = tdb_buffer->end - tdb_buffer->start + off;
		break;
	  default:
		newpos = -EINVAL;
		break;
	}

	return newpos;
}

static int fdt_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	return VM_FAULT_SIGBUS;
}

struct vm_operations_struct fdt_vm_ops = {	
	.fault		= fdt_vm_fault,
};

static int fdt_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long offset	= vma->vm_pgoff << PAGE_SHIFT;
	unsigned long physaddr	= virt_to_phys((void *)tdb_buffer->start) + offset;
	unsigned long pfn		= physaddr >> PAGE_SHIFT;
	unsigned long psize		= tdb_buffer->end - tdb_buffer->start - offset;

	if (remap_pfn_range(vma, vma->vm_start, pfn, psize, vma->vm_page_prot)) {
		printk(KERN_ERR PFX	" fdt_mmap: pfn_range failed !!\n");
		return -EAGAIN;
	}

	vma->vm_ops				=  &fdt_vm_ops;
	vma->vm_flags			|= VM_RESERVED;
	vma->vm_private_data	=  filp->private_data;

	return 0;
}


struct file_operations fdt_fops = {
	.owner		= THIS_MODULE,
	.read		= fdt_read,
	.open		= fdt_open,
	.llseek		= fdt_llseek,
	.mmap		= fdt_mmap,
	.release	= fdt_release,
};

static int fdtexport_probe(struct platform_device *pdev)
{
	int rc = 0;

	tdb_buffer = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!tdb_buffer) {
		printk(KERN_ERR "fdt tdb buffer memory ressource is NULL\n");
		return -EINVAL;
	}

	rc = register_chrdev(FDT_DEV_MAJOR, "fdtexport", &fdt_fops);
	if (rc < 0) {
		printk(KERN_ERR "fdt can't create char device: %d\n", rc);
		goto out;
	}

out:
	return rc;
};

static int fdtexport_remove(struct platform_device *pdev)
{
	unregister_chrdev(FDT_DEV_MAJOR, "fdtexport");
	return 0;
}

static struct platform_driver fdtexport_driver = {
	.probe		= fdtexport_probe,
	.remove		= fdtexport_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "fdtexport",
	}
};

static char banner[] = KERN_INFO "fdtexport driver, (c) 2009 TomTom BV\n";

int __init fdt_export_init_module(void)
{
	printk(banner);
	return platform_driver_register(&fdtexport_driver);
}

void __exit fdt_export_exit_module(void)
{
	platform_driver_unregister(&fdtexport_driver);
}

module_init(fdt_export_init_module);
module_exit(fdt_export_exit_module);

MODULE_DESCRIPTION("fdt_export");
MODULE_LICENSE("GPL");
