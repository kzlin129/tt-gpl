/* linux/drivers/media/video/samsung/gvg/s3c_fimgvg.c
 *
 * Driver file for Samsung OpenVG Accelerator(FIMG-VG)
 *
 * Copyright (c) 2009 Samsung Electronics
 *	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/mman.h>
#include <mach/map.h>
#include <plat/media.h>

#define DEBUG_S3C_GVG

#ifdef DEBUG_S3C_GVG
#define DEBUG		printk
#else
#define DEBUG(fmt,args...)		do {} while(0)
#endif

#define VG_VERSION			0x10
#define VG_INTERRUPT_ENABLE		0x04
#define VG_INTPENDING			0x08
#define VG_FIFO_STAT			0x0C
#define VG_SCCF_STAT			0xE0C
#define VG_FT_END			0xD84

#define VG_IOCTL_MAGIC			'S'

#define S3C_GVG_WAIT_FOR_CF_FLUSH	_IO(VG_IOCTL_MAGIC, 100)
#define S3C_GVG_WAIT_FOR_FILTER_FLUSH	_IO(VG_IOCTL_MAGIC, 101)
#define S3C_GVG_WAIT_FOR_SCCF_FLUSH	_IO(VG_IOCTL_MAGIC, 102)

#define S3C_GVG_MEM_ALLOC		_IOWR(VG_IOCTL_MAGIC, 310, struct s3c_gvg_mem_descr)
#define S3C_GVG_MEM_FREE		_IOWR(VG_IOCTL_MAGIC, 311, struct s3c_gvg_mem_descr)
#define S3C_GVG_TESSELLATE_EDGE_GET	_IOWR(VG_IOCTL_MAGIC, 350, struct s3c_gvg_mem_descr)
#define S3C_GVG_TESSELLATE_VERTEX_GET	_IOWR(VG_IOCTL_MAGIC, 351, struct s3c_gvg_mem_descr)

#define VG_CHUNK_NUM CONFIG_VIDEO_GVG_RESOURCE_MEM_BLOCK_NUM
#define VG_CHUNK_SIZE CONFIG_VIDEO_GVG_RESOURCE_MEM_BLOCK_SIZE * SZ_1K
#define VG_RESERVED_MEM_SIZE (VG_CHUNK_SIZE * VG_CHUNK_NUM)

// TODO: could be removed
/*
#define VG_CHUNK_AVAILABLE		0
#define VG_CHUNK_RESERVED		1
*/

#define PFX				"s3c_gvg "
#define GVG_MINOR			201

#define GVG_195MHZ			195000000

enum eVGInterruptMask
{
	VG_INT_MASK_SCCF = 1 << 5,
	VG_INT_MASK_FILTER = 1 << 4,
	VG_INT_MASK_EDGE_OVERFLOW = 1 << 3,
	VG_INT_MASK_VERTEX_OVERFLOW = 1 << 2,
	VG_INT_MASK_CURRENT_COMMAND =  1 << 0,
	VG_INT_MASK_ALL_COMMAND_END = VG_INT_MASK_SCCF | VG_INT_MASK_FILTER | VG_INT_MASK_CURRENT_COMMAND,
	VG_INT_MASK_ALL_OVERFLOW = VG_INT_MASK_EDGE_OVERFLOW | VG_INT_MASK_VERTEX_OVERFLOW,
	VG_INT_MASK_ALL = VG_INT_MASK_ALL_COMMAND_END | VG_INT_MASK_ALL_OVERFLOW,
};

struct s3c_gvg_mem_descr {
	int		size;
	unsigned int	virt_addr;
	unsigned int	phys_addr;	// TODO: does userspace really use this or is it just used as unique key?
	unsigned int	is_used;	// TODO: not used (by the kernel)
	unsigned int    file_desc_id;	// TODO: not used (by the kernel)
	pid_t cur_pid;			// TODO: not used (by the kernel)
};

struct s3c_gvg_chunk {
	struct s3c_gvg_mem_descr	descr;
	void				*addr;
	struct list_head		list;
};

struct s3c_gvg_chunk_list {
	struct list_head	list;
	struct mutex		lock;
};

struct s3c_gvg_tessel_mem {
	unsigned long	phys_addr;
	void		*virt_addr;
	unsigned int	size;
};

struct s3c_gvg_iomem {
	struct resource	*res;
	void __iomem	*base;
	unsigned long	base_phys;
};

struct s3c_gvg_mmap {
	unsigned long	phys_addr;
	struct mutex	lock;
};

struct s3c_gvg {
	struct s3c_gvg_iomem		iomem;
	struct s3c_gvg_tessel_mem	mem_vertexes;
	struct s3c_gvg_tessel_mem	mem_edges;
	struct s3c_gvg_chunk_list	available_chunks;
	int				irq;
	wait_queue_head_t		wq_irq;
};

struct s3c_gvg_instance {
	struct s3c_gvg_chunk_list	reserved_chunks;
	struct s3c_gvg_mmap		mmap;
};


static struct s3c_gvg s3c_gvg;


// allocate memory chunks
//
static int s3c_gvg_allocate_chunks(void)
{
	int i;

	mutex_lock(&s3c_gvg.available_chunks.lock);

	for (i = 0; i < VG_CHUNK_NUM; i++) {
		struct s3c_gvg_chunk* chunk = kmalloc(sizeof(*chunk), GFP_KERNEL);
		if (NULL == chunk)
			return -ENOMEM;

		INIT_LIST_HEAD(&chunk->list);

		chunk->addr = kmalloc(VG_CHUNK_SIZE, GFP_KERNEL);
		if (NULL == chunk->addr) {
			kfree(chunk);
			return -ENOMEM;
		}

		chunk->descr.phys_addr = (unsigned int)virt_to_phys((void*)chunk->addr);
		chunk->descr.size = VG_CHUNK_SIZE;
		/* TODO: we don't need this, does userspace?
		  chunk->descr.is_used = VG_CHUNK_AVAILABLE;
		  chunk->descr.file_desc_id = 0;
		  chunk->descr.cur_pid = task_pid_nr(current);
		*/

		list_add_tail(&chunk->list, &s3c_gvg.available_chunks.list);
	}

	mutex_unlock(&s3c_gvg.available_chunks.lock);

	return 0;
}

// free memory chunks
//
static void s3c_gvg_free_chunks(void)
{
	struct s3c_gvg_chunk *chunk, *next;

	mutex_lock(&s3c_gvg.available_chunks.lock);

	list_for_each_entry_safe(chunk, next, &s3c_gvg.available_chunks.list, list) {
		list_del(&chunk->list);

		kfree(chunk->addr);
		kfree(chunk);
	}

	mutex_unlock(&s3c_gvg.available_chunks.lock);
}

// Find a chunk of memory reserved by the calling process. The key to the chunk
// is its physical address
//
static struct s3c_gvg_chunk *s3c_gvg_get_reserved_chunk(struct s3c_gvg_instance *s3c_gvg_inst, unsigned long phys_addr)
{
	struct s3c_gvg_chunk *chunk;
	bool chunk_found = 0;

	mutex_lock(&s3c_gvg_inst->reserved_chunks.lock);

	list_for_each_entry(chunk, &s3c_gvg.available_chunks.list, list) {
		if (chunk->descr.phys_addr == phys_addr) {
			chunk_found = 1;
			break;
		}
	}

	mutex_unlock(&s3c_gvg_inst->reserved_chunks.lock);

	if (!chunk_found)
		chunk = NULL;

	return chunk;
}

// Reserve a chunk of memory of a given size (or larger)
//
static struct s3c_gvg_chunk *s3c_gvg_reserve_chunk(struct s3c_gvg_instance *s3c_gvg_inst, unsigned int size)
{
	struct s3c_gvg_chunk *chunk;
	bool chunk_found = 0;

	mutex_lock(&s3c_gvg.available_chunks.lock);

	list_for_each_entry(chunk, &s3c_gvg.available_chunks.list, list) {
		if (chunk->descr.size >= size) {
			// remove the chunk from the list of free chunks and add it to the
			// list of reserved chunks of the calling process
			mutex_lock(&s3c_gvg_inst->reserved_chunks.lock);

			list_move_tail(&chunk->list, &s3c_gvg_inst->reserved_chunks.list);

			mutex_unlock(&s3c_gvg_inst->reserved_chunks.lock);

			/* TODO: we don't need this, does userspace?
			   chunk->descr.is_used = VG_CHUNK_RESERVED;
			   chunk->descr.file_desc_id = (int)s3c_gvg_inst;
			   chunk->descr.cur_pid = task_pid_nr(current);
			*/

			chunk_found = 1;

			break;
		}
	}

	mutex_unlock(&s3c_gvg.available_chunks.lock);

	if (chunk_found) {
		printk(KERN_DEBUG PFX "reserved chunk of size %d at address 0x%X (requested size: %d bytes)\n",
                        chunk->descr.size, chunk->descr.phys_addr, size);
	}
	else {
		printk(KERN_DEBUG PFX "unable to reserve chunk of size %d\n", size);
		chunk = NULL;
	}

	return chunk;
}

// release a chunk of memory
//
// precondition: s3c_gvg.available_chunks.lock and
//               s3c_gvg_inst->reserved_chunks.lock are held
//
static void __s3c_gvg_release_chunk(struct s3c_gvg_chunk *chunk)
{
	printk(KERN_DEBUG PFX "releasing chunk of size %d at address 0x%X\n",
		chunk->descr.size, chunk->descr.phys_addr);

	do_munmap(current->mm, chunk->descr.virt_addr, chunk->descr.size);

	list_move_tail(&chunk->list, &s3c_gvg.available_chunks.list);

	/* TODO: we don't need this, does userspace?
	chunk->descr.is_used = VG_CHUNK_AVAILABLE;
	chunk->descr.file_desc_id = 0;
	chunk->descr.cur_pid = 0;
	*/
}

// release a chunk of memory reserved by the calling process
//
static void s3c_gvg_release_chunk(struct s3c_gvg_instance *s3c_gvg_inst, struct s3c_gvg_chunk *chunk)
{
	mutex_lock(&s3c_gvg.available_chunks.lock);
	mutex_lock(&s3c_gvg_inst->reserved_chunks.lock);

	__s3c_gvg_release_chunk(chunk);

	mutex_unlock(&s3c_gvg_inst->reserved_chunks.lock);
	mutex_unlock(&s3c_gvg.available_chunks.lock);
}

// release all chunks reserved by a process
//
static void s3c_gvg_release_reserved_chunks(struct file *filp)
{
	struct s3c_gvg_instance *s3c_gvg_inst = (struct s3c_gvg_instance *)filp->private_data;
	struct s3c_gvg_chunk *chunk, *next;

	mutex_lock(&s3c_gvg.available_chunks.lock);
	mutex_lock(&s3c_gvg_inst->reserved_chunks.lock);

	list_for_each_entry_safe(chunk, next, &s3c_gvg_inst->reserved_chunks.list, list) {
		__s3c_gvg_release_chunk(chunk);
	}

	mutex_unlock(&s3c_gvg_inst->reserved_chunks.lock);
	mutex_unlock(&s3c_gvg.available_chunks.lock);
}

// ISR
//
static irqreturn_t s3c_gvg_isr(int irq, void *dev_id)
{
	u32 pending_irqs;

	// check for pending interrupts
	pending_irqs = __raw_readl(s3c_gvg.iomem.base + VG_INTPENDING);

	if (pending_irqs & VG_INT_MASK_EDGE_OVERFLOW)
		if (printk_ratelimit())
			printk(KERN_INFO PFX "not enough edge memory to draw path\n");

	if (pending_irqs & VG_INT_MASK_VERTEX_OVERFLOW)
		if (printk_ratelimit())
			printk(KERN_INFO PFX "not enough vertex memory to draw path\n");

	// clear pending interrupts
	pending_irqs &= ~(VG_INT_MASK_ALL);
	__raw_writel(pending_irqs, s3c_gvg.iomem.base + VG_INTPENDING);

	// wake up waiting processes
	wake_up_interruptible(&s3c_gvg.wq_irq);

	return IRQ_HANDLED;
}

static int s3c_gvg_open(struct inode *inode, struct file *file)
{
	struct s3c_gvg_instance *s3c_gvg_inst;

	printk(KERN_DEBUG PFX "open()\n");

	s3c_gvg_inst = kmalloc(sizeof(*s3c_gvg_inst), GFP_KERNEL);
	if (NULL == s3c_gvg_inst)
		return -ENOMEM;

	file->private_data = s3c_gvg_inst;

	// initialize fields of the driver instance struct
	INIT_LIST_HEAD(&s3c_gvg_inst->reserved_chunks.list);
	mutex_init(&s3c_gvg_inst->reserved_chunks.lock);
	s3c_gvg_inst->mmap.phys_addr = 0;
	mutex_init(&s3c_gvg_inst->mmap.lock);

	return 0;
}

static int s3c_gvg_release(struct inode *inode, struct file *file)
{
	struct s3c_gvg_instance *s3c_gvg_inst  = (struct s3c_gvg_instance *)file->private_data;

	printk(KERN_DEBUG PFX "close()\n");

	// release memory chunks reserved by the caller
	s3c_gvg_release_reserved_chunks(file);

	kfree(s3c_gvg_inst);

	return 0;
}

// wait for some hardware operation to complete
//
static void s3c_gvg_wait_for_flush(unsigned int reg, unsigned int bit)
{
	while (1) {
		const unsigned int value = __raw_readl(s3c_gvg.iomem.base + reg);

		if (value & bit)
			break;

		// wait for the ISR to be executed
		wait_event_interruptible(s3c_gvg.wq_irq, 1);
	}

}

static int s3c_gvg_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct s3c_gvg_instance *s3c_gvg_inst = (struct s3c_gvg_instance *)file->private_data;
	struct s3c_gvg_mem_descr mem_descr;

	switch (cmd) {
	case S3C_GVG_WAIT_FOR_CF_FLUSH:
		// wait for command to be completed

		s3c_gvg_wait_for_flush(VG_FIFO_STAT, 0x1);

		break;

	case S3C_GVG_WAIT_FOR_FILTER_FLUSH:
		// wait for filtering to be completed

		s3c_gvg_wait_for_flush(VG_FT_END, 0x1);

		break;

	case S3C_GVG_WAIT_FOR_SCCF_FLUSH:
		// wait for internal SCCF operation to be completed

		s3c_gvg_wait_for_flush(VG_SCCF_STAT, 0x1);

		break;

	case S3C_GVG_MEM_ALLOC:
	{
		// reserve memory

		struct s3c_gvg_chunk *chunk;

		if (copy_from_user(&mem_descr, (void *)arg, sizeof(mem_descr)))
			return -EFAULT;

		chunk = s3c_gvg_reserve_chunk(s3c_gvg_inst, mem_descr.size);
		if (NULL == chunk)
			return -ENOMEM;

		mutex_lock(&s3c_gvg_inst->mmap.lock);

		s3c_gvg_inst->mmap.phys_addr = chunk->descr.phys_addr;

		chunk->descr.virt_addr = do_mmap(file, 0, chunk->descr.size, PROT_READ | PROT_WRITE, MAP_SHARED, 0);

		s3c_gvg_inst->mmap.phys_addr = 0;

		mutex_unlock(&s3c_gvg_inst->mmap.lock);

		if (-EINVAL == chunk->descr.virt_addr) {
			s3c_gvg_release_chunk(s3c_gvg_inst, chunk);
			printk(KERN_WARNING PFX "Failed to mmap chunk\n");

			return -EFAULT;
		}

		if (copy_to_user((void *)arg, &chunk->descr, sizeof(chunk->descr))) {
			s3c_gvg_release_chunk(s3c_gvg_inst, chunk);

			return -EFAULT;
		}

		break;
	}

	case S3C_GVG_MEM_FREE:
	{
		// release reserved memory

		struct s3c_gvg_chunk *chunk;
		int rc;

		if (copy_from_user(&mem_descr, (void *)arg, sizeof(mem_descr)))
			return -EFAULT;

		chunk = s3c_gvg_get_reserved_chunk(s3c_gvg_inst, mem_descr.phys_addr);
		if (NULL == chunk) {
			// no such chunk
			printk(KERN_WARNING PFX "No chunk with physical address 0x%p\n", (void *)mem_descr.phys_addr);
			return -EFAULT;
		}

		s3c_gvg_release_chunk(s3c_gvg_inst, chunk);

		rc = do_munmap(current->mm, mem_descr.virt_addr, mem_descr.size);
		if (rc != 0) {
			printk(KERN_ERR PFX "Unable to unmap memory at 0x%p\n",  (void *)mem_descr.virt_addr);
			return rc;
		}

		mem_descr.size = 0;

		if (copy_to_user((void *)arg, &mem_descr, sizeof(mem_descr)))
			return -EFAULT;

		break;
	}

	case S3C_GVG_TESSELLATE_EDGE_GET:
		// get tesselation memory for edges

#ifdef CONFIG_VIDEO_GVG_USE_KMALLOC
		// we don't use boot memory

		if (NULL == s3c_gvg.mem_edges.virt_addr) {
			// first use => allocate memory
			s3c_gvg.mem_edges.virt_addr = kmalloc(s3c_gvg.mem_edges.size, GFP_KERNEL);
			if ( NULL == s3c_gvg.mem_edges.virt_addr)
				return -ENOMEM;

			s3c_gvg.mem_edges.phys_addr = virt_to_phys(s3c_gvg.mem_edges.virt_addr);
		}
#endif

		mem_descr.phys_addr = s3c_gvg.mem_edges.phys_addr;
		mem_descr.size = s3c_gvg.mem_edges.size;

		DEBUG(PFX "edge memory: phys_addr = 0x%X, size = %d\n",
			mem_descr.phys_addr, mem_descr.size);

		if (copy_to_user((void *)arg, &mem_descr, sizeof(mem_descr)))
			return -EFAULT;

		break;

	case S3C_GVG_TESSELLATE_VERTEX_GET:
		// get tesselation memory for vertexes

#ifdef CONFIG_VIDEO_GVG_USE_KMALLOC
		// we don't use boot memory

		if (NULL == s3c_gvg.mem_vertexes.virt_addr) {
			// first use => allocate memory
			s3c_gvg.mem_vertexes.virt_addr = kmalloc(s3c_gvg.mem_vertexes.size, GFP_KERNEL);
			if (NULL == s3c_gvg.mem_vertexes.virt_addr)
				return -ENOMEM;

			s3c_gvg.mem_vertexes.phys_addr = virt_to_phys(s3c_gvg.mem_vertexes.virt_addr);
		}
#endif

		mem_descr.phys_addr = s3c_gvg.mem_vertexes.phys_addr;
		mem_descr.size = s3c_gvg.mem_vertexes.size;

		DEBUG(PFX "vertex memory: phys_addr = 0x%X, size = %d\n",
			mem_descr.phys_addr, mem_descr.size);

		if (copy_to_user((void *)arg, &mem_descr, sizeof(mem_descr)))
			return -EFAULT;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int s3c_gvg_mmap(struct file* filp, struct vm_area_struct *vma)
{
	unsigned long pfn;
	const unsigned long area_size = vma->vm_end - vma->vm_start;
	struct s3c_gvg_instance *s3c_gvg_inst = (struct s3c_gvg_instance *)filp->private_data;

	if (s3c_gvg_inst->mmap.phys_addr == 0) {
		// it's a normal mapping (of the edge or vertex memory)

		// page frame number of the address for a source G2D_SFR_SIZE to be stored at.
		pfn = __phys_to_pfn(s3c_gvg.iomem.base_phys);
		DEBUG(PFX "mmap: vma->end = 0x%p, vma->start = 0x%p, size = %d\n",
			(void *)(vma->vm_end), (void *)(vma->vm_start), (int)area_size);

		if (area_size > resource_size(s3c_gvg.iomem.res)) {
			printk(KERN_WARNING "mmap: Memory region has an invalid size (%ld)\n", area_size);
			return -EINVAL;
		}
	}
	else {
		// it's a memory reservation (called on behalf of ioctl())

		pfn = __phys_to_pfn(s3c_gvg_inst->mmap.phys_addr);
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED)) {
		printk(KERN_WARNING PFX "Writable VG_SFR_SIZE mapping must be shared !\n");
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, pfn, area_size, vma->vm_page_prot)) {
		printk(KERN_WARNING PFX "Unable to map memory region\n");
		return -EINVAL;
	}

	return 0;
}

static struct file_operations s3c_gvg_fops = {
	.owner	= THIS_MODULE,
	.ioctl	= s3c_gvg_ioctl,
	.open	= s3c_gvg_open,
	.release = s3c_gvg_release,
	.mmap	= s3c_gvg_mmap,
};

static struct miscdevice s3c_gvg_dev = {
	.minor		= GVG_MINOR,
	.name		= "s3c-gvg",
	.fops		= &s3c_gvg_fops,
};

static int s3c_gvg_setup_clock(struct platform_device *pdev)
{
	struct clk *clk;

	clk = clk_get(&pdev->dev, "fimgvg");
	if (IS_ERR(clk)) {
		printk(KERN_WARNING PFX "failed to get fimgvg clk\n");
		return PTR_ERR(clk);
	}

	if (clk_get_rate(clk) != GVG_195MHZ)
		clk_set_rate(clk, GVG_195MHZ);

	clk_enable(clk);

	clk_put(clk);

	return 0;
}

static int s3c_gvg_disable_clock(struct platform_device *pdev)
{
	struct clk *clk;

	clk = clk_get(&pdev->dev, "fimgvg");
	if (IS_ERR(clk)) {
		printk(KERN_WARNING PFX "failed to get fimgvg clk\n");
		return PTR_ERR(clk);
	}

	clk_disable(clk);

	clk_put(clk);

	return 0;
}

static int s3c_gvg_probe(struct platform_device *pdev)
{
	struct resource *res;
	int rc;
	int value;

	printk(KERN_DEBUG PFX "s3c_gvg_probe()\n");

	memset(&s3c_gvg, 0, sizeof(s3c_gvg));

	/* get the memory region for the post processor driver */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR PFX "failed to get memory region resouce\n");
		return -ENOENT;
	}

	s3c_gvg.iomem.base_phys = (unsigned int)res->start;

	s3c_gvg.iomem.res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (s3c_gvg.iomem.res == NULL) {
		printk(KERN_ERR PFX "failed to reserve memory region\n");
		return -ENOENT;
	}

	s3c_gvg.iomem.base = ioremap(s3c_gvg.iomem.res->start, resource_size(s3c_gvg.iomem.res) + 1);
	if (s3c_gvg.iomem.base == NULL) {
		printk(KERN_ERR PFX "ioremap() failed\n");
		rc = -ENOENT;
		goto err_rel_mem_region;
	}

	// initialize variables dealing with tesselation memory
	INIT_LIST_HEAD(&s3c_gvg.available_chunks.list);
	mutex_init(&s3c_gvg.available_chunks.lock);

	s3c_gvg.mem_vertexes.virt_addr = 0;
	s3c_gvg.mem_edges.virt_addr = 0;

#ifdef CONFIG_VIDEO_GVG_USE_BOOTMEM
	s3c_gvg.mem_vertexes.phys_addr = (unsigned int)s3c_get_media_memory(S3C_MDEV_GVG_VERTEX);
	s3c_gvg.mem_vertexes.size = (unsigned int)s3c_get_media_memsize(S3C_MDEV_GVG_VERTEX);

	s3c_gvg.mem_edges.phys_addr = (unsigned int)s3c_get_media_memory(S3C_MDEV_GVG_EDGE);
	s3c_gvg.mem_edges.size = (unsigned int)s3c_get_media_memsize(S3C_MDEV_GVG_EDGE);
#else
	s3c_gvg.mem_vertexes.phys_addr = 0;
	s3c_gvg.mem_vertexes.size = CONFIG_VIDEO_GVG_TESSELLATE_VERTEX_MEM_SIZE * SZ_1K;

	s3c_gvg.mem_edges.phys_addr = 0;
	s3c_gvg.mem_edges.size = CONFIG_VIDEO_GVG_TESSELLATE_EDGE_MEM_SIZE * SZ_1K;
#endif

	// TODO: interrupt (or was it clock?) must be enabled before reading from the iomem
	// printk(KERN_DEBUG PFX "version : 0x%x\n",__raw_readl(s3c_gvg.iomem.base + VG_VERSION));
	printk(KERN_DEBUG PFX "VG_RESERVED_MEM_SIZE : %d MB\n", VG_RESERVED_MEM_SIZE / SZ_1M);
	printk(KERN_DEBUG PFX "VG_CHUNK_SIZE : %d MB\n", VG_CHUNK_SIZE / SZ_1M);
	printk(KERN_DEBUG PFX "VG_CHUNK_NUM : %d\n", VG_CHUNK_NUM);

	rc = s3c_gvg_allocate_chunks();
	if (rc)
		goto err_free_memory;

	init_waitqueue_head(&s3c_gvg.wq_irq);

	// TODO: should we enable the clock only when the device is open?
	rc = s3c_gvg_setup_clock(pdev);
	if (rc)
		goto err_free_memory;

	s3c_gvg.irq = platform_get_irq(pdev, 0);
	if (s3c_gvg.irq <= 0) {
		printk(KERN_ERR PFX "failed to get irq number\n");
		rc = -ENOENT;
		goto err_disable_clock;
	}

	// TODO should we enable the IRQ only when the device is open?
	rc = request_irq(s3c_gvg.irq, s3c_gvg_isr, IRQF_DISABLED, pdev->name, NULL);
	if (rc) {
		printk(KERN_ERR PFX "request_irq failed\n");
		goto err_disable_clock;
	}

	// enable interrupts
	value = __raw_readl(s3c_gvg.iomem.base + VG_INTERRUPT_ENABLE);
	value |= VG_INT_MASK_ALL;
	__raw_writel(value, s3c_gvg.iomem.base + VG_INTERRUPT_ENABLE);

	rc = misc_register(&s3c_gvg_dev);
	if (rc) {
		printk(KERN_ERR PFX "unable to register misc device\n");
		goto err_free_irq;
	}

	return 0;

err_free_irq:
	free_irq(s3c_gvg.irq, NULL);
	s3c_gvg.irq = 0;

err_disable_clock:
	s3c_gvg_disable_clock(pdev);

err_free_memory:
	s3c_gvg_free_chunks();

err_rel_mem_region:
	release_mem_region(res->start, resource_size(res));
	s3c_gvg.iomem.res = 0;

	return rc;
}


static int s3c_gvg_remove(struct platform_device *pdev)
{
	misc_deregister(&s3c_gvg_dev);

	if (s3c_gvg.irq != 0)
		free_irq(s3c_gvg.irq, NULL);

	s3c_gvg_disable_clock(pdev);

	s3c_gvg_free_chunks();

	if (s3c_gvg.iomem.res != NULL) {
		if (s3c_gvg.iomem.base != NULL)
			iounmap(s3c_gvg.iomem.base);

		release_mem_region(s3c_gvg.iomem.res->start, resource_size(s3c_gvg.iomem.res));
		release_resource(s3c_gvg.iomem.res);
	}

#ifdef CONFIG_VIDEO_GVG_USE_KMALLOC
	if (s3c_gvg.mem_vertexes.virt_addr)
		kfree(s3c_gvg.mem_vertexes.virt_addr);

	if (s3c_gvg.mem_edges.virt_addr)
		kfree(s3c_gvg.mem_edges.virt_addr);
#endif

	return 0;
}

static int s3c_gvg_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int s3c_gvg_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver s3c_gvg_driver = {
	.probe          = s3c_gvg_probe,
	.remove         = s3c_gvg_remove,
	.suspend        = s3c_gvg_suspend,
	.resume         = s3c_gvg_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-gvg",
	},
};

static char banner[] __initdata = KERN_INFO "S3C GVG Driver, (c) 2010 Samsung Electronics\n";
static char init_error[] __initdata = KERN_ERR "Intialization of S3C GVG driver is failed\n";

static int __init  s3c_gvg_init(void)
{
	printk(banner);

	if (platform_driver_register(&s3c_gvg_driver) != 0) {
		printk(init_error);
		return -1;
	}

	printk(KERN_DEBUG PFX "init()\n");

	return 0;
}

static void s3c_gvg_exit(void)
{
	platform_driver_unregister(&s3c_gvg_driver);

	printk(KERN_DEBUG PFX "exit()\n");
}

module_init(s3c_gvg_init);
module_exit(s3c_gvg_exit);

MODULE_AUTHOR("gwan.kim@samsung.com");
MODULE_DESCRIPTION("S3C GVG Device Driver");
MODULE_LICENSE("GPL");
