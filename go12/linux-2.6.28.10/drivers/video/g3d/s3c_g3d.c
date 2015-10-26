/*
 * linux/drivers/video/g3d/s3c_g3d.c
 *
 * Revision 1.0
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    S3C G3D driver
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/errno.h> 	/* error codes */
#include <asm/div64.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <asm/io.h>
#include <asm/irq.h>
//#include <asm/hardware.h>
#include <asm/uaccess.h>
//#include <asm/arch/map.h>
#include <linux/miscdevice.h>
#include <linux/mman.h>

#include <linux/unistd.h>

#include <linux/version.h>
#include <asm/dma.h>

#include <asm/mach/dma.h>
//#include <asm/plat-s3c24xx/dma.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>

#define WINDOW_FLIPPING

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)

#ifdef CONFIG_PLAT_S3C64XX

#define S3C_G3D_PA 	(0x72000000)

#define S3C_G3D_IRQ (IRQ_S3C6410_G3D)

#else

#ifdef CONFIG_PLAT_S5PC1XX

// Starting from 2.6.28 registration is done inside of driver
//#define S3C_G3D_REGISTER_IN_DRIVER

#define S3C_G3D_PA 	(0xEF000000)

#define S3C_G3D_IRQ (IRQ_3D)

#else

#error Unsupported platfotm

#endif // CONFIG_PLAT_S5PC1XX

#endif // CONFIG_PLAT_S3C64XX

#else

#ifdef CONFIG_PLAT_S3C64XX

#define S3C_G3D_PA 	(0x72000000)

#define S3C_G3D_IRQ (IRQ_3D)

#else

#ifdef CONFIG_PLAT_S5PC1XX

#define S3C_G3D_PA 	(0xEF000000)

#define S3C_G3D_IRQ (IRQ_3D)

#else

#error Unsupported platfotm

#endif // CONFIG_PLAT_S5PC1XX

#endif // CONFIG_PLAT_S3C64XX


#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(2,8,0)

#define S3C6410_SZ_G3D 		SZ_4K

#define DEBUG_S3C_G3D
#undef	DEBUG_S3C_G3D

#ifdef DEBUG_S3C_G3D
#define DEBUG(fmt,args...) printk(fmt, ##args)
#else
#define DEBUG(fmt,args...) do {} while(0)
#endif


//#define TN_LIMO_SUPPORT

#ifdef TN_LIMO_SUPPORT
#include <plat/media.h>

#define G3D_RESERVED_MEM_ADDR_PHY                (unsigned int)s3c_get_media_memory(S3C_MDEV_G3D)
#define G3D_RESERVED_MEM_SIZE                       (unsigned int)s3c_get_media_memsize(S3C_MDEV_G3D)

#define G3D_CHUNK_SIZE SZ_2M
#define G3D_CHUNK_NUM 9


static void* Malloc_3D_ChunkMem(unsigned int szReq, int ithMem)
{	
	unsigned long physicAddr = (G3D_RESERVED_MEM_ADDR_PHY + G3D_RESERVED_MEM_SIZE) - (G3D_CHUNK_SIZE * (ithMem+ 1));
	void * virtAddr = phys_to_virt((unsigned long)physicAddr);

	return virtAddr;
}

static void Free_3D_ChunkMem(void* virtAddr)
{	
}

 #else //TN_LIMO_SUPPORT
#define USE_BIG_CHUNK_BLOCK
#ifdef USE_BIG_CHUNK_BLOCK // special customer request
  #define G3D_CHUNK_NUM 2
  #define G3D_CHUNK_SIZE SZ_8M
 #else
#define G3D_CHUNK_NUM 8
#define G3D_CHUNK_SIZE SZ_2M
	#endif

static void* Malloc_3D_ChunkMem(unsigned int szReq, int ithMem)
{
	return kmalloc(szReq, GFP_KERNEL);
}

static void Free_3D_ChunkMem(void* virtAddr)
{
	kfree(virtAddr);
}

#endif


#ifdef WINDOW_FLIPPING

#define FLIP_WIN_NUM				0
static volatile unsigned int * LCDControllerBase=NULL;

#endif

#define G3D_RESERVED_MEM_SIZE (G3D_CHUNK_SIZE * G3D_CHUNK_NUM)


#define G3D_CHUCNK_AVALIABLE		0
#define G3D_CHUCNK_RESERVED		1

struct s3c_g3d_bootmem {
	int		size;
	unsigned int 	vir_addr;
	unsigned int 	phy_addr;
	unsigned int 	in_used;	/* 0 means avaliable, 1 means reserved */
       unsigned int    file_desc_id;
};

struct s3c_g3d_bootmem g3d_bootm[G3D_CHUNK_NUM];

typedef struct{
	unsigned int pool_buffer_addr;
	unsigned int pool_buffer_size;
	unsigned int hardware_has_single_pipeline;
	unsigned int is_dma_available;
	unsigned int dma_buffer_addr;
	unsigned int dma_buffer_size;
} G3D_CONFIG_STRUCT;

typedef struct{
	unsigned int offset; 	// should be word aligned
	unsigned int size; 	// byte size, should be word aligned
} DMA_BLOCK_STRUCT;

typedef struct {
	ulong src;
	ulong dst;
	int len;
} s3c_3d_dma_info;

#define FGGB_PIPESTATE		0x00
#define FGGB_CACHECTL		0x04
#define FGGB_RST		0x08
#define FGGB_VERSION		0x10
#define FGGB_INTPENDING		0x40
#define FGGB_INTMASK		0x44
#define FGGB_PIPEMASK		0x48
#define FGGB_HOSTINTERFACE	0xc000

G3D_CONFIG_STRUCT g3d_config={
#ifdef CONFIG_PLAT_S3C64XX
	S3C_G3D_PA, 	// pool buffer addr
	0x90000, 	// pool_buffer_size
	1, 	// hardware_has_single_pipeline
	0, 	// is_dma_available
#else
#ifdef CONFIG_PLAT_S5PC1XX
    S3C_G3D_PA,     // pool buffer addr
    0x90000,    // pool_buffer_size
    0,  // hardware_has_single_pipeline
    1,   // is_dma_available
#endif
#endif
0x57800000,     //dma buffer addr
0x800000    //dma buffer size
};

#define G3D_IOCTL_MAGIC		'S'
#define WAIT_FOR_FLUSH		_IO(G3D_IOCTL_MAGIC, 100)
#define GET_CONFIG 		_IO(G3D_IOCTL_MAGIC, 101)
#define START_DMA_BLOCK 	_IO(G3D_IOCTL_MAGIC, 102)

#define S3C_3D_MEM_ALLOC		_IOWR(G3D_IOCTL_MAGIC, 310, struct s3c_3d_mem_alloc)
#define S3C_3D_MEM_FREE			_IOWR(G3D_IOCTL_MAGIC, 311, struct s3c_3d_mem_alloc)
#define S3C_3D_SFR_LOCK			_IO(G3D_IOCTL_MAGIC, 312)
#define S3C_3D_SFR_UNLOCK		_IO(G3D_IOCTL_MAGIC, 313)
#define S3C_3D_MEM_ALLOC_SHARE		_IOWR(G3D_IOCTL_MAGIC, 314, struct s3c_3d_mem_alloc)
#define S3C_3D_MEM_SHARE_FREE		_IOWR(G3D_IOCTL_MAGIC, 315, struct s3c_3d_mem_alloc)

#ifdef WINDOW_FLIPPING

#define S3C_3D_FLIPWINDOW		_IOWR(G3D_IOCTL_MAGIC, 316, struct s3c_3d_mem_alloc)

#endif


#define MEM_ALLOC		1
#define MEM_ALLOC_SHARE		2

#define PFX 			"s3c_g3d"
#define G3D_MINOR  		249

static wait_queue_head_t waitq;
static struct resource *s3c_g3d_mem;
static void __iomem *s3c_g3d_base;
static int s3c_g3d_irq;
static struct clk *g3d_clock;
static struct clk *h_clk;

static DEFINE_MUTEX(mem_alloc_lock);
static DEFINE_MUTEX(mem_sfr_lock);

static DEFINE_MUTEX(mem_alloc_share_lock);
static DEFINE_MUTEX(mem_share_free_lock);

void *dma_3d_done;

static struct s3c2410_dma_client s3c6410_3d_dma_client = {
	.name		= "s3c6410-3d-dma",
};


struct s3c_3d_mem_alloc {
	int		size;
	unsigned int 	vir_addr;
	unsigned int 	phy_addr;
};

static unsigned int mutex_lock_processID = 0;

static int flag = 0;

static unsigned int physical_address;

int interrupt_already_recevied;

unsigned int s3c_g3d_base_physical;



///////////// for check memory leak
//*-------------------------------------------------------------------------*/
typedef struct _memalloc_desc
{
	int		size;
	unsigned int 	vir_addr;
	unsigned int 	phy_addr;
	int*    newid;	
	struct _memalloc_desc*  next;
	struct _memalloc_desc*  prev;
} Memalloc_desc;

void grabageCollect(int *newid);

/////////////////////////////////////


irqreturn_t s3c_g3d_isr(int irq, void *dev_id)
{
	__raw_writel(0, s3c_g3d_base + FGGB_INTPENDING);

	interrupt_already_recevied = 1;
	wake_up_interruptible(&waitq);

	return IRQ_HANDLED;
}


unsigned int s3c_g3d_get_current_used_mem(void)
{
	int loop_i;
	int iAvalable = 0;
	int iUsed = 0;
	
	for(loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ){
		if( g3d_bootm[loop_i].in_used == G3D_CHUCNK_AVALIABLE){
			iAvalable++;			
		}
		else { /*G3D_CHUCNK_RESERVED Used Memory*/
			iUsed++;
		}
	}
	printk("used count %d\n", iUsed);

	return iUsed * G3D_CHUNK_SIZE / SZ_1M;
}


void s3c_g3d_dma_finish(struct s3c2410_dma_chan *dma_ch, void *buf_id,
	int size, enum s3c2410_dma_buffresult result){
//	printk("3d dma transfer completed.\n");
	complete(dma_3d_done);
}

int s3c_g3d_open(struct inode *inode, struct file *file)
{
	int *newid;
	
	newid = (int*)vmalloc(sizeof(int));
	file->private_data = newid;
	
	return 0;
}

int s3c_g3d_release(struct inode *inode, struct file *file)
{
	int *newid = file->private_data;
	if(mutex_lock_processID != 0 && mutex_lock_processID == (unsigned int)file->private_data) {
		mutex_unlock(&mem_sfr_lock);
		printk("Abnormal close of pid # %d\n", task_pid_nr(current));        
	}

	grabageCollect(newid);
	vfree(newid);

	return 0;
}


unsigned long s3c_g3d_available_chunk_size(unsigned int request_size)
{
	unsigned int loop_i, loop_j;
	for(loop_j = 0; loop_j < 10; loop_j++ ) {
		for(loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ) {
			if( g3d_bootm[loop_i].in_used == G3D_CHUCNK_AVALIABLE ) {
				if (request_size <= g3d_bootm[loop_i].size)
					return g3d_bootm[loop_i].size;
			}
		}

		mutex_unlock(&mem_alloc_lock);
		printk("wait 0.%d sec to get releaing memory\n", loop_j);
		msleep(100);	
		mutex_lock(&mem_alloc_lock);
	}

	printk("s3c_g3d_available_chunk_size failed : Cannot find adequate memory!\n");
    
	return 0;
}

unsigned long s3c_g3d_reserve_chunk(struct file* filp, unsigned int size)
{
	unsigned int loop_i;

 	for(loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ) {
		if( g3d_bootm[loop_i].in_used == G3D_CHUCNK_AVALIABLE ) {
			if (size != g3d_bootm[loop_i].size)
				return 0;

			g3d_bootm[loop_i].in_used = G3D_CHUCNK_RESERVED;
			g3d_bootm[loop_i].file_desc_id = (int)filp->private_data;

			return g3d_bootm[loop_i].phy_addr;
		}
	}

	printk("s3c_g3d_reserve_chunk failed : Cannot find adequate memory!\n");
    
	return 0;
}

void s3c_g3d_release_chunk(unsigned int phy_addr)
{
	unsigned int loop_i, j;
	struct mm_struct *mm = current->mm;

	for(loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ) {
		if( g3d_bootm[loop_i].phy_addr == phy_addr ) {
			g3d_bootm[loop_i].in_used = G3D_CHUCNK_AVALIABLE;
			g3d_bootm[loop_i].file_desc_id = 0;
			do_munmap(mm, g3d_bootm[loop_i].vir_addr, g3d_bootm[loop_i].size);

			break;
		}
	}
	
	if(loop_i >= G3D_CHUNK_NUM)
		printk("s3c_g3d_release_chunk failed : Cannot find the phys_addr : 0x%p\n", (void*)phy_addr);
#if 1
	j = 0;
	for(loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ){
		if( g3d_bootm[loop_i].in_used == G3D_CHUCNK_AVALIABLE){
			j++;
		}
	}
	printk("Available Count %d\n", j);
#endif
}



static int s3c_g3d_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 val;
	DMA_BLOCK_STRUCT dma_block;
	s3c_3d_dma_info dma_info;
	DECLARE_COMPLETION_ONSTACK(complete);

//	u_int virt_addr;
	struct mm_struct *mm = current->mm;
	struct s3c_3d_mem_alloc param;
	
//	Memalloc_desc   *memdesc;
	switch (cmd) {
	case WAIT_FOR_FLUSH:

		//if fifo has already been flushed, return;
		val = __raw_readl(s3c_g3d_base+FGGB_PIPESTATE);
		//printk("read pipestate = 0x%x\n",val);
		if((val & arg) ==0) break;

		// enable interrupt
		interrupt_already_recevied = 0;
		//__raw_writel(0x0001171f,s3c_g3d_base+FGGB_PIPEMASK);
		__raw_writel(arg,s3c_g3d_base+FGGB_PIPEMASK);
		__raw_writel(1,s3c_g3d_base+FGGB_INTMASK);

		//printk("wait for flush (arg=0x%lx)\n",arg);


		while(1) {
			wait_event_interruptible(waitq, (interrupt_already_recevied>0));
			__raw_writel(0,s3c_g3d_base+FGGB_INTMASK);
			interrupt_already_recevied = 0;
			//if(interrupt_already_recevied==0)interruptible_sleep_on(&waitq);
			val = __raw_readl(s3c_g3d_base+FGGB_PIPESTATE);
			//printk("in while read pipestate = 0x%x\n",val);
			if(val & arg){
			} else{
				break;
			}
			__raw_writel(1,s3c_g3d_base+FGGB_INTMASK);
		}
		break;

	case GET_CONFIG:
		copy_to_user((void *)arg,&g3d_config,sizeof(G3D_CONFIG_STRUCT));
		break;
#ifdef CONFIG_PLAT_S5PC1XX
	case START_DMA_BLOCK:
		copy_from_user(&dma_block,(void *)arg,sizeof(DMA_BLOCK_STRUCT));

		if (dma_block.offset%4!=0) {
			printk("G3D: dma offset is not aligned by word\n");
			return -EINVAL;
		}
		if (dma_block.size%4!=0) {
			printk("G3D: dma size is not aligned by word\n");
			return -EINVAL;
		}
		if (dma_block.offset+dma_block.size >g3d_config.dma_buffer_size) {
			printk("G3D: offset+size exceeds dam buffer\n");
			return -EINVAL;
		}

		dma_info.src = g3d_config.dma_buffer_addr+dma_block.offset;
		dma_info.len = dma_block.size;
		dma_info.dst = s3c_g3d_base_physical+FGGB_HOSTINTERFACE;

		DEBUG(" dma src=0x%x\n", dma_info.src);
		DEBUG(" dma len =%u\n", dma_info.len);
		DEBUG(" dma dst = 0x%x\n", dma_info.dst);

		dma_3d_done = &complete;

		if (s3c2410_dma_request(DMACH_3D_M2M, &s3c6410_3d_dma_client, NULL)) {
			printk(KERN_WARNING "Unable to get DMA channel(DMACH_3D_M2M).\n");
			return -EFAULT;
		}

		s3c2410_dma_set_buffdone_fn(DMACH_3D_M2M, s3c_g3d_dma_finish);
		s3c2410_dma_devconfig(DMACH_3D_M2M, S3C_DMA_MEM2MEM, 1, (u_long) dma_info.src);
		s3c2410_dma_config(DMACH_3D_M2M, 4, 4);
		s3c2410_dma_setflags(DMACH_3D_M2M, S3C2410_DMAF_AUTOSTART);

		//consistent_sync((void *) dma_info.dst, dma_info.len, DMA_FROM_DEVICE);
	//	s3c2410_dma_enqueue(DMACH_3D_M2M, NULL, (dma_addr_t) virt_to_dma(NULL, dma_info.dst), dma_info.len);
		s3c2410_dma_enqueue(DMACH_3D_M2M, NULL, (dma_addr_t) dma_info.dst, dma_info.len);

	//	printk("wait for end of dma operation\n");
		wait_for_completion(&complete);
	//	printk("dma operation is performed\n");

		s3c2410_dma_free(DMACH_3D_M2M, &s3c6410_3d_dma_client);

		break;
#endif
	case S3C_3D_MEM_ALLOC:
		mutex_lock(&mem_alloc_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_alloc_lock);			
			return -EFAULT;
		}
       
		flag = MEM_ALLOC;

             param.size = s3c_g3d_available_chunk_size(param.size);
             
		param.vir_addr = do_mmap(file, 0, param.size, PROT_READ|PROT_WRITE, MAP_SHARED, 0);
		DEBUG("param.vir_addr = %08x\n", param.vir_addr);

		if(param.vir_addr == -EINVAL) {
			printk("S3C_3D_MEM_ALLOC FAILED\n");
			flag = 0;
			mutex_unlock(&mem_alloc_lock);			
			return -EFAULT;
		}
		param.phy_addr = physical_address;        

		// printk("alloc %d\n", param.size);
		DEBUG("KERNEL MALLOC : param.phy_addr = 0x%X \t size = %d \t param.vir_addr = 0x%X\n", param.phy_addr, param.size, param.vir_addr);

		if(copy_to_user((struct s3c_3d_mem_alloc *)arg, &param, sizeof(struct s3c_3d_mem_alloc))){
			flag = 0;
			mutex_unlock(&mem_alloc_lock);
			return -EFAULT;		
		}

		flag = 0;
		
		printk("\n\n====Success the malloc from kernel=====\n");
		mutex_unlock(&mem_alloc_lock);
		
		break;
		
	case S3C_3D_MEM_FREE:	
		mutex_lock(&mem_alloc_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_alloc_lock);
			return -EFAULT;
		}

		DEBUG("KERNEL FREE : param.phy_addr = 0x%X \t size = %d \t param.vir_addr = 0x%X\n", param.phy_addr, param.size, param.vir_addr);

		/*
		if (do_munmap(mm, param.vir_addr, param.size) < 0) {
			printk("do_munmap() failed !!\n");
			mutex_unlock(&mem_alloc_lock);
			return -EINVAL;
		}
		*/

		s3c_g3d_release_chunk(param.phy_addr);
		//printk("KERNEL : virt_addr = 0x%X\n", virt_addr);
		//printk("free %d\n", param.size);


		param.size = 0;
		DEBUG("do_munmap() succeed !!\n");

		if(copy_to_user((struct s3c_3d_mem_alloc *)arg, &param, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_alloc_lock);
			return -EFAULT;
		}

		mutex_unlock(&mem_alloc_lock);
		
		break;
	case S3C_3D_SFR_LOCK:
		mutex_lock(&mem_sfr_lock);
		mutex_lock_processID = (unsigned int)file->private_data;
		DEBUG("s3c_g3d_ioctl() : You got a muxtex lock !!\n");
		break;

	case S3C_3D_SFR_UNLOCK:
		mutex_lock_processID = 0;
		mutex_unlock(&mem_sfr_lock);
		DEBUG("s3c_g3d_ioctl() : The muxtex unlock called !!\n");
		break;

	case S3C_3D_MEM_ALLOC_SHARE:		
		mutex_lock(&mem_alloc_share_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_alloc_share_lock);
			return -EFAULT;
		}
		flag = MEM_ALLOC_SHARE;

		physical_address = param.phy_addr;

		DEBUG("param.phy_addr = %08x\n", physical_address);

		param.vir_addr = do_mmap(file, 0, param.size, PROT_READ|PROT_WRITE, MAP_SHARED, 0);
		DEBUG("param.vir_addr = %08x\n", param.vir_addr);

		if(param.vir_addr == -EINVAL) {
			printk("S3C_3D_MEM_ALLOC_SHARE FAILED\n");
			flag = 0;
			mutex_unlock(&mem_alloc_share_lock);
			return -EFAULT;
		}

		DEBUG("MALLOC_SHARE : param.phy_addr = 0x%X \t size = %d \t param.vir_addr = 0x%X\n", param.phy_addr, param.size, param.vir_addr);

		if(copy_to_user((struct s3c_3d_mem_alloc *)arg, &param, sizeof(struct s3c_3d_mem_alloc))){
			flag = 0;
			mutex_unlock(&mem_alloc_share_lock);
			return -EFAULT;		
		}

		flag = 0;
		
		mutex_unlock(&mem_alloc_share_lock);
		
		break;

	case S3C_3D_MEM_SHARE_FREE:	
		mutex_lock(&mem_share_free_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_share_free_lock);
			return -EFAULT;		
		}

		DEBUG("MEM_SHARE_FREE : param.phy_addr = 0x%X \t size = %d \t param.vir_addr = 0x%X\n", param.phy_addr, param.size, param.vir_addr);

		if (do_munmap(mm, param.vir_addr, param.size) < 0) {
			printk("do_munmap() failed - MEM_SHARE_FREE!!\n");
			mutex_unlock(&mem_share_free_lock);
			return -EINVAL;
		}

		param.vir_addr = 0;
		DEBUG("do_munmap() succeed !! - MEM_SHARE_FREE\n");

		if(copy_to_user((struct s3c_3d_mem_alloc *)arg, &param, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_share_free_lock);
			return -EFAULT;		
		}

		mutex_unlock(&mem_share_free_lock);
		
		break;
#ifdef WINDOW_FLIPPING
	case S3C_3D_FLIPWINDOW:
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){		
			return -EFAULT;
		}		

		#if (FLIP_WIN_NUM == 0)		
		LCDControllerBase[0xa0/4] = param.phy_addr;
		LCDControllerBase[0xd0/4] = (param.phy_addr+param.size)&0xffffff;
		#elif (FLIP_WIN_NUM == 1)
		LCDControllerBase[0xa8/4] = param.phy_addr;
		LCDControllerBase[0xd8/4] = (param.phy_addr+param.size)&0xffffff;		
		#elif (FLIP_WIN_NUM == 2)
		LCDControllerBase[0xb0/4] = param.phy_addr;
		LCDControllerBase[0xe0/4] = (param.phy_addr+param.size)&0xffffff;
		#elif (FLIP_WIN_NUM == 3)
		LCDControllerBase[0xb8/4] = param.phy_addr;
		LCDControllerBase[0xe8/4] = (param.phy_addr+param.size)&0xffffff;		
		#else
		LCDControllerBase[0xc0/4] = param.phy_addr;
		LCDControllerBase[0xf0/4] = (param.phy_addr+param.size)&0xffffff;			
		#endif
		
		break;
#endif
	default:
		DEBUG("s3c_g3d_ioctl() : default !!\n");
		return -EINVAL;
	}
	
	return 0;
}

int s3c_g3d_mmap(struct file* filp, struct vm_area_struct *vma)
{
	unsigned long pageFrameNo, size, virt_addr = 0, phys_addr;

	size = vma->vm_end - vma->vm_start;

	switch (flag) { 
	case MEM_ALLOC :
		phys_addr = s3c_g3d_reserve_chunk(filp, size);

		if (phys_addr == 0) {
			printk("There is no reserved memory for G3D!\n");
			return -EINVAL;
		}

		DEBUG("MMAP_ALLOC : virt addr = 0x%p, size = %d\n", virt_addr, size);
		printk("MMAP_ALLOC : virt addr = 0x%p, phys addr = 0x%p, size = %d\n", (void*)virt_addr, (void*)phys_addr, (int)(size));
		physical_address = (unsigned int)phys_addr;

		pageFrameNo = __phys_to_pfn(phys_addr);
		break;

	case MEM_ALLOC_SHARE :
		DEBUG("MMAP_KMALLOC_SHARE : phys addr = 0x%p\n", physical_address);
		
		// page frame number of the address for the physical_address to be shared.
		pageFrameNo = __phys_to_pfn(physical_address);
		//DEBUG("MMAP_KMALLOC_SHARE: PFN = 0x%x\n", pageFrameNo);
		DEBUG("MMAP_KMALLOC_SHARE : vma->end = 0x%p, vma->start = 0x%p, size = %d\n", vma->vm_end, vma->vm_start, size);
		break;
		
	default :
		printk("here\n");

		// page frame number of the address for a source G2D_SFR_SIZE to be stored at.
		pageFrameNo = __phys_to_pfn(S3C_G3D_PA);
		DEBUG("MMAP : vma->end = 0x%p, vma->start = 0x%p, size = %d\n", vma->vm_end, vma->vm_start, size);

		if(size > S3C6410_SZ_G3D) {
			printk("The size of G3D_SFR_SIZE mapping is too big!\n");
			return -EINVAL;
		}
		break;
	}
	
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED)) {
		printk("s3c_g3d_mmap() : Writable G3D_SFR_SIZE mapping must be shared !\n");
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, pageFrameNo, size, vma->vm_page_prot)) {
		printk("s3c_g3d_mmap() : remap_pfn_range() failed !\n");
		return -EINVAL;
	}

	return 0;
}

void grabageCollect(int* newid)
{
        int loop_i;

        printk("====Garbage Collect is executed====\n");
        mutex_lock(&mem_alloc_lock);

	for(loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ){
		if((g3d_bootm[loop_i].file_desc_id) == (unsigned int)newid){
				g3d_bootm[loop_i].file_desc_id = 0;
				if (g3d_bootm[loop_i].in_used == G3D_CHUCNK_RESERVED){
	        		s3c_g3d_release_chunk(g3d_bootm[loop_i].phy_addr);
				}
	        }
       }
        mutex_unlock(&mem_alloc_lock);  
}    

static struct file_operations s3c_g3d_fops = {
	.owner 	= THIS_MODULE,
	.ioctl 	= s3c_g3d_ioctl,
	.open 	= s3c_g3d_open,
	.release = s3c_g3d_release,
	.mmap	= s3c_g3d_mmap,
};


static struct miscdevice s3c_g3d_dev = {
	.minor		= G3D_MINOR,
	.name		= "s3c-g3d",
	.fops		= &s3c_g3d_fops,
};

static int s3c_g3d_remove(struct platform_device *dev)
{
	//clk_disable(g3d_clock);

#ifdef WINDOW_FLIPPING
	if(LCDControllerBase)
		iounmap(LCDControllerBase);
#endif

	free_irq(s3c_g3d_irq, NULL);

	if (s3c_g3d_mem != NULL) {
		pr_debug("s3c_g3d: releasing s3c_post_mem\n");
		iounmap(s3c_g3d_base);
		release_resource(s3c_g3d_mem);
		kfree(s3c_g3d_mem);
	}

	misc_deregister(&s3c_g3d_dev);
	return 0;
}

int s3c_g3d_probe(struct platform_device *pdev)
{
	struct resource *res;

	int ret;
	int i, loop_i;

	DEBUG("s3c_g3d probe() called\n");

	s3c_g3d_irq = platform_get_irq(pdev, 0);


	if(s3c_g3d_irq <= 0) {
		printk(KERN_ERR PFX "failed to get irq resouce\n");
		return -ENOENT;
	}

	/* get the memory region for the post processor driver */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res == NULL) {
		printk(KERN_ERR PFX "failed to get memory region resouce\n");
		return -ENOENT;
	}

	s3c_g3d_base_physical = (unsigned int)res->start;

	s3c_g3d_mem = request_mem_region(res->start, res->end-res->start+1, pdev->name);
	if(s3c_g3d_mem == NULL) {
		printk(KERN_ERR PFX "failed to reserve memory region\n");
		return -ENOENT;
	}


	s3c_g3d_base = ioremap(s3c_g3d_mem->start, s3c_g3d_mem->end - res->start + 1);
	if(s3c_g3d_base == NULL) {
		printk(KERN_ERR PFX "failed ioremap\n");
		return -ENOENT;
	}

	g3d_clock = clk_get(&pdev->dev, "post");
	if(g3d_clock == NULL) {
		printk(KERN_ERR PFX "failed to find post clock source\n");
		return -ENOENT;
	}

	clk_enable(g3d_clock);

	h_clk = clk_get(&pdev->dev, "hclk");
	if(h_clk == NULL) {
		printk(KERN_ERR PFX "failed to find h_clk clock source\n");
		return -ENOENT;
	}

	init_waitqueue_head(&waitq);

	ret = misc_register(&s3c_g3d_dev);
	if (ret) {
		printk (KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
				G3D_MINOR, ret);
		return ret;
	}

	// device reset
	__raw_writel(1,s3c_g3d_base+FGGB_RST);
	for(i=0;i<1000;i++);
	__raw_writel(0,s3c_g3d_base+FGGB_RST);
	for(i=0;i<1000;i++);

	ret = request_irq(s3c_g3d_irq, s3c_g3d_isr, IRQF_DISABLED,
			pdev->name, NULL);
	if (ret) {
		printk("request_irq(S3D) failed.\n");
		return ret;
	}

	printk("s3c_g3d version : 0x%x\n",__raw_readl(s3c_g3d_base + FGGB_VERSION));
	printk("G3D_RESERVED_MEM_SIZE : %d MB\n", G3D_RESERVED_MEM_SIZE/SZ_1M);
	printk("G3D_CHUNK_SIZE : %d MB\n", G3D_CHUNK_SIZE/SZ_1M);
	printk("G3D_CHUNK_NUM : %d\n", G3D_CHUNK_NUM);

	for( loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ){
		g3d_bootm[loop_i].vir_addr = (unsigned int)Malloc_3D_ChunkMem(G3D_CHUNK_SIZE, loop_i);
		g3d_bootm[loop_i].phy_addr = (unsigned int)virt_to_phys((void*)g3d_bootm[loop_i].vir_addr);
		g3d_bootm[loop_i].in_used = G3D_CHUCNK_AVALIABLE;
		g3d_bootm[loop_i].size = G3D_CHUNK_SIZE;
		g3d_bootm[loop_i].file_desc_id = 0;

		printk("%d th virt_addr = 0x%p, phy_addr = 0x%p\n", (int)loop_i, (void*)(g3d_bootm[loop_i].vir_addr), (void*)(g3d_bootm[loop_i].phy_addr));
	}

#ifdef WINDOW_FLIPPING

	LCDControllerBase = (volatile unsigned int *)ioremap(0x77100000,1024);

#endif

	/* check to see if everything is setup correctly */
	return 0;
}

static int s3c_g3d_suspend(struct platform_device *dev, pm_message_t state)
{
	//clk_disable(g3d_clock);
	return 0;
}
static int s3c_g3d_resume(struct platform_device *pdev)
{
	//clk_enable(g3d_clock);
	return 0;
}

/*static struct platform_driver s3c_g3d_driver = {
	.probe          = s3c_g3d_probe,
	.remove         = s3c_g3d_remove,
	.suspend        = s3c_g3d_suspend,
	.resume         = s3c_g3d_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-g3d",
	},
};*/

static struct platform_driver s3c_g3d_driver = {
	.driver		= {
		.name	= "s3c-g3d",
	},
	.probe          = s3c_g3d_probe,
	.remove         = s3c_g3d_remove,
};

static char banner[] __initdata = KERN_INFO "S3C G3D Driver, (c) 2007-2009 Samsung Electronics\n";

static char init_error[] __initdata = KERN_ERR "Intialization of S3C G3D driver is failed\n";

#ifdef S3C_G3D_REGISTER_IN_DRIVER

static struct resource s3c_g3d_resource[] = {
	[0] = {
		.start = S3C_G3D_PA,
		.end   = S3C_G3D_PA + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
                .start = S3C_G3D_IRQ,
                .end   = S3C_G3D_IRQ,
                .flags = IORESOURCE_IRQ,
        }
};

static struct platform_device *s3c_g3d_device; 

#endif // S3C_G3D_REGISTER_IN_DRIVER

int __init  s3c_g3d_init(void)
{
	printk(banner);

	if(platform_driver_register(&s3c_g3d_driver)!=0) {
		printk(init_error);
		return -1;
	}

#ifdef S3C_G3D_REGISTER_IN_DRIVER

	s3c_g3d_device = platform_device_alloc("s3c-g3d", -1);
	if (s3c_g3d_device == 0) {
		printk(init_error);

		platform_driver_unregister(&s3c_g3d_driver);

		return -1;
	}

	if (platform_device_add_resources(s3c_g3d_device, &s3c_g3d_resource, ARRAY_SIZE(s3c_g3d_resource)) < 0) {
		printk(init_error);

		platform_device_put(s3c_g3d_device);
		platform_driver_unregister(&s3c_g3d_driver);

		return -1;
	}

	if (platform_device_add(s3c_g3d_device) < 0) {
		printk(init_error);

		platform_device_put(s3c_g3d_device);
		platform_driver_unregister(&s3c_g3d_driver);

		return -1;
	}

#endif // S3C_G3D_REGISTER_IN_DRIVER

	return 0;
}

void  s3c_g3d_exit(void)
{
#ifdef S3C_G3D_REGISTER_IN_DRIVER
	platform_device_unregister(s3c_g3d_device);
#endif // S3C_G3D_REGISTER_IN_DRIVER
        int loop_i;
	platform_driver_unregister(&s3c_g3d_driver);

	for( loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ){
		Free_3D_ChunkMem((void*)g3d_bootm[loop_i].vir_addr);
		
		g3d_bootm[loop_i].vir_addr = 0;
		g3d_bootm[loop_i].phy_addr = 0;
		g3d_bootm[loop_i].in_used = G3D_CHUCNK_AVALIABLE;
		g3d_bootm[loop_i].file_desc_id = 0;
		g3d_bootm[loop_i].size = 0;

		printk("%d th virt_addr = 0x%p, phy_addr = 0x%p\n", loop_i, (void*)(g3d_bootm[loop_i].vir_addr), (void*)(g3d_bootm[loop_i].phy_addr));
	}

	printk("S3C G3D module exit\n");
}

module_init(s3c_g3d_init);
module_exit(s3c_g3d_exit);

MODULE_AUTHOR("lee@samsung.com");
MODULE_DESCRIPTION("S3C G3D Device Driver");
MODULE_LICENSE("GPL");

