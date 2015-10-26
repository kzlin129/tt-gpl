/*
 * Camera driver for s3c241x camera interface, with ov camchip.
 *
 *              This program is free software; you can redistribute it and/or
 *              modify it under the terms of the GNU General Public License
 *              as published by the Free Software Foundation; either version
 *              2 of the License, or (at your option) any later version.
 *
 * Author:      Ithamar R. Adema, <ithamar.adema@tomtom.com>
 */

#include <linux/interrupt.h>
#include <linux/videodev.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <media/ovcamchip.h>

#include <asm/arch/regs-camif.h>

#if CONFIG_MACH_TOMTOMGO
#include <barcelona/gopins.h>
#endif

#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-camif.h>
#include <asm/kmap_types.h>
#include <asm/cacheflush.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <linux/notifier.h>
#include <linux/cpufreq.h>
#endif

#include "camera.h"
static int fastcp_frame( struct s3c_cam *cam );
extern int cam_stream_off(struct s3c_cam *cam);
extern void s3c_init_camif( void );

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
extern int s3c_cam_freq_policy(struct notifier_block *nb, unsigned long val, void *data);
extern int s3c_cam_freq_transition(struct notifier_block *nb, unsigned long val, void *data);
extern int s3c_cam_check_clockpolicy(__u32 min_freq_mhz, __u32 max_freq_mhz, struct s3c_cam *cam);
#endif

/* Rules for handling shadow copy of frame slots:
 * 1) _only_ cam_poke_slot() can update the shadow copy (and hardware)
 * 2) need to have cam lock held when changing any fields of buffers in that are/could be in the slots
 */
/* Interrupt routine, called when CAMIF codec was successfully received and processed a single frame */
static irqreturn_t
cam_irq(int irqno, void *dev_id, struct pt_regs *regs)
{
	struct s3c_cam		*cam = (struct s3c_cam*)dev_id;

	/* If a target userspace buffer is available, copy the data to it. */
	/* Note that the STATECHANGE flag has two functions in this routine:*/
	/* - If this is the first occurring interrupt, there is no data yet as the IRQ happens before the */
	/*   capturing of the frame. We need to skip this interrupt as it only contains bogus info. */
	/* - If the streamoff function was called, this should be the last interrupt. */
	if( !(cam->flags & S3C_CAM_STATECHANGE) )
	{
		if( cam->userspace.user_ptr )
		{
			dma_sync_single_for_cpu( &cam->pdev->dev, cam->dma_ptr, CAMI_BUFFER_SIZE, DMA_FROM_DEVICE );
			fastcp_frame( cam );
			dma_sync_single_for_device( &cam->pdev->dev, cam->dma_ptr, CAMI_BUFFER_SIZE, DMA_FROM_DEVICE );
			cam->num_done_buffers=1;
		}

		/* Check if we need to work around the problem of the next frame starting to stream while we're still */
		/* copying. The workaround consists of using LASTIRQ to disable the streaming while we are copying. */
		if( cam->slots_in_use == 1 )
		{
			/* Restart the camera. */
			cam->vCIIMGCPT=readl( S3C2413_CIIMGCPT );
			cam->vCIIMGCPT|=S3C2413_CIIMGCPT_IMGCPTEN | S3C2413_CIIMGCPT_DMAEN;
			writel( cam->vCIIMGCPT, S3C2413_CIIMGCPT );

			/* Skip next frame. */
			cam->flags |= S3C_CAM_STATECHANGE;
		}
	}
	else
	{
		/* Camera has started, and first frame was skipped. Clear the STATE_CHANGE flag. */
		cam->flags&=~(S3C_CAM_STATECHANGE);

		/* Check if we need to work around. */
		if( cam->slots_in_use == 1 )
		{
			/* Enable LASTIRQ. */
			writel( cam->vCICOCTRL | S3C2413_CICOCTRL_LASTIRQEN, S3C2413_CICOCTRL );

			/* Stop the camera again. It will stop after the next frame, giving us plenty of time */
			/* to copy the data. */
			cam->vCIIMGCPT=readl( S3C2413_CIIMGCPT );
			cam->vCIIMGCPT&=~(S3C2413_CIIMGCPT_IMGCPTEN | S3C2413_CIIMGCPT_DMAEN);
			writel( cam->vCIIMGCPT, S3C2413_CIIMGCPT );
		}
		
		return IRQ_HANDLED;
	}

	/* Check if the streaming flag has been cleared, stop the camera. */
	if( !(cam->flags & S3C_CAM_STREAMING) )
	{
		/* This is the last IRQ that ought to happen. Stop the camera, and set the STATE_CHANGE flag. */
		cam->vCIIMGCPT=readl( S3C2413_CIIMGCPT );
		cam->vCIIMGCPT&=~(S3C2413_CIIMGCPT_IMGCPTEN | S3C2413_CIIMGCPT_DMAEN);
		writel( cam->vCIIMGCPT, S3C2413_CIIMGCPT );
		cam->flags|=S3C_CAM_STATECHANGE;
	}

	/* Wake up from the queue. */
	wake_up( &cam->inq );
	return IRQ_HANDLED;
}

void disable_cam_irq( struct s3c_cam *cam )
{
	/* We need to be able to stop the camera IRQ. As the local_irq_save/restore functions */
	/* disable the IRQ on the CPU (not the interrupt controller) and the down_read/up_read*/
	/* routines (used for get_user_pages) enable the IRQ on the CPU side again upon exit, */
	/* we need to disable the IRQ on the interrupt controller's side. */
	disable_irq( cam->irq_num );
	return;
}

void enable_cam_irq( struct s3c_cam *cam )
{
	enable_irq( cam->irq_num );
	return;
}

static int 
cam_open(struct inode *inode, struct file *file)
{
	struct s3c_cam		*cam = (struct s3c_cam*)video_get_drvdata(video_devdata(file));
	struct v4l2_rect	defres={0,0,OVCC_MAX_WIDTH, OVCC_MAX_HEIGHT};
        struct resource		*res = NULL;
	int			rc = -EBUSY;

	if( file->f_flags & O_NONBLOCK )
		return -EINVAL;

	if (cam == NULL) {
		rc = -ENODEV;
		goto bail;
	}
	
	spin_lock(&cam->lock);

	if (cam->open)
		goto leave;

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	/* Check if the current clockpolicy allows us to start up the camera. */ 	 
	if (s3c_cam_check_clockpolicy(S3C_CLOCKPOLICY_MIN_FREQ, S3C_CLOCKPOLICY_MAX_FREQ, cam) < 0) { 	 
		dev_err(&cam->pdev->dev, "memorybus too slow to open cam\n"); 	 
		rc=-ENODEV; 	 
		goto leave; 	 
	}
#endif

	/* Make sure the camera chip is awake. */
	if( clk_enable(cam->clk) )
	{
		rc=-ENODEV;
		goto leave;
	}

	/* Prepare the clock. */
	if( clk_use(cam->clk) )
	{
		rc=-ENODEV;
		goto leave;
	}

	/* Prepare the wait queue (to be used for the the read system call. */
	init_waitqueue_head(&cam->inq);

	/* I2C init. */
	cam->ovclient = i2c_get_client(I2C_DRIVERID_OVCAMCHIP, 0, NULL);
	if (cam->ovclient == NULL) {
		dev_err(&cam->pdev->dev, "No sensor detected on i2c bus!\n");
		rc = -ENODEV;
		goto leave;
	}

	/* Mark i2c client as in use */
	i2c_use_client(cam->ovclient);

	/* We have I2C. Wake up the camera chip if it isn't already awake. */
	cam->ovclient->driver->command( cam->ovclient, OVCAMCHIP_CAM_WAKE, NULL );

	/* Program the right resolution settings. */
	if( cam_set_defres( cam, &defres ) != 0 )
	{
		dev_err(&cam->pdev->dev, "Unable to set sensor's mode!\n");
		goto i2c_client_err;
	}

	/* Get the IRQ. */
	if ((res=platform_get_resource(cam->pdev,IORESOURCE_IRQ,0)) == NULL) {
		dev_err(&cam->pdev->dev, "Unable to request IRQ for camera!\n");
		rc = -EIO;
		goto leave;
	}
	
	cam->irq_num = res->start;
	if ((rc=request_irq(cam->irq_num, cam_irq, 0, cam->pdev->name, cam)) != 0) {
		dev_err(&cam->pdev->dev, "Could not install IRQ handler %d (error:%d)!\n", cam->irq_num, rc);
		goto i2c_client_err;
	}

	/* All ok, mark camera as open */
	rc = 0;
	++cam->open;
        file->private_data = cam;
	cam->num_done_buffers=0;
	goto leave;

i2c_client_err:
	cam->ovclient = NULL;
leave:
	spin_unlock(&cam->lock);
bail:
	return rc;
}

int
cam_cclose( struct s3c_cam *cam )
{
	/* Ensure the camera is stopped if we close the device. */
	spin_lock( &cam->lock );
	if( cam->flags & S3C_CAM_STREAMING )
	{
		spin_unlock( &cam->lock );
		cam_stream_off( cam );
	}
	else
	{
		spin_unlock( &cam->lock );
	}

	/* Clear the state change flag just in case. */
	cam->flags&=~(S3C_CAM_STATECHANGE);

	/* Reset the registers. */
	writel(readl(S3C2413_CISRCFMT) | S3C2413_CISRCFMT_FMT_BT601, S3C2413_CISRCFMT);
	writel(readl(S3C2413_CIGCTRL) | S3C2413_CIGCTRL_SWRST, S3C2413_CIGCTRL);
	mdelay(10);

	/* Put the camera to sleep. No need for it any longer. */
	cam->ovclient->driver->command( cam->ovclient, OVCAMCHIP_CAM_SLEEP, NULL );

	/* Release the I2C interface. */
	i2c_release_client(cam->ovclient);
	cam->ovclient=NULL;

	/* Release the clock. */
	if (cam->clk != NULL && !IS_ERR(cam->clk)) {
		clk_unuse(cam->clk);
		clk_disable(cam->clk);
	}

	/* Remove interrupt handler */
	free_irq(cam->irq_num, cam);

	/* Mark as not opened. */
	cam->open=0;

	/* Done. */
	return 0;
}

static int 
cam_close(struct inode *inode, struct file *file)
{
	struct s3c_cam		*cam = (struct s3c_cam*)video_get_drvdata(video_devdata(file));
	int			rc;

	if( cam != NULL )
		rc=cam_cclose( cam );
	else
		rc=-ENODEV;
	return rc;
}

/* NOTE: Calling routine is required to disable camera interrupt before calling this routine! */
static int map_pages( struct s3c_cam *cam, struct task_struct *task, void *pa_user_addr ) 
{
	int			result;
	__u32			framesize;
	__u32			page_offset=((unsigned long int) pa_user_addr) & ~PAGE_MASK;
	int			count;

	/* Determine size and number of pages. */
	framesize=cam->currfmt.fmt.pix.sizeimage;
	if( (framesize + page_offset) & ~PAGE_MASK )
		cam->userspace.nr_pages=(framesize + page_offset)/PAGE_SIZE + 1;
	else
		cam->userspace.nr_pages=(framesize + page_offset)/PAGE_SIZE;

	/* Get the userspace pages. */
	down_read( &task->mm->mmap_sem );
	result=get_user_pages( task, task->mm, ((unsigned long) (pa_user_addr)) & PAGE_MASK,
			       cam->userspace.nr_pages, 1, 0, cam->userspace.page_array, NULL );
	up_read( &task->mm->mmap_sem );
	if( result != cam->userspace.nr_pages )
	{
		cam->userspace.user_ptr=NULL;
		cam->userspace.nr_pages=0;
		return -1;
	}

	/* Now map it all into kernel space. */
	for( count=0; count < cam->userspace.nr_pages; count++ )
	{
		/* Map the whole array into kernelspace. We can get the address of the page using the page_address macro. */
		if( kmap( cam->userspace.page_array[count] ) == NULL )
		{
			for( count-=1; count >=0; count-- )
				kunmap( page_address( cam->userspace.page_array[count] ) );

			for( count=0; count < cam->userspace.nr_pages; count ++ )
				page_cache_release( cam->userspace.page_array[count] );

			cam->userspace.user_ptr=NULL;
			cam->userspace.nr_pages=0;
			return -2;
		}
	}

	/* Lastly set the userspace pointer. */
	cam->userspace.user_ptr=pa_user_addr;

	return 0;
}

/* Note: Calling routine is required to disable camera interrupt before calling this routine. */
void unmap_pages( struct s3c_cam *cam )
{
	int			count=0;

	for( count=0; count < cam->userspace.nr_pages; count++ )
	{
		kunmap( page_address( cam->userspace.page_array[count] ) );

		if( !PageReserved( cam->userspace.page_array[count] ) )
			SetPageDirty( cam->userspace.page_array[count] );
		page_cache_release( cam->userspace.page_array[count] );
	}

	cam->userspace.nr_pages=0;
	cam->userspace.user_ptr=NULL;
	return;
}

/* This routine makes the following assumptions: */
/* addresses (both source and destination are on 4 bytes boundary. Source address is even on cache boundary. */
static int fastcp_frame( struct s3c_cam *cam )
{
	unsigned long int		page_offset=((unsigned long int) cam->userspace.user_ptr) & ~PAGE_MASK;
	register unsigned long int	framesize=cam->currfmt.fmt.pix.sizeimage;
	unsigned long int		index=0;
	register unsigned long int	count;
	register void			*pUserBuf=NULL;
	register void			*pCamBuf=NULL;
	register void			*pCamEndBuf=NULL;
	unsigned long int		framecount;

	/* Get the current frame index. Note that the interrupt currently occurring is for the next frame. We want to copy */
	/* Framecounter always shows the framenumber of the frame to be done after this frame. As the IRQ happens before */
	/* the frame is handled (not counting LASTIRQ), the framenumber of the frame just finished is 2 lower. */
	framecount=(readl( S3C2413_CICOSTATUS ) & S3C2413_CICOSTATUS_FRAMECNT_MASK) >> S3C2413_CICOSTATUS_FRAMECNT_SHIFT;
	framecount=(framecount < 2 ? (framecount + CAMI_MAX_SLOTS - 2) : (framecount - 2));
	framecount%=cam->slots_in_use;

	/* Prepare the copying of the first page or part of it. */
	pUserBuf=page_address( cam->userspace.page_array[0] );
	pCamBuf=cam->kernel_ptr + (framecount * cam->currfmt.fmt.pix.sizeimage);
	pCamEndBuf=pCamBuf + PAGE_SIZE;

	/* Check if the user buffer is 256 bytes aligned. If not, we need to make sure it is before we start. */
	if( page_offset != 0 )
	{
		/* Copy until the next 256 byte boundary. */
		pUserBuf+=page_offset;
		count=((256 - (page_offset & 0x000000FF)) & 0x000000FF);
		memcpy( pUserBuf, pCamBuf, count );

		/* Prepare for fastcopy. */
		pUserBuf+=count;
		pCamBuf+=count;
		pCamEndBuf-=page_offset;
		count=PAGE_SIZE - page_offset;

		/* Check if the page is finished. */
		if( !(((unsigned long int) pUserBuf) & ~(PAGE_SIZE - 1)) )
		{
			/* We are on page alignment. Next address is a new page. */
			framesize-=count;
			pUserBuf=page_address( cam->userspace.page_array[++index] );
			count=(framesize < PAGE_SIZE ? framesize & 0xFFFFFF00 : PAGE_SIZE);
		}
	}
	else
	{
		/* It's aligned on page boundary. No need to worry. */
		count=PAGE_SIZE;
	}

	/* Start copying. */
	while( framesize >= 256 )
	{
		/* Next size. */
		framesize-=count;
	
		/* Copy as fast as possible until the next page boundary. */
		asm volatile(
                        "       stmfd   sp!, {r3-r12, r14}\n"
                        "fastcp_frame_loop:\n"
                        "       ldmia   %0!, {r3-r12, r14}\n"
                        "       stmia   %1!, {r3-r12, r14}\n"

                        "       ldmia   %0!, {r3-r12, r14}\n"
                        "       stmia   %1!, {r3-r12, r14}\n"

                        "       ldmia   %0!, {r3-r12, r14}\n"
                        "       stmia   %1!, {r3-r12, r14}\n"

                        "       ldmia   %0!, {r3-r12, r14}\n"
                        "       stmia   %1!, {r3-r12, r14}\n"

                        "       ldmia   %0!, {r3-r12, r14}\n"
                        "       stmia   %1!, {r3-r12, r14}\n"

                        "       ldmia   %0!, {r3-r11}\n"
                        "       stmia   %1!, {r3-r11}\n"

                        "       cmp     %0, %2\n"
                        "       blo     fastcp_frame_loop\n"
                        "       ldmfd   sp!, {r3-r12, r14}\n" :
                        "=r" (pCamBuf), "=r" (pUserBuf), "=r" (pCamEndBuf) :
                        "0" (pCamBuf), "1" (pUserBuf), "2" (pCamEndBuf) :
                        "cc", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r12", "r14", "memory" );

		/* Set the next address. Ensure that we don't exceed the page array. */
		if( index >= (cam->userspace.nr_pages - 1) ) continue;

		pUserBuf=page_address( cam->userspace.page_array[++index] );

		/* Prepare for the next copy cycle. */
		count=(framesize < PAGE_SIZE ? framesize & 0xFFFFFF00 : PAGE_SIZE);
		pCamEndBuf=pCamBuf+count;
	}

	/* Copy the last bytes. */
	if( framesize != 0 )
		memcpy( pUserBuf, pCamBuf, framesize );

	return 0;
}

static ssize_t
cam_read(struct file *file, char __user * buf, size_t count, loff_t *ppos)
{
	struct s3c_cam		*cam=(struct s3c_cam*)video_get_drvdata(video_devdata(file));
	ssize_t			retval;

	/* Lock the camera, and make sure NO interrupts happen. Then check if there is a buffer queued up already. */
	spin_lock( &cam->lock );
	disable_cam_irq( cam );
	if( count != cam->currfmt.fmt.pix.sizeimage )
	{
		enable_cam_irq( cam );
		spin_unlock( &cam->lock );
		return -EINVAL;
	}

	/* Map the userspace pages into kernelspace. */
	if( map_pages( cam, current, buf ) != 0 )
	{
		enable_cam_irq( cam );
		spin_unlock( &cam->lock );
		return -EIO;
	}

	/* Wait until the camera interface tells us a buffer is ready and copied. */
	/* Note: We check for S3C_CAM_STREAMING also. This is for if we're in a   */
	/* read call, and the cpu_freq is set differently, forcing us to close the*/
	/* camera. */
	while( (cam->num_done_buffers == 0) && (cam->flags & S3C_CAM_STREAMING) )
	{
		enable_cam_irq( cam );
		spin_unlock( &cam->lock );

		/* else wait for a buffer to become available */
		if( wait_event_interruptible(cam->inq, (cam->num_done_buffers > 0) || ((cam->flags & S3C_CAM_STREAMING) == 0) ) )
		{
			spin_lock( &cam->lock );
			disable_cam_irq( cam );
			unmap_pages( cam );
			enable_cam_irq( cam );
			spin_unlock( &cam->lock );
			return -ERESTARTSYS;
		}

		spin_lock( &cam->lock );
		disable_cam_irq( cam );
	}

	/* Unmap all pages. */
	unmap_pages( cam );

	/* Set number of buffers done back to 0. */
	cam->num_done_buffers=0;

	enable_cam_irq( cam );

	/* Set the return value. This depending on if we are still streaming or not. */
	if( cam->flags & S3C_CAM_STREAMING )
		retval=cam->currfmt.fmt.pix.sizeimage;
	else
		retval=-ERESTARTSYS;

	spin_unlock( &cam->lock );
	return retval; 
}

static int
cam_ioctl(struct inode *inode, struct file *file, unsigned cmd, unsigned long arg)
{
	/* We have no ioctl()s of our own yet, just the standard v4l2 ones */
	return s3c_cam_do_ioctl(inode,file,cmd,(void*)arg);
}

struct file_operations
s3c_camif_fops = {
	.owner		= THIS_MODULE,
	.open		= cam_open,
	.release	= cam_close,
	.read		= cam_read,
	.poll		= NULL,
	.mmap		= NULL,
	.ioctl		= cam_ioctl,
	.llseek		= no_llseek,
};
