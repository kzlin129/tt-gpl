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

#include <linux/config.h>
#include <linux/slab.h>
#include <linux/videodev.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>                     
#include <asm/arch/regs-camif.h>                                
#include <asm/arch/regs-gpioj.h>
#include <asm/arch/map.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-gpio.h>                                
#include <asm/hardware/clock.h>
#include <asm/system.h>
#include <media/ovcamchip.h>
#include <barcelona/gopins.h>
#include <linux/bootmem.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include "camera.h"

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <linux/cpufreq.h>
#include <barcelona/cpufreq_order.h>
#endif

extern void unmap_pages( struct s3c_cam *cam );
extern void disable_cam_irq( struct s3c_cam *cam );
extern void enable_cam_irq( struct s3c_cam *cam );
extern int cam_cclose( struct s3c_cam *cam );
extern int cam_stream_on(struct s3c_cam *cam);
extern int cam_stream_off(struct s3c_cam *cam);

static struct video_device 
s3c_camif_template = {
	.name		= "s3c2412-camif",
	.type		= VID_TYPE_CAPTURE,
	.hardware	= VID_HARDWARE_S3C2413,
	.release	= video_device_release,
	.fops		= &s3c_camif_fops,
	.minor		= 0
};

static inline struct s3c_cam*
device_to_cam(struct device* dev)
{
	return (struct s3c_cam*)dev_get_drvdata(dev);
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
/*
 * CPU clock speed change handler. 
 * s3c cam is using HCLK based memory-mapped IO and HCLK OR external UPLL clock
 * check for memory bandwidth and MEMCLK has to be > camera's UPLL derived clock
 */
int s3c_cam_freq_transition(struct notifier_block *nb, unsigned long val, void *data)
{
	struct s3c_cam		*cam=container_of( nb, struct s3c_cam, cpu_transition_notifier );
	unsigned long int	old_hclk;
	unsigned long int	new_hclk;
	unsigned long int	clkdivn;
	struct cpufreq_freqs	*f = data;

	f->trans2hclk( f, &old_hclk, &new_hclk );

	switch( val )
	{
		case CPUFREQ_PRECHANGE :
		{
			/* Before the change. Stop streaming (temporarily). */
			if( cam->open )
			{
				if( cam->flags & S3C_CAM_STREAMING )
				{
					/* Stop streaming. */
					cam_stream_off( cam );
				}

				/* Stop the clock (against artifacts) */
				clk_disable( cam->clk );
			}
			break;
		}

		case CPUFREQ_POSTCHANGE :
		{
			/* Calculate the new divider. */
			cam->curr_divider=(new_hclk*1000 + S3C_CLOCKPOLICY_MAX_FREQ)/S3C_CLOCKPOLICY_MAX_FREQ;

			/* Restart streaming. It's ok again. */
			if( (((1000*new_hclk)/cam->curr_divider) >= S3C_CLOCKPOLICY_MIN_FREQ) &&
			    (((1000*new_hclk)/cam->curr_divider) <= S3C_CLOCKPOLICY_MAX_FREQ) )
			{
				/* Get the current clock divider register value. */
				clkdivn=readl( S3C2410_CLKDIVN );
				clkdivn&=~(0x0F << 16);
				clkdivn|=((cam->curr_divider - 1) & 0x0F) << 16;

				/* Program the new divider. */
				writel( clkdivn, S3C2410_CLKDIVN );

				/* Check if it was still running. */
				if( cam->open )
				{
					/* After the change. Start the clock again. */
					clk_enable( cam->clk );

					if( cam->flags & S3C_CAM_STREAMING )
					{
						/* Restart streaming. */
						cam_stream_on( cam );
					}
				}
			}
			else
			{
				/* Illegal frequency detected. */
				printk( KERN_WARNING "Camera disabled. Invalid input frequency selected.\n" );
			}
			break;
		}
	}
	return 0;
}

int s3c_cam_check_clockpolicy(__u32 min_freq_mhz, __u32 max_freq_mhz, struct s3c_cam *cam)
{
	/* This routine checks the clockpolicy. Note that it's heavily dependant on whether we are running from */
	/* USYSCLK or from HCLK. */
#ifdef OVCLOCK_REALLY_24MHZ
	/* We're running from USYSCLK. The CPUFREQ code assumes only MREFCLK is changed, not UREFCLK, so no need*/
	/* to change anything. */
	return 0;
#else
	struct clk		*hclk;
	unsigned long int	curr_cam_freq;

	hclk=clk_get( &cam->pdev->dev, "hclk" );
	if( IS_ERR( hclk ) )
	{
		/* Can't determine HCLK. Assume we can't get it. */
		return -1;
	}

	/* Calculate cam frequency. Note we use the actual divider from the registers. If this is not a valid value */
	/* the current frequency will be out of boundaries. */
	curr_cam_freq=clk_get_rate( hclk )/cam->curr_divider;
	clk_put( hclk );
	
	/* We're running from HCLK. This means that changes in MREFCLK (where HCLK is derived from not only */
	/* change the CPUFREQ, it also changes CAMFREQ. */ 
	/* Check current HCLK. */
	if( (min_freq_mhz <= curr_cam_freq) && (max_freq_mhz >= curr_cam_freq) )
		return 0;
	else
		return -1;
#endif
}

int s3c_cam_freq_policy(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_policy	*policy=data;
	struct s3c_cam		*cam=container_of( nb, struct s3c_cam, cpu_policy_notifier );
	unsigned long int	min_freq=(S3C_CLOCKPOLICY_MIN_FREQ*1)/1000;
	unsigned long int	max_freq=(S3C_CLOCKPOLICY_MAX_FREQ*16)/1000;
	unsigned long int	low_hclk;
	unsigned long int	high_hclk;

	policy->policy2hclk( policy, &low_hclk, &high_hclk );

	switch (val) {
		case CPUFREQ_ADJUST:
			/* Set the minimum and maximum for the camera. */
			if( cam->open )
			{
				policy->hclk2policy( policy, low_hclk, high_hclk );
			}
			break;

		case CPUFREQ_INCOMPATIBLE:
			/* Check if the new values are compatible. */
			if( cam->open )
			{
				if( (low_hclk >= min_freq) && (low_hclk <= max_freq) )
					min_freq=low_hclk;

				if( (high_hclk >= min_freq) && (high_hclk <= max_freq) )
					max_freq=high_hclk;

				if( (high_hclk != max_freq) || (low_hclk != min_freq) )
					policy->hclk2policy( policy, min_freq, max_freq );
			}
			break;
			
		case CPUFREQ_NOTIFY:
			/* Handling of the invalid frequency is done in the transition routine. */
			break;
	}
	return 0;
}
#endif

void s3c_init_camif( void )
{
	unsigned long int	regval;

	/* Make sure power down and reset are set correctly. */
	IO_Deactivate(CAM_DPWDN);
	msleep( 1 );

	/* Reset the camera interface registers. */
	regval=readl( S3C2413_CISRCFMT ) | S3C2413_CISRCFMT_FMT_BT601;
	writel( regval, S3C2413_CISRCFMT );
	regval=readl( S3C2413_CIGCTRL ) | S3C2413_CIGCTRL_SWRST;
	writel( regval, S3C2413_CIGCTRL );
	msleep( 1 );
	writel( regval & ~(S3C2413_CIGCTRL_SWRST), S3C2413_CIGCTRL );

	/* Setup camera pins */
	IO_SetFunction(CAMRESET);
	IO_SetFunction(CAMCLKOUT);
	IO_SetFunction(CAMPCLK);
	IO_SetFunction(CAMVSYNC);
	IO_SetFunction(CAMHREF);
	IO_SetFunction(CAMDATA0);
	IO_SetFunction(CAMDATA1);
	IO_SetFunction(CAMDATA2);
	IO_SetFunction(CAMDATA3);
	IO_SetFunction(CAMDATA4);
	IO_SetFunction(CAMDATA5);
	IO_SetFunction(CAMDATA6);
	IO_SetFunction(CAMDATA7);

	/* Enable the pull down regs. */
	writel( 0, S3C2410_GPIOREG(0x84) );
	writel( 0, S3C2410_GPIOREG(0x88) );

	/* Reset the camera. */
	regval=readl( S3C2413_CIGCTRL ) & ~(S3C2413_CIGCTRL_CAMRST);
	writel( regval, S3C2413_CIGCTRL );
	msleep( 1 );
	regval=readl( S3C2413_CIGCTRL ) | S3C2413_CIGCTRL_CAMRST;
	writel( regval, S3C2413_CIGCTRL );
	msleep( 1 );

	/* Done. */
	return;
}

static int
s3c_cam_probe(struct device *dev)
{
	struct s3c_cam* camdev=NULL;
	struct resource *camres=NULL;
	int result = 0;

	printk(KERN_INFO "TomTom GO Camera Driver, (C) 2006 TomTom BV\n");

	camdev = kcalloc(1, sizeof(*camdev), GFP_KERNEL);
	if (camdev == NULL) {
		dev_err(dev, "Cannot allocate our camera structure!\n");
		result = -ENOMEM;
		goto err_exit;
	}

	/* Allocate a video device */
	camdev->vdev = video_device_alloc();
	if (camdev->vdev == NULL) {
		dev_err(dev, "Unable to allocate video device!\n");
		result = -ENOMEM;
		goto err_dev;
	}

	/* Copy our video_device template into the allocated device */
	*(camdev->vdev) = s3c_camif_template;
	camdev->vdev->owner = THIS_MODULE;
	camdev->vdev->dev = dev;
	video_set_drvdata(camdev->vdev, camdev);
	dev_set_drvdata(dev, camdev);
	camdev->pdev = to_platform_device(dev);

	/* Configure the camera interface and bring it in reset state. */
	s3c_init_camif( );

	/* Prepare the clock. */
	camdev->clk = clk_get(&camdev->pdev->dev, "cam");
	if (IS_ERR(camdev->clk)) {
		dev_err(&camdev->pdev->dev, "cannot get clock\n");
		result = -ENOENT;
		goto err_clk_get;
	}

	if( clk_use(camdev->clk) )
	{
		result=-ENODEV;
		goto err_clk_use;
	}

	if( clk_enable(camdev->clk) )
	{
		result=-ENODEV;
		goto err_clk_ena;
	}

	/* Setup defaults for the V4L variables. These are just for after the module has loaded, any settings
	 * changes an application will do will be remembered over different open/closes of the driver. */
	s3c_cam_setup_defaults(camdev);

	/* Register our camera with video4linux... */
	if ((result=video_register_device(camdev->vdev, VFL_TYPE_GRABBER, -1)) != 0) {
		dev_err(dev, "Cannot register the video device (%x)!\n", result);
		goto err_dev;
	}

	/* Map the memory for the capture buffer. */
	camres=platform_get_resource( camdev->pdev, IORESOURCE_DMA | IORESOURCE_MEM, 0 );
	if( (camres == NULL) || (camres->start == 0) || ((camres->end - camres->start) < CAMI_BUFFER_SIZE) )
	{
		dev_err( dev, "Cannot get camera framebuffer memory, or not enough memory available!\n" );
		goto err_dmamem;
	}

	/* First kernel... */
	camdev->kernel_ptr=(unsigned char *) camres->start;
	if( camdev == NULL)
	{
		dev_err( dev, "Cannot allocate DMA buffer.\n" );
		result = -ENOMEM;
		goto err_dmamem;
	}

	/* Then DMA... */
	camdev->dma_ptr=dma_map_single( &camdev->pdev->dev, camdev->kernel_ptr, CAMI_BUFFER_SIZE, DMA_FROM_DEVICE );
	if( camdev->dma_ptr == 0 )
	{
		dev_err( dev, "Cannot map camera framebuffer memory to DMA address space.\n" );
		result=-ENODEV;
		goto err_dmamem;
	}

	/* Allocate for the pages array. Always allocate the maximum frames that can be used so we can never run into */
	/* The situation where we do not have enough memory available. For this reason we assume RGB24 (something which*/
	/* will likely not happen). Add one incase the division by PAGE_SIZE gives a fraction. */
	camdev->userspace.page_array=(struct page **) kcalloc( (CAMI_MAX_WIDTH*CAMI_MAX_HEIGHT*sizeof( __u32 )/PAGE_SIZE) + 1,
								sizeof( struct page * ), GFP_KERNEL );
	if( camdev->userspace.page_array == NULL )
	{
		dev_err( dev, "Allocation of the page array buffer failed.\n" );
		result=-ENOMEM;
		goto err_dmamap;
	}

	camdev->userspace.nr_pages=0;

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	/* Register the notifiers. Note: If something goes wrong, unregister them. */
	memset( &(camdev->cpu_policy_notifier), 0, sizeof( camdev->cpu_policy_notifier ) );
	memset( &(camdev->cpu_transition_notifier), 0, sizeof( camdev->cpu_transition_notifier ) );
	camdev->cpu_policy_notifier.notifier_call=s3c_cam_freq_policy;
	camdev->cpu_transition_notifier.notifier_call=s3c_cam_freq_transition;
	camdev->cpu_transition_notifier.priority=CPUFREQ_ORDER_S3C24XX_CAMERA_PRIO;
	cpufreq_register_notifier(&(camdev->cpu_policy_notifier), CPUFREQ_POLICY_NOTIFIER);
	cpufreq_register_notifier(&(camdev->cpu_transition_notifier), CPUFREQ_TRANSITION_NOTIFIER);
#endif

	/* Done with the camera. Stop the clock. We can't put it to sleep as we don't have the I2C interface to the */
	/* camera chip available. */
	clk_disable( camdev->clk );

	spin_lock_init( &camdev->lock );

	/* All ok */
	return 0;
err_dmamap:
	dma_unmap_single( &camdev->pdev->dev, camdev->dma_ptr, CAMI_BUFFER_SIZE, DMA_FROM_DEVICE );
err_dmamem:
	video_unregister_device(camdev->vdev);
	video_device_release( camdev->vdev );
err_dev:
	clk_disable( camdev->clk );
err_clk_ena:
	clk_unuse( camdev->clk );
err_clk_use:
	clk_put( camdev->clk );
err_clk_get:
	kfree(camdev);
err_exit:
	return result;
}	

static int
s3c_cam_remove(struct device *dev)
{
	struct s3c_cam* camdev = device_to_cam(dev);

        dev_dbg(dev,"Unregistering device\n");

	if (camdev != NULL && camdev->vdev != NULL) {
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
		/* Remove the cpu frequency scaling notifiers. */
		cpufreq_unregister_notifier( &(camdev->cpu_policy_notifier), CPUFREQ_POLICY_NOTIFIER );
		cpufreq_unregister_notifier( &(camdev->cpu_transition_notifier), CPUFREQ_TRANSITION_NOTIFIER );
		camdev->cpu_policy_notifier.notifier_call=NULL;
		camdev->cpu_transition_notifier.notifier_call=NULL;
#endif
		/* Close the device if it is open. */
		if( camdev->open )
			cam_cclose( camdev );
			
		/* TODO Stop any activity */
		video_unregister_device(camdev->vdev);
		if( camdev->userspace.nr_pages != 0 )
		{
			unmap_pages( camdev ); 
			kfree( camdev->userspace.page_array );
			camdev->userspace.nr_pages=0;
		}
		clk_put( camdev->clk );
		camdev->clk=NULL;
		dma_unmap_single( &camdev->pdev->dev, camdev->dma_ptr, CAMI_BUFFER_SIZE, DMA_FROM_DEVICE );
		video_device_release( camdev->vdev );
		kfree( camdev );
	}

	return 0;
}
	
static void
s3c_cam_shutdown(struct device *dev)
{
	dev_dbg(dev,"Shutting down\n");
	s3c_cam_remove( dev );
	return;
}

#ifdef CONFIG_PM
#include <media/ovcamchip.h>

struct camif_regs
{
	void __iomem	*camreg;
	u32		regval;
};

static struct camif_regs	backup_array[]=
{
	{S3C2413_CISRCFMT, 0},
	{S3C2413_CIWDOFST, 0},
	{S3C2413_CIGCTRL, 0},
	{S3C2413_CIDOWSFT2, 0},
	{S3C2413_CICOYSA1, 0},
	{S3C2413_CICOYSA2, 0},
	{S3C2413_CICOYSA3, 0},
	{S3C2413_CICOYSA4, 0},
	{S3C2413_CICOCBSA1, 0},
	{S3C2413_CICOCBSA2, 0},
	{S3C2413_CICOCBSA3, 0},
	{S3C2413_CICOCBSA4, 0},
	{S3C2413_CICOCRSA1, 0},
	{S3C2413_CICOCRSA2, 0},
	{S3C2413_CICOCRSA3, 0},
	{S3C2413_CICOCRSA4, 0},
	{S3C2413_CICOTRGFMT, 0},
	{S3C2413_CICOCTRL, 0},
	{S3C2413_CICOSCPRERATIO, 0},
	{S3C2413_CICOSCPREDST, 0},
	{S3C2413_CICOSCCTRL, 0},
	{S3C2413_CICOTAREA, 0},
	{S3C2413_CICOSTATUS, 0},
	{S3C2413_CICOCPTSEQ, 0},
	{S3C2413_CICOSCOS, 0},
	{S3C2413_CIIMGCPT, 0},
	{NULL, 0xFFFFFFFF}
};

static int
s3c_cam_suspend(struct device *dev, u32 state, u32 level)
{
	struct s3c_cam		*cam=device_to_cam( dev );
	int			count=0;

        dev_dbg(dev, "state = %u, level = %u\n", state, level);

        if( level == SUSPEND_POWER_DOWN )
	{
		/* Lock the spinlock. */
	        spin_lock(&cam->lock);
		disable_cam_irq( cam );

		/* If the camera isn't opened, we need to wake it up. */
		if( !cam->open )
		{
			/* Enable the clock. */
			if( clk_enable(cam->clk) )
			{
				dev_err(&cam->pdev->dev, "Can't start camera clock to save state. Camera suspend failed!\n");
				enable_cam_irq( cam );
				spin_unlock( &cam->lock );
				return -ENODEV;
			}

			/* Prepare the clock. */
			if( clk_use(cam->clk) )
			{
				dev_err(&cam->pdev->dev, "Can't use camera clock to save state. Camera suspend failed!\n");
				clk_disable( cam->clk );
				enable_cam_irq( cam );
				spin_unlock( &cam->lock );
				return -ENODEV;
			}

			/* Get the I2C bus. */
			if( cam->ovclient == NULL )
				cam->ovclient = i2c_get_client(I2C_DRIVERID_OVCAMCHIP, 0, NULL);

			if( cam->ovclient == NULL )
			{
				dev_err( &cam->pdev->dev, "Can't find I2C driver to save state. Camera suspend failed!\n" );
				clk_unuse( cam->clk );
				clk_disable( cam->clk );
				enable_cam_irq( cam );
				spin_unlock( &cam->lock );
				return -ENODEV;
			}

			/* Mark i2c client as in use */
			i2c_use_client(cam->ovclient);
		}

		/* We have I2C. Wake up the camera chip if it isn't already awake. */
		cam->ovclient->driver->command( cam->ovclient, OVCAMCHIP_CAM_WAKE, NULL );

		/* First save all S3C2413 CAM interface registers. */
		count=0;
		while( backup_array[count].camreg != NULL )
		{
			backup_array[count].regval=readl( backup_array[count].camreg );
			count++;
		}

		/* Save the state and prepare the camera chip for suspend. */
		cam->ovclient->driver->command( cam->ovclient, OVCAMCHIP_CAM_SUSPEND, NULL );

		/* Check if we just started up the I2C for the purpose of suspending. If so */
		/* bring it back in the original state. */
		if( !cam->open )
		{
			/* Release the I2C interface. */
			i2c_release_client(cam->ovclient);
			cam->ovclient=NULL;
		}

		/* Stop the clock. */
		clk_disable( cam->clk );
		
		/* Release the spinlock. */
		enable_cam_irq( cam );
	        spin_unlock(&cam->lock);
	}

	return 0;
}

static int
s3c_cam_resume(struct device *dev, u32 level)
{
	struct s3c_cam		*cam=device_to_cam( dev );
	int			count=0;

        dev_dbg(dev, "level = %u\n", level);
	
	if( level == RESUME_POWER_ON )
	{
		/* Lock the spinlock. */
	        spin_lock(&cam->lock);
		disable_cam_irq( cam );

		/* Restart the clock. */
		clk_enable( cam->clk );
		
		/* Reget the I2C driver. */
		cam->ovclient = i2c_get_client(I2C_DRIVERID_OVCAMCHIP, 0, NULL);
		if (cam->ovclient == NULL)
		{
			dev_err(&cam->pdev->dev, "No sensor detected on i2c bus! Failing to resume!\n");
			enable_cam_irq( cam );
	        	spin_unlock(&cam->lock);
			return -ENODEV;
		}

		/* Reset the camera. */
		s3c_init_camif( );

		/* Restore the state and resume the camera chip. */
		cam->ovclient->driver->command( cam->ovclient, OVCAMCHIP_CAM_RESUME, NULL );

		/* Now restore all S3C2413 CAM interface registers. */
		count=0;
		while( backup_array[count].camreg != NULL )
		{
			writel( backup_array[count].regval, backup_array[count].camreg );
			count++;
		}

		/* If the camera isn't opened, put it back in sleep state. */
		if( !cam->open )
		{
			cam->ovclient->driver->command( cam->ovclient, OVCAMCHIP_CAM_SLEEP, NULL );
			clk_disable( cam->clk );
			i2c_release_client(cam->ovclient);
			cam->ovclient=NULL;
			clk_unuse(cam->clk);
		}

		/* Unlock the spinlock. */
		enable_cam_irq( cam );
        	spin_unlock(&cam->lock);
	}

	return 0;
}
#else
	#define s3c_cam_suspend	NULL
	#define s3c_cam_resume	NULL
#endif /* CONFIG_PM */

static struct device_driver
s3c_cam_driver = {
	.owner		= THIS_MODULE,
	.name           = "s3c2412-camif",
	.bus            = &platform_bus_type,
	.probe          = s3c_cam_probe,
	.remove         = s3c_cam_remove,
	.shutdown       = s3c_cam_shutdown,
	.suspend        = s3c_cam_suspend,
	.resume         = s3c_cam_resume,
};

static int __init
s3c_cam_mod_init(void)
{
	int ret;

	ret = driver_register(&s3c_cam_driver);
	if (ret) {
		printk(KERN_ERR "Unable to register camera driver (%x)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit
s3c_cam_mod_exit(void)
{
	driver_unregister(&s3c_cam_driver);
}

module_init(s3c_cam_mod_init);
module_exit(s3c_cam_mod_exit);

MODULE_AUTHOR("Ithamar Adema <ithamar.adema@tomtom.com> ((c) 2006 TomTom BV)");
MODULE_DESCRIPTION("TomTom GO Camera driver");
MODULE_LICENSE("GPL");
