/*
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/platform_device.h>
#include <sound/driver.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

#include <asm/arch/hw_cfg.h>

#include "brcm_alsa.h"

//  Module declarations.
//
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom AP sound interface");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,Broadcom AP soundcard}}");

//global
brcm_alsa_chip_t *g_brcm_alsa_chip=NULL;
int debug = 0; 

//
static char *id = NULL;
static int enable = 1;
static int index = 0;

//
module_param(index, int, 0444);
MODULE_PARM_DESC(index, "Index value for Broadcom soundcard.");
module_param(id, charp, 0444);
MODULE_PARM_DESC(id, "ID string for Broadcom soundcard.");
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Enable the Broadcom soundcard.");
module_param(debug, int, 0444);
MODULE_PARM_DESC(debug, "debug value for Broadcom soundcard.");



static int __devinit brcm_alsa_omx_probe(struct platform_device *pdev)
{
	struct snd_card *card;
	int err;
	
	DEBUG("\n %lx:probe \n",jiffies);
	
	err = -ENODEV;
	if (!enable)
      return err;
    
    err = -ENOMEM;  
	card = snd_card_new(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1, 
		THIS_MODULE, sizeof(brcm_alsa_chip_t));
	if (!card)
      goto err;
            
	g_brcm_alsa_chip = (brcm_alsa_chip_t*)card->private_data;
	g_brcm_alsa_chip->card = card;
	
	card->dev = &pdev->dev;
	strncpy(card->driver, pdev->dev.driver->name, sizeof(card->driver));
	
	//PCM interface	
	err = brcm_alsa_omx_pcm_new(card);
	if (err)
    	goto err;

	//CTRL interface	
	err = brcm_alsa_omx_ctl_new(card);
	if (err)
    	goto err;

	//TODO: other interface
	
	
	strcpy(card->driver, "Broadcom");
	strcpy(card->shortname, "Broadcom ALSA");
	sprintf(card->longname, "Broadcom ALSA PCM %i", 0);
	
	
	err = snd_card_register(card);
	if (err==0)
	{
      platform_set_drvdata(pdev, card);
      return 0;
	}

err:
	DEBUG("\n probe failed =%d\n",err);
	if (card)
		snd_card_free(card);
	
	g_brcm_alsa_chip=NULL;
	return err;
}

static int brcm_alsa_omx_remove(struct platform_device *pdev)
{
	return 0;
}

static int brcm_alsa_omx_suspend(
		struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int brcm_alsa_omx_resume(struct platform_device *pdev)
{
	return 0;
}


typedef struct brcm_alsa_dev 
{
	int init;	
} brcm_alsa_dev_t;

static brcm_alsa_dev_t brcm_alsa_device_info = 
{
	.init = 0,
};                                

static void brcm_alsa_device_release(struct device *pdev)
{
	DEBUG("\n TO DO:what am i supposed to do\n");	
}

static struct platform_device brcm_alsa_device = 
{
	.name		= "brcm_alsa_device",
	.dev		= 
	{
		.platform_data	= &brcm_alsa_device_info,
		.release = brcm_alsa_device_release,
	},
	.id		= -1,
};

static struct platform_driver brcm_alsa_omx_driver = 
{
	.probe		= brcm_alsa_omx_probe,
	.remove 	= brcm_alsa_omx_remove,
	.suspend	= brcm_alsa_omx_suspend,
	.resume		= brcm_alsa_omx_resume,
	.driver		= 
		{
		.name	= "brcm_alsa_device",
		.owner	= THIS_MODULE,
		},
};

static int __devinit brcm_alsa_device_init(void)
{
	int err;
	
	DEBUG("\n %lx:debg=%d id=%s\n",jiffies,debug,id);
	
	//do VC03 OMX related init first
	err = brcm_alsa_omx_vc03_init();
	DEBUG("\n %lx:VC03 OMX init done %d\n",jiffies,err);
	if (err)
		return err;
	
	err =  platform_device_register(&brcm_alsa_device);
	DEBUG("\n %lx:device register done %d\n",jiffies,err);
	if (err)
		return err;
		
	err = platform_driver_register(&brcm_alsa_omx_driver);
	DEBUG("\n %lx:driver register done %d\n",jiffies,err);

	return err;
}

static void __devexit brcm_alsa_device_exit(void)
{
	int err;
	
	DEBUG("\n %lx:brcm_alsa_device_exit\n",jiffies);
	//do VC03 OMX exit
	err = brcm_alsa_omx_vc03_exit();
	DEBUG("\n %lx:VC03 OMX exit done %d\n",jiffies,err);
	
	snd_card_free(g_brcm_alsa_chip->card);
	
	platform_driver_unregister(&brcm_alsa_omx_driver);
	platform_device_unregister(&brcm_alsa_device);
	
	DEBUG("\n %lx:exit done \n",jiffies);
}

module_init(brcm_alsa_device_init);
module_exit(brcm_alsa_device_exit);
