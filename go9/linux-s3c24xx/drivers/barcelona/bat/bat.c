/* drivers/barcelona/bat/bat.c
 *
 * Implementation of the battery driver.
 *
 * Copyright (C) 2004,2005 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Includes */ 
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/hardware/clock.h>
#include <barcelona/Barc_Battery.h>
#include <barcelona/Barc_adc.h>
#include <barcelona/debug.h>
#include <barcelona/gopins.h>

/* defines */
#define PFX "bat: "
#define PK_DBG PK_DBG_FUNC
#define DSP_SKIP_RATE		20
//#define DSP_VOLTAGE_DEBUG	1

#ifdef DSP_VOLTAGE_DEBUG
static int dsp_skip = 0;
#endif
static int bat_disabled;

#ifdef BASIC_BATTERY_ADC_CALIBRATION
int use_basic_battery_adc_calibration = 0;
unsigned int adc_cal_diff;
unsigned int adc_cal_offset;
#endif
static int weighted_sample;
/* these parameters are shared with tomtomgo-wake.c */
unsigned int ain4_refraw_calc;
unsigned int adc_factor = 4000;
unsigned int adc_offset = 20;

extern atomic_t low_dc_vcc_event_count;

static irqreturn_t low_dc_vcc_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	atomic_add(1, &low_dc_vcc_event_count);
	return IRQ_HANDLED;
}

static void bat_hw_init(void) 
{
	IO_SetInput(CHARGEFAULT);
	IO_SetInput(CHARGING);
	IO_SetInput(ACPWR);
	IO_SetInput(LOW_DC_VCC);	/*moved from gpio.c*/
	IO_Deactivate(CHARGE_OUT);
//	IO_Deactivate(USB_PWR_BYPASS);

	if (IO_HaveADCAIN4Ref())
		ain4_refraw_calc = IO_GetAIN4RefRawValue();

//	adc_offset = 20;
//	adc_factor = 4000;
//	printk("%s: called\n\n", __func__);
}

static void bat_adc_poll_ain4(short adcBuffer[ADC_CHANNELS], void* arg )
{
	unsigned int sample;

	// 10 + 12 = 22 bits, within range
	sample = (adcBuffer[ADC_BAT] * ain4_refraw_calc) / (adcBuffer[ADC_REF]);

	weighted_sample = (3*weighted_sample + sample)/4;
}

static void bat_adc_poll(short adcBuffer[ADC_CHANNELS], void* arg )
{
	weighted_sample = (3*weighted_sample + adcBuffer[ADC_BAT])/4;
}

static int bat_get_voltage(void)
{
	unsigned int result;

	if ( use_basic_battery_adc_calibration ) {
//		result = weighted_sample - adc_cal_offset;
//		result =  result * BATT_ADC_CAL_DIFF_VOLTAGE / adc_cal_diff + BATT_ADC_CAL_LOW_VOLTAGE;
		result = CALIBRATE_SAMPLE(weighted_sample, adc_cal_offset, BATT_ADC_CAL_DIFF_VOLTAGE, adc_cal_diff, BATT_ADC_CAL_LOW_VOLTAGE);
		return result;
	} /* otherwise use old method */

	// 12 + 12 = 24 bits, in range
	result = (weighted_sample * IO_GetADCREFVoltage()) / IO_GetADCRange(); // fast divide by 2^ number

	// 12 + 14 + 5 = 31 bits, in range
	result = (result * adc_factor * IO_GetBattVoltNumerator()) / (4000 * IO_GetBattVoltDenomenator());

	result += adc_offset;

	return result;
}

static int bat_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	static BATTERY_STATUS bs;
	int ret;

	if (bat_disabled)
		return -ENODEV;

	switch (cmd) {
	case IO_ENABLE_CHARGING:
		IO_SetInput(BATT_TEMP_OVER);
		ret = 0;
		break;
	case IO_DISABLE_CHARGING:
		IO_Activate(BATT_TEMP_OVER);
		ret = 0;
		break;
	case IOR_BATTERY_STATUS:
		if (!access_ok(VERIFY_WRITE, (void __user *) arg, sizeof(BATTERY_STATUS))) {
			ret = -EFAULT;
		} else {
			bs.u16BatteryVoltage   = bat_get_voltage();

#ifdef DSP_VOLTAGE_DEBUG
			if ( ++dsp_skip == DSP_SKIP_RATE ) {
				printk("%s: adc_off=%0d adc_factor=%d adc_cal_off=%0d adc_cal_diff=%d\n", __func__, adc_offset, adc_factor,
							 adc_cal_offset, adc_cal_diff);
				dsp_skip = 0;
				printk("%s: batt voltage=%d mV(%d) %s\n", __func__, bs.u16BatteryVoltage, weighted_sample,
							 use_basic_battery_adc_calibration ? "bcal" : " ");
			}	
#endif
			if (IO_GetInput(ACPWR)) {
				if (IO_GetInput(CHARGING)) {
					bs.u8ChargeStatus = CHARGE_STATE_CHARGING;
					bs.u16ChargeCurrent = 200; /* Charge current should be ignored, but keep in for compatibility */
				} else {
					bs.u8ChargeStatus = CHARGE_STATE_COMPLETED;
					bs.u16ChargeCurrent = 0;
				}
			} else {
				bs.u8ChargeStatus = CHARGE_STATE_NO_POWER;
				bs.u16ChargeCurrent = 0;
			}

			ret = __copy_to_user((void __user *) arg, &bs, sizeof(BATTERY_STATUS)) ? -EFAULT : 0;
		}
		break;
	case IOW_SET_CALIBRATION:
		if (!access_ok(VERIFY_READ, (void __user *) arg, sizeof(BATTERY_CALIBRATION))) {
			ret = -EFAULT;
		} else {
			/* input NOR flash calibration values */
			if (IO_HaveBatteryCalibration()) {
				BATTERY_CALIBRATION batcal;

				memcpy(&batcal, (void __user *)arg, sizeof(BATTERY_CALIBRATION));

				printk("%s: IOW_CALIBRATION called\n\n", __func__);
#ifdef BASIC_BATTERY_ADC_CALIBRATION
				/* do not apply this for Cagliari / Treviso yet nor not having cal. data */
				if ( !IO_HaveADCAIN4Ref() && 
					 (batcal.u16BatteryRaw4000mVADCREF3300mV != 0xffff) ) {
					use_basic_battery_adc_calibration = 1;
					adc_cal_offset = batcal.u16BatteryRaw3500mVADCREF3300mV; /* adc count @3500 mV */
					adc_cal_diff = batcal.u16BatteryRaw4000mVADCREF3300mV - batcal.u16BatteryRaw3500mVADCREF3300mV;
#ifdef DSP_VOLTAGE_DEBUG
					printk("%s: hc=0x%x lc=0x%x\n", __func__,
						 batcal.u16BatteryRaw4000mVADCREF3300mV, batcal.u16BatteryRaw3500mVADCREF3300mV);
#endif
					return 0;
				} /* otherwise use old calibration method */
#endif
				/* set adcoffset and factor */
				adc_offset = 10;	// 10 equaled 0x0319 on reference unit
				/* add difference of raw counts in 3500 mV times n to adc_offset */
				adc_offset += ((batcal.u16BatteryRaw3500mVADCREF3300mV - 0x0319) * 
				               (IO_GetCalibrationREFVoltage() - IO_GetADCREFVoltage()) ) / 
				                IO_GetADCRange();
				adc_factor = 4001;	// 4001 equaled (0x036f - 0x319) on reference unit
				adc_factor -= (((batcal.u16BatteryRaw4000mVADCREF3300mV - 
						 batcal.u16BatteryRaw3500mVADCREF3300mV) - 
						(0x036f - 0x0319)) *
					       (IO_GetCalibrationREFVoltage() - IO_GetADCREFVoltage()) ) / 
					       IO_GetADCRange();
			}

			// optimise out one integer divide
			if (IO_HaveADCAIN4Ref())
				ain4_refraw_calc = IO_GetAIN4RefRawValue();

			ret = 0;
		}
		break;
	default:
		PK_WARN("Invalid ioctl command %u\n", cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int bat_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int bat_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* Kernel interface */
static struct file_operations bat_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= bat_ioctl,
	.open		= bat_open,
	.release	= bat_release,
};

static int bat_probe(struct device *dev)
{
	int ret;
	struct platform_device	*pdev;
	struct resource		    *pres;

	bat_hw_init();

	if(IO_HaveGpioLowBattDetect())
	{
		PK_DBG("Registering LOW_DC_VCC ISR\n");
		/* Get the resources assigned */
		pdev = to_platform_device(dev);
		pres = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
		/* Register the interrupt. */
		if ((pres == NULL) || request_irq(pres->start,
				low_dc_vcc_irq_handler, SA_INTERRUPT | SA_SAMPLE_RANDOM,
				"LOW_DC_VCC_irq", dev))
		{
			PK_ERR("Can't register low dc vcc interrupt !\n");
			return -ENODEV;
		}
		IO_SetInterruptOnDeactivation(LOW_DC_VCC);
	}

	PK_DBG("Registering ADC pollfunc\n");
	if (IO_HaveADCAIN4Ref())
		ret = adc_register_poll(bat_adc_poll_ain4, NULL, ADC_RATE_BAT );
	else
		ret = adc_register_poll(bat_adc_poll, NULL, ADC_RATE_BAT );

	if (ret < 0) {
		PK_ERR("Unable to register ADC pollfunc (%d)\n", ret);
		return ret;
	}

	PK_DBG("Registering chardev\n");
	ret = register_chrdev(BATTERY_MAJOR, BATTERY_DEVNAME, &bat_fops);
	if (ret != 0) {
		PK_ERR("Unable to register chardev on major=%d (%d)\n", BATTERY_MAJOR, ret);
		return ret;
	}

	PK_DBG("Done\n");
	return 0;
}

static int bat_remove(struct device *dev)
{
	PK_DBG("Unregistering chardev\n");
	unregister_chrdev(BATTERY_MAJOR, BATTERY_DEVNAME);

	PK_DBG("Done\n");
	return 0;
}

static void bat_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");
	/* Nothing yet */
}

#ifdef CONFIG_PM

static int bat_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);
	if (level == SUSPEND_POWER_DOWN) {
		bat_disabled = 1;
		if (IO_HaveUsbBusPowered()){
//			IO_Deactivate(USB_PWR_BYPASS);
		}
	}

	return 0;
}

static int bat_resume(struct device *dev, u32 level)
{
	PK_DBG("dev = %p, level = %u\n", dev, level);
	if (level == RESUME_POWER_ON) {
		bat_hw_init();
		bat_disabled = 0;
		if (IO_HaveUsbBusPowered()){
			/*make sure we don't blow up our friend PC's */
//			IO_Deactivate(USB_PWR_BYPASS);
		}
	}
	return 0;
}

#else /* CONFIG_PM */
#define bat_suspend NULL
#define bat_resume  NULL
#endif /* CONFIG_PM */

static struct device_driver bat_driver = {
	.name		= "tomtomgo-bat",
	.bus		= &platform_bus_type,
	.probe		= bat_probe,
	.remove		= bat_remove,
	.shutdown	= bat_shutdown,
	.suspend	= bat_suspend,
	.resume		= bat_resume,
};

static int __init bat_mod_init(void)
{
	int ret;

	printk(KERN_INFO "TomTom GO Battery Driver, (C) 2004,2005,2006,2007 TomTom BV\n");
	PK_DBG("Registering driver\n");
	ret = driver_register(&bat_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit bat_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&bat_driver);
	PK_DBG("Done\n");
}

module_init(bat_mod_init);
module_exit(bat_mod_exit);

MODULE_AUTHOR("Dimitry Andric <dimitry.andric@tomtom.com>");
MODULE_DESCRIPTION("TomTom GO Battery Driver");
MODULE_LICENSE("GPL");

/* EOF */
