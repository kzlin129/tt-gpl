/*
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Benoit Leffray <benoit.leffray@tomtom.com>
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

#include <linux/platform_device.h>
#include <plat/regs-gpio.h>

#include <linux/i2c.h>
#include <mach/hardware.h>

#include <plat/devs.h>
#include <plat/mendoza.h>
#include <plat/mendoza_battery.h>

#include <plat/battery.h>


static int  ttbattery_plat_init	 (struct platform_device *pdevice);
static void ttbattery_plat_remove(struct platform_device *pdevice);

/* Those are defaults for Seoul/Cordoba.
   Use mendoza_battery_adc_setup to adjust before registering the device.
 */

static struct battery_poll battery_voltage =
{
	.channel 	= AIN2,
	.samples	= 1,
};

static struct battery_poll battery_current =
{
	.channel	= AIN0,
	.samples 	= 1,
};

static batt_discharge_lut_t dis_200[]=
{
        {4137100,	100,	21660},
        {4033674,	90,	19710},
        {3940577,	80,	17820},
        {3870431,	70,	15930},
        {3805551,	60,	13770},
        {3761126,	50,	11610},
        {3729942,	40,	9180},
        {3710580,	30,	7020},
        {3689341,	20,	4860},
        {3629511,	10,	2430},
        {3619838,	7,	1800},
        {3608970,	5,	1350},
        {3587496,	3,	990},
        {3561458,	2,	720},
        {3515540,	1,	540},
        {3000000,	0,	0},
};

static batt_discharge_lut_t dis_300[]=
{
        {4104300,	100,	14440},
        {3995995,	90,	13140},
        {3903343,	80,	11880},
        {3835407,	70,	10620},
        {3775704,	60,	9180},
        {3732107,	50,	7740},
        {3700714,	40,	6120},
        {3679665,	30,	4680},
        {3656642,	20,	3240},
        {3601191,	10,	1620},
        {3590058,	7,	1200},
        {3577660,	5,	900},
        {3555081,	3,	660},
        {3533506,	2,	480},
        {3496997,	1,	360},
        {3000000,	0,	0},
};

static batt_discharge_lut_t dis_400[]=
{
        {4024100,	100,	10830},
        {3936560,	90,	9855},
        {3848146,	80,	8910},
        {3788788,	70,	7965},
        {3739411,	60,	6885},
        {3700581,	50,	5805},
        {3670598,	40,	4590},
        {3654582,	30,	3510},
        {3632715,	20,	2430},
        {3586812,	10,	1215},
        {3573167,	7,	900},
        {3560459,	5,	675},
        {3541675,	3,	495},
        {3532400,	2,	360},
        {3485700,	1,	270},
        {3000000,	0,	0},
};

#define DISCHARGE_CURVE(x) x, ARRAY_SIZE(x)
static batt_discharge_curve_t discharge_curves[] = 
{
	{400, DISCHARGE_CURVE(dis_400) },
	{300, DISCHARGE_CURVE(dis_300) },
	{200, DISCHARGE_CURVE(dis_200) },
};
#undef DISCHARGE_CURVE


#ifdef CONFIG_MACH_TORINOS

static struct ttbatt_info b1400mah_batt =
{
        .adc=
        {
		.accuracy					= 12,
		.reference_voltage			= 3300,		/* mV    */
		.battery_voltage_resistor_1	= 1000,		/* KOhms */
		.battery_voltage_resistor_2	= 3000, 	/* KOhms */
		.charge_current_resistor	= 10000,	/*  Ohms */
        },

	.battery=
	{
		.discharge_curves		= discharge_curves,
		.curves				= ARRAY_SIZE(discharge_curves),
		.Vmax = 4000000, /* mV */
		.Vmin = 3500000, /* mV */
	},
};

static struct ttbatt_info *batt = &b1400mah_batt;

#elif CONFIG_MACH_VENICE

static struct ttbatt_info b1100mah_batt =
{
		.adc=
		{
		.accuracy					= 12,
		.reference_voltage			= 3300,		/* mV    */
		.battery_voltage_resistor_1	= 1200,		/* KOhms */
		.battery_voltage_resistor_2	= 3000, 	/* KOhms */
		.charge_current_resistor	= 1820,	/*  Ohms */
        },

	.battery=
	{
		.discharge_curves		= discharge_curves,
		.curves				= ARRAY_SIZE(discharge_curves),
		.Vmax = 4000000, /* mV */
		.Vmin = 3500000, /* mV */
	},
};

static struct ttbatt_info *batt = &b1100mah_batt;

#else 

static struct ttbatt_info b1100mah_batt =
{
        .adc=
        {
		.accuracy					= 12,
		.reference_voltage		= 3300,		/* mV    */
		.battery_voltage_resistor_1	= 1200,		/* KOhms */
		.battery_voltage_resistor_2	= 3000, 	/* KOhms */
		.charge_current_resistor	= 10000,	/*  Ohms */
        },

	.battery=
	{
		.discharge_curves		= discharge_curves,
		.curves				= ARRAY_SIZE(discharge_curves),
		.Vmax = 4000000, /* mV */
		.Vmin = 3500000, /* mV */
	},
};

static struct ttbatt_info *batt = &b1100mah_batt;

#endif


static inline u32 get_battery_value(struct battery_poll *poll)
{
	down(&poll->mutex);
	/* TODO If value is fairly recent, use it */
	//if (time_after(poll->last_read + HZ / 10, jiffies)) {
	if (0) {
		up(&poll->mutex);
		return poll->current_value;
	}
	init_completion(&poll->adc_done);
	
	s3c_adc_start(poll->adc_client, poll->channel, poll->samples);
	wait_for_completion(&poll->adc_done);
	up(&poll->mutex);
	
	return poll->current_value;
}

static u32 cordoba_get_charge_current(void)
{
	return get_battery_value(&battery_current);
}

static u32 cordoba_get_battery_voltage(void)
{
	return get_battery_value(&battery_voltage);
}

/* s3c_adc generic callback */
static void ttbattery_adc_callback(unsigned int sample, unsigned int not_used,
		void *data)
{
	struct battery_poll *poll = (struct battery_poll*)data;
        unsigned long result;

        result = (((u32)sample * batt->adc.reference_voltage) 
		>> batt->adc.accuracy) * 1000;
	poll->current_value = result;
	poll->last_read = jiffies;
	complete(&poll->adc_done);
}

static int mendoza_battery_suspend(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "suspending\n");
	down(&battery_current.mutex);
	down(&battery_voltage.mutex);
	return 0;
}

static int mendoza_battery_resume(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "resuming\n");
	up(&battery_voltage.mutex);
	up(&battery_current.mutex);
	return 0;
}

static struct ttbattery_platform_info mendoza_platform_info =
{
        .platform_init   = ttbattery_plat_init,
        .platform_remove = ttbattery_plat_remove,
	.platform_suspend = mendoza_battery_suspend,
	.platform_resume = mendoza_battery_resume,

        .get_charge_current_adc  = cordoba_get_charge_current,
        .get_battery_voltage_adc = cordoba_get_battery_voltage,
#ifdef CONFIG_MACH_TORINOS
	.battery_info = &b1400mah_batt,
#else
	.battery_info = &b1100mah_batt,
#endif
};

struct platform_device mendoza_device_battery = {
	.name   = "battery",
	.id     = -1,
	.dev    = {
		.parent		= &s3c_device_adc.dev,
		.platform_data	= &mendoza_platform_info
	},
};

static int ttbattery_plat_init(struct platform_device *pdevice)
{
	struct s3c_adc_client *adc_client;

	init_MUTEX(&battery_voltage.mutex);

	adc_client = s3c_adc_register(pdevice, NULL,
			ttbattery_adc_callback, &battery_voltage, 0);
	if (adc_client == NULL) {
		printk(KERN_ERR "ttbattery.c: Unable to get ADC client\n");
		goto err_adc_voltage;	
	}
	
	battery_voltage.adc_client = adc_client;

	init_MUTEX(&battery_current.mutex);
		
        adc_client = s3c_adc_register(pdevice, NULL,
			ttbattery_adc_callback, &battery_current, 0);
        if (adc_client == NULL) {
                printk(KERN_ERR "ttbattery.c: Unable to get ADC client\n");
                goto err_adc_current;
        }

        battery_current.adc_client = adc_client;
		
	return 0;

err_adc_current:
	s3c_adc_release(battery_voltage.adc_client);
err_adc_voltage:
        return -ENOENT;
}

static void ttbattery_plat_remove (struct platform_device *pdevice)
{
	/* This ensure that if a read is currently in process
	 * we wait for it to finish. */
	down(&battery_current.mutex);
	down(&battery_voltage.mutex);
}

void mendoza_battery_adc_setup(unsigned int voltage_channel, unsigned int current_channel)
{
	battery_voltage.channel = voltage_channel;
	battery_current.channel = current_channel;
}

