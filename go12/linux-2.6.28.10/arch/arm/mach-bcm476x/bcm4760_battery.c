#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/bcm_adc.h>
#include <asm/arch/bcm_gen_battery.h>
#include <linux/delay.h>
#include <plat/fdt.h>

#define PFX "bcm4760_battery"

struct bcm_ll_adc_req {
	bcm_adc_request_t	adc;	/* ADC request structure for request function */
	unsigned short		sample;	/* sample response from ADC integration */
	struct completion	cmp;	/* completion structure for this request */
};

static DEFINE_MUTEX(bcm_bat_mutex);
static DEFINE_MUTEX(bcm_vmbat_mutex);

#if defined(CONFIG_PMU_DEVICE_BCM59002) || \
    defined(CONFIG_BATTERY_BCM59040) || \
    defined(CONFIG_PMU_DEVICE_BCM59040)
/****************************************************************************
 *
 *   This function initializes the bcm4760_std_battery structure if
 *   we are not using the standard test battery. It may modify the battery_info
 *   member in the bcm4760_battery_device structure to point to a completely
 *   different bcm_batt_info structure if it isn't one of the standard
 *   batteries.
 *
 *****************************************************************************/
static int bcm_batt_platform_init( struct platform_device *pdevice )
{
	return 0;
}

/****************************************************************************
 *
 *   This function does any cleanup in preparation for removal of the
 *   battery device/driver. This is called infrequently at best but the
 *   function is here for completeness.
 *
 *****************************************************************************/

static void bcm_batt_platform_remove( struct platform_device *pdevice )
{
	return;
}

/****************************************************************************
 *
 *   This function is the callback for the ADC function. It is called when
 *   the PMU ADC finishes integration after an ADC request.
 *
 *****************************************************************************/

void bcm_adc_vmbat_handler(struct bcm_adc_request_s *req,
                           unsigned short sample,
                           int error)
{
	struct bcm_ll_adc_req *adc_req;

	mutex_lock(&bcm_vmbat_mutex);
	adc_req = (struct bcm_ll_adc_req *)req->param;
	adc_req->sample = sample;		
	complete(&adc_req->cmp);
	mutex_unlock(&bcm_vmbat_mutex);
}


/****************************************************************************
 *
 *   This function obtains the current battery voltage (VMBAT) using the
 *   PMU ADC. This function will pend the calling task until the ADC completes
 *   integration and returns the voltage.
 *
 *****************************************************************************/
u32 bcm_adc_read(int device, int channel)
{
	struct bcm_ll_adc_req   adc_req;
	u32                     data;

	memset( &adc_req, 0, sizeof( adc_req ) );

	/* Fill in request structure for ADC */
	mutex_lock(&bcm_bat_mutex);
	adc_req.adc.device = device;
	adc_req.adc.channel = channel;
	adc_req.adc.param = (unsigned long int)&adc_req;
	init_completion(&adc_req.cmp);
	adc_req.adc.callback = &bcm_adc_vmbat_handler;

	if (bcm_adc_request(&adc_req.adc) != 0) {
		printk(KERN_ERR PFX "ADC request failed\n");
		mutex_unlock(&bcm_bat_mutex);
		return 0;
	}

	mutex_unlock(&bcm_bat_mutex);
	if (wait_for_completion_timeout(&adc_req.cmp, HZ/2))
		data = (u32)(adc_req.sample);
	else {
		bcm_adc_cancel_request (&adc_req.adc);
		printk (KERN_INFO PFX "ADC request timed out\n");
		data = 0;
	}

	return data; 
}
#endif

#if defined(CONFIG_BATTERY_BCM59040)
/****************************************************************************
 *
 *   This function reads the fuel gauge sample and returns it.
 *
 *****************************************************************************/

static s16 bcm_batt_get_fuel_gauge_sample( void )
{
    short sample;

    PMU_fuelgauge_enable(current_pmu,1);
    msleep(300);
    PMU_fuelgauge_get_sample(current_pmu, &sample, 1);
    PMU_fuelgauge_enable(current_pmu,0);

    return (s16)sample;
}

/****************************************************************************
 *
 *   This function obtains the charge current being used by the PMU battery
 *   charger currently.
 *
 *****************************************************************************/

static u32 bcm_batt_get_charge_current( void )
{
    return (u32)PMU_charger_get_current(current_pmu);
}
#else
#define bcm_batt_get_charge_current NULL
#define bcm_batt_get_fuel_gauge_sample NULL
#endif

static batt_discharge_lut_t siena_foxlink_maxell_1100_batt_cap[] = 
{
	[0]  = { 4100000, 100, 0 },
	[1]  = { 4050000,  98, 0 },
	[2]  = { 4000000,  94, 0 },
	[3]  = { 3900000,  83, 0 },
	[4]  = { 3850000,  78, 0 },
	[5]  = { 3800000,  71, 0 },
	[6]  = { 3750000,  62, 0 },
	[7]  = { 3699000,  52, 0 },
	[8]  = { 3650000,  38, 0 },
	[9]  = { 3630000,  30, 0 },
	[10] = { 3600000,  20, 0 },
	[11] = { 3552000,  11, 0 },
	[12] = { 3530000,   8, 0 },
	[13] = { 3495000,   4, 0 },
	[14] = { 3457000,   3, 0 },
	[15] = { 3430000,   2, 0 },
	[16] = { 3392000,   1, 0 },
	[17] = { 3330000,   0, 0 }
};

static batt_discharge_lut_t siena_samsung_sdi_1100_batt_cap[] = 
{
	[0]  = { 4100000, 100, 0 },
	[1]  = { 4050000,  98, 0 },
	[2]  = { 4000000,  93, 0 }, 
	[3]  = { 3880000,  80, 0 },
	[4]  = { 3800000,  69, 0 },
	[5]  = { 3750000,  61, 0 },
	[6]  = { 3699000,  50, 0 },
	[7]  = { 3650000,  34, 0 },
	[8]  = { 3630000,  25, 0 },
	[9]  = { 3590000,  15, 0 },
	[10] = { 3552000,   7, 0 },
	[11] = { 3530000,   5, 0 },
	[12] = { 3495000,   3, 0 },
	[13] = { 3457000,   2, 0 },
	[14] = { 3392000,   1, 0 },
	[15] = { 3330000,   0, 0 }
};

static batt_discharge_lut_t siena_formosa_maxell_1100_batt_cap[] = 
{
	[0]  = { 4150000, 100, 0 },
	[1]  = { 4050000,  94, 0 },
	[2]  = { 4000000,  90, 0 },
	[3]  = { 3930000,  83, 0 },
	[4]  = { 3850000,  75, 0 },
	[5]  = { 3800000,  68, 0 },
	[6]  = { 3750000,  60, 0 },
	[7]  = { 3699000,  50, 0 },
	[8]  = { 3650000,  37, 0 },
	[9]  = { 3630000,  30, 0 },
	[10] = { 3600000,  20, 0 },
	[11] = { 3552000,  10, 0 },
	[12] = { 3520000,   7, 0 },
	[13] = { 3495000,   5, 0 },
	[14] = { 3457000,   3, 0 },
	[15] = { 3430000,   2, 0 },
	[16] = { 3392000,   1, 0 },
	[17] = { 3330000,   0, 0 }
};

/* ------------------------------------------------------------------------- */

static batt_discharge_lut_t carlisle_samsung_sdi_1100_batt_cap[] = 
{
	[0]  = { 4100000, 100, 0 },
	[1]  = { 4050000,  98, 0 },
	[2]  = { 4000000,  93, 0 },
	[3]  = { 3880000,  80, 0 },
	[4]  = { 3800000,  69, 0 },
	[5]  = { 3750000,  61, 0 },
	[6]  = { 3700000,  50, 0 },
	[7]  = { 3650000,  34, 0 },
	[8]  = { 3630000,  25, 0 },
	[9]  = { 3600000,  17, 0 },
	[10] = { 3550000,   7, 0 },
	[11] = { 3500000,   3, 0 },
	[12] = { 3450000,   2, 0 },
	[13] = { 3350000,   0, 0 },
};

static batt_discharge_lut_t carlisle_formosa_maxell_1100_batt_cap[] = 
{
	[0]  = { 4150000, 100, 0 },
	[1]  = { 4100000,  98, 0 },
	[2]  = { 4000000,  90, 0 },
	[3]  = { 3900000,  80, 0 },
	[4]  = { 3850000,  75, 0 },
	[5]  = { 3800000,  68, 0 },
	[6]  = { 3750000,  60, 0 },
	[7]  = { 3700000,  50, 0 },
	[8]  = { 3650000,  37, 0 },
	[9]  = { 3600000,  20, 0 },
	[10] = { 3550000,  10, 0 },
	[11] = { 3500000,   5, 0 },
	[12] = { 3450000,   3, 0 },
	[13] = { 3420000,   2, 0 },
	[14] = { 3400000,   1, 0 },
	[15] = { 3340000,   0 ,0 }
};

static batt_discharge_lut_t carlisle_foxlink_maxell_1100_batt_cap[] = 
{
	[0]  = { 4100000, 100, 0 },
	[1]  = { 4050000,  98, 0 },
	[2]  = { 4000000,  94, 0 },
	[3]  = { 3900000,  83, 0 },
	[4]  = { 3850000,  78, 0 },
	[5]  = { 3800000,  71, 0 },
	[6]  = { 3750000,  62, 0 },
	[7]  = { 3700000,  52, 0 },
	[8]  = { 3650000,  38, 0 },
	[9]  = { 3600000,  20, 0 },
	[10] = { 3550000,  10, 0 },
	[11] = { 3500000,   5, 0 },
	[12] = { 3470000,   3, 0 },
	[13] = { 3450000,   2, 0 },
	[14] = { 3400000,   1, 0 },
	[15] = { 3330000,   0 ,0 }
};

static batt_discharge_lut_t siena_carlisle_samsung_sdi_1100_batt_cap[] = 
{
	[0]  = { 4100000, 100, 0 },
	[1]  = { 4050000,  98, 0 },
	[2]  = { 4000000,  93, 0 },
	[3]  = { 3880000,  80, 0 },
	[4]  = { 3800000,  69, 0 },
	[5]  = { 3750000,  61, 0 },
	[6]  = { 3699000,  50, 0 },
	[7]  = { 3650000,  34, 0 },
	[8]  = { 3630000,  25, 0 },
	[9]  = { 3590000,  15, 0 },
	[10] = { 3552000,   7, 0 },
	[11] = { 3530000,   5, 0 },
	[12] = { 3495000,   3, 0 },
	[13] = { 3457000,   2, 0 },
	[14] = { 3392000,   1, 0 },
	[15] = { 3330000,   0, 0 }
};


/* --------------------------------------------------------------------------- */

static batt_discharge_lut_t catania_920_batt_cap[] = 
{
	[0]  = { 4080000, 100, 0 },
	[1]  = { 4040000,  99, 0 },
	[2]  = { 4000000,  96, 0 },
	[3]  = { 3950000,  92, 0 },
	[4]  = { 3910000,  88, 0 },
	[5]  = { 3850000,  82, 0 },
	[6]  = { 3800000,  75, 0 },
	[7]  = { 3750000,  67, 0 },
	[8]  = { 3700000,  57, 0 },
	[9]  = { 3650000,  44, 0 },
	[10] = { 3600000,  25, 0 },
	[11] = { 3570000,  18, 0 },
	[12] = { 3550000,  14, 0 },
	[13] = { 3520000,   9, 0 },
	[14] = { 3500000,   7, 0 },
	[15] = { 3450000,   4, 0 },
	[16] = { 3430000,   3, 0 },
	[17] = { 3400000,   2, 0 },
	[18] = { 3350000,   1 ,0 },
	[19] = { 3300000,   0 ,0 }
};

static batt_discharge_lut_t catania_920_formosa_maxell_batt_cap[] = 
{
	[0]  = { 4080000, 100, 0 },
	[1]  = { 4040000,  99, 0 },
	[2]  = { 4000000,  96, 0 },
	[3]  = { 3950000,  92, 0 },
	[4]  = { 3910000,  88, 0 },
	[5]  = { 3850000,  82, 0 },
	[6]  = { 3800000,  75, 0 },
	[7]  = { 3750000,  67, 0 },
	[8]  = { 3700000,  57, 0 },
	[9]  = { 3650000,  44, 0 },
	[10] = { 3600000,  25, 0 },
	[11] = { 3570000,  18, 0 },
	[12] = { 3550000,  14, 0 },
	[13] = { 3520000,   9, 0 },
	[14] = { 3500000,   7, 0 },
	[15] = { 3450000,   4, 0 },
	[16] = { 3430000,   3, 0 },
	[17] = { 3400000,   2, 0 },
	[18] = { 3350000,   1 ,0 },
	[19] = { 3300000,   0 ,0 }
};

static batt_discharge_lut_t catania_900_batt_cap[] = 
{
	[0]  = { 4090000, 100, 0 },
	[1]  = { 4040000,  99, 0 },
	[2]  = { 4000000,  95, 0 },
	[3]  = { 3900000,  85, 0 },
	[4]  = { 3800000,  73, 0 },
	[5]  = { 3750000,  66, 0 },
	[6]  = { 3700000,  56, 0 },
	[7]  = { 3650000,  43, 0 },
	[8]  = { 3630000,  35, 0 },
	[9]  = { 3600000,  23, 0 },
	[10] = { 3580000,  18, 0 },
	[11] = { 3550000,  13, 0 },
	[12] = { 3500000,   8, 0 },
	[13] = { 3450000,   5, 0 },
	[14] = { 3400000,   3, 0 },
	[15] = { 3300000,   0, 0 }
};

static batt_discharge_lut_t catania_770_batt_cap[] = 
{
	[0]  = { 4090000, 100, 0 },
	[1]  = { 4040000,  98, 0 },
	[2]  = { 4000000,  95, 0 },
	[3]  = { 3900000,  85, 0 },
	[4]  = { 3800000,  72, 0 },
	[5]  = { 3750000,  63, 0 },
	[6]  = { 3700000,  54, 0 },
	[7]  = { 3650000,  41, 0 },
	[8]  = { 3630000,  35, 0 },
	[9]  = { 3600000,  24, 0 },
	[10] = { 3580000,  18, 0 },
	[11] = { 3550000,  12, 0 },
	[12] = { 3500000,   5, 0 },
	[13] = { 3450000,   3, 0 },
	[14] = { 3400000,   2, 0 },
	[15] = { 3300000,   0, 0 }
};

static batt_discharge_lut_t catania_720_batt_cap[] = 
{
	[0]  = { 4080000, 100, 0 },
	[1]  = { 4040000,  99, 0 },
	[2]  = { 4010000,  97, 0 },
	[3]  = { 3900000,  85, 0 },
	[4]  = { 3800000,  72, 0 },
	[5]  = { 3750000,  64, 0 },
	[6]  = { 3700000,  54, 0 },
	[7]  = { 3650000,  40, 0 },
	[8]  = { 3630000,  32, 0 },
	[9]  = { 3600000,  22, 0 },
	[10] = { 3570000,  15, 0 },
	[11] = { 3550000,  11, 0 },
	[12] = { 3520000,   7, 0 },
	[13] = { 3500000,   5, 0 },
	[14] = { 3490000,   4, 0 },
	[15] = { 3460000,   3, 0 },
	[16] = { 3430000,   2, 0 },
	[17] = { 3400000,   1 ,0 },
	[18] = { 3300000,   0 ,0 }
};

static batt_discharge_lut_t catania_formosa_maxell_720_batt_cap[] = 
{
	[0]  = { 3980000, 100, 0 },
	[1]  = { 3900000,  98, 0 },
	[2]  = { 3850000,  94, 0 },
	[3]  = { 3800000,  89, 0 },
	[4]  = { 3750000,  83, 0 },
	[5]  = { 3700000,  75, 0 },
	[6]  = { 3650000,  64, 0 },
	[7]  = { 3630000,  59, 0 },
	[8]  = { 3600000,  52, 0 },
	[9]  = { 3570000,  40, 0 },
	[10] = { 3550000,  32, 0 },
	[11] = { 3530000,  24, 0 },
	[12] = { 3500000,  17, 0 },
	[13] = { 3450000,   9, 0 },
	[14] = { 3400000,   5, 0 },
	[15] = { 3310000,   0, 0 },
};

static batt_discharge_lut_t catania_sdi_icp553436_760_batt_cap[] = 
{
	[0]  = { 4050000,  99, 0 },
	[1]  = { 4010000,  97, 0 },
	[2]  = { 3950000,  91, 0 },
	[3]  = { 3900000,  85, 0 },
	[4]  = { 3850000,  79, 0 },
	[5]  = { 3800000,  72, 0 },
	[6]  = { 3750000,  64, 0 },
	[7]  = { 3700000,  54, 0 },
	[8]  = { 3650000,  39, 0 },
	[9]  = { 3600000,  22, 0 },
	[10] = { 3550000,  11, 0 },
	[11] = { 3500000,   5, 0 },
	[12] = { 3450000,   3, 0 },
	[13] = { 3400000,   1, 0 },
	[14] = { 3310000,   0, 0 },
};

static batt_discharge_lut_t messina_920_batt_cap[] = 
{
	[0]  = { 4110000, 100, 0 },
	[1]  = { 4000000,  90, 0 },
	[2]  = { 3900000,  80, 0 },
	[3]  = { 3800000,  65, 0 },
	[4]  = { 3750000,  56, 0 },
	[5]  = { 3700000,  43, 0 },
	[6]  = { 3650000,  23, 0 },
	[7]  = { 3600000,  13, 0 },
	[8]  = { 3550000,   5, 0 },
	[9]  = { 3500000,   3, 0 },
	[10] = { 3450000,   2, 0 },
	[11] = { 3400000,   1, 0 },
	[12] = { 3300000,   0, 0 }
};

static batt_discharge_lut_t messina_1100_batt_cap[] = 
{
	[0]  = { 4110000, 100, 0 },
	[1]  = { 4050000,  96, 0 },
	[2]  = { 4000000,  91, 0 },
	[3]  = { 3900000,  81, 0 },
	[4]  = { 3800000,  67, 0 },
	[5]  = { 3750000,  59, 0 },
	[6]  = { 3700000,  48, 0 },
	[7]  = { 3650000,  31, 0 },
	[8]  = { 3600000,  16, 0 },
	[9]  = { 3550000,   7, 0 },
	[10] = { 3500000,   3, 0 },
	[11] = { 3450000,   2, 0 },
	[12] = { 3400000,   1, 0 },
	[13] = { 3300000,   0, 0 }
};

static batt_discharge_lut_t lausanne_720_batt_cap[] = 
{
	[0]  = { 4080000, 100, 0 },
	[1]  = { 4040000,  99, 0 },
	[2]  = { 4010000,  97, 0 },
	[3]  = { 3900000,  85, 0 },
	[4]  = { 3800000,  72, 0 },
	[5]  = { 3750000,  64, 0 },
	[6]  = { 3700000,  54, 0 },
	[7]  = { 3650000,  40, 0 },
	[8]  = { 3630000,  32, 0 },
	[9]  = { 3600000,  22, 0 },
	[10] = { 3570000,  15, 0 },
	[11] = { 3550000,  11, 0 },
	[12] = { 3520000,   7, 0 },
	[13] = { 3500000,   5, 0 },
	[14] = { 3490000,   4, 0 },
	[15] = { 3460000,   3, 0 },
	[16] = { 3430000,   2, 0 },
	[17] = { 3400000,   1 ,0 },
	[18] = { 3300000,   0 ,0 }
};

static batt_discharge_lut_t lausanne_920_batt_cap[] = 
{
	[0]  = { 4080000, 100, 0 },
	[1]  = { 4040000,  99, 0 },
	[2]  = { 4000000,  96, 0 },
	[3]  = { 3950000,  92, 0 },
	[4]  = { 3910000,  88, 0 },
	[5]  = { 3850000,  82, 0 },
	[6]  = { 3800000,  75, 0 },
	[7]  = { 3750000,  67, 0 },
	[8]  = { 3700000,  57, 0 },
	[9]  = { 3650000,  44, 0 },
	[10] = { 3600000,  25, 0 },
	[11] = { 3570000,  18, 0 },
	[12] = { 3550000,  14, 0 },
	[13] = { 3520000,   9, 0 },
	[14] = { 3500000,   7, 0 },
	[15] = { 3450000,   4, 0 },
	[16] = { 3430000,   3, 0 },
	[17] = { 3400000,   2, 0 },
	[18] = { 3350000,   1 ,0 },
	[19] = { 3300000,   0 ,0 }
};

static batt_discharge_lut_t lausanne_920_formosa_maxell_batt_cap[] = 
{
	[0]  = { 4080000, 100, 0 },
	[1]  = { 4040000,  99, 0 },
	[2]  = { 4000000,  96, 0 },
	[3]  = { 3950000,  92, 0 },
	[4]  = { 3910000,  88, 0 },
	[5]  = { 3850000,  82, 0 },
	[6]  = { 3800000,  75, 0 },
	[7]  = { 3750000,  67, 0 },
	[8]  = { 3700000,  57, 0 },
	[9]  = { 3650000,  44, 0 },
	[10] = { 3600000,  25, 0 },
	[11] = { 3570000,  18, 0 },
	[12] = { 3550000,  14, 0 },
	[13] = { 3520000,   9, 0 },
	[14] = { 3500000,   7, 0 },
	[15] = { 3450000,   4, 0 },
	[16] = { 3430000,   3, 0 },
	[17] = { 3400000,   2, 0 },
	[18] = { 3350000,   1 ,0 },
	[19] = { 3300000,   0 ,0 }
};

static batt_discharge_lut_t lausanne_formosa_maxell_720_batt_cap[] = 
{
	[0]  = { 3980000, 100, 0 },
	[1]  = { 3900000,  98, 0 },
	[2]  = { 3850000,  94, 0 },
	[3]  = { 3800000,  89, 0 },
	[4]  = { 3750000,  83, 0 },
	[5]  = { 3700000,  75, 0 },
	[6]  = { 3650000,  64, 0 },
	[7]  = { 3630000,  59, 0 },
	[8]  = { 3600000,  52, 0 },
	[9]  = { 3570000,  40, 0 },
	[10] = { 3550000,  32, 0 },
	[11] = { 3530000,  24, 0 },
	[12] = { 3500000,  17, 0 },
	[13] = { 3450000,   9, 0 },
	[14] = { 3400000,   5, 0 },
	[15] = { 3310000,   0, 0 },
};

static batt_discharge_lut_t lausanne_sdi_icp553436_760_batt_cap[] = 
{
	[0]  = { 4050000,  99, 0 },
	[1]  = { 4010000,  97, 0 },
	[2]  = { 3950000,  91, 0 },
	[3]  = { 3900000,  85, 0 },
	[4]  = { 3850000,  79, 0 },
	[5]  = { 3800000,  72, 0 },
	[6]  = { 3750000,  64, 0 },
	[7]  = { 3700000,  54, 0 },
	[8]  = { 3650000,  39, 0 },
	[9]  = { 3600000,  22, 0 },
	[10] = { 3550000,  11, 0 },
	[11] = { 3500000,   5, 0 },
	[12] = { 3450000,   3, 0 },
	[13] = { 3400000,   1, 0 },
	[14] = { 3310000,   0, 0 },
};

static batt_discharge_lut_t messina_samsung_sdi_icp653443m_1100_batt_cap[] = 
{
	[0]  = { 4050000, 100, 0 },
	[1]  = { 4000000,  99, 0 },
	[2]  = { 3900000,  90, 0 },
	[3]  = { 3800000,  78, 0 },
	[4]  = { 3700000,  62, 0 },
	[5]  = { 3650000,  51, 0 },
	[6]  = { 3630000,  46, 0 },
	[7]  = { 3600000,  36, 0 },
	[8]  = { 3550000,  18, 0 },
	[9]  = { 3500000,   8, 0 },
	[10] = { 3450000,   3, 0 },
	[11] = { 3400000,   2, 0 },
	[12] = { 3350000,   1, 0 },
	[13] = { 3303000,   0, 0 }
};

static batt_discharge_lut_t messina_samsung_sdi_icp553443e_920_batt_cap[] = 
{
	[0]  = { 4080000, 100, 0 },
	[1]  = { 4000000,  94, 0 },
	[2]  = { 3900000,  84, 0 },
	[3]  = { 3800000,  72, 0 },
	[4]  = { 3750000,  64, 0 },
	[5]  = { 3700000,  54, 0 },
	[6]  = { 3670000,  47, 0 },
	[7]  = { 3650000,  37, 0 },
	[8]  = { 3630000,  30, 0 },
	[9]  = { 3600000,  22, 0 },
	[10] = { 3550000,  16, 0 },
	[11] = { 3500000,  12, 0 },
	[12] = { 3450000,   8, 0 },
	[13] = { 3400000,   5, 0 },
	[14] = { 3350000,   2, 0 },
	[15] = { 3301000,   0, 0 },
};

static batt_discharge_lut_t catania_samsung_sdi_920_batt_cap[] = 
{
	[0]  = { 4100000, 100, 0 },
	[1]  = { 4000000,  94, 0 },
	[2]  = { 3900000,  84, 0 },
	[3]  = { 3800000,  72, 0 },
	[4]  = { 3750000,  64, 0 },
	[5]  = { 3700000,  55, 0 },
	[6]  = { 3670000,  47, 0 },
	[7]  = { 3650000,  38, 0 },
	[8]  = { 3630000,  29, 0 },
	[9]  = { 3600000,  21, 0 },
	[10] = { 3550000,  16, 0 },
	[11] = { 3500000,  12, 0 },
	[12] = { 3450000,   8, 0 },
	[13] = { 3400000,   4, 0 },
	[14] = { 3350000,   2, 0 },
	[15] = { 3301000,   0, 0 }
};

static batt_discharge_lut_t lausanne_samsung_sdi_920_batt_cap[] = 
{
	[0]  = { 4100000, 100, 0 },
	[1]  = { 4000000,  94, 0 },
	[2]  = { 3900000,  84, 0 },
	[3]  = { 3800000,  72, 0 },
	[4]  = { 3750000,  64, 0 },
	[5]  = { 3700000,  55, 0 },
	[6]  = { 3670000,  47, 0 },
	[7]  = { 3650000,  38, 0 },
	[8]  = { 3630000,  29, 0 },
	[9]  = { 3600000,  21, 0 },
	[10] = { 3550000,  16, 0 },
	[11] = { 3500000,  12, 0 },
	[12] = { 3450000,   8, 0 },
	[13] = { 3400000,   4, 0 },
	[14] = { 3350000,   2, 0 },
	[15] = { 3301000,   0, 0 }
};

static batt_discharge_lut_t geneva_formosa_maxell_1100_batt_cap[] = 
{
	[0]  = { 4150000, 100, 0 },
	[1]  = { 4050000,  94, 0 },
	[2]  = { 4000000,  90, 0 },
	[3]  = { 3930000,  83, 0 },
	[4]  = { 3850000,  75, 0 },
	[5]  = { 3800000,  68, 0 },
	[6]  = { 3750000,  60, 0 },
	[7]  = { 3699000,  50, 0 },
	[8]  = { 3650000,  37, 0 },
	[9]  = { 3630000,  30, 0 },
	[10] = { 3600000,  20, 0 },
	[11] = { 3552000,  10, 0 },
	[12] = { 3520000,   7, 0 },
	[13] = { 3495000,   5, 0 },
	[14] = { 3457000,   3, 0 },
	[15] = { 3430000,   2, 0 },
	[16] = { 3392000,   1, 0 },
	[17] = { 3330000,   0, 0 }
};

static batt_discharge_lut_t geneva_wtenergy_maxell_1100_batt_cap[] = 
{
	[0]  = { 4120000, 100, 0 },
	[1]  = { 4050000,  96, 0 },
	[2]  = { 4000000,  91, 0 },
	[3]  = { 3930000,  83, 0 },
	[4]  = { 3850000,  74, 0 },
	[5]  = { 3800000,  67, 0 },
	[6]  = { 3750000,  58, 0 },
	[7]  = { 3690000,  45, 0 },
	[8]  = { 3650000,  31, 0 },
	[9]  = { 3630000,  23, 0 },
	[10] = { 3600000,  16, 0 },
	[11] = { 3550000,   7, 0 },
	[12] = { 3520000,   4, 0 },
	[13] = { 3490000,   3, 0 },
	[14] = { 3450000,   2, 0 },
	[15] = { 3390000,   1, 0 },
	[16] = { 3330000,   0, 0 }
};

static batt_discharge_lut_t messina_formosa_maxell_920_batt_cap[] = 
{
	[0]  = { 4130000, 100, 0 },
	[1]  = { 4070000,  97, 0 },
	[2]  = { 4000000,  91, 0 },
	[3]  = { 3900000,  80, 0 },
	[4]  = { 3850000,  75, 0 },
	[5]  = { 3800000,  68, 0 },
	[6]  = { 3750000,  59, 0 },
	[7]  = { 3700000,  46, 0 },
	[8]  = { 3670000,  37, 0 },
	[9]  = { 3650000,  29, 0 },
	[10] = { 3630000,  23, 0 },
	[11] = { 3600000,  17, 0 },
	[12] = { 3580000,  13, 0 },
	[13] = { 3550000,   8, 0 },
	[14] = { 3530000,   6, 0 },
	[15] = { 3500000,   4, 0 },
	[16] = { 3480000,   3, 0 },
	[17] = { 3450000,   2, 0 },
	[18] = { 3400000,   1, 0 },
	[19] = { 3300000,   0, 0 }
};

static batt_discharge_lut_t messina_formosa_maxell_1100_batt_cap[] = 
{
	[0]  = { 4110000, 100, 0 },
	[1]  = { 4100000,  99, 0 },
	[2]  = { 4050000,  95, 0 },
	[3]  = { 4000000,  90, 0 },
	[4]  = { 3950000,  85, 0 },
	[5]  = { 3900000,  81, 0 },
	[6]  = { 3850000,  74, 0 },
	[7]  = { 3800000,  66, 0 },
	[8]  = { 3750000,  56, 0 },
	[9]  = { 3700000,  43, 0 },
	[10] = { 3670000,  34, 0 },
	[11] = { 3650000,  26, 0 },
	[12] = { 3630000,  19, 0 },
	[13] = { 3600000,  14, 0 },
	[14] = { 3570000,   9, 0 },
	[15] = { 3550000,   7, 0 },
	[16] = { 3500000,   3, 0 },
	[17] = { 3450000,   2, 0 },
	[18] = { 3400000,   1, 0 },
	[19] = { 3350000,   0, 0 }
};

static batt_discharge_lut_t sarnen_formosa_maxell_1100_batt_cap[] = 
{
	[0]  = { 4070000, 100, 0 },
	[1]  = { 4000000,  97, 0 },
	[2]  = { 3900000,  87, 0 },
	[3]  = { 3850000,  82, 0 },
	[4]  = { 3800000,  76, 0 },
	[5]  = { 3750000,  68, 0 },
	[6]  = { 3700000,  59, 0 },
	[7]  = { 3670000,  53, 0 },
	[8]  = { 3650000,  48, 0 },
	[9]  = { 3630000,  42, 0 },
	[10] = { 3600000,  30, 0 },
	[11] = { 3580000,  23, 0 },
	[12] = { 3550000,  16, 0 },
	[13] = { 3530000,  13, 0 },
	[14] = { 3500000,   7, 0 },
	[15] = { 3480000,   4, 0 },
	[16] = { 3450000,   3, 0 },
	[17] = { 3400000,   1, 0 },
	[18] = { 3300000,   0, 0 }
};

static batt_discharge_lut_t sarnen_wtenergy_maxell_1100_batt_cap[] = 
{
	[0]  = { 4040000, 100, 0 },
	[1]  = { 4000000,  98, 0 },
	[2]  = { 3900000,  88, 0 },
	[3]  = { 3850000,  82, 0 },
	[4]  = { 3800000,  76, 0 },
	[5]  = { 3750000,  68, 0 },
	[6]  = { 3700000,  60, 0 },
	[7]  = { 3670000,  53, 0 },
	[8]  = { 3650000,  48, 0 },
	[9]  = { 3630000,  43, 0 },
	[10] = { 3600000,  32, 0 },
	[11] = { 3580000,  24, 0 },
	[12] = { 3550000,  17, 0 },
	[13] = { 3530000,  13, 0 },
	[14] = { 3500000,   7, 0 },
	[15] = { 3480000,   5, 0 },
	[16] = { 3450000,   3, 0 },
	[17] = { 3400000,   1, 0 },
	[18] = { 3300000,   0, 0 }
};

static batt_discharge_lut_t lausanne_wtenergy_maxell_720_batt_cap[] = 
{
	[0]  = { 4070000, 100, 0 },
	[1]  = { 4000000,  97, 0 },
	[2]  = { 3900000,  87, 0 },
	[3]  = { 3850000,  82, 0 },
	[4]  = { 3800000,  75, 0 },
	[5]  = { 3750000,  67, 0 },
	[6]  = { 3700000,  58, 0 },
	[7]  = { 3670000,  51, 0 },
	[8]  = { 3650000,  45, 0 },
	[9]  = { 3630000,  38, 0 },
	[10] = { 3600000,  28, 0 },
	[11] = { 3580000,  23, 0 },
	[12] = { 3550000,  16, 0 },
	[13] = { 3530000,  12, 0 },
	[14] = { 3500000,   8, 0 },
	[15] = { 3480000,   6, 0 },
	[16] = { 3450000,   4, 0 },
	[17] = { 3400000,   2, 0 },
	[18] = { 3300000,   0, 0 }
};

static batt_esr_curve_t bcm4760_std_batt_esr[] = 
{
	[0] = { 4162300, 259600	},
	[1] = { 4096000, 233300	},
	[2] = { 4022800, 232800	},
	[3] = { 3815800, 243000	},
	[4] = { 3609000, 271100	},
	[5] = { 3402000, 285700	},
	[6] = { 3200000, 305300 },
};

static struct bcm_battery siena_foxlink_maxell_1100_batt = {
	.model				= "Siena_Foxlink_Maxell_653443LIR_1100",
	.discharge_curves		= siena_foxlink_maxell_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(siena_foxlink_maxell_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery siena_samsung_sdi_1100_batt = {
	.model				= "Siena_Samsung_SDI_1100",
	.discharge_curves		= siena_samsung_sdi_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(siena_samsung_sdi_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery siena_formosa_maxell_1100_batt = {
	.model				= "Siena_Formosa_Maxell_653443LIR_1100",
	.discharge_curves		= siena_formosa_maxell_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(siena_formosa_maxell_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery carlisle_samsung_sdi_1100_batt = {
	.model				= "Carlisle_Samsung_SDI_1100",
	.discharge_curves		= carlisle_samsung_sdi_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(carlisle_samsung_sdi_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery carlisle_formosa_maxell_1100_batt = {
	.model				= "Carlisle_Formosa_Maxell_653443LIR_1100",
	.discharge_curves		= carlisle_formosa_maxell_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(carlisle_formosa_maxell_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery carlisle_foxlink_maxell_1100_batt = {
	.model				= "Carlisle_Foxlink_Maxell_653443LIR_1100",
	.discharge_curves		= carlisle_foxlink_maxell_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(carlisle_foxlink_maxell_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery catania_920_batt = {
	.model				= "Catania_KL1_Maxell_ICP553443AR_920",
	.discharge_curves		= catania_920_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(catania_920_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery catania_formosa_maxel_920_batt = {
	.model				= "Catania_Formosa_Maxell_ICP553443AR_920",
	.discharge_curves		= catania_920_formosa_maxell_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(catania_920_formosa_maxell_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery catania_900_batt = {
	.model				= "Catania_KL1_Lishen_553443RD_900",
	.discharge_curves		= catania_900_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(catania_900_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery catania_770_batt = {
	.model				= "Catania_KM1_Sanyo_GS_LP523436D_770",
	.discharge_curves		= catania_770_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(catania_770_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery catania_720_batt = {
	.model				= "Catania_KM1_Lishen_LP553436AB_720",
	.discharge_curves		= catania_720_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(catania_720_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery catania_formosa_maxell_720_batt = {
	.model				= "Catania_Formosa_Maxell_ICP053436AB_720",
	.discharge_curves		= catania_formosa_maxell_720_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(catania_formosa_maxell_720_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery catania_sdi_icp553436_760_batt = {
	.model				= "Catania_Samsung_SDI_ICP553436_760",
	.discharge_curves		= catania_sdi_icp553436_760_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(catania_sdi_icp553436_760_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery messina_920_batt = {
	.model				= "Messina_Foxlink_Maxell_553443LIR_920",
	.discharge_curves		= messina_920_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(messina_920_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery messina_1100_batt = {
	.model				= "Messina_Foxlink_Maxell_653443LIR_1100",
	.discharge_curves		= messina_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(messina_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery siena_carlisle_samsung_sdi_1100_batt = {
	.model				= "Siena_Carlisle_Samsung_SDI_653443LIR_1100",
	.discharge_curves		= siena_carlisle_samsung_sdi_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(siena_carlisle_samsung_sdi_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery lausanne_720_batt = {
	.model				= "Lausanne_KM1_Lishen_LP553436AB_720",
	.discharge_curves		= lausanne_720_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(lausanne_720_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery lausanne_920_batt = {
	.model				= "Lausanne_KL1_Maxell_ICP553443AR_920",
	.discharge_curves		= lausanne_920_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(lausanne_920_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery lausanne_formosa_maxell_920_batt = {
	.model				= "Lausanne_Formosa_Maxell_ICP553443AR_920",
	.discharge_curves		= lausanne_920_formosa_maxell_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(lausanne_920_formosa_maxell_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery lausanne_formosa_maxell_720_batt = {
	.model				= "Lausanne_Formosa_Maxell_ICP053436AB_720",
	.discharge_curves		= lausanne_formosa_maxell_720_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(lausanne_formosa_maxell_720_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery lausanne_sdi_icp553436_760_batt = {
	.model				= "Lausanne_Samsung_SDI_ICP553436_760",
	.discharge_curves		= lausanne_sdi_icp553436_760_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(lausanne_sdi_icp553436_760_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery messina_samsung_sdi_icp553443e_920_batt = {
	.model				= "Messina_Samsung_SDI_ICP553443E_920",
	.discharge_curves		= messina_samsung_sdi_icp553443e_920_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(messina_samsung_sdi_icp553443e_920_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery messina_samsung_sdi_icp653443m_1100_batt = {
	.model				= "Messina_Samsung_SDI_ICP653443M_1100",
	.discharge_curves		= messina_samsung_sdi_icp653443m_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(messina_samsung_sdi_icp653443m_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery catania_samsung_sdi_920_batt = {
	.model				= "Catania_Samsung_SDI_ICP553443E_920",
	.discharge_curves		= catania_samsung_sdi_920_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(catania_samsung_sdi_920_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery lausanne_samsung_sdi_920_batt = {
	.model				= "Lausanne_Samsung_SDI_ICP553443E_920",
	.discharge_curves		= lausanne_samsung_sdi_920_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(lausanne_samsung_sdi_920_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery geneva_formosa_maxell_1100_batt = {
	.model				= "Geneva_Formosa_Maxell_653443LIR_1100",
	.discharge_curves		= geneva_formosa_maxell_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(geneva_formosa_maxell_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery geneva_wtenergy_maxell_1100_batt = {
	.model				= "Geneva_Wtenergy_Maxell_653443LIR_1100",
	.discharge_curves		= geneva_wtenergy_maxell_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(geneva_wtenergy_maxell_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery messina_formosa_maxell_920_batt = {
	.model				= "Messina_Formosa_Maxell_ICP553443AR_920",
	.discharge_curves		= messina_formosa_maxell_920_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(messina_formosa_maxell_920_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery messina_formosa_maxell_1100_batt = {
	.model				= "Messina_Formosa_Maxell_ICP653443AR_1100",
	.discharge_curves		= messina_formosa_maxell_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(messina_formosa_maxell_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery sarnen_formosa_maxell_1100_batt = {
	.model				= "Sarnen_Formosa_Maxell_ICP653443AR_1100",
	.discharge_curves		= sarnen_formosa_maxell_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(sarnen_formosa_maxell_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery sarnen_wtenergy_maxell_1100_batt = {
	.model				= "Sarnen_Wtenergy_Maxell_ICP653443AR_1100",
	.discharge_curves		= sarnen_wtenergy_maxell_1100_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(sarnen_wtenergy_maxell_1100_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery lausanne_wtenergy_maxell_720_batt = {
	.model				= "Lausanne_Wtenergy_Maxell_ICP054346_720",
	.discharge_curves		= lausanne_wtenergy_maxell_720_batt_cap,
	.discharge_curve_entries	= ARRAY_SIZE(lausanne_wtenergy_maxell_720_batt_cap),
	.esr_curves			= bcm4760_std_batt_esr,
	.esr_curve_entries		= ARRAY_SIZE(bcm4760_std_batt_esr),
};

static struct bcm_battery *batteries[] =
{
	&siena_formosa_maxell_1100_batt,	// Siena    1st source
	&siena_foxlink_maxell_1100_batt,	// Siena    2nd source
	&siena_samsung_sdi_1100_batt,		// Siena    2nd source
	&carlisle_formosa_maxell_1100_batt,	// Carlisle 1st source
	&carlisle_samsung_sdi_1100_batt,	// Carlisle 2nd source
	&carlisle_foxlink_maxell_1100_batt,	// Carlisle 2nd source
	&siena_carlisle_samsung_sdi_1100_batt,	// Carlisle & Siena 2nd source
	&catania_920_batt,			// Catania
	&catania_900_batt,			// Catania
	&catania_770_batt,			// Catania
	&catania_720_batt,			// Catania
	&catania_samsung_sdi_920_batt,                // Catania
        &messina_920_batt,                      // Messina
        &messina_1100_batt,                     // Messina
        &lausanne_720_batt,                     // Lausanne
        &lausanne_920_batt,                     // Lausanne
        &lausanne_samsung_sdi_920_batt,         // Lausanne 5' 2nd source
        &messina_samsung_sdi_icp553443e_920_batt,    // Messina 2nd source
        &messina_samsung_sdi_icp653443m_1100_batt,   // Messina 2nd source
        &lausanne_formosa_maxell_920_batt,           // Lausanne 5' 3rd source
        &catania_formosa_maxel_920_batt,             // Catania 5' 3rd source
        &lausanne_formosa_maxell_720_batt,           //Lausanne 4.3' 2nd source 
        &catania_formosa_maxell_720_batt,            //Catania 4.3' 2nd source 
        &catania_sdi_icp553436_760_batt,             //Catania 4.3' 3nd source  
        &lausanne_sdi_icp553436_760_batt,            //Lausanne 4.3' 3nd source  
        &geneva_formosa_maxell_1100_batt,            // Geneva 1st source
        &geneva_wtenergy_maxell_1100_batt,           // Geneva 2nd source
        &messina_formosa_maxell_920_batt,           // Messina 4, 3rd source
        &messina_formosa_maxell_1100_batt,          // Messina 5, 3rd source
        &sarnen_formosa_maxell_1100_batt,           // Sarnen 1st source
        &sarnen_wtenergy_maxell_1100_batt,          // Sarnen 2nd source        
        &lausanne_wtenergy_maxell_720_batt,        // Lausanne/Lanzhou 4th source        
};

struct bcm_battery *default_batt = &siena_formosa_maxell_1100_batt;

static struct bcm_batt_info bcm4760_std_battery = {
    .adc = {
	.battery_voltage_resistor_1	= 0,
	.battery_voltage_resistor_2	= 0,
	.charge_current_resistor	= 10000,		/* Resistance in uOhm */
	.reference_voltage		= 0,
	.accuracy			= 0,
	.correction			= 0
    },
};

static struct bcm_battery_platform_info bcm4760_battery_platinfo= {
	.platform_init		= bcm_batt_platform_init,
	.platform_remove	= bcm_batt_platform_remove,
	.adc_read		= bcm_adc_read,
	.battery_info		= &bcm4760_std_battery,

#if defined(CONFIG_BATTERY_BCM59040)
	.get_charge_current_adc	= bcm_batt_get_charge_current,
	.get_fuel_gauge_sample	= bcm_batt_get_fuel_gauge_sample,
#endif

};

static struct platform_device bcm59040_battery_device = 
{
    .name           = BCM_BATTERY_DRIVER,
    .id             = -1,
    .dev = {
    	.platform_data	= &bcm4760_battery_platinfo,
    },
};

static __init int bcm4760_battery_init(void)
{
	struct bcm_battery *battery = NULL;
	const char *batt_model;
	int i;

	batt_model = fdt_get_string("/options/battery", "battery-model", "");

	for (i = 0; i < ARRAY_SIZE(batteries); i++) {
		if (strcmp(batt_model, batteries[i]->model) == 0) {
			battery = batteries[i];
			break;
		}
	}
	if (!battery) {
		// use the default
		printk (KERN_INFO PFX " Couldn't find battery model %s.\n", batt_model[0] ? batt_model : "not set");
		battery = default_batt;
	} else {
		printk (KERN_INFO PFX " Found battery model %s.\n", batt_model);
	}
	bcm4760_battery_platinfo.battery_info->battery = battery;
	return platform_device_register(&bcm59040_battery_device);
}

module_init(bcm4760_battery_init);
