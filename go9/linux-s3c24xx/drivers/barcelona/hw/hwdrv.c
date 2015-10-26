/* drivers/barcelona/hw/hwdrv.c
 *
 * Implementation of the hardware detection driver.
 *
 * Copyright (C) 2004-2007 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
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
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/hardware/clock.h>
#include <barcelona/gopins.h>
#include <barcelona/debug.h>
#include <barcelona/Barc_Gpio.h>

extern int barcelona_reboot_on_sd_removal;
extern HARDWARE_STATUS gpio_hw_status;

/* Defines */
#define PFX "hw: "
#define PK_DBG PK_DBG_FUNC

/* Local variables */
static const char hw_proc_root_name[] = "barcelona";
static struct proc_dir_entry *hw_proc_root;

/* Local types */
struct hw_proc_entry {
	const char *name;
	read_proc_t *read_func;
	write_proc_t *write_func;

};

extern unsigned int reset_state;
extern char boot_device_id[24];

static int hw_proc_read(char *page, char **start, off_t offset, int count, int *eof, const char *msg)
{
	int avail = strlen(msg) - offset;

	if (count >= avail) {
		count = avail;
		*eof = 1;
	}
	if (count <= 0)
		return 0;

	*start = page;
	memcpy(page, msg + offset, count);

	return count;
}

static int hw_proc_read_int(char *page, char **start, off_t offset, int count, int *eof, int i)
{
	unsigned char msg[16];
	snprintf(msg, sizeof msg, "%d\n", i);
	return hw_proc_read(page, start, offset, count, eof, msg);
}

static int hw_proc_read_u16(char *page, char **start, off_t offset, int count, int *eof, u16 u)
{
	unsigned char msg[16];
	snprintf(msg, sizeof msg, "%u\n", (unsigned) u);
	return hw_proc_read(page, start, offset, count, eof, msg);
}

static int hw_proc_read_str(char *page, char **start, off_t offset, int count, int *eof, const char *s)
{
	unsigned char msg[128];
	snprintf(msg, sizeof msg, "%s\n", s == NULL ? "" : s);
	return hw_proc_read(page, start, offset, count, eof, msg);
}

static int hw_proc_read_boot_device(char* page, char **start, off_t offset, int count, int *eof, const void* buf, size_t maxlen)
{
	unsigned char msg[2*maxlen+1+1];
	const unsigned char* p = buf;
	int binary = !(p[19] >= ' ');

	if (binary) { /* If we're binary, then we can assume we're a SD/MMC CID, so convert it to hex */
		int i;
		for (i=0; i < 16; i++) {
			sprintf(msg+(i*2), "%02x", p[i]);
		}

		strcat(msg, "\n");
	} else {
		strncpy(msg, buf, maxlen);
		msg[maxlen+1] = '\n';
		msg[maxlen+2] = '\0';
	}

	return hw_proc_read(page, start, offset, count, eof, msg);
}

/*
 * This function will scan the input buffer for an integer. If it was able to do a conversion, it will use that value and
 * store it at location i. The rest of the input is ignored but accepted (count is returned).
 * If no conversion could be done, EINVAL is returned.
*/
static int hw_proc_write_int(struct file *file, const char __user *buffer, unsigned long count, void *data, int *p)
{
	int result;
	int value;

	result = sscanf(buffer, "%i", &value);
	if (!result) {
		/* No conversion could be done	*/
		return -EINVAL;
	}
	*p = value; /* assign the value */
	return count;
}

static int hw_proc_write_u16(struct file *file, const char __user *buffer, unsigned long count, void *data, u16 *p)
{
	int result;
	unsigned value;

	result = sscanf(buffer, "%u", &value);
	if (!result || value < 0 || value > 65535) {
		/* No conversion could be done	*/
		return -EINVAL;
	}
	*p = (u16) value; /* assign the value */
	return count;
}

#define PROC_READ_INT(name, func) \
	int hw_proc_read_##name(char *page, char **start, off_t offset, int count, int *eof, void *data) \
	{ \
		return hw_proc_read_int(page, start, offset, count, eof, IO_##func()); \
	}

#define PROC_READ_INT_VAR(name, var) \
	int hw_proc_read_##name(char *page, char **start, off_t offset, int count, int *eof, void *data) \
	{ \
		return hw_proc_read_int(page, start, offset, count, eof, var); \
	}

#define PROC_READ_U16_VAR(name, var) \
	int hw_proc_read_##name(char *page, char **start, off_t offset, int count, int *eof, void *data) \
	{ \
		return hw_proc_read_u16(page, start, offset, count, eof, var); \
	}

#define PROC_READ_STR(name, func) \
	int hw_proc_read_##name(char *page, char **start, off_t offset, int count, int *eof, void *data) \
	{ \
		return hw_proc_read_str(page, start, offset, count, eof, IO_##func()); \
	}

#define PROC_READ_BOOT(name, arr) \
	int hw_proc_read_##name(char *page, char **start, off_t offset, int count, int *eof, void *data) \
	{ \
		return hw_proc_read_boot_device(page, start, offset, count, eof, arr, sizeof(arr)); \
	}

#define PROC_WRITE_INT_VAR(name, var) \
	int hw_proc_write_##name(struct file *file, const char __user *buffer, unsigned long count, void *data) \
	{ \
		return hw_proc_write_int(file, buffer, count, data, &var); \
	}

#define PROC_WRITE_U16_VAR(name, var) \
	int hw_proc_write_##name(struct file *file, const char __user *buffer, unsigned long count, void *data) \
	{ \
		return hw_proc_write_u16(file, buffer, count, data, &var); \
	}

PROC_READ_BOOT(bootdevice, boot_device_id)
PROC_READ_INT(modelid, GetModelId)
PROC_READ_STR(modelname, GetModelName)
PROC_READ_STR(familyname, GetFamilyName)
PROC_READ_STR(usbname, GetUsbName)
PROC_READ_STR(btname, GetBtName)
PROC_READ_INT(cputype, GetCpuType)

PROC_READ_INT(bluetooth, HaveBluetooth)
PROC_READ_STR(btdev, GetBluetoothDevice)
PROC_READ_INT(btchip, GetBluetoothChip)
PROC_READ_INT(btusb, HaveBluetoothUsb)
PROC_READ_INT(btspeed, GetBluetoothSpeed)
PROC_READ_INT(btclock, GetBluetoothClock)
PROC_READ_INT(btclass, GetBluetoothClass)
PROC_READ_INT(handsfree, HaveHandsfree)
PROC_READ_INT(headsetgw, HaveHeadsetGw)
PROC_READ_INT(a2dp, HaveA2DP)
PROC_READ_INT(lowbutton, HaveLoweredButton)
PROC_READ_INT(sdcard, HaveSdCardInterface)
PROC_READ_INT(hs_sdcard, HaveHsSdCardInterface)
PROC_READ_INT(hsmmc, HaveHsMmcInterface)
PROC_READ_INT(sdslot, HaveSdSlot)
PROC_READ_INT(harddisk, HaveHarddisk)
PROC_READ_INT(tsfets, HaveTsFets)
PROC_READ_INT(remote, HaveRemote)
PROC_READ_STR(dockdev, GetDockDevice)
PROC_READ_INT(docking, HaveDocking)
PROC_READ_INT(dockstate, GetDockState)
PROC_READ_INT(tfttype, GetTftType)
PROC_READ_INT(gpstype, GetGpsType)
PROC_READ_STR(gpsdev, GetGpsDevice)
PROC_READ_INT(gldetected, HaveGlDetected)
PROC_READ_INT(gpsephemeris, HaveGpsEphemeris)
PROC_READ_INT(headphoneconnector, HaveHeadphoneConnector)
PROC_READ_INT(codectype, GetCodecType)
PROC_READ_INT(canrecordaudio, CanRecordAudio)
PROC_READ_INT(codecmaster, GetCodecMaster)
PROC_READ_INT(acctype, GetAccType)
PROC_READ_INT(usbslavetype, GetUSBSlaveType)
PROC_READ_INT(lightsensor, HaveLightSensor)
PROC_READ_INT(deadreckoning, HaveDeadReckoning)
PROC_READ_INT(loquendo, HaveLoquendo)
PROC_READ_INT(alg, HaveAdvancedLaneGuidanceFeature)
PROC_READ_INT(rdstmc, HaveRDSTMCFeature)
PROC_READ_INT(sixbuttonui, HaveSixButtonUiFeature)
PROC_READ_INT(tomtomwork, HasTomTomWorkFeatures)
PROC_READ_INT(gprsmodem, HaveGprsModem)
PROC_READ_STR(gprsmodemdev, GetGprsModemDevice)
PROC_READ_INT(pnp, HavePNP)
PROC_READ_INT(usbhost, HaveUsbHost)
PROC_READ_INT_VAR(resetstate, reset_state)
PROC_READ_INT(fmtransmitter, HaveFMTransmitter)
PROC_READ_INT(internalflash, HaveInternalFlash)
PROC_READ_INT(batterytype, GetBatteryType)

PROC_READ_INT_VAR(rebootonsdremoval, barcelona_reboot_on_sd_removal)
PROC_WRITE_INT_VAR(rebootonsdremoval, barcelona_reboot_on_sd_removal)
PROC_READ_U16_VAR(onoff, gpio_hw_status.u8InputStatus)
PROC_WRITE_U16_VAR(onoff, gpio_hw_status.u8InputStatus)

#define HW_PROC_RO_ENTRY(x) { .name = #x, .read_func = hw_proc_read_##x }
#define HW_PROC_RW_ENTRY(x) { .name = #x, .read_func = hw_proc_read_##x, .write_func = hw_proc_write_##x }
#define HW_PROC_WO_ENTRY(x) { .name = #x, .write_func = hw_proc_write_##x }

static struct hw_proc_entry hw_proc_entries[] = {
	HW_PROC_RO_ENTRY(modelid),
	HW_PROC_RO_ENTRY(modelname),
	HW_PROC_RO_ENTRY(familyname),
	HW_PROC_RO_ENTRY(usbname),
	HW_PROC_RO_ENTRY(btname),
	HW_PROC_RO_ENTRY(cputype),
	HW_PROC_RO_ENTRY(bluetooth),
	HW_PROC_RO_ENTRY(btdev),
	HW_PROC_RO_ENTRY(btchip),
	HW_PROC_RO_ENTRY(btusb),
	HW_PROC_RO_ENTRY(btspeed),
	HW_PROC_RO_ENTRY(btclock),
	HW_PROC_RO_ENTRY(btclass),
	HW_PROC_RO_ENTRY(handsfree),
	HW_PROC_RO_ENTRY(headsetgw),
	HW_PROC_RO_ENTRY(a2dp),
	HW_PROC_RO_ENTRY(lowbutton),
	HW_PROC_RO_ENTRY(sdcard),
	HW_PROC_RO_ENTRY(hs_sdcard),
	HW_PROC_RO_ENTRY(sdslot),
	HW_PROC_RO_ENTRY(hsmmc),
	HW_PROC_RO_ENTRY(harddisk),
	HW_PROC_RO_ENTRY(tsfets),
	HW_PROC_RO_ENTRY(remote),
	HW_PROC_RO_ENTRY(dockdev),
	HW_PROC_RO_ENTRY(docking),
	HW_PROC_RO_ENTRY(dockstate),
	HW_PROC_RO_ENTRY(tfttype),
	HW_PROC_RO_ENTRY(gpstype),
	HW_PROC_RO_ENTRY(gpsdev),
	HW_PROC_RO_ENTRY(gldetected),
	HW_PROC_RO_ENTRY(gpsephemeris),
	HW_PROC_RO_ENTRY(headphoneconnector),
	HW_PROC_RO_ENTRY(codectype),
	HW_PROC_RO_ENTRY(canrecordaudio),
	HW_PROC_RO_ENTRY(codecmaster),
	HW_PROC_RO_ENTRY(acctype),
	HW_PROC_RO_ENTRY(usbslavetype),
	HW_PROC_RO_ENTRY(lightsensor),
	HW_PROC_RO_ENTRY(deadreckoning),
	HW_PROC_RO_ENTRY(loquendo),
	HW_PROC_RO_ENTRY(alg),
	HW_PROC_RO_ENTRY(tomtomwork),
	HW_PROC_RO_ENTRY(resetstate),
	HW_PROC_RO_ENTRY(rdstmc),
	HW_PROC_RO_ENTRY(sixbuttonui),
	HW_PROC_RO_ENTRY(gprsmodem),
	HW_PROC_RO_ENTRY(gprsmodemdev),
	HW_PROC_RO_ENTRY(pnp),
	HW_PROC_RO_ENTRY(usbhost),
	HW_PROC_RO_ENTRY(fmtransmitter),
	HW_PROC_RO_ENTRY(internalflash),
	HW_PROC_RO_ENTRY(batterytype),
	HW_PROC_RO_ENTRY(bootdevice),
	HW_PROC_RW_ENTRY(rebootonsdremoval),
	HW_PROC_RW_ENTRY(onoff),
};

static int hw_proc_init(void)
{
	int i;

	hw_proc_root = proc_mkdir(hw_proc_root_name, NULL);
	if (hw_proc_root == NULL) {
		PK_ERR("Unable to create hw /proc root %s\n", hw_proc_root_name);
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(hw_proc_entries); ++i) {
		mode_t mode = 0;
		struct proc_dir_entry *res;

		if (hw_proc_entries[i].read_func)
			mode |= S_IRUGO;
		if (hw_proc_entries[i].write_func)
			mode |= S_IWUGO;

		res = create_proc_entry(hw_proc_entries[i].name , mode, hw_proc_root);
        	if (res) {
                	res->read_proc = hw_proc_entries[i].read_func; /* can be NULL */
                	res->write_proc = hw_proc_entries[i].write_func; /*can be NULL */
                	res->data = NULL;
        	}
		else{
			PK_ERR("Unable to create hw /proc entry %s (mode is 0%o)\n", hw_proc_entries[i].name, mode);
			return -1;
		}
	}

	return 0;
}

static void hw_proc_exit(void)
{
	int i;

	if (hw_proc_root != NULL) {
		for (i = ARRAY_SIZE(hw_proc_entries) - 1; i >= 0; --i) {
			remove_proc_entry(hw_proc_entries[i].name, hw_proc_root);
		}
		remove_proc_entry(hw_proc_root_name, NULL);
		hw_proc_root = NULL;
	}
}

static int hw_probe(struct device *dev)
{
	int ret;

	PK_DBG("Setting up proc entries\n");
	ret = hw_proc_init();
	if (ret < 0) {
		PK_ERR("Unable to setup proc entries\n");
		return -ENODEV;
	}

	PK_DBG("Done\n");
	return 0;
}

static int hw_remove(struct device *dev)
{
	PK_DBG("Removing proc entries\n");
	hw_proc_exit();

	PK_DBG("Done\n");
	return 0;
}

static void hw_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");
}

#ifdef CONFIG_PM

static int hw_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);
	if (level == SUSPEND_POWER_DOWN) {
		hw_shutdown(dev);
	}
	return 0;
}

static int hw_resume(struct device *dev, u32 level)
{
	PK_DBG("dev = %p, level = %u\n", dev, level);
	return 0;
}

#else /* CONFIG_PM */
#define hw_suspend NULL
#define hw_resume  NULL
#endif /* CONFIG_PM */

static struct device_driver hw_driver = {
	.name		= "tomtomgo-hw",
	.bus		= &platform_bus_type,
	.probe		= hw_probe,
	.remove		= hw_remove,
	.shutdown	= hw_shutdown,
	.suspend	= hw_suspend,
	.resume		= hw_resume,
};

static int __init hw_mod_init(void)
{
	int ret;

	printk(KERN_INFO "TomTom GO Hardware Detection Driver, (C) 2004,2005 TomTom BV\n");
	PK_DBG("Registering driver\n");
	ret = driver_register(&hw_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit hw_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&hw_driver);
	PK_DBG("Done\n");
}

module_init(hw_mod_init);
module_exit(hw_mod_exit);

MODULE_AUTHOR("Dimitry Andric <dimitry.andric@tomtom.com> and Jeroen Taverne <jeroen.taverne@tomtom.com>");
MODULE_DESCRIPTION("TomTom GO Hardware Detection Driver");
MODULE_LICENSE("GPL");

/* EOF */
