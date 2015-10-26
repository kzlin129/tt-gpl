#include <asm/arch/pmu_device_bcm59040.h>
#include <plat/irvine-pmu.h>

static struct bcm59040_control_defaults pnd_control_defaults = 
{
	0x79,	/* PONKCNTRL1 */
	0x2F,	/* PONKCNTRL2 */
	0x85	/* RSTRTCNTRL, RESTARTHOLD is 15s */	
};

static struct bcm59040_batt_defaults pnd_batt_defaults =
{
	/* Factory defaults same as silicon defaults */
	0xA8,	/* FGCTRL1 */
	0x00,	/* FGCTRL2 */
	0x00	/* FGCTRL3 */
};

/* TODO: 
 *   Inicializar todos los GPIO del PMU como salidas
 */

static struct bcm59040_charger_defaults pnd_charger_defaults =
{
	/* Enable watchdog timer | Disable CV WDT */
	0xAD,	/* MBCCTRL1 */	/* 10101101 */
	/* Disabled CC1 and CC2 */
	0xF8,	/* MBCCTRL2 */	/* 11111000 */
	/* Enable charger | WALL regulated to FC2 | USBC regulated to FC1 */
	/* Enable Maintenance charge | WAC and USB0C regulates voltage to CV */
	/* USB1C stops charging when VUBGR_FC2 = 0 and VMBAT > MBWV and CHGDET = 1 */
	0x73,	/* MBCCTRL3 */	/* 01110011 */
	/* FC1 and FC2 CV regulated to 3.75 */
	0x00,	/* MBCCTRL4 */	/* 00000000 */
	/* Qualification charging current 0mA */
	0x00,	/* MBCCTRL5 */	/* 00000000 */
	/* CC Charging current 500 mA */
	0x15,	/* MBCCTRL6 */	/* 00010101 */
	/* End of current charge: 100mA */
	0x05,	/* MBCCTRL7 */	/* 00000101 */
	/* THE PRIORITY SHOULD NOT BE CHANGED!!!!, this need to match the OTP otherwise we run into booting problems */
	/* Disable USB LDO | 15 clocks transit | WAC has higher prio */
	/* System can be boot up with > 100mA */
	0xF8,	/* MBCCTRL8 */	/* 11111000 */
	/* USB Limit = 500mA | WAC limit 500mA */
	/* disable WCMBC and UBMBC */
	/* VSR enabled if USBC applied */
	0x95,	/* MBCCTRL9 */	/* 10011111 */
	/* VSR enabled if WAC attached */
	/* Don't resume charge unless the failed one is removed */
	/* 1 time temperature can go too high or too low before int is generated */
	0x1F,	/* MBCCTRL10 */	/* 00011111 */
	/* Continous mode | MBS block is ON | Delay 8ms | On in all PM modes */
	0x02,	/* NTCCTRL1 */	/* 00000010 */
	0x03,	/* NTCCTRL2 */
	0x0F,	/* MBTEMPHYS. Hysteresis set to 0%. This since 6% means temp needs to drop down to 34.5oC, */
		/* 4% to 37.75oC, 3% to 39.5oC. Not ideal, since we can't use interrupts, but otherwise */
		/* around 45 degrees we run the risk of toggling and any other temperature would give us */
		/* the risk of not being able to charge (much). */
	1100	/* Capacity of the battery in mA. */
};

void bcm59040_register_pnd_defaults(void)
{
	irvine_pmu_register_defaults("battery", &pnd_batt_defaults);
	irvine_pmu_register_defaults("charger", &pnd_charger_defaults);
	irvine_pmu_register_defaults("control", &pnd_control_defaults);
}

