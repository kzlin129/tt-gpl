#include <linux/kernel.h>
#include <barcelona/debug.h>
#include <barcelona/gopins.h>

#include "spi.h"
#include "wm8711.h"

#define PFX "wm8711: "
#define PK_DBG PK_DBG_FUNC

#define R8_FS256 0x20
#define R8_FS384 0x22

#define PFX "wm8711: "
#define PK_DBG PK_DBG_FUNC

static uint16_t wm8711[] = {
	0x0,		/* R0 */
	0x0,		/* R1 */
	0x180,		/* R2 */
	0x0,		/* R3 */
	0x10,		/* R4 */
	0x0,		/* R5 */
	0x60,		/* R6, Power down the clock and the osscilator. */
	0x02,		/* R7 */
	R8_FS256,	/* R8 */
	0x01		/* R9 */
};

static inline void WM8711_WRREG(uint16_t reg, uint16_t value)
{
	if (reg <= 9)
		wm8711[reg] = value;
	PK_DBG("R%d=0x%x\n", reg, value);
	spi_write_register(reg, value);
}

static inline uint16_t WM8711_RDREG(uint16_t reg)
{
	return wm8711[reg];
}

static void wm8711_init_registers(void)
{
	int i;

	WM8711_WRREG(15, 0);
	for (i = 0; i < sizeof wm8711 / sizeof *wm8711; i++) {
		if (WM8711_RDREG(i)) WM8711_WRREG(i, WM8711_RDREG(i));
	}
	/* Unmute. */
	WM8711_WRREG(5, 0);
}

static int wm8711_set_volume(unsigned int volume)
{
	if (volume > 121) volume = 121;

	PK_DBG("Setting volume to: %d\n", volume);

	WM8711_WRREG(2, 0x180 | volume);
	return 0;
}

static int wm8711_set_volume_dB(unsigned int volume)
{
	if (volume < 90) 
		volume = 121 - volume;
	else
		volume = 0; /* mute */

	PK_DBG("Setting volume to: %d\n", volume);

	WM8711_WRREG(2, 0x180 | volume);
	return 0;
}

static int wm8711_set_treble(unsigned int treble)
{
	/* Treble is not supported, so do nothing */
	return 0;
}

static int wm8711_set_bass(unsigned int bass)
{
	/* Bass is not supported, so do nothing */
	return 0;
}

static int wm8771_set_fs(enum codec_fs fs,unsigned int samplerate, unsigned int clockrate)
{
	uint16_t newr8;

	if ((IO_GetCodecMaster()) && (!IO_HasPin(CDCLK_12MHZ))) {
		/* Set codec in master mode */
		WM8711_WRREG(7, 0x42);

		/* Codec is master with fixed 6 MHz clock so set sample frequency according to double the rate as defined in the datasheet */
		switch (samplerate) {
		case 16000:
			newr8 = 0x19;
			break;
		case 22050:
			newr8 = 0x23;
			break;
		case 24000:
			newr8 = 0x01;
			break;
		case 44100:
			newr8 = 0x3f;
			break;
		case 48000:
			newr8 = 0x1d;
			break;
		default:
			return -1;
		}
	} else {
		switch (fs) {
		case FS256:
			newr8 = R8_FS256;
			break;
		case FS384:
			newr8 = R8_FS384;
			break;
		default:
			PK_ERR("Unknown fs %d\n", (int) fs);
			return -EINVAL;
		}
	}
	WM8711_WRREG(8, newr8);

	return 0;
}

static int wm8711_switch_line_in(unsigned int line_in)
{
	/* line in is not supported, so do nothing */
	return 0;
}

static int wm8711_set_register(unsigned int data)
{
	unsigned int reg = data >> 9;
	unsigned int value = (0x1ff & data);

	WM8711_WRREG(reg, value);
	return 0;
}

static void wm8711_powerdown(void)
{
	spi_write_register(15, 0);
}

void wm8711_init(struct codec_ops *ops)
{
	ops->set_volume = wm8711_set_volume;
	ops->set_volume_internal = wm8711_set_volume;
	ops->set_volume_external = wm8711_set_volume;
	ops->set_register = wm8711_set_register;
	ops->resume = wm8711_init_registers;
	ops->suspend = wm8711_powerdown;
	ops->set_treble = wm8711_set_treble;
	ops->set_bass = wm8711_set_bass;
	ops->set_fs = wm8771_set_fs;
	ops->set_volume_external_left = wm8711_set_volume;
	ops->set_volume_external_right = wm8711_set_volume;
	ops->switch_line_in = wm8711_switch_line_in;
	ops->set_volume_dB = wm8711_set_volume_dB;

	wm8711_init_registers();
}

/* EOF */
