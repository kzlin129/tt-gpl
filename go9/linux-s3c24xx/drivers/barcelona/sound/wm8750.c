#include <linux/kernel.h>
#include <linux/delay.h>
#include <barcelona/debug.h>
#include <barcelona/gopins.h>

#include "spi.h"
#include "wm8750.h"

#define PFX "wm8750: "
#define PK_DBG PK_DBG_FUNC

#define R8_FS256 0x20
#define R8_FS384 0x22

#define NO_INIT 0xffff

#if 1
/* Keep all the register values in an array.
 * If a value equals NO_INIT don't ever set it...
 */
static uint16_t wm8750[43] = {
	0x80,		/* R0 Left input vol, don't touch... */
	0x80,		/* R1 Right input vol, unmute, zerocross + VOL (0 - 63). Both needs to be updated. */
	0x79,		/* R2 LOUT1 vol, update R3, zerocross + VOL (0 - 127). */
	0x79,		/* R3 ROUT1 vol, don't touch, updated by R2. */
	NO_INIT,	/* R4 Reserved, don't touch. */
	0x08,		/* R5 ADC and DAC Control, XXX - Needs to be zero to unmute... how?? */
	NO_INIT,	/* R6 Reserved, don't touch. */
	0x2,		/* R7 Audio Interface, IIS Mode. */
	R8_FS256,	/* R8 Sample rate */
	NO_INIT,	/* R9 Reserved, don't touch. */
	0xff,		/* R10 LDAC vol, don't touch. */
	0xff,		/* R11 RDAC vol, don't touch. */
	0xf,		/* R12 Bass control, don't touch for now. XXX - Add for acoustics. */
	0xf,		/* R13 Treble control, don't touch for now. XXX - Add for acoustics. */
	NO_INIT,	/* R14 Don't mentioned in the spec, don't touch. */
	NO_INIT,	/* R15 Reset, only write zero to this reg. */
	NO_INIT,	/* R16 Reserved, don't touch. */
	0x7b,		/* R17 ALC control... set to 0. XXX - need that later for record*/
	0x0,		/* R18 AlC control... set to 0. XXX - ... */
	0x32,		/* R19 ALC control... set to 0. XXX - ... */
	0x0,		/* R20 Noise gate, XXX ... */
	0x1ff,		/* R21 Left ADC control, don't touch... not used. */
	0x1ff,		/* R22 Right ADC control, keep default value. */
	0xc0,		/* R23 Additional control 1, XXX - unused for now. Could enable mono mixing for power 			saving and some other things.... should be researched. */
	0x10,		/* R24 Additional control 2, don't touch. */
	0xfe,		/* R25 Power mgmt 1, 50kOhm enabled, VREF, right PGA and ADC, MICBIAS. */
	0x1e4,		/* R26 Power mgmt 2, left DAC and OUT1, right DAC and OUT2, MONO out. */
	0x0,		/* R27 Additional control 3, don't touch. */
	NO_INIT,	/* R28 Don't mentioned. */
	NO_INIT,	/* R29 Don't mentioned. */
	NO_INIT,	/* R30 Don't mentioned. */
	0x0,		/* R31 ADC input mode, Digital MONO mix. */
	0x0,		/* R32 Left ADC signal path, don't touch. */
	0x0,		/* R33 Right ADC signal path, MIC and some boost. */
	0x150,		/* R34 Left out mix 1, Left DAC. */
	0x50,		/* R35 Left out mix 2, don't touch. */
	0x50,		/* R36 Right out mix 1, don't touch. */
	0x150,		/* R37 Right out mix 2, Right DAC. */
	0x150,		/* R38 Mono out mix, Left Dac. */
	0x150,		/* R39 Mono out mix, Right Dac. */
	0x79,		/* R40 LOUT2 volume, unsued, don't touch. */
	0x79,		/* R41 ROUT2 volume, unused, don't touch. */
	0x80,		/* R42 MONO volume, zero cross + vol (0 - 127). */
};

static uint16_t wm8750_current[43] = {
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
};
#else
/* Keep all the register values in an array.
 * If a value is zero don't ever set it...
 */
static uint16_t wm8750[43] = {
	0,				/* R0 Left input vol, don't touch... */
	0x100 | 64,		/* R1 Right input vol, unmute, zerocross + VOL (0 - 63). Both needs to be updated. */
	0x00  | 121,	/* R2 LOUT1 vol, update R3, zerocross + VOL (0 - 127). */
	0x100 | 121,	/* R3 ROUT1 vol, don't touch, updated by R2. */
	0,				/* R4 Reserved, don't touch. */
	0,				/* R5 ADC and DAC Control, XXX - Needs to be zero to unmute... how?? */
	0,				/* R6 Reserved, don't touch. */
	0x2,			/* R7 Audio Interface, IIS Mode. */
	R8_FS256,		/* R8 Sample rate */
	0,				/* R9 Reserved, don't touch. */
	0,				/* R10 LDAC vol, don't touch. */
	0,				/* R11 RDAC vol, don't touch. */
	0xc0,			/* R12 Bass control, don't touch for now. XXX - Add for acoustics. */
	0xf,			/* R13 Treble control, don't touch for now. XXX - Add for acoustics. */
	0,				/* R14 Don't mentioned in the spec, don't touch. */
	0,				/* R15 Reset, only write zero to this reg. */
	0,				/* R16 Reserved, don't touch. */
	0,				/* R17 ALC control... set to 0. XXX - need that later for record*/
	0,				/* R18 AlC control... set to 0. XXX - ... */
	0,				/* R19 ALC control... set to 0. XXX - ... */
	0,				/* R20 Noise gate, XXX ... */
	0x1ff,			/* R21 Left ADC control, don't touch... not used. */
	0x1ff,			/* R22 Right ADC control, keep default value. */
	0,				/* R23 Additional control 1, XXX - unused for now. Could enable mono mixing for power 			saving and some other things.... should be researched. */
	0,				/* R24 Additional control 2, don't touch. */
	0xd6,			/* R25 Power mgmt 1, 50kOhm enabled, VREF, right PGA and ADC, MICBIAS. */
	0x1e4,			/* R26 Power mgmt 2, left DAC and OUT1, right DAC and OUT2, MONO out. */
	0,				/* R27 Additional control 3, don't touch. */
	0,				/* R28 Don't mentioned. */
	0,				/* R29 Don't mentioned. */
	0,				/* R30 Don't mentioned. */
	0x11,			/* R31 ADC input mode, Digital MONO mix. */
	0x0,			/* R32 Left ADC signal path, don't touch. */
	0x70,			/* R33 Right ADC signal path, MIC and some boost. */
	0x000,			/* R34 Left out mix 1, Left DAC. */
	0x100,			/* R35 Left out mix 2, don't touch. */
	0x100,			/* R36 Right out mix 1, don't touch. */
	0x000,			/* R37 Right out mix 2, Right DAC. */
	0x100,			/* R38 Mono out mix, Left Dac. */
	0x100,			/* R39 Mono out mix, Right Dac. */
	0x0,			/* R40 LOUT2 volume, unsued, don't touch. */
	0x0,			/* R41 ROUT2 volume, unused, don't touch. */
	0x80 | 127		/* R42 MONO volume, zero cross + vol (0 - 127). */
};
#endif

static inline void WM8750_WRREG(uint16_t reg, uint16_t value)
{
	wm8750_current[reg] = value;
	PK_DBG("R%d=0x%x\n", reg, value);
	spi_write_register(reg, value);
}

static inline uint16_t WM8750_RDREG(uint16_t reg)
{
	return wm8750_current[reg];
}

static void wm8750_init_registers(void)
{
	int i;
	uint16_t r;

	WM8750_WRREG(15, 0);
	for (i = 0; i < ARRAY_SIZE(wm8750); i++) {
		if (i != 15) {
			if ((r = wm8750[i]) != NO_INIT)
			{
				if ((i == 26) && (IO_GetCodecAmpType() != GOCODECAMP_SINGLEENDED))
				{
					/* enable OUT2 */
					r |= 0X18;
				}
				WM8750_WRREG(i, r);
			}
		}
	}

	/* Unmute. */
	WM8750_WRREG(5, 0);
}

static int wm8750_set_volume_dB(unsigned int volume)
{
	if (volume < 90) 
		volume = 121 - volume;
	else
		volume = 0; /* mute */

	WM8750_WRREG(40, 0x180 | volume);
	WM8750_WRREG(41, 0x180 | volume);
	WM8750_WRREG(42, 0x80 | volume);

	return 0;
}

static int wm8750_set_volume_internal(unsigned int volume)
{
	if (volume > 127)
		volume = 127;

	WM8750_WRREG(40, 0x180 | volume);
	WM8750_WRREG(41, 0x180 | volume);
	WM8750_WRREG(42, 0x80 | volume);

	return 0;
}

static int wm8750_set_volume_external(unsigned int volume)
{
	if (volume > 127)
		volume = 127;

	WM8750_WRREG(2, 0x180 | volume);
	WM8750_WRREG(3, 0x180 | volume);

	return 0;
}

static int wm8750_set_volume_external_left(unsigned int volume)
{
	if (volume > 127)
		volume = 127;

	WM8750_WRREG(2, 0x180 | volume);

	return 0;
}

static int wm8750_set_volume_external_right(unsigned int volume)
{
	if (volume > 127)
		volume = 127;

	WM8750_WRREG(3, 0x180 | volume);

	return 0;
}

static int wm8750_set_volume(unsigned int volume)
{
	wm8750_set_volume_internal(volume);
	wm8750_set_volume_external(volume);

	return 0;
}

static int wm8750_set_treble(unsigned int treble)
{
	WM8750_WRREG(13, treble);

	return 0;
}

static int wm8750_set_bass(unsigned int bass)
{
	WM8750_WRREG(12, bass ^ 0x80);

	return 0;
}

static int wm8750_set_fixed12(unsigned int samplerate, uint16_t *newr8) 
{
	/* fixed 12 MHz clock from UPLL or XTAL */
	switch (samplerate) {
		case 8000:
			*newr8 = 0x0d;
			break;
		case 11025:
			*newr8 = 0x33;
			break;
		case 12000:
			*newr8 = 0x11;
			break;
		case 16000:
			*newr8 = 0x15;
			break;
		case 22050:
			*newr8 = 0x37;
			break;
		case 24000:
			*newr8 = 0x39;
			break;
		case 32000:
			*newr8 = 0x19;
			break;
		case 44100:
			*newr8 = 0x23;
			break;
		case 48000:
			*newr8 = 0x01;
			break;
		case 96000:
			*newr8 = 0x1b;
			break;
		default:
			return -1;
	}

	return 0;
}

static int wm8750_set_switch1211(unsigned int samplerate, uint16_t *newr8) 
{
	/* switch between 12.228 and 11.2896 UPLL clock */
	switch (samplerate) {
		case 8000:
			*newr8 = 0x0c;
			break;
		case 11025:
			*newr8 = 0x30;
			break;
		case 12000:
			*newr8 = 0x10;
			break;
		case 16000:
			*newr8 = 0x14;
			break;
		case 22050:
			*newr8 = 0x34;
			break;
		case 24000:
			*newr8 = 0x38;
			break;
		case 32000:
			*newr8 = 0x18;
			break;
		case 44100:
			*newr8 = 0x20;
			break;
		case 48000:
			*newr8 = 0x00;
			break;
		case 96000:
			*newr8 = 0x1a;
			break;
		default:
			return -1;
	}

	return 0;
}

static int wm8750_set_fs(enum codec_fs fs,unsigned int samplerate, unsigned int clockrate)
{
	int ret;
	uint16_t newr8;

	if (IO_GetCodecMaster() == GOCODECCFG_INTERNAL_MASTER ) {
		/* Set codec in master mode */
		WM8750_WRREG(7, 0x42);

		/* Codec is master with fixed 6 MHz clock, set sample frequency according to double the rate as defined in the datasheet */
		switch (samplerate) {
		case 8000:
			newr8 = 0x15;
			break;
		case 11025:
			newr8 = 0x37;
			break;
		case 12000:
			newr8 = 0x39;
			break;
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
			newr8 = 0x1b;
			break;
		default:
			return -1;
		}
	} else if (IO_GetCodecMaster() == GOCODECCFG_EXTERNAL_MASTER ) {
		/* Set codec in master mode */
		if((IO_GetCpuType() == GOCPU_S3C2443 ) ||
		   (IO_GetCpuType() == GOCPU_S3C2450 ) )
                {
#ifdef SIX_MHZ_AUDIO_TEST
			WM8750_WRREG(7, 0x42);
			samplerate *= 2;
#else
	
			WM8750_WRREG(7, 0xc2);
#endif	/* SIX_MHZ_AUDIO_TEST	*/
		} else {
			WM8750_WRREG(7, 0x42);
		}

		if (IO_HaveUsbHost()) {
			if ((ret = wm8750_set_fixed12(samplerate, &newr8)) == -1) return ret;
		}
		else {
			switch (IO_GetGpsType()) {
				/* GL type chips */
				case GOGPS_GL:
				case GOGPS_GL_INT_LNA:
				case GOGPS_GL_BCM4750:

					if ((ret = wm8750_set_switch1211(samplerate, &newr8)) == -1) return ret;
					break;

				/* SiRF chips */
				case GOGPS_SIRF1:
				case GOGPS_SIRF2:
				case GOGPS_SIRF3:

					if ((ret = wm8750_set_fixed12(samplerate, &newr8)) == -1) return ret;
					break;

				default:
					printk("%s: ERROR: GPS chip type needs to be set!\n", __FUNCTION__);
					return -1;
			}
		}

#ifdef SIX_MHZ_AUDIO_TEST
		if((IO_GetCpuType() == GOCPU_S3C2443) ||
		   (IO_GetCpuType() == GOCPU_S3C2450) ) 
                {
			newr8 |= (1<<6); /* divide Master clock by two!	*/
		}
#endif /* SIX_MHZ_AUDIO_TEST	*/

	} else { 	/* IO_GetCodecMaster()) == GOCODECCFG_SLAVE */
		/* Set codec in slave mode */
		WM8750_WRREG(7, 0x02);

		/* Codec is slave, so only set oversample frequency */
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

	WM8750_WRREG(8, newr8);

	return 0;
}

static int wm8750_switch_line_in(unsigned int line_in)
{
	unsigned int value ;

	line_in = ((line_in & 1) << 6);
	value = WM8750_RDREG(32);
	value = (value & ~(0xc0)) | line_in;
	WM8750_WRREG(32, value);
	value = WM8750_RDREG(33);
	value = (value & ~(0xc0)) | line_in;
	WM8750_WRREG(33, value);

	return 0;
}

static int wm8750_set_register(unsigned int data)
{
	unsigned int reg = data >> 9;
	unsigned int value = (0x1ff & data);

	PK_DBG("SetRegister: %u => 0x%03x\n", reg, value);
	WM8750_WRREG(reg, value);
	return 0;
}

static void wm8750_powerdown(void)
{
	spi_write_register(5, 8);
	mdelay(50);
	spi_write_register(25, 0xc0);
	spi_write_register(26, 0x184);
	mdelay(50);
	spi_write_register(5, 8);
	mdelay(50);
	spi_write_register(25, 0);
	spi_write_register(26, 0);
	mdelay(50);
}

void wm8750_init(struct codec_ops *ops)
{
	ops->set_volume = wm8750_set_volume;
	ops->set_volume_internal = wm8750_set_volume_internal;
	ops->set_volume_external = wm8750_set_volume_external;
	ops->set_register = wm8750_set_register;
	ops->resume = wm8750_init_registers;
	ops->suspend = wm8750_powerdown;
	ops->set_treble = wm8750_set_treble;
	ops->set_bass = wm8750_set_bass;
	ops->set_fs = wm8750_set_fs;
	ops->set_volume_external_left = wm8750_set_volume_external_left;
	ops->set_volume_external_right = wm8750_set_volume_external_right;
	ops->switch_line_in = wm8750_switch_line_in;
	ops->set_volume_dB = wm8750_set_volume_dB;

	wm8750_init_registers();
}

/* EOF */

