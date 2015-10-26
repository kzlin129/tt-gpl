#include <mach/hardware.h>
#include <asm/arch/bcm4760_reg.h>
#include <mach/pinmux.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include "pinmux.h"

static const struct bcm4760_pin_mux	bcm4760_pin_mux_array[]=	/* Index into this array is the GPIO number. */
{
	/* GPIO 0 - 7 */
	{CMU_R_CHIP_PIN_MUX11_MEMADDR, CMU_F_GPIO_00_MXSEL_R, 0x00000007, {0,	-1,	-1,	-1}},
	{CMU_R_CHIP_PIN_MUX11_MEMADDR, CMU_F_GPIO_01_MXSEL_R, 0x00000007, {0,	-1,	-1,	-1}},
	{CMU_R_CHIP_PIN_MUX11_MEMADDR, CMU_F_GPIO_02_MXSEL_R, 0x00000007, {0,	 1,	-1,	-1}},
	{CMU_R_CHIP_PIN_MUX11_MEMADDR, CMU_F_GPIO_03_MXSEL_R, 0x00000007, {0,	 1,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX11_MEMADDR, CMU_F_GPIO_04_MXSEL_R, 0x00000007, {0,	 1,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX11_MEMADDR, CMU_F_GPIO_05_MXSEL_R, 0x00000007, {0,	 1,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX11_MEMADDR, CMU_F_GPIO_06_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX11_MEMADDR, CMU_F_GPIO_07_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},

	/* GPIO 8 - 17 */
	{CMU_R_CHIP_PIN_MUX5_MEMADDR, CMU_F_GPIO_08_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX5_MEMADDR, CMU_F_GPIO_09_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX5_MEMADDR, CMU_F_GPIO_10_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX5_MEMADDR, CMU_F_GPIO_11_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX5_MEMADDR, CMU_F_GPIO_12_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX5_MEMADDR, CMU_F_GPIO_13_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX5_MEMADDR, CMU_F_GPIO_14_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX5_MEMADDR, CMU_F_GPIO_15_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX5_MEMADDR, CMU_F_GPIO_16_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX5_MEMADDR, CMU_F_GPIO_17_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},

	/* GPIO 18 - 27 */
	{CMU_R_CHIP_PIN_MUX6_MEMADDR, CMU_F_GPIO_18_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX6_MEMADDR, CMU_F_GPIO_19_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX6_MEMADDR, CMU_F_GPIO_20_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX6_MEMADDR, CMU_F_GPIO_21_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX6_MEMADDR, CMU_F_GPIO_22_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX6_MEMADDR, CMU_F_GPIO_23_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX6_MEMADDR, CMU_F_GPIO_24_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX6_MEMADDR, CMU_F_GPIO_25_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX6_MEMADDR, CMU_F_GPIO_26_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},
	{CMU_R_CHIP_PIN_MUX6_MEMADDR, CMU_F_GPIO_27_MXSEL_R, 0x00000007, {0,	 1,	 2,	 3}},

	/* GPIO 28 - 37 */
	{CMU_R_CHIP_PIN_MUX4_MEMADDR, CMU_F_GPIO_28_MXSEL_R, 0x00000007, {3,	 0,	-1,	-1}},
	{CMU_R_CHIP_PIN_MUX4_MEMADDR, CMU_F_GPIO_29_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX4_MEMADDR, CMU_F_GPIO_30_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX4_MEMADDR, CMU_F_GPIO_31_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX4_MEMADDR, CMU_F_GPIO_32_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX4_MEMADDR, CMU_F_GPIO_33_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX4_MEMADDR, CMU_F_GPIO_34_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX4_MEMADDR, CMU_F_GPIO_35_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX4_MEMADDR, CMU_F_GPIO_36_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX4_MEMADDR, CMU_F_GPIO_37_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},

	/* GPIO 38 - 45 */
	{CMU_R_CHIP_PIN_MUX2_MEMADDR, CMU_F_GPIO_38_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX2_MEMADDR, CMU_F_GPIO_39_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX2_MEMADDR, CMU_F_GPIO_40_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX2_MEMADDR, CMU_F_GPIO_41_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX2_MEMADDR, CMU_F_GPIO_42_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX2_MEMADDR, CMU_F_GPIO_43_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX2_MEMADDR, CMU_F_GPIO_44_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX2_MEMADDR, CMU_F_GPIO_45_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},

	/* GPIO 46 - 53 */
	{CMU_R_CHIP_PIN_MUX1_MEMADDR, CMU_F_GPIO_46_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX1_MEMADDR, CMU_F_GPIO_47_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX1_MEMADDR, CMU_F_GPIO_48_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX1_MEMADDR, CMU_F_GPIO_49_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX1_MEMADDR, CMU_F_GPIO_50_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX1_MEMADDR, CMU_F_GPIO_51_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX1_MEMADDR, CMU_F_GPIO_52_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX1_MEMADDR, CMU_F_GPIO_53_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},

	/* GPIO 54 - 61 */
	{CMU_R_CHIP_PIN_MUX0_MEMADDR, CMU_F_GPIO_54_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX0_MEMADDR, CMU_F_GPIO_55_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX0_MEMADDR, CMU_F_GPIO_56_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX0_MEMADDR, CMU_F_GPIO_57_MXSEL_R, 0x00000007, {3,	 0,	 1,	-1}},
	{CMU_R_CHIP_PIN_MUX0_MEMADDR, CMU_F_GPIO_58_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX0_MEMADDR, CMU_F_GPIO_59_MXSEL_R, 0x00000007, {3,	 0,	 1,	-1}},
	{CMU_R_CHIP_PIN_MUX0_MEMADDR, CMU_F_GPIO_60_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX0_MEMADDR, CMU_F_GPIO_61_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},

	/* GPIO 62 - 64 */
	{CMU_R_CHIP_PIN_MUX3_MEMADDR, CMU_F_GPIO_62_MXSEL_R, 0x00000007, {3,	 0,	 2,	-1}},
	{CMU_R_CHIP_PIN_MUX3_MEMADDR, CMU_F_GPIO_63_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX3_MEMADDR, CMU_F_GPIO_64_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},

	/* GPIO 65 */
	{CMU_R_CHIP_PIN_MUX2_MEMADDR, CMU_F_GPIO_65_MXSEL_R, 0x00000007, {3,	 0,	 1,	-1}},

	/* GPIO 66 - 69 */
	{CMU_R_CHIP_PIN_MUX7_MEMADDR, CMU_F_GPIO_66_MXSEL_R, 0x00000007, {3,	 0,	 1,	-1}},
	{CMU_R_CHIP_PIN_MUX7_MEMADDR, CMU_F_GPIO_67_MXSEL_R, 0x00000007, {3,	 0,	 1,	-1}},
	{CMU_R_CHIP_PIN_MUX7_MEMADDR, CMU_F_GPIO_68_MXSEL_R, 0x00000007, {3,	 0,	 1,	-1}},
	{CMU_R_CHIP_PIN_MUX7_MEMADDR, CMU_F_GPIO_69_MXSEL_R, 0x00000007, {3,	 0,	 1,	-1}},

	/* GPIO 70 - 73 */
	{CMU_R_CHIP_PIN_MUX8_MEMADDR, CMU_F_GPIO_70_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX8_MEMADDR, CMU_F_GPIO_71_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX8_MEMADDR, CMU_F_GPIO_72_MXSEL_R, 0x00000007, {3,	 0,	 1,	-1}},
	{CMU_R_CHIP_PIN_MUX8_MEMADDR, CMU_F_GPIO_73_MXSEL_R, 0x00000007, {3,	 0,	 1,	-1}},

	/* GPIO 74 - 75 */
	{CMU_R_CHIP_PIN_MUX9_MEMADDR, CMU_F_GPIO_74_MXSEL_R, 0x00000007, {3,	 0,	 1,	-1}},
	{CMU_R_CHIP_PIN_MUX9_MEMADDR, CMU_F_GPIO_75_MXSEL_R, 0x00000007, {3,	 0,	 1,	-1}},

	/* GPIO 76 - 81 */
	{CMU_R_CHIP_PIN_MUX8_MEMADDR, CMU_F_GPIO_76_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX8_MEMADDR, CMU_F_GPIO_77_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX8_MEMADDR, CMU_F_GPIO_78_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX8_MEMADDR, CMU_F_GPIO_79_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX8_MEMADDR, CMU_F_GPIO_80_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX8_MEMADDR, CMU_F_GPIO_81_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},

	/* GPIO 82 - 89 */
	{CMU_R_CHIP_PIN_MUX9_MEMADDR, CMU_F_GPIO_82_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX9_MEMADDR, CMU_F_GPIO_83_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX9_MEMADDR, CMU_F_GPIO_84_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX9_MEMADDR, CMU_F_GPIO_85_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX9_MEMADDR, CMU_F_GPIO_86_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX9_MEMADDR, CMU_F_GPIO_87_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX9_MEMADDR, CMU_F_GPIO_88_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},
	{CMU_R_CHIP_PIN_MUX9_MEMADDR, CMU_F_GPIO_89_MXSEL_R, 0x00000007, {3,	 0,	 1,	 2}},

	/* GPIO 90 -94 */
	{CMU_R_CHIP_PIN_MUX7_MEMADDR, CMU_F_GPIO_90_MXSEL_R, 0x00000007, {3,	 0,	-1,	-1}},
	{CMU_R_CHIP_PIN_MUX7_MEMADDR, CMU_F_GPIO_91_MXSEL_R, 0x00000007, {3,	 0,	-1,	-1}},
	{CMU_R_CHIP_PIN_MUX7_MEMADDR, CMU_F_GPIO_92_MXSEL_R, 0x00000007, {3,	 0,	-1,	-1}},
	{CMU_R_CHIP_PIN_MUX7_MEMADDR, CMU_F_GPIO_93_MXSEL_R, 0x00000007, {3,	 0,	-1,	-1}},
	{CMU_R_CHIP_PIN_MUX7_MEMADDR, CMU_F_GPIO_94_MXSEL_R, 0x00000007, {3,	 0,	-1,	-1}},

	/* Virtual GPIOs. This is for the entries for which no GPIO exists, but for which we do have Pinmux settings. */
	{CMU_R_CHIP_PIN_MUX11_MEMADDR, CMU_F_PMU_PC1_MXSEL_R, 0x00000007, {-1,	 0,	-1,	-1}},
	{CMU_R_CHIP_PIN_MUX11_MEMADDR, CMU_F_PMU_PC2_MXSEL_R, 0x00000007, {-1,	 0,	-1,	-1}},
};

/* Get the current setting of the pin mux. */
int bcm4760_get_pin_mux( unsigned pin )
{
	int			value;
	int			loop=0;
	unsigned long int	flags;

	/* Save interrupt flags. */
	local_irq_save( flags );

	/* Invalid entry. */
	if( pin >= ARRAY_SIZE( bcm4760_pin_mux_array ) )
	{
		local_irq_restore( flags );
		return -1;
	}

	/* Get the value. */
	value=((int) ((readl( IO_ADDRESS( bcm4760_pin_mux_array[pin].address ) ) >> bcm4760_pin_mux_array[pin].bitpos) & bcm4760_pin_mux_array[pin].mask));
	for( loop=0; loop < ARRAY_SIZE( bcm4760_pin_mux_array[pin].func_val ); loop++ )
	{
		if( bcm4760_pin_mux_array[pin].func_val[loop] == value )
		{
			local_irq_restore( flags );
			return loop;
		}
	}

	/* Restore irqflags. */
	local_irq_restore( flags );
	return -2;
}

/* Set the pin mux */
int bcm4760_set_pin_mux( unsigned pin, int setting )
{
	int			value;
	uint32_t		regval;
	unsigned long int	flags;

	/* Save interrupt flags. */
	local_irq_save( flags );

	/* Invalid entry. */
	if( pin >= ARRAY_SIZE( bcm4760_pin_mux_array ) )
	{
		local_irq_restore( flags );
		return -1;
	}

	/* Create correct value. */
	if( setting >= ARRAY_SIZE( bcm4760_pin_mux_array[pin].func_val ) )
	{
		local_irq_restore( flags );
		return -2;
	}

	value=bcm4760_pin_mux_array[pin].func_val[setting];
	if( value < 0 )
	{
		local_irq_restore( flags );
		return -3;
	}

	/* Write back to the register. */
	regval=readl( IO_ADDRESS( bcm4760_pin_mux_array[pin].address ) );
	regval&=~(bcm4760_pin_mux_array[pin].mask << bcm4760_pin_mux_array[pin].bitpos);
	regval|=((uint32_t) value) << bcm4760_pin_mux_array[pin].bitpos;
	writel( regval, IO_ADDRESS( bcm4760_pin_mux_array[pin].address ) );

	/* Restore IRQ context. */
	local_irq_restore( flags );
	return 0;
}

EXPORT_SYMBOL( bcm4760_set_pin_mux );
EXPORT_SYMBOL( bcm4760_get_pin_mux );
