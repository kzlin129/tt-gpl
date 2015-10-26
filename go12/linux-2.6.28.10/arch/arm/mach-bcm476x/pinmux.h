#ifndef __ARCH_ARM_MACH_BCM4760_PINMUX_H__
#define __ARCH_ARM_MACH_BCM4760_PINMUX_H__
#define CMU_F_GPIO_28_MXSEL_R	0
#define CMU_F_GPIO_29_MXSEL_R	3
#define CMU_F_GPIO_30_MXSEL_R	6
#define CMU_F_GPIO_31_MXSEL_R	9
#define CMU_F_GPIO_32_MXSEL_R	12
#define CMU_F_GPIO_33_MXSEL_R	15
#define CMU_F_GPIO_34_MXSEL_R	18
#define CMU_F_GPIO_35_MXSEL_R	21
#define CMU_F_GPIO_36_MXSEL_R	24
#define CMU_F_GPIO_37_MXSEL_R	27
#define CMU_F_GPIO_38_MXSEL_R	0
#define CMU_F_GPIO_39_MXSEL_R	3
#define CMU_F_GPIO_40_MXSEL_R	6
#define CMU_F_GPIO_41_MXSEL_R	9
#define CMU_F_GPIO_42_MXSEL_R	12
#define CMU_F_GPIO_43_MXSEL_R	15
#define CMU_F_GPIO_44_MXSEL_R	18
#define CMU_F_GPIO_45_MXSEL_R	21
#define CMU_F_GPIO_46_MXSEL_R	0
#define CMU_F_GPIO_47_MXSEL_R	3
#define CMU_F_GPIO_48_MXSEL_R	6
#define CMU_F_GPIO_49_MXSEL_R	9
#define CMU_F_GPIO_50_MXSEL_R	12
#define CMU_F_GPIO_51_MXSEL_R	15
#define CMU_F_GPIO_52_MXSEL_R	18
#define CMU_F_GPIO_53_MXSEL_R	21
#define CMU_F_GPIO_54_MXSEL_R	0
#define CMU_F_GPIO_55_MXSEL_R	3
#define CMU_F_GPIO_56_MXSEL_R	6
#define CMU_F_GPIO_57_MXSEL_R	9
#define CMU_F_GPIO_58_MXSEL_R	12
#define CMU_F_GPIO_59_MXSEL_R	15
#define CMU_F_GPIO_60_MXSEL_R	18
#define CMU_F_GPIO_61_MXSEL_R	21
#define CMU_F_GPIO_62_MXSEL_R	0
#define CMU_F_GPIO_63_MXSEL_R	3
#define CMU_F_GPIO_64_MXSEL_R	6
#define CMU_F_GPIO_65_MXSEL_R	24
#define CMU_F_GPIO_66_MXSEL_R	18
#define CMU_F_GPIO_67_MXSEL_R	21
#define CMU_F_GPIO_68_MXSEL_R	24
#define CMU_F_GPIO_69_MXSEL_R	27
#define CMU_F_GPIO_70_MXSEL_R	24
#define CMU_F_GPIO_71_MXSEL_R	27
#define CMU_F_GPIO_72_MXSEL_R	18
#define CMU_F_GPIO_73_MXSEL_R	21
#define CMU_F_GPIO_74_MXSEL_R	18
#define CMU_F_GPIO_75_MXSEL_R	21
#define CMU_F_GPIO_76_MXSEL_R	0
#define CMU_F_GPIO_77_MXSEL_R	3
#define CMU_F_GPIO_78_MXSEL_R	6
#define CMU_F_GPIO_79_MXSEL_R	9
#define CMU_F_GPIO_80_MXSEL_R	12
#define CMU_F_GPIO_81_MXSEL_R	15
#define CMU_F_GPIO_82_MXSEL_R	0
#define CMU_F_GPIO_83_MXSEL_R	3
#define CMU_F_GPIO_84_MXSEL_R	6
#define CMU_F_GPIO_85_MXSEL_R	9
#define CMU_F_GPIO_86_MXSEL_R	12
#define CMU_F_GPIO_87_MXSEL_R	15
#define CMU_F_GPIO_88_MXSEL_R	24
#define CMU_F_GPIO_89_MXSEL_R	27
#define CMU_F_GPIO_90_MXSEL_R	0
#define CMU_F_GPIO_91_MXSEL_R	3
#define CMU_F_GPIO_92_MXSEL_R	6
#define CMU_F_GPIO_93_MXSEL_R	9
#define CMU_F_GPIO_94_MXSEL_R	12

struct bcm4760_pin_mux
{
	uint32_t	address;			/* Address the register is located at. */
	int		bitpos;				/* Bit position of bit 0 in the register. */
	uint32_t	mask;				/* Mask for all values. */
	int		func_val[4];			/* Uses the bcm4760_pin_mux_values enum as index. */
};
#endif
