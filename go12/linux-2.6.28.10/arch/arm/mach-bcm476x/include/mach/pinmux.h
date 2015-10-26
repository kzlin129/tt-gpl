#ifndef __INCLUDE_MACH_PINMUX_H__
extern int bcm4760_set_pin_mux( unsigned pin, int setting );
extern int bcm4760_get_pin_mux( unsigned pin );

enum bcm4760_pin_mux_values
{
        BCM4760_PIN_MUX_GPIO,
        BCM4760_PIN_MUX_SF1,
        BCM4760_PIN_MUX_SF2,
        BCM4760_PIN_MUX_SF3,
};
#endif
