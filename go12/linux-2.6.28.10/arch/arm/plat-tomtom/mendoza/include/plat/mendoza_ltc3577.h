#ifndef _MENDOZA_LTC3577_H
#define _MENDOZA_LTC3577_H

extern struct platform_device mendoza_ltc3577_bl;
extern struct platform_device mendoza_ltc3577_pmic;

#ifndef CONFIG_MFD_LTC3577
#define mendoza_ltc3577_i2c_init() do {} while(0)
#else
int mendoza_ltc3577_i2c_init(void);
#endif

#endif

