#ifndef __LINUX_PWM_PDATA_H
#define __LINUX_PWM_PDATA_H


#include <linux/device.h>

struct pwm_pdata {
	int (*pwm_config_gpio) (int id);
};

#endif /* __LINUX_PWM_PDATA_H */
