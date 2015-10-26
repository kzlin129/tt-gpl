#ifndef __REGULATOR_STD_H__
#define __REGULATOR_STD_H__

#include <linux/regulator/bcm59040-regulators.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/userspace-consumer.h>
#include <asm/arch/bcm59040.h>

#define BOOT_ON		1
#define BOOT_OFF	0
#define SUSPEND_ON	1
#define SUSPEND_OFF	0

#define FOR_ALL_REGULATORS(m,d) \
        d(m,LDO1,0), d(m,LDO2,1), d(m,LDO3,2), d(m,LDO4,3), \
        d(m,LDO5,4), d(m,LDO6,5), d(m,CSR ,6), d(m,IOSR,7),

#define IRVINE_CONSUMER_SUPPLY(m,r,i) { .supply = #r, }

#define IRVINE_CONSUMER_DATA(m,r,i) \
{ \
        .name           = #r, \
        .num_supplies   = 1, \
        .supplies       = &_##m##_consumer_supplies[i], \
}

#define _IRVINE_USERSPACE_CONSUMER(m,r,i) \
{ \
        .name           = "reg-us-consumer", \
        .id             = i, \
        .dev            = { \
                .platform_data = &_##m##_consumer_data[i], \
        } \
}

#define IRVINE_USERSPACE_CONSUMER_SUPPLY(m, r,i) \
{ \
        .dev    = &_##m##_userspace_consumers[i].dev, \
        .supply = #r, \
}

#define IRVINE_USERSPACE_CONSUMER_SUPPLY_1(m,r,i) \
static struct regulator_consumer_supply _## m ##_ ## r ## _consumers[] = { \
        IRVINE_USERSPACE_CONSUMER_SUPPLY(m,r,i), \
}

#define IRVINE_USERSPACE_CONSUMER_SUPPLY_NULL(m,r,i) \
static struct regulator_consumer_supply _## m ##_ ## r ## _consumers[] = { \
        { \
                .dev = NULL,  \
                .supply = #r, \
        }, \
	IRVINE_USERSPACE_CONSUMER_SUPPLY(m,r,i), \
}

#define IRVINE_REGULATOR(m,r,uv,suspend,boot) \
static struct regulator_init_data _## m ## _ ## r ## _data = { \
	.constraints = { \
		.name = #r,\
		.min_uV = uv, \
		.max_uV = uv, \
		.valid_modes_mask = REGULATOR_MODE_NORMAL, \
		.valid_ops_mask = REGULATOR_CHANGE_STATUS, \
		.state_disk = { \
			.uV = suspend ? uv : 0, \
			.enabled = suspend, \
		}, \
		.state_mem = { \
			.uV = suspend ? uv : 0, \
			.enabled = suspend, \
		}, \
		.state_standby = { \
			.uV = suspend ? uv : 0, \
			.enabled = suspend, \
		}, \
		.initial_state = PM_SUSPEND_MEM, \
		.boot_on = boot, \
	}, \
	.num_consumer_supplies = ARRAY_SIZE(_##m##_##r##_consumers), \
	.consumer_supplies = _##m##_##r##_consumers, \
}

#define IRVINE_REGULATOR_DVS(m,r,maxuv,minuv,suspend,boot) \
static struct regulator_init_data _## m ## _ ## r ## _data = { \
	.constraints = { \
		.name = #r,\
		.min_uV = maxuv, \
		.max_uV = minuv, \
		.valid_modes_mask = REGULATOR_MODE_NORMAL, \
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS, \
		.state_disk = { \
			.uV = suspend, \
			.enabled = suspend ? 1 : 0, \
		}, \
		.state_mem = { \
			.uV = suspend, \
			.enabled = suspend ? 1 : 0, \
		}, \
		.state_standby = { \
			.uV = suspend, \
			.enabled = suspend ? 1 : 0, \
		}, \
		.initial_state = PM_SUSPEND_MEM, \
		.boot_on = boot, \
	}, \
	.num_consumer_supplies = ARRAY_SIZE(_##m##_##r##_consumers), \
	.consumer_supplies = _##m##_##r##_consumers, \
}
#endif /* __REGULATOR_STD_H__ */

