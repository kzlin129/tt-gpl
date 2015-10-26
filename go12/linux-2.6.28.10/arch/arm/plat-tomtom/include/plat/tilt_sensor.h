/* Platform data for Tilt sensor*/

#define TILT_SENSOR_MAX_IRQ	2

struct tilt_sensor_platform_data {

	int (*tilt_power_set) (int);
	int (*tilt_power_get) (void);
	int (*tilt_value) (void);
	int (*config_gpio) (void);
	unsigned int (*tilt_raw) (void);
	
	int irq[TILT_SENSOR_MAX_IRQ];
	int irq_max;
};
