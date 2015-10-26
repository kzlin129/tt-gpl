#ifndef __LMS480WV_H__
#define __LMS480WV_H__

struct lms480wv_platform_data 
{
	void (*suspend) (void);
	void (*resume)  (void);
	void (*reset_lcm) (void);
	void (*enable_lcm)  (void);
	void (*config_gpio) (void);
	int (*request_gpio)  (void);
	void (*free_gpio) (void);
	int (*simple_backlight_on) (void);
};

#endif
