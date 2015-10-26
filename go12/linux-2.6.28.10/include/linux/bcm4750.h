#ifndef __BCM4750_H__
#define __BCM4650_H__

struct bcm4750_platform_data 
{
	void (*suspend) (void);
	void (*resume)  (void);
};

#endif
