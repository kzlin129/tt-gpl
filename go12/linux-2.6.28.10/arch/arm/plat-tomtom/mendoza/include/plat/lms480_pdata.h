#include <linux/cordoba_lms480wv.h>
#if 0
#include <mach/cordoba.h>

#define LCM_SPI_CLK(x)		(S5P64XX_GPC(1 + (x * 4)))
#define LCM_SPI_MOSI(x)		(S5P64XX_GPC(2 + (x * 4)))
#define LCM_SPI_CH     		0   											/* spi channel for module init */
#endif

struct lms480wv_platform_data * setup_lms480wv_pdata(void);
