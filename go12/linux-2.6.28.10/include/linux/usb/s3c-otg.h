#ifndef __S3C_OTH_H
#define __S3C_OTH_H

struct s3c_otg_platform_data {
	void	(*otg_phy_init)(void);
	void	(*otg_phy_off)(void);
};

#endif
