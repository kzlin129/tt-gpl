/* linux/arch/arm/plat-s3c/include/plat/fb.h
 *
 * Platform header file for Samsung Display Controller (FIMD) driver
 *
 * Jinsung Yang, Copyright (c) 2009 Samsung Electronics
 * 	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _FB_H
#define _FB_H

#define FB_SWAP_WORD	(1 << 24)
#define FB_SWAP_HWORD	(1 << 16)
#define FB_SWAP_BYTE	(1 << 8)
#define FB_SWAP_BIT	(1 << 0)

struct platform_device;

/*
 * struct s3cfb_lcd_polarity
 * @rise_vclk:	if 1, video data is fetched at rising edge
 * @inv_hsync:	if HSYNC polarity is inversed
 * @inv_vsync:	if VSYNC polarity is inversed
 * @inv_vden:	if VDEN polarity is inversed
*/
struct s3cfb_lcd_polarity {
	int	rise_vclk;
	int	inv_hsync;
	int	inv_vsync;
	int	inv_vden;
};

/*
 * struct s3cfb_lcd_timing
 * @h_fp:	horizontal front porch
 * @h_bp:	horizontal back porch
 * @h_sw:	horizontal sync width
 * @v_fp:	vertical front porch
 * @v_fpe:	vertical front porch for even field
 * @v_bp:	vertical back porch
 * @v_bpe:	vertical back porch for even field
*/
struct s3cfb_lcd_timing {
	int	h_fp;
	int	h_bp;
	int	h_sw;
	int	v_fp;
	int	v_fpe;
	int	v_bp;
	int	v_bpe;
	int	v_sw;
};

/*
 * struct s3cfb_lcd
 * @width:		horizontal resolution
 * @height:		vertical resolution
 * @bpp:		bits per pixel
 * @freq:		vframe frequency
 * @timing:		timing values
 * @polarity:		polarity settings
 * @init_ldi:		pointer to LDI init function
 *
*/
struct s3cfb_lcd {
	int	width;
	int	height;
	int	bpp;
	int	freq;
	struct 	s3cfb_lcd_timing timing;
	struct 	s3cfb_lcd_polarity polarity;

	void 	(*init_ldi)(void);
};

struct s3c_platform_fb {
	int		hw_ver;
	const char	clk_name[16];
	int		nr_wins;
	int		nr_buffers[5];
	int		default_win;
	int		swap;

	struct s3cfb_lcd *lcd;

	void		(*cfg_gpio)(struct platform_device *dev);
	int		(*backlight_on)(struct platform_device *dev);
	int		(*reset_lcd)(struct platform_device *dev);
	int		(*suspend)(struct platform_device *dev);
	int		(*resume)(struct platform_device *dev);
};

extern void s3cfb_set_platdata(struct s3c_platform_fb *fimd);

/* defined by architecture to configure gpio */
extern void s3cfb_cfg_gpio(struct platform_device *pdev);
extern int s3cfb_backlight_on(struct platform_device *pdev);
extern int s3cfb_reset_lcd(struct platform_device *pdev);

#endif /* _FB_H */

