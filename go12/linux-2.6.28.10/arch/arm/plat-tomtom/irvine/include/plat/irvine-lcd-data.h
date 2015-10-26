#ifndef __IRVINE_LCD_PANELS_H__
#define __IRVINE_LCD_PANELS_H__

#include <plat/irvine-lcd.h>

/* Samsung LCM panel */
extern struct clcd_paneldev LMS350_WithPLL;
extern struct clcd_paneldev LMS430_WithPLL;
extern struct clcd_paneldev LMS500_WithPLL;
extern struct clcd_paneldev LMS430HF37_WithPLL;
extern struct clcd_paneldev LMS500HF10_WithPLL;
extern struct clcd_paneldev LMS501KF03_WithPLL;
extern struct clcd_paneldev LMS606KF01_WithPLL;

/* AUO LCM panel */
extern struct clcd_paneldev A043FW05V4_WithPLL;
extern struct clcd_paneldev A050FW03V4_WithPLL;

/* LG LCM panel */
extern struct clcd_paneldev LB043WQ3_WithPLL;
extern struct clcd_paneldev LD050WQ1_WithPLL;

#endif /* __IRVINE_LCD_PANELS_H__ */

