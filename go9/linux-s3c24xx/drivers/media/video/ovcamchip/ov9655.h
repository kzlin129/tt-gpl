/* Private member variables for the ov9655 driver. */ 
struct ov9655_private
{
        /* Image properties. */
        struct
        {
                signed long int saturation;
                signed long int contrast;
                signed long int brightness;
                signed long int hue;
        } image_properties;
};

/* formula a * gain * sin( alpha ) + b * gain * cos( alpha ) + c */
/* ?_10k == 10000 * ?, so a_10k = 10000 * a. */
struct color_correct
{       
        signed long int        a_10k;
        signed long int        b_10k;
        signed long int        c_10k;
        unsigned long int      target_reg;
};

/* Default image property values. */
//#define OV9XX0_DEF_SATURATION	65;
//#define OV9XX0_DEF_CONTRAST	200;
//#define OV9XX0_DEF_BRIGHTNESS	-105;
//#define OV9XX0_DEF_HUE		2340;
#define OV9XX0_DEF_SATURATION	66;
#define OV9XX0_DEF_CONTRAST	100;
#define OV9XX0_DEF_BRIGHTNESS	-75;
#define OV9XX0_DEF_HUE		1700;

/* Color correction routine. */
extern int ov9xx0_do_color_correct( struct i2c_client *c, signed long int alpha, signed long int gain );
