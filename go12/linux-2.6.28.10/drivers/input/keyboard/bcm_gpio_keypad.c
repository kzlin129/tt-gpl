/*****************************************************************************
* Copyright 2007 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/



#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <asm/arch/reg_gpio.h>
#include <asm/arch/reg_irq.h>
#include <linux/platform_device.h>
#include <linux/bcm_gpio_keypad.h>
#include <linux/broadcom/hw_cfg.h>
#include <linux/broadcom/gpio.h>
#include <asm/gpio.h>

/*Debug messages */
//#define	BCMKP_DEBUG	
#ifdef BCMKP_DEBUG
	#define BCMKP_DBG(format, args...)	printk( __FILE__ ":" format, ## args)
#else
	#define	BCMKP_DBG(format, args...)
#endif

#ifndef BCM_NO_COLUMN
#define BCM_NO_COLUMN -1
#endif

#define ROW_MASK              0xf
#define MAX_ROWS              (ROW_MASK + 1)
#define COL_MASK              0xf
#define MAX_COLS              (COL_MASK + 1)
#define MAX_SCANCODES         ( MAX_ROWS * MAX_COLS )

#define SCANCODE(r,c)         (unsigned int)((((r) & ROW_MASK) << 4) | \
                                           ((c) & COL_MASK))
#define SCANCODE_ROW(s)       (((s) >> 4) & ROW_MASK)
#define SCANCODE_COL(s)       ((s) & COL_MASK)

#define ARRAYSIZE(a)         ((sizeof(a))/(sizeof(a[0])))

struct CBLK {
	struct input_dev *input_dev;
    unsigned int row_count;
    unsigned int col_count;
    unsigned int key_count;
    int direct_key_col_index;
    int keyRowArray[MAX_ROWS]; 
    int keyColArray[MAX_COLS];    
    unsigned int    keycode[ MAX_SCANCODES ];           /* Use scancode as an index to lookup the corresponding keycode */  
    unsigned long   prevDown[ ( MAX_SCANCODES + BITS_PER_LONG - 1 ) / BITS_PER_LONG ];      /* Bitmask Array for previous key pressed */
};


static int elementExistInArray( int value, unsigned int num_of_elements, int array[] );

/****************************************************************************f
*
*   Scan the matrix, and generate event for changes.
*
***************************************************************************/
static void keypad_scan( int gpio_pin, struct CBLK *bcm_kp )
{
	unsigned int stat;
	int r, gpio_c, value;
    int c = 0;

    r = elementExistInArray( gpio_pin, bcm_kp->row_count , bcm_kp->keyRowArray );
    stat = gpio_get_value(gpio_pin);
    if(stat)
    {
        /* At this point we know that the some key was released. So no point in scanning the 
        Matrix. Just report the last pressed key on same GPIO as released. */  
        
        /* Handle if its a Direct key release*/
        if(( bcm_kp->direct_key_col_index != -1 ) && ( bcm_kp->keycode[SCANCODE(r,bcm_kp->direct_key_col_index)] )) 
        {
            input_report_key(bcm_kp->input_dev, bcm_kp->keycode[SCANCODE(r,bcm_kp->direct_key_col_index)], !stat);         
            input_sync( bcm_kp->input_dev );
        }

        /* Handle Matrix Key Release */
        else
        {
            for(c=0;c< bcm_kp->col_count;c++)
            {
                if( test_bit( SCANCODE(r,c), bcm_kp->prevDown ) )
                {
                    input_report_key(bcm_kp->input_dev, bcm_kp->keycode[SCANCODE(r,c)], !stat);
                    input_sync( bcm_kp->input_dev );                    
                    change_bit( SCANCODE(r,c), bcm_kp->prevDown );
                    BCMKP_DBG(KERN_DEBUG "Key released r=%d c=%d..\n", r, c);
                } 
            }            
        }            
    }
    
    else {
        /* The Key is presses, so scan the Matrix */
        //Now we have the GPIO row that cause the interrupt 
        if(( bcm_kp->direct_key_col_index != -1 ) && ( bcm_kp->keycode[SCANCODE(r,bcm_kp->direct_key_col_index)] ))
        {
            /* The Key is a Direct Key */
            input_report_key(bcm_kp->input_dev, bcm_kp->keycode[SCANCODE(r,bcm_kp->direct_key_col_index)], !stat);            
            BCMKP_DBG(KERN_DEBUG "GPIO %d row press found Direct Key..\n", gpio_pin);
        }
        
        else{
            //Now set all Col as High
            for(c=0;c< bcm_kp->col_count;c++)
            {
                gpio_c = bcm_kp->keyColArray[c];
                if( gpio_c != BCM_NO_COLUMN )
                    gpio_set_value( gpio_c, 1 );
            }
            
            for(c=0;c< bcm_kp->col_count;c++)
            {
                gpio_c = bcm_kp->keyColArray[c];
                if( gpio_c == BCM_NO_COLUMN )
                    continue;
                gpio_set_value( gpio_c, 0 ); 
                value = gpio_get_value(gpio_pin); 
                gpio_set_value( gpio_c, 1 );
                
                if( !value )  //Found the Key
                {
                    if( bcm_kp->keycode[SCANCODE(r,c)] && !test_bit( SCANCODE(r,c), bcm_kp->prevDown ) )
                    {
                        /* report status to input-subsystem */
                        input_report_key(bcm_kp->input_dev, bcm_kp->keycode[SCANCODE(r,c)], !value);
                        input_sync( bcm_kp->input_dev );
                        change_bit( SCANCODE(r,c), bcm_kp->prevDown );
                        BCMKP_DBG(KERN_DEBUG "Key Pressed r=%d c=%d..\n", r, c);
                        break;
                    }
                }
                
            }
            
             //Now restore all the Col as Low for next interrupt
            for(c=0;c< bcm_kp->col_count;c++)
            {
                gpio_c = bcm_kp->keyColArray[c];
                if( gpio_c != BCM_NO_COLUMN )
                    gpio_set_value( gpio_c, 0 );
            }           
            
        }
	}
	return;
}

/****************************************************************************
*
*   This function free the IRQ for the GPIO interrupt
*
***************************************************************************/
static void free_all_irqs( struct CBLK *bcm_kp )
{
    int i;

    for(i=0;i<bcm_kp->row_count;i++)
    {
        if( bcm_kp->keyRowArray[i] != BCM_NO_COLUMN )
            free_irq( GPIO_TO_IRQ(bcm_kp->keyRowArray[i]), (void *)bcm_kp );
    }

	return;
}

/****************************************************************************
*
*   Interrupt handler, called whenever a keypress/release occurs
*
***************************************************************************/
static irqreturn_t keypad_gpio_irq(int irq, void *dev_id)
{
    struct CBLK *bcm_kp = dev_id;
    int gpio_pin = IRQ_TO_GPIO(irq);
    
    disable_irq(irq);
    reg_gpio_set_debounce_disable( gpio_pin );
	keypad_scan( gpio_pin, bcm_kp );
    reg_gpio_set_debounce_enable( gpio_pin );
    reg_gpio_clear_interrupt( gpio_pin );
	enable_irq(irq);

	return IRQ_HANDLED;
}

/****************************************************************************
*
*   This function looks at the given array and returns the index for the matching value
*
***************************************************************************/
static int elementExistInArray( int value, unsigned int num_of_elements, int array[] )
{
    int i;
    
    for(i=0; i< num_of_elements; i++){
        if( array[i] == value )
            return (i);
    }
    return(-1);    
}

/****************************************************************************
*
*   This function takes the Keymap definition and create a list of GPIO Rows
*
***************************************************************************/
static int CreateRowArray( struct CBLK *bcm_kp, struct BCM_GPIO_KEYMAP *keymap_p )
{
    int i, j=0;
    
    for(i=0;i< bcm_kp->key_count; i++ )
        bcm_kp->keyRowArray[i] = BCM_NO_COLUMN;
    
    //Fill the keyRowArray here
    for(i=0;i<bcm_kp->key_count;i++){
        if( ( keymap_p->gpio_row != BCM_NO_COLUMN ) && 
            ( (elementExistInArray( keymap_p->gpio_row, i , bcm_kp->keyRowArray )) < 0 )) 
        {
            bcm_kp->keyRowArray[j] = keymap_p->gpio_row;
            BCMKP_DBG(KERN_DEBUG "Found Row %d\n", keymap_p->gpio_row );
            j++;
        }
        keymap_p++;
    }
    return j;
}

/****************************************************************************
*
*   This function takes the Keymap definition and create a list of GPIO Column
*
***************************************************************************/
static int CreateColArray( struct CBLK *bcm_kp, struct BCM_GPIO_KEYMAP *keymap_p )
{
    int i, j=0;
 
    for(i=0;i< bcm_kp->key_count; i++ )
        bcm_kp->keyColArray[i] = BCM_NO_COLUMN;
    
    //Fill the keyColArray here    
    for(i=0;i<bcm_kp->key_count;i++){
        if( elementExistInArray( keymap_p->gpio_col, i, bcm_kp->keyColArray ) < 0 )
        {
            bcm_kp->keyColArray[j] = keymap_p->gpio_col;
            BCMKP_DBG(KERN_DEBUG "Found Col %d\n", keymap_p->gpio_col );
            j++;
        }
        keymap_p++;
    }
    
    
    if( elementExistInArray( BCM_NO_COLUMN, j, bcm_kp->keyColArray ) >= 0 )
    {
        bcm_kp->direct_key_col_index = elementExistInArray( BCM_NO_COLUMN, j, bcm_kp->keyColArray );
        BCMKP_DBG(KERN_DEBUG "GPIO Col Found %d, %d\n", bcm_kp->keyColArray[bcm_kp->direct_key_col_index] , bcm_kp->direct_key_col_index);
    }
    return j;   
}

/****************************************************************************
*
*   This function creates the Keymap Array with scan code RC pointing to KeyCode
*
***************************************************************************/
static int CreateKeyMap( struct CBLK *bcm_kp, struct BCM_GPIO_KEYMAP *keymap_p ) 
{
    int r,k, c=0;

    for(k=0;k<bcm_kp->key_count; k++)
    {
        r =  elementExistInArray( keymap_p->gpio_row, bcm_kp->row_count, bcm_kp->keyRowArray );
        c =  elementExistInArray( keymap_p->gpio_col, bcm_kp->col_count, bcm_kp->keyColArray );

        bcm_kp->keycode[SCANCODE(r,c)] = keymap_p->key_code;
        BCMKP_DBG(KERN_NOTICE "setting keymap r= %d, c=%d, code=%d\n", r,c,keymap_p->key_code);
        set_bit(keymap_p->key_code ,bcm_kp->input_dev->keybit);
        keymap_p++;
    }
    return(1);
}

/****************************************************************************
*
*   This function all the GPIO Col as output
*
***************************************************************************/
static int SetColOutput( int array[], int col_count )
{
    int i,gpio;
 
    for(i=0;i< col_count; i++ )
    {
        gpio = array[i];
        if( gpio != BCM_NO_COLUMN )
        {
            gpio_direction_output( gpio, 0 );
        }
    }
    return(0);
}

/****************************************************************************
*
*   This function sets all the GPIO Rows input and request to register the IRQ
*
***************************************************************************/
static int SetRowInput( int array[], int row_count, struct CBLK *bcm_kp )
{
    int i, gpio;
    int ret;
    
    for(i=0;i< row_count; i++ )
    {
        gpio = array[i];
        if( gpio != BCM_NO_COLUMN )
        {
            reg_gpio_set_pull_up_down(gpio, 1);
            reg_gpio_set_pull_up_down_enable(gpio);
            reg_gpio_set_debounce_value_enable( gpio, GPIO_DEBOUNCE_16_MS );
            ret = request_irq( GPIO_TO_IRQ(gpio), keypad_gpio_irq, ( IRQF_SHARED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING ), \
                "GPIO Keypad", bcm_kp);
            if(ret < 0)
            {
                printk(KERN_ERR "%s(%s:%u)::GPIO %d request_irq failed IRQ %d\n",  __FUNCTION__, __FILE__, __LINE__, gpio, GPIO_TO_IRQ(gpio));
                return ret;
            }
        }
    } 
   
    return(0);
}

/****************************************************************************
*
*   Called to perform module initialization when the module is loaded
*
***************************************************************************/
static int __devinit bcm_gpio_kp_probe(struct platform_device *pdev)
{
    int ret;
    struct CBLK    *bcm_kp; 
    struct BCM_KEYPAD_PLATFORM_DATA *pdata = pdev->dev.platform_data;
    struct BCM_GPIO_KEYMAP *keymap_p = pdata->keymap;

	BCMKP_DBG(KERN_NOTICE "bcm_gpio_kp_probe\n");

    if(( bcm_kp = kmalloc(sizeof(*bcm_kp), GFP_KERNEL)) == NULL )
    {
		printk(KERN_ERR "%s(%s:%u)::Failed to allocate keypad structure...\n",  __FUNCTION__, __FILE__, __LINE__);
		return -ENOMEM;
    }
    memset( bcm_kp, 0, sizeof(*bcm_kp));
    
   	if(( bcm_kp->input_dev = input_allocate_device()) == NULL )
    {
		printk(KERN_ERR "%s(%s:%u)::Failed to allocate input device...\n", __FUNCTION__, __FILE__, __LINE__);
		return -ENOMEM;
   	}
    platform_set_drvdata(pdev, bcm_kp);
    
    /* Setup input device */
   	set_bit(EV_KEY , bcm_kp->input_dev->evbit);
	set_bit(EV_REP, bcm_kp->input_dev->evbit);
    
    bcm_kp->input_dev->name = "keypad";
	bcm_kp->input_dev->phys = "keypad/input0";
	bcm_kp->input_dev->id.bustype = BUS_HOST;
	bcm_kp->input_dev->id.vendor = 0x0001;
	bcm_kp->input_dev->id.product = 0x0001;
	bcm_kp->input_dev->id.version = 0x0100;
    bcm_kp->input_dev->keycode = bcm_kp->keycode;
    bcm_kp->input_dev->keycodesize = sizeof( bcm_kp->keycode[0] );
    bcm_kp->input_dev->keycodemax = ARRAYSIZE( bcm_kp->keycode ) ; 
    memset( bcm_kp->prevDown, 0, sizeof( bcm_kp->prevDown ));


#ifdef  SET_GPIO_KEYPAD
    /* If any special hardware specific initilization is needed before gpios can be used, please put them under 
    this macro. e.g in BCM2153 chip, by default the keypad gpios goes through keypad controller and this macro 
    switchs them back to be used as regular gpios. */
    SET_GPIO_KEYPAD;
#endif
    
    /* Setup GPIO and Interrupt handler */
    bcm_kp->direct_key_col_index = -1;
    bcm_kp->key_count = pdata->array_size;
    bcm_kp->row_count = CreateRowArray( bcm_kp, keymap_p );
    bcm_kp->col_count = CreateColArray( bcm_kp, keymap_p );
    CreateKeyMap( bcm_kp, keymap_p );
    SetColOutput( bcm_kp->keyColArray, bcm_kp->col_count );
    ret = SetRowInput( bcm_kp->keyRowArray, bcm_kp->row_count, bcm_kp ); 
    if (ret < 0) 
    {
        printk(KERN_ERR "%s(%s:%u)::Unable to register GPIO-keypad IRQ\n", __FUNCTION__, __FILE__, __LINE__);
        goto free_irq;
    }

	ret = input_register_device(bcm_kp->input_dev);
    if (ret < 0) 
    {
        printk(KERN_ERR "%s(%s:%u)::Unable to register GPIO-keypad input device\n", __FUNCTION__, __FILE__, __LINE__);
        goto free_dev;
    }
    
    /* Initialization Finished */
	BCMKP_DBG(KERN_DEBUG "BCM keypad initialization completed...\n");
    return (ret);

free_dev:
    input_unregister_device(bcm_kp->input_dev);
    input_free_device(bcm_kp->input_dev);
    
free_irq:
	free_all_irqs( bcm_kp );
    
    return -EINVAL;
}

/****************************************************************************
*
*   Called to perform module cleanup when the module is unloaded.
*
***************************************************************************/
static void __exit bcm_gpio_kp_remove(struct platform_device *pdev)
{
    struct CBLK *bcm_kp = platform_get_drvdata(pdev);
	BCMKP_DBG(KERN_NOTICE "bcm_gpio_kp_remove\n");

	/*disable keypad interrupt handling*/
	free_all_irqs( bcm_kp );
    
    /* unregister everything */
    input_unregister_device(bcm_kp->input_dev);
    input_free_device(bcm_kp->input_dev);
	return;
}
/****************************************************************************/

struct platform_driver gpio_keys_device_driver = {
	.probe		= bcm_gpio_kp_probe,
	.remove		= __devexit_p(bcm_gpio_kp_remove),
	.driver		= {
		.name	= "bcm-gpio-keypad",
	}
};

static int __init bcm_gpio_kp_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit bcm_gpio_kp_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(bcm_gpio_kp_init);
module_exit(bcm_gpio_kp_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM GPIO Keypad Driver");
MODULE_LICENSE("GPL");
