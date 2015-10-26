/*****************************************************************************
* Copyright 2002 - 2008 Broadcom Corporation.  All rights reserved.
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




/*
*
****************************************************************************
*
*    Description:
*      AK4642 External Codec I2C Functions
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/kernel.h>
#include <linux/i2c.h>

#include "ext_codec_ak4642_i2c.h"

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

/* I2C */
#define IF_NAME         "ak4642i2c"
#define I2C_DRIVERID_AK4642   (0xF000 | EXT_CODEC_AK4642_I2C_ADDR)
#define EXT_CODEC_AK4642_I2C_ADDR (0x12) /* 00100100, CODEC CSN pin is low in AK4642 */
                                         /* 00100101, CODEC CSN pin is low in AK4642 */
/* ---- Private Variables ------------------------------------------------ */

/* I2C variables */
/* Addresses to scan */
static unsigned short normal_i2c[] = {EXT_CODEC_AK4642_I2C_ADDR, I2C_CLIENT_END};
static unsigned short probe[2]        = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short ignore[2]       = { I2C_CLIENT_END, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
   .normal_i2c = normal_i2c,
   .probe      = probe,
   .ignore     = ignore,
};

struct ext_codec_ak4642_i2c_info
{
    struct i2c_client client;
};
static struct ext_codec_ak4642_i2c_info *ext_codec_ak4642_i2c_datap;

/* ---- Private Function Prototypes -------------------------------------- */
static int ext_codec_ak4642_i2c_attach(struct i2c_adapter *adap, int addr, int kind);
static int ext_codec_ak4642_i2c_attach_adapter(struct i2c_adapter *adap);
static int ext_codec_ak4642_i2c_detach_client(struct i2c_client *device);
static int ext_codec_ak4642_i2c_command(struct i2c_client *device, unsigned int cmd, void *arg);

struct i2c_driver i2c_driver_ext_codec_ak4642 =
{
	.driver = {
   .name           = IF_NAME,
   },
   .id             = I2C_DRIVERID_AK4642,
   .attach_adapter = ext_codec_ak4642_i2c_attach_adapter,
   .detach_client  = ext_codec_ak4642_i2c_detach_client,
   .command        = ext_codec_ak4642_i2c_command,
};

/* ---- Functions -------------------------------------------------------- */

int ext_codec_ak4642_ic_init(void)
{
   int rc;

    /* set up the I2C */
   rc = i2c_add_driver(&i2c_driver_ext_codec_ak4642);
   if (rc != 0)
   {
      printk("AK4642 - Failed to initialize I2C\n");
   }
   return( rc );
}

int ext_codec_ak4642_ic_release(void)
{
   int err = 0;

   /* Uninstall I2C */
   i2c_del_driver( &i2c_driver_ext_codec_ak4642 );

   return err;
}

/****************************************************************************
*
*  ext_codec_ak4642_i2c_read
*
*  reg: address of register to read
*
*  returns: data read (8 bits) or -1 on error
*
***************************************************************************/
int ext_codec_ak4642_i2c_read(unsigned char reg)
{
   /* Guard against calling in an atomic context */
   might_sleep();

   if ( ext_codec_ak4642_i2c_datap == NULL )
   {
      printk( "i2c bus not configured for ext_codec_ak4642\n" );
      return -1;
   }
   return i2c_smbus_read_byte_data(&ext_codec_ak4642_i2c_datap->client, reg);
}

/****************************************************************************
*
*  ext_codec_ak4642_i2c_write
*
*  reg: address of register to write
*  value: value to be written
*
*  returns: 0 on success, -1 on error
*
***************************************************************************/
int ext_codec_ak4642_i2c_write(unsigned char reg, unsigned char value)
{
   /* Guard against calling in an atomic context */
   might_sleep();

   if ( ext_codec_ak4642_i2c_datap == NULL )
   {
      printk( "i2c bus not configured for ext_codec_ak4642\n" );
      return -1;
   }
   return i2c_smbus_write_byte_data(&ext_codec_ak4642_i2c_datap->client, reg, value);
}

/****************************************************************************
*
*  ext_codec_ak4642_i2c_attach
*
***************************************************************************/
static int ext_codec_ak4642_i2c_attach(struct i2c_adapter *adap, int addr, int kind)
{
   struct i2c_client       *client;
   int err;

   (void) kind;

   ext_codec_ak4642_i2c_datap = kmalloc( sizeof( *ext_codec_ak4642_i2c_datap ), GFP_KERNEL );
   memset( ext_codec_ak4642_i2c_datap, 0, sizeof( *ext_codec_ak4642_i2c_datap ));

   client = &ext_codec_ak4642_i2c_datap->client;
   client->adapter = adap;
   client->addr = addr;
   client->driver = &i2c_driver_ext_codec_ak4642;
   sprintf(client->name, "%s-%x", IF_NAME, addr);

   i2c_set_clientdata( client, ext_codec_ak4642_i2c_datap );
   if ((err = i2c_attach_client(client)) < 0)
   {
      kfree( ext_codec_ak4642_i2c_datap );
      ext_codec_ak4642_i2c_datap = NULL;
      return err;
   }

   return 0;
}

/****************************************************************************
*
*  ext_codec_ak4642_i2c_attach_adapter
*
***************************************************************************/
static int ext_codec_ak4642_i2c_attach_adapter(struct i2c_adapter *adap)
{
   /* Look for this device on the given adapter (bus) */
   if ( (adap->id != I2C_HW_B_BCM1160) &&
        (adap->id != I2C_HW_B_BCM1161) )
   {
      return -1;
   }

   return i2c_probe(adap, &addr_data, &ext_codec_ak4642_i2c_attach);
}

/****************************************************************************
*
*  ext_codec_ak4642_i2c_detach_client
*
***************************************************************************/
static int ext_codec_ak4642_i2c_detach_client(struct i2c_client *device)
{
   int rc = 0;

   (void) device;
   if ((rc = i2c_detach_client(&ext_codec_ak4642_i2c_datap->client)) != 0)
   {
      printk(IF_NAME "detach failed: %d\n", rc);
   }
   else
   {
      kfree(ext_codec_ak4642_i2c_datap);
      ext_codec_ak4642_i2c_datap = NULL;
   }
   return rc;
}

/****************************************************************************
*
*  ext_codec_ak4642_i2c_command
*
***************************************************************************/
static int ext_codec_ak4642_i2c_command(struct i2c_client *device, unsigned int cmd, void *arg)
{
   (void)device; (void)cmd; (void)arg;
   return -1;
}
