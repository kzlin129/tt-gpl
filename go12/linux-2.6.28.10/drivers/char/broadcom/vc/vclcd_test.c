/*****************************************************************************
* Copyright 2005 - 2008 Broadcom Corporation.  All rights reserved.
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
 * Project  :  vmcs_host_apps\apps\tutorials
 * Module   :  dispman2
 * File     :  $Workfile: simulate.h $
 * Revision :  $Revision: 1.2.2.1 $

 * FILE DESCRIPTION
 * dispman2
*****************************************************************************/

//#include <memory.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>

//#include "host_app.h"
//#include "target.h"
//#include "graphics.h"
#include <linux/string.h>

#include "vclcd_test.h"
#include "vcgencmd.h"
#include "vc_dispman2.h"
#include "vchost_config.h"

#include <linux/broadcom/lcd.h>

#define DISPMAN2_OBJ_SCREEN_DISPLAY    0

//the type of bitmap
#if 1
   #define DISPMAN2_OBJ_BITMAP_TYPE       VC_FORMAT_RGB565
   #define DISPMAN2_OBJ_BYTES_PER_PIXEL   2
   typedef uint16_t  Pixel_t;
#else
   #define DISPMAN2_OBJ_BITMAP_TYPE       VC_FORMAT_RGBA32
   #define DISPMAN2_OBJ_BYTES_PER_PIXEL   4
   typedef uint32_t  Pixel_t;
#endif

//the number of bitmaps
#define DISPMAN2_OBJ_NUM_BITMAPS       8

//the size of the bitmaps
#define DISPMAN2_OBJ_WIDTH_BITMAPS     32
#define DISPMAN2_OBJ_HEIGHT_BITMAPS    32




//the structure which stores the information about a bitmap
typedef struct
{
   //the bitmap data
   Pixel_t     data[ DISPMAN2_OBJ_WIDTH_BITMAPS * DISPMAN2_OBJ_HEIGHT_BITMAPS ];

   //total bitmap size
   uint32_t    data_size;

   //the width and height of the bitmap
   uint32_t    width;
   uint32_t    height;

   //the pitch of the bitmap. this is the width of the bitmap aligned to a 16byte boundary
   //it is used by videocore to improve the efficiency of the bitmap processing
   uint32_t    pitch;

   //the current coordinates of the bitmap on the screen
   int32_t     x;
   int32_t     y;

} DISPMAN2_OBJ_BITMAPS_T;


//the structure which stores the dispman2_obj application state
typedef struct
{
   //the bitmaps
   DISPMAN2_OBJ_BITMAPS_T  bitmaps[ DISPMAN2_OBJ_NUM_BITMAPS ];

   //resource handles for the bitmaps
   uint32_t                bitmap_resource_handles[ DISPMAN2_OBJ_NUM_BITMAPS ];

   //object handles for the bitmaps
   uint32_t                bitmap_object_handles[ DISPMAN2_OBJ_NUM_BITMAPS ];

} DISPMAN2_OBJ_STATE_T;


/******************************************************************************
Static funcs forwards
******************************************************************************/

/* Routine used to create the initial bitmaps and display them on the screen */
static int dispman2_obj_create_and_display_bitmaps(DISPMAN2_OBJ_BITMAPS_T *bitmaps,
                                                   const uint32_t num_bitmaps,
                                                   const uint32_t bitmap_width,
                                                   const uint32_t bitmap_height,
                                                   uint32_t *resource_handles,
                                                   uint32_t *object_handles );

//Routine used to create the bitmaps
static int dispman2_obj_create_bitmaps(DISPMAN2_OBJ_BITMAPS_T *bitmaps,
                                       const uint32_t num_bitmaps,
                                       const uint32_t bitmap_width,
                                       const uint32_t bitmap_height );

//Routin to create the resource handles for use by dispman
static int dispman2_obj_create_resources( DISPMAN2_OBJ_BITMAPS_T *bitmaps,
                                          uint32_t *resource_handles,
                                          const uint32_t num_bitmaps );

//Routine to display a list of bitmaps on the screen
static int dispman2_obj_display_bitmaps(  DISPMAN2_OBJ_BITMAPS_T *bitmaps,
                                          uint32_t *resource_handles,
                                          uint32_t *object_handles,
                                          const uint32_t num_bitmaps,
                                          const int32_t display_bitmaps );


//Routine to fill in a bitmap with colour
static int dispman2_obj_fill_bitmap(DISPMAN2_OBJ_BITMAPS_T *bitmap,
                                    const uint32_t colour );

/******************************************************************************
Static Data
******************************************************************************/

//graphics macro's
#define R_555_MASK (0x7600)
#define G_555_MASK (0x03E0)
#define B_555_MASK (0x001F)

#define R_565_MASK (0xF800)
#define G_565_MASK (0x07E0)
#define B_565_MASK (0x001F)

#define R_888_MASK      (0x00FF0000)
#define G_888_MASK      (0x0000FF00)
#define B_888_MASK      (0x000000FF)
#define ALPHA_888_MASK  (0xFF000000)

#define GRAPHICS_RGB565( r, g, b ) ( ((r << (5+6)) & R_565_MASK) | ((g << 5) & G_565_MASK) | (b & B_565_MASK) )


//this data is used by the bitmap colouring in -
const Pixel_t dispman2_obj_bitmap_colours[ DISPMAN2_OBJ_NUM_BITMAPS ] =
{
   //white
   GRAPHICS_RGB565( 0xFF, 0xFF, 0xFF ),

   //red
   GRAPHICS_RGB565( 0xFF, 0x00, 0x00 ),

   //green
   GRAPHICS_RGB565( 0x00, 0xFF, 0x00 ),

   //blue
   GRAPHICS_RGB565( 0x00, 0x00, 0xFF ),

   //red
   GRAPHICS_RGB565( 0xFF, 0x00, 0x00 ),

   //green
   GRAPHICS_RGB565( 0x00, 0xFF, 0x00 ),

   //blue
   GRAPHICS_RGB565( 0x00, 0x00, 0xFF ),


   //yellow
   GRAPHICS_RGB565( 0xFF, 0xFF, 0x00 )
};

static DISPMAN2_OBJ_STATE_T   gDispman2ObjState;

static LCD_Info_t    gLcdInfo;


/******************************************************************************
Global functions
******************************************************************************/


/***********************************************************
 * Name: vclcd_test
 *
 * Arguments: None.
 *
 * Description: The entry point for the test app
 *
 * Returns: void
 *
 ***********************************************************/
void vclcd_test( void )
{
   int success = 0; //success by default
   static DISPMAN2_OBJ_STATE_T *dispman2_obj_state; //the dispman2_obj tutorials state

   lcd_get_info( &gLcdInfo );


   {
      //malloc and zero a some storage to contain this apps state
      //dispman2_obj_state = (DISPMAN2_OBJ_STATE_T *)calloc( 1, sizeof( DISPMAN2_OBJ_STATE_T ) );
      dispman2_obj_state = &gDispman2ObjState;
      memset( &gDispman2ObjState, 0, sizeof( gDispman2ObjState ) );

      if( NULL != dispman2_obj_state)
      {
         //create the initial bitmaps, fill them in with colours and display them on the screen
         success = dispman2_obj_create_and_display_bitmaps( dispman2_obj_state->bitmaps, //pointers to the bitmaps
                                                            DISPMAN2_OBJ_NUM_BITMAPS, //number of bitmaps to add to the display
                                                            DISPMAN2_OBJ_WIDTH_BITMAPS, //the width of the bitmaps to create
                                                            DISPMAN2_OBJ_HEIGHT_BITMAPS, //the height of the bitmaps to create
                                                            dispman2_obj_state->bitmap_resource_handles, //the resource handles of the bitmaps
                                                            dispman2_obj_state->bitmap_object_handles ); //the object handles of the bitmaps

      }

      //check the initialisation worked
      vc_assert( success >= 0 );
   }
}

/******************************************************************************
Static functions
******************************************************************************/

/***********************************************************
 * Name: dispman2_obj_create_and_display_bitmaps
 *
 * Arguments:
 *       DISPMAN2_OBJ_BITMAPS_T *bitmaps - a ptr to the bitmap data
         const uint32_t num_bitmaps - the number of bitmaps to initialise
         const uint32_t bitmap_width - the bitmap height
         const uint32_t bitmap_heigh - the bitmap width
         uint32_t *resource_handles - the resource handles for the bitmaps
         uint32_t *object_handles - the object handles for the bitmaps
 *
 * Description: A routine to create the bitmaps, fill them in with colour
 *              and display them on the screen
 *
 * Returns: int - < 0 is fail
 *
 ***********************************************************/
static int dispman2_obj_create_and_display_bitmaps(DISPMAN2_OBJ_BITMAPS_T *bitmaps,
                                                   const uint32_t num_bitmaps,
                                                   const uint32_t bitmap_width,
                                                   const uint32_t bitmap_height,
                                                   uint32_t *resource_handles,
                                                   uint32_t *object_handles )
{
   int success = -1; //fail by default

   //first, create some bitmaps which we will use to display on the screen
   //These bitmaps get created with the width and height passed into the function
   //and are filled in with random colours
   success = dispman2_obj_create_bitmaps( bitmaps,
                                          num_bitmaps,
                                          bitmap_width,
                                          bitmap_height );

   //next, create dispman resource id's for each bitmap (this is so dispman knows how to access the bitmaps)
   if( success >= 0 )
   {
      success = dispman2_obj_create_resources(  bitmaps,
                                                resource_handles,
                                                num_bitmaps );
   }

   //finally, add all the bitmaps to the screen
   if( success >= 0 )
   {
      success = dispman2_obj_display_bitmaps(bitmaps, //pointers to the bitmaps
                                             resource_handles, //the resource handles of the bitmaps
                                             object_handles, //the object handles of the bitmaps
                                             num_bitmaps, //number of bitmaps to add to the display
                                             1 ); //this parameter says whether to display the bitmaps or not
   }

   return success;
}

/***********************************************************
 * Name: dispman2_obj_create_bitmaps
 *
 * Arguments:
 *       DISPMAN2_OBJ_BITMAPS_T *bitmaps - a ptr to the bitmap data
         const uint32_t num_bitmaps - the number of bitmaps to initialise
         const uint32_t bitmap_width - the bitmap height
         const uint32_t bitmap_heigh - the bitmap width
 *
 * Description: A routine to create the bitmaps
 *
 * Returns: int - < 0 is fail
 *
 ***********************************************************/
static int dispman2_obj_create_bitmaps(DISPMAN2_OBJ_BITMAPS_T *bitmaps,
                                       const uint32_t num_bitmaps,
                                       const uint32_t bitmap_width,
                                       const uint32_t bitmap_height )
{
   int success = -1; //fail by default


   if( ( NULL != bitmaps) && (0 != num_bitmaps ) )
   {
      int32_t count = 0;
      uint32_t pitch = 0;
      uint32_t bitmap_data_size = 0;

      //default to success!
      success = 0;

      //first, calculate the pitch of the bitmaps.
      //pitch = (((bitmap_width * sizeof( uint32_t ))+15) & (~0xF));
      pitch = (((bitmap_width * sizeof( Pixel_t ))+15) & (~0xF));

      //calculate the size of the bitmap
      bitmap_data_size = pitch * bitmap_height;

      for( count = 0; count < num_bitmaps; count++)
      {
         //malloc some data for the bitmaps
         //bitmap->data = malloc( bitmap_data_size );

         if( NULL != bitmaps[ count ].data )
         {
            //store the width and height
            bitmaps[ count ].width = bitmap_width;
            bitmaps[ count ].height = bitmap_height;

            //store the bitmap data size and the pitcj
            bitmaps[ count ].pitch = pitch;
            bitmaps[ count ].data_size = bitmap_data_size;

            //set the initial coordinates
            bitmaps[ count ].x = (gLcdInfo.width / num_bitmaps) * count;
            bitmaps[ count ].y = (gLcdInfo.height / num_bitmaps) * count;


            //finally, fill the bitmap in with different colours
            success = dispman2_obj_fill_bitmap( &bitmaps[ count ],
                                                dispman2_obj_bitmap_colours[ count ] );
         }
         else
         {
            //malloc failed!
            vc_assert( 0 );

            //failed
            success = -1;

            //break; //exit for loop
         }
      }
   }
   else
   {
      //bad parameters
      vc_assert( 0 );
   }

   return success;
}

/***********************************************************
 * Name: dispman2_obj_create_resources
 *
 * Arguments:
 *       DISPMAN2_OBJ_BITMAPS_T *bitmaps,
         uint32_t *resource_handles,
         const uint32_t num_bitmaps
 *
 * Description: A routine to create the resources which dispman can use
 *
 * Returns: int - < 0 is fail
 *
 ***********************************************************/
static int dispman2_obj_create_resources( DISPMAN2_OBJ_BITMAPS_T *bitmaps,
                                          uint32_t *resource_handles,
                                          const uint32_t num_bitmaps )
{
   int success = -1; //fail by default

   if( (NULL != bitmaps) && (NULL != resource_handles) && (0 != num_bitmaps ) )
   {
      int32_t count = 0;

      //default to success!
      success = 0;

      for( count = 0; count < num_bitmaps; count++)
      {
         VC_IMAGE_PARAM_T image_params;
         uint32_t pitch = 0;

         //calculate the pitch of the bitmap
         //pitch = (((bitmaps[count].width * sizeof( uint32_t ))+15) & (~0xF));
         pitch = (((bitmaps[count].width * sizeof( Pixel_t ))+15) & (~0xF));

         //set up the dispman image params
         image_params.type = DISPMAN2_OBJ_BITMAP_TYPE;
         image_params.width = bitmaps[count].width;   // width in pixels
         image_params.height = bitmaps[count].height;  // height in pixels
         image_params.pitch = pitch;   // pitch of image_data array in *bytes*
         image_params.size = pitch * bitmaps[count].height;  // number of *bytes* available in the image_data array
         image_params.pointer = (uint32_t) bitmaps[count].data;    // pointer for image_data

         //now, map the local resource to the
         if( 0 != vc_dispman_resource_create(&resource_handles[count],
                                             &image_params,
                                             VC_RESOURCE_TYPE_HOST) )
         {
            //failed to created resource
            vc_assert( 0 );

            //failed
            success = -1;

            //break;
         }
      }
   }
   else
   {
      //bad parameters
      vc_assert( 0 );
   }

   return success;
}


/***********************************************************
 * Name: dispman2_obj_display_bitmaps
 *
 * Arguments:
 *       DISPMAN2_OBJ_BITMAPS_T *bitmaps,
         uint32_t *resource_handles,
         uint32_t *object_handles,
         const uint32_t num_bitmaps,
         const int32_t display_bitmaps
 *
 * Description: A routine to display bitmaps on the screen
 *
 * Returns: int - < 0 is fail
 *
 ***********************************************************/
static int dispman2_obj_display_bitmaps(  DISPMAN2_OBJ_BITMAPS_T *bitmaps,
                                          uint32_t *resource_handles,
                                          uint32_t *object_handles,
                                          const uint32_t num_bitmaps,
                                          const int32_t display_bitmaps )
{
   int success = -1; //fail by default

   if( (NULL != bitmaps) && ( NULL != resource_handles) && (0 != num_bitmaps ) )
   {
      int32_t count = 0;
      int32_t response = 0;

      //default to success!
      success = 0;

      //start an update
      if( 0 == vc_dispman_update_start(&response) )
      {
         for( count = 0; count < num_bitmaps; count++)
         {
            //do we want to display the bitmaps?
            if( display_bitmaps )
            {
               success = vc_dispman_object_add( object_handles,
                                                0,
                                                0, //the z order of the bitmap - we use the count var to give some depth to the image
                                                bitmaps[count].x,
                                                bitmaps[count].y,
                                                bitmaps[count].width,
                                                bitmaps[count].height,
                                                resource_handles[ count ],
                                                0,
                                                0,
                                                0);
            }
         }

         // This call blocks until the update is done.
         success += vc_dispman_update_end(&response);
      }
   }
   else
   {
      //bad parameters
      vc_assert( 0 );
   }

   return success;
}


/***********************************************************
 * Name: dispman2_obj_fill_bitmap
 *
 * Arguments:
 *       DISPMAN2_OBJ_BITMAPS_T *bitmap,
         const uint32_t colour
 *
 * Description: Routine to update the bitmaps colour
 *
 * Returns: int - < 0 is fail
 *
 ***********************************************************/
static int dispman2_obj_fill_bitmap(DISPMAN2_OBJ_BITMAPS_T *bitmap,
                                    const uint32_t colour )
{
   int success = -1; //fail by default

   if(NULL != bitmap)
   {
      int32_t height_count = 0;
      int32_t width_count = 0;

      //for every row of data
      for( height_count = 0; height_count < bitmap->height; height_count++)
      {
         //calculate a ptr to the start of each row of data
         Pixel_t *row_ptr = (Pixel_t *) ((uint8_t *)bitmap->data + (bitmap->pitch * height_count) );

         //fill in each pixel in the row
         for( width_count = 0; width_count < bitmap->width; width_count++)
         {
            *(row_ptr + width_count) = colour;
         }
      }

      //success!
      success = 0;
   }

   return success;
}


/****************************** End of file *******************************************/
