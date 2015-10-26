/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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




/*=============================================================================

Project  :  VMCS Host Apps
Module   :  Tutorials - Dispman2 frame buffer
File     :  $RCSfile: dispman2_framebuf.c,v $
Revision :  $Revision: #3 $

FILE DESCRIPTION
Dispman2_framebuf tutorial.
=============================================================================*/

#include <linux/broadcom/vc.h>
#include <linux/broadcom/vc03/vciface.h>
#include <linux/broadcom/vc03/vcos.h>
//#include <linux/broadcom/vc03/vc_vchi_gencmd.h>
#include <linux/broadcom/vc03/vc_vchi_dispmanx.h>
#include <linux/broadcom/vc03/graphics_x.h>
//#include <linux/broadcom/vc03/vchi/host_msgfifo_wrapper.h>
#include <linux/broadcom/vc03/vchi/vchi.h>
#include <linux/broadcom/vc03/vchi/message.h>
#include <linux/broadcom/vc03/vchi/endian.h>
#include <linux/broadcom/vc03/vchi/mphi.h>
#include <linux/broadcom/vc03/vchi/control_service.h>
#include <linux/broadcom/vc03/dispmanx_types.h>
#include <linux/broadcom/vc03/vc_dispmanx.h>

#if defined(__KERNEL__)

#include <linux/string.h>
#include <linux/random.h>

#define calloc(sz1, sz2) \
  ({						\
    void* __ptr = vmalloc((sz1) * (sz2));	\
    memset(__ptr, 0, (sz1) * (sz2));            \
    __ptr;                                      \
  })

#else // __KERNEL__

#include <string.h>
#include <stdio.h>

#endif

/******************************************************************************
Private typedefs, macros and constants
******************************************************************************/

#define DISPMANX_FRAMEBUF_SCREEN_DISPLAY     0

//the type of bitmap
#define DISPMANX_FRAMEBUF_BITMAP_TYPE             VC_IMAGE_RGBA32

//the size of the boxes to display on the screen
#define DISPMANX_FRAMEBUF_BOX_WIDTH             40
#define DISPMANX_FRAMEBUF_BOX_HEIGHT            40

//the size of the frame to put around the boxes
#define DISPMANX_FRAMEBUF_BOX_BORDER_SIZE       4

//the number of colours in use
#define DISPMANX_FRAMEBUF_NUM_COLOURS           8

//the ID for the timer which updates the colours in the framebuffer
#define DISPMANX_FRAMEBUF_UPDATE_TIMER_ID       1

//the freq of the timer
#define DISPMANX_FRAMEBUF_TIMER_FREQ_IN_MS      50//ms


//the structure which stores the information about the framebuffer
typedef struct
{
   //the bitmap data
   void        *data;

   //total bitmap size
   uint32_t    data_size;

   //the pitch of the bitmap
   uint32_t    pitch;

   //the resource handle in vmcs
   uint32_t    resource_handle;

   //the element handle (when it is on the display)
   uint32_t    element_handle;

} DISPMANX_FRAMEBUF_T;


//the structure used to describe a box
typedef struct
{
   uint16_t    x;
   uint16_t    y;

   uint32_t    colour_index;

} DISPMANX_FRAMEBUF_BOX_T;


//the structure which stores the information about the boxes of colour which
//are displayed on the framebuffer
typedef struct
{
   //the number of boxes on the screen
   uint32_t       num_boxes;

   //the box data
   DISPMANX_FRAMEBUF_BOX_T *box_data;

} DISPMANX_BOXES_T;


//the structure which stores the dispmanx_framebuf application state
typedef struct
{
   //display size
   uint32_t width;
   uint32_t height;

   //the framebuffer
   DISPMANX_FRAMEBUF_T     framebuffer_bitmap;

   //the state of the boxes of colour
   DISPMANX_BOXES_T        boxes;

   // the display
   DISPMANX_DISPLAY_HANDLE_T display;
} DISPMANX_FRAMEBUF_STATE_T;


/******************************************************************************
Static funcs forwards
******************************************************************************/

/* Routine used to create the initial bitmaps and display them on the screen */
static int dispmanx_framebuf_create_and_display_buffer(  DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
      const uint32_t screen_width,
      const uint32_t screen_height,
      DISPMANX_DISPLAY_HANDLE_T display );

/* Routine used to display boxes of colour on the framebuffer */
static int dispmanx_framebuf_create_and_display_boxes(DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
      DISPMANX_BOXES_T *boxes,
      const uint32_t screen_width,
      const uint32_t screen_height );

//Routine to update the colours in the bitmaps
static int dispmanx_framebuf_update_framebuffer(DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
      const uint16_t x,
      const uint16_t y,
      const uint16_t width,
      const uint16_t height,
      const uint32_t screen_width,
      const uint32_t screen_height );

/* Routine to render a box onto the framebuffer */
static int dispmanx_framebuf_draw_box( DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
                                       DISPMANX_FRAMEBUF_BOX_T *box,
                                       const uint32_t screen_width,
                                       const uint32_t screen_height );

//Routine to fill in an area of a bitmap with colour
static int dispmanx_framebuf_fill_bitmap( DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
      const uint32_t x,
      const uint32_t y,
      const uint32_t width,
      const uint32_t height,
      const uint32_t colour );

static int dispmanx_framebuf_undisplay( DISPMANX_FRAMEBUF_T *framebuffer_bitmap);
int dispmanx_test1(void*);


/******************************************************************************
Static Data
******************************************************************************/

const uint32_t dispmanx_framebuf_colours[ DISPMANX_FRAMEBUF_NUM_COLOURS ] =
  {
    GRAPHICS_RGBA888( 0xFF, 0x00, 0x00, 0xFF ),
    GRAPHICS_RGBA888( 0x00, 0xFF, 0x00, 0xFF ),
    GRAPHICS_RGBA888( 0xFF, 0xFF, 0x00, 0xFF ),
    GRAPHICS_RGBA888( 0x00, 0x00, 0xFF, 0xFF ),
    GRAPHICS_RGBA888( 0xFF, 0x00, 0xFF, 0xFF ),
    GRAPHICS_RGBA888( 0x00, 0xFF, 0xFF, 0xFF ),
    GRAPHICS_RGBA888( 0xFF, 0xFF, 0xFF, 0xFF ),
    GRAPHICS_RGBA888( 0xFF, 0xFF, 0x00, 0x80 ),
  };

static int dispmanx_test_in_progress = 0;

/*
 *
 */

static uint16_t get_random(void)
{
  static int random_number = 99;
  random_number = random_number * 1103515245 + 12345;
  return (uint16_t)(random_number/65536) % 32768;
}

/*
 *
 */
int vc_do_dispmanxtest( testnum )
{
   if( testnum > 0 )
   {
      if( dispmanx_test_in_progress == 0 )
{
         kthread_run(dispmanx_test1, NULL, "dispmanx_test1");		
         dispmanx_test_in_progress = 1;
      }
   }
   else if( (testnum == 0) && (dispmanx_test_in_progress == 1) )
   {
      dispmanx_test_in_progress = -1;
   }

  return 0;
}

/*
 *
 */
int dispmanx_test1(void* ptr) 
{  
  int success = 0; //success by default
  static DISPMANX_FRAMEBUF_STATE_T *dispmanx_framebuf_state; //the dispmanx_framebuf tutorials state
  char response[255];
  DISPMANX_MODEINFO_T modeinfo;
  int iter;
  uint32_t rand_box_num;
  DISPMANX_FRAMEBUF_BOX_T *box;
  
  (void)ptr;
  // enable the displays
  //vc_gencmd( response, 255, "display_control 0 power=2 backlight=2" );
  //vc_gencmd( response, 255, "display_control 2 power=2 mode=0 svideo=1" );
  
  //reset the static state for this tutorial
  dispmanx_framebuf_state = calloc( 1, sizeof( DISPMANX_FRAMEBUF_STATE_T ) );
  
  //get the platform display setup
  dispmanx_framebuf_state->display = vc_dispmanx_display_open(DISPMANX_FRAMEBUF_SCREEN_DISPLAY);
  vc_dispmanx_display_get_info( dispmanx_framebuf_state->display, &modeinfo );
  dispmanx_framebuf_state->width = modeinfo.width;
  dispmanx_framebuf_state->height = modeinfo.height;
  
  if ( NULL != dispmanx_framebuf_state)
    {
      //create the initial bitmaps, fill them in with colours and display them on the screen
      success = dispmanx_framebuf_create_and_display_buffer( &dispmanx_framebuf_state->framebuffer_bitmap,
							     dispmanx_framebuf_state->width,
							     dispmanx_framebuf_state->height,
							     dispmanx_framebuf_state->display );

      //if the framebuffer was created and displayed ok, add the boxes of colour on to the bitmap
      if ( success >= 0 )
	{
	  success = dispmanx_framebuf_create_and_display_boxes( &dispmanx_framebuf_state->framebuffer_bitmap,
								&dispmanx_framebuf_state->boxes,
								dispmanx_framebuf_state->width,
								dispmanx_framebuf_state->height );
	}
      
         //mark the whole framebuffer as dirty, causing the data to be re-fetched for the display
      if ( success >= 0 )
	{
	  success = dispmanx_framebuf_update_framebuffer( &dispmanx_framebuf_state->framebuffer_bitmap,
							  0, 0, //x and y pos
							  (uint16_t)dispmanx_framebuf_state->width,
							  (uint16_t)dispmanx_framebuf_state->height,
							  dispmanx_framebuf_state->width,
							  dispmanx_framebuf_state->height );
	  
	}
      
      iter = 1;
      while(iter < 1024 * 128)
	{
	  //	  VC_DEBUG(Trace, "iter=%d\n", iter);
	  //randomly, pick a box and update its colour!
	  rand_box_num = get_random(); 
	  rand_box_num %= dispmanx_framebuf_state->boxes.num_boxes;
	  box = &dispmanx_framebuf_state->boxes.box_data[ rand_box_num ];
	  
	  //check the random number doesn't overflow
	  assert( rand_box_num < dispmanx_framebuf_state->boxes.num_boxes );
	  
	  //inc the colour index of this box
	  box->colour_index = get_random();
	  box->colour_index = box->colour_index % DISPMANX_FRAMEBUF_NUM_COLOURS;
	  
	  //redraw the box on the screen
	  success = dispmanx_framebuf_draw_box(  &dispmanx_framebuf_state->framebuffer_bitmap,
						 box,
						 (uint16_t)dispmanx_framebuf_state->width,
                                                (uint16_t)dispmanx_framebuf_state->height);
	  
	  //tell vmcs that this box has been updated - it will only then refetch the data that represents the box,
	  //not the entire image.
	  success = dispmanx_framebuf_update_framebuffer( &dispmanx_framebuf_state->framebuffer_bitmap,
							  box->x,
							  box->y,
							  DISPMANX_FRAMEBUF_BOX_WIDTH,
							  DISPMANX_FRAMEBUF_BOX_HEIGHT,
							  dispmanx_framebuf_state->width,
							  dispmanx_framebuf_state->height );
	  ++iter; 
	  //msleep(10);
     if( dispmanx_test_in_progress == -1 )
     {
        iter = (1024*128);
     }

	}
    }
  
   dispmanx_framebuf_undisplay( &dispmanx_framebuf_state->framebuffer_bitmap );

   free( dispmanx_framebuf_state->boxes.box_data );
   free( dispmanx_framebuf_state->framebuffer_bitmap.data );
   free( dispmanx_framebuf_state );
   
  
   dispmanx_test_in_progress = 0;
  return 0;
}

/******************************************************************************
Static functions
******************************************************************************/


/***********************************************************
 * Name: dispmanx_framebuf_create_and_display_buffer
 *
 * Arguments:
 *       DISPMANX_FRAMEBUF_T *framebuffer_bitmap - the framebuffer to display
 *
 * Description: A routine to create a framebuffer and display it on the screen
 *
 * Returns: int - < 0 is fail
 *
 ***********************************************************/
static int dispmanx_framebuf_create_and_display_buffer(  DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
                                                         const uint32_t screen_width,
                                                         const uint32_t screen_height,
                                                         DISPMANX_DISPLAY_HANDLE_T display )
{
   int success = -1; //fail by default
   uint32_t pitch = 0;
   uint32_t bitmap_data_size = 0;
   uint32_t native_image_ptr;

   //store the bitmap data size and the pitch

   //set up the dispman image params
   if (DISPMANX_FRAMEBUF_BITMAP_TYPE == VC_IMAGE_RGBA32)
      pitch = (screen_width*4+31)&~31;
   assert( pitch );
   bitmap_data_size = pitch * screen_height;
   framebuffer_bitmap->data = calloc( 1, bitmap_data_size );

   if ( framebuffer_bitmap->data ) {

      //store the bitmap data size and the pitch
      framebuffer_bitmap->pitch = pitch;
      framebuffer_bitmap->data_size = bitmap_data_size;

      //now, map the local resource to videocore
      framebuffer_bitmap->resource_handle = vc_dispmanx_resource_create( DISPMANX_FRAMEBUF_BITMAP_TYPE,
                                                                         screen_width,
                                                                         screen_height,
                                                                         &native_image_ptr );

      if ( framebuffer_bitmap->resource_handle )
      {
         DISPMANX_UPDATE_HANDLE_T update;

         //start an update
         update = vc_dispmanx_update_start(10);
         if ( update )
         {
            VC_RECT_T src_rect, dest_rect;

            // Need to blat the data across to videocore. Remember the src_rect for the element_add
            // takes u16.16 coords.
            vc_dispmanx_rect_set( &src_rect, 0, 0, screen_width<<16, screen_height<<16 );
            vc_dispmanx_rect_set( &dest_rect, 0, 0, screen_width, screen_height );
            vc_dispmanx_resource_write_data( framebuffer_bitmap->resource_handle,
                                             DISPMANX_FRAMEBUF_BITMAP_TYPE,
                                             framebuffer_bitmap->pitch,
                                             framebuffer_bitmap->data,
                                             &dest_rect );

            framebuffer_bitmap->element_handle = vc_dispmanx_element_add( update,
                                                                          display,
                                                                          3,
                                                                          &dest_rect,
                                                                          framebuffer_bitmap->resource_handle,
                                                                          &src_rect,
                                                                          DISPMANX_FLAGS_ALPHA_SOURCE|DISPMANX_FLAGS_ALPHA_FIXED,
                                                                          60,
									                                      0,
                                                                          VC_IMAGE_ROT0 );
            if (framebuffer_bitmap->element_handle)
               success = 0;

            // This call blocks until the update is done.
            success += vc_dispmanx_update_submit_sync(update);
         }
      }
   }

   return success;
}


/***********************************************************
 * Name: dispmanx_framebuf_create_and_display_boxes
 *
 * Arguments:
 *       DISPMANX_FRAMEBUF_T *framebuffer_bitmap - the framebuffer to display
         DISPMANX_BOXES_T *boxes - the state of the boxes coloured in on the framebuffer
 *
 * Description: A routine to create and displayed boxes of colour in the framebuffer
 *
 * Returns: int - < 0 is fail
 *
 ***********************************************************/
static int dispmanx_framebuf_create_and_display_boxes(DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
      DISPMANX_BOXES_T *boxes,
      const uint32_t screen_width,
      const uint32_t screen_height )
{
   int success = -1; //fail by default
   uint32_t num_boxes_x = 0;
   uint32_t num_boxes_y = 0;
   uint32_t box_index = 0;

   //first, work out how many boxes we can fit on the screen
   num_boxes_x = (screen_width / ( DISPMANX_FRAMEBUF_BOX_WIDTH + (DISPMANX_FRAMEBUF_BOX_BORDER_SIZE * 2)));

   num_boxes_y = (screen_height / ( DISPMANX_FRAMEBUF_BOX_HEIGHT + (DISPMANX_FRAMEBUF_BOX_BORDER_SIZE * 2)));

   //store the number of boxes
   boxes->num_boxes = (num_boxes_x * num_boxes_y);

   //now, malloc some memory to put the box data in
   boxes->box_data = calloc( 1, sizeof( DISPMANX_FRAMEBUF_BOX_T ) * boxes->num_boxes );

   if ( NULL != boxes->box_data)
   {
      uint32_t y_count = 0;
      uint32_t x_count = 0;
      uint16_t current_y_pos = DISPMANX_FRAMEBUF_BOX_BORDER_SIZE;

      //set the coordinates of the boxes
      for ( y_count = 0; y_count < num_boxes_y; y_count++ )
      {
         uint16_t current_x_pos = DISPMANX_FRAMEBUF_BOX_BORDER_SIZE;

         for ( x_count = 0; x_count < num_boxes_x; x_count++ )
         {
            DISPMANX_FRAMEBUF_BOX_T *box_ptr = &(boxes->box_data[ box_index ]);

            //store the x + y positions
            box_ptr->y = current_y_pos;
            box_ptr->x = current_x_pos;

            //store the colour index
            box_ptr->colour_index = (x_count % DISPMANX_FRAMEBUF_NUM_COLOURS);

            //render the box!
            success = dispmanx_framebuf_draw_box(  framebuffer_bitmap,
                                                   box_ptr,
                                                   screen_width,
                                                   screen_height );

            //increase the x pos
            current_x_pos += (DISPMANX_FRAMEBUF_BOX_BORDER_SIZE * 2) + DISPMANX_FRAMEBUF_BOX_WIDTH;

            //inc the box index
            box_index++;
         }

         current_y_pos += (DISPMANX_FRAMEBUF_BOX_BORDER_SIZE * 2) + DISPMANX_FRAMEBUF_BOX_HEIGHT;
      }
   }

   return success;
}


/***********************************************************
 * Name: dispmanx_framebuf_update_framebuffer
 *
 * Arguments:
 *       DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
         const uint32_t x,
         const uint32_t y,
         const uint32_t width,
         const uint32_t height
 *
 * Description: Routine to update an area of the framebuffer on videocore
 *
 * Returns: int - < 0 is fail
 *
 ***********************************************************/
static int dispmanx_framebuf_update_framebuffer(DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
      const uint16_t x,
      const uint16_t y,
      const uint16_t width,
      const uint16_t height,
      const uint32_t screen_width,
      const uint32_t screen_height )
{
   int success = -1; //fail by default

   if (NULL != framebuffer_bitmap)
   {
      DISPMANX_UPDATE_HANDLE_T update;

      //start an update
      update = vc_dispmanx_update_start(10);
      if ( update )
      {
         //assert( (x + width) <= screen_width );
         //assert( (y + height) <= screen_height );

         VC_RECT_T rect;
         
         // Need to blat the data across to videocore.
         vc_dispmanx_rect_set( &rect, x, y, width, height );
         vc_dispmanx_resource_write_data( framebuffer_bitmap->resource_handle,
                                          DISPMANX_FRAMEBUF_BITMAP_TYPE,
                                          framebuffer_bitmap->pitch,
                                          framebuffer_bitmap->data,
                                          &rect );
         //mdelay(10);
         //mark the bitmap as having been updated
         success = vc_dispmanx_element_modified( update,
                                                 framebuffer_bitmap->element_handle,
                                                 &rect );
         //mdelay(10);
         // This call blocks until the update is done.
         success += vc_dispmanx_update_submit_sync(update);
      }
   }
   else
   {
      //bad parameters
      assert( 0 );
   }

   return success;
}


/***********************************************************
 * Name: dispmanx_framebuf_draw_box
 *
 * Arguments:
 *       DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
         DISPMANX_FRAMEBUF_BOX_T *box
 *
 * Description: Routine to draw a box
 *
 * Returns: int - < 0 is fail
 *
 ***********************************************************/
static int dispmanx_framebuf_draw_box( DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
                                       DISPMANX_FRAMEBUF_BOX_T *box,
                                       const uint32_t screen_width,
                                       const uint32_t screen_height )
{
   int success = -1; //fail by default

   if ( (NULL != framebuffer_bitmap) && (NULL != box) )
   {
      //draw a box by filling in an area with colour
      success = dispmanx_framebuf_fill_bitmap(  framebuffer_bitmap,
                box->x,
                box->y,
                DISPMANX_FRAMEBUF_BOX_WIDTH,
                DISPMANX_FRAMEBUF_BOX_HEIGHT,
                dispmanx_framebuf_colours[ box->colour_index ] );
   }

   return success;
}


/***********************************************************
 * Name: dispmanx_framebuf_fill_bitmap
 *
 * Arguments:
 *       DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
         const uint32_t x,
         const uint32_t y,
         const uint32_t width,
         const uint32_t height
         const uint32_t colour
 *
 * Description: Routine to update part of the framebuffer by filling it in with a colour
 *
 * Returns: int - < 0 is fail
 *
 ***********************************************************/
static int dispmanx_framebuf_fill_bitmap( DISPMANX_FRAMEBUF_T *framebuffer_bitmap,
      const uint32_t x,
      const uint32_t y,
      const uint32_t width,
      const uint32_t height,
      const uint32_t colour )
{
   int success = -1; //fail by default

   //check the parameters
   if (NULL != framebuffer_bitmap)
   {
      uint32_t height_count = 0;
      uint32_t width_count = 0;

      assert( x != 0 );
      assert( y != 0 );

      //for every row of data
      for ( height_count = y; height_count < (height + y); height_count++)
      {
         //calculate a ptr to the start of each row of data
         uint32_t *row_ptr = (uint32_t *) ((uint8_t *)framebuffer_bitmap->data + (framebuffer_bitmap->pitch * height_count) );

         //fill in each pixel in the row
         for ( width_count = x; width_count < (width + x); width_count++)
         {
            *(row_ptr + width_count) = colour;
         }
      }

      //success!
      success = 0;
   }

   return success;
}
/***********************************************************
 * Name: dispmanx_framebuf_update_framebuffer
 *
 * Arguments:
 *       DISPMANX_FRAMEBUF_T *framebuffer_bitmap
 *
 * Description: Routine to update an area of the framebuffer on videocore
 *
 * Returns: int - < 0 is fail
 *
 ***********************************************************/
static int dispmanx_framebuf_undisplay(DISPMANX_FRAMEBUF_T *framebuffer_bitmap )
{
   int success = 0;
   DISPMANX_UPDATE_HANDLE_T update;

   update = vc_dispmanx_update_start( 10 );
   success = vc_dispmanx_element_remove( update, framebuffer_bitmap->element_handle );
   success += vc_dispmanx_update_submit_sync( update );

   success += vc_dispmanx_resource_delete( framebuffer_bitmap->resource_handle );

   return success;
}


