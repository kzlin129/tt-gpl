/**
 * \file
 * Command line utility to dump touchpad events to stdout.
 *
 * Copyright (c) 2007-2009 Synaptics Incorporated
 *
 * This file is triple licensed under the GPL2, MPL, and Apache licenses.
 */
#include <stdio.h>
#include "linux/input.h"
#include <fcntl.h>
#include <unistd.h>


int
main(void)
{
    int fd, retval;
    struct input_event event;
    int X1, X2, X3, X4, X5, Y1, Y2, Y3, Y4, Y5, Z1, Z2, Z3, Z4, Z5, W1, W2, W3, W4, W5, F;
    int B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12;
    int GESTURE;
    int XFLICKDISTANCE, YFLICKDISTANCE, FLICKTIME, PINCHMOTION;

    F = 0;
    X1 = X2 = X3 = X4 = X5 = 0;
    Y1 = Y2 = Y3 = Y4 = Y5 = 0;
    Z1 = Z2 = Z3 = Z4 = Z5 = 0;
    W1 = W2 = W3 = W4 = W5 = 0;
    B1 = B2 = B3 = B4 = B5 = B6 = B7 = B8 = B9 = B10 = B11 = B12 = 0;
    GESTURE = 0;
    XFLICKDISTANCE = YFLICKDISTANCE = FLICKTIME = PINCHMOTION = 0;

    fd = open("/dev/input/event3", O_RDONLY);

    while(1) {
        retval = read(fd, &event, sizeof(event));
        if(retval > 0) {
            if(event.type == EV_SYN) {
                printf("\rX1 %8d, Y1 %8d, Z1 %3d, W1 %2d, X2 %8d, Y2 %8d, Z2 %3d, W2 %2d, X3 %8d, Y3 %8d, Z3 %3d, W3 %2d, X4 %8d, Y4 %8d, Z4 %3d, W4 %2d, X5 %8d, Y5 %8d, Z5 %3d, W5 %2d, F %d \r",
                    X1, Y1, Z1, W1, X2, Y2, Z2, W2, X3, Y3, Z3, W3, X4, Y4, Z4, W4, X5, Y5, Z5, W5, F);
            
                if ( B1 || B2 || B3 || B4 || B5 || B6 || B7 || B8 || B9 || B10 || B11 || B12 )
                  printf("\rB1 %2d, B2 %2d, B3 %2d, B4 %2d, B5 %2d, B6 %2d, B7 %2d, B8 %2d, B9 %2d, B10 %2d, B11 %2d, B12 %2d \r",
                      B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12);
                B1 = B2 = B3 = B4 = B5 = B6 = B7 = B8 = B9 = B10 = B11 = B12 = 0;
            
                if (GESTURE)
                  printf("\rGESTURE %8x, XFLICKDISTANCE %8d YFLICKDISTANCE %8d FLICKTIME %8d PINCHMOTION %8d \r", GESTURE, XFLICKDISTANCE, YFLICKDISTANCE, FLICKTIME, PINCHMOTION);
		fflush(stdout);
  
                GESTURE = 0;
                XFLICKDISTANCE = YFLICKDISTANCE = FLICKTIME = PINCHMOTION = 0;
            } else {
                switch(event.type) {
                    case EV_ABS:
                        switch(event.code) {
                            case ABS_X                  : X1 = event.value; break;
                            case ABS_Y                  : Y1 = event.value; break;
                            case ABS_PRESSURE           : Z1 = event.value; break;
                            case ABS_TOOL_WIDTH         : W1 = event.value; break;
                            case ABS_FINGER2_X          : X2 = event.value; break;
                            case ABS_FINGER2_Y          : Y2 = event.value; break;
                            case ABS_FINGER2_PRESSURE   : Z2 = event.value; break;
                            case ABS_FINGER2_TOOL_WIDTH : W2 = event.value; break;
                            case ABS_FINGER3_X          : X3 = event.value; break;
                            case ABS_FINGER3_Y          : Y3 = event.value; break;
                            case ABS_FINGER3_PRESSURE   : Z3 = event.value; break;
                            case ABS_FINGER3_TOOL_WIDTH : W3 = event.value; break;
                            case ABS_FINGER4_X          : X4 = event.value; break;
                            case ABS_FINGER4_Y          : Y4 = event.value; break;
                            case ABS_FINGER4_PRESSURE   : Z4 = event.value; break;
                            case ABS_FINGER4_TOOL_WIDTH : W4 = event.value; break;
                            case ABS_FINGER5_X          : X5 = event.value; break;
                            case ABS_FINGER5_Y          : Y5 = event.value; break;
                            case ABS_FINGER5_PRESSURE   : Z5 = event.value; break;
                            case ABS_FINGER5_TOOL_WIDTH : W5 = event.value; break;
                            case ABS_FINGERS            : F  = event.value; break;
                            case ABS_MISC               :
                              if ( GESTURE & 0x10 ) { /* for a flick we need to store the subsequent flick values as they come in order */
                                if (XFLICKDISTANCE == 0)
                                  XFLICKDISTANCE = event.value;
                                if (YFLICKDISTANCE == 0)
                                  YFLICKDISTANCE = event.value;
                                if (FLICKTIME == 0)
                                  FLICKTIME = event.value;
                              } else if ( GESTURE & 0x40 ) { /* for a pinch we need to store the subsequent pinch motion */
                                PINCHMOTION = event.value;
                              }
                              break;
                        }
                        break;

                    case EV_KEY:
                       switch(event.code) {
                            case BTN_0:                 : B1  = event.value; break;
                            case BTN_1:                 : B2  = event.value; break;
                            case BTN_2:                 : B3  = event.value; break;
                            case BTN_3:                 : B4  = event.value; break;
                            case BTN_4:                 : B5  = event.value; break;
                            case BTN_5:                 : B6  = event.value; break;
                            case BTN_6:                 : B7  = event.value; break;
                            case BTN_7:                 : B8  = event.value; break;
                            case BTN_8:                 : B9  = event.value; break;
                            case BTN_9:                 : B10 = event.value; break;
                            case BTN_LEFT:              : B11 = event.value; break;
                            case BTN_RIGHT:             : B12 = event.value; break;
                       }
                       break;

                    case EV_MISC:
                       switch(event.code) {
                           case MISC_GESTURE:
                             GESTURE = event.value;
                             break;
                       }
                       break;

                    default: printf("Uncoded"); break;
                }
            }
        }
    }

    close(fd);
    return 1;
}
