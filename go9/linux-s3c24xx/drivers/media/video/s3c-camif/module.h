/*
 * Camera driver for s3c241x camera interface, with ov camchip.
 *
 *              This program is free software; you can redistribute it and/or
 *              modify it under the terms of the GNU General Public License
 *              as published by the Free Software Foundation; either version
 *              2 of the License, or (at your option) any later version.
 */

void s3c_cam_set_clockpolicy(int freq_mhz, struct s3c_cam *cam);
int s3c_cam_check_clockpolicy(int freq_mhz, struct s3c_cam *cam);
