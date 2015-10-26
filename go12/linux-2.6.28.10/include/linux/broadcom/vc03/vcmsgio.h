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



#ifndef VCGFX_H
#define VCGFX_H

int vc_init_service(int serviceID);
int vc_msgio_init(int inum);
int vc_msgio_write_blocking(msgio_write_blocking_t* arg);
int vc_msgio_send_command_blocking(msgio_send_cmd_blocking_t* arg);
int vc_msgio_write_flush(int ogles_inum);
int vc_msgio_input_bytes_available(int inum);
int vc_msgio_read_blocking(msgio_read_blocking_t* arg);
int vc_msgio_read_header(msgio_read_header_t* arg);
int vc_msgio_read_flush(int ogles_inum);

#endif // VCGFX_H
