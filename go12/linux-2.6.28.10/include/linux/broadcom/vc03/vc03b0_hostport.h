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

#ifndef VC03B0_HOSTPORT_H
#define VC03B0_HOSTPORT_H

#include <stdarg.h>
#include <linux/types.h>

#include <cfg_global.h>
#if (CFG_GLOBAL_CPU == MIPS32)
// Tempoary kludge until the DMA stuff gets moved out of vc_drv.c
#define USE_DMA 0
#else
#define USE_DMA 1
#define DMA_LL_SIZE	1024
#define DMA_LLI_SIZE	4096
#endif

// The following defines are used when reading from address 0
// also known as the message status register
#define VC_HOSTPORT_STATUS_M_FLAG           ( 1 << 30 )
#define VC_HOSTPORT_STATUS_N_FLAG           ( 1 << 29 )
#define VC_HOSTPORT_STATUS_T_FLAG           ( 1 << 28 )
#define VC_HOSTPORT_STATUS_UF_FLAG          ( 1 << 27 )
#define VC_HOSTPORT_STATUS_OF_FLAG          ( 1 << 26 )
#define VC_HOSTPORT_STATUS_FS_FLAG          ( 1 << 25 )
#define VC_HOSTPORT_STATUS_GET_LEN(v)       ( 1 + ((( (v) & 0x01FF0000 ) >> 1 ) | ( (v) & 0x00007FFF )))

// The following defines are used when writing to address 0 
// also known as the message parameter register
#define VC_HOSTPORT_PARAM_MSG_TYPE_CTRL     ( 0 << 0 )  // 0
#define VC_HOSTPORT_PARAM_MSG_TYPE_DATA     ( 1 << 0 )  // 1
#define VC_HOSTPORT_PARAM_TERMINATE_DMA     ( 1 << 1 )  // 2
#define VC_HOSTPORT_PARAM_USER_DMA          ( 1 << 2 ); // 4

#if defined( __KERNEL__ )
#include <linux/interrupt.h>
int     vchost_gpioirq_handle(int irq, void *ptr);
int     vchost_request_irq( irq_handler_t irqHandler, void *devId );
void    vchost_free_irq( void *devId );
#endif

// gpio and bus setup
int     vchost_pininit(void);
int     vchost_portinit(void* portcfg);

void vchost_set_interface_for_boot( void );
void vchost_set_interface_for_run( void );


void    vchost_reset(void);

// This is the old interface, which will be going away
void    vchost_gpio_pintype_set(int pin, int pintype, int interrupttype); // __attribute__ ((deprecated));
void    vchost_gpio_pinval_set(int pin, int pinval)                     ; // __attribute__ ((deprecated));

// This is the new interface (which mirrors gpiolib)
int     vchost_gpio_request( unsigned pin, const char *label );
int     vchost_gpio_direction_input( unsigned pin );
int     vchost_gpio_direction_output( unsigned pin, int initialValue );
void    vchost_gpio_set_value( unsigned pin, int value );
int     vchost_gpio_get_value( unsigned pin );

void    vchost_vlog( const char *function, int logType, const char *fmt, va_list args );
void    vchost_log( const char *function, int logType, const char *fmt, ... );

void    vchost_delay_msec( unsigned msec );
void    vc_host_delay_msec( unsigned msec ); // __attribute__ ((deprecated));

// host port low level write
int     vchost_writedata(uint8_t* ptr, int size_bytes);
void    vchost_writeparam(uint16_t data);
int     vchost_writemsg(void* msg2, uint32_t len_bytes, int8_t ctrl);

// host port low level read
uint16_t vchost_readdata(void);
uint16_t vchost_readparam(void);
int      vchost_readmsg(void* msg2, uint32_t maxlen_bytes, uint8_t* ctrl);

// host port read status
int     vchost_readavail(int pin); // return 1 is HAT pin is asserted 
int     vchost_msgavail(void);     // return -1 if message is not available, 0 or 1 if ctrl or data message is available

// read/write from/to vc3 memory address
int vc_host_send_interrupt(int channel);
uint32_t vc_host_read32(uint32_t vc_addr, int channel);
int vc_host_read_consecutive (void *host_addr, uint32_t vc_addr, int nbytes, int channel);
int vc_host_write_consecutive (uint32_t vc_addr, void *host_addr, int nbytes, int channel);
#define VC_HOST_READ_BYTESWAPPED_32 vc_host_read_byteswapped_32
#define VC_HOST_READ_BYTESWAPPED_16 vc_host_read_consecutive
#define VC_HOST_WRITE_BYTESWAPPED_16 vc_host_write_consecutive
#define VC_HOST_WRITE_BYTESWAPPED_32 vc_host_write_consecutive

#endif // VC03B0_HOSTPORT_H
