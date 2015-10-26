/*****************************************************************************
* Copyright 2004 - 2008 Broadcom Corporation.  All rights reserved.
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




#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/net.h>
#include <linux/socket.h>
#include <linux/types.h>
#include <linux/time.h>

#if defined(CONFIG_BCM_EPTDRIVER_SUPPORT) && defined(CONFIG_NET)
extern long sys_socket(int family, int type, int protocol);
extern long sys_accept(int fd, struct sockaddr *upeer_sockaddr, int *upeer_addrlen);
extern long sys_select(int n, fd_set *inp, fd_set *outp, fd_set *exp, struct timeval *tvp);
extern long sys_close(unsigned int fd);
extern int sock_getsockopt(struct socket *sock, int level, int op, char *optval, int *optlen);

/**
* The purpose of this file is to expose some system socket functions
*/


long socketCreate( int family, int type, int protocol )
{
   return sys_socket( family, type, protocol );
}
EXPORT_SYMBOL(socketCreate);

long socketAccept( int fd, struct sockaddr *upeer_sockaddr, int *upeer_addrlen )
{
   return sys_accept( fd, upeer_sockaddr, upeer_addrlen );
}
EXPORT_SYMBOL(socketAccept);

long socketSelect( int n, fd_set *inp, fd_set *outp, fd_set *exp, struct timeval *tvp )
{
   return sys_select( n, inp, outp, exp, tvp );
}
EXPORT_SYMBOL(socketSelect);

long socketClose( unsigned int fd )
{
   return sys_close( fd );
}
EXPORT_SYMBOL(socketClose);

int socketGetOpt(struct socket *sock, int level, int op, char *optval, int *optlen)
{
   return sock_getsockopt( sock, level, op, optval, optlen );
}
EXPORT_SYMBOL(socketGetOpt);
#endif
