/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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
*****************************************************************************
*
*  trample.h
*
*  PURPOSE:
*
*     Test code to detect some specific memory tramples.
*
*  NOTES:
*
*****************************************************************************/


#if !defined( TRAMPLE_H )
#define  TRAMPLE_H

void TestForTrample( const char *fileName, int lineNum );

#define  TESTFORTRAMPLE()  TestForTrample( __FILE__,  __LINE__ );

#endif

