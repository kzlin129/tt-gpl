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



	.text
	.section .text.start, "ax"
	.set noreorder
	.set mips3

	/* Reserved */
	.word 0x00000000
	.word 0x00000000

	/* Magic words identifying a software image */
	.word	0xABCD1234
	.word 0xDEADBEEF

	/* Reserved */
	.word 0x00000000

	/* Image header version */
	.word	0x00000001

	/* image start and size */
	.word	_image_start
	.word	_image_size

	/* Compression */
	.word 0x00000001

	/* CRC32 over uncompressed image */
#include "kImage.crc32"
	//.word CRC32

	/* start of kernel and entrypoint in uncompressed image */
	.word	_kernel_start
	.word	_kernel_entry

	/* Here we have room for future flags */
	.org	0x30
reldate:
	.word	TIMESTAMP

	.org	0x40
release:
	.string VERSION
