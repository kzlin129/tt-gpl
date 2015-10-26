/*****************************************************************************
* Copyright 2006 - 2009 Broadcom Corporation.  All rights reserved.
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
 *  include/asm-arm/arch-bcm47xx/pci_emu.h
 *  BCM28XX platform PCI emulation header file.
 */

#ifndef _ASM_BCM47XX_PCI_EMU_H
#define _ASM_BCM47XX_PCI_EMU_H

/* The PCI config space */
#define	PCI_BAR_MAX			6
#define	PCR_RSVDA_MAX		2

typedef struct pci_config_regs {
	u16	vendor;
	u16	device;
	u16	command;
	u16	status;
	u8	rev_id;
	u8	prog_if;
	u16	class;
	u8	cache_line_size;
	u8	latency_timer;
	u8	header_type;
	u8	bist;
	u32	base[PCI_BAR_MAX];
	u32	cardbus_cis;
	u16	subsys_vendor;
 	u16	subsys_id;
	u32	baserom;
	u32	rsvd_a[PCR_RSVDA_MAX];
	u8	int_line;
	u8	int_pin;
	u8	min_gnt;
	u8	max_lat;
	u8	dev_dep[192];
} pci_config_regs;

#define sprom_control	dev_dep[0x88 - 0x40]


struct pci_emu_dev_t;

typedef void (*pci_emu_initf)( const struct pci_emu_dev_t * );
typedef int  (*pci_emu_readf)( u32, u32, u32, int, int, u32 * );
typedef int  (*pci_emu_writef)( u32, u32, u32, int, int, u32 );

typedef struct _bar_info_t
{
	int mapping;
	int size;
} bar_info_t;


typedef struct pci_emu_cfg_t
{
	/* Emulated PCI configuration registers
	 */
	u16		device;		/* PCI device ID */
	u8		prog_if;	/* Class - Program Interface */
	u16		class;		/* Class - Base and Sub Class */
	u8		header;		/* Header Type */
	u8		latency_timer;	/* Latency Timer */
	u8		int_line;   /* Interrupt line */
	u8		int_pin;	/* Interrupt pin */
	bar_info_t bars[PCI_BAR_MAX];  /* BAR related information */

	/* PCI configuration register access functions 
	 */
	pci_emu_readf	read;
	pci_emu_writef	write;

	/* Emulated PCI configuration space 
	 */
	pci_config_regs	pci_cfg_regs;
} pci_emu_cfg_t;	


typedef struct pci_emu_dev_t	
{
	u32				dev_base;		/* Device base address */
	pci_emu_initf	init;			/* Device init function */
	int				num_funcs;		/* Number of PCI functions from this device */
	pci_emu_cfg_t *	pci_emu_cfgs;	/* PCI function(s) configuration array */
} pci_emu_dev_t;	

#endif /* _ASM_BCM47XX_PCI_EMU_H */
