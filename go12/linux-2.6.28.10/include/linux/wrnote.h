/* wrnote.h - Wind River ELF note header */

/*
 * Copyright (c) 2007 Wind River Systems, Inc.
 */

/*
modification history
--------------------
01a,04jul07,fle  written
*/

#ifndef __INCwrnoteh
#define __INCwrnoteh

#include <linux/elf.h>

#define NT_WR_KERNEL_INFO	50
#define NT_WR_PRPSINFO		51
#define NT_WR_PRSTATUS		52

#define WR_ELF_CHAR		1	/* signed 8 bits value */
#define WR_ELF_UCHAR		2	/* unsigned 8 bits value */
#define WR_ELF_INT16		3	/* signed 16 bits value */
#define WR_ELF_UINT16		4	/* unsigned 16 bits value */
#define WR_ELF_INT32		5	/* signed 32 bits value */
#define WR_ELF_UINT32		6	/* unsigned 32 bits value */
#define WR_ELF_INT64		7	/* signed 64 bits value */
#define WR_ELF_UINT64		8	/* unsigned 64 bits value */
#define WR_ELF_FLOAT32		9	/* 32 bits floating point value */
#define WR_ELF_FLOAT64		10	/* 64 bits floating point value */

struct wr_elf_note_data {
	Elf32_Word type;	/* type of the data to read */
	Elf32_Word count;	/* number of data to read ( > 1 means array) */
	void *data;		/* note data */
};

extern void free_wr_note_data(void);

extern void *get_wr_note(int type,	/* type of the input data */
			 void *data	/* input data to write to note data */
    );

extern int get_wr_note_size(int type,	/* type of input data */
			    void *data	/* input data */
    );

#endif /* __INCwrnoteh */
