/*
 *  linux/drivers/mmc/s3c2413mci.h - Samsung S3C2413 SDI Interface driver
 *
 * $Id: s3cmci.h,v 1.3 2007/08/15 13:30:59 rst Exp $
 *
 *  Copyright (C) 2004 Thomas Kleffel, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


/* FIXME: DMA Resource management ?! */
#ifdef CONFIG_ARCH_MDIRAC3
	#define S3C_SDI_DMA 3
#else
	#define S3C_SDI_DMA 0
#endif

enum s3c_sdi_waitfor {
	COMPLETION_NONE,
	COMPLETION_CMDSENT,
	COMPLETION_RSPFIN,
	COMPLETION_XFERFINISH,
	COMPLETION_XFERFINISH_RSPFIN,
};

struct s3c_sdi_host {
	void __iomem		*base;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;
	struct mmc_host		*mmc;
	struct clk		*clk;
	struct resource		*mem;

	unsigned int		data_xfered;

	struct timer_list       timer;

	int			irq;
	int			irq_cd;
	int			dma;
	int			subchannel;
	int			dma_dir;

	spinlock_t		complete_lock;
	struct completion	complete_request;
	struct completion	complete_dma;
	enum s3c_sdi_waitfor	complete_what;

	struct scatterlist	*sg_ptr;
	unsigned int		sg_len;
	unsigned int		sg_off;
	unsigned int		size;
	int			ena_2410_workaround;
	unsigned int		prescaler;
	struct work_struct	irq_cd_wq;
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	struct notifier_block	freq_transition;
	struct notifier_block	freq_policy;
#endif
};
