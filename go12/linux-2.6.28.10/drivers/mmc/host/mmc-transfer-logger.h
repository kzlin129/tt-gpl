/*
 *  linux/drivers/mmc/host/mmc-transfer-logger.h - MMC transfer logger tool
 */

void transfer_logger_log_transfer(struct mmc_host *host,
 unsigned long start, unsigned int blksz,
 unsigned long bytes_xfered, bool write);

int transfer_logger_add(struct mmc_host *host);

int transfer_logger_remove(struct mmc_host *host);
