/*
 *  drivers/mtd/nand_ecc512.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This file is the header for the ECC algorithm.
 */

#ifndef __MTD_NAND_ECC512_H__
#define __MTD_NAND_ECC512_H__

struct mtd_info;

/*
 * Calculate 3 byte ECC code for 512 byte block
 */
int nand_calculate_ecc512(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code);

/*
 * Detect and correct a 2 bit error for 512 byte block
 */
int nand_correct_data512(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc);

#endif /* __MTD_NAND_ECC512_H__ */
