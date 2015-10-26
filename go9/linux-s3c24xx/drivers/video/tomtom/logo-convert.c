/* drivers/video/tomtom/logo-convert.c
 *
 * Convert 24 bit BMP logos to TomTom GO display format.
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Author: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAXWIDTH  4096
#define MAXHEIGHT 4096

#define PACKED __attribute__ ((__packed__))

struct bmpfilehdr {
	uint8_t  magic1;		/* B */
	uint8_t  magic2;		/* M */
	uint32_t size;			/* size of file */
	uint16_t res1;			/* reserved */
	uint16_t res2;			/* reserved */
	uint32_t offbits;		/* offset to bits */
} PACKED;

struct bmpinfohdr {
	uint32_t size;			/* size of bmpinfohdr */
	int32_t  width;			/* width of picture */
	int32_t  height;		/* height of picture */
	uint16_t planes;		/* number of planes */
	uint16_t bpp;			/* bits per pixel */
	uint32_t compr;			/* compression type */
	uint32_t szimage;		/* size of image */
	int32_t  xppm;			/* pixels per meter */
	int32_t  yppm;			/* pixels per meter */
	uint32_t palused;		/* palette entries used */
	uint32_t palreq;		/* palette entries required */
} PACKED;

struct bmppixel {
	uint8_t b;
	uint8_t g;
	uint8_t r;
} PACKED;

const char *progname;

int error(const char *fmt, ...)
{
	va_list ap;

	fprintf(stderr, "%s: ERROR: ", progname);
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, ": %s\n", strerror(errno));

	return EXIT_FAILURE;
}

int errorx(const char *fmt, ...)
{
	va_list ap;

	fprintf(stderr, "%s: ERROR: ", progname);
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");

	return EXIT_FAILURE;
}

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((((uint16_t) r + 4) >> 3) << 11) | ((((uint16_t) g + 2) >> 2) << 5) | ((((uint16_t) b + 4) >> 3) << 0);
}

int logo2c(const char *filename, int orientation)
{
	FILE *fp;
	long flen;
	struct bmpfilehdr bfh;
	struct bmpinfohdr bih;
	uint32_t imgsize;
	uint16_t *picbuf;
	int32_t x;
	int32_t y;
	struct bmppixel pix;
	uint16_t npix;

	if (orientation != 0 && orientation != 90 && orientation != 180 && orientation != 270)
		return errorx("Orientation must be in {0, 90, 180, 270}");

	fp = fopen(filename, "rb");
	if (fp == NULL)
		return error("Unable to open `%s'", filename);

	if (fseek(fp, 0, SEEK_END) == -1 || (flen = ftell(fp)) == -1L || fseek(fp, 0, SEEK_SET) == -1)
		return error("Unable to get length of `%s'", filename);

	if (flen < (long) (sizeof bfh + sizeof bih))
		return errorx("`%s' is too short to be a BMP file.", filename);

	/* Check BMP headers */
	if (fread(&bfh, sizeof bfh, 1, fp) != 1)
		return error("Unable to read BMP file header from `%s'", filename);
	if (bfh.magic1 != 'B' || bfh.magic2 != 'M')
		return errorx("`%s' is not a BMP file.", filename);
	if (bfh.size != (uint32_t) flen)
		return errorx("Invalid size %u (should be %ld) in BMP file header of `%s'.", bfh.size, flen, filename);
	if (bfh.offbits != sizeof bfh + sizeof bih)
		return errorx("Invalid offbits in BMP file header of `%s'.", filename);
	if (fread(&bih, sizeof bih, 1, fp) != 1)
		return error("Unable to read BMP info header from `%s'", filename);
	if (bih.size != sizeof bih || bih.planes != 1)
		return errorx("Invalid BMP info header in `%s'.", filename);
	if (bih.width <= 0 || bih.width > MAXWIDTH)
		return errorx("Width %d of BMP too large (max is %d).", bih.width, MAXWIDTH);
	if (bih.height <= 0 || bih.height > MAXHEIGHT)
		return errorx("Height %d of BMP too large (max is %d).\n", bih.height, MAXHEIGHT);
	if (bih.bpp != 24)
		return errorx("Only 24bpp BMP files are supported.");
	imgsize = bih.width * bih.height * bih.bpp / 8;
	if (bfh.offbits + imgsize != bfh.size)
		return errorx("Inconsistent BMP header in `%s'.", filename);
	if (bih.compr != 0)
		return errorx("Only uncompressed BMP files are supported.");
	if (bih.szimage != 0 && bih.szimage != imgsize)
		return errorx("Invalid image size in BMP info header of `%s'.", filename);
	if (bih.palused != 0 || bih.palreq != 0)
		return errorx("Invalid palette in BMP info header of `%s'.", filename);
	if ((picbuf = malloc(imgsize)) == NULL)
		return errorx("Unable to allocate buffer of %u bytes.", imgsize);

	/* BMP files are stored flipped, so y goes from bottom to top */
	for (y = bih.height - 1; y >= 0; --y) {
		for (x = 0; x < bih.width; ++x) {
			if (fread(&pix, sizeof pix, 1, fp) != 1)
				return error("Unable to read from `%s'", filename);
			npix = rgb565(pix.r, pix.g, pix.b);
			picbuf[y * bih.width + x] = npix;
		}
	}
	if (getc(fp) != EOF)
		return errorx("Trailing garbage at end of `%s'.", filename);
	if (ferror(fp))
		return error("Unable to read from `%s'", filename);
	fclose(fp);

	fprintf(stdout, "static __u16 logo[] __initdata = {\n");
	switch (orientation) {
	case 0:
		for (y = 0; y <= bih.height - 1; ++y) {
			putc('\t', stdout);
			for (x = 0; x <= bih.width - 1; ++x) {
				fprintf(stdout, "0x%04x,", picbuf[y * bih.width + x]);
			}
			putc('\n', stdout);
		}
		break;
	case 90:
		for (x = bih.width - 1; x >= 0; --x) {
			putc('\t', stdout);
			for (y = 0; y <= bih.height - 1; ++y) {
				fprintf(stdout, "0x%04x,", picbuf[y * bih.width + x]);
			}
			putc('\n', stdout);
		}
		break;
	case 180:
		for (y = bih.height - 1; y >= 0; --y) {
			putc('\t', stdout);
			for (x = bih.width - 1; x >= 0; --x) {
				fprintf(stdout, "0x%04x,", picbuf[y * bih.width + x]);
			}
			putc('\n', stdout);
		}
		break;
	case 270:
		for (x = 0; x <= bih.width - 1; ++x) {
			putc('\t', stdout);
			for (y = bih.height - 1; y >= 0; --y) {
				fprintf(stdout, "0x%04x,", picbuf[y * bih.width + x]);
			}
			putc('\n', stdout);
		}
		break;
	}
	fprintf(stdout, "};\n");
	free(picbuf);

	return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
	int orientation;

	progname = argv[0];
	if (argc < 2 || argc > 3) {
		fprintf(stderr, "Usage: %s filename [orientation]\n", progname);
		return EXIT_FAILURE;
	} else if (argc == 2) {
		orientation = 0;
	} else if (sscanf(argv[2], "%d", &orientation) != 1) {
		return errorx("Invalid orientation `%s'", argv[2]);
	}

	return logo2c(argv[1], orientation);
}

/* EOF */
