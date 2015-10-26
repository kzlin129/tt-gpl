#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#ifndef N_PPS
#define N_PPS __N_PPS
#endif

void usage(char *name)
{
	fprintf(stderr, "usage: %s <ttyS>\n", name);

	exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
	int fd;
	int ldisc = N_PPS;
	int ret;

	if (argc < 2)
		usage(argv[0]);

	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		perror("open");
		exit(EXIT_FAILURE);
	}

	ret = ioctl(fd, TIOCSETD, &ldisc);
	if (ret < 0) {
		perror("ioctl(TIOCSETD)");
		exit(EXIT_FAILURE);
	}

	pause();

	return 0;
}
