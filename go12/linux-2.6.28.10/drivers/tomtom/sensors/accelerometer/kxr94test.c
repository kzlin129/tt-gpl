/* Control program for kxr94 accelerometer */

#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <alloca.h>
#include <err.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "../../../../include/linux/gadc.h"

#ifdef VERBOSE
# define V(x) x
#else
# define V(x) while(0)
#endif

#define TIMEOUT 1
#define REPEATS 20
#define READLEN 128

struct {
	char *path;
	int fd;
}
channels[] = {

#define FD_N (channels[n].fd)
#define PATH_N (channels[n].path)

	{
		.path = "/dev/acc0_x",
		.fd = -1,
	},
	{
		.path = "/dev/acc0_y",
		.fd = -1,
	},
	{
		.path = "/dev/acc0_z",
		.fd = -1,
	},
	{
		.path = "/dev/acc0_aux",
		.fd = -1,
	},
};
#define NUM_CHANNELS (sizeof(channels)/sizeof(channels[0]))

/* Apologies for the following ... */

#define DEFINE_CHANNEL_ACTION(name, command) \
int name##_channel(int n) \
{ \
	V(printf("Channel %d (%s): " #name " ... ", n, channels[n].path)); \
	V(fflush(stdout)); \
	if ((command) < 0) { \
		V(puts("failed")); \
		return -1; \
	} \
	V(puts("OK")); \
	return 0; \
}

#define DEFINE_CHANNEL_IOCTL(name, req, argtype) \
int name##_channel_ioctl(int n, argtype *arg) \
{ \
DEFINE_CHANNEL_ACTION(name, ioctl(FD_N, req, *arg)) \
	CHANNEL_ACTION(name, n); \
}

#define CHANNEL_ACTION(name, n) name##_channel(n)
#define TERMINAL_CHANNEL_ACTION(name, n) \
{ \
	if (CHANNEL_ACTION(name, n)) \
		err(1, "Failed " #name); \
}
#define NONTERMINAL_CHANNEL_ACTION(name, n) \
{ \
	if (CHANNEL_ACTION(name, n)) \
		warn("Failed " #name); \
}

#define CHANNEL_IOCTL(name, n, arg) name##_channel_ioctl(n, arg)

DEFINE_CHANNEL_ACTION(open, FD_N = open(PATH_N, O_RDONLY))
DEFINE_CHANNEL_ACTION(close, close(FD_N))
DEFINE_CHANNEL_IOCTL(set_rate, ADC_SET_SAMPLERATE, unsigned long int)
DEFINE_CHANNEL_IOCTL(get_rate, ADC_GET_SAMPLERATE, unsigned long int)
DEFINE_CHANNEL_IOCTL(get_bufferstatus, ADC_GET_BUFFERSTATUS, unsigned long int)
DEFINE_CHANNEL_IOCTL(get_format, ADC_GET_FORMAT, struct gadc_format *)

int channel_read(int n, struct gadc_sample *buf, size_t count)
{
	ssize_t result;
DEFINE_CHANNEL_ACTION(read, result = read(FD_N, buf, count))
	NONTERMINAL_CHANNEL_ACTION(read, n);
	V(printf("Read %zd bytes\n", result));

	return result/sizeof(struct gadc_sample);
}


int wait_on_channels(fd_set *rd_set)
{
	int n, r;
	fd_set *wr_set = alloca(sizeof(fd_set));
	fd_set *ex_set = alloca(sizeof(fd_set));
	struct timeval timeout = {
		.tv_sec = TIMEOUT,
		.tv_usec = 0,
	};

	FD_ZERO(rd_set);
	for(n=0; n<NUM_CHANNELS; n++)
		FD_SET(FD_N, rd_set);

	FD_ZERO(wr_set);
	FD_ZERO(ex_set);

	V(printf("Select ... "));
	V(fflush(stdout));
	if ((r = select(20, rd_set, wr_set, ex_set, &timeout)) < 0) {
		V(puts("failed"));
		warn("Failed select");
		return -1;
	}
	V(puts("OK"));
	V(printf("Select - %d file descriptors ready\n", r));
	
	return r;
}

int main(int argc, char *argv[])
{
	int n, i, r;
	struct gadc_sample *buf = alloca(READLEN);
	fd_set *rd_set = alloca(sizeof(fd_set));

	for (n=0; n<NUM_CHANNELS; n++) {
		unsigned long int val = 10000;
		TERMINAL_CHANNEL_ACTION(open, n);
		if (CHANNEL_IOCTL(set_rate, n, &val))
			warn("Failed IOCTL");
	}

	for(i=0; i<REPEATS; i++) {
		wait_on_channels(rd_set);

		for (n=0; n<NUM_CHANNELS; n++) {
			if (FD_ISSET(FD_N, rd_set)) {
				r = channel_read(n, buf, READLEN);
				if (r > 0) {
					FILE *f;
#if 1
					while (r-- > 0) {
						printf("%u\t%10d.%.6d\t%#8lx\n", n, buf[r].timestamp.tv_sec, buf[r].timestamp.tv_usec, buf[r].value);
					}
#else
					if ((f = popen("hexdump", "w")) == NULL)
						err(1, "Couldn't open hexdump");
					fwrite(buf, sizeof(struct gadc_sample), r, f);
					pclose(f);
#endif
				}
			}
		}
	}

	for (n=0; n<NUM_CHANNELS; n++) {
		NONTERMINAL_CHANNEL_ACTION(close, n);
	}

	exit(EXIT_SUCCESS);
}

