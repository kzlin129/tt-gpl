/*
 * ltt/probes/fs-trace.c
 *
 * FS tracepoint probes.
 */

#include <linux/module.h>
#include <linux/buffer_head.h>
#include <trace/fs.h>

void probe_fs_buffer_wait_start(struct buffer_head *bh)
{
	trace_mark_tp(fs_buffer_wait_start, fs_buffer_wait_start,
		probe_fs_buffer_wait_start, "bh %p", bh);
}

void probe_fs_buffer_wait_end(struct buffer_head *bh)
{
	trace_mark_tp(fs_buffer_wait_end, fs_buffer_wait_end,
		probe_fs_buffer_wait_end, "bh %p", bh);
}

void probe_fs_exec(char *filename)
{
	trace_mark_tp(fs_exec, fs_exec, probe_fs_exec, "filename %s", filename);
}

void probe_fs_ioctl(unsigned int fd, unsigned int cmd, unsigned long arg)
{
	trace_mark_tp(fs_ioctl, fs_ioctl, probe_fs_ioctl,
		"fd %u cmd %u arg %lu", fd, cmd, arg);
}

void probe_fs_open(int fd, char *filename)
{
	trace_mark_tp(fs_open, fs_open, probe_fs_open,
		"fd %d filename %s", fd, filename);
}

void probe_fs_close(unsigned int fd)
{
	trace_mark_tp(fs_close, fs_close, probe_fs_close, "fd %u", fd);
}

void probe_fs_lseek(unsigned int fd, long offset, unsigned int origin)
{
	trace_mark_tp(fs_lseek, fs_lseek, probe_fs_lseek,
		"fd %u offset %ld origin %u", fd, offset, origin);
}

void probe_fs_llseek(unsigned int fd, loff_t offset, unsigned int origin)
{
	trace_mark_tp(fs_llseek, fs_llseek, probe_fs_llseek,
		"fd %u offset %lld origin %u", fd,
		(long long)offset, origin);
}

void probe_fs_read(unsigned int fd, char __user *buf, size_t count,
		ssize_t ret)
{
	trace_mark_tp(fs_read, fs_read, probe_fs_read,
		"fd %u count %zu", fd, count);
}

void probe_fs_write(unsigned int fd, const char __user *buf,
		size_t count, ssize_t ret)
{
	trace_mark_tp(fs_write, fs_write, probe_fs_write,
		"fd %u count %zu", fd, count);
}

void probe_fs_pread64(unsigned int fd, char __user *buf, size_t count,
		loff_t pos, ssize_t ret)
{
	trace_mark_tp(fs_pread64, fs_pread64, probe_fs_pread64,
		"fd %u count %zu pos %llu",
		fd, count, (unsigned long long)pos);
}

void probe_fs_pwrite64(unsigned int fd, const char __user *buf,
		size_t count, loff_t pos, ssize_t ret)
{
	trace_mark_tp(fs_pwrite64, fs_pwrite64, probe_fs_pwrite64,
		"fd %u count %zu pos %llu",
		fd, count, (unsigned long long)pos);
}

void probe_fs_readv(unsigned long fd, const struct iovec __user *vec,
		unsigned long vlen, ssize_t ret)
{
	trace_mark_tp(fs_readv, fs_readv, probe_fs_readv,
		"fd %lu vlen %lu", fd, vlen);
}

void probe_fs_writev(unsigned long fd, const struct iovec __user *vec,
		unsigned long vlen, ssize_t ret)
{
	trace_mark_tp(fs_writev, fs_writev, probe_fs_writev,
		"fd %lu vlen %lu", fd, vlen);
}

void probe_fs_select(int fd, s64 timeout)
{
	trace_mark_tp(fs_select, fs_select, probe_fs_select,
		"fd %d timeout #8d%lld", fd, (long long)timeout);
}

void probe_fs_poll(int fd)
{
	trace_mark_tp(fs_pollfd, fs_poll, probe_fs_poll,
		"fd %d", fd);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("FS Tracepoint Probes");
