#ifndef _LTT_TYPE_SERIALIZER_H
#define _LTT_TYPE_SERIALIZER_H

#include <linux/ltt-tracer.h>

/*
 * largest_align must be non-zero, equal to the minimum between the largest type
 * and sizeof(void *).
 */
extern void _ltt_specialized_trace(void *probe_data,
		void *serialize_private, unsigned int data_size,
		unsigned int largest_align);

/*
 * Statically check that 0 < largest_align < sizeof(void *) to make sure it is
 * dumb-proof. It will make sure 0 is changed into 1 and unsigned long long is
 * changed into sizeof(void *) on 32-bit architectures.
 */
static inline void ltt_specialized_trace(void *probe_data,
		void *serialize_private, unsigned int data_size,
		unsigned int largest_align)
{
	largest_align = min_t(unsigned int, largest_align, sizeof(void *));
	largest_align = max_t(unsigned int, largest_align, 1);
	_ltt_specialized_trace(probe_data, serialize_private, data_size,
		largest_align);
}

/*
 * Type serializer definitions.
 */

/*
 * Return size of structure without end-of-structure padding.
 */
#define serialize_sizeof(type)	offsetof(typeof(type), end_field)

struct serialize_long_int {
	unsigned long f1;
	unsigned int f2;
	unsigned char end_field[0];
} LTT_ALIGN;

struct serialize_int_int_long {
	unsigned int f1;
	unsigned int f2;
	unsigned long f3;
	unsigned char end_field[0];
} LTT_ALIGN;

struct serialize_long_long_long {
	unsigned long f1;
	unsigned long f2;
	unsigned long f3;
	unsigned char end_field[0];
} LTT_ALIGN;

struct serialize_long_long_int {
	unsigned long f1;
	unsigned long f2;
	unsigned int f3;
	unsigned char end_field[0];
} LTT_ALIGN;

#endif /* _LTT_TYPE_SERIALIZER_H */
