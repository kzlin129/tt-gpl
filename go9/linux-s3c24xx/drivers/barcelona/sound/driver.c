/*
 *
 *
 * Copyright (c) 2005 TomTom BV
 * Written by: Thomas Kleffel <tk@maintech.de>
 *
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/soundcard.h>
#include <linux/vmalloc.h>

#include <asm/uaccess.h>
#include <asm/irq.h>

#include <barcelona/Barc_snd.h>
#include <barcelona/Barc_Gpio.h>
#include <barcelona/gopins.h>
#include <barcelona/debug.h>
#include <linux/cpufreq.h>

#include "iis.h"
#include "dma.h"
#include "codec.h"

#include <asm/arch/regs-iis.h>
#include <asm/io.h>

#define PFX "CoolSound: "

#define PK_MAGIC(...) do {} while (0);
#define PK_DBG PK_DBG_FUNC
#if 0
#define SANITY_CHECK \
	do { \
		if (destination == NULL || source == NULL || samples == 0 || samples > 10000) { \
			PK_WARN("Unsane input, dest=%p, src=%p, smp=%zu\n", destination, source, samples); \
			return; \
		} \
	} while (0)
#else
#define SANITY_CHECK do {} while (0)
#endif

#define COOLSOUND_NATIVE_MAJOR 244
#define COOLSOUND_OSSEMU_MAJOR 14

#define CSB_MAX 16

//#define PRI_CHUNK_SIZE (8 * 1024)
#define PRI_CHUNK_SIZE 1024
//#define PRI_CHUNK_SIZE 128
#define PRI_CHUNKS 4

static char sound_name[] = "tomtomgo-sound";
static unsigned int do_filter;

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <asm/hardware/clock.h>
#include <barcelona/cpufreq_order.h>

static struct notifier_block	freq_transition;
static struct notifier_block	freq_policy;
extern unsigned			current_samplerate;
#endif

static struct fasync_struct *fasync_queue;

static struct cs_buffer csb[CSB_MAX];

static void magic_tasklet(unsigned long data);
DECLARE_TASKLET(magic_tasklet_handle, magic_tasklet, 0L);


struct cs_primary_buffer_description priInputDesc = {
	.channels 	 = 2,
	.bits_per_sample = 16,
	.frequency 	 = INITIAL_SAMPLERATE,
	.chunksize	 = PRI_CHUNK_SIZE,
	.chunks		 = PRI_CHUNKS,
};

struct cs_primary_buffer_description priOutputDesc = {
	.channels 	 = 2,
	.bits_per_sample = 16,
	.frequency 	 = INITIAL_SAMPLERATE,
	.chunksize	 = PRI_CHUNK_SIZE,
	.chunks		 = PRI_CHUNKS,
};

//#define TIMING_INFO
#ifdef TIMING_INFO
#include <linux/time.h>
struct write_timestamp {
	struct timeval tv;
	int bytes;
};
static struct write_timestamp wts[32];
static int wts_num = 0;
#endif

#ifdef TDEV
struct tdev_link outlink_nolink = {
	.active = 0,
	.data = NULL,
	.offset = 0,
	.next = NULL
};

struct tdev_link* llink = &outlink_nolink;
struct tdev_link* outlink = &outlink_nolink;
struct tdev_link* wlink = &outlink_nolink;
int out_count;
int silence = 1;
static int bufnum = 0;
#endif

static int coolsound_valid(int csbno) {
	return ((csbno >= 0) && (csbno < CSB_MAX));
}

static int setup_primixer(struct cs_buffer *, struct cs_primary_buffer_description *);

static void pmx_copy_1(void *destination, void *source, size_t samples, unsigned int flags) {
	SANITY_CHECK;
	memcpy(destination, source, samples);
}

static void pmx_copy_2(void *destination, void *source, size_t samples, unsigned int flags) {
	SANITY_CHECK;
	memcpy(destination, source, samples<<1);
}

static void pmx_copy_4(void *destination, void *source, size_t samples, unsigned int flags) {
	SANITY_CHECK;
	memcpy(destination, source, samples<<2);
}

#if 0

static void pmx_zero_1(void *destination, size_t samples) {
	memset(destination, 0, samples<<0);
}

static void pmx_zero_2(void *destination, size_t samples) {
	memset(destination, 0, samples<<1);
}

static void pmx_zero_4(void *destination, size_t samples) {
	memset(destination, 0, samples<<2);
}

#endif

static void pmx_mono_16_playmix(void *destination, void *source, size_t samples, unsigned int flags) {
	int16_t *d, *s;
	int32_t res;

	SANITY_CHECK;

	//If we are the first buffer to be mixed in - just copy;
	if(flags & PMXF_MIXIN_FIRST) {
		pmx_copy_2(destination, source, samples, flags);
		return;
	}


	d=(int16_t *) destination;
	s=(int16_t *) source;

	while(samples--) {
		res = d[0] + s[0];
		if(res < -0x7FFF) res = -0x7FFF;
		else if(res > 0x7FFF) res = 0x7FFF;
		d[0] = (int16_t) res;

		d++; s++;
	}
}

static void pmx_stereo_16_playmix(void *destination, void *source, size_t samples, unsigned int flags) {
	//Like mono - only twice the samples...
	SANITY_CHECK;
	pmx_mono_16_playmix(destination, source, samples << 1, flags);
}

static void pmx_mono_to_stereo_16_playback(void *destination, void *source, size_t samples, unsigned int flags) {
	int16_t *d, *s;

	SANITY_CHECK;

	s = (int16_t *) source;
	d = (int16_t *) destination;

	while(samples--) {
		*(d++) = *(s);
		*(d++) = *(s++);
	}
}

static void pmx_mono_to_stereo_16_playmix(void *destination, void *source, size_t samples, unsigned int flags) {
	int16_t *d, *s;
	int32_t res;

	SANITY_CHECK;

	s = (int16_t *) source;
	d = (int16_t *) destination;

	if(flags & PMXF_MIXIN_FIRST) {
		return pmx_mono_to_stereo_16_playback(destination, source, samples, flags);
	}

	while(samples--) {
		res = d[0] + s[0];
		if(res < -0x7FFF) res = -0x7FFF;
		else if(res > 0x7FFF) res = 0x7FFF;
		d[0] = (int16_t) res;

		res = d[1] + s[0];
		if(res < -0x7FFF) res = -0x7FFF;
		else if(res > 0x7FFF) res = 0x7FFF;
		d[1] = (int16_t) res;

		d+=2; s++;
	}
}

static void pmx_mono_to_stereo_16_record(void *destination, void *source, size_t samples, unsigned int flags) {
	int16_t *d, *s;

	SANITY_CHECK;

	s = (int16_t *) source;
	d = (int16_t *) destination;

	while(samples--) {
		//*(d++) = (s[0]>>1) + (s[1]>>1);
		// Maybe this is mono to mono, with one silent channel?
		// Thomas, I know, dirty hack, sorry :-) This basically
		// maps mono disquised as stereo to mono.
		*(d++) = (s[0]) + (s[1]);
		s+=2;
	}
}

static void pmx_stereo_to_stereo_16_record(void *destination, void *source, size_t samples, unsigned int flags) {
	int16_t *d, *s;

	SANITY_CHECK;

	s = (int16_t *) source;
	d = (int16_t *) destination;

	while(samples--) {
		*d++ = *s++;
		*d++ = *s++;
	}
}

static int csb_selected;
void cs_do_magic(void);

int csb_init(void) {
	int i;

	for(i=0;i<CSB_MAX;i++) {
		csb[i].no = i;
		csb[i].state = CS_BUFFER_FREE;
		csb[i].owner = NULL;
		csb[i].buffer = NULL;
	}

	return 0;
}

int csb_size(int csb_no) {
	if((csb_no < 0) || (csb_no >= CSB_MAX)) return 0;
	if(csb[csb_no].state == CS_BUFFER_FREE) return 0;
	if(csb[csb_no].state == CS_BUFFER_INITIALIZING) return 0;

	return csb[csb_no].size;
}

int csb_acquire(void) {
	int i, res;

	res = -EBUSY;

	tasklet_disable(&magic_tasklet_handle);
		for(i=0;i<CSB_MAX;i++) {
			PK_DBG("state %d %d\n",csb[i].state, CS_BUFFER_FREE);
			if(csb[i].state == CS_BUFFER_FREE) {
				csb[i].state = CS_BUFFER_INITIALIZING;
				res = i; break;
			}
		}
	tasklet_enable(&magic_tasklet_handle);

	return res;
}

int csb_run_in_out(struct cb_in_out *cbio) {
	int res = 0;
	int csb_in_no;
	int csb_out_no;
	int in = 0;
	int out = 0;
	unsigned long flags;

	if(!cbio) return -EINVAL;

	csb_in_no = cbio->in;
	csb_out_no = cbio->out;

	if((csb_out_no < 0) || (csb_out_no >= CSB_MAX)) {
		PK_WARN("Trying to run invalid buffer handle (%d)\n", csb_out_no);
		return -EINVAL;
	}

	tasklet_disable(&magic_tasklet_handle);
		if(csb[csb_in_no].state == CS_BUFFER_FREE) {
			PK_WARN("Trying to run free buffer (%d)\n", csb_in_no);
			res = -EBUSY;
			goto csb_run_out;
		}

		if(csb[csb_in_no].state == CS_BUFFER_INITIALIZING) {
			PK_WARN("Trying to run initializing buffer  (%d)\n", csb_in_no);
			res = -EBUSY;
			goto csb_run_out;
		}

		if(csb[csb_out_no].state == CS_BUFFER_FREE) {
			PK_WARN("Trying to run free buffer (%d)\n", csb_out_no);
			res = -EBUSY;
			goto csb_run_out;
		}

		if(csb[csb_out_no].state == CS_BUFFER_INITIALIZING) {
			PK_WARN("Trying to run initializing buffer  (%d)\n", csb_out_no);
			res = -EBUSY;
			goto csb_run_out;
		}


		if(csb[csb_out_no].desc.type == CSB_TYPE_PLAYBACK) {
			if((res = setup_primixer(&csb[csb_out_no], &priOutputDesc))) {
				PK_ERR("Could not find a suitable playback primixer for buffer %i.\n", csb_out_no);
				res = -EBUSY;
				goto csb_run_out;
			}

			if(csb[csb_out_no].samples < (priOutputDesc.chunksize * priOutputDesc.chunks)) {
				PK_ERR("Can't run buffer %i with less (%u) samples than primary buffer(%u).\n",
					csb_out_no, csb[csb_out_no].samples, (priOutputDesc.chunksize * priOutputDesc.chunks));
				res = -EBUSY;
				goto csb_run_out;
			}

			if(!sound_dma_output_ready()) {
				PK_ERR("Can't run secondary playback buffer %d when primary buffer is not ready.\n",
					csb_out_no);
				res = -EBUSY;
				goto csb_run_out;
			}
		}

		if(csb[csb_in_no].desc.type == CSB_TYPE_RECORD) {
			if((res = setup_primixer(&csb[csb_in_no], &priInputDesc))) {
				PK_ERR("Could not find a suitable record primixer for buffer %i.\n", csb_in_no);
				goto csb_run_out;
			}

			if(csb[csb_in_no].samples < (priInputDesc.chunksize * priInputDesc.chunks)) {
				PK_ERR("Can't run buffer %i with less (%u) samples than primary buffer(%u).\n",
					csb_in_no, csb[csb_in_no].samples, (priInputDesc.chunksize * priInputDesc.chunks));
				res = -EBUSY;
				goto csb_run_out;
			}

			if(!sound_dma_input_ready()) {
				PK_ERR("Can't run secondary record buffer %d when primary buffer is not ready.\n",
					csb_in_no);
				res = -EBUSY;
				goto csb_run_out;
			}
		}

		csb[csb_in_no].state = CS_BUFFER_RUNNING;
		csb[csb_out_no].state = CS_BUFFER_RUNNING;
		if(!sound_dma_input_running()) {
			PK_DBG("Running input buffer.\n");
			sound_dma_input_reset();
			in = 1;
		}

		if(!sound_dma_output_running()) {
			PK_DBG("Running output buffer.\n");
			sound_dma_output_reset();
			out = 1;
		}

		if ((in) || (out)) {
			cs_do_magic();
		}

		local_irq_save(flags);
		if (in) {
			iis_control_cmd(IIS_ALIGN_RX);

			if((res = sound_dma_input_start())) {
				PK_ERR("could not start primary input buffer (%d).\n", res);
				local_irq_restore(flags);
				goto csb_run_out;
				//TODO: bring back state machine to sane state.
			}

			iis_control_cmd(IIS_ACTIVE_RX);
		}

		if (out) {
			iis_control_cmd(IIS_ALIGN_TX);

			if((res = sound_dma_output_start())) {
				PK_ERR("could not start primary input buffer (%d).\n", res);
				local_irq_restore(flags);
				goto csb_run_out;
				//TODO: bring back state machine to sane state.
			}

			iis_control_cmd(IIS_ACTIVE_TX);
		}
		local_irq_restore(flags);

csb_run_out:
	tasklet_enable(&magic_tasklet_handle);

	return res;
}


int csb_run(int csb_no) {
	int res = 0;
	if((csb_no < 0) || (csb_no >= CSB_MAX)) {
		PK_WARN("Trying to run invalid buffer handle (%d)\n", csb_no);
		return -EINVAL;
	}

	tasklet_disable(&magic_tasklet_handle);
		if(csb[csb_no].state == CS_BUFFER_FREE) {
			PK_WARN("Trying to run free buffer (%d)\n", csb_no);
			res = -EBUSY;
			goto csb_run_out;
		}

		if(csb[csb_no].state == CS_BUFFER_INITIALIZING) {
			PK_WARN("Trying to run initializing buffer  (%d)\n", csb_no);
			res = -EBUSY;
			goto csb_run_out;
		}

		if(csb[csb_no].desc.type == CSB_TYPE_PLAYBACK) {
			if((res = setup_primixer(&csb[csb_no], &priOutputDesc))) {
				PK_ERR("Could not find a suitable playback primixer for buffer %i.\n", csb_no);
				res = -EBUSY;
				goto csb_run_out;
			}

			if(csb[csb_no].samples < (priOutputDesc.chunksize * priOutputDesc.chunks)) {
				PK_ERR("Can't run buffer %i with less (%u) samples than primary buffer(%u).\n",
					csb_no, csb[csb_no].samples, (priOutputDesc.chunksize * priOutputDesc.chunks));
				res = -EBUSY;
				goto csb_run_out;
			}

			if(!sound_dma_output_ready()) {
				PK_ERR("Can't run secondary playback buffer %d when primary buffer is not ready.\n",
					csb_no);
				res = -EBUSY;
				goto csb_run_out;
			}

			csb[csb_no].state = CS_BUFFER_RUNNING;
			if(!sound_dma_output_running()) {
				PK_DBG("Running output buffer.\n");
				sound_dma_output_reset();
				cs_do_magic();
				iis_control_cmd(IIS_ALIGN_TX);

				if((res = sound_dma_output_start())) {
					PK_ERR("could not start primary input buffer (%d).\n", res);
					goto csb_run_out;
					//TODO: bring back state machine to sane state.
				}


				iis_control_cmd(IIS_ACTIVE_TX);
			} else {
				PK_DBG("Output buffer already running.\n");
			}
		} else if(csb[csb_no].desc.type == CSB_TYPE_RECORD) {
			if((res = setup_primixer(&csb[csb_no], &priInputDesc))) {
				PK_ERR("Could not find a suitable record primixer for buffer %i.\n", csb_no);
				goto csb_run_out;
			}

			if(csb[csb_no].samples < (priInputDesc.chunksize * priInputDesc.chunks)) {
				PK_ERR("Can't run buffer %i with less (%u) samples than primary buffer(%u).\n",
					csb_no, csb[csb_no].samples, (priInputDesc.chunksize * priInputDesc.chunks));
				res = -EBUSY;
				goto csb_run_out;
			}

			if(!sound_dma_input_ready()) {
				PK_ERR("Can't run secondary record buffer %d when primary buffer is not ready.\n",
					csb_no);
				res = -EBUSY;
				goto csb_run_out;
			}

			csb[csb_no].state = CS_BUFFER_RUNNING;
			if(!sound_dma_input_running()) {
				PK_DBG("Running input buffer.\n");
				sound_dma_input_reset();
				cs_do_magic();
				iis_control_cmd(IIS_ALIGN_RX);

				if((res = sound_dma_input_start())) {
					PK_ERR("could not start primary input buffer (%d).\n", res);
					goto csb_run_out;
					//TODO: bring back state machine to sane state.
				}

				iis_control_cmd(IIS_ACTIVE_RX);
			} else {
				PK_DBG("Input buffer already running.\n");
			}
		} else {
			PK_WARN("Trying to buffer (%d) of unknown type %d.\n", csb_no, csb[csb_no].desc.type);
		}


csb_run_out:
	tasklet_enable(&magic_tasklet_handle);

	return res;
}

int csb_stop(int csb_no) {
	int res = 0;
	if((csb_no < 0) || (csb_no >= CSB_MAX)) return -EINVAL;

	tasklet_disable(&magic_tasklet_handle);
		if(csb[csb_no].state == CS_BUFFER_FREE) {
			PK_WARN("Tried to stop free buffer %d.\n",csb_no);
			res = -EBUSY;
			goto csb_stop_out;
		}

		if(csb[csb_no].state == CS_BUFFER_INITIALIZING) {
			PK_WARN("Tried to stop initializing buffer %d.\n",csb_no);
			res = -EBUSY;
			goto csb_stop_out;
		}

		csb[csb_no].state = CS_BUFFER_STOPPED;
		PK_DBG("Buffer %d now stopped.\n",csb_no);

csb_stop_out:
	tasklet_enable(&magic_tasklet_handle);
	return res;
}

int csb_free(int csb_no, int force) {
	struct cs_buffer *my_csb = &csb[csb_no];
	if(!coolsound_valid(csb_no)) return -EINVAL;

	csb_stop(csb_no);

	if(my_csb->mapcount) {
		if(force) {
			PK_DBG("Warning: csb_free forced to free csb[%i] which is still mapped %i times.\n", csb_no, my_csb->mapcount);
		} else {
			return -EINVAL;
		}
	}

	vfree(my_csb->buffer);
	my_csb->buffer = NULL;
	my_csb->size = 0;
	my_csb->owner = NULL;

	my_csb->state = CS_BUFFER_FREE;
	my_csb->owner = NULL;

	return 0;
}

int csb_queue_stop(struct cb_qstop *cbq) {
	int res = 0;
	int csb_no;

	if(!cbq) return -EINVAL;
	csb_no = cbq->buffer;

	if((csb_no < 0) || (csb_no >= CSB_MAX)) return -EINVAL;

	tasklet_disable(&magic_tasklet_handle);
		if(csb[csb_no].state == CS_BUFFER_FREE) {
			PK_WARN("Tried to queue stop on free buffer %d.\n",csb_no);
			res = -EBUSY;
			goto csb_queue_stop_out;
		}

		if(csb[csb_no].state == CS_BUFFER_INITIALIZING) {
			PK_WARN("Tried to queue stop on initializing buffer %d.\n",csb_no);
			res = -EBUSY;
			goto csb_queue_stop_out;
		}

		if((cbq->stop_position < 0) || (cbq->stop_position >= csb[csb_no].size)) {
			PK_WARN("Tried to queue stop on buffer [%d] on out-of-limits position [%d]. Size of buffer is [%d].\n",
				csb_no, cbq->stop_position, csb[csb_no].size);
			res = -EINVAL;
			goto csb_queue_stop_out;
		}

		csb[csb_no].state = CS_BUFFER_STOP_PENDING;
		csb[csb_no].spos  = cbq->stop_position;
		PK_DBG("Buffer %d now stops at position %d. Current position is %d.\n", csb_no, csb[csb_no].spos, csb[csb_no].pointer);

csb_queue_stop_out:
	tasklet_enable(&magic_tasklet_handle);
	return res;
}

int csb_setpos(int csb_no, int pos) {
	int res = 0;
	if((csb_no < 0) || (csb_no >= CSB_MAX)) return -EINVAL;

	tasklet_disable(&magic_tasklet_handle);
		if(csb[csb_no].state == CS_BUFFER_FREE) {
			PK_WARN("Tried to setpos free buffer %d.\n",csb_no);
			res = -EBUSY;
			goto csb_setpos_out;
		}

		if(csb[csb_no].state == CS_BUFFER_INITIALIZING) {
			PK_WARN("Tried to setpos initializing buffer %d.\n",csb_no);
			res = -EBUSY;
			goto csb_setpos_out;
		}

		if(csb[csb_no].size <= pos) {
			PK_WARN("Tried to setpos %d with pos(%d) bigger than size (%d).\n",csb_no, pos, csb[csb_no].size);
			res = -EBUSY;
			goto csb_setpos_out;
		}

		csb[csb_no].pointer = pos;

csb_setpos_out:
	tasklet_enable(&magic_tasklet_handle);
	return res;

}

int csb_getpos(int csb_no) {
	int res = 0;
	if((csb_no < 0) || (csb_no >= CSB_MAX)) return -EINVAL;

	if(csb[csb_no].state == CS_BUFFER_FREE) {
		PK_WARN("Tried to getpos free buffer %d.\n",csb_no);
		res = -EBUSY;
		goto csb_getpos_out;
	}

	if(csb[csb_no].state == CS_BUFFER_INITIALIZING) {
		PK_WARN("Tried to getpos initializing buffer %d.\n",csb_no);
		res = -EBUSY;
		goto csb_getpos_out;
	}

	res = csb[csb_no].pointer;

csb_getpos_out:
	return res;
}

int csb_setcbfreq(int csb_no, int cbfrequency) {
	int res = 0;
	if((csb_no < 0) || (csb_no >= CSB_MAX)) return -EINVAL;

	tasklet_disable(&magic_tasklet_handle);
		if(csb[csb_no].state == CS_BUFFER_FREE) {
			PK_WARN("Tried to setcbfreq free buffer %d.\n",csb_no);
			res = -EBUSY;
			goto csb_setcbfreq_out;
		}

		if(csb[csb_no].state == CS_BUFFER_INITIALIZING) {
			PK_WARN("Tried to setcbfreq initializing buffer %d.\n",csb_no);
			res = -EBUSY;
			goto csb_setcbfreq_out;
		}

		csb[csb_no].cbfreq = cbfrequency;

csb_setcbfreq_out:
	tasklet_enable(&magic_tasklet_handle);
	return res;
}

static int sound_setup_buffer(int csb_no, struct cs_buffer_description *bds) {
	struct cs_buffer *my_csb;

	if((csb_no < 0) || (csb_no >= CSB_MAX))
		return -EINVAL;

	if(!bds) return -EINVAL;

	my_csb = &csb[csb_no];

	if((my_csb->state != CS_BUFFER_STOPPED) && (my_csb->state != CS_BUFFER_INITIALIZING))
		return -EINVAL;

	//Fixed for now...
	//Don't forget to change size calculation!!!
	if(bds->frequency == 0) return -EINVAL;
	if(bds->channels<1)		return -EINVAL;
	if(bds->channels>2)		return -EINVAL;
	if(bds->bits_per_sample != 16)	return -EINVAL;
	if((bds->samples == 0) || (bds->samples > (1024 * 1024)))
		return -EINVAL;


	if(my_csb->buffer) {
		vfree(my_csb->buffer);
		my_csb->buffer = NULL;
	}

	my_csb->desc 	= (*bds);
	my_csb->bytes_per_sample = 2 * bds->channels;
	my_csb->size	= bds->samples * my_csb->bytes_per_sample;
	my_csb->samples	= bds->samples;
	my_csb->buffer	= (void *) vmalloc(my_csb->size);
	my_csb->pointer	= 0;

	if(!my_csb->buffer) {
		PK_ERR("Failed to allocate DMA buffer for %u.\n", csb_no);
		return -EINVAL;
	}

	PK_DBG("CSB[%u] set to %uHz, %uCh, %uBit %uSmp, %uBytes\n", csb_no,
		my_csb->desc.frequency, my_csb->desc.channels,
		my_csb->desc.bits_per_sample, my_csb->samples,
		my_csb->size);

	return 0;
}

static int sound_set_buffer_frequency(int csb_no, int frequency) {
	struct cs_buffer_description new_csbd;
	struct cs_buffer *my_csb;

	if((csb_no < 0) || (csb_no >= CSB_MAX))
		return -EINVAL;

	my_csb = &csb[csb_no];
	if((my_csb->state != CS_BUFFER_STOPPED) && (my_csb->state != CS_BUFFER_INITIALIZING))
		return -EINVAL;

	new_csbd = my_csb->desc;
	new_csbd.frequency = frequency;

	return sound_setup_buffer(csb_no, &new_csbd);
}

static int sound_request_buffer(struct cs_buffer_description *bds, struct file *owner) {
	int res = 0;
	int my_csb_no = csb_acquire();
	struct cs_buffer *my_csb = &csb[my_csb_no];

	if(my_csb_no < 0) return my_csb_no;

	my_csb->owner = (void *) owner;

	res = sound_setup_buffer(my_csb_no, bds);
	if(res) {
		PK_ERR("Failed to setup secondary buffer #%u\n", my_csb_no);
		csb_free(my_csb_no, 1);
		return -EINVAL;
	}

	my_csb->state	= CS_BUFFER_STOPPED;

	csb_selected = my_csb_no;
	return my_csb_no;
}

void cs_signal(int buffer) {
	static int z=1234;
	//PK_DBG("cs_signal(%d)\n", z);
	kill_fasync(&fasync_queue, SIGIO, z);
	z++;
}

void cs_do_magic(void) {
	int i, smp;
	int inbuffers, outbuffers;
	int do_more_magic;

	static int zeros = 0;

do_more_magic_here:

	inbuffers = 0;
	outbuffers = 0;
	do_more_magic = 0;



	//Care for output buffer
	smp = sound_dma_output_free();
	for(i=0;i<CSB_MAX;i++) {
		int is_rest = 0;
		int optr, nptr;

		if(csb[i].state == CS_BUFFER_FREE) {
			continue;
		}

		if(csb[i].state == CS_BUFFER_INITIALIZING) {
			continue;
		}
		if(csb[i].desc.type != CSB_TYPE_PLAYBACK) {
			continue;
		}

		if((csb[i].state == CS_BUFFER_RUNNING) || (csb[i].state == CS_BUFFER_STOP_PENDING)) {
			PK_MAGIC("Mixing in %d.\n",i);
		} else continue;

		if(csb[i].state == CS_BUFFER_STOP_PENDING) {
			int remaining;
			if(csb[i].pointer <= csb[i].spos) 	remaining = csb[i].spos - csb[i].pointer;
			else					remaining = csb[i].size - (csb[i].pointer - csb[i].spos);

			//Align to full samples;
			remaining /= csb[i].bytes_per_sample;
			remaining *= csb[i].bytes_per_sample;

			if(remaining <= (smp * csb[i].bytes_per_sample)) {
				smp = remaining / csb[i].bytes_per_sample;
				is_rest = 1;
			}
		}

		zeros = 0;
		optr = csb[i].pointer;
		PK_MAGIC("csb:%u pointer:%ub size:%ub smp:%us fillsize:%ub, fillspace:%u\n",
			i, csb[i].pointer, csb[i].size, smp, (smp * csb[i].bytes_per_sample), (csb[i].size - csb[i].pointer));
		if((csb[i].size - csb[i].pointer) >= (smp * csb[i].bytes_per_sample)) {
			PK_MAGIC("sound_dma_output_put(%p, %u, %u)\n", csb[i].buffer + csb[i].pointer, smp, 0);
			sound_dma_output_put(csb[i].buffer + csb[i].pointer, smp, 0, csb[i].primixer, outbuffers?0:PMXF_MIXIN_FIRST);
			csb[i].pointer+=smp * csb[i].bytes_per_sample;
		} else {
			int csmp = (csb[i].size - csb[i].pointer) / csb[i].bytes_per_sample;
			PK_MAGIC("sound_dma_output_put(%p, %u, %u) A\n", csb[i].buffer + csb[i].pointer, csmp, 0);
			sound_dma_output_put(csb[i].buffer + csb[i].pointer, csmp, 0, csb[i].primixer, outbuffers?0:PMXF_MIXIN_FIRST);
			PK_MAGIC("sound_dma_output_put(%p, %u, %u) B\n", csb[i].buffer, (smp-csmp), csmp);
			sound_dma_output_put(csb[i].buffer, (smp-csmp), csmp, csb[i].primixer, outbuffers?0:PMXF_MIXIN_FIRST);
			csb[i].pointer = (smp-csmp) * csb[i].bytes_per_sample;
		}

		nptr = csb[i].pointer;

		//Check if pointer went over notification...
		if(csb[i].cbfreq) {
			int notify_usermode = 0;

			if(nptr < optr) notify_usermode = 1;
			else if((smp * csb[i].bytes_per_sample) >= csb[i].cbfreq) notify_usermode = 1;
			else if((nptr%csb[i].cbfreq) < (optr%csb[i].cbfreq)) notify_usermode = 1;

			if(notify_usermode) cs_signal(i);
		}

		if(csb[i].state == CS_BUFFER_STOP_PENDING) {
			if(is_rest) {
				//TODO: Perhaps notify usermode of stopped buffer
				csb[i].state = CS_BUFFER_STOPPED;
				do_more_magic = 1;
				PK_DBG("Secondary buffer %d executed queued stop at position %d.\n", i, csb[i].spos);
			}
		}

		outbuffers++;
	}

#if 1
	//PK_DBG("outbuffers=%d\n", outbuffers);
	if(!outbuffers) {
		int running, size;

		zeros += smp;
		//PK_DBG("sound_dma_output_zeroes(%u, 0)\n", smp);
		sound_dma_output_zeroes(smp, 0);

		running = sound_dma_output_running();
		size = sound_dma_output_size() >> 2;
		//PK_DBG("running=%d, zeros=%d, size=%d\n", running, zeros, size);
		if(running && zeros >= size) {
			PK_DBG("Stop Primary Playback!\n");
			iis_control_cmd(IIS_IDLE_TX);
			sound_dma_output_stop();
			iis_control_cmd(IIS_FLUSH_TX);
		}
	}
#else
	/* TEMPORARY HACK: Forget zeroing out anything, until the suspend
	 * function has a way to wait for us to finish. */
	if(!outbuffers && sound_dma_output_running()) {
		PK_DBG("Stop Primary Playback!\n");
		iis_control_cmd(IIS_IDLE_TX);
		sound_dma_output_stop();
		iis_control_cmd(IIS_FLUSH_TX);
	}
#endif

	sound_dma_output_advance(smp);

	//Do black filter magic
	if(do_filter) sound_dma_filter_magic();

	//care for input driver
	smp = sound_dma_input_filled();

	for(i=0;i<CSB_MAX;i++) {
		if(csb[i].state == CS_BUFFER_FREE) {
			continue;
		}

		if(csb[i].state == CS_BUFFER_INITIALIZING) {
			continue;
		}

		if(csb[i].desc.type != CSB_TYPE_RECORD) {
			continue;
		}

		if(csb[i].state == CS_BUFFER_RUNNING) {
			int nptr;
			int optr = csb[i].pointer;

			if((csb[i].size - csb[i].pointer) >= smp * csb[i].bytes_per_sample) {
				sound_dma_input_get(csb[i].buffer + csb[i].pointer, smp, 0, csb[i].primixer, 0);
				csb[i].pointer+=smp * csb[i].bytes_per_sample;
			} else {
				int csmp = ((csb[i].size - csb[i].pointer) / csb[i].bytes_per_sample);
				sound_dma_input_get(csb[i].buffer + csb[i].pointer, csmp, 0, csb[i].primixer, 0);
				sound_dma_input_get(csb[i].buffer, (smp-csmp), csmp, csb[i].primixer, 0);
				csb[i].pointer = (smp-csmp) * csb[i].bytes_per_sample;
			}

			nptr = csb[i].pointer;

			if(csb[i].cbfreq) {
				int notify_usermode = 0;

				if(nptr < optr) notify_usermode = 1;
				else if((smp*csb[i].bytes_per_sample) >= csb[i].cbfreq) notify_usermode = 1;
				else if((nptr%csb[i].cbfreq) < (optr%csb[i].cbfreq)) notify_usermode = 1;

				if(notify_usermode) cs_signal(i);
			}
			inbuffers++;
		}
		break;
	}
	sound_dma_input_advance(smp);
	if(sound_dma_input_running() && (!inbuffers)) {
		PK_DBG("Stop Primary Record!\n");
		iis_control_cmd(IIS_IDLE_RX);
		sound_dma_input_stop();
		iis_control_cmd(IIS_FLUSH_RX);
	}

	if(do_more_magic) goto do_more_magic_here;
}

int sound_setup_primary(struct cs_primary_buffer_description *cpbd) {
	int res = 0;

	if(!cpbd) return -EINVAL;
	if(cpbd->bits_per_sample != 16) {
		PK_WARN("invalid bits_per_sample (%u) for primary buffers rejected.\n",
			cpbd->bits_per_sample);
		return -EINVAL;
	}

	if(cpbd->channels != 2) {
		PK_WARN("invalid channels (%u) for primary buffers rejected.\n",
			cpbd->channels);
		return -EINVAL;
	}

	if((cpbd->chunksize < 16 ) || (cpbd->chunksize > 4096 )) {
		PK_WARN("invalid chunksize (%u) for primary buffers rejected.\n",
			cpbd->chunksize);
		return -EINVAL;
	}

	if((cpbd->chunks < 3) || (cpbd->chunks > 16)) {
		PK_WARN("invalid chunk# (%u) for primary buffers rejected.\n",
			cpbd->chunks);
		return -EINVAL;
	}


	tasklet_disable(&magic_tasklet_handle);
		if(sound_dma_output_running() || sound_dma_input_running()) {
			//PK_WARN("dma still busy\n");
			res = -EBUSY;
			goto sound_setup_primary_out;
		}

		if((res = sound_dma_setup_buffers(cpbd->chunksize * 4, cpbd->chunks))) {
			PK_WARN("setup buffers\n");
			goto sound_setup_primary_out;
		}

		priInputDesc.chunksize 	= cpbd->chunksize;
		priInputDesc.chunks		= cpbd->chunks;

		priOutputDesc.chunksize = cpbd->chunksize;
		priOutputDesc.chunks	= cpbd->chunks;

		if((res = iis_set_samplerate(cpbd->frequency))) {
			PK_WARN("set_samplerate\n");
			goto sound_setup_primary_out;
		}

		priInputDesc.frequency = cpbd->frequency;
		priOutputDesc.frequency = cpbd->frequency;

sound_setup_primary_out:
	tasklet_enable(&magic_tasklet_handle);
	return res;
}

/*
 * helper functions for direct setup
 * of primary buffer parameters
 */

int sound_setup_primary_frequency(unsigned int frequency) {
	struct cs_primary_buffer_description new_input 	= priInputDesc;
	struct cs_primary_buffer_description new_output = priOutputDesc;

	new_input.frequency 	= frequency;
	new_output.frequency	= frequency;

	//TODO: input and output cannot be set seperately yet
	sound_setup_primary(&new_input);

	return priInputDesc.frequency;
}

static int setup_primixer(struct cs_buffer *pSecondary, struct cs_primary_buffer_description *pPrimaryDescription) {
	struct cs_buffer_description *pSD;
	struct cs_primary_buffer_description *pPD;

	if(!pSecondary) 		return -EINVAL;
	if(!pPrimaryDescription)	return -EINVAL;

	pSD = &pSecondary->desc;
	pPD = pPrimaryDescription;

	if(!(pSD && pPD)) return -EINVAL;

	if((pSD->bits_per_sample != 8) && (pSD->bits_per_sample != 16)) {
		PK_WARN("Can't deal with %d bits per sample in secondary buffer.\n", pSD->bits_per_sample);
		return -EBUSY;
	}

	if((pSD->channels != 1) && (pSD->channels != 2)) {
		PK_WARN("Can't deal with %d channels in secondary buffer.\n", pSD->channels);
		return -EBUSY;
	}

	//We can't do 8<->16 Bit conversion by now
	if(pSD->bits_per_sample != pPD->bits_per_sample) {
		PK_WARN("Can't convert from secondary %d bits/sample to primary %d bits/sample.\n", pSD->bits_per_sample, pPD->bits_per_sample);
		return -EBUSY;
	}

	//We can't do sampling frequency conversion by now
	if(pSD->frequency != pPD->frequency) {
		PK_WARN("Can't convert from secondary %d smp/s to primary %d smp/s.\n", pSD->frequency, pPD->frequency);
		return -EBUSY;
	}

	if(pSD->channels == pPD->channels) {
		int bps = (pSD->bits_per_sample / 8) * pSD->channels;
		PK_DBG("Primary buffer format equals secondary buffer format(%d bit, %d channels, %d Hz).\n",
			pSD->bits_per_sample, pSD->channels, pSD->frequency);

		if((pSD->bits_per_sample == 16) && (pSD->channels == 1) && (pSD->type == CSB_TYPE_PLAYBACK)) {
			PK_DBG("Choose 'pmx_mono_16_playmix()' as primixer function.\n");
			pSecondary->primixer = pmx_mono_16_playmix;
			return 0;
		}

		if((pSD->bits_per_sample == 16) && (pSD->channels == 2) && (pSD->type == CSB_TYPE_PLAYBACK)) {
			PK_DBG("Choose 'pmx_stereo_16_playmix()' as primixer function.\n");
			pSecondary->primixer = pmx_stereo_16_playmix;
			return 0;
		}

		if((pSD->bits_per_sample == 16) && (pSD->channels == 2) && (pSD->type == CSB_TYPE_RECORD)) {
			PK_DBG("Choose 'pmx_stereo_to_stereo_record()' as primixer function.\n");
			pSecondary->primixer = pmx_stereo_to_stereo_16_record;
			return 0;
		}

		switch(bps) {
			case 1:
				PK_DBG("Choose 'pmx_copy_1()' as primixer function.\n");
				pSecondary->primixer = pmx_copy_1;
				return 0;

			case 2:
				PK_DBG("Choose 'pmx_copy_2()' as primixer function.\n");
				pSecondary->primixer = pmx_copy_2;
				return 0;

			case 4:
				PK_DBG("Choose 'pmx_copy_4()' as primixer function.\n");
				pSecondary->primixer = pmx_copy_4;
				return 0;

			default:
				PK_WARN("Don't have copy function for strange %d bytes per sample.\n", bps);
				return -EBUSY;
		}

	} else if((pSD->channels == 1) && (pPD->channels == 2)) {
		if(pSD->type == CSB_TYPE_PLAYBACK) {
			PK_DBG("Choose 'pmx_mono_to_stereo_16_playback()' as primixer function.\n");
			pSecondary->primixer = pmx_mono_to_stereo_16_playmix;
			return 0;
		} else if(pSD->type == CSB_TYPE_RECORD) {
			PK_DBG("Choose 'pmx_mono_to_stereo_16_record()' as primixer function.\n");
			pSecondary->primixer = pmx_mono_to_stereo_16_record;
			return 0;
		} else {
			PK_WARN("Unknown buffer type %d.\n", (int) pSD->type);
			return -EBUSY;
		}
	} else {
		PK_WARN("Can't convert from secondary %d channels to primary %d channels.\n", pSD->channels, pPD->channels);
		return -EBUSY;
	}
}


static void magic_tasklet(unsigned long data) {
	cs_do_magic();
}


void cs_callback(enum eDMAPurpose purpose, int chunk) {
	tasklet_schedule(&magic_tasklet_handle);
}


/* open: Not much here, just check that there isn't anyone
 * using the device.
 */
static int sound_open(struct inode *inode, struct file *fp)
{
	PK_DBG("trace\n");

	if (!(fp->f_mode & FMODE_WRITE || fp->f_mode & FMODE_READ))
		return -ENODEV;

	return 0;
}

static int sound_fasync(int fd, struct file *fp, int mode)
{
	return fasync_helper(fd, fp, mode, &fasync_queue);
}

static int sound_release(struct inode *inode, struct file *fp)
{
	int i;
	if (!(fp->f_mode & FMODE_WRITE || fp->f_mode & FMODE_READ))
		return -ENODEV;
	for(i=0;i<CSB_MAX;i++) {
		if(csb[i].state == CS_BUFFER_FREE) continue;
		if(csb[i].owner != (void *) fp) continue;

		csb_free(i, 1);
	}
	sound_fasync(-1, fp, 0);
	return 0;
}

void coolsound_dump_buf0(void) {
	size_t size = csb[0].size;
	size_t pos  = 0;

	printk("CoolSound: Dumping contents of secondary buffer 0 (%u Bytes at [%p]):\n", csb[0].size, csb[0].buffer);

	for(pos = 0; pos<size; pos++) {
		printk("%02X", ((unsigned char *)csb[0].buffer)[pos]);
		if(pos == 0) continue;
		if((pos%16) == 0) printk("\n");
		if((pos%8) == 0) printk(" ");
	}

	printk("\nfinished.\n");
}

void usermode_dbgcall(unsigned long arg) {
	PK_INFO("Usermode debug [%lu].\n", arg);
	switch(arg) {
		case 1: iis_showbus("usermode debug"); return;
		case 2: sound_dma_usermode_debug(); return;
		case 3: coolsound_dump_buf0(); return;
		default:
			PK_WARN("Unknown usermode debug request %lu.\n", arg);
			break;
	}
}

static int coolsound_native_ioctl(struct inode *inode, struct file *fp, unsigned int cmd, unsigned long arg) {
	if (_IOC_TYPE(cmd) != COOLSOUND_DRIVER_MAGIC) return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_WRITE)
		if (!access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd))) return -EFAULT;

	switch(cmd) {

	case COOLSOUND_SETUP_PRIMARY: {
		struct cs_primary_buffer_description cpbd;
		if(!access_ok(VERIFY_READ, (void *)arg, sizeof(struct cs_primary_buffer_description))) return -EFAULT;
		if(copy_from_user(&cpbd, (void *)arg, sizeof(struct cs_primary_buffer_description))) return -EFAULT;
		return sound_setup_primary(&cpbd);
	}

	case COOLSOUND_GET_BUFFER: {
		struct cs_buffer_description cbd;
		if(!access_ok(VERIFY_READ, (void *)arg, sizeof(struct cs_buffer_description))) return -EFAULT;
		if(copy_from_user(&cbd, (void *)arg, sizeof(struct cs_buffer_description))) return -EFAULT;
		return sound_request_buffer(&cbd, fp);
	}

	case COOLSOUND_GET_BUFSIZE: {
		if(!coolsound_valid(arg)) return -EINVAL;

		return csb[arg].size;
	}

	case COOLSOUND_PUT_BUFFER: {
		if(!coolsound_valid(arg)) return -EINVAL;
		if(csb[arg].owner != fp) return -EINVAL;
		if(0 != csb_free(arg, 0)) {
			PK_DBG("CoolSound: Warning: COOLSOUND_PUT_BUFFER on %lu failed.\n", arg);
		}
	}

	case COOLSOUND_SEL_BUFFER: {
		csb_selected = arg;
		break;
	}

	case COOLSOUND_RUN_IN_OUT: {
		struct cb_in_out cbio;
		if(!access_ok(VERIFY_READ, (void *)arg, sizeof(struct cb_in_out))) return -EFAULT;
		if(copy_from_user(&cbio, (void *)arg, sizeof(struct cb_in_out))) return -EFAULT;
		return csb_run_in_out(&cbio);
	}

	case COOLSOUND_RUN_BUFFER: {
		return csb_run(arg);
	}

	case COOLSOUND_STOP_BUFFER: {
		return csb_stop(arg);
	}

	case COOLSOUND_QUEUE_STOP: {
		struct cb_qstop cqs;
		if(!access_ok(VERIFY_READ, (void *)arg, sizeof(struct cb_qstop))) return -EFAULT;
		if(copy_from_user(&cqs, (void *)arg, sizeof(struct cb_qstop))) return -EFAULT;
		return csb_queue_stop(&cqs);
	}

	case COOLSOUND_WAIT_STOP: {
		if(!coolsound_valid(arg)) return -EINVAL;
		if(csb[arg].owner != fp) return -EINVAL;

		while((csb[arg].state == CS_BUFFER_RUNNING) || (csb[arg].state == CS_BUFFER_STOP_PENDING)) {
			msleep(3);
		}
	}

	case COOLSOUND_GETPOS: {
		return csb_getpos(arg);
	}

	case COOLSOUND_SETPOS: {
		struct cb_chpos chp;
		if(!access_ok(VERIFY_READ, (void *)arg, sizeof(struct cb_chpos))) return -EFAULT;
		if(copy_from_user(&chp, (void *)arg, sizeof(struct cb_chpos))) return -EFAULT;
		return csb_setpos(chp.buffer, chp.position);
	}

	case COOLSOUND_SETCBFREQ: {
		struct cb_cbfreq cbf;
		if(!access_ok(VERIFY_READ, (void *)arg, sizeof(struct cb_cbfreq))) return -EFAULT;
		if(copy_from_user(&cbf, (void *)arg, sizeof(struct cb_cbfreq))) return -EFAULT;
		return csb_setcbfreq(cbf.buffer, cbf.frequency);
	}

	case COOLSOUND_SETFILTER: {
		tasklet_disable(&magic_tasklet_handle);
		if(arg) {
			sound_dma_filter_init();

			do_filter=1;
			sound_dma_filter_resync();
		} else {
			do_filter=0;
			sound_dma_filter_shutdown();
		}
		tasklet_enable(&magic_tasklet_handle);
		break;
	}

	case COOLSOUND_GET_HEADPHONE_CONNECTED: {
		/* detect whether a headphone and or pdk is connected */
		unsigned int external_speaker = 0;
		if (IO_GetInput(HEADPHONE_DETECT)) external_speaker |= COOLSOUND_HEADPHONE;
		if ((IO_GetDockState()) == GODOCK_CRIB) external_speaker |= COOLSOUND_PDK;
		if(copy_to_user((void *)arg, &external_speaker, sizeof(external_speaker))) return -EFAULT;
		return 0;
	}

	case COOLSOUND_GET_EXTMIC_CONNECTED: {
		unsigned int extmicConnected = *(unsigned int*)arg;
		if ((IO_GetDockState()) != GODOCK_CRIB) extmicConnected = 0;
		switch (extmicConnected) {
			default:
			case 0:
				/* detect mic */
				extmicConnected = IO_GetInput(EXTMIC_DETECT);
				if (extmicConnected) IO_Activate(MIC_SW);
				else IO_Deactivate(MIC_SW);
				break;

			case 1:
				/* internal mic */
				extmicConnected = 0;
				IO_Deactivate(MIC_SW);
				break;

			case 2:
				/* external mic */
				extmicConnected = 1;
				IO_Activate(MIC_SW);
				break;
		}

		if(copy_to_user((void *)arg, &extmicConnected, sizeof(extmicConnected))) return -EFAULT;
		return 0;
	}

	case COOLSOUND_SET_EXTMIC_CONNECTION: {
		unsigned int extmicConnected = *(unsigned int*)arg;
		switch (extmicConnected) {
			case 0:
				IO_Deactivate(MIC_SW);
				break;

			case 1:
				IO_Activate(MIC_SW);
				break;
		}
		return 0;
	}

	case COOLSOUND_SWITCH_LINE_IN: {
		return codec_switch_line_in(arg);
	}

	case COOLSOUND_MUTE_INTERNAL:
		IO_Deactivate(AMP_ON);
		return 0;

	case COOLSOUND_UNMUTE_INTERNAL:
		IO_Activate(AMP_ON);
		return 0;

	case COOLSOUND_MUTE_EXTERNAL:
		IO_Activate(MUTE_EXT);
		gpio_force_update();
		return 0;

	case COOLSOUND_UNMUTE_EXTERNAL:
		IO_Deactivate(MUTE_EXT);
		gpio_force_update();
		return 0;

	case COOLSOUND_MUTE_NAVIGATION:
		IO_Activate(NAVI_MUTE);
		break;

	case COOLSOUND_UNMUTE_NAVIGATION:
		IO_Deactivate(NAVI_MUTE);
		break;

	case COOLSOUND_MUTE_TELEPHONE:
		IO_Activate(TEL_MUTE);
		break;

	case COOLSOUND_UNMUTE_TELEPHONE:
		IO_Deactivate(TEL_MUTE);
		break;

	case COOLSOUND_SET_VOLUME:
		return codec_set_volume(arg);

	case COOLSOUND_SET_VOLUME_dB:
		return codec_set_volume_dB(arg);

	case COOLSOUND_SET_VOLUME_INTERNAL:
		return codec_set_volume_internal(arg);

	case COOLSOUND_SET_VOLUME_EXTERNAL:
		return codec_set_volume_external(arg);

	case COOLSOUND_SET_VOLUME_EXTERNAL_LEFT:
		return codec_set_volume_external_left(arg);

	case COOLSOUND_SET_VOLUME_EXTERNAL_RIGHT:
		return codec_set_volume_external_right(arg);

	case COOLSOUND_SET_TREBLE:
		return codec_set_treble(arg);

	case COOLSOUND_SET_BASS:
		return codec_set_bass(arg);

	case COOLSOUND_SET_REG:
		return codec_set_register(arg);

	/* for debugging purposes only */
	case COOLSOUND_DEBUG:
		usermode_dbgcall(arg);
		return 0;
	}

	return -ENOTTY;
}

//#define OSSEMU_FRAGSIZE (1152)
//OSS needs big chunks to work without skipping
#define OSSEMU_FRAGSIZE (PRI_CHUNK_SIZE*8)
#define OSSEMU_FRAGMENTS (8)
#define OSSEMU_BUFFERSIZE (OSSEMU_FRAGSIZE * OSSEMU_FRAGMENTS)


static int coolsound_ossemu_open(struct inode *inode, struct file *fp)
{
	int res;
	struct cs_buffer_description cbd;
	struct cs_ossemu_private *priv;

	#ifdef TIMING_INFO
	wts_num = 0;
	#endif

	sound_setup_primary_frequency(INITIAL_SAMPLERATE);

	if (!(fp->f_mode & FMODE_WRITE || fp->f_mode & FMODE_READ))
		return -ENODEV;

	memset(&cbd, 0, sizeof(cbd));

	cbd.frequency 		= priOutputDesc.frequency;
	cbd.channels		= priOutputDesc.channels >> SECONDARY_CHANNEL_SHIFT;
	cbd.bits_per_sample	= priOutputDesc.bits_per_sample;
	printk("oss f %d, c %d\n",cbd.frequency,cbd.channels);
	#ifdef TDEV
	cbd.samples		= (WRITE_CHUNKS*WRITE_BYTES) + (WRITE_CHUNKS*sizeof(struct tdev_link));
	#else
	cbd.samples		= OSSEMU_BUFFERSIZE / (cbd.channels * 2);
	#endif
	cbd.type		= CSB_TYPE_PLAYBACK;

	res = sound_request_buffer(&cbd, fp);
	if(res < 0) return res;

	priv = fp->private_data = kmalloc(sizeof(struct cs_ossemu_private), GFP_USER);
	if(!priv) {
		csb_free(res, 1);
		return -ENOMEM;
	}

	memset(csb[res].buffer, 0, csb_size(res));

	priv->buffno_read 	= -1;
	priv->buffno_write	= res;
	priv->pointer_read	= 0;
	priv->pointer_write = 0;
	priv->start_write = 0;
	priv->start_read = 0;

	IO_Deactivate(MUTE_EXT);
	IO_Activate(AMP_ON);

	printk("oss volume %d\n",INITIAL_VOLUME);
	codec_set_volume_external(INITIAL_VOLUME);
	codec_set_volume_internal(INITIAL_VOLUME);

	#ifdef TDEV
	bufnum = res;
	unsigned char* data = csb[bufnum].buffer;
	wlink = (struct tdev_link*)(data + (WRITE_CHUNKS*WRITE_BYTES));
	outlink = wlink;
	llink = wlink;
	out_count = 0;
	struct tdev_link* plink = wlink;
	struct tdev_link* nlink = wlink;
	int i = 0;
	for (;i < WRITE_CHUNKS; i++) {
		nlink = (struct tdev_link*)((unsigned char*)plink + sizeof(struct tdev_link));
		plink->active = 0;
		plink->data = data;
		plink->offset = 0;
		plink->next = nlink;
		data += WRITE_BYTES;
		plink = nlink;
	}
	plink = (struct tdev_link*)((unsigned char*)plink - sizeof(struct tdev_link));
	plink->next = wlink;

	unsigned long flags;
	local_irq_save(flags);
	sound_dma_output_reset();
	iis_control_cmd(IIS_ALIGN_TX);
	if ((res = sound_dma_output_start())) {
		printk("could not start primary output buffer (%d).\n", res);
		return res;
	}
	iis_control_cmd(IIS_ACTIVE_TX);
	local_irq_restore(flags);
	#endif

	return 0;
}

static int coolsound_ossemu_release(struct inode *inode, struct file *fp)
{
	struct cs_ossemu_private *priv = (struct cs_ossemu_private *)fp->private_data;

	if (!(fp->f_mode & FMODE_WRITE || fp->f_mode & FMODE_READ))
		return -ENODEV;

	#ifdef TDEV
		iis_control_cmd(IIS_IDLE_TX);
		sound_dma_output_stop();
		iis_control_cmd(IIS_FLUSH_TX);
	#endif

	csb_free(priv->buffno_read, 1);
	csb_free(priv->buffno_write, 1);

	kfree(priv);

	return 0;
}

#ifdef TIMING_INFO
static void coolsound_timing_info(void) {
	int i = 0;
	while (i < wts_num) {
		printk("%ld:%ld %d ",wts[i].tv.tv_sec,wts[i].tv.tv_usec,wts[i].bytes);
		if ((wts[i].tv.tv_usec + ((wts[i].bytes >> 2) * 125)) >= 1000000) printk("%ld:%ld\n",wts[i].tv.tv_sec + 1, ((wts[i].tv.tv_usec + ((wts[i].bytes >> 2) * 125)) - 1000000));
		else printk("%ld:%ld\n",wts[i].tv.tv_sec, (wts[i].tv.tv_usec + ((wts[i].bytes >> 2) * 125)));
		i++;
	}
	printk("\n");
	i = 0;
	struct timeval tv;
	tv.tv_sec = wts[i].tv.tv_sec;
	tv.tv_usec = wts[i].tv.tv_usec;
	printk("start time   ---   write time\n");
	while (i < wts_num) {
		printk("%ld:%ld\n",wts[i].tv.tv_sec,wts[i].tv.tv_usec);
		if ((wts[i].tv.tv_usec + ((wts[i].bytes >> 2) * 125)) >= 1000000) { tv.tv_sec = wts[i].tv.tv_sec + 1; tv.tv_usec = ((wts[i].tv.tv_usec + ((wts[i].bytes >> 2) * 125)) - 1000000); }
		else tv.tv_usec = (wts[i].tv.tv_usec + ((wts[i].bytes >> 2) * 125));
		printk("%ld:%ld ",tv.tv_sec,tv.tv_usec);
		i++;
	}
	printk("\n");
	printk("\n");
	wts_num = 0;
}
#endif

#ifdef TDEV
static ssize_t coolsound_ossemu_write(struct file *fp, const char __user *data, size_t count, loff_t *offset) {
	int written = 0;
	int write = 0;

	#ifdef TIMING_INFO
	if (wts_num < 32) {
		do_gettimeofday(&wts[wts_num].tv);
		wts[wts_num].bytes = count;
		wts_num++;
	}
	#endif

	//printk("dw:%p:%d\n",data,count);
	int active = 0;
	struct tdev_link* plink = wlink;
	while (count) {
		if (wlink->active != 0) { msleep(3); continue; }
		write = count;
		if (write > WRITE_BYTES) write = WRITE_BYTES;
		//printk("w:%d:%d %p:%d:%p %p %d\n",written,count,wlink,wlink->active,wlink->data,data + written,write);
		if ((copy_from_user(wlink->data, data + written, write)) == -1) return -1;
		wlink->offset = 0;
		if (active == -1) wlink->active = write;
		else if (active == 0) active = write;
		else { plink->active = active; wlink->active = write; active = -1; }
		wlink = wlink->next;
		written += write;
		count -= write;
	}
	if (active > 0) plink->active = active;
	return written;
}

#else

static ssize_t coolsound_ossemu_write(struct file *fp, const char __user *data, size_t count, loff_t *offset) {
	struct cs_ossemu_private *priv = (struct cs_ossemu_private *)fp->private_data;
	int buf = priv->buffno_write;
	int bufsize = csb_size(buf);
	int written = 0;
	int res;

	#ifdef TIMING_INFO
	if (wts_num < 32) {
		do_gettimeofday(&wts[wts_num].tv);
		wts[wts_num].bytes = count;
		wts_num++;
	}
	#endif

	if(buf < 0) return -EINVAL;
	if(!bufsize) return -EINVAL;

	while(count) {
		int maxwrite;
		int pos = csb_getpos(buf);
		if(pos < 0) return -EINVAL;

		maxwrite = bufsize - priv->pointer_write;
		if(pos == 0) maxwrite -=1;
		if(pos > priv->pointer_write) maxwrite = pos - (priv->pointer_write + 1);

		if(maxwrite > count) maxwrite = count;

		if(maxwrite == 0) {
			if(csb[buf].state!=CS_BUFFER_RUNNING) {
				int err = csb_run(buf);
				if(err) return err;
				priv->start_write = 1;
			}

			msleep(3);
			continue;
		}

		res = copy_from_user(csb[buf].buffer + priv->pointer_write, data + written, maxwrite);
		if(res) return res;

		priv->pointer_write += maxwrite;
		while(priv->pointer_write >= bufsize) {
			priv->pointer_write -= bufsize;
		}
		written += maxwrite;
		count -= maxwrite;
	}

	if((csb[buf].state!=CS_BUFFER_RUNNING) && (priv->pointer_write > (PRI_CHUNK_SIZE * PRI_CHUNKS))) {
		int err = csb_run(buf);
		if(err) return err;
		priv->start_write = 1;
	}

	return written;
}
#endif

static void coolsound_ossemu_sync(struct file *fp) {
	#ifdef TDEV
	while(outlink->active != 0) msleep(3);
	#else
	struct cs_ossemu_private *priv = (struct cs_ossemu_private *)fp->private_data;
	struct cb_qstop qs;
	int buf = priv->buffno_write;

	if((csb[buf].state!=CS_BUFFER_RUNNING) && (priv->start_write == 0)) {
		size_t bytes = (PRI_CHUNK_SIZE * PRI_CHUNKS) - priv->pointer_write;
		if (bytes > 0) bytes = (priv->pointer_write + bytes + (PRI_CHUNK_SIZE - 1)) & ~(PRI_CHUNK_SIZE);
		else bytes = (priv->pointer_write + (PRI_CHUNK_SIZE - 1)) & ~(PRI_CHUNK_SIZE);

		if (bytes > 0) memset(csb[buf].buffer + priv->pointer_write, 0, bytes);
		csb_run(buf);
	}

	qs.buffer = buf;
	qs.stop_position = priv->pointer_write;
	csb_queue_stop(&qs);

	while(csb[buf].state == CS_BUFFER_STOP_PENDING) msleep(3);

	csb_setpos(buf, 0);
	priv->pointer_write = 0;
	priv->start_write = 0;
	#endif
	#ifdef TIMING_INFO
	coolsound_timing_info();
	#endif
}

static int coolsound_ossemu_ioctl(struct inode *inode, struct file *fp, unsigned int cmd, unsigned long arg) {
	struct cs_ossemu_private *priv = (struct cs_ossemu_private *)fp->private_data;

	unsigned int ret = 0;
	unsigned int req = 0;

	void __user *pArg = (void __user *)arg;

	if (_IOC_TYPE(cmd) != 'P') return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_WRITE)
		if (!access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd))) return -EFAULT;

	if (_IOC_DIR(cmd) & _IOC_READ)
		if (!access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd))) return -EFAULT;

	switch(cmd) {
		case SNDCTL_DSP_RESET:
			csb_stop(priv->buffno_write);
			csb_stop(priv->buffno_read);

			csb_setpos(priv->buffno_write, 0);
			csb_setpos(priv->buffno_read, 0);

			priv->pointer_write = 0;
			priv->pointer_read  = 0;

			return 0;

		case SNDCTL_DSP_SPEED:
			req = *((unsigned int *)pArg);
			ret = sound_setup_primary_frequency(req);
			sound_set_buffer_frequency(priv->buffno_write, ret);
			printk("OSS req:%u ret:%u\n", req, ret);

			break;

		case SNDCTL_DSP_CHANNELS:
			ret =  priOutputDesc.channels;
			break;

		case SNDCTL_DSP_STEREO:
			ret = priOutputDesc.channels==2;
			break;

		case SNDCTL_DSP_GETFMTS:
		case SNDCTL_DSP_SETFMT:
			if(priOutputDesc.bits_per_sample == 8) ret = AFMT_S8;
			else if(priOutputDesc.bits_per_sample == 16) ret = AFMT_S16_NE;
			else return -EINVAL;
			break;

		case SNDCTL_DSP_GETODELAY: {
			int wp = priv->pointer_write;
			int rp = csb_getpos(priv->buffno_write);

			if(rp < 0) {
				return -EINVAL;
			}

			if(rp == wp) ret = 0;
			if(wp  > rp) ret = wp - rp;
			if(rp  > wp) ret = wp + (OSSEMU_BUFFERSIZE-wp);
			break;
		}

		case SNDCTL_DSP_GETOSPACE: {
			audio_buf_info abi;

			int wp = priv->pointer_write;
			int rp = csb_getpos(priv->buffno_write);

			if(rp < 0) {
				return -EINVAL;
			}

			abi.fragstotal 	= OSSEMU_FRAGMENTS;
			abi.fragsize   	= OSSEMU_FRAGSIZE;

			if(rp == wp) 	abi.bytes = OSSEMU_BUFFERSIZE - 1;
			if(rp > wp) 	abi.bytes = (rp - wp) - 1;
			if(wp > rp)		abi.bytes = (rp + (OSSEMU_BUFFERSIZE-wp)) - 1;

			abi.fragments	= abi.bytes / abi.fragsize;

			if(copy_to_user(pArg, (void *)&abi, sizeof(abi))) {
				return -EFAULT;
			}
			return 0;
		}

		case SNDCTL_DSP_SYNC:
			coolsound_ossemu_sync(fp);
			return 0;

		default:
			PK_WARN("ossemu_ioctl: received unknown ioctl nr %u.\n", _IOC_NR(cmd));
			return -ENOTTY;
	}

	if(put_user(ret, (int __user *)pArg) && (ret > 0))
		return -EFAULT;

	return 0;
}

static void coolsound_vma_open(struct vm_area_struct *vma) {
	struct cs_buffer *csb = vma->vm_private_data;
	csb->mapcount++;
}

static void coolsound_vma_close(struct vm_area_struct *vma) {
	struct cs_buffer *csb = vma->vm_private_data;
	csb->mapcount--;
}


static struct page *coolsound_vma_nopage(struct vm_area_struct *vma, unsigned long address, int *type) {
	unsigned long offset;
	struct cs_buffer *csb = vma->vm_private_data;
	struct page *page = NOPAGE_SIGBUS;

	offset = (address - vma->vm_start) + (vma->vm_pgoff << PAGE_SHIFT);


	if(offset > csb->size) {
		return NULL;
	}

	page = vmalloc_to_page(csb->buffer + offset);
	get_page(page);
	if(type) *type = VM_FAULT_MINOR;

	return page;
}

static struct vm_operations_struct coolsound_vm_ops = {
	.open	 = coolsound_vma_open,
	.close	 = coolsound_vma_close,
	.nopage	 = coolsound_vma_nopage,
};

static int sound_mmap(struct file *fp, struct vm_area_struct *vma)
{
	vma->vm_ops			 = &coolsound_vm_ops;
	vma->vm_flags	   	|= VM_RESERVED;
	vma->vm_private_data = &csb[csb_selected];

	coolsound_vma_open(vma);

	return 0;
}

static struct file_operations coolsound_native_fops = {
	.owner   = THIS_MODULE,
	.open    = sound_open,
	.release = sound_release,
	.ioctl   = coolsound_native_ioctl,
	.mmap    = sound_mmap,
	.fasync  = sound_fasync
};

static struct file_operations coolsound_ossemu_fops = {
	.owner   = THIS_MODULE,
	.open    = coolsound_ossemu_open,
	.release = coolsound_ossemu_release,
	.ioctl   = coolsound_ossemu_ioctl,
	.write   = coolsound_ossemu_write,
//	.read	 = coolsound_ossemu_read,
	.fasync  = sound_fasync
};


static void sound_poweron(void)
{
	IO_SetInput(HEADPHONE_DETECT);

	IO_Deactivate(AMP_ON);
	IO_Deactivate(MUTE_INT);
	IO_Activate(DAC_PWR_ON);
	msleep(50);

	IO_Deactivate(I2SSDO);
	IO_SetInput(CDCLK_12MHZ);
	IO_SetFunction(CDCLK);
	IO_SetFunction(I2SSCLK);
	IO_SetFunction(I2SLRCK);
	IO_SetFunction(I2SSDI);

	IO_Activate(MUTE_INT);
	msleep(50);

	IO_Deactivate(MIC_SW);
}

static void sound_poweroff(void)
{
	IO_Deactivate(MUTE_EXT);
	IO_Deactivate(AMP_ON);
	IO_Deactivate(MUTE_INT);
	IO_Deactivate(DAC_PWR_ON);
	gpio_force_update();
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
unsigned long int slave_rate_calc( unsigned long int new_pclk, unsigned long int div, unsigned long int arg )
{
	enum codec_fs	fs=(enum codec_fs) arg;

	return (new_pclk/(fs * div));
}

unsigned long int intmast_rate_calc( unsigned long int new_pclk, unsigned long int div, unsigned long int arg )
{
	return (new_pclk/div);
}

void find_match( unsigned long int arg, unsigned long int new_pclk, unsigned long int *real_rate, unsigned long int *div,
		 unsigned long int (*rate_calc)( unsigned long int, unsigned long int, unsigned long int ), unsigned long int rate )
{
	unsigned long int	old_diff=0;
	unsigned long int	new_diff=new_pclk;

	*div=0;
	*real_rate=0;

	do
	{
		*div+=1;
		*real_rate=rate_calc( new_pclk, *div, arg );

		old_diff=new_diff;

		if( *real_rate < rate )
			new_diff=rate - *real_rate;
		else
			new_diff=*real_rate - rate;
	}
	while( (old_diff > new_diff) && (*div <= 32) );

	*div-=1;
	*real_rate=rate_calc( new_pclk, *div, arg );
	return;
}

#define slave_find_match( fs, new_pclk, real_rate, div ) find_match( ((enum codec_fs) fs), new_pclk, real_rate, div, slave_rate_calc, current_samplerate )
#define intmast_find_match( new_pclk, real_rate, div ) find_match( 0, new_pclk, real_rate, div, intmast_rate_calc, iis_clockrate )

void sound_get_minmax_pclk( unsigned long int *min_pclk, unsigned long int *max_pclk )
{
	switch( IO_GetCodecMaster() )
	{
		case GOCODECCFG_INTERNAL_MASTER:
			/* Divider is in use from pclk. */
			*min_pclk=iis_clockrate * 1;
			*max_pclk=iis_clockrate * 32;
			break;

		case GOCODECCFG_EXTERNAL_MASTER:
			/* Doesn't matter. The clock is not divided from pclk. */
			*min_pclk=0;
			*max_pclk=0xFFFFFFFF;
			break;

		case GOCODECCFG_SLAVE:
		default:
			/* Also derived from pclk. */
			*min_pclk=FS256 * 1 * current_samplerate;
			*max_pclk=FS384 * 32 * current_samplerate;
			break;
	}
	return;
}

void sound_set_new_rate( unsigned long int pclk_new )
{ 
	unsigned long int	realrate[2];
	unsigned long int	div[2];

	/* Only set the new rate if the rate is legal. Otherwise leave sound disabled. */
	switch( IO_GetCodecMaster() )
	{
		case GOCODECCFG_INTERNAL_MASTER:
			/* Internal master. Determine the best divider. */
			intmast_find_match( pclk_new, &(realrate[0]), &(div[0]) );
			iis_clockrate=realrate[0];
			iis_clk_div=div[0];

			/* Set the requested samplerate. */
			iis_set_samplerate( current_samplerate );
			break;

		case GOCODECCFG_EXTERNAL_MASTER:
			/* External master has it's own clock. No need to calculate. */
			break;

		case GOCODECCFG_SLAVE:
		default:
			/* Slave mode. Determine the best divider and fs. */
			slave_find_match( FS256, pclk_new, &(realrate[0]), &(div[0]) );
			slave_find_match( FS384, pclk_new, &(realrate[1]), &(div[1]) );
			if( realrate[0] > current_samplerate )
				realrate[0]-=current_samplerate;
			else
				realrate[0]=current_samplerate - realrate[0];
			if( realrate[1] > current_samplerate )
				realrate[1]-=current_samplerate;
			else
				realrate[1]=current_samplerate - realrate[1];

			/* Closest match doesn't differ much from the real rate. */
			if( realrate[0] < realrate[1] )
				iis_clk_div=div[0];
			else
				iis_clk_div=div[1];
			iis_clockrate=pclk_new;

			/* Set the requested samplerate. */
			iis_set_samplerate( current_samplerate );
	}

	/* Done. */
	return;
}
/*
 * CPU clock speed change handler. Change samplerate, or refuse policy.
 */
static int
sound_freq_transition(struct notifier_block *nb, unsigned long val,
			 void *data)
{
	unsigned long int		pclk_new;
	unsigned long int		pclk_old;
	unsigned long int		pclk_min;
	unsigned long int		pclk_max;
	struct cpufreq_freqs		*f = data;
	static unsigned long int	iiscon_sav;

	f->trans2pclk( f, &pclk_old, &pclk_new );
	sound_get_minmax_pclk( &pclk_min, &pclk_max );
	pclk_old*=1000;
	pclk_new*=1000;

	switch (val) {
	case CPUFREQ_PRECHANGE:
		/* If the current clock is legal, we need to disable the tasklet and clock. */
		if( (pclk_old >= pclk_min) || (pclk_old <= pclk_max) )
		{
			/* Check if we transition from illegal. If so we don't need to do anything. */
			tasklet_disable(&magic_tasklet_handle);
			switch( IO_GetCpuType() )
			{
				case GOCPU_S3C2412:
				case GOCPU_S3C2443:
				case GOCPU_S3C2450:
					iiscon_sav=readl( S3C2412_IISCON );
					writel( S3C2412_IISCON_TXDMAPAUSE | S3C2412_IISCON_RXDMAPAUSE | 
						S3C2412_IISCON_TXCHPAUSE | S3C2412_IISCON_RXCHPAUSE | iiscon_sav,
						S3C2412_IISCON );
					break;

				default:
					iiscon_sav=readl( S3C2410_IISCON );
					writel( iiscon_sav & ~(S3C2410_IISCON_IISEN), S3C2410_IISCON );
					break;
			}

			/* If we switch from a low to a high frequency the chance exists the divider will be too low. */
			/* In this case adjust the divider now. */
			if( pclk_new >= pclk_old )
				sound_set_new_rate( pclk_new );
		}
		break;

	case CPUFREQ_POSTCHANGE:
		/* Determine the new rate. */
		if( (pclk_new >= pclk_min) || (pclk_new <= pclk_max) )
		{
			/* If the new clock is lower than the old clock, we should change the divider after the transition. */
			if( pclk_new < pclk_old )
				sound_set_new_rate( pclk_new );

			/* Reenable clock and start the tasklet again. */
			switch( IO_GetCpuType() )
			{
				case GOCPU_S3C2412:
				case GOCPU_S3C2443:
				case GOCPU_S3C2450:
					writel( iiscon_sav, S3C2412_IISCON );
					break;

				default:
					writel( iiscon_sav, S3C2410_IISCON );
					break;
			}
			tasklet_enable(&magic_tasklet_handle);
		}
		break;
	}
	return 0;
}

static int
sound_freq_policy(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	unsigned long int	pclk_max;
	unsigned long int	pclk_min;
	unsigned long int	pclk_min_lim;
	unsigned long int	pclk_max_lim;
	struct cpufreq_policy	*policy = data;

	policy->policy2pclk( policy, &pclk_min, &pclk_max );
	sound_get_minmax_pclk( &pclk_min_lim, &pclk_max_lim );

	switch (val) {
	case CPUFREQ_ADJUST:
		policy->pclk2policy( policy, pclk_min, pclk_max );
		break;
	case CPUFREQ_INCOMPATIBLE:
		if( (pclk_min >= pclk_min_lim) && (pclk_min <= pclk_max_lim) )
			pclk_min_lim=pclk_min;

		if( (pclk_max >= pclk_min_lim) && (pclk_max <= pclk_max_lim) )
			pclk_max_lim=pclk_max;

		if( (pclk_min_lim != pclk_min) || (pclk_max_lim != pclk_max) )
			policy->pclk2policy( policy, pclk_min_lim, pclk_max_lim );
		break;
	case CPUFREQ_NOTIFY:
		/* Transition routine handles illegal values. */
		break;
	}
	return 0;
}
#endif

static int sound_probe(struct device *dev)
{
	int ret = 0;
	unsigned int divider;

	if((ret = sound_dma_init(dev, cs_callback))) {
		PK_ERR("Unable to associate primary DMA buffers. Error:%d\n", ret);
		return ret;
	}

#ifdef TDEV
	if((ret = sound_dma_setup_buffers(DMA_SIZE, DMA_CHUNKS))) {
		PK_ERR("Unable to allocate primary DMA buffers. Error:%d\n", ret);
		return ret;
	}
#else
	if((ret = sound_dma_setup_buffers(PRI_CHUNK_SIZE, PRI_CHUNKS))) {
		PK_ERR("Unable to allocate primary DMA buffers. Error:%d\n", ret);
		return ret;
	}
#endif

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	freq_transition.notifier_call = sound_freq_transition;
	freq_transition.priority = CPUFREQ_ORDER_S3C24XX_SOUND_PRIO;
	freq_policy.notifier_call = sound_freq_policy;
	cpufreq_register_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	PK_DBG("Registering chardev %s (%p)\n", sound_name, dev);
	ret = register_chrdev(COOLSOUND_NATIVE_MAJOR, sound_name, &coolsound_native_fops);
	if (ret < 0) {
		PK_ERR("Unable to register chardev on major=%d (%d)\n", COOLSOUND_NATIVE_MAJOR, ret);
		return ret;
	}

	ret = register_chrdev(COOLSOUND_OSSEMU_MAJOR, sound_name, &coolsound_ossemu_fops);
	if (ret < 0) {
		PK_ERR("Unable to register chardev on major=%d (%d)\n", COOLSOUND_OSSEMU_MAJOR, ret);
		return ret;
	}
	
	divider = ((struct barcelona_sound_info*) (dev->platform_data ))->divider;
	
	sound_poweron();
	iis_init( divider );
	return 0;
}


static int sound_remove(struct device *dev)
{
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	cpufreq_unregister_notifier(&freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	tasklet_kill(&magic_tasklet_handle);
	iis_exit();
	sound_poweroff();

	PK_DBG("Unregistering chardev\n");
	unregister_chrdev(COOLSOUND_NATIVE_MAJOR, sound_name);
	unregister_chrdev(COOLSOUND_OSSEMU_MAJOR, sound_name);
	return 0;
}

static void sound_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");

	tasklet_kill(&magic_tasklet_handle);
	iis_exit();
	sound_poweroff();
}

#ifdef CONFIG_PM

static int sound_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);
	if (level == SUSPEND_POWER_DOWN) {
		/* TODO: Need to notify cs_do_magic that it needs to quit ASAP.
		 * Then we must WAIT here until it's finished, before returning
		 * from this function, or else the tasklet will be killed in
		 * the middle of its work. */
		sound_poweroff();
		sound_dma_suspend();
		codec_suspend();
		iis_suspend();
	}
	return 0;
}

static int sound_resume(struct device *dev, u32 level)
{
	PK_DBG("dev = %p, level = %u\n", dev, level);
	if (level == RESUME_POWER_ON) {
		sound_poweron();	/* Setup our GPIO pins */
		iis_resume();		/* Startup codec/iis */
		sound_dma_resume();	/* Startup DMA */
	}
	return 0;
}

#else /* CONFIG_PM */
#define sound_suspend NULL
#define sound_resume  NULL
#endif /* CONFIG_PM */

static struct device_driver sound_driver = {
	.name		= "s3c2410-iis",
	.bus		= &platform_bus_type,
	.probe		= sound_probe,
	.remove		= sound_remove,
	.shutdown	= sound_shutdown,
	.suspend	= sound_suspend,
	.resume		= sound_resume,
};

static int __init sound_mod_init(void)
{
	int ret;

	fasync_queue = NULL;
	do_filter = 0;

	csb_init();

	printk(KERN_INFO "CoolSound engine, (C) 2004,2005 TomTom BV\n");
	PK_DBG("Registering driver\n");
	ret = driver_register(&sound_driver);
	if (ret) {
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit sound_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&sound_driver);
	PK_DBG("Done\n");
}

late_initcall(sound_mod_init);
module_exit(sound_mod_exit);

MODULE_AUTHOR("Thomas Kleffel <tk@maintech.de> ((c) 2004,2005 TomTom BV)");
MODULE_DESCRIPTION("CoolSound engine");
MODULE_LICENSE("GPL");

