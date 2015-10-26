#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>

#include <asm/errno.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/hardware.h>
#include <barcelona/Barc_snd.h>

#include <barcelona/gotype.h>

#include "dma.h"

#define DMA_SIZEMASK (0xFFFFF)

struct sDMABuffer primaryOutput;
struct sDMABuffer primaryInput;

int DMABuffer_running(struct sDMABuffer *pBuffer);
void DMABuffer_stop(struct sDMABuffer *pBuffer);

#ifdef TDEV
extern struct tdev_link* outlink;
extern int silence;
extern struct tdev_link* llink;
extern int out_count;
void output_callback(s3c_dma_chan_t *chan, int chunk) {
#if 0
	//printk("oc:%p %i\n",outlink,chunk);
	unsigned* buf = primaryOutput.virt + (chunk * DMA_SIZE);
	int count = 0;
	while (count < DMA_SIZE) {
		int write = outlink->active;
		int written = 0;
		unsigned char* data = outlink->data + outlink->offset;
		//printk("  :%x %x %i\n",buf,data,write,outlink->offset);
		while (written < write) {
			//printk("  :%d %d\n",count,written);
			int bytes = write - written;
			if (bytes > (DMA_SIZE - count)) bytes = DMA_SIZE - count;
			//printk("  :%i\n",bytes);
			if (bytes > 0) {
				//printk("  :%x %i %x %i\n",(buf + (count >> 2)), count, (data + (written >> 2)), bytes);
				memcpy((buf + (count >> 2)), (data + (written >> 2)), bytes);
				written += bytes;
				count += bytes;
			} else break;
		}
		if (written == 0) break;
		outlink->offset = written;
		written = write - written;
		outlink->active = written;
		//printk("  :w %i\n",written);
		if (written == 0) outlink = outlink->next;
	}

	//printk("--:%d, %x %i\n",count,(buf + (count >> 2)),DMA_SIZE - count);
	if (count < DMA_SIZE) memset((buf + (count >> 2)), 0, DMA_SIZE - count);
#else
	//printk("oc:%p %i\n",outlink,chunk);
	if (silence != 0) {
		while (1) {
			if (out_count >= PRE_BUFFER) { silence = 0; break; }
			int add_count = llink->active;
			if (add_count == 0) break;
			out_count += add_count;
			llink = llink->next;
		}
	}

	unsigned* buf = primaryOutput.virt + (chunk * DMA_SIZE);
	int count = 0;
	if (silence == 0) {
	while (count < DMA_SIZE) {
		int write = outlink->active;
		int written = 0;
		unsigned char* data = outlink->data + outlink->offset;
		//printk("  :%x %x %i\n",buf,data,write,outlink->offset);
		while (written < write) {
			//printk("  :%d %d\n",count,written);
			int bytes = write - written;
			if (bytes > ((DMA_SIZE - count) >> 1)) bytes = (DMA_SIZE - count) >> 1;
			//printk("  :%i\n",bytes);
			if (bytes > 0) {
				//printk("  :%x %i %x %i\n",(buf + (count >> 2)), count, (data + (written >> 2)), bytes);
				size_t samples = (size_t)(bytes >> 1);
				int16_t* s =  (int16_t*)(data + (written >> 2));
				int16_t* d =  (int16_t*)(buf + (count >> 2));
				while (samples--) {
					*(d++) = *(s);
					*(d++) = *(s++);
				}
				written += bytes;
				count += (bytes << 1);
			} else break;
		}
		if (written == 0) break;
		outlink->offset = written;
		written = write - written;
		outlink->active = written;
		//printk("  :w %i\n",written);
		if (written == 0) outlink = outlink->next;
	}
	}

	//printk("--:%d, %x %i\n",count,(buf + (count >> 2)),DMA_SIZE - count);
	if (count < DMA_SIZE) { silence = 1; out_count = 0; llink = outlink; memset((buf + (count >> 2)), 0, DMA_SIZE - count); }
#endif
}
#else
void output_callback(s3c_dma_chan_t *chan, int chunk) {
	if(primaryOutput.fill > primaryOutput.chunksize) {
		primaryOutput.fill -= primaryOutput.chunksize;
		primaryOutput.underrun = 0;
		//printk("CoolSound: OutputCB: chunk %d done. PrimaryOutput fill:%u\n", chunk, primaryOutput.fill);
	} else {
		if(!primaryOutput.underrun) printk("CoolSound: Warning: Primary output buffer underrun.\n");
		primaryOutput.fill = 0;
		primaryOutput.underrun = 1;
	}

	if(primaryOutput.callback) primaryOutput.callback(DMAP_PLAY, chunk);
}
#endif

void input_callback(s3c_dma_chan_t *chan, int chunk) {
	if((primaryInput.size-primaryInput.fill) > primaryInput.chunksize) {
		primaryInput.fill += primaryInput.chunksize;
		primaryInput.overflow = 0;
		//printk("CoolSound: InputCB: chunk %d done. PrimaryInput fill:%u\n", chunk, primaryInput.fill);
	} else {
		if(!primaryInput.overflow) printk("CoolSound: Warning: Primary input buffer overflow.\n");
		primaryInput.fill = primaryInput.size;
		primaryInput.overflow = 1;
	}


	if(primaryInput.callback) primaryInput.callback(DMAP_RECORD, chunk);
}

void DMABuffer_free(struct sDMABuffer *pBuffer) {
	//return;
	if(!pBuffer) return;

	if(pBuffer->flags & DBF_ALLOCATED)
		dma_free_coherent(pBuffer->pDev, pBuffer->real_size, pBuffer->virt, pBuffer->phys);

	pBuffer->size = 0;
	pBuffer->real_size = 0;
	pBuffer->virt = NULL;
	pBuffer->phys = (dma_addr_t) NULL;

	pBuffer->flags &= ~DBF_ALLOCATED;
}

#define MINIMUM_DMABUFSIZE    (64 * 1024)
int DMABuffer_allocate(struct sDMABuffer *pBuffer, size_t chunksize, int chunks) {
	size_t size;

	if(!pBuffer) return -EINVAL;
	if(chunks < 3) return -EINVAL;

	size = chunks * chunksize;

	//Check if requested size is too big for DMA
	if(size > DMA_SIZEMASK) return -EINVAL;

	// Only reallocate buffer if it is buffer
	if (size > pBuffer->real_size) {
    size_t new_real_size = max(size, (size_t)(MINIMUM_DMABUFSIZE));

		//Calling this function is save in any case
		DMABuffer_free(pBuffer);

    printk("reallocating sound DMA buffer (previous:%zu new:%zu allocating:%zu)\n", pBuffer->real_size, size, new_real_size);
		pBuffer->virt = dma_alloc_coherent(pBuffer->pDev, new_real_size, &pBuffer->phys, GFP_KERNEL|GFP_DMA);
		//pBuffer->virt = dma_alloc_writecombine(pBuffer->pDev, size, &pBuffer->phys, GFP_KERNEL|GFP_DMA);

		if(!pBuffer->virt) {
			printk("CoolSound: allocation of %u Bytes coherent dma buffer failed.\n", size);
			DMABuffer_free(pBuffer);
			return -ENOMEM;
		}

    pBuffer->real_size 	= new_real_size;
	}

	pBuffer->chunksize = chunksize;
	pBuffer->chunks = chunks;
	pBuffer->size 	= size;
	pBuffer->fill	= 0;
	pBuffer->flags |= DBF_ALLOCATED;

	return 0;
}

void DMABuffer_disassociate(struct sDMABuffer *pBuffer) {
	if(!pBuffer) return;

	if(DMABuffer_running(pBuffer))
		DMABuffer_stop(pBuffer);
	if(pBuffer->flags & DBF_ASSOCIATED)
		s3c_dma_free(pBuffer->channel, &pBuffer->client);

	pBuffer->flags &= ~DBF_ASSOCIATED;
}

int DMABuffer_associate(struct sDMABuffer *pBuffer, enum eDMAPurpose purpose) {
	if(!pBuffer) return -EINVAL;

	DMABuffer_disassociate(pBuffer);

	if(purpose == DMAP_PLAY) {
		pBuffer->client.name = "TomTom CoolSound Play";
		if(s3c_dma_request(2, &pBuffer->client, pBuffer->pDev)) return -EBUSY;
		s3c_dma_devconfig(2, S3C_DMASRC_MEM, 3, 0x55000010 );
		s3c_dma_config(2, DMA_UNIT_SIZE, 0, S3C_REQSEL_IIS_TX| S3C_REQSEL_HWTRIG );
		s3c_dma_setflags(2, S3C_DMAF_AUTOSTART | S3C_DMAF_LOOPING);
		s3c_dma_set_loopchunk_fn(2, output_callback);
		pBuffer->channel = 2;
		pBuffer->flags |= DBF_ASSOCIATED;
		return 0;
	}

	if(purpose == DMAP_RECORD) {
		pBuffer->client.name = "TomTom CoolSound Record";
		if(s3c_dma_request(1, &pBuffer->client, pBuffer->pDev)) return -EBUSY;
		
		if ((IO_GetCpuType() == GOCPU_S3C2412) || 
                    (IO_GetCpuType() == GOCPU_S3C2443) ||
                    (IO_GetCpuType() == GOCPU_S3C2450) )
			s3c_dma_devconfig(1, S3C_DMASRC_HW, 3, 0x55000014 );
		else
			s3c_dma_devconfig(1, S3C_DMASRC_HW, 3, 0x55000010 );
		
		s3c_dma_config(1, DMA_UNIT_SIZE, 0, S3C_REQSEL_IIS_RX| S3C_REQSEL_HWTRIG);
		s3c_dma_setflags(1, S3C_DMAF_AUTOSTART | S3C_DMAF_LOOPING);
		s3c_dma_set_loopchunk_fn(1, input_callback);
		pBuffer->channel = 1;
		pBuffer->flags |= DBF_ASSOCIATED;
		return 0;
	}

	return -EINVAL;
}


int DMABuffer_run(struct sDMABuffer *pBuffer) {
	int ret;

	pr_debug("DMABufferRun [%p]\n",pBuffer);
	if(!pBuffer) return -EINVAL;

	if(!(pBuffer->flags & DBF_ALLOCATED)) return -EBUSY;
	if(!(pBuffer->flags & DBF_ASSOCIATED)) return -EBUSY;

	if(pBuffer->flags & DBF_RUNNING) return 0;
	pr_debug("DMABufferRun2 [%p]\n",pBuffer);

	pBuffer->flags |= DBF_RUNNING;

	if((ret = s3c_dma_setlooping(pBuffer->channel, pBuffer->phys, pBuffer->size, pBuffer->chunks)))
		return ret;

	pr_debug("Buffer[%p] now running.\n", pBuffer);
	return 0;
}

int DMABuffer_running(struct sDMABuffer *pBuffer) {
	if(!pBuffer) return 0;
	return (pBuffer->flags & DBF_RUNNING);
}

int DMABuffer_ready(struct sDMABuffer *pBuffer) {
	if(!pBuffer) return 0;
	return (pBuffer->flags & ( DBF_ASSOCIATED | DBF_ALLOCATED ));
}

int DMABuffer_getRWPos(struct sDMABuffer *pBuffer) {
	unsigned long pos;
	int res;
	//printk("getRWPos\n");
	if(!pBuffer) return -EINVAL;

	if(!(pBuffer->flags & DBF_ALLOCATED)) return -EBUSY;
	if(!(pBuffer->flags & DBF_ASSOCIATED)) return -EBUSY;

	if(!(pBuffer->flags & DBF_RUNNING)) {
		//printk("Buffer[%p] getRWPos while not running.\n", pBuffer);
		return 0;
	}

	res = s3c_dma_getpos(pBuffer->channel, NULL, &pos);
	if(res) return res;

	return pos;
}

void DMABuffer_stop(struct sDMABuffer *pBuffer) {
	if(!pBuffer) return;

	if(!(pBuffer->flags & DBF_ASSOCIATED)) return;
	s3c_dma_ctrl(pBuffer->channel, S3C_DMAOP_STOP);
	s3c_dma_ctrl(pBuffer->channel, S3C_DMAOP_FLUSH);
	pBuffer->flags &= (~DBF_RUNNING);
	//printk("Buffer[%p] now stopped.\n");
}


int sound_dma_init(struct device *pDev, sound_dma_callback_t callback) {
	int res;

	//Setup device pointer
	primaryOutput.flags = 0;
	primaryInput.flags = 0;

	primaryOutput.pDev = pDev;
	primaryInput.pDev = pDev;

	primaryOutput.callback = callback;
	primaryInput.callback = callback;

	//Associate buffers
	if((res = DMABuffer_associate(&primaryOutput, DMAP_PLAY))) return res;
	if((res = DMABuffer_associate(&primaryInput, DMAP_RECORD))) return res;

	printk("CoolSound: sound_dma_init: Buffers allocated and associated.\n");
	return 0;
}

int sound_dma_suspend() {
	if (DMABuffer_running(&primaryOutput))
		DMABuffer_stop(&primaryOutput);

	if (DMABuffer_running(&primaryInput))
		DMABuffer_stop(&primaryInput);

	return 0;
}

int sound_dma_resume() {
	int res = 0;

	//DMA suspend/resume is broken - we need to reassociate buffers
	if((res = DMABuffer_associate(&primaryOutput, DMAP_PLAY))) return res;
	if((res = DMABuffer_associate(&primaryInput, DMAP_RECORD))) return res;

	return res;
}

int sound_dma_setup_buffers(int chunksize, int chunks) {
	int res;

	//Allocate buffers
	if((res = DMABuffer_allocate(&primaryOutput, chunksize, chunks))) return res;
	if((res = DMABuffer_allocate(&primaryInput, chunksize, chunks))) return res;

	return 0;
}

int sound_dma_output_running(void) {
	return DMABuffer_running(&primaryOutput);
}

int sound_dma_input_running(void) {
	return DMABuffer_running(&primaryInput);
}

int sound_dma_output_ready(void) {
	return DMABuffer_ready(&primaryOutput);
}

int sound_dma_input_ready(void) {
	return DMABuffer_ready(&primaryInput);
}

int sound_dma_output_start(void) {
	return DMABuffer_run(&primaryOutput);
}

void sound_dma_output_stop(void) {
	DMABuffer_stop(&primaryOutput);
}

int sound_dma_output_pos(void) {
	return DMABuffer_getRWPos(&primaryOutput);
}

int sound_dma_input_start(void) {
	return DMABuffer_run(&primaryInput);
}

void sound_dma_input_stop(void) {
	DMABuffer_stop(&primaryInput);
}

int sound_dma_input_pos(void) {
	return DMABuffer_getRWPos(&primaryInput);
}

int sound_dma_output_allocated(void) {
	return (primaryOutput.flags & DBF_ALLOCATED);
}

size_t sound_dma_output_size(void) {
	return (primaryOutput.size);
}

void *sound_dma_output_phys(void) {
	return (void *)(primaryOutput.phys);
}

int sound_dma_input_allocated(void) {
	return (primaryInput.flags & DBF_ALLOCATED);
}

size_t sound_dma_input_size(void) {
	return (primaryInput.size);
}

void *sound_dma_input_phys(void) {
	return (void *)(primaryInput.phys);
}

void sound_dma_output_reset(void) {
	primaryOutput.pointer 	= 0;
	primaryOutput.fill	= 0;
	primaryOutput.underrun	= 0;
	primaryOutput.overflow	= 0;

}

void sound_dma_input_reset(void) {
	primaryInput.pointer 	= 0;
	primaryInput.fill	= 0;
	primaryInput.underrun	= 0;
	primaryInput.overflow	= 0;
}

int sound_dma_output_free(void) {
	int res;
/*
	int rp, wp;

	rp = DMABuffer_getRWPos(&primaryOutput);
	wp = primaryOutput.pointer;

	if(rp>wp) 	res =  (rp - wp) - 1;
	else 		res = primaryOutput.size - (wp-rp) - 1;
*/

	res = primaryOutput.size - primaryOutput.fill;

	res>>=2;
	//printk("sound_dma_output_free() wp:%d rp:%d free:%d\n",wp, rp, res);
	return res;
}

int sound_dma_input_filled(void) {
	int res;
/*
	int rp, wp;

	wp = DMABuffer_getRWPos(&primaryInput);
	rp = primaryInput.pointer;

	if(wp>=rp) 	res =  (wp - rp);
	else 		res = primaryInput.size - (rp-wp);
*/

	res = primaryInput.fill;

	res>>=2;
	//printk("sound_dma_input_filled() wp:%d rp:%d fill:%d\n",wp, rp, res);
	return res;
}


void sound_dma_output_zeroes(int samples, int offset) {
	int pointer;

	if(samples > (primaryOutput.size>>2)) {
		printk("CoolSound: Warning: sound_dma_output_zero() samples(%i) > buffersize(%u).\n",
			samples, (primaryOutput.size>>2));

		samples = primaryOutput.size >> 2;
	}

	pointer = (primaryOutput.pointer+(offset<<2)) % primaryOutput.size;

	if((primaryOutput.size - pointer) >= (samples<<2)) {
		memset(primaryOutput.virt + pointer, 0, samples<<2);
	} else {
		memset(primaryOutput.virt + pointer, 0, (primaryOutput.size - pointer));
		samples-=(primaryOutput.size - pointer)>>2;
		memset(primaryOutput.virt, 0, samples<<2);
	}
}

void sound_dma_output_put(void *pData, int size, int offset, cs_primixer_t primixer, unsigned int flags) {
	int pointer;

	if (size <= 0)
		return;

	if (pData == NULL) {
		printk(KERN_WARNING "%s: NULL data pointer\n", __FUNCTION__);
		return;
	}

	if ((size<<2) > primaryOutput.size) {
		printk(KERN_WARNING "%s: Invalid size %d\n", __FUNCTION__, size);
		return;
	}

	pointer = (primaryOutput.pointer+(offset<<2)) % primaryOutput.size;

	//printk("sound_dma_output_put(): pointer:%ub buffersize:%ub size:%us offset:%us real_pointer:%ub pData:%p\n",
	//	primaryOutput.pointer, primaryOutput.size, size, offset, pointer, pData);

	if((primaryOutput.size - pointer) >= (size<<2)) {
		//printk("primixer(%p, %p, %zu)\n",primaryOutput.virt + pointer, pData, size);
		//memcpy(primaryOutput.virt + primaryOutput.pointer, pData, size);
		primixer(primaryOutput.virt + pointer, pData, size, flags);
	} else {
		//memcpy(primaryOutput.virt + primaryOutput.pointer, pData, (primaryOutput.size - primaryOutput.pointer));
		//printk("primixer(%p, %p, %zu)\n",primaryOutput.virt + pointer, pData, (primaryOutput.size - pointer)>>2);
		primixer(primaryOutput.virt + pointer, pData, (primaryOutput.size - pointer)>>2, flags);
		size-=(primaryOutput.size - pointer)>>2;
		pData+=(primaryOutput.size - pointer)>>2;
		//memcpy(primaryOutput.virt, pData, size);
		//printk("primixer(%p, %p, %zu)\n",primaryOutput.virt, pData, size);
		primixer(primaryOutput.virt, pData, size, flags);
		//primaryOutput.pointer = size<<2;
	}

	//printk("output_put %d [%d]->[%d]\n", sz, pt, primaryOutput.pointer);
}

void sound_dma_input_get(void *pData, int size, int offset, cs_primixer_t primixer, unsigned int flags) {
	int pointer;

	if (size <= 0)
		return;

	if (pData == NULL) {
		printk(KERN_WARNING "%s: NULL data pointer\n", __FUNCTION__);
		return;
	}

	if ((size<<2) > primaryInput.size) {
		printk(KERN_WARNING "%s: Invalid size %d\n", __FUNCTION__, size);
		return;
	}

	pointer = (primaryInput.pointer+(offset<<2)) % primaryInput.size;

	//printk("sound_dma_input_get() size:%d pointer:%p\n", size, pData);
	if((primaryInput.size - pointer) >= (size<<2)) {
		//printk("primixer(%zu, %zu, %zu)\n",pData, primaryInput.virt + primaryInput.pointer, size);
		primixer(pData, primaryInput.virt + pointer, size, flags);
		//memcpy(pData, primaryInput.virt + primaryInput.pointer, size);
	} else {
		//memcpy(pData, primaryInput.virt + primaryInput.pointer, (primaryInput.size - primaryInput.pointer));
		//printk("primixer(%zu, %zu, %zu)\n",pData, primaryInput.virt + primaryInput.pointer, (primaryInput.size - primaryInput.pointer)>>2);
		primixer(pData, primaryInput.virt + pointer, (primaryInput.size - pointer)>>2, flags);
		size 	-= (primaryInput.size - pointer)>>2;
		pData	+= (primaryInput.size - pointer)>>2;
		//memcpy(pData, primaryInput.virt, size);
		//printk("primixer(%zu, %zu, %zu)\n",pData, primaryInput.virt, size);
		primixer(pData, primaryInput.virt, size, flags);
	}
}

void sound_dma_input_advance(int size) {
	unsigned long flags;

	primaryInput.pointer = (primaryInput.pointer + (size<<2)) % primaryInput.size;

	local_irq_save(flags);
	if(primaryInput.fill < (size<<2)) {
		printk("CoolSound: Warning: primary Input buffer underrun!\n");
		primaryInput.fill = 0;
	} else {
		primaryInput.fill -= (size<<2);
	}
	local_irq_restore(flags);
}

void sound_dma_output_advance(int size) {
	unsigned long flags;

	primaryOutput.pointer = (primaryOutput.pointer + (size<<2)) % primaryOutput.size;

	local_irq_save(flags);
	primaryOutput.fill += (size<<2);
	if(primaryOutput.fill > primaryOutput.size) {
		printk("CoolSound: Warning: primary Output buffer overflow!\n");
		primaryOutput.fill %= primaryOutput.size;
	}
	local_irq_restore(flags);
}

void sound_dma_usermode_debug(void) {
	if(primaryInput.flags & DBF_ASSOCIATED) {
		printk("DMA for primary Input:\n");
		s3c_dma_debug_channel(primaryInput.channel);
	} else {
		printk("Primary Input not associated to a DMA channel.\n");
	}

	printk("\n");

	if(primaryOutput.flags & DBF_ASSOCIATED) {
		printk("DMA for primary Output:\n");
		s3c_dma_debug_channel(primaryOutput.channel);
	} else {
		printk("Primary Output not associated to a DMA channel.\n");
	}

	printk("\n");
}

#ifdef CONFIG_BARCELONA_SOUND_FILTER
void sound_dma_filter_init() {}
void sound_dma_filter_shutdown() {}
void sound_dma_filter_resync() {}
void sound_dma_filter_magic() {}
#else /* COOLSOUND_FILTER_API */
void sound_dma_filter_init() {}
void sound_dma_filter_shutdown() {}
void sound_dma_filter_resync() {}
void sound_dma_filter_magic() {}
#endif
