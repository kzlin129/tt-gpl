#ifndef SOUND_DMA_H
#define SOUND_DMA_H

#include <asm/dma.h>

/* Define the DMA unit size for IIS to use. This is different for 24xx & 2412... */
#ifndef CONFIG_CPU_S3C2412
  #define DMA_UNIT_SIZE 2
#else
  #include <barcelona/gopins.h>
  #define DMA_UNIT_SIZE (((IO_GetCpuType() == GOCPU_S3C2412) || \
                          (IO_GetCpuType() == GOCPU_S3C2443) || \
                          (IO_GetCpuType() == GOCPU_S3C2450)) ? 4 : 2)
#endif

enum eDMAPurpose {
	DMAP_PLAY,
	DMAP_RECORD,
};

typedef void (*sound_dma_callback_t)(enum eDMAPurpose purpose, int chunk);

enum eDMABufferFlags {
	DBF_NONE        = 0,
	DBF_ALLOCATED   = (1<<0),
	DBF_ASSOCIATED  = (1<<1),
	DBF_RUNNING     = (1<<2),
};

struct sDMABuffer {
	dmach_t					channel;
	s3c_dma_client_t	client;
	dma_addr_t				phys;
	void *					virt;
	size_t					size;
	size_t					real_size;
	size_t					chunksize;
	int						chunks;
	int						fill;
	int						pointer;
	int						fpointer;

	int						underrun;
	int						overflow;

	enum eDMABufferFlags	flags;

	struct device			*pDev;

	sound_dma_callback_t	callback;
};

int 	sound_dma_init(struct device *pDev, sound_dma_callback_t callback);
int 	sound_dma_suspend(void);
int 	sound_dma_resume(void);
int 	sound_dma_setup_buffers(int chunksize, int chunks);
void	sound_dma_usermode_debug(void);

int 	sound_dma_output_start(void);
void 	sound_dma_output_stop(void);
int 	sound_dma_input_allocated(void);
int 	sound_dma_input_ready(void);
int 	sound_dma_input_running(void);
size_t	sound_dma_input_size(void);
void *	sound_dma_input_phys(void);
int 	sound_dma_input_pos(void);
int 	sound_dma_input_filled(void);
void 	sound_dma_input_reset(void);
void 	sound_dma_input_get(void *pData, int size, int offset, cs_primixer_t primixer, unsigned int flags);
void 	sound_dma_input_advance(int size);

int 	sound_dma_input_start(void);
void 	sound_dma_input_stop(void);
int 	sound_dma_output_allocated(void);
int 	sound_dma_output_ready(void);
int 	sound_dma_output_running(void);
size_t	sound_dma_output_size(void);
void *	sound_dma_output_phys(void);
int 	sound_dma_output_pos(void);
int 	sound_dma_output_free(void);
void 	sound_dma_output_reset(void);
void 	sound_dma_output_put(void *pData, int size, int offset, cs_primixer_t primixer, unsigned int flags);
void 	sound_dma_output_advance(int size);
void 	sound_dma_output_zeroes(int samples, int offset);

/* Filter API */
void 	sound_dma_filter_init(void);
void 	sound_dma_filter_shutdown(void);
void 	sound_dma_filter_resync(void);
void 	sound_dma_filter_magic(void);

#endif /* SOUND_DMA_H */
