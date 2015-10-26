#
## Makefile for ALSA BRCM OMX
#
#
EXTRA_CFLAGS := -Wno-error
EXTRA_CFLAGS += -I$(LINUX_DIR)/include/linux/broadcom/omx
EXTRA_CFLAGS += -I$(LINUX_DIR)/include/linux/broadcom/vc03

## doesn't look good;need to fix it
##EXTRA_CFLAGS += -I$(LINUX_VOB)/target/kernel/added/drivers/char/broadcom/vc03

obj-$(CONFIG_SND_ARM_BRCM_OMX) += snd-brcm-omx.o
snd-brcm-omx-objs := brcm_alsa_dev.o brcm_alsa_pcm.o brcm_alsa_omx.o
snd-brcm-omx-objs += brcm_alsa_ctl.o
