#===================================================
# Makefile for ALSA SOC BRCM BCM476X
#===================================================
#
# Platform Support
#
snd-bcm476x-soc-objs := bcm476x_snd_pcm.o
snd-bcm476x-pcm-ext-objs := bcm476x_snd_pcm_ext.o
snd-bcm476x-i2s-objs := bcm476x_i2s.o
snd-bcm476x-ac97-objs := 
#
obj-$(CONFIG_SND_BCM476X_SOC) += snd-bcm476x-soc.o
#obj-$(CONFIG_SND_BCM476X_SOC_EXTERNAL) += snd-bcm476x-pcm-ext.o
obj-$(CONFIG_SND_BCM476X_SOC_I2S)  += snd-bcm476x-i2s.o 

#
# Machine Support
#
bcm476x-internal-snd-objs := bcm476x_snd_internal.o 
#bcm476x-wm8750-objs   := bcm476x_wm8750.o 
#bcm476x-rt5627-objs   := bcm476x_rt5627.o 
#
obj-$(CONFIG_SND_BCM476X_SOC_INTERNAL) += bcm476x-internal-snd.o
#obj-$(CONFIG_SND_BCM476X_SOC_EXT_WM8750) += bcm476x-wm8750.o
#obj-$(CONFIG_SND_BCM476X_SOC_EXT_RT5627) += bcm476x-rt5627.o

