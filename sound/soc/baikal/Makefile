# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_SND_BAIKAL_I2S) += baikal_i2s.o
obj-$(CONFIG_SND_BAIKAL_I2S) += baikal_snd.o

baikal_i2s-y := baikal-i2s.o
baikal_snd-y := baikal-snd.o

ifneq ($(CONFIG_ACPI),)
baikal_snd-objs += baikal-snd-simple-acpi.o
endif

baikal_i2s-$(CONFIG_SND_BAIKAL_PIO_PCM) += baikal-pio-pcm.o
