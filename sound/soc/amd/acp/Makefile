# SPDX-License-Identifier: BSD-3-Clause
# This file is provided under a BSD license. When using or
# redistributing this file, you may do so under this license.
#
# Copyright(c) 2021 Advanced Micro Devices, Inc. All rights reserved.

#common acp driver
snd-acp-pcm-objs     := acp-platform.o
snd-acp-i2s-objs     := acp-i2s.o

#platform specific driver
snd-acp-renoir-objs     := acp-renoir.o

#machine specific driver
snd-acp-config-objs     := acp-config.o

obj-$(CONFIG_SND_SOC_AMD_ACP_PCM) += snd-acp-pcm.o
obj-$(CONFIG_SND_SOC_AMD_ACP_I2S) += snd-acp-i2s.o

obj-$(CONFIG_SND_AMD_ASOC_RENOIR) += snd-acp-renoir.o

obj-$(CONFIG_SND_AMD_ACP_CONFIG) += snd-acp-config.o

