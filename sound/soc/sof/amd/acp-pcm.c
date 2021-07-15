// SPDX-License-Identifier: BSD-3-Clause
//
// This file is provided under a BSD license.  When using or
// redistributing this file, you may do so under this license.
//
// Copyright(c) 2021 Advanced Micro Devices, Inc. All rights reserved.
//
// Author: Ajit Kumar Pandey <AjitKumar.Pandey@amd.com >

/*
 * PCM interface for generic AMD audio ACP DSP block
 */
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#include "acp.h"

int acp_pcm_hw_params(struct snd_sof_dev *sdev,
			  struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params,
			  struct sof_ipc_stream_params *ipc_params)
{
	uint32_t size;
	struct acp_dsp_stream *stream = substream->runtime->private_data;
	u16 num_pages;
	dma_addr_t dma_addr;
	unsigned int buf_offset, index;

	struct snd_dma_buffer *dmab;

	dmab = substream->runtime->dma_buffer_p;

	size = ipc_params->buffer.size;
	dma_addr = ipc_params->buffer.phy_addr;
	num_pages = ipc_params->buffer.pages;

	config_pte_for_stream(sdev, stream, num_pages);

	ipc_params->buffer.phy_addr = stream->reg_offset;
	ipc_params->stream_tag = stream->stream_tag;

	/* write buffer size of stream in scratch memory */

	buf_offset = offsetof(struct acp_scratch_register_config, buf_size);
	index = stream->stream_tag - 1;
	buf_offset = buf_offset + (index*4);

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_SCRATCH_REG_0 + buf_offset,
			  size);

	return 0;
}
EXPORT_SYMBOL_NS(acp_pcm_hw_params, SND_SOC_SOF_AMD_COMMON);

int acp_pcm_open(struct snd_sof_dev *sdev,
		   struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	struct acp_dsp_stream *stream;

	stream = acp_dsp_stream_get(sdev);
	if (!stream)
		return -ENOMEM;

	substream->runtime->private_data = stream;
	stream->substream = substream;

	runtime = substream->runtime;

	return 0;
}
EXPORT_SYMBOL_NS(acp_pcm_open, SND_SOC_SOF_AMD_COMMON);

int acp_pcm_close(struct snd_sof_dev *sdev,
		    struct snd_pcm_substream *substream)
{
	int ret;

	struct acp_dsp_stream *stream = substream->runtime->private_data;

	ret = acp_dsp_stream_put(sdev, stream);
	substream->runtime->private_data = NULL;

	return 0;
}
EXPORT_SYMBOL_NS(acp_pcm_close, SND_SOC_SOF_AMD_COMMON);

