// SPDX-License-Identifier: BSD-3-Clause
//
// This file is provided under a dual BSD license. When using or
// redistributing this file, you may do so under this license.
//
// Copyright(c) 2021 Advanced Micro Devices, Inc. All rights reserved.
//
// Authors: Ajit Kumar Pandey <AjitKumar.Pandey@amd.com>
//

/*
 * Generic interface for ACP audio blck PCM component
 */

#define DRV_NAME "acp3x_rv_i2s_dma"

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>

#include "amd.h"

static const struct snd_pcm_hardware acp_pcm_hardware_playback = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_BATCH |
		SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |  SNDRV_PCM_FMTBIT_S8 |
		   SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S24_LE |
		   SNDRV_PCM_FMTBIT_S32_LE,
	.channels_min = 2,
	.channels_max = 8,
	.rates = SNDRV_PCM_RATE_8000_96000,
	.rate_min = 8000,
	.rate_max = 96000,
	.buffer_bytes_max = PLAYBACK_MAX_NUM_PERIODS * PLAYBACK_MAX_PERIOD_SIZE,
	.period_bytes_min = PLAYBACK_MIN_PERIOD_SIZE,
	.period_bytes_max = PLAYBACK_MAX_PERIOD_SIZE,
	.periods_min = PLAYBACK_MIN_NUM_PERIODS,
	.periods_max = PLAYBACK_MAX_NUM_PERIODS,
};

static const struct snd_pcm_hardware acp_pcm_hardware_capture = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_BATCH |
		SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S8 |
		   SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S24_LE |
		   SNDRV_PCM_FMTBIT_S32_LE,
	.channels_min = 2,
	.channels_max = 2,
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.buffer_bytes_max = CAPTURE_MAX_NUM_PERIODS * CAPTURE_MAX_PERIOD_SIZE,
	.period_bytes_min = CAPTURE_MIN_PERIOD_SIZE,
	.period_bytes_max = CAPTURE_MAX_PERIOD_SIZE,
	.periods_min = CAPTURE_MIN_NUM_PERIODS,
	.periods_max = CAPTURE_MAX_NUM_PERIODS,
};

static irqreturn_t i2s_irq_handler(int irq, void *data)
{
	struct acp_dev_data *adata = data;
	struct acp_stream *stream;
	u16 i2s_flag;
	u32 val, i = 0;

	if (!adata)
		return IRQ_NONE;

	i2s_flag = 0;

	val = acp_readl(adata->acp3x_base + ACP_EXTERNAL_INTR_STAT);

	for (i = 0; i < ACP_MAX_STREAM; i++) {
		stream = adata->stream[i];
		if (stream != NULL && (val & stream->irq_bit)) {
			acp_writel(stream->irq_bit, adata->acp3x_base +
				   ACP_EXTERNAL_INTR_STAT);
			snd_pcm_period_elapsed(stream->substream);
			i2s_flag = 1;
			break;
		}
	}

	if (i2s_flag)
		return IRQ_HANDLED;
	else
		return IRQ_NONE;
}

static void config_pte_for_stream(struct acp_dev_data *adata, struct acp_stream *stream)
{
	u32 pte_reg, pte_size, reg_val;
	unsigned int offset = 0;

	pte_reg = ACPAXI2AXI_ATU_BASE_ADDR_GRP_5;
	pte_size =  ACPAXI2AXI_ATU_PAGE_SIZE_GRP_5;
	stream->reg_offset = 0x02000000;

	/* Group Enable */
	reg_val = ACP_SRAM_PTE_OFFSET + offset;
	acp_writel(reg_val | BIT(31), adata->acp3x_base + pte_reg);
	acp_writel(PAGE_SIZE_4K_ENABLE,  adata->acp3x_base + pte_size);

}
static void config_acp_dma(struct acp_dev_data *adata, int cpu_id, int size)
{
	u16 page_idx;
	u32 low = 0, high = 0, val = 0;
	dma_addr_t addr;
	struct acp_stream *stream;
	struct snd_pcm_substream *substream;
	int direction, num_pages;
	u32 low2, high2, offset2;

	stream = adata->stream[cpu_id];
	substream = stream->substream;
	direction = substream->stream;

	addr = substream->dma_buffer.addr;
	num_pages = (PAGE_ALIGN(size) >> PAGE_SHIFT);

	val = stream->pte_offset;

	for (page_idx = 0; page_idx < num_pages; page_idx++) {
		/* Load the low address of page int ACP SRAM through SRBM */
		low = lower_32_bits(addr);
		high = upper_32_bits(addr);
		acp_writel(low, adata->acp3x_base + ACP_SCRATCH_REG_0 + val);
		high |= BIT(31);
		acp_writel(high, adata->acp3x_base + ACP_SCRATCH_REG_0 + val
				+ 4);

		low2 = acp_readl(adata->acp3x_base + ACP_SCRATCH_REG_0 + val);
		high2 = acp_readl(adata->acp3x_base + ACP_SCRATCH_REG_0 + val + 4);
		offset2 =  ACP_SCRATCH_REG_0 + val;
		/* Move to next physically contiguous page */
		val += 8;
		addr += PAGE_SIZE;
	}
}

static int acp_dma_open(struct snd_soc_component *component,
			  struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_runtime = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(soc_runtime, 0);
	struct device *dev = component->dev;
	struct acp_dev_data *adata = dev_get_drvdata(dev);
	int ret = 0;
	int stream_id = (cpu_dai->driver->id)*2 + substream->stream;
	struct acp_stream *stream;

	stream = devm_kzalloc(dev, sizeof(*stream), GFP_KERNEL);

	stream->substream = substream;
	adata->stream[stream_id] = stream;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		runtime->hw = acp_pcm_hardware_playback;
	else
		runtime->hw = acp_pcm_hardware_capture;

	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		dev_err(component->dev, "set integer constraint failed\n");
		return ret;
	}
	runtime->private_data = stream;

	acp_writel(1, adata->acp3x_base + ACP_EXTERNAL_INTR_ENB);
	return ret;
}


static int acp_dma_hw_params(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	struct acp_dev_data *adata;
	struct snd_soc_pcm_runtime *soc_runtime = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(soc_runtime, 0);
	struct acp_stream *stream = substream->runtime->private_data;
	u64 size;
	u32 period_bytes;
	u32 physical_addr;
	int stream_id;

	adata = snd_soc_component_get_drvdata(component);
	stream_id = (cpu_dai->driver->id)*2 + substream->stream;
	size = params_buffer_bytes(params);
	period_bytes = params_period_bytes(params);
	config_pte_for_stream(adata, stream);
	config_acp_dma(adata, stream_id, size);

	return 0;
}

static snd_pcm_uframes_t acp_dma_pointer(struct snd_soc_component *component,
					   struct snd_pcm_substream *substream)
{
	struct device *dev = component->dev;
	struct acp_dev_data *adata;
	struct acp_stream *stream;
	u32 pos, buffersize;
	u64 bytescount;

	adata = dev_get_drvdata(dev);
	stream = substream->runtime->private_data;

	buffersize = frames_to_bytes(substream->runtime,
				     substream->runtime->buffer_size);

	bytescount = acp_get_byte_count(adata, stream->dai_id, substream->stream);

	if (bytescount > stream->bytescount)
		bytescount -= stream->bytescount;

	pos = do_div(bytescount, buffersize);

	return bytes_to_frames(substream->runtime, pos);
}

static int acp_dma_new(struct snd_soc_component *component,
			 struct snd_soc_pcm_runtime *rtd)
{
	struct device *parent = component->dev->parent;

	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
				       parent, MIN_BUFFER, MAX_BUFFER);
	return 0;
}

static int acp_dma_mmap(struct snd_soc_component *component,
			  struct snd_pcm_substream *substream,
			  struct vm_area_struct *vma)
{
	return snd_pcm_lib_default_mmap(substream, vma);
}

static int acp_dma_close(struct snd_soc_component *component,
			   struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_runtime = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(soc_runtime, 0);
	struct device *dev = component->dev;
	struct acp_dev_data *adata = dev_get_drvdata(dev);
	int stream_id = (cpu_dai->driver->id)*2 + substream->stream;

	adata->stream[stream_id] = 0;

	return 0;
}

static const struct snd_soc_component_driver acp_pcm_component = {
	.name		= DRV_NAME,
	.open		= acp_dma_open,
	.close		= acp_dma_close,
	.hw_params	= acp_dma_hw_params,
	.pointer	= acp_dma_pointer,
	.mmap		= acp_dma_mmap,
	.pcm_construct	= acp_dma_new,
};

int acp_platform_register(struct device *dev)
{
	struct acp_dev_data *adata = dev_get_drvdata(dev);
	struct snd_soc_dai_driver;
	unsigned int status = 0;

	status = devm_request_irq(dev, adata->i2s_irq, i2s_irq_handler,
				  IRQF_SHARED, "ACP3x_I2S_IRQ", adata);
	if (status) {
		dev_err(dev, "ACP3x I2S IRQ request failed\n");
		return -ENODEV;
	}

	status = devm_snd_soc_register_component(dev, &acp_pcm_component,
						 adata->dai_driver,
						 adata->num_dai);
	if (status) {
		dev_err(dev, "Fail to register acp i2s component\n");
		return -ENODEV;
	}
	return 0;
}
EXPORT_SYMBOL(acp_platform_register);

int acp_platform_unregister(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	return 0;
}

MODULE_DESCRIPTION("AMD ACP 3.x PCM Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRV_NAME);
