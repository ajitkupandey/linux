// SPDX-License-Identifier: BSD-3-Clause
//
// This file is provided under a BSD license.  When using or
// redistributing this file, you may do so under this license.
//
// Copyright(c) 2021 Advanced Micro Devices, Inc. All rights reserved.
//
// Authors: Ajit Kumar Pandey <AjitKumar.Pandey@amd.com>
//

/*
 * Hardware interface for generic AMD audio DSP ACP IP
 */

#include <linux/platform_device.h>
#include <linux/err.h>
#include <sound/pcm_params.h>
#include <linux/module.h>
#include <sound/sof.h>
#include <sound/sof/xtensa.h>
#include "../sof-priv.h"
#include "../sof-audio.h"

#include "../ops.h"
#include "acp.h"

#define PTE_GRP1_OFFSET		0x00000000
#define PTE_GRP2_OFFSET		0x00800000
#define PTE_GRP3_OFFSET		0x01000000
#define PTE_GRP4_OFFSET		0x01800000
#define PTE_GRP5_OFFSET		0x02000000
#define PTE_GRP6_OFFSET		0x02800000
#define PTE_GRP7_OFFSET		0x03000000
#define PTE_GRP8_OFFSET		0x03800000

int config_pte_for_stream(struct snd_sof_dev *sdev,
			  struct acp_dsp_stream *stream, int num_pages)
{
	struct snd_pcm_substream *substream = stream->substream;
	int stream_tag = stream->stream_tag;
	struct snd_dma_buffer *dmab = substream->runtime->dma_buffer_p;
	unsigned int pte_reg = 0, pte_size = 0;
	unsigned int phy_addr_offset = 0;
	unsigned int index = 0;
	u32 low, high, offset = 0, reg_val = 0;
	dma_addr_t addr = 0;
	int page_idx;

	switch (stream_tag) {
	case 1:
		pte_reg = ACPAXI2AXI_ATU_BASE_ADDR_GRP_1;
		pte_size =  ACPAXI2AXI_ATU_PAGE_SIZE_GRP_1;
		offset = offsetof(struct acp_scratch_register_config,
				  acp_atu_grp1_pte);
		stream->reg_offset = PTE_GRP1_OFFSET;
		break;
	case 2:
		pte_reg = ACPAXI2AXI_ATU_BASE_ADDR_GRP_2;
		pte_size =  ACPAXI2AXI_ATU_PAGE_SIZE_GRP_2;
		offset = offsetof(struct acp_scratch_register_config,
				  acp_atu_grp2_pte);
		stream->reg_offset = PTE_GRP2_OFFSET;
		break;
	case 3:
		pte_reg = ACPAXI2AXI_ATU_BASE_ADDR_GRP_3;
		pte_size =  ACPAXI2AXI_ATU_PAGE_SIZE_GRP_3;
		offset = offsetof(struct acp_scratch_register_config,
				  acp_atu_grp3_pte);
		stream->reg_offset = PTE_GRP3_OFFSET;
		break;
	case 4:
		pte_reg = ACPAXI2AXI_ATU_BASE_ADDR_GRP_4;
		pte_size =  ACPAXI2AXI_ATU_PAGE_SIZE_GRP_4;
		offset = offsetof(struct acp_scratch_register_config,
				  acp_atu_grp4_pte);
		stream->reg_offset = PTE_GRP4_OFFSET;
		break;
	case 5:
		pte_reg = ACPAXI2AXI_ATU_BASE_ADDR_GRP_5;
		pte_size =  ACPAXI2AXI_ATU_PAGE_SIZE_GRP_5;
		offset = offsetof(struct acp_scratch_register_config,
				  acp_atu_grp5_pte);
		stream->reg_offset = PTE_GRP5_OFFSET;
		break;
	case 6:
		pte_reg = ACPAXI2AXI_ATU_BASE_ADDR_GRP_6;
		pte_size =  ACPAXI2AXI_ATU_PAGE_SIZE_GRP_6;
		offset = offsetof(struct acp_scratch_register_config,
				  acp_atu_grp6_pte);
		stream->reg_offset = PTE_GRP6_OFFSET;
		break;
	case 7:
		pte_reg = ACPAXI2AXI_ATU_BASE_ADDR_GRP_7;
		pte_size =  ACPAXI2AXI_ATU_PAGE_SIZE_GRP_7;
		offset = offsetof(struct acp_scratch_register_config,
				  acp_atu_grp7_pte);
		stream->reg_offset = PTE_GRP7_OFFSET;
		break;
	case 8:
		pte_reg = ACPAXI2AXI_ATU_BASE_ADDR_GRP_8;
		pte_size =  ACPAXI2AXI_ATU_PAGE_SIZE_GRP_8;
		offset = offsetof(struct acp_scratch_register_config,
				  acp_atu_grp8_pte);
		stream->reg_offset = PTE_GRP8_OFFSET;
		break;
	default:
		pr_err("Invalid stream tag %d\n", stream_tag);
		break;
	}

	/* write phy_addr in scratch memory */

	phy_addr_offset = offsetof(struct acp_scratch_register_config,
				   reg_offset);
	index = stream_tag - 1;
	phy_addr_offset = phy_addr_offset + (index*4);

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_SCRATCH_REG_0 +
			  phy_addr_offset, stream->reg_offset);

	/* Group Enable */
	reg_val = ACP_SRAM_PTE_OFFSET + offset;
	snd_sof_dsp_write(sdev, ACP_DSP_BAR, pte_reg, reg_val | BIT(31));
	snd_sof_dsp_write(sdev, ACP_DSP_BAR, pte_size, PAGE_SIZE_4K_ENABLE);

	for (page_idx = 0; page_idx < num_pages; page_idx++) {

		addr = snd_sgbuf_get_addr(dmab, page_idx * PAGE_SIZE);

		/* Load the low address of page int ACP SRAM through SRBM */
		low = lower_32_bits(addr);
		high = upper_32_bits(addr);

		snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_SCRATCH_REG_0 + offset, low);

		high |= BIT(31);
		snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_SCRATCH_REG_0 + offset + 4, high);
		/* Move to next physically contiguous page */
		offset += 8;
	}

	return 0;
}

struct acp_dsp_stream *
acp_dsp_stream_get(struct snd_sof_dev *sdev)
{
	struct acp_dev_data *adata = sdev->pdata->hw_pdata;
	struct acp_dsp_stream *stream;
	int node = 0;

	/* get an unused stream */
	list_for_each_entry(stream, &adata->stream_list, list) {
		node++;
		if (stream->active == 0)
			break;
	}

	stream->active = 1;
	return stream;
}
EXPORT_SYMBOL_NS(acp_dsp_stream_get, SND_SOC_SOF_AMD_COMMON);

int acp_dsp_stream_put(struct snd_sof_dev *sdev,
		       struct acp_dsp_stream *acp_stream)
{
	struct acp_dev_data *adata = sdev->pdata->hw_pdata;
	struct acp_dsp_stream *stream;
	int ret = 0;

	/* Free an active stream */
	list_for_each_entry(stream, &adata->stream_list, list) {
		if (stream == acp_stream)
			break;
	}

	if (stream->active == 1) {
		stream->active = 0;
	} else {
		dev_err(sdev->dev, "Stream not active\n");
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL_NS(acp_dsp_stream_put, SND_SOC_SOF_AMD_COMMON);

int acp_dsp_stream_init(struct snd_sof_dev *sdev)
{
	struct acp_dev_data *adata = sdev->pdata->hw_pdata;
	struct acp_dsp_stream *acp_stream;
	int num_dai = sof_ops(sdev)->num_drv;
	int num_stream = num_dai*2;
	int i;

	for (i = 0; i < num_stream; i++) {
		acp_stream = devm_kzalloc(sdev->dev, sizeof(*acp_stream),
					  GFP_KERNEL);
		if (!acp_stream)
			return -ENOMEM;

		acp_stream->sdev = sdev;
		acp_stream->active = 0;
		acp_stream->stream_tag = i+1;

		list_add_tail(&acp_stream->list, &adata->stream_list);
	}
	return 0;
}
EXPORT_SYMBOL_NS(acp_dsp_stream_init, SND_SOC_SOF_AMD_COMMON);

