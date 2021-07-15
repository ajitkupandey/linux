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
 * Hardware interface for ACP DSP Firmware binaries loader
 */

#include <linux/pci.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <sound/sof.h>
#include <sound/sof/xtensa.h>
#include "../ops.h"
#include "acp.h"

int acp_dsp_block_read(struct snd_sof_dev *sdev,
		       enum snd_sof_fw_blk_type blk_type, u32 offset,
		       void *dest, size_t size)
{
	struct acp_dev_data *adata = sdev->pdata->hw_pdata;
	unsigned char *ring_buffer =
			adata->acp_sregmap->scratch_buf_info.sofOutBoxBuffer;

	switch (blk_type) {
	case SOF_FW_BLK_TYPE_SRAM:
		offset = offset - ACP_SCRATCH_MEMORY_ADDRESS;
		acp_copy_from_scratch_memory(sdev, ring_buffer, offset,
					     dest, size);
		break;
	default:
		dev_err(sdev->dev, "error: bad blk type 0x%x\n", blk_type);
	}

	return 0;
}
EXPORT_SYMBOL_NS(acp_dsp_block_read, SND_SOC_SOF_AMD_COMMON);

int acp_dsp_block_write(struct snd_sof_dev *sdev,
			enum snd_sof_fw_blk_type blk_type, u32 offset,
			void *src, size_t size)
{
	struct acp_dev_data *adata = NULL;
	void *dest;
	struct pci_dev *pci = to_pci_dev(sdev->dev);
	u32 dma_size = 0;
	u32 page_count = 0;
	unsigned int size_fw = 0;
	struct snd_sof_pdata *plat_data = sdev->pdata;
	unsigned char *ring_buffer;

	adata =  sdev->pdata->hw_pdata;

	switch (blk_type) {
	case SOF_FW_BLK_TYPE_IRAM:
		if (adata->bin_buf == NULL) {
			size_fw = plat_data->fw->size;
			page_count = PAGE_ALIGN(size_fw) >> PAGE_SHIFT;
			dma_size = page_count * ACP_PAGE_SIZE;
			adata->bin_buf = pci_alloc_consistent(pci, dma_size,
							      &adata->sha_dma_addr);
			memset((void *)adata->bin_buf, 0, dma_size);
		}
	adata->fw_bin_size = size + offset;
	dest = adata->bin_buf + offset;
	break;
	case SOF_FW_BLK_TYPE_DRAM:
		if (adata->data_buf == NULL) {
			adata->data_buf = pci_alloc_consistent(pci,
						ACP_DEFAULT_DRAM_LENGTH,
						&adata->dma_addr);
			memset((void *)adata->data_buf, 0,
				ACP_DEFAULT_DRAM_LENGTH);
		}
		dest = adata->data_buf + offset;
		adata->fw_data_bin_size = size + offset;

		break;
	case SOF_FW_BLK_TYPE_SRAM:
		ring_buffer =
			adata->acp_sregmap->scratch_buf_info.sofInBoxBuffer;
		offset = offset - ACP_SCRATCH_MEMORY_ADDRESS;
		acp_copy_to_scratch_memory(sdev, ring_buffer, offset, src,
					   size);
		return 0;
	default:
		dev_err(sdev->dev, "error: bad blk type 0x%x\n", blk_type);
		return -1;
	}

	memcpy(dest, src, size);
	return 0;
}
EXPORT_SYMBOL_NS(acp_dsp_block_write, SND_SOC_SOF_AMD_COMMON);

int acp_get_bar_index(struct snd_sof_dev *sdev, u32 type)
{
	return type;
}
EXPORT_SYMBOL_NS(acp_get_bar_index, SND_SOC_SOF_AMD_COMMON);

void configure_pte_for_fw_loading(int type, int num_pages,
				 struct acp_dev_data *adata)
{
	struct snd_sof_dev *sdev;
	u16 page_idx;
	u32 offset;
	dma_addr_t addr;
	__le32 low;
	__le32 high;

	sdev = adata->dev;

	switch (type) {
	case FW_BIN:
		offset = FW_BIN_PTE_OFFSET;
		addr = adata->sha_dma_addr;
		break;
	case FW_DATA_BIN:
		offset = adata->fw_bin_page_count * 8;
		addr = adata->dma_addr;
		break;
	}

	for (page_idx = 0; page_idx < num_pages; page_idx++) {
		low = cpu_to_le32(lower_32_bits(addr));
		high = cpu_to_le32(upper_32_bits(addr));
		snd_sof_dsp_write(sdev, ACP_DSP_BAR,
				ACP_SCRATCH_REG_0 + offset, low);
		high |= BIT(31);
		snd_sof_dsp_write(sdev, ACP_DSP_BAR,
			ACP_SCRATCH_REG_0 + offset + 4, high);
		offset += 8;
		addr += PAGE_SIZE;
	}
}

/* pre fw run operations */
int acp_dsp_pre_fw_run(struct snd_sof_dev *sdev)
{
	struct acp_dev_data *adata = NULL;
	struct pci_dev *pci = to_pci_dev(sdev->dev);
	u32 fw_bin_buf_size = 0;
	u32 page_count = 0;
	int ret = 0;
	unsigned char ch_num = 0x00;
	unsigned int src_addr = 0;
	unsigned int size_fw ;//=  plat_data->fw->size;

	adata =  sdev->pdata->hw_pdata;
	size_fw = adata->fw_bin_size;

	page_count = PAGE_ALIGN(size_fw) >> PAGE_SHIFT;
	adata->fw_bin_page_count = page_count;
	fw_bin_buf_size = page_count * ACP_PAGE_SIZE;

	configure_pte_for_fw_loading(FW_BIN, page_count, adata);
	ret = acpbus_configure_and_run_sha_dma(adata, adata->bin_buf,
					       ACP_SYSTEM_MEMORY_WINDOW,
					       ACP_IRAM_BASE_ADDRESS,
					       size_fw);
	if (ret < 0) {
		pr_err("SHA DMA transfer failed status: 0x%x\n", ret);
		return ret;
	}
	configure_pte_for_fw_loading(FW_DATA_BIN, ACP_DRAM_PAGE_COUNT,
					adata);

	src_addr =  ACP_SYSTEM_MEMORY_WINDOW +
		    (page_count * ACP_PAGE_SIZE);
	ret = acpbus_configure_and_run_dma(adata,
					   src_addr,
					   ACP_DATA_RAM_BASE_ADDRESS,
					   adata->fw_data_bin_size,
					   &ch_num);
	if (ret < 0) {
		pr_err("acp dma channel reset failed:%d\n", ret);
		return ret;
	}

	ret = acp_dma_status(adata, ch_num);
	if (ret < 0) {
		pr_err("acp dma transfer status :%d\n", ret);
		return ret;
	}

	/* Free memory once DMA is complete */
	pci_free_consistent(pci, size_fw, &adata->bin_buf,
			    adata->sha_dma_addr);
	pci_free_consistent(pci, ACP_DEFAULT_DRAM_LENGTH, &adata->data_buf,
			    adata->dma_addr);
	return 0;
}
EXPORT_SYMBOL_NS(acp_dsp_pre_fw_run, SND_SOC_SOF_AMD_COMMON);

int acp_sof_dsp_run(struct snd_sof_dev *sdev)
{
	int val = 0;

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_DSP0_RUNSTALL, val);
	val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_DSP0_RUNSTALL);
	dev_info(sdev->dev, "ACP_DSP0_RUNSTALL : 0x%0x\n", val);

	return 0;
}
EXPORT_SYMBOL_NS(acp_sof_dsp_run, SND_SOC_SOF_AMD_COMMON);
