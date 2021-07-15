// SPDX-License-Identifier: BSD-3-Clause
//
// This file is provided under a BSD license. When using or
// redistributing this file, you may do so under this license.
//
// Copyright(c) 2021 Advanced Micro Devices, Inc. All rights reserved.
//
// Authors: Vijendar Mukunda <Vijendar.Mukunda@amd.com>
//	    Ajit Kumar Pandey <AjitKumar.Pandey@amd.com>

/*
 * Hardware interface for generic AMD ACP processor
 */

#include <linux/pci.h>
#include <linux/module.h>
#include <linux/io.h>
#include <sound/sof.h>
#include "acp.h"
#include "../ops.h"

static void acp_enable_extirq(struct acp_dev_data *adata)
{
	int val = 0;
	struct snd_sof_dev *sdev = adata->dev;

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_EXTERNAL_INTR_ENB, 0x01);
	val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_EXTERNAL_INTR_CNTL);
	val |= ACP_ERROR_MASK;
	snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_EXTERNAL_INTR_CNTL, val);
	val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_DSP_SW_INTR_CNTL);
	val |= 0x01;
	snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_DSP_SW_INTR_CNTL, val);
	val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_DSP_SW_INTR_CNTL);

}

/* Group1 & Group2 with 4kb as page size mapped to ACP Scratch memory */
static void configure_acp_groupregisters(struct acp_dev_data *adata)
{
	u32 index, val;
	struct snd_sof_dev *sdev = adata->dev;

	/* Group Enable */
	snd_sof_dsp_write(sdev, ACP_DSP_BAR,
		ACPAXI2AXI_ATU_BASE_ADDR_GRP_1,
				ACP_SRAM_PTE_OFFSET | BIT(31));
	snd_sof_dsp_write(sdev, ACP_DSP_BAR,
		ACPAXI2AXI_ATU_PAGE_SIZE_GRP_1, PAGE_SIZE_4K_ENABLE);

	for (index = 0; index < 2; index++) {
		val = (ACP_SRBM_DATA_RAM_BASE_OFFSET + (index * 0x4000))
		       | BIT(31);
		snd_sof_dsp_write(sdev, ACP_DSP_BAR,
			  ACPAXI2AXI_ATU_BASE_ADDR_GRP_2 + (index * 0x08),
			  val);
		snd_sof_dsp_write(sdev, ACP_DSP_BAR,
				  ACPAXI2AXI_ATU_PAGE_SIZE_GRP_2 + (index * 0x08),
				  PAGE_SIZE_4K_ENABLE);
	}
	snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACPAXI2AXI_ATU_CTRL, 0x01);
}

static void configure_dma_descriptiors(struct acp_dev_data *adata)
{
	unsigned int dma_desc_base_addr;
	struct snd_sof_dev *sdev = adata->dev;

	dma_desc_base_addr = ACP_SRAM_PTE_OFFSET +
			     offsetof(struct acp_scratch_register_config,
				      acp_config_dma_desc);

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_DMA_DESC_BASE_ADDR,
			  dma_desc_base_addr);
	snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_DMA_DESC_MAX_NUM_DSCR, 0x02);
}

static void acpbus_configure_dma_descriptor(struct acp_dev_data *adata,
					    unsigned short dscr_idx,
					    struct acp_config_dma_descriptor
					    *dscr_info)
{
	unsigned int scratch_mem_offset =
				(ACP_SCRATCH_REG_0 +
				 offsetof(struct acp_scratch_register_config,
					  acp_config_dma_desc)) +
				(dscr_idx *
				 sizeof(struct acp_config_dma_descriptor));

	struct snd_sof_dev *sdev = adata->dev;

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, scratch_mem_offset,
			  dscr_info->src_addr);

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, (scratch_mem_offset + 0x4),
			  dscr_info->dest_addr);

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, (scratch_mem_offset + 0x8),
			  dscr_info->dma_transfer_count.u32_all);
}

static int acpbus_config_dma_channel(struct acp_dev_data *adata,
				     unsigned char ch_num,
				     unsigned short dscr_start_idx,
				     unsigned short dscr_count,
				     unsigned int priority_level)
{
	struct snd_sof_dev *sdev = adata->dev;
	int status = 0;
	u32 val, acp_error_stat, dma_error_stat, acp_dma_dscr_cnt0;
	u32 acp_dma_dscr_start_idx0, acp_dma_prio0, acp_dma_cntl0;
	u32 timeout  = 0x00;

	val = snd_sof_dsp_read(sdev, ACP_DSP_BAR,
			      (ACP_DMA_CNTL_0 + (ch_num * sizeof(u32))));
	dev_info(sdev->dev, "mACP_DMA_CNTL_0 : 0x%x\n", val);
	val |= 0x11;

	snd_sof_dsp_write(sdev, ACP_DSP_BAR,
			  (ACP_DMA_CNTL_0 + (ch_num * sizeof(u32))), 0x11);
	val = snd_sof_dsp_read(sdev, ACP_DSP_BAR,
			       ACP_DMA_CNTL_0 + (ch_num * sizeof(u32)));

	dev_info(sdev->dev, "mACP_DMA_CNTL_0 : 0x%x\n", val);

	while (1) {
		val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_DMA_CH_RST_STS);
		dev_info(sdev->dev, "ACP_DMA_CH_RST_STS : 0x%x\n", val);
		val &= 0xFF;
		if (!(val & (1 << ch_num))) {
			udelay(5);
			if (timeout > 0xff) {
				dev_err(sdev->dev, "DMA_CHANNEL RESET ERROR TIMEOUT\n");
				acp_error_stat = snd_sof_dsp_read(sdev,
							ACP_DSP_BAR,
							ACP_ERROR_STATUS);
				dev_err(sdev->dev, "ACP_ERROR_STATUS :0x%x\n",
				       acp_error_stat);
				dma_error_stat = snd_sof_dsp_read(sdev,
						ACP_DSP_BAR,
						(ACP_DMA_ERR_STS_0 +
						(ch_num * sizeof(u32))));
				dev_err(sdev->dev, "ACP_DMA_ERR_STS :0x%x\n",
				       dma_error_stat);
				status = -ETIMEDOUT;
				break;
			}
			timeout++;
		} else {
			dev_info(sdev->dev, "ACP_DMA_CH_RST_%d is done\n",
				 ch_num);
			break;
		}
	}

	if (status < 0)
		return status;
	acp_dma_cntl0 = 0x00;
	snd_sof_dsp_write(sdev, ACP_DSP_BAR, (ACP_DMA_CNTL_0 +
			  ch_num * sizeof(u32)), acp_dma_cntl0);

	acp_dma_cntl0 =  snd_sof_dsp_read(sdev, ACP_DSP_BAR,
					  (ACP_DMA_CNTL_0 +
					  (ch_num * sizeof(u32))));

	dev_info(sdev->dev, "ACP_DMA_CNTL_0_%d : 0x%x\n", ch_num,
		 acp_dma_cntl0);

	acp_dma_dscr_cnt0 = dscr_count;

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, (ACP_DMA_DSCR_CNT_0 +
			  (ch_num * sizeof(u32))), acp_dma_dscr_cnt0);

	acp_dma_dscr_cnt0 =  snd_sof_dsp_read(sdev, ACP_DSP_BAR,
					      (ACP_DMA_DSCR_CNT_0 +
					      (ch_num * sizeof(u32))));

	dev_info(sdev->dev, "ACP_DMA_DSCR_CNT_0 : 0x%x\n",
		 acp_dma_dscr_cnt0);

	acp_dma_dscr_start_idx0 = dscr_start_idx;

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, (ACP_DMA_DSCR_STRT_IDX_0 +
			  (ch_num * sizeof(u32))), acp_dma_dscr_start_idx0);


	acp_dma_dscr_start_idx0 =  snd_sof_dsp_read(sdev, ACP_DSP_BAR,
						(ACP_DMA_DSCR_STRT_IDX_0 +
						(ch_num * sizeof(u32))));
	dev_info(sdev->dev, "ACP_DMA_DSCR_STRT_IDX_0 : 0x%x\n",
		 acp_dma_dscr_start_idx0);
	acp_dma_prio0 = priority_level;

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, (ACP_DMA_PRIO_0 +
			  (ch_num * sizeof(u32))), acp_dma_prio0);
	acp_dma_prio0 =  snd_sof_dsp_read(sdev, ACP_DSP_BAR,
				(ACP_DMA_PRIO_0 + (ch_num * sizeof(u32))));

	dev_info(sdev->dev, "ACP_DMA_PRIO_0 : 0x%x\n", acp_dma_prio0);

	acp_dma_cntl0 |= ACP_DMA_CH_RUN | ACP_DMA_CH_IOC_ENABLE;

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, (ACP_DMA_CNTL_0 +
			  (ch_num * sizeof(u32))), acp_dma_cntl0);
	acp_dma_cntl0 =  snd_sof_dsp_read(sdev, ACP_DSP_BAR,
				 (ACP_DMA_CNTL_0 + (ch_num * sizeof(u32))));
	dev_info(sdev->dev, "ACP_DMA_CNTL_0_%d : 0x%x\n",
		 ch_num, acp_dma_cntl0);
	return  status;
}

static int acpbus_dma_start(struct acp_dev_data *adata,
			    unsigned char *p_ch_num,
			    unsigned short *p_dscr_count,
			    struct acp_config_dma_descriptor *p_dscr_info,
			    unsigned int priority_level)
{
	struct snd_sof_dev *sdev = adata->dev;
	int status = -1;
	u16 dscr_start_idx = 0x00;
	u16 dscr;
	u16 dscr_count;
	u16 dscr_num;

	if ((p_ch_num) && (p_dscr_count) && (p_dscr_info)) {
		status = 0x00;
		dscr_count = *p_dscr_count;
		/* currently channel number hard coded to zero,
		 * As single channel is used for ACP DMA transfers
		 * from host driver.
		 * Below are the instances ACP DMA transfer
		 * initiated from ACP Loader driver.
		 */
		*p_ch_num = 0x00;
		for (dscr = 0; dscr < *p_dscr_count; dscr++) {
			dscr_num = dscr_start_idx + dscr;
			/* configure particular descriptor registers */
			acpbus_configure_dma_descriptor(adata,
							dscr_num,
							p_dscr_info++);
		}
		/* configure the dma channel and start the dma transfer */
		status = acpbus_config_dma_channel(adata,
						   *p_ch_num,
						   dscr_start_idx,
						   *p_dscr_count,
						    priority_level);
		if (status < 0)
			dev_err(sdev->dev,
				"acpbus config dma channel failed:%d\n", status);
	}
	return status;
}

int acpbus_configure_and_run_dma(struct acp_dev_data *adata,
					unsigned int src_addr,
					unsigned int  dest_addr,
					unsigned int dsp_data_size,
					unsigned char *p_ch_num)
{
	struct snd_sof_dev *sdev = adata->dev;
	int status = 0x00;
	u32 priority_level = 0x00;
	unsigned int desc_count = 0;
	unsigned char ch_no = 0xFF;
	unsigned int index;


	while (dsp_data_size >= ACP_PAGE_SIZE) {
		adata->dscr_info[desc_count].src_addr = src_addr +
							(desc_count *
							 ACP_PAGE_SIZE);
		adata->dscr_info[desc_count].dest_addr = dest_addr +
							 (desc_count *
							  ACP_PAGE_SIZE);
		adata->dscr_info[desc_count].dma_transfer_count.bits.count =
								ACP_PAGE_SIZE;
		desc_count++;
		dsp_data_size -= ACP_PAGE_SIZE;
	}
	if (dsp_data_size > 0) {
		adata->dscr_info[desc_count].src_addr = src_addr +
							(desc_count *
							 ACP_PAGE_SIZE);
		adata->dscr_info[desc_count].dest_addr = dest_addr +
							 (desc_count *
							  ACP_PAGE_SIZE);
		adata->dscr_info[desc_count].dma_transfer_count.bits.count =
								dsp_data_size;
		adata->dscr_info[desc_count].dma_transfer_count.bits.ioc = 0x01;
		desc_count++;
	} else if (desc_count > 0) {
		adata->dscr_info[desc_count].dma_transfer_count.bits.ioc = 0x01;
	}
	status = acpbus_dma_start(adata, &ch_no, (unsigned short *)&desc_count,
				  adata->dscr_info, priority_level);
	if (status == 0x00)
		*p_ch_num = ch_no;
	else
		dev_err(sdev->dev, "acpbus_dma_start failed\n");
	for (index = 0; index < desc_count; index++) {
		memset(&adata->dscr_info[index], 0x00,
		       sizeof(struct acp_config_dma_descriptor));
	}
	return status;
}

int acpbus_configure_and_run_sha_dma(struct acp_dev_data *adata,
					     void *image_addr,
					     unsigned int sha_dma_start_addr,
					     unsigned int sha_dma_dest_addr,
					     unsigned int image_length)
{
	struct snd_sof_dev *sdev = adata->dev;
	int  ret = 0;
	int  val = 0;
	int  sha_dma_status = 0;
	u32  delay_count = 0;
	u32  image_offset = 0;
	u32  sha_message_length = 0;

	u32      acp_sha_dma_start_addr;
	u32      acp_sha_dma_dest_addr;
	u32      acp_sha_msg_length;
	u32      acp_sha_dma_cmd =  0;
	u32      acp_sha_transfer_byte_count =  0;
	u32      acp_sha_dsp_fw_qualifier = 0;
	unsigned long jiffies = msecs_to_jiffies(3);
	unsigned int value = 0;

	if (image_addr) {
		image_offset = 0;
		sha_message_length = image_length;

		val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_SHA_DMA_CMD);
		if (val & 0x01) {
			val = 0x02;
			snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_SHA_DMA_CMD, val);
			delay_count = 0;

			while (1) {
				val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_SHA_DMA_CMD_STS);
				if (!(val & 0x02)) {
					if (delay_count > 0x100) {
						dev_err(sdev->dev, "SHA DMA Failed to Reset\n");
						ret  = -1;
						break;
					}
					udelay(1);
					delay_count++;
				} else {
					val = 0x0;
					snd_sof_dsp_write(sdev, ACP_DSP_BAR,
							  ACP_SHA_DMA_CMD, val);
					break;
				}
			}
		}
		acp_sha_dma_start_addr = sha_dma_start_addr + image_offset;
		snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_SHA_DMA_STRT_ADDR,
				  acp_sha_dma_start_addr);
		acp_sha_dma_dest_addr = sha_dma_dest_addr;
		snd_sof_dsp_write(sdev, ACP_DSP_BAR,
				  ACP_SHA_DMA_DESTINATION_ADDR,
				  acp_sha_dma_dest_addr);
		acp_sha_msg_length = sha_message_length;
		snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_SHA_MSG_LENGTH,
				  acp_sha_msg_length);
		/* enables SHA DMA IOC bit & Run Bit */
		acp_sha_dma_cmd = 0x05;
		snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_SHA_DMA_CMD,
				  acp_sha_dma_cmd);
		acp_sha_dma_cmd = snd_sof_dsp_read(sdev, ACP_DSP_BAR,
						   ACP_SHA_DMA_CMD);

		delay_count = 0;
		while (1) {
			acp_sha_transfer_byte_count = snd_sof_dsp_read(sdev,
							ACP_DSP_BAR,
							ACP_SHA_TRANSFER_BYTE_CNT);

			if (acp_sha_transfer_byte_count != sha_message_length) {
				if (delay_count > 0xA00) {
					dev_err(sdev->dev, "sha dma count :0X%x\n",
					       acp_sha_transfer_byte_count);
					ret  = -1;
					break;
				}
				udelay(1);
				delay_count++;
			} else {
				pr_info("sha dma transfer count :0X%x\n",
					 acp_sha_transfer_byte_count);
				break;
			}
		}

		if (ret) {
			dev_err(sdev->dev, "SHA DMA Failed to Transfer Message Length\n");
			goto sha_exit;
		}
		sha_dma_status =
		wait_event_interruptible_timeout(adata->wait_queue_sha_dma,
						 adata->sha_irq_stat != 0,
						 jiffies);
		if (!sha_dma_status)
			dev_info(sdev->dev,
				 "sha dma waitqueue timeout error :0x%x\n",
				 sha_dma_status);
		else if (sha_dma_status < 0)
			dev_err(sdev->dev, "sha dma wait queue error: 0x%x\n",
				 sha_dma_status);
		pr_info("sha irq status : 0x%x\n", adata->sha_irq_stat);
		adata->sha_irq_stat  = 0;
		value = snd_sof_dsp_read(sdev, ACP_DSP_BAR,
					 ACP_SHA_TRANSFER_BYTE_CNT);
		pr_info("ACP_SHA_TRANSFER_BYTE_CNT 0x%x\n", value);
		value = snd_sof_dsp_read(sdev, ACP_DSP_BAR,
					 ACP_SHA_DMA_ERR_STATUS);
		pr_info("ACP_SHA_DMA_ERR_STATUS 0x%x\n", value);


		pr_info("ACP_SHA_DSP_FW_QUALIFIER 0x%x\n",
			 acp_sha_dsp_fw_qualifier);

		acp_sha_dsp_fw_qualifier = snd_sof_dsp_read(sdev, ACP_DSP_BAR,
						ACP_SHA_DSP_FW_QUALIFIER);

		pr_info("ACP_SHA_DSP_FW_QUALIFIER 0x%x\n",
			 acp_sha_dsp_fw_qualifier);
		snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_SHA_DSP_FW_QUALIFIER, 0x01);

		acp_sha_dsp_fw_qualifier = snd_sof_dsp_read(sdev, ACP_DSP_BAR,
						ACP_SHA_DSP_FW_QUALIFIER);
		pr_info("ACP_SHA_DSP_FW_QUALIFIER 0x%x\n",
			 acp_sha_dsp_fw_qualifier);

		if ((acp_sha_dsp_fw_qualifier & 0x01) != 0x01)
			dev_err(sdev->dev, "PSP validation failed on ACP FW/Plugin image\n");
	}

sha_exit:
	return ret;
}

int acp_dma_status(struct acp_dev_data *adata, unsigned char ch_num)
{
	struct snd_sof_dev *sdev = adata->dev;
	int  status = 0x00;
	u32  timeout = 0;
	u32 acp_dma_cntl0 = 0, acp_dma_ch_stat = 0;
	u32 val = 0;

	acp_dma_cntl0 = snd_sof_dsp_read(sdev, ACP_DSP_BAR,
					 ACP_DMA_CNTL_0 +
					 (ch_num * sizeof(u32)));
	if (acp_dma_cntl0 & 0x02) {
		dev_info(sdev->dev, "DMA_CHANNEL_%d run bit set\n", ch_num);
		while (1) {
			acp_dma_ch_stat = snd_sof_dsp_read(sdev, ACP_DSP_BAR,
							   ACP_DMA_CH_STS);
			if (acp_dma_ch_stat & (1 << ch_num)) {
				udelay(1);
				if (timeout > 3000) {
					dev_err(sdev->dev, "DMA_CHANNEL status error timeout\n");
					status = -ETIMEDOUT;
					break;
				}
				timeout++;
			} else {
				dev_info(sdev->dev,
					 "DMA_CHANNEL_%d transfer completed\n",
					 ch_num);
				break;
			}
		}
	} else {
		dev_err(sdev->dev, "DMA_CHANNEL_%d run bit not set\n", ch_num);
	}
	val =  snd_sof_dsp_read(sdev, ACP_DSP_BAR,
			(ACP_DMA_ERR_STS_0 + (ch_num * sizeof(u32))));
	dev_info(sdev->dev, "ACP_DMA_ERR_STS_0 : 0x%x\n", val);
	return status;
}

static void acpbus_init_scratch_memory(struct acp_dev_data *adata,
					struct AcpScratchRegisterConfig *scratch_buf_info)
{
	writel(0, (u32 *)(&scratch_buf_info->sofHostMsgWrite));
	writel(0, (u32 *)(&scratch_buf_info->sofHostAckwrite));
}

static void  acpbus_config_command_resp_buffers(struct acp_dev_data *adata)
{
	acpbus_init_scratch_memory(adata,
					&adata->acp_sregmap->scratch_buf_info);
}

void acp_copy_from_scratch_memory(struct snd_sof_dev *sdev,
				  unsigned char *ring_buffer, u32  offset,
				  void *read_buffer, u32 bytes_to_write)
{
		int index;
		int val = 0;
		unsigned int *src_buf = (unsigned int *)(ring_buffer + offset);
		unsigned int *dst_buf = (unsigned int *)read_buffer;
		unsigned int scratch_reg = 0;

		for (index = 0; index < (bytes_to_write / 4); index++) {
			val = readl((u32 *)(src_buf + index));
			scratch_reg = (ACP_SCRATCH_REG_0 + (index * 4));
			*(dst_buf + index) = val;
		}
}

void acp_copy_to_scratch_memory(struct snd_sof_dev *sdev,
				unsigned char *ring_buffer,
				u32  offset,
				void *write_buffer,
				u32 bytes_to_write)
{
	int index;
	unsigned int *src_buf = (unsigned int *)write_buffer;
	unsigned int *dest_buf = (unsigned int *)ring_buffer;

	for (index = 0; index < (bytes_to_write / 4); index++)
		writel(*(src_buf + index),
				(u32 *)(dest_buf + (offset / 4) + index));
}

static void init_scratch_memory(struct acp_dev_data *adata)
{
	struct snd_sof_dev *sdev = adata->dev;

	adata->acp_sregmap = (struct acp_scratch_register_config *)
			     (sdev->bar[ACP_DSP_BAR] + ACP_SCRATCH_REG_0 -
			      ACPBUS_REG_BASE_OFFSET);

	if (adata->acp_sregmap == NULL)
		dev_err(sdev->dev, "failed to assign acp regmap\n");


	memset(adata->acp_sregmap_context, 0x00,
	       sizeof(struct acp_scratch_register_config));
	acp_copy_to_scratch_memory(sdev, (unsigned char *)adata->acp_sregmap,
				   0x00,
				   adata->acp_sregmap_context,
				   sizeof(struct acp_scratch_register_config));
	acpbus_config_command_resp_buffers(adata);
}

static int acp_probe_continue(struct acp_dev_data *adata)
{
	struct pci_dev *pci;
	struct snd_sof_dev *sdev = adata->dev;

	pci = pci_get_device(PCI_VENDOR_ID_AMD, 0x15e2, NULL);
	acp_enable_extirq(adata);
	configure_acp_groupregisters(adata);
	configure_dma_descriptiors(adata);
	adata->acp_sregmap_context =
		devm_kzalloc(sdev->dev,
			     sizeof(struct acp_scratch_register_config),
			     GFP_KERNEL);
	if (!adata->acp_sregmap_context) {
		dev_err(sdev->dev, "buffer allocation failed for sregmap_conext\n");
		return -ENOMEM;
	}
	init_scratch_memory(adata);
	return 0;
}

static void acp_probe_work(struct work_struct *work)
{
	int ret = 0;
	struct acp_dev_data *adata = container_of(work, struct acp_dev_data,
						    probe_work);
	struct snd_sof_dev *sdev = adata->dev;

	ret = acp_probe_continue(adata);
	if (ret < 0)
		dev_err(sdev->dev, "acp fw load sequence failed\n");
}
int acp_power_on(struct snd_sof_dev *sdev)
{
	u32 val;
	int timeout;

	val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_PGFSM_STATUS);

	if (val == 0)
		return val;

	if (!((val & ACP_PGFSM_STATUS_MASK) ==
				ACP_POWER_ON_IN_PROGRESS))
		snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_PGFSM_CONTROL,
				  ACP_PGFSM_CNTL_POWER_ON_MASK);

	timeout = 0;
	while (++timeout < 500) {
		val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_PGFSM_STATUS);
		if (!val)
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

int acp_reset(struct snd_sof_dev *sdev)
{
	u32 val;
	int timeout;

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_SOFT_RESET, 1);
	timeout = 0;
	while (++timeout < 500) {
		val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_SOFT_RESET);
		if (val & ACP_SOFT_RESET_DONE_MASK)
			break;
		cpu_relax();
	}

	snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_SOFT_RESET, 0);
	timeout = 0;
	while (++timeout < 500) {
		val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_SOFT_RESET);
		if (!val)
			return 0;
		cpu_relax();
	}
	return -ETIMEDOUT;
}

static irqreturn_t acp_irq_thread(int irq, void *context)
{
	u16 acp_flag = 0, dsp_flag = 0;
	unsigned int val = 0;
	struct snd_sof_dev *sdev = context;

	val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_EXTERNAL_INTR_STAT);

	if (((val & ACP_EXT_INTR_ERROR_STAT) >> 29) & 0x01) {
		snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_EXTERNAL_INTR_STAT,
				  ACP_EXT_INTR_ERROR_STAT);
		snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_ERROR_STATUS, 0x00);
		acp_flag =  0x01;
	}

	val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_DSP_SW_INTR_STAT);
	if (val != 0x00) {
		if (val & 0x04) {
			sof_ops(sdev)->irq_thread(irq, sdev);
			val |= 0x04;
			snd_sof_dsp_write(sdev, ACP_DSP_BAR,
					  ACP_DSP_SW_INTR_STAT, val);
			dsp_flag = 0x01;
		}
	}

	if (acp_flag | dsp_flag)
		return IRQ_HANDLED;
	return IRQ_NONE;
};

static irqreturn_t acp_irq_handler(int irq, void *dev_id)
{
	/* TODO: Read global intr and wake thread only then */
	unsigned int val = 0;
	struct snd_sof_dev *sdev = dev_id;

	val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_EXTERNAL_INTR_STAT);

	return IRQ_WAKE_THREAD;
}

static int acp_init(struct snd_sof_dev *sdev)
{
	int ret;

	/* power on */
	ret = acp_power_on(sdev);
	if (ret) {
		dev_err(sdev->dev, "ACP power on failed\n");
		return ret;
	}
	/* Reset */
	ret = acp_reset(sdev);
	if (ret) {
		dev_err(sdev->dev, "ACP reset failed\n");
		return ret;
	}
	return 0;
}

int amd_sof_acp_probe(struct snd_sof_dev *sdev)
{
	struct pci_dev *pci = to_pci_dev(sdev->dev);
	struct acp_dev_data *adata;
	unsigned int ret = 0, addr = 0;

	adata = devm_kzalloc(sdev->dev, sizeof(struct acp_dev_data),
			     GFP_KERNEL);
	if (!adata)
		return -ENOMEM;

	adata->dev = sdev;
	addr = pci_resource_start(pci, ACP_DSP_BAR);
	sdev->bar[ACP_DSP_BAR] = devm_ioremap(sdev->dev, addr,
				   pci_resource_len(pci, ACP_DSP_BAR));

	pci_set_master(pci);

	sdev->pdata->hw_pdata = adata;

	sdev->ipc_irq = pci->irq;
	ret = request_threaded_irq(sdev->ipc_irq, acp_irq_handler,
				   acp_irq_thread, IRQF_SHARED, "AudioDSP",
				   sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to register IRQ %d\n",
			sdev->ipc_irq);
		return ret;
	}

	ret = acp_init(sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed ACP init\n");
		goto out;
	}

	INIT_WORK(&adata->probe_work, acp_probe_work);
	init_waitqueue_head(&adata->wait_queue_sha_dma);

	schedule_work(&adata->probe_work);

	return ret;
out:
	free_irq(sdev->ipc_irq, sdev);
	return ret;
}
EXPORT_SYMBOL_NS(amd_sof_acp_probe, SND_SOC_SOF_AMD_COMMON);

int amd_sof_acp_remove(struct snd_sof_dev *sdev)
{
	unsigned int ret = 0;

	ret = acp_reset(sdev);
	if (ret < 0)
		dev_err(sdev->dev, "ACP reset failed\n");

	if (sdev->ipc_irq)
		free_irq(sdev->ipc_irq, sdev);

	return ret;
}
EXPORT_SYMBOL_NS(amd_sof_acp_remove, SND_SOC_SOF_AMD_COMMON);

MODULE_DESCRIPTION("AMD ACP sof driver");
MODULE_LICENSE("GPL v2");
