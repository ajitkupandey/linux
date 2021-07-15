/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * This file is provided under a BSD license. When using or
 * redistributing this file, you may do so under this license.
 *
 * Copyright(c) 2021 Advanced Micro Devices, Inc. All rights reserved.
 *
 * Author: Ajit Kumar Pandey <AjitKumar.Pandey@amd.com>
 */

#ifndef __SOF_AMD_ACP_H
#define __SOF_AMD_ACP_H

#include "acp-dsp-offset.h"

#define ACP_DSP_BAR	0

#define ACP_PGFSM_CNTL_POWER_ON_MASK		0x01
#define ACP_PGFSM_CNTL_POWER_OFF_MASK		0x00
#define ACP_PGFSM_STATUS_MASK			0x03
#define ACP_POWERED_ON				0x00
#define ACP_POWER_ON_IN_PROGRESS		0x01
#define ACP_POWERED_OFF				0x02
#define ACP_POWER_OFF_IN_PROGRESS		0x03
#define ACP_SOFT_RESET_DONE_MASK		0x00010001
#define ACP_EXT_INTR_ERROR_STAT			0x20000000

#define ACP_SRAM_PTE_OFFSET			0x02050000
#define ACP_ERROR_MASK				0x20000000
#define ACP_SHA_STAT				0x8000
#define FW_BIN_PTE_OFFSET			0x00
#define FW_DATA_BIN_PTE_OFFSET			0x08
#define PAGE_SIZE_4K_ENABLE			0x2
#define ACP_SRBM_DATA_RAM_BASE_OFFSET		0x0100C000
#define ACP_PAGE_SIZE				0x1000
#define ACP_DMA_CH_RUN				0x02
#define ACP_DMA_CH_IOC_ENABLE			0x04
#define ACPBUS_REG_BASE_OFFSET			ACP_DMA_CNTL_0

#define ACP_DEFAULT_DRAM_LENGTH			0x00080000
#define ACP_SCRATCH_MEMORY_ADDRESS		0x02050000
#define ACP_SYSTEM_MEMORY_WINDOW		0x4000000
#define ACP_IRAM_BASE_ADDRESS			0x000000
#define ACP_DATA_RAM_BASE_ADDRESS		0x01000000
#define ACP_DEFAULT_DRAM_LENGTH			0x00080000
#define ACP_DRAM_PAGE_COUNT			128

/* DSP hardware descriptor */
struct sof_amd_dsp_desc {
	int cores_num;
	int acp_reg_base;
	int acp_reg_end;
	int num_res;
	char *dev_name;
	struct resource *res;
	int num;
};

struct  acp_atu_grp_pte {
	u32 addr_low;
	u32 addr_high;
};

union acp_config_dma_trnsfer_cnt {
	struct {
		unsigned int  count : 19;
		unsigned int  reserved : 12;
		unsigned  ioc : 1;
	} bitfields, bits;
	unsigned int    u32_all;
	signed int  i32_all;
};

struct acp_config_dma_descriptor {
	unsigned int    src_addr;
	unsigned int    dest_addr;
	union acp_config_dma_trnsfer_cnt  dma_transfer_count;
	unsigned int	reserved;
};

/* Scratch memory structure for communication b/w
 * host and dsp.
 */

struct  AcpScratchRegisterConfig {

	/* DSP mailbox */
	u8      sofOutBoxBuffer[512];
	/* Host mailbox */
	u8      sofInBoxBuffer[512];
	/* Debug memory */
	u8      sofDebugBuffer[1024];
	/* Exception memory*/
	u8      sofExceptBuffer[1024];
	/* Stream buffer */
	u8      sofStreamBuffer[1024];
	/* Trace buffer */
	u8      sofTraceBuffer[1024];
	/* Host msg flag */
	u32     sofHostMsgWrite;
	/* Host ack flag*/
	u32     sofHostAckwrite;
	/* DSP msg flag */
	u32     sofDspMsgWrite;
	/* Dsp ack flag */
	u32     sofDspAckwrite;
};

/* acp_scratch_register_config structure used to map scratch memory
 * acp_atu_grp1_pte - Group1 PTE entries structure
 * acp_config_dma_desc - acp dma descriptor structure
 * acp_cmd_ring_buf_info - this structure hold information about
 * acp command ring buffer like write_index, current_write_index
 * acp_cmd_buf - acp command ring buffer
 * acp_resp_ring_buf_info - acp response ring buffer information
 * such as read index, write index
 * acp_resp_buf - acp response ring buffer
 */
struct  acp_scratch_register_config {
	struct AcpScratchRegisterConfig scratch_buf_info;
	struct acp_atu_grp_pte    acp_atu_grp1_pte[16];
	struct acp_atu_grp_pte    acp_atu_grp2_pte[16];
	struct acp_atu_grp_pte    acp_atu_grp3_pte[16];
	struct acp_atu_grp_pte    acp_atu_grp4_pte[16];
	struct acp_atu_grp_pte    acp_atu_grp5_pte[16];
	struct acp_atu_grp_pte    acp_atu_grp6_pte[16];
	struct acp_atu_grp_pte    acp_atu_grp7_pte[16];
	struct acp_atu_grp_pte    acp_atu_grp8_pte[16];
	struct acp_config_dma_descriptor	acp_config_dma_desc[64];
	unsigned int reg_offset[8];
	unsigned int buf_size[8];
	u8      acpTransmitFifoBuffer[256];
	u8      acpReceiveFifoBuffer[256];
	unsigned int	reserve[];
};

enum buffer_type {
	FW_BIN = 0,
	FW_DATA_BIN,
};
/* Common device data struct for ACP devices */
struct acp_dev_data {
	struct snd_sof_dev  *dev;
	struct acp_scratch_register_config *acp_sregmap;
	struct acp_scratch_register_config *acp_sregmap_context;
	unsigned int fw_bin_size;
	unsigned int fw_data_bin_size;

	u32 fw_bin_page_count;
	dma_addr_t sha_dma_addr;
	u8 *bin_buf;
	dma_addr_t dma_addr;
	u8 *data_buf;

	u32 sha_irq_stat;
	struct work_struct probe_work;

	wait_queue_head_t wait_queue_sha_dma;
	struct acp_config_dma_descriptor dscr_info[128];
};

/* Generic ACP helper services */
void acp_copy_to_scratch_memory(struct snd_sof_dev *sdev,
				unsigned char *ring_buffer,
				u32  offset,
				void *write_buffer,
				u32 bytes_to_write);
void acp_copy_from_scratch_memory(struct snd_sof_dev *sdev,
				  unsigned char *ring_buffer,
				  u32  offset,
				  void *read_buffer,
				  u32 bytes_to_write);

int acp_dma_status(struct acp_dev_data *adata, unsigned char ch_num);
int acpbus_configure_and_run_dma(struct acp_dev_data *adata,
				 unsigned int src_addr,
				 unsigned int dest_addr,
				 unsigned int dsp_data_size,
				 unsigned char *p_ch_num);
int  acpbus_configure_and_run_sha_dma(struct acp_dev_data *adata,
				      void *image_addr,
				      unsigned int sha_dma_start_addr,
				      unsigned int sha_dma_dest_addr,
				      unsigned int image_length);
/* ACP device probe/remove */
int amd_sof_acp_probe(struct snd_sof_dev *sdev);
int amd_sof_acp_remove(struct snd_sof_dev *sdev);

/* DSP Loader callbacks */
int acp_sof_dsp_run(struct snd_sof_dev *sdev);
int acp_dsp_pre_fw_run(struct snd_sof_dev *sdev);
int acp_get_bar_index(struct snd_sof_dev *sdev, u32 type);

/* Block IO callbacks */
int acp_dsp_block_write(struct snd_sof_dev *sdev,
			enum snd_sof_fw_blk_type blk_type, u32 offset, void *src,
			size_t size);
int acp_dsp_block_read(struct snd_sof_dev *sdev,
		       enum snd_sof_fw_blk_type blk_type, u32 offset, void *dest,
		       size_t size);

extern const struct snd_sof_dsp_ops sof_renoir_ops;
#endif
