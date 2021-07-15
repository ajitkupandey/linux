// SPDX-License-Identifier: BSD-3-Clause
//
// This file is provided under a BSD license.  When using or
// redistributing this file, you may do so under this license.
//
// Copyright(c) 2021 Advanced Micro Devices, Inc. All rights reserved.
//
// Authors: Balakishore Pati <Balakishore.pati@amd.com>
//	    Ajit Kumar Pandey	<AjitKumar.Pandey@amd.com>

/* ACP-specific SOF IPC code */

#include <linux/pci.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <sound/sof.h>
#include <sound/sof/xtensa.h>
#include "../ops.h"

#include "acp.h"

void acpbus_trigger_host_to_dsp_swintr(struct acp_dev_data *adata)
{
	struct snd_sof_dev *sdev = adata->dev;
	u32 swintr_trigger = 0x00;

	swintr_trigger = snd_sof_dsp_read(sdev, ACP_DSP_BAR,
					  ACP_SW_INTR_TRIG);
	swintr_trigger |= 0x01;
	snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_SW_INTR_TRIG,
			  swintr_trigger);
}

void sof_ipc_host_ack_set(struct AcpScratchRegisterConfig *scratch_buf_info)
{
	writel(1, (u32 *)(&scratch_buf_info->sofHostAckwrite));
}

void sof_ipc_host_msg_set(struct AcpScratchRegisterConfig *scratch_buf_info)
{
	writel(1, (u32 *)(&scratch_buf_info->sofHostMsgWrite));
}

static void acp_dsp_ipc_host_done(struct AcpScratchRegisterConfig *scratch_buf_info)
{
	 writel(0, (u32 *)(&scratch_buf_info->sofDspMsgWrite));
}

static void acp_dsp_ipc_dsp_done(struct AcpScratchRegisterConfig *scratch_buf_info)
{
	 writel(0, (u32 *)(&scratch_buf_info->sofDspAckwrite));
}

/*
 * This function is called when host has some thing to send
 * data to DSP.So the direction is Host->DSP
 * @sdev: sof  device.
 * @msg : Which has the msg data to send to DSP.
 *
 */
int acp_sof_ipc_send_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	struct acp_dev_data *adata = sdev->pdata->hw_pdata;
	unsigned char *ring_buffer = adata->acp_sregmap->scratch_buf_info.sofInBoxBuffer;
	int HostMsgWrite = 0;

	acp_copy_to_scratch_memory(sdev, ring_buffer, 0, msg->msg_data,
				   msg->msg_size);
	sof_ipc_host_msg_set(&adata->acp_sregmap->scratch_buf_info);
	HostMsgWrite = readl(&adata->acp_sregmap->scratch_buf_info.sofHostMsgWrite);

	/* Trigger host to dsp interrupt for the msg */
	acpbus_trigger_host_to_dsp_swintr(adata);
	return 0;
}
EXPORT_SYMBOL_NS(acp_sof_ipc_send_msg, SND_SOC_SOF_AMD_COMMON);

void acp_dsp_ipc_get_reply(struct snd_sof_dev *sdev)
{
	struct snd_sof_ipc_msg *msg = sdev->msg;
	struct sof_ipc_reply reply;
	struct sof_ipc_cmd_hdr *hdr;
	int ret = 0;
	struct acp_dev_data *adata = sdev->pdata->hw_pdata;
	unsigned char *ring_buffer = adata->acp_sregmap->scratch_buf_info.sofInBoxBuffer;

       /*
	* Sometimes, there is unexpected reply ipc arriving. The reply
	* ipc belongs to none of the ipcs sent from driver.
	* In this case, the driver must ignore the ipc.
	*/
	if (!msg) {
		dev_warn(sdev->dev, "unexpected ipc interrupt raised!\n");
		return;
	}
	hdr = msg->msg_data;
	if (hdr->cmd == (SOF_IPC_GLB_PM_MSG | SOF_IPC_PM_CTX_SAVE) ||
	    hdr->cmd == (SOF_IPC_GLB_PM_MSG | SOF_IPC_PM_GATE)) {
		/*
		 * memory windows are powered off before sending IPC reply,
		 * so we can't read the mailbox for CTX_SAVE and PM_GATE
		 * replies.
		 */
		reply.error = 0;
		reply.hdr.cmd = SOF_IPC_GLB_REPLY;
		reply.hdr.size = sizeof(reply);
		memcpy(msg->reply_data, &reply, sizeof(reply));
		goto out;
	}
	/* get IPC reply from DSP in the mailbox */
	acp_copy_from_scratch_memory(sdev, ring_buffer, 0, &reply, sizeof(reply));
	if (reply.error < 0) {
		memcpy(msg->reply_data, &reply, sizeof(reply));
		ret = reply.error;
	} else {
		/* reply correct size ? */
		if (reply.hdr.size != msg->reply_size &&
		    !(reply.hdr.cmd & SOF_IPC_GLB_PROBE)) {
			dev_err(sdev->dev, "error: reply expected %zu got %u bytes\n",
				msg->reply_size, reply.hdr.size);
			ret = -EINVAL;
		}
		/* read the message */
		if (msg->reply_size > 0)
			acp_copy_from_scratch_memory(sdev, ring_buffer, 0,
						     msg->reply_data, msg->reply_size);
	}
out:
	msg->reply_error = ret;
}

irqreturn_t acp_sof_ipc_irq_thread(int irq, void *context)
{
	struct snd_sof_dev *sdev = context;
	bool ipc_irq = false;
	int msg = 0, DspMsg = 0, DspAck = 0;
	struct acp_dev_data *adata = sdev->pdata->hw_pdata;

	DspMsg = readl(&adata->acp_sregmap->scratch_buf_info.sofDspMsgWrite);
	if (DspMsg) {
		snd_sof_ipc_msgs_rx(sdev);
		acp_dsp_ipc_host_done(&adata->acp_sregmap->scratch_buf_info);
		ipc_irq = true;
	}
	DspAck = readl(&adata->acp_sregmap->scratch_buf_info.sofDspAckwrite);
	if (DspAck) {
		spin_lock_irq(&sdev->ipc_lock);
		/* handle immediate reply from DSP core */
		acp_dsp_ipc_get_reply(sdev);
		snd_sof_ipc_reply(sdev, msg);
		/* set the done bit */
		acp_dsp_ipc_dsp_done(&adata->acp_sregmap->scratch_buf_info);
		spin_unlock_irq(&sdev->ipc_lock);
		ipc_irq = true;
	}
	if (!ipc_irq) {
       /*
	* This interrupt is not shared so no need to return IRQ_NONE.
	*/
		dev_dbg_ratelimited(sdev->dev,
				"nothing to do in IPC IRQ thread\n");
	}
	return IRQ_HANDLED;
}
EXPORT_SYMBOL_NS(acp_sof_ipc_irq_thread, SND_SOC_SOF_AMD_COMMON);
/*
 * This function is called when DSP has some thing to send
 * data to Host.So the direction is DSP->Host
 * @sdev: sof  device.
 * @stream : DSP stream..
 * @p:message to host
 * @sz : size of msg.
 */
int acp_sof_ipc_msg_data(struct snd_sof_dev *sdev,
			  struct snd_pcm_substream *substream,
			  void *p, size_t sz)
{
	struct acp_dev_data *adata = sdev->pdata->hw_pdata;
	unsigned char *ring_buffer = adata->acp_sregmap->scratch_buf_info.sofOutBoxBuffer;

	if (!substream || !sdev->stream_box.size)
		acp_copy_from_scratch_memory(sdev, ring_buffer, 0, p, sz);

	return 0;
}
EXPORT_SYMBOL_NS(acp_sof_ipc_msg_data, SND_SOC_SOF_AMD_COMMON);


int acp_sof_ipc_pcm_params(struct snd_sof_dev *sdev,
			struct snd_pcm_substream *substream,
			const struct sof_ipc_pcm_params_reply *reply)
{
	return 0;
}
EXPORT_SYMBOL_NS(acp_sof_ipc_pcm_params, SND_SOC_SOF_AMD_COMMON);

/*
 * This function is called to get the offset of DSP buffer
 * where the data will be copied from host
 * @sdev: sof  device.
 *
 */
int acp_sof_ipc_get_mailbox_offset(struct snd_sof_dev *sdev)
{
	return ACP_SCRATCH_MEMORY_ADDRESS;
}
EXPORT_SYMBOL_NS(acp_sof_ipc_get_mailbox_offset, SND_SOC_SOF_AMD_COMMON);

MODULE_DESCRIPTION("AMD ACP sof-ipc driver");
MODULE_LICENSE("GPL v2");
