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
	u16 acp_flag = 0;
	unsigned int val = 0;
	struct snd_sof_dev *sdev = context;
	struct acp_dev_data *adata = sdev->pdata->hw_pdata;

	val = snd_sof_dsp_read(sdev, ACP_DSP_BAR, ACP_EXTERNAL_INTR_STAT);

	if (((val & ACP_EXT_INTR_ERROR_STAT) >> 29) & 0x01) {
		snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_EXTERNAL_INTR_STAT,
				  ACP_EXT_INTR_ERROR_STAT);
		snd_sof_dsp_write(sdev, ACP_DSP_BAR, ACP_ERROR_STATUS, 0x00);
		acp_flag =  0x01;
	}

	if (acp_flag)
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
