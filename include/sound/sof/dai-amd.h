/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * This file is provided under a BSD license.  When using or
 * redistributing this file, you may do so under this license.
 *
 * Copyright(c) 2021 Advanced Micro Devices, Inc.. All rights reserved.
 */

#ifndef __INCLUDE_SOUND_SOF_DAI_AMD_H__
#define __INCLUDE_SOUND_SOF_DAI_AMD_H__

#include <sound/sof/header.h>

/* ACP Configuration Request - SOF_IPC_DAI_AMD_CONFIG */
struct sof_ipc_dai_acp_params {
	struct sof_ipc_hdr hdr;

	uint32_t fsync_rate;    /* FSYNC frequency in Hz */
	uint32_t tdm_slots;
} __packed;
#endif

