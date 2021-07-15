/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * This file is provided under a BSD license. When using or
 * redistributing this file, you may do so under this license.
 *
 * Copyright(c) 2021 Advanced Micro Devices, Inc. All rights reserved.
 *
 * Author: Ajit Kumar Pandey <AjitKumar.Pandey@amd.com>
 */

#define FLAG_SOF                        BIT(1)
#define FLAG_SOF_ONLY_DMIC              BIT(2)

#define ACP_PCI_DEV_ID			0x15E2

extern struct snd_soc_acpi_mach snd_soc_acpi_amd_sof_machines[];
extern struct snd_soc_acpi_mach snd_soc_acpi_amd_acp_machines[];

struct config_entry {
	u32 flags;
	u16 device;
	const struct dmi_system_id *dmi_table;
};

