// SPDX-License-Identifier: BSD-3-Clause
//
// This file is provided under a dual BSD license. When using or
// redistributing this file, you may do so under this license.
//
// Copyright(c) 2021 Advanced Micro Devices, Inc. All rights reserved.
//
// Authors: Ajit Kumar Pandey <AjitKumar.Pandey@amd.com>
//

/* ACP machine configuration module */

#include <linux/acpi.h>
#include <linux/bits.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <sound/soc-acpi.h>
#include "mach-config.h"

#define DRV_NAME	"acp-config"

unsigned int  acp_quirk_data;

static const struct dmi_system_id renoir_quirk_table[] = {
		{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "AMD"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Majolica-CZN"),
		},
	},
	{},
};

static const struct config_entry config_table[] = {
	{
		.flags = FLAG_SOF_ONLY_DMIC,
		.device = 0x15e2,
		.dmi_table = renoir_quirk_table,
	},
};

int snd_amd_acp_find_config(struct pci_dev *pci)
{
	u16 device;
	int i = 0;
	const struct config_entry *table = config_table;

	device = pci->device;

	for (i = 0; i < ARRAY_SIZE(config_table); i++, table++) {
		if (table->device != device)
			continue;
		if (table->dmi_table && !dmi_check_system(table->dmi_table))
			continue;
		acp_quirk_data = table->flags;
		return table->flags;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_amd_acp_find_config);

struct snd_soc_acpi_mach snd_soc_acpi_amd_sof_machines[] = {
	{
		.id = "AMDI1019",
		.drv_name = "renoir-dsp",
		.pdata = (void *)&acp_quirk_data,
		.fw_filename = "sof-rn.ri",
		.sof_tplg_filename = "sof-acp.tplg",
	},
	{},
};
EXPORT_SYMBOL_GPL(snd_soc_acpi_amd_sof_machines);

struct snd_soc_acpi_mach snd_soc_acpi_amd_acp_machines[] = {
	{
		.id = "AMDI1019",
		.drv_name = "renoir-acp",
		.pdata = (void *)&acp_quirk_data,
	},
	{},
};
EXPORT_SYMBOL_GPL(snd_soc_acpi_amd_acp_machines);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:"DRV_NAME);

