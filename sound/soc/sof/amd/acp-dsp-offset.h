/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * This file is provided under a BSD license. When using or
 * redistributing this file, you may do so under this license.
 *
 * Copyright(c) 2021 Advanced Micro Devices, Inc. All rights reserved.
 *
 * Author: Ajit Kumar Pandey <AjitKumar.Pandey@amd.com>
 */

#ifndef _ACP_DSP_IP_OFFSET_H
#define _ACP_DSP_IP_OFFSET_H

#define ACP_SOFT_RESET				0x1000

// Registers from ACP_PGFSM block
#define ACP_PGFSM_CONTROL			0x141C
#define ACP_PGFSM_STATUS			0x1420

#define ACP_EXTERNAL_INTR_ENB			0x1800
#define ACP_EXTERNAL_INTR_CNTL			0x1804
#define ACP_EXTERNAL_INTR_STAT			0x1808
#define ACP_ERROR_STATUS			0x18C4

#endif
