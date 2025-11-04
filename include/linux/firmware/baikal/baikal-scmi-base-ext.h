// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025, Baikal Electronics, JSC
 */

#ifndef __BAIKAL_SCMI_BASE_EXT_H
#define __BAIKAL_SCMI_BASE_EXT_H

#include <linux/scmi_protocol.h>

#define BAIKAL_SCMI_PROTOCOL_BASE_EXT	0x80

enum baikal_scmi_base_ext_protocol_cmd {
	XCP_CONFIG_GET = 0x3,
	SOC_STATUS_GET = 0x5,
};

enum baikal_scmi_service_id {
	SID_BASIC    = 0x00,
	SID_RESET    = 0x01,
	SID_PVT      = 0x02,
	SID_EFUSE    = 0x03,
	SID_STATUS   = 0x04,
	SID_DEBUG    = 0x05,
	SID_HARDWARE = 0x06,
};

enum baikal_scmi_config_id_basic {
	Boot_Flags            = 0x0000,
	Boot_SlaveSocToMks    = 0x0004,
	Boot_SlaveSocCfrTries = 0x0008,
	Boot_Core0StartAddr   = 0x000C,
	Run_Flags             = 0x0100,
	Run_Spi0RateMhz       = 0x0104,
	Run_Spi1RateMhz       = 0x0106,
	Run_Uart0Rate         = 0x0108,
	Run_EventPeriodMks    = 0x010C,
	Run_MacAddrBase       = 0x0110,
	Run_CpuDisable        = 0x0118,
	Run_DsuDisable        = 0x0120,
	Run_DdrRankDisable    = 0x0124,
	Run_DdrRateMHz        = 0x0128,
	Run_CmnFreqMHz        = 0x012A,
	Run_DsuFreqMHz        = 0x012C,
	Run_CpuFreqMHz        = 0x012E,
};

enum baikal_scmi_config_id_reset {
	Rst_Flags           = 0x0000,
	Rst_ScpWdtTimeoutMs = 0x0004,
	Rst_WdtTimeoutMs    = 0x0008,
	Rst_WdtCooldownMs   = 0x000C,
	Rst_OsShutdownToMs  = 0x0010,
};

enum baikal_scmi_config_id_pvt {
	PVT_Flags     = 0x0000,
	PVT_VHYThresh = 0x0004,
	PVT_VLYThresh = 0x0006,
	PVT_VHRThresh = 0x0008,
	PVT_VLRThresh = 0x000A,
	PVT_TYThresh  = 0x001C,
	PVT_TRThresh  = 0x001E,
};

enum baikal_scmi_status_id_basic {
	XCP_Status  = 0x0000,
	XCP_Version = 0x0001,
	XCP_Date    = 0x0002,
};

enum baikal_scmi_status_id_pvt {
	PVT_Threshold  = 0x0000,
	PVT_NofActive  = 0x0002,
	PVT_ActiveMask = 0x0004,
	PVT_TAvr       = 0x0008,
	PVT_VAvr       = 0x000C,
	PVT_Tall       = 0x0010,
	PVT_Vall       = 0x0028,
	PVT_HVTall     = 0x0040,
	PVT_SVTall     = 0x0058,
	PVT_LVTall     = 0x0070,
	PVT_Value      = 0x1000,
};

enum baikal_scmi_status_id_efuse {
	EFUSE_Process         = 0x0000,
	EFUSE_ProjectName     = 0x0001,
	EFUSE_ProjectRevision = 0x0002,
	EFUSE_LotID           = 0x0003,
	EFUSE_SerialNumber    = 0x0004,
	EFUSE_MacOffset       = 0x0005,
	EFUSE_BinMask         = 0x0006,
};

enum baikal_scmi_status_id_status {
	STATUS_ScpFreq     = 0x0000,
	STATUS_CmnFreq     = 0x0010,
	STATUS_DsuFreq     = 0x0100,
	STATUS_DsuMode     = 0x0101,
	STATUS_CpuFreq     = 0x0200,
	STATUS_CpuMode     = 0x0201,
	STATUS_DdrFreq     = 0x0300,
	STATUS_DdrMode     = 0x0301,
	STATUS_PcieMode    = 0x0400,
	STATUS_PcieModeExt = 0x0401,
};

struct baikal_scmi_base_ext_req {
	u32 service_id;
	u32 value_id;
};

struct baikal_scmi_base_ext_resp {
	u32 count;
	u32 value[4];
};

struct baikal_scmi_base_ext_proto_ops {
	unsigned int (*node_id)(void *ph);
	int (*xcp_config_get)(void *ph,
			      struct baikal_scmi_base_ext_req *req,
			      struct baikal_scmi_base_ext_resp *resp);
	int (*soc_status_get)(void *ph,
			      struct baikal_scmi_base_ext_req *req,
			      struct baikal_scmi_base_ext_resp *resp);
};

/* ACPI support */

#if IS_ENABLED(CONFIG_BAIKAL_SCMI_BASE_EXT_ACPI)

struct baikal_scmi_base_ext_proto_handle;

const struct baikal_scmi_base_ext_proto_ops *baikal_scmi_base_ext_proto_get(
		struct device *dev,
		struct baikal_scmi_base_ext_proto_handle **ph);
void baikal_scmi_base_ext_proto_put(struct baikal_scmi_base_ext_proto_handle *ph);

#endif /* CONFIG_BAIKAL_SCMI_BASE_EXT_ACPI */

#endif /* __BAIKAL_SCMI_BASE_EXT_H */
