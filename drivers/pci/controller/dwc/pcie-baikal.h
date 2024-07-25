/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCIe controller driver for Baikal Electronics SoCs
 *
 * Copyright (C) 2023 Baikal Electronics, JSC
 */

#ifndef _PCIE_BAIKAL_H
#define _PCIE_BAIKAL_H

void bm1000_pcie_phy_enable(struct dw_pcie *pci);
void bm1000_pcie_phy_disable(struct dw_pcie *pci);

void bm1000_pcie_tune(struct dw_pcie *pci);

#endif /* _PCIE_BAIKAL_H */
