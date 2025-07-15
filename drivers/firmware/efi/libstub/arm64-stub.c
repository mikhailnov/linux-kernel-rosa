// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2013, 2014 Linaro Ltd;  <roy.franz@linaro.org>
 *
 * This file implements the EFI boot stub for the arm64 kernel.
 * Adapted from ARM version by Mark Salter <msalter@redhat.com>
 */


#include <linux/efi.h>
#include <asm/efi.h>
#include <asm/memory.h>
#include <asm/sections.h>

#include "efistub.h"
#include <linux/libfdt.h>


/*
 * Guess if we are booting on Baikal-M (aka BE-M1000) SoC.
 * Use FDT and SMBIOS
 */
static bool is_baikalm(void) {
	static const char *baikalm_compat[] = {
		"baikal,baikal-m",
		"baikal,bm1000",
	};
	struct efi_smbios_type4_record *cpu_record = NULL;
	const u32 __aligned(1) *socid = NULL;
	const char *match = NULL;
	const void *fdt = NULL;

	fdt = get_efi_config_table(DEVICE_TREE_GUID);
	cpu_record = (struct efi_smbios_type4_record *)efi_get_smbios_record(4);

	if (fdt) {
		for (size_t i = 0; i < ARRAY_SIZE(baikalm_compat); i++) {
			match = baikalm_compat[i];
			if (fdt_node_check_compatible(fdt, 0, match) == 0) {
				efi_info_once("detected Baikal-M via FDT: %s\n", match);
				return true;
			}
		}
	} else {
		efi_debug_once("failed to retrive FDT from EFI\n");
	}

	if (cpu_record) {
		socid = (u32 *)cpu_record->processor_id;
		switch (*socid) {
		case 0x411fd073U:
			efi_info_once("detected Baikal-M CPU via SMBIOS (SoC ID: %x)\n", *socid);
			return true;
			break;
		default:
			return false;
		}
	} else {
		efi_debug_once("failed to retrive processor_id from SMBIOS\n");
	}
	return false;
}

struct efi_smbios_type0_record {
	struct efi_smbios_record header;
	u8 data[];
};

static void get_bios_version(unsigned *fw_major, unsigned *fw_minor,
			     const struct efi_smbios_type0_record *firmware_record) {

	if (!fw_major || !fw_minor ||!firmware_record) {
		return;
	}
	if (firmware_record->header.length >= 0x18 &&
			firmware_record->data[0x10] != 0xff &&
			firmware_record->data[0x11] != 0xff) {
		*fw_major = firmware_record->data[0x10];
		*fw_minor = firmware_record->data[0x11];
	}
}

/*
 * Guess if we are booting on BE-M1000 CPU with pre SDK-M 5.3 firmware.
 */
static bool is_old_baikalm(void) {
	struct efi_smbios_type0_record *firmware_record = NULL;
	unsigned fw_major = 0xffU, fw_minor = 0xffU;

	if (!is_baikalm()) {
		return false;
	}

	firmware_record = (struct efi_smbios_type0_record *)efi_get_smbios_record(0);
	if (firmware_record) {
		get_bios_version(&fw_major, &fw_minor, firmware_record);
	} else {
		efi_debug("failed to retrive BIOS version record from SMBIOS\n");
		return true;
	}
	if (fw_major != 0xff && fw_minor != 0xff) {
		efi_info_once("booting on Baikal-M with SDK-M %u.%u\n",
			      fw_major,
			      fw_minor);
		return fw_major <= 4
			/* SDK-M 4.x */
			|| (fw_major == 5 && fw_minor < 3)
			/* SDK-M < 5.3 */;
	} else {
		efi_info("failed to figure out Baikal-M firmware version\n");
		return true; /* assume firmware is broken by default */
	}
	return false;
}

efi_status_t handle_kernel_image(unsigned long *image_addr,
				 unsigned long *image_size,
				 unsigned long *reserve_addr,
				 unsigned long *reserve_size,
				 efi_loaded_image_t *image,
				 efi_handle_t image_handle)
{
	efi_status_t status;
	unsigned long kernel_size, kernel_codesize, kernel_memsize;

	if (image->image_base != _text && !is_baikalm()) {
		efi_err("FIRMWARE BUG: efi_loaded_image_t::image_base has bogus value\n");
		image->image_base = _text;
	}

	if (!IS_ALIGNED((u64)_text, SEGMENT_ALIGN) && !is_baikalm())
		efi_err("FIRMWARE BUG: kernel image not aligned on %dk boundary\n",
			SEGMENT_ALIGN >> 10);

	kernel_size = _edata - _text;
	kernel_codesize = __inittext_end - _text;
	kernel_memsize = kernel_size + (_end - _edata);
	*reserve_size = kernel_memsize;
	*image_addr = (unsigned long)_text;


	if (is_baikalm() && is_old_baikalm()) {
		static const unsigned long BAIKALM_RAM_START = 0x80000000UL;
		static const unsigned long BAIKALM_ARMTF_RAM_END = 0x8FFFFFFFUL;
		u64 min_kimg_align = efi_get_kimg_min_align();
		if ((unsigned long)_end <= BAIKALM_ARMTF_RAM_END &&
				(unsigned long)_text >= BAIKALM_RAM_START &&
				IS_ALIGNED(*image_addr, min_kimg_align)) {
			/* just execute from wherever we were loaded */
			efi_info("Baikal-M: kernel loaded into lower 256 MB, executing from there\n");
			*reserve_size = 0;
			return EFI_SUCCESS;
		}
		efi_info("Baikal-M: kernel loaded outside of lower 256 MB, forcing relocation\n");
		return efi_relocate_kernel(image_addr,
					   kernel_size,
					   kernel_memsize,
					   BAIKALM_RAM_START,
					   min_kimg_align,
					   BAIKALM_RAM_START);
	}

	status = efi_kaslr_relocate_kernel(image_addr,
					   reserve_addr, reserve_size,
					   kernel_size, kernel_codesize,
					   kernel_memsize,
					   efi_kaslr_get_phys_seed(image_handle));
	if (status != EFI_SUCCESS)
		return status;

	return EFI_SUCCESS;
}

asmlinkage void primary_entry(void);

unsigned long primary_entry_offset(void)
{
	/*
	 * When built as part of the kernel, the EFI stub cannot branch to the
	 * kernel proper via the image header, as the PE/COFF header is
	 * strictly not part of the in-memory presentation of the image, only
	 * of the file representation. So instead, we need to jump to the
	 * actual entrypoint in the .text region of the image.
	 */
	return (char *)primary_entry - _text;
}

void efi_icache_sync(unsigned long start, unsigned long end)
{
	caches_clean_inval_pou(start, end);
}
