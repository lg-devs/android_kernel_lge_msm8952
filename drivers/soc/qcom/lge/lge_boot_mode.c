/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/string.h>

#include <soc/qcom/lge/lge_boot_mode.h>

#include <linux/platform_data/lge_android_usb.h>
#include <linux/platform_device.h>
/* get boot mode information from cmdline.
 * If any boot mode is not specified,
 * boot mode is normal type.
 */
static enum lge_boot_mode_type lge_boot_mode = LGE_BOOT_MODE_NORMAL;
static int __init lge_boot_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGER;
	else if (!strcmp(s, "chargerlogo"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGERLOGO;
	else if (!strcmp(s, "qem_56k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_56K;
	else if (!strcmp(s, "qem_130k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_130K;
	else if (!strcmp(s, "qem_910k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_910K;
	else if (!strcmp(s, "pif_56k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_56K;
	else if (!strcmp(s, "pif_130k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_130K;
	else if (!strcmp(s, "pif_910k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_910K;
	else if (!strcmp(s, "miniOS"))
		lge_boot_mode = LGE_BOOT_MODE_MINIOS;
	pr_info("ANDROID BOOT MODE : %d %s\n", lge_boot_mode, s);

	return 1;
}
__setup("androidboot.mode=", lge_boot_mode_init);

enum lge_boot_mode_type lge_get_boot_mode(void)
{
	return lge_boot_mode;
}
#ifdef CONFIG_TIANMA_ILI9488_HVGA_CMD_PANEL
static enum lge_boot_reason_type lge_boot_reason = UNKNOWN_REASON; /*  undefined for error checking */
static int __init lge_check_bootreason(char *reason)
{

	if (!strcmp(reason, "FOTA_Reboot")) {
		lge_boot_reason = FOTA_REBOOT;
#ifdef CONFIG_LGE_DISPLAY_LCD_OFF_DIMMING
	} else if (!strcmp(reason, "FOTA_Reboot_LCDOFF")) {
		lge_boot_reason = FOTA_REBOOT_LCDOFF;
	} else if (!strcmp(reason, "FOTA_Reboot_OUT_LCDOFF")) {
		lge_boot_reason = FOTA_REBOOT_OUT_LCDOFF;
#endif
	} else if (!strcmp(reason, "Recovery_mode")) {
		lge_boot_reason = RECOVERY_MODE;
	}

	if(lge_boot_reason == UNKNOWN_REASON) {
		pr_info("LGE REBOOT REASON: Couldn't get bootreason : %d\n", lge_boot_reason);
	} else {
		pr_info("LGE REBOOT REASON : %d %s\n", lge_boot_mode, reason);
	}

	return 1;
}
__setup("lge.bootreason=", lge_check_bootreason);

enum lge_boot_reason_type lge_get_bootreason(void)
{
	return lge_boot_reason;
}
#endif

int lge_get_factory_boot(void)
{
	int res;

	/*   if boot mode is factory,
	 *   cable must be factory cable.
	 */
	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_QEM_56K:
	case LGE_BOOT_MODE_QEM_130K:
	case LGE_BOOT_MODE_QEM_910K:
	case LGE_BOOT_MODE_PIF_56K:
	case LGE_BOOT_MODE_PIF_130K:
	case LGE_BOOT_MODE_PIF_910K:
	case LGE_BOOT_MODE_MINIOS:
		res = 1;
		break;
	default:
		res = 0;
		break;
	}
	return res;
}

static bool is_mfts_mode = 0;
static int __init lge_mfts_mode_init(char *s)
{
	if(strncmp(s,"1",1) == 0)
		 is_mfts_mode = 1;
	return 0;
}
__setup("mfts.mode=", lge_mfts_mode_init);

bool lge_get_mfts_mode(void)
{
	return is_mfts_mode;
}

static int get_factory_cable(void)
{
	int res = 0;

	/* if boot mode is factory, cable must be factory cable. */
	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_QEM_56K:
	case LGE_BOOT_MODE_PIF_56K:
		res = LGEUSB_FACTORY_56K;
		break;

	case LGE_BOOT_MODE_QEM_130K:
	case LGE_BOOT_MODE_PIF_130K:
		res = LGEUSB_FACTORY_130K;
		break;

	case LGE_BOOT_MODE_QEM_910K:
	case LGE_BOOT_MODE_PIF_910K:
		res = LGEUSB_FACTORY_910K;
		break;

	default:
		res = 0;
		break;
	}

	return res;
}

struct lge_android_usb_platform_data lge_android_usb_pdata = {
	.vendor_id = 0x1004,
	.factory_pid = 0x6000,
	.iSerialNumber = 0,
	.product_name = "LGE Android Phone",
	.manufacturer_name = "LG Electronics Inc.",
	.factory_composition = "acm,diag",
	.get_factory_cable = get_factory_cable,
};

struct platform_device lge_android_usb_device = {
	.name = "lge_android_usb",
	.id = -1,
	.dev = {
		.platform_data = &lge_android_usb_pdata,
	},
};

int __init lge_add_android_usb_devices(void)
{
	return platform_device_register(&lge_android_usb_device);
}
arch_initcall(lge_add_android_usb_devices);

#ifdef CONFIG_LGE_USB_DIAG_LOCK
static struct platform_device lg_diag_cmd_device = {
	.name = "lg_diag_cmd",
	.id = -1,
	.dev    = {
		.platform_data = 0, /* &lg_diag_cmd_pdata */
	},
};

static int __init lge_diag_devices_init(void)
{
	return platform_device_register(&lg_diag_cmd_device);
}
arch_initcall(lge_diag_devices_init);
#endif

#ifdef CONFIG_LGE_USB_EMBEDDED_BATTERY
/*
   for download complete using LAF image
   return value : 1 --> right after laf complete & reset
*/

int android_dlcomplete = 0;

int __init lge_android_dlcomplete(char *s)
{
	if(strncmp(s,"1",1) == 0)
		android_dlcomplete = 1;
	else
		android_dlcomplete = 0;
			printk("androidboot.dlcomplete = %d\n", android_dlcomplete);

	return 1;
}
__setup("androidboot.dlcomplete=", lge_android_dlcomplete);

int lge_get_android_dlcomplete(void)
{
	return android_dlcomplete;
}
#endif
