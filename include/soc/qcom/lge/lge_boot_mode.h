#ifndef __LGE_BOOT_MODE_H
#define __LGE_BOOT_MODE_H

enum lge_boot_mode_type {
	LGE_BOOT_MODE_NORMAL = 0,
	LGE_BOOT_MODE_CHARGER,
	LGE_BOOT_MODE_CHARGERLOGO,
	LGE_BOOT_MODE_QEM_56K,
	LGE_BOOT_MODE_QEM_130K,
	LGE_BOOT_MODE_QEM_910K,
	LGE_BOOT_MODE_PIF_56K,
	LGE_BOOT_MODE_PIF_130K,
	LGE_BOOT_MODE_PIF_910K,
	LGE_BOOT_MODE_MINIOS    /* LGE_UPDATE for MINIOS2.0 */
};
enum lge_boot_mode_type lge_get_boot_mode(void);
int lge_get_factory_boot(void);

#ifdef CONFIG_TIANMA_ILI9488_HVGA_CMD_PANEL
enum lge_boot_reason_type {
	FOTA_REBOOT = 0,
#ifdef CONFIG_LGE_DISPLAY_LCD_OFF_DIMMING
	FOTA_REBOOT_LCDOFF,
	FOTA_REBOOT_OUT_LCDOFF,
#endif
	RECOVERY_MODE,
	UNKNOWN_REASON
};
enum lge_boot_reason_type lge_get_bootreason(void);
#endif
#ifdef CONFIG_LGE_USB_EMBEDDED_BATTERY
int lge_get_android_dlcomplete(void);
#endif
#ifdef CONFIG_LGE_USB_G_ANDROID
int __init lge_add_android_usb_devices(void);
#endif
#endif
