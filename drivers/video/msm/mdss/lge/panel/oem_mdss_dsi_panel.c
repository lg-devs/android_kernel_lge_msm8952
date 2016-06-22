/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/leds.h>

#include "oem_mdss_dsi_common.h"
#include "oem_mdss_dsi.h"
#include <soc/qcom/lge/lge_display_panel_check.h>

int tovis_fhd_video_mdss_dsi_panel_init(struct device_node *node, \
		struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mdss_panel_info *pinfo;

	pinfo = &ctrl_pdata->panel_data.panel_info;

	if (strncmp(pinfo->panel_name, "TOVIS FHD video", 15) == 0) {
		pr_info("%s: panel_type is TOVIS_FHD_VIDEO_PANEL\n", __func__);
		pinfo->lge_pan_info.panel_type = TOVIS_FHD_VIDEO_PANEL;
	}
	lge_set_panel(pinfo->lge_pan_info.panel_type);

	return 0;
}

int sharp_fhd_video_mdss_dsi_panel_init(struct device_node *node, \
		struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mdss_panel_info *pinfo;

	pinfo = &ctrl_pdata->panel_data.panel_info;

	if (strncmp(pinfo->panel_name, "SHARP FHD video", 15) == 0) {
		pr_info("%s: panel_type is SHARP_FHD_VIDEO_PANEL\n", __func__);
		pinfo->lge_pan_info.panel_type = SHARP_FHD_VIDEO_PANEL;
	}
	lge_set_panel(pinfo->lge_pan_info.panel_type);

	return 0;
}

int innolux_wuxga_video_mdss_dsi_panel_init(struct device_node *node, \
		struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mdss_panel_info *pinfo;

	pinfo = &ctrl_pdata->panel_data.panel_info;

	if (strncmp(pinfo->panel_name, "innolux NT51021 wuxga video mode panel", 38) == 0) {
		pr_info("%s: panel_type is INNOLUX_WUXGA_VIDEO_PANEL\n", __func__);
		pinfo->lge_pan_info.panel_type = INNOLUX_WUXGA_VIDEO_PANEL;
	}
	lge_set_panel(pinfo->lge_pan_info.panel_type);

	return 0;
}

int tovis_wuxga_video_mdss_dsi_panel_init(struct device_node *node, \
		struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mdss_panel_info *pinfo;

	pinfo = &ctrl_pdata->panel_data.panel_info;

	if (strncmp(pinfo->panel_name, "TOVIS NT51021 wuxga video mode panel", 36) == 0) {
		pr_info("%s: panel_type is TOVIS_WUXGA_VIDEO_PANEL\n", __func__);
		pinfo->lge_pan_info.panel_type = TOVIS_WUXGA_VIDEO_PANEL;
	}
	lge_set_panel(pinfo->lge_pan_info.panel_type);

	return 0;
}
