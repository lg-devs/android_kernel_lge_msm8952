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

#define INIT_FUNC(x, y) ((x->y) = y)
#include "mdss_debug.h"
#include "oem_mdss_dsi_common.h"
#include "oem_mdss_dsi.h"

static struct panel_list supp_panels[] = {
	{"TOVIS FHD video mode dsi panel", TOVIS_ILI7807B_FHD_VIDEO_PANEL},
	{"SHARP FHD video mode dsi panel", SHARP_NT35596_FHD_VIDEO_PANEL},
	{"innolux NT51021 wuxga video mode panel", INNOLUX_NT51021_WUXGA_VIDEO_PANEL},
	{"TOVIS NT51021 wuxga video mode panel", TOVIS_NT51021_WUXGA_VIDEO_PANEL},
};

static int panel_id;

struct lge_mdss_dsi_interface panel_init_fnc = {
	pre_msm_dss_enable_vreg,
	post_msm_dss_enable_vreg,
	pre_mdss_dsi_panel_reset,
	post_mdss_dsi_panel_reset,
	pre_mdss_dsi_panel_power_ctrl,
	post_mdss_dsi_panel_power_ctrl,
	post_mdss_dsi_blank,
	post_mdss_dsi_panel_on,
	post_mdss_dsi_panel_off,
	lge_mdss_dsi_event_handler,
	lge_msm_dss_enable_vreg,
	lge_mdss_dsi_request_gpios,
	lge_mdss_dsi_panel_reset,
	lge_mdss_dsi_lane_config,
	lge_mdss_dsi_ctrl_probe,
	lge_dsi_panel_device_register,
	lge_mdss_panel_parse_dt,
	lge_panel_device_create,
	lge_mdss_dsi_cmdlist_commit,
	lge_mdss_dsi_panel_init,
	lge_dump_mdss_reg,
	lge_mdss_xlog_tout_handler_default,
	lge_mdss_create_xlog_debug,
	lge_mdss_dsi_panel_bl_ctrl
};

int panel_name_to_id(struct panel_list supp_panels[],
			  uint32_t supp_panels_size,
			  const char *panel_name)
{
	int i;
	int panel_id = UNKNOWN_PANEL;

	if (!panel_name) {
		pr_err("Invalid panel name\n");
		return panel_id;
	}

	/* Remove any leading whitespaces */
	panel_name += strspn(panel_name, " ");
	for (i = 0; i < supp_panels_size; i++) {
		if (!strncmp(panel_name, supp_panels[i].name,
			MAX_PANEL_ID_LEN)) {
			panel_id = supp_panels[i].id;
			break;
		}
	}

	return panel_id;
}

int pre_msm_dss_enable_vreg(struct mdss_panel_data *pdata, int enable)
{
	return 0;
}

int post_msm_dss_enable_vreg(struct mdss_panel_data *pdata, int enable)
{
	return 0;
}

int pre_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	return 0;
}

int post_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	return 0;
}

int pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	return 0;
}

int post_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	return 0;
}

int post_mdss_dsi_blank(struct mdss_panel_data *pdata, int power_state)
{
	return 0;
}

int post_mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	return 0;
}

int post_mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	return 0;
}

int lge_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	return 0;
}


int lge_msm_dss_enable_vreg(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{
	return 0;
}

int lge_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	return 0;
}

int lge_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	return 0;
}

int lge_mdss_dsi_lane_config(struct mdss_panel_data *pdata, int enable)
{
	return 0;
}

int lge_mdss_dsi_ctrl_probe(struct platform_device *pdev)
{
	return 0;
}

int lge_dsi_panel_device_register(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	return 0;
}

int lge_mdss_panel_parse_dt(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	return 0;
}

int lge_panel_device_create(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	return 0;
}

int lge_mdss_dsi_cmdlist_commit(struct mdss_dsi_ctrl_pdata *ctrl, int from_mdp)
{
	return 0;
}

int lge_mdss_dsi_panel_init(struct device_node *node, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;
	switch (panel_id) {
	case TOVIS_ILI7807B_FHD_VIDEO_PANEL:
		rc = tovis_fhd_video_mdss_dsi_panel_init(node, ctrl_pdata);
		break;
	case SHARP_NT35596_FHD_VIDEO_PANEL:
		rc = sharp_fhd_video_mdss_dsi_panel_init(node, ctrl_pdata);
		break;
	case INNOLUX_NT51021_WUXGA_VIDEO_PANEL:
		rc = innolux_wuxga_video_mdss_dsi_panel_init(node, ctrl_pdata);
		break;
	case TOVIS_NT51021_WUXGA_VIDEO_PANEL:
		rc = tovis_wuxga_video_mdss_dsi_panel_init(node, ctrl_pdata);
		break;
	default:
		break;
	}

	return rc;
}
int lge_dump_mdss_reg(void)
{
	return 0;
}

int lge_mdss_xlog_tout_handler_default(void)
{
	return 0;
}

int lge_mdss_create_xlog_debug(struct mdss_debug_data *mdd)
{
	return 0;
}

int lge_mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata, u32 bl_level)
{
	return 0;
}

int get_lge_panel_id(void)
{
	return panel_id;
}

void lge_mdss_dsi_seperate_panel_api_init(struct lge_mdss_dsi_interface *pdata, struct device_node *dsi_pan_node)
{
	static const char *panel_name;
	u32 *panel_power_sequence;
	int rc, i, num, fnum;
	struct property *data;

	panel_name = of_get_property(dsi_pan_node, "qcom,mdss-dsi-panel-name", NULL);

	panel_id = panel_name_to_id(supp_panels,
			ARRAY_SIZE(supp_panels), panel_name);

	data = of_find_property(dsi_pan_node, "lge,panel_power_sequence", &num);

	if (!data || !num) {
		pr_err("%s:%d, error reading lge,panel_power_sequence, length found = %d\n",
			__func__, __LINE__, num);
	} else {
		num /= sizeof(u32);
		fnum = sizeof(struct lge_mdss_dsi_interface)/sizeof(int *);
		if (num > fnum) {
			pr_err("%s: dtsi#:%d, header#:%d\n", __func__, num, fnum);
			num = fnum;
		}

		panel_power_sequence = kzalloc(sizeof(u32) * num, GFP_KERNEL);
		if (!panel_power_sequence) {
			pr_err("%s: Not enough memory\n", __func__);
			return;
		}

		rc = of_property_read_u32_array(dsi_pan_node, "lge,panel_power_sequence",
				panel_power_sequence, num);

		if (rc) {
			pr_err("%s: unable to read panel_power_sequence\n", __func__);
			kfree(panel_power_sequence);
			return;
		}

		for (i = 0; i < num; i++) {
			if (panel_power_sequence[i])
				memcpy(&pdata->pre_msm_dss_enable_vreg+i,
						&panel_init_fnc.pre_msm_dss_enable_vreg+i, sizeof(int *));
		}
		kfree(panel_power_sequence);
	}
}
