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

#ifndef OEM_MDSS_DSI_COMMON_H
#define OEM_MDSS_DSI_COMMON_H

#include <linux/list.h>
#include <linux/mdss_io_util.h>
#include <linux/irqreturn.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/of_device.h>
#include <linux/kernel.h>

#include "../../mdss_debug.h"
#include "../../mdss_panel.h"
#include "../../mdss_dsi.h"
#include "../../mdss_fb.h"

struct lge_pan_data {
	int disp_vdd_gpio;
	int disp_io_gpio;
	struct regulator *vreg_disp_vdd;
};

#define MAX_PANEL_ID_LEN 64

struct panel_list {
	char name[MAX_PANEL_ID_LEN];
	uint32_t id;
};

struct lge_mdss_dsi_interface {
	int (*pre_msm_dss_enable_vreg)(struct mdss_panel_data *pdata, int enable);
	int (*post_msm_dss_enable_vreg)(struct mdss_panel_data *pdata, int enable);
	int (*pre_mdss_dsi_panel_reset)(struct mdss_panel_data *pdata, int enable);
	int (*post_mdss_dsi_panel_reset)(struct mdss_panel_data *pdata, int enable);
	int (*pre_mdss_dsi_panel_power_ctrl)(struct mdss_panel_data *pdata, int enable);
	int (*post_mdss_dsi_panel_power_ctrl)(struct mdss_panel_data *pdata, int enable);
	int (*post_mdss_dsi_blank)(struct mdss_panel_data *pdata, int power_state);
	int (*post_mdss_dsi_panel_on)(struct mdss_panel_data *pdata);
	int (*post_mdss_dsi_panel_off)(struct mdss_panel_data *pdata);
	int (*lge_mdss_dsi_event_handler)(struct mdss_panel_data *pdata, int event, void *arg);
	int (*lge_msm_dss_enable_vreg)(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable);
	int (*lge_mdss_dsi_request_gpios)(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	int (*lge_mdss_dsi_panel_reset)(struct mdss_panel_data *pdata, int enable);
	int (*lge_mdss_dsi_lane_config)(struct mdss_panel_data *pdata, int enable);
	int (*lge_mdss_dsi_ctrl_probe)(struct platform_device *pdev);
	int (*lge_dsi_panel_device_register)(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	int (*lge_mdss_panel_parse_dt)(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	int (*lge_panel_device_create)(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	int (*lge_mdss_dsi_cmdlist_commit)(struct mdss_dsi_ctrl_pdata *ctrl, int from_mdp);
	int (*lge_mdss_dsi_panel_init)(struct device_node *node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	int (*lge_dump_mdss_reg)(void);
	int (*lge_mdss_xlog_tout_handler_default)(void);
	int (*lge_mdss_create_xlog_debug)(struct mdss_debug_data *mdd);
	int (*lge_mdss_dsi_panel_bl_ctrl)(struct mdss_panel_data *pdata, u32 bl_level);
};

int pre_msm_dss_enable_vreg(struct mdss_panel_data *pdata, int enable);
int post_msm_dss_enable_vreg(struct mdss_panel_data *pdata, int enable);
int pre_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
int post_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
int pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable);
int post_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable);
int post_mdss_dsi_blank(struct mdss_panel_data *pdata, int power_state);
int post_mdss_dsi_panel_on(struct mdss_panel_data *pdata);
int post_mdss_dsi_panel_off(struct mdss_panel_data *pdata);

int lge_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg);
int lge_msm_dss_enable_vreg(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable);
int lge_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);

int lge_mdss_dsi_lane_config(struct mdss_panel_data *pdata, int enable);
int lge_dsi_panel_device_register(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_mdss_dsi_ctrl_probe(struct platform_device *pdev);
int lge_mdss_panel_parse_dt(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_panel_device_create(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_mdss_dsi_cmdlist_commit(struct mdss_dsi_ctrl_pdata *ctrl, int from_mdp);
int lge_mdss_dsi_panel_init(struct device_node *node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_dump_mdss_reg(void);
int lge_mdss_xlog_tout_handler_default(void);
int lge_mdss_create_xlog_debug(struct mdss_debug_data *mdd);
int lge_mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata, u32 bl_level);

void lge_mdss_dsi_seperate_panel_api_init(struct lge_mdss_dsi_interface *pdata, struct device_node *dsi_pan_node);

int get_lge_panel_id(void);

extern struct mdss_panel_data *pdata_base;
extern struct lge_mdss_dsi_interface lge_mdss_dsi;

#endif /* OEM_MDSS_DSI_COMMON_H */
