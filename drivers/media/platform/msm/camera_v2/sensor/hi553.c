/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#include "msm_sensor.h"
#define hi553_SENSOR_NAME "hi553"
DEFINE_MSM_MUTEX(hi553_mut);

static struct msm_sensor_ctrl_t hi553_s_ctrl;

static struct msm_sensor_power_setting hi553_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_LDO_EN,
		.config_val = GPIO_OUT_HIGH,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info hi553_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SGBRG10_1X10,  //need check
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id hi553_i2c_id[] = {
	{hi553_SENSOR_NAME, (kernel_ulong_t)&hi553_s_ctrl},
	{ }
};

static int32_t msm_hi553_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hi553_s_ctrl);
}

static struct i2c_driver hi553_i2c_driver = {
	.id_table = hi553_i2c_id,
	.probe  = msm_hi553_i2c_probe,
	.driver = {
		.name = hi553_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hi553_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id hi553_dt_match[] = {
	{.compatible = "qcom,hi553", .data = &hi553_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hi553_dt_match);

static struct platform_driver hi553_platform_driver = {
	.driver = {
		.name = "qcom,hi553",
		.owner = THIS_MODULE,
		.of_match_table = hi553_dt_match,
	},
};

static int32_t hi553_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	pr_err("%s: E\n", __func__);
	match = of_match_device(hi553_dt_match, &pdev->dev);
	if(!match)
	{
		  pr_err(" %s failed ",__func__);
		  return -ENODEV;
	 }
	rc = msm_sensor_platform_probe(pdev, match->data);
	pr_err("%s: X\n", __func__);
	return rc;
}

static int __init hi553_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);

	rc = platform_driver_probe(&hi553_platform_driver,
		hi553_platform_probe);
	if (!rc)
		return rc;
	pr_info("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&hi553_i2c_driver);
}

static void __exit hi553_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hi553_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hi553_s_ctrl);
		platform_driver_unregister(&hi553_platform_driver);
	} else
		i2c_del_driver(&hi553_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t hi553_s_ctrl = {
	.sensor_i2c_client = &hi553_sensor_i2c_client,
	.power_setting_array.power_setting = hi553_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hi553_power_setting),
#if 0	
	/*LGE add for sequence of power down */
	.power_setting_array.power_down_setting= hi553_power_down_setting,
	.power_setting_array.size_down = ARRAY_SIZE(hi553_power_down_setting),
#endif
	/*LGE add for sequence of power down */
	.msm_sensor_mutex = &hi553_mut,
	.sensor_v4l2_subdev_info = hi553_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hi553_subdev_info),
};

module_init(hi553_init_module);
module_exit(hi553_exit_module);
MODULE_DESCRIPTION("hi553");
MODULE_LICENSE("GPL v2");
