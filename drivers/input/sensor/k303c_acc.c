/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
 *
 * File Name          : lsm303c_acc.c
 * Authors            : AMS - Motion Mems Division - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Denis Ciocca (denis.ciocca@st.com)
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.6_ST
 * Date               : 2014/Jun/18
 * Description        : LSM303C accelerometer driver
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/uaccess.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>
#include	<linux/kernel.h>
#include	<linux/device.h>
#include	<linux/module.h>
#include	<linux/moduleparam.h>

#include <linux/regulator/consumer.h>

#include	"k303c.h"

#define CALIBRATION_TO_FILE

#ifdef CALIBRATION_TO_FILE
#include <linux/syscalls.h>
#include <linux/fs.h>
#define CALIBRATION_PATH "/persist/sns/sensor_cal_data.txt"
#define K303B_SUCCESS						0
#define K303B_ERR_I2C						-1
#define K303B_ERR_STATUS					-3
#define K303B_ERR_SETUP_FAILURE			-4
#define K303B_ERR_GETGSENSORDATA			-5
#define K303B_ERR_IDENTIFICATION			-6
#endif /* CALIBRATION_TO_FILE */

#define G_MAX			(7995148) /* (SENSITIVITY_8G * ((2^15)-1)) */
#define G_MIN			(-7995392) /* (-SENSITIVITY_8G * (2^15)) */
#define FUZZ			0
#define FLAT			0
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5
#define I2C_AUTO_INCREMENT	(0x00)

#define MS_TO_NS(x)		(x * 1000000L)

#define SENSITIVITY_2G		 61	/**	ug/LSB	*/
#define SENSITIVITY_4G		122	/**	ug/LSB	*/
#define SENSITIVITY_8G		244	/**	ug/LSB	*/


/* Accelerometer Sensor Operating Mode */
#define LSM303C_ACC_ENABLE	(0x01)
#define LSM303C_ACC_DISABLE	(0x00)

#define AXISDATA_REG		(0x28)
#define WHOAMI_LSM303C_ACC	(0x41)	/*	Expctd content for WAI	*/
#define ALL_ZEROES		(0x00)
#define LSM303C_ACC_PM_OFF	(0x00)
#define ACC_ENABLE_ALL_AXES	(0x07)

/*	CONTROL REGISTERS	*/
#define TEMP_L			(0x0B)
#define TEMP_H			(0x0C)
#define WHO_AM_I		(0x0F)	/*	WhoAmI register		*/
#define ACT_THS			(0x1E)	/*	Activity Threshold	*/
#define ACT_DUR			(0x1F)	/*	Activity Duration	*/
/* ctrl 1: HR ODR2 ODR1 ODR0 BDU Zenable Yenable Xenable */
#define CTRL1			(0x20)	/*	control reg 1		*/
#define CTRL2			(0x21)	/*	control reg 2		*/
#define CTRL3			(0x22)	/*	control reg 3		*/
#define CTRL4			(0x23)	/*	control reg 4		*/
#define CTRL5			(0x24)	/*	control reg 5		*/
#define CTRL6			(0x25)	/*	control reg 6		*/
#define CTRL7			(0x26)	/*	control reg 7		*/

#define FIFO_CTRL		(0x2E)	/*	fifo control reg	*/

#define INT_CFG1		(0x30)	/*	interrupt 1 config	*/
#define INT_SRC1		(0x31)	/*	interrupt 1 source	*/
#define INT_THSX1		(0x32)	/*	interrupt 1 threshold x	*/
#define INT_THSY1		(0x33)	/*	interrupt 1 threshold y	*/
#define INT_THSZ1		(0x34)	/*	interrupt 1 threshold z	*/
#define INT_DUR1		(0x35)	/*	interrupt 1 duration	*/

#define INT_CFG2		(0x36)	/*	interrupt 2 config	*/
#define INT_SRC2		(0x37)	/*	interrupt 2 source	*/
#define INT_THS2		(0x38)	/*	interrupt 2 threshold	*/
#define INT_DUR2		(0x39)	/*	interrupt 2 duration	*/

#define REF_XL			(0x3A)	/*	reference_l_x		*/
#define REF_XH			(0x3B)	/*	reference_h_x		*/
#define REF_YL			(0x3C)	/*	reference_l_y		*/
#define REF_YH			(0x3D)	/*	reference_h_y		*/
#define REF_ZL			(0x3E)	/*	reference_l_z		*/
#define REF_ZH			(0x3F)	/*	reference_h_z		*/
/*	end CONTROL REGISTRES	*/



#define ACC_ODR10		(0x10)	/*   10Hz output data rate */
#define ACC_ODR50		(0x20)	/*   50Hz output data rate */
#define ACC_ODR100		(0x30)	/*  100Hz output data rate */
#define ACC_ODR200		(0x40)	/*  200Hz output data rate */
#define ACC_ODR400		(0x50)	/*  400Hz output data rate */
#define ACC_ODR800		(0x60)	/*  800Hz output data rate */
#define ACC_ODR_MASK		(0X70)

/* Registers configuration Mask and settings */
/* CTRL1 */
#define CTRL1_HR_DISABLE	(0x00)
#define CTRL1_HR_ENABLE		(0x80)
#define CTRL1_HR_MASK		(0x80)
#define CTRL1_BDU_ENABLE	(0x08)
#define CTRL1_BDU_MASK		(0x08)

/* CTRL2 */
#define CTRL2_IG1_INT1		(0x08)

/* CTRL3 */
#define CTRL3_IG1_INT1		(0x08)
#define CTRL3_DRDY_INT1

/* CTRL4 */
#define CTRL4_IF_ADD_INC_EN	(0x04)
#define CTRL4_BW_SCALE_ODR_AUT	(0x00)
#define CTRL4_BW_SCALE_ODR_SEL	(0x08)
#define CTRL4_ANTALIAS_BW_400	(0x00)
#define CTRL4_ANTALIAS_BW_200	(0x40)
#define CTRL4_ANTALIAS_BW_100	(0x80)
#define CTRL4_ANTALIAS_BW_50	(0xC0)
#define CTRL4_ANTALIAS_BW_MASK	(0xC0)

/* CTRL5 */
#define CTRL5_HLACTIVE_L	(0x02)
#define CTRL5_HLACTIVE_H	(0x00)

/* CTRL6 */
#define CTRL6_IG2_INT2		(0x10)
#define CTRL6_DRDY_INT2		(0x01)

/* CTRL7 */
#define CTRL7_LIR2		(0x08)
#define CTRL7_LIR1		(0x04)
/* */

#define NO_MASK			(0xFF)

#define INT1_DURATION_MASK	(0x7F)
#define INT1_THRESHOLD_MASK	(0x7F)



/* RESUME STATE INDICES */
#define RES_CTRL1		0
#define RES_CTRL2		1
#define RES_CTRL3		2
#define RES_CTRL4		3
#define RES_CTRL5		4
#define RES_CTRL6		5
#define RES_CTRL7		6

#define RES_INT_CFG1		7
#define RES_INT_THSX1		8
#define RES_INT_THSY1		9
#define RES_INT_THSZ1		10
#define RES_INT_DUR1		11


#define RES_INT_CFG2		12
#define RES_INT_THS2		13
#define RES_INT_DUR2		14

#define RES_TEMP_CFG_REG	15
#define RES_REFERENCE_REG	16
#define RES_FIFO_CTRL	17

#define RESUME_ENTRIES		18
/* end RESUME STATE INDICES */

#define OUTPUT_ALWAYS_ANTI_ALIASED 1

/*
	Self-Test threhold for 16bit/4G output
*/
#define	LSM303C_SHAKING_DETECT_THRESHOLD 1597/* 200->0.195g X 8192 LSB/g */

#define	TESTLIMIT_XY			(1475)/* 8192 LSB/g X 0.180g */
#define	TESTLIMIT_Z_USL_LSB		(10240)/* 8192 LSB + 8192 LSB/g X 0.250g */
#define	TESTLIMIT_Z_LSL_LSB		(6144)/* 8192 LSB - 8192 LSB/g X 0.250g */

#define HWST_LSL_LSB			(573)/* 8192LSB * 0.07g */
#define HWST_USL_LSB			(12288)/* 8192LSB * 1.5g */

#define	CALIBRATION_DATA_AMOUNT	10

#if 1 /* LGE_SENSOR_FACTORY_AAT */
static int cal_result; // 0: FAIL 1: PASS
#endif

#ifdef CALIBRATION_TO_FILE
#define OFFSET_APPLIED 1
static int cal_file_status;
static int k303b_calibration_read(int *cal_read);
static int k303b_calibration_save(int *cal);
#endif /* CALIBRATION_TO_FILE */

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lsm303c_acc_odr_table[] = {
		{    2, ACC_ODR800 },
		{    3, ACC_ODR400  },
		{    5, ACC_ODR200  },
		{   10, ACC_ODR100  },
#if (!OUTPUT_ALWAYS_ANTI_ALIASED)
		{   20, ACC_ODR50   },
		{  100, ACC_ODR10   },
#endif
};

static int int1_gpio = LSM303C_ACC_DEFAULT_INT1_GPIO;
module_param(int1_gpio, int, S_IRUGO);

struct lsm303c_acc_status {
	struct i2c_client *client;
	struct lsm303c_acc_platform_data *pdata;

	struct mutex lock;
	struct work_struct input_poll_work;
	struct hrtimer hr_timer_poll;
	ktime_t polling_ktime;
	struct workqueue_struct *hr_timer_poll_work_queue;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	int use_smbus;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

#ifdef DEBUG
	u8 reg_addr;
#endif
	atomic_t fast_calib_rslt;
	int cali_sw[3];
	struct regulator *vdd_reg;
	struct regulator *vdd_i2c;

};

static struct lsm303c_acc_platform_data default_lsm303c_acc_pdata = {
	.fs_range = LSM303C_ACC_FS_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 1,
	.negate_z = 0,
	.poll_interval = 100,
	.min_interval = LSM303C_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LSM303C_ACC_DEFAULT_INT1_GPIO,
};

/* sets default init values to be written in registers at probe stage */
static void lsm303c_acc_set_init_register_values(
						struct lsm303c_acc_status *stat)
{
	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CTRL1] = (ALL_ZEROES | CTRL1_HR_DISABLE | CTRL1_BDU_ENABLE | ACC_ENABLE_ALL_AXES);

	if (stat->pdata->gpio_int1 >= 0)
		stat->resume_state[RES_CTRL3] = (stat->resume_state[RES_CTRL3] | CTRL3_IG1_INT1);

	stat->resume_state[RES_CTRL4] = (ALL_ZEROES | CTRL4_IF_ADD_INC_EN);

	stat->resume_state[RES_CTRL5] = (ALL_ZEROES | CTRL5_HLACTIVE_H);

	stat->resume_state[RES_CTRL7] = (ALL_ZEROES | CTRL7_LIR2 | CTRL7_LIR1);

}

static int lsm303c_acc_i2c_read(struct lsm303c_acc_status *stat, u8 *buf, int len)
{
	int ret;
	u8 reg = buf[0];
	u8 cmd = reg;

	if (len > 1)
		cmd = (I2C_AUTO_INCREMENT | reg);
	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(stat->client, cmd);
			buf[0] = ret & 0xff;
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_byte_data: ret=0x%02x, len:%d , command=0x%02x, buf[0]=0x%02x\n",
				ret, len, cmd , buf[0]);
#endif
		} else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(stat->client, cmd, len, buf);
		} else
			ret = -1;

		if (ret < 0) {
			dev_err(&stat->client->dev, "read transfer error: len:%d, command=0x%02x\n", len, cmd);
			return 0; /* failure */
		}
		return len; /* success */
	}

	ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(stat->client, buf, len);
}

static int lsm303c_acc_i2c_write(struct lsm303c_acc_status *stat, u8 *buf, int len)
{
	int ret;
	u8 reg, value;

	if (len > 1)
		buf[0] = (I2C_AUTO_INCREMENT | buf[0]);

	reg = buf[0];
	value = buf[1];

	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(stat->client, reg, value);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_byte_data: ret=%d, len:%d, command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);
#endif
			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(stat->client, reg, len, buf + 1);
			return ret;
		}
	}

	ret = i2c_master_send(stat->client, buf, len+1);
	return (ret == len+1) ? 0 : ret;
}

static int lsm303c_acc_hw_init(struct lsm303c_acc_status *stat)
{
	int err = -1;
	u8 buf[7];

	pr_info("%s: hw init start\n", LSM303C_ACC_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = lsm303c_acc_i2c_read(stat, buf, 1);
	if (err < 0) {
		dev_warn(&stat->client->dev, "Error reading WHO_AM_I: is device available/working?\n");
		goto err_firstread;
	} else
		stat->hw_working = 1;

	if (buf[0] != WHOAMI_LSM303C_ACC) {
		dev_err(&stat->client->dev,
			"device unknown. Expected: 0x%02x, Replies: 0x%02x\n", WHOAMI_LSM303C_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = CTRL4;
	buf[1] = stat->resume_state[RES_CTRL4];
	err = lsm303c_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = FIFO_CTRL;
	buf[1] = stat->resume_state[RES_FIFO_CTRL];
	err = lsm303c_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = INT_THSX1;
	buf[1] = stat->resume_state[RES_INT_THSX1];
	buf[2] = stat->resume_state[RES_INT_THSY1];
	buf[3] = stat->resume_state[RES_INT_THSZ1];
	buf[4] = stat->resume_state[RES_INT_DUR1];
	err = lsm303c_acc_i2c_write(stat, buf, 4);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG1;
	buf[1] = stat->resume_state[RES_INT_CFG1];
	err = lsm303c_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;


	buf[0] = CTRL2;
	buf[1] = stat->resume_state[RES_CTRL2];
	buf[2] = stat->resume_state[RES_CTRL3];
	err = lsm303c_acc_i2c_write(stat, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = CTRL5;
	buf[1] = stat->resume_state[RES_CTRL5];
	buf[2] = stat->resume_state[RES_CTRL6];
	buf[3] = stat->resume_state[RES_CTRL7];
	err = lsm303c_acc_i2c_write(stat, buf, 3);
	if (err < 0)
		goto err_resume_state;

	buf[0] = CTRL1;
	buf[1] = stat->resume_state[RES_CTRL1];
	err = lsm303c_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	stat->hw_initialized = 1;
	pr_info("%s: hw init done\n", LSM303C_ACC_DEV_NAME);
	return 0;

err_firstread:
	stat->hw_working = 0;
err_unknown_device:
err_resume_state:
	stat->hw_initialized = 0;
	dev_err(&stat->client->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}

static void lsm303c_acc_device_power_off(struct lsm303c_acc_status *stat)
{
	int err;
	u8 buf[2] = { CTRL1, LSM303C_ACC_PM_OFF };

	err = lsm303c_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "soft power off failed: %d\n", err);

	if (stat->pdata->power_off) {
		if (stat->pdata->gpio_int1 >= 0)
			disable_irq_nosync(stat->irq1);

		stat->pdata->power_off();
		stat->hw_initialized = 0;
	}
	if (stat->hw_initialized) {
		if (stat->pdata->gpio_int1 >= 0)
			disable_irq_nosync(stat->irq1);

		stat->hw_initialized = 0;
	}

}

static int lsm303c_acc_device_power_on(struct lsm303c_acc_status *stat)
{
	int err = -1;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0) {
			dev_err(&stat->client->dev, "power_on failed: %d\n", err);
			return err;
		}
		if (stat->pdata->gpio_int1 >= 0)
			enable_irq(stat->irq1);
	}

	if (!stat->hw_initialized) {
		err = lsm303c_acc_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			lsm303c_acc_device_power_off(stat);
			return err;
		}
	}

	if (stat->hw_initialized) {
		if (stat->pdata->gpio_int1 >= 0)
			enable_irq(stat->irq1);
	}
	return 0;
}


static int lsm303c_acc_update_fs_range(struct lsm303c_acc_status *stat, u8 new_fs_range)
{
	int err = -1;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LSM303C_ACC_FS_MASK;

	switch (new_fs_range) {
	case LSM303C_ACC_FS_2G:
		sensitivity = SENSITIVITY_2G;
		break;
	case LSM303C_ACC_FS_4G:
		sensitivity = SENSITIVITY_4G;
		break;
	case LSM303C_ACC_FS_8G:
		sensitivity = SENSITIVITY_8G;
		break;
	default:
		dev_err(&stat->client->dev, "invalid fs range requested: %u\n", new_fs_range);
		return -EINVAL;
	}


	/* Updates configuration register 4,
	* which contains fs range setting */
	buf[0] = CTRL4;
	err = lsm303c_acc_i2c_read(stat, buf, 1);
	if (err < 0)
		goto error;
	init_val = buf[0];
	stat->resume_state[RES_CTRL4] = init_val;
	new_val = new_fs_range;
	updated_val = ((mask & new_val) | ((~mask) & init_val));
	buf[1] = updated_val;
	buf[0] = CTRL4;
	err = lsm303c_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto error;
	stat->resume_state[RES_CTRL4] = updated_val;
	stat->sensitivity = sensitivity;

	return err;
error:
	dev_err(&stat->client->dev, "update fs range failed 0x%02x,0x%02x: %d\n", buf[0], buf[1], err);

	return err;
}

static int lsm303c_acc_update_odr(struct lsm303c_acc_status *stat, int poll_interval_ms)
{
	int err;
	int i;
	u8 config[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = ACC_ODR_MASK;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lsm303c_acc_odr_table) - 1; i >= 0; i--) {
		if ((lsm303c_acc_odr_table[i].cutoff_ms <= poll_interval_ms) || (i == 0))
			break;
	}
	new_val = lsm303c_acc_odr_table[i].mask;

	/* Updates configuration register 1,
	* which contains odr range setting if enabled,
	* otherwise updates RES_CTRL1 for when it will */
	if (atomic_read(&stat->enabled)) {
		config[0] = CTRL1;
		err = lsm303c_acc_i2c_read(stat, config, 1);
		if (err < 0)
			goto error;
		init_val = config[0];
		stat->resume_state[RES_CTRL1] = init_val;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		config[1] = updated_val;
		config[0] = CTRL1;
		err = lsm303c_acc_i2c_write(stat, config, 1);
		if (err < 0)
			goto error;
		stat->resume_state[RES_CTRL1] = updated_val;
		return err;
	} else {
		init_val = stat->resume_state[RES_CTRL1];
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		stat->resume_state[RES_CTRL1] = updated_val;
		return 0;
	}

error:
	dev_err(&stat->client->dev, "update odr failed 0x%02x,0x%02x: %d\n", config[0], config[1], err);
	return err;
}



static int lsm303c_acc_register_write(struct lsm303c_acc_status *stat, u8 *buf, u8 reg_address, u8 new_value)
{
	int err = -1;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	buf[0] = reg_address;
	buf[1] = new_value;
	err = lsm303c_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		return err;
	return err;
}


static int lsm303c_acc_get_lsb(struct lsm303c_acc_status *stat, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s32 hw_d[3] = { 0 };

	acc_data[0] = (AXISDATA_REG);
	err = lsm303c_acc_i2c_read(stat, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (s16)((acc_data[1] << 8) | acc_data[0]);
	hw_d[1] = (s16)((acc_data[3] << 8) | acc_data[2]);
	hw_d[2] = (s16)((acc_data[5] << 8) | acc_data[4]);
#ifdef DEBUG
	netdev_warn(KERN_WARNING "k303c_acc1 = %2x,%2x,%2x,%2x,%2x,%2x\n",
		acc_data[0], acc_data[1], acc_data[2], acc_data[3], acc_data[4], acc_data[5]);
#endif
	xyz[0] = hw_d[0];
	xyz[1] = hw_d[1];
	xyz[2] = hw_d[2];

	return err;
}


static int lsm303c_acc_get_data(struct lsm303c_acc_status *stat, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s32 hw_d[3] = { 0 };

	acc_data[0] = (AXISDATA_REG);
	err = lsm303c_acc_i2c_read(stat, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (s16)((acc_data[1] << 8) | acc_data[0]) - stat->cali_sw[0];
	hw_d[1] = (s16)((acc_data[3] << 8) | acc_data[2]) - stat->cali_sw[1];
	hw_d[2] = (s16)((acc_data[5] << 8) | acc_data[4]) - stat->cali_sw[2];
#ifdef DEBUG
	netdev_warn(KERN_WARNING "k303c_acc1 = %2x,%2x,%2x,%2x,%2x,%2x\n",
		acc_data[0], acc_data[1], acc_data[2], acc_data[3], acc_data[4], acc_data[5]);
#endif

	hw_d[0] = hw_d[0] * stat->sensitivity;
	hw_d[1] = hw_d[1] * stat->sensitivity;
	hw_d[2] = hw_d[2] * stat->sensitivity;

	xyz[0] = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x]) : (hw_d[stat->pdata->axis_map_x]));
	xyz[1] = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y]) : (hw_d[stat->pdata->axis_map_y]));
	xyz[2] = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z]) : (hw_d[stat->pdata->axis_map_z]));

	return err;
}

static void lsm303c_acc_report_values(struct lsm303c_acc_status *stat, int *xyz)
{
	input_report_abs(stat->input_dev, ABS_X, xyz[0]);
	input_report_abs(stat->input_dev, ABS_Y, xyz[1]);
	input_report_abs(stat->input_dev, ABS_Z, xyz[2]);
	input_sync(stat->input_dev);
}

static void lsm303c_acc_report_triple(struct lsm303c_acc_status *stat)
{
	int err;
	int xyz[3];

	err = lsm303c_acc_get_data(stat, xyz);
	if (err < 0)
		dev_err(&stat->client->dev, "get_data failed\n");
	else
		lsm303c_acc_report_values(stat, xyz);
}

static irqreturn_t lsm303c_acc_isr1(int irq, void *dev)
{
	struct lsm303c_acc_status *stat = dev;

	disable_irq_nosync(irq);
	queue_work(stat->irq1_work_queue, &stat->irq1_work);
	pr_debug("%s: isr1 queued\n", LSM303C_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void lsm303c_acc_irq1_work_func(struct work_struct *work)
{
	struct lsm303c_acc_status *stat = container_of(work, struct lsm303c_acc_status, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm303c_acc_get_int1_source(stat); */
	/* ; */
	pr_debug("%s: IRQ1 served\n", LSM303C_ACC_DEV_NAME);
/* exit: */
	enable_irq(stat->irq1);
}

static int lsm303c_acc_enable(struct lsm303c_acc_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = lsm303c_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
		stat->polling_ktime = ktime_set(stat->pdata->poll_interval / 1000,
				MS_TO_NS(stat->pdata->poll_interval % 1000));
		hrtimer_start(&stat->hr_timer_poll,
					stat->polling_ktime, HRTIMER_MODE_REL);
	}
	return 0;
}

static int lsm303c_acc_disable(struct lsm303c_acc_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		cancel_work_sync(&stat->input_poll_work);
		lsm303c_acc_device_power_off(stat);
	}

	return 0;
}


static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	int err;

	u8 data = reg;
	err = lsm303c_acc_i2c_read(stat, &data, 1);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02x\n", data);
}

static int write_reg(struct device *dev, const char *buf, u8 reg, u8 mask, int resume_index)
{
	int err = -1;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lsm303c_acc_register_write(stat, x, reg, new_val);
	if (err < 0)
		return err;
	stat->resume_state[resume_index] = new_val;
	return err;
}

static ssize_t attr_get_polling_rate(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val;

	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max_t(unsigned int, (unsigned int)interval_ms, stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	err = lsm303c_acc_update_odr(stat, interval_ms);
	if (err >= 0) {
		stat->pdata->poll_interval = interval_ms;
		stat->polling_ktime = ktime_set(stat->pdata->poll_interval / 1000,
				MS_TO_NS(stat->pdata->poll_interval % 1000));
	}
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev, struct device_attribute *attr, char *buf)
{
	char val;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	char range = 2;

	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range;

	switch (val) {
	case LSM303C_ACC_FS_2G:
		range = 2;
		break;
	case LSM303C_ACC_FS_4G:
		range = 4;
		break;
	case LSM303C_ACC_FS_8G:
		range = 8;
		break;
	}

	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 2:
		range = LSM303C_ACC_FS_2G;
		break;
	case 4:
		range = LSM303C_ACC_FS_4G;
		break;
	case 8:
		range = LSM303C_ACC_FS_8G;
		break;
	default:
		dev_err(&stat->client->dev, "invalid range request: %lu, discarded\n", val);
		return -EINVAL;
	}

	mutex_lock(&stat->lock);
	err = lsm303c_acc_update_fs_range(stat, range);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}
	stat->pdata->fs_range = range;
	mutex_unlock(&stat->lock);
	dev_info(&stat->client->dev, "range set to: %lu g\n", val);

	return size;
}

static ssize_t attr_get_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

#ifdef CALIBRATION_TO_FILE
	int err = 0;
#endif /* CALIBRATION_TO_FILE */

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

#ifdef CALIBRATION_TO_FILE
	if (val && (cal_file_status != OFFSET_APPLIED)) {
		err = k303b_calibration_read(stat->cali_sw);
		if (err) {
			pr_err("Read Cal Fail from file !!!\n");
		} else {
			pr_info("Read Cal file success !!!\n");
			cal_file_status = OFFSET_APPLIED;
		}
	}
#endif /* CALIBRATION_TO_FILE */


	if (val)
		lsm303c_acc_enable(stat);
	else
		lsm303c_acc_disable(stat);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev, struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev, struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_set_threshx1(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THSX1, INT1_THRESHOLD_MASK, RES_INT_THSX1);
}

static ssize_t attr_get_threshx1(struct device *dev, struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THSX1);
}

static ssize_t attr_set_threshy1(struct device *dev, struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THSY1, INT1_THRESHOLD_MASK, RES_INT_THSY1);
}

static ssize_t attr_get_threshy1(struct device *dev, struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THSY1);
}

static ssize_t attr_set_threshz1(struct device *dev, struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THSZ1, INT1_THRESHOLD_MASK, RES_INT_THSZ1);
}

static ssize_t attr_get_threshz1(struct device *dev, struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THSZ1);
}

static ssize_t attr_get_source1(struct device *dev, struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}


#define XL_SELF_TEST_2G_MAX_LSB	(24576)
#define XL_SELF_TEST_2G_MIN_LSB	(1146)

static ssize_t attr_get_selftest(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	int val, i, en_state = 0;
	ssize_t ret;
	u8 x[8];
	s32 NO_ST[3] = {0, 0, 0};
	s32 ST[3] = {0, 0, 0};

	en_state = atomic_read(&stat->enabled);
	lsm303c_acc_disable(stat);

	lsm303c_acc_device_power_on(stat);

	x[0] = CTRL1;
	x[1] = 0x3f;
	lsm303c_acc_i2c_write(stat, x, 1);
	x[0] = CTRL4;
	x[1] = 0x04;
	x[2] = 0x00;
	x[3] = 0x00;
	lsm303c_acc_i2c_write(stat, x, 3);

	mdelay(80);

	x[0] = AXISDATA_REG;
	lsm303c_acc_i2c_read(stat, x, 6);

	for (i = 0; i < 5; i++) {
		while (1) {
			x[0] = 0x27;
			val = lsm303c_acc_i2c_read(stat, x, 1);
			if (val < 0) {
				ret = sprintf(buf, "I2C fail. (%d)\n", val);
				goto ST_EXIT;
			}
			if (x[0] & 0x08)
				break;
		}
		x[0] = AXISDATA_REG;
		lsm303c_acc_i2c_read(stat, x, 6);
		NO_ST[0] += (s16)((x[1] << 8) | x[0]);
		NO_ST[1] += (s16)((x[3] << 8) | x[2]);
		NO_ST[2] += (s16)((x[5] << 8) | x[4]);
	}
	NO_ST[0] /= 5;
	NO_ST[1] /= 5;
	NO_ST[2] /= 5;

	x[0] = CTRL5;
	x[1] = 0x04;
	lsm303c_acc_i2c_write(stat, x, 1);

	mdelay(80);

	x[0] = AXISDATA_REG;
	lsm303c_acc_i2c_read(stat, x, 6);

	for (i = 0; i < 5; i++) {
		while (1) {
			x[0] = 0x27;
			val = lsm303c_acc_i2c_read(stat, x, 1);
			if (val < 0) {
				ret = sprintf(buf, "I2C fail. (%d)\n", val);
				goto ST_EXIT;
			}
			if (x[0] & 0x08)
				break;
		}
		x[0] = AXISDATA_REG;
		lsm303c_acc_i2c_read(stat, x, 6);
		ST[0] += (s16)((x[1] << 8) | x[0]);
		ST[1] += (s16)((x[3] << 8) | x[2]);
		ST[2] += (s16)((x[5] << 8) | x[4]);
	}
	ST[0] /= 5;
	ST[1] /= 5;
	ST[2] /= 5;

	for (val = 1, i = 0; i < 3; i++) {
		ST[i] -= NO_ST[i];
		ST[i] = abs(ST[i]);

		if ((XL_SELF_TEST_2G_MIN_LSB > ST[i]) || (ST[i] > XL_SELF_TEST_2G_MAX_LSB)) {
			pr_info("ST[%d]: Out of range!! (%d)\n", i, ST[i]);
			val = 0;
		}
	}

	if (val)
		ret = sprintf(buf, "Self test: OK (%d, %d, %d)\n", ST[0], ST[1], ST[2]);
	else
		ret = sprintf(buf, "Self test: NG (%d, %d, %d)\n", ST[0], ST[1], ST[2]);

ST_EXIT:
	x[0] = CTRL1;
	x[1] = 0x00;
	lsm303c_acc_i2c_write(stat, x, 1);
	x[0] = CTRL5;
	x[1] = 0x00;
	lsm303c_acc_i2c_write(stat, x, 1);

	lsm303c_acc_device_power_off(stat);

	if (en_state)
		lsm303c_acc_enable(stat);

	return ret;
}



#ifdef DEBUG
/* PAY ATTENTION: These DEBUG functions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int rc;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	x[0] = stat->reg_addr;
	mutex_unlock(&stat->lock);
	x[1] = val;
	rc = lsm303c_acc_i2c_write(stat, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	rc = lsm303c_acc_i2c_read(stat, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);

	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	stat->reg_addr = val;
	mutex_unlock(&stat->lock);

	return size;
}
#endif

static ssize_t attr_fastcal_get(struct device *dev, struct device_attribute *attr, char *buf)
{
#if 1 /* LGE_SENSOR_FACTORY_AAT */
	ssize_t ret;

	ret = sprintf(buf, "%d\n", cal_result);
	return ret;
#else
	ssize_t ret;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	int data;

	pr_info("ST 1\n");

	mutex_lock(&stat->lock);
	pr_info("ST 2\n");
	data = atomic_read(&stat->fast_calib_rslt);
	pr_info("ST 3\n");
	mutex_unlock(&stat->lock);
	pr_info("ST 4\n");

	ret = sprintf(buf, "%d %d %d\n", stat->cali_sw[0], stat->cali_sw[1], stat->cali_sw[2]);
	return ret;
#endif /* LGE_SENSOR_FACTORY_AAT */
}


static int lsm303c_do_calibrate(struct lsm303c_acc_status *stat)
{
	int en_state = atomic_read(&stat->enabled);
	int ret = 0;
	int xyz[3] = {0}, prev_xyz[3] = {0};
	int sum[3] = {0}, average[3] = {0};
	int i = 0, count = 0;
	int lsb1g = 8192;

	cal_result = 0; /* LGE_SENSOR_FACTORY_AAT */

	lsm303c_acc_disable(stat);
	lsm303c_acc_device_power_on(stat);

	while (count < CALIBRATION_DATA_AMOUNT) {
		mdelay(20);
		ret = lsm303c_acc_get_lsb(stat, xyz);
		if (ret < 0) {
			pr_info("ST calibrate : read error (%d)\n", ret);
			goto err_handle;
		}

		if ((count > 0) &&
			((abs(prev_xyz[0] - xyz[0]) > LSM303C_SHAKING_DETECT_THRESHOLD) ||
			(abs(prev_xyz[1] - xyz[1]) > LSM303C_SHAKING_DETECT_THRESHOLD) ||
			(abs(prev_xyz[2] - xyz[2]) > LSM303C_SHAKING_DETECT_THRESHOLD))) {

			pr_info("ST calibrate : in shaking(%d %d %d)\n", xyz[0], xyz[1], xyz[2]);
		} else {
			count++;
			for (i = 0; i < 3; i++) {
				sum[i] += xyz[i];
				prev_xyz[i] = xyz[i];
			}
		}
	}

	for (i = 0; i < 3; i++)
		average[i] = sum[i]/count;

	if ((abs(average[0]) > TESTLIMIT_XY) ||
		(abs(average[1]) > TESTLIMIT_XY) ||
		(abs(average[2]) > TESTLIMIT_Z_USL_LSB || abs(average[2]) < TESTLIMIT_Z_LSL_LSB)) {
		pr_info("ST Calibration zero-g offset check failed (%d, %d, %d)\n",
			average[0], average[1], average[2]);
		ret = -1;
		goto err_handle;
	}

	stat->cali_sw[0] = average[0];
	stat->cali_sw[1] = average[1];
	if (average[2] > 0)
		stat->cali_sw[2] = average[2] - lsb1g;
	else
		stat->cali_sw[2] = lsb1g + average[2];

#ifdef CALIBRATION_TO_FILE
	ret = k303b_calibration_save(stat->cali_sw);
	if (ret)
		pr_err("ST Calibration fail!!! could not save to file\n");

#endif /* CALIBRATION_TO_FILE */

	cal_result = 1; /* LGE_SENSOR_FACTORY_AAT */

	pr_info("ST Calibration success!!!(%d %d %d)\n", average[0], average[1], average[2]);

err_handle:
	lsm303c_acc_device_power_off(stat);
	if (en_state)
		lsm303c_acc_enable(stat);

	return ret;
}

/*----------------------------------------------------------------------------*/
#ifdef CALIBRATION_TO_FILE
static int k303b_calibration_read(int *cal_read)
{
	int fd;
	int i;
	int res;
	char *fname = CALIBRATION_PATH;
	mm_segment_t old_fs = get_fs();
	char temp_str[5];

	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_RDONLY, 0);
	if (fd < 0) {
		pr_err("[%s] File Open Error !!!\n", __func__);
		sys_close(fd);
		return -EINVAL;
	}

	for (i = 0; i < 3; i++) {
		memset(temp_str, 0x00, sizeof(temp_str));
		res = sys_read(fd, temp_str, sizeof(temp_str));
		if (res < 0) {
			pr_err("[%s] Read Error !!!\n", __func__);
			sys_close(fd);
			return -EINVAL;
		}
		res = sscanf(temp_str, "%d", &cal_read[i]);
		if (!res) {
			pr_err("[%s] sscanf Read Error !!!\n", __func__);
			return -EINVAL;
		}
		pr_info("k303b_calibration_read : cal_read[%d]=%d\n", i, cal_read[i]);
	}
	sys_close(fd);
	set_fs(old_fs);
	pr_info("k303b_calibration_read Done.\n");
	pr_info("***k303b_calibration_read data : %d %d %d\n", cal_read[0], cal_read[1], cal_read[2]);

	return K303B_SUCCESS;
}

static int k303b_calibration_save(int *cal)
{
	int fd;
	int i;
	int res;
	char *fname = CALIBRATION_PATH;
	mm_segment_t old_fs = get_fs();
	char temp_str[5];

	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_WRONLY|O_CREAT|S_IROTH, 0666);
	if (fd < 0) {
		pr_err("[%s] File Open Error !!!(%d)\n", __func__, fd);
		sys_close(fd);
		return -EINVAL;
	}

	for (i = 0; i < 3; i++) {
		memset(temp_str, 0x00, sizeof(temp_str));
		sprintf(temp_str, "%d", cal[i]);
		res = sys_write(fd, temp_str, sizeof(temp_str));

		if (res < 0) {
			pr_err("[%s] Write Error !!!\n", __func__);
			sys_close(fd);
			return -EINVAL;
		}
	}
	sys_fsync(fd);
	sys_close(fd);

	sys_chmod(fname, 0664);
	set_fs(old_fs);

	pr_info("k303b_calibration_save Done.\n");
	pr_info("***k303b_calibration_save data : %d %d %d\n", cal[0], cal[1], cal[2]);

	return K303B_SUCCESS;
}

#endif /* CALIBRATION_TO_FILE */
/*----------------------------------------------------------------------------*/

static ssize_t attr_fastcal_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	lsm303c_do_calibrate(stat);
	mutex_unlock(&stat->lock);

	return size;
}
#if 1 /* LGE_SENSOR_FACTORY_AAT */
static ssize_t attr_get_cal_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	int data;

	pr_info("ST 1\n");

	mutex_lock(&stat->lock);
	pr_info("ST 2\n");
	data = atomic_read(&stat->fast_calib_rslt);
	pr_info("ST 3\n");
	mutex_unlock(&stat->lock);
	pr_info("ST 4\n");

	ret = sprintf(buf, "%d %d %d\n", stat->cali_sw[0], stat->cali_sw[1], stat->cali_sw[2]);
	return ret;

}


static ssize_t attr_set_dummy(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static ssize_t attr_get_dummy(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}


static DEVICE_ATTR(enable, 0666, attr_get_enable, attr_set_enable);
static DEVICE_ATTR(poll_delay, 0666, attr_get_polling_rate, attr_set_polling_rate);

static DEVICE_ATTR(full_scale, 0666, attr_get_range, attr_set_range);
static DEVICE_ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1);
static DEVICE_ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1);
static DEVICE_ATTR(int1_thresholdx, 0664, attr_get_threshx1, attr_set_threshx1);
static DEVICE_ATTR(int1_thresholdy, 0664, attr_get_threshy1, attr_set_threshy1);
static DEVICE_ATTR(int1_thresholdz, 0664, attr_get_threshz1, attr_set_threshz1);
static DEVICE_ATTR(int1_source, 0444, attr_get_source1, NULL);
static DEVICE_ATTR(run_calibration, 0664, attr_fastcal_get, attr_fastcal_set);
static DEVICE_ATTR(run_fast_calibration, 0664, attr_fastcal_get, attr_fastcal_set);

static DEVICE_ATTR(cal_offset, 0444, attr_get_cal_offset, NULL);
static DEVICE_ATTR(fast_calibration_x, 0664, attr_get_dummy, attr_set_dummy);
static DEVICE_ATTR(fast_calibration_y, 0664, attr_get_dummy, attr_set_dummy);
static DEVICE_ATTR(fast_calibration_z, 0664, attr_get_dummy, attr_set_dummy);
static DEVICE_ATTR(self_test, 0444, attr_get_selftest, NULL);

#ifdef DEBUG
static DEVICE_ATTR(reg_value, 0600, attr_reg_get, attr_reg_set);
static DEVICE_ATTR(reg_addr, 0200, NULL, attr_addr_set);
#endif

static struct attribute *k303c_acc_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_full_scale.attr,
	&dev_attr_int1_config.attr,
	&dev_attr_int1_duration.attr,
	&dev_attr_int1_thresholdx.attr,
	&dev_attr_int1_thresholdy.attr,
	&dev_attr_int1_thresholdz.attr,
	&dev_attr_int1_source.attr,
	&dev_attr_run_calibration.attr,
	&dev_attr_run_fast_calibration.attr,
	&dev_attr_cal_offset.attr,
	&dev_attr_fast_calibration_x.attr,
	&dev_attr_fast_calibration_y.attr,
	&dev_attr_fast_calibration_z.attr,
	&dev_attr_self_test.attr,
#ifdef DEBUG
	&dev_attr_reg_value.attr,
	&dev_attr_reg_addr.attr,
#endif
	NULL,
};
static const struct attribute_group k303c_acc_attr_group = {
	.attrs = k303c_acc_attributes,
};

#else /* LGE_SENSOR_FACTORY_AAT */
static struct device_attribute attributes[] = {
	__ATTR(enable, 0666, attr_get_enable, attr_set_enable),
	__ATTR(poll_delay, 0666, attr_get_polling_rate, attr_set_polling_rate),

	__ATTR(full_scale, 0666, attr_get_range, attr_set_range),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_thresholdx, 0664, attr_get_threshx1, attr_set_threshx1),
	__ATTR(int1_thresholdy, 0664, attr_get_threshy1, attr_set_threshy1),
	__ATTR(int1_thresholdz, 0664, attr_get_threshz1, attr_set_threshz1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(run_fast_calibration, 0664, attr_fastcal_get, attr_fastcal_set),
	__ATTR(self_test, 0444, attr_get_selftest, NULL),

#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;

	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);

	return 0;
}
#endif /* LGE_SENSOR_FACTORY_AAT */

static void lsm303c_acc_input_poll_work_func(struct work_struct *work)
{
	struct lsm303c_acc_status *stat;

	stat = container_of((struct work_struct *) work,
			struct lsm303c_acc_status, input_poll_work);

	mutex_lock(&stat->lock);
	lsm303c_acc_report_triple(stat);
	mutex_unlock(&stat->lock);

	if (atomic_read(&stat->enabled))
		hrtimer_start(&stat->hr_timer_poll, stat->polling_ktime, HRTIMER_MODE_REL);
}

enum hrtimer_restart lsm303c_acc_hr_timer_poll_function(struct hrtimer *timer)
{
	struct lsm303c_acc_status *stat;

	stat = container_of((struct hrtimer *)timer, struct lsm303c_acc_status, hr_timer_poll);

	queue_work(stat->hr_timer_poll_work_queue, &stat->input_poll_work);

	return HRTIMER_NORESTART;
}

int lsm303c_acc_input_open(struct input_dev *input)
{
	struct lsm303c_acc_status *stat = input_get_drvdata(input);

	dev_dbg(&stat->client->dev, "%s\n", __func__);

	return 0; /*lsm303c_acc_enable(stat);*/
}

void lsm303c_acc_input_close(struct input_dev *dev)
{
	struct lsm303c_acc_status *stat = input_get_drvdata(dev);

	dev_dbg(&stat->client->dev, "%s\n", __func__);
	lsm303c_acc_disable(stat);
}

static int lsm303c_acc_validate_pdata(struct lsm303c_acc_status *stat)
{
	/* checks for correctness of minimal polling period */
		stat->pdata->min_interval =
		max((unsigned int)LSM303C_ACC_MIN_POLL_PERIOD_MS,
						stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval, stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 || stat->pdata->axis_map_y > 2 || stat->pdata->axis_map_z > 2) {
		dev_err(&stat->client->dev, "invalid axis_map value x:%u y:%u z%u\n",
						stat->pdata->axis_map_x,
						stat->pdata->axis_map_y,
						stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 || stat->pdata->negate_y > 1 || stat->pdata->negate_z > 1) {
		dev_err(&stat->client->dev, "invalid negate value x:%u y:%u z:%u\n",
						stat->pdata->negate_x,
						stat->pdata->negate_y,
						stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(&stat->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm303c_acc_input_init(struct lsm303c_acc_status *stat)
{
	int err;

	INIT_WORK(&stat->input_poll_work, lsm303c_acc_input_poll_work_func);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(&stat->client->dev, "input device allocation failed\n");
		goto err0;
	}

	stat->input_dev->open = lsm303c_acc_input_open;
	stat->input_dev->close = lsm303c_acc_input_close;

	stat->input_dev->name = LSM303C_ACC_DEVICE_CUSTOM_NAME;

#if 1 /* LGE_SENSOR_FACTORY_AAT */
	stat->input_dev->dev.init_name = "lge_accelerometer";
#endif
	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);

	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, stat->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, stat->input_dev->absbit);

	input_set_abs_params(stat->input_dev, ABS_X, G_MIN, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Y, G_MIN, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Z, G_MIN, G_MAX, FUZZ, FLAT);

	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);

	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(&stat->client->dev, "unable to register input device %s\n", stat->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev);
err0:
	return err;
}

static void lsm303c_acc_input_cleanup(struct lsm303c_acc_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

static int lsm303c_parse_dts(struct lsm303c_acc_status *stat)
{
	int rc;
	struct device_node *np;

	pr_info("%s\n", __func__);

	np = stat->client->dev.of_node;
	if (!np)
		return -EINVAL;


	stat->vdd_reg = regulator_get(&stat->client->dev, "st,vdd_ana");
	if (IS_ERR(stat->vdd_reg)) {
		rc = PTR_ERR(stat->vdd_reg);
		dev_err(&stat->client->dev,
			"Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}
	if (regulator_count_voltages(stat->vdd_reg) > 0) {
		rc = regulator_set_voltage(stat->vdd_reg,
					   3000000,
					   3000000);
	}

	stat->vdd_i2c = regulator_get(&stat->client->dev, "st,vddio_i2c");
	if (IS_ERR(stat->vdd_i2c)) {
		rc = PTR_ERR(stat->vdd_i2c);
		dev_err(&stat->client->dev,
			"Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(stat->vdd_i2c) > 0) {
		rc = regulator_set_voltage(stat->vdd_i2c,
					   1800000,
					   1800000);
	}

	rc = regulator_enable(stat->vdd_i2c);
	rc = regulator_enable(stat->vdd_reg);

	mdelay(300);

	return 0;
}

static int lsm303c_acc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct lsm303c_acc_status *stat;

	u32 smbus_func = (I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK);

	int err = -1;

#ifdef CONFIG_LGE_SENSOR
	int lge_hw_revision = 0;

	lge_hw_revision = lge_get_board_revno();
	dev_info(&client->dev, "lge hw revision : %s, enum : %d.\n", lge_get_board_rev(), lge_hw_revision);

	if (lge_hw_revision == HW_REV_A || lge_hw_revision == HW_REV_B) {
		/*HW_REV_A == 3. Rev 0*/
		/*HW_REV_B == 4. Rev A*/
		default_lsm303c_acc_pdata.negate_x = 1;
		default_lsm303c_acc_pdata.negate_y = 1;
		default_lsm303c_acc_pdata.negate_z = 1;
	} else if (lge_hw_revision == HW_REV_HDKA) {
		/*HDK Power Board.*/
		default_lsm303c_acc_pdata.negate_x = 0;
		default_lsm303c_acc_pdata.negate_y = 1;
		default_lsm303c_acc_pdata.negate_z = 0;
	} else {
		default_lsm303c_acc_pdata.negate_x = 1;
		default_lsm303c_acc_pdata.negate_y = 1;
		default_lsm303c_acc_pdata.negate_z = 1;
	}
#endif/*#ifdef CONFIG_LGE_SENSOR*/

	dev_info(&client->dev, "probe start.\n");

	stat = kzalloc(sizeof(struct lsm303c_acc_status), GFP_KERNEL);
	if (stat == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: %d\n", err);
		goto exit_check_functionality_failed;
	}

	/* Support for both I2C and SMBUS adapter interfaces. */
	stat->use_smbus = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			stat->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			goto exit_check_functionality_failed;
		}
	}

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	stat->client = client;
	i2c_set_clientdata(client, stat);

	lsm303c_parse_dts(stat);
	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

	if (client->dev.platform_data == NULL) {
		default_lsm303c_acc_pdata.gpio_int1 = int1_gpio;

		memcpy(stat->pdata, &default_lsm303c_acc_pdata, sizeof(*stat->pdata));
		dev_info(&client->dev, "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, client->dev.platform_data, sizeof(*stat->pdata));
	}
	stat->hr_timer_poll_work_queue = 0;

	err = lsm303c_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}


	if (stat->pdata->init) {
		err = stat->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	if (stat->pdata->gpio_int1 >= 0) {
		stat->irq1 = gpio_to_irq(stat->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d, mapped on gpio:%d\n",
			LSM303C_ACC_DEV_NAME, __func__, stat->irq1, stat->pdata->gpio_int1);
	}

	lsm303c_acc_set_init_register_values(stat);

	err = lsm303c_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&stat->enabled, 1);

	err = lsm303c_acc_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lsm303c_acc_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	stat->hr_timer_poll_work_queue =
			create_workqueue("lsm303c_acc_hr_timer_poll_wq");
	hrtimer_init(&stat->hr_timer_poll, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_poll.function = &lsm303c_acc_hr_timer_poll_function;

	err = lsm303c_acc_input_init(stat);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_remove_hr_work_queue;
	}

#if 1 /* LGE_SENSOR_FACTORY_AAT */
	/* Register sysfs hooks */
	err = sysfs_create_group(&stat->input_dev->dev.kobj, &k303c_acc_attr_group);
#else
	err = create_sysfs_interfaces(&client->dev);
#endif
	if (err < 0) {
		dev_err(&client->dev,
		   "device %s sysfs register failed\n", LSM303C_ACC_DEV_NAME);
		goto err_input_cleanup;
	}


	lsm303c_acc_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

	if (stat->pdata->gpio_int1 >= 0) {
		INIT_WORK(&stat->irq1_work, lsm303c_acc_irq1_work_func);
		stat->irq1_work_queue = create_singlethread_workqueue("lsm303c_acc_wq1");
		if (!stat->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev, "cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(stat->irq1, lsm303c_acc_isr1, IRQF_TRIGGER_RISING, "lsm303c_acc_irq1", stat);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(stat->irq1);
	}

	mutex_unlock(&stat->lock);

	dev_info(&client->dev, "%s: probed\n", LSM303C_ACC_DEV_NAME);

	return 0;

err_destoyworkqueue1:
	if (stat->pdata->gpio_int1 >= 0)
		destroy_workqueue(stat->irq1_work_queue);
err_remove_sysfs_int:
#if 1 /* LGE_SENSOR_FACTORY_AAT */
	sysfs_remove_group(&stat->input_dev->dev.kobj, &k303c_acc_attr_group);
#else
	remove_sysfs_interfaces(&client->dev);
#endif
err_input_cleanup:
	lsm303c_acc_input_cleanup(stat);
err_remove_hr_work_queue:
	if (!stat->hr_timer_poll_work_queue) {
			flush_workqueue(stat->hr_timer_poll_work_queue);
			destroy_workqueue(stat->hr_timer_poll_work_queue);
	}
err_power_off:
	lsm303c_acc_device_power_off(stat);
err_pdata_init:
	if (stat->pdata->exit)
		stat->pdata->exit();
exit_kfree_pdata:
	kfree(stat->pdata);
err_mutexunlock:
	mutex_unlock(&stat->lock);
/* err_freedata: */
	kfree(stat);
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", LSM303C_ACC_DEV_NAME);
	return err;
}

static int lsm303c_acc_remove(struct i2c_client *client)
{

	struct lsm303c_acc_status *stat = i2c_get_clientdata(client);

	dev_info(&stat->client->dev, "driver removing\n");

	if (stat->pdata->gpio_int1 >= 0) {
		free_irq(stat->irq1, stat);
		gpio_free(stat->pdata->gpio_int1);
		destroy_workqueue(stat->irq1_work_queue);
	}

	lsm303c_acc_disable(stat);
	lsm303c_acc_input_cleanup(stat);

#if 1 /* LGE_SENSOR_FACTORY_AAT */
	sysfs_remove_group(&stat->input_dev->dev.kobj, &k303c_acc_attr_group);
#else
	remove_sysfs_interfaces(&client->dev);
#endif
	if (!stat->hr_timer_poll_work_queue) {
			flush_workqueue(stat->hr_timer_poll_work_queue);
			destroy_workqueue(stat->hr_timer_poll_work_queue);
	}

	if (stat->pdata->exit)
		stat->pdata->exit();
	kfree(stat->pdata);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM
static int lsm303c_acc_resume(struct i2c_client *client)
{
	struct lsm303c_acc_status *stat = i2c_get_clientdata(client);

	if (stat->on_before_suspend)
		return lsm303c_acc_enable(stat);

	return 0;
}

static int lsm303c_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lsm303c_acc_status *stat = i2c_get_clientdata(client);

	stat->on_before_suspend = atomic_read(&stat->enabled);

	return lsm303c_acc_disable(stat);
}
#else
#define lsm303c_acc_suspend	NULL
#define lsm303c_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm303c_acc_id[] = {
		{ LSM303C_ACC_DEV_NAME, 0 },
		{ },
};

MODULE_DEVICE_TABLE(i2c, lsm303c_acc_id);


static struct i2c_driver lsm303c_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM303C_ACC_DEV_NAME,
	},
	.probe = lsm303c_acc_probe,
	.remove = lsm303c_acc_remove,
	.suspend = lsm303c_acc_suspend,
	.resume = lsm303c_acc_resume,
	.id_table = lsm303c_acc_id,
};

static int __init lsm303c_acc_init(void)
{
	pr_info("%s accelerometer driver: init\n", LSM303C_ACC_DEV_NAME);
	return i2c_add_driver(&lsm303c_acc_driver);
}

static void __exit lsm303c_acc_exit(void)
{
	i2c_del_driver(&lsm303c_acc_driver);
}

module_init(lsm303c_acc_init);
module_exit(lsm303c_acc_exit);

MODULE_DESCRIPTION("lsm303c accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");

