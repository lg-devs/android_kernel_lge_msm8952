/***************************************************************************
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *    File  : lgtp_device_s3320.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[S3320]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_3/lgtp_common.h>

#include <linux/input/unified_driver_3/lgtp_common_driver.h>
#include <linux/input/unified_driver_3/lgtp_platform_api.h>
#include <linux/input/unified_driver_3/lgtp_device_s3320.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
/* Enable TCI Debug */
#define ENABLE_TCI_DEBUG

/* RMI4 spec from 511-000405-01 Rev.D
 * Function	Purpose				See page
 * $01		RMI Device Control		45
 * $1A		0-D capacitive button sensors	61
 * $05		Image Reporting			68
 * $07		Image Reporting			75
 * $08		BIST				82
 * $09		BIST				87
 * $11		2-D TouchPad sensors		93
 * $19		0-D capacitive button sensors	141
 * $30		GPIO/LEDs			148
 * $31		LEDs				162
 * $34		Flash Memory Management		163
 * $36		Auxiliary ADC			174
 * $54		Test Reporting			176
 */

#define RMI_DEVICE_CONTROL			0x01
#define TOUCHPAD_SENSORS			0x12
#define FLASH_MEMORY_MANAGEMENT		0x34
#define LPWG_CONTROL				0x51
#define ANALOG_CONTROL				0x54
#define SENSOR_CONTROL				0x55

/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */
#define DEVICE_CONTROL_REG			(ts->common_fc.dsc.control_base)
#define MANUFACTURER_ID_REG			(ts->common_fc.dsc.query_base)
#define CUSTOMER_FAMILY_REG			(ts->common_fc.dsc.query_base+2)
#define FW_REVISION_REG				(ts->common_fc.dsc.query_base+3)
#define PRODUCT_ID_REG				(ts->common_fc.dsc.query_base+11)

#define COMMON_PAGE					(ts->common_fc.function_page)
#define LPWG_PAGE					(ts->lpwg_fc.function_page)
#define FINGER_PAGE					(ts->finger_fc.function_page)
#define ANALOG_PAGE					(ts->analog_fc.function_page)
#define FLASH_PAGE					(ts->flash_fc.function_page)
#define SENSOR_PAGE					(ts->sensor_fc.function_page)

#define DEVICE_STATUS_REG			(ts->common_fc.dsc.data_base)
#define INTERRUPT_STATUS_REG		(ts->common_fc.dsc.data_base+1)
#define INTERRUPT_ENABLE_REG		(ts->common_fc.dsc.control_base+1)

/* TOUCHPAD_SENSORS */
#define FINGER_DATA_REG_START		(ts->finger_fc.dsc.data_base)
#define OBJECT_ATTENTION_REG		(ts->finger_fc.dsc.data_base+3)
#define FINGER_REPORT_REG			(ts->finger_fc.dsc.control_base+7)
#define OBJECT_REPORT_ENABLE_REG	(ts->finger_fc.dsc.control_base+9)
#define WAKEUP_GESTURE_ENABLE_REG	(ts->finger_fc.dsc.control_base+12)

#define DYNAMIC_SENSING_CONTROL_REG	(ts->analog_fc.dsc.control_base+40)

#define FLASH_CONFIG_ID_REG			(ts->flash_fc.dsc.control_base)
#define FLASH_STATUS_REG			(ts->flash_fc.dsc.data_base+3)

#define LPWG_STATUS_REG				(ts->lpwg_fc.dsc.data_base)
#define LPWG_DATA_REG				(ts->lpwg_fc.dsc.data_base+1)
#define LPWG_OVER_TAPCOUNT			(ts->lpwg_fc.dsc.data_base+73)
#define LPWG_FAIL_REASON			(ts->lpwg_fc.dsc.data_base+74)

#define LPWG_TAPCOUNT_REG			(ts->lpwg_fc.dsc.control_base)
#define LPWG_MIN_INTERTAP_REG		(ts->lpwg_fc.dsc.control_base+1)
#define LPWG_MAX_INTERTAP_REG		(ts->lpwg_fc.dsc.control_base+2)
#define LPWG_TOUCH_SLOP_REG			(ts->lpwg_fc.dsc.control_base+3)
#define LPWG_TAP_DISTANCE_REG		(ts->lpwg_fc.dsc.control_base+4)
#define LPWG_INTERRUPT_DELAY_REG	(ts->lpwg_fc.dsc.control_base+6)
#define LPWG_TAPCOUNT_REG2			(ts->lpwg_fc.dsc.control_base+7)
#define LPWG_MIN_INTERTAP_REG2		(ts->lpwg_fc.dsc.control_base+8)
#define LPWG_MAX_INTERTAP_REG2		(ts->lpwg_fc.dsc.control_base+9)
#define LPWG_TOUCH_SLOP_REG2		(ts->lpwg_fc.dsc.control_base+10)
#define LPWG_TAP_DISTANCE_REG2		(ts->lpwg_fc.dsc.control_base+11)
#define LPWG_INTERRUPT_DELAY_REG2	(ts->lpwg_fc.dsc.control_base+13)

#if defined(ENABLE_SWIPE_MODE)
#define SWIPE_COOR_START_X_LSB_REG			(ts->lpwg_fc.dsc.data_base + 75)
#define SWIPE_COOR_START_X_MSB_REG			(ts->lpwg_fc.dsc.data_base + 76)
#define SWIPE_COOR_START_Y_LSB_REG			(ts->lpwg_fc.dsc.data_base + 77)
#define SWIPE_COOR_START_Y_MSB_REG			(ts->lpwg_fc.dsc.data_base + 78)
#define SWIPE_COOR_END_X_LSB_REG			(ts->lpwg_fc.dsc.data_base + 79)
#define SWIPE_COOR_END_X_MSB_REG			(ts->lpwg_fc.dsc.data_base + 80)
#define SWIPE_COOR_END_Y_LSB_REG			(ts->lpwg_fc.dsc.data_base + 81)
#define SWIPE_COOR_END_Y_MSB_REG			(ts->lpwg_fc.dsc.data_base + 82)

#define SWIPE_FAIL_REASON_REG				(ts->lpwg_fc.dsc.data_base + 83)
#define SWIPE_TIME_LSB_REG					(ts->lpwg_fc.dsc.data_base + 84)
#define SWIPE_TIME_MSB_REG					(ts->lpwg_fc.dsc.data_base + 85)

#define SWIPE_ENABLE_REG						(ts->lpwg_fc.dsc.control_base + 15)

#define SWIPE_DOWN_DISTANCE_REG					(ts->lpwg_fc.dsc.control_base + 16)
#define SWIPE_DOWN_RATIO_THRESHOLD_REG			(ts->lpwg_fc.dsc.control_base + 17)
#define SWIPE_DOWN_RATIO_CHECK_PERIOD_REG		(ts->lpwg_fc.dsc.control_base + 18)
#define SWIPE_DOWN_RATIO_CHK_MIN_DISTANCE_REG	(ts->lpwg_fc.dsc.control_base + 19)
#define SWIPE_DOWN_MIN_TIME_THRES_LSB_REG		(ts->lpwg_fc.dsc.control_base + 20)
#define SWIPE_DOWN_MIN_TIME_THRES_MSB_REG		(ts->lpwg_fc.dsc.control_base + 21)
#define SWIPE_DOWN_MAX_TIME_THRES_LSB_REG		(ts->lpwg_fc.dsc.control_base + 22)
#define SWIPE_DOWN_MAX_TIME_THRES_MSB_REG		(ts->lpwg_fc.dsc.control_base + 23)
#endif

#if defined(ENABLE_SWIPE_UP)
#define SWIPE_UP_DISTANCE_REG					(ts->lpwg_fc.dsc.control_base + 32)
#define SWIPE_UP_RATIO_THRESHOLD_REG			(ts->lpwg_fc.dsc.control_base + 33)
#define SWIPE_UP_RATIO_CHECK_PERIOD_REG			(ts->lpwg_fc.dsc.control_base + 34)
#define SWIPE_UP_RATIO_CHK_MIN_DISTANCE_REG		(ts->lpwg_fc.dsc.control_base + 35)
#define SWIPE_UP_MIN_TIME_THRES_LSB_REG			(ts->lpwg_fc.dsc.control_base + 36)
#define SWIPE_UP_MIN_TIME_THRES_MSB_REG			(ts->lpwg_fc.dsc.control_base + 37)
#define SWIPE_UP_MAX_TIME_THRES_LSB_REG			(ts->lpwg_fc.dsc.control_base + 38)
#define SWIPE_UP_MAX_TIME_THRES_MSB_REG			(ts->lpwg_fc.dsc.control_base + 39)
#endif

#define FILTER_SMALL_Z_CTRL						(ts->lpwg_fc.dsc.control_base + 53)

#define MAX_PRESSURE			255
#define LPWG_BLOCK_SIZE			7
#define KNOCKON_DELAY			68	/* 700ms */
#define KNOCKCODE_DELAY			25	/* 250ms */


/****************************************************************************
 * Macros
 ****************************************************************************/
/* Get user-finger-data from register.
 */
#define GET_X_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define GET_Y_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define GET_WIDTH_MAJOR(_width_x, _width_y) \
	(((_width_x - _width_y) > 0) ? _width_x : _width_y)
#define GET_WIDTH_MINOR(_width_x, _width_y) \
	(((_width_x - _width_y) > 0) ? _width_y : _width_x)

#define GET_ORIENTATION(_width_y, _width_x) \
	(((_width_y - _width_x) > 0) ? 0 : 1)
#define GET_PRESSURE(_pressure) \
		_pressure

#define GET_LOW_U8_FROM_U16(_u16_data) \
	((u8)((_u16_data) & 0x00FF))
#define GET_HIGH_U8_FROM_U16(_u16_data) \
	((u8)(((_u16_data) & 0xFF00) >> 8))

/****************************************************************************
* Type Definitions
****************************************************************************/

/****************************************************************************
* Variables
****************************************************************************/
#if defined(TOUCH_MODEL_C70)
static const char defaultFirmware[] = "synaptics/c70/PLG455-V1.22-PR1815058-DS5.2.12.0.1013_40047196.img";
#elif defined(TOUCH_MODEL_Y90)
static const char defaultFirmware[] = "synaptics/y90/PLG465-V1.11-PR1815155-DS5.2.12.0.13-4005018B.img";
#elif defined(TOUCH_MODEL_Y70)
static const char defaultFirmware[] = "synaptics/y70/PLG455-V1.22-PR1815058-DS5.2.12.0.1013_40047196.img";
#elif defined(TOUCH_MODEL_C90)
static const char defaultFirmware[] = "synaptics/c90/PLG465-V1.11-PR1815155-DS5.2.12.0.13-4005018B.img";
#elif defined(TOUCH_MODEL_C90NAS)
static const char defaultFirmware[] = "synaptics/c90nas_spr_us/PLG465-V1.10-PR1815155-DS5.2.12.0.13-4005018A.img";
#elif defined(TOUCH_MODEL_P1B)
static const char defaultFirmware[] = "synaptics/p1b/PLG449-V1.08-PR1888337-DS5.2.13.0.1014_40052188.img";
#elif defined(TOUCH_MODEL_P1C)
static const char defaultFirmware[] = "synaptics/p1c/PLG465-V1.09-PR1815155-DS5.2.12.0.13-40050189.img";
#elif defined(TOUCH_MODEL_YG)
static const char defaultFirmware[] = "synaptics/yg/PLG521-V1.03-PR1854132-DS5.2.12.1013_40050183.img";
#elif defined(TOUCH_MODEL_C100N)
static const char defaultFirmware[] = "synaptics/yg/PLG521-V1.03-PR1854132-DS5.2.12.1013_40050183.img";
#else
#error "Model should be defined"
#endif

struct synaptics_ts_data *ts = NULL;
static struct synaptics_ts_exp_fhandler rmidev_fhandler;
int enable_rmi_dev = 0;
#if defined(ENABLE_SWIPE_MODE)
int get_swipe_mode = 1;
int wakeup_by_swipe = 0;
#endif

/****************************************************************************
* Local Function Prototypes
****************************************************************************/

/****************************************************************************
* Device Specific Function Prototypes
****************************************************************************/
static int S3320_Initialize(struct i2c_client *client);
static void S3320_Reset(struct i2c_client *client);
static int S3320_Connect(void);
static int S3320_InitRegister(struct i2c_client *client);
static int S3320_InterruptHandler(struct i2c_client *client, TouchReadData *pData);
static int S3320_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo);
static int S3320_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo);
static int S3320_UpdateFirmware(struct i2c_client *client, char *pFilename);
static int S3320_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting);
static int S3320_DoSelfDiagnosis(struct i2c_client *client, int *pRawStatus, int *pChannelStatus, char *pBuf,
					int bufSize, int *pDataLen);
static int S3320_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue);


/****************************************************************************
* Local Functions
****************************************************************************/
static int synaptics_ts_page_data_read(struct i2c_client *client,
	 u8 page, u8 reg, int size, u8 *data)
{
	int ret = 0;

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, page);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	ret = Touch_I2C_Read(client, reg, data, size);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	return TOUCH_SUCCESS;
}

static int synaptics_ts_page_data_write(struct i2c_client *client,
	 u8 page, u8 reg, int size, u8 *data)
{
	int ret = 0;

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, page);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	ret = Touch_I2C_Write(client, reg, data, size);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	return TOUCH_SUCCESS;
}

static int synaptics_ts_page_data_write_byte(struct i2c_client *client,
	 u8 page, u8 reg, u8 data)
{
	int ret = 0;

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, page);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	ret = Touch_I2C_Write_Byte(client, reg, data);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	return TOUCH_SUCCESS;
}

static int read_page_description_table(struct i2c_client *client)
{
	int ret = 0;
	struct function_descriptor buffer;

	unsigned short address = 0;
	unsigned short page_num = 0;

	TOUCH_FUNC();

	memset(&buffer, 0x0, sizeof(struct function_descriptor));
	memset(&ts->common_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->finger_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->lpwg_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->sensor_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->analog_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->flash_fc, 0x0, sizeof(struct ts_ic_function));

	for (page_num = 0; page_num < PAGE_MAX_NUM; page_num++) {
		ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, page_num);
		if (ret == TOUCH_FAIL)
			return TOUCH_FAIL;

		for (address = DESCRIPTION_TABLE_START; address > 10;
				address -= sizeof(struct function_descriptor)) {
			ret = Touch_I2C_Read(client, address, (unsigned char *)&buffer, sizeof(buffer));
			if (ret == TOUCH_FAIL)
				return TOUCH_FAIL;

			if (buffer.id == 0)
				break;

			switch (buffer.id) {
			case RMI_DEVICE_CONTROL:
				ts->common_fc.dsc = buffer;
				ts->common_fc.function_page = page_num;
				break;
			case TOUCHPAD_SENSORS:
				ts->finger_fc.dsc = buffer;
				ts->finger_fc.function_page = page_num;
				break;
			case LPWG_CONTROL:
				ts->lpwg_fc.dsc = buffer;
				ts->lpwg_fc.function_page = page_num;
				break;
			case SENSOR_CONTROL:
				ts->sensor_fc.dsc = buffer;
				ts->sensor_fc.function_page = page_num;
				break;
			case ANALOG_CONTROL:
				ts->analog_fc.dsc = buffer;
				ts->analog_fc.function_page = page_num;
				break;
			case FLASH_MEMORY_MANAGEMENT:
				ts->flash_fc.dsc = buffer;
				ts->flash_fc.function_page = page_num;
			default:
				break;
			}
		}
	}

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, 0x00);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	if (ts->common_fc.dsc.id == 0 || ts->finger_fc.dsc.id == 0 || ts->analog_fc.dsc.id == 0
		|| ts->flash_fc.dsc.id == 0) {
		TOUCH_ERR("failed to read page description\n");
		return TOUCH_FAIL;
	}

	TOUCH_DBG("common[%dP:0x%02x] finger[%dP:0x%02x] lpwg[%dP:0x%02x] sensor[%dP:0x%02x] \
		 analog[%dP:0x%02x] flash[%dP:0x%02x]\n",
		ts->common_fc.function_page, ts->common_fc.dsc.id,
		ts->finger_fc.function_page, ts->finger_fc.dsc.id,
		ts->lpwg_fc.function_page, ts->lpwg_fc.dsc.id,
		ts->sensor_fc.function_page, ts->sensor_fc.dsc.id,
		ts->analog_fc.function_page, ts->analog_fc.dsc.id,
		ts->flash_fc.function_page, ts->flash_fc.dsc.id);

	return TOUCH_SUCCESS;

}

static int check_firmware_status(struct i2c_client *client)
{
	u8 device_status = 0;
	u8 flash_status = 0;

	Touch_I2C_Read_Byte(client, FLASH_STATUS_REG, &flash_status);
	Touch_I2C_Read_Byte(client, DEVICE_STATUS_REG, &device_status);

	if ((device_status & (DEVICE_STATUS_FLASH_PROG|DEVICE_CRC_ERROR_MASK)) || (flash_status & 0xFF)) {
		TOUCH_ERR("FLASH_STATUS[0x%x] DEVICE_STATUS[0x%x]\n", (u32)flash_status, (u32)device_status);
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

#if defined(ENABLE_SWIPE_MODE)
static int swipe_control(struct i2c_client *client, int mode)
{
	int ret = 0;
	u8 buf = 0;

#if defined(ENABLE_SWIPE_UP)
	buf = mode ? 0x03 : 0x00;
#else
	buf = mode ? 0x01 : 0x00;
#endif

	ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, SWIPE_ENABLE_REG, buf);

	if (ret) {
		TOUCH_ERR("I2C fail\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int swipe_setParam(struct i2c_client *client)
{
	int ret = 0;

	ret |= Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, LPWG_PAGE);

	ret |= Touch_I2C_Write_Byte(client, SWIPE_DOWN_DISTANCE_REG, 7);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_DOWN_RATIO_CHK_MIN_DISTANCE_REG, 1);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_DOWN_RATIO_THRESHOLD_REG, 200);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_DOWN_RATIO_CHECK_PERIOD_REG, 30);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_DOWN_MIN_TIME_THRES_LSB_REG, GET_LOW_U8_FROM_U16(0));
	ret |= Touch_I2C_Write_Byte(client, SWIPE_DOWN_MIN_TIME_THRES_MSB_REG, GET_HIGH_U8_FROM_U16(0));
	ret |= Touch_I2C_Write_Byte(client, SWIPE_DOWN_MAX_TIME_THRES_LSB_REG, GET_LOW_U8_FROM_U16(0xF4));
	ret |= Touch_I2C_Write_Byte(client, SWIPE_DOWN_MAX_TIME_THRES_MSB_REG, GET_HIGH_U8_FROM_U16(0x01));

#if defined(ENABLE_SWIPE_UP)
	ret |= Touch_I2C_Write_Byte(client, SWIPE_UP_DISTANCE_REG, 7);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_UP_RATIO_CHK_MIN_DISTANCE_REG, 1);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_UP_RATIO_THRESHOLD_REG, 200);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_UP_RATIO_CHECK_PERIOD_REG, 30);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_UP_MIN_TIME_THRES_LSB_REG, GET_LOW_U8_FROM_U16(0));
	ret |= Touch_I2C_Write_Byte(client, SWIPE_UP_MIN_TIME_THRES_MSB_REG, GET_HIGH_U8_FROM_U16(0));
	ret |= Touch_I2C_Write_Byte(client, SWIPE_UP_MAX_TIME_THRES_LSB_REG, GET_LOW_U8_FROM_U16(0xF4));
	ret |= Touch_I2C_Write_Byte(client, SWIPE_UP_MAX_TIME_THRES_MSB_REG, GET_HIGH_U8_FROM_U16(0x01));
#endif

	ret |= Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);

	if (ret) {
		TOUCH_ERR("I2C fail\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}
#endif

static int tci_control(struct i2c_client *client, int type, u8 value)
{
	int ret = 0;
	u8 buffer[3] = {0};

	switch (type) {
	case REPORT_MODE_CTRL:
		ret = Touch_I2C_Read(client, INTERRUPT_ENABLE_REG, buffer, 1);
		if (ret < 0)
			return TOUCH_FAIL;

		ret = Touch_I2C_Write_Byte(client, INTERRUPT_ENABLE_REG,
				value ? buffer[0] & ~INTERRUPT_MASK_ABS0 : buffer[0] | INTERRUPT_MASK_ABS0);
		if (ret < 0)
			return TOUCH_FAIL;

		ret = Touch_I2C_Read(client, FINGER_REPORT_REG, buffer, 3);
		if (ret < 0)
			return TOUCH_FAIL;

		buffer[2] = (buffer[2] & 0xfc) | (value ? 0x2 : 0x0);
		ret = Touch_I2C_Write(client, FINGER_REPORT_REG, buffer, 3);
		if (ret < 0)
			return TOUCH_FAIL;

		TOUCH_DBG("report mode: %d\n", value);
		break;
	case TCI_ENABLE_CTRL:
		ret = synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer);
		if (ret < 0)
			return TOUCH_FAIL;

		buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
		ret = synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer);
		if (ret < 0)
			return TOUCH_FAIL;
		break;
	case TCI_ENABLE_CTRL2:
		ret = synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer);
		if (ret < 0)
			return TOUCH_FAIL;

		buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
		ret = synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer);
		if (ret < 0)
			return TOUCH_FAIL;

		break;
	case TAP_COUNT_CTRL:
		ret = synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer);
		if (ret < 0)
			return TOUCH_FAIL;

		buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
		ret = synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer);
		if (ret < 0)
			return TOUCH_FAIL;

		break;
	case TAP_COUNT_CTRL2:
		ret = synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer);
		if (ret < 0)
			return TOUCH_FAIL;

		buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
		ret = synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer);
		if (ret < 0)
			return TOUCH_FAIL;
		break;
	case MIN_INTERTAP_CTRL:
		ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_MIN_INTERTAP_REG, value);
		if (ret < 0)
			return TOUCH_FAIL;

		break;
	case MIN_INTERTAP_CTRL2:
		ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_MIN_INTERTAP_REG2, value);
		if (ret < 0)
			return TOUCH_FAIL;
		break;
	case MAX_INTERTAP_CTRL:
		ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_MAX_INTERTAP_REG, value);
		if (ret < 0)
			return TOUCH_FAIL;
		break;
	case MAX_INTERTAP_CTRL2:
		ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_MAX_INTERTAP_REG2, value);
		if (ret < 0)
			return TOUCH_FAIL;
		break;
	case TOUCH_SLOP_CTRL:
		ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_TOUCH_SLOP_REG, value);
		if (ret < 0)
			return TOUCH_FAIL;
		break;
	case TOUCH_SLOP_CTRL2:
		ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_TOUCH_SLOP_REG2, value);
		if (ret < 0)
			return TOUCH_FAIL;
		break;
	case TAP_DISTANCE_CTRL:
		ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_TAP_DISTANCE_REG, value);
		if (ret < 0)
			return TOUCH_FAIL;
		break;
	case TAP_DISTANCE_CTRL2:
		ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_TAP_DISTANCE_REG2, value);
		if (ret < 0)
			return TOUCH_FAIL;
		break;
	case INTERRUPT_DELAY_CTRL:
		buffer[0] = value ? ((KNOCKON_DELAY << 1) | 0x1) : 0;
		ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG, buffer[0]);
		if (ret < 0)
			return TOUCH_FAIL;
		break;
	case INTERRUPT_DELAY_CTRL2:
		buffer[0] = value ? ((KNOCKCODE_DELAY << 1) | 0x1) : 0;
		ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG2, buffer[0]);
		if (ret < 0)
			return TOUCH_FAIL;
		break;

	default:
		break;
	}

	return TOUCH_SUCCESS;
}

static int sleep_control(struct i2c_client *client, int mode, int recal)
{
	int ret = 0;
	u8 curr = 0;
	u8 next = 0;

	ret = Touch_I2C_Read(client, DEVICE_CONTROL_REG, &curr, 1);
	if (ret < 0)
		return TOUCH_FAIL;

	next = (curr & 0xF8);

	if (mode == 1)
		next = next | DEVICE_CONTROL_NOSLEEP;
	else
		next = next | DEVICE_CONTROL_SLEEP;

	ret = Touch_I2C_Write_Byte(client, DEVICE_CONTROL_REG, next);
	if (ret < 0)
		return TOUCH_FAIL;

	TOUCH_DBG("%s : curr = [%x] next[%x]\n", __func__, curr, next);

	return TOUCH_SUCCESS;
}

static int lpwg_control(struct i2c_client *client, TouchState newState)
{
	int ret = 0;
	u8 readValue = 0;

	TOUCH_FUNC();

	/* backup interrupt enable register */
	ret = Touch_I2C_Read_Byte(client, INTERRUPT_ENABLE_REG, &readValue);
	if (ret < 0)
		return TOUCH_FAIL;

	/* disable interrupt */
	ret = Touch_I2C_Write_Byte(client, INTERRUPT_ENABLE_REG, 0x00);
	if (ret < 0)
		return TOUCH_FAIL;

	switch (newState) {
	case STATE_NORMAL:
		if (ts->currState == STATE_OFF) {
			sleep_control(client, 1, 0);
			msleep(5);
		}
		tci_control(client, TCI_ENABLE_CTRL, 0);
		tci_control(client, TCI_ENABLE_CTRL2, 0);
		tci_control(client, REPORT_MODE_CTRL, 0);
		msleep(25);
		break;

	case STATE_KNOCK_ON_ONLY:
		if (ts->currState == STATE_NORMAL) {
			msleep(70);
		} else if (ts->currState == STATE_OFF) {
			sleep_control(client, 1, 0);
			msleep(5);
			tci_control(client, REPORT_MODE_CTRL, 0);
		}

		tci_control(client, TAP_COUNT_CTRL, 2);
		tci_control(client, MIN_INTERTAP_CTRL, 0);
		tci_control(client, MAX_INTERTAP_CTRL, 70);
		tci_control(client, TOUCH_SLOP_CTRL, 100);
		tci_control(client, TAP_DISTANCE_CTRL, 10);
		tci_control(client, INTERRUPT_DELAY_CTRL, 0);

		tci_control(client, TCI_ENABLE_CTRL, 1);
		tci_control(client, TCI_ENABLE_CTRL2, 0);
		tci_control(client, REPORT_MODE_CTRL, 1);

#if defined(ENABLE_SWIPE_MODE)
		if (ts->lpwgSetting.coverState || !get_swipe_mode) {
			swipe_control(client, 0);
		} else {
			swipe_control(client, 1);
			swipe_setParam(client);
		}
#endif
		break;

	case STATE_KNOCK_ON_CODE:
		if (ts->currState == STATE_NORMAL) {
			msleep(70);
		} else if (ts->currState == STATE_OFF) {
			sleep_control(client, 1, 0);
			msleep(5);
			tci_control(client, REPORT_MODE_CTRL, 0);
		}

		tci_control(client, TAP_COUNT_CTRL, 2);
		tci_control(client, MIN_INTERTAP_CTRL, 0);
		tci_control(client, MAX_INTERTAP_CTRL, 70);
		tci_control(client, TOUCH_SLOP_CTRL, 100);
		tci_control(client, TAP_DISTANCE_CTRL, 10);
		tci_control(client, INTERRUPT_DELAY_CTRL, (u8)ts->lpwgSetting.isFirstTwoTapSame);

		tci_control(client, TAP_COUNT_CTRL2, (u8)ts->lpwgSetting.tapCount);
		tci_control(client, MIN_INTERTAP_CTRL2, 0);
		tci_control(client, MAX_INTERTAP_CTRL2, 70);
		tci_control(client, TOUCH_SLOP_CTRL2, 100);
		tci_control(client, TAP_DISTANCE_CTRL2, 255);
		tci_control(client, INTERRUPT_DELAY_CTRL2, 1);

		tci_control(client, TCI_ENABLE_CTRL, 1);
		tci_control(client, TCI_ENABLE_CTRL2, 1);
		tci_control(client, REPORT_MODE_CTRL, 1);

#if defined(ENABLE_SWIPE_MODE)
		if (ts->lpwgSetting.coverState || !get_swipe_mode) {
			swipe_control(client, 0);
		} else {
			swipe_control(client, 1);
			swipe_setParam(client);
		}
#endif
		break;

	case STATE_OFF:
		if (ts->currState == STATE_NORMAL)
			msleep(70);

		tci_control(client, TCI_ENABLE_CTRL, 0);
		tci_control(client, TCI_ENABLE_CTRL2, 0);
		tci_control(client, REPORT_MODE_CTRL, 1);
		msleep(5);
		sleep_control(client, 0, 0);
		break;

	default:
		TOUCH_ERR("invalid touch state ( %d )\n", newState);
		break;

	}

	/* restore interrupt enable register */
	ret = Touch_I2C_Write_Byte(client, INTERRUPT_ENABLE_REG, readValue);
	if (ret < 0)
		return TOUCH_FAIL;

	return TOUCH_SUCCESS;

}

static int get_object_count(struct i2c_client *client)
{
	u8 object_num = 0;
	u8 buf[2] = {0};
	u16 object_attention_data = 0;
	u16 i = 0;
	int ret = 0;

	ret = Touch_I2C_Read(client, OBJECT_ATTENTION_REG, buf, 2);
	if (ret == TOUCH_FAIL)
		TOUCH_WARN("failed to read finger number\n");

	object_attention_data = (((u16)((buf[1] << 8)  & 0xFF00)  | (u16)((buf[0]) & 0xFF)));

	for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
		if (object_attention_data & (0x1 << i))
			object_num = i + 1;
	}

	return object_num;
}

/****************************************************************************
* Device Specific Functions
****************************************************************************/
int synaptics_ts_rmidev_function(struct synaptics_ts_exp_fn *rmidev_fn, bool insert)
{
	rmidev_fhandler.inserted = insert;

	if (insert)
		rmidev_fhandler.exp_fn = rmidev_fn;
	else
		rmidev_fhandler.exp_fn = NULL;

	return TOUCH_SUCCESS;
}

/****************************************************************************
* show_firmware is uesd for firmware information in hidden menu
****************************************************************************/
static ssize_t show_version(struct i2c_client *client, char *buf)
{
	u8 readData[FW_VER_INFO_NUM] = {0};
	u8 fw_product_id[11];
	int ret = 0;

	ret |= Touch_I2C_Read(client, FLASH_CONFIG_ID_REG, readData, FW_VER_INFO_NUM);
	ret |= Touch_I2C_Read(client, PRODUCT_ID_REG, fw_product_id, sizeof(fw_product_id) - 1);

	WRITE_SYSBUF(buf, ret, "\n======== Auto Touch Test ========\n");
	WRITE_SYSBUF(buf, ret, "version : v%d.%02d\n", (readData[3] >> 7) & 0x01, readData[3] & 0x7F);
	WRITE_SYSBUF(buf, ret, "IC_product_id[%s]\n", fw_product_id);
	WRITE_SYSBUF(buf, ret, "Touch IC : s3320\n\n");

	return ret;
}

static ssize_t store_tci(struct i2c_client *client,
	const char *buf, size_t count)
{
	u32 type = 0, tci_num = 0, value = 0;

	sscanf(buf, "%d %d %d", &type, &tci_num, &value);

	tci_control(client, type, (u8)value);

	return count;
}

static ssize_t show_tci(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int i = 0;
	u8 buffer[7] = {0};

	Touch_I2C_Read(client, FINGER_REPORT_REG, buffer, 3);
	WRITE_SYSBUF(buf, ret, "report_mode [%s]\n",
		(buffer[2] & 0x3) == 0x2 ? "WAKEUP_ONLY" : "NORMAL");

	Touch_I2C_Read(client, WAKEUP_GESTURE_ENABLE_REG, buffer, 1);
	WRITE_SYSBUF(buf, ret, "wakeup_gesture [%d]\n\n", buffer[0]);

	for (i = 0; i < 2; i++) {
		synaptics_ts_page_data_read(client, LPWG_PAGE,
			LPWG_TAPCOUNT_REG + (i * LPWG_BLOCK_SIZE), 7, buffer);
		WRITE_SYSBUF(buf, ret, "TCI - %d\n", i+1);
		WRITE_SYSBUF(buf, ret, "TCI [%s]\n",
			(buffer[0] & 0x1) == 1 ? "enabled" : "disabled");
		WRITE_SYSBUF(buf, ret, "Tap Count [%d]\n",
			(buffer[0] & 0xf8) >> 3);
		WRITE_SYSBUF(buf, ret, "Min InterTap [%d]\n", buffer[1]);
		WRITE_SYSBUF(buf, ret, "Max InterTap [%d]\n", buffer[2]);
		WRITE_SYSBUF(buf, ret, "Touch Slop [%d]\n", buffer[3]);
		WRITE_SYSBUF(buf, ret, "Tap Distance [%d]\n", buffer[4]);
		WRITE_SYSBUF(buf, ret, "Interrupt Delay [%d]\n\n", buffer[6]);
	}

	return ret;
}

static ssize_t store_reg_ctrl(struct i2c_client *client,
	const char *buf, size_t count)
{
	u8 buffer[50] = {0};
	char command[6] = {0};
	int page = 0;
	int reg = 0;
	int offset = 0;
	int value = 0;

	sscanf(buf, "%s %d %d %d %d ", command, &page, &reg, &offset, &value);

	if (!strcmp(command, "write")) {
		synaptics_ts_page_data_read(client, page,
			reg, offset+1, buffer);
		buffer[offset] = (u8)value;
		synaptics_ts_page_data_write(client, page,
			reg, offset+1, buffer);
	} else if (!strcmp(command, "read")) {
		synaptics_ts_page_data_read(client, page,
			reg, offset+1, buffer);
		TOUCH_DBG("page[%d] reg[%d] offset[%d] = 0x%x\n",
			page, reg, offset, buffer[offset]);
	} else {
		TOUCH_DBG("Usage\n");
		TOUCH_DBG("Write page reg offset value\n");
		TOUCH_DBG("Read page reg offset\n");
	}

	return count;
}

static ssize_t show_object_report(struct i2c_client *client, char *buf)
{
	u8 object_report_enable_reg;
	u8 temp[8];

	int ret = 0;
	int i;

	ret = Touch_I2C_Read(client, OBJECT_REPORT_ENABLE_REG, &object_report_enable_reg,
				 sizeof(object_report_enable_reg));
	if (ret < 0)
		return ret;

	for (i = 0; i < 8; i++)
		temp[i] = (object_report_enable_reg >> i) & 0x01;

	WRITE_SYSBUF(buf, ret,
		"\n======= read object_report_enable register =======\n");
	WRITE_SYSBUF(buf, ret,
		" Addr Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0 HEX\n");
	WRITE_SYSBUF(buf, ret,
		"--------------------------------------------------\n");
	WRITE_SYSBUF(buf, ret,
		" 0x%02X %4d %4d %4d %4d %4d %4d %4d %4d 0x%02X\n",
		OBJECT_REPORT_ENABLE_REG, temp[7], temp[6],
		temp[5], temp[4], temp[3], temp[2], temp[1], temp[0],
		object_report_enable_reg);
	WRITE_SYSBUF(buf, ret,
		"--------------------------------------------------\n");
	WRITE_SYSBUF(buf, ret,
		" Bit0  : [F]inger -> %7s\n", temp[0] ? "Enable" : "Disable");
	WRITE_SYSBUF(buf, ret,
		" Bit1  : [S]tylus -> %7s\n", temp[1] ? "Enable" : "Disable");
	WRITE_SYSBUF(buf, ret,
		" Bit2  : [P]alm -> %7s\n", temp[2] ? "Enable" : "Disable");
	WRITE_SYSBUF(buf, ret,
		" Bit3  : [U]nclassified Object -> %7s\n",
		temp[3] ? "Enable" : "Disable");
	WRITE_SYSBUF(buf, ret,
		" Bit4  : [H]overing Finger -> %7s\n",
		temp[4] ? "Enable" : "Disable");
	WRITE_SYSBUF(buf, ret,
		" Bit5  : [G]loved Finger -> %7s\n",
		temp[5] ? "Enable" : "Disable");
	WRITE_SYSBUF(buf, ret,
		" Bit6  : [N]arrow Object Swipe -> %7s\n",
		temp[6] ? "Enable" : "Disable");
	WRITE_SYSBUF(buf, ret,
		" Bit7  : Hand[E]dge  -> %7s\n",
		temp[7] ? "Enable" : "Disable");
	WRITE_SYSBUF(buf, ret,
		"==================================================\n\n");

	return ret;
}

static ssize_t store_object_report(struct i2c_client *client,
	const char *buf, size_t count)
{
	int ret = 0;
	char select[16];
	u8 value = 2;
	int select_cnt;
	int i;
	u8 bit_select = 0;
	u8 object_report_enable_reg_old;
	u8 object_report_enable_reg_new;

	sscanf(buf, "%s %hhu", select, &value);

	if ((strlen(select) > 8) || (value > 1)) {
		TOUCH_DBG("<writing object_report guide>\n");
		TOUCH_DBG("echo [select] [value] > object_report\n");
		TOUCH_DBG("select: [F]inger, [S]tylus, [P]alm, \
			 [U]nclassified Object, [H]overing Finger, \
			 [G]loved Finger, [N]arrow Object Swipe, \
			 Hand[E]dge\n");
		TOUCH_DBG("select length: 1~8, value: 0~1\n");
		TOUCH_DBG("ex) echo F 1 > object_report         (enable [F]inger)\n");
		TOUCH_DBG("ex) echo s 1 > object_report         (enable [S]tylus)\n");
		TOUCH_DBG("ex) echo P 0 > object_report         (disable [P]alm)\n");
		TOUCH_DBG("ex) echo u 0 > object_report         (disable [U]nclassified Object)\n");
		TOUCH_DBG("ex) echo HgNe 1 > object_report      (enable [H]overing Finger, [G]loved Finger, \
			 [N]arrow Object Swipe, Hand[E]dge)\n");
		TOUCH_DBG("ex) echo eNGh 1 > object_report      (enable Hand[E]dge, [N]arrow Object Swipe, \
			 [G]loved Finger, [H]overing Finger)\n");
		TOUCH_DBG("ex) echo uPsF 0 > object_report      (disable [U]nclassified Object, [P]alm, \
			 [S]tylus, [F]inger)\n");
		TOUCH_DBG("ex) echo HguP 0 > object_report      (disable [H]overing Finger, [G]loved Finger, \
			 [U]nclassified Object, [P]alm)\n");
		TOUCH_DBG("ex) echo HFnuPSfe 1 > object_report  (enable all object)\n");
		TOUCH_DBG("ex) echo enghupsf 0 > object_report  (disbale all object)\n");
	} else {
		select_cnt = strlen(select);

		for (i = 0; i < select_cnt; i++) {
			switch ((char)(*(select + i))) {
			case 'F': case 'f': /* (F)inger */
				bit_select |= (0x01 << 0);
				break;
			case 'S': case 's': /* (S)tylus */
				bit_select |= (0x01 << 1);
				break;
			case 'P': case 'p': /* (P)alm */
				bit_select |= (0x01 << 2);
				break;
			case 'U': case 'u': /* (U)nclassified Object */
				bit_select |= (0x01 << 3);
				break;
			case 'H': case 'h': /* (H)overing Filter */
				bit_select |= (0x01 << 4);
				break;
			case 'G': case 'g': /* (G)loved Finger */
				bit_select |= (0x01 << 5);
				break;
			case 'N': case 'n': /* (N)arrow Ojbect Swipe */
				bit_select |= (0x01 << 6);
				break;
			case 'E': case 'e': /* Hand (E)dge */
				bit_select |= (0x01 << 7);
				break;
			default:
				break;
			}
		}

		ret = Touch_I2C_Read(client, OBJECT_REPORT_ENABLE_REG, &object_report_enable_reg_old,
					 sizeof(object_report_enable_reg_old));
		if (ret < 0)
			return count;

		object_report_enable_reg_new = object_report_enable_reg_old;

		if (value > 0)
			object_report_enable_reg_new |= bit_select;
		else
			object_report_enable_reg_new &= ~(bit_select);

		ret = Touch_I2C_Write_Byte(client, OBJECT_REPORT_ENABLE_REG, object_report_enable_reg_new);
		if (ret < 0)
			return count;

	}

	return count;
}

static ssize_t show_use_rmi_dev(struct i2c_client *client, char *buf)
{
	int ret = 0;

	WRITE_SYSBUF(buf, ret, "%u\n", enable_rmi_dev);

	return ret;
}

static ssize_t store_use_rmi_dev(struct i2c_client *client, const char *buf, size_t count)
{
	int ret = 0;
	int value = 0;

	sscanf(buf, "%d", &value);

	if (value < 0 || value > 1) {
		TOUCH_DBG("Invalid enable_rmi_dev value:%d\n", value);
		return count;
	}

	enable_rmi_dev = value;
	TOUCH_DBG("enable_rmi_dev:%u\n", enable_rmi_dev);

	if (enable_rmi_dev && rmidev_fhandler.inserted) {
		if (!rmidev_fhandler.initialized) {
			ret = rmidev_fhandler.exp_fn->init(ts);
			if (ret < 0) {
				TOUCH_ERR("fail to enable_rmi_dev\n");
				return count;
			}

			rmidev_fhandler.initialized = true;
		}
	} else {
		rmidev_fhandler.exp_fn->remove(ts);
		rmidev_fhandler.initialized = false;
	}

	return count;
}

#if defined(ENABLE_SWIPE_MODE)
static ssize_t show_swipe_mode(struct i2c_client *client, char *buf)
{
	int ret = 0;

	WRITE_SYSBUF(buf, ret, "%u\n", get_swipe_mode);

	return ret;
}

static ssize_t store_swipe_mode(struct i2c_client *client,
	const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	get_swipe_mode = value;

	return count;
}
#endif

static LGE_TOUCH_ATTR(version, S_IRUGO | S_IWUSR, show_version, NULL);
static LGE_TOUCH_ATTR(tci, S_IRUGO | S_IWUSR, show_tci, store_tci);
static LGE_TOUCH_ATTR(reg_ctrl, S_IRUGO | S_IWUSR, NULL, store_reg_ctrl);
static LGE_TOUCH_ATTR(object_report, S_IRUGO | S_IWUSR, show_object_report, store_object_report);
static LGE_TOUCH_ATTR(enable_rmi_dev, S_IRUGO | S_IWUSR, show_use_rmi_dev, store_use_rmi_dev);
#if defined(ENABLE_SWIPE_MODE)
static LGE_TOUCH_ATTR(swipe_mode, S_IRUGO | S_IWUSR, show_swipe_mode, store_swipe_mode);
#endif

static struct attribute *S3320_attribute_list[] = {
	&lge_touch_attr_version.attr,
	&lge_touch_attr_tci.attr,
	&lge_touch_attr_reg_ctrl.attr,
	&lge_touch_attr_object_report.attr,
	&lge_touch_attr_enable_rmi_dev.attr,
#if defined(ENABLE_SWIPE_MODE)
	&lge_touch_attr_swipe_mode.attr,
#endif
	NULL,
};

static int S3320_Initialize(struct i2c_client *client)
{
	int ret = 0;

	TOUCH_FUNC();

	/* IMPLEMENT : Device initialization at Booting */
	ts = devm_kzalloc(&client->dev, sizeof(struct synaptics_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		TOUCH_ERR("failed to allocate memory for device driver data\n");
		return TOUCH_FAIL;
	}

	ts->client = client;

	/* read initial page description */
	ret = read_page_description_table(client);
	if (ret == TOUCH_FAIL) {
		if (check_firmware_status(client) == TOUCH_FAIL) {
			int cnt = 0;

			do {
				ret = S3320_UpdateFirmware(client, NULL);
				cnt++;
			} while (ret == TOUCH_FAIL && cnt < 3);

			if (ret == TOUCH_FAIL) {
				devm_kfree(&client->dev, ts);
				return TOUCH_FAIL;
			}
		}
	}

	return TOUCH_SUCCESS;
}

static void S3320_Reset(struct i2c_client *client)
{
#if defined(ENABLE_SWIPE_MODE)
	if (wakeup_by_swipe) {
		wakeup_by_swipe = 0;
		TOUCH_LOG("swipe mode!! no reset\n");
	} else
#endif
	{
		/* IMPLEMENT : Device Reset function */
		if (client != NULL) {
			int ret = 0;
			synaptics_ts_page_data_write_byte(client, ANALOG_PAGE, DYNAMIC_SENSING_CONTROL_REG,
								 SENSING_CONTROL_120HZ);
			if (ret == TOUCH_FAIL)
				TOUCH_WARN("failed to change sensing control to 120Hz\n");
		}

		TouchResetCtrl(0);
		msleep(10);
		TouchResetCtrl(1);
		msleep(150);

		TOUCH_LOG("Device was reset\n");
	}
}


static int S3320_Connect(void)
{
	TOUCH_FUNC();

	/* IMPLEMENT : Device detection function */

	return TOUCH_SUCCESS;
}

static int S3320_InitRegister(struct i2c_client *client)
{
	int ret = 0;
	u8 reg = 0;
	u8 data[2] = {0};

	reg = DEVICE_CONTROL_REG;
	data[0] = DEVICE_CONTROL_NOSLEEP | DEVICE_CONTROL_CONFIGURED;
	ret = Touch_I2C_Write(client, reg, data, 1);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	reg = INTERRUPT_ENABLE_REG;
	data[0] = 0;
	ret = Touch_I2C_Read(client, reg, data, 1);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	reg = INTERRUPT_ENABLE_REG;
	data[0] |= INTERRUPT_MASK_ABS0;
	ret = Touch_I2C_Write(client, reg, data, 1);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	reg = FINGER_REPORT_REG;
	data[0] = 0;
	ret = Touch_I2C_Write(client, reg, data, 2);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	reg = INTERRUPT_STATUS_REG;
	data[0] = 0;
	ret = Touch_I2C_Read(client, reg, data, 1);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	ts->currState = STATE_NORMAL;

	return TOUCH_SUCCESS;
}

static void S3320_ClearInterrupt(struct i2c_client *client)
{
	int ret = 0;

	u8 regDevStatus = 0;
	u8 regIntStatus = 0;

	ret = Touch_I2C_Read_Byte(client, DEVICE_STATUS_REG, &regDevStatus);
	if (ret == TOUCH_FAIL)
		TOUCH_ERR("failed to read device status reg\n");

	ret = Touch_I2C_Read_Byte(client, INTERRUPT_STATUS_REG, &regIntStatus);
	if (ret == TOUCH_FAIL)
		TOUCH_ERR("failed to read interrupt status reg\n");

	if (ret == TOUCH_FAIL)
		TOUCH_ERR("failed to clear interrupt\n");

	return;
}

static int S3320_InterruptHandler(struct i2c_client *client, TouchReadData *pData)
{
	TouchFingerData *pFingerData = NULL;
	int ret = 0;
	u8 i = 0;
	u8 lpwgTapCount = 0;
	u8 readFingerCnt = 0;
	u8 regDevStatus = 0;
	u8 regIntStatus = 0;
	u8 regFingerData[MAX_NUM_OF_FINGERS][NUM_OF_EACH_FINGER_DATA_REG];
	u8 buffer[12][4] = { {0} };

	pData->type = DATA_UNKNOWN;
	pData->count = 0;

	ret = Touch_I2C_Read_Byte(client, DEVICE_STATUS_REG, &regDevStatus);
	if (ret == TOUCH_FAIL) {
		TOUCH_WARN("failed to read device status\n");
		return TOUCH_FAIL;
	}

	ret = Touch_I2C_Read_Byte(client, INTERRUPT_STATUS_REG, &regIntStatus);
	if (ret == TOUCH_FAIL) {
		TOUCH_WARN("failed to read interrupt status\n");
		return TOUCH_FAIL;
	}

	if (regIntStatus == INTERRUPT_MASK_NONE) {
		return TOUCH_SUCCESS;
	} else if (regIntStatus & INTERRUPT_MASK_ABS0) {
		readFingerCnt = get_object_count(client);

		if (readFingerCnt == 0)
			readFingerCnt = MAX_NUM_OF_FINGERS;

		if (readFingerCnt > 0) {
			ret = Touch_I2C_Read(client, FINGER_DATA_REG_START,
				(u8 *)regFingerData, (NUM_OF_EACH_FINGER_DATA_REG * readFingerCnt));

			if (ret == TOUCH_SUCCESS) {
				pData->type = DATA_FINGER;
				for (i = 0; i < readFingerCnt; i++) {
					if (regFingerData[i][0]) {
						pFingerData = &pData->fingerData[pData->count];
						pFingerData->id = i;
						pFingerData->type = regFingerData[i][REG_OBJECT];
						pFingerData->x = GET_X_POSITION(regFingerData[i][REG_X_MSB],
										 regFingerData[i][REG_X_LSB]);
						pFingerData->y = GET_Y_POSITION(regFingerData[i][REG_Y_MSB],
										 regFingerData[i][REG_Y_LSB]);
						pFingerData->width_major = GET_WIDTH_MAJOR(regFingerData[i][REG_WX],
											 regFingerData[i][REG_WY]);
						pFingerData->width_minor = GET_WIDTH_MINOR(regFingerData[i][REG_WX],
										 regFingerData[i][REG_WY]);
						pFingerData->orientation = GET_ORIENTATION(regFingerData[i][REG_WY],
										 regFingerData[i][REG_WX]);
						pFingerData->pressure = GET_PRESSURE(regFingerData[i][REG_Z]);

						pData->count++;
					}
				}
			} else {
				pData->type = DATA_UNKNOWN;
				TOUCH_ERR("Unexpected ABS Interrupt Data\n");
				return TOUCH_FAIL;
			}
		} else {
			pData->type = DATA_FINGER;
		}


	} else if (regIntStatus & INTERRUPT_MASK_CUSTOM) {
		u8 status = 0;

		ret |= synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_STATUS_REG, 1, &status);

		if (status & 0x1) { /* TCI-1 : Double-Tap */
			TOUCH_LOG("Knock-on Detected\n");
			pData->type = DATA_KNOCK_ON;
		} else if (status & 0x2) {  /* TCI-2 : Multi-Tap */
			TOUCH_LOG("Knock-code Detected\n");
			pData->type = DATA_KNOCK_CODE;

			ret |= synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_DATA_REG, 4*ts->lpwgSetting.tapCount, &buffer[0][0]);

			for (i = 0; i < ts->lpwgSetting.tapCount; i++) {
				pData->knockData[i].x
					= GET_X_POSITION(buffer[i][1], buffer[i][0]);
				pData->knockData[i].y
					= GET_Y_POSITION(buffer[i][3], buffer[i][2]);
			}

			pData->count = ts->lpwgSetting.tapCount;
#if defined(ENABLE_SWIPE_MODE)
		} else if (status & 0x4) {
			u16 swipe_start_x = 0;
			u16 swipe_start_y = 0;
			u16 swipe_end_x = 0;
			u16 swipe_end_y = 0;
			u16 swipe_time = 0;
			u8 buf_lsb = 0;
			u8 buf_msb = 0;
			u8 fail_reason = 0;

			ret |= Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, LPWG_PAGE);

			ret |= Touch_I2C_Read_Byte(client, SWIPE_FAIL_REASON_REG, &fail_reason);

			ret |= Touch_I2C_Read_Byte(client, SWIPE_COOR_START_X_LSB_REG, &buf_lsb);
			ret |= Touch_I2C_Read_Byte(client, SWIPE_COOR_START_X_MSB_REG, &buf_msb);
			swipe_start_x = (buf_msb << 8) | buf_lsb;

			ret |= Touch_I2C_Read_Byte(client, SWIPE_COOR_START_Y_LSB_REG, &buf_lsb);
			ret |= Touch_I2C_Read_Byte(client, SWIPE_COOR_START_Y_MSB_REG, &buf_msb);
			swipe_start_y = (buf_msb << 8) | buf_lsb;

			ret |= Touch_I2C_Read_Byte(client, SWIPE_COOR_END_X_LSB_REG, &buf_lsb);
			ret |= Touch_I2C_Read_Byte(client, SWIPE_COOR_END_X_MSB_REG, &buf_msb);
			swipe_end_x = (buf_msb << 8) | buf_lsb;

			ret |= Touch_I2C_Read_Byte(client, SWIPE_COOR_END_Y_LSB_REG, &buf_lsb);
			ret |= Touch_I2C_Read_Byte(client, SWIPE_COOR_END_Y_MSB_REG, &buf_msb);
			swipe_end_y = (buf_msb << 8) | buf_lsb;

			ret |= Touch_I2C_Read_Byte(client, SWIPE_TIME_LSB_REG, &buf_lsb);
			ret |= Touch_I2C_Read_Byte(client, SWIPE_TIME_MSB_REG, &buf_msb);
			swipe_time = (buf_msb << 8) | buf_lsb;

			ret |= Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);

			if (lockscreen_stat) {
				wakeup_by_swipe = 1;
				swipe_control(client, 0);
			}

			pData->count = 1;
			pData->knockData[0].x = swipe_end_x;
			pData->knockData[0].y = swipe_end_y;

#if defined(ENABLE_SWIPE_UP)
			if (fail_reason & 0x03) {
				TOUCH_LOG("Swipe up - Detected\n");
				pData->type = DATA_SWIPE_UP;
			} else {
				TOUCH_LOG("Swipe down - Detected\n");
				pData->type = DATA_SWIPE_DOWN;
			}

			TOUCH_LOG("Swipe Fail reason : 0x%x\n", (fail_reason & 0xfc) >> 2);
#else
			TOUCH_LOG("Swipe down - Detected\n");
			pData->type = DATA_SWIPE_DOWN;
			TOUCH_LOG("Swipe Fail reason : 0x%x\n", fail_reason);
#endif

			TOUCH_LOG("LPWG Swipe Gesture: start(%4d,%4d) end(%4d,%4d)\n", swipe_start_x, swipe_start_y, swipe_end_x, swipe_end_y);
#endif
		} else if (status == 0) {
			u8 readFailReason = 0;
			u8 knock_on_fail = 0;
			u8 knock_code_fail = 0;

			ret |= synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_FAIL_REASON, 1, &readFailReason);
			knock_on_fail = readFailReason & 0x0f;
			knock_code_fail = (readFailReason & 0xf0) >> 4;

			if (knock_on_fail) {
				TOUCH_LOG("[0x%x] Knock-on Fail reason\n", knock_on_fail);
				pData->type = DATA_LPWG_FAIL;
			} else if (knock_code_fail) {
				TOUCH_LOG("[0x%x] Knock-code Fail reason\n", knock_code_fail);
				pData->type = DATA_LPWG_FAIL;
			} else {
				ret |= synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_OVER_TAPCOUNT, 1, &lpwgTapCount);

				if (ts->lpwgSetting.tapCount < lpwgTapCount) {
					TOUCH_LOG("OverTap Knock-code Detected\n");

					pData->knockData[0].x = 1;
					pData->knockData[0].y = 1;
					pData->knockData[1].x = -1;
					pData->knockData[1].y = -1;
					pData->count = ts->lpwgSetting.tapCount;
					pData->type = DATA_KNOCK_CODE;
				}
			}
		} else {
			TOUCH_ERR("Unexpected LPWG Interrupt Status ( 0x%X )\n", status);
		}

		if (ret != TOUCH_SUCCESS) {
			TOUCH_ERR("Unexpected Custom Interrupt Data\n");
			return TOUCH_FAIL;
		}
	} else {
		TOUCH_ERR("Unexpected Interrupt Status ( 0x%X )\n", regIntStatus);
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}


/******************************************************************
* module maker : ELK(0), Suntel(1), Tovis(2), Innotek(3), JDI(4), LGD(5)
*
*
*
*
********************************************************************/
static int S3320_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	u8 readData[FW_VER_INFO_NUM] = {0};

	TOUCH_FUNC();

	ret = Touch_I2C_Read(client, FLASH_CONFIG_ID_REG, readData, FW_VER_INFO_NUM);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	TOUCH_LOG("Maker=%d, Key=%d, Supplier=%d\n", (readData[0] >> 4) & 0xF, readData[0] & 0xF,
			 (readData[1] >> 4) & 0x0F);
	TOUCH_LOG("Panel=%d, Size =%d.%d [Inch]\n", readData[2] & 0xF, readData[1] & 0xF, (readData[2] >> 4) & 0xF);
	TOUCH_LOG("Version=%d ( %s )\n", readData[3] & 0x7F,
			 ((readData[3] & 0x80) == 0x80) ? "Official Release" : "Test Release");

	pFwInfo->moduleMakerID = (readData[0] >> 4) & 0x0F;
	pFwInfo->moduleVersion = readData[2] & 0x0F;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = (readData[3] >> 7) & 0x01;
	pFwInfo->version = readData[3] & 0x7F;

	return TOUCH_SUCCESS;

}

static int S3320_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	char *pFwFilename = NULL;
	u8 *pReadData = NULL;

	TOUCH_FUNC();

	if (pFilename == NULL)
		pFwFilename = (char *)defaultFirmware;
	else
		pFwFilename = pFilename;

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if (ret) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	pBin = (u8 *)(fw->data);
	pReadData = &pBin[0x16d00];

	TOUCH_LOG("Maker=%d, Key=%d, Supplier=%d\n", (pReadData[0] >> 4) & 0xF, pReadData[0] & 0xF,
			 (pReadData[1] >> 4) & 0x0F);
	TOUCH_LOG("Panel=%d, Size =%d.%d [Inch]\n", pReadData[2] & 0xF, pReadData[1] & 0xF,
			 (pReadData[2] >> 4) & 0xF);
	TOUCH_LOG("Version=%d ( %s )\n", pReadData[3] & 0x7F, ((pReadData[3] & 0x80) == 0x80) ?
			 "Official Release" : "Test Release");

	pFwInfo->moduleMakerID = (pReadData[0] >> 4) & 0x0F;
	pFwInfo->moduleVersion = pReadData[2] & 0x0F;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = (pReadData[3] >> 7) & 0x01;
	pFwInfo->version = pReadData[3] & 0x7F;

	/* Free firmware image buffer */
	release_firmware(fw);

	return TOUCH_SUCCESS;
}


static int S3320_UpdateFirmware(struct i2c_client *client, char *pFilename)
{
	int ret = 0;
	char *pFwFilename = NULL;

	TOUCH_FUNC();

	if (pFilename == NULL)
		pFwFilename = (char *)defaultFirmware;
	else
		pFwFilename = pFilename;

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	ret = synaptics_ts_page_data_write_byte(client, ANALOG_PAGE, DYNAMIC_SENSING_CONTROL_REG,
							 SENSING_CONTROL_120HZ);
	if (ret == TOUCH_FAIL) {
		TOUCH_ERR("failed to change sensing control to 120Hz\n");
		return TOUCH_FAIL;
	}

	FirmwareUpgrade(ts, pFwFilename);

	/* read changed page description */
	ret = read_page_description_table(client);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	ret = check_firmware_status(client);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	return TOUCH_SUCCESS;
}

static int S3320_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int ret = TOUCH_SUCCESS;

	TOUCH_FUNC();

	memcpy(&ts->lpwgSetting, pLpwgSetting, sizeof(LpwgSetting));

#if defined(ENABLE_SWIPE_MODE)
	if (ts->lpwgSetting.coverState)
		swipe_control(client, 0);
#endif

	if (ts->currState == newState) {
		TOUCH_LOG("device state is same as driver requested\n");
		return TOUCH_SUCCESS;
	}

	if ((newState < STATE_NORMAL) && (newState > STATE_KNOCK_ON_CODE)) {
		TOUCH_LOG("invalid request state ( state = %d )\n", newState);
		return TOUCH_FAIL;
	}

	ret = lpwg_control(client, newState);
	if (ret == TOUCH_FAIL) {
		TOUCH_ERR("failed to set lpwg mode in device\n");
		return TOUCH_FAIL;
	}

	if (ret == TOUCH_SUCCESS)
		ts->currState = newState;

	switch (newState) {
	case STATE_NORMAL:
		TOUCH_LOG("device was set to NORMAL\n");
		break;
	case STATE_OFF:
		TOUCH_LOG("device was set to OFF\n");
		break;
	case STATE_KNOCK_ON_ONLY:
		TOUCH_LOG("device was set to KNOCK_ON_ONLY\n");
		break;
	case STATE_KNOCK_ON_CODE:
		TOUCH_LOG("device was set to KNOCK_ON_CODE\n");
		break;
	default:
		TOUCH_LOG("impossilbe state ( state = %d )\n", newState);
		ret = TOUCH_FAIL;
		break;
	}

	return TOUCH_SUCCESS;

}

static int S3320_DoSelfDiagnosis(struct i2c_client *client, int *pRawStatus, int *pChannelStatus, char *pBuf,
					 int bufSize, int *pDataLen)
{
	int ret = 0;
	int dataLen = 0;

	/* CAUTION : be careful not to exceed buffer size */

	int full_raw_cap;
	int trx_to_trx;
	int high_resistance;

	TOUCH_FUNC();

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, ANALOG_PAGE);
	if (ret == TOUCH_FAIL)
		return TOUCH_FAIL;

	full_raw_cap = F54TestHandle(ts, F54_FULL_RAW_CAP, 0, pBuf);
	high_resistance = F54TestHandle(ts, F54_HIGH_RESISTANCE, 0, pBuf);
	trx_to_trx = F54TestHandle(ts, F54_TRX_TO_TRX, 0, pBuf);

	*pRawStatus = full_raw_cap;

	if (high_resistance == TOUCH_SUCCESS && trx_to_trx == TOUCH_SUCCESS)
		*pChannelStatus = TOUCH_SUCCESS;
	else
		*pChannelStatus = TOUCH_FAIL;

	*pDataLen = dataLen;

	return TOUCH_SUCCESS;
}

static int S3320_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
{
	int ret = 0;

	switch (cmd) {
	case READ_IC_REG:
		ret = Touch_I2C_Read_Byte(client, (u8)reg, (u8 *)pValue);
		if (ret == TOUCH_FAIL)
			return TOUCH_FAIL;
		break;

	case WRITE_IC_REG:
		ret = Touch_I2C_Write_Byte(client, (u8)reg, (u8)*pValue);
		if (ret == TOUCH_FAIL)
			return TOUCH_FAIL;
		break;

	default:
		TOUCH_ERR("Invalid access command ( cmd = %d )\n", cmd);
		return TOUCH_FAIL;
		break;
	}

	return TOUCH_SUCCESS;

}

static void S3320_NotifyHandler(struct i2c_client *client, TouchNotify notify, int data)
{
	switch (notify) {
	case NOTIFY_CALL:
		TOUCH_LOG("Call was notified ( data = %d )\n", data);
		break;

	case NOTIFY_Q_COVER:
		TOUCH_LOG("Quick Cover was notified ( data = %d )\n", data);
		break;

	case NOTIFY_FPS_CHANGED:
#if 0 /* this code is not confirmed yet from synaptics */
	if (data == 0) {
			synaptics_ts_page_data_write_byte(client, ANALOG_PAGE,
								DYNAMIC_SENSING_CONTROL_REG,  SENSING_CONTROL_120HZ);
		} else {
			synaptics_ts_page_data_write_byte(client, ANALOG_PAGE,
								DYNAMIC_SENSING_CONTROL_REG,  SENSING_CONTROL_AUTO);
		}
#endif
		break;
	case NOTIFY_TA_STATUS:
		synaptics_ts_page_data_write_byte(client, LPWG_PAGE, FILTER_SMALL_Z_CTRL, (u8) data);
		break;

	default:
		TOUCH_ERR("Invalid notification ( notify = %d )\n", notify);
		break;
	}

	return;

}

TouchDeviceSpecificFunction S3320_Func = {

	.Initialize = S3320_Initialize,
	.Reset = S3320_Reset,
	.Connect = S3320_Connect,
	.InitRegister = S3320_InitRegister,
	.ClearInterrupt = S3320_ClearInterrupt,
	.InterruptHandler = S3320_InterruptHandler,
	.ReadIcFirmwareInfo = S3320_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = S3320_GetBinFirmwareInfo,
	.UpdateFirmware = S3320_UpdateFirmware,
	.SetLpwgMode = S3320_SetLpwgMode,
	.DoSelfDiagnosis = S3320_DoSelfDiagnosis,
	.AccessRegister = S3320_AccessRegister,
	.NotifyHandler = S3320_NotifyHandler,
	.device_attribute_list = S3320_attribute_list,

};


/* End Of File */


