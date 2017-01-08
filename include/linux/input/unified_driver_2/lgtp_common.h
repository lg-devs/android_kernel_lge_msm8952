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
 *    File  	: lgtp_common.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined ( _LGTP_COMMON_H_ )
#define _LGTP_COMMON_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/
#include <linux/types.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/jiffies.h>
#include <linux/version.h>

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/mutex.h>
#include <linux/async.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <linux/firmware.h>

#include <linux/syscalls.h>
#include <linux/string.h>

#if defined ( CONFIG_HAS_EARLYSUSPEND )
#include <linux/earlysuspend.h>
#elif defined (CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 4, 67))
#define KERNEL_ABOVE_3_4_67
#endif

#include <linux/input/unified_driver_2/lgtp_project_setting.h>

#if defined ( TOUCH_PLATFORM_MTK )
#include <mach/wd_api.h>
#include <mach/eint.h>
#include <mach/mt_wdt.h>
#include <mach/mt_gpt.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <cust_eint.h>
#include "tpd.h"
#endif

#if defined ( TOUCH_PLATFORM_QCT )
#if !defined (TOUCH_PLATFORM_MSM8936)
#include <mach/board.h>
#endif
#include <mach/board_lge.h>
#endif

//==========================================================
// Driver Capability
//==========================================================
#define MAX_DEVICE_SUPPORT 5

#define MAX_FINGER	10
#define MAX_KEY		4
#define MAX_KNOCK	16

#define MAX_FILENAME 256

#define MAX_FW_UPGRADE_RETRY 3

/****************************************************************************
* Type Definitions
****************************************************************************/
enum {
	TOUCH_FALSE = 0,
	TOUCH_TRUE,
};

enum {
	TOUCH_SUCCESS = 0,
	TOUCH_FAIL = -1,
};

enum {
	MT_PROTOCOL_A = 0,
	MT_PROTOCOL_B,
};

enum {
	KEY_RELEASED = 0,
	KEY_PRESSED,
	KEY_CANCELED = 0xFF,
};

enum {
	HOVER_FAR = 0,
	HOVER_NEAR,
};

enum {
	UEVENT_IDLE = 0,
	UEVENT_BUSY,
};

enum {
	READ_IC_REG = 0,
	WRITE_IC_REG,
};

enum {
	BOOT_NORMAL = 0,
	BOOT_OFF_CHARGING,
	BOOT_MINIOS,
};

enum {
	PM_RESUME = 0,
	PM_SUSPEND,
	PM_SUSPEND_IRQ,
};

typedef enum
{
	STATE_UNKNOWN = 0,
	
	STATE_BOOT,
	STATE_NORMAL,
	STATE_OFF,
	STATE_KNOCK_ON_ONLY,
	STATE_KNOCK_ON_CODE,
	STATE_NORMAL_HOVER,
	STATE_HOVER,
	STATE_UPDATE_FIRMWARE,
	STATE_SELF_DIAGNOSIS,

} TouchState;

#if defined(ENABLE_GHOST_DETECT_SOLUTION)
enum{
	EX_INIT,
	EX_FIRST_INT,
	EX_PREV_PRESS,
	EX_CURR_PRESS,
	EX_FIRST_GHOST_DETECT,
	EX_SECOND_GHOST_DETECT,
	EX_CURR_INT,
	EX_PROFILE_MAX
};
enum{
	REBASE_DONE,
	REBASE_NEEDED,
	REBASE_DOING
};
#endif

typedef struct TouchModelConfigTag
{
	u32	button_support;
	u32	number_of_button;
	u32	button_name[MAX_KEY];
	u32	protocol_type;
	u32	max_x;
	u32	max_y;
	u32	max_pressure;
	u32	max_width;
	u32	max_orientation;
	u32	max_id;
	
} TouchModelConfig;

typedef struct TouchDeviceInfoTag
{
	u16 moduleMakerID;
	u16 moduleVersion;
	u16 modelID;
	u16 isOfficial;
	u16 version;
	
} TouchFirmwareInfo;

typedef enum
{
	LPWG_CMD_UNKNOWN = 0,
	
	LPWG_CMD_MODE = 1,
	LPWG_CMD_LCD_PIXEL_SIZE = 2,
	LPWG_CMD_ACTIVE_TOUCH_AREA = 3,
	LPWG_CMD_TAP_COUNT = 4,
	LPWG_CMD_TAP_DISTANCE = 5,
	LPWG_CMD_LCD_STATUS = 6,
	LPWG_CMD_PROXIMITY_STATUS = 7,
	LPWG_CMD_FIRST_TWO_TAP = 8,
	LPWG_CMD_UPDATE_ALL = 9,
	LPWG_CMD_CALL = 10,
	
} LpwgCmd;

typedef struct LpwgSettingTag
{
	unsigned int mode; 		/* LPWG mode ( Bit field ) : 0 = disable, 1 = knock-on, 2 = knock-code, 4 = Signature(Not Used) */
	unsigned int lcdPixelSizeX;
	unsigned int lcdPixelSizeY;
	unsigned int activeTouchAreaX1;
	unsigned int activeTouchAreaX2;
	unsigned int activeTouchAreaY1;
	unsigned int activeTouchAreaY2;
	unsigned int tapCount; 	/* knock-code length */
	unsigned int isFirstTwoTapSame; 	/* 1 = first two code is same, 0 = not same */
	unsigned int lcdState; 		/* 1 = LCD ON, 0 = OFF */
	unsigned int proximityState; 	/* 1 = Proximity FAR, 0 = NEAR */
	unsigned int coverState; 	/* 1 = Cover Close, 0 = Open*/
	unsigned int callState; 	/* 0 = Call End, 1 = Ringing, 2 = Call Received */
	
} LpwgSetting;


typedef enum
{
	DATA_UNKNOWN = 0,
		
	DATA_FINGER,
	DATA_KEY,
	DATA_KNOCK_ON,
	DATA_KNOCK_CODE,
	DATA_HOVER,
#if defined(ENABLE_SWIPE_MODE)
	DATA_SWIPE,
#endif
#if defined(ENABLE_REALTIME_LPWG_FAIL_REASON)
	DATA_LPWG_FAIL,
#endif
} TouchDataType;

typedef struct TouchFingerDataTag
{
	u16	id; /* finger ID */
	u16	x;
	u16	y;
	u16	width_major;
	u16	width_minor;
	u16	orientation;
	u16	pressure; /* 0=Hover / 1~MAX-1=Finger / MAX=Palm */
	u16	type; /* finger, palm, pen, glove, hover */
	
} TouchFingerData;

typedef struct TouchKeyDataTag
{
	u16	index; 		/* key index ( should be assigned from 1 ) */
	u16	pressed; 	/* 1 means pressed, 0 means released */
	
} TouchKeyData;

typedef struct TouchPointTag
{
	int x;
	int y;
	
} TouchPoint;

typedef struct TouchReadDataTag
{
	TouchDataType type;
	u16 count;
	TouchFingerData fingerData[MAX_FINGER+1];
	TouchKeyData keyData;
	TouchPoint knockData[MAX_KNOCK+1];
	int hoverState;
	
} TouchReadData;

typedef struct TouchReportDataTag
{
	u32 finger; 	/* bit field for each finger : 1 means pressed, 0 means released */
	u32 key; 	/* key index ( 0 means all key was released ) */
	u32 knockOn; 	/* 1 means event sent, 0 means event processed by CFW */
	u32 knockCode; 	/* 1 means event sent, 0 means event processed by CFW */
	u32 hover;	/* 0 means far, near means 1 */
	u32 knockCount;
	TouchPoint knockData[MAX_KNOCK+1];
	
} TouchReportData;

#if defined(ENABLE_NOISE_LOG)
enum{
	TS_NOISE_LOG_DISABLE = 0,
	TS_NOISE_LOG_ENABLE,
};

enum{
	MENU_OUT = 0,
	MENU_ENTER,
};

typedef struct NoiseStateTag {
	u8	noise_log_flag;
	u8	check_noise_menu;
} NoiseState;
#endif

typedef struct TouchDriverDataTag
{
	struct input_dev		*input_dev;
	struct i2c_client		*client;
	struct kobject 		lge_touch_kobj;

	#if defined ( CONFIG_HAS_EARLYSUSPEND )
	struct early_suspend	early_suspend;
	#elif defined ( CONFIG_FB )
	struct notifier_block	fb_notif;
	#endif

	#if defined(TOUCH_TYPE_INCELL)
	struct notifier_block	lcd_notif;
	#endif

	struct delayed_work		work_upgrade;
	struct delayed_work 	work_irq;
	struct delayed_work 	work_init;
	struct delayed_work		work_power_off_charging;
	struct mutex			thread_lock;
	struct wake_lock		lpwg_wake_lock;

	int isSuspend;
	int bootMode;
	int fpsChanged;

	TouchState currState;
	TouchState nextState;

	TouchModelConfig mConfig;

	LpwgSetting lpwgSetting;

	TouchFirmwareInfo icFwInfo;
	TouchFirmwareInfo binFwInfo;

	int useDefaultFirmware;
	char fw_image[MAX_FILENAME];

	TouchReportData reportData;
#if defined(ENABLE_GHOST_DETECT_SOLUTION)
	TouchFingerData prevFingerData[MAX_FINGER+1];
	atomic_t needToRebase;
#endif

#if defined(ENABLE_NOISE_LOG)
	NoiseState noiseState;
#endif

} TouchDriverData;

typedef struct TouchDeviceSpecificFuncTag {

	int (*Initialize)(struct i2c_client *client);
	void (*Reset)(struct i2c_client *client);
	int (*Connect)(void);
	int (*InitRegister)(struct i2c_client *client);
	void (*ClearInterrupt)(struct i2c_client *client);
	int (*InterruptHandler)(struct i2c_client *client, TouchReadData *pData);
	int (*ReadIcFirmwareInfo)(struct i2c_client *client, TouchFirmwareInfo *pFwInfo);
	int (*GetBinFirmwareInfo)(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo);
	int (*UpdateFirmware)(struct i2c_client *client, char *pFilename);
	int (*SetLpwgMode)(struct i2c_client *client, TouchState newState, LpwgSetting *pLpwgSetting);
	int (*DoSelfDiagnosis)(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen);
	int (*AccessRegister)(struct i2c_client *client, int cmd, int reg, int *pValue);
	struct attribute **device_attribute_list;
#if defined(ENABLE_GHOST_DETECT_SOLUTION)
	int (*rebase_ic)(struct i2c_client *client, TouchReadData *pData, atomic_t *needToRebase);
#endif

} TouchDeviceSpecificFunction;


/* sysfs
 *
 */
struct lge_touch_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct i2c_client *client, char *buf);
	ssize_t (*store)(struct i2c_client *client,
		const char *buf, size_t count);
};

#define LGE_TOUCH_ATTR(_name, _mode, _show, _store)	\
struct lge_touch_attribute lge_touch_attr_##_name 	\
	= __ATTR(_name, _mode, _show, _store)



/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/
#define LGTP_DEBUG 1

#define LGTP_TAG "[TOUCH]"

/* LGTP_MODULE : will be defined in each c-files */
#if defined ( LGTP_DEBUG )
#define TOUCH_ERR(fmt, args...)		printk(KERN_ERR LGTP_TAG"[E]"LGTP_MODULE" %s() line=%d : "fmt, __FUNCTION__, __LINE__, ##args)
#define TOUCH_WARN(fmt, args...)		printk(KERN_ERR LGTP_TAG"[W]"LGTP_MODULE" %s() line=%d : "fmt, __FUNCTION__, __LINE__, ##args)
#define TOUCH_LOG(fmt, args...)		printk(KERN_ERR LGTP_TAG"[L]"LGTP_MODULE" " fmt, ##args)
#define TOUCH_DBG(fmt, args...)		printk(KERN_ERR LGTP_TAG"[D]"LGTP_MODULE" " fmt, ##args)
#define TOUCH_FUNC(f)  	   			printk(KERN_ERR LGTP_TAG"[F]"LGTP_MODULE" %s()\n", __FUNCTION__)
#define TOUCH_FUNC_EXT(fmt, args...) 	printk(KERN_ERR LGTP_TAG"[F]"LGTP_MODULE" %s() : "fmt, __FUNCTION__, ##args)
#else
#define TOUCH_ERR(fmt, args...)		printk(KERN_ERR LGTP_TAG"[E]"LGTP_MODULE" %s() line=%d : "fmt, __FUNCTION__, __LINE__, ##args)
#define TOUCH_WARN(fmt, args...)		printk(KERN_WARNING LGTP_TAG"[W]"LGTP_MODULE" %s() line=%d : "fmt, __FUNCTION__, __LINE__, ##args)
#define TOUCH_LOG(fmt, args...)		printk(KERN_INFO LGTP_TAG"[L]"LGTP_MODULE" " fmt, ##args)
#define TOUCH_DBG(fmt, args...)		printk(KERN_DEBUG LGTP_TAG"[D]"LGTP_MODULE" " fmt, ##args)
#define TOUCH_FUNC(f)  	   			printk(KERN_INFO LGTP_TAG"[F]"LGTP_MODULE" %s()\n", __FUNCTION__)
#define TOUCH_FUNC_EXT(fmt, args...) 	printk(KERN_INFO LGTP_TAG"[F]"LGTP_MODULE" %s() : "fmt, __FUNCTION__, ##args)
#endif

/****************************************************************************
* Global Function Prototypes
****************************************************************************/



#endif /* _LGTP_COMMON_H_ */


/* End Of File */

