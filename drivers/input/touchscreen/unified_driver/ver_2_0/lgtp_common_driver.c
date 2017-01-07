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
 *    File  	: lgtp_common_driver.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description : 
 *
 ***************************************************************************/
#define LGTP_MODULE "[COMMON]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>

#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>
#include <linux/input/unified_driver_2/lgtp_model_config.h>


/****************************************************************************
* Local Function Prototypes
****************************************************************************/
static void Device_Touch_Release(struct device *dev);


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define LGE_TOUCH_NAME "lge_touch"


/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/
enum
{
	UEVENT_KNOCK_ON = 0,
	UEVENT_KNOCK_CODE,
	UEVENT_SIGNATURE,
#if defined(ENABLE_SWIPE_MODE)
	UEVENT_SWIPE,
#endif
	NUM_OF_UEVENT
};

int lockscreen_stat = 0;
int debug_abs = 0;
#if defined ( TOUCH_PLATFORM_QCT ) && !defined ( TOUCH_DEVICE_LU202X )
atomic_t pm_state;
#endif
/****************************************************************************
* Variables
****************************************************************************/
static TouchDeviceSpecificFunction *pDeviceSpecificFunc;

struct workqueue_struct* touch_wq;

struct mutex* pMutexTouch;
struct wake_lock* pWakeLockTouch;
struct delayed_work* pWorkTouch;

#define MAX_ATTRIBUTE_ARRAY_SIZE 		30

static char *touch_uevent[NUM_OF_UEVENT][2] = {
	{"TOUCH_GESTURE_WAKEUP=WAKEUP", NULL},
	{"TOUCH_GESTURE_WAKEUP=PASSWORD", NULL},
	{"TOUCH_GESTURE_WAKEUP=SIGNATURE", NULL},
#if defined(ENABLE_SWIPE_MODE)
	{"TOUCH_GESTURE_WAKEUP=SWIPE", NULL}
#endif
};

static struct bus_type touch_subsys = {
	.name = LGE_TOUCH_NAME,
	.dev_name = "lge_touch",
};

static struct device device_touch = {
	.id    = 0,
	.bus   = &touch_subsys,
	.release = Device_Touch_Release,
};



#if defined ( TOUCH_PLATFORM_MTK )
static struct i2c_board_info __initdata i2c_tpd = {I2C_BOARD_INFO(LGE_TOUCH_NAME, 0x20)};
#endif

#if defined(ENABLE_SWIPE_MODE)
extern int wakeup_by_swipe;
#endif


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/



/****************************************************************************
* Local Functions
****************************************************************************/
static void Device_Touch_Release(struct device *dev)
{
    return;
}
static void SetDriverState(TouchDriverData *pDriverData, TouchState newState )
{
	int ret = 0;

	if( newState == STATE_BOOT ) {
		TOUCH_LOG("STATE = BOOT\n");
		pDriverData->isSuspend = TOUCH_FALSE;
		pDriverData->lpwgSetting.lcdState = 1;
		pDriverData->lpwgSetting.mode = -1; /* if there is no CFW, driver will remain NORMAL state. ( LAF / FOTA ) */
	} else if( newState == STATE_NORMAL ) {
		TOUCH_LOG("STATE = NORMAL\n");
	} else if ( newState == STATE_OFF ) {
		TOUCH_LOG("STATE = OFF\n");
	} else if ( newState == STATE_KNOCK_ON_ONLY ) {
		TOUCH_LOG("STATE = KNOCK_ON_ONLY\n");
	} else if ( newState == STATE_KNOCK_ON_CODE ) {
		TOUCH_LOG("STATE = KNOCK_ON_CODE\n");
	} else if ( newState == STATE_NORMAL_HOVER ) {
		TOUCH_LOG("STATE = NORMAL_HOVER\n");
	} else if ( newState == STATE_HOVER ) {
		TOUCH_LOG("STATE = HOVER\n");
	} else if ( newState == STATE_UPDATE_FIRMWARE ) {
		TOUCH_LOG("STATE = UPDATE_FIRMWARE\n");
	} else if ( newState == STATE_SELF_DIAGNOSIS ) {
		TOUCH_LOG("STATE = SELF_DIAGNOSIS\n");
	} else if ( newState == STATE_UNKNOWN ) {
		TOUCH_LOG("STATE = UNKNOWN\n");
	} else {
		TOUCH_WARN("Invalid state ( %d )\n", newState);
		ret = TOUCH_FAIL;
	}

	if( ret == TOUCH_SUCCESS ) {
		pDriverData->currState = newState;
		pDriverData->nextState = pDriverData->currState;
	}

}

static void report_finger(TouchDriverData *pDriverData, TouchReadData *pReadData)
{
	u32 reportedFinger = pDriverData->reportData.finger;

	u32 newFinger = 0;
	u32 changedFinger = 0;
	u32 pressedFinger = 0;
	u32 releasedFinger = 0;
	int i = 0;
	int j = 0;

	for( i=0 ; i < pReadData->count ; i++ )
	{
		input_mt_slot(pDriverData->input_dev, pReadData->fingerData[i].id);

		input_report_abs(pDriverData->input_dev, ABS_MT_TRACKING_ID, pReadData->fingerData[i].id);
		input_report_abs(pDriverData->input_dev, ABS_MT_POSITION_X, pReadData->fingerData[i].x);
		input_report_abs(pDriverData->input_dev, ABS_MT_POSITION_Y, pReadData->fingerData[i].y);
		input_report_abs(pDriverData->input_dev, ABS_MT_PRESSURE, pReadData->fingerData[i].pressure);
		input_report_abs(pDriverData->input_dev, ABS_MT_WIDTH_MAJOR, pReadData->fingerData[i].width_major);
		input_report_abs(pDriverData->input_dev, ABS_MT_WIDTH_MINOR, pReadData->fingerData[i].width_minor);
		input_report_abs(pDriverData->input_dev, ABS_MT_ORIENTATION, pReadData->fingerData[i].orientation);
		if(debug_abs){
			TOUCH_DBG("<%d> pos[%4d,%4d] w_m[%2d] w_n[%2d] o[%2d] p[%3d]\n",
				pReadData->fingerData[i].id, pReadData->fingerData[i].x, pReadData->fingerData[i].y,
				pReadData->fingerData[i].width_major, pReadData->fingerData[i].width_minor,
				pReadData->fingerData[i].orientation, pReadData->fingerData[i].pressure);
		}
	}

	for(i=0 ; i < pReadData->count ; i++) {
		newFinger |= 1 << pReadData->fingerData[i].id ;
	}

	changedFinger = reportedFinger ^ newFinger;
	pressedFinger = newFinger & changedFinger;
	releasedFinger = reportedFinger & changedFinger;

	for(i=0 ; i < MAX_FINGER ; i++)
	{
		if((pressedFinger >> i) & 0x1) {
			for (j = 0; j < pReadData->count; j++) {
				if (pReadData->fingerData[j].id == i) {
					if (lockscreen_stat){
						TOUCH_LOG("[FINGER] PRESS<%d> x = xxxx, y = xxxx, z= xxxx\n", pReadData->fingerData[j].id);
					} else {
						TOUCH_LOG("[FINGER] PRESS<%d> x = %d, y = %d, z= %d\n",
						pReadData->fingerData[j].id,
						pReadData->fingerData[j].x,
						pReadData->fingerData[j].y,
						pReadData->fingerData[j].pressure);
					}
				}
			}
		}

		if((releasedFinger >> i) & 0x1) {
			TOUCH_LOG("[FINGER] RELEASE<%d>\n", i);
			input_mt_slot(pDriverData->input_dev, i);
			input_report_abs(pDriverData->input_dev, ABS_MT_TRACKING_ID, -1);
		}
	}

	pDriverData->reportData.finger = newFinger;

	input_sync(pDriverData->input_dev);

}

static void report_key(TouchDriverData *pDriverData, TouchReadData *pReadData)
{
	u16 keyIndex = pReadData->keyData.index;
	u16 keyPressed = pReadData->keyData.pressed;
	u32 keyCode = pDriverData->mConfig.button_name[keyIndex-1];
	u32 reportedKeyIndex = pDriverData->reportData.key;
	u32 reportedKeyCode = pDriverData->mConfig.button_name[reportedKeyIndex-1];

	if( reportedKeyIndex == 0 )
	{
		if( keyPressed == KEY_PRESSED )
		{
			input_report_key(pDriverData->input_dev, keyCode, KEY_PRESSED);
			TOUCH_LOG("KEY REPORT : PRESS KEY[%d]\n", keyCode);
			reportedKeyIndex = keyIndex;
		}
		else
		{
			TOUCH_WARN("Invalid key report from device(keyIndex=%d, Pressed=%d)\n", keyIndex, keyPressed);
		}
	}
	else
	{
		if( keyPressed == KEY_PRESSED )
		{
			/* release previous key first */
			input_report_key(pDriverData->input_dev, reportedKeyCode, KEY_RELEASED);
			TOUCH_LOG("KEY REPORT : RELEASE KEY[%d]\n", reportedKeyCode);
			input_sync(pDriverData->input_dev);

			/* report new key */
			input_report_key(pDriverData->input_dev, keyCode, KEY_PRESSED);
			TOUCH_LOG("KEY REPORT : PRESS KEY[%d]\n", keyCode);
			reportedKeyIndex = keyIndex;
		}
		else
		{
			if( reportedKeyIndex == keyIndex )
			{
				input_report_key(pDriverData->input_dev, keyCode, KEY_RELEASED);
				TOUCH_LOG("KEY REPORT : RELEASE KEY[%d]\n", keyCode);
				reportedKeyIndex = 0;
			}
			else
			{
				TOUCH_ERR("Invalid key report from device(keyIndex=%d, Pressed=%d)\n", keyIndex, keyPressed);

				/* protection code */
				input_report_key(pDriverData->input_dev, reportedKeyIndex, KEY_RELEASED);
				TOUCH_LOG("KEY REPORT : RELEASE KEY[%d]\n", reportedKeyIndex);
				reportedKeyIndex = 0;
			}
		}
	}

	pDriverData->reportData.key = reportedKeyIndex;

	input_sync(pDriverData->input_dev);
	
}

static void cancel_key(TouchDriverData *pDriverData)
{
	u32 reportedKeyIndex = pDriverData->reportData.key;
	u32 reportedKeyCode = pDriverData->mConfig.button_name[reportedKeyIndex-1];

	if( reportedKeyIndex )
	{
		input_report_key(pDriverData->input_dev, reportedKeyCode, KEY_CANCELED);
		TOUCH_LOG("KEY REPORT : CANCEL KEY[%d]\n", reportedKeyCode);
		reportedKeyIndex = 0;
	}
	
	pDriverData->reportData.key = reportedKeyIndex;

	input_sync(pDriverData->input_dev);
}

static void send_uevent(TouchDriverData *pDriverData, u8 eventIndex)
{
	if( eventIndex < NUM_OF_UEVENT )
	{
		wake_lock_timeout(pWakeLockTouch, msecs_to_jiffies(3000));
		kobject_uevent_env(&device_touch.kobj, KOBJ_CHANGE, touch_uevent[eventIndex]);
		
		if( eventIndex == UEVENT_KNOCK_ON ) {
			pDriverData->reportData.knockOn = 1;
		} else if( eventIndex == UEVENT_KNOCK_CODE ) {
			pDriverData->reportData.knockCode = 1;
#if defined(ENABLE_SWIPE_MODE)
		} else if( eventIndex == UEVENT_SWIPE ) {
			pDriverData->reportData.knockCode = 1;
#endif
		} else {
			TOUCH_WARN("UEVENT_SIGNATURE is not supported\n");
		}
	}
	else
	{
		TOUCH_ERR("Invalid event index ( index = %d )\n", eventIndex);
	}

#if 1 /* LGE_BSP_COMMON : branden.you@lge.com_20141013 : */
#else
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	wake_lock_timeout(&pDriverData->lpwg_wake_lock, msecs_to_jiffies(3000));

	if (type > 0 && type <= NUM_OF_LPWG_EVENT
		&& atomic_read(&pDriverData->state.uevent_state) == UEVENT_IDLE) {
		atomic_set(&pDriverData->state.uevent_state, UEVENT_BUSY);
		send_uevent(lpwg_uevent[type-1]);
	}
#endif
}

static void report_hover(TouchDriverData *pDriverData, TouchReadData *pReadData)
{
	if( pDriverData->reportData.hover != pReadData->hoverState )
	{
		pDriverData->reportData.hover = pReadData->hoverState;
		
		if( pReadData->hoverState == HOVER_NEAR )
		{
			/* TBD */
			TOUCH_LOG("HOVER REPORT : NEAR\n");
		}
		else
		{
			/* TBD */
			TOUCH_LOG("HOVER REPORT : FAR\n");
		}
	}
	else
	{
		TOUCH_WARN("Duplicated Hover Event from Device ( %d )\n", pReadData->hoverState);
	}
	
}

static void release_all_finger(TouchDriverData *pDriverData)
{
	if( pDriverData->reportData.finger )
	{
		TouchReadData readData;

		memset(&readData, 0x0, sizeof(TouchReadData));
#if defined(ENABLE_GHOST_DETECT_SOLUTION)
		memcpy(pDriverData->prevFingerData, readData.fingerData, (MAX_FINGER+1)*sizeof(TouchFingerData));
#endif

		readData.type = DATA_FINGER;
		readData.count = 0;
		report_finger(pDriverData, &readData);
		TOUCH_LOG("all finger released\n");
	}
}

static void release_all_key(TouchDriverData *pDriverData)
{
	if( pDriverData->reportData.key )
	{
		TouchReadData readData;

		memset(&readData, 0x0, sizeof(TouchReadData));
		
		readData.type = DATA_KEY;
		readData.keyData.index = pDriverData->reportData.key;
		readData.keyData.pressed = KEY_RELEASED;
		report_key(pDriverData, &readData);
		TOUCH_LOG("all key released\n");
	}
}

static void release_all_touch_event(TouchDriverData *pDriverData)
{
	release_all_finger(pDriverData);
	
	if( pDriverData->mConfig.button_support ) {
		release_all_key(pDriverData);
	}
}

static void WqTouchInit(struct work_struct *work_init)
{
	TouchDriverData *pDriverData = container_of(to_delayed_work(work_init),
		TouchDriverData, work_init);

	mutex_lock(pMutexTouch);

	if( pDriverData->bootMode != BOOT_OFF_CHARGING )
		TouchDisableIrq();
	
	pDeviceSpecificFunc->Reset(pDriverData->client);
	pDeviceSpecificFunc->InitRegister(pDriverData->client);
	if( pDriverData->nextState == STATE_NORMAL_HOVER ) {
		pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, pDriverData->nextState, &pDriverData->lpwgSetting);
	}
	
	SetDriverState(pDriverData, pDriverData->nextState);
	
	if( pDriverData->bootMode != BOOT_OFF_CHARGING )
		TouchEnableIrq();

	mutex_unlock(pMutexTouch);
	
}

//==========================================================
// Interrupt Service Routine ( Triggered by HW Interrupt )
//==========================================================
#if defined ( TOUCH_PLATFORM_QCT )
static irqreturn_t TouchIrqHandler(int irq, void *dev_id)
{

#if !defined ( TOUCH_DEVICE_LU202X )
	if (atomic_read(&pm_state) >= PM_SUSPEND) {
		TOUCH_LOG("IRQ in suspend\n");
		atomic_set(&pm_state, PM_SUSPEND_IRQ);
		wake_lock_timeout(pWakeLockTouch, msecs_to_jiffies(3000));

		return IRQ_HANDLED;
	}
#endif

	if(pWorkTouch != NULL) {
		/* trigger work queue to process interrupt */
		queue_delayed_work(touch_wq, pWorkTouch, 0); /* It will call "WqTouchIrqHandler()" */
	}

	return IRQ_HANDLED;
}
#elif defined ( TOUCH_PLATFORM_MTK )
static void TouchIrqHandler(void)
{
	if(pWorkTouch != NULL) {
		/* trigger work queue to process interrupt */
		queue_delayed_work(touch_wq, pWorkTouch, 0); /* It will call "WqTouchIrqHandler()" */
	}
}
#else
#error "Platform should be defined"
#endif


//==========================================================
// Triggered by ISR ( TouchIrqHandler() )
//==========================================================
static void WqTouchIrqHandler(struct work_struct *work_irq)
{
	 TouchDriverData *pDriverData = container_of(to_delayed_work(work_irq),
		 TouchDriverData, work_irq);
	 int ret = 0;

	 TouchReadData readData;

	mutex_lock(pMutexTouch);

	/* TBD : init buffer */
	memset(&readData, 0x0, sizeof(TouchReadData));
	readData.type = DATA_UNKNOWN;

	/* do TouchIC specific interrupt processing */
	ret = pDeviceSpecificFunc->InterruptHandler(pDriverData->client, &readData);
#if defined(ENABLE_GHOST_DETECT_SOLUTION)
	memcpy(pDriverData->prevFingerData, readData.fingerData, (MAX_FINGER+1)*sizeof(TouchFingerData));
#endif

     if(ret == -1) { 
       	 TOUCH_LOG("WqTouchIrqHandler : goto error !!!\n");
    	 goto error;
     	}

	/* do processing according to the data from TouchIC */
	if( readData.type == DATA_FINGER )
	{
		if( pDriverData->currState == STATE_NORMAL || pDriverData->currState == STATE_NORMAL_HOVER )
		{
			/* report cancel key to CFW */
			if( ( pDriverData->mConfig.button_support ) && ( readData.count != 0 ) ) {
				cancel_key(pDriverData);
			}
			
			/* report finger data to CFW */
			report_finger(pDriverData, &readData);
		}
	}
	else if( readData.type == DATA_KEY )
	{
		if( pDriverData->currState == STATE_NORMAL || pDriverData->currState == STATE_NORMAL_HOVER )
		{
			if( pDriverData->mConfig.button_support )
			{
				if( pDriverData->reportData.finger == 0 )
				{
					/* report key to CFW */
					report_key(pDriverData, &readData);
				}
				else
				{
					TOUCH_LOG("Finger event exist so key event was ignored\n"); 
				}
			}
			else
			{
				TOUCH_ERR("Invalid event from device ( event = KEY )\n");
			}
		}
	}
	else if( readData.type == DATA_KNOCK_ON )
	{
		if( pDriverData->currState == STATE_KNOCK_ON_ONLY || pDriverData->currState == STATE_KNOCK_ON_CODE )
		{
			/* report knock-on event to CFW */
			send_uevent(pDriverData, UEVENT_KNOCK_ON);
		}
		else
		{
			TOUCH_WARN("Unmatched event from device ( event = KNOCK_ON )\n");
		}
	}
	else if( readData.type == DATA_KNOCK_CODE )
	{
		if( pDriverData->currState == STATE_KNOCK_ON_CODE )
		{
			pDriverData->reportData.knockCount = readData.count;
			memcpy( pDriverData->reportData.knockData, readData.knockData, readData.count * sizeof(TouchPoint) );
			
			/* report knock-code event to CFW */
			send_uevent(pDriverData, UEVENT_KNOCK_CODE);
		}
	}
#if defined(ENABLE_SWIPE_MODE)
	else if( readData.type == DATA_SWIPE )
	{
		if(lockscreen_stat) {
			pDriverData->reportData.knockCount = readData.count;
			memcpy( pDriverData->reportData.knockData, readData.knockData, readData.count * sizeof(TouchPoint) );

			/* report swipe event to CFW */
			send_uevent(pDriverData, UEVENT_SWIPE);
		}
	}
#endif
#if defined(ENABLE_REALTIME_LPWG_FAIL_REASON)
	else if( readData.type == DATA_LPWG_FAIL)
	{
		/* Do nothing */
	}
#endif
	else if( readData.type == DATA_HOVER )
	{
		if( pDriverData->currState == STATE_NORMAL_HOVER || pDriverData->currState == STATE_HOVER )
		{
			/* report hover state */
			report_hover(pDriverData, &readData);
		}
		else
		{
			TOUCH_WARN("Unmatched event from device ( event = HOVER )\n");
		}
	}

error: 
	
	mutex_unlock(pMutexTouch);

	if(ret == TOUCH_FAIL) {
		if(pDriverData->lpwgSetting.lcdState==1){
#if defined(ENABLE_GHOST_DETECT_SOLUTION)
			release_all_touch_event(pDriverData);
			if(atomic_read(&pDriverData->needToRebase) == REBASE_NEEDED) {
				ret = pDeviceSpecificFunc->rebase_ic(pDriverData->client, &readData, &pDriverData->needToRebase);
				if(ret == TOUCH_FAIL)
					queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
			} else {
				TOUCH_ERR("Abnormal IC status. Touch IC will be reset\n");
				queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
			}
#else
			TOUCH_ERR("Abnormal IC status. Touch IC will be reset\n");
			queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
#endif
		} else {
			TOUCH_ERR("Abnormal IC status in LPWG mode. Skip Touch IC Reset\n");
#if defined(ENABLE_GHOST_DETECT_SOLUTION)
			if(atomic_read(&pDriverData->needToRebase) == REBASE_NEEDED) {
				atomic_set(&pDriverData->needToRebase, REBASE_DONE);
			}
#endif
	   	}
	}
}


/****************************************************************************
* Global Functions
****************************************************************************/
static void WqfirmwareUpgrade(struct work_struct *work_upgrade)
{
	TouchDriverData *pDriverData = container_of(to_delayed_work(work_upgrade),
		TouchDriverData, work_upgrade);

	int i = 0;
	int result = TOUCH_SUCCESS;
	char *pFilename = NULL;

	mutex_lock(pMutexTouch);
	wake_lock(pWakeLockTouch);

	TOUCH_FUNC();

	TouchDisableIrq();

	SetDriverState(pDriverData, STATE_UPDATE_FIRMWARE);

	if( pDriverData->useDefaultFirmware == TOUCH_FALSE ) {
		pFilename = pDriverData->fw_image;
	}

	for ( i = 0 ; i < MAX_FW_UPGRADE_RETRY ; i++ )
	{
		result = pDeviceSpecificFunc->UpdateFirmware(pDriverData->client, pFilename);
		if ( result == TOUCH_SUCCESS ) {
			TOUCH_LOG("Firmware upgrade was Succeeded\n");
			break;
		}

		TOUCH_WARN("Retry firmware upgrade\n");
	}

	if( result == TOUCH_FAIL ) {
		TOUCH_ERR("failed to upgrade firmware\n");
	}

	pDeviceSpecificFunc->Reset(pDriverData->client);
	pDeviceSpecificFunc->InitRegister(pDriverData->client);
	pDeviceSpecificFunc->ReadIcFirmwareInfo(pDriverData->client, &pDriverData->icFwInfo);
	
	SetDriverState(pDriverData, STATE_NORMAL);

	TouchEnableIrq();

	wake_unlock(pWakeLockTouch);
	mutex_unlock(pMutexTouch);
	
}

#if !defined ( TOUCH_MODEL_C70 ) && !defined ( TOUCH_MODEL_C90NAS ) && !defined ( TOUCH_MODEL_P1B ) && !defined ( TOUCH_MODEL_P1C ) && !defined ( TOUCH_MODEL_YG ) && !defined ( TOUCH_MODEL_C100N )/* TBD : temporally block ( C70 is not ready ) */
static void MftsTouchOnOff(TouchDriverData *pDriverData, int isOn)
{
	if( isOn == 1 )
	{
		TOUCH_LOG("Touch On in MFTS\n");
		
		/* turn on the power of touch */
		TouchPower(1);

		/* reset */
		pDeviceSpecificFunc->Reset(pDriverData->client);

		/* initialise touch device */
		pDeviceSpecificFunc->InitRegister(pDriverData->client);

		/* clear interrupt */
		pDeviceSpecificFunc->ClearInterrupt(pDriverData->client);
		
		SetDriverState(pDriverData, STATE_NORMAL);
		
		/* enable interrupt */
		TouchEnableIrq();
		
	}
	else
	{
		TOUCH_LOG("Touch Off in MFTS\n");
		
		/* disable interrupt */
		TouchDisableIrq();
		
		SetDriverState(pDriverData, STATE_UNKNOWN);
		
		/* turn off the power of touch */
		TouchPower(0);
	}
	
}
#endif

static void UpdateLpwgSetting ( LpwgSetting *pLpwgSetting, LpwgCmd lpwgCmd, int *param )
{
	u8 str[20] = {0};

	switch ( lpwgCmd )
	{
		case LPWG_CMD_MODE:
			pLpwgSetting->mode = param[0];
			sprintf(str, "%s", "MODE");
			break;

		case LPWG_CMD_LCD_PIXEL_SIZE:
			pLpwgSetting->lcdPixelSizeX = param[0];
			pLpwgSetting->lcdPixelSizeY = param[1];
			sprintf(str, "%s", "PIXEL_SIZE");
			break;

		case LPWG_CMD_ACTIVE_TOUCH_AREA:
			pLpwgSetting->activeTouchAreaX1 = param[0];
			pLpwgSetting->activeTouchAreaX2 = param[1];
			pLpwgSetting->activeTouchAreaY1 = param[2];
			pLpwgSetting->activeTouchAreaY2 = param[3];
			sprintf(str, "%s", "ACTIVE_AREA");
			break;

		case LPWG_CMD_TAP_COUNT:
			pLpwgSetting->tapCount = param[0];
			sprintf(str, "%s", "TAP_COUNT");
			break;

		case LPWG_CMD_TAP_DISTANCE:
			TOUCH_WARN ( "Invalide LPWG Command ( LPWG_CMD_TAP_DISTANCE )\n" );
			break;

		case LPWG_CMD_LCD_STATUS:
			pLpwgSetting->lcdState = param[0];
			sprintf(str, "%s", "LCD");
			break;

		case LPWG_CMD_PROXIMITY_STATUS:
			pLpwgSetting->proximityState = param[0];
			sprintf(str, "%s", "PROXIMITY");
			break;

		case LPWG_CMD_FIRST_TWO_TAP:
			pLpwgSetting->isFirstTwoTapSame = param[0];
			sprintf(str, "%s", "FIRST_TWO_TAP");
			break;

		case LPWG_CMD_UPDATE_ALL:
			pLpwgSetting->mode = param[0];
			pLpwgSetting->lcdState = param[1];
			pLpwgSetting->proximityState = param[2];
			pLpwgSetting->coverState = param[3];
			sprintf(str, "%s", "ALL");
			break;

		case LPWG_CMD_CALL:
			pLpwgSetting->callState = param[0];
			sprintf(str, "%s", "CALL");
			break;

		default:
			TOUCH_ERR ( "Invalide LPWG Command ( Type = %d )\n", lpwgCmd );
			break;
			
	}
	
	TOUCH_LOG("LPWG SETTING : CMD[%s] M[%d] L[%d] P[%d] Cover[%d] Call[%d]\n",
		str, pLpwgSetting->mode, pLpwgSetting->lcdState, pLpwgSetting->proximityState, pLpwgSetting->coverState, pLpwgSetting->callState);

}

static TouchState DecideNextDriverState( LpwgSetting *pLpwgSetting )
{
	TouchState nextState = STATE_UNKNOWN;
	
	if( pLpwgSetting->lcdState == 1 )
	{
		if( pLpwgSetting->callState == 2 ) {
			nextState = STATE_NORMAL_HOVER;
		} else {
			nextState = STATE_NORMAL;
		}
	} 
	else
	{
		if( pLpwgSetting->callState == 2 )
		{
			nextState = STATE_HOVER;
		}
		else
		{
			if( pLpwgSetting->mode == 0 ) {
				nextState = STATE_OFF;
			} else if( pLpwgSetting->mode == 1 ) {
				nextState = STATE_KNOCK_ON_ONLY;
			} else if( pLpwgSetting->mode == 2 ) {
				nextState = STATE_KNOCK_ON_CODE; /* TBD : CFW Bug but Cover it. Report it to CFW. */
			} else if( pLpwgSetting->mode == 3 ) {
				nextState = STATE_KNOCK_ON_CODE;
			} else {
				TOUCH_ERR("Invalid Mode Setting ( mode = %d )\n", pLpwgSetting->mode);
				TOUCH_ERR("State was forcely changed to Normal\n");
				nextState = STATE_NORMAL;
			}
		}
	}

	return nextState;
	
}


//==========================================================
// CFW will use it to check if knock-on is supported or not.
// If you return "0", CFW will not send LPWG command.
//==========================================================
static ssize_t show_knock_on_type(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%d\n", 1);

	return ret;
}


//==========================================================
// CFW will use it to send LPWG command.
//==========================================================
static ssize_t store_lpwg_notify(struct i2c_client *client, const char *buf, size_t count)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	LpwgCmd lpwgCmd = LPWG_CMD_UNKNOWN;
	LpwgSetting *pLpwgSetting = NULL;
	TouchState nextState = STATE_UNKNOWN;

	int type = 0;
	int value[4] = {0};

	/* Get command and parameter from buffer */
	sscanf(buf, "%d %d %d %d %d", &type, &value[0], &value[1], &value[2], &value[3]);

	lpwgCmd = (LpwgCmd)type;

	/* load stored previous setting */
	pLpwgSetting = &pDriverData->lpwgSetting;

	#if !defined ( ENABLE_HOVER_DETECTION )
	if( lpwgCmd == LPWG_CMD_CALL ) {
		return count;
	}
	#endif
	
	mutex_lock(pMutexTouch);

	/* update new lpwg setting */
	UpdateLpwgSetting(pLpwgSetting, lpwgCmd, value);

	if( ( lpwgCmd != LPWG_CMD_UPDATE_ALL ) && ( lpwgCmd != LPWG_CMD_CALL ) ) {
		mutex_unlock(pMutexTouch);
		return count;
	}

	/* decide next driver state */
	nextState = DecideNextDriverState(pLpwgSetting);

	/* apply it using device driver function */
	if( ( nextState != STATE_UNKNOWN ) && ( pDriverData->currState != nextState ) )
	{
		
		#if defined ( TOUCH_DEVICE_S3320 )	 || defined ( TOUCH_DEVICE_MIT200 )
		
		if( ( pDriverData->currState != STATE_NORMAL ) && ( nextState != STATE_NORMAL ) )
		{
			pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, nextState, &pDriverData->lpwgSetting);
			SetDriverState(pDriverData, nextState);
		}
		else
		{
			/* store next state to use later ( suspend or resume ) */
			pDriverData->nextState = nextState;
			TOUCH_LOG("LPWG Setting will be processed on suspend or resume\n");
		}
		
		#else /* General Add-on type touch */

		if( nextState == STATE_NORMAL || nextState == STATE_NORMAL_HOVER )
		{
			if( ( pDriverData->currState == STATE_NORMAL ) ||( pDriverData->currState == STATE_NORMAL_HOVER ) )
			{
				pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, nextState, &pDriverData->lpwgSetting);
				SetDriverState(pDriverData, nextState);
			}
			else
			{
				release_all_touch_event(pDriverData);
				pDriverData->nextState = nextState;
				queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
			}
		}
		else
		{
			pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, nextState, &pDriverData->lpwgSetting);
			SetDriverState(pDriverData, nextState);
		}

		#endif
		
	}

	mutex_unlock(pMutexTouch);

	return count;
	
}

//==========================================================
// CFW will use it to read knock-code data
//==========================================================
static ssize_t show_lpwg_data(struct i2c_client *client, char *buf)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	int i = 0;
	int ret = 0;

	mutex_lock(pMutexTouch);

	/* We already get the data on ISR, so we can return the data immediately */
	for( i=0 ; i<pDriverData->reportData.knockCount ; i++ ) {
		if ( pDriverData->reportData.knockData[i].x == -1 && pDriverData->reportData.knockData[i].y == -1 )
			break;

		ret += sprintf(buf+ret, "%d %d\n", pDriverData->reportData.knockData[i].x, pDriverData->reportData.knockData[i].y);
	}

	TOUCH_LOG("LPWG data was read by CFW\n"); 

	mutex_unlock(pMutexTouch);

	return ret;
	
}

//==========================================================
// CFW will use it to give a feedback ( result ) on the event we sent before on ISR
//==========================================================
static ssize_t store_lpwg_data(struct i2c_client *client, const char *buf, size_t count)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	int reply = 0;

	sscanf(buf, "%d", &reply);

	mutex_lock(pMutexTouch);

	if( reply == 1 )
	{
		TOUCH_LOG("LPWG result was informed by CFW ( Code is matched )\n");
		/* Code is matched, do something for leaving "LPWG Mode" if you need. But normally "Resume" will be triggered soon. */
	}
	else
	{
		TOUCH_LOG("LPWG result was informed by CFW ( Code is NOT matched )\n");
#if defined(ENABLE_SWIPE_MODE)
		if(wakeup_by_swipe){
			wakeup_by_swipe = 0;
			TOUCH_LOG("skip lpwg setting in case of wakeup_by_swipe[%d]\n",wakeup_by_swipe);
		}
#endif

	}

	/* clear knock data */
	pDriverData->reportData.knockOn = 0;
	pDriverData->reportData.knockCode = 0;
	memset( pDriverData->reportData.knockData, 0x00, sizeof(pDriverData->reportData.knockData) );

	wake_unlock(pWakeLockTouch);
	mutex_unlock(pMutexTouch);

	return count;
	
}

//==========================================================
// show_firmware is uesd for firmware information in hidden menu
//==========================================================
static ssize_t show_firmware(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	ret = sprintf(buf, "\n======== IC Firmware Info ========\n");

	ret += sprintf(buf+ret,	"FW Version: %d.%02d\n",	pDriverData->icFwInfo.isOfficial, pDriverData->icFwInfo.version);

	ret += sprintf(buf+ret, "\n====== Binary Firmware Info ======\n");

	ret += sprintf(buf+ret,	"Bin Version: %d.%02d\n",	pDriverData->binFwInfo.isOfficial, pDriverData->binFwInfo.version);

	return ret;
}


//==========================================================
// "at%touchfwver" will use it to get firmware information of touch IC
//==========================================================
static ssize_t show_atcmd_fw_ver(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	mutex_lock(pMutexTouch);

	ret = sprintf(buf, "V%d.%02d (%d/%d/%d)\n",
			pDriverData->icFwInfo.isOfficial, pDriverData->icFwInfo.version,
			pDriverData->icFwInfo.moduleMakerID, pDriverData->icFwInfo.moduleVersion,
			pDriverData->icFwInfo.modelID);

	mutex_unlock(pMutexTouch);
		
	return ret;
	
}
//==========================================================
// "testmode" will use it to get firmware information of touch IC
//==========================================================
static ssize_t show_testmode_fw_ver(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	mutex_lock(pMutexTouch);

	ret = sprintf(buf, "V%d.%02d (%d/%d/%d)\n",
			pDriverData->icFwInfo.isOfficial, pDriverData->icFwInfo.version,
			pDriverData->icFwInfo.moduleMakerID, pDriverData->icFwInfo.moduleVersion,
			pDriverData->icFwInfo.modelID);

	mutex_unlock(pMutexTouch);
		
	return ret;
	
}


//==========================================================
// AAT will use it to get the result of self-diagnosis
//==========================================================
static ssize_t show_sd_info(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int channelStatus = 0;
	int rawStatus = 0;
	u8 *pBuf = NULL;
	int bufSize = 2*1024;
	int dataLen = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	mutex_lock(pMutexTouch);

	/* allocate buffer for additional debugging information */
	pBuf = kzalloc(bufSize, GFP_KERNEL);
	if (pBuf == NULL) {
		TOUCH_ERR("failed to allocate memory for self diagnosis\n");
		return ret;
	}

	#if defined ( TOUCH_DEVICE_S3320 )
	release_all_touch_event(pDriverData);
    #endif

	TouchDisableIrq();

	SetDriverState(pDriverData, STATE_SELF_DIAGNOSIS);

	/* TBD : consider in case of LCD Off ( means LPWG mode ) */
	pDeviceSpecificFunc->DoSelfDiagnosis(client, &rawStatus, &channelStatus, pBuf, bufSize, &dataLen);
	pDeviceSpecificFunc->Reset(pDriverData->client);
	pDeviceSpecificFunc->InitRegister(pDriverData->client);
	
	SetDriverState(pDriverData, STATE_NORMAL);

	TouchEnableIrq();

	mutex_unlock(pMutexTouch);

	/* basic information ( return data string format can't be changed ) */
	ret = sprintf(buf, "========RESULT=======\n");
	ret += sprintf(buf+ret, "Channel Status : %s", (channelStatus==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
	ret += sprintf(buf+ret, "Raw Data : %s", (rawStatus==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");

	if( dataLen > bufSize  ) {
		TOUCH_ERR("buffer size was overflowed ( use size = %d )\n", dataLen);
		kfree(pBuf);
		return ret;
	}
	
	/* addition information for debugging */
	memcpy(buf+ret, pBuf, dataLen);
	ret += dataLen;

	kfree(pBuf);

	return ret;
	
}

#if defined ( TOUCH_PLATFORM_MTK )
static ssize_t store_mfts_onoff(struct i2c_client *client, const char *buf, size_t count)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	int isOn = 0;

	sscanf(buf, "%d", &isOn);

	if ( isOn < 0 || isOn > 1) {
		TOUCH_WARN("Invalid input value ( isOn = %d )\n", isOn);
		return count;
	}

	MftsTouchOnOff(pDriverData, isOn);
	
	return count;
	
}
#endif

//==========================================================
// Developer will use it to upgrade firmware with filename
//==========================================================
static ssize_t store_upgrade(struct i2c_client *client, const char *buf, size_t count)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	if( count > MAX_FILENAME )
	{
		return count;
	}

	mutex_lock(pMutexTouch);

	pDriverData->useDefaultFirmware = TOUCH_FALSE;

	memset(pDriverData->fw_image, 0x00, sizeof(pDriverData->fw_image));
	sscanf(buf, "%s", pDriverData->fw_image);

	queue_delayed_work(touch_wq, &pDriverData->work_upgrade, 0);
	
	mutex_unlock(pMutexTouch);

	return count;
	
}

//==========================================================
// Developer will use it to upgrade firmware with default firmware
//==========================================================
static ssize_t show_upgrade(struct i2c_client *client, char *buf)
{
	int ret = 0;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	mutex_lock(pMutexTouch);

	pDriverData->useDefaultFirmware = TOUCH_TRUE;

	ret = sprintf(buf, "default firmware upgrade\n");

	queue_delayed_work(touch_wq, &pDriverData->work_upgrade, 0);

	mutex_unlock(pMutexTouch);

	return ret;
}

static ssize_t store_rewrite_bin_fw(struct i2c_client *client, const char *buf, size_t count)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	mutex_lock(pMutexTouch);

	pDriverData->useDefaultFirmware = TOUCH_TRUE;

	queue_delayed_work(touch_wq, &pDriverData->work_upgrade, 0);

	mutex_unlock(pMutexTouch);

	return count;
}

//==========================================================
// Developer will use it to read register from or write register to touch IC
// To Write : echo write $reg $data > ic_rw
// To Read : echo read $reg > ic_rw ==> cat > ic_rw
//==========================================================
static int regAddr=0, readCount=0;
static ssize_t show_ic_rw(struct i2c_client *client, char *buf)
{
	int result = 0;
	int ret = 0;
	int readValue = 0;

	int i = 0;

	mutex_lock(pMutexTouch);

	for( i=0 ; i<readCount ; i++ )
	{
		result = pDeviceSpecificFunc->AccessRegister(client, READ_IC_REG, regAddr, &readValue);
		if( result == TOUCH_FAIL ) {
			TOUCH_ERR("failed to read register ( reg = %d )\n", regAddr);
			break;
		}
		ret += sprintf(buf+ret, "0x%04X=%d\n", regAddr, readValue);
		regAddr++;
	}

	mutex_unlock(pMutexTouch);

	TOUCH_DBG("Read IC Register ( Begin )\n");
	TOUCH_DBG("%s\n", buf);
	TOUCH_DBG("Read IC Register ( End )\n");
	
	return ret;
	
}

static ssize_t store_ic_rw(struct i2c_client *client, const char *buf, size_t count)
{
	int ret = 0;
	
	u8 cmd[30] = {0};
	int reg = 0;
	int data = 0;
	

	sscanf(buf, "%s %d %d", cmd, &reg, &data);

	if ((strcmp(cmd, "write") && strcmp(cmd, "read"))) {
		return count;
	}

	mutex_lock(pMutexTouch);

	if( strcmp(cmd, "write") == 0 )
	{
		ret = pDeviceSpecificFunc->AccessRegister(client, WRITE_IC_REG, reg, &data);
		if( ret == TOUCH_FAIL ) {
			TOUCH_ERR("failed to write register ( reg = %d, data =%d )\n", reg, data );
		} else {
			TOUCH_DBG("Success to write register ( reg = %d, data =%d )\n", reg, data );
		}
	}
	else
	{
		TOUCH_DBG("Ready to read register ( reg = %d )\n", reg);
		regAddr = reg;
		readCount = data;
		if( readCount == 0 ) {
			readCount = 1;
		}			
	}

	mutex_unlock(pMutexTouch);

	return count;
	
}

static ssize_t store_keyguard_info(struct i2c_client *client,
		const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);

	switch (value) {
	case 0:
		lockscreen_stat = 0;
		TOUCH_DBG("Lockscreen unlocked\n");
		break;
	case 1:
		lockscreen_stat = 1;
		TOUCH_DBG("Lockscreen locked\n");
		break;
    	default:
		    break;
	}

	return count;
}

//==========================================================
// Developer will use it to debug touch coordinates in real time
// debug_abs 1: on , 0: off
//==========================================================
static ssize_t store_debug_abs(struct i2c_client *client,
		const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);
	switch (value) {
	case 0:
		debug_abs = 0;
		TOUCH_DBG("debug_abs = %d\n", debug_abs);
		break;
	case 1:
		debug_abs = 1;
		TOUCH_DBG("debug_abs = %d\n", debug_abs);
		break;
    default:
		    break;
	}

	return count;
}


static LGE_TOUCH_ATTR(keyguard, S_IRUGO | S_IWUSR, NULL, store_keyguard_info);
static LGE_TOUCH_ATTR(knock_on_type, S_IRUGO | S_IWUSR, show_knock_on_type, NULL);
static LGE_TOUCH_ATTR(lpwg_notify, S_IRUGO | S_IWUSR, NULL, store_lpwg_notify);
static LGE_TOUCH_ATTR(lpwg_data, S_IRUGO | S_IWUSR, show_lpwg_data, store_lpwg_data);
static LGE_TOUCH_ATTR(firmware, S_IRUGO | S_IWUSR, show_firmware, NULL);
static LGE_TOUCH_ATTR(fw_ver, S_IRUGO | S_IWUSR, show_atcmd_fw_ver, NULL);
static LGE_TOUCH_ATTR(testmode_ver, S_IRUGO | S_IWUSR, show_testmode_fw_ver, NULL);
static LGE_TOUCH_ATTR(sd, S_IRUGO | S_IWUSR, show_sd_info, NULL);
static LGE_TOUCH_ATTR(fw_upgrade, S_IRUGO | S_IWUSR, show_upgrade, store_upgrade);
static LGE_TOUCH_ATTR(rewrite_bin_fw, S_IRUGO | S_IWUSR, NULL, store_rewrite_bin_fw);
static LGE_TOUCH_ATTR(ic_rw, S_IRUGO | S_IWUSR, show_ic_rw, store_ic_rw);
static LGE_TOUCH_ATTR(debug_abs, S_IRUGO | S_IWUSR, NULL, store_debug_abs);


static struct attribute *lge_touch_attribute_list[] = {
	&lge_touch_attr_keyguard.attr,
	&lge_touch_attr_knock_on_type.attr,
	&lge_touch_attr_lpwg_notify.attr,
	&lge_touch_attr_lpwg_data.attr,
	&lge_touch_attr_firmware.attr,
	&lge_touch_attr_fw_ver.attr,
	&lge_touch_attr_testmode_ver.attr,
	&lge_touch_attr_sd.attr,
	&lge_touch_attr_fw_upgrade.attr,
	&lge_touch_attr_rewrite_bin_fw.attr,
	&lge_touch_attr_ic_rw.attr,
	&lge_touch_attr_debug_abs.attr,
	NULL,
};

static ssize_t lge_touch_attr_show(struct kobject *lge_touch_kobj,
	struct attribute *attr, char *buf)
{
	TouchDriverData *pDriverData = container_of(lge_touch_kobj,
		TouchDriverData, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->show)
		ret = lge_touch_priv->show(pDriverData->client, buf);

	return ret;
}

static ssize_t lge_touch_attr_store(struct kobject *lge_touch_kobj,
	struct attribute *attr,
			      const char *buf, size_t count)
{
	TouchDriverData *pDriverData = container_of(lge_touch_kobj,
		TouchDriverData, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->store)
		ret = lge_touch_priv->store(pDriverData->client, buf, count);

	return ret;
}

static const struct sysfs_ops lge_touch_sysfs_ops = {
	.show	= lge_touch_attr_show,
	.store	= lge_touch_attr_store,
};

static struct kobj_type lge_touch_kobj_type = {
	.sysfs_ops	= &lge_touch_sysfs_ops,
};

/* sysfs_register
 *
 * get_attribute_array_size
 * : attribute_list should has NULL value at the end of list.
 */

static int get_attribute_array_size(struct attribute **list)
{
	int i = 0;

	while (list[i] != NULL && i < MAX_ATTRIBUTE_ARRAY_SIZE)
		i++;

	return i <= MAX_ATTRIBUTE_ARRAY_SIZE ? i : 0;
}

static int sysfs_register(TouchDriverData *pDriverData,
	struct attribute **attribute_list)
{
	struct attribute **new_attribute_list;

	int ret = 0;
	int n1 = get_attribute_array_size(lge_touch_attribute_list);
	int n2 = attribute_list ? get_attribute_array_size(attribute_list) : 0;

	TOUCH_FUNC();

	new_attribute_list = devm_kzalloc(&pDriverData->client->dev, (n1+n2+1) * sizeof(struct attribute *), GFP_KERNEL);
	if( new_attribute_list == NULL ) {
		TOUCH_ERR("Fail to allocation memory\n");
		return -ENOMEM;
	}

	memcpy(new_attribute_list, lge_touch_attribute_list, n1 * sizeof(struct attribute *));

	if( attribute_list ) {
		memcpy(new_attribute_list + n1, attribute_list, n2 * sizeof(struct attribute *));
	}

	lge_touch_kobj_type.default_attrs = new_attribute_list;

	ret = subsys_system_register(&touch_subsys, NULL);
	if (ret < 0) {
		TOUCH_ERR("Fail to register subsys ( error = %d )\n", ret);
		return -ENODEV;
	}

	ret = device_register(&device_touch);
	if (ret < 0) {
		TOUCH_ERR("Fail to register device ( error = %d )\n", ret);
		return -ENODEV;
	}

	ret = kobject_init_and_add(&pDriverData->lge_touch_kobj,
			&lge_touch_kobj_type,
			pDriverData->input_dev->dev.kobj.parent, "%s", LGE_TOUCH_NAME);
	if( ret < 0 ) {
		TOUCH_ERR("Fail to init and add kobject ( error = %d )\n", ret);
		device_unregister(&device_touch);
		return -ENODEV;
	}

	return TOUCH_SUCCESS;
		
}

static void sysfs_unregister(TouchDriverData *pDriverData)
{
	kobject_del(&pDriverData->lge_touch_kobj);
	device_unregister(&device_touch);
	devm_kfree(&pDriverData->client->dev, lge_touch_kobj_type.default_attrs);
}

static int register_input_dev(TouchDriverData *pDriverData)
{
	int ret = 0;
	int idx = 0;

	TOUCH_FUNC();

	pDriverData->input_dev = input_allocate_device();
	if( pDriverData->input_dev == NULL ) {
		TOUCH_ERR("failed at input_allocate_device()\n");
		return TOUCH_FAIL;
	}

	pDriverData->input_dev->name = "touch_dev";

	set_bit(EV_SYN, pDriverData->input_dev->evbit);
	set_bit(EV_ABS, pDriverData->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, pDriverData->input_dev->propbit);

	input_set_abs_params(pDriverData->input_dev, ABS_MT_POSITION_X, 0, pDriverData->mConfig.max_x, 0, 0);
	input_set_abs_params(pDriverData->input_dev, ABS_MT_POSITION_Y, 0, pDriverData->mConfig.max_y, 0, 0);
	input_set_abs_params(pDriverData->input_dev, ABS_MT_PRESSURE, 0, pDriverData->mConfig.max_pressure, 0, 0);
	input_set_abs_params(pDriverData->input_dev, ABS_MT_WIDTH_MAJOR, 0, pDriverData->mConfig.max_width, 0, 0);
	input_set_abs_params(pDriverData->input_dev, ABS_MT_WIDTH_MINOR, 0, pDriverData->mConfig.max_width, 0, 0);
	input_set_abs_params(pDriverData->input_dev, ABS_MT_ORIENTATION, 0, pDriverData->mConfig.max_orientation, 0, 0);

	if (pDriverData->mConfig.protocol_type == MT_PROTOCOL_B)
	{
		#if defined ( KERNEL_ABOVE_3_4_67 )
		ret = input_mt_init_slots(pDriverData->input_dev, pDriverData->mConfig.max_id, 0);
		#else
		ret = input_mt_init_slots(pDriverData->input_dev, pDriverData->mConfig.max_id);
		#endif
		if( ret < 0 ) {
			TOUCH_ERR("failed at input_mt_init_slots() ( error = %d )\n", ret);
			input_free_device(pDriverData->input_dev);
			return TOUCH_FAIL;
		}
	}
	else
	{
		input_set_abs_params(pDriverData->input_dev, ABS_MT_TRACKING_ID, 0, pDriverData->mConfig.max_id, 0, 0);
	}

	if (pDriverData->mConfig.button_support) {
		set_bit(EV_KEY, pDriverData->input_dev->evbit);
		for( idx = 0; idx < pDriverData->mConfig.number_of_button; idx++ ) {
			set_bit(pDriverData->mConfig.button_name[idx], pDriverData->input_dev->keybit);
		}
	}

	ret = input_register_device(pDriverData->input_dev);
	if( ret < 0 ) {
		TOUCH_ERR("failed at input_register_device() ( error = %d )\n", ret);
		if (pDriverData->mConfig.protocol_type == MT_PROTOCOL_B) {
			input_mt_destroy_slots(pDriverData->input_dev);
		}
		input_free_device(pDriverData->input_dev);
		return TOUCH_FAIL;
	}

	input_set_drvdata(pDriverData->input_dev, pDriverData);

	return TOUCH_SUCCESS;
	
}

static void unregister_input_dev ( TouchDriverData *pDriverData)
{
	if( pDriverData->mConfig.protocol_type == MT_PROTOCOL_B ) {
		input_mt_destroy_slots(pDriverData->input_dev);
	}
	input_unregister_device(pDriverData->input_dev);

}

#if defined ( CONFIG_HAS_EARLYSUSPEND )

static void touch_early_suspend(struct early_suspend *h)
{
	TouchDriverData *pDriverData = container_of(h, TouchDriverData, early_suspend);

	mutex_lock(pMutexTouch);
	
	TOUCH_FUNC();

	if( pDriverData->isSuspend == TOUCH_TRUE ) {
		mutex_unlock(pMutexTouch);
		return;
	} else {
		pDriverData->isSuspend = TOUCH_TRUE;
	}

	if( pDriverData->bootMode == BOOT_MINIOS ) /* MFTS BOOT */
	{
		TOUCH_DBG("MFTS\n");
		/* remove after miniOS change code not to call suspend */
	}
	else /* NORMAL BOOT */
	{
		#if defined ( TOUCH_DEVICE_S3320 ) || defined ( TOUCH_DEVICE_MIT200 )
		cancel_delayed_work_sync(&pDriverData->work_init);

		release_all_touch_event(pDriverData);
		
		pDriverData->lpwgSetting.lcdState = 0;
		pDriverData->nextState = DecideNextDriverState(&pDriverData->lpwgSetting);
		pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, pDriverData->nextState, &pDriverData->lpwgSetting);
		SetDriverState(pDriverData, pDriverData->nextState);
		#endif
	}
	
	TOUCH_FUNC_EXT("Exit\n");
	
	mutex_unlock(pMutexTouch);
	
}

static void touch_late_resume(struct early_suspend *h)
{
	TouchDriverData *pDriverData = container_of(h, TouchDriverData, early_suspend);

	mutex_lock(pMutexTouch);

	TOUCH_FUNC();

	if( pDriverData->isSuspend == TOUCH_FALSE ) {
		mutex_unlock(pMutexTouch);
		return;
	} else {
		pDriverData->isSuspend = TOUCH_FALSE;
	}

	if( pDriverData->bootMode == BOOT_MINIOS ) /* MFTS BOOT */
	{
		TOUCH_DBG("MFTS\n");
		/* remove after miniOS change code not to call resume */
	}
	else /* NORMAL BOOT */
	{
		#if defined ( TOUCH_DEVICE_S3320 )
		pDriverData->lpwgSetting.lcdState = 1;
		pDriverData->nextState = DecideNextDriverState(&pDriverData->lpwgSetting);
		pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, pDriverData->nextState, &pDriverData->lpwgSetting);
		SetDriverState(pDriverData, pDriverData->nextState);
		
		queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
		#endif
		
		#if defined ( TOUCH_DEVICE_MIT200 )
		queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
		#endif
	}

	TOUCH_FUNC_EXT("Exit\n");

	mutex_unlock(pMutexTouch);

}

#elif defined ( CONFIG_FB )

#if defined ( TOUCH_PLATFORM_MSM8916 ) || defined ( TOUCH_PLATFORM_MSM8936 )
//===========================================
// 1. Sequence
//    : FB_EARLY_EVENT_BLANK ==> LCD ==> FB_EVENT_BLANK
// 2. Suspend : FB_BLANK_UNBLANK
// 3. Resune : FB_BLANK_POWERDOWN
//===========================================
static int touch_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = (struct fb_event *)data;
	TouchDriverData *pDriverData = container_of(self, TouchDriverData, fb_notif);
#if !defined ( TOUCH_MODEL_C90NAS )
	static int isEarlySuspend = TOUCH_FALSE;
#endif
	int *blank = NULL;
	#if defined ( TOUCH_DEVICE_S3320 ) && !defined ( TOUCH_MODEL_C90NAS )
	/* WORK_AROUND_1 : skip first set of suspend and resume pair to avoid black screen on C70(S3320) */
	static int skipCount = 1;
	#endif

	if (evdata && evdata->data) {
		blank = (int *)evdata->data;
	} else {
		return TOUCH_SUCCESS;
	}

	mutex_lock(pMutexTouch);

#if !defined ( TOUCH_MODEL_C90NAS )
	/* to avoid un-wanted control caused by duplicated callback */
	if(event == FB_EARLY_EVENT_BLANK)
	{
		if (*blank == FB_BLANK_POWERDOWN)
		{
			TOUCH_FUNC_EXT("Early Suspend\n");
			if( isEarlySuspend == TOUCH_TRUE ) {
				mutex_unlock(pMutexTouch);
				return TOUCH_SUCCESS;
			} else {
				isEarlySuspend = TOUCH_TRUE;
			}
		}
		else if(*blank == FB_BLANK_UNBLANK)
		{
			TOUCH_FUNC_EXT("Early Resume\n");
			if( isEarlySuspend == TOUCH_FALSE ) {
				mutex_unlock(pMutexTouch);
				return TOUCH_SUCCESS;
			} else {
				isEarlySuspend = TOUCH_FALSE;
			}
		}
	}
	else if(event == FB_EVENT_BLANK)
	{
		if (*blank == FB_BLANK_POWERDOWN)
		{
			TOUCH_FUNC_EXT("Suspend\n");
			if( pDriverData->isSuspend == TOUCH_TRUE ) {
				mutex_unlock(pMutexTouch);
				return TOUCH_SUCCESS;
			} else {
				pDriverData->isSuspend = TOUCH_TRUE;
			}
		}
		else if (*blank == FB_BLANK_UNBLANK)
		{
			TOUCH_FUNC_EXT("Resume\n");
			if( pDriverData->isSuspend == TOUCH_FALSE ) {
				mutex_unlock(pMutexTouch);
				return TOUCH_SUCCESS;
			} else {
				pDriverData->isSuspend = TOUCH_FALSE;
			}

			#if defined ( TOUCH_DEVICE_S3320 )
			/* WORK_AROUND_1 : skip first set of suspend and resume pair to avoid black screen on C70(S3320) */
			if( pDriverData->bootMode == BOOT_OFF_CHARGING )
			{
				if( skipCount >= 0 ) {
					skipCount--;
				}
				
				if( skipCount == 0 ) {
					mutex_unlock(pMutexTouch);
					return TOUCH_SUCCESS;
				}
			}
			#endif
			
		}
	}
#endif

	#if defined ( TOUCH_DEVICE_S3320 ) && !defined ( TOUCH_MODEL_C90NAS )
	//WORK_AROUND_1 : skip first set of suspend and resume pair to avoid black screen on C70(S3320)
	if( pDriverData->bootMode == BOOT_OFF_CHARGING )
	{
		if( skipCount > 0 ) {
			mutex_unlock(pMutexTouch);
			return TOUCH_SUCCESS;
		}
	}
	#endif

	#if !defined ( TOUCH_MODEL_C70 ) && !defined ( TOUCH_MODEL_C90NAS ) && !defined ( TOUCH_MODEL_P1B ) && !defined ( TOUCH_MODEL_P1C ) && !defined ( TOUCH_MODEL_YG ) && !defined ( TOUCH_MODEL_C100N )/* TBD : temporally block ( C70 is not ready ) */
	if( pDriverData->bootMode == BOOT_MINIOS ) /* MFTS BOOT */
	{
		TOUCH_DBG("MFTS\n");
		if(event == FB_EARLY_EVENT_BLANK)
		{
			if (*blank == FB_BLANK_POWERDOWN)
			{
				MftsTouchOnOff(pDriverData, 0);
			}
			else if(*blank == FB_BLANK_UNBLANK)
			{
				/* do nothing */
			}
		}
		else if(event == FB_EVENT_BLANK)
		{
			if (*blank == FB_BLANK_POWERDOWN)
			{
				/* do nothing */
			}
			else if (*blank == FB_BLANK_UNBLANK)
			{
				MftsTouchOnOff(pDriverData, 1);
			}
		}
	}
	else /* NORMAL BOOT */
	#endif
	{
		if(event == FB_EARLY_EVENT_BLANK)
		{
			if (*blank == FB_BLANK_POWERDOWN)
			{
				#if defined ( TOUCH_DEVICE_S3320 ) || defined ( TOUCH_DEVICE_MIT200 )
				cancel_delayed_work_sync(&pDriverData->work_init);
				#endif
			}
			else if(*blank == FB_BLANK_UNBLANK)
			{
				#if defined ( TOUCH_DEVICE_S3320 )
				pDriverData->lpwgSetting.lcdState = 1;
				pDriverData->nextState = DecideNextDriverState(&pDriverData->lpwgSetting);
				pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, pDriverData->nextState, &pDriverData->lpwgSetting);
				SetDriverState(pDriverData, pDriverData->nextState);
				#endif

				#if defined ( TOUCH_DEVICE_MIT200 )
				queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
				#endif
			}
		}
		else if(event == FB_EVENT_BLANK)
		{
			if (*blank == FB_BLANK_POWERDOWN)
			{
				#if defined ( TOUCH_DEVICE_S3320 ) || defined ( TOUCH_DEVICE_MIT200 )
				release_all_touch_event(pDriverData);
				pDriverData->lpwgSetting.lcdState = 0;
				pDriverData->nextState = DecideNextDriverState(&pDriverData->lpwgSetting);
				pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, pDriverData->nextState, &pDriverData->lpwgSetting);
				SetDriverState(pDriverData, pDriverData->nextState);
				#endif

			}
			else if (*blank == FB_BLANK_UNBLANK)
			{
				#if defined ( TOUCH_DEVICE_S3320 )
				queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
				#endif
			}
		}
	}

	mutex_unlock(pMutexTouch);

	return TOUCH_SUCCESS;

}
#elif defined ( TOUCH_PLATFORM_MSM8210 )
//===========================================
// 1. Sequence : LCD ==> FB_EVENT_BLANK
// 2. Suspend : FB_BLANK_UNBLANK
// 3. Resune : FB_BLANK_POWERDOWN
//===========================================
static int touch_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = (struct fb_event *)data;
	TouchDriverData *pDriverData = container_of(self, TouchDriverData, fb_notif);
	int *blank = NULL;

	if (evdata && evdata->data) {
		blank = (int *)evdata->data;
	} else {
		return TOUCH_SUCCESS;
	}
		
	mutex_lock(pMutexTouch);

	if(event == FB_EVENT_BLANK)
	{
		if (*blank == FB_BLANK_POWERDOWN)
		{
			TOUCH_FUNC_EXT("Suspend\n");
			if( pDriverData->isSuspend == TOUCH_TRUE ) {
				mutex_unlock(pMutexTouch);
				return TOUCH_SUCCESS;
			} else {
				pDriverData->isSuspend = TOUCH_TRUE;
			}
		}
		else if (*blank == FB_BLANK_UNBLANK)
		{
			TOUCH_FUNC_EXT("Resume\n");
			if( pDriverData->isSuspend == TOUCH_FALSE ) {
				mutex_unlock(pMutexTouch);
				return TOUCH_SUCCESS;
			} else {
				pDriverData->isSuspend = TOUCH_FALSE;
			}
		}
	}
	
	if( pDriverData->bootMode == BOOT_MINIOS ) /* MFTS BOOT */
	{
		TOUCH_DBG("MFTS\n");
		if(event == FB_EVENT_BLANK)
		{
			if (*blank == FB_BLANK_POWERDOWN)
			{
				MftsTouchOnOff(pDriverData, 0);
			}
			else if (*blank == FB_BLANK_UNBLANK)
			{
				MftsTouchOnOff(pDriverData, 1);
			}
		}
	}
	else /* NORMAL BOOT */
	{
		if(event == FB_EVENT_BLANK)
		{
			if (*blank == FB_BLANK_POWERDOWN)
			{
				/* do nothing */
			}
			else if (*blank == FB_BLANK_UNBLANK)
			{
				/* do nothing */
			}
		}
	}
	
	TOUCH_FUNC_EXT("Exit\n");
	
	mutex_unlock(pMutexTouch);
	
	return TOUCH_SUCCESS;
	
}
#else
#error "Platform should be defined"
#endif

#endif

static int touch_pm_suspend(struct device *dev)
{
	#if defined ( TOUCH_DEVICE_LU202X )
	mutex_lock(pMutexTouch);
	#endif

	TOUCH_FUNC();

#if defined ( TOUCH_PLATFORM_QCT ) && !defined ( TOUCH_DEVICE_LU202X )
	atomic_set(&pm_state, PM_SUSPEND);
#endif

	return TOUCH_SUCCESS;
}

static int touch_pm_resume(struct device *dev)
{
	TOUCH_FUNC();

	#if defined ( TOUCH_DEVICE_LU202X )
	mutex_unlock(pMutexTouch);
	#endif

#if defined ( TOUCH_PLATFORM_QCT ) && !defined ( TOUCH_DEVICE_LU202X )
	if (atomic_read(&pm_state) == PM_SUSPEND_IRQ) {
		TouchDriverData *pDriverData = dev_get_drvdata(dev);
		struct irq_desc *desc;

		desc = irq_to_desc(pDriverData->client->irq);
		if (desc == NULL) {
			TOUCH_ERR("Null Pointer from irq_to_desc!\n");
			return -ENOMEM;
		}

		atomic_set(&pm_state, PM_RESUME);

		irq_set_pending(pDriverData->client->irq);
		check_irq_resend(desc, pDriverData->client->irq);

		return TOUCH_SUCCESS;
	}

	atomic_set(&pm_state, PM_RESUME);
#endif

	return TOUCH_SUCCESS;
}

static int touch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	TouchDriverData *pDriverData;
	struct attribute **specific_attribute_list = NULL;

	TOUCH_FUNC();

	/* memory allocation for globally used driver data */
	pDriverData = devm_kzalloc(&client->dev, sizeof(TouchDriverData), GFP_KERNEL);
	if( pDriverData == NULL ) {
		TOUCH_ERR("fail to allocation memory\n");
		return -ENOMEM;
	}

	/* get boot mode for later */
	pDriverData->bootMode = TouchGetBootMode();

	/* set driver state to boot */
	SetDriverState(pDriverData, STATE_BOOT);

	/* set driver data to i2c client data */
	pDriverData->client = client;
	i2c_set_clientdata(client, pDriverData);

	/* get model configuratio */
	TouchGetModelConfig(pDriverData);

	/* register input device */
	ret  = register_input_dev(pDriverData);
	if( ret  == TOUCH_FAIL ) {
		TOUCH_ERR("failed at register_input_dev()\n");
		goto exit_register_input_dev_fail;
	}

	/* register sysfs ( core driver + TouchIC specific ) */
	specific_attribute_list = pDeviceSpecificFunc->device_attribute_list;
	ret = sysfs_register(pDriverData, specific_attribute_list);
	if( ret  < 0 ) {
		TOUCH_ERR("failed at sysfs_register()\n");
		goto exit_sysfs_register_fail;
	}

	/* initialise system resource ( mutex, wakelock, work function ) */
	mutex_init(&pDriverData->thread_lock);

	INIT_DELAYED_WORK(&pDriverData->work_upgrade, WqfirmwareUpgrade);
	INIT_DELAYED_WORK(&pDriverData->work_irq, WqTouchIrqHandler);
	INIT_DELAYED_WORK(&pDriverData->work_init, WqTouchInit);

	wake_lock_init(&pDriverData->lpwg_wake_lock, WAKE_LOCK_SUSPEND, "touch_lpwg");

	/* store system resource to global variable */
	pMutexTouch = &pDriverData->thread_lock;
	pWakeLockTouch = &pDriverData->lpwg_wake_lock;
	pWorkTouch = &pDriverData->work_irq;

	ret = pDeviceSpecificFunc->Initialize(client);
	if( ret  == TOUCH_FAIL ) {
		goto exit_device_init_fail;
	}

	ret = pDeviceSpecificFunc->InitRegister(client);
	if( ret  == TOUCH_FAIL ) {
		goto exit_device_init_fail;
	}
	
	ret = pDeviceSpecificFunc->ReadIcFirmwareInfo(client, &pDriverData->icFwInfo);
	if( ret  == TOUCH_FAIL ) {
		goto exit_device_init_fail;
	}
	
	ret = pDeviceSpecificFunc->GetBinFirmwareInfo(client, NULL, &pDriverData->binFwInfo);
	if( ret  == TOUCH_FAIL ) {
		goto exit_device_init_fail;
	}

	if( pDriverData->bootMode != BOOT_OFF_CHARGING ){
		/* Register & Disable Interrupt */
		ret = TouchRegisterIrq(pDriverData, (irq_handler_t)TouchIrqHandler);
		if( ret  == TOUCH_FAIL ) {
			goto exit_device_init_fail;
		}
	}else{
		TouchDisableIrq();
	}

	#if defined ( ENABLE_TOUCH_AT_OFF_CHARGING )
	if( pDriverData->bootMode == BOOT_OFF_CHARGING ) {
		pDriverData->lpwgSetting.mode = 0; /* there is no CFW, but we need to save power */
	}
	#else
	if( pDriverData->bootMode == BOOT_OFF_CHARGING )
	{
		TOUCH_LOG("power off charging mode so don't probe touch\n");

		#if defined ( TOUCH_DEVICE_S3320 )
		
		/*
		** we want lowest current consumption but there is no-way in MTK
		** "suspend" and "resume" are not called at power off charing in MTK
		*/
		SetDriverState(pDriverData, STATE_NORMAL);
		
		#else
		
		/* lowest current consumption first ( so disable touch ) */
		pDriverData->nextState = STATE_OFF;
		pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, pDriverData->nextState, &pDriverData->lpwgSetting);
		SetDriverState(pDriverData, pDriverData->nextState);
		
		#endif

		/* do not probe */
		goto exit_device_init_fail;
		
	}
	#endif
	
	/* set and register "suspend" and "resume" */
	#if defined ( CONFIG_HAS_EARLYSUSPEND )
	pDriverData->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	pDriverData->early_suspend.suspend = touch_early_suspend;
	pDriverData->early_suspend.resume = touch_late_resume;
	register_early_suspend(&pDriverData->early_suspend);
	#elif defined( CONFIG_FB )
	pDriverData->fb_notif.notifier_call = touch_fb_notifier_callback;
	fb_register_client(&pDriverData->fb_notif);
	#endif

	/* Set driver state to normal */
	SetDriverState(pDriverData, STATE_NORMAL);

	/* Trigger firmware upgrade if needed */
	if( pDriverData->icFwInfo.isOfficial )
	{
		if( pDriverData->binFwInfo.version != pDriverData->icFwInfo.version )
		{
			pDriverData->useDefaultFirmware = TOUCH_TRUE;
			queue_delayed_work(touch_wq, &pDriverData->work_upgrade, 0);
		} else {
			queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
		}
	}

	return TOUCH_SUCCESS;

exit_device_init_fail:
	
	wake_lock_destroy(&pDriverData->lpwg_wake_lock);
	
	sysfs_unregister(pDriverData);

exit_sysfs_register_fail:
	
	unregister_input_dev(pDriverData);

exit_register_input_dev_fail:
	
	devm_kfree(&pDriverData->client->dev, pDriverData);
	
	return TOUCH_FAIL;

}

static int touch_remove(struct i2c_client *client)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	TOUCH_FUNC();

	free_irq(pDriverData->client->irq, pDriverData);

	wake_lock_destroy(&pDriverData->lpwg_wake_lock);

	#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&pDriverData->early_suspend);
	#elif defined(CONFIG_FB)
	fb_unregister_client(&pDriverData->fb_notif);
	#endif

	sysfs_unregister(pDriverData);

	unregister_input_dev(pDriverData);
	
	devm_kfree(&pDriverData->client->dev, pDriverData);

	return TOUCH_SUCCESS;
	
}


static struct i2c_device_id lge_ts_id[] = {
	{LGE_TOUCH_NAME, 0},
};

static struct dev_pm_ops touch_pm_ops = {
	.suspend = touch_pm_suspend,
	.resume = touch_pm_resume,
};

static struct of_device_id match_table[] = {
	{ .compatible = "unified_driver,ver2", },
	{ },
};

static struct i2c_driver lge_touch_driver = {
	.probe = touch_probe,
	.remove = touch_remove,
	.id_table = lge_ts_id,
	.driver = {
		.name = LGE_TOUCH_NAME,
		.owner = THIS_MODULE,
		.pm = &touch_pm_ops,
		.of_match_table = &match_table[0],
	},
};

static int __init touch_init(void)
{
	int result = TOUCH_SUCCESS;
	int ret = 0;
	int idx = 0;
	TouchDeviceSpecificFunction **pDeviceFunc = NULL;

	TOUCH_FUNC();

	TouchGetDeviceSpecificDriver(&pDeviceFunc);

	ret = TouchInitializePlatform();
	if( ret == TOUCH_FAIL ) {
		return -ENODEV;
	}

	#if defined ( TOUCH_PLATFORM_MTK )
	i2c_register_board_info(0, &i2c_tpd, 1);
	#endif

	/* turn on the power */
	TouchPower(1);

	/* reset */
	pDeviceFunc[0]->Reset(NULL);

	while( pDeviceFunc[idx] != NULL )
	{
		/* check if device is connected */
		result = pDeviceFunc[idx]->Connect();
		if( result == TOUCH_SUCCESS ) {
			break;
		}
		idx++;
	}

	if( pDeviceFunc[idx] == NULL ) {
		TOUCH_ERR("No device connected\n");
		return -ENODEV;
	}

	pDeviceSpecificFunc = pDeviceFunc[idx];

	touch_wq = create_singlethread_workqueue("touch_wq");
	if( touch_wq == NULL ) {
		TOUCH_ERR("failed to create workqueue\n");
		return -ENODEV;
	}

	if( i2c_add_driver ( &lge_touch_driver ) ) {
		TOUCH_ERR("failed at i2c_add_driver()\n" );
		destroy_workqueue(touch_wq);
		return -ENODEV;
	}

	return TOUCH_SUCCESS;
	
}

static void __exit touch_exit(void)
{
	TOUCH_FUNC();

	i2c_del_driver(&lge_touch_driver);

	if (touch_wq) {
		destroy_workqueue(touch_wq);
	}
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("D3 BSP Touch Team");
MODULE_DESCRIPTION("LGE Touch Unified Driver");
MODULE_LICENSE("GPL");

/* End Of File */

