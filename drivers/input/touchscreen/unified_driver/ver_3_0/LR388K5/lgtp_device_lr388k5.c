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
 *    File  	: lgtp_device_lr388k5.c
 *    Author(s)   : BSP Touch Team
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[LR388K5]"

/****************************************************************************
 * Include Files
 ****************************************************************************/
#include <linux/input/unified_driver_3/lgtp_common.h>

#include <linux/input/unified_driver_3/lgtp_common_driver.h>
#include <linux/input/unified_driver_3/lgtp_platform_api.h>
#include <linux/input/unified_driver_3/lgtp_device_lr388k5.h>

#include <linux/fs.h>
#include "shtsc_ioctl.h"

/****************************************************************************
 * Manifest Constants / Defines
 ****************************************************************************/
/* INT STATUS */
#define SHTSC_STATUS_TOUCH_READY    (1<<0)
#define SHTSC_STATUS_POWER_UP       (1<<1)
#define SHTSC_STATUS_RESUME_PROX    (1<<2)
#define SHTSC_STATUS_WDT            (1<<3)
#define SHTSC_STATUS_DCMAP_READY    (1<<4)
#define SHTSC_STATUS_COMMAND_RESULT (1<<5)
#define SHTSC_STATUS_LG_LPWG_TCI1   (1<<6)
#define SHTSC_STATUS_LG_LPWG_TCI2   (1<<7)
#define SHTSC_STATUS_FLASH_LOAD_ERROR    (1<<8)
#define SHTSC_STATUS_PLL_UNLOCK     (1<<9)

/* DONE IND */
#define SHTSC_IND_CMD   0x20
#define SHTSC_IND_TOUCH 0x01
#define SHTSC_IND_DCMAPDONE 0x10

/* BANK Address */
#define SHTSC_BANK_TOUCH_REPORT   0x00
#define SHTSC_BANK_LPWG_DATA      0x00
#define SHTSC_BANK_LPWG_PARAM     0x01
#define SHTSC_BANK_COMMAND        0x02
#define SHTSC_BANK_COMMAND_RESULT 0x03
#define SHTSC_BANK_DCMAP          0x05

/* Common Register Address */
#define SHTSC_ADDR_INT0  0x00
#define SHTSC_ADDR_INTMASK0 0x01
#define SHTSC_ADDR_BANK 0x02
#define SHTSC_ADDR_IND  0x03
#define SHTSC_ADDR_INT1  0x04
#define SHTSC_ADDR_INTMASK1 0x05

/* Touch Report Register Address */
#define SHTSC_ADDR_TOUCH_NUM 0x08
#define SHTSC_ADDR_RESUME_PROX 0x09
#define SHTSC_ADDR_TOUCH_REPORT 0x10
#define SHTSC_ADDR_LPWG_REPORT 0x10

/* Touch Parmeters */
#define SHTSC_MAX_FINGERS 10
#define SHTSC_MAX_TOUCH_1PAGE 10
#define SHTSC_LENGTH_OF_TOUCH 8
#define SHTSC_LENGTH_OF_LPWG 4

/* Touch Status */
#define SHTSC_F_TOUCH ((u8)0x01)
#define SHTSC_F_TOUCH_OUT ((u8)0x03)
//#define SHTSC_P_TOUCH ((u8)0x02)
//#define SHTSC_P_TOUCH_OUT ((u8)0x04)

#define SHTSC_TOUCHOUT_STATUS ((u8)0x80)

#define SHTSC_ADDR_COMMAND 0x08

#define CMD_GETPROPERTY "\xE0\x00\x00\x11"
#define CMD_GETPROPERTY_LEN 4

#define CMD_SETSYSTEMSTATE_SLEEP "\x02\x00\x01\x00\x00"
#define CMD_SETSYSTEMSTATE_SLEEP_LEN 5
#define CMD_SETSYSTEMSTATE_DEEPIDLE "\x02\x00\x01\x00\x04"
#define CMD_SETSYSTEMSTATE_DEEPIDLE_LEN 5
#define CMD_SETSYSTEMSTATE_IDLE "\x02\x00\x01\x00\x03"
#define CMD_SETSYSTEMSTATE_IDLE_LEN 5
#define CMD_GETSYSTEMSTATE "\x03\x00\x00\x01"
#define CMD_GETSYSTEMSTATE_LEN 4

#define CMD_SETSYSTEMSTATE_PEN_MODE "\x02\x00\x01\x00\x08"
#define CMD_SETSYSTEMSTATE_PEN_MODE_LEN 5

#define CMD_SETSYSTEMSTATE_NORMAL_MODE "\x02\x00\x01\x00\x03"
#define CMD_SETSYSTEMSTATE_NORMAL_MODE_LEN 5


//2014.11.20 added
//

#define CMD_DELAY             16

#define WAIT_NONE   (0)
#define WAIT_CMD    (1)
#define WAIT_RESET  (2)

#define MAX_16BIT			0xffff
#define MAX_12BIT			0xfff

#define MAX_DCMAP_SIZE (37*37*2)

#define LOW_LEVEL 0
#define HIGH_LEVEL 1

#define MAX_COMMAND_RESULT_LEN (64-8)

#define SHTSC_DEVBUF_SIZE 1500

#define FLASH_WAIT 500
//#define FIRMWARE_SIZE (44*1024)

#define KNOCKDATA_SIZE (40)

/* KNOCK ON/CODE FAILURE STATUS */
#define SHTSC_KNOCKON_FAILURE_DISTANCE_INTER_TAP    (1<<0)
#define SHTSC_KNOCKON_FAILURE_DISTANCE_TOUCHSLOP    (1<<1)
#define SHTSC_KNOCKON_FAILURE_TIMEOUT_INTERTAP      (1<<2)
#define SHTSC_KNOCKON_FAILURE_MULTI_FINGER          (1<<3)
#define SHTSC_KNOCKON_FAILURE_DELAY_TIME            (1<<4)
#define SHTSC_KNOCKON_FAILURE_PALM_STATE            (1<<5)

#define KNOCK_CODE_FAILURE_REASONS (\
                                    SHTSC_KNOCKON_FAILURE_DISTANCE_INTER_TAP | \
                                    SHTSC_KNOCKON_FAILURE_DISTANCE_TOUCHSLOP | \
                                    SHTSC_KNOCKON_FAILURE_TIMEOUT_INTERTAP   | \
                                    SHTSC_KNOCKON_FAILURE_MULTI_FINGER       | \
                                    SHTSC_KNOCKON_FAILURE_DELAY_TIME         | \
                                    SHTSC_KNOCKON_FAILURE_PALM_STATE         )
                                    

/* Software reset delay */
#define SHTSC_RESET_TIME	250	/* msec */
#define SHTSC_SLEEP_TIME	100	/* msec */

/* ======== EEPROM command ======== */
#define	FLASH_CMD_WRITE_EN		(0x06)		/* Write enable command */
#define	FLASH_CMD_PAGE_WR			(0x02)		/* Page write command */
#define	FLASH_CMD_READ_ST			(0x05)		/* Read status command */
#define	FLASH_CMD_SECTOR_ERASE	(0xD7)		/* Sector erase command */
#define	FLASH_CMD_READ			(0x03)		/* Read command */

/* ======== EEPROM status ======== */
#define	FLASH_ST_BUSY		(1 << 0)	/* Busy with a write operation */

#define CMD_SETSYSTEMSTATE_LEN 5
#define CMD_SETSYSTEMSTATE "\x02\x00\x01\x00\xff"

/* ======== Version information  ======== */

#define DEVICE_CODE_LR388K5 2 

#define VERSION_YEAR 15
#define VERSION_MONTH 6
#define VERSION_DAY 12
#define VERSION_SERIAL_NUMBER 20
#define VERSION_MODEL_CODE DEVICE_CODE_LR388K5
#define DRIVER_VERSION_LEN 5

#define DRIVER_NAME "shtsc"
/* ======= SelfDiagnosis ====== */
#define RESULT_DATA_MAX_LEN (8 * 1024)
#define FILE_PATH_RESULT "/mnt/sdcard/touch_self_test.txt"

/****************************************************************************
 * Macros
 ****************************************************************************/
#define DBGLOG TOUCH_LOG

//#define DEBUG_SHTSC 0

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

typedef enum _cmd_state_e {
	CMD_STATE_SLEEP         = 0x00,
	CMD_STATE_IDLE          = 0x03,
	CMD_STATE_DEEP_IDLE     = 0x04,
	CMD_STATE_HOVER         = 0x06,  // if applicable
	CMD_STATE_PROX          = 0x07,  // if applicable
	CMD_STATE_GLOVE         = 0x08,  // if applicable
	CMD_STATE_COVER         = 0x0D,  // if applicable
	CMD_STATE_MAX
} dCmdState_e ;

/*
 * The touch driver structure.
 */
struct shtsc_touch {
	u8 status;
	u8 id;
	u8 size;
	u8 type;
	u16 x;
	u16 y;
	u16 z;
};


struct shtsc_i2c {
	u8 cmd;//2014.11.19 added
	u8 wait_state;//2014.11.19 added
	bool wait_result;//2014.11.19 added
	u8 disabled;		/* interrupt status */ //2014.11.6 added
	struct input_dev *input;
	struct shtsc_i2c_pdata *pdata;//2014.10.16 added
	char phys[32];
	struct i2c_client *client;
	int reset_pin;
	int irq_pin;
	struct shtsc_touch touch[SHTSC_MAX_FINGERS];
	struct mutex mutex;
	//#ifdef FORCE_FIRM_UPDATE
	struct work_struct workstr;
	struct workqueue_struct *workqueue;
	//#endif /* FORCE_FIRM_UPDATE */
	LpwgSetting lpwgSetting;

	unsigned int            max_num_touch;
	int                     min_x;
	int                     min_y;
	int                     max_x;
	int                     max_y;
	int                     pressure_max;
	int                     touch_num_max;
	bool                    flip_x;
	bool                    flip_y;
	bool                    swap_xy;
};

/****************************************************************************
 * Variables
 ****************************************************************************/
//static const char defaultFirmware[] = "leadingUI/dummy_firmware.img";

//static const char defaultFirmware[] = "sharp/p1v/PLG449-V1.08-PR1888337-DS5.2.13.0.1014_40052188.img";
// 120Hz
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f1A01_p1A03_150718_ext_h.bin";
// 60Hz
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f1A01_p1A04_150723_ext_h.bin";
// 120Hz
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f1A01_p1A05_150723_ext_h.bin";
//
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f1A01_p1A06_150724_ext_h.bin";
//
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f1A01_p1A07_150724_ext_h.bin";
//
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f1A01_p1A08_150724_ext_h.bin";
// Timing change
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f1A01_p1A0A_150804_ext_h.bin";
// Self D Testing
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_fF0001A03_p1A00_150801_ext_h.bin";
// Display Timing
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f1A01_p1A0C_150806_ext_h.bin";
// 0808 FW check
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f1A01_p1A0D_150807_ext_h.bin";
// 0821 Pen improvment
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f1A03_p1A03_150821_ext_h.bin";
// 0824 CG 0.4T
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f1A04_p1A03_150821_ext_h.bin";
// 0825 Pressure change
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_fFFFF1A04_p1A04_150825_ext_h.bin";
// 0825 Pen support
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_fFFFF1A04_p1A05_150825_ext_h.bin";
// 0826 Pen pressure range
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_fFFFE1A04_p1A07_150826_ext_h.bin";
// 0826 Pen pressure range & stablization
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_fFFF11A04_p1A08_150826_ext_h.bin";
// 0827 Remove the multi touch limit
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_fFFF21A04_p1A0A_150827_ext_h.bin";
// 0902 LPWG setting changed
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_fF0601A04_p1A02_150901_ext_h.bin";
// 0908 LPWG setting changed
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_fF1201A04_p1A01_150908_ext_h.bin";
// 0908 LPWG setting fixed valued 
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_fF1201A04_p1A01_150908_ext_h_debug.bin";
// 0909 Knock on / Knock code test 
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f04072_p1A01_150909_ext_h_debug2.bin";
// 0909 Knock on / Knock code fixed version 
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f04073_p1A01_150909_ext_h_debug3.bin";
// 0910 Knock on / Knock code fixed version with WDT issue 
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f4531_p1A01_150910_ext_h.bin";
// 0910 Knock on / Knock code working without WDT issue
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f4541_p1A01_150910_ext_h_TG270ms.bin";
// 0911 Knock on / Knock code register setting from the device driver 
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f4580_p1A01_150911_ext_h.bin";
// 0915 Knock on / Knock code register with HWR improvement 
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f6020_pF0FF1A01_150915_ext_h.bin";
// 0915 LPWG fail reason bug fixed 
//static const char defaultFirmware[] = "sharp/p1v/LR388K5_f6030_pF0FF1A01_150915_ext_h.bin";
// 0916 LPWG fail reason bug fixed 2
static const char defaultFirmware[] = "sharp/p1v/LR388K5_f6050_pF0FF1A01_150915_ext_h.bin";

int	G_reset_done = false;
u16 G_Irq_Mask = 0xffff;
unsigned G_touch=0;

unsigned char resumeStatus; // bank 0, address 9

unsigned char CommandResultBuf[MAX_COMMAND_RESULT_LEN];

u8 dcmapBuf[MAX_DCMAP_SIZE+128];
u8 dcmap[31*18*2+128]; // greater than 17*32*2


volatile static int buf_pos;
static u8 devbuf[SHTSC_DEVBUF_SIZE];

struct shtsc_i2c g_ts;

#define REP_SIZE (4+4+4+120)
#define MAX_REPORTS 14

unsigned char reportBuf[4+ MAX_REPORTS*REP_SIZE];
volatile unsigned char Mutex_GetReport = false;

int G_pen_mode = 0;

int current_page = 0;

#define IO_BUF_SIZE 4*1024
char iobuf[IO_BUF_SIZE];

#define SHTSC_BUF_SIZE 16*1024
u8 s_shtsc_addr;
u8 s_shtsc_buf[SHTSC_BUF_SIZE];
unsigned long s_shtsc_len;

pid_t pid = 0;

#define NUM_DRIVE (26)
#define NUM_SENSE (15-1)

int lpwg_fail_reason = 0;
unsigned int lpwg_fail_reason_mask = KNOCK_CODE_FAILURE_REASONS ;

//====================================================================
// NORMAL : general touch(finger,key) is working
// OFF : touch is not working even knock-on ( lowest power saving )
// KNOCK_ON_ONLY : knock-on is only enabled
// KNOCK_ON_CODE : knock-on and knock-code are enabled
// NORMAL_HOVER : hover detection is enabled and general touch is working
// HOVER : only hover detection is enabled
//====================================================================
static TouchState dummyDeviceState = STATE_UNKNOWN;

/****************************************************************************
 * Extern Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Local Function Prototypes
 ****************************************************************************/
int flash_access_start_shtsc(void *_ts);
int flash_access_end_shtsc(void *_ts);
int flash_erase_page_shtsc(void *_ts, int page);
int flash_write_page_shtsc(void *_ts, int page, unsigned char *data);
int flash_verify_page_shtsc(void *_ts, int page, unsigned char *data);
int flash_read(void *_ts, unsigned address, unsigned length, unsigned char *data);
static int WriteMultiBytes(void *ts, u8 u8Addr, u8 *data, int len);
static int WriteOneByte(void *ts, u8 u8Addr, u8 u8Val);
static u8 ReadOneByte(void *ts, u8 u8Addr);
static void ReadMultiBytes(void *ts, u8 u8Addr, u16 u16Len, u8 *u8Buf);
static int tci_control(struct i2c_client *client, int type, unsigned int value);
int issue_command(void *ts, unsigned char *cmd, unsigned int len);
int lpwg_param_command(void *ts, u8 u8Addr, unsigned char *cmd, unsigned int len);

static void LR388K5_Reset(struct i2c_client *client);
static int LR388K5_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen);

static int lr388k5_misc_open(struct inode *inode, struct file *file);
static int lr388k5_misc_release(struct inode *inode, struct file *file);
static ssize_t lr388k5_misc_read(struct file *file, char *buf, size_t count, loff_t *pos);
static long lr388k5_misc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static int WaitAsync(void *_ts);

/****************************************************************************
 * Local Functions
 ****************************************************************************/


/****************************************************************************
 * Device Specific Functions
 ****************************************************************************/
static int write_result_data_to_file(char* filename, char *data, int datalen){
	int fd = 0;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	if(filename == NULL) {
		fd = sys_open(FILE_PATH_RESULT, O_WRONLY | O_CREAT | O_APPEND, 0666);
		TOUCH_LOG(" SelfD Test default result file has been opened size %d \n", datalen);
	} else {
		fd = sys_open(filename, O_WRONLY | O_CREAT , 0666);
		if(fd == 0 ){
			TOUCH_ERR(" SelfD Test result file %s has not been created  \n", filename);
			return 0;
		} else {
			TOUCH_LOG(" SelfD Test result file %s has been opened \n", filename);
		}
	}
	if( fd > 0 ) {
		sys_write(fd, data, datalen);
		sys_close(fd);
	}
	set_fs(old_fs);

	return datalen;
}


static int shtsc_system_init(void *ts)
{
	struct input_dev *input_dev = ((struct shtsc_i2c *)ts)->input;

	TOUCH_FUNC();

	//HMOON
	return 0;

	{
		unsigned int cnt;
		for(cnt=0;cnt<SHTSC_MAX_FINGERS;cnt++){
			input_mt_slot(input_dev, cnt);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("shtsc_system_init() clear mt_slot(%d)\n",cnt);
#endif
		}
	}
	input_report_key(input_dev, BTN_TOUCH, false );

	input_sync(input_dev);
	return 0;
}

#if 0
static void shtsc_reset_delay(void)
{
	msleep(SHTSC_RESET_TIME);
}

static void shtsc_reset(void *_ts, bool reset)
{
	struct shtsc_i2c *ts = _ts;

	TOUCH_FUNC();

	if (ts->reset_pin) {
		G_reset_done = reset;
		if (reset) {
			shtsc_reset_delay();

		}
		//TOUCH_LOG("shtsc: shtsc_reset: %d\n", reset);
		if(reset = false){
			TouchResetCtrl(0);
			G_Irq_Mask = 0xffff;
		} else {
			TouchResetCtrl(1);
		}

		gpio_direction_output(ts->reset_pin, reset);
		if (! reset) {
			G_Irq_Mask = 0xffff;
		}
	}
}

#endif

static int WriteMultiBytes(void *_ts, u8 u8Addr, u8 *data, int len)
{
	struct shtsc_i2c *ts = _ts;

	return Touch_I2C_Write(ts->client, u8Addr, data, len);
}

static int WriteOneByte(void *_ts, u8 u8Addr, u8 u8Val)
{
	struct shtsc_i2c *ts = _ts;
	u8 wData[1];
	wData[0] = u8Val;

	if( (ts == NULL) ){
		TOUCH_LOG("ts is NULL error\n");
		return 0;
	}

	return Touch_I2C_Write_Byte(ts->client, u8Addr, u8Val);
}

static u8 ReadOneByte(void *_ts, u8 u8Addr)
{  
	struct shtsc_i2c *ts = _ts;
	u8 rData[1+1]; //requires one more byte to hold

	Touch_I2C_Read_Byte(ts->client, u8Addr, rData);
	return rData[0];
}

static void ReadMultiBytes(void *_ts, u8 u8Addr, u16 u16Len, u8 *u8Buf)
{
	struct shtsc_i2c *ts = _ts;

	if( (ts == NULL) || (ts->client == NULL) || (u8Buf == NULL) ){
		TOUCH_LOG("ts is NULL or client is NULL or u8Buf is NULL error\n");
		return ;
	}

	Touch_I2C_Read(ts->client, u8Addr, u8Buf, u16Len);
}

static void SetBankAddr(void *_ts, u8 u8Bank)
{
	struct shtsc_i2c *ts = _ts;

	if( (ts == NULL) ){
		TOUCH_LOG("ts is NULL error\n");
		return ;
	}

#if 1
	TOUCH_LOG("SetBankAddr %d \n", u8Bank);
#endif

	WriteOneByte(ts, SHTSC_ADDR_BANK, u8Bank);  
}

static void Sharp_ClearInterrupt(void *_ts, u16 u16Val)
{
	struct shtsc_i2c *ts = _ts;

	//TOUCH_FUNC();
	TOUCH_LOG(" Sharp_ClearInterrupt %04X \n", u16Val);

	if(u16Val & 0x00FF)
		WriteOneByte(ts, SHTSC_ADDR_INT0, (u16Val & 0x00FF));
	if((u16Val & 0xFF00) >> 8)
		WriteOneByte(ts, SHTSC_ADDR_INT1, ((u16Val & 0xFF00) >> 8));
}

static void SetIndicator(void *_ts, u8 u8Val)
{
	struct shtsc_i2c *ts = _ts;

	WriteOneByte(ts, SHTSC_ADDR_IND, u8Val);
}


int flash_access_start_shtsc(void *_ts)
{
	struct shtsc_i2c *ts = _ts;

	// mask everything
	WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
	msleep(100);

	/* TRIM_OSC = 0 */
	WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x18); // bank change reg
	WriteOneByte(ts, (unsigned char)0x11, (unsigned char)0x19);
	return 0;
}

int flash_access_end_shtsc(void *_ts)
{
	struct shtsc_i2c *ts = _ts;

	TOUCH_FUNC();

	//HMOON
	//return 0;

	msleep(100);

	TouchResetCtrl(0);
	G_Irq_Mask = 0xffff;

	//shtsc_reset(ts, false);

	msleep(100);

	//shtsc_reset(ts, true);
	TouchResetCtrl(1);

	msleep(10);
	shtsc_system_init(ts);

	msleep(100);

	return 0;
}

#define RETRY_COUNT (2000*10) //experimental
int flash_erase_page_shtsc(void *_ts, int page)
{
	struct shtsc_i2c *ts = _ts;

	//  int chan= 0;
	volatile unsigned char readData;
	int retry=0;

	/* BankChange */
	//SPI_ArrieRegWrite(0x02,0x1C);
	WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x1C); // bank change reg

	/* CLKON_CTL0 */
	//SPI_ArrieRegWrite(0x14,0x22);	/* CLKON_FLC,CLKON_REGBUS */
	WriteOneByte(ts, (unsigned char)0x14, (unsigned char)0x22);

	/* FLC_CTL CS_HIGH,WP_DISABLE */
	//	SPI_ArrieRegWrite(0x3C,0x14);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);

	/* FLC_CTL CS_LOW,WP_DISABLE */
	//	SPI_ArrieRegWrite(0x3C,0x16);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);
	/* FLC_TxDATA WRITE_ENABLE */
	//SPI_ArrieRegWrite(0x3D,SPI_CMD_WRITE_EN);
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)FLASH_CMD_WRITE_EN);
	/* FLC_CTL CS_HIGH,WP_DISABLE */
	//	SPI_ArrieRegWrite(0x3C,0x14);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);

	/* FLC_CTL CS_LOW,WP_DISABLE */
	//	SPI_ArrieRegWrite(0x3C,0x16);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);


	/* FLC_TxDATA CHIP_ERASE_COMMAND */
	//	SPI_ArrieRegWrite(0x3D,SPI_CMD_CHIP_ERASE);
	// not a chip erase, but a sector erase for the backward compatibility!!
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)FLASH_CMD_SECTOR_ERASE);
	// 24bit address. 4kByte=001000H. 00x000H:x=0-f -> 4kB*16=64kB
	// for K6 or more than 64kB, 0(0x page)000 : page:00H-1FH for 128kB
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)((page>>4)&0xFF));//20150912
	// K5 code below
	//WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)((page<<4)&0xFF));
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);

	/* FLC_CTL CS_HIGH,WP_DISABLE */
	//	SPI_ArrieRegWrite(0x3C,0x14);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);


	/* wait until 'BUSY = LOW' */
	do {
		retry++;
		////		msleep(10);
		if (retry > RETRY_COUNT)
			goto RETRY_ERROR;

		/* FLC_CTL CS_LOW,WP_DISABLE */
		//		SPI_ArrieRegWrite(0x3C,0x16);
		WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);
		/* FLC_TxDATA READ_STATUS_COMMAND*/
		//		SPI_ArrieRegWrite(0x3D,SPI_CMD_READ_ST);
		WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_READ_ST);
		/* Dummy data */
		//		SPI_ArrieRegWrite(0x3D,0);
		WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
		/* FLC_RxDATA */
		//		readData = SPI_ArrieRegRead(0x3F);
		readData = ReadOneByte(ts, 0x3F);
		/* FLC_CTL CS_HIGH,WP_DISABLE */
		//		SPI_ArrieRegWrite(0x3C,0x14);
		WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);
	} while (readData & FLASH_ST_BUSY); 		/* check busy bit */

	return 0;

RETRY_ERROR:
	TOUCH_LOG("FATAL: flash_erase_page_shtsc retry %d times for page %d - FAILED!\n", retry, page);
	return 1;
}
#define FLASH_PAGE_SIZE (4<<10) // 4k block for each page
#define FLASH_PHYSICAL_PAGE_SIZE (256) // can write 256bytes at a time.

int flash_write_page_shtsc(void *_ts, int page, unsigned char *data)
{
	struct shtsc_i2c *ts = _ts;

	//  int chan = 0;
	int retry = 0;
	//  unsigned addr; // in-page address (from 0 to FLASH_PAGE_SIZE-1)
	unsigned paddr; // address (32 or 64kB area)
	volatile unsigned char readData;
	int cnt, idx;

	/* BankChange */
	//	SPI_ArrieRegWrite(0x02,0x1C);
	WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x1C); // bank change reg

	/* CLKON_CTL0 */
	//	SPI_ArrieRegWrite(0x14,0x22);	/* CLKON_FLC,CLKON_REGBUS */
	WriteOneByte(ts, (unsigned char)0x14, (unsigned char)0x22);

	/* FLC_CTL */
	//	SPI_ArrieRegWrite(0x3C,0x14);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);

	/* 256 bytes / Flash write page, 4kByte / logical(virtual) flash page */
	for (cnt = 0; cnt < (FLASH_PAGE_SIZE / FLASH_PHYSICAL_PAGE_SIZE); cnt++) {
		paddr = (page * FLASH_PAGE_SIZE) + (cnt * FLASH_PHYSICAL_PAGE_SIZE);
		// 4k page offset + in-page offset. 4k*n+256*m

		/* FLC_CTL CS_LOW,WP_DISABLE */
		//		SPI_ArrieRegWrite(0x3C,0x16);
		WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);
		/* FLC_TxDATA WRITE_ENABLE */
		//		SPI_ArrieRegWrite(0x3D,SPI_CMD_WRITE_EN);
		WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_WRITE_EN);
		/* FLC_CTL CS_HIGH,WP_DISABLE */
		//		SPI_ArrieRegWrite(0x3C,0x14);
		WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);

		/* FLC_CTL CS_LOW,WP_DISABLE */
		//		SPI_ArrieRegWrite(0x3C,0x16);
		WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);

		/* FLC_TxDATA PAGE_PROGRAM_COMMAND */
		//		SPI_ArrieRegWrite(0x3D,SPI_CMD_PAGE_WR);
		WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_PAGE_WR);
		/* FLC_TxDATA Address(bit16~23) */
		//		SPI_ArrieRegWrite(0x3D,((address>>16)&0xFF));
		WriteOneByte(ts, (unsigned char)0x3D, ((paddr>>16)&0xFF));
		/* FLC_TxDATA Address(bit8~15) */
		//		SPI_ArrieRegWrite(0x3D,((address>>8)&0xFF));
		WriteOneByte(ts, (unsigned char)0x3D, ((paddr>>8)&0xFF));
		/* FLC_TxDATA Address(bit0~7) */
		//		SPI_ArrieRegWrite(0x3D,(address&0xFF));
		WriteOneByte(ts, (unsigned char)0x3D, (paddr&0xFF));
		/* Data write 1page = 256byte */
		for(idx=0;idx<256;idx++){
			//			SPI_ArrieRegWrite(0x3D,*pData++);
			// addr=in-page(virtual, in 4k block) 256xN
			WriteOneByte(ts, (unsigned char)0x3D, data[(cnt*FLASH_PHYSICAL_PAGE_SIZE) +idx]);
		}
		/* FLC_CTL CS_HIGH,WP_DISABLE */
		//		SPI_ArrieRegWrite(0x3C,0x14);
		WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);


		/* wait until 'BUSY = LOW' */
		do {
			retry++;
			////		  msleep(10);
			if (retry > RETRY_COUNT)
				goto RETRY_ERROR;

			/* FLC_CTL CS_LOW,WP_DISABLE */
			//		SPI_ArrieRegWrite(0x3C,0x16);
			WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);
			/* FLC_TxDATA READ_STATUS_COMMAND*/
			//		SPI_ArrieRegWrite(0x3D,SPI_CMD_READ_ST);
			WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_READ_ST);
			/* Dummy data */
			//		SPI_ArrieRegWrite(0x3D,0);
			WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
			/* FLC_RxDATA */
			//		readData = SPI_ArrieRegRead(0x3F);
			readData = ReadOneByte(ts, 0x3F);
			/* FLC_CTL CS_HIGH,WP_DISABLE */
			//		SPI_ArrieRegWrite(0x3C,0x14);
			WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);
		} while (readData & FLASH_ST_BUSY); 		/* check busy bit */
	}

	return 0;

RETRY_ERROR:
	TOUCH_LOG("FATAL: flash_write_page_shtsc retry %d times for page %d, addr %04X - FAILED!\n", retry, page, paddr);
	return 1;
}

#define FLASH_VERIFY_SIZE 512
unsigned char readBuf[FLASH_VERIFY_SIZE];

int flash_verify_page_shtsc(void *_ts, int page, unsigned char *data)
{
	struct shtsc_i2c *ts = _ts;
	unsigned addr; // in-page address (from 0 to FLASH_PAGE_SIZE-1)
	int cnt;

	/* BankChange */
	//	SPI_ArrieRegWrite(0x02,0x1C);
	WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x1C); // bank change reg

	/* CLKON_CTL0 */
	//	SPI_ArrieRegWrite(0x14,0x22);	/* CLKON_FLC,CLKON_REGBUS */
	WriteOneByte(ts, (unsigned char)0x14, (unsigned char)0x22);

	/* FLC_CTL CS_HIGH,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x10);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);

	/* FLC_CTL CS_LOW,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x12);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x12);

	/* FLC_TxDATA READ_COMMAND*/
	//	SPI_ArrieRegWrite(0x3D,SPI_CMD_READ);
	WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_READ);
	/* FLC_TxDATA Address(bit16~23) */
	//	SPI_ArrieRegWrite(0x3D,((address>>16)&0xFF));
	
	WriteOneByte(ts, (unsigned char)0x3D, ((page>>4)&0xFF)); //20150912
	//	K5 code below
	//WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);

	/* FLC_TxDATA Address(bit8~15) */
	//	SPI_ArrieRegWrite(0x3D,((address>>8)&0xFF));
	WriteOneByte(ts, (unsigned char)0x3D, ((page<<4)&0xFF));
	/* FLC_TxDATA Address(bit0~7) */
	//	SPI_ArrieRegWrite(0x3D,(address&0xFF));
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
	/* FLC_TxDATA Dummy data */
	//	SPI_ArrieRegWrite(0x3D,0);
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);

	for (addr = 0; addr < FLASH_PAGE_SIZE; addr += FLASH_VERIFY_SIZE) {
		for(cnt=0; cnt<FLASH_VERIFY_SIZE; cnt++){
			/* FLC_RxDATA */
			//		*pData++ = SPI_ArrieRegRead(0x3F);
			readBuf[cnt] = ReadOneByte(ts, 0x3F);
		}
		if (memcmp((unsigned char *)&(data[addr]), (unsigned char *)readBuf, FLASH_VERIFY_SIZE)) {
			goto VERIFY_ERROR;
		}
	}

	/* FLC_CTL CS_LOW,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x10);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);

	return 0;

VERIFY_ERROR:
	/* FLC_CTL CS_LOW,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x10);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);
	// verify error
	TOUCH_LOG("FATAL: flash_verify_page_shtsc for page %d - FAILED!\n", page);

	return 1;
}

int flash_read(void *_ts, unsigned address, unsigned length, unsigned char *data)
{
	struct shtsc_i2c *ts = _ts;
	int cnt;

	/* BankChange */
	//	SPI_ArrieRegWrite(0x02,0x1C);
	WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x1C); // bank change reg

	/* CLKON_CTL0 */
	//	SPI_ArrieRegWrite(0x14,0x22);	/* CLKON_FLC,CLKON_REGBUS */
	WriteOneByte(ts, (unsigned char)0x14, (unsigned char)0x22);

	/* FLC_CTL CS_HIGH,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x10);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);

	/* FLC_CTL CS_LOW,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x12);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x12);

	/* FLC_TxDATA READ_COMMAND*/
	//	SPI_ArrieRegWrite(0x3D,SPI_CMD_READ);
	WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_READ);
	/* FLC_TxDATA Address(bit16~23) */
	//	SPI_ArrieRegWrite(0x3D,((address>>16)&0xFF));
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)((address>>16)&0xFF));
	/* FLC_TxDATA Address(bit8~15) */
	//	SPI_ArrieRegWrite(0x3D,((address>>8)&0xFF));
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)((address>>8)&0xFF));
	/* FLC_TxDATA Address(bit0~7) */
	//	SPI_ArrieRegWrite(0x3D,(address&0xFF));
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)(address&0xFF));
	/* FLC_TxDATA Dummy data */
	//	SPI_ArrieRegWrite(0x3D,0);
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);

	for(cnt=0; cnt<length; cnt++){
		/* FLC_RxDATA */
		//		*pData++ = SPI_ArrieRegRead(0x3F);
		data[cnt] = ReadOneByte(ts, 0x3F);
	}

	/* FLC_CTL CS_LOW,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x10);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);

	return 0;
}

#define FLASH_WAIT 500

/*
 * force re-programming the firm image held in the driver
 */
int update_flash(void *_ts, unsigned char *data, unsigned int len)
{
	int page;

	TOUCH_LOG("shtsc: force updating K5 firmware....\n");
	TOUCH_LOG("shtsc: flash_access start\n");
	flash_access_start_shtsc(_ts);

	for (page = 0; page < (len/FLASH_PAGE_SIZE); page++) {
		msleep(FLASH_WAIT);
		flash_erase_page_shtsc(_ts, page);
		TOUCH_LOG("shtsc: flash_erase_page_shtsc done: page %d\n",  page);
		msleep(FLASH_WAIT);
		flash_write_page_shtsc(_ts, page, &(data[page*FLASH_PAGE_SIZE]));
		TOUCH_LOG("shtsc: flash_write_page_shtsc done: page %d\n",  page);
		msleep(FLASH_WAIT);
		flash_verify_page_shtsc(_ts, page, &(data[page*FLASH_PAGE_SIZE]));
		TOUCH_LOG("shtsc: flash_verify_page_shtsc done: page %d\n",  page);
	}

	TOUCH_LOG("shtsc: flash_access end\n");
	//flash_access_end_shtsc(_ts);

	TOUCH_LOG("shtsc: force updating K5 firmware....done\n");

	return 0;
}

static void GetTouchReport(void *ts, u8 u8Num, u8 *u8Buf, TouchReadData *pData  )
{
	struct shtsc_touch *touch = ((struct shtsc_i2c *)ts)->touch;
	//struct input_dev *input_dev = ((struct shtsc_i2c *)ts)->input;

	int i;
	u8 ID,Status;
	int touchNum = 0;//2014.11.12 added

	//TOUCH_FUNC();

	if( u8Num > SHTSC_MAX_TOUCH_1PAGE ){
#if defined(DEBUG_SHTSC)
		TOUCH_LOG("shtsc touch number erro (num=%d)\n",u8Num);
#endif
		return;
	}


	for(i = 0;i < u8Num;i++){
		Status = (u8Buf[i * SHTSC_LENGTH_OF_TOUCH] & 0xF0);
		touch[i].id = ID = (u8Buf[i * SHTSC_LENGTH_OF_TOUCH] & 0x0F );
		touch[i].size = u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 1] & 0x3F;
		touch[i].type = (u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 1] & 0xC0) >> 6;
		touch[i].x =
			(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 2] << 0) |
			(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 3] << 8);
		touch[i].y =
			(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 4] << 0) |
			(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 5] << 8);
		touch[i].z =
			(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 6] << 0) |
			(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 7] << 8);


#if defined(DEBUG_SHTSC)
		TOUCH_LOG("shtsc ID=%2d, Status=%02x, Size=%3d, X=%5d, Y=%5d, z=%5d, num=%2d\n"
				,ID
				,Status
				,touch[i].size
				,touch[i].x
				,touch[i].y
				,touch[i].z
				,u8Num);
#endif

		//      input_mt_slot(input_dev, ID);
		if(Status & SHTSC_TOUCHOUT_STATUS){
			//TOUCH_LOG(" release event \n");
			touch[i].status = SHTSC_F_TOUCH_OUT;
			continue;
		}

		pData->fingerData[i].id =  touch[i].id;
		pData->fingerData[i].x =  touch[i].x;
		pData->fingerData[i].y =  touch[i].y;
		pData->fingerData[i].pressure = ((touch[i].z) > 4095) ? (0xff) : ((touch[i].z & 0x0ff0)>>4);
		pData->fingerData[i].width_major =  touch[i].size;
		pData->fingerData[i].width_minor =  touch[i].size;
		//pData->fingerData[i].orientation =  touch[i].;

		touchNum++;

		//		touch[i].status = SHTSC_F_TOUCH;
	}

	pData->type = DATA_FINGER;
	pData->count = touchNum;
}

static irqreturn_t shtsc_irq_thread(int irq, void *_ts, TouchReadData *pData) 
{
	struct shtsc_i2c *ts = _ts;

	u16 u16status;
	u8 u8Num = 0;
	u8 tmpbuf[128];
	u8 numDriveLine2, numSenseLine2;
	u8 num_adc_dmy[3];
	u8 regcommonbuf[11];

	//TOUCH_FUNC();

	//return IRQ_HANDLED;

#if defined(DEBUG_SHTSC)
	if (G_touch) {
		TOUCH_LOG("[IRQ] shtsc touch-ready %d\n", G_touch);
	}
#endif

#if defined(DEBUG_SHTSC)
	TOUCH_LOG("[ENTER] shtsc_irq\n");
#endif

	G_reset_done = true;

	/* Get Interrupt State */
	ReadMultiBytes(ts,SHTSC_ADDR_INT0,11,regcommonbuf);
	u16status = ((regcommonbuf[SHTSC_ADDR_INT1] << 8) | regcommonbuf[SHTSC_ADDR_INT0]);

	//TOUCH_ERR("[IRQ] shtsc_irq STATUS [ 0x%04X ] IRQ Mask (0x%04x) \n", u16status, G_Irq_Mask);

	u16status &= G_Irq_Mask;
#if defined(DEBUG_SHTSC)
	if ((u16status != 0x0001) && (u16status != 0x0000)) {
		TOUCH_ERR("[IRQ] shtsc_irq: %04X\n", u16status);
	}
#endif

	//TOUCH_ERR("[IRQ] shtsc_irq STATUS [ 0x%04X ] \n", u16status);

	while(u16status != 0){

		// PLL_UNLOCK
		// 1<<9
		if (u16status & SHTSC_STATUS_PLL_UNLOCK) {
			//
			// PLL unlock
			//

#if  1 // defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc_irq PLL_UNLOCK: %04X\n", u16status);
#endif
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc pll-unlock\n");
#endif
			// mask it
			WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
			regcommonbuf[SHTSC_ADDR_INTMASK1] = 0x03;
			// from now on, no int for this
			G_Irq_Mask &= ~SHTSC_STATUS_PLL_UNLOCK;

			/* Clear Interrupt */
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_PLL_UNLOCK);
			u16status &= ~SHTSC_STATUS_PLL_UNLOCK;

#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc_irq PLL_UNLOCK: %04X\n", u16status);
#endif
		}

		// POWER_UP
		// 1 << 1
		if (u16status & SHTSC_STATUS_POWER_UP) {
			//
			// Power-up
			//

#if  1 //defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc power-up\n");
#endif

			/* Clear Interrupt */
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_POWER_UP);
			u16status &= ~SHTSC_STATUS_POWER_UP;

			if (ts->wait_state == WAIT_RESET) {//2014.11.20 added
				ts->wait_state = WAIT_NONE;
				ts->wait_result = true;
			} 
		}
		// WDT
		//
		if (u16status & SHTSC_STATUS_WDT) {
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_WDT);
			u16status &= ~SHTSC_STATUS_WDT;
			TOUCH_ERR("[IRQ] shtsc_irq WDT");

			// WDT make LR388K5 power recyle so that it needs PLL_UNLOCK handling
			G_Irq_Mask = 0xffff;
		}
		// RESUME_PROX
		//
		if (u16status & SHTSC_STATUS_RESUME_PROX) {
			//
			// Resume from DeepIdle
			// or
			// PROXIMITY
			//

#if defined(DEBUG_SHTSC)
			TOUCH_ERR("[IRQ] shtsc resume from DeepIdle or prox\n");
#endif

			SetBankAddr(ts, SHTSC_BANK_TOUCH_REPORT);
			resumeStatus = ReadOneByte(ts, SHTSC_ADDR_RESUME_PROX);

			/* Clear Interrupt */
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_RESUME_PROX);
			u16status &= ~SHTSC_STATUS_RESUME_PROX;

		}
		// FLASH_LOAD_ERROR
		//
		if (u16status & SHTSC_STATUS_FLASH_LOAD_ERROR) {
			//
			// FLASH_LOAD_ERROR
			// occurs when flash is erased
			// nothing can be done
			//
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc flash load error\n");
#endif
			if (ts->wait_state == WAIT_RESET) {//2014.11.20 added
				ts->wait_state = WAIT_NONE;
				ts->wait_result = true;
			} 
			// from now on, no int for this
			G_Irq_Mask &= ~SHTSC_STATUS_FLASH_LOAD_ERROR;

			/* Clear Interrupt */
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_FLASH_LOAD_ERROR);
			u16status &= ~SHTSC_STATUS_FLASH_LOAD_ERROR;
		}
		// TOUCH_READY
		//
		if (u16status & SHTSC_STATUS_TOUCH_READY) {
			//
			// Touch report
			//

#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc touch-ready cleared\n");
#endif


			/* Get number of touches */
			{
				u8 u8Buf[128];
				if( regcommonbuf[SHTSC_ADDR_BANK] != SHTSC_BANK_TOUCH_REPORT ){
					WriteOneByte(ts, SHTSC_ADDR_BANK, SHTSC_BANK_TOUCH_REPORT);
					ReadMultiBytes(ts,SHTSC_ADDR_TOUCH_NUM,3,regcommonbuf+SHTSC_ADDR_TOUCH_NUM);
				}
				u8Num = regcommonbuf[SHTSC_ADDR_TOUCH_NUM];

#if defined(DEBUG_SHTSC)
				TOUCH_LOG("shtsc touch num=%d\n", u8Num);
#endif

				/* Retrieve touch report */
				if (u8Num > 0){
					ReadMultiBytes((struct shtsc_i2c *)ts, SHTSC_ADDR_TOUCH_REPORT, SHTSC_LENGTH_OF_TOUCH * u8Num, u8Buf);
				}
				/* Clear Interrupt */
				u16status &= ~SHTSC_STATUS_TOUCH_READY;
				regcommonbuf[SHTSC_ADDR_INT0] = SHTSC_STATUS_TOUCH_READY;
				regcommonbuf[SHTSC_ADDR_BANK] = SHTSC_BANK_TOUCH_REPORT;
				regcommonbuf[SHTSC_ADDR_IND] = SHTSC_IND_TOUCH;
				WriteMultiBytes(ts,SHTSC_ADDR_INT0,regcommonbuf,4);
				if (u8Num > 0){
					GetTouchReport(ts, u8Num, u8Buf, pData);
				}

				//HMOON
				//input_sync(ts->input);
			}
		}

		// COMMAND_RESULT
		//
		if (u16status & SHTSC_STATUS_COMMAND_RESULT) {
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc command result\n");
#endif
			SetBankAddr(ts,SHTSC_BANK_COMMAND_RESULT);
			ReadMultiBytes(ts, 0x08, MAX_COMMAND_RESULT_LEN, CommandResultBuf); // always read entire result. TEGAPP will handle it.

#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc command result, %02X, %02X, %02X\n", CommandResultBuf[0], CommandResultBuf[1], CommandResultBuf[2]);
#endif


			/* Clear Interrupt */
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_COMMAND_RESULT);
			u16status &= ~SHTSC_STATUS_COMMAND_RESULT;

			if (ts->wait_state == WAIT_CMD) {
				if ((CommandResultBuf[0] != ts->cmd) || (CommandResultBuf[1] != 0)) {
					ts->wait_state = WAIT_NONE;
					ts->wait_result = false;
				} else {
					ts->wait_state = WAIT_NONE;
					ts->wait_result = true;
				}
			}
		}
		// DCAMP_READY
		//
		if (u16status & SHTSC_STATUS_DCMAP_READY) {
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc DCMAP READY\n");
#endif
			/* Clear Interrupt */
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_DCMAP_READY);
			u16status &= ~SHTSC_STATUS_DCMAP_READY;

			{ // L2
				unsigned char dsFlag, readingSenseNum, ram_addr[2];
				unsigned vramAddr;
				unsigned readingSize;

				// get SD/DS and size
				ram_addr[0] = 0x58;
				ram_addr[1] = 0xBF;
				WriteMultiBytes(ts,(unsigned char)0x06,ram_addr,2);

				SetBankAddr(ts,SHTSC_BANK_DCMAP);
				ReadMultiBytes(ts, 0x08, 5, tmpbuf);

				numSenseLine2 = tmpbuf[0];
				numDriveLine2 = tmpbuf[1];

				dsFlag = tmpbuf[4]; // 1 for DS, 0 for SD
				vramAddr = ((tmpbuf[3]<<8) | tmpbuf[2]);
				// readingSenseNum is greater or equal to itself, but a multiply of 4
				readingSenseNum = (unsigned char)((numSenseLine2+3)/4); // not a double but int
				readingSenseNum *= 4;

				readingSize = readingSenseNum * numDriveLine2 * 2; /* 2:16bit */

				num_adc_dmy[0] = num_adc_dmy[2] = 0; //top and left have no 0-filled lines
				num_adc_dmy[1] = readingSenseNum - numSenseLine2; //right side

				// read DCmap values from register
				// store it to read buffer memory for read action
				{ // L1
					/* read 120 bytes from Bank5, address 8 */
					/* read loop required */
					int bytes = readingSize;
					int size;
					int index = 0;
					//SetBankAddr(ts,SHTSC_BANK_DCMAP);

					//	      TOUCH_LOG("%s(%d):readingSize:%d\n", __FILE__, __LINE__, readingSize);
					while (bytes > 0) {
						ram_addr[0] = (unsigned char)(vramAddr&0xff); // address to read (Lower)
						ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // address to read (Higher)
						WriteMultiBytes(ts,(unsigned char)0x06,ram_addr,2);

						size = ((bytes >= 120) ? 120 : bytes);
						//		TOUCH_LOG("%s(%d):bytes:%d, size:%d, index:%d, vramAddr:%x\n", __FILE__, __LINE__, bytes, size, index, vramAddr);

						ReadMultiBytes(ts, 0x08, size, &(dcmapBuf[index]));
						index += size;
						bytes -= size;
						vramAddr += size;
					} // while
				} // L1

#if 0
				TOUCH_LOG("DCmap Drive x Sense: %d x %d, dsFlag:%02X, num_adc_dmy:%02X %02X %02X\n", numDriveLine2, numSenseLine2, dsFlag, num_adc_dmy[0], num_adc_dmy[1], num_adc_dmy[2]);
#endif /* 0 */

				{ //L3
					int sindex = 0, dindex = 0;
					int l, x, y;
#if 0 // debug
					static unsigned char frm_count = 0;
#endif /* 1 */

					// dcmap header
					// [0]: horizontal data num (in short, not byte)
					// [1]: vertical data num

					x = dcmap[dindex++] = numSenseLine2;
					y = dcmap[dindex++] = numDriveLine2;
					dcmap[dindex++] = dsFlag;
#if 0 // debug
					dcmap[dindex++] = frm_count++; // just for debug
#else /* 1 */
					dcmap[dindex++] = 0x00; // reserved
#endif /* 1 */

					//top
					sindex = (num_adc_dmy[0] + x + num_adc_dmy[1]) * num_adc_dmy[2] * 2;

					// contents line
					for (l = 0; l < y; l++) {
						// left
						sindex += (num_adc_dmy[0] * 2);

						// contents
						memcpy((u8 *)&(dcmap[dindex]), (u8 *)&(dcmapBuf[sindex]), (x*2));
						dindex += (x*2);
						sindex += (x*2);

						// right
						sindex += (num_adc_dmy[1] * 2);
					}

					// for read()
					//	      TOUCH_LOG("check buf_pos: %d\n", buf_pos);
					if (buf_pos == 0) {
						memcpy((u8 *)devbuf, (u8 *)dcmap, (4+x*y*2));
						//		TOUCH_LOG("setting buf_pos: %d\n", buf_pos);
						buf_pos = (4+x*y*2);
						//		TOUCH_LOG("set buf_pos: %d\n", buf_pos);
					}

#if 0 // DCmap debug
					TOUCH_LOG("DC map size HxV: %d x %d = %d\n", dcmap[0], dcmap[1], dcmap[0]*dcmap[1]);
					TOUCH_LOG("[0-3, %d-%d]: %02X %02X %02X %02X, %02X %02X %02X %02X\n", (x*y-4), (x*y-1), dcmap[4], dcmap[5], dcmap[6], dcmap[7], dcmap[x*y*2+4-4], dcmap[x*y*2+4-3], dcmap[x*y*2+4-2], dcmap[x*y*2+4-1]);
#endif /* 0 */
#if 0 // DCmap debug
					for (j = 0; j < y; j++) {
						for (i = 0; i < x; i++) {
							TOUCH_LOG("%d: %02X ", (y*j+i), dcmap[y*j + i + 4]);
						}
						TOUCH_LOG("\n");
					}
#endif
				} //L3
			} // L2 DEVICE_LR388K5 block
			//*************************
		}

		// SHTSC_STATUS_TCI_1(KNOCK_ON)
		if (u16status & SHTSC_STATUS_LG_LPWG_TCI1) {
            u8 buffer[3] = {0};
            u8 failureReason;

			Sharp_ClearInterrupt(ts, SHTSC_STATUS_LG_LPWG_TCI1);
			u16status &= ~(SHTSC_STATUS_LG_LPWG_TCI1);
			TOUCH_LOG("[IRQ] shtsc_irq KNOCK_ON\n");

            SetBankAddr(&g_ts, SHTSC_BANK_LPWG_PARAM);
            ReadMultiBytes(&g_ts, FAILURE_REASON_REG, 1, buffer);
            failureReason = buffer[0] & 0x0F; // now it's ready
            TOUCH_LOG("TCI1 Failure Reason [%d]\n", failureReason);

            switch (failureReason) {
            case 0:
              TOUCH_LOG("TCI1 SUCCESS\n");
              pData->type = DATA_KNOCK_ON;
              break;
            case 1:
              TOUCH_LOG("TCI1 FAIL - DISTANCE_INTER_TAP\n");
              break;
            case 2:
              TOUCH_LOG("TCI1 FAIL - DISTANCE_TOUCHSLOP\n");
              break;
            case 3:
              TOUCH_LOG("TCI1 FAIL - TIMEOUT_INTERTAP\n");
              break;
            case 4:
              TOUCH_LOG("TCI1 FAIL - MULTI_FINGER\n");
              break;
            case 5:
              TOUCH_LOG("TCI1 FAIL - DELAY_TIME\n");
              break;
            case 6:
              TOUCH_LOG("TCI1 FAIL - PALM_STATE\n");
              break;
            default:
              TOUCH_LOG("TCI1 FAIL - RESERVED\n");
              break;
            }
		}
		// SHTSC_STATUS_TCI_2(KNOCK_CODE)
		if (u16status & SHTSC_STATUS_LG_LPWG_TCI2) {
            u8 buffer[3] = {0};
            u8 failureReason;
			int i;
			struct shtsc_touch *touch = ((struct shtsc_i2c *)ts)->touch;
			u8 knockData_buf[KNOCKDATA_SIZE];
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_LG_LPWG_TCI2);
			u16status &= ~(SHTSC_STATUS_LG_LPWG_TCI2);
            TOUCH_LOG("[IRQ] shtsc_irq KNOCK_CODE count %d\n", g_ts.lpwgSetting.tapCount);

            SetBankAddr(&g_ts, SHTSC_BANK_LPWG_PARAM);
            ReadMultiBytes(&g_ts, FAILURE_REASON_REG, 1, buffer);
            failureReason = (buffer[0] & 0xF0) >> 4; // now it's ready
            TOUCH_LOG("TCI1 Failure Reason [%d]\n", failureReason);

            switch (failureReason) {
            case 0:
              TOUCH_LOG("TCI2 SUCCESS\n");
              break;
            case 1:
              TOUCH_LOG("TCI1 FAIL - DISTANCE_INTER_TAP\n");
              break;
            case 2:
              TOUCH_LOG("TCI1 FAIL - DISTANCE_TOUCHSLOP\n");
              break;
            case 3:
              TOUCH_LOG("TCI1 FAIL - TIMEOUT_INTERTAP\n");
              break;
            case 4:
              TOUCH_LOG("TCI1 FAIL - MULTI_FINGER\n");
              break;
            case 5:
              TOUCH_LOG("TCI1 FAIL - DELAY_TIME\n");
              break;
            case 6:
              TOUCH_LOG("TCI1 FAIL - PALM_STATE\n");
              break;
            default:
              TOUCH_LOG("TCI1 FAIL - RESERVED\n");
              break;
            }

          if (failureReason == 0) { // SUCCESS
			pData->type = DATA_KNOCK_CODE;
			SetBankAddr(ts,SHTSC_BANK_LPWG_DATA);
			ReadMultiBytes(ts,SHTSC_ADDR_LPWG_REPORT,KNOCKDATA_SIZE,knockData_buf);

			pData->count = g_ts.lpwgSetting.tapCount;

			for (i = 0; i < g_ts.lpwgSetting.tapCount; i++) {
				touch[i].x =
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG    ] << 0) |
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 1] << 8);
				touch[i].y =
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 2] << 0) |
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 3] << 8);

				pData->knockData[i].x =  touch[i].x;
				pData->knockData[i].y =  touch[i].y;
			}
          }
		}
		if (u16status != 0) {
			TOUCH_LOG("[IRQ] shtsc unknown interrupt status %04X\n", u16status);
			/* Clear all interrupts and mask.  */
			WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xFF);
			WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
			u16status = 0;
			Sharp_ClearInterrupt(ts, u16status);
		}
	}

	if (u8Num != 0 ) {
#if defined(DEBUG_SHTSC)
		//TOUCH_LOG( "shtsc flush touch input(%d)\n", u8Num);
#endif
		/*
		   input_report_key(ts->input, BTN_TOUCH, touchNum?true:false );

		   input_sync(ts->input);
		   */
	}

#if defined(DEBUG_SHTSC)
	TOUCH_LOG("[IRQ] shtsc_irq done \n");
#endif
	return IRQ_HANDLED;
}

static ssize_t store_tci(struct i2c_client *client,
	const char *buf, size_t count)
{
	u32 type = 0, tci_num = 0, value = 0;

	sscanf(buf, "%d %d %d", &type, &tci_num, &value);

	tci_control(client, type, (unsigned int)value);

	return count;
}

static ssize_t show_tci(struct i2c_client *client, char *buf)
{
	int ret = 0;
	u8 buffer[3] = {0};

	SetBankAddr(&g_ts, SHTSC_BANK_LPWG_PARAM);

	ReadMultiBytes(&g_ts, REPORT_RATE_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Hz [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));

	ReadMultiBytes(&g_ts, SENSITIVITY_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Sensitivity [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));


	ReadMultiBytes(&g_ts, ACTIVE_AREA_X1_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "ActiveArea X1 [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, ACTIVE_AREA_Y1_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "ActiveArea Y1 [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, ACTIVE_AREA_X2_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "ActiveArea X2 [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, ACTIVE_AREA_Y2_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "ActiveArea Y2 [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));


	WRITE_SYSBUF(buf, ret, "TCI - 1\n");
	ReadMultiBytes(&g_ts, SHTSC_ADDR_INTMASK0, 1, buffer);
	WRITE_SYSBUF(buf, ret, "TCI [%s]\n",
		((buffer[0] & SHTSC_STATUS_LG_LPWG_TCI1) == SHTSC_STATUS_LG_LPWG_TCI1 ) ? "disabled" : "enabled");
	ReadMultiBytes(&g_ts, TOUCH_SLOP_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Touch Slop [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, TAP_DISTANCE_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Tap Distance [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, MIN_INTERTAP_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Min InterTap [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, MAX_INTERTAP_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Max InterTap [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, TAP_COUNT_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Tap Count [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, INTERRUPT_DELAY_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Interrupt Delay [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));

	ReadMultiBytes(&g_ts, FAILURE_REASON_REG, 1, buffer);
	WRITE_SYSBUF(buf, ret, "Failure Reason [%d]\n",
		buffer[0] & 0x0F);
	ReadMultiBytes(&g_ts, FAILURE_INT_ENABLE_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Failure INT Enable [0x%2X]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, FAILURE_INT_STATUS_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Failure INT Status [0x%2X]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));

	WRITE_SYSBUF(buf, ret, "TCI - 2\n");
	ReadMultiBytes(&g_ts, SHTSC_ADDR_INTMASK0, 1, buffer);
	WRITE_SYSBUF(buf, ret, "TCI [%s]\n",
		((buffer[0] & SHTSC_STATUS_LG_LPWG_TCI2) == SHTSC_STATUS_LG_LPWG_TCI2 ) ? "disabled" : "enabled");
	ReadMultiBytes(&g_ts, TOUCH_SLOP_CTRL2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Touch Slop [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, TAP_DISTANCE_CTRL2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Tap Distance [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, MIN_INTERTAP_CTRL2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Min InterTap [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, MAX_INTERTAP_CTRL2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Max InterTap [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, TAP_COUNT_CTRL2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Tap Count [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, INTERRUPT_DELAY_CTRL2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Interrupt Delay [%d]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));

	ReadMultiBytes(&g_ts, FAILURE_REASON_REG, 1, buffer);
	WRITE_SYSBUF(buf, ret, "Failure Reason [%d]\n",
		(buffer[0] & 0xF0) >> 4);
	ReadMultiBytes(&g_ts, FAILURE_INT_ENABLE2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Failure INT Enable [0x%2X]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, FAILURE_INT_STATUS2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Failure INT Status [0x%2X]\n",
		(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));

	return ret;
}

static ssize_t show_lpwg_touch_data(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int i;
	u8 total_touch_points;
	u8 knockData_buf[KNOCKDATA_SIZE];

	if((dummyDeviceState == STATE_KNOCK_ON_ONLY)||(dummyDeviceState == STATE_KNOCK_ON_CODE)){

		SetBankAddr(&g_ts,SHTSC_BANK_LPWG_DATA);
		ReadMultiBytes(&g_ts , SHTSC_ADDR_TOUCH_NUM , 1 , &total_touch_points);
		ReadMultiBytes(&g_ts , SHTSC_ADDR_LPWG_REPORT , KNOCKDATA_SIZE , knockData_buf);
	
		total_touch_points = total_touch_points & 0xF;
	
		WRITE_SYSBUF(buf, ret, "total_touch_points [%d]\n",total_touch_points);
	
		for (i = 0; i < total_touch_points; i++) {
			WRITE_SYSBUF(buf, ret, "lpwg_touch_data[%d] x = [%d] y = [%d] \n",i,
				(knockData_buf[i * SHTSC_LENGTH_OF_LPWG] << 0) |(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 1] << 8),
				(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 2] << 0) |(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 3] << 8));
		}
	}else{
		WRITE_SYSBUF(buf, ret, "current state is not LPWG mode\n");
	}

	return ret;
}


int shtsc_CMD_SetPenMode(void *ts, int pen_mode)
{
	int count = 0;
	u8 value = 0;

	TOUCH_FUNC();

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	if(pen_mode == 1 ){ // PEN_MODE
		WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_SETSYSTEMSTATE_PEN_MODE, CMD_SETSYSTEMSTATE_PEN_MODE_LEN);
	} else {
		WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_SETSYSTEMSTATE_NORMAL_MODE, CMD_SETSYSTEMSTATE_NORMAL_MODE_LEN);
	}

	SetIndicator(ts, SHTSC_IND_CMD);

	value = ReadOneByte(ts, SHTSC_ADDR_INT0);

	TOUCH_LOG("Value check value %d SHTSC_STATUS_COMMAND_RESULT & value %d \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 10)
			break;
	}
	return 0;
}

static int tci_control(struct i2c_client *client, int type, unsigned int value)
{
	int ret = TOUCH_SUCCESS;
	volatile unsigned char readData;
	u8 buffer[3] = {0};

	//TOUCH_FUNC();

	//HMOON TEST
	//return ret;


	buffer[0] = (u8)(value & 0xFF);
	buffer[1] = (u8)(value >> 8) & 0xFF;

	switch (type) {
	case REPORT_RATE_CTRL:
		lpwg_param_command(&g_ts, REPORT_RATE_CTRL_REG, buffer, 2);
		break;
	case SENSITIVITY_CTRL:
		lpwg_param_command(&g_ts, SENSITIVITY_CTRL_REG, buffer, 2);
		break;

	// Active area
	case ACTIVE_AREA_X1_CTRL:
		lpwg_param_command(&g_ts, ACTIVE_AREA_X1_CTRL_REG, buffer, 2);
		break;
	case ACTIVE_AREA_Y1_CTRL:
		lpwg_param_command(&g_ts, ACTIVE_AREA_Y1_CTRL_REG, buffer, 2);
		break;
	case ACTIVE_AREA_X2_CTRL:
		lpwg_param_command(&g_ts, ACTIVE_AREA_X2_CTRL_REG, buffer, 2);
		break;
	case ACTIVE_AREA_Y2_CTRL:
		lpwg_param_command(&g_ts, ACTIVE_AREA_Y2_CTRL_REG, buffer, 2);
		break;

	// TCI1
	case TCI_ENABLE_CTRL:
		readData = ReadOneByte(&g_ts, SHTSC_ADDR_INTMASK0);
		if (value) {
		  readData &= ~SHTSC_STATUS_LG_LPWG_TCI1;
		} else {
		  readData |= SHTSC_STATUS_LG_LPWG_TCI1;
		}
		WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)readData);
		break;
	case TOUCH_SLOP_CTRL:
		lpwg_param_command(&g_ts, TOUCH_SLOP_CTRL_REG, buffer, 2);
		break;
	case TAP_DISTANCE_CTRL:
		lpwg_param_command(&g_ts, TAP_DISTANCE_CTRL_REG, buffer, 2);
		break;
	case MIN_INTERTAP_CTRL:
		lpwg_param_command(&g_ts, MIN_INTERTAP_CTRL_REG, buffer, 2);
		break;
	case MAX_INTERTAP_CTRL:
		lpwg_param_command(&g_ts, MAX_INTERTAP_CTRL_REG, buffer, 2);
		break;
	case TAP_COUNT_CTRL:
		lpwg_param_command(&g_ts, TAP_COUNT_CTRL_REG, buffer, 2);
		break;
	case INTERRUPT_DELAY_CTRL:
		lpwg_param_command(&g_ts, INTERRUPT_DELAY_CTRL_REG, buffer, 2);
		break;
	case FAILURE_INT_ENABLE:
		lpwg_param_command(&g_ts, FAILURE_INT_ENABLE_REG, buffer, 2);
		break;

	// TCI2
	case TCI_ENABLE_CTRL2:
		readData = ReadOneByte(&g_ts, SHTSC_ADDR_INTMASK0);
		if (value) {
		  readData &= ~SHTSC_STATUS_LG_LPWG_TCI2;
		} else {
		  readData |= SHTSC_STATUS_LG_LPWG_TCI2;
		}
		WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)readData);
		break;
	case TOUCH_SLOP_CTRL2:
		lpwg_param_command(&g_ts, TOUCH_SLOP_CTRL2_REG, buffer, 2);
		break;
	case TAP_DISTANCE_CTRL2:
		lpwg_param_command(&g_ts, TAP_DISTANCE_CTRL2_REG, buffer, 2);
		break;
	case MIN_INTERTAP_CTRL2:
		lpwg_param_command(&g_ts, MIN_INTERTAP_CTRL2_REG, buffer, 2);
		break;
	case MAX_INTERTAP_CTRL2:
		lpwg_param_command(&g_ts, MAX_INTERTAP_CTRL2_REG, buffer, 2);
		break;
	case TAP_COUNT_CTRL2:
		lpwg_param_command(&g_ts, TAP_COUNT_CTRL2_REG, buffer, 2);
		break;
	case INTERRUPT_DELAY_CTRL2:
		lpwg_param_command(&g_ts, INTERRUPT_DELAY_CTRL2_REG, buffer, 2);
		break;
	case FAILURE_INT_ENABLE2:
		lpwg_param_command(&g_ts, FAILURE_INT_ENABLE2_REG, buffer, 2);
		break;

	default:
		TOUCH_ERR("invalid tci param type ( %d )\n", type);
		ret = TOUCH_FAIL;
		break;
	}

	return ret;
}

static int lpwg_control(struct i2c_client *client, TouchState newState)
{
	int ret = TOUCH_SUCCESS;

	TOUCH_FUNC();
	//HMOON 
	//return ret;

	switch (newState) {
	case STATE_NORMAL:
		//tci_control(client, TCI_ENABLE_CTRL, 0);
		//tci_control(client, TCI_ENABLE_CTRL2, 0);

		// To Idle
		//issue_command(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);
		break;

	case STATE_KNOCK_ON_ONLY:
		tci_control(client, TAP_COUNT_CTRL, 2);
		tci_control(client, TAP_COUNT_CTRL2, 4);
		tci_control(client, MIN_INTERTAP_CTRL, 0);
		tci_control(client, MAX_INTERTAP_CTRL, 500);
		tci_control(client, TOUCH_SLOP_CTRL, 100);
		tci_control(client, TAP_DISTANCE_CTRL, 100);
		tci_control(client, INTERRUPT_DELAY_CTRL, 0);

		tci_control(client, ACTIVE_AREA_X1_CTRL, g_ts.lpwgSetting.activeTouchAreaX1);
		tci_control(client, ACTIVE_AREA_Y1_CTRL, g_ts.lpwgSetting.activeTouchAreaY1);
		tci_control(client, ACTIVE_AREA_X2_CTRL, g_ts.lpwgSetting.activeTouchAreaX2);
		tci_control(client, ACTIVE_AREA_Y2_CTRL, g_ts.lpwgSetting.activeTouchAreaY2);

		tci_control(client, FAILURE_INT_ENABLE, lpwg_fail_reason_mask); // KNOCK_CODE_FAILURE_REASONS);
		tci_control(client, TCI_ENABLE_CTRL, 1);
		tci_control(client, TCI_ENABLE_CTRL2, 0);

		// To Deep Idle
		issue_command(&g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);
		break;

	case STATE_KNOCK_ON_CODE:
		tci_control(client, TAP_COUNT_CTRL, 2);
		tci_control(client, MIN_INTERTAP_CTRL, 0);
		tci_control(client, MAX_INTERTAP_CTRL, 500);
		tci_control(client, TOUCH_SLOP_CTRL, 100);
		tci_control(client, TAP_DISTANCE_CTRL, 100);
		tci_control(client, INTERRUPT_DELAY_CTRL, 500);

		tci_control(client, ACTIVE_AREA_X1_CTRL, g_ts.lpwgSetting.activeTouchAreaX1);
		tci_control(client, ACTIVE_AREA_Y1_CTRL, g_ts.lpwgSetting.activeTouchAreaY1);
		tci_control(client, ACTIVE_AREA_X2_CTRL, g_ts.lpwgSetting.activeTouchAreaX2);
		tci_control(client, ACTIVE_AREA_Y2_CTRL, g_ts.lpwgSetting.activeTouchAreaY2);

		tci_control(client, TAP_COUNT_CTRL2, g_ts.lpwgSetting.tapCount);
		tci_control(client, MIN_INTERTAP_CTRL2, 0);
		tci_control(client, MAX_INTERTAP_CTRL2, 500);
		tci_control(client, TOUCH_SLOP_CTRL2, 100);
		tci_control(client, TAP_DISTANCE_CTRL2, 1700);
		tci_control(client, INTERRUPT_DELAY_CTRL2, 0);

		tci_control(client, FAILURE_INT_ENABLE, lpwg_fail_reason_mask); //KNOCK_CODE_FAILURE_REASONS);
		tci_control(client, FAILURE_INT_ENABLE2, lpwg_fail_reason_mask); //KNOCK_CODE_FAILURE_REASONS);
		tci_control(client, TCI_ENABLE_CTRL, 1);
		tci_control(client, TCI_ENABLE_CTRL2, 1);

		// To Deep Idle
		issue_command(&g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);
		break;

	case STATE_OFF:
		tci_control(client, TCI_ENABLE_CTRL, 0);
		tci_control(client, TCI_ENABLE_CTRL2, 0);

		// To Idle
		issue_command(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);
		break;

	default:
		TOUCH_ERR("invalid touch state ( %d )\n", newState);
		ret = TOUCH_FAIL;
		break;

	}

	return ret;
}

int shtsc_CMD_SetSystemStateSleep_wo_IRQ(void *ts)
{
	//int ret;
	int count = 0;
	u8 value = 0;

	TOUCH_FUNC();

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);

	SetIndicator(ts, SHTSC_IND_CMD);

	value = ReadOneByte(ts, SHTSC_ADDR_INT0);
	TOUCH_LOG("Value check value %d SHTSC_STATUS_COMMAND_RESULT & value %d \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 10)
			break;
	}

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	return 0;
}


int shtsc_CMD_GetProperty(
		void *ts,		/**<[in] pointor to struct shtsc_i2c or struct shtsc_spi */
		unsigned char *buf	/**<[in] 17 byte buffer for result */
		)
{
	int count = 0;

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	SetIndicator(ts, SHTSC_IND_CMD);

	//value = ReadOneByte(ts, SHTSC_ADDR_INT0);
	//TOUCH_LOG("Value check value %d SHTSC_STATUS_COMMAND_RESULT & value %d \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 10)
			break;
	}

	SetBankAddr(ts,SHTSC_BANK_COMMAND_RESULT);
	ReadMultiBytes(ts, 0x08, MAX_COMMAND_RESULT_LEN, CommandResultBuf); // always read entire result. TEGAPP will handle it.

#ifdef DEBUG_SHTSC
	TOUCH_LOG("[IRQ] shtsc command result, %02X, %02X, %02X\n", CommandResultBuf[0], CommandResultBuf[1], CommandResultBuf[2]);
#endif

	memcpy(buf,CommandResultBuf+2,17-2);

	//WriteOneByte(ts, SHTSC_ADDR_INT0, SHTSC_STATUS_COMMAND_RESULT);

	return 0;
}

int shtsc_CMD_SetSystemState(void *ts, dCmdState_e eState)
{
	char cmdArray[CMD_SETSYSTEMSTATE_LEN];
	int ret;

	memcpy(cmdArray, CMD_SETSYSTEMSTATE, CMD_SETSYSTEMSTATE_LEN);
	cmdArray[4] = eState;
	ret =  issue_command(ts, cmdArray, CMD_SETSYSTEMSTATE_LEN);
	return ret;
}


/****************************************************************************
 * Device Specific Functions
 ****************************************************************************/


static ssize_t show_pen_mode(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	WRITE_SYSBUF(buf, ret,	"PEN MODE 1, NORMAL MODE 0 :  %d\n", G_pen_mode);

	return ret;
}

static ssize_t store_pen_mode(struct i2c_client *client, const char *buf, size_t count)
{
	int value = 0;

	TOUCH_FUNC();

	sscanf(buf, "%d", &value);

	TOUCH_LOG(" current mode %d, new value %d\n", G_pen_mode, value);

	if(value == 1) {

		if(G_pen_mode == 0){
			shtsc_CMD_SetPenMode(&g_ts, value);
		}
		G_pen_mode = 1;

	} else {

		if(G_pen_mode == 1){
			shtsc_CMD_SetPenMode(&g_ts, value);
		}

		G_pen_mode = 0;
	}

	return count;
}

static ssize_t show_device_name(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	ret += sprintf(buf+ret, "%s\n", "Sharp LR388K5");

	return ret;
}

static ssize_t show_property(struct i2c_client *client, char *buf)
{
	int ret = 0;
	unsigned firmver = 0;
	unsigned paramver = 0;

	TOUCH_FUNC();

	issue_command(&g_ts, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	firmver = ((CommandResultBuf[0x0a-0x08 +3] << 24) |
			(CommandResultBuf[0x0a-0x08 +2] << 16) |
			(CommandResultBuf[0x0a-0x08 +1] << 8) |
			(CommandResultBuf[0x0a-0x08 +0] << 0));
	paramver = ((CommandResultBuf[0x12-0x08 +3] << 24) |
			(CommandResultBuf[0x12-0x08 +2] << 16) |
			(CommandResultBuf[0x12-0x08 +1] << 8) |
			(CommandResultBuf[0x12-0x08 +0] << 0));

	WRITE_SYSBUF(buf, ret,	"Firmware %08X, Parameter %08X\n", firmver, paramver);

	TOUCH_LOG("Firmware %08X, Parameter %08X\n", firmver, paramver);

	return ret;
}


// ONE SHORT CALIBRATION
//#define CMD_EXECCALIBRATION "\x0F\x00\x03\x00\x00\x50\0x00"
// 800 ms ALIBRATION
#define CMD_EXECCALIBRATION "\x0F\x00\x03\x00\x05\x20\0x03"
#define CMD_EXECCALIBRATION_LEN 7

int shtsc_CMD_ExecCalibration_wo_IRQ(void *ts)
{
	//int ret;
	int count = 0;
	u8 value = 0;

	//TOUCH_FUNC();

    // should be in idle or active mode

	SetBankAddr(ts, SHTSC_BANK_COMMAND);
#ifdef DEBUG_SHTSC
	TOUCH_LOG("exec calibration command\n");
#endif

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_EXECCALIBRATION, CMD_EXECCALIBRATION_LEN);

	SetIndicator(ts, SHTSC_IND_CMD);

	//msleep(400);

	value = ReadOneByte(ts, SHTSC_ADDR_INT0);
	TOUCH_LOG("Value check value %d SHTSC_STATUS_COMMAND_RESULT & value %d \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 100)
			break;
	}

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	TOUCH_ERR("CALIBRATION done\n");

	return 0;
}

static ssize_t show_calibration(struct i2c_client *client, char *buf)
{
	int ret = 0;

	struct timeval start;
	struct timeval finish;

	long elapsed_time = 0;

	TOUCH_FUNC();

	do_gettimeofday(&start);

	shtsc_CMD_ExecCalibration_wo_IRQ(&g_ts);

	do_gettimeofday(&finish);

	if(finish.tv_sec == start.tv_sec) {
		elapsed_time = finish.tv_usec - start.tv_usec;
	} else if ( finish.tv_sec > start.tv_sec) {
		elapsed_time = (finish.tv_usec + 1000000 ) - start.tv_usec;	
	} else {
		TOUCH_ERR("Time wrong. check!\n");
	}		

	WRITE_SYSBUF(buf, ret,	"Calbiration done duration  %ld\n", elapsed_time);
	TOUCH_LOG("Calbiration done elapsed time  %ld\n", elapsed_time);

	return ret;
}


/* get DC map */
#define DC_CALIBED_DATA 0
#define DC_RAW_DATA 1

//august06
#if 0
static int readDcMapResult(char *buf, int *pDataLen)
{
	/*
	 *  (0,0)L,H (1,0)L,H ....(13,0)L,H = 28byte
	 *  (0,1)L,H (1,1)L,H ....(13,1)L,H = 28byte
	 *  (0,2)L,H (1,2)L,H ....(13,2)L,H = 28byte
	 *  :
	 *  (0,25)L,H (1,25)L,H ....(13,25)L,H = 28byte
	 *  singed Little endian
	 */

	int ret = 0;
	u8 tmpbuf[128];
    unsigned char dsFlag, readingSenseNum, ram_addr[2];
    unsigned vramAddr;
    unsigned readingSize;
	u8 numDriveLine2;
	u8 numSenseLine2;
	u8 num_adc_dmy[3];

	int bytes;
	int size;
	int index;
	int drive;

	TOUCH_FUNC();

	TOUCH_LOG("DCMap readingSize\n");


    // get SD/DS and size
    ram_addr[0] = 0x58;
    ram_addr[1] = 0xBF;
    WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);

    SetBankAddr(&g_ts,SHTSC_BANK_DCMAP);
    ReadMultiBytes(&g_ts, 0x08, 5, tmpbuf);

    numSenseLine2 = tmpbuf[0];
    numDriveLine2 = tmpbuf[1];

    dsFlag = tmpbuf[4]; // 1 for DS, 0 for SD
    vramAddr = ((tmpbuf[3]<<8) | tmpbuf[2]);
    // readingSenseNum is greater or equal to itself, but a multiply of 4
    readingSenseNum = (unsigned char)((numSenseLine2+3)/4); // not a double but int
    readingSenseNum *= 4;

    readingSize = readingSenseNum * numDriveLine2 * 2; /* 2:16bit */

    num_adc_dmy[0] = num_adc_dmy[2] = 0; //top and left have no 0-filled lines
    num_adc_dmy[1] = readingSenseNum - numSenseLine2; //right side

    // read DCmap values from register
    // store it to read buffer memory for read action

	*pDataLen = readingSize;




	SetBankAddr(&g_ts,SHTSC_BANK_DCMAP);
	/* read 120 bytes from Bank5, address 8 */
	/* read loop required */

	while (bytes > 0) {
		/* 16bit signed value for each point = read two bytes for one point */

		ram_addr[0] = (unsigned char)(vramAddr&0xff); // set address to read (Lower)
		ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // set address to read (Higher)
		WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);

		size = ((bytes >= 120) ? 120 : bytes);
		TOUCH_LOG("size : %d\n", size);

		/* 120 bytes = 120/2=60 elements (signed 16bit little endian) */
		ReadMultiBytes(&g_ts, 0x08, size, &(buf[index]));
		index += size;
		bytes -= size;
		vramAddr += size;
		TOUCH_LOG("vramAddr : %04X\n", vramAddr);
	} // end of while

	{ //L3
	  int sindex = 0, dindex = 0;
	  int l, x, y;

	  // dcmap header
	  // [0]: horizontal data num (in short, not byte)
	  // [1]: vertical data num

	  x = numSenseLine2;
	  y = numDriveLine2;
//	  x = dcmap[dindex++] = numSenseLine2;
//	  y = dcmap[dindex++] = numDriveLine2;
//	  dcmap[dindex++] = dsFlag;

	  //top
	  sindex = (num_adc_dmy[0] + x + num_adc_dmy[1]) * num_adc_dmy[2] * 2;

	  // contents line
	  for (l = 0; l < y; l++) {
	    // left
	    sindex += (num_adc_dmy[0] * 2);

	    // contents
	    memcpy((u8 *)&(dcmap[dindex]), (u8 *)&(dcmapBuf[sindex]), (x*2));
	    dindex += (x*2);
	    sindex += (x*2);

	    // right
	    sindex += (num_adc_dmy[1] * 2);
	  }

	} //L3


    /* dc map read done */
	SetIndicator(&g_ts, SHTSC_IND_DCMAPDONE);


//	TOUCH_LOG("========== \t \t \t Raw data  -  AVG value \t \t \t ========== \n");
	TOUCH_LOG("\t     1\t     2\t     3\t     4\t     5\t     6\t     7\t     8\t     9\t    10\t    1l\t    12\t    13\t   14\n");

//	WRITE_SYSBUF(buf, ret, "========== \t \t \t Raw data  -  AVG value \t \t \t ========== \n");
	WRITE_SYSBUF(buf, ret, "\t     1\t     2\t     3\t     4\t     5\t     6\t     7\t     8\t     9\t    10\t    1l\t    12\t    13\t   14\n");

	for (drive = 0; drive < NUM_DRIVE; drive++) {
#if 1
		TOUCH_LOG("[%02d]\t %5d\t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t%5d\n",\
				drive,
				(signed short)(dcmap[drive*NUM_SENSE + 0]),\
				(signed short)(dcmap[drive*NUM_SENSE + 1]),\
				(signed short)(dcmap[drive*NUM_SENSE + 2]),\
				(signed short)(dcmap[drive*NUM_SENSE + 3]),\
				(signed short)(dcmap[drive*NUM_SENSE + 4]),\
				(signed short)(dcmap[drive*NUM_SENSE + 5]),\
				(signed short)(dcmap[drive*NUM_SENSE + 6]),\
				(signed short)(dcmap[drive*NUM_SENSE + 7]),\
				(signed short)(dcmap[drive*NUM_SENSE + 8]),\
				(signed short)(dcmap[drive*NUM_SENSE + 9]),\
				(signed short)(dcmap[drive*NUM_SENSE + 10]),\
				(signed short)(dcmap[drive*NUM_SENSE + 11]),\
				(signed short)(dcmap[drive*NUM_SENSE + 12]),\
				(signed short)(dcmap[drive*NUM_SENSE + 13]));
		WRITE_SYSBUF(buf, ret, "[%02d]\t %5d\t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t%5d\n",\
				drive,
				(signed short)(dcmap[drive*NUM_SENSE + 0]),\
				(signed short)(dcmap[drive*NUM_SENSE + 1]),\
				(signed short)(dcmap[drive*NUM_SENSE + 2]),\
				(signed short)(dcmap[drive*NUM_SENSE + 3]),\
				(signed short)(dcmap[drive*NUM_SENSE + 4]),\
				(signed short)(dcmap[drive*NUM_SENSE + 5]),\
				(signed short)(dcmap[drive*NUM_SENSE + 6]),\
				(signed short)(dcmap[drive*NUM_SENSE + 7]),\
				(signed short)(dcmap[drive*NUM_SENSE + 8]),\
				(signed short)(dcmap[drive*NUM_SENSE + 9]),\
				(signed short)(dcmap[drive*NUM_SENSE + 10]),\
				(signed short)(dcmap[drive*NUM_SENSE + 11]),\
				(signed short)(dcmap[drive*NUM_SENSE + 12]),\
				(signed short)(dcmap[drive*NUM_SENSE + 13]));
#else /* 1 */
    		TOUCH_LOG("[%02d]\t %5d\t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t%5d\n",\
				drive,
				(signed short)((buf[NUM_SENSE*drive*2 + 1]<<8) | buf[NUM_SENSE*drive*2 + 0]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 3]<<8) | buf[NUM_SENSE*drive*2 + 2]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 5]<<8) | buf[NUM_SENSE*drive*2 + 4]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 7]<<8) | buf[NUM_SENSE*drive*2 + 6]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 9]<<8) | buf[NUM_SENSE*drive*2 + 8]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 11]<<8) | buf[NUM_SENSE*drive*2 + 10]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 13]<<8) | buf[NUM_SENSE*drive*2 + 12]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 15]<<8) | buf[NUM_SENSE*drive*2 + 14]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 17]<<8) | buf[NUM_SENSE*drive*2 + 16]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 19]<<8) | buf[NUM_SENSE*drive*2 + 18]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 21]<<8) | buf[NUM_SENSE*drive*2 + 20]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 23]<<8) | buf[NUM_SENSE*drive*2 + 22]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 25]<<8) | buf[NUM_SENSE*drive*2 + 24]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 27]<<8) | buf[NUM_SENSE*drive*2 + 26]));

		WRITE_SYSBUF(buf, ret, "[%02d]\t %5d\t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t%5d\n",\
				drive,
				(signed short)((buf[NUM_SENSE*drive*2 + 1]<<8) | buf[NUM_SENSE*drive*2 + 0]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 3]<<8) | buf[NUM_SENSE*drive*2 + 2]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 5]<<8) | buf[NUM_SENSE*drive*2 + 4]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 7]<<8) | buf[NUM_SENSE*drive*2 + 6]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 9]<<8) | buf[NUM_SENSE*drive*2 + 8]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 11]<<8) | buf[NUM_SENSE*drive*2 + 10]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 13]<<8) | buf[NUM_SENSE*drive*2 + 12]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 15]<<8) | buf[NUM_SENSE*drive*2 + 14]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 17]<<8) | buf[NUM_SENSE*drive*2 + 16]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 19]<<8) | buf[NUM_SENSE*drive*2 + 18]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 21]<<8) | buf[NUM_SENSE*drive*2 + 20]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 23]<<8) | buf[NUM_SENSE*drive*2 + 22]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 25]<<8) | buf[NUM_SENSE*drive*2 + 24]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 27]<<8) | buf[NUM_SENSE*drive*2 + 26]));
#endif /* 1 */
    }

	TOUCH_LOG("readDcMapResult done ret = %d\n", ret );

	return ret;
}
#endif


#define CMD_DCMAP_ON  "\xD7\x00\x01\x00\x01"
#define CMD_DCMAP_OFF "\xD7\x00\x01\x00\x00"
#define CMD_DCMAP_ON_LEN 5
#define CMD_DCMAP_OFF_LEN 5

int shtsc_CMD_getDCMap_wo_IRQ(void *ts, int rawflag)
{
	//int ret;
	int count = 0;
	u8 value = 0;

	TOUCH_FUNC();


#define DC_CALIBED_DATA 0
#define DC_RAW_DATA 1

  SetBankAddr(ts, 0x12);
  if (rawflag == DC_CALIBED_DATA) {
    // normal DC data (delta)
    // bank=0x12, addr=0x6e, data=0x01
    WriteOneByte(&g_ts, 0x6e, 0x01);
	TOUCH_LOG("delta (calibrated data) \n");
  } else {
    // calib by-pass data (rawdata)
    // bank=0x12, addr=0x6e, data=0x21
    WriteOneByte(&g_ts, 0x6e, 0x21);
	TOUCH_LOG("raw data (DC by-pass data) \n");
  }
    // should be in idle or active mode

	SetBankAddr(ts, SHTSC_BANK_COMMAND);
#ifdef DEBUG_SHTSC
    TOUCH_LOG("exec DCMAP command (%s)\n", (rawflag ? "calib by-pass": "normal"));
#endif

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_DCMAP_ON, CMD_DCMAP_ON_LEN);

	SetIndicator(ts, SHTSC_IND_CMD);

	value = ReadOneByte(ts, SHTSC_ADDR_INT0);
	TOUCH_LOG("Value check value %02XH SHTSC_STATUS_COMMAND_RESULT & value %02XH \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 100)
			break;
	}

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

    /* read DC map */
    //readDcMapResult(pResultData, &resultDataLen);




	SetBankAddr(ts, SHTSC_BANK_COMMAND);
#ifdef DEBUG_SHTSC
    TOUCH_LOG("exec DCMAP command (%s)\n", (rawflag ? "calib by-pass": "normal"));
#endif

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_DCMAP_OFF, CMD_DCMAP_OFF_LEN);

	SetIndicator(ts, SHTSC_IND_CMD);

	value = ReadOneByte(ts, SHTSC_ADDR_INT0);
	TOUCH_LOG("Value check value %02XH SHTSC_STATUS_COMMAND_RESULT & value %02XH \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 100)
			break;
	}

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	return 0;
}



static ssize_t show_delta(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	shtsc_CMD_getDCMap_wo_IRQ(&g_ts, DC_CALIBED_DATA);

	TOUCH_LOG("Show delta done\n");

	return ret;
}

static ssize_t show_rawdata(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	shtsc_CMD_getDCMap_wo_IRQ(&g_ts, DC_RAW_DATA);

	TOUCH_LOG("Show rawdata done\n");

	return ret;
}

static ssize_t show_pen_support(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	WRITE_SYSBUF(buf, ret,	"1\n");

	return ret;
}

static ssize_t show_lpwg_fail_reason(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	WRITE_SYSBUF(buf, ret,	"%d\n", lpwg_fail_reason);

	return ret;
}

static ssize_t store_lpwg_fail_reason(struct i2c_client *client, const char *buf, size_t count)
{
	int value = 0;

	TOUCH_FUNC();

	sscanf(buf, "%d", &value);

	TOUCH_LOG(" lpwg_fail_reason %d\n", value);

	if( value == 1 ) {
		lpwg_fail_reason_mask = KNOCK_CODE_FAILURE_REASONS ;
	} else {
		lpwg_fail_reason_mask = 0x00;
	}
	
	lpwg_fail_reason = value;

	return count;
}


static LGE_TOUCH_ATTR(device_name, S_IRUGO | S_IWUSR, show_device_name, NULL);
static LGE_TOUCH_ATTR(pen_mode, S_IRUGO | S_IWUGO, show_pen_mode, store_pen_mode);
static LGE_TOUCH_ATTR(property, S_IRUGO | S_IWUGO, show_property, NULL);
static LGE_TOUCH_ATTR(rawdata, S_IRUGO | S_IWUGO, show_rawdata, NULL);
static LGE_TOUCH_ATTR(delta, S_IRUGO | S_IWUGO, show_delta, NULL);
static LGE_TOUCH_ATTR(calibration, S_IRUGO | S_IWUGO, show_calibration, NULL);
static LGE_TOUCH_ATTR(tci, S_IRUGO | S_IWUSR, show_tci, store_tci);
static LGE_TOUCH_ATTR(lpwg_touch, S_IRUGO | S_IWUSR, show_lpwg_touch_data, NULL);
static LGE_TOUCH_ATTR(pen_support, S_IRUGO | S_IWUSR, show_pen_support, NULL);
static LGE_TOUCH_ATTR(lpwg_fail_reason, S_IRUGO | S_IWUSR, show_lpwg_fail_reason, store_lpwg_fail_reason);

static struct attribute *LR388K5_attribute_list[] = {
	&lge_touch_attr_device_name.attr,
	&lge_touch_attr_pen_mode.attr,
	&lge_touch_attr_property.attr,
	&lge_touch_attr_rawdata.attr,
	&lge_touch_attr_delta.attr,
	&lge_touch_attr_calibration.attr,
	&lge_touch_attr_tci.attr,
	&lge_touch_attr_lpwg_touch.attr,
	&lge_touch_attr_pen_support.attr,
	&lge_touch_attr_lpwg_fail_reason.attr,
	NULL,
};


static const struct file_operations lr388k5_fops = {
	.owner = THIS_MODULE,
	.open	= lr388k5_misc_open,
	.release = lr388k5_misc_release,
	.read	= lr388k5_misc_read,
	.llseek = no_llseek,
	.unlocked_ioctl = lr388k5_misc_unlocked_ioctl,
	.compat_ioctl = lr388k5_misc_unlocked_ioctl,
};

static int lr388k5_misc_open(struct inode *inode, struct file *file)
{
	TOUCH_FUNC();

	return 0;
}

static int lr388k5_misc_release(struct inode *inode, struct file *file)
{
	TOUCH_FUNC();

	return 0;
}

static ssize_t lr388k5_misc_read(struct file *file, char *buf, size_t count, loff_t *pos)
{
	int copy_len;
	int i;

	TOUCH_FUNC();

	if( count > buf_pos){
		copy_len = buf_pos;
	} else {
		copy_len = count;
	}


	if( copy_to_user( buf, devbuf, copy_len) ) {
		TOUCH_ERR(" copy_to_user failed \n");
		return -EFAULT;
	}

	*pos += copy_len;

	for ( i = copy_len; i < buf_pos ; i++){
		devbuf[ i - copy_len ] = devbuf[i];
	}
	buf_pos -= copy_len;

	return copy_len;
}

static long lr388k5_misc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	u8 addr = 0;
	u8 val = 0;
	int r = 0;

#if defined(DEBUG_SHTSC)
	TOUCH_LOG(" cmd : %x, arg : %lx \n", cmd, arg);
#endif

	switch(cmd) {

		case SHTSC_IOCTL_SET_PAGE:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case SET_PAGE; cmd %x, arg %lx\n", cmd, arg);
#endif
			current_page = arg;
			break;

		case SHTSC_IOCTL_FLASH_ACCESS_START:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case FLASH_ACCESS_START; cmd %x\n", cmd);
#endif
			flash_access_start_shtsc(&g_ts);
			break;

		case SHTSC_IOCTL_FLASH_ACCESS_END:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case FLASH_ACCESS_END; cmd %x\n", cmd);
#endif
			flash_access_end_shtsc(&g_ts);
			break;

		case SHTSC_IOCTL_ERASE:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case ERASE; cmd %x, current_page %d\n", cmd, current_page);
#endif
#ifdef FLASH_CHECK
			if (flash_erase_page_shtsc(&g_ts, current_page))
				return -1;
#else /* FLASH_CHECK */
			flash_erase_page_shtsc(&g_ts, current_page);
#endif /* FLASH_CHECK */
			break;

		case SHTSC_IOCTL_WRITE:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case WRITE; cmd %x, current_page %d\n", cmd, current_page);
#endif
			memset(iobuf, 0x0, IO_BUF_SIZE);
			if (copy_from_user(iobuf, (char *)arg, IO_BUF_SIZE)) {
				TOUCH_ERR("ERROR by copy_from_user\n");
				return -1;
			}
#ifdef FLASH_CHECK
			if (flash_write_page_shtsc(&g_ts, current_page, iobuf))
				return -1;
#else /* FLASH_CHECK */
			flash_write_page_shtsc(&g_ts, current_page, iobuf);
#endif /* FLASH_CHECK */
			break;

		case SHTSC_IOCTL_VERIFY:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case VERIFY; cmd %x, current_page %d\n", cmd, current_page);
#endif
			memset(iobuf, 0x0, IO_BUF_SIZE);
			if (copy_from_user(iobuf, (char *)arg, IO_BUF_SIZE)) {
				TOUCH_ERR("ERROR by copy_from_user\n");
				return -1;
			}
#ifdef FLASH_CHECK
			if (flash_verify_page_shtsc(&g_ts, current_page, iobuf))
				return -1;
#else /* FLASH_CHECK */
			flash_verify_page_shtsc(&g_ts, current_page, iobuf);
#endif /* FLASH_CHECK */
			break;

		case SHTSC_IOCTL_ERASE_ALL:
			{
#define LAST_PAGE 16
				int page;
				TOUCH_LOG("case ERASE_ALL flash_access start\n");
				flash_access_start_shtsc(&g_ts);
				for (page = 0; page < LAST_PAGE; page++) {
					flash_erase_page_shtsc(&g_ts, page);
					TOUCH_LOG("flash_erase_page_shtsc done: page %d\n",  page);
				}
				TOUCH_LOG("flash_access end\n");
				flash_access_end_shtsc(&g_ts);
				TOUCH_LOG("flash erased.\n");
			}
			break;
		case SHTSC_IOCTL_REG_1WRITE:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case REG_1WRITE: cmd %x, arg %lx \n", cmd, arg);
#endif
			addr = (arg >> 8) & 0xFF;
			val = arg & 0xFF;
			TOUCH_LOG("cmd %x, arg %lx a:0x%02X value :0x%02X(%d)\n", cmd, arg, addr, val, val);

			WriteOneByte(&g_ts, addr, val);
			break;

		case SHTSC_IOCTL_REG_1READ:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case REG_1READ: cmd %x, arg %lx\n", cmd, arg);
#endif
			val = ReadOneByte(&g_ts, ((struct reg *)arg)->addr);
			((struct reg *)arg)->data = val;

			TOUCH_LOG("cmd %x, arg %lx a:0x%02X value :0x%02X(%d)\n", cmd, arg, ((struct reg *)arg)->addr, val, val);
			break;


		case SHTSC_IOCTL_REG_N_RW_SET_ADDR:
			s_shtsc_addr = 0xFF & (arg >> 16);
			s_shtsc_len = 0xFFFF & arg;
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case REG_N_RW_SET_ADDR: cmd %x, arg %lx a:0x%x len:%d\n", cmd, arg, s_shtsc_addr, s_shtsc_len);
#endif
			break;

		case SHTSC_IOCTL_REG_N_WRITE_1ADDR_GO:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case REG_N_WRITE_1ADDR_GO : cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, addr, val, val);
#endif
			/* s_shtsc_len is initialized __IF__ SHTSC_IOCTL_REG_N_RW_SET_ADDR is issued before */
			r = copy_from_user(s_shtsc_buf, (char *)arg, s_shtsc_len);
			if (r != 0) {
				TOUCH_LOG("ERROR by copy_from_user(%d)\n", r);
				return -1;
			}
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("Driver Multibyte write. addr %02X, len: %x, data[0-3]: %02X %02X %02X %02X ....\n", s_shtsc_addr, s_shtsc_len, s_shtsc_buf[0], s_shtsc_buf[1], s_shtsc_buf[2], s_shtsc_buf[3]);
#endif
#if defined(DEBUG_SHTSC)
			TOUCH_LOG(" cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, s_shtsc_addr, s_shtsc_len, s_shtsc_len);
#endif
			WriteMultiBytes(&g_ts, s_shtsc_addr, s_shtsc_buf, s_shtsc_len);

			break;

		case SHTSC_IOCTL_REG_N_READ_1ADDR_GO:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case REG_N_READ_1ADDR_GO : cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, addr, val, val);
#endif
			/* s_shtsc_len is initialized __IF__ SHTSC_IOCTL_REG_N_RW_SET_ADDR is issued before */
			ReadMultiBytes(&g_ts, s_shtsc_addr, s_shtsc_len, s_shtsc_buf);
			//msleep(10); // not checked yet
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("Driver Multibyte read done. addr %02X, len: %x, data[0-3]: %02X %02X %02X %02X ....\n",
					s_shtsc_addr, s_shtsc_len, s_shtsc_buf[0], s_shtsc_buf[1], s_shtsc_buf[2], s_shtsc_buf[3]);
#endif
			r = copy_to_user((char *)arg, s_shtsc_buf, s_shtsc_len);
			if (r != 0) {
				TOUCH_LOG("ERROR by copy_to_user(%d)\n", r);
				return -1;
			}

			break;

		case SHTSC_IOCTL_SETIRQMASK:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case SETIRQMASK; cmd %x, arg %lx\n", cmd, arg);
#endif
			if (arg) {
				TouchEnableIrq();
			} else {
				TouchDisableIrq();
			}
			break;

		case SHTSC_IOCTL_RESET:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case RESET; cmd %x, arg %lx\n", cmd, arg);
#endif
			if (arg) {
				g_ts.wait_state = WAIT_RESET;
				g_ts.wait_result = false;
				//shtsc_reset(&g_ts, true);
				TouchResetCtrl(1);
				// wait
				WaitAsync(&g_ts);
				shtsc_system_init(&g_ts);
				msleep(100);
			} else {
				TouchResetCtrl(0);
				G_Irq_Mask = 0xffff;
				//shtsc_reset(&g_ts, false);
			}
			break;

		case SHTSC_IOCTL_DEBUG:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case DEBUG; cmd %x, arg %lx\n", cmd, arg);
#endif
			break;

		case SHTSC_IOCTL_CMD_ISSUE_RESULT:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case CMD_ISSUE_RESULT; cmd %x, arg %lx\n", cmd, arg);
#endif
			{
				unsigned int magicnumber; /* 'int' width is 32bit for LP64 and LLP64 mode */
				if ( copy_from_user(&magicnumber, (char *) arg, 4) ){
					magicnumber = 0;
				}
				if( magicnumber != 0xA5A5FF00 ){
					msleep(100);
				}
				r = copy_to_user((char *) arg, CommandResultBuf,MAX_COMMAND_RESULT_LEN);
				if( magicnumber == 0xA5A5FF00 ){
					CommandResultBuf[0] = 0;
				}
			}
			if (r != 0) {
				TOUCH_LOG("ERROR by copy_to_user(%d)\n", r);
				return -1;
			}
			break;

		case SHTSC_IOCTL_DRIVER_VERSION_READ:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case DRIVER_VERSION_READ; cmd %x, arg %lx\n", cmd, arg);
#endif
			{
				char versionBuf[16];
				versionBuf[0] = VERSION_YEAR;
				versionBuf[1] = VERSION_MONTH;
				versionBuf[2] = VERSION_DAY;
				versionBuf[3] = VERSION_SERIAL_NUMBER;
				versionBuf[4] = VERSION_MODEL_CODE;

				r = copy_to_user((char *)arg, versionBuf, DRIVER_VERSION_LEN);
				if (r != 0) {
					TOUCH_LOG("ERROR by copy_to_user(%d)\n", r);
					return -1;
				}
			}
			break;

		case SHTSC_IOCTL_DCMAP:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case DCMAP; cmd %x, arg %lx\n", cmd, arg);
#endif
			{
				int x = dcmap[0];
				int y = dcmap[1];
				int len = (x * y * 2) + 4;

#if defined(DEBUG_SHTSC)
				TOUCH_LOG("cmd %x, arg %lx, %d*%d+4=%d\n", cmd, arg, x, y, len);
#endif

				if (buf_pos) {
					/* DC map ready to send out */
					r = copy_to_user((char *)arg, dcmap, len);
					if (r != 0) {
						TOUCH_LOG("ERROR by copy_to_user failed\n" );
						return -EFAULT;
					}
					buf_pos = 0;
				}
				break;
			}

#ifdef GET_REPORT
		case SHTSC_IOCTL_GET_REPORT:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case GET_REPORT;  cmd %x, arg %lx\n", cmd, arg);
#endif
			{
				volatile int count;
				int len;


				while (Mutex_GetReport)
					;
				Mutex_GetReport = true;

				count = reportBuf[0];
				len = 4+ count*REP_SIZE;

#if defined(DEBUG_SHTSC)
				TOUCH_LOG("cmd %x, arg %lx, count=%d, len=%d\n", cmd, arg, count, len);
#endif

				r = copy_to_user((char *)arg, reportBuf, len);
				if (r != 0) {
					TOUCH_LOG("ERROR by copy_to_user(%d)\n", r);
					return -1;
				}

				reportBuf[0] = (unsigned char)0;

				Mutex_GetReport = false;
			}
			break;
#endif /* GET_REPORT */

		case SHTSC_IOCTL_FLASH_READ:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case FLASH_READ;  cmd %x, arg %lx\n", cmd, arg);
#endif
			{
				/* arg: pointer to the content buffer */
				/* arg[3:0]: address to read (little endian) */
				/* arg[5:4]: length to read (little endian) */

				unsigned address;
				unsigned length;

				memset(iobuf, 0x0, IO_BUF_SIZE);

				if (copy_from_user(iobuf, (char *)arg, (4+2))) {
					TOUCH_LOG("ERROR by copy_from_user\n");
					return -1;
				}
				address = (iobuf[3] << 24) | (iobuf[2] << 16) | (iobuf[1] << 8) | (iobuf[0] << 0);
				length = (iobuf[5] << 8) | (iobuf[4] << 0);

#if defined(DEBUG_SHTSC)
				TOUCH_LOG("addr %x, arg %x\n", address, length);
#endif

				flash_read(&g_ts, address, length, iobuf);
				if ( copy_to_user( (char *)arg, iobuf, length ) ) {
					printk( KERN_INFO "shtsc : copy_to_user failed\n" );
					return -EFAULT;
				}
				break;
			}

		case SHTSC_IOCTL_NOTIFY_PID:
			pid = arg; // save pid for later kill();
			TOUCH_LOG("case NOTIFY_PID: pid: %d\n", pid);
			break;

		case SHTSC_IOCTL_GET_INTERRUPT_STATUS:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case GET_INTERRUPT_STATUS, %d\n", resumeStatus);
#endif
			r = copy_to_user((char *)arg, &resumeStatus, 1); // copy one-byte status
			if (r != 0) {
				TOUCH_LOG("ERROR by copy_to_user(%d)\n", r);
				return -1;
			}
			break;

		default:
			ret = 0;
			break;
	}
	return ret;
}

//====================================================================
// Function : LR388K5_Initialize
// Description
//   - 
//   - 
//====================================================================

static int LR388K5_Initialize(struct i2c_client *client)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	TOUCH_FUNC();

	// init g_ts;
	g_ts.client = client;
	g_ts.wait_state = WAIT_NONE;
	g_ts.wait_result = false;


	//HMOON TESTING
	//LR388K5_Reset(client);

	pDriverData->isMiscDevice = 1;
	pDriverData->touch_misc_device.name = DRIVER_NAME;
	pDriverData->touch_misc_device.fops = &lr388k5_fops;

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : LR388K5_Reset
// Description
//   - 
//   - 
//====================================================================
static void LR388K5_Reset(struct i2c_client *client)
{
	TOUCH_FUNC();

	TouchResetCtrl(0);
	msleep(50);
	// frequent irq interrupted
	//
	G_Irq_Mask = 0xffff;
	TouchResetCtrl(1);

	//HMOON
	msleep(200);

	dummyDeviceState = STATE_NORMAL;
}


//====================================================================
// Function : LR388K5_Connect
// Description
//   - 
//   - 
//====================================================================
static int LR388K5_Connect(void)
{
	TOUCH_FUNC();

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : LR388K5_InitRegister
// Description
//   - Initialize touch IC register
//   - will be called after IC reset ( by reset pin )
//====================================================================
static int LR388K5_InitRegister(struct i2c_client *client)
{
	//	struct device_node *np;
	//	TouchDriverData *pDriverData;

	TOUCH_FUNC();

	//msleep(100);

	return TOUCH_SUCCESS;

}

//====================================================================
// Function : LR388K5_ClearInterrupt
// Description
//   - Clear interrupt
//   - will be called before interrupt enable to clear interrupt happened during interrupt disabled time
//====================================================================
static void LR388K5_ClearInterrupt(struct i2c_client *client)
{
	TOUCH_FUNC();

	return;
}

//====================================================================
// Function : LR388K5_InterruptHandler
// Description
//   - process interrupt
//   - will be called if interrupt detected by AP
//====================================================================
static int LR388K5_InterruptHandler(struct i2c_client *client, TouchReadData *pData)
{
	//TOUCH_FUNC();

	g_ts.client = client;

	shtsc_irq_thread(0, &g_ts, pData);

	return TOUCH_SUCCESS;
}

static int WaitAsync(void *_ts)
{
	struct shtsc_i2c *ts = _ts;

	int i;
	for(i=0;i<50;i++) {
		mdelay(CMD_DELAY);//16ms
		//DBGLOG("shtsc: wait 16ms for state change\n");
		switch(ts->wait_state) {
			case WAIT_RESET:
				break;
			case WAIT_CMD:
				break;
			case WAIT_NONE:
				if (ts->wait_result == true) {
					TOUCH_LOG("shtsc: wait state change: success\n");
					return 0;
				}
				else
					return -EIO;
			default:
				break;
		}
	}
	//DBGLOG("wait state change: failure\n");
	return -EIO;
}

int issue_command(void *ts, unsigned char *cmd, unsigned int len)
{
	int err = 0;

	//disable_irq(g_ts.client->irq);

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, cmd, len);
	TOUCH_LOG("set command (%x)\n",cmd[0]);

	// prepare waiting
	((struct shtsc_i2c *)ts)->cmd = cmd[0];
	((struct shtsc_i2c *)ts)->wait_state = WAIT_CMD;
	((struct shtsc_i2c *)ts)->wait_result = true;

	// do it
	SetIndicator(ts, SHTSC_IND_CMD);
	//DBGLOG("do it\n");

	//enable_irq(g_ts.client->irq);

	// wait
	WaitAsync(ts);
	TOUCH_LOG("WaitAsync in issue_command return %d\n",err);

	return err;
}

int lpwg_param_command(void *ts, u8 u8Addr, unsigned char *cmd, unsigned int len)
{
	int err = 0;

	SetBankAddr(ts, SHTSC_BANK_LPWG_PARAM);
	//TOUCH_LOG("tci bank command LPWG_PARM \n");
	// set command
	WriteMultiBytes(ts, u8Addr, cmd, len);
	TOUCH_LOG("tci set LPWG_PARM command (0x%x) (0x%x) \n",cmd[0], cmd[1]);

	return err;
}

//====================================================================
// Function : LR388K5_ReadIcFirmwareInfo
// Description
//   - Read firmware information from touch IC
//   - will be called at boot time or after writing new firmware
//====================================================================
static int LR388K5_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{

	//unsigned firmver = 0;
	//unsigned paramver = 0;

	int value =0;

	TOUCH_FUNC();

#if 0
	issue_command(&g_ts, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	firmver = ((CommandResultBuf[0x0a-0x08 +3] << 24) |
			(CommandResultBuf[0x0a-0x08 +2] << 16) |
			(CommandResultBuf[0x0a-0x08 +1] << 8) |
			(CommandResultBuf[0x0a-0x08 +0] << 0));
	paramver = ((CommandResultBuf[0x12-0x08 +3] << 24) |
			(CommandResultBuf[0x12-0x08 +2] << 16) |
			(CommandResultBuf[0x12-0x08 +1] << 8) |
			(CommandResultBuf[0x12-0x08 +0] << 0));

#endif

	// Bank Address
	SetBankAddr(&g_ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(&g_ts, SHTSC_ADDR_COMMAND, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	// Indicator
	SetIndicator(&g_ts, SHTSC_IND_CMD);

	msleep(50);

	value = ReadOneByte(&g_ts, SHTSC_ADDR_INT0);
	TOUCH_LOG("1st Value check value %d SHTSC_STATUS_COMMAND_RESULT & value %d \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	// Bank address
	SetBankAddr(&g_ts,SHTSC_BANK_COMMAND_RESULT);

	// READ result
	ReadMultiBytes(&g_ts, 0x08, MAX_COMMAND_RESULT_LEN, CommandResultBuf); // always read entire result. TEGAPP will handle it.

#ifdef DEBUG_SHTSC
	TOUCH_LOG(" command result, %02X, %02X, %02X, %02X\n", CommandResultBuf[0], CommandResultBuf[1], CommandResultBuf[2],CommandResultBuf[3]);
	TOUCH_LOG(" command result, %02X, %02X, %02X, %02X\n", CommandResultBuf[4], CommandResultBuf[5], CommandResultBuf[6],CommandResultBuf[7]);
	TOUCH_LOG(" command result, %02X, %02X, %02X, %02X\n", CommandResultBuf[8], CommandResultBuf[9], CommandResultBuf[10],CommandResultBuf[11]);
	TOUCH_LOG(" command result, %02X, %02X, %02X, %02X\n", CommandResultBuf[12], CommandResultBuf[13], CommandResultBuf[14],CommandResultBuf[15]);
#endif

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	/* below value is same as default firmware image ( dummy_firmware.img ) */
	// what should I do for these?
	pFwInfo->moduleMakerID = 2;  
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 3;
	pFwInfo->isOfficial = 1;
	//	pFwInfo->version = CommandResultBuf[2+8]; // HMOON ((firmver >> 32) | paramver); // 32bit value
	pFwInfo->version = CommandResultBuf[0x12-0x08];

	TOUCH_LOG("FW  version %d \n", pFwInfo->version);

	if(pFwInfo->version == 0) {
		unsigned paramver = 0;
		msleep(50);
		paramver = ((CommandResultBuf[0x12-0x08 +3] << 24) |
				(CommandResultBuf[0x12-0x08 +2] << 16) |
				(CommandResultBuf[0x12-0x08 +1] << 8) |
				(CommandResultBuf[0x12-0x08 +0] << 0));
		TOUCH_LOG(" paramver  %08X \n", paramver);
	}


	return TOUCH_SUCCESS;	
}

//====================================================================
// Function : LR388K5_GetBinFirmwareInfo
// Description
//   - parse and return firmware information from firmware image
//   - if filename is NULL, return information of default firmware image
//   - will be called at boot time or needed
//====================================================================
static int LR388K5_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	char *pFwFilename = NULL;
	unsigned long image_size = 0;

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

#if 1
	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret ) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	image_size = fw->size;
	pBin = kzalloc(sizeof(char) * (image_size+1), GFP_KERNEL);
	if (pBin == NULL) {
		TOUCH_ERR("Can not allocate memory\n");
		goto  error;
	}

	memcpy(pBin, fw->data, image_size);

	TOUCH_ERR("success from image to buffer size %lu \n", image_size );

#if 1//HMOON
	TOUCH_LOG("file fw ver=%02x%02x%02x%02x,param ver=%02x%02x%02x%02x\n",
			pBin[0x23+5],pBin[0x22+5],pBin[0x21+5],pBin[0x20+5],
			pBin[image_size-8],pBin[image_size-9],pBin[image_size-10],pBin[image_size-11] );
#endif

	pFwInfo->version = pBin[image_size-11]; // HMOON ((firmver >> 32) | paramver); // 32bit value

#endif

	//HMOON TEMP
	pFwInfo->moduleMakerID = 2;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 3;
	pFwInfo->isOfficial = 1;
	pFwInfo->version = pBin[image_size-11]; // HMOON ((firmver >> 32) | paramver); // 32bit value

	TOUCH_LOG("BIN version %d \n", pFwInfo->version);

	if(pBin != NULL){
		kfree(pBin);
		TOUCH_ERR("free buffer \n");
		pBin=NULL;
	}
	/* Free firmware image buffer */
	release_firmware(fw);
	return TOUCH_SUCCESS;

error:
	//HMOON TEMP
	pFwInfo->moduleMakerID = 2;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 3;
	pFwInfo->isOfficial = 1;
	pFwInfo->version = 0; // HMOON ((firmver >> 32) | paramver); // 32bit value


	if(pBin != NULL){
		kfree(pBin);
		TOUCH_ERR("free buffer \n");
		pBin = NULL;
	}

	/* Free firmware image buffer */
	release_firmware(fw);
	return TOUCH_FAIL;
}


//====================================================================
// Function : LR388K5_UpdateFirmware
// Description
//   - Write firmware to touch IC
//   - if filename is NULL, use default firmware image
//   - common driver will call Reset(), InitRegister() and ReadIcFirmwareInfo() one by one after writing
//====================================================================
static int LR388K5_UpdateFirmware(struct i2c_client *client, char *pFilename)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	char *pFwFilename = NULL;
	unsigned long image_size;

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret ) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	image_size = fw->size;
	pBin = kzalloc(sizeof(char) * (image_size+1), GFP_KERNEL);
	if (pBin == NULL) {
		TOUCH_ERR("Can not allocate memory\n");
		goto  error;
	}

	memcpy(pBin, fw->data, image_size);

	TOUCH_ERR("success from image to buffer size %lu \n", image_size);

	/* IMPLEMENT : firmware update function */
	//shtsc_proc_firmup_func(pBin, image_size);

	shtsc_CMD_SetSystemStateSleep_wo_IRQ(&g_ts);

	update_flash(&g_ts,pBin,image_size);

error:

	if(pBin != NULL){
		kfree(pBin);
	}

	/* Free firmware image buffer */
	release_firmware(fw);

	return TOUCH_SUCCESS;

}

//====================================================================
// Function : LR388K5_SetLpwgMode
// Description
//   - Set device to requested state
//====================================================================
static int LR388K5_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int ret = TOUCH_SUCCESS;

	TOUCH_FUNC();

	TOUCH_LOG("mode [%d]\n",				pLpwgSetting->mode);
	TOUCH_LOG("lcdPixelSizeX [%d]\n",		pLpwgSetting->lcdPixelSizeX);
	TOUCH_LOG("lcdPixelSizeY [%d]\n",		pLpwgSetting->lcdPixelSizeY);
	//pLpwgSetting->activeTouchAreaX1 = 200;// 0->200
	//pLpwgSetting->activeTouchAreaX2 = 880;// 1080->880
	//pLpwgSetting->activeTouchAreaY1 = 400;// 0->400
	//pLpwgSetting->activeTouchAreaY2 = 1520;// 1920->1520
	TOUCH_LOG("activeTouchAreaX1 [%d]\n",	pLpwgSetting->activeTouchAreaX1);
	TOUCH_LOG("activeTouchAreaX2 [%d]\n",	pLpwgSetting->activeTouchAreaX2);
	TOUCH_LOG("activeTouchAreaY1 [%d]\n",	pLpwgSetting->activeTouchAreaY1);
	TOUCH_LOG("activeTouchAreaY2 [%d]\n",	pLpwgSetting->activeTouchAreaY2);
	TOUCH_LOG("tapCount [%d]\n",			pLpwgSetting->tapCount);
	TOUCH_LOG("isFirstTwoTapSame [%d]\n",	pLpwgSetting->isFirstTwoTapSame);
	TOUCH_LOG("lcdState [%d]\n",			pLpwgSetting->lcdState);
	TOUCH_LOG("proximityState [%d]\n",		pLpwgSetting->proximityState);
	TOUCH_LOG("coverState [%d]\n",			pLpwgSetting->coverState);
	TOUCH_LOG("callState [%d]\n",			pLpwgSetting->callState);

	memcpy(&(g_ts.lpwgSetting), pLpwgSetting, sizeof(LpwgSetting));

	if( dummyDeviceState == newState ) {
		TOUCH_LOG("device state is same as driver requested\n");
		return TOUCH_SUCCESS;
	}

	if (dummyDeviceState == newState) {
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

	if( ret == TOUCH_SUCCESS ) {
		dummyDeviceState = newState;
	}
	switch( newState )
	{
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
		case STATE_NORMAL_HOVER:
			TOUCH_LOG("device was set to NORMAL_HOVER\n");
			break;
		case STATE_HOVER:
			TOUCH_LOG("device was set to HOVER\n");
			break;
		default:
			TOUCH_LOG("invalid state ( state = %d )\n", newState);
			ret = TOUCH_FAIL;
			break;

	}

	if( ret == TOUCH_SUCCESS ) {
		dummyDeviceState = newState;
	}

	return TOUCH_SUCCESS;

}

static int readDiagResult(char *buf, int *pDataLen)
{
	/*
	 *  (0,0)L,H (1,0)L,H ....(13,0)L,H = 28byte
	 *  (0,1)L,H (1,1)L,H ....(13,1)L,H = 28byte
	 *  (0,2)L,H (1,2)L,H ....(13,2)L,H = 28byte
	 *  :
	 *  (0,25)L,H (1,25)L,H ....(13,25)L,H = 28byte
	 *  singed Little endian
	 */

	unsigned char ram_addr[2];
	unsigned vramAddr = 0xBAE0; // the first address of the map
	unsigned readingSize = (NUM_DRIVE*NUM_SENSE*2);

	int bytes = readingSize;
	int size;
	int index = 0;

	int drive; // for debug
	//  int sense; // for debug

	int ret = *pDataLen;

	// Version information
	// Self testing
	// Raw dataLen
	// Other testing 

	//*pDataLen = readingSize;

	TOUCH_FUNC();

	TOUCH_LOG("SelfDiag readingSize(%d) pDataLen %d\n", readingSize, *pDataLen);

	SetBankAddr(&g_ts,SHTSC_BANK_DCMAP);
	/* read 120 bytes from Bank5, address 8 */
	/* read loop required */

	while (bytes > 0) {
		/* 16bit signed value for each point = read two bytes for one point */

		ram_addr[0] = (unsigned char)(vramAddr&0xff); // set address to read (Lower)
		ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // set address to read (Higher)
		WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);

		size = ((bytes >= 120) ? 120 : bytes);
		TOUCH_LOG("size : %d\n", size);

		/* 120 bytes = 120/2=60 elements (signed 16bit little endian) */
		ReadMultiBytes(&g_ts, 0x08, size, &(buf[index]));
		index += size;
		bytes -= size;
		vramAddr += size;
		TOUCH_LOG("vramAddr : %04X\n", vramAddr);
	} // end of while

	TOUCH_LOG("========== \t \t \t Raw data  -  AVG value \t \t \t ========== \n");
	TOUCH_LOG("\t     1\t     2\t     3\t     4\t     5\t     6\t     7\t     8\t     9\t    10\t    1l\t    12\t    13\t   14\n");

	WRITE_SYSBUF(buf, ret, "========== \t \t \t Raw data  -  AVG value \t \t \t ========== \n");
	WRITE_SYSBUF(buf, ret, "\t     1\t     2\t     3\t     4\t     5\t     6\t     7\t     8\t     9\t    10\t    1l\t    12\t    13\t   14\n");

	for (drive = 0; drive < NUM_DRIVE; drive++) {

		TOUCH_LOG("[%02d]\t %5d\t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t%5d\n",\
				drive,
				(signed short)((buf[NUM_SENSE*drive*2 + 1]<<8) | buf[NUM_SENSE*drive*2 + 0]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 3]<<8) | buf[NUM_SENSE*drive*2 + 2]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 5]<<8) | buf[NUM_SENSE*drive*2 + 4]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 7]<<8) | buf[NUM_SENSE*drive*2 + 6]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 9]<<8) | buf[NUM_SENSE*drive*2 + 8]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 11]<<8) | buf[NUM_SENSE*drive*2 + 10]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 13]<<8) | buf[NUM_SENSE*drive*2 + 12]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 15]<<8) | buf[NUM_SENSE*drive*2 + 14]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 17]<<8) | buf[NUM_SENSE*drive*2 + 16]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 19]<<8) | buf[NUM_SENSE*drive*2 + 18]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 21]<<8) | buf[NUM_SENSE*drive*2 + 20]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 23]<<8) | buf[NUM_SENSE*drive*2 + 22]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 25]<<8) | buf[NUM_SENSE*drive*2 + 24]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 27]<<8) | buf[NUM_SENSE*drive*2 + 26]));

		WRITE_SYSBUF(buf, ret, "[%02d]\t %5d\t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t %5d \t%5d\n",\
				drive,
				(signed short)((buf[NUM_SENSE*drive*2 + 1]<<8) | buf[NUM_SENSE*drive*2 + 0]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 3]<<8) | buf[NUM_SENSE*drive*2 + 2]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 5]<<8) | buf[NUM_SENSE*drive*2 + 4]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 7]<<8) | buf[NUM_SENSE*drive*2 + 6]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 9]<<8) | buf[NUM_SENSE*drive*2 + 8]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 11]<<8) | buf[NUM_SENSE*drive*2 + 10]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 13]<<8) | buf[NUM_SENSE*drive*2 + 12]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 15]<<8) | buf[NUM_SENSE*drive*2 + 14]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 17]<<8) | buf[NUM_SENSE*drive*2 + 16]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 19]<<8) | buf[NUM_SENSE*drive*2 + 18]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 21]<<8) | buf[NUM_SENSE*drive*2 + 20]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 23]<<8) | buf[NUM_SENSE*drive*2 + 22]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 25]<<8) | buf[NUM_SENSE*drive*2 + 24]),\
				(signed short)((buf[NUM_SENSE*drive*2 + 27]<<8) | buf[NUM_SENSE*drive*2 + 26]));
	}

	*pDataLen = ret;

	TOUCH_LOG("readDiagResult done ret = %d pDatalen = %d\n", ret, *pDataLen );

	return ret;
}


#define NO_INTERRUPT_FOR_SELFDIAG 1
#if defined ( NO_INTERRUPT_FOR_SELFDIAG)

#define CMD_SELFDIAG "\xDA\x00\x00\x01"
#define CMD_SELFDIAG_LEN 4

int shtsc_CMD_SelfDiag_wo_IRQ(void *ts)
{
	//int ret;
	int count = 0;
	u8 value = 0;

	TOUCH_FUNC();

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_SELFDIAG, CMD_SELFDIAG_LEN);

	SetIndicator(ts, SHTSC_IND_CMD);

	msleep(100);

	value = ReadOneByte(ts, SHTSC_ADDR_INT0);
	TOUCH_LOG("Value check value %d SHTSC_STATUS_COMMAND_RESULT & value %d \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#if 1 //def DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 100)
			break;
	}

	// Bank address
	SetBankAddr(&g_ts,SHTSC_BANK_COMMAND_RESULT);

	// READ result
	ReadMultiBytes(&g_ts, 0x08, MAX_COMMAND_RESULT_LEN, CommandResultBuf);

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	return 0;
}

#endif /* NO_INTERRUPT_FOR_SELFDIAG */



//====================================================================
// Function : LR388K5_DoSelfDiagnosis
// Description
//   - diagnose touch pannel and return result
//   - can use pBuf to give more information ( be careful not to exceed buffer size )
//   - should create a file of result ( TBD : consider file saving on common driver side )
//====================================================================

static int LR388K5_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	u8 * pResultData = NULL;
	int resultDataLen = 0;
	int dataLen = 0;
	int ret = 0;
	char cmdbin_selfdiag[]={0xDA,0x00,0x00,0x01}; // selfdiag command

	TOUCH_FUNC();


	*pDataLen = (int) 0;

	*pRawStatus = (int) TOUCH_FAIL;

	*pChannelStatus = (int) TOUCH_FAIL;


	pResultData =  kzalloc(sizeof(u8) * RESULT_DATA_MAX_LEN , GFP_KERNEL);
	if(pResultData == NULL){
		TOUCH_ERR("kzalloc failed size %d \n", ( sizeof(u8) *  RESULT_DATA_MAX_LEN ) );
		return TOUCH_FAIL;
	}

	// 1 Calbiration 
	shtsc_CMD_ExecCalibration_wo_IRQ(&g_ts);

	//WRITE_SYSBUF(pBuf, ret, "========== Calibration Done ========== \n");
	WRITE_SYSBUF(pResultData, resultDataLen, "========== Calibration Done ========== \n\n");

	// 2 Self Diagnosis
	shtsc_CMD_SelfDiag_wo_IRQ(&g_ts);


#if 1 //def 

	TOUCH_LOG("SelfDiag command CommandResultBuf 0x08-0x08 0x%X\n", CommandResultBuf[0x08-0x08]);
	TOUCH_LOG("SelfDiag command CommandResultBuf 0x09-0x08 0x%X\n", CommandResultBuf[0x09-0x08]);
	TOUCH_LOG("SelfDiag command CommandResultBuf 0x0a-0x08 0x%X\n", CommandResultBuf[0x0a-0x08]);

#endif

	//WRITE_SYSBUF(pResultData, resultDataLen, "========== RESULT  ========== \n");

	if ((CommandResultBuf[0x08-0x08] == cmdbin_selfdiag[0]) && // test command
			(CommandResultBuf[0x09-0x08] == 0x00)) { // error code  // now test is done. check ok or not


		if (CommandResultBuf[0x0a-0x08] == 0x00) { // Result
			// test is passed
	
			*pRawStatus = (int) TOUCH_SUCCESS;

			*pChannelStatus = (int) TOUCH_SUCCESS;

			TOUCH_LOG("SelfDiag command test passed\n");

			ret = TOUCH_SUCCESS;

		} else if (CommandResultBuf[0x0a-0x08] == 0x01) {
			// test is not passed
			TOUCH_LOG("SelfDiag command test not passed\n");

			ret =  TOUCH_SUCCESS; // or TOUCH_FAIL? which should I return?

		} else if (CommandResultBuf[0x0a-0x08] == 0xff) {
			// abnormal termination
			TOUCH_LOG("SelfDiag command test abnormal termination\n");
			ret =  TOUCH_FAIL;
		}

	} else if (CommandResultBuf[0x09-0x08] != 0x00) {
		// what happend?
		TOUCH_LOG("SelfDiag ErrorCode(%02X)\n", CommandResultBuf[0x09-0x08]);
		ret = TOUCH_FAIL;
	}

	//WRITE_SYSBUF(pResultData, resultDataLen, "Channel Status : %s \n", (*pChannelStatus) ? "Success" : "Fail" );
	//WRITE_SYSBUF(pResultData, resultDataLen, "Raw Data : %s \n", (*pRawStatus) ? "Success" : "Fail" );


	readDiagResult(pResultData, &resultDataLen);

	WRITE_SYSBUF(pResultData, resultDataLen, "======== Additional Information =========\n" );
	WRITE_SYSBUF(pResultData, resultDataLen, " Device name = LR388K5 , Provider = Sharp \n" );

	write_result_data_to_file(NULL, pResultData, resultDataLen);


	// what are these for?
	//*pRawStatus = TOUCH_SUCCESS;
	//*pChannelStatus = TOUCH_SUCCESS;

	dataLen += sprintf(pBuf, "%s", "========= Additional Information =========\n");
	dataLen += sprintf(pBuf+dataLen, "%s", "Device Name = LR388K5\n");

	*pDataLen = (int) dataLen;

	if(pResultData != NULL){
		kfree(pResultData);
		pResultData = NULL;
	}
	return ret;
}

//====================================================================
// Function : LR388K5_AccessRegister
// Description
//   - read from or write to touch IC
//====================================================================
static int LR388K5_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
{
	int ret = 0;

	TOUCH_FUNC();

	switch( cmd )
	{
		case READ_IC_REG:
			ret = Touch_I2C_Read_Byte(client, (u8)reg, (u8 *)pValue);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		case WRITE_IC_REG:
			ret = Touch_I2C_Write_Byte(client, (u8)reg, (u8)*pValue);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		default:
			TOUCH_ERR("Invalid access command ( cmd = %d )\n", cmd);
			return TOUCH_FAIL;
			break;
	}

	return TOUCH_SUCCESS;

}

static void LR388K5_NotifyHandler(struct i2c_client *client, TouchNotify notify, int data)
{
	switch (notify) {
		case NOTIFY_CALL:
			TOUCH_LOG("Call was notified ( data = %d )\n", data);
			break;

		case NOTIFY_Q_COVER:
			TOUCH_LOG("Quick Cover was notified ( data = %d )\n", data);
			break;

		case NOTIFY_FPS_CHANGED:
			TOUCH_LOG("FPS change  was notified ( data = %d )\n", data);
			break;

		case NOTIFY_TA_STATUS:
			TOUCH_LOG("TA status  was notified ( data = %d )\n", data);
			break;

		default:
			TOUCH_ERR("Invalid notification ( notify = %d )\n", notify);
			break;
	}
	return;
}


TouchDeviceSpecificFunction LR388K5_Func = {

	.Initialize = LR388K5_Initialize,
	.Reset = LR388K5_Reset,
	.Connect = LR388K5_Connect,
	.InitRegister = LR388K5_InitRegister,
	.ClearInterrupt = LR388K5_ClearInterrupt,
	.InterruptHandler = LR388K5_InterruptHandler,
	.ReadIcFirmwareInfo = LR388K5_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = LR388K5_GetBinFirmwareInfo,
	.UpdateFirmware = LR388K5_UpdateFirmware,
	.SetLpwgMode = LR388K5_SetLpwgMode,
	.DoSelfDiagnosis = LR388K5_DoSelfDiagnosis,
	.AccessRegister = LR388K5_AccessRegister,
	.NotifyHandler = LR388K5_NotifyHandler,
	.device_attribute_list = LR388K5_attribute_list,

};


/* End Of File */


