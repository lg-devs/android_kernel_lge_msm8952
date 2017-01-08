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
 *    File  	: lgtp_device_lr388k6.c
 *    Author(s)   : BSP Touch Team
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[LR388K6]"

/****************************************************************************
 * Include Files
 ****************************************************************************/
#include <linux/input/unified_driver_3/lgtp_common.h>

#include <linux/input/unified_driver_3/lgtp_common_driver.h>
#include <linux/input/unified_driver_3/lgtp_platform_api.h>
#include <linux/input/unified_driver_3/lgtp_device_lr388k6.h>

#include <linux/fs.h>
#include "shtsc_ioctl.h"
#include "lgtp_lr388k6_panel_spec.h"

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
#define SHTSC_BANK_SYSTEM_HA      0x1C

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
#define WAIT_DCMAP  (3)

#define MAX_16BIT			0xffff
#define MAX_12BIT			0xfff

#define NUM_DRIVE (31)
#define NUM_SENSE (18)

#define MAX_DCMAP_SIZE ( NUM_DRIVE * (NUM_SENSE + 2 ) * 2 ) // 1240 = 31 * 20 * 2

#define LOW_LEVEL 0
#define HIGH_LEVEL 1

#define MAX_COMMAND_RESULT_LEN (64-8)

#define SHTSC_DEVBUF_SIZE 1500

#define ADDR_ID_JIG_4REN 0x1F300
#define LEN_ID_JIG_4REN 10
#define ADDR_ID_JIG_LPC 0x1F100
#define LEN_ID_JIG_LPC 9
#define TYPE_ID_JIG_4REN 1
#define TYPE_ID_JIG_LPC 2

//#define FLASH_WAIT 500
//#define FIRMWARE_SIZE (44*1024)

#define KNOCKDATA_SIZE (4*12) // 20151105 for maximum 12 taps for knock-code

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

#define DEVICE_CODE_LR388K6 2 

#define VERSION_YEAR 15
#define VERSION_MONTH 6
#define VERSION_DAY 12
#define VERSION_SERIAL_NUMBER 20
#define VERSION_MODEL_CODE DEVICE_CODE_LR388K6
#define DRIVER_VERSION_LEN 5

#define DRIVER_NAME "shtsc"
/* ======= SelfDiagnosis ====== */
#define RESULT_DATA_MAX_LEN (8 * 1024)
#define TIME_DATA_MAX_LEN 64
#define FILE_PATH_RESULT "/data/logger/touch_self_test.txt"


/* FLASH TIME IMPROVEMENT FEATURE */
#define FLASH_TIME_ZERO_WAIT
#define FLASH_MULTI_WRITE
#define FLASH_NO_VERIFY

#ifdef FLASH_TIME_ZERO_WAIT
#define FLASH_WAIT 0
#else
#define FLASH_WAIT 500
#endif

/****************************************************************************
 * Macros
 ****************************************************************************/
#define DBGLOG TOUCH_LOG

#define TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG 1

//#define DEBUG_SHTSC 0

#define WRITE_RESULT(_desc, _size, fmt, args...) do {					\
	_size += snprintf(_desc + _size, RESULT_DATA_MAX_LEN  - _size, fmt, ##args);	\
} while (0)

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
// 0923 LR388K6 bring up 
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2200_p2203_150921_ext_h.bin";
// 1001 SelfD testing
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_fF2012203_pF2002203_151001_ext_h.bin";
// 1007 Pen / Linearity improved
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2201_p2205_151007_ext_h.bin";
// 1022 Edge improvement
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2203_p2207_151021_ext_h.bin";
// 1109 Drawing line improvement
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2204_p220F_151109_ext_h.bin";
// 1124 Palm & Pen improvement
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f70429006_p221E_151123_ext_h.bin";
// 1125 LPWG improvement & power consumption
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f220D_p221F_151125_ext_h.bin";
// 1130 Multi touch ghost 1.32
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f220E_p2220_151130_ext_h.bin";
// 1208 Evaluation Version 1.38
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2214_p2226_151208_ext_h.bin";
// 1214 FW checklist improvement & AFE ON Counter 64 - 1.40
static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2215_p12228_151212_ext_h.bin";

int	G_reset_done = false;
u16 G_Irq_Mask = 0xffff;
unsigned G_touch=0;

unsigned char resumeStatus; // bank 0, address 9

unsigned char CommandResultBuf[MAX_COMMAND_RESULT_LEN];

u8 dcmapBuf[MAX_DCMAP_SIZE+128];
u8 dcmap[MAX_DCMAP_SIZE+128]; // greater than 17*32*2


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
#define TOTAL_CHANNAL (NUM_DRIVE * NUM_SENSE)
#define UPPER_DATA 0
#define LOWER_DATA 1

int lpwg_fail_reason = 0;
unsigned int lpwg_fail_reason_mask = KNOCK_CODE_FAILURE_REASONS ;


// firmware version
unsigned K6_firmver = 0;
unsigned K6_paramver = 0;

TouchFingerData release_fingerData[MAX_FINGER+1];

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
int issue_command_wo_IRQ(void *ts, unsigned char *cmd, unsigned int len);
int issue_command2(void *ts, unsigned char *cmd, unsigned int len);
int lpwg_param_command(void *ts, u8 u8Addr, unsigned char *cmd, unsigned int len);

static void LR388K6_Reset(struct i2c_client *client);
static int LR388K6_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen);

static int lr388k6_misc_open(struct inode *inode, struct file *file);
static int lr388k6_misc_release(struct inode *inode, struct file *file);
static ssize_t lr388k6_misc_read(struct file *file, char *buf, size_t count, loff_t *pos);
static long lr388k6_misc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static int WaitAsync(void *_ts);
//char * LR388K6_property(void);

static int LR388K6_UpdateFirmware(struct i2c_client *client, char *pFilename);


/****************************************************************************
 * Local Functions
 ****************************************************************************/


/****************************************************************************
 * Device Specific Functions
 ****************************************************************************/
static int get_current_time(char * buf)
{
	struct timespec my_time;
	struct tm my_date;

    if(buf == NULL)
    {
        TOUCH_ERR("buf is NULL.\n");
        return TOUCH_FAIL;
    }

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
    snprintf(buf, 64, "%02d-%02d %02d:%02d:%02d.%03lu ",
		my_date.tm_mon + 1, my_date.tm_mday,
		my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
		(unsigned long) my_time.tv_nsec / 1000000);

    TOUCH_LOG("CURRENT TIME : %s\n", buf);

    return TOUCH_SUCCESS;
}


static int write_result_data_to_file(char* filename, char *data, int datalen){
	int fd = 0;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	if(filename == NULL) {
		fd = sys_open(FILE_PATH_RESULT, O_WRONLY | O_CREAT | O_APPEND, 0666);
		TOUCH_LOG(" SelfD Test default result file has been opened fd %d size %d \n", fd, datalen);
	} else {
		fd = sys_open(filename, O_WRONLY | O_CREAT , 0666);
		if(fd == 0 ){
			TOUCH_ERR(" SelfD Test result file %s has not been created \n", filename);
			return 0;
		} else {
			TOUCH_LOG(" SelfD Test result file %s has been opened fd %d \n", filename, fd);
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
	//struct input_dev *input_dev = ((struct shtsc_i2c *)ts)->input;

	TOUCH_FUNC();

	// NOT USED any more
	return 0;

#if 0
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
#endif

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

static int SetBankAddr(void *_ts, u8 u8Bank)
{
	int ret = TOUCH_FAIL;
	struct shtsc_i2c *ts = _ts;

	if( (ts == NULL) ){
		TOUCH_LOG("ts is NULL error\n");
		return ret;
	}

#if 0
	TOUCH_LOG("SetBankAddr %d \n", u8Bank);
#endif

	ret = WriteOneByte(ts, SHTSC_ADDR_BANK, u8Bank);  

	return ret;
}

static void Sharp_ClearInterrupt(void *_ts, u16 u16Val)
{
	struct shtsc_i2c *ts = _ts;

	//TOUCH_FUNC();
#if 0
	TOUCH_LOG(" Sharp_ClearInterrupt %04X \n", u16Val);
#endif

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

#ifdef FLASH_MULTI_WRITE
		{
			// random access mode to speed up

			char tbuf[512];
			for(idx=0;idx<256;idx++){
				tbuf[idx*2] = (0x80 | 0x3D);
				tbuf[idx*2+1] = data[(cnt*FLASH_PHYSICAL_PAGE_SIZE) +idx];
			}
			WriteMultiBytes(ts,tbuf[0],&(tbuf[1]),511);
		}
#else
		/* Data write 1page = 256byte */
		for(idx=0;idx<256;idx++){
			//			SPI_ArrieRegWrite(0x3D,*pData++);
			// addr=in-page(virtual, in 4k block) 256xN
			WriteOneByte(ts, (unsigned char)0x3D, data[(cnt*FLASH_PHYSICAL_PAGE_SIZE) +idx]);
		}
#endif

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
#ifndef FLASH_NO_VERIFY
		msleep(FLASH_WAIT);
		flash_verify_page_shtsc(_ts, page, &(data[page*FLASH_PAGE_SIZE]));
		TOUCH_LOG("shtsc: flash_verify_page_shtsc done: page %d\n",  page);
#endif
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

#if defined(DEBUG_SHTSC)
	TOUCH_LOG("shtsc touch number num=%d\n",u8Num);
#endif

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
		TOUCH_LOG("shtsc [%d] ID=%2d, Status=%02x, Size=%3d, X=%5d, Y=%5d, Z=%5d\n"
				,u8Num
				,ID
				,Status
				,touch[i].size
				,touch[i].x
				,touch[i].y
				,touch[i].z);
#endif



		if(Status & SHTSC_TOUCHOUT_STATUS){
#if defined(DEBUG_SHTSC)
			TOUCH_LOG(" release event \n");
#endif
			touch[i].status = SHTSC_F_TOUCH_OUT;
			release_fingerData[ID].id =  touch[i].id;
			release_fingerData[ID].x =  touch[i].x;
			release_fingerData[ID].y =  touch[i].y;
			release_fingerData[ID].pressure = ((touch[i].z) > 4095) ? (0xff) : ((touch[i].z & 0x0ff0)>>4);
			release_fingerData[ID].width_major =  touch[i].size;
			release_fingerData[ID].width_minor =  touch[i].size;
			continue;
		}


		pData->fingerData[touchNum].id =  touch[i].id;
		pData->fingerData[touchNum].x =  touch[i].x;
		pData->fingerData[touchNum].y =  touch[i].y;
		pData->fingerData[touchNum].pressure = ((touch[i].z) > 4095) ? (0xff) : ((touch[i].z & 0x0ff0)>>4);
		//pData->fingerData[touchNum].pressure = touch[i].z;  // report the value from K6 with no change
		pData->fingerData[touchNum].width_major =  touch[i].size;
		pData->fingerData[touchNum].width_minor =  touch[i].size;

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
			TOUCH_LOG("[IRQ] shtsc_irq  POWER_UP\n");
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

			// WDT make LR388K6 power recyle so that it needs PLL_UNLOCK handling
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

			TOUCH_LOG("[IRQ] shtsc flash load error\n");

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
				memset(u8Buf, 0x0, 128);
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

				//input_sync(ts->input);
			}
		}

		// COMMAND_RESULT
		//
		if (u16status & SHTSC_STATUS_COMMAND_RESULT) {
#if 1 //defined(DEBUG_SHTSC)
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
				unsigned char dsFlag, readingSenseNum, ram_addr[3];
				unsigned int vramAddr;
				unsigned int readingSize;

				// get SD/DS and size
				ram_addr[0] = 0x58;
				ram_addr[1] = 0x7F;
				ram_addr[2] = 0x01;
				WriteMultiBytes(ts,(unsigned char)0x06,ram_addr,2);
               	WriteOneByte(ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
				WriteOneByte(ts, 0x43, ram_addr[2]);

				SetBankAddr(ts,SHTSC_BANK_DCMAP);
				ReadMultiBytes(ts, 0x08, 7, tmpbuf);

				numSenseLine2 = tmpbuf[0];
				numDriveLine2 = tmpbuf[1];

				dsFlag = tmpbuf[6]; // 1 for DS, 0 for SD
				vramAddr = ((tmpbuf[4]<<16) | (tmpbuf[3]<<8) | tmpbuf[2]);
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

					memset(dcmapBuf, 0x0, sizeof(dcmapBuf));

					//	      TOUCH_LOG("%s(%d):readingSize:%d\n", __FILE__, __LINE__, readingSize);
					while (bytes > 0) {
						ram_addr[0] = (unsigned char)(vramAddr&0xff); // address to read (Lower)
						ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // address to read (Middle)
					    ram_addr[2] = (unsigned char)((vramAddr&0xff0000)>>16); // address to read (Higher)
					    WriteMultiBytes(ts,(unsigned char)0x06,ram_addr,2);
					    WriteOneByte(ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
					    WriteOneByte(ts, 0x43, ram_addr[2]);
					    WriteOneByte(ts, SHTSC_ADDR_BANK, SHTSC_BANK_DCMAP);

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

					TOUCH_LOG(" Sense : %d Drive : %d\n", numSenseLine2, numDriveLine2);

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
			} // L2 DEVICE_LR388K6 block
			//*************************

			if (ts->wait_state == WAIT_DCMAP) {//2015.11.12 added
				ts->wait_state = WAIT_NONE;
				ts->wait_result = true;
			}
		}

		// SHTSC_STATUS_TCI_1(KNOCK_ON)
		if (u16status & SHTSC_STATUS_LG_LPWG_TCI1) {
            u8 buffer[3] = {0};
            u8 failureReason;

#if TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG
			struct shtsc_touch *touch = ((struct shtsc_i2c *)ts)->touch; // for testing Dec.2
			u8 knockData_buf[KNOCKDATA_SIZE];
			int i = 0, n = 0;
#endif

			Sharp_ClearInterrupt(ts, SHTSC_STATUS_LG_LPWG_TCI1);
			u16status &= ~(SHTSC_STATUS_LG_LPWG_TCI1);
			TOUCH_LOG("[IRQ] shtsc_irq KNOCK_ON\n");

            SetBankAddr(&g_ts, SHTSC_BANK_LPWG_PARAM);
            ReadMultiBytes(&g_ts, FAILURE_REASON_REG, 1, buffer);
            failureReason = buffer[0] & 0x0F; // now it's ready
#if 0
            TOUCH_LOG("TCI1 Failure Reason [%d]\n", failureReason);
#endif

            switch (failureReason) {
            case 0:
              TOUCH_LOG("TCI1 SUCCESS\n");
              pData->type = DATA_KNOCK_ON;
              break;
            case 1:
              TOUCH_LOG("TCI1 FAIL - DISTANCE_INTER_TAP\n");
#if TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG
              n = 3;
#endif /* TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG */
              break;
            case 2:
              TOUCH_LOG("TCI1 FAIL - DISTANCE_TOUCHSLOP\n");
#if TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG
              n = 4;
#endif /* TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG */
              break;
            case 3:
              TOUCH_LOG("TCI1 FAIL - TIMEOUT_INTERTAP\n");
              break;
            case 4:
              TOUCH_LOG("TCI1 FAIL - MULTI_FINGER\n");
#if TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG
              n = 2;
#endif /* TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG */
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

#if TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG
			memset(knockData_buf, 0x0, KNOCKDATA_SIZE);

			SetBankAddr(ts,SHTSC_BANK_LPWG_DATA);
			ReadMultiBytes(ts,SHTSC_ADDR_LPWG_REPORT,KNOCKDATA_SIZE,knockData_buf);
			for ( i = 0; i < n; i++) { // just two points to print out
				touch[i].x =
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG    ] << 0) |
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 1] << 8);
				touch[i].y =
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 2] << 0) |
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 3] << 8);
				TOUCH_LOG("[IRQ][DEBUG] KNOCK_ON coodinate(%d) (x, y) = (%d, %d)\n", i, touch[i].x, touch[i].y);
            }
#endif /* TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG */

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
#if 0
            TOUCH_LOG("[IRQ] shtsc_irq KNOCK_CODE count %d\n", g_ts.lpwgSetting.tapCount);
#endif

            SetBankAddr(&g_ts, SHTSC_BANK_LPWG_PARAM);
            ReadMultiBytes(&g_ts, FAILURE_REASON_REG, 1, buffer);
            failureReason = (buffer[0] & 0xF0) >> 4; // now it's ready
#if 0
            TOUCH_LOG("TCI2 Failure Reason [%d]\n", failureReason);
#endif

            switch (failureReason) {
            case 0:
              TOUCH_LOG("TCI2 SUCCESS\n");
              break;
            case 1:
              TOUCH_LOG("TCI2 FAIL - DISTANCE_INTER_TAP\n");
              break;
            case 2:
              TOUCH_LOG("TCI2 FAIL - DISTANCE_TOUCHSLOP\n");
              break;
            case 3:
              TOUCH_LOG("TCI2 FAIL - TIMEOUT_INTERTAP\n");
              break;
            case 4:
              TOUCH_LOG("TCI2 FAIL - MULTI_FINGER\n");
              break;
            case 5:
              TOUCH_LOG("TCI2 FAIL - DELAY_TIME\n");
              break;
            case 6:
              TOUCH_LOG("TCI2 FAIL - PALM_STATE\n");
              break;
            default:
              TOUCH_LOG("TCI2 FAIL - RESERVED\n");
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
		WRITE_BUFFER(buf, ret, "current state is not LPWG mode\n");
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

	TOUCH_LOG("Value check value %02XH SHTSC_STATUS_COMMAND_RESULT & value %02XH \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 10) {
			TOUCH_LOG("waiting the reult count %d. Please check it\n", count);
			break;
		}
	}
	return 0;
}

static int tci_control(struct i2c_client *client, int type, unsigned int value)
{
	int ret = TOUCH_SUCCESS;
	volatile unsigned char readData;
	u8 buffer[3] = {0};

	//TOUCH_FUNC();


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

	//TOUCH_FUNC();

	switch (newState) {
	case STATE_NORMAL:
		//tci_control(client, TCI_ENABLE_CTRL, 0);
		//tci_control(client, TCI_ENABLE_CTRL2, 0);

		// To Idle
		//issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);
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
		issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);
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
		issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);
		break;

	case STATE_OFF:
		tci_control(client, TCI_ENABLE_CTRL, 0);
		tci_control(client, TCI_ENABLE_CTRL2, 0);

		// To Idle
		issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);
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

	//TOUCH_FUNC();

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);

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

		if(count > 10){
			TOUCH_LOG("waiting the reult count %d. Please check it\n", count);
			break;
		}
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

		if(count > 10){
			TOUCH_LOG("waiting the reult count %d. Please check it\n", count);
			break;
		}
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

	WRITE_BUFFER(buf, ret,	"PEN MODE 1, NORMAL MODE 0 :  %d\n", G_pen_mode);

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

static ssize_t get_unique_id(char *idBuf)
{
    int ret = 0;
	unsigned char irqmask[2];

    // read LCM ID
	issue_command(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);
    // save reg values
	irqmask[0] = ReadOneByte(&g_ts,(unsigned char)SHTSC_ADDR_INTMASK0);
	irqmask[1] = ReadOneByte(&g_ts,(unsigned char)SHTSC_ADDR_INTMASK1);

	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);

    // use 4Ren-JIG testing ID for LCM unique ID
	flash_read(&g_ts, ADDR_ID_JIG_4REN, LEN_ID_JIG_4REN, idBuf);
    idBuf[11] = TYPE_ID_JIG_4REN;

    if (idBuf[0] == 0xFF) {
      // if no data, use another for LCM unique ID
      flash_read(&g_ts, ADDR_ID_JIG_LPC, LEN_ID_JIG_LPC, idBuf);
      idBuf[11] = TYPE_ID_JIG_LPC;
    }
    
    // restore irqmask
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, irqmask[0]);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, irqmask[1]);

	issue_command(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);

    return ret;
}

static ssize_t get_unique_id_wo_IRQ(char *idBuf)
{
    int ret = 0;
	unsigned char irqmask[2];

    // read LCM ID
	issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);
    // save reg values
	irqmask[0] = ReadOneByte(&g_ts,(unsigned char)SHTSC_ADDR_INTMASK0);
	irqmask[1] = ReadOneByte(&g_ts,(unsigned char)SHTSC_ADDR_INTMASK1);

	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);

    // use 4Ren-JIG testing ID for LCM unique ID
	flash_read(&g_ts, ADDR_ID_JIG_4REN, LEN_ID_JIG_4REN, idBuf);
    idBuf[11] = TYPE_ID_JIG_4REN;

    if (idBuf[0] == 0xFF) {
      // if no data, use another for LCM unique ID
      flash_read(&g_ts, ADDR_ID_JIG_LPC, LEN_ID_JIG_LPC, idBuf);
      idBuf[11] = TYPE_ID_JIG_LPC;
    }

    // restore irqmask
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, irqmask[0]);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, irqmask[1]);

	issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);

    return ret;
}

static ssize_t show_device_name(struct i2c_client *client, char *buf)
{
	int ret = 0;
    unsigned char unique_id_Buf[16];

	TOUCH_FUNC();

	issue_command(&g_ts, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	K6_firmver = 0;
	K6_paramver = 0;

	K6_firmver = ((CommandResultBuf[0x0a-0x08 +3] << 24) |
			(CommandResultBuf[0x0a-0x08 +2] << 16) |
			(CommandResultBuf[0x0a-0x08 +1] << 8) |
			(CommandResultBuf[0x0a-0x08 +0] << 0));
	K6_paramver = ((CommandResultBuf[0x12-0x08 +3] << 24) |
			(CommandResultBuf[0x12-0x08 +2] << 16) |
			(CommandResultBuf[0x12-0x08 +1] << 8) |
			(CommandResultBuf[0x12-0x08 +0] << 0));

	WRITE_SYSBUF(buf, ret,	"Sharp LR388K6 Firmware=%08X, Parameter=%08X\n", K6_firmver, K6_paramver);

    if(get_unique_id_wo_IRQ(unique_id_Buf) == TOUCH_SUCCESS){
        if (unique_id_Buf[11] == TYPE_ID_JIG_4REN) {
            TOUCH_LOG("Unique ID (4Ren-JIG): %02X/%02X/%02X B6:%4d pic_slot:%1d serial:%5d pic_rev:%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    ((unique_id_Buf[4] << 8) | unique_id_Buf[3]), //little endian
                    unique_id_Buf[5],
                    ((unique_id_Buf[7] << 8) | unique_id_Buf[6]), //little endian
                    ((unique_id_Buf[9] << 8) | unique_id_Buf[8])); //little endian

            WRITE_SYSBUF(buf, ret, "Unique ID (4Ren-JIG): %02X/%02X/%02X B6:%4d pic_slot:%1d serial:%5d pic_rev:%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    ((unique_id_Buf[4] << 8) | unique_id_Buf[3]), //little endian
                    unique_id_Buf[5],
                    ((unique_id_Buf[7] << 8) | unique_id_Buf[6]), //little endian
                    ((unique_id_Buf[9] << 8) | unique_id_Buf[8])); //little endian

        } else {
            TOUCH_LOG("Unique ID (LPC-JIG): %02d%02d%02d-%01d%01d-%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    unique_id_Buf[3],
                    unique_id_Buf[4],
                    ((unique_id_Buf[6] << 8) | unique_id_Buf[5])); //little endian

            WRITE_SYSBUF(buf, ret, "Unique ID (LPC-JIG): %02d%02d%02d-%01d%01d-%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    unique_id_Buf[3],
                    unique_id_Buf[4],
                    ((unique_id_Buf[6] << 8) | unique_id_Buf[5])); //little endian

        }
    } else {
        TOUCH_LOG("get_unique_id_wo_IRQ fail \n");
		return TOUCH_FAIL;
    }

	return ret;
}

static ssize_t show_property(struct i2c_client *client, char *buf)
{
	int ret = 0;
//	unsigned firmver = 0;
//	unsigned paramver = 0;
    unsigned char unique_id_Buf[16];

	TOUCH_FUNC();

	issue_command(&g_ts, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	K6_firmver = ((CommandResultBuf[0x0a-0x08 +3] << 24) |
			(CommandResultBuf[0x0a-0x08 +2] << 16) |
			(CommandResultBuf[0x0a-0x08 +1] << 8) |
			(CommandResultBuf[0x0a-0x08 +0] << 0));
	K6_paramver = ((CommandResultBuf[0x12-0x08 +3] << 24) |
			(CommandResultBuf[0x12-0x08 +2] << 16) |
			(CommandResultBuf[0x12-0x08 +1] << 8) |
			(CommandResultBuf[0x12-0x08 +0] << 0));

	WRITE_SYSBUF(buf, ret,	"Firmware %08X, Parameter %08X\n", K6_firmver, K6_paramver);

	TOUCH_LOG("Firmware %08X, Parameter %08X\n", K6_firmver, K6_paramver);

    if(get_unique_id_wo_IRQ(unique_id_Buf) == TOUCH_SUCCESS){
        if (unique_id_Buf[11] == TYPE_ID_JIG_4REN) {
            TOUCH_LOG("Unique ID (4Ren-JIG): %02X/%02X/%02X B6:%4d pic_slot:%1d serial:%5d pic_rev:%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    ((unique_id_Buf[4] << 8) | unique_id_Buf[3]), //little endian
                    unique_id_Buf[5],
                    ((unique_id_Buf[7] << 8) | unique_id_Buf[6]), //little endian
                    ((unique_id_Buf[9] << 8) | unique_id_Buf[8])); //little endian

            WRITE_SYSBUF(buf, ret, "Unique ID (4Ren-JIG): %02X/%02X/%02X B6:%4d pic_slot:%1d serial:%5d pic_rev:%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    ((unique_id_Buf[4] << 8) | unique_id_Buf[3]), //little endian
                    unique_id_Buf[5],
                    ((unique_id_Buf[7] << 8) | unique_id_Buf[6]), //little endian
                    ((unique_id_Buf[9] << 8) | unique_id_Buf[8])); //little endian

        } else {
            TOUCH_LOG("Unique ID (LPC-JIG): %02d%02d%02d-%01d%01d-%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    unique_id_Buf[3],
                    unique_id_Buf[4],
                    ((unique_id_Buf[6] << 8) | unique_id_Buf[5])); //little endian

            WRITE_SYSBUF(buf, ret, "Unique ID (LPC-JIG): %02d%02d%02d-%01d%01d-%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    unique_id_Buf[3],
                    unique_id_Buf[4],
                    ((unique_id_Buf[6] << 8) | unique_id_Buf[5])); //little endian

        }
    } else {
        TOUCH_LOG("get_unique_id_wo_IRQ fail \n");
		return TOUCH_FAIL;
    }

	return ret;
}
#if 0 
// 20150917
// call when getting K6 version from common driver
// K6_firmware and K6_parameter are global variables
// and set in the LR388K6_ReadIcFirmwareInfo()
char * LR388K6_property(void)
{
	char property[128];

	TOUCH_FUNC();

	sprintf(property, "SHARP LR388K6 (Firmware:%08X, Parameter:%08X)", K6_firmver, K6_paramver);

	return property;
}
#endif

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
	TOUCH_LOG("Value check value %02XH SHTSC_STATUS_COMMAND_RESULT & value %02XH \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 100){
			TOUCH_LOG("waiting the reult count %d. Please check it\n", count);
			break;
		}
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

static int readDcMapResult(char *buf, int *pDataLen)
{
	/*
	 *  (0,0)L,H (1,0)L,H ....(16,0)L,H = 34byte
	 *  (0,1)L,H (1,1)L,H ....(16,1)L,H = 34byte
	 *  (0,2)L,H (1,2)L,H ....(16,2)L,H = 34byte
	 *  :
	 *  (0,30)L,H (1,30)L,H ....(16,30)L,H = 34byte
	 *  singed Little endian
	 */

    // total S(18-1) * D 31 * 2byte = 1054 byte

	u8* pDcDataBuffer = &(dcmap[4]);
	int drive =0;
	int ret = *pDataLen;

	TOUCH_FUNC();
	msleep(50);

#if 0 // dcmap is read by interrupt handler and stored in dcmap[]
	u8* pDcDataBuffer = NULL;
	u8 tmpbuf[128];
    unsigned char dsFlag, readingSenseNum, ram_addr[3];
    unsigned int vramAddr;
    unsigned int readingSize;
	u8 numDriveLine2;
	u8 numSenseLine2;
	u8 num_adc_dmy[3];
	//u8 buf[MAX_DCMAP_SIZE+128];

	int bytes =0;
	int size =0;
	int index =0;


	TOUCH_LOG("DCMap readingSize\n");

	pDcDataBuffer =  kzalloc(sizeof(u8) * (MAX_DCMAP_SIZE + 128) , GFP_KERNEL);
	if(pDcDataBuffer == NULL){
		TOUCH_ERR("kzalloc failed size %d \n", ( sizeof(u8) *  (MAX_DCMAP_SIZE + 128) ));
		return TOUCH_FAIL;
	}

    // get SD/DS and size
    ram_addr[0] = 0x58;
    ram_addr[1] = 0x7F;
    ram_addr[2] = 0x01;
    WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);
    WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
    WriteOneByte(&g_ts, 0x43, ram_addr[2]);

    SetBankAddr(&g_ts,SHTSC_BANK_DCMAP);
    ReadMultiBytes(&g_ts, 0x08, 7, tmpbuf);

    numSenseLine2 = tmpbuf[0];
    numDriveLine2 = tmpbuf[1];

    dsFlag = tmpbuf[6]; // 1 for DS, 0 for SD
    vramAddr = ((tmpbuf[4]<<16) | (tmpbuf[3]<<8) | tmpbuf[2]);
    // readingSenseNum is greater or equal to itself, but a multiply of 4
    readingSenseNum = (unsigned char)((numSenseLine2+3)/4); // not a double but int
    readingSenseNum *= 4;

    readingSize = readingSenseNum * numDriveLine2 * 2; /* 2:16bit */

    num_adc_dmy[0] = num_adc_dmy[2] = 0; //top and left have no 0-filled lines
    num_adc_dmy[1] = readingSenseNum - numSenseLine2; //right side

    // read DCmap values from register
    // store it to read buffer memory for read action

	*pDataLen = readingSize;

	bytes = readingSize;

	pDcDataBuffer =  kzalloc(sizeof(u8) * (MAX_DCMAP_SIZE + 128) , GFP_KERNEL);
	if(pDcDataBuffer == NULL){
		TOUCH_ERR("kzalloc failed size %d \n", ( sizeof(u8) *  (MAX_DCMAP_SIZE + 128) ));
		return TOUCH_FAIL;
	}

	TOUCH_LOG("DCMap readingSize = %d\n", readingSize);

	SetBankAddr(&g_ts,SHTSC_BANK_DCMAP);
	/* read 120 bytes from Bank5, address 8 */
	/* read loop required */

	while (bytes > 0) {
		/* 16bit signed value for each point = read two bytes for one point */

		ram_addr[0] = (unsigned char)(vramAddr&0xff); // address to read (Lower)
		ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // address to read (Middle)
		ram_addr[2] = (unsigned char)((vramAddr&0xff0000)>>16); // address to read (Higher)
		WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);
		WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
		WriteOneByte(&g_ts, 0x43, ram_addr[2]);
		WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_DCMAP);

		size = ((bytes >= 120) ? 120 : bytes);
		TOUCH_LOG("size : %d\n", size);

		/* 120 bytes = 120/2=60 elements (signed 16bit little endian) */
		ReadMultiBytes(&g_ts, 0x08, size, &(pDcDataBuffer[index]));
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

      x = dcmap[dindex++] = numSenseLine2;
      y = dcmap[dindex++] = numDriveLine2;
      dcmap[dindex++] = dsFlag;
      dcmap[dindex++] = 0x00; // reserved

	  // should be 17 x 31
	  TOUCH_LOG("readDcMapResult %d x %d items\n", x, y);

	  //top
	  sindex = (num_adc_dmy[0] + x + num_adc_dmy[1]) * num_adc_dmy[2] * 2;

	  // contents line
	  for (l = 0; l < y; l++) {
	    // left
	    sindex += (num_adc_dmy[0] * 2);

	    // contents
	    memcpy((u8 *)&(dcmap[dindex]), (u8 *)&(pDcDataBuffer[sindex]), (x*2));
	    dindex += (x*2);
	    sindex += (x*2);

	    // right
	    sindex += (num_adc_dmy[1] * 2);
	  }

	} //L3

#endif /* 0 */


// for DCmap, sense(18)-1=17
	TOUCH_LOG("    [%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d]\n", 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1);

	WRITE_SYSBUF(buf, ret, "    [%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d]\n", 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1);

	for (drive = 0; drive < NUM_DRIVE; drive++) {
//                        1  2  3  4  5  6  7  8  9  0  1  2  3  4  5  6  7
		TOUCH_LOG("[%02d]%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d\n",\
				drive,
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 33]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 32]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 31]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 30]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 29]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 28]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 27]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 26]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 25]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 24]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 23]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 22]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 21]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 20]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 19]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 18]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 17]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 16]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 15]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 14]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 13]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 12]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 11]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 10]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 9]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 8]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 7]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 6]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 5]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 4]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 3]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 2]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 1]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 0]));

//                                     1  2  3  4  5  6  7  8  9  0  1  2  3  4  5  6  7
		WRITE_SYSBUF(buf, ret, "[%02d]%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d\n",\
				drive,

				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 33]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 32]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 31]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 30]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 29]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 28]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 27]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 26]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 25]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 24]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 23]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 22]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 21]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 20]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 19]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 18]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 17]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 16]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 15]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 14]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 13]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 12]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 11]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 10]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 9]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 8]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 7]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 6]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 5]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 4]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 3]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 2]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 1]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 0]));
    }

	*pDataLen = ret;

	TOUCH_LOG("readDcMapResult done ret = %d\n", ret );

#if 0
	if(pDcDataBuffer != NULL)
		kfree(pDcDataBuffer);
#endif /* 0 */

	return ret;
}


#define CMD_DCMAP_ON  "\xD7\x00\x01\x00\x01"
#define CMD_DCMAP_OFF "\xD7\x00\x01\x00\x00"
#define CMD_DCMAP_ON_LEN 5
#define CMD_DCMAP_OFF_LEN 5

#define CMD_SETREGTBL_CALIB_ON   "\x06\x00\x04\x00\x00\x12\x6E\x01"
#define CMD_SETREGTBL_CALIB_OFF  "\x06\x00\x04\x00\x00\x12\x6E\x21"
#define CMD_SETREGTBL_CALIB_LEN 8
#define DC_CALIBED_DATA 0
#define DC_RAW_DATA 1

int shtsc_CMD_getDCMap_wo_IRQ(void *ts, int rawflag, char* buf)
{
	int resultDataLen = 0;
	int err = 0;

	TOUCH_FUNC();


	//TEMP 1116 SetBankAddr(ts, 0x12);
  if (rawflag == DC_CALIBED_DATA) {
    // normal DC data (delta)
    // bank=0x12, addr=0x6e, data=0x01
	issue_command(&g_ts, CMD_SETREGTBL_CALIB_ON, CMD_SETREGTBL_CALIB_LEN);
	TOUCH_LOG("delta (calibrated data) \n");
	WRITE_SYSBUF(buf, resultDataLen, "======== Deltadata ========\n");
  } else {
    // calib by-pass data (rawdata)
    // bank=0x12, addr=0x6e, data=0x21
	issue_command(&g_ts, CMD_SETREGTBL_CALIB_OFF, CMD_SETREGTBL_CALIB_LEN);
	TOUCH_LOG("raw data (DC by-pass data) \n");
	WRITE_SYSBUF(buf, resultDataLen, "======== DC Map ========\n");
  }
  msleep(50); // need for changing mode


#ifdef DEBUG_SHTSC
    TOUCH_LOG("exec DCMAP ON command (%s)\n", (rawflag ? "calib by-pass": "normal"));
#endif

	// set command
	issue_command(&g_ts, CMD_DCMAP_ON, CMD_DCMAP_ON_LEN);

//	msleep(500);
	// waiting for DCMAP ready
	(&g_ts)->wait_state = WAIT_DCMAP;
	(&g_ts)->wait_result = false;

	// wait for DCMAP ready
	err = WaitAsync(&g_ts);
	if( err != 0){
		//dump_stack();
		TOUCH_LOG("WaitAsync in getDCMap return %d\n",err);
	}
#if 0
	TOUCH_LOG("WaitAsync for DCMAP-ready is done");
#endif

    /* read DC map */
    readDcMapResult(buf, &resultDataLen);


#ifdef DEBUG_SHTSC
    TOUCH_LOG("exec DCMAP OFF command (%s)\n", (rawflag ? "calib by-pass": "normal"));
#endif

	// set command2. waitAsync is in later
	issue_command2(&g_ts, CMD_DCMAP_OFF, CMD_DCMAP_OFF_LEN);

    /* dc map read done */
    /* this must  be done first before COMMAND_RESULT interrupt*/
	SetIndicator(&g_ts, SHTSC_IND_DCMAPDONE);

	err = 0;
	err = WaitAsync(&g_ts); // for CMD_DCMAP_OFF in issue command2
	if( err != 0){
		//dump_stack();
		TOUCH_LOG("WaitAsync in getDCMap  return %d\n",err);
	}


	issue_command(&g_ts, CMD_SETREGTBL_CALIB_ON, CMD_SETREGTBL_CALIB_LEN);
	//TOUCH_LOG("back to normal DC (CALIB-ed = delta) mode \n");
	msleep(50); // need for changing mode

	return resultDataLen;
}



static ssize_t show_delta(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	ret = shtsc_CMD_getDCMap_wo_IRQ(&g_ts, DC_CALIBED_DATA, buf );

	TOUCH_LOG("Show delta done\n");

	return ret;
}

static ssize_t show_rawdata(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	ret = shtsc_CMD_getDCMap_wo_IRQ(&g_ts, DC_RAW_DATA, buf);

	TOUCH_LOG("Show rawdata done\n");

	return ret;
}

static ssize_t show_pen_support(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	WRITE_BUFFER(buf, ret,	"1\n");

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

static ssize_t show_unique_id(struct i2c_client *client, char *buf)
{
	int ret = 0;

    unsigned char unique_id_Buf[16];

    if(get_unique_id(unique_id_Buf) == TOUCH_SUCCESS){
        if (unique_id_Buf[11] == TYPE_ID_JIG_4REN) {
            TOUCH_LOG("Unique ID (4Ren-JIG): %02X/%02X/%02X B6:%4d pic_slot:%1d serial:%5d pic_rev:%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    ((unique_id_Buf[4] << 8) | unique_id_Buf[3]), //little endian
                    unique_id_Buf[5],
                    ((unique_id_Buf[7] << 8) | unique_id_Buf[6]), //little endian
                    ((unique_id_Buf[9] << 8) | unique_id_Buf[8])); //little endian

            WRITE_SYSBUF(buf, ret, "Unique ID (4Ren-JIG): %02X/%02X/%02X B6:%4d pic_slot:%1d serial:%5d pic_rev:%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    ((unique_id_Buf[4] << 8) | unique_id_Buf[3]), //little endian
                    unique_id_Buf[5],
                    ((unique_id_Buf[7] << 8) | unique_id_Buf[6]), //little endian
                    ((unique_id_Buf[9] << 8) | unique_id_Buf[8])); //little endian

        } else {
            TOUCH_LOG("Unique ID (LPC-JIG): %02d%02d%02d-%01d%01d-%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    unique_id_Buf[3],
                    unique_id_Buf[4],
                    ((unique_id_Buf[6] << 8) | unique_id_Buf[5])); //little endian

            WRITE_SYSBUF(buf, ret, "Unique ID (LPC-JIG): %02d%02d%02d-%01d%01d-%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    unique_id_Buf[3],
                    unique_id_Buf[4],
                    ((unique_id_Buf[6] << 8) | unique_id_Buf[5])); //little endian

        }
    } else {
        TOUCH_LOG("get_unique_id_wo_IRQ fail \n");
		WRITE_SYSBUF(buf, ret, "Read failed\n");
    }

	return ret;
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
static LGE_TOUCH_ATTR(unique_id, S_IRUGO | S_IWUSR, show_unique_id, NULL);

static struct attribute *LR388K6_attribute_list[] = {
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
	&lge_touch_attr_unique_id.attr,
	NULL,
};


static const struct file_operations lr388k6_fops = {
	.owner = THIS_MODULE,
	.open	= lr388k6_misc_open,
	.release = lr388k6_misc_release,
	.read	= lr388k6_misc_read,
	.llseek = no_llseek,
	.unlocked_ioctl = lr388k6_misc_unlocked_ioctl,
	.compat_ioctl = lr388k6_misc_unlocked_ioctl,
};

static int lr388k6_misc_open(struct inode *inode, struct file *file)
{
	TOUCH_FUNC();

	return 0;
}

static int lr388k6_misc_release(struct inode *inode, struct file *file)
{
	TOUCH_FUNC();

	return 0;
}

static ssize_t lr388k6_misc_read(struct file *file, char *buf, size_t count, loff_t *pos)
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

static long lr388k6_misc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
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
			TOUCH_LOG("case REG_N_RW_SET_ADDR: cmd %x, arg %lx a:0x%x len:%ld\n", cmd, arg, s_shtsc_addr, s_shtsc_len);
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
			TOUCH_LOG("Driver Multibyte write. addr %02X, len: %lx, data[0-3]: %02X %02X %02X %02X ....\n", s_shtsc_addr, s_shtsc_len, s_shtsc_buf[0], s_shtsc_buf[1], s_shtsc_buf[2], s_shtsc_buf[3]);
#endif
#if defined(DEBUG_SHTSC)
			TOUCH_LOG(" cmd %x, arg %lx a:0x%x d:0x%lx(%ld)\n", cmd, arg, s_shtsc_addr, s_shtsc_len, s_shtsc_len);
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
			TOUCH_LOG("Driver Multibyte read done. addr %02X, len: %lx, data[0-3]: %02X %02X %02X %02X ....\n",
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
				int err = 0;
				g_ts.wait_state = WAIT_RESET;
				g_ts.wait_result = false;
				//shtsc_reset(&g_ts, true);
				TouchResetCtrl(1);
				// wait
				err = WaitAsync(&g_ts);
				if( err != 0){
					//dump_stack();
					TOUCH_LOG("WaitAsync in SHTSC_IOCTL_RESET return %d\n",err);
				}
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

static int check_flash_load_error(struct i2c_client *client)
{
	u16 u16status = 0;
	u8 regcommonbuf[11] = {0, };	

	g_ts.client = client;

	ReadMultiBytes(&g_ts,SHTSC_ADDR_INT0,11,regcommonbuf);
	u16status = ((regcommonbuf[SHTSC_ADDR_INT1] << 8) | regcommonbuf[SHTSC_ADDR_INT0]);

	if (u16status & SHTSC_STATUS_FLASH_LOAD_ERROR){
		TOUCH_LOG("LR388K6_Initialize [IRQ] shtsc flash load error u16status=0x%04x\n",u16status);
		return TOUCH_FAIL;
	}	

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : LR388K6_Initialize
// Description
//   - 
//   - 
//====================================================================

static int LR388K6_Initialize(struct i2c_client *client)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	int retry_cnt = 0;
	int result = TOUCH_SUCCESS;

	TOUCH_FUNC();

	// init g_ts;
	g_ts.client = client;
	g_ts.wait_state = WAIT_NONE;
	g_ts.wait_result = false;


	pDriverData->isMiscDevice = 1;
	pDriverData->touch_misc_device.name = DRIVER_NAME;
	pDriverData->touch_misc_device.fops = &lr388k6_fops;

	if(check_flash_load_error(client) == TOUCH_FAIL){
		for(retry_cnt = 0; retry_cnt < 3; retry_cnt++){
			result = LR388K6_UpdateFirmware(client, NULL);
			if(result == TOUCH_SUCCESS){
				TOUCH_LOG("Firmware Recovery was Succeeded\n");
				break;
			}
			TOUCH_WARN("Retry firmware Recovery\n");
		}
		LR388K6_Reset(client);
	}

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : LR388K6_Reset
// Description
//   - 
//   - 
//====================================================================
static void LR388K6_Reset(struct i2c_client *client)
{
//	int count = 0;

	//TOUCH_FUNC();

	TouchResetCtrl(0);
	msleep(50);
	// frequent irq interrupted
	//
	G_Irq_Mask = 0xffff;
	TouchResetCtrl(1);

	msleep(200);

#if 0 //HMOON TEMP
	// if POWER_UP or FLASH_LOAD_ERROR, then break
	while (! ((SHTSC_STATUS_POWER_UP & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) ||
             (SHTSC_STATUS_FLASH_LOAD_ERROR & ReadOneByte(&g_ts, SHTSC_ADDR_INT1)))) {
		; // waiting for ready status after reset
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(10);
		count++;

		if(count > 100)
			break;
	}

#endif

	dummyDeviceState = STATE_NORMAL;
}


//====================================================================
// Function : LR388K6_Connect
// Description
//   - 
//   - 
//====================================================================
static int LR388K6_Connect(void)
{
	TOUCH_FUNC();

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : LR388K6_InitRegister
// Description
//   - Initialize touch IC register
//   - will be called after IC reset ( by reset pin )
//====================================================================
static int LR388K6_InitRegister(struct i2c_client *client)
{
	//	struct device_node *np;
	//	TouchDriverData *pDriverData;

	TOUCH_FUNC();

	//msleep(100);

	return TOUCH_SUCCESS;

}

//====================================================================
// Function : LR388K6_ClearInterrupt
// Description
//   - Clear interrupt
//   - will be called before interrupt enable to clear interrupt happened during interrupt disabled time
//====================================================================
static void LR388K6_ClearInterrupt(struct i2c_client *client)
{
	//TOUCH_FUNC();

	return;
}

//====================================================================
// Function : LR388K6_InterruptHandler
// Description
//   - process interrupt
//   - will be called if interrupt detected by AP
//====================================================================
static int LR388K6_InterruptHandler(struct i2c_client *client, TouchReadData *pData)
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
			case WAIT_DCMAP:
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

// this function should be called while IRQ is disabled or multex is locked
// i.e. WaitAsync() cannot be called because it will be timeout without IRQ handler to move to WAIT_NONE
int issue_command_wo_IRQ(void *ts, unsigned char *cmd, unsigned int len)
{
	int count = 0;
	u8 value = 0;

	//TOUCH_FUNC();

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, cmd, len);

	SetIndicator(ts, SHTSC_IND_CMD);

	value = ReadOneByte(ts, SHTSC_ADDR_INT0);
#ifdef DEBUG_SHTSC
	TOUCH_LOG("Value check value %02XH SHTSC_STATUS_COMMAND_RESULT & value %02XH \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));
#endif

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 100){
			TOUCH_LOG("waiting the reult count %d. Please check it\n", count);
			break;
		}
	}

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	return 0;
}

int issue_command(void *ts, unsigned char *cmd, unsigned int len)
{
	int err = 0;

	//disable_irq(g_ts.client->irq);

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, cmd, len);
#if 0
	TOUCH_LOG("set command (%x)\n",cmd[0]);
#endif

	// prepare waiting
	((struct shtsc_i2c *)ts)->cmd = cmd[0];
	((struct shtsc_i2c *)ts)->wait_state = WAIT_CMD;
	((struct shtsc_i2c *)ts)->wait_result = true;

	// do it
	SetIndicator(ts, SHTSC_IND_CMD);
	//DBGLOG("do it\n");

	//enable_irq(g_ts.client->irq);

	// wait
	err = WaitAsync(ts);
	if( err != 0){
		//dump_stack();
		TOUCH_LOG("WaitAsync in issue_command return %d\n",err);
	}

	return err;
}

/* only for DCmap off for one shot */
int issue_command2(void *ts, unsigned char *cmd, unsigned int len)
{
	int err = 0;

	//disable_irq(g_ts.client->irq);

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, cmd, len);
#if 0
	TOUCH_LOG("set command (%x)\n",cmd[0]);
#endif

	// prepare waiting
	((struct shtsc_i2c *)ts)->cmd = cmd[0];
	((struct shtsc_i2c *)ts)->wait_state = WAIT_CMD;
	((struct shtsc_i2c *)ts)->wait_result = true;

	// do it
	SetIndicator(ts, SHTSC_IND_CMD);
	//DBGLOG("do it\n");

	//enable_irq(g_ts.client->irq);

	// wait
//	WaitAsync(ts);
	TOUCH_LOG("issue_command2 executed.\n");

	return err;
}

int lpwg_param_command(void *ts, u8 u8Addr, unsigned char *cmd, unsigned int len)
{
	int err = 0;

	SetBankAddr(ts, SHTSC_BANK_LPWG_PARAM);
	//TOUCH_LOG("tci bank command LPWG_PARM \n");
	// set command
	WriteMultiBytes(ts, u8Addr, cmd, len);
	//TOUCH_LOG("tci set LPWG_PARM command (0x%x) (0x%x) \n",cmd[0], cmd[1]);

	return err;
}

//====================================================================
// Function : LR388K6_ReadIcFirmwareInfo
// Description
//   - Read firmware information from touch IC
//   - will be called at boot time or after writing new firmware
//====================================================================
static int LR388K6_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{

	// unsigned firmver = 0;
	// unsigned paramver = 0;

	int value =0;
	int count =0;

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
	if( TOUCH_FAIL == SetBankAddr(&g_ts, SHTSC_BANK_COMMAND)){
		TOUCH_LOG("I2C failure \n");
		return TOUCH_FAIL;
	}

	// set command
	WriteMultiBytes(&g_ts, SHTSC_ADDR_COMMAND, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	// Indicator
	SetIndicator(&g_ts, SHTSC_IND_CMD);

	msleep(100);
	TOUCH_LOG("Value check value %02XH SHTSC_STATUS_COMMAND_RESULT & value %02XH \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(&g_ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 10){
			TOUCH_LOG("waiting the reult count %d. Please check it\n", count);
			break;
		}
	}

	value = ReadOneByte(&g_ts, SHTSC_ADDR_INT0);
	TOUCH_LOG("1st Value check value %d SHTSC_STATUS_COMMAND_RESULT & value %d \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	// Bank address
	SetBankAddr(&g_ts,SHTSC_BANK_COMMAND_RESULT);

	// READ result
	ReadMultiBytes(&g_ts, 0x08, MAX_COMMAND_RESULT_LEN, CommandResultBuf); // always read entire result. TEGAPP will handle it.
	TOUCH_LOG("Command result (GetProperty[E0]), Operation code: %02X, Error code:%02X\n", CommandResultBuf[0], CommandResultBuf[1]);

#if 0
	TOUCH_LOG(" command result, %02X, %02X, %02X, %02X\n", CommandResultBuf[0], CommandResultBuf[1], CommandResultBuf[2],CommandResultBuf[3]);
	TOUCH_LOG(" command result, %02X, %02X, %02X, %02X\n", CommandResultBuf[4], CommandResultBuf[5], CommandResultBuf[6],CommandResultBuf[7]);
	TOUCH_LOG(" command result, %02X, %02X, %02X, %02X\n", CommandResultBuf[8], CommandResultBuf[9], CommandResultBuf[10],CommandResultBuf[11]);
	TOUCH_LOG(" command result, %02X, %02X, %02X, %02X\n", CommandResultBuf[12], CommandResultBuf[13], CommandResultBuf[14],CommandResultBuf[15]);
#endif

	K6_firmver = 0;
	K6_paramver = 0;

	K6_firmver = ((CommandResultBuf[0x0a-0x08 +3] << 24) |
			(CommandResultBuf[0x0a-0x08 +2] << 16) |
			(CommandResultBuf[0x0a-0x08 +1] << 8) |
			(CommandResultBuf[0x0a-0x08 +0] << 0));
	K6_paramver = ((CommandResultBuf[0x12-0x08 +3] << 24) |
			(CommandResultBuf[0x12-0x08 +2] << 16) |
			(CommandResultBuf[0x12-0x08 +1] << 8) |
			(CommandResultBuf[0x12-0x08 +0] << 0));

	TOUCH_LOG("Firmware %08X, Parameter %08X\n", K6_firmver, K6_paramver);

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	pFwInfo->moduleMakerID = 2;  
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 3;
	pFwInfo->isOfficial = (CommandResultBuf[0x12-0x08+2] & 0x01);
	pFwInfo->version = CommandResultBuf[0x12-0x08];

	TOUCH_LOG("FW official %d version %d \n", pFwInfo->isOfficial, pFwInfo->version);

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
// Function : LR388K6_GetBinFirmwareInfo
// Description
//   - parse and return firmware information from firmware image
//   - if filename is NULL, return information of default firmware image
//   - will be called at boot time or needed
//====================================================================
static int LR388K6_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
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
			pBin[0x23+7],pBin[0x22+7],pBin[0x21+7],pBin[0x20+7],
			pBin[image_size-6],pBin[image_size-7],pBin[image_size-8],pBin[image_size-9] );
#endif

	pFwInfo->version = pBin[image_size-9]; // HMOON ((firmver >> 32) | paramver); // 32bit value

#endif

	pFwInfo->moduleMakerID = 2;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 3;
	pFwInfo->isOfficial = (pBin[image_size-7] & 0x01);
	pFwInfo->version = pBin[image_size-9];

	TOUCH_LOG("BIN official %d version %d \n", pFwInfo->isOfficial, pFwInfo->version);

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
//	pFwInfo->isOfficial = 1;
	pFwInfo->isOfficial = 0;
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
// Function : LR388K6_UpdateFirmware
// Description
//   - Write firmware to touch IC
//   - if filename is NULL, use default firmware image
//   - common driver will call Reset(), InitRegister() and ReadIcFirmwareInfo() one by one after writing
//====================================================================
static int LR388K6_UpdateFirmware(struct i2c_client *client, char *pFilename)
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
// Function : LR388K6_SetLpwgMode
// Description
//   - Set device to requested state
//====================================================================
static int LR388K6_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int ret = TOUCH_SUCCESS;


    /* FOLIOCOVER_CLOSE : 0 FOLIOCOVER_OPEN : 1 */
    if(pLpwgSetting->coverState == FOLIOCOVER_CLOSED) {

    }
    else {
        pLpwgSetting->activeTouchAreaX1 = 106;// 7mm : 106 pixel
        pLpwgSetting->activeTouchAreaX2 = 973;// 1080 - 106
        pLpwgSetting->activeTouchAreaY1 = 0;//
        pLpwgSetting->activeTouchAreaY2 = 1920;//
    }
#if 0
	TOUCH_FUNC();
	TOUCH_LOG("mode [%d]\n",				pLpwgSetting->mode);
	TOUCH_LOG("lcdPixelSizeX [%d]\n",		pLpwgSetting->lcdPixelSizeX);
	TOUCH_LOG("lcdPixelSizeY [%d]\n",		pLpwgSetting->lcdPixelSizeY);
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

#endif
	memcpy(&(g_ts.lpwgSetting), pLpwgSetting, sizeof(LpwgSetting));

	if( dummyDeviceState == newState ) {
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

	switch( newState )
	{
		case STATE_NORMAL:
			//TOUCH_LOG("device was set to NORMAL\n");
			break;
		case STATE_OFF:
			//TOUCH_LOG("device was set to OFF\n");
			break;
		case STATE_KNOCK_ON_ONLY:
			//TOUCH_LOG("device was set to KNOCK_ON_ONLY\n");
			break;
		case STATE_KNOCK_ON_CODE:
			//TOUCH_LOG("device was set to KNOCK_ON_CODE\n");
			break;
		case STATE_NORMAL_HOVER:
			//TOUCH_LOG("device was set to NORMAL_HOVER\n");
			break;
		case STATE_HOVER:
			//TOUCH_LOG("device was set to HOVER\n");
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

static int checkLimitData(u8* buffer, signed short* limitData, int limitType, char* buf, int dataLen)
{
    int driving = 0;
    int sensing = 0;
    int brokenPoint = 0;
    unsigned short rawData = 0;

    WRITE_RESULT(buf, dataLen, "%s limit Test\n", (limitType == UPPER_DATA)?"Upper":"Lower");
    WRITE_RESULT(buf, dataLen, "[RX]\t[TX]\tValue\n");
    TOUCH_LOG("%s limit Test\n", (limitType == UPPER_DATA)?"Upper":"Lower");
    TOUCH_LOG("[RX]\t[TX]\tValue\n");

    for(sensing = 0; sensing < NUM_SENSE; sensing++)
    {
        for(driving = 0; driving < NUM_DRIVE; driving++)
        {
            rawData = (signed short)((buffer[(NUM_SENSE * driving + sensing) * 2 +  1]<<8) | buffer[(NUM_SENSE * driving + sensing) * 2 ]);

			if( rawData == 0 ) {
				brokenPoint++;

			} else if((limitType == UPPER_DATA)){
                if((rawData > limitData[(NUM_DRIVE * sensing) + driving])){
					if(brokenPoint < 5){
						WRITE_RESULT(buf, dataLen, "[%2d]\t[%2d]\t%d\n", sensing + 1, driving + 1, rawData);
						TOUCH_LOG("[%2d]\t[%2d]\t%d\n", sensing + 1, driving + 1, rawData);
					}
                    brokenPoint++;
                }
            } else {
                if((rawData < limitData[(NUM_DRIVE * sensing) + driving])){
					if(brokenPoint < 5){
						WRITE_RESULT(buf, dataLen, "[%2d]\t[%2d]\t%d\n", sensing + 1, driving + 1, rawData);
						TOUCH_LOG("[%2d]\t[%2d]\t%d\n", sensing + 1, driving + 1, rawData);
					}
                    brokenPoint++;
                }
            }
        }
    }

    WRITE_RESULT(buf, dataLen, "\tFail\t%4d\n", brokenPoint);
    TOUCH_LOG("\tFail\t%4d\n", brokenPoint);

    return dataLen;
}

static int readDiagResult(char *buf, int *pDataLen, int *pRawDataStatus)
{
	/*
	 *  (0,0)L,H (1,0)L,H ....(17,0)L,H = 36byte
	 *  (0,1)L,H (1,1)L,H ....(17,1)L,H = 36byte
	 *  (0,2)L,H (1,2)L,H ....(17,2)L,H = 36byte
	 *  :
	 *  (0,30)L,H (1,30)L,H ....(17,30)L,H = 36byte
	 *  singed Little endian
	 */

	unsigned char ram_addr[3];
	unsigned int vramAddr = 0x0170E8; // the first address of the map
	//unsigned int readingSize = (NUM_DRIVE*NUM_SENSE*2);
	//int bytes = readingSize;
	//int size;
	int index = 0;

	//int drive = 0; // for debug
	int sense = 0; // for debug

	int ret = *pDataLen;

	// Version information
	// Self testing
	// Raw dataLen
	// Other testing 

	//*pDataLen = readingSize;

	u8* pRawDataBuffer = NULL;

	//TOUCH_FUNC();
	//TOUCH_LOG("SelfDiag readingSize(%d) pDataLen %d\n", readingSize, *pDataLen);

	pRawDataBuffer =  kzalloc(sizeof(u8) * 1536 , GFP_KERNEL);
	if(pRawDataBuffer == NULL){
		TOUCH_ERR("kzalloc failed size %d \n", ( sizeof(u8) *  1536 ) );
		return TOUCH_FAIL;
	}

	if(pRawDataBuffer == NULL){
		TOUCH_ERR("kzalloc failed size %d \n", ( sizeof(signed short) *  TOTAL_CHANNAL ) );
		return TOUCH_FAIL;
	}

	memset(pRawDataBuffer, 0x0, (sizeof(u8) * 1536));

	SetBankAddr(&g_ts,SHTSC_BANK_DCMAP);
	/* read 120 bytes from Bank5, address 8 */
	/* read loop required */

	for(index=0;index<NUM_DRIVE;index++){
		ram_addr[0] = (unsigned char)(vramAddr&0xff); // set address to read (Lower)
		ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // set address to read (Middle)
		ram_addr[2] = (unsigned char)((vramAddr&0xff0000)>>16); // set address to read (Higher)
		WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);
		WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
		WriteOneByte(&g_ts, 0x43, ram_addr[2]);
		WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_DCMAP);

		/* 36 bytes = 36/2=18 elements (signed 16bit little endian) */
		ReadMultiBytes(&g_ts, 0x08, NUM_SENSE*sizeof(s16), &(pRawDataBuffer[index*NUM_SENSE*sizeof(s16)]));
		vramAddr += (NUM_SENSE+3)/4*8;  //40 bytes including 4 bytes padding
		//TOUCH_LOG("vramAddr 0x%X\n", vramAddr);

	}

    /* Panal limit data filtering */
    if(*pRawDataStatus == TOUCH_SUCCESS){
        ret = checkLimitData(pRawDataBuffer, Upper_limit, UPPER_DATA, buf, ret);
        ret = checkLimitData(pRawDataBuffer, Lower_limit, LOWER_DATA, buf, ret);
    }

	TOUCH_LOG("RawCap data\n");
	WRITE_RESULT(buf, ret, "RawCap data\n");

#if 0

	TOUCH_LOG("     [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d]\n", \
			1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18);
	WRITE_RESULT(buf, ret, "     [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d] [%3d]\n", \
			1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18);

	for (drive = 0; drive < NUM_DRIVE; drive++) {

		TOUCH_LOG("[%02d]%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d\n",\
				drive+1,
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 +  1]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 0]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 +  3]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 2]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 +  5]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 4]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 +  7]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 6]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 +  9]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 8]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 11]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 10]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 13]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 12]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 15]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 14]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 17]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 16]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 19]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 18]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 21]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 20]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 23]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 22]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 25]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 24]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 27]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 26]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 29]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 28]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 31]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 30]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 33]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 32]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 35]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 34]));

		WRITE_RESULT(buf, ret, "[%02d]%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d%6d\n",\
				drive+1,
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 +  1]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 0]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 +  3]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 2]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 +  5]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 4]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 +  7]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 6]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 +  9]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 8]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 11]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 10]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 13]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 12]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 15]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 14]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 17]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 16]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 19]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 18]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 21]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 20]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 23]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 22]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 25]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 24]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 27]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 26]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 29]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 28]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 31]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 30]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 33]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 32]),\
				(signed short)((pRawDataBuffer[NUM_SENSE*drive*2 + 35]<<8) | pRawDataBuffer[NUM_SENSE*drive*2 + 34]));
	}
#endif

	TOUCH_LOG("     [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d]\n", \
			1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31);

	WRITE_RESULT(buf, ret, "     [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d]\n", \
			1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31);

	for (sense = 0; sense < NUM_SENSE; sense++) {


		TOUCH_LOG("[%02d]%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d\n",\
				sense+1,
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  0 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  0 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  1 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  1 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  2 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  2 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  3 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  3 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  4 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  4 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  5 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  5 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  6 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  6 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  7 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  7 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  8 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  8 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  9 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  9 + sense ) * 2 ]),\

				(signed short)((pRawDataBuffer[ (NUM_SENSE * 10 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 10 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 11 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 11 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 12 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 12 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 13 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 13 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 14 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 14 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 15 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 15 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 16 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 16 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 17 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 17 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 18 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 18 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 19 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 19 + sense ) * 2 ]),\

				(signed short)((pRawDataBuffer[ (NUM_SENSE * 20 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 20 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 21 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 21 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 22 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 22 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 23 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 23 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 24 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 24 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 25 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 25 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 26 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 26 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 27 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 27 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 28 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 28 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 29 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 29 + sense ) * 2 ]),\

				(signed short)((pRawDataBuffer[ (NUM_SENSE * 30 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 30 + sense ) * 2 ]));


		WRITE_RESULT(buf, ret, "[%02d]%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d\n",\
				sense+1,
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  0 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  0 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  1 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  1 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  2 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  2 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  3 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  3 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  4 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  4 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  5 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  5 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  6 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  6 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  7 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  7 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  8 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  8 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  9 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  9 + sense ) * 2 ]),\

				(signed short)((pRawDataBuffer[ (NUM_SENSE * 10 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 10 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 11 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 11 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 12 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 12 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 13 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 13 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 14 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 14 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 15 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 15 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 16 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 16 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 17 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 17 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 18 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 18 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 19 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 19 + sense ) * 2 ]),\

				(signed short)((pRawDataBuffer[ (NUM_SENSE * 20 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 20 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 21 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 21 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 22 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 22 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 23 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 23 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 24 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 24 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 25 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 25 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 26 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 26 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 27 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 27 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 28 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 28 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 29 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 29 + sense ) * 2 ]),\

				(signed short)((pRawDataBuffer[ (NUM_SENSE * 30 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 30 + sense ) * 2 ]));

	}

	*pDataLen = ret;

	TOUCH_LOG("readDiagResult done ret = %d pDatalen = %d\n", ret, *pDataLen );

	if(pRawDataBuffer != NULL)
		kfree(pRawDataBuffer);

	return TOUCH_SUCCESS;
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

	//TOUCH_FUNC();

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_SELFDIAG, CMD_SELFDIAG_LEN);

	SetIndicator(ts, SHTSC_IND_CMD);

	msleep(500);

	value = ReadOneByte(ts, SHTSC_ADDR_INT0);
#if defined(DEBUG_SHTSC)
	TOUCH_LOG("Value check value %02XH SHTSC_STATUS_COMMAND_RESULT & value %02XH \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));
#endif

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the reult count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 200){
			TOUCH_LOG("waiting the reult count %d. Please check it\n", count);
			break;
		}
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
// Function : LR388K6_DoSelfDiagnosis
// Description
//   - diagnose touch pannel and return result
//   - can use pBuf to give more information ( be careful not to exceed buffer size )
//   - should create a file of result ( TBD : consider file saving on common driver side )
//====================================================================

static int LR388K6_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	u8 * pResultData = NULL;
	int resultDataLen = 0;
	int dataLen = 0;
	int ret = 0;
	char cmdbin_selfdiag[]={0xDA,0x00,0x00,0x01}; // selfdiag command
    unsigned char unique_id_Buf[16];

    TouchDriverData *pDriverData = i2c_get_clientdata(client);

	u8 *time_string = NULL;


	TOUCH_FUNC();

	*pDataLen = (int) 0;

	*pRawStatus = (int) TOUCH_FAIL;

	*pChannelStatus = (int) TOUCH_FAIL;


	pResultData =  kzalloc(sizeof(u8) * RESULT_DATA_MAX_LEN , GFP_KERNEL);
	if(pResultData == NULL){
		TOUCH_ERR("kzalloc failed size %d \n", ( sizeof(u8) *  RESULT_DATA_MAX_LEN ) );
		return TOUCH_FAIL;
	}

	time_string =  kzalloc(sizeof(u8) * TIME_DATA_MAX_LEN, GFP_KERNEL);
	if(time_string == NULL){
		TOUCH_ERR("kzalloc failed size %d \n", ( sizeof(u8) *  TIME_DATA_MAX_LEN ) );
		if(pResultData != NULL)
        {
		    kfree(pResultData);
            pResultData = NULL;
        }
        return TOUCH_FAIL;
	}

	memset(time_string, 0x0, (sizeof(u8) * TIME_DATA_MAX_LEN));

    // 0 Current Time
    if(TOUCH_FAIL == get_current_time(time_string))
    {
		if(pResultData != NULL)
        {
		    kfree(pResultData);
            pResultData = NULL;
        }
		if(time_string != NULL)
        {
		    kfree(time_string);
            time_string = NULL;
        }
        TOUCH_ERR("get current time failed!\n");
        return TOUCH_FAIL;
    }

    //WRITE_RESULT(pResultData, resultDataLen, "\n======== Current Time =========\n" );
    WRITE_RESULT(pResultData, resultDataLen, "\n%s\n", time_string);

    // 1 Calbiration
	//shtsc_CMD_ExecCalibration_wo_IRQ(&g_ts);

    //WRITE_RESULT(pResultData, resultDataLen, "\n========== Calibration Done ==========\n");

	// 2 Self Diagnosis
	shtsc_CMD_SelfDiag_wo_IRQ(&g_ts);


#if 0 //def

	TOUCH_LOG("SelfDiag command CommandResultBuf 0x08-0x08 0x%X\n", CommandResultBuf[0x08-0x08]);
	TOUCH_LOG("SelfDiag command CommandResultBuf 0x09-0x08 0x%X\n", CommandResultBuf[0x09-0x08]);
	TOUCH_LOG("SelfDiag command CommandResultBuf 0x0a-0x08 0x%X\n", CommandResultBuf[0x0a-0x08]);

#endif

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



    WRITE_RESULT(pResultData, resultDataLen, "=== Result ===\n" );
	WRITE_RESULT(pResultData, resultDataLen, "Channel Status : %s \n", (*pChannelStatus == TOUCH_SUCCESS) ? "Success" : "Fail" );
	WRITE_RESULT(pResultData, resultDataLen, "Raw Data : %s \n", (*pRawStatus == TOUCH_SUCCESS) ? "Success" : "Fail" );

	WRITE_RESULT(pResultData, resultDataLen, "=== Information ===\n");
	WRITE_RESULT(pResultData, resultDataLen, "FW Info        = %d.%02d\n", pDriverData->icFwInfo.isOfficial, pDriverData->icFwInfo.version);
    WRITE_RESULT(pResultData, resultDataLen, "Module Info    = %d\n", pDriverData->icFwInfo.moduleVersion);
    WRITE_RESULT(pResultData, resultDataLen, "Model ID       = %d\n", pDriverData->icFwInfo.modelID);

	//WRITE_RESULT(pResultData, resultDataLen, "========= Additional Information =========\n");
	WRITE_RESULT(pResultData, resultDataLen, "Device name    = LR388K6 , Provider = Sharp \n" );
    WRITE_RESULT(pResultData, resultDataLen, "Product ID     = %d\n", pDriverData->icFwInfo.moduleMakerID);
	WRITE_RESULT(pResultData, resultDataLen, "Firmware Ver   = %08X\n", K6_firmver);
	WRITE_RESULT(pResultData, resultDataLen, "Parameter Ver  = %08X\n", K6_paramver);

	TOUCH_LOG("Firmware %08X, Parameter %08X\n", K6_firmver, K6_paramver);

	readDiagResult(pResultData, &resultDataLen, pRawStatus);

    if(get_unique_id_wo_IRQ(unique_id_Buf) == TOUCH_SUCCESS){
        if (unique_id_Buf[11] == TYPE_ID_JIG_4REN) {
            TOUCH_LOG("Unique ID (4Ren-JIG): %02X/%02X/%02X B6:%4d pic_slot:%1d serial:%5d pic_rev:%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    ((unique_id_Buf[4] << 8) | unique_id_Buf[3]), //little endian
                    unique_id_Buf[5],
                    ((unique_id_Buf[7] << 8) | unique_id_Buf[6]), //little endian
                    ((unique_id_Buf[9] << 8) | unique_id_Buf[8])); //little endian

            WRITE_RESULT(pResultData, resultDataLen, "Unique ID (4Ren-JIG): %02X/%02X/%02X B6:%4d pic_slot:%1d serial:%5d pic_rev:%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    ((unique_id_Buf[4] << 8) | unique_id_Buf[3]), //little endian
                    unique_id_Buf[5],
                    ((unique_id_Buf[7] << 8) | unique_id_Buf[6]), //little endian
                    ((unique_id_Buf[9] << 8) | unique_id_Buf[8])); //little endian

        } else {
            TOUCH_LOG("Unique ID (LPC-JIG): %02d%02d%02d-%01d%01d-%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    unique_id_Buf[3],
                    unique_id_Buf[4],
                    ((unique_id_Buf[6] << 8) | unique_id_Buf[5])); //little endian

            WRITE_RESULT(pResultData, resultDataLen, "Unique ID (LPC-JIG): %02d%02d%02d-%01d%01d-%04X\n",
                    unique_id_Buf[0],
                    unique_id_Buf[1],
                    unique_id_Buf[2],
                    unique_id_Buf[3],
                    unique_id_Buf[4],
                    ((unique_id_Buf[6] << 8) | unique_id_Buf[5])); //little endian

        }
    } else {
        TOUCH_LOG("get_unique_id_wo_IRQ fail \n");
		return TOUCH_FAIL;
    }

	//readDiagResult(pResultData, &resultDataLen, pRawStatus);

	write_result_data_to_file(NULL, pResultData, resultDataLen);

#if 0
	dataLen += sprintf(pBuf, "%s", "\n========= Version =========\n");
	dataLen += sprintf(pBuf+dataLen, "%s", "Device Name   = LR388K6\n");
	dataLen += sprintf(pBuf+dataLen, "FW Info       = %d.%02d\n", pDriverData->icFwInfo.isOfficial, pDriverData->icFwInfo.version);
    dataLen += sprintf(pBuf+dataLen, "Firmware Ver. = %08X\n", K6_firmver);
    dataLen += sprintf(pBuf+dataLen, "Parameter Ver.= %08X\n", K6_paramver);
#endif

	memcpy(pBuf+dataLen, pResultData, resultDataLen);

	dataLen = dataLen + resultDataLen;

	TOUCH_LOG(" PAGE_SIZE  %ld resultDataLen %d\n", PAGE_SIZE, resultDataLen);

	*pDataLen = (int) dataLen;

	if(pResultData != NULL){
		kfree(pResultData);
		pResultData = NULL;
    }

    if(time_string != NULL)
    {
        kfree(time_string);
        time_string = NULL;
    }

#if 0

	// hard-reset is required after self-d
	TouchResetCtrl(0);
	msleep(50);
	G_Irq_Mask = 0xffff;
	TouchResetCtrl(1);
	msleep(200);
#endif

    return ret;
}

//====================================================================
// Function : LR388K6_AccessRegister
// Description
//   - read from or write to touch IC
//====================================================================
static int LR388K6_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
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

static void LR388K6_NotifyHandler(struct i2c_client *client, TouchNotify notify, int data)
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


TouchDeviceSpecificFunction LR388K6_Func = {

	.Initialize = LR388K6_Initialize,
	.Reset = LR388K6_Reset,
	.Connect = LR388K6_Connect,
	.InitRegister = LR388K6_InitRegister,
	.ClearInterrupt = LR388K6_ClearInterrupt,
	.InterruptHandler = LR388K6_InterruptHandler,
	.ReadIcFirmwareInfo = LR388K6_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = LR388K6_GetBinFirmwareInfo,
	.UpdateFirmware = LR388K6_UpdateFirmware,
	.SetLpwgMode = LR388K6_SetLpwgMode,
	.DoSelfDiagnosis = LR388K6_DoSelfDiagnosis,
	.AccessRegister = LR388K6_AccessRegister,
	.NotifyHandler = LR388K6_NotifyHandler,
	.device_attribute_list = LR388K6_attribute_list,

};

/* End Of File */
