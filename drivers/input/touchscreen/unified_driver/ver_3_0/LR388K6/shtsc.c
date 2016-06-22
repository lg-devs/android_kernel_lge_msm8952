#if 0
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
#endif /* 0 */

//#define DEBUG_IRQ_DISABLE

/*
 * Driver for sharp touch screen controller
 *
 * Copyright (c) 2013 Sharp Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>

//#include <generated/uapi/linux/version.h>

//#define DP_8074_DEMO // old board
#define DP_8084_EVA // new board
#ifdef DP_8074_DEMO
#elif defined DP_8084_EVA
#include <linux/regulator/consumer.h> // 2015.04.09 added by Y.Nakamura for initialize regulator (pma8084)
#endif /* DP_8074_DEMO */

// should be defined automatically
#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#endif /* KERNEL_VERSION */

#ifndef LINUX_VERSION_CODE
#ifdef DP_8074_DEMO
#define LINUX_VERSION_CODE (KERNEL_VERSION(3,4,0)) //dp qualcomm
#elif defined DP_8084_EVA
#define LINUX_VERSION_CODE (KERNEL_VERSION(3,10,0)) //dp new qualcomm
#endif /* DP_8074_DEMO */
#endif /* LINUX_VERSION_CODE */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0))
#define __devinit
#define __devexit
#define __devexit_p
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)) */


//2014.10.16 added
#include <linux/string.h>
#include <linux/of_gpio.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <linux/mutex.h>

#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
#include <linux/i2c.h>
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
#include <linux/spi/spi.h>
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

#include "shtsc_ioctl.h"
#include "shtsc.h"

// device code
#define DEVICE_CODE_LR388K5 2

char VersionYear = 15;
char VersionMonth = 6;
char VersionDay = 12;
char VersionSerialNumber = 20; // reset on another day
char VersionModelCode = DEVICE_CODE_LR388K5;
#define DRIVER_VERSION_LEN (5) // do not change

#define SHTSC_I2C_P_MAX		((1<<16)-1) //8000 // depends on firmware
#define SHTSC_I2C_SIZE_MAX		((1<<6)-1)

/* DEBUG */
#if 1
#ifndef DEBUG_SHTSC
#define DEBUG_SHTSC
#endif//DEBUG_SHTSC
#endif /* 0 */

#define DBGLOG(format, args...)  printk(KERN_INFO format, ## args)
#define ERRLOG(format, args...)  printk(KERN_ERR format, ## args)

/* INT STATUS */
#define SHTSC_STATUS_TOUCH_READY    (1<<0)
#define SHTSC_STATUS_POWER_UP       (1<<1)
#define SHTSC_STATUS_RESUME_PROX    (1<<2)
#define SHTSC_STATUS_WDT            (1<<3)
#define SHTSC_STATUS_DCMAP_READY    (1<<4)
#define SHTSC_STATUS_COMMAND_RESULT (1<<5)
#define SHTSC_STATUS_KNOCK_CODE     (1<<6)
#define SHTSC_STATUS_FLASH_LOAD_ERROR    (1<<8)
#define SHTSC_STATUS_PLL_UNLOCK     (1<<9)

/* DONE IND */
#define SHTSC_IND_CMD   0x20
#define SHTSC_IND_TOUCH 0x01

/* BANK Address */
#define SHTSC_BANK_TOUCH_REPORT   0x00
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

/* Touch Parmeters */
#define SHTSC_MAX_FINGERS 10
#define SHTSC_MAX_TOUCH_1PAGE 10
#define SHTSC_LENGTH_OF_TOUCH 8

/* Touch Status */
#define SHTSC_F_TOUCH ((u8)0x01)
#define SHTSC_F_TOUCH_OUT ((u8)0x03)
//#define SHTSC_P_TOUCH ((u8)0x02)
//#define SHTSC_P_TOUCH_OUT ((u8)0x04)

#define SHTSC_TOUCHOUT_STATUS ((u8)0x80)

#define SHTSC_ADDR_COMMAND 0x08

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

typedef enum _cmd_CalibratioMode_e
{
  CMD_CALIB_MANUAL	  = 0x00,
  CMD_CALIB_START_OF_FORCE= 0x01,
  CMD_CALIB_END_OF_FORCE  = 0x02,
  CMD_CALIB_MAX
} dCmdCalibMode_e;

#define CMD_INIT              0x01
#define CMD_SETSYSTEM_STATE   0x02
#define CMD_EXEC_CALIBRATION  0x0F

#define CMD_PAYLOAD_LENGTH(X)              \
	X == CMD_INIT		?	0x04 : \
	X == CMD_SETSYSTEM_STATE?	0x04 : \
	X == CMD_EXEC_CALIBRATION?	0x04 : \
	0x0

#define SHTSC_ICON_KEY_NUM 3
#ifdef SHTSC_ICON_KEY_NUM
#define USE_APPSELECT /* KEY_APPSELECT instead of KEY_MENU */
#ifdef USE_APPSELECT
const int shtsc_keyarray[SHTSC_ICON_KEY_NUM]={158,102,0x244}; /* KEY_BACK,KEY_HOME,KEY_APPSELECT */
#else /* USE_APPSELECT */
const int shtsc_keyarray[SHTSC_ICON_KEY_NUM]={158,102,139}; /* KEY_BACK,KEY_HOME,KEY_MENU */
#endif /* USE_APPSELECT */
#endif

//2014.11.20 added
#define CMD_DELAY             16

#define WAIT_NONE   (0)
#define WAIT_CMD    (1)
#define WAIT_RESET  (2)


#define MAX_16BIT			0xffff
#define MAX_12BIT			0xfff

#define MAX_DCMAP_SIZE (37*37*2)

#define LOW_LEVEL 0
#define HIGH_LEVEL 1


/* ======== EEPROM command ======== */
#define	FLASH_CMD_WRITE_EN		(0x06)		/* Write enable command */
#define	FLASH_CMD_PAGE_WR			(0x02)		/* Page write command */
#define	FLASH_CMD_READ_ST			(0x05)		/* Read status command */
#define	FLASH_CMD_SECTOR_ERASE	(0xD7)		/* Sector erase command */
#define	FLASH_CMD_READ			(0x03)		/* Read command */

/* ======== EEPROM status ======== */
#define	FLASH_ST_BUSY		(1 << 0)	/* Busy with a write operation */


#define SHTSC_COORDS_ARR_SIZE	4 //2014.10.17 added
/* Software reset delay */
#define SHTSC_RESET_TIME	250	/* msec */
#define SHTSC_SLEEP_TIME	100	/* msec */

#define FLASH_CHECK // experimental code - not checked yet

#ifdef DP_8074_DEMO
#elif defined DP_8084_EVA
// 2015.04.09 added by Y.Nakamura for initialize regulator (pma8084)
#define SHTSC_VTG_MIN_UV       2800000 // not used
#define SHTSC_VTG_MAX_UV       2800000 // not used
#define SHTSC_ACTIVE_LOAD_UA     15000
#define SHTSC_I2C_VTG_MIN_UV       1800000
#define SHTSC_I2C_VTG_MAX_UV       1800000
#define SHTSC_I2C_LOAD_UA        10000
// ---
#endif /* DP_8074_DEMO */

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

/* force flash reprogramming */
#define FORCE_FIRM_UPDATE  // enable if required
#define CGID_CHECK  // enable if required
#ifdef FORCE_FIRM_UPDATE
#include "firm/LR388K5_fFFF0102A_p1028_150601_ext_h.h" // change to your firmware
#ifdef CGID_CHECK
#include "firm/LR388K5_fFFF1102A_p1028_150601_ext_h.h" // change to your firmware
#define CGID_ADDRESS 0xF200
#endif /* CGID_CHECK */
int update_flash(void *, unsigned char *, unsigned int);
#define CMD_GETPROPERTY "\xE0\x00\x00\x11"
#define CMD_GETPROPERTY_LEN 4
#define CMD_SETSYSTEMSTATE_SLEEP "\x02\x00\x01\x00\x00"
#define CMD_SETSYSTEMSTATE_SLEEP_LEN 5
int CheckingFirmwareVersion = 0;
int FlashUpdateByNoFirm = 0;
#endif /* FORCE_FIRM_UPDATE */

/* gesture in suspend */
#define ENABLE_SUSPEND_GESTURE // enable if required
#define FORCE_DISPLAY_ON

#ifdef ENABLE_SUSPEND_GESTURE
#define CMD_SETSYSTEMSTATE_DEEPIDLE "\x02\x00\x01\x00\x04"
#define CMD_SETSYSTEMSTATE_DEEPIDLE_LEN 5
#define CMD_SETSYSTEMSTATE_IDLE "\x02\x00\x01\x00\x03"
#define CMD_SETSYSTEMSTATE_IDLE_LEN 5
#define CMD_GETSYSTEMSTATE "\x03\x00\x00\x01"
#define CMD_GETSYSTEMSTATE_LEN 4
#define CMD_READRAM "\xD3\x00\x04\x20\x14\x68\x00\x00"
#define CMD_READRAM_LEN 8

#if 0
/* user must define these value for their purpose */
#define KEY_GESTURE_DOUBLE_TAP 0x01
#define KEY_GESTURE_SLIDE_UP 0x02
#define	KEY_GESTURE_SLIDE_DOWN 0x03
#define KEY_GESTURE_SLIDE_RIGHT 0x04
#define KEY_GESTURE_SLIDE_LEFT 0x05
#define KEY_GESTURE_CHAR_C 0x63
#define KEY_GESTURE_CHAR_E 0x65
#define KEY_GESTURE_CHAR_M 0x6D
#define KEY_GESTURE_CHAR_O 0x6F
#define KEY_GESTURE_CHAR_V 0x76
#define KEY_GESTURE_CHAR_W 0x77
#define KEY_GESTURE_UNKNOWN 0x7F
#endif /* 0 */

#endif /* ENABLE_SUSPEND_GESTURE */


#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
struct shtsc_i2c *g_ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
struct spi_device *g_spi;
struct shtsc_spi *g_ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)


#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
struct shtsc_i2c {
#ifdef DP_8074_DEMO
#elif defined DP_8084_EVA
	struct regulator *vdd;	// 2015.04.09 added by Y.Nakamura (not used)
	struct regulator *vcc_i2c;	// 2015.04.09 added by Y.Nakamura
#endif /* DP_8074_DEMO */
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
#if defined(CONFIG_FB)
  struct notifier_block fb_notif;
  bool dev_sleep;
#endif
#ifdef FORCE_FIRM_UPDATE
  struct work_struct workstr;
  struct workqueue_struct *workqueue;
#endif /* FORCE_FIRM_UPDATE */

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
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
struct shtsc_spi {
  u8 cmd;//2014.11.19 added
  u8 wait_state;//2014.11.19 added
  bool wait_result;//2014.11.19 added
  struct spi_device	*spi;
  struct shtsc_touch touch[SHTSC_MAX_FINGERS];
  struct input_dev	*input;

  char			phys[32];
  struct mutex		mutex;
  unsigned int		irq;
  int reset_pin;  
  int spiss_pin;  
  spinlock_t		lock;
  struct timer_list	timer;

  unsigned int            max_num_touch;
  int                     min_x;
  int                     min_y;
  int                     max_x;
  int                     max_y;
  bool                    flip_x;
  bool                    flip_y;
  bool                    swap_xy;

  bool			opened;
  bool			suspended;
};
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

#define SHTSC_DEVBUF_SIZE 1500

volatile static int buf_pos;
static u8 devbuf[SHTSC_DEVBUF_SIZE];

static int WriteMultiBytes(void *ts, u8 u8Addr, u8 *data, int len);
static int WriteOneByte(void *ts, u8 u8Addr, u8 u8Val);
static u8 ReadOneByte(void *ts, u8 u8Addr);
static void ReadMultiBytes(void *ts, u8 u8Addr, u16 u16Len, u8 *u8Buf);
static int issue_command(void*, unsigned char *, unsigned int);
int flash_access_start_shtsc(void *_ts);
int flash_access_end_shtsc(void *_ts);
int flash_erase_page_shtsc(void *_ts, int page);
int flash_write_page_shtsc(void *_ts, int page, unsigned char *data);
int flash_verify_page_shtsc(void *_ts, int page, unsigned char *data);
int flash_read(void *_ts, unsigned address, unsigned length, unsigned char *data);
static void shtsc_reset_delay(void);

u8 s_shtsc_addr;
u8 s_shtsc_buf[4*4096];
u8 s_shtsc_i2c_data[4*4096];
unsigned s_shtsc_len;

#define MAX_COMMAND_RESULT_LEN (64-8)
unsigned char CommandResultBuf[MAX_COMMAND_RESULT_LEN];

int	G_reset_done = false;
u16 G_Irq_Mask = 0xffff;
unsigned G_touch=0;

pid_t pid = 0;
unsigned char resumeStatus; // bank 0, address 9


#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
static int shtsc_write_regs(struct shtsc_i2c *tsc, unsigned char addr,
				unsigned char len, unsigned char *value)
{
  struct i2c_client *client = tsc->client;
  int ret;

  s_shtsc_i2c_data[0] = addr;
  memcpy(s_shtsc_i2c_data + 1, value, len);

  ret = i2c_master_send(client, s_shtsc_i2c_data, len + 1);

  return ret;
}

static int shtsc_read_regs(struct shtsc_i2c *tsc,
			       unsigned char *data, unsigned short len, unsigned char addr)
{
  struct i2c_client *client = tsc->client;
  struct i2c_adapter *adap = client->adapter;
  struct i2c_msg msg[2];
  int ret;

  msg[0].addr = client->addr;
  msg[0].flags = client->flags & I2C_M_TEN;
  msg[0].len = 1;
  msg[0].buf = &addr;

  msg[1].addr = client->addr;
  msg[1].flags = client->flags & I2C_M_TEN;
  msg[1].flags |= I2C_M_RD;
  msg[1].len = len;
  msg[1].buf = data;

  ret = i2c_transfer(adap, msg, 2);

  if( ret < 2 ){
    dev_err(&client->dev, "Unable to read i2c bus (adr=%02x)\n",addr);
    goto out;
  }

  return 0;
 out:
  return -1;
}
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
static int shtsc_write_regs(struct shtsc_spi *tsc, u8 u8Addr, int len, u8 *data)
{
  u8 tx_buf[256+3];
  struct spi_message msg;
  struct spi_transfer t = {
    .tx_buf		= &tx_buf[0],
    .len		= (2+len),
    .bits_per_word	= 8,
    .speed_hz           = SHTSC_MAX_SPI_SPEED_IN_HZ,
  };
  int err;

  tx_buf[0] = 0x00;
  tx_buf[1] = u8Addr;
  memcpy(&(tx_buf[2]), data, len);

  spi_message_init(&msg);
  spi_message_add_tail(&t, &msg);
  err = spi_sync(tsc->spi, &msg);
  if (err) {
    dev_err(&tsc->spi->dev, "%s: failed, error: %d\n",
	    __func__, err);
    return err;
  }
  return 0;
}

static int shtsc_read_regs(struct shtsc_spi *tsc, u8 *u8Buf,  u16 u16Len, u8 u8Addr)
{
  u8 tx_buf[128];
  //  u8 tx_buf[128], rx_buf[512+128];
  struct spi_message msg;
  struct spi_transfer t = {
    .tx_buf		= &tx_buf[0],
    //    .rx_buf             = &rx_buf[0],
    .rx_buf             = &u8Buf[0],
    .len		= 2,
    .bits_per_word	= 8,
    .speed_hz           = SHTSC_MAX_SPI_SPEED_IN_HZ,
  };
  int err;
  int i;

  tx_buf[0] = 0x00;
  tx_buf[1] = u8Addr;

  spi_message_init(&msg);
  spi_message_add_tail(&t, &msg);
  err = spi_sync(tsc->spi, &msg);
  if (err) {
    dev_err(&tsc->spi->dev, "%s: failed, error: %d\n",
	    __func__, err);
    goto exitReadMultiBytes;
  }

  tx_buf[0] = 0x80;
  t.len = u16Len + 1;

  spi_message_init(&msg);
  spi_message_add_tail(&t, &msg);
  err = spi_sync(tsc->spi, &msg);
  if (err) {
    dev_err(&tsc->spi->dev, "%s: failed, error: %d\n",
	    __func__, err);
    goto exitReadMultiBytes;    
  }
  for(i = 0;i < u16Len;i++){
    u8Buf[i] = u8Buf[1 + i];
  }
  u8Buf[i] = 0; // not required. just for testing
  return 0;

 exitReadMultiBytes:  
  return err;
}
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

static int WriteMultiBytes(void *_ts, u8 u8Addr, u8 *data, int len)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  return shtsc_write_regs(ts, u8Addr, len, data);
}

static int WriteOneByte(void *_ts, u8 u8Addr, u8 u8Val)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  u8 wData[1];
  wData[0] = u8Val;

  return shtsc_write_regs(ts, u8Addr, 1, wData);
}

static u8 ReadOneByte(void *_ts, u8 u8Addr)
{  
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  u8 rData[1+1]; //requires one more byte to hold
  
  shtsc_read_regs(ts, rData, 1, u8Addr);
  return rData[0];
}

static void ReadMultiBytes(void *_ts, u8 u8Addr, u16 u16Len, u8 *u8Buf)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  shtsc_read_regs(ts, u8Buf, u16Len, u8Addr);
}

static int WaitAsync(void *_ts)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  int i;
  for(i=0;i<50;i++) {
    mdelay(CMD_DELAY);//16ms
    DBGLOG("shtsc: wait 16ms for state change\n");
    switch(ts->wait_state) {
    case WAIT_RESET:
      break;
    case WAIT_CMD:
      break;
    case WAIT_NONE:
      if (ts->wait_result == true) {
	DBGLOG("shtsc: wait state change: success\n");
	return 0;
      }
      else
	return -EIO;
    default:
      break;
    }
  }
  DBGLOG("wait state change: failure\n");
  return -EIO;
}

#ifndef DEBUG_IRQ_DISABLE
static void SetBankAddr(void *_ts, u8 u8Bank)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  WriteOneByte(ts, SHTSC_ADDR_BANK, u8Bank);  
}

static void ClearInterrupt(void *_ts, u16 u16Val)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  if(u16Val & 0x00FF)
    WriteOneByte(ts, SHTSC_ADDR_INT0, (u16Val & 0x00FF));
  if((u16Val & 0xFF00) >> 8)
    WriteOneByte(ts, SHTSC_ADDR_INT1, ((u16Val & 0xFF00) >> 8));
}

static void SetIndicator(void *_ts, u8 u8Val)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  WriteOneByte(ts, SHTSC_ADDR_IND, u8Val);
}
#endif /* DEBUG_IRQ_DISABLE */

static int shtsc_system_init(void *ts)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct input_dev *input_dev = ((struct shtsc_i2c *)ts)->input;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct input_dev *input_dev = ((struct shtsc_spi *)ts)->input;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  /* PLL unlock - not used*/
  //  WriteOneByte(ts, SHTSC_ADDR_INTMASK1, (unsigned char)0x02);
  //  WriteOneByte(ts, (unsigned char)0x04, (unsigned char)0x02);
#if 0
  int i;
  u8 txbuf[8];
  u8 rxbuf[8];
  for(i=0;i<10;i++) {
    txbuf[0] = 0x02;
    txbuf[1] = i;
    i2c_master_send(((struct shtsc_i2c *)ts)->client, txbuf,2);
    i2c_master_send(((struct shtsc_i2c *)ts)->client, txbuf,1);
    i2c_master_recv(((struct shtsc_i2c *)ts)->client, rxbuf,1);
    printk(KERN_INFO "i2c master: tx=0x%x, rx=0x%x\n",i,rxbuf[0]);
  }
#endif
  {
    unsigned int cnt;
    for(cnt=0;cnt<SHTSC_MAX_FINGERS;cnt++){
      input_mt_slot(input_dev, cnt);
      input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc_system_init() clear mt_slot(%d)\n",cnt);
#endif
    }
  }
  input_report_key(input_dev, BTN_TOUCH, false );

#ifdef SHTSC_ICON_KEY_NUM
  {
    unsigned int cnt;
    for(cnt=0;cnt<SHTSC_ICON_KEY_NUM;cnt++){
      input_report_key( input_dev, shtsc_keyarray[cnt] , false );
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc_system_init() clear report_key(%d)\n",cnt);
#endif
    }
  }
#endif
  input_sync(input_dev);
  return 0;
}


#ifndef DEBUG_IRQ_DISABLE
static void GetTouchReport(void *ts, u8 u8Num, u8 *u8Buf)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_touch *touch = ((struct shtsc_i2c *)ts)->touch;
  struct input_dev *input_dev = ((struct shtsc_i2c *)ts)->input;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_touch *touch = ((struct shtsc_spi *)ts)->touch;
  struct input_dev *input_dev = ((struct shtsc_spi *)ts)->input;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  int i;
  u8 ID,Status;
  int touchNum = 0;//2014.11.12 added

  if( u8Num > SHTSC_MAX_TOUCH_1PAGE ){
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc touch number erro (num=%d)\n",u8Num);
#endif
    return;
  }

    for(i = 0;i < u8Num;i++){
      Status = (u8Buf[i * SHTSC_LENGTH_OF_TOUCH] & 0xF0);
      touch[i].id = ID = u8Buf[i * SHTSC_LENGTH_OF_TOUCH] & 0x0F;    
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

#if 0 // no hover?
      if (!touch[i].z) {//14.11.28 added
	touch[i].z = 1;
      }
#endif /* 0 */

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc ID=%2d, Status=%02x, Size=%3d, X=%5d, Y=%5d, z=%5d, num=%2d\n"
	     ,ID
	     ,Status
	     ,touch[i].size
	     ,touch[i].x
	     ,touch[i].y
	     ,touch[i].z
	     ,u8Num);
#endif

      input_mt_slot(input_dev, ID);
      if(Status & SHTSC_TOUCHOUT_STATUS){
	touch[i].status = SHTSC_F_TOUCH_OUT;
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false); // the 2nd parameter is DON'T CARE
	continue;
      }

      if(((struct shtsc_i2c *)ts)->swap_xy){//2014.11.12 added
	int tmp;
	tmp = touch[i].x;
	touch[i].x = touch[i].y;
	touch[i].y = tmp;
      }
      if(((struct shtsc_i2c *)ts)->flip_x){
	touch[i].x = (((struct shtsc_i2c *)ts)->max_x - touch[i].x) < 0 ? 0 : ((struct shtsc_i2c *)ts)->max_x - touch[i].x;
      }
      if(((struct shtsc_i2c *)ts)->flip_y){
	touch[i].y = (((struct shtsc_i2c *)ts)->max_y - touch[i].y) < 0 ? 0 : ((struct shtsc_i2c *)ts)->max_y - touch[i].y;
      }
      touchNum++;

      touch[i].status = SHTSC_F_TOUCH;
      input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);

      input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, touch[i].size);
      input_report_abs(input_dev, ABS_MT_POSITION_X, touch[i].x);
      input_report_abs(input_dev, ABS_MT_POSITION_Y, touch[i].y);
      input_report_abs(input_dev, ABS_MT_PRESSURE  , touch[i].z);
    }
  input_report_key(input_dev, BTN_TOUCH, touchNum?true:false );//2014.11.12 added
}
#endif /* DEBUG_IRQ_DISABLE */


#ifdef FORCE_FIRM_UPDATE
void update_flash_func(struct work_struct *work)
{
  printk(KERN_INFO "shtsc: update_flash_func\n");

  if (FlashUpdateByNoFirm) {
    // no firmware found in flash
#ifdef CGID_CHECK
	unsigned char *firmverbin,*paramverbin,*firmbin;
	unsigned int firmlen;
	unsigned char cgid,irqmask[2];
	CheckingFirmwareVersion = 2; /* no more update */
	flash_access_start_shtsc(g_ts);
	irqmask[0] = ReadOneByte(g_ts,(unsigned char)SHTSC_ADDR_INTMASK0);
	irqmask[1] = ReadOneByte(g_ts,(unsigned char)SHTSC_ADDR_INTMASK1);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
	flash_read(g_ts,CGID_ADDRESS,1,&cgid);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, irqmask[0]);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, irqmask[1]);
	if( cgid == CG_VENDOR_ID1 ){
	  firmverbin = (unsigned char *)FIRM_VERSION;
	  paramverbin = (unsigned char *)PARAM_VERSION;
	  firmbin = (unsigned char *)FIRM_IMAGE;
	  firmlen = FIRM_SIZE;
	  printk(KERN_INFO "shtsc: CG Vendor ID : INDEX=1\n");
	}
	else if( cgid == CG_VENDOR_ID2 ){
	  firmverbin = (unsigned char *)FIRM_VERSION2;
	  paramverbin = (unsigned char *)PARAM_VERSION2;
	  firmbin = (unsigned char *)FIRM_IMAGE2;
	  firmlen = FIRM_SIZE2;
	  printk(KERN_INFO "shtsc: CG Vendor ID : INDEX=2\n");
	}
        else{
	  firmverbin = NULL;
	  paramverbin = NULL;
	  firmbin = NULL;
	  firmlen = 0;
	  printk(KERN_INFO "shtsc: CG Vendor ID : INDEX=NONE\n");
	  printk(KERN_INFO "shtsc: no firmware selected\n");
	}
	printk(KERN_INFO "shtsc: CG Vendor ID : %02x\n",cgid);
#else /* CGID_CHECK */
	unsigned char *firmverbin,*paramverbin,*firmbin;
	unsigned int firmlen;
	CheckingFirmwareVersion = 2; /* no more update */
	firmverbin = (unsigned char *)FIRM_VERSION;
	paramverbin = (unsigned char *)PARAM_VERSION;
	firmbin = (unsigned char *)FIRM_IMAGE;
	firmlen = FIRM_SIZE;
#endif /* CGID_CHECK */
	FlashUpdateByNoFirm = 0;
    
	/* force flash reprogramming */
	if ( firmverbin ) {
	  printk(KERN_INFO "shtsc: force updating flash by no valid firmware\n");
	  update_flash(g_ts, firmbin, firmlen);
	  printk(KERN_INFO "shtsc: force updating flash by no valid firmware.... done\n");
	} else {
	  printk(KERN_INFO "shtsc: no valid CGID found. so write firmware for INDEX 1\n");
	  firmbin = (unsigned char *)FIRM_IMAGE;
	  firmlen = FIRM_SIZE;
	  printk(KERN_INFO "shtsc: force CG Vendor ID : INDEX=1\n");
	  update_flash(g_ts, firmbin, firmlen);
	  printk(KERN_INFO "shtsc: no valid CGID found. so write firmware for INDEX 1.... done\n");
	}

  } else {
      if (CheckingFirmwareVersion == 0) {
	CheckingFirmwareVersion = 1;

	printk(KERN_INFO "shtsc: issue_command in workqueue\n");

	/* issue GetProperty command to read out the firmware/parameter version */
	issue_command(g_ts, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	printk(KERN_INFO "shtsc: issue_command in workqueue... done\n");
      }

      if (CheckingFirmwareVersion == 1) {
#ifdef CGID_CHECK
	unsigned char *firmverbin,*paramverbin,*firmbin;
	unsigned int firmlen;
	unsigned char cgid,irqmask[2];
	CheckingFirmwareVersion = 2; /* no more update */
	flash_access_start_shtsc(g_ts);
	irqmask[0] = ReadOneByte(g_ts,(unsigned char)SHTSC_ADDR_INTMASK0);
	irqmask[1] = ReadOneByte(g_ts,(unsigned char)SHTSC_ADDR_INTMASK1);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
	flash_read(g_ts,CGID_ADDRESS,1,&cgid);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, irqmask[0]);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, irqmask[1]);
	if( cgid == CG_VENDOR_ID1 ){
	  firmverbin = (unsigned char *)FIRM_VERSION;
	  paramverbin = (unsigned char *)PARAM_VERSION;
	  firmbin = (unsigned char *)FIRM_IMAGE;
	  firmlen = FIRM_SIZE;
	  printk(KERN_INFO "shtsc: CG Vendor ID : INDEX=1\n");
	}
	else if( cgid == CG_VENDOR_ID2 ){
	  firmverbin = (unsigned char *)FIRM_VERSION2;
	  paramverbin = (unsigned char *)PARAM_VERSION2;
	  firmbin = (unsigned char *)FIRM_IMAGE2;
	  firmlen = FIRM_SIZE2;
	  printk(KERN_INFO "shtsc: CG Vendor ID : INDEX=2\n");
	}
        else{
	  firmverbin = NULL;
	  paramverbin = NULL;
	  firmbin = NULL;
	  firmlen = 0;
	  printk(KERN_INFO "shtsc: CG Vendor ID : INDEX=NONE\n");
	}
	printk(KERN_INFO "shtsc: CG Vendor ID : %02x\n",cgid);
#else /* CGID_CHECK */
	unsigned char *firmverbin,*paramverbin,*firmbin;
	unsigned int firmlen;
	CheckingFirmwareVersion = 2; /* no more update */
	firmverbin = (unsigned char *)FIRM_VERSION;
	paramverbin = (unsigned char *)PARAM_VERSION;
	firmbin = (unsigned char *)FIRM_IMAGE;
	firmlen = FIRM_SIZE;
#endif /* CGID_CHECK */
	if ( firmverbin && 
	    ( memcmp(firmverbin, (unsigned char *)&(CommandResultBuf[0x0a-0x08]),4)
	    || memcmp(paramverbin, (unsigned char *)&(CommandResultBuf[0x12-0x08]),4))) {

	  issue_command(g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);
	  printk(KERN_INFO "shtsc: now K5 becomes sleep state.\n");

	  /* the written firmware is not intended version - so overwrite */
	  printk(KERN_INFO "shtsc: Update firmware version: f%02X%02X%02X%02X p%02X%02X%02X%02X to f%02X%02X%02X%02X p%02X%02X%02X%02X\n", 
		 CommandResultBuf[0x0D-0x08], // current firm version
		 CommandResultBuf[0x0C-0x08],
		 CommandResultBuf[0x0B-0x08],
		 CommandResultBuf[0x0A-0x08],
		 CommandResultBuf[0x15-0x08], // current param version
		 CommandResultBuf[0x14-0x08],
		 CommandResultBuf[0x13-0x08],
		 CommandResultBuf[0x12-0x08],
		 firmverbin[3], // target firm version
		 firmverbin[2],
		 firmverbin[1],
		 firmverbin[0],
		 paramverbin[3], // target param version
		 paramverbin[2],
		 paramverbin[1],
		 paramverbin[0]);

	  update_flash(g_ts, firmbin, firmlen);
	} else {
	  printk(KERN_INFO "shtsc: Firmware version update NOT required. Current version: f%02X%02X%02X%02X p%02X%02X%02X%02X\n", 
		 CommandResultBuf[0x0D-0x08], // firm version
		 CommandResultBuf[0x0C-0x08],
		 CommandResultBuf[0x0B-0x08],
		 CommandResultBuf[0x0A-0x08],
		 CommandResultBuf[0x15-0x08], // param version
		 CommandResultBuf[0x14-0x08],
		 CommandResultBuf[0x13-0x08],
		 CommandResultBuf[0x12-0x08]);
	}

      }

  }
  return;
}
#endif /* FORCE_FIRM_UPDATE */


//#define GET_REPORT  // experimental

#ifdef GET_REPORT
#include <linux/time.h>

#define REP_SIZE (4+4+4+120)
#define MAX_REPORTS 14

unsigned char reportBuf[4+ MAX_REPORTS*REP_SIZE];
volatile unsigned char Mutex_GetReport = false;

#endif /* GET_REPORT */

u8 dcmapBuf[MAX_DCMAP_SIZE+128];
u8 dcmap[31*18*2+128]; // greater than 17*32*2

#ifndef DEBUG_IRQ_DISABLE
static irqreturn_t shtsc_irq_thread(int irq, void *_ts)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

  u16 u16status;
  u8 u8Num = 0;
  u8 tmpbuf[128];
  u8 numDriveLine2, numSenseLine2;
  u8 num_adc_dmy[3];
  u8 regcommonbuf[11];
#if 0
  {
    struct sched_param param;
    int ret;
    param.sched_priority = 64;
    ret = sched_setscheduler_nocheck(current, SCHED_FIFO, &param);
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc sched_setscheduler_nocheck %d\n", ret);
#endif
  }
#endif


#if defined(DEBUG_SHTSC)
  if (G_touch) {
    printk(KERN_INFO "[IRQ] shtsc touch-ready %d\n", G_touch);
  }
#endif

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[ENTER] shtsc_irq\n");
#endif

  if (! G_reset_done) {
    // On POR. not intended interrupt by RESET pin
    // mask everything
    WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
    WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
    // and clear them
    WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INT0, (unsigned char)0xff);
    WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INT1, (unsigned char)0x03);
    return IRQ_HANDLED;
  }

  /* Get Interrupt State */
  ReadMultiBytes(ts,SHTSC_ADDR_INT0,11,regcommonbuf);
  u16status = ((regcommonbuf[SHTSC_ADDR_INT1] << 8) | regcommonbuf[SHTSC_ADDR_INT0]);
  u16status &= G_Irq_Mask;
#if defined(DEBUG_SHTSC)
  if ((u16status != 0x0001) && (u16status != 0x0000)) {
    printk(KERN_CRIT "[IRQ] shtsc_irq: %04X\n", u16status);
  }
#endif
	
  while(u16status != 0){
    if (u16status & SHTSC_STATUS_PLL_UNLOCK) {
      //
      // PLL unlock
      //

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc_irq PLL_UNLOCK: %04X\n", u16status);
#endif
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc pll-unlock\n");
#endif
      // mask it
      WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
      regcommonbuf[SHTSC_ADDR_INTMASK1] = 0x03;
      // from now on, no int for this
      G_Irq_Mask &= ~SHTSC_STATUS_PLL_UNLOCK;

      /* Clear Interrupt */
      ClearInterrupt(ts, SHTSC_STATUS_PLL_UNLOCK);
      u16status &= ~SHTSC_STATUS_PLL_UNLOCK;

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc_irq PLL_UNLOCK: %04X\n", u16status);
#endif
    }

    if (u16status & SHTSC_STATUS_POWER_UP) {
      //
      // Power-up
      //

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc power-up\n");
#endif

      /* Clear Interrupt */
      ClearInterrupt(ts, SHTSC_STATUS_POWER_UP);
      u16status &= ~SHTSC_STATUS_POWER_UP;

      if (ts->wait_state == WAIT_RESET) {//2014.11.20 added
	ts->wait_state = WAIT_NONE;
	ts->wait_result = true;
      } 
#ifdef FORCE_FIRM_UPDATE
      printk(KERN_INFO "shtsc: calling firmware update workqueue\n");
      queue_work(ts->workqueue, &ts->workstr);
      //      flush_workqueue(ts->workqueue);
#endif /* FORCE_FIRM_UPDATE */
   }

    if (u16status & SHTSC_STATUS_WDT) {
      ClearInterrupt(ts, SHTSC_STATUS_WDT);
      u16status &= ~SHTSC_STATUS_WDT;
      ERRLOG("[IRQ] shtsc_irq WDT");
    }

    if (u16status & SHTSC_STATUS_KNOCK_CODE) {
      ClearInterrupt(ts, SHTSC_STATUS_KNOCK_CODE);
      u16status &= ~SHTSC_STATUS_KNOCK_CODE;
      ERRLOG("[IRQ] shtsc_irq KNOCK_CODE");
    }

    if (u16status & SHTSC_STATUS_RESUME_PROX) {
      //
      // Resume from DeepIdle
      // or
      // PROXIMITY
      //
      
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc resume from DeepIdle or prox\n");
#endif

      SetBankAddr(ts, SHTSC_BANK_TOUCH_REPORT);
      resumeStatus = ReadOneByte(ts, SHTSC_ADDR_RESUME_PROX);

#ifdef ENABLE_SUSPEND_GESTURE
      DBGLOG("shtsc: entering gesture interrupt\n");

      // input_report_key( ts->input, code , true ); to kernel
      {
	int key_gesture_code = 0x00;
#if 0
	switch (resumeStatus) {
	case 0x01:
	  key_gesture_code = KEY_GESTURE_DOUBLE_TAP;
	  break;
	case 0x02:
	  key_gesture_code = KEY_GESTURE_SLIDE_UP;
	  break;
	case 0x03:
	  key_gesture_code = KEY_GESTURE_SLIDE_DOWN;
	  break;
	case 0x04:
	  key_gesture_code = KEY_GESTURE_SLIDE_RIGHT;
	  break;
	case 0x05:
	  key_gesture_code = KEY_GESTURE_SLIDE_LEFT;
	  break;
	case 0x63:
	  key_gesture_code = KEY_GESTURE_CHAR_C;
	  break;
	case 0x65:
	  key_gesture_code = KEY_GESTURE_CHAR_E;
	  break;
	case 0x6D:
	  key_gesture_code = KEY_GESTURE_CHAR_M;
	  break;
	case 0x6F:
	  key_gesture_code = KEY_GESTURE_CHAR_O;
	  break;
	case 0x76:
	  key_gesture_code = KEY_GESTURE_CHAR_V;
	  break;
	case 0x77:
	  key_gesture_code = KEY_GESTURE_CHAR_W;
	  break;
	default:
	  key_gesture_code = KEY_GESTURE_UNKNOWN;
	  break;
	}
#else
	/* you need to change this for your environment */
	key_gesture_code = resumeStatus;
#endif /* 0 */

	if (key_gesture_code) {
	  input_report_key( ts->input, key_gesture_code, true );
	  input_sync(ts->input);
	  input_report_key( ts->input, key_gesture_code, false );
	  input_sync(ts->input);
	}
#if defined(DEBUG_SHTSC)
	printk(KERN_INFO "[IRQ] shtsc resumeStatus %02X\n", resumeStatus);
#endif
      }
#else /* ENABLE_SUSPEND_GESTURE */
      // throw interrupt to userland
    {
      struct siginfo info;
      struct task_struct *task;
      
      memset(&info, 0, sizeof(struct siginfo));
      info.si_signo = SHTSC_SIGNAL;
      info.si_code = SI_QUEUE;
      info.si_int = 0;

      rcu_read_lock();
      task = find_task_by_vpid(pid);
      rcu_read_unlock();

      if (task) {
	send_sig_info(SHTSC_SIGNAL, &info, task); //send signal to user land
	printk(KERN_INFO "[SIGNAL] sending shtsc signal (resume_prox)\n");
      }
    }
#endif /* ENABLE_SUSPEND_GESTURE */
 
      /* Clear Interrupt */
      ClearInterrupt(ts, SHTSC_STATUS_RESUME_PROX);
      u16status &= ~SHTSC_STATUS_RESUME_PROX;

#ifdef ENABLE_SUSPEND_GESTURE

#ifdef FORCE_DISPLAY_ON
	/* set display on without any control of the system */
	/* the system must control the display without this */
	input_report_key( ts->input, KEY_POWER, true );
	input_sync(ts->input);
	input_report_key( ts->input, KEY_POWER, false );
	input_sync(ts->input);
	msleep(1000); /* wait HSYNC becomes stable */
#endif /* FORCE_DISPLAY_ON */

	//      DBGLOG("shtsc: issue IDLE command\n");
	//      msleep(SHTSC_GESTURE_WAIT_TIME);

	//      issue_command(ts, CMD_GETSYSTEMSTATE, CMD_GETSYSTEMSTATE_LEN); //debug
	//      msleep(300); //debug

	//	issue_command(ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);
	//	msleep(300); //debug

	//      issue_command(ts, CMD_GETSYSTEMSTATE, CMD_GETSYSTEMSTATE_LEN); //debug
	//      msleep(300); //debug


#endif /* ENABLE_SUSPEND_GESTURE */
    }

    if (u16status & SHTSC_STATUS_FLASH_LOAD_ERROR) {
      //
      // FLASH_LOAD_ERROR
      // occurs when flash is erased
      // nothing can be done
      //
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc flash load error\n");
#endif
      if (ts->wait_state == WAIT_RESET) {//2014.11.20 added
	ts->wait_state = WAIT_NONE;
	ts->wait_result = true;
      } 
      // from now on, no int for this
      G_Irq_Mask &= ~SHTSC_STATUS_FLASH_LOAD_ERROR;

      /* Clear Interrupt */
      ClearInterrupt(ts, SHTSC_STATUS_FLASH_LOAD_ERROR);
      u16status &= ~SHTSC_STATUS_FLASH_LOAD_ERROR;

#ifdef FORCE_FIRM_UPDATE
      printk(KERN_INFO "shtsc: calling firmware update workqueue\n");
      FlashUpdateByNoFirm = 1;
      queue_work(ts->workqueue, &ts->workstr);
      //      flush_workqueue(ts->workqueue);
#endif /* FORCE_FIRM_UPDATE */
    }

    if (u16status & SHTSC_STATUS_TOUCH_READY) {
      //
      // Touch report
      //

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc touch-ready cleared\n");
#endif

#ifdef GET_REPORT
      {
	/*
	  header:
	  1: counts of report (not a number of touch)
	  3: reserved

	  repeated contents if "counts of report"  is more than one.
	  1: cyclic counter (from 0 to 255. return to zero on next to 255)
	  3: reserved
	  4: report time (sec)
	  4: report time (usec)
	  120: bank0 report
	*/

	static unsigned char cyclic = 255;
	volatile unsigned char count;
	unsigned char bank0[120];
	struct timeval tv;

	while (Mutex_GetReport)
	  ;

	Mutex_GetReport = true;

	count = reportBuf[0];

	if (cyclic == 255) {
	  cyclic = 0;
	} else {
	  cyclic++;
	}
	
#if defined(DEBUG_SHTSC)
	//	printk(KERN_INFO "touch report: count %d, cyclic %d\n", count, cyclic);
#endif
	if (count == MAX_REPORTS) {
	  //	  printk(KERN_INFO "touch report buffer full\n");
	  ;
	} else {	
	  do_gettimeofday(&tv);
#if defined(DEBUG_SHTSC)
	  //	  printk(KERN_INFO "touch time: %ld.%ld, bank0pos:%d\n", (long)tv.tv_sec, (long)tv.tv_usec, (1+3+ count*REP_SIZE + 12));
#endif

	  reportBuf[1+3+ count*REP_SIZE +0] = cyclic;
	  reportBuf[1+3+ count*REP_SIZE +1] = 0;
	  reportBuf[1+3+ count*REP_SIZE +2] = 0;
	  reportBuf[1+3+ count*REP_SIZE +3] = 0;

	  reportBuf[1+3+ count*REP_SIZE +4+0] = (tv.tv_sec & 0xff);
	  reportBuf[1+3+ count*REP_SIZE +4+1] = ((tv.tv_sec >> 8) & 0xff);
	  reportBuf[1+3+ count*REP_SIZE +4+2] = ((tv.tv_sec >> 16) & 0xff);
	  reportBuf[1+3+ count*REP_SIZE +4+3] = ((tv.tv_sec >> 24) & 0xff);

	  reportBuf[1+3+ count*REP_SIZE +4+4+0] = (tv.tv_usec & 0xff);
	  reportBuf[1+3+ count*REP_SIZE +4+4+1] = ((tv.tv_usec >> 8) & 0xff);
	  reportBuf[1+3+ count*REP_SIZE +4+4+2] = ((tv.tv_usec >> 16) & 0xff);
	  reportBuf[1+3+ count*REP_SIZE +4+4+3] = ((tv.tv_usec >> 24) & 0xff);
	
	  SetBankAddr(ts, SHTSC_BANK_TOUCH_REPORT);
	  ReadMultiBytes(ts, 0x08, 120, bank0);

	  memcpy((unsigned char *)(&reportBuf[1+3+ count*REP_SIZE + 12]), (unsigned char *)bank0, 120);
	  reportBuf[0] = (unsigned char)(count+1);
	}

	Mutex_GetReport = false;
      }
#endif /* GET_REPORT */

      /* Get number of touches */
      {
	u8 u8Buf[128];
	if( regcommonbuf[SHTSC_ADDR_BANK] != SHTSC_BANK_TOUCH_REPORT ){
	  WriteOneByte(ts, SHTSC_ADDR_BANK, SHTSC_BANK_TOUCH_REPORT);
	  ReadMultiBytes(ts,SHTSC_ADDR_TOUCH_NUM,3,regcommonbuf+SHTSC_ADDR_TOUCH_NUM);
	}
	u8Num = regcommonbuf[SHTSC_ADDR_TOUCH_NUM];

#if defined(DEBUG_SHTSC)
	printk(KERN_INFO "shtsc touch num=%d\n", u8Num);
#endif

#ifdef SHTSC_ICON_KEY_NUM
	{
	  unsigned int buttonBit,cnt;
	  buttonBit = ReadOneByte(ts, 0x0A);
#if defined(DEBUG_SHTSC)
	  printk(KERN_INFO "shtsc(k5) icon key bitfield = %02x\n", buttonBit );
#endif
	  for(cnt=0;cnt<SHTSC_ICON_KEY_NUM;cnt++){
	    if(buttonBit&(0x01<<cnt)){
	      input_report_key( ts->input, shtsc_keyarray[cnt] , true );
	    }
	    else{
	      input_report_key( ts->input, shtsc_keyarray[cnt] , false );
	    }
	  }
	}
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
	  GetTouchReport(ts, u8Num, u8Buf);
	}

	input_sync(ts->input);
      }
    }

    if (u16status & SHTSC_STATUS_COMMAND_RESULT) {
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc command result\n");
#endif
      SetBankAddr(ts,SHTSC_BANK_COMMAND_RESULT);
      ReadMultiBytes(ts, 0x08, MAX_COMMAND_RESULT_LEN, CommandResultBuf); // always read entire result. TEGAPP will handle it.

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc command result, %02X, %02X, %02X\n", CommandResultBuf[0], CommandResultBuf[1], CommandResultBuf[2]);
#endif

#if 0
      {
	int i;
	printk(KERN_INFO "[IRQ] shtsc command result (long) ");
	for (i = 0; i < 34; i++) {
	  printk(KERN_INFO "%02X ", CommandResultBuf[i]);
	}
      }
#endif // debug

      /* Clear Interrupt */
      ClearInterrupt(ts, SHTSC_STATUS_COMMAND_RESULT);
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

    if (u16status & SHTSC_STATUS_DCMAP_READY) {
#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[IRQ] shtsc DCMAP READY\n");
#endif
      /* Clear Interrupt */
      ClearInterrupt(ts, SHTSC_STATUS_DCMAP_READY);
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

	//	      printk(KERN_INFO "%s(%d): num_adc_dmy[1]:%d\n", __FILE__, __LINE__, num_adc_dmy[1]);
	//	      printk(KERN_INFO "%s(%d):Sense:%d, Drive:%d\n", __FILE__, __LINE__, numSenseLine2, numDriveLine2);
	//	      printk(KERN_INFO "%s(%d): dsFlag:%d\n", __FILE__, __LINE__, dsFlag);

	// read DCmap values from register
	// store it to read buffer memory for read action
	{ // L1
	  /* read 120 bytes from Bank5, address 8 */
	  /* read loop required */
	  int bytes = readingSize;
	  int size;
	  int index = 0;
	  //SetBankAddr(ts,SHTSC_BANK_DCMAP);

	  //	      printk(KERN_INFO "%s(%d):readingSize:%d\n", __FILE__, __LINE__, readingSize);
	  while (bytes > 0) {
	    ram_addr[0] = (unsigned char)(vramAddr&0xff); // address to read (Lower)
	    ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // address to read (Higher)
	    WriteMultiBytes(ts,(unsigned char)0x06,ram_addr,2);

	    size = ((bytes >= 120) ? 120 : bytes);
	    //		printk(KERN_INFO "%s(%d):bytes:%d, size:%d, index:%d, vramAddr:%x\n", __FILE__, __LINE__, bytes, size, index, vramAddr);

	    ReadMultiBytes(ts, 0x08, size, &(dcmapBuf[index]));
	    index += size;
	    bytes -= size;
	    vramAddr += size;
	  } // while
	} // L1

#if 0
	printk(KERN_INFO "DCmap Drive x Sense: %d x %d, dsFlag:%02X, num_adc_dmy:%02X %02X %02X\n", numDriveLine2, numSenseLine2, dsFlag, num_adc_dmy[0], num_adc_dmy[1], num_adc_dmy[2]);
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
	  //	      printk(KERN_INFO "check buf_pos: %d\n", buf_pos);
	  if (buf_pos == 0) {
	    memcpy((u8 *)devbuf, (u8 *)dcmap, (4+x*y*2));
	    //		printk(KERN_INFO "setting buf_pos: %d\n", buf_pos);
	    buf_pos = (4+x*y*2);
	    //		printk(KERN_INFO "set buf_pos: %d\n", buf_pos);
	  }

#if 0 // DCmap debug
	  printk(KERN_INFO "DC map size HxV: %d x %d = %d\n", dcmap[0], dcmap[1], dcmap[0]*dcmap[1]);
	  printk(KERN_INFO "[0-3, %d-%d]: %02X %02X %02X %02X, %02X %02X %02X %02X\n", (x*y-4), (x*y-1), dcmap[4], dcmap[5], dcmap[6], dcmap[7], dcmap[x*y*2+4-4], dcmap[x*y*2+4-3], dcmap[x*y*2+4-2], dcmap[x*y*2+4-1]);
#endif /* 0 */
#if 0 // DCmap debug
	  for (j = 0; j < y; j++) {
	    for (i = 0; i < x; i++) {
	      printk(KERN_INFO "%d: %02X ", (y*j+i), dcmap[y*j + i + 4]);
	    }
	    printk(KERN_INFO "\n");
	  }
#endif
	} //L3
      } // L2 DEVICE_LR388K5 block
      //*************************
    }
    if (u16status != 0) {
      printk(KERN_INFO "[IRQ] shtsc unknown interrupt status %04X\n", u16status);
      /* Clear all interrupts and mask.  */
      WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xFF);
      WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
      u16status = 0;
      ClearInterrupt(ts, u16status);
    }
  }

  if (u8Num != 0 ) {
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc flush touch input(%d)\n", u8Num);
#endif
    /*
      input_report_key(ts->input, BTN_TOUCH, touchNum?true:false );
		
      input_sync(ts->input);
    */
  }

  return IRQ_HANDLED;
}
#endif /* DEBUG_IRQ_DISABLE */

static ssize_t 
dev_read( struct file* filp, char* buf, size_t count, loff_t* pos )
{
  int copy_len;
  int i;

  //	printk( KERN_INFO "shtsc : read()  called, buf_pos: %d, count: %d\n", buf_pos, count);
  if ( count > buf_pos )
    copy_len = buf_pos;
  else
    copy_len = count;

  //	printk( KERN_INFO "shtsc : copy_len = %d\n", copy_len );
  if ( copy_to_user( buf, devbuf, copy_len ) ) {
    printk( KERN_INFO "shtsc : copy_to_user failed\n" );
    return -EFAULT;
  }

  *pos += copy_len;

  for ( i = copy_len; i < buf_pos; i ++ )
    devbuf[ i - copy_len ] = devbuf[i];

  buf_pos -= copy_len;

  //	printk( KERN_INFO "shtsc : buf_pos = %d\n", buf_pos );
  return copy_len;
}

static void shtsc_reset(void *_ts, bool reset)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

    if (ts->reset_pin) {
      G_reset_done = reset;
      if (reset) {
	shtsc_reset_delay();
      }
      printk(KERN_INFO "shtsc: shtsc_reset: %d\n", reset);
      gpio_direction_output(ts->reset_pin, reset);
      if (! reset) {
	G_Irq_Mask = 0xffff;
      }
    }
}

//#define USE_RESET_L_H
#ifdef USE_RESET_L_H
static void shtsc_reset_L_H(void *ts, bool reset)
{
  shtsc_reset(ts, false);
  shtsc_reset(ts, true);
}
#endif /* USE_RESET_L_H */


int flash_access_start_shtsc(void *_ts)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

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
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

    msleep(100);

  shtsc_reset(ts, false);
  msleep(100);
  shtsc_reset(ts, true);
  msleep(10);
  shtsc_system_init(ts);

  msleep(100);

  return 0;
}

#define RETRY_COUNT (2000*10) //experimental
int flash_erase_page_shtsc(void *_ts, int page)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

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
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
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
  printk(KERN_INFO "FATAL: flash_erase_page_shtsc retry %d times for page %d - FAILED!\n", retry, page);
  return 1;
}
#define FLASH_PAGE_SIZE (4<<10) // 4k block for each page
#define FLASH_PHYSICAL_PAGE_SIZE (256) // can write 256bytes at a time.

int flash_write_page_shtsc(void *_ts, int page, unsigned char *data)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

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
  printk(KERN_INFO "FATAL: flash_write_page_shtsc retry %d times for page %d, addr %04X - FAILED!\n", retry, page, paddr);
  return 1;
}

#define FLASH_VERIFY_SIZE 512
unsigned char readBuf[FLASH_VERIFY_SIZE];

int flash_verify_page_shtsc(void *_ts, int page, unsigned char *data)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
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
  WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
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
  printk(KERN_INFO "FATAL: flash_verify_page_shtsc for page %d - FAILED!\n", page);

  return 1;
}

int flash_read(void *_ts, unsigned address, unsigned length, unsigned char *data)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
  struct shtsc_i2c *ts = _ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
  struct shtsc_spi *ts = _ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
  error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
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

#ifdef FORCE_FIRM_UPDATE
#define FLASH_WAIT 500
/*
 * force re-programming the firm image held in the driver
 */
int update_flash(void *_ts, unsigned char *data, unsigned int len)
{
  int page;

  printk(KERN_INFO "shtsc: force updating K5 firmware....\n");
  printk(KERN_INFO "shtsc: flash_access start\n");
  flash_access_start_shtsc(_ts);

  for (page = 0; page < (len/FLASH_PAGE_SIZE); page++) {
    msleep(FLASH_WAIT);
    flash_erase_page_shtsc(_ts, page);
    printk(KERN_INFO "shtsc: flash_erase_page_shtsc done: page %d\n",  page);
    msleep(FLASH_WAIT);
    flash_write_page_shtsc(_ts, page, &(data[page*FLASH_PAGE_SIZE]));
    printk(KERN_INFO "shtsc: flash_write_page_shtsc done: page %d\n",  page);
    msleep(FLASH_WAIT);
    flash_verify_page_shtsc(_ts, page, &(data[page*FLASH_PAGE_SIZE]));
    printk(KERN_INFO "shtsc: flash_verify_page_shtsc done: page %d\n",  page);
  }

  printk(KERN_INFO "shtsc: flash_access end\n");
  flash_access_end_shtsc(_ts);

  printk(KERN_INFO "shtsc: force updating K5 firmware....done\n");

  return 0;
}
#endif /* FORCE_FIRM_UPDATE */

int issue_command(void *ts, unsigned char *cmd, unsigned int len)
{
    int err = 0;

    SetBankAddr(ts, SHTSC_BANK_COMMAND);
    DBGLOG("bank command\n");
    // set command
    WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, cmd, len);
    DBGLOG("set command (%x)\n",cmd[0]);

    // prepare waiting
    ((struct shtsc_i2c *)ts)->cmd = cmd[0];
    ((struct shtsc_i2c *)ts)->wait_state = WAIT_CMD;
    ((struct shtsc_i2c *)ts)->wait_result = true;

    // do it
    SetIndicator(ts, SHTSC_IND_CMD);
    DBGLOG("do it\n");
    
    // wait
    err = WaitAsync(ts);

    return err;
}

#define USE_SETSYSTEMSTATE_CMD

#ifdef USE_SETSYSTEMSTATE_CMD
#define CMD_SETSYSTEMSTATE_LEN 5
#define CMD_SETSYSTEMSTATE "\x02\x00\x01\x00\xff"
int shtsc_CMD_SetSystemState(void *ts, dCmdState_e eState)
{
  char cmdArray[CMD_SETSYSTEMSTATE_LEN];
  int ret;

  memcpy(cmdArray, CMD_SETSYSTEMSTATE, CMD_SETSYSTEMSTATE_LEN);
  cmdArray[4] = eState;
  ret =  issue_command(ts, cmdArray, CMD_SETSYSTEMSTATE_LEN);
  return ret;
}
#endif /* USE_SETSYSTEMSTATE_CMD */



#define USE_SETPARAM_CMD
#ifdef USE_SETPARAM_CMD
#define CMD_SETPARAM_LEN 9
#define CMD_SETPARAM "\x10\x00\x05\x00\xff\xff\xff\xff\xff"

typedef enum _cmd_param_e {
  CMD_PARAM_NOTUSED       = 0x00,
  CMD_PARAM_LEFT_COVER    = 0x01,
  CMD_PARAM_TOP_COVER     = 0x02,
  CMD_PARAM_RIGHT_COVER   = 0x03,
  CMD_PARAM_BOTTOM_COVER  = 0x04,
  CMD_PARAM_MAX
} dCmdParam_e ;

int shtsc_CMD_SetParam(void *ts, dCmdParam_e eParam, unsigned paramVal)
{
  char cmdArray[CMD_SETPARAM_LEN];

  memcpy(cmdArray, CMD_SETPARAM, CMD_SETPARAM_LEN);
  cmdArray[4] = eParam;
  cmdArray[5] = (paramVal & 0x000000ff) >> 0; // little endian 4 byte // LSB
  cmdArray[6] = (paramVal & 0x0000ff00) >> 8;
  cmdArray[7] = (paramVal & 0x00ff0000) >> 16;
  cmdArray[8] = (paramVal & 0xff000000) >> 24; // MSB
  return issue_command(ts, cmdArray, CMD_SETPARAM_LEN);
}
#endif /* USE_SETPARAM_CMD */

int detectDevice(void)
{
  return DEVICE_CODE_LR388K5;
}

static void shtsc_reset_delay(void)
{
  msleep(SHTSC_RESET_TIME);
}

static void touch_enable (struct shtsc_i2c *ts)
{
  if(ts->disabled)
    {
#ifndef DEBUG_IRQ_DISABLE
      if(ts->client->irq) enable_irq(ts->client->irq);
#endif
      ts->disabled = false;
    }
}


static	void	touch_disable(struct shtsc_i2c *ts)
{
  if(!ts->disabled)	{
    if(ts->client->irq)		disable_irq(ts->client->irq);
    ts->disabled = true;
  }
}

static int shtsc_input_open(struct input_dev *dev)
{
  struct shtsc_i2c *ts = input_get_drvdata(dev);

  touch_enable(ts);

  printk("%s\n", __func__);

  return 0;
}

static void shtsc_input_close(struct input_dev *dev)
{
  struct shtsc_i2c *ts = input_get_drvdata(dev);

  touch_disable(ts);

  printk("%s\n", __func__);
}

#ifdef CONFIG_OF
static int shtsc_get_dt_coords(struct device *dev, char *name,
			       struct shtsc_i2c_pdata *pdata)
{
  u32 coords[SHTSC_COORDS_ARR_SIZE];
  struct property *prop;
  struct device_node *np = dev->of_node;
  int coords_size, rc;

  prop = of_find_property(np, name, NULL);
  if (!prop)
    return -EINVAL;
  if (!prop->value)
    return -ENODATA;

  coords_size = prop->length / sizeof(u32);
  if (coords_size != SHTSC_COORDS_ARR_SIZE) {
    dev_err(dev, "invalid %s\n", name);
    return -EINVAL;
  }

  rc = of_property_read_u32_array(np, name, coords, coords_size);
  if (rc && (rc != -EINVAL)) {
    dev_err(dev, "Unable to read %s\n", name);
    return rc;
  }

  if (strncmp(name, "sharp,panel-coords",
	      sizeof("sharp,panel-coords")) == 0) {
    pdata->panel_minx = coords[0];
    pdata->panel_miny = coords[1];
    pdata->panel_maxx = coords[2];
    pdata->panel_maxy = coords[3];
  } else if (strncmp(name, "sharp,display-coords",
		     sizeof("sharp,display-coords")) == 0) {
    pdata->disp_minx = coords[0];
    pdata->disp_miny = coords[1];
    pdata->disp_maxx = coords[2];
    pdata->disp_maxy = coords[3];
  } else {
    dev_err(dev, "unsupported property %s\n", name);
    return -EINVAL;
  }

  return 0;
}

#ifdef DP_8074_DEMO
#elif defined DP_8084_EVA
//   2015.04.09 added by Y.Nakamura
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

//   2015.04.09 added by Y.Nakamura
static int shtsc_regulator_configure(struct shtsc_i2c *ts, bool on )
{
	int retval;

	if( on==false )
		goto hw_shutdown;

	/*
	ts->vdd = regulator_get(&ts->client->dev, "vdd");
	if( IS_ERR(ts->vdd) ){
		dev_err(&ts->client->dev, "%s: failed to get vdd regulator\n", __func__);
		return PTR_ERR(ts->vdd);
	}

	if (regulator_count_voltages(ts->vdd) > 0) {
		retval = regulator_set_voltage(ts->vdd,
									   SHTSC_VTG_MIN_UV, SHTSC_VTG_MAX_UV);//3300000, 3300000);
		if (retval) {
			dev_err(&ts->client->dev,
				"regulator set_vtg failed retval =%d\n",
				retval);
			goto err_set_vtg_vdd;
		}
	}
	*/

	if (ts->pdata->i2c_pull_up) {
		ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc_i2c");
		if (IS_ERR(ts->vcc_i2c)) {
			dev_err(&ts->client->dev, "%s: Failed to get i2c regulator\n",
					__func__);
			retval = PTR_ERR(ts->vcc_i2c);
			goto err_get_vtg_i2c;
		}

		if (regulator_count_voltages(ts->vcc_i2c) > 0) {
			retval = regulator_set_voltage(ts->vcc_i2c,
										   SHTSC_I2C_VTG_MIN_UV, SHTSC_I2C_VTG_MAX_UV);//1800000, 1800000);
			if (retval) {
				dev_err(&ts->client->dev, "reg set i2c vtg failed retval =%d\n",
					retval);
			goto err_set_vtg_i2c;
			}
		}
	}
	return 0;

err_set_vtg_i2c:
	if (ts->pdata->i2c_pull_up)
		regulator_put(ts->vcc_i2c);
err_get_vtg_i2c:
//	if (regulator_count_voltages(ts->vdd) > 0)
//		regulator_set_voltage(ts->vdd, 0, SHTSC_VTG_MAX_UV);
//err_set_vtg_vdd:
//	regulator_put(ts->vdd);
	return retval;

hw_shutdown:
//	if (regulator_count_voltages(ts->vdd) > 0)
//		regulator_set_voltage(ts->vdd, 0, SHTSC_VTG_MAX_UV);
//	regulator_put(ts->vdd);
	if (ts->pdata->i2c_pull_up) {
		if (regulator_count_voltages(ts->vcc_i2c) > 0)
			regulator_set_voltage(ts->vcc_i2c, 0, SHTSC_I2C_VTG_MAX_UV);
		regulator_put(ts->vcc_i2c);
	}
	pr_info("%s: shutdown \n", __func__);

	return 0;
}

//   2015.04.09 added by Y.Nakamura
static int shtsc_power_on(struct shtsc_i2c *ts, bool on)
{
	int retval;

	if (on == false)
		goto power_off;

	/*
	retval = reg_set_optimum_mode_check(ts->vdd,
		SHTSC_ACTIVE_LOAD_UA);
	if (retval < 0) {
		dev_err(&ts->client->dev, "Regulator vdd set_opt failed rc=%d\n", retval);
		return retval;
	}

	retval = regulator_enable(ts->vdd);
	if (retval) {
		dev_err(&ts->client->dev, "Regulator vdd enable failed rc=%d\n",retval);
		goto error_reg_en_vdd;
	}
	*/

	if (ts->pdata->i2c_pull_up) {
		retval = reg_set_optimum_mode_check(ts->vcc_i2c,
			SHTSC_I2C_LOAD_UA);
		if (retval < 0) {
			dev_err(&ts->client->dev, "Regulator vcc_i2c set_opt failed rc=%d\n", retval);
			goto error_reg_opt_i2c;
		}

		retval = regulator_enable(ts->vcc_i2c);
		if (retval) {
			dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", retval);
			goto error_reg_en_vcc_i2c;
		}
	}
	return 0;

error_reg_en_vcc_i2c:
	if (ts->pdata->i2c_pull_up)
		reg_set_optimum_mode_check(ts->vcc_i2c, 0);
error_reg_opt_i2c:
//	regulator_disable(ts->vdd);
//error_reg_en_vdd:
//	reg_set_optimum_mode_check(ts->vdd, 0);
	return retval;

power_off:
//	reg_set_optimum_mode_check(ts->vdd, 0);
//	regulator_disable(ts->vdd);
	if (ts->pdata->i2c_pull_up) {
		reg_set_optimum_mode_check(ts->vcc_i2c, 0);
		regulator_disable(ts->vcc_i2c);
	}
	pr_info("%s: power off \n", __func__);
	return 0;

}
// ---
#endif /* DP_8074_DEMO */

static int shtsc_parse_dt(struct device *dev, struct shtsc_i2c_pdata *pdata)
{
  struct device_node *np = dev->of_node;
  int rc;
  u32 temp_val;

  rc = shtsc_get_dt_coords(dev, "sharp,panel-coords", pdata);
  if (rc)
    return rc;

  rc = shtsc_get_dt_coords(dev, "sharp,display-coords", pdata);
  if (rc)
    return rc;

  /* regulator info */
  pdata->i2c_pull_up = of_property_read_bool(np, "sharp,i2c-pull-up");

  /* reset, irq gpio info */
  pdata->reset_gpio = of_get_named_gpio_flags(np, "sharp,reset-gpio",0, &pdata->reset_gpio_flags);
  pdata->irq_gpio = of_get_named_gpio_flags(np, "sharp,irq-gpio",0, &pdata->irq_gpio_flags);
				
  rc = of_property_read_u32(np, "ts_touch_num_max", &temp_val);
  if( !rc ) pdata->ts_touch_num_max = temp_val;
  rc = of_property_read_u32(np, "ts_pressure_max", &temp_val);
  if( !rc ) pdata->ts_pressure_max = temp_val;
  rc = of_property_read_u32(np, "ts_flip_x", &temp_val);
  if( !rc ) pdata->ts_flip_x = temp_val;
  rc = of_property_read_u32(np, "ts_flip_y", &temp_val);
  if( !rc ) pdata->ts_flip_y = temp_val;
  rc = of_property_read_u32(np, "ts_swap_xy", &temp_val);
  if( !rc ) pdata->ts_swap_xy = temp_val;
#if 1
  printk(KERN_INFO "[SHTP PARSE DT] reset gpio = %d\n",pdata->reset_gpio);
  printk(KERN_INFO "[SHTP PARSE DT] irq gpio = %d\n",pdata->irq_gpio);
#endif
  return 0;
}
#else
static int shtsc_parse_dt(struct device *dev, struct shtsc_i2c_pdata *pdata)
{
  return -ENODEV;
}
#endif

#if defined(CONFIG_FB)
static int shtsc_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct shtsc_i2c *data = i2c_get_clientdata(client);
	//	struct input_dev *input_dev = data->input;

	if (data->dev_sleep) {
	        DBGLOG("Device already in sleep\n");
		return 0;
	}

#ifdef ENABLE_SUSPEND_GESTURE
	/* issue SetSystemState(DeepIdle) command to activate gesture */

	issue_command(g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);
	DBGLOG("shtsc: issue deepidle command\n");
	DBGLOG("shtsc: enable irq IRQ_NO:%d\n", data->client->irq);
	enable_irq_wake(data->client->irq);
#else /* ENABLE_SUSPEND_GESTURE */
	disable_irq(data->client->irq);

	DBGLOG("Device in sleep\n");
#endif /* ENABLE_SUSPEND_GESTURE */
	
	data->dev_sleep = true;
	return 0;
}

static int shtsc_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct shtsc_i2c *data = i2c_get_clientdata(client);
	//	struct input_dev *input_dev = data->input;

	if (!data->dev_sleep) {
	        DBGLOG( "Device already in resume\n");
		return 0;
	}

#ifdef ENABLE_SUSPEND_GESTURE
	DBGLOG("shtsc: disable irq wake IRQ_NO:%d\n", data->client->irq);
	disable_irq_wake(data->client->irq);


#define READ_GESTURE_IN_RESUME
#ifdef READ_GESTURE_IN_RESUME

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "[RESUME] shtsc resume from DeepIdle\n");
#endif

      SetBankAddr(g_ts, SHTSC_BANK_TOUCH_REPORT);
      resumeStatus = ReadOneByte(g_ts, SHTSC_ADDR_RESUME_PROX);

      // input_report_key( ts->input, code , true ); to kernel
      {
	int key_gesture_code = 0x00;
#if 0
	switch (resumeStatus) {
	case 0x01:
	  key_gesture_code = KEY_GESTURE_DOUBLE_TAP;
	  break;
	case 0x02:
	  key_gesture_code = KEY_GESTURE_SLIDE_UP;
	  break;
	case 0x03:
	  key_gesture_code = KEY_GESTURE_SLIDE_DOWN;
	  break;
	case 0x04:
	  key_gesture_code = KEY_GESTURE_SLIDE_RIGHT;
	  break;
	case 0x05:
	  key_gesture_code = KEY_GESTURE_SLIDE_LEFT;
	  break;
	case 0x63:
	  key_gesture_code = KEY_GESTURE_CHAR_C;
	  break;
	case 0x65:
	  key_gesture_code = KEY_GESTURE_CHAR_E;
	  break;
	case 0x6D:
	  key_gesture_code = KEY_GESTURE_CHAR_M;
	  break;
	case 0x6F:
	  key_gesture_code = KEY_GESTURE_CHAR_O;
	  break;
	case 0x76:
	  key_gesture_code = KEY_GESTURE_CHAR_V;
	  break;
	case 0x77:
	  key_gesture_code = KEY_GESTURE_CHAR_W;
	  break;
	default:
	  key_gesture_code = KEY_GESTURE_UNKNOWN;
	  break;
	}
#else
	/* you need to change this for your environment */
	key_gesture_code = resumeStatus;
#endif /* 0 */

	if (key_gesture_code) {
	  input_report_key( g_ts->input, key_gesture_code, true );
	  input_sync(g_ts->input);
	  input_report_key( g_ts->input, key_gesture_code, false );
	  input_sync(g_ts->input);
	}
#if defined(DEBUG_SHTSC)
	printk(KERN_INFO "[RESUME] shtsc resumeStatus %02X\n", resumeStatus);
#endif
      }

#if 0
#ifdef FORCE_DISPLAY_ON
	/* set display on without any control of the system */
	/* the system must control the display without this */
	input_report_key( g_ts->input, KEY_POWER, true );
	input_sync(g_ts->input);
	input_report_key( g_ts->input, KEY_POWER, false );
	input_sync(g_ts->input);
	msleep(1000); /* wait HSYNC becomes stable */
#endif /* FORCE_DISPLAY_ON */
#endif /* 0 */

#endif /* READ_GESTURE_IN_RESUME */


	// reset K5 but not enable_irq()
	shtsc_reset(g_ts, false);

	G_Irq_Mask = 0xffff;
#ifdef GET_REPORT
	reportBuf[0] = (unsigned char)0;
#endif /* GET_REPORT */

	//	shtsc_reset_delay();

	g_ts->wait_state = WAIT_RESET;
	g_ts->wait_result = false;
	shtsc_reset(g_ts, true);
	// wait
	WaitAsync(g_ts);

	shtsc_reset(g_ts, true);
	msleep(10);
	shtsc_system_init(g_ts);
	msleep(100);

	DBGLOG("Device in active\n");

#else /* ENABLE_SUSPEND_GESTURE */
	shtsc_reset(g_ts, false);

	G_Irq_Mask = 0xffff;
#ifdef GET_REPORT
	reportBuf[0] = (unsigned char)0;
#endif /* GET_REPORT */

	//	shtsc_reset_delay();

	shtsc_reset(g_ts, true);
	msleep(10);
	shtsc_system_init(g_ts);
	msleep(100);

	DBGLOG("Device in active\n");

#ifndef DEBUG_IRQ_DISABLE
	enable_irq(data->client->irq);
#endif
#endif /* ENABLE_SUSPEND_GESTURE */

	data->dev_sleep = false;
	return 0;
}

static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	struct shtsc_i2c *shtsc_dev_data =
		container_of(self, struct shtsc_i2c, fb_notif);

	if (evdata && evdata->data && shtsc_dev_data && shtsc_dev_data->client) {
		if (event == FB_EVENT_BLANK) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK)
				shtsc_resume(&shtsc_dev_data->client->dev);
			else if (*blank == FB_BLANK_POWERDOWN)
				shtsc_suspend(&shtsc_dev_data->client->dev);
		}
	}
	return 0;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
static int __devinit shtsc_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
  //  const struct shtsc_pdata *pdata = client->dev.platform_data;
  struct shtsc_i2c_pdata *pdata;
  struct shtsc_i2c *ts;
  struct input_dev *input_dev;
  int err;

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[ENTER] shtsc_probe\n");
#endif

  //2014.10.16 added
  if (client->dev.of_node) {
    pdata = devm_kzalloc(&client->dev,
			 sizeof(struct shtsc_i2c_pdata), GFP_KERNEL);
    if (!pdata) {
      dev_err(&client->dev, "Failed to allocate memory\n");
      return -ENOMEM;
    }
    err = shtsc_parse_dt(&client->dev, pdata);
    if (err){
      kfree(pdata);
      return err;
    }
  } else{
    pdata = client->dev.platform_data;
  }

  /* No pdata no way forward */
  if (pdata == NULL) {
    dev_err(&client->dev, "no pdata\n");
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "%s(%d):\n", __FILE__, __LINE__);
#endif
    return -ENODEV;
  }

  if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
    return -EIO;

  ts = kzalloc(sizeof(struct shtsc_i2c), GFP_KERNEL);
  g_ts = ts;

  input_dev = input_allocate_device();
  if (!ts || !input_dev) {
    err = -ENOMEM;
    goto err_free_mem;
  }

  ts->client = client;
  ts->input = input_dev;
  ts->pdata = pdata;//2014.10.16 added

  ts->min_x = pdata->panel_minx;
  ts->max_x = pdata->panel_maxx;
  ts->min_y = pdata->panel_miny;
  ts->max_y = pdata->panel_maxy;
  ts->pressure_max = pdata->ts_pressure_max;
  ts->touch_num_max = pdata->ts_touch_num_max;//2014.10.16 added
  ts->flip_x = pdata->ts_flip_x;
  ts->flip_y = pdata->ts_flip_y;
  ts->swap_xy = pdata->ts_swap_xy;

  ts->reset_pin = pdata->reset_gpio;
  ts->irq_pin = pdata->irq_gpio;

  mutex_init(&(ts->mutex));

  snprintf(ts->phys, sizeof(ts->phys),
	   "%s/input0", dev_name(&client->dev));

  input_dev->name = SHTSC_DRIVER_NAME;
  input_dev->phys = ts->phys;
  input_dev->id.bustype = BUS_I2C;
  input_dev->dev.parent = &client->dev;
  input_dev->open = shtsc_input_open;//2014.10.16 added
  input_dev->close = shtsc_input_close;//2014.10.16 added

  __set_bit(EV_ABS, input_dev->evbit);
  __set_bit(EV_KEY, input_dev->evbit);
  __set_bit(BTN_TOUCH, input_dev->keybit);//2014.10.16 added
  __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);//2014.10.16 added

  /* For multi-touch */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0))
  err = input_mt_init_slots(input_dev, SHTSC_MAX_FINGERS, INPUT_MT_DIRECT);
#else /*  KERNEL_3_10 */
  err = input_mt_init_slots(input_dev, SHTSC_MAX_FINGERS);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)) */

  if (err)
    goto err_free_mem;
  input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, SHTSC_I2C_SIZE_MAX, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_X,
		       pdata->disp_minx, pdata->disp_maxx, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
		       pdata->disp_miny, pdata->disp_maxy, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_PRESSURE  , 0, SHTSC_I2C_P_MAX, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE , 0, MT_TOOL_MAX, 0, 0);

#ifdef SHTSC_ICON_KEY_NUM
  {
    int cnt;
    for(cnt=0;cnt<SHTSC_ICON_KEY_NUM;cnt++){
      input_set_capability( input_dev , EV_KEY , shtsc_keyarray[cnt] );
    }
  }
#endif

#ifdef ENABLE_SUSPEND_GESTURE
#if 0
  /* you need to add the KEYCODEs for your environment! */
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_DOUBLE_TAP );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_SLIDE_UP );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_SLIDE_DOWN );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_SLIDE_RIGHT );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_SLIDE_LEFT );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_CHAR_C );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_CHAR_E );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_CHAR_M );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_CHAR_O );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_CHAR_V );
  input_set_capability( input_dev , EV_KEY , KEY_GESTURE_CHAR_W );
#endif /* 0 */
#ifdef FORCE_DISPLAY_ON
  input_set_capability( input_dev , EV_KEY , KEY_POWER );
#endif /* FORCE_DISPLAY_ON */
#endif /* ENABLE_SUSPEND_GESTURE */

#ifdef DP_8074_DEMO
#elif defined DP_8084_EVA
// 2015.04.09 added by Y.Nakamura for pma8084
  err = shtsc_regulator_configure(ts, true);
  if(err < 0) {
	  dev_err(&client->dev, "failed to configure regulators\n");
	  goto err_reg_configure;
  }
  err = shtsc_power_on(ts, true);
  if(err < 0) {
	  dev_err(&client->dev, "failed to power on\n");
	  goto err_power_device;
  }
// ---
#endif /* DP_8074_DEMO */

  if (ts->reset_pin) {
    err = gpio_request(ts->reset_pin, NULL);
    if (err) {
      dev_err(&client->dev,
	      "Unable to request GPIO pin %d.\n",
	      ts->reset_pin);
      goto err_free_mem;
    }
  }

  shtsc_reset(ts, false);
  //  shtsc_reset_delay();//2014.11.12 added

  if (gpio_is_valid(ts->irq_pin)) {//2014.11.12 added
    /* configure touchscreen irq gpio */
    err = gpio_request(ts->irq_pin, "shtsc_irq_gpio");
    if (err) {
      dev_err(&client->dev, "unable to request gpio [%d]\n",
	      ts->irq_pin);
      goto err_free_irq_gpio;
    }
  }
  err = gpio_direction_input(ts->irq_pin);
  if (err < 0) {
    dev_err(&client->dev,
	    "Failed to configure input direction for GPIO %d, error %d\n",
	    ts->irq_pin, err);
    goto err_free_irq_gpio;
  }

  client->irq = gpio_to_irq(ts->irq_pin);
  if (client->irq < 0) {
    err = client->irq;
    dev_err(&client->dev,
	    "Unable to get irq number for GPIO %d, error %d\n",
	    ts->irq_pin, err);
    goto err_free_irq_gpio;
  }
#ifdef FORCE_FIRM_UPDATE
  ts->workqueue = create_singlethread_workqueue("shtsc_work");
  INIT_WORK(&ts->workstr, update_flash_func);
#endif /* FORCE_FIRM_UPDATE */
#ifndef DEBUG_IRQ_DISABLE
  err = request_threaded_irq(client->irq, NULL, shtsc_irq_thread,
                             (IRQF_TRIGGER_HIGH|IRQF_ONESHOT), client->dev.driver->name, ts);
#endif

  if (err < 0) {
    dev_err(&client->dev,
	    "irq %d busy? error %d\n", client->irq, err);
    goto err_free_irq_gpio;
  }

  input_set_drvdata(input_dev, ts);//2014.10.16 added
  err = input_register_device(input_dev);
  if (err)
    goto err_free_irq;

  i2c_set_clientdata(client, ts);
  device_init_wakeup(&client->dev, 1);

  ts->wait_state = WAIT_RESET;
  ts->wait_result = false;
  shtsc_reset(ts, true);
  // wait
  err = WaitAsync(ts);
  if (err)
    goto err_free_irq;

  VersionModelCode = detectDevice();
  shtsc_system_init(ts);

#if defined(CONFIG_FB)
  ts->fb_notif.notifier_call = fb_notifier_callback;
  ts->dev_sleep = false;
  err = fb_register_client(&ts->fb_notif);

  if (err)
    dev_err(&ts->client->dev, "Unable to register fb_notifier: %d\n",
      err);
#endif

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[EXIT] shtsc_probe\n");
#endif

  printk(KERN_INFO "SHARP touchscreen controller I2C driver\n");

  return 0;

 err_free_irq:
  free_irq(client->irq, ts);
 err_free_irq_gpio:
  gpio_free(ts->irq_pin);
  shtsc_reset(ts, true);
  if (ts->reset_pin)
    gpio_free(ts->reset_pin);
 err_free_mem:
#ifdef DP_8074_DEMO
#elif defined DP_8084_EVA
// 2015.04.09 added by Y.Nakamura
  shtsc_power_on(ts, false);
err_power_device:
  shtsc_regulator_configure(ts, false);
err_reg_configure:
// ---
#endif /* DP_8074_DEMO */
  input_free_device(input_dev);
  kfree(ts);

  return err;
}

static int __devexit shtsc_i2c_remove(struct i2c_client *client)
{
  struct shtsc_i2c *ts = i2c_get_clientdata(client);

  shtsc_reset(ts, true);

  mutex_init(&(ts->mutex));

  //	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
  free_irq(ts->client->irq, ts);
  input_unregister_device(ts->input);
#if defined(CONFIG_FB)
  if (fb_unregister_client(&ts->fb_notif))
    dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
  //	unregister_early_suspend(&ts->early_suspend);
#endif
  if (gpio_is_valid(ts->pdata->reset_gpio))
    gpio_free(ts->pdata->reset_gpio);

  if (gpio_is_valid(ts->pdata->irq_gpio))
    gpio_free(ts->pdata->irq_gpio);

  if (client->dev.of_node) {
    kfree(ts->pdata);
  }

#ifdef DP_8074_DEMO
#elif defined DP_8084_EVA
  // 2015.04.09 added by Y.Nakamura for pma8084
  shtsc_power_on(ts, false);
  shtsc_regulator_configure(ts, false);
#endif /* DP_8074_DEMO */

  //  kfree(ts->object_table);
  kfree(ts);

  //	debugfs_remove_recursive(debug_base);

  return 0;
}

#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)

/* must be called with ts->mutex held */
static void shtsc_disable(struct shtsc_spi *ts)
{
}

/* must be called with ts->mutex held */
static void shtsc_enable(struct shtsc_spi *ts)
{
}

static int shtsc_open(struct input_dev *input)
{
  struct shtsc_spi *ts = input_get_drvdata(input);

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[ENTER] shtsc_open\n");
#endif
  mutex_lock(&ts->mutex);

  if (!ts->suspended)
    shtsc_enable(ts);

  ts->opened = true;

  mutex_unlock(&ts->mutex);

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[EXIT] shtsc_open\n");
#endif
  return 0;
}

static void shtsc_close(struct input_dev *input)
{
  struct shtsc_spi *ts = input_get_drvdata(input);

  mutex_lock(&ts->mutex);

  if (!ts->suspended)
    shtsc_disable(ts);

  ts->opened = false;

  mutex_unlock(&ts->mutex);
}

static int __devinit shtsc_spi_probe(struct spi_device *spi)
{
  //  const struct shtsc_pdata *pdata = spi->dev.platform_data;
  struct shtsc_i2c_pdata *pdata;

  struct shtsc_spi *ts;
  struct input_dev *input_dev;

  int error;

  g_spi = spi;

#ifdef DEBUG_SHTSC
  printk(KERN_INFO "[ENTER] shtsc_probe\n");
#endif /* DEBUG_SHTSC */

  //2014.10.16 added
  if (client->dev.of_node) {
    pdata = devm_kzalloc(&client->dev,
			 sizeof(struct shtsc_i2c_pdata), GFP_KERNEL);
    if (!pdata) {
      dev_err(&client->dev, "Failed to allocate memory\n");
      return -ENOMEM;
    }
    err = shtsc_parse_dt(&client->dev, pdata);
    if (err){
      kfree(pdata);
      return err;
    }
  } else{
    pdata = client->dev.platform_data;
  }

  /* No pdata no way forward */
  if (!pdata) {
    dev_dbg(&spi->dev, "no platform data\n");
    return -ENODEV;
  }

  if (spi->irq <= 0) {
    dev_dbg(&spi->dev, "no irq\n");
    return -ENODEV;
  }
	
#ifdef DEBUG_SHTSC
  printk(KERN_INFO "[2] shtsc_probe %d\n", spi->irq);
#endif /* DEBUG_SHTSC */
	
  spi->mode = SPI_MODE_3;
  spi->bits_per_word = 8;
  if (!spi->max_speed_hz)
    spi->max_speed_hz = SHTSC_MAX_SPI_SPEED_IN_HZ;

  error = spi_setup(spi);
  if (error)
    return error;

#ifdef DEBUG_SHTSC
  printk(KERN_INFO "[3] shtsc_probe\n");
#endif /* DEBUG_SHTSC */

  ts = kzalloc(sizeof(*ts), GFP_KERNEL);
  g_ts = ts;
  input_dev = input_allocate_device();
	
  if (!ts || !input_dev) {
    error = -ENOMEM;
    goto err_free_mem;
  }
	
#ifdef DEBUG_SHTSC
  printk(KERN_INFO "[4] shtsc_probe\n");
#endif /* DEBUG_SHTSC */
	
  ts->spi = spi;
  ts->input = input_dev;

  ts->pdata = pdata;//2014.10.16 added

  ts->min_x = pdata->panel_minx;
  ts->max_x = pdata->panel_maxx;
  ts->min_y = pdata->panel_miny;
  ts->max_y = pdata->panel_maxy;
  ts->pressure_max = pdata->ts_pressure_max;
  ts->touch_num_max = pdata->ts_touch_num_max;//2014.10.16 added
  ts->flip_x = pdata->ts_flip_x;
  ts->flip_y = pdata->ts_flip_y;
  ts->swap_xy = pdata->ts_swap_xy;

  ts->reset_pin = pdata->reset_gpio;
  ts->irq_pin = pdata->irq_gpio;
  /*
    ts->irq = spi->irq;

    ts->reset_pin = OMAP4PANDA_GPIO_SHTSC_RESET;
  */
  mutex_init(&ts->mutex);
  spin_lock_init(&ts->lock);
  //	setup_timer(&ts->timer, shtsc_timer, (unsigned long)ts); 20141014

  snprintf(ts->phys, sizeof(ts->phys),
	   "%s/input-ts", dev_name(&spi->dev));

  input_dev->name = SHTSC_DRIVER_NAME;
  input_dev->phys = ts->phys;
  input_dev->id.bustype = BUS_SPI;
  input_dev->dev.parent = &spi->dev;
  input_dev->open = shtsc_input_open;//2014.10.16 added
  input_dev->close = shtsc_input_close;//2014.10.16 added

  __set_bit(EV_ABS, input_dev->evbit);
  __set_bit(EV_KEY, input_dev->evbit);
  __set_bit(BTN_TOUCH, input_dev->keybit);//2014.10.16 added
  __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);//2014.10.16 added

  error = input_mt_init_slots(input_dev, SHTSC_MAX_FINGERS);

  if (error)
    goto err_free_mem;
	
  input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, SHTSC_I2C_SIZE_MAX, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_X,
		       pdata->disp_minx, pdata->disp_maxx, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
		       pdata->disp_miny, pdata->disp_maxy, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_PRESSURE  , 0, SHTSC_I2C_P_MAX, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE , 0, MT_TOOL_MAX, 0, 0);

  if (ts->reset_pin) {
    error = gpio_request(ts->reset_pin, NULL);
    if (error) {
      dev_err(&spi->dev,
	      "Unable to request GPIO pin %d.\n",
	      ts->reset_pin);
      goto err_free_mem;
    }
  }

  if (ts->spiss_pin) {
    error = gpio_request(ts->spiss_pin, NULL);
    if (error) {
      dev_err(&spi->dev,
	      "Unable to request GPIO pin %d.\n",
	      ts->spiss_pin);
      goto err_free_mem;
    }
  }
	
  input_dev->open = shtsc_open;
  input_dev->close = shtsc_close;

  input_set_drvdata(input_dev, ts);

#ifndef DEBUG_IRQ_DISABLE
  error = request_threaded_irq(spi->irq, NULL, shtsc_irq_thread,
			       pdata->irqflags, client->dev.driver->name, ts);
#endif

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "%s(%d): %d\n", __FILE__, __LINE__, error);
#endif
  if (error) {
    dev_err(&spi->dev, "Failed to request irq, err: %d\n", error);
    goto err_free_mem;
  }

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[5] shtsc_probe\n");
#endif

  spi_set_drvdata(spi, ts);
  input_set_drvdata(ts->input, ts);//2014.10.16 added
  error = input_register_device(ts->input);
  if (error) {
    dev_err(&spi->dev,
	    "Failed to register input device, err: %d\n", error);
    goto err_clear_drvdata;
  }

  shtsc_reset(g_ts, false);
  msleep(100);
  shtsc_reset(g_ts, true);
  msleep(10);

  VersionModelCode = detectDevice();

  shtsc_system_init(ts);
  msleep(100);

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[EXIT] shtsc_probe\n");
#endif

  printk(KERN_INFO "SHARP touchscreen controller SPI driver\n");

  return 0;

 err_clear_drvdata:
  spi_set_drvdata(spi, NULL);
  free_irq(spi->irq, ts);
 err_free_mem:
  input_free_device(input_dev);
  kfree(ts);
  return error;
}

static int __devexit shtsc_spi_remove(struct spi_device *spi)
{
  struct shtsc_spi *ts = spi_get_drvdata(spi);

  free_irq(ts->spi->irq, ts);
  input_unregister_device(ts->input);
  kfree(ts);

  spi_set_drvdata(spi, NULL);
  return 0;
}
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

int current_page = 0;

char iobuf[4*1024];
static long dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
  long ret = true;

  //  unsigned int index;
  u8 addr, val;
  //  unsigned long r;
  int r;

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[ENTER] shtsc_ioctl\n");    
#endif

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "shtsc dev_ioctl switch - cmd: %x, arg: %lx\n", cmd, arg);
#endif

  switch(cmd) {

  case SHTSC_IOCTL_SET_PAGE:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - SET_PAGE; cmd %x, arg %lx\n", cmd, arg);
#endif
    current_page = arg;
    break;

  case SHTSC_IOCTL_FLASH_ACCESS_START:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - FLASH_ACCESS_START; cmd %x\n", cmd);
#endif
    flash_access_start_shtsc(g_ts);
    break;

  case SHTSC_IOCTL_FLASH_ACCESS_END:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - FLASH_ACCESS_END; cmd %x\n", cmd);
#endif
    flash_access_end_shtsc(g_ts);
    break;

  case SHTSC_IOCTL_ERASE:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - ERASE; cmd %x, current_page %d\n", cmd, current_page);
#endif
#ifdef FLASH_CHECK
    if (flash_erase_page_shtsc(g_ts, current_page))
      return -1;
#else /* FLASH_CHECK */
    flash_erase_page_shtsc(g_ts, current_page);
#endif /* FLASH_CHECK */
    break;

  case SHTSC_IOCTL_WRITE:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - WRITE; cmd %x, current_page %d\n", cmd, current_page);
#endif
    if (copy_from_user(iobuf, (char *)arg, (4*1024))) {
      printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_from_user\n");
      return -1;
    }
#ifdef FLASH_CHECK
    if (flash_write_page_shtsc(g_ts, current_page, iobuf))
      return -1;
#else /* FLASH_CHECK */
    flash_write_page_shtsc(g_ts, current_page, iobuf);
#endif /* FLASH_CHECK */
    break;

  case SHTSC_IOCTL_VERIFY:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - VERIFY; cmd %x, current_page %d\n", cmd, current_page);
#endif
    if (copy_from_user(iobuf, (char *)arg, (4*1024))) {
      printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_from_user\n");
      return -1;
    }
#ifdef FLASH_CHECK
    if (flash_verify_page_shtsc(g_ts, current_page, iobuf))
      return -1;
#else /* FLASH_CHECK */
    flash_verify_page_shtsc(g_ts, current_page, iobuf);
#endif /* FLASH_CHECK */
    break;

  case SHTSC_IOCTL_ERASE_ALL:
    {
#define LAST_PAGE 16 
      int page;
      printk(KERN_INFO "%s(%d): flash_access start\n", __FILE__, __LINE__);
      flash_access_start_shtsc(g_ts);
      for (page = 0; page < LAST_PAGE; page++) {
	flash_erase_page_shtsc(g_ts, page);
	printk(KERN_INFO "flash_erase_page_shtsc done: page %d\n",  page);
      }
      printk(KERN_INFO "%s(%d): flash_access end\n", __FILE__, __LINE__);
      flash_access_end_shtsc(g_ts);
      printk(KERN_INFO "%s(%d): flash erased.\n", __FILE__, __LINE__);
    }
    break;
  case SHTSC_IOCTL_REG_1WRITE:
    //    printk(KERN_INFO "SHTSC_IOCTL_REG_1WRITE\n");
    addr = (arg >> 8) & 0xFF;
    val = arg & 0xFF;
    //    printk(KERN_INFO "SHTSC_IOCTL_REG_1WRITE: cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, addr, val, val);
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "SHTSC_IOCTL_REG_1WRITE: addr: %02X (Hex), data: %02X (Hex)\n", addr, val);
#endif
    WriteOneByte(g_ts, addr, val);
    //    printk(KERN_INFO "SHTSC_IOCTL_REG_1WRITE done\n");
    break;

  case SHTSC_IOCTL_REG_1READ:
    //    printk(KERN_INFO "SHTSC_IOCTL_REG_1READ\n");
    //addr = 0xFF & arg;
    val = ReadOneByte(g_ts, ((struct reg *)arg)->addr);
    ((struct reg *)arg)->data = val;

#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "SHTSC_IOCTL_REG_1READ: cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, ((struct reg *)arg)->addr, val, val);
#endif
    //    printk(KERN_INFO "SHTSC_IOCTL_REG_1READ done\n");
    break;

  case SHTSC_IOCTL_REG_N_RW_SET_ADDR:
    s_shtsc_addr = 0xFF & (arg >> 16);
    s_shtsc_len = 0xFFFF & arg;
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "SHTSC_IOCTL_REG_N_RW_SET_ADDR: cmd %x, arg %lx a:0x%x len:%d\n", cmd, arg, s_shtsc_addr, s_shtsc_len);
#endif
    break;

  case SHTSC_IOCTL_REG_N_WRITE_1ADDR_GO:
    /* s_shtsc_len is initialized __IF__ SHTSC_IOCTL_REG_N_RW_SET_ADDR is issued before */
    r = copy_from_user(s_shtsc_buf, (char *)arg, s_shtsc_len);
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "Driver Multibyte write. addr %02X, len: %x, data[0-3]: %02X %02X %02X %02X ....\n", 
	   s_shtsc_addr, s_shtsc_len, s_shtsc_buf[0], s_shtsc_buf[1], s_shtsc_buf[2], s_shtsc_buf[3]);
#endif
    if (r != 0) {
      printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_from_user(%d)\n", r);
      return -1;
    }
    //    printk(KERN_INFO "SHTSC_IOCTL_REG_N_WRITE_1ADDR: cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, addr, val, val);
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "SHTSC_IOCTL_REG_N_WRITE_1ADDR: cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, s_shtsc_addr, s_shtsc_len, s_shtsc_len);
#endif
    WriteMultiBytes(g_ts, s_shtsc_addr, s_shtsc_buf, s_shtsc_len);

    break;

  case SHTSC_IOCTL_REG_N_READ_1ADDR_GO:
    /* s_shtsc_len is initialized __IF__ SHTSC_IOCTL_REG_N_RW_SET_ADDR is issued before */
    ReadMultiBytes(g_ts, s_shtsc_addr, s_shtsc_len, s_shtsc_buf);
    //msleep(10); // not checked yet
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "Driver Multibyte read done. addr %02X, len: %x, data[0-3]: %02X %02X %02X %02X ....\n", 
	   s_shtsc_addr, s_shtsc_len, s_shtsc_buf[0], s_shtsc_buf[1], s_shtsc_buf[2], s_shtsc_buf[3]);
#endif
    r = copy_to_user((char *)arg, s_shtsc_buf, s_shtsc_len);
    if (r != 0) {
      printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_to_user(%d)\n", r);
      return -1;
    }

    break;

  case SHTSC_IOCTL_SETIRQMASK:
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
    if (arg) {
      enable_irq(g_ts->client->irq);
    } else {
      disable_irq(g_ts->client->irq);
    }
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
    if (arg) {
      enable_irq(g_ts->spi->irq);
    } else {
      disable_irq(g_ts->spi->irq);
    }
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
    error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - SETIRQMASK; cmd %x, arg %lx\n", cmd, arg);
#endif
    break;

  case SHTSC_IOCTL_RESET:
    if (arg) {
      g_ts->wait_state = WAIT_RESET;
      g_ts->wait_result = false;
      shtsc_reset(g_ts, true);
      // wait
      WaitAsync(g_ts);
      shtsc_system_init(g_ts);
      msleep(100);
    } else {
      shtsc_reset(g_ts, false);
    }
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - RESET; cmd %x, arg %lx\n", cmd, arg);
#endif
    break;

  case SHTSC_IOCTL_DEBUG:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "shtsc dev_ioctl case - DEBUG; cmd %x, arg %lx\n", cmd, arg);
#endif
    break;

  case SHTSC_IOCTL_CMD_ISSUE_RESULT:
    {
      unsigned int magicnumber;	/* 'int' width is 32bit for LP64 and LLP64 mode */
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
      printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_to_user(%d)\n", r);
      return -1;
    }
    break;

  case SHTSC_IOCTL_DRIVER_VERSION_READ:
    {
      char versionBuf[16];
      versionBuf[0] = VersionYear;
      versionBuf[1] = VersionMonth;
      versionBuf[2] = VersionDay;
      versionBuf[3] = VersionSerialNumber;
      versionBuf[4] = VersionModelCode;

      r = copy_to_user((char *)arg, versionBuf, DRIVER_VERSION_LEN); 
      if (r != 0) {
	printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_to_user(%d)\n", r);
	return -1;
      }
    }
    break;

  case SHTSC_IOCTL_DCMAP:
    {
      int x = dcmap[0];
      int y = dcmap[1];
      int len = (x * y * 2) + 4;

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc dev_ioctl case - DCMAP; cmd %x, arg %lx, %d*%d+4=%d\n", cmd, arg, x, y, len);
#endif

      if (buf_pos) {
	/* DC map ready to send out */
	if ( copy_to_user( (char *)arg, dcmap, len ) ) {
	  printk( KERN_INFO "shtsc : copy_to_user failed\n" );
	  return -EFAULT;
	}
	buf_pos = 0;
      }
      break;
    }

#ifdef GET_REPORT
  case SHTSC_IOCTL_GET_REPORT:
    {
      volatile int count;
      int len;


      while (Mutex_GetReport)
	;
      Mutex_GetReport = true;
      
      count = reportBuf[0];
      len = 4+ count*REP_SIZE;

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc dev_ioctl case - GET_REPORT; cmd %x, arg %lx, count=%d, len=%d\n", cmd, arg, count, len);
#endif

      r = copy_to_user((char *)arg, reportBuf, len);
      if (r != 0) {
	printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_to_user(%d)\n", r);
	return -1;
      }

      reportBuf[0] = (unsigned char)0;

      Mutex_GetReport = false;
    }
    break;
#endif /* GET_REPORT */

  case SHTSC_IOCTL_FLASH_READ:
    {
      /* arg: pointer to the content buffer */
      /* arg[3:0]: address to read (little endian) */
      /* arg[5:4]: length to read (little endian) */

      unsigned address;
      unsigned length;

      if (copy_from_user(iobuf, (char *)arg, (4+2))) {
	printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_from_user\n");
	return -1;
      }
      address = (iobuf[3] << 24) | (iobuf[2] << 16) | (iobuf[1] << 8) | (iobuf[0] << 0);
      length = (iobuf[5] << 8) | (iobuf[4] << 0);

#if defined(DEBUG_SHTSC)
      printk(KERN_INFO "shtsc dev_ioctl case - FLASH_READ; addr %x, arg %x\n", address, length);
#endif

      flash_read(g_ts, address, length, iobuf);
      if ( copy_to_user( (char *)arg, iobuf, length ) ) {
	printk( KERN_INFO "shtsc : copy_to_user failed\n" );
	return -EFAULT;
      }
	break;
    }

  case SHTSC_IOCTL_NOTIFY_PID:
    pid = arg; // save pid for later kill();
    printk(KERN_INFO "SHTSC_IOCTL_NOTIFY_PID: pid: %d\n", pid);
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "SHTSC_IOCTL_NOTIFY_PID: pid: %d\n", pid);
#endif
    break;

  case SHTSC_IOCTL_GET_INTERRUPT_STATUS:
#if defined(DEBUG_SHTSC)
    printk(KERN_INFO "Received SHTSC_IOCTL_GET_INTERRUPT_STATUS, %d\n", resumeStatus);
#endif

    r = copy_to_user((char *)arg, &resumeStatus, 1); // copy one-byte status
    if (r != 0) {
      printk(KERN_INFO "shtsc dev_ioctl ERROR by copy_to_user(%d)\n", r);
      return -1;
    }
    break;

  default:
    ret = false;
    break;
  }

#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "[EXIT] shtsc_ioctl\n");
#endif

  return ret;
}

int issue_command2(void *ts, unsigned char *cmd, unsigned int len)
{
#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
    struct shtsc_i2c *lts = ts;
#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)
    struct shtsc_spi *lts = ts;
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
    int err = 0;

    SetBankAddr(ts, SHTSC_BANK_COMMAND);
    DBGLOG("bank command\n");
    // set command
    WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, cmd, len);
    DBGLOG("set command (%x)\n",cmd[0]);

    // prepare waiting
    lts->cmd = cmd[0];
    lts->wait_state = WAIT_CMD;
    lts->wait_result = true;

    INIT_COMPLETION(lts->sync_completion);
    // do it
    SetIndicator(ts, SHTSC_IND_CMD);
    DBGLOG("do it\n");
    
    return err;
}


#define MAX_FRAME_SIZE	(31*28)	/* maximum number of dcmap node */
#define MAX_LINE_BUF	512	/* maximum line size for test spec file */
#define SHTSC_TEST_FRAME_NUM 30

#define CALIBRATION_DATA_SRAM_ADDRESS	0xD600
#define CALIBRATION_DATA_SRAM_SIZE	0x0900
#define CALIBRATION_DATA_FLASH_ADDRESS	0xB000
#define CALIBRATION_DATA_FLASH_SIZE	0x2000

static int local_gets(
	char *buf,
	int size,
	int fd
){
	int cnt=0;
	while( sys_read(fd,buf+cnt,1) == 1 ){
		switch( buf[cnt] ){
		case 0x0D:
		case 0x0A:
			if( cnt ){
				return cnt;
			}
			break;
		case ' ':
		case '\t':
			break;
		default:
			cnt++;
			break;
		}
		if( cnt >= size ) return cnt;
	}
	return cnt;
}

static int get_frame_data(
	int fd,
	int snsNum,
	int drvNum,
	int *array
){
	char linebuf[MAX_LINE_BUF];
	int cnty,value[32],getnum;

	for(cnty=0;cnty<drvNum;cnty++){
		if( !(local_gets(linebuf,MAX_LINE_BUF,fd)) ){
			DBGLOG("shtsc:Error(parse test spec):Illegal line(y=%d)\n",cnty);
			return 0;
		}
		getnum = sscanf(linebuf,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
				value+ 0,value+ 1,value+ 2,value+ 3,value+ 4,value+ 5,value+ 6,value+ 7,
				value+ 8,value+ 9,value+10,value+11,value+12,value+13,value+14,value+15,
				value+16,value+17,value+18,value+19,value+20,value+21,value+22,value+23,
				value+24,value+25,value+26,value+27,value+28,value+29,value+30,value+31);
		if( getnum < snsNum ){
			DBGLOG("shtsc:Error(parse test spec):Illegal value(x=%d,y=%d)\n",snsNum,cnty);
			return 0;
		}
		if( getnum > snsNum ){
			DBGLOG("shtsc:Error(parse test spec):Warnning:Line has many data(line=%d,num=%d)\n",cnty,getnum);
		}
		memcpy(array+cnty*snsNum,value,sizeof(int)*snsNum);
	}
	return 1;
}

int *ts_average;
int *ts_diff;
int *ts_sigma;
static int get_testspec(
	int fd,
	int *snsNum,
	int *drvNum
){
	char linebuf[MAX_LINE_BUF];
	int setfl=0;
	*snsNum = 0;
	*drvNum = 0;
	while(local_gets(linebuf,MAX_LINE_BUF,fd)){
		if( !strncmp(linebuf,"average",7) ){
			if( !(setfl&0x01) || !get_frame_data(fd,*snsNum,*drvNum,ts_average) ){
				DBGLOG("shtsc:Error(parse test spec):illegal test spec(average)\n");
				return 0;
			}
			setfl |= 8;
		}
		else if( !strncmp(linebuf,"diff",4) ){
			if( !(setfl&0x01) || !get_frame_data(fd,*snsNum,*drvNum,ts_diff) ){
				DBGLOG("shtsc:Error(parse test spec):illegal test spec(diff)\n");
				return 0;
			}
			setfl |= 4;
		}
		else if( !strncmp(linebuf,"sigma",5) ){
			if( !(setfl&0x01) || !get_frame_data(fd,*snsNum,*drvNum,ts_sigma) ){
				DBGLOG("shtsc:Error(parse test spec):illegal test spec(sigma)\n");
				return 0;
			}
			setfl |= 2;
		}
		else if( !strncmp(linebuf,"size",4) ){
			if(!local_gets(linebuf,MAX_LINE_BUF,fd)){
				DBGLOG("shtsc:Error(parse test spec):illegal size(test spec)\n");
				return 0;
			}
			if( sscanf(linebuf,"%d,%d",snsNum,drvNum) != 2 ){
				DBGLOG("shtsc:Error(parse test spec):illegal size(test spec)\n");
				return 0;
			}
			if( *snsNum * *drvNum > MAX_FRAME_SIZE ){
				DBGLOG("shtsc:Error(parse test spec):illegal size(test spec (%d,%d))\n",*snsNum,*drvNum);
				return 0;
			}
			ts_average = kzalloc(*snsNum * *drvNum * sizeof(int), GFP_KERNEL);
			ts_diff = kzalloc(*snsNum * *drvNum * sizeof(int), GFP_KERNEL);
			ts_sigma = kzalloc(*snsNum * *drvNum * sizeof(int), GFP_KERNEL);
			if( !ts_average || !ts_diff || !ts_sigma ){
				DBGLOG("shtsc:Error:malloc error(%d))\n",*snsNum * *drvNum);
				return 0;
			}
			setfl |= 1;
		}
	}
	if( setfl != 0xF ){
		DBGLOG("shtsc:Error(parse test spec):Some spec can't be found in test spec file.");
		return 0;
	}
	return 1;
}

/*************************************************************************//**
 * Issue get panel param command and get result to buffer
 * @retval 0:success
 ****************************************************************************/
int shtsc_CMD_GetPanelParam(
	void *ts,		/**<[in] pointor to struct shtsc_i2c or struct shtsc_spi */
	unsigned char *buf	/**<[in] 4 byte buffer for result */
){
	int ret;
	char cmdbin_getpanelparam[]={0xD8,0x00,0x00,0x06};
	ret = issue_command(ts, cmdbin_getpanelparam, sizeof(cmdbin_getpanelparam));
	if( !ret ){
		memcpy(buf,CommandResultBuf+2,6-2);
	}
	return ret;
}

/*************************************************************************//**
 * Issue exec calibration command 
 * @retval 0:success
 ****************************************************************************/
int shtsc_CMD_ExecCalibration(
	void *ts,		/**<[in] pointor to struct shtsc_i2c or struct shtsc_spi */
	int calfactor,	/**<[in] IIR factor for calibration */
	int caltime		/**<[in] calibraiont time(ms) 1~1000 */
){
	int ret;
	char cmdbin_execcalib[]={0x0F,0x00,0x03,0x00,0x00,0x00,0x00};
	cmdbin_execcalib[4] = calfactor&0xFF;
	cmdbin_execcalib[5] = caltime&0xFF;
	cmdbin_execcalib[6] = (caltime>>8)&0xFF;
	ret = issue_command(ts, cmdbin_execcalib, sizeof(cmdbin_execcalib));
	return ret;
}

/*************************************************************************//**
 * Issue get property command and get result to buffer
 * @retval 0:success
 ****************************************************************************/
int shtsc_CMD_GetProperty(
	void *ts,		/**<[in] pointor to struct shtsc_i2c or struct shtsc_spi */
	unsigned char *buf	/**<[in] 17 byte buffer for result */
){
	int ret;
	char cmdbin_getproperty[]={0xE0,0x00,0x00,0x11};
	ret = issue_command(ts, cmdbin_getproperty, sizeof(cmdbin_getproperty));
	if( !ret ){
		memcpy(buf,CommandResultBuf+2,17-2);
	}
	return ret;
}

static ssize_t shtsc_proc_paneltest_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int fd = -1;
	mm_segment_t old_fs;
	int drvNum,snsNum,snsNum2;
	int cnty,cntx,cntfrm,cntidx;
	short *pDCMap[SHTSC_TEST_FRAME_NUM];
	char cmdbin_setdcmapread[]={0xD7,0x00,0x01,0x00,0x01};
	char cmdbin_setregtable[]={0x06,0x00,0x04,0x00,0x00,0x12,0x6E,0x21};
	long *average=NULL;
	long long *variance=NULL;
	char retstr[256];
	int numretstr=0;
	unsigned char resultbuf[4];
	unsigned char *calibbuf=NULL;
	int cnt;
	unsigned char intmask[2];
	int remain,bufidx=8;
	int address=CALIBRATION_DATA_SRAM_ADDRESS;
	int datasize=0;

#ifdef DEBUG_SHTSC
	DBGLOG("shtsc-baseline-test:enter spec func(%p,%zd,%llu)\n",user_buf,count,*ppos);
#endif
	/* return for indication of EOF */
	if( *ppos ) return 0;

	intmask[0] = ReadOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK0);
	intmask[1] = ReadOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK1);

	/* Read test spec file */
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fd = sys_open("/sdcard/test.txt", O_RDONLY, 0);
	if( fd < 0 ){
		set_fs(old_fs);
		numretstr += sprintf(retstr+numretstr, "Error:spec file open failure(fd=%d)\n",fd);
		goto errExit;
	}
	if( !get_testspec(fd,&snsNum,&drvNum) ){
		sys_close(fd);
		set_fs(old_fs);
		numretstr += sprintf(retstr+numretstr, "Error:parse spec file\n");
		goto errExit;
	}
	sys_close(fd);
	set_fs(old_fs);
#ifdef DEBUG_SHTSC
	DBGLOG("shtsc-baseline-test:size(%d,%d)\n",snsNum,drvNum);
#endif

	/* get panel param */
	if( shtsc_CMD_GetPanelParam(g_ts, resultbuf) ){
		numretstr += sprintf(retstr+numretstr, "Error:command issue(get panel param)\n");
		goto errExit;
	}
	snsNum = resultbuf[1];
	drvNum = resultbuf[2];
	if( snsNum * drvNum > MAX_FRAME_SIZE || snsNum != resultbuf[1] || drvNum != resultbuf[2] ){
		numretstr += sprintf(retstr+numretstr, "Error:panel param error size specsheet=(%d,%d),fw=(%d,%d)\n",snsNum,drvNum,resultbuf[1],resultbuf[2]);
		goto errExit;
	}
	if( snsNum&1 ) snsNum2 = snsNum+1;
	else snsNum2 = snsNum;

	/* Allocate memory for DCMap */
	memset(pDCMap,0,sizeof(pDCMap));
	for( cntx=0 ; cntx<SHTSC_TEST_FRAME_NUM ; cntx++ ){
		pDCMap[cntx] = kzalloc(snsNum * drvNum * sizeof(short), GFP_KERNEL);
		if( !pDCMap[cntx] ) break;
	}
	if( cntx < SHTSC_TEST_FRAME_NUM ){
		numretstr += sprintf(retstr+numretstr, "Error:kzmalloc failure(frm=%d)\n",cntx);
		goto errExit;
	}
	average = kzalloc(snsNum * drvNum * sizeof(long), GFP_KERNEL);
	variance = kzalloc(snsNum * drvNum * sizeof(long long), GFP_KERNEL);
	if( !average || !variance ){
		numretstr += sprintf(retstr+numretstr, "Error:kzmalloc failure\n");
		goto errExit;
	}

	g_ts->disable_touchreport = true;
	/* Calib bypass */
	if( issue_command(g_ts, cmdbin_setregtable, sizeof(cmdbin_setregtable)) ){
		numretstr += sprintf(retstr+numretstr, "Error:command issue(calib bypass)\n");
		goto errExit;
	}

	/* Enable DCMapRead */
	dcmap[0] = 0;
	INIT_COMPLETION(g_ts->dcmap_completion);
	if( issue_command(g_ts, cmdbin_setdcmapread, sizeof(cmdbin_setdcmapread)) ){
		numretstr += sprintf(retstr+numretstr, "Error:command issue(enable dcmap read)\n");
		goto errExit;
	}

	for( cnty=0 ; ; ){
		if( !wait_for_completion_timeout(&g_ts->dcmap_completion,HZ) ){
			numretstr += sprintf(retstr+numretstr, "Error:dcmap timeout(frame=%d)\n",cnty);
			cnty = -1;
		}
		else{
			if( snsNum != dcmap[0] || drvNum != dcmap[1] ){
				numretstr += sprintf(retstr+numretstr, "Error:dcmap illegal size (sns=%d,drv=%d)-(sns=%d,drv=%d)\n",snsNum,drvNum,dcmap[0],dcmap[1]);
				cnty = -1;
			}
			if( dcmap[0]*dcmap[1] <= MAX_FRAME_SIZE ){
				memcpy(pDCMap[cnty],dcmap+4,snsNum * drvNum *sizeof(short) );
				cnty++;
			}
		}
		dcmap[0] = 0;
		if( cnty >= SHTSC_TEST_FRAME_NUM || cnty == -1 ){
			/* Disable DCMapRead */
			cmdbin_setdcmapread[4] = 0x00;
			issue_command2(g_ts, cmdbin_setdcmapread, sizeof(cmdbin_setdcmapread));
			SetIndicator(g_ts, 0x10);/* dcmap read complete */
			break;
		}
		INIT_COMPLETION(g_ts->dcmap_completion);
		SetIndicator(g_ts, 0x10);/* dcmap read complete */
	}
	// wait
	if( WaitAsync(g_ts) ){
		numretstr += sprintf(retstr+numretstr, "Warnning:command issue(disable dcmap read)\n");
	}
	/* calib bypass disable */
	cmdbin_setregtable[7] = 0x01;
	if( issue_command(g_ts, cmdbin_setregtable, sizeof(cmdbin_setregtable)) ){
		numretstr += sprintf(retstr+numretstr, "Warnning:command issue(disable calib bypass)\n");
	}

	if( cnty == -1 ){
		goto errExit;
	}

	memset(average,0,snsNum * drvNum * sizeof(long));
	/* calculate sum and check value range */
	for( cntfrm=0 ; cntfrm<SHTSC_TEST_FRAME_NUM ; cntfrm++ ){
		cntidx = 0;
		for( cnty=0 ; cnty<drvNum ; cnty++ ){
			for( cntx=0 ; cntx<snsNum ; cntx++ ){
				if( pDCMap[cntfrm][cntidx] > ts_average[cntidx] + ts_diff[cntidx] ||
				    pDCMap[cntfrm][cntidx] < ts_average[cntidx] - ts_diff[cntidx] ){
					numretstr += sprintf(retstr+numretstr, "CheckError:range over(x=%d,y=%d,frm=%d) value=%d range(%d,%d)\n",cntx,cnty,cntfrm,pDCMap[cntfrm][cntidx],ts_average[cntidx] - ts_diff[cntidx],ts_average[cntidx] + ts_diff[cntidx]);
					goto errExit;
				}
				average[cntidx] += pDCMap[cntfrm][cntidx];
				cntidx++;
			}
		}
	}

#if 0
	/* Print average of dcmap to kernel log */
	cntidx = 0;
	for( cnty=0 ; cnty<drvNum ; cnty++ ){
		for( cntx=0 ; cntx<snsNum ; cntx++ ){
			long temp = average[cntidx]*256/SHTSC_TEST_FRAME_NUM;
			printk("% 6ld.%02lx,",temp>>8,(temp>0)?(temp&0xFF):(-temp&0xFF));
			cntidx++;
		}
		printk("\n");
	}
	printk("\n");
#endif

	/* calculate variance */
	memset(variance,0,snsNum * drvNum * sizeof(long long));
	for( cntfrm=0 ; cntfrm<SHTSC_TEST_FRAME_NUM ; cntfrm++ ){
		cntidx = 0;
		for( cnty=0 ; cnty<drvNum ; cnty++ ){
			for( cntx=0 ; cntx<snsNum ; cntx++ ){
				long temp;
				temp = pDCMap[cntfrm][cntidx]*SHTSC_TEST_FRAME_NUM - average[cntidx];
				variance[cntidx] += (long long)temp * temp;
				cntidx++;
			}
		}
	}

	/* check sigma */
	cntidx = 0;
	for( cnty=0 ; cnty<drvNum ; cnty++ ){
		for( cntx=0 ; cntx<snsNum ; cntx++ ){
			long long specval;
			specval = (long long)ts_sigma[cntidx] * ts_sigma[cntidx] * SHTSC_TEST_FRAME_NUM * SHTSC_TEST_FRAME_NUM * SHTSC_TEST_FRAME_NUM;
			if( specval < variance[cntidx] ){
				numretstr += sprintf(retstr+numretstr, "CheckError:sigma over(x=%d,y=%d),val=%lld,spec=%lld\n",cntx,cnty,specval,variance[cntidx]);
				goto errExit;
			}
			cntidx++;
		}
	}

#if 0
	/* Print sigma to kernel log */
	printk(KERN_INFO "shtsc:sigma\n");
	cntidx = 0;
	for( cnty=0 ; cnty<drvNum ; cnty++ ){
		for( cntx=0 ; cntx<snsNum ; cntx++ ){
			unsigned long temp = (unsigned long)variance[cntidx]/SHTSC_TEST_FRAME_NUM;
			temp = int_sqrt(temp);
			temp = temp*256/SHTSC_TEST_FRAME_NUM;
			printk("% 6ld.%02lx,",temp>>8,(temp>0)?(temp&0xFF):(-temp&0xFF));
			cntidx++;
		}
		printk("\n");
	}
	printk("\n");
#endif

	numretstr += sprintf(retstr+numretstr, "OK(PanelCheck):No error\n");

	/* allocate memory for calibration data */
	calibbuf = kzalloc(CALIBRATION_DATA_FLASH_SIZE, GFP_KERNEL);
	if( !calibbuf ){
		numretstr += sprintf(retstr+numretstr, "Error(Calib):malloc failure\n");
		ret = simple_read_from_buffer(user_buf, count, ppos, retstr, numretstr);
		return ret;
	}

	// mask everything
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
	/* Check flash area for calib */
	flash_read(g_ts,CALIBRATION_DATA_FLASH_ADDRESS,CALIBRATION_DATA_FLASH_SIZE,calibbuf);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, intmask[0]);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, intmask[1]);
	for( cnt=0 ; cnt<CALIBRATION_DATA_FLASH_SIZE ; cnt++ ){
		if( calibbuf[cnt] != 0xFF ) break;
	}
	if( cnt < CALIBRATION_DATA_FLASH_SIZE ){
		//numretstr += sprintf(retstr+numretstr, "Error(Calib):flash area for calib has data\n");
		goto errExit;
	}

	/* calibraion(calfactor=0,time=100ms) */
	if( shtsc_CMD_ExecCalibration(g_ts, 0,100) ){
		//numretstr += sprintf(retstr+numretstr, "Error(Calib):command issue(exec calibration(cal=0,time=100))\n");
		goto errExit;
	}
	/* calibraion(calfactor=7,time=1000ms) */
	if( shtsc_CMD_ExecCalibration(g_ts, 7,1000) ){
		//numretstr += sprintf(retstr+numretstr, "Error(Calib):command issue(exec calibration(cal=7,time=1000))\n");
		goto errExit;
	}
	/* set system state to sleep */
	if( shtsc_CMD_SetSystemState(g_ts, CMD_STATE_SLEEP) ){
		//numretstr += sprintf(retstr+numretstr, "Error(Calib):command issue(set system state(sleep))\n");
		goto errExit;
	}

	
	/* ReadSRAM */
	/* disable interrupt */
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
	/* Change bank to 5 */
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_BANK, (unsigned char)0x05);
	remain = snsNum2 * drvNum * 4;
	while(remain){
		int remainLocal;
		/* Set SRAM Address */
		WriteOneByte(g_ts, 0x06, (unsigned char)(address&0xFF));
		WriteOneByte(g_ts, 0x07, (unsigned char)((address>>8)&0xFF));
		remainLocal = remain;
		if( remainLocal > 120 ) remainLocal = 120;
#ifdef DEBUG_SHTSC
		DBGLOG("shtsc-updatecalib-test:info sram read address=%08x size=%d\n",address,remainLocal);
#endif
		ReadMultiBytes(g_ts,0x08,remainLocal,calibbuf+bufidx);
		remain -= remainLocal;
		address += remainLocal;
		bufidx += remainLocal;
	}
	
	/* build calib data */
	{
		unsigned long checksum=0;
		int cnt;
		/* Data number */
		calibbuf[0] = (bufidx&0xFF);
		calibbuf[1] = ((bufidx>>8)&0xFF);
		/* Type */
		calibbuf[2] = 0;
		/* Sense data number */
		calibbuf[3] = snsNum;
		/* Drive data number */
		calibbuf[4] = drvNum;
		/* Calculate check sum */
		for( cnt=0 ; cnt<bufidx ; cnt++ ){
			checksum += calibbuf[cnt];
		}
		calibbuf[bufidx+0] = ((checksum>>0)&0xFF);
		calibbuf[bufidx+1] = ((checksum>>8)&0xFF);
		calibbuf[bufidx+2] = ((checksum>>16)&0xFF);
	}
	datasize = (bufidx+3);
	
	/* Write calibration data to flash */
	if( flash_write_page_shtsc(g_ts,CALIBRATION_DATA_FLASH_ADDRESS/FLASH_PAGE_SIZE,calibbuf) ){
		//numretstr += sprintf(retstr+numretstr, "Error(Calib):flash write error\n");
		goto errExit;
	}
	if( datasize > FLASH_PAGE_SIZE ){
		if( flash_write_page_shtsc(g_ts,CALIBRATION_DATA_FLASH_ADDRESS/FLASH_PAGE_SIZE+1,calibbuf+FLASH_PAGE_SIZE) ){
			//numretstr += sprintf(retstr+numretstr, "Error(Calib):flash write error\n");
			goto errExit;
		}
	}
	
	/* Verify */
	if( flash_verify_page_shtsc(g_ts,CALIBRATION_DATA_FLASH_ADDRESS/FLASH_PAGE_SIZE,calibbuf) ){
		//numretstr += sprintf(retstr+numretstr, "Error(Calib):flash verify error\n");
		goto errExit;
	}
	if( datasize > FLASH_PAGE_SIZE ){
		if( flash_verify_page_shtsc(g_ts,CALIBRATION_DATA_FLASH_ADDRESS/FLASH_PAGE_SIZE+1,calibbuf+FLASH_PAGE_SIZE) ){
			//numretstr += sprintf(retstr+numretstr, "Error(Calib):flash verify error\n");
			goto errExit;
		}
	}
	/* set system state to idle */
	if( shtsc_CMD_SetSystemState(g_ts, CMD_STATE_IDLE) ){
		//numretstr += sprintf(retstr+numretstr, "Warnning(Calib):command issue(set system state(idle))\n");
	}
	//numretstr += sprintf(retstr+numretstr, "OK:Update calib area\n");

errExit:
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, intmask[0]);
	WriteOneByte(g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, intmask[1]);

	g_ts->disable_touchreport = false;
	for( cntx=0 ; cntx<SHTSC_TEST_FRAME_NUM ; cntx++ ){
		if( pDCMap[cntx] ) kfree(pDCMap[cntx]);
	}
	if( ts_average ) {kfree(ts_average);ts_average=NULL;}
	if( ts_diff ) {kfree(ts_diff);ts_diff=NULL;}
	if( ts_sigma ) {kfree(ts_sigma);ts_sigma=NULL;}
	if( average ) kfree(average);
	if( variance ) kfree(variance);
	if( calibbuf ) kfree( calibbuf );
	ret = simple_read_from_buffer(user_buf, count, ppos, retstr, numretstr);
	return ret;
}


static const struct file_operations shtsc_proc_paneltest_fops = {
	.read =  shtsc_proc_paneltest_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t shtsc_proc_getproperty_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char retstr[256];
	int numretstr=0;
	unsigned char resultbuf[17];

#ifdef DEBUG_SHTSC
	DBGLOG("shtsc-getproperty-test:enter spec func(%p,%zd,%llu)\n",user_buf,count,*ppos);
#endif
	/* return for indication of EOF */
	if( shtsc_CMD_GetProperty(g_ts, resultbuf) ){
		numretstr += sprintf(retstr+numretstr, "Error:command issue\n");
	}
	else{
		numretstr += sprintf(retstr+numretstr, "FirmwareVersion=%02x%02x%02x%02x\nParameterVersion=%02x%02x%02x%02x\n",
			resultbuf[3],resultbuf[2],resultbuf[1],resultbuf[0],
			resultbuf[11],resultbuf[10],resultbuf[9],resultbuf[8]);
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, retstr, numretstr);
	return ret;
}

static const struct file_operations shtsc_proc_getproperty_fops = {
	.read =  shtsc_proc_getproperty_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};


#define SHTSC_FW_SIZE	(44*1024)

static ssize_t shtsc_proc_firmup_func(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char argstr[16];
	//int numretstr=0;
	unsigned char resultbuf[17];
	int fd = -1;
	mm_segment_t old_fs;
	char *fwbuf=NULL;
	int fwsize=0;

#ifdef DEBUG_SHTSC
	DBGLOG("shtsc-firmup:enter fwup func(%p,%zd,%llu)\n",user_buf,count,*ppos);
#endif
	ret = simple_write_to_buffer(argstr,16,ppos,user_buf,count);

#ifdef DEBUG_SHTSC
	DBGLOG("shtsc-firmup:input=%d,%s\n",ret,argstr);
#endif

	/* allocate temp memory for fw */
	fwbuf = kzalloc(SHTSC_FW_SIZE, GFP_KERNEL);
	if( !fwbuf ){
#ifdef DEBUG_SHTSC
		DBGLOG("shtsc-firmup:error malloc failure\n");
#endif
		return 0;
	}

	/* Read fw file */
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fd = sys_open("/sdcard/fw.bin", O_RDONLY, 0);
	if( fd < 0 ){
		set_fs(old_fs);
		kfree(fwbuf);
#ifdef DEBUG_SHTSC
		DBGLOG("shtsc-firmup:error open fw file\n");
#endif
		return ret;
	}
	fwsize = sys_read(fd,fwbuf,SHTSC_FW_SIZE);
	sys_close(fd);
	set_fs(old_fs);
	if( fwsize != SHTSC_FW_SIZE ){
		kfree(fwbuf);
#ifdef DEBUG_SHTSC
		DBGLOG("shtsc-firmup:error read fw file(%d)\n",fwsize);
#endif
		return ret;
	}
#ifdef DEBUG_SHTSC
	DBGLOG("shtsc-firmup:file fw ver=%02x%02x%02x%02x,param ver=%02x%02x%02x%02x\n",
	        fwbuf[0x23+5],fwbuf[0x22+5],fwbuf[0x21+5],fwbuf[0x20+5],
	        fwbuf[fwsize-8],fwbuf[fwsize-9],fwbuf[fwsize-10],fwbuf[fwsize-11] );
#endif

	if( argstr[0] == '0' ){
		/* check fw version */
		if( shtsc_CMD_GetProperty(g_ts, resultbuf) ){
#ifdef DEBUG_SHTSC
			DBGLOG("shtsc-firmup:error command(get property) failure\n");
#endif
			kfree(fwbuf);
			return ret;
		}
#ifdef DEBUG_SHTSC
		DBGLOG("shtsc-firmup:device fw ver=%02x%02x%02x%02x,param ver=%02x%02x%02x%02x\n",
		        resultbuf[3],resultbuf[2],resultbuf[1],resultbuf[0],
			resultbuf[11],resultbuf[10],resultbuf[9],resultbuf[8]);
#endif
		if( resultbuf[0] == fwbuf[0x20+5] && resultbuf[1] == fwbuf[0x21+5] && resultbuf[2] == fwbuf[0x22+5] && resultbuf[3] == fwbuf[0x23+5] &&
		    resultbuf[8] == fwbuf[fwsize-11] && resultbuf[9] == fwbuf[fwsize-10] && resultbuf[10] == fwbuf[fwsize-9] && resultbuf[11] == fwbuf[fwsize-8] ){
#ifdef DEBUG_SHTSC
			DBGLOG("shtsc-firmup:check firm/param version are same as fw file\n");
#endif
			kfree(fwbuf);
			return ret;
		}
	}
#if 1
	/* set system state to sleep */
	if( shtsc_CMD_SetSystemState(g_ts, CMD_STATE_SLEEP) ){
#ifdef DEBUG_SHTSC
		DBGLOG("shtsc-firmup:erroor command issue set system state(sleep)\n");
#endif
		kfree(fwbuf);
		return ret;
	}
	update_flash(g_ts,fwbuf,fwsize);
#endif

#ifdef DEBUG_SHTSC
	DBGLOG("shtsc-firmup:exit fwup func\n");
#endif

	kfree(fwbuf);
	return ret;
}

static const struct file_operations shtsc_proc_firmup_fops = {
	.write =  shtsc_proc_firmup_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

struct proc_dir_entry *proc_entry_tp  = NULL;
static int init_shtsc_proc(void)
{
	int ret = 0;
	struct proc_dir_entry *proc_entry_node  = NULL;
	proc_entry_tp = proc_mkdir("touchpanel", NULL);
	if( proc_entry_tp == NULL ){
		ret = -ENOMEM;
		printk(KERN_INFO"shtsc:error Couldn't create TP proc entry\n");
	}
	proc_entry_node = proc_create( "baseline_test", 0444, proc_entry_tp, &shtsc_proc_paneltest_fops);
	if(proc_entry_node == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"shtsc: Couldn't create proc entry\n");
	}
	proc_entry_node = proc_create( "get_property", 0444, proc_entry_tp, &shtsc_proc_getproperty_fops);
	if(proc_entry_node == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"shtsc: Couldn't create proc entry\n");
	}
	proc_entry_node = proc_create( "fw_update", 0222, proc_entry_tp, &shtsc_proc_firmup_fops);
	if(proc_entry_node == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"shtsc: Couldn't create proc entry\n");
	}
	
	return ret;
}



#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
static int dev_open(struct inode *inode, struct file *filp)
{
#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "shtsc dev_open\n");
#endif
  return 0;
}

static int dev_release(struct inode *inode, struct file *filp)
{
#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "shtsc dev_release\n");
#endif
  return 0;
}

#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)

static int dev_open(struct inode *inode, struct file *filp)
{
  struct shtsc_spi *ts = spi_get_drvdata(g_spi);

  if(!ts)
    return -EFAULT;

  return 0;
}

static int dev_release(struct inode *inode, struct file *filp)
{
  return 0;
}
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

static const struct file_operations dev_fops = {
  .owner = THIS_MODULE,
  .open = dev_open,
  .release = dev_release,
  .read = dev_read,
  //	.write = dev_write,
  .unlocked_ioctl = dev_ioctl,
};

static struct miscdevice shtsc_miscdev = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = SHTSC_DRIVER_NAME, // should be "/dev/shtsc"
  .fops = &dev_fops,
};

#if defined(CONFIG_TOUCHSCREEN_SHTSC_I2C)
static struct i2c_device_id shtsc_i2c_idtable[] = {
  { SHTSC_DRIVER_NAME, 0 },
  { }
};
#ifdef CONFIG_OF
static struct of_device_id shtsc_match_table[] = {
  { .compatible = "sharp,shtsc_i2c",},
  { },
};
#else
#define shtsc_match_table NULL
#endif

MODULE_DEVICE_TABLE(i2c, shtsc_i2c_idtable);

static struct i2c_driver shtsc_i2c_driver = {
  .driver		= {
    .owner	= THIS_MODULE,
    .name	= SHTSC_DRIVER_NAME,
    .of_match_table = shtsc_match_table,//2014.10.16 added
  },
  .id_table	= shtsc_i2c_idtable,
  .probe	= shtsc_i2c_probe,
  .remove	= __devexit_p(shtsc_i2c_remove),
};

static int __init shtsc_init(void)
{
  int ret;
  ret = misc_register(&shtsc_miscdev);
  if (ret) {
    printk(KERN_INFO "%s(%d): misc_register returns %d. Failed.\n", __FILE__, __LINE__, ret);
  }
#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "%s(%d): loaded successfully\n", __FILE__, __LINE__);
#endif

  return i2c_add_driver(&shtsc_i2c_driver);
}
static void __exit shtsc_exit(void)
{
  misc_deregister(&shtsc_miscdev);
  i2c_del_driver(&shtsc_i2c_driver);
}

#elif defined (CONFIG_TOUCHSCREEN_SHTSC_SPI)

static struct spi_driver shtsc_spi_driver = {
  .driver		= {
    .owner	= THIS_MODULE,
    .name	= SHTSC_DRIVER_NAME,
  },
  .probe	= shtsc_spi_probe,
  .remove	= __devexit_p(shtsc_spi_remove),
};

static int __init shtsc_init(void)
{
  int ret;
  ret = misc_register(&shtsc_miscdev);
  if (ret) {
    printk(KERN_INFO "%s(%d): misc_register returns %d. Failed.\n", __FILE__, __LINE__, ret);
  }
#if defined(DEBUG_SHTSC)
  printk(KERN_INFO "%s(%d): loaded successfully\n", __FILE__, __LINE__);
#endif

  return spi_register_driver(&shtsc_spi_driver);
}
static void __exit shtsc_exit(void)
{
  misc_deregister(&shtsc_miscdev);
  spi_unregister_driver(&shtsc_spi_driver);
}
#else // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)
error
#endif // (CONFIG_TOUCHSCREEN_SHTSC_UNKNOWNIF??)

module_init(shtsc_init);
module_exit(shtsc_exit);

MODULE_DESCRIPTION("shtsc SHARP Touchscreen controller Driver");
MODULE_LICENSE("GPL v2");
