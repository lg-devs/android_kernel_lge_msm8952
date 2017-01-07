#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/pm_qos.h>

#include "broadcast_dmb_typedef.h"
#include "broadcast_dmb_drv_ifdef.h"
#include "broadcast_fc8300.h"
#include "fci_types.h"
#include "fci_oal.h"
#include "bbm.h"
#include "fc8300_drv_api.h"
#include "fc8300_isr.h"
#include "fci_ringbuffer.h"

/*#define _NOT_USE_WAKE_LOCK_*/

#if defined (CONFIG_ARCH_MSM8996) || defined (CONFIG_MACH_MSM8952_B5_JP_KDI)
#define FEATURE_DMB_USE_INTERNAL_LDO
#undef FEATURE_DMB_USE_REGULATOR
#undef FEATURE_DMB_USE_XO
#else
#undef FEATURE_DMB_USE_INTERNAL_LDO
#define FEATURE_DMB_USE_REGULATOR
#define FEATURE_DMB_USE_XO
#endif

#define FEATURE_DMB_USE_PINCTRL

#define FC8300_NAME        "broadcast_isdbt"
#define RING_BUFFER_SIZE    (188 * 320 * 20)
static struct task_struct *isdbt_kthread = NULL;
struct fci_ringbuffer        RingBuffer;
//static wait_queue_head_t isdbt_isr_wait;
//u8 irq_error_cnt;
static u8 isdbt_isr_sig=0;
static u8 isdbt_isr_start=0;
u32 totalTS=0;
u32 totalErrTS=0;

u32 yk_irq_cnt = 0;

enum ISDBT_MODE{
    ISDBT_POWERON       = 0,
    ISDBT_POWEROFF        = 1,
    ISDBT_DATAREAD        = 2
};
enum ISDBT_MODE driver_mode = ISDBT_POWEROFF;

u8 static_ringbuffer[RING_BUFFER_SIZE];
static DEFINE_MUTEX(ringbuffer_lock);

#ifdef FEATURE_DMB_USE_PINCTRL
#include <linux/pinctrl/consumer.h>
#endif

#ifdef FEATURE_DMB_USE_REGULATOR
#include <linux/regulator/consumer.h>
#endif

/* proto type declare */
static int broadcast_spi_probe(struct spi_device *spi);
static int broadcast_spi_remove(struct spi_device *spi);
static int broadcast_spi_suspend(struct spi_device *spi, pm_message_t mesg);
static int broadcast_spi_resume(struct spi_device *spi);

int data_callback(u32 hDevice, u8 bufid, u8 *data, int len);

struct broadcast_fc8300_ctrl_data
{
    int                     pwr_state;
    struct wake_lock        wake_lock;
    struct spi_device*      spi_dev;
    struct i2c_client*      pclient;
    struct platform_device* pdev;
    uint32                  enable_gpio;
    uint32                  reset_gpio;
    /* Interrupt pin is not used in TSIF mode */
    uint32                  interrupt_gpio;
#ifdef FEATURE_DMB_USE_XO
    struct clk*             xo_clk;
#endif
#ifdef FEATURE_DMB_USE_REGULATOR
    struct regulator*       vdd_io;
#endif
};

static struct broadcast_fc8300_ctrl_data  IsdbCtrlInfo;

struct i2c_client*	FCI_GET_I2C_DRIVER(void)
{
	return IsdbCtrlInfo.pclient;
}
struct spi_device*    FCI_GET_SPI_DRIVER(void)
{
    return IsdbCtrlInfo.spi_dev;
}

static Device_drv device_fc8300 = {
    &broadcast_fc8300_drv_if_power_on,
    &broadcast_fc8300_drv_if_power_off,
    &broadcast_fc8300_drv_if_open,
    &broadcast_fc8300_drv_if_close,
    &broadcast_fc8300_drv_if_set_channel,
    &broadcast_fc8300_drv_if_resync,
    &broadcast_fc8300_drv_if_detect_sync,
    &broadcast_fc8300_drv_if_get_sig_info,
    &broadcast_fc8300_drv_if_get_ch_info,
    &broadcast_fc8300_drv_if_get_dmb_data,
    &broadcast_fc8300_drv_if_reset_ch,
    &broadcast_fc8300_drv_if_user_stop,
    &broadcast_fc8300_drv_if_select_antenna,
    &broadcast_fc8300_drv_if_isr,
    &broadcast_fc8300_drv_if_read_control,
    &broadcast_fc8300_drv_if_get_mode,
};

#ifdef FEATURE_DMB_USE_PINCTRL
static int isdbt_pinctrl_init(void)
{
	struct pinctrl *tdmb_pinctrl;
	struct pinctrl_state *gpio_state_suspend;

	tdmb_pinctrl = devm_pinctrl_get(&(IsdbCtrlInfo.pdev->dev));


	if(IS_ERR_OR_NULL(tdmb_pinctrl)) {
		pr_err("%s: Getting pinctrl handle failed\n", __func__);
		return -EINVAL;
	}
	gpio_state_suspend
	 = pinctrl_lookup_state(tdmb_pinctrl, "isdbt_gpio");

	 if(IS_ERR_OR_NULL(gpio_state_suspend)) {
	 	pr_err("%s: [dtv]Failed to get the suspend state pinctrl handle\n", __func__);
	 	return -EINVAL;
	}

	if(pinctrl_select_state(tdmb_pinctrl, gpio_state_suspend)) {
		pr_err("%s: [dtv]error on pinctrl_select_state DTV GPIOs\n", __func__);
		return -EINVAL;
	}
	else {
		printk("%s: success to set pinctrl_select_state for DTV GPIOss\n", __func__);
	}

	return 0;
}
#endif

#ifdef FEATURE_DMB_USE_REGULATOR
static int broadcast_isdbt_set_regulator(int onoff)
{
    int rc = -1;

    if(!IsdbCtrlInfo.vdd_io)
    {
        IsdbCtrlInfo.vdd_io = devm_regulator_get(&(IsdbCtrlInfo.pdev->dev),"isdbt_vdd_io");
        if(IS_ERR(IsdbCtrlInfo.vdd_io)){
            dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv]could not get regulator %s \n","isdbt_vdd_io-supply");
            return rc;
        }
    }

    if(onoff)
    {
        rc = regulator_set_voltage(IsdbCtrlInfo.vdd_io,1800000,2950000);
        if(rc)
        {
            dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv] could not set regulator ret=%d \n",rc);
            return rc;
        }

        rc = regulator_enable(IsdbCtrlInfo.vdd_io);
        if(rc)
        {
            dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv]could not enable regulator ret=%d\n",rc);
            return rc;
        }
    }
    else
    {
        rc = regulator_disable(IsdbCtrlInfo.vdd_io);
        if(rc)
        {
            dev_err(&(IsdbCtrlInfo.pdev->dev), "[dtv]could not disable regulator %d \n",rc);
            return rc;
        }
    }

    printk("%s: success to set pm8994 voltage control(mode:%d)\n", __func__,onoff);
    return rc;
}

#endif

int fc8300_power_on(void)
{
    int i =0;
    int rc = OK;
    //struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	if(IsdbCtrlInfo.pwr_state != 1)
	{
#ifndef _NOT_USE_WAKE_LOCK_
        wake_lock(&IsdbCtrlInfo.wake_lock);
#endif
        while(driver_mode == ISDBT_DATAREAD)
        {
            msWait(100);
            print_log(NULL, "[FC8300]ISDBT_DATARREAD mode i=(%d)\n", i);

            if(i++>5)
                break;
        }

        gpio_set_value(IsdbCtrlInfo.reset_gpio, 0);

#ifdef FEATURE_DMB_USE_INTERNAL_LDO
        gpio_set_value(IsdbCtrlInfo.enable_gpio, 1);
#endif

        mdelay(3);
        gpio_set_value(IsdbCtrlInfo.reset_gpio, 1);
        mdelay(2);
        gpio_direction_input(IsdbCtrlInfo.interrupt_gpio);

#ifdef FEATURE_DMB_USE_REGULATOR
        broadcast_isdbt_set_regulator(1);
#endif
#ifdef FEATURE_DMB_USE_XO
        if(IsdbCtrlInfo.xo_clk!= NULL) {
            printk("[dtv]fc8300_power on clk enable\n");
            rc = clk_prepare_enable(IsdbCtrlInfo.xo_clk);
        }
        mdelay(2);
#endif

        driver_mode = ISDBT_POWERON;
	}
	else
	{
		printk("[dtv]aready on!! \n");
	}

       bbm_com_ts_callback_register(0, data_callback);
	IsdbCtrlInfo.pwr_state = 1;

	return rc;
}

int fc8300_is_power_on()
{
	return (int)IsdbCtrlInfo.pwr_state;
}

int fc8300_power_off(void)
{
    driver_mode = ISDBT_POWEROFF;
	if(IsdbCtrlInfo.pwr_state == 0)
	{
		print_log(NULL, "Isdb_fc8300_power is immediately off\n");
		return OK;
	}
	else
	{
#ifdef FEATURE_DMB_USE_XO
            if(IsdbCtrlInfo.xo_clk != NULL)
            {
                clk_disable_unprepare(IsdbCtrlInfo.xo_clk);
            }
#endif

#ifdef FEATURE_DMB_USE_REGULATOR
            broadcast_isdbt_set_regulator(0);
#endif

            printk("[dtv]Isdb_fc8300_power_off\n");
            gpio_direction_output(IsdbCtrlInfo.interrupt_gpio, 0);
            mdelay(1);

#ifdef FEATURE_DMB_USE_INTERNAL_LDO
            gpio_set_value(IsdbCtrlInfo.enable_gpio, 0);
#endif

            mdelay(1);
            gpio_set_value(IsdbCtrlInfo.reset_gpio, 0);
            mdelay(5);
	}

#ifndef _NOT_USE_WAKE_LOCK_
	wake_unlock(&IsdbCtrlInfo.wake_lock);
#endif

    bbm_com_ts_callback_deregister();
	IsdbCtrlInfo.pwr_state = 0;

	return OK;
}

static int broadcast_Isdb_config_gpios(void)
{
    int rc = OK;
    int err_count = 0;

#ifdef FEATURE_DMB_USE_INTERNAL_LDO
    IsdbCtrlInfo.enable_gpio = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node,
        "fci_spi_fc8300,en-gpio" ,0);

    rc =  gpio_request(IsdbCtrlInfo.enable_gpio, "ISDBT_EN");
    if(rc < 0) {
        err_count++;
        printk("[dtv]Failed ISDBT_EN GPIO request\n");
    }
    udelay(10);
    gpio_direction_output(IsdbCtrlInfo.enable_gpio, 0);
#endif

    IsdbCtrlInfo.reset_gpio = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node,
        "fci_spi_fc8300,reset-gpio" ,0);

    rc =  gpio_request(IsdbCtrlInfo.reset_gpio, "ISDBT_RESET");
    if(rc < 0) {
        err_count++;
        printk("[dtv]Failed ISDBT_RESET GPIO request\n");
    }
    udelay(10);

    IsdbCtrlInfo.interrupt_gpio = of_get_named_gpio(IsdbCtrlInfo.pdev->dev.of_node,
        "fci_spi_fc8300,irq-gpio" ,0);

    rc =  gpio_request(IsdbCtrlInfo.interrupt_gpio, "ISDBT_INT");
    if(rc < 0) {
        err_count++;
        printk("[dtv]Failed ISDBT_INT GPIO request\n");
    }

    udelay(10);
    gpio_direction_output(IsdbCtrlInfo.reset_gpio, 0);
    udelay(10);
    gpio_direction_output(IsdbCtrlInfo.interrupt_gpio, 0);

    printk("[dtv]enable_gpio =(%d), reset_gpio =(%d), interrupt_gpio=(%d)",
            IsdbCtrlInfo.enable_gpio, IsdbCtrlInfo.reset_gpio, IsdbCtrlInfo.interrupt_gpio);

    return rc;
}

unsigned int fc8300_get_ts(void *buf, unsigned int size)
{
    s32 avail;
    ssize_t len, total_len = 0;

    if ( fci_ringbuffer_empty(&RingBuffer) )
    {
        //print_log(hInit, "return fci_ringbuffer_empty EWOULDBLOCK\n");
        return 0;
    }

    //mutex_lock(&ringbuffer_lock);
    avail = fci_ringbuffer_avail(&RingBuffer);

    if (size >= avail)
        len = avail;
    else
        len = size - (size % 188);

    total_len = fci_ringbuffer_read_user(&RingBuffer, buf, len);
    //mutex_unlock(&ringbuffer_lock);

    return total_len;
}

#if 0
static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
    //print_log(NULL, "[FC8300] isdbt_irq MODE %d  ERR : %d, SIGCNT : %d\n", driver_mode, irq_error_cnt, isdbt_isr_sig);
    //bbm_byte_write(NULL, DIV_MASTER, 0x26, 0x00);

    if((driver_mode == ISDBT_POWEROFF)||(isdbt_isr_start)) {
        print_log(0, "fc8300 isdbt_irq : abnormal Interrupt occurred fc8300 power off state.cnt : %d\n", irq_error_cnt);
        irq_error_cnt++;
        isdbt_isr_start = 0;
    }
    else {
        isdbt_isr_sig++;
        wake_up(&isdbt_isr_wait);
    }

    return IRQ_HANDLED;
}
#else
static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
    //print_log(NULL, "[FC8300] isdbt_irq MODE %d  ERR : %d, SIGCNT : %d\n", driver_mode, irq_error_cnt, isdbt_isr_sig);
    //bbm_byte_write(NULL, DIV_MASTER, 0x26, 0x00);

    if(driver_mode == ISDBT_POWERON)
    {
        driver_mode = ISDBT_DATAREAD;
        bbm_com_isr(NULL);
        driver_mode = ISDBT_POWERON;
    }

    return IRQ_HANDLED;
}
#endif

int data_callback(u32 hDevice, u8 bufid, u8 *data, int len)
{
    int i;
    unsigned long ts_error_count = 0;
    totalTS +=(len/188);

    for(i=0;i<len;i+=188)
    {
        if((data[i+1]&0x80)||data[i]!=0x47) {
            ts_error_count++;
        }
    }

    if(ts_error_count > 0)
    {
        totalErrTS += ts_error_count;
        print_log(NULL, "[FC8300] data_callback totalErrTS : %d, len : %d\n", totalErrTS, len);
    }
    //mutex_lock(&ringbuffer_lock);
    if(fci_ringbuffer_free(&RingBuffer) < len )
        FCI_RINGBUFFER_SKIP(&RingBuffer, len);

    fci_ringbuffer_write(&RingBuffer, data, len);
    //wake_up_interruptible(&(RingBuffer.queue));

    //mutex_unlock(&ringbuffer_lock);

    return 0;
}

#if 0
static int isdbt_thread(void *hDevice)
{
    struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

    #ifdef MTK_FTRACE_TEST
    unsigned long previous_time = 0;
    unsigned int isdbt_ftrace_mode = 0;
    #endif

    //struct sched_param param = { .sched_priority = 0 };
    //struct sched_param param = { .sched_priority = 98 };
    //sched_setscheduler ( current, SCHED_FIFO, &param );

    set_user_nice(current, -20);

    print_log(hInit, "isdbt_kthread enter\n");

    bbm_com_ts_callback_register((u32)hInit, data_callback);

    init_waitqueue_head(&isdbt_isr_wait);

    while(1)
    {
        wait_event_interruptible(isdbt_isr_wait, isdbt_isr_sig || kthread_should_stop());

#if 0
        if (irq_error_cnt >= 1){
            print_log(0, "fc8300 isdbt_irq : abnormal Interrupt occurred fc8300 power off state.cnt : %d\n", irq_error_cnt);
            irq_error_cnt = 0;
        }
#endif
        //print_log(NULL, "[FC8300] isdbt_thread mode %d sigcnt : %d\n", driver_mode, isdbt_isr_sig);

        //isdbt_thread_check_cnt++;
        if(driver_mode == ISDBT_POWERON)
        {
            driver_mode = ISDBT_DATAREAD;
            //print_log(hInit, "[YK] isr ++\n");
            bbm_com_isr(NULL);
            //print_log(hInit, "[YK] isr --\n");
            driver_mode = ISDBT_POWERON;
        }

        if(isdbt_isr_sig > 0)
        {
            #ifdef MTK_FTRACE_TEST
            extern void tracing_off(void);
            isdbt_isr_sig--;
            if(isdbt_isr_sig > 0) {
                if(isdbt_ftrace_mode == 0)
                {
                    if(isdbt_isr_sig > 1)
                    {
                        tracing_off();
                        isdbt_ftrace_mode = 1;
                    }
                    else if(isdbt_isr_sig)
                    {
                        if((previous_time) && ((unsigned long)jiffies - previous_time) < 300) //3sec)
                        {
                            tracing_off();
                            isdbt_ftrace_mode = 1;
                        }
                        previous_time = (unsigned long)jiffies;
                    }
                }
                isdbt_isr_sig = 0;
            }
            #else
            isdbt_isr_sig--;
            if(isdbt_isr_sig > 0) {
                //print_log(0, "[YK] yk_irq_cnt: %d, yk_read_cnt: %d,  cnt_gap: %d, isdbt_isr_sig: %d\n", yk_irq_cnt, yk_read_cnt, yk_irq_cnt-yk_read_cnt, isdbt_isr_sig);
                isdbt_isr_sig = 0;
            }
            #endif
        }

         if (kthread_should_stop())
            break;
    }

    bbm_com_ts_callback_deregister();

    print_log(NULL, "isdbt_kthread exit\n");

    return 0;
}
#endif

void fci_irq_disable(void)
{
    isdbt_isr_sig = 0;
    print_log(NULL, "[fc8300] fci_irq_disable %d\n", IsdbCtrlInfo.spi_dev->irq);
}

void fci_irq_enable(void)
{
    //enable_irq(IsdbCtrlInfo.spi_dev->irq);
    isdbt_isr_sig = 0;
    isdbt_isr_start = 1;
    print_log(NULL, "[fc8300] fci_irq_enable %d\n", IsdbCtrlInfo.spi_dev->irq);
}

void broadcast_fci_ringbuffer_flush(void)
{
    fci_ringbuffer_flush(&RingBuffer);
}

#if 0
static int broadcast_dmb_fc8300_check_chip_id(void)
{
    int rc = ERROR;

    rc = fc8300_power_on();
    rc = bbm_com_hostif_select(NULL, BBM_SPI);
    rc |= bbm_com_probe(NULL, DIV_BROADCAST);

    print_log(NULL, "broadcast_dmb_fc8300_check_chip_id ret(%d)\n", rc);

    fc8300_power_off();

    return rc;
}
#endif

static int broadcast_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
    print_log(NULL, "[fc8300] Suspend\n");
    return 0;
}

static int broadcast_spi_resume(struct spi_device *spi)
{
    print_log(NULL, "[fc8300] Resume\n");
    return 0;
}

static int broadcast_spi_remove(struct spi_device *spi)
{
    print_log(NULL, "[fc8300] broadcast_spi_remove \n");

    free_irq(IsdbCtrlInfo.interrupt_gpio, NULL);

     if(isdbt_kthread)
    {
        kthread_stop(isdbt_kthread);
        isdbt_kthread = NULL;
    }
     bbm_com_hostif_deselect(NULL);

    wake_lock_destroy(&IsdbCtrlInfo.wake_lock);

    return 0;
}

static int broadcast_spi_probe(struct spi_device *spi)
{
    int rc;

    spi->mode             = SPI_MODE_0;
    spi->bits_per_word    = 8;
    spi->max_speed_hz     = (40000*1000);
    //spi->max_speed_hz     = (50000*1000);
    //spi->max_speed_hz     = (33000*1000);
    IsdbCtrlInfo.pdev = to_platform_device(&spi->dev);

    print_log(NULL, "[fc8300] broadcast_spi_probe++\n");

    rc = spi_setup(spi);
    if (rc) {
        print_log(NULL, "[fc8300] Spi setup error(%d)\n", rc);
        return rc;
    }

    IsdbCtrlInfo.spi_dev = spi;

#ifndef _NOT_USE_WAKE_LOCK_
    wake_lock_init(&IsdbCtrlInfo.wake_lock, WAKE_LOCK_SUSPEND,
                    dev_name(&spi->dev));
#endif
    fci_ringbuffer_init(&RingBuffer, &static_ringbuffer[0], RING_BUFFER_SIZE);

#if 0
    if(broadcast_dmb_fc8300_check_chip_id() != OK) {
        rc = ERROR;
    }
#endif

     //if (!isdbt_kthread)
    //{
        //print_log(NULL, "kthread run\n");
        //isdbt_kthread = kthread_run(isdbt_thread, NULL, "isdbt_thread");
    //}
    broadcast_Isdb_config_gpios();

#ifdef FEATURE_DMB_USE_PINCTRL
    isdbt_pinctrl_init();
#endif

#if 0
    rc = request_irq(spi->irq, isdbt_irq
        , IRQF_ONESHOT | IRQF_DISABLED | IRQF_TRIGGER_FALLING, spi->dev.driver->name, &IsdbCtrlInfo);
#else
    rc = request_threaded_irq(spi->irq, NULL, isdbt_irq, IRQF_ONESHOT | IRQF_DISABLED | IRQF_TRIGGER_FALLING,
                       spi->dev.driver->name, &IsdbCtrlInfo);
#endif

    if (rc){
        print_log(NULL, "[fc8300] dmb request irq fail : %d\n", rc);
    }

    rc = broadcast_dmb_drv_start(&device_fc8300);
    if (rc) {
        print_log(NULL, "[fc8300] Failed to load (%d)\n", rc);
        rc = ERROR;
    }

    if (rc < 0)
        goto free_irq;

    fci_irq_disable(); /* Must disabled */

    print_log(NULL, "[fc8300] broadcast_spi_probe--\n");

    return 0;

free_irq:
    broadcast_spi_remove(IsdbCtrlInfo.spi_dev);

    return rc;
}

static struct of_device_id fci_spi_fc8300_table[] = {
    {
	.compatible = "fci,fc8300-spi", //Compatible node must match dts
    }, 
    { }
};

static struct spi_driver broadcast_spi_driver = {
	.driver = {
		.name = "isdbt",
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = fci_spi_fc8300_table,
	},

	.probe = broadcast_spi_probe,
	.suspend = broadcast_spi_suspend,
	.resume	= broadcast_spi_resume,
	.remove	= broadcast_spi_remove,
};

static int __init broadcast_dmb_fc8300_drv_init(void)
{
    int ret = 0;

    // Todo(add revision check rountine)

    if(broadcast_dmb_drv_check_module_init() != OK) {
        ret = ERROR;
        return ret;
    }

    ret = spi_register_driver(&broadcast_spi_driver);
    if (ret < 0)
        print_log(NULL, "[FC8300] SPI driver register failed\n");

    return ret;
}

static void __exit broadcast_dmb_fc8300_drv_exit(void)
{
    spi_unregister_driver(&broadcast_spi_driver);
}

#if 0
static int broadcast_Isdb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	int addr = 0;

#if defined (CONFIG_ARCH_MSM8992) || defined (CONFIG_ARCH_MSM8994) || defined (CONFIG_ARCH_MSM8996)
    printk("[dtv]broadcast_Isdb_i2c_probe client:0x%lX\n", (UDynamic_32_64)client);
#else
    printk("[dtv]broadcast_Isdb_i2c_probe client:0x%X\n", (UDynamic_32_64)client);
#endif

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		print_log(NULL, "need I2C_FUNC_I2C\n");
		return -ENODEV;
	}
    IsdbCtrlInfo.pdev = to_platform_device(&client->dev);

	/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
	addr = client->addr; //Slave Addr
	pr_err("[dtv] i2c Slaveaddr [%x] \n", addr);

	IsdbCtrlInfo.pclient = client;
	//i2c_set_clientdata(client, (void*)&IsdbCtrlInfo.pclient);

#ifdef FEATURE_DMB_USE_XO
    IsdbCtrlInfo.xo_clk = clk_get(&IsdbCtrlInfo.pclient->dev, "isdbt_xo");
    if(IS_ERR(IsdbCtrlInfo.xo_clk)){
        rc = PTR_ERR(IsdbCtrlInfo.xo_clk);
        dev_err(&IsdbCtrlInfo.pclient->dev, "[dtv]could not get clock\n");
        return rc;
    }
    /* We enable/disable the clock only to assure it works */
    rc = clk_prepare_enable(IsdbCtrlInfo.xo_clk);
    if(rc) {
        dev_err(&IsdbCtrlInfo.pclient->dev, "[dtv] could not enable clock\n");
        return rc;
    }
    clk_disable_unprepare(IsdbCtrlInfo.xo_clk);
#endif

#ifdef FEATURE_DMB_USE_PINCTRL
    isdbt_pinctrl_init();
#endif

    /* Config GPIOs */
    broadcast_Isdb_config_gpios();

#ifdef FEATURE_DMB_USE_REGULATOR
    broadcast_isdbt_set_regulator(1);
    broadcast_isdbt_set_regulator(0);
#endif

#ifndef _NOT_USE_WAKE_LOCK_
	wake_lock_init(&IsdbCtrlInfo.wake_lock, WAKE_LOCK_SUSPEND,
					dev_name(&client->dev));
#endif

#if defined (CONFIG_ARCH_MSM8992) || defined (CONFIG_ARCH_MSM8994) || defined (CONFIG_ARCH_MSM8996)
    fc8300_power_on();
    tunerbb_drv_fc8300_read_chip_id();
    fc8300_power_off();
#endif

	return rc;
}

static int broadcast_Isdb_i2c_remove(struct i2c_client* client)
{
	int rc = 0;

	print_log(NULL, "[%s]\n", __func__);
#ifndef _NOT_USE_WAKE_LOCK_
	wake_lock_destroy(&IsdbCtrlInfo.wake_lock);
#endif
	memset((unsigned char*)&IsdbCtrlInfo, 0x0, sizeof(struct broadcast_fc8300_ctrl_data));
	//TcpalDeleteSemaphore(&fc8300DrvSem);
	return rc;
}

static int broadcast_Isdb_i2c_suspend(struct i2c_client* client, pm_message_t mesg)
{
	int rc = 0;
	print_log(NULL, "[%s]\n", __func__);
	return rc;
}

static int broadcast_Isdb_i2c_resume(struct i2c_client* client)
{
	int rc = 0;
	print_log(NULL, "[%s]\n", __func__);
	return rc;
}

static const struct i2c_device_id isdbt_fc8300_id[] = {
/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
	{"tcc3535_i2c",	0},
/* taew00k.kang added for Device Tree Structure 2013-06-04 [end] */
	{},
};

MODULE_DEVICE_TABLE(i2c, isdbt_fc8300_id);


/* taew00k.kang added for Device Tree Structure 2013-06-04 [start] */
static struct of_device_id tcc3535_i2c_table[] = {
{ .compatible = "telechips,tcc3535-i2c",}, //Compatible node must match dts
{ },
};
/* taew00k.kang added for Device Tree Structure 2013-06-04 [end] */

static struct i2c_driver broadcast_Isdb_driver = {
	.driver = {
		.name = "tcc3535_i2c",
		.owner = THIS_MODULE,
		.of_match_table = tcc3535_i2c_table,
	},
	.probe = broadcast_Isdb_i2c_probe,
	.remove	= broadcast_Isdb_i2c_remove,
	.id_table = isdbt_fc8300_id,
};

int broadcast_dmb_fc8300_drv_init(void)
{
	int rc;
	print_log(NULL, "[%s]\n", __func__);
	rc = broadcast_dmb_drv_start(&device_fc8300);
	if (rc)
	{
		print_log(NULL, "failed to load\n");
		return rc;
	}
	print_log(NULL, "[%s add i2c driver]\n", __func__);
	rc = i2c_add_driver(&broadcast_Isdb_driver);
	print_log(NULL, "broadcast_add_driver rc = (%d)\n", rc);
	return rc;
}

static void __exit broadcast_dmb_fc8300_drv_exit(void)
{
	i2c_del_driver(&broadcast_Isdb_driver);
}
#endif // 0

module_init(broadcast_dmb_fc8300_drv_init);
module_exit(broadcast_dmb_fc8300_drv_exit);
MODULE_DESCRIPTION("broadcast_dmb_drv_init");
MODULE_LICENSE("FCI");
