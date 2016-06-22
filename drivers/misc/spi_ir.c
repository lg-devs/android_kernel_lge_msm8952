#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_OF
static struct of_device_id sdc_spi_ir_table[] = {
	{ .compatible = "sdc,spi-ir",},
	{ },
};
#else
#define sdc_spi_ir_table NULL
#endif /* CONFIG_OF */

/* SPI IR features */
#undef SPI_IR_DATA_DUMP        /* Data Dump for Debugging */
#undef SPI_IR_DEBUG_MSG        /* Message for Debugging */
#undef SPI_IR_MARK_ERROR       /* Message for Mark Error */

/* Define Frequency & Time */
#define FREQ_1K_HZ              (1000)
#define FREQ_1M_HZ              (1000000)
#define TIME_1SEC_MS            (1000)
#define TIME_1SEC_US            (1000000)

/* Target Dependent WORD */
#define WORD_SIZE               (2)
#define INTER_WORD_DELAY        (2)

/* TX Buffer Size */
#define TX_BUFF_SIZE            (64*1024)

#define EXT_LDO_ENABLE          1
#define EXT_LDO_DISABLE         0

struct spi_ir_device {
	struct spi_device       *spi;

	struct bin_attribute    bin;
	struct bin_attribute    state;

	struct spi_message      spi_msg;
	struct spi_transfer     spi_xfer;

	u32                     carrier_hz;
	int                     duty_rate;

	unsigned int            tx_size;
	u8                      *tx_buf;

	int                     ext_ldo_ctrl;
	int                     gpio_ldo_ctrl;
};

static struct spi_ir_device *g_spi_device = NULL;

static unsigned int carrier_frequency[] = {
	26000, /* 25961 */
	26500, /* 26455 */
	27000, /* 26969 */
	27500, /* 27503 */
	28000, /* 28058 */
	28500, /* 28637 */
	29000, /* 28935 */
	29500, /* 29551 */
	30000, /* 29869 */
	30300, /* 30303 */
	30500, /* 30525 */
	30750, /* 30864 */
	31000, /* 30864 */
	31250, /* 31211 */
	31500, /* 31566 */
	31750, /* 31928 */
	32000, /* 31928 */
	32250, /* 32300 */
	32500, /* 32680 */
	32750, /* 32680 */
	33000, /* 32938 */
	33250, /* 33069 */
	33500, /* 33467 */
	33750, /* 33875 */
	34000, /* 33875 */
	34250, /* 34294 */
	34500, /* 34294 */
	34750, /* 34722 */
	35000, /* 35162 */
	35250, /* 35162 */
	35500, /* 35613 */
	35750, /* 35613 */
	36000, /* 36075 */
	36250, /* 36075 */
	36500, /* 36550 */
	36750, /* 36550 */
	37000, /* 37037 */
	37250, /* 37037 */
	37500, /* 37538 */
	37750, /* 37538 */
	38000, /* 38052 */
	38250, /* 38052 */
	38500, /* 38580 */
	38750, /* 38580 */
	39000, /* 39124 */
	39250, /* 39124 */
	39500, /* 39683 */
	39750, /* 39683 */
	40000, /* 40016 */
	40500, /* 40355 */
	41000, /* 41051 */
	41500, /* 41408 */
	42000, /* 42141 */
	42500, /* 42517 */
	43000, /* 42900 */
	43500, /* 43687 */
	44000, /* 44092 */
	44500, /* 44504 */
	45000, /* 44924 */
	45500, /* 45351 */
	46000, /* 45788 */
	46500, /* 46685 */
	47000, /* 47148 */
	47500, /* 47619 */
	48000, /* 48100 */
	48500, /* 48591 */
	49000, /* 49092 */
	49500, /* 49603 */
	50000, /* 50125 */
	50500, /* 50659 */
	51000, /* 51203 */
	51500, /* 51760 */
	52000, /* 51760 */
	52500, /* 52329 */
	53000, /* 52910 */
	53500, /* 53505 */
	54000, /* 54113 */
	54500, /* 54735 */
	55000, /* 54735 */
	55500, /* 55371 */
	56000, /* 56022 */
	500000,
};

typedef enum {
	SPI_IR_STATUS_NONE        = '0',
	SPI_IR_STATUS_TRANSMIT    = 'A',
	SPI_IR_STATUS_ERROR       = 'F',
} spi_ir_status_t;

static spi_ir_status_t spi_ir_status = SPI_IR_STATUS_NONE;

static int spi_ir_write(struct spi_ir_device *spi_ir)
{
	struct spi_message *spi_msg = &spi_ir->spi_msg;
	struct spi_transfer *spi_xfer = &spi_ir->spi_xfer;
	int ret = -1;

	u8 *tx_buf = spi_ir->tx_buf;

	spi_message_init(spi_msg);

	spi_xfer->tx_buf = tx_buf;
	spi_xfer->len = (spi_ir->tx_size < TX_BUFF_SIZE)?spi_ir->tx_size:TX_BUFF_SIZE;
	spi_xfer->bits_per_word = 16;
	spi_xfer->speed_hz = spi_ir->carrier_hz;

	spi_message_add_tail(spi_xfer, spi_msg);

	ret = spi_sync(spi_ir->spi, spi_msg);

	return ret;
}


static ssize_t
spi_ir_bin_write(struct file *fp, struct kobject *kobj, struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev;
	struct spi_ir_device *spi_ir;
	int size;
	int i;

	unsigned int min_hz = 0xffffffff, min_val = 0xffffffff;
	unsigned int delta_hz, cur_hz, carrier_hz;

	dev = container_of(kobj, struct device, kobj);
	spi_ir = dev_get_drvdata(dev);

	if (off == 0 && count == sizeof(int)) {
		carrier_hz = *(u32 *)(buf+off);

		for (i = 0 ; i < ARRAY_SIZE(carrier_frequency) ; i++) {
			cur_hz = carrier_frequency[i];

			if (carrier_hz < cur_hz)
				delta_hz = cur_hz - carrier_hz;
			else
				delta_hz = carrier_hz - cur_hz;

			if (min_val > delta_hz) {
				min_val = delta_hz;
				min_hz = cur_hz;
			}
		}
		spi_ir->carrier_hz = min_hz;

#ifdef SPI_IR_DEBUG_MSG
		pr_err("Set Hz = %d\n", min_hz);
#endif
	} else if (count > 0) {
		memcpy(spi_ir->tx_buf+off, buf, count);
		size = (off+count)/WORD_SIZE;

		if (size > (TX_BUFF_SIZE))
			size = TX_BUFF_SIZE;

		if (count != PAGE_SIZE) {
			spi_ir->tx_size = size*WORD_SIZE;
			spi_ir_write(spi_ir);
#ifdef SPI_IR_DEBUG_MSG
		pr_err("off = %ld, count = %d, spi_ir->tx_size = %d\n", (long)off, (int)count, spi_ir->tx_size);
#endif
		}
	}

	return count;
}

static ssize_t
spi_ir_state_read(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	if(count > 2)
		count = 2;

	memcpy(buf, (char*)&spi_ir_status, count);
#ifdef SPI_IR_DEBUG_MSG
	pr_err("spi_ir_state_read %c\n", spi_ir_status);
#endif

	return count;
}

static struct bin_attribute spi_ir_bin = {
	.attr = {
		.name = "ir_data",
		.mode = S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP,
	},

	.size = TX_BUFF_SIZE,
	.write = spi_ir_bin_write,
};

static struct bin_attribute spi_ir_state = {
	.attr = {
		.name = "ir_state",
		.mode = S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP,
	},

	.size = 2,
	.read = spi_ir_state_read,
};

static int spi_ir_ldo_ctrl_init(struct device *dev, struct spi_ir_device *pdata)
{
	int err;
	struct device_node *dev_node = dev->of_node;

	err = of_property_read_u32(dev_node, "lge,ext-ldo-ctrl",
		&pdata->ext_ldo_ctrl);
	if (err == -EINVAL) {
		pdata->ext_ldo_ctrl = EXT_LDO_DISABLE;
	}

	if (pdata->ext_ldo_ctrl == EXT_LDO_ENABLE) {
		pdata->gpio_ldo_ctrl = of_get_named_gpio_flags(
			dev_node, "lge,gpio-ldo-ctrl", 0, NULL);
		pr_err("SPI IrRC using external LDO\n");
		if (gpio_is_valid(pdata->gpio_ldo_ctrl)) {
			err = gpio_request(pdata->gpio_ldo_ctrl, "irrc_ldo_ctrl");
			if (err) {
				pr_err("SPI request_gpio failed : %d\n", pdata->gpio_ldo_ctrl);
				return err;
			}
			gpio_direction_output(pdata->gpio_ldo_ctrl, 1);
		}
	} else {
		pdata->gpio_ldo_ctrl = EXT_LDO_DISABLE;
		pr_err("SPI IrRC not using external LDO\n");
	}

	return 0;
}

static int spi_ir_probe(struct spi_device *spi)
{
	struct spi_ir_device *spi_ir;

	u8 *tx_buf;
	int err;

	u32 carrier_hz = 38000;
	int duty_rate = 6; /* 33% */

	pr_err("%s\n", __func__);

	/* Allocate memory for spi_ir */
	spi_ir = kzalloc(sizeof(struct spi_ir_device), GFP_KERNEL);

	if (spi_ir == NULL) {
		pr_err("%s: spi_ir mem alloc failed\n", __func__);
		err = -ENOMEM;
		goto err_spi_device_alloc;
	}

	/* Allocate memory for transfer */
	tx_buf = kzalloc(TX_BUFF_SIZE, GFP_ATOMIC);

	if (tx_buf == NULL) {
		pr_err("%s: tx_buf mem alloc failed\n", __func__);
		err = -ENOMEM;
		goto err_free_spi_ir;
	}

	/* Setup SPI_IR Driver */
	spi_ir->spi = spi;

	spi_ir->tx_buf = tx_buf;

	/* SPI_MODE_0 = (0|0),// SPI_MODE_1 = (0|SPI_CPHA),
	   SPI_MODE_2 = (SPI_CPOL|0), // SPI_MODE_3 = (SPI_CPOL|SPI_CPHA)
	   SPI_MSB_FIRST */
	spi_ir->spi->mode = SPI_MODE_0;
	pr_err("%s: SPI Mode = %d\n", __func__, spi->mode);

	/* SPI BIT Size */
	spi_ir->spi->bits_per_word = WORD_SIZE*8;
	pr_err("%s: SPI bit = %d bit\n", __func__, spi_ir->spi->bits_per_word);

	err = spi_setup(spi_ir->spi);
	if (err < 0) {
		pr_err("%s: SPI Setup error (%d)\n", __func__, err);
		goto err_free_tx_buf;
	}

	spi_ir->carrier_hz = carrier_hz;
	pr_err("%s: sdc_spi_ir: Carrier Freauency = %d Hz\n", __func__, spi_ir->carrier_hz);
	spi_ir->duty_rate = duty_rate;
	pr_err("%s: sdc_spi_ir: Duty Rate = %d%%\n", __func__, duty_rate*100/18);

	/* Export SPI_IR Bin */
	spi_ir->bin = spi_ir_bin;
	err = sysfs_create_bin_file(&spi->dev.kobj, &spi_ir->bin);
	if (err < 0) {
		pr_err("%s: SPI Sysfs bin error (%d)\n", __func__, err);
		goto err_remove_bin_attr;//bin_attr;
	}

	/* Export SPI_IR State */
	spi_ir->state = spi_ir_state;
	err = sysfs_create_bin_file(&spi->dev.kobj, &spi_ir->state);
	if(err < 0) {
		pr_err("%s: SPI Sysfs state error (%d)\n", __func__, err);
		goto err_remove_state_attr;
	}

	err = spi_ir_ldo_ctrl_init(&spi->dev, spi_ir);
	if (err  < 0) {
		pr_err("%s: SPI ldo ctrl init fail (%d)\n", __func__, err);
		goto err_remove_state_attr;
	}

	g_spi_device = spi_ir;

	spi_set_drvdata(spi, spi_ir);

	return 0;

err_remove_state_attr:
	sysfs_remove_bin_file(&spi->dev.kobj, &spi_ir->state);

err_remove_bin_attr:
	sysfs_remove_bin_file(&spi->dev.kobj, &spi_ir->bin);

err_free_tx_buf:
	spi_ir->tx_buf = NULL;
	kfree(tx_buf);

err_free_spi_ir:
	kfree(spi_ir);

err_spi_device_alloc:
	spi_set_drvdata(spi, NULL);

	return err;
}

static int spi_ir_remove(struct spi_device *spi)
{
	struct spi_ir_device *spi_ir;
	spi_ir = spi_get_drvdata(spi);

	if (spi_ir->ext_ldo_ctrl == EXT_LDO_ENABLE) {
		gpio_free(spi_ir->gpio_ldo_ctrl);
	}

	sysfs_remove_bin_file(&spi->dev.kobj, &spi_ir->bin);
	sysfs_remove_bin_file(&spi->dev.kobj, &spi_ir->state);

	if (spi_ir->tx_buf) {
		kfree(spi_ir->tx_buf);
		spi_ir->tx_buf = 0;
	}

	kfree(spi_ir);
	spi_set_drvdata(spi, NULL);

	pr_err("%s\n", __func__);

	return 0;
}

static int spi_ir_suspend(struct spi_device *spi, pm_message_t mesg)
{
	gpio_direction_output(g_spi_device->gpio_ldo_ctrl, 0);
	return 0;
}

static int spi_ir_resume(struct spi_device *spi)
{
	gpio_direction_output(g_spi_device->gpio_ldo_ctrl, 1);
	return 0;
}

static struct spi_driver spi_ir_driver = {
	.driver = {
	.name = "sdc_spi_ir",
	.owner = THIS_MODULE,
	.of_match_table = sdc_spi_ir_table,
	},
	.probe = spi_ir_probe,
	.remove = spi_ir_remove,
	.suspend = spi_ir_suspend,
	.resume = spi_ir_resume,
};

static void __exit spi_ir_exit(void)
{
	spi_unregister_driver(&spi_ir_driver);
}

module_spi_driver(spi_ir_driver);
module_exit(spi_ir_exit);

MODULE_DESCRIPTION("SDC SPI IR");
MODULE_LICENSE("GPL v2");
