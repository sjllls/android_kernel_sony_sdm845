/*
 * Copyright (C) 2017 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#define ENABLE_LOGE
/*#define ENABLE_LOGD*/
/*#define ENABLE_LOGI*/

#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/stat.h>
#include <linux/kthread.h>
#include <linux/thermal.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/pm_runtime.h>
#include <cam_cci_dev.h>
#include <sony_camera.h>
#include <sony_camera_firmware_binary.h>

#ifdef ENABLE_LOGE
#define LOGE(f, a...)	pr_err("%s: " f, __func__, ##a)
#else
#define LOGE(f, a...)
#endif

#ifdef ENABLE_LOGD
#define LOGD(f, a...)	pr_debug("%s: " f, __func__, ##a)
#else
#define LOGD(f, a...)
#endif

#ifdef ENABLE_LOGI
#define LOGI(f, a...)	pr_info("%s: " f, __func__, ##a)
#else
#define LOGI(f, a...)
#endif

#define SONY_CAMERA_I2C_MAX_DATA_LEN		8192
#define SONY_CAMERA_NAME_LEN			8
#define SONY_CAMERA_MCLK_DEFAULT		8000000
#define SONY_CAMERA_MAX_RETRY_COUNT		5
#define SONY_CAMERA_FRONT_SENSOR_POWER_UP_WAIT	10
#define SONY_CAMERA_SPI_TX_MAX_LEN		(32 * 1024)	// 32K
#define SONY_CAMERA_SPI_RX_MAX_LEN		4096		// 4k

#define SONY_CAMERA_I2C_MAX_LEN			(256)
#define SONY_CAMERA_I2C_MAX_SIZE		(sizeof(struct cam_sensor_i2c_reg_array) * SONY_CAMERA_I2C_MAX_LEN)
#define SONY_CAMERA_SPI_HEADER_SIZE		(16)
#define SONY_CAMERA_SPI_PAYLOAD_SIZE		(8192)

#define SONY_CAMERA_DEV_NAME			"sony_camera"
#define SONY_CAMERA_THERMAL_NAME_0		"sony_camera_0"
#define SONY_CAMERA_THERMAL_NAME_1		"sony_camera_1"
#define SONY_CAMERA_THERMAL_NAME_2		"sony_camera_2"
#define SONY_CAMERA_THERMAL_NAME_3		"sony_camera_isp"
#define SONY_CAMERA_THERMAL_NUM			(3)

#define SONY_CAMERA_PINCTRL_STATE_SLEEP		"cam_suspend"
#define SONY_CAMERA_PINCTRL_STATE_DEFAULT	"cam_default"

#define SONY_CAMERA_GPIO_RESET			"SONY_CAMERA_RESET"
#define SONY_CAMERA_GPIO_IRQ_FATAL		"SONY_CAMERA_FATAL"
#define SONY_CAMERA_GPIO_IRQ_GP_STATUS		"SONY_CAMERA_GP_STATUS"
#define SONY_CAMERA_GPIO_IRQ_SPI_READY		"SONY_CAMERA_SPI_READY"
#define SONY_CAMERA_GPIO_IRQ_SOF		"SONY_CAMERA_SOF"

enum sony_camera_irq_type {
	SONY_CAMERA_IRQ_FATAL_EVENT			=  1,
	SONY_CAMERA_IRQ_GP_ASSERT_EVENT			=  2,
	SONY_CAMERA_IRQ_GP_DEASSERT_EVENT		=  3,
	SONY_CAMERA_IRQ_SPI_READY_ASSERT_EVENT		=  4,
	SONY_CAMERA_IRQ_SPI_READY_DEASSERT_EVENT	=  5,
	SONY_CAMERA_IRQ_SOF_EVENT			=  6,
	SONY_CAMERA_IRQ_EVENT_MAX			=  7,
};

enum sony_camera_cmd {
	SONY_CAM_VDIG	=  0,
	SONY_CAM_VIO	=  1,
	SONY_CAM_VANA	=  2,
	SONY_CAM_VAF	=  3,
	SONY_CAM_CLK	=  4,
	SONY_GPIO_RESET	=  5,
	SONY_I2C_WRITE	=  6,
	SONY_MCAM_VDIG	=  7,
	SONY_SCAM_VDIG	=  8,
	SONY_ISP_MEM	=  9,
	SONY_ISP_CORE	= 10,
	SONY_ISP_0P9	= 11,
	SONY_ISP_1P2	= 12,
	SONY_ISP_SPI	= 13,
	EXIT		= 14,
};

enum sony_camera_state {
	SONY_CAMERA_STATE_POWER_UP,
	SONY_CAMERA_STATE_POWER_DOWN,
	SONY_CAMERA_STATE_MAX
};

struct sony_camera_task_queue_cmd {
	struct list_head		list;
	const char			*irq_name;
	enum sony_camera_irq_type	irq_type;
	bool				cmd_used;
};

struct sony_camera_seq {
	enum sony_camera_cmd	cmd;
	int			val1;
	int			val2;
	int			wait;
};

struct sony_camera_module {
	const char		*name;
	struct sony_camera_seq	*seq_on;
	struct sony_camera_seq	*seq_off;
	uint32_t		i2c_freq_mode;
};

struct sony_camera_info {
	uint16_t			i2c_addr;
	uint16_t			eeprom_addr;
	int				eeprom_type;
	uint16_t			eeprom_max_len;
	uint16_t			use_spi;
	int				modules_num;
	struct sony_camera_module	*modules;
	const char			*default_module_name;
};

struct sony_camera_event_list_data {
	struct list_head		list;
	struct sony_camera_event_data	event_data;
};

struct sony_camera_event_sof_data32 {
	uint32_t		sof_count;
	struct compat_timeval	mono_timestamp;
};

struct sony_camera_event32 {
	uint32_t	type;
	union {
		struct sony_camera_event_sof_data32	sof_data;
		uint8_t					gp_status;
		uint8_t					err_code;
	} data;
};

struct sony_camera_spi_data32 {
	uint32_t	header_size;
	compat_uptr_t	header;
	uint32_t	payload_size;
	compat_uptr_t	payload;
};

struct sony_camera_data {
	// platform device
	uint32_t			id;
	struct platform_device		*p_dev;
	struct sony_camera_module	*module;
	bool				probe_done;
	// CCI
	struct cam_sensor_cci_client	cci_info;
	char				*i2c_data;
	// SPI
	struct spi_device		*spi_master;
	char				*tx_buf;
	char				*rx_buf;
	char				*spi_header;
	char				*spi_payload;
	// pin controll
	struct pinctrl			*pinctrl;
	struct pinctrl_state		*gpio_state_active;
	struct pinctrl_state		*gpio_state_suspend;
	struct gpio			*gpio_req_tbl;
	uint8_t				gpio_req_tbl_size;
	bool				gpio_requested;
	bool				has_hw_sof;
	// eeprom
	uint8_t				eeprom[SONY_CAMERA_MAX_EEPROM_DATA];
	uint16_t			eeprom_len;
	// lot id
	bool				lot_id_available;
	uint8_t				lot_id[20];
	// power
	struct regulator		*cam_vio;
	struct regulator		*cam_vana;
	struct regulator		*cam_vdig;
	struct regulator		*cam_vaf;
	struct regulator		*cam_clk;
	struct regulator		*cam_main_vdig;
	struct regulator		*cam_sub_vdig;
	struct regulator		*isp_mem;
	struct regulator		*isp_core;
	struct regulator		*isp_0p9;
	struct regulator		*isp_1p2;
	struct clk			*clk_handle;
	struct mutex			state_lock;
	enum sony_camera_state		state;
	// thermal
	struct mutex			thermal_lock;
	struct thermal_zone_device	*thermal_zone_dev[SONY_CAMERA_THERMAL_NUM];
	int32_t				thermal_sensor_temperature[SONY_CAMERA_THERMAL_NUM];
	int				thermal_ret_val;
	// Sensor SOF count
	spinlock_t			sof_lock;
	uint32_t			sof_count;
	uint8_t				gp_status;

	// User event
	spinlock_t			event_list_lock;
	wait_queue_head_t		event_wait_q;
	struct list_head		event_available;
	// Kernel event
	struct completion		spi_assert_complete;
	struct completion		spi_deassert_complete;
	struct completion		gp_assert_complete;
	spinlock_t			task_lock;
	struct task_struct		*sensor_task;
	wait_queue_head_t		wait_q;
	struct list_head		task_q;
	atomic_t			irq_cnt;
	uint8_t				taskq_idx;
	struct sony_camera_task_queue_cmd
			task_queue_cmd[SONY_CAMERA_TASKQ_SIZE];
	struct mutex			command_lock;
	uint32_t			open_count;
};

static int dev_id;
static struct cdev cdev;
static struct class *c = NULL;
static uint16_t sensor_num;
static struct platform_device *camera_device;
static struct class *camera_device_class;

static struct sony_camera_info camera_info[] = {
	{
	},
	{
	},
};

static struct sony_camera_data camera_data[] = {
	{
		.id = 0,
		.thermal_sensor_temperature = { 0, 0, 0 },
		.thermal_ret_val = -ENODEV,
		.has_hw_sof = 0,
		.tx_buf = NULL,
		.rx_buf = NULL,
	},
	{
		.id = 1,
		.thermal_sensor_temperature = { 0, 0, 0 },
		.thermal_ret_val = -ENODEV,
		.has_hw_sof = 0,
		.tx_buf = NULL,
		.rx_buf = NULL,
	},
};

static void sony_camera_hex_dump(const char *buf, uint32_t len)
{
	char linebuf[SONY_CAMERA_HEX_DUMP_LINE_LENGTH];
	uint32_t i = 0;
	uint32_t j = 0;

	memset(linebuf, 0, sizeof(linebuf));
	for (i = 0; i < len / SONY_CAMERA_HEX_DUMP_LINE_BYTES + 1; i++) {
		char *p = linebuf;

		p += scnprintf(p, 20, "%p: ", buf + i * SONY_CAMERA_HEX_DUMP_LINE_BYTES);
		for (j = 0; j < SONY_CAMERA_HEX_DUMP_LINE_BYTES &&
			(i * SONY_CAMERA_HEX_DUMP_LINE_BYTES + j < len); j++) {
			p += scnprintf(p, 4, "%02x ",
				(uint8_t)buf[i * SONY_CAMERA_HEX_DUMP_LINE_BYTES + j]);
		}
		LOGI("%s\n", linebuf);
	}
}

static int sony_camera_get_dt_gpio_req_tbl(struct device_node *of_node,
	struct sony_camera_data *camera_data)
{
	int32_t rc = 0, i = 0;
	uint32_t count = 0;
	uint16_t *gpio_array = NULL;
	int16_t gpio_array_size = 0;
	uint32_t *val_array = NULL;

	gpio_array_size = of_gpio_count(of_node);

	if (gpio_array_size <= 0)
		return 0;

	LOGI("%s %d gpio count %d\n", __func__, __LINE__, gpio_array_size);

	gpio_array = vzalloc(gpio_array_size * sizeof(uint16_t));
	if (!gpio_array)
		goto exit;

	for (i = 0; i < gpio_array_size; i++)
		gpio_array[i] = of_get_gpio(of_node, i);

	if (!of_get_property(of_node, "gpio-req-tbl-num", &count))
		return 0;

	count /= sizeof(uint32_t);
	if (!count) {
		LOGE("%s %d gpio-req-tbl-num 0\n", __func__, __LINE__);
		return 0;
	}

	val_array = vzalloc(count * sizeof(uint32_t));
	if (!val_array) {
		vfree(gpio_array);
		return -ENOMEM;
	}

	camera_data->gpio_req_tbl = vzalloc(count * sizeof(struct gpio));
	if (!camera_data->gpio_req_tbl) {
		rc = -ENOMEM;
		goto free_val_array;
	}
	camera_data->gpio_req_tbl_size = count;

	rc = of_property_read_u32_array(of_node, "gpio-req-tbl-num",
		val_array, count);
	if (rc) {
		LOGE("%s %d failed in reading gpio-req-tbl-num, rc = %d\n",
			__func__, __LINE__, rc);
		goto free_gpio_req_tbl;
	}

	for (i = 0; i < count; i++) {
		if (val_array[i] >= gpio_array_size) {
			LOGE("%s %d gpio req tbl index %d invalid\n",
				__func__, __LINE__, val_array[i]);
			goto free_gpio_req_tbl;
		}
		camera_data->gpio_req_tbl[i].gpio = gpio_array[val_array[i]];
	}

	rc = of_property_read_u32_array(of_node, "gpio-req-tbl-flags",
		val_array, count);
	if (rc) {
		LOGE("%s %d Failed in gpio-req-tbl-flags, rc %d\n",
			__func__, __LINE__, rc);
		goto free_gpio_req_tbl;
	}

	for (i = 0; i < count; i++)
		camera_data->gpio_req_tbl[i].flags = val_array[i];

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node,
			"gpio-req-tbl-label", i,
			&camera_data->gpio_req_tbl[i].label);
		if (rc) {
			LOGE("%s %d Failed rc %d\n", __func__, __LINE__, rc);
			goto free_gpio_req_tbl;
		}
	}

	vfree(val_array);
	vfree(gpio_array);

	return rc;

free_gpio_req_tbl:
	vfree(camera_data->gpio_req_tbl);
free_val_array:
	vfree(val_array);
	camera_data->gpio_req_tbl_size = 0;
	vfree(gpio_array);
exit:

	return rc;
}

static struct gpio *sony_camera_get_gpio_pin(
	struct sony_camera_data *camera_data, const char *label)
{
	uint32_t i = 0;
	uint32_t size = 0;
	struct gpio *gpio_tbl = NULL;

	if (camera_data == NULL || label == NULL) {
		LOGE("%s:%d invalid args:%p, %p\n", __func__,
			__LINE__, camera_data, label);
		return NULL;
	}
	gpio_tbl = camera_data->gpio_req_tbl;
	size = camera_data->gpio_req_tbl_size;
	for (i = 0; i < size; i++) {
		if (!strcmp(label, gpio_tbl->label))
			return gpio_tbl;
		gpio_tbl++;
	}
	return NULL;
}

static int sony_camera_info_deinit(uint32_t id)
{
	uint16_t i = 0;

	if (camera_info[id].modules) {
		for (i = 0; i < camera_info[id].modules_num; i++) {
			vfree(camera_info[id].modules[i].seq_on);
			vfree(camera_info[id].modules[i].seq_off);
		}
		vfree(camera_info[id].modules);
	}
	memset(&(camera_info[id]), 0, sizeof(struct sony_camera_info));

	return 0;
}

static int sony_camera_info_init(struct platform_device *p_dev,
	uint32_t id)
{
	int rc = 0;
	int count = 0;
	uint16_t i = 0;
	uint16_t j = 0;
	uint32_t val_u32[4] = {0};
	struct device_node *of_node = p_dev->dev.of_node;
	struct device_node *of_node_modules = NULL;
	struct device_node *of_node_modules_power_off = NULL;
	struct device_node *of_node_modules_power_on = NULL;
	const int8_t *power_order_name = NULL;

	rc = of_property_read_u32(of_node, "eeprom_addr", &val_u32[0]);
	if (rc < 0) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		goto fail;
	}
	camera_info[id].eeprom_addr = val_u32[0];

	rc = of_property_read_u32(of_node, "eeprom_type", &val_u32[0]);
	if (rc < 0) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		goto fail;
	}
	camera_info[id].eeprom_type = val_u32[0];

	rc = of_property_read_u32(of_node, "eeprom_max_len", &val_u32[0]);
	if (rc < 0) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		goto fail;
	}
	camera_info[id].eeprom_max_len = val_u32[0];

	rc = of_property_read_u32(of_node, "use_spi", &val_u32[0]);
	if (rc < 0) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		goto fail;
	}
	camera_info[id].use_spi = val_u32[0];

	rc = of_property_read_u32(of_node, "cci-master", &val_u32[0]);
	if (rc < 0) {
		camera_data[id].cci_info.cci_i2c_master = MASTER_0;
	} else {
		camera_data[id].cci_info.cci_i2c_master = val_u32[0];
	}

	rc = sony_camera_get_dt_gpio_req_tbl(of_node, &camera_data[id]);
	if (rc < 0) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		goto fail;
	}

	if (sony_camera_get_gpio_pin(&camera_data[id],
		SONY_CAMERA_GPIO_IRQ_SOF) == NULL) {
		camera_data[id].has_hw_sof = 0;
	} else {
		camera_data[id].has_hw_sof = 1;
	}

	count = of_property_count_strings(of_node, "module_names");
	if (count < 0) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		rc = -EFAULT;
		goto fail;
	}
	camera_info[id].modules_num = count;

	camera_info[id].modules = vzalloc(sizeof(struct sony_camera_module) * count);
	if (!camera_info[id].modules) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < camera_info[id].modules_num; i++) {
		rc = of_property_read_string_index(of_node,
			"module_names", i,
			(const char **)(&camera_info[id].modules[i].name));
		LOGD("%s name[%d] = %s\n", __func__, i,
			camera_info[id].modules[i].name);
		if (rc < 0) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			goto fail;
		}
	}

	for (i = 0; i < camera_info[id].modules_num; i++) {
		of_node_modules = of_find_node_by_name(of_node,
			camera_info[id].modules[i].name);
		if (!of_node_modules) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			rc = -EFAULT;
			goto fail;
		}

		rc = of_property_read_u32(of_node_modules,
			"i2c_freq_mode",
			&camera_info[id].modules[i].i2c_freq_mode);

		if (rc < 0) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			goto fail;
		}

		of_node_modules_power_off = of_find_node_by_name(
			of_node_modules, "power_off");
		if (!of_node_modules_power_off) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			rc = -EFAULT;
			goto fail;
		}

		count = of_property_count_strings(of_node_modules_power_off,
			"commands");
		if (count < 0) {
			LOGE("%s failed power off commands 0\n", __func__);
			rc = -EFAULT;
			goto fail;
		}
		camera_info[id].modules[i].seq_off = vzalloc(sizeof(struct sony_camera_seq) * count);
		if (!camera_info[id].modules[i].seq_off) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			goto fail;
		}

		for (j = 0; j < count; j++) {
			rc = of_property_read_string_index(
				of_node_modules_power_off,
				"commands", j,
				(const char **)(&power_order_name));
			if (rc < 0) {
				LOGE("%s failed %d\n", __func__, __LINE__);
				goto fail;
			}

			rc = of_property_read_u32_array(
				of_node_modules_power_off, power_order_name,
				&val_u32[0], 4);
			if (rc < 0) {
				LOGE("%s failed %d\n", __func__, __LINE__);
				goto fail;
			}
			camera_info[id].modules[i].seq_off[j].cmd = val_u32[0];
			camera_info[id].modules[i].seq_off[j].val1 = val_u32[1];
			camera_info[id].modules[i].seq_off[j].val2 = val_u32[2];
			camera_info[id].modules[i].seq_off[j].wait = val_u32[3];
		}

		of_node_modules_power_on = of_find_node_by_name(of_node_modules,
			"power_on");
		if (!of_node_modules_power_on) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			rc = -EFAULT;
			goto fail;
		}

		count = of_property_count_strings(of_node_modules_power_on,
			"commands");
		if (count < 0) {
			LOGE("%s failed power on commands 0\n", __func__);
			rc = -EFAULT;
			goto fail;
		}

		camera_info[id].modules[i].seq_on = vzalloc(sizeof(struct sony_camera_seq) * count);
		if (!camera_info[id].modules[i].seq_on) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			goto fail;
		}

		for (j = 0; j < count; j++) {
			rc = of_property_read_string_index(
				of_node_modules_power_on,
				"commands", j,
				(const char **)(&power_order_name));
			if (rc < 0) {
				LOGE("%s failed %d j=%d count=%d\n", __func__, __LINE__, j, count);
				goto fail;
			}

			rc = of_property_read_u32_array(
				of_node_modules_power_on, power_order_name,
				&val_u32[0], 4);
			if (rc < 0) {
				LOGE("%s failed %d j=%d count=%d\n", __func__, __LINE__, j, count);
				goto fail;
			}
			camera_info[id].modules[i].seq_on[j].cmd = val_u32[0];
			camera_info[id].modules[i].seq_on[j].val1 = val_u32[1];
			camera_info[id].modules[i].seq_on[j].val2 = val_u32[2];
			camera_info[id].modules[i].seq_on[j].wait = val_u32[3];
		}
	}

	rc = of_property_read_string(of_node,
		"default_module_name",
		&camera_info[id].default_module_name);
	if (rc < 0) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		goto fail;
	}

	return 0;
fail:
	sony_camera_info_deinit(i);

	return rc;
}

static int sony_camera_gpio_init(struct sony_camera_data *data)
{
	int rc = 0;
	uint16_t i = 0;
	struct gpio *gpio_tbl = data->gpio_req_tbl;
	uint8_t size = data->gpio_req_tbl_size;

	if (!data->gpio_requested) {
		for (i = 0; i < size; i++) {
			rc = gpio_request_one(gpio_tbl[i].gpio,
				gpio_tbl[i].flags, gpio_tbl[i].label);
			if (rc) {
				// TODO: After GPIO request fails, contine to apply new gpios,
				// outout a error message for driver bringup debug
				LOGE("%s:%d gpio %d:%s request fails\n",
					__func__, __LINE__,
					gpio_tbl[i].gpio, gpio_tbl[i].label);
			}
		}
		if (rc == 0)
			data->gpio_requested = true;
	}
	return rc;
}

static int sony_camera_gpio_deinit(struct sony_camera_data *data)
{
	int rc = 0;
	struct gpio *gpio_tbl = data->gpio_req_tbl;
	uint8_t size = data->gpio_req_tbl_size;

	if (data->gpio_requested) {
		gpio_free_array(gpio_tbl, size);
		data->gpio_requested = false;
	}
	return rc;
}

static int sony_camera_gpio_set(struct sony_camera_data *data,
	int gpio_pin, int value)
{
	int rc = 0;

	if (data->gpio_requested)
		gpio_set_value_cansleep(gpio_pin, value);
	else
		rc = -EPERM;

	return rc;
}

static int sony_camera_regist_gpio_irq(
	struct sony_camera_data *camera_data, irq_handler_t handler,
	unsigned int flags, const char *label)
{
	int irq;
	int rc = 0;
	struct gpio *irq_gpio = NULL;

	irq_gpio = sony_camera_get_gpio_pin(camera_data, label);
	if (!irq_gpio) {
		LOGE("%s:%d Sony sensor: can't find gpio irq:%s\n",
			__func__, __LINE__, label);
		return -EINVAL;
	}
	irq = gpio_to_irq(irq_gpio->gpio);
	rc = request_irq(irq, handler, flags, irq_gpio->label, camera_data);
	if (rc) {
		LOGE("%s:%d Sony sensor: can't regist gpio irq:%s\n",
			__func__, __LINE__, label);
		return -EINVAL;
	}
	LOGD("%s regist gpio %d for %s\n", __func__, irq_gpio->gpio, label);
	return 0;
}

static int sony_camera_unregist_gpio_irq(
	struct sony_camera_data *camera_data, const char *label)
{
	int irq;
	struct gpio *irq_gpio = NULL;

	irq_gpio = sony_camera_get_gpio_pin(camera_data, label);
	if (!irq_gpio) {
		LOGE("%s:%d Sony sensor: can't find gpio irq:%s\n",
			__func__, __LINE__, label);
		return -EINVAL;
	}
	irq = gpio_to_irq(irq_gpio->gpio);
	disable_irq(irq);
	free_irq(irq, camera_data);
	LOGD("%s unregist gpio %d for %s\n", __func__, irq_gpio->gpio, label);
	return 0;
}

static void sony_camera_send_event(struct sony_camera_data *data,
	const struct sony_camera_event_data *sensor_event) {
	unsigned long flags;

	struct sony_camera_event_list_data *event = kzalloc(
		sizeof(struct sony_camera_event_list_data), GFP_ATOMIC);
	if (!event) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		return;
	}
	LOGI("sony_camera_send_event type 0x%x\n", sensor_event->type);
	memcpy(&event->event_data, sensor_event,
		sizeof(struct sony_camera_event_data));
	spin_lock_irqsave(&camera_data->event_list_lock, flags);
	list_add_tail(&event->list, &data->event_available);
	spin_unlock_irqrestore(&camera_data->event_list_lock, flags);
	wake_up_interruptible(&data->event_wait_q);
}

static irqreturn_t sony_camera_irq_handler(int irq, void *info)
{
	struct irq_desc *desc = NULL;
	struct sony_camera_data *camera_data = NULL;
	struct sony_camera_task_queue_cmd *queue_cmd = NULL;
	unsigned long flags;
	int irq_type = SONY_CAMERA_IRQ_EVENT_MAX;

	camera_data = (struct sony_camera_data *)info;
	if (!camera_data) {
		LOGE("%s:%d err camera_data is NULL\n", __func__, __LINE__);
		goto exit;
	}
	spin_lock_irqsave(&camera_data->task_lock, flags);
	desc = irq_to_desc(irq);
	if (desc != NULL && !strcmp(desc->action->name, SONY_CAMERA_GPIO_IRQ_SOF)) {
		struct sony_camera_event_data camera_event;
		struct timespec ts;
		unsigned long sof_lock_flags;

		get_monotonic_boottime(&ts);
		memset(&camera_event, 0, sizeof(camera_event));
		spin_lock_irqsave(&camera_data->sof_lock, sof_lock_flags);
		camera_data->sof_count++;
		if (camera_data->sof_count > 0xFFFFFFF0)
			camera_data->sof_count = 1;
		camera_event.data.sof_data.sof_count = camera_data->sof_count;
		spin_unlock_irqrestore(&camera_data->sof_lock, sof_lock_flags);
		camera_event.type = SONY_CAMERA_EVT_SOF;
		camera_event.data.sof_data.mono_timestamp.tv_sec = ts.tv_sec;
		camera_event.data.sof_data.mono_timestamp.tv_usec = ts.tv_nsec / 1000;
		LOGI("%s:%d sof_count = %d\n",
			__func__, __LINE__, camera_data->sof_count);
		sony_camera_send_event(camera_data, &camera_event);
		spin_unlock_irqrestore(&camera_data->task_lock, flags);
		return IRQ_HANDLED;
	} else {
		struct gpio *irq_gpio = NULL;

		if (desc != NULL &&
			!strcmp(desc->action->name, SONY_CAMERA_GPIO_IRQ_FATAL)) {
			irq_type = SONY_CAMERA_IRQ_FATAL_EVENT;
		} else if (desc != NULL &&
			!strcmp(desc->action->name, SONY_CAMERA_GPIO_IRQ_SPI_READY)) {
			irq_gpio = sony_camera_get_gpio_pin(
			camera_data,
			SONY_CAMERA_GPIO_IRQ_SPI_READY);
			irq_type = gpio_get_value(irq_gpio->gpio) ?
			SONY_CAMERA_IRQ_SPI_READY_DEASSERT_EVENT :
			SONY_CAMERA_IRQ_SPI_READY_ASSERT_EVENT;
		} else if (desc != NULL &&
			!strcmp(desc->action->name, SONY_CAMERA_GPIO_IRQ_GP_STATUS)) {
			irq_gpio = sony_camera_get_gpio_pin(
			camera_data,
			SONY_CAMERA_GPIO_IRQ_GP_STATUS);
			irq_type = gpio_get_value(irq_gpio->gpio) ?
			SONY_CAMERA_IRQ_GP_DEASSERT_EVENT :
			SONY_CAMERA_IRQ_GP_ASSERT_EVENT;
		} else {
			LOGE("%s:%d Unknown interrupt: %s\n",
				__func__, __LINE__, desc->action->name);
			goto exit;
		}

		queue_cmd = &camera_data->task_queue_cmd[camera_data->taskq_idx];
		if (!queue_cmd) {
			spin_unlock_irqrestore(&camera_data->task_lock, flags);
			LOGE("%s:%d err queue_cmd is NULL\n", __func__, __LINE__);
			goto exit;
		}
		if (queue_cmd->cmd_used) {
			LOGE("%s: err task queue overflow\n", __func__);
			list_del(&queue_cmd->list);
		} else {
			atomic_add(1, &camera_data->irq_cnt);
		}
		queue_cmd->cmd_used = true;
		camera_data->taskq_idx = (camera_data->taskq_idx + 1) %
			SONY_CAMERA_TASKQ_SIZE;
		if (!desc || !(desc->action) || !(desc->action->name)) {
			LOGE("%s:%d err, can't find irq's name\n",
				__func__, __LINE__);
			queue_cmd->irq_name = "Stray irq";
		} else {
			queue_cmd->irq_name = desc->action->name;
		}
		queue_cmd->irq_type = irq_type;
		LOGD("%s:%d irq_name = %s irq_type = %d taskq_idx = %d\n",
			__func__, __LINE__, desc->action->name,
			irq_type, camera_data->taskq_idx);
		list_add_tail(&queue_cmd->list, &camera_data->task_q);
		spin_unlock_irqrestore(&camera_data->task_lock, flags);
		wake_up_interruptible(&camera_data->wait_q);
	}

exit:
	return IRQ_HANDLED;
}

static int sony_camera_irq_init(
	struct sony_camera_data *camera_data,
	struct sony_camera_info *camera_info)
{
	int rc = 0;

	if (camera_info->use_spi) {
		rc = sony_camera_regist_gpio_irq(camera_data,
			sony_camera_irq_handler,
			IRQF_TRIGGER_FALLING,
			SONY_CAMERA_GPIO_IRQ_FATAL);
		if (rc < 0)
			LOGE("regist gpio irq: %s failed\n",
				SONY_CAMERA_GPIO_IRQ_FATAL);
		rc = sony_camera_regist_gpio_irq(camera_data,
			sony_camera_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			SONY_CAMERA_GPIO_IRQ_GP_STATUS);
		if (rc < 0)
			LOGE("regist gpio irq: %s failed\n",
				SONY_CAMERA_GPIO_IRQ_GP_STATUS);
		rc = sony_camera_regist_gpio_irq(camera_data,
			sony_camera_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			SONY_CAMERA_GPIO_IRQ_SPI_READY);
		if (rc < 0)
			LOGE("regist gpio irq: %s failed\n",
				 SONY_CAMERA_GPIO_IRQ_SPI_READY);
	}
	if (camera_data->has_hw_sof) {
		rc = sony_camera_regist_gpio_irq(camera_data,
			sony_camera_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			SONY_CAMERA_GPIO_IRQ_SOF);
		if (rc < 0)
			LOGE("regist gpio irq: %s failed\n",
				SONY_CAMERA_GPIO_IRQ_SOF);
	}

	return rc;
}

static int sony_camera_irq_deinit(struct sony_camera_data *camera_data,
	struct sony_camera_info *camera_info)
{
	int rc = 0;

	if (camera_data->has_hw_sof) {
		rc = sony_camera_unregist_gpio_irq(camera_data,
			SONY_CAMERA_GPIO_IRQ_SOF);
		if (rc < 0)
			LOGE("unregist gpio irq: %s failed\n",
				SONY_CAMERA_GPIO_IRQ_SOF);
	}
	if (camera_info->use_spi) {
		rc = sony_camera_unregist_gpio_irq(camera_data,
			SONY_CAMERA_GPIO_IRQ_SPI_READY);
		if (rc < 0)
			LOGE("unregist gpio irq: %s failed\n",
				SONY_CAMERA_GPIO_IRQ_SPI_READY);
		rc = sony_camera_unregist_gpio_irq(camera_data,
			SONY_CAMERA_GPIO_IRQ_GP_STATUS);
		if (rc < 0)
			LOGE("unregist gpio irq: %s failed\n",
				SONY_CAMERA_GPIO_IRQ_GP_STATUS);
		rc = sony_camera_unregist_gpio_irq(camera_data,
			SONY_CAMERA_GPIO_IRQ_FATAL);
		if (rc < 0)
			LOGE("unregist gpio irq: %s failed\n",
				SONY_CAMERA_GPIO_IRQ_FATAL);
	}
	return rc;
}

static int sony_camera_check_irq(struct sony_camera_data *camera_data)
{
	unsigned long flags;
	int irq_cnt;

	spin_lock_irqsave(&camera_data->task_lock, flags);
	irq_cnt = atomic_read(&camera_data->irq_cnt);
	spin_unlock_irqrestore(&camera_data->task_lock, flags);
	return irq_cnt;
}

static int sony_camera_vreg_set(struct sony_camera_data *data,
	enum sony_camera_cmd cmd, int level, int op_mode)
{
	int rc = 0;
	struct regulator *vreg = NULL;
	struct device *dev = &data->p_dev->dev;

	if (cmd == SONY_CAM_VDIG) {
		if (data->cam_vdig) {
			vreg = data->cam_vdig;
		} else {
			vreg = regulator_get(dev, "cam_vdig");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_vdig, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_vdig = vreg;
		}
	} else if (cmd == SONY_CAM_VIO) {
		if (data->cam_vio) {
			vreg = data->cam_vio;
		} else {
			vreg = regulator_get(dev, "cam_vio");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_vio, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_vio = vreg;
		}
	} else if (cmd == SONY_CAM_VANA) {
		if (data->cam_vana) {
			vreg = data->cam_vana;
		} else {
			vreg = regulator_get(dev, "cam_vana");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_vana, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_vana = vreg;
		}
	} else if (cmd == SONY_CAM_VAF) {
		if (data->cam_vaf) {
			vreg = data->cam_vaf;
		} else {
			vreg = regulator_get(dev, "cam_vaf");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_vaf, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_vaf = vreg;
		}
	} else if (cmd == SONY_CAM_CLK) {
		if (data->cam_clk) {
			vreg = data->cam_clk;
		} else {
			vreg = regulator_get(dev, "cam_clk");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_clk, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_clk = vreg;
		}
	} else if (cmd == SONY_MCAM_VDIG) {
		if (data->cam_main_vdig) {
			vreg = data->cam_main_vdig;
		} else {
			vreg = regulator_get(dev, "cam_main_vdig");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_main_vdig, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_main_vdig = vreg;
		}
	} else if (cmd == SONY_SCAM_VDIG) {
		if (data->cam_sub_vdig) {
			vreg = data->cam_sub_vdig;
		} else {
			vreg = regulator_get(dev, "cam_sub_vdig");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_sub_vdig, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_sub_vdig = vreg;
		}
	} else if (cmd == SONY_ISP_MEM) {
		if (data->isp_mem) {
			vreg = data->isp_mem;
		} else {
			vreg = regulator_get(dev, "cam_isp_mem");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_isp_mem, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->isp_mem = vreg;
		}
	} else if (cmd == SONY_ISP_CORE) {
		if (data->isp_core) {
			vreg = data->isp_core;
		} else {
			vreg = regulator_get(dev, "cam_isp_core");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_isp_core, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->isp_core = vreg;
		}
	} else if (cmd == SONY_ISP_0P9) {
		if (data->isp_0p9) {
			vreg = data->isp_0p9;
		} else {
			vreg = regulator_get(dev, "cam_isp_0p9");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_isp_0p9, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->isp_0p9 = vreg;
		}
	} else if (cmd == SONY_ISP_1P2) {
		if (data->isp_1p2) {
			vreg = data->isp_1p2;
		} else {
			vreg = regulator_get(dev, "cam_isp_1p2");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_isp_1p2, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->isp_1p2 = vreg;
		}
	} else {
		rc = -EINVAL;
		LOGE("invalid resource\n");
		goto exit;
	}

	level *= 1000;
	if (level >= 0) {
		if (level > 0) {
			rc = regulator_set_voltage(vreg, level, level);
			if (rc < 0)
				goto set_voltage_fail;
		}
		if (op_mode > 0) {
			rc = regulator_set_load(vreg, op_mode);
			if (rc < 0)
				goto set_voltage_fail;
		}
		rc = regulator_enable(vreg);
		if (rc < 0)
			goto enable_fail;
	} else {
		if (op_mode == 0)
			(void)regulator_set_load(vreg, 0);
		(void)regulator_disable(vreg);
		regulator_put(vreg);
	}
	goto exit;

enable_fail:
	(void)regulator_set_load(vreg, 0);
set_voltage_fail:
	regulator_put(vreg);
exit:
	if (rc < 0 || level < 0) {
		if (vreg == data->cam_vdig)
			data->cam_vdig = NULL;
		else if (vreg == data->cam_vio)
			data->cam_vio = NULL;
		else if (vreg == data->cam_vana)
			data->cam_vana = NULL;
		else if (vreg == data->cam_vaf)
			data->cam_vaf = NULL;
		else if (vreg == data->cam_main_vdig)
			data->cam_main_vdig = NULL;
		else if (vreg == data->cam_sub_vdig)
			data->cam_sub_vdig = NULL;
		else if (vreg == data->isp_mem)
			data->isp_mem = NULL;
		else if (vreg == data->isp_core)
			data->isp_core = NULL;
		else if (vreg == data->isp_0p9)
			data->isp_0p9 = NULL;
		else if (vreg == data->isp_1p2)
			data->isp_1p2 = NULL;
		else if (vreg == data->cam_clk)
			data->cam_clk = NULL;
	}

	if (rc < 0)
		LOGE("error happened (%d)\n", rc);
	return rc;
}

static int sony_camera_mclk_set(struct sony_camera_data *data, int value)
{
	int rc = 0;
	int clk_rate = 0;
	struct device *dev = &data->p_dev->dev;
	struct clk *clk_handle = data->clk_handle;

	if (value >= 0) {
		clk_handle = clk_get(dev, "cam_clk");
		if (clk_handle == NULL) {
			LOGE("%s line %d get clk failed\n", __func__, __LINE__);
			rc = -EINVAL;
			goto fail_get;
		}
		clk_rate = clk_round_rate(clk_handle,
			value ? value : SONY_CAMERA_MCLK_DEFAULT);
		if (clk_rate < 0) {
			LOGE("%s line %d %s round failed\n", __func__, __LINE__, "cam_clk");
			goto fail_set;
		}
		rc = clk_set_rate(clk_handle, clk_rate);
		if (rc < 0) {
			LOGE("%s line %d %s set failed\n", __func__, __LINE__, "cam_clk");
			goto fail_set;
		}
		rc = clk_prepare_enable(clk_handle);
		if (rc < 0) {
			LOGE("%s line %d %s enable failed\n", __func__, __LINE__, "cam_clk");
			goto fail_prepare_enable;
		}
		data->clk_handle = clk_handle;
	} else {
		if (clk_handle != NULL) {
			clk_disable_unprepare(clk_handle);
			clk_put(clk_handle);
			data->clk_handle = NULL;
		}
	}

	if (rc < 0)
		LOGE("error happened (%d)\n", rc);
	return rc;

fail_prepare_enable:
	clk_disable_unprepare(clk_handle);
fail_set:
	clk_put(clk_handle);
fail_get:
	return rc;
}

static int sony_camera_cci_init(struct sony_camera_data *data)
{
	int rc = 0;
	struct cam_cci_ctrl cci_ctrl;

	memset(&cci_ctrl, 0, sizeof(cci_ctrl));
	cci_ctrl.cmd = MSM_CCI_INIT;
	cci_ctrl.cci_info = &data->cci_info;
	rc = v4l2_subdev_call(data->cci_info.cci_subdev,
		core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc < 0) {
		LOGE("%s line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	rc = cci_ctrl.status;
	return rc;
}

static int sony_camera_cci_deinit(struct sony_camera_data *data)
{
	int rc = 0;
	struct cam_cci_ctrl cci_ctrl;

	memset(&cci_ctrl, 0, sizeof(cci_ctrl));
	cci_ctrl.cmd = MSM_CCI_RELEASE;
	cci_ctrl.cci_info = &data->cci_info;
	rc = v4l2_subdev_call(data->cci_info.cci_subdev,
		core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc < 0) {
		LOGE("%s line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	rc = cci_ctrl.status;
	return rc;
}

static int sony_camera_i2c_read(struct sony_camera_data *data,
			uint8_t slave_addr, uint32_t addr,
			uint8_t type, uint16_t len, uint8_t *buf)
{
	int rc = 0;
	int i = 0;
	int retry_cnt = SONY_CAMERA_MAX_RETRY_COUNT;
	uint8_t read_data[len];
	struct cam_cci_ctrl cci_ctrl;

	memset(&cci_ctrl, 0, sizeof(cci_ctrl));

	if (type > CAMERA_SENSOR_I2C_TYPE_MAX) {
		rc = -EINVAL;
	} else {
		cci_ctrl.cmd = MSM_CCI_I2C_READ;
		cci_ctrl.cci_info = &data->cci_info;
		cci_ctrl.cci_info->sid = slave_addr >> 1;
		cci_ctrl.cfg.cci_i2c_read_cfg.addr = addr;
		cci_ctrl.cfg.cci_i2c_read_cfg.addr_type = type;
		cci_ctrl.cfg.cci_i2c_read_cfg.data = read_data;
		cci_ctrl.cfg.cci_i2c_read_cfg.num_byte = len;
		while ((rc = v4l2_subdev_call(data->cci_info.cci_subdev,
			core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl)) && retry_cnt) {
			retry_cnt--;
			usleep_range(1000, 3000);
			LOGE("Retry slave0x%04x,addr0x%04x,type0x%02x,len0x%02x\n",
				slave_addr, addr, type, len);
		}
		if (rc < 0) {
			LOGE("slave0x%04x,addr0x%04x,type0x%02x,len0x%02x\n",
				slave_addr, addr, type, len);
			LOGE("i2c read failed(%d)\n", rc);
			rc = -EIO;
		} else {
			rc = cci_ctrl.status;
			for (i = 0; i < len; i++)
				buf[i] = read_data[i];
		}
	}
	return rc;
}

static int sony_camera_i2c_read_eeprom(struct sony_camera_data *data,
	uint8_t slave_addr, uint32_t addr,
	uint8_t type, uint16_t len, uint8_t *buf)
{
	int rc = 0;
	int i = 0;
	uint16_t read_addr = 0x00;
	uint8_t read_data = 0x00;

	if (len > SONY_CAMERA_I2C_MAX_DATA_LEN) {
		rc = -EINVAL;
	} else {
		for (i = 0; i < len; i++) {
			read_addr = addr + i;
			rc = sony_camera_i2c_read(data, slave_addr, read_addr, type,
				CAMERA_SENSOR_I2C_TYPE_BYTE, &read_data);
			if (rc < 0) {
				LOGE("slave0x%04x,addr0x%04x,type0x%02x,len0x%02x\n",
					slave_addr, addr, type, len);
				LOGE("i2c read failed(%d)\n", rc);
				rc = -EIO;
				break;
			} else {
				buf[i] = (uint8_t)(read_data & 0xFF);
			}
		}
	}
	return rc;
}

static int sony_camera_i2c_write(struct sony_camera_data *data,
	uint8_t slave_addr, uint32_t addr, uint8_t type,
	uint16_t len, uint8_t *buf)
{
	int32_t rc = -EINVAL;
	int i = 0;
	struct cam_cci_ctrl cci_ctrl;
	struct cam_sensor_i2c_reg_array* reg_settings;
	bool need_free = FALSE;
	if (len > SONY_CAMERA_I2C_MAX_LEN) {
		reg_settings = vzalloc(sizeof(struct cam_sensor_i2c_reg_array) * len);
		need_free = TRUE;
	} else {
		memset(data->i2c_data, 0, SONY_CAMERA_I2C_MAX_SIZE);
		reg_settings = (struct cam_sensor_i2c_reg_array*)data->i2c_data;
	}

	for (i = 0; i < len; i++) {
		reg_settings[i].reg_addr = addr + i;
		reg_settings[i].reg_data = buf[i];
		reg_settings[i].delay = 0;
		reg_settings[i].data_mask = 0;
	}
	memset(&cci_ctrl, 0, sizeof(cci_ctrl));
	cci_ctrl.cmd = MSM_CCI_I2C_WRITE_BURST;
	cci_ctrl.cci_info = &data->cci_info;
	cci_ctrl.cci_info->sid = slave_addr >> 1;
	cci_ctrl.cfg.cci_i2c_write_cfg.reg_setting = reg_settings;
	cci_ctrl.cfg.cci_i2c_write_cfg.data_type = 1; // TODO: CAMERA_SENSOR_I2C_TYPE_BYTE
	cci_ctrl.cfg.cci_i2c_write_cfg.addr_type = type;
	cci_ctrl.cfg.cci_i2c_write_cfg.size = len;
	rc = v4l2_subdev_call(data->cci_info.cci_subdev,
		core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (need_free)
		vfree(reg_settings);
	return rc;
}

static int sony_camera_spi_read(
	struct sony_camera_data *camera_data,
	uint8_t *buf, uint32_t len)
{
	struct spi_transfer rx = {
		.rx_buf		 = buf,
		.len		 = len,
		.speed_hz	 = camera_data->spi_master->max_speed_hz / 2,
	};
	struct spi_message m;
	int rc = 0;
	uint32_t i = 0;

	if (camera_data->rx_buf == NULL) {
		LOGE("%s: rx_buf kmalloc failed\n", __func__);
		return -ENOMEM;
	}

	while (i < len && !rc) {
		memset(camera_data->rx_buf, 0, SONY_CAMERA_SPI_RX_MAX_LEN);
		rx.len = ((len - i) < SONY_CAMERA_SPI_RX_MAX_LEN) ?
			len % SONY_CAMERA_SPI_RX_MAX_LEN : SONY_CAMERA_SPI_RX_MAX_LEN;
		rx.rx_buf = camera_data->rx_buf;
		spi_message_init(&m);
		spi_message_add_tail(&rx, &m);

		rc = spi_sync(camera_data->spi_master, &m);
		memcpy(&buf[i], camera_data->rx_buf, rx.len);
		i += rx.len;
		LOGD("%s: spi_sync %d/%d tx.len=%d\n", __func__, i, len, rx.len);
	}
	return rc;
}

static int sony_camera_spi_write(
	struct sony_camera_data *camera_data,
	const uint8_t *buf, uint32_t len)
{
	struct spi_transfer tx = {
		.tx_buf  = NULL,
		.len	 = 0,
	};
	struct spi_message m;
	int rc = 0;
	uint32_t i = 0;

	if (camera_data->tx_buf == NULL) {
		LOGE("%s: tx_buf kmalloc failed\n", __func__);
		return -ENOMEM;
	}

	while (i < len && !rc) {
		memset(camera_data->tx_buf, 0, SONY_CAMERA_SPI_TX_MAX_LEN);
		tx.len = ((len - i) < SONY_CAMERA_SPI_TX_MAX_LEN) ?
			len % SONY_CAMERA_SPI_TX_MAX_LEN : SONY_CAMERA_SPI_TX_MAX_LEN;
		memcpy(camera_data->tx_buf, &buf[i], tx.len);
		i += tx.len;
		tx.tx_buf = camera_data->tx_buf;
		spi_message_init(&m);
		spi_message_add_tail(&tx, &m);

		rc = spi_sync(camera_data->spi_master, &m);
		LOGD("%s: spi_sync %d/%d tx.len=%d\n", __func__, i, len, tx.len);
	}
	return rc;
}

static int sony_camera_spi_read_with_status(
	struct sony_camera_data *camera_data, uint8_t *buf,
	uint32_t len, enum sony_camera_spi_status status)
{
	int rc = 0;
	int val = 0;
	int retry = 0;
	int timeout = 0;
	struct gpio *irq_gpio = NULL;

	if (status == SPI_ASSERT) {
		timeout = wait_for_completion_timeout(
			&camera_data->spi_assert_complete,
			SONY_CAMERA_SPI_COMPLETION_TIMEOUT);

		LOGD("Wait spi_assert_complete done, rc:%d.\n", timeout);
		if (!timeout) {
			rc = -ETIMEDOUT;
			LOGE("%s: spi status assert wait timeout\n", __func__);
			goto exit;
		}
	} else {
		timeout = wait_for_completion_timeout(
			&camera_data->spi_deassert_complete,
			SONY_CAMERA_SPI_COMPLETION_TIMEOUT);

		LOGD("Wait spi_deassert_complete done, rc:%d.\n", timeout);
		if (!timeout) {
			rc = -ETIMEDOUT;
			LOGE("%s: spi status deassert wait timeout\n", __func__);
			goto exit;
		}
	}

	irq_gpio = sony_camera_get_gpio_pin(camera_data,
		SONY_CAMERA_GPIO_IRQ_SPI_READY);
	val = gpio_get_value(irq_gpio->gpio);
	LOGD(" spi status: expect=%d, current=%d\n", status, val);
	while (val != status) {
		usleep_range(200, 300);
		val = gpio_get_value(irq_gpio->gpio);
		if (++retry > 10000) {
			LOGE(" wait spi status(%d) timeout\n", status);
			rc = -ETIMEDOUT;
			goto exit;
		}
	}
	rc = sony_camera_spi_read(camera_data, buf, len);
exit:
	LOGI(" X rc = %d\n", rc);
	return rc;
}

static int sony_camera_spi_write_with_status(
	struct sony_camera_data *camera_data, const uint8_t *buf,
	uint32_t len, enum sony_camera_spi_status status)
{
	int rc = 0;
	int val = 0;
	int retry = 0;
	struct gpio *irq_gpio = NULL;
	int timeout = 0;

	if (status == SPI_ASSERT) {
		timeout = wait_for_completion_timeout(
			&camera_data->spi_assert_complete,
			SONY_CAMERA_SPI_COMPLETION_TIMEOUT);

		LOGD("Wait spi_assert_complete done, rc:%d.\n", timeout);
		if (!timeout) {
			rc = -ETIMEDOUT;
			LOGE("%s: spi status assert wait timeout\n", __func__);
			goto exit;
		}
	} else {
		timeout = wait_for_completion_timeout(
			&camera_data->spi_deassert_complete,
			SONY_CAMERA_SPI_COMPLETION_TIMEOUT);

		LOGD("Wait spi_deassert_complete done, rc:%d.\n", timeout);
		if (!timeout) {
			rc = -ETIMEDOUT;
			LOGE("%s: spi status deassert wait timeout\n", __func__);
			goto exit;
		}
	}
	irq_gpio = sony_camera_get_gpio_pin(camera_data,
		SONY_CAMERA_GPIO_IRQ_SPI_READY);
	val = gpio_get_value(irq_gpio->gpio);
	LOGD(" spi status: expect=%d, current=%d\n", status, val);
	while (val != status) {
		usleep_range(200, 300);
		val = gpio_get_value(irq_gpio->gpio);
		if (++retry > 10000) {
			LOGE(" wait spi status(%d) timeout\n", status);
			rc = -ETIMEDOUT;
			goto exit;
		}
	}
	rc = sony_camera_spi_write(camera_data, buf, len);
exit:
	return rc;
}

static int sony_camera_spi_set(struct sony_camera_data *data,
	int value)
{
	int rc = 0;

	if (data->spi_master) {
		if (data->probe_done) {
			if (value >= 0) {
				rc = pm_runtime_get_sync(data->spi_master->master->dev.parent);
				if (rc < 0)
					LOGE("%s: spi_active failed rc %d\n", __func__, rc);
			} else {
				rc = pm_runtime_put_sync(data->spi_master->master->dev.parent);
				if (rc < 0)
					LOGE("%s: spi_suspend failed rc %d\n", __func__, rc);
			}
		}
	} else
		rc = -ENODEV;

	return rc;
}

static int sony_camera_power_ctrl(struct sony_camera_data *data,
	uint8_t on)
{
	int rc = 0;
	const struct sony_camera_module *mod = data->probe_done ?
		data->module : camera_info[data->id].modules;
	const struct sony_camera_seq *seq = on ? mod->seq_on : mod->seq_off;
	struct gpio *gpio_reset = NULL;

	while (seq->cmd != EXIT) {
		uint8_t iodt = 0x00;
		LOGD("%s sony_camera_power_ctrl %d cmd %d val1 %d val2 %d\n",
			__func__, __LINE__, seq->cmd, seq->val1, seq->val2);
		switch (seq->cmd) {
		case SONY_GPIO_RESET:
			gpio_reset = sony_camera_get_gpio_pin(data,
				SONY_CAMERA_GPIO_RESET);
			if (gpio_reset->gpio <= 0) {
				rc = -EPERM;
				break;
			}
			LOGD("reset gpio %d->%d\n", gpio_reset->gpio, seq->val1);
			rc = sony_camera_gpio_set(data, gpio_reset->gpio, seq->val1);
			break;
		case SONY_CAM_VDIG:
		case SONY_CAM_VIO:
		case SONY_CAM_VANA:
		case SONY_CAM_VAF:
		case SONY_ISP_MEM:
		case SONY_MCAM_VDIG:
		case SONY_SCAM_VDIG:
		case SONY_ISP_CORE:
		case SONY_ISP_0P9:
		case SONY_ISP_1P2:
			rc = sony_camera_vreg_set(data,
				seq->cmd, seq->val1, seq->val2);
			break;
		case SONY_ISP_SPI:
			rc = sony_camera_spi_set(data,seq->val1);
			break;
		case SONY_CAM_CLK:
			rc = sony_camera_vreg_set(data,
				seq->cmd, on ? 0 : -1, seq->val2);
			if (rc < 0) {
				LOGE("%s mclk vregfailed %d\n", __func__, __LINE__);
			} else {
				rc = sony_camera_mclk_set(data, seq->val1);
			}
			break;
		case SONY_I2C_WRITE:
			rc = sony_camera_i2c_write(data,
				camera_info[data->id].i2c_addr, seq->val1, 2, 1, &iodt);
			break;
		default:
			goto exit;
		}
		mdelay(seq->wait);
		if (rc < 0 && on)
			goto exit;
		seq++;
	}
exit:
	return rc;
}

static int sony_camera_task_handler(void *data)
{
	int rc = 0;
	int irq_type = SONY_CAMERA_IRQ_EVENT_MAX;
	struct sony_camera_task_queue_cmd *queue_cmd = NULL;
	unsigned long flags;
	struct sony_camera_data *camera_data = NULL;
	struct task_struct *tsk;
	struct sched_param PARAM = {.sched_priority = MAX_RT_PRIO };

	tsk = current;
	sched_setscheduler(tsk, SCHED_FIFO, &PARAM);

	if (data == NULL)
		return -EINVAL;

	camera_data = (struct sony_camera_data *)data;
	while (!kthread_should_stop()) {
		wait_event_interruptible(camera_data->wait_q,
			sony_camera_check_irq(camera_data) ||
			kthread_should_stop());
		while (atomic_read(&camera_data->irq_cnt)) {
			spin_lock_irqsave(&camera_data->task_lock, flags);
			queue_cmd = list_first_entry(&camera_data->task_q,
				struct sony_camera_task_queue_cmd, list);
			if (!queue_cmd) {
				atomic_set(&camera_data->irq_cnt, 0);
				spin_unlock_irqrestore(&camera_data->task_lock,
					flags);
				break;
			}
			irq_type = queue_cmd->irq_type;
			atomic_sub(1, &camera_data->irq_cnt);
			list_del(&queue_cmd->list);
			queue_cmd->cmd_used = false;
			spin_unlock_irqrestore(&camera_data->task_lock, flags);
			LOGI("Handle %d in Task Handler \n", irq_type);
			switch (irq_type) {
			case SONY_CAMERA_IRQ_FATAL_EVENT:
				mutex_lock(&camera_data->state_lock);
				if (camera_data->state == SONY_CAMERA_STATE_POWER_UP) {
					struct sony_camera_event_data camera_event;
					uint8_t err_status = 0;

					rc = sony_camera_i2c_read(
						camera_data,
						camera_info[camera_data->id].eeprom_addr,
						SONY_CAMERA_FATAL_ERROR_STATUS_ADDR,
						CAMERA_SENSOR_I2C_TYPE_WORD,
						1, &err_status);
					if (rc < 0) {
						LOGE("Fail to read FATAL_ERROR_STATUS=0x%02x rc=%d\n",
							err_status, rc);
						mutex_unlock(&camera_data->state_lock);
						break;
					}
					LOGE("FATAL_ERROR_STATUS=0x%02x rc=%d\n",
						err_status, rc);

					memset(&camera_event, 0, sizeof(camera_event));
					camera_event.type = SONY_CAMERA_EVT_FATAL;
					camera_event.data.err_code = err_status;
					sony_camera_send_event(camera_data, &camera_event);
				} else {
					LOGE("Warning: irq_type %x come before power_up_done\n",
						irq_type);
				}
				mutex_unlock(&camera_data->state_lock);
				break;
			case SONY_CAMERA_IRQ_GP_ASSERT_EVENT: {
				uint8_t r_data = 0;
				uint8_t w_data[8] = { 0x7f, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 };
				mutex_lock(&camera_data->state_lock);
				if (camera_data->state == SONY_CAMERA_STATE_POWER_UP) {
					rc = sony_camera_i2c_read(
						camera_data,
						camera_info[camera_data->id].eeprom_addr,
						SONY_CAMERA_GP_MASTER_STATUS_ADDR,
						CAMERA_SENSOR_I2C_TYPE_WORD,
						1, &r_data);
					if (rc < 0) {
						LOGE("I2C read failed: slave:0x%x, reg_addr:0x%x rc=%d\n",
							 camera_info[camera_data->id].eeprom_addr,
							 SONY_CAMERA_GP_MASTER_STATUS_ADDR, rc);
						mutex_unlock(&camera_data->state_lock);
						break;
					}
					w_data[4] |= r_data;
					rc = sony_camera_i2c_write(
						camera_data,
						camera_info[camera_data->id].eeprom_addr,
						SONY_CAMERA_GP_STATUS_MASK_ADDR,
						CAMERA_SENSOR_I2C_TYPE_WORD,
						8, w_data);
					if (rc < 0) {
						LOGE("I2C write failed: slave:0x%x, reg_addr:0x%x rc=%d\n",
							camera_info[camera_data->id].eeprom_addr, 0x1000, rc);
						mutex_unlock(&camera_data->state_lock);
						break;
					}
					camera_data->gp_status = r_data;
					if (!camera_data->probe_done)
						complete(&camera_data->gp_assert_complete);
				} else
					LOGE("Warning: irq_type %x come before power_up_done\n",
						irq_type);
				mutex_unlock(&camera_data->state_lock);
				break;
			}
			case SONY_CAMERA_IRQ_GP_DEASSERT_EVENT: {
				if (camera_data->gp_status) {
					uint8_t w_data[8] = { 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 };

					rc = sony_camera_i2c_write(camera_data,
						camera_info[camera_data->id].eeprom_addr,
						SONY_CAMERA_GP_STATUS_MASK_ADDR,
						CAMERA_SENSOR_I2C_TYPE_WORD,
						8, w_data);
					if (rc < 0) {
						LOGE("I2C write failed: slave:0x%x, reg_addr:0x%x rc=%d\n",
							camera_info[camera_data->id].eeprom_addr,
							SONY_CAMERA_GP_STATUS_MASK_ADDR, rc);
						camera_data->gp_status = 0;
						break;
					}
					if (camera_data->state == SONY_CAMERA_STATE_POWER_UP &&
						camera_data->probe_done) {
						struct sony_camera_event_data camera_event;

						memset(&camera_event, 0, sizeof(camera_event));
						camera_event.type = SONY_CAMERA_EVT_GP_STATUS;
						camera_event.data.gp_status = camera_data->gp_status;
						sony_camera_send_event(camera_data, &camera_event);
					}
				} else
					LOGE("Warning: irq_type %x come before power_up_done\n",
						irq_type);

				camera_data->gp_status = 0;
				break;
			}
			case SONY_CAMERA_IRQ_SPI_READY_ASSERT_EVENT:
				complete(&camera_data->spi_assert_complete);
				reinit_completion(&camera_data->spi_deassert_complete);
				break;
			case SONY_CAMERA_IRQ_SPI_READY_DEASSERT_EVENT:
				complete(&camera_data->spi_deassert_complete);
				reinit_completion(&camera_data->spi_assert_complete);
				break;
			case SONY_CAMERA_IRQ_SOF_EVENT:
				LOGE("Warning: SOF irq processed in interrupt context\n");
				break;
			default:
				break;
			}
		}
	}
	return rc;
}

void sony_camera_flush_task_queue(struct sony_camera_data *camera_data)
{
	unsigned long flags;
	struct sony_camera_task_queue_cmd *queue_cmd = NULL;

	spin_lock_irqsave(&camera_data->task_lock, flags);
	while (atomic_read(&camera_data->irq_cnt)) {
		queue_cmd = list_first_entry(&camera_data->task_q,
			struct sony_camera_task_queue_cmd, list);
		if (!queue_cmd) {
			atomic_set(&camera_data->irq_cnt, 0);
			break;
		}
		atomic_sub(1, &camera_data->irq_cnt);
		list_del(&queue_cmd->list);
		queue_cmd->cmd_used = false;
	}
	spin_unlock_irqrestore(&camera_data->task_lock, flags);
}


static int sony_camera_thermal_get_temp(
	struct thermal_zone_device *thermal, int *temp)
{
	int rc = 0;
	int id = 0;
	int thermal_id = 0;

	if (!temp) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		rc = -EPERM;
		goto error;
	}

	if (!strncmp(thermal->type, SONY_CAMERA_THERMAL_NAME_0,
		sizeof(SONY_CAMERA_THERMAL_NAME_0))) {
		id = 0;
		thermal_id = 0;
	} else if (!strncmp(thermal->type, SONY_CAMERA_THERMAL_NAME_1,
		sizeof(SONY_CAMERA_THERMAL_NAME_1))) {
		id = 1;
		thermal_id = 0;
	} else if (!strncmp(thermal->type, SONY_CAMERA_THERMAL_NAME_2,
		sizeof(SONY_CAMERA_THERMAL_NAME_2))) {
		id = 0;
		thermal_id = 1;
	} else if (!strncmp(thermal->type, SONY_CAMERA_THERMAL_NAME_3,
		sizeof(SONY_CAMERA_THERMAL_NAME_3))) {
		id = 0;
		thermal_id = 2;
	} else {
		rc = -EPERM;
		goto error;
	}

	mutex_lock(&camera_data[id].thermal_lock);
	if (camera_data[id].thermal_ret_val < 0)
		rc = camera_data[id].thermal_ret_val;
	else
		*temp = (int)camera_data[id].thermal_sensor_temperature[thermal_id];
		LOGD("%s %d, id = %d rc = %d *temp = %d\n",
			__func__, __LINE__, id, rc, *temp);
	mutex_unlock(&camera_data[id].thermal_lock);

error:
	return rc;
}

static struct thermal_zone_device_ops sony_camera_thermal_ops = {
	.get_temp = sony_camera_thermal_get_temp,
};

static int sony_camera_eeprom_load(uint32_t id)
{
	int rc = 0;
	uint16_t i;
	int timeout = 0;
	uint8_t status_reg = 0;
	uint16_t len = 0;
	uint8_t *d = camera_data[id].eeprom;

	LOGI("load eeprom\n");
	camera_data[id].lot_id_available = false;

	/* load eeprom */
	if (camera_info[id].eeprom_type == 0) {
		for (i = 0; i < camera_info[id].eeprom_max_len;
			i += SONY_CAMERA_MAX_I2C_DATA) {
			uint8_t slave_addr = camera_info[id].eeprom_addr
				+ ((i & 0x0700) >> 7);
			uint32_t offset = i & 0x00FF;

			rc = sony_camera_i2c_read_eeprom(
				&camera_data[id],
				slave_addr,
				offset, 1,
				SONY_CAMERA_MAX_I2C_DATA, d + i);
			if (rc < 0) {
				LOGE("eeprom type %d i2c read fail %d\n",
					camera_info[id].eeprom_type, rc);
				camera_data[id].eeprom_len = 0;
				goto exit;
			}
		}
		len = i;
	} else if (camera_info[id].eeprom_type == 2) {
		len = camera_info[id].eeprom_max_len;
		rc = sony_camera_i2c_read_eeprom(
			&camera_data[id],
			camera_info[id].eeprom_addr,
			0, 2,
			camera_info[id].eeprom_max_len, d);
		if (rc < 0) {
			LOGE("eeprom type %d i2c read fail %d\n",
				camera_info[id].eeprom_type, rc);
			camera_data[id].eeprom_len = 0;
			goto exit;
		}
	} else if (camera_info[id].eeprom_type == 3) {
		LOGI("Begin to send [Aube transfer_info] for read eeprom size = %d\n",
			 SONY_CAMERA_TRANSFER_INFO_BIN_LEN);
		rc = sony_camera_spi_write_with_status(&camera_data[id],
			sony_camera_transfer_info_bin, SONY_CAMERA_TRANSFER_INFO_BIN_LEN,
			SPI_ASSERT);
		LOGI("End Send [Aube transfer_info] for eeprom size = %d rc = %d\n",
			SONY_CAMERA_TRANSFER_INFO_BIN_LEN, rc);
		if (rc < 0) {
			LOGE("Send [Aube Transfer_Info] failed!\n");
			goto exit;
		}

		LOGI("Begin to send [Aube pre_proc] for read eeprom size = %d\n",
			SONY_CAMERA_PRE_PROC_BIN_LEN);
		rc = sony_camera_spi_write_with_status(&camera_data[id],
			sony_camera_pre_proc_bin, SONY_CAMERA_PRE_PROC_BIN_LEN,
			SPI_ASSERT);
		LOGI("End Send [Aube pre_proc] for eeprom size = %d rc = %d\n",
			SONY_CAMERA_PRE_PROC_BIN_LEN, rc);
		if (rc < 0) {
			LOGE("Send [Aube pre_proc] failed!\n");
			goto exit;
		}

		timeout = wait_for_completion_timeout(
			&camera_data[id].gp_assert_complete,
			SONY_CAMERA_SPI_COMPLETION_TIMEOUT);
		LOGI("Wait gp_assert_complete done, rc:%d.\n", timeout);
		if (!timeout) {
			LOGE("%s: gp status assert wait timeout\n", __func__);
			rc = -ETIMEDOUT;
			goto exit;
		}

		do {
			rc = sony_camera_i2c_read(&camera_data[id],
				camera_info[id].eeprom_addr,
				SONY_CAMERA_EEPROM_PREPROCESS_STATUS,
				CAMERA_SENSOR_I2C_TYPE_WORD,
				1, &status_reg);
			LOGI("SONY_CAMERA_EEPROM_PREPROCESS_STATUS:%d\n", (int)status_reg);
			if (rc < 0) {
				LOGE("eeprom i2c read SONY_CAMERA_EEPROM_PREPROCESS_STATUS, fail rc: %d\n", rc);
				goto exit;
			}

			switch (status_reg) {
			case SONY_CAMERA_EEPROM_PREPROCESS_STATUS_PREPARATION:
			case SONY_CAMERA_EEPROM_PREPROCESS_STATUS_IDLE:
				continue;
			case SONY_CAMERA_EEPROM_PREPROCESS_STATUS_READY:
				break;
			case SONY_CAMERA_EEPROM_PREPROCESS_STATUS_ERROR:
				rc = sony_camera_i2c_read(
					&camera_data[id],
					camera_info[id].eeprom_addr,
					SONY_CAMERA_EEPROM_BAYER_EEPROM_STATUS,
					CAMERA_SENSOR_I2C_TYPE_WORD,
					1, &status_reg);
				LOGE("BAYER_EEPROM_STATUS error code:%d\n",
					 (int)status_reg);
				if (rc < 0) {
					LOGE("Read bayer SONY_CAMERA_EEPROM_PREPROCESS_STATUS_ERROR fail rc: %d\n", rc);
					goto exit;
				}
				rc = sony_camera_i2c_read(
					&camera_data[id],
					camera_info[id].eeprom_addr,
					SONY_CAMERA_EEPROM_BW_EEPROM_STATUS,
					CAMERA_SENSOR_I2C_TYPE_WORD,
					1, &status_reg);
				LOGE("BW_EEPROM_STATUS error code:%d\n",
					(int)status_reg);
				if (rc < 0) {
					LOGE("Read BW SONY_CAMERA_EEPROM_PREPROCESS_STATUS_ERROR fail rc: %d\n", rc);
					goto exit;
				}
				rc = -ENODEV;
				goto exit;
			default:
				LOGI("PREPROCESS_STATUS error, val: %d\n",
					(int)status_reg);
				rc = -EINVAL;
				goto exit;
			}
		} while (status_reg == SONY_CAMERA_EEPROM_PREPROCESS_STATUS_PREPARATION ||
			status_reg == SONY_CAMERA_EEPROM_PREPROCESS_STATUS_IDLE);

		rc = sony_camera_i2c_read(&camera_data[id],
			camera_info[id].eeprom_addr,
			SONY_CAMERA_EEPROM_BAYER_EEPROM_STATUS,
			CAMERA_SENSOR_I2C_TYPE_WORD, 1, &status_reg);
		LOGI("SONY_CAMERA_EEPROM_BAYER_EEPROM_STATUS:%d\n", (int)status_reg);
		if (rc < 0) {
			LOGE("Read BAYER_EEPROM_STATUS, fail rc: %d\n", rc);
			goto exit;
		}

		if (status_reg & (1 << SONY_CAMERA_EEPROM_STATUS_SHIFT_SIZE_2K))
			len = 2048;
		else if (status_reg & (1 << SONY_CAMERA_EEPROM_STATUS_SHIFT_SIZE_4K))
			len = 4096;
		else {
			LOGE("Can't get eeprom size from BAYER_EEPROM_STATUS\n");
			len = 2048;
		}

		//Read main sensor.
		d = camera_data[id].eeprom;
		rc = sony_camera_i2c_read_eeprom(&camera_data[id],
			camera_info[id].eeprom_addr,
			SONY_CAMERA_EEPROM_BAYER_EEPROM_OFFSET,
			CAMERA_SENSOR_I2C_TYPE_WORD, len, d);
		if (rc < 0) {
			LOGE("eeprom i2c read type %d fail %d\n",
				camera_info[id].eeprom_type, rc);
			camera_data[id].eeprom_len = 0;
			goto exit;
		}
		sony_camera_hex_dump((uint8_t *)d, len);
		LOGI("MAIN MODULE NAME: len=%d, name=%c%c%c%c%c%c%c%c\n",
			len, d[0], d[1], d[2], d[3],
			d[4], d[5], d[6], d[7]);

		rc = sony_camera_i2c_read(&camera_data[id],
			camera_info[id].eeprom_addr,
			SONY_CAMERA_EEPROM_BW_EEPROM_STATUS,
			CAMERA_SENSOR_I2C_TYPE_WORD,
			1, &status_reg);
		LOGI("SONY_CAMERA_EEPROM_BW_EEPROM_STATUS:%d\n", (int)status_reg);
		if (rc < 0) {
			LOGE("Read BAYER_EEPROM_STATUS, fail rc: %d\n", rc);
			goto exit;
		}

		if (status_reg & (1 << SONY_CAMERA_EEPROM_STATUS_SHIFT_SIZE_2K))
			len = 2048;
		else if (status_reg & (1 << SONY_CAMERA_EEPROM_STATUS_SHIFT_SIZE_4K))
			len = 4096;
		else {
			LOGE("Can't get eeprom size from BW_EEPROM_STATUS\n");
			len = 2048;
		}

		//Read sub sensor.
		d = &camera_data[id].eeprom[4096];
		rc = sony_camera_i2c_read_eeprom(&camera_data[id],
			camera_info[id].eeprom_addr,
			SONY_CAMERA_EEPROM_BW_EEPROM_OFFSET,
			CAMERA_SENSOR_I2C_TYPE_WORD, len, d);
		if (rc < 0) {
			LOGE("eeprom i2c read type %d fail %d\n",
				camera_info[id].eeprom_type, rc);
			camera_data[id].eeprom_len = 0;
			goto exit;
		}
		sony_camera_hex_dump((uint8_t *)d, len);
		LOGI("SUB MODULE NAME: len=%d, name=%c%c%c%c%c%c%c%c\n",
			len, d[0], d[1], d[2], d[3],
			d[4], d[5], d[6], d[7]);
		if (rc < 0) {
			LOGE("eeprom type %d i2c read fail %d\n",
			camera_info[id].eeprom_type, rc);
			camera_data[id].eeprom_len = 0;
			goto exit;
		}
		d = camera_data[id].eeprom;
		len = SONY_CAMERA_MAX_EEPROM_DATA;

		//Read main sensor lot_id and sub sensor lot_id.
		rc = sony_camera_i2c_read_eeprom(&camera_data[id],
			camera_info[id].eeprom_addr,
			0x0820,
			CAMERA_SENSOR_I2C_TYPE_WORD,
			0x12, camera_data[id].lot_id);
		if (rc < 0) {
			LOGE("Lot ID read fail\n");
			goto exit;
		} else {
			camera_data[id].lot_id_available = true;
		}
		sony_camera_hex_dump((uint8_t *)camera_data[id].lot_id, 0x12);
	}

	/* identify sensor module */
	for (i = 0; i < camera_info[id].modules_num; i++) {
		if (!strncmp(camera_info[id].modules[i].name,
			camera_data[id].eeprom, SONY_CAMERA_NAME_LEN)) {
			camera_data[id].module =
				&camera_info[id].modules[i];
			LOGD("detected sensor module name\n");
			break;
		}
	}

	if (camera_data[id].module)
		camera_data[id].eeprom_len = len;
	else {
		LOGE("Module name not recognized. Force name.\n");

		camera_data[id].module = &camera_info[id].modules[0];

		for (i = 0; i < camera_info[id].modules_num; i++) {
			if (!strncmp(camera_info[id].modules[i].name,
				camera_info[id].default_module_name,
				SONY_CAMERA_NAME_LEN)) {
				/* Check module name for CHI05BN1, because
				 some modules are invalid name */
				if (!strncmp(camera_info[id].modules[i].name,
					"CHI05BN1",
					SONY_CAMERA_NAME_LEN)) {
					memcpy(camera_data[id].eeprom,
						camera_info[id].default_module_name,
						SONY_CAMERA_NAME_LEN);
				}
				camera_data[id].module
					= &camera_info[id].modules[i];
				LOGD("detected sensor Force name\n");
				break;
			}
		}
		camera_data[id].eeprom_len = len;
	}

	LOGI("MODULE NAME: len=%d, name=%c%c%c%c%c%c%c%c\n",
		len, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);

exit:
	return rc;
}

static int sony_camera_clear(struct sony_camera_data *data)
{
	int rc = 0;
	unsigned long flags;
	spin_lock_irqsave(&camera_data->event_list_lock, flags);
	while (!list_empty(&data->event_available)) {
		struct sony_camera_event_list_data *event = list_entry(
			data->event_available.next,
			struct sony_camera_event_list_data, list);

		list_del(&event->list);
		kfree(event);
	}
	spin_unlock_irqrestore(&camera_data->event_list_lock, flags);
	spin_lock_irqsave(&camera_data->sof_lock, flags);
	data->sof_count = 0;
	spin_unlock_irqrestore(&camera_data->sof_lock, flags);

	return rc;
}

static int sony_camera_power_down(struct sony_camera_data *data)
{
	int rc = 0;

	if (data->state == SONY_CAMERA_STATE_POWER_DOWN) {
	    goto exit;
	}
	mutex_lock(&data->state_lock);
	data->state = SONY_CAMERA_STATE_POWER_DOWN;
	mutex_unlock(&data->state_lock);

	rc = sony_camera_power_ctrl(data, false);
	if (rc < 0)
		LOGE("power_down fail\n");

	if (camera_info[data->id].use_spi || data->has_hw_sof) {
		rc = sony_camera_irq_deinit(data,
			&camera_info[data->id]);
		if (rc < 0)
			LOGE("%s: irq_deinit failed\n", __func__);
	}

	if (camera_info[data->id].use_spi && data->sensor_task) {
		rc = kthread_stop(data->sensor_task);
		if (rc < 0)
			LOGE("kthread_stop failed %d\n", rc);
		sony_camera_flush_task_queue(data);
		if (data->tx_buf)
			kfree(data->tx_buf);
		if (data->rx_buf)
			kfree(data->rx_buf);
		if (data->spi_header)
			kfree(data->spi_header);
		if (data->spi_payload)
			kfree(data->spi_payload);
	}
	if (data->i2c_data)
		kfree(data->i2c_data);
	rc = sony_camera_cci_deinit(data);
	if (rc < 0)
		LOGE("%s cci_deinit failed\n", __func__);

	rc = pinctrl_select_state(data->pinctrl,
		data->gpio_state_suspend);
	if (rc)
		pr_err("%s:%d cannot set pin to suspend state",
			__func__, __LINE__);

	rc = sony_camera_gpio_deinit(data);
	if (rc < 0)
		LOGE("%s: gpio_deinit failed\n", __func__);
	mutex_lock(&data->thermal_lock);
	data->thermal_ret_val = -ENODEV;
	mutex_unlock(&data->thermal_lock);
	sony_camera_clear(data);

exit:

	return rc;
}

static int sony_camera_power_up(struct sony_camera_data *data)
{
	int rc = 0;
	uint8_t retry_cnt = SONY_CAMERA_MAX_RETRY_COUNT;

	if (data->state == SONY_CAMERA_STATE_POWER_UP) {
	    goto exit;
	}

	LOGD("%s: %d\n", __func__, __LINE__);
	rc = sony_camera_gpio_init(data);
	if (rc < 0) {
		LOGE("%s: gpio_init failed\n", __func__);
		sony_camera_gpio_deinit(data);
		goto exit;
	}

	rc = pinctrl_select_state(data->pinctrl,
		data->gpio_state_active);
	if (rc) {
		LOGE("%s:%d cannot set pin to active state\n",
			__func__, __LINE__);
		sony_camera_gpio_deinit(data);
		pinctrl_select_state(data->pinctrl,
			data->gpio_state_suspend);
		goto exit;
	}

	LOGI("%s: id = %d, has_hw_sof %d use_spi %d %p\n", __func__,
		data->id, data->has_hw_sof, camera_info[data->id].use_spi,
		dev_name(&data->p_dev->dev));

	if (camera_info[data->id].use_spi) {
		while (!(data->tx_buf =
			kzalloc(SONY_CAMERA_SPI_TX_MAX_LEN, GFP_KERNEL | GFP_DMA)) && retry_cnt) {
			retry_cnt--;
			usleep_range(1000, 3000);
			LOGE("Retry kmalloc tx_buf\n");
		}
		if (data->tx_buf == NULL) {
			LOGE("%s: tx_buf kmalloc failed\n", __func__);
			return -ENOMEM;
		}
		retry_cnt = SONY_CAMERA_MAX_RETRY_COUNT;
		while (!(data->rx_buf =
			kzalloc(SONY_CAMERA_SPI_RX_MAX_LEN, GFP_KERNEL | GFP_DMA)) && retry_cnt) {
			retry_cnt--;
			usleep_range(1000, 3000);
			LOGE("Retry kmalloc rx_buf\n");
		}
		if (data->rx_buf == NULL) {
			LOGE("%s: rx_buf kmalloc failed\n", __func__);
			return -ENOMEM;
		}
		retry_cnt = SONY_CAMERA_MAX_RETRY_COUNT;
		while (!(data->spi_header = kzalloc(SONY_CAMERA_SPI_HEADER_SIZE, GFP_KERNEL)) &&
				retry_cnt) {
			retry_cnt--;
			usleep_range(1000, 3000);
			LOGE("Retry kmalloc spi header\n");
		}
		if (!data->spi_header) {
			LOGE("%s: spi header kmalloc failed\n", __func__);
			return -ENOMEM;
		}
		retry_cnt = SONY_CAMERA_MAX_RETRY_COUNT;
		while (!(data->spi_payload = kzalloc(SONY_CAMERA_SPI_PAYLOAD_SIZE, GFP_KERNEL)) &&
				retry_cnt) {
			retry_cnt--;
			usleep_range(1000, 3000);
			LOGE("Retry kmalloc spi payload\n");
		}
		if (!data->spi_payload) {
			LOGE("%s: spi payload kmalloc failed\n", __func__);
			return -ENOMEM;
		}
		data->sensor_task =
			kthread_run(sony_camera_task_handler,
				data, "sony_camera_task");
		if (IS_ERR(data->sensor_task)) {
			LOGE("%s:%d failed to run kernel thread, rc = %d\n",
				__func__, __LINE__, rc);
			rc = PTR_ERR(data->sensor_task);
			goto exit;
		}
		reinit_completion(&data->spi_assert_complete);
		reinit_completion(&data->spi_deassert_complete);
		if (!data->probe_done)
			reinit_completion(&data->gp_assert_complete);
	}
	if (camera_info[data->id].use_spi || data->has_hw_sof) {
		rc = sony_camera_irq_init(data, &camera_info[data->id]);
		if (rc < 0) {
			LOGE("irq_init failed\n");
			goto exit;
		}
	}

	rc = sony_camera_cci_init(data);
	if (rc < 0) {
		LOGE("%s cci_init failed\n", __func__);
		sony_camera_gpio_deinit(data);
		pinctrl_select_state(data->pinctrl,
			data->gpio_state_suspend);
		sony_camera_cci_deinit(data);
		goto exit;
	}

	retry_cnt = SONY_CAMERA_MAX_RETRY_COUNT;
	while (!(data->i2c_data = kzalloc(SONY_CAMERA_I2C_MAX_SIZE, GFP_KERNEL)) &&
			retry_cnt) {
		retry_cnt--;
		usleep_range(1000, 3000);
		LOGE("Retry kmalloc i2c data\n");
	}
	if (!data->i2c_data) {
		LOGE("%s: i2c data kmalloc failed\n", __func__);
		return -ENOMEM;
	}
	rc = sony_camera_power_ctrl(data, true);
	if (rc < 0) {
		LOGE("power_up fail\n");
		sony_camera_power_down(data);
		goto exit;
	}
	mutex_lock(&data->state_lock);
	data->state = SONY_CAMERA_STATE_POWER_UP;
	mutex_unlock(&data->state_lock);
	mutex_lock(&data->thermal_lock);
	data->thermal_ret_val = -EINVAL;
	mutex_unlock(&data->thermal_lock);

exit:

	return rc;
}

static int sony_camera_open(struct inode* inode, struct file* file)
{
	int rc = 0;
	unsigned int id = iminor(inode);
	file->private_data = &camera_data[id];

	mutex_lock(&camera_data[id].command_lock);

	if (camera_data[id].open_count == 0) {
		device_init_wakeup(&camera_data[id].p_dev->dev, 1);
		pm_stay_awake(&camera_data[id].p_dev->dev);
	}
	camera_data[id].open_count++;

	mutex_unlock(&camera_data[id].command_lock);

	LOGI("sensor opened %d\n", id);
	return rc;
}

static int sony_camera_close(struct inode* inode, struct file* file)
{
	int rc = 0;
	struct sony_camera_data* data = (struct sony_camera_data*)file->private_data;

	mutex_lock(&data->command_lock);

	if (0 < data->open_count) {
		data->open_count--;
	}
	if (data->open_count == 0) {
		sony_camera_power_down(data);
		pm_relax(&data->p_dev->dev);
		device_init_wakeup(&data->p_dev->dev, 0);
	}

	mutex_unlock(&data->command_lock);

	LOGI("sensor closed\n");
	return rc;
}

static unsigned int sony_camera_poll(struct file *file,
	struct poll_table_struct *wait)
{
	unsigned int rc = 0;
	unsigned long flags;
	int empty = 0;
	struct sony_camera_data* data = (struct sony_camera_data *)file->private_data;

	spin_lock_irqsave(&camera_data->event_list_lock, flags);
	empty = list_empty(&data->event_available);
	spin_unlock_irqrestore(&camera_data->event_list_lock, flags);
	if (empty) {
		poll_wait(file, &data->event_wait_q, wait);
	}
	spin_lock_irqsave(&camera_data->event_list_lock, flags);
	empty = list_empty(&data->event_available);
	spin_unlock_irqrestore(&camera_data->event_list_lock, flags);
	if (!empty) {
		rc = POLLPRI;
	}
	return rc;
}

static long sony_camera_ioctl_common(struct file *file,
	unsigned int cmd, unsigned long parm, bool compat)
{
	int rc = 0;
	unsigned long flags;
	struct sony_camera_data *data = (struct sony_camera_data *)file->private_data;
	struct sony_camera_i2c_data setting;

	if (!data->probe_done) {
		LOGE("device probe not done\n");
		rc = -ENODEV;
		goto exit;
	}

	switch (cmd) {
	case SONY_CAMERA_CMD_POWER_UP:
		LOGI("sensor power up %d\n", data->id);
		rc = sony_camera_power_up(data);
		break;
	case SONY_CAMERA_CMD_POWER_DOWN:
		LOGI("sensor power down %d\n", data->id);
		rc = sony_camera_power_down(data);
		break;
	case SONY_CAMERA_CMD_I2C_READ:
		LOGI("sensor I2C read id=%d\n", data->id); // TODO: change LOGD
		rc = copy_from_user(&setting,
			(void __user *) parm, sizeof(setting));
		if (rc < 0) {
			LOGE("%s: copy_from_user failed\n", __func__);
			goto exit;
		}
		rc = sony_camera_i2c_read(data,
			setting.slave_addr, setting.addr, setting.addr_type,
			setting.len, setting.data);
		if (rc < 0) {
			LOGE("%s: sony_camera_i2c_read failed\n", __func__);
			goto exit;
		}
		rc = copy_to_user((void __user *) parm,
			&setting, sizeof(setting));
		if (rc < 0)
			LOGE("%s: copy_to_user failed\n", __func__);
		break;
	case SONY_CAMERA_CMD_I2C_WRITE:
		// TODO: copy_from_user
		// TODO: copy_from_user for data
//		LOGI("sensor I2C write id=%d\n", data->id); // TODO: change LOGD
		rc = copy_from_user(&setting,
			(void __user *) parm, sizeof(setting));
		if (rc < 0) {
			LOGE("%s: copy_from_user failed\n", __func__);
			goto exit;
		}
		rc = sony_camera_i2c_write(data,
			setting.slave_addr, setting.addr, setting.addr_type,
			setting.len, setting.data);
		break;
	case SONY_CAMERA_CMD_DOWNLOAD_FW:
	{
		struct file *fp = NULL;
		uint32_t size = 0;
		char *fw_buf = NULL;
		struct sony_camera_firmware_download_data *firmware = NULL;

		firmware = vzalloc(sizeof(struct sony_camera_firmware_download_data));
		if (firmware == NULL) {
			LOGE("no memroy to alloc firmware\n");
			rc = -ENOMEM;
			break;
		}

		rc = copy_from_user(firmware, (void __user *) parm,
			sizeof(struct sony_camera_firmware_download_data));
		if (rc < 0) {
			vfree(firmware);
			break;
		}

		LOGI("Update fireware:%s.\n", firmware->fw_path);
		fp = filp_open(firmware->fw_path, O_RDONLY, 0644);
		if (IS_ERR(fp)) {
			LOGE("%s:%d Open file error.\n", __func__, __LINE__);
			vfree(firmware);
			rc = -EIO;
			break;
		}

		size = fp->f_path.dentry->d_inode->i_size;
		fw_buf = vzalloc(size);
		if (!fw_buf) {
			LOGE("%s:%d No memory\n", __func__, __LINE__);
			filp_close(fp, NULL);
			vfree(firmware);
			rc = -ENOMEM;
			break;
		}

		LOGI("Firmware size:%d.\n", size);
		if (kernel_read(fp, 0, fw_buf, size) != size) {
			LOGE(KERN_INFO "Failed to read %s\n",
				 firmware->fw_path);
			filp_close(fp, NULL);
			vfree(fw_buf);
			vfree(firmware);
			rc = -EIO;
			break;
		}

		rc = sony_camera_spi_write_with_status(data,
			fw_buf, size, SPI_ASSERT);

		filp_close(fp, NULL);
		vfree(fw_buf);
		vfree(firmware);
		break;
	}
	case SONY_CAMERA_CMD_SPI_READ:
	{
		struct spi_device *spi_dev =
			data->spi_master;
		uint32_t spi_tx_width =
			spi_dev->bits_per_word / 8;
		uint32_t header_size;
		uint32_t payload_size;
		uint32_t aligned_size;
		bool need_free_header = FALSE;
		bool need_free_payload = FALSE;
		void *header = NULL;
		void *payload = NULL;

		if (compat) {
			struct sony_camera_spi_data32 camera_spi_data;
			rc = copy_from_user(&camera_spi_data,
				(void *) parm, sizeof(camera_spi_data));
			if (rc < 0)
				break;
			header_size =
				camera_spi_data.header_size;
			payload_size =
				camera_spi_data.payload_size;
			aligned_size =
				(payload_size + spi_tx_width - 1) & ~(spi_tx_width - 1);
			if (header_size > SONY_CAMERA_SPI_HEADER_SIZE) {
				header = vzalloc(header_size);
				if (header == NULL) {
					LOGE("%s:%d vzalloc(%d) failed\n",
						__func__, __LINE__, header_size);
					rc = -ENOMEM;
					break;
				}
				need_free_header = TRUE;
			} else {
				memset(data->spi_header, 0, SONY_CAMERA_SPI_HEADER_SIZE);
				header = data->spi_header;
			}
			rc = copy_from_user(header,
				(void *)compat_ptr(camera_spi_data.header),
					header_size);
			if (rc < 0) {
				if (need_free_header)
					vfree(header);
				break;
			}
			if (aligned_size > SONY_CAMERA_SPI_PAYLOAD_SIZE) {
				payload = vzalloc(aligned_size);
				if (payload == NULL) {
					if (need_free_header)
						vfree(header);
					LOGE("%s:%d vzalloc(%d) failed\n",
						__func__, __LINE__, aligned_size);
					rc = -ENOMEM;
					break;
				}
				need_free_payload = TRUE;

			} else {
				memset(data->spi_payload, 0, SONY_CAMERA_SPI_PAYLOAD_SIZE);
				payload = data->spi_payload;
			}
			memset(payload, 0, aligned_size);

			LOGI("Begin read header_size = %d\n", header_size);
			rc = sony_camera_spi_write_with_status(data,
				(const uint8_t *)header, header_size, SPI_DEASSERT);
			LOGI("End read header_size = %d rc = %d\n", header_size, rc);
			if (rc < 0) {
				LOGE("%s:%d sony_camera_spi_write_with_status failed\n",
					__func__, __LINE__);
				if (need_free_header)
					vfree(header);
				if (need_free_payload)
					vfree(payload);
				break;
			}

			LOGI("Begin read payload payload_size = %d\n", payload_size);
			rc = sony_camera_spi_read_with_status(data,
				payload, aligned_size, SPI_ASSERT);
			LOGI("End read payload payload_size = %d rc = %d\n",
				payload_size, rc);
			if (rc < 0) {
				LOGE("%s:%d sony_camera_spi_write_with_status failed\n",
					__func__, __LINE__);
				if (need_free_header)
					vfree(header);
				if (need_free_payload)
					vfree(payload);
				break;
			}
			if (copy_to_user((void *)compat_ptr(camera_spi_data.payload),
				payload, camera_spi_data.payload_size)) {
				if (need_free_header)
					vfree(header);
				if (need_free_payload)
					vfree(payload);
				rc = -EFAULT;
				break;
			}
		} else {
			struct sony_camera_spi_data camera_spi_data;
			rc = copy_from_user(&camera_spi_data,
				(void *) parm, sizeof(camera_spi_data));
			if (rc < 0)
				break;
			header_size =
				camera_spi_data.header_size;
			payload_size =
				camera_spi_data.payload_size;
			aligned_size =
				(payload_size + spi_tx_width - 1) & ~(spi_tx_width - 1);

			if (header_size > SONY_CAMERA_SPI_HEADER_SIZE) {
				header = vzalloc(header_size);
				if (header == NULL) {
					LOGE("%s:%d vzalloc(%d) failed\n",
						__func__, __LINE__, header_size);
					rc = -ENOMEM;
					break;
				}
				need_free_header = TRUE;
			} else {
				memset(data->spi_header, 0, SONY_CAMERA_SPI_HEADER_SIZE);
				header = data->spi_header;
			}
			rc = copy_from_user(header,
				(void *)camera_spi_data.header,
					header_size);
			if (rc < 0) {
				if (need_free_header)
					vfree(header);
				break;
			}
			if (aligned_size > SONY_CAMERA_SPI_PAYLOAD_SIZE) {
				payload = vzalloc(aligned_size);
				if (payload == NULL) {
					if (need_free_header)
						vfree(header);
					LOGE("%s:%d vzalloc(%d) failed\n",
						__func__, __LINE__, aligned_size);
					rc = -ENOMEM;
					break;
				}
				need_free_payload = TRUE;

			} else {
				memset(data->spi_payload, 0, SONY_CAMERA_SPI_PAYLOAD_SIZE);
				payload = data->spi_payload;
			}
			memset(payload, 0, aligned_size);

			LOGI("Begin read header_size = %d\n", header_size);
			rc = sony_camera_spi_write_with_status(data,
				(const uint8_t *)header, header_size, SPI_DEASSERT);
			LOGI("End read header_size = %d rc = %d\n", header_size, rc);
			if (rc < 0) {
				LOGE("%s:%d sony_camera_spi_write_with_status failed\n",
					__func__, __LINE__);
				if (need_free_header)
					vfree(header);
				if (need_free_payload)
					vfree(payload);
				break;
			}

			LOGI("Begin read payload payload_size = %d\n", payload_size);
			rc = sony_camera_spi_read_with_status(data,
				payload, aligned_size, SPI_ASSERT);
			LOGI("End read payload payload_size = %d rc = %d\n",
				payload_size, rc);
			if (rc < 0) {
				LOGE("%s:%d sony_camera_spi_write_with_status failed\n",
					__func__, __LINE__);
				if (need_free_header)
					vfree(header);
				if (need_free_payload)
					vfree(payload);
				break;
			}
			if (copy_to_user((void *)camera_spi_data.payload,
				payload, camera_spi_data.payload_size)) {
				if (need_free_header)
					vfree(header);
				if (need_free_payload)
					vfree(payload);
				rc = -EFAULT;
				break;
			}
		}
		if (need_free_header)
			vfree(header);
		if (need_free_payload)
			vfree(payload);
		break;
	}
	case SONY_CAMERA_CMD_SPI_WRITE:
	{
		struct spi_device *spi_dev =
			data->spi_master;
		uint32_t spi_tx_width =
			spi_dev->bits_per_word / 8;
		uint32_t header_size;
		uint32_t payload_size;
		uint32_t aligned_size;
		bool need_free_header = FALSE;
		bool need_free_payload = FALSE;
		void *header = NULL;
		void *payload = NULL;

		if (compat) {
			struct sony_camera_spi_data32 camera_spi_data;

			rc = copy_from_user(&camera_spi_data,
				(void __user *) parm, sizeof(camera_spi_data));
			if (rc < 0)
				break;

			header_size =
				camera_spi_data.header_size;
			payload_size =
				camera_spi_data.payload_size;
			aligned_size =
				(payload_size + spi_tx_width - 1) & ~(spi_tx_width - 1);
			if (header_size > SONY_CAMERA_SPI_HEADER_SIZE) {
				header = vzalloc(header_size);
				if (header == NULL) {
					LOGE("%s:%d vzalloc(%d) failed\n",
						__func__, __LINE__, header_size);
					rc = -ENOMEM;
					break;
				}
				need_free_header = TRUE;
			} else {
				memset(data->spi_header, 0, SONY_CAMERA_SPI_HEADER_SIZE);
				header = data->spi_header;
			}
			rc = copy_from_user(header,
				(void *)compat_ptr(camera_spi_data.header),
					header_size);
			if (rc < 0) {
				if (need_free_header)
					vfree(header);
				break;
			}
			if (aligned_size > SONY_CAMERA_SPI_PAYLOAD_SIZE) {
				payload = vzalloc(aligned_size);
				if (payload == NULL) {
					LOGE("%s:%d vzalloc(%d) failed\n",
						__func__, __LINE__, aligned_size);
					rc = -ENOMEM;
					if (need_free_header)
						vfree(header);
					break;
				}
				need_free_payload = TRUE;
			} else {
				memset(data->spi_payload, 0, SONY_CAMERA_SPI_PAYLOAD_SIZE);
				payload = data->spi_payload;
			}
			rc = copy_from_user(payload,
				(void *)compat_ptr(camera_spi_data.payload),
					camera_spi_data.payload_size);
			if (rc < 0) {
				if (need_free_header)
					vfree(header);
				if (need_free_payload)
					vfree(payload);
				rc = -EFAULT;
				break;
			}
		} else {
			struct sony_camera_spi_data camera_spi_data;

			rc = copy_from_user(&camera_spi_data,
				(void __user *) parm, sizeof(camera_spi_data));
			if (rc < 0)
				break;

			header_size =
				camera_spi_data.header_size;
			payload_size =
				camera_spi_data.payload_size;
			aligned_size =
				(payload_size + spi_tx_width - 1) & ~(spi_tx_width - 1);
			if (header_size > SONY_CAMERA_SPI_HEADER_SIZE) {
				header = vzalloc(header_size);
				if (header == NULL) {
					LOGE("%s:%d vzalloc(%d) failed\n",
						__func__, __LINE__, header_size);
					rc = -ENOMEM;
					break;
				}
				need_free_header = TRUE;
			} else {
				memset(data->spi_header, 0, SONY_CAMERA_SPI_HEADER_SIZE);
				header = data->spi_header;
			}
			rc = copy_from_user(header,
				(void *)camera_spi_data.header,
					header_size);
			if (rc < 0) {
				if (need_free_header)
					vfree(header);
				break;
			}
			if (aligned_size > SONY_CAMERA_SPI_PAYLOAD_SIZE) {
				payload = vzalloc(aligned_size);
				if (payload == NULL) {
					LOGE("%s:%d vzalloc(%d) failed\n",
						__func__, __LINE__, aligned_size);
					rc = -ENOMEM;
					if (need_free_header)
						vfree(header);
					break;
				}
				need_free_payload = TRUE;
			} else {
				memset(data->spi_payload, 0, SONY_CAMERA_SPI_PAYLOAD_SIZE);
				payload = data->spi_payload;
			}
			rc = copy_from_user(payload,
				(void *)camera_spi_data.payload,
					camera_spi_data.payload_size);
			if (rc < 0) {
				if (need_free_header)
					vfree(header);
				if (need_free_payload)
					vfree(payload);
				rc = -EFAULT;
				break;
			}
		}

		LOGD("Begin write header_size = %d\n", header_size);
		rc = sony_camera_spi_write_with_status(data,
			(const uint8_t *)header, header_size, SPI_DEASSERT);
		LOGD("End write header_size = %d rc = %d\n", header_size, rc);
		if (rc < 0) {
			LOGE("%s:%d sony_camera_spi_write_with_status failed\n",
				__func__, __LINE__);
			if (need_free_header)
				vfree(header);
			if (need_free_payload)
				vfree(payload);
			break;
		}

		LOGD("Begin write payload payload_size = %d\n", payload_size);
		rc = sony_camera_spi_write_with_status(data,
			(const uint8_t *)payload, aligned_size, SPI_ASSERT);
		LOGD("End write payload payload_size = %d rc = %d\n",
			payload_size, rc);
		if (need_free_header)
			vfree(header);
		if (need_free_payload)
			vfree(payload);
		break;
	}
	case SONY_CAMERA_CMD_GET_EEPROM:
		// TODO: copy_to_user
		LOGI("sensor eeprom get id=%d\n", data->id); // TODO: change LOGD
		rc = copy_to_user((void __user*) parm,
			data->eeprom, data->eeprom_len);
		if (!rc)
			rc = data->eeprom_len;
		break;
	case SONY_CAMERA_CMD_GET_EVENT:
		// Get event from list to User data copy
		spin_lock_irqsave(&camera_data->event_list_lock, flags);
		if (list_empty(&data->event_available)) {
			rc = -ENODATA;
			spin_unlock_irqrestore(&camera_data->event_list_lock, flags);
		} else {
			struct sony_camera_event_list_data *event = list_entry(
				data->event_available.next,
				struct sony_camera_event_list_data, list);
			list_del(&event->list);
			spin_unlock_irqrestore(&camera_data->event_list_lock, flags);
			if (compat) {
				struct sony_camera_event32 event_data32;
				memset(&event_data32, 0, sizeof(event_data32));
				event_data32.type = event->event_data.type;
				event_data32.data.sof_data.sof_count = event->event_data.data.sof_data.sof_count;
				event_data32.data.sof_data.mono_timestamp.tv_sec = event->event_data.data.sof_data.mono_timestamp.tv_sec;
				event_data32.data.sof_data.mono_timestamp.tv_usec = event->event_data.data.sof_data.mono_timestamp.tv_usec;
				rc = copy_to_user((void __user *) parm,
					&event_data32 , sizeof(event_data32));
			} else {
				rc = copy_to_user((void __user *) parm,
					&event->event_data , sizeof(event->event_data));
			}
			kfree(event);
		}
		break;
	case SONY_CAMERA_CMD_SET_THERMAL:
		mutex_lock(&data->thermal_lock);
		data->thermal_ret_val = 0;
		rc = copy_from_user(data->thermal_sensor_temperature,
			(void __user *) parm, sizeof(data->thermal_sensor_temperature));
		mutex_unlock(&data->thermal_lock);
		if (rc < 0) {
			LOGE("%s:%d copy_from_user failed\n", __func__, __LINE__);
			rc = -EINVAL;
		}
		break;
	case SONY_CAMERA_CMD_GET_LOT_ID:
		// Support dual camera only
		if (!data->lot_id_available) {
			rc = -ENOENT;
		} else {
			LOGI("get lot id from camera%d\n", data->id);
			rc = copy_to_user((void __user*) parm,
				data->lot_id, 0x12);
			if (!rc) {
				LOGE("%s:%d copy_to_user failed\n", __func__, __LINE__);
				rc = -EINVAL;
			}
		}
		break;

	case SONY_CAMERA_CMD_CLEAR:
		rc = sony_camera_clear(data);
		break;
	case SONY_CAMERA_CMD_BUG_ON:
		BUG_ON(1);
		break;
	default:
		rc = -EINVAL;
		break;
	}
exit:
	return rc;
}

static long sony_camera_ioctl(struct file* file,
	unsigned int cmd, unsigned long parm)
{
	unsigned int rc = 0;
	struct sony_camera_data *data = (struct sony_camera_data *)file->private_data;

	mutex_lock(&data->command_lock);
	rc = sony_camera_ioctl_common(file, cmd, parm, false);
	mutex_unlock(&data->command_lock);

	return rc;
}

static long sony_camera_ioctl32(struct file* file,
	unsigned int cmd, unsigned long parm)
{
	unsigned int rc = 0;
	struct sony_camera_data *data = (struct sony_camera_data *)file->private_data;

	mutex_lock(&data->command_lock);
	rc = sony_camera_ioctl_common(file, cmd, parm, true);
	mutex_unlock(&data->command_lock);

	return rc;
}

static struct file_operations sony_camera_fops =
{
	.owner   = THIS_MODULE,
	.poll    = sony_camera_poll,
	.open    = sony_camera_open,
	.release = sony_camera_close,
#ifdef CONFIG_COMPAT
	.compat_ioctl = sony_camera_ioctl32,
#endif	/* CONFIG_COMPAT */
	.unlocked_ioctl = sony_camera_ioctl,
};

static const struct of_device_id sony_camera_0_dt_match[] = {
	{
		.compatible = "sony_camera_0",
		.data = &camera_data[0]
	},
	{
	},
};

static const struct of_device_id sony_camera_1_dt_match[] = {
	{
		.compatible = "sony_camera_1",
		.data = &camera_data[1]
	},
	{
	},
};

static const struct of_device_id sony_camera_spi_dt_match[] = {
	{
		.compatible = "sony_aube"
	},
	{ },
};

MODULE_DEVICE_TABLE(of, sony_camera_0_dt_match);
MODULE_DEVICE_TABLE(of, sony_camera_1_dt_match);
MODULE_DEVICE_TABLE(of, sony_camera_spi_dt_match);



static int sony_camera_spi_probe(struct spi_device *spi)
{
	int rc = 0;
	uint32_t id = 0;

	spi->bits_per_word = 32;
	spi_setup(spi);
	for (id = 0; id < 2; id++) {
		if (camera_info[id].use_spi)
			camera_data[id].spi_master = spi;
	}
	return rc;
}

static struct spi_driver sony_camera_spi_driver = {
	.driver = {
		.name = "sony_camera_spi",
		.owner = THIS_MODULE,
		.of_match_table = sony_camera_spi_dt_match,
	},
	.probe = sony_camera_spi_probe,
};

static struct platform_driver sony_camera_platform_driver[] = {
	{
		.driver = {
			.name = "sony_camera_0",
			.owner = THIS_MODULE,
			.of_match_table = sony_camera_0_dt_match,
		},
	},
	{
		.driver = {
			.name = "sony_camera_1",
			.owner = THIS_MODULE,
			.of_match_table = sony_camera_1_dt_match,
		},
	},
};

static void sony_camera_platform_cleanup(void)
{
	uint16_t i;
	uint16_t j;

	for (i = 0; i < sensor_num; i++) {
		platform_driver_unregister(&sony_camera_platform_driver[i]);
		sony_camera_info_deinit(i);
		for (j = 0; j < SONY_CAMERA_THERMAL_NUM; j++) {
			if (camera_data[i].thermal_zone_dev[j]) {
				thermal_zone_device_unregister(camera_data[i].thermal_zone_dev[j]);
			}
		}
	}
}

static int sony_camera_platform_probe(struct platform_device *p_dev)
{
	int rc = 0;
	uint32_t id = 0;
	const struct of_device_id *match;
	char *thermal_name[2] = {SONY_CAMERA_THERMAL_NAME_0,
		SONY_CAMERA_THERMAL_NAME_1};

	match = of_match_device(sony_camera_0_dt_match, &p_dev->dev);
	if (!match && 1 < sensor_num) {
		match = of_match_device(sony_camera_1_dt_match, &p_dev->dev);
		id = 1;
	}
	if (!match) {
		LOGE("of_match_device fail\n");
		rc = -EFAULT;
		goto fail;
	}

	camera_data[id].id = id;
	camera_data[id].p_dev = p_dev;
	camera_data[id].probe_done = false;

	rc = sony_camera_info_init(p_dev, id);
	if (rc < 0) {
		LOGE("%s sony_camera_info_init failed %d\n",
			__func__, __LINE__);
		goto fail;
	}
	// CCI initialize
	camera_data[id].cci_info.cci_subdev = cam_cci_get_subdev();
	// camera_data[id].cci_info.freq = ??;//
	camera_data[id].cci_info.i2c_freq_mode = camera_info[id].modules[0].i2c_freq_mode;
	camera_data[id].cci_info.sid = 0;
	camera_data[id].cci_info.cid = 0;
	camera_data[id].cci_info.timeout = 0;
	camera_data[id].cci_info.retries = 3;
	camera_data[id].cci_info.id_map = 0;
	camera_data[id].pinctrl = devm_pinctrl_get(&p_dev->dev);
	if (IS_ERR_OR_NULL(camera_data[id].pinctrl)) {
		LOGE("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		rc = -EINVAL;
		goto fail;
	}
	camera_data[id].gpio_state_active =
		pinctrl_lookup_state(camera_data[id].pinctrl,
			SONY_CAMERA_PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(camera_data[id].gpio_state_active)) {
		LOGE("%s:%d Failed to get the active state pinctrl handle\n",
			__func__, __LINE__);
		rc = -EINVAL;
		goto fail;
	}
	camera_data[id].gpio_state_suspend =
		pinctrl_lookup_state(camera_data[id].pinctrl,
			SONY_CAMERA_PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(camera_data[id].gpio_state_suspend)) {
		LOGE("%s:%d Failed to get the suspend state pinctrl handle\n",
			__func__, __LINE__);
		rc = -EINVAL;
		goto fail;
	}

	if (camera_info[id].use_spi) {
		/* register spi driver for sensor */
		spi_register_driver(&sony_camera_spi_driver);
	}

	rc = sony_camera_power_up(&camera_data[id]);
	if (rc < 0) {
		LOGE("%s sony_camera_power_up failed %d\n",
			__func__, __LINE__);
		goto fail;
	}

	rc = sony_camera_eeprom_load(id);
	if (rc < 0) {
		LOGE("%s sony_camera_info_init failed %d\n",
			__func__, __LINE__);
		sony_camera_power_down(&camera_data[id]);
		goto fail;
	}

	rc = sony_camera_power_down(&camera_data[id]);
	if (rc < 0) {
		LOGE("%s sony_camera_power_down failed %d\n",
			__func__, __LINE__);
		goto fail;
	}

	if (camera_info[id].use_spi) {
		char *thermal_name_aube[3] = {SONY_CAMERA_THERMAL_NAME_0,
			SONY_CAMERA_THERMAL_NAME_2,
			SONY_CAMERA_THERMAL_NAME_3};
		uint16_t i;
		for (i = 0; i < SONY_CAMERA_THERMAL_NUM; i++) {
			camera_data[id].thermal_zone_dev[i] =
				thermal_zone_device_register(thermal_name_aube[i],
				0, 0, 0, &sony_camera_thermal_ops, 0, 0, 0);
			if (IS_ERR(camera_data[id].thermal_zone_dev[i])) {
				LOGE("%s thermal_zone_device_register (%u) %d\n",
					__func__, i, __LINE__);
				rc = PTR_ERR(camera_data[id].thermal_zone_dev[i]);
				goto fail;
			}
		}
	} else {
		camera_data[id].thermal_zone_dev[0] =
			thermal_zone_device_register(thermal_name[id],
			0, 0, 0, &sony_camera_thermal_ops, 0, 0, 0);
		if (IS_ERR(camera_data[id].thermal_zone_dev[0])) {
			LOGE("%s thermal_zone_device_register (%u) %d\n",
				__func__, id, __LINE__);
			rc = PTR_ERR(camera_data[id].thermal_zone_dev[0]);
			goto fail;
		}
	}

	camera_data[id].probe_done = true;
	LOGI("camera %d probe ok\n", id);

	return 0;

fail:
	return rc;
}

static int __init sony_camera_init_module(void)
{
	int rc = 0;
	uint16_t i;
	uint16_t probe_count = 0;

	sensor_num = ARRAY_SIZE(sony_camera_platform_driver);

	for (i = 0; i < sensor_num; i++) {
		camera_data[i].state = SONY_CAMERA_STATE_POWER_DOWN;
		camera_data[i].taskq_idx = 0;
		camera_data[i].open_count = 0;
		mutex_init(&camera_data[i].command_lock);
		spin_lock_init(&camera_data[i].task_lock);
		spin_lock_init(&camera_data[i].event_list_lock);
		mutex_init(&camera_data[i].thermal_lock);
		spin_lock_init(&camera_data[i].sof_lock);
		mutex_init(&camera_data[i].state_lock);
		init_waitqueue_head(&camera_data[i].event_wait_q);
		init_waitqueue_head(&camera_data[i].wait_q);
		INIT_LIST_HEAD(&camera_data[i].event_available);
		INIT_LIST_HEAD(&camera_data[i].task_q);
		init_completion(&camera_data[i].spi_assert_complete);
		init_completion(&camera_data[i].spi_deassert_complete);
		if (!camera_data[i].probe_done)
			init_completion(&camera_data[i].gp_assert_complete);
		rc = platform_driver_probe(
			&sony_camera_platform_driver[i],
			sony_camera_platform_probe);
		if (rc < 0) {
			LOGE("%s platform_driver_probe (%u) %d\n",
				__func__, i, __LINE__);
			continue;
		}
		probe_count++;
		msleep(SONY_CAMERA_FRONT_SENSOR_POWER_UP_WAIT);
	}

	if (!probe_count) {
		LOGE("%s platform_driver_probe (%u) %d\n",
			__func__, probe_count, __LINE__);
		goto fail_probe;
	}

	/* memory allocation */
	camera_device = platform_device_alloc(SONY_CAMERA_DEV_NAME, -1);
	if (!camera_device) {
		LOGE("%s platform_device_alloc() failed.\n", __func__);
		rc = -ENOMEM;
		goto fail_probe;
	}
	dev_id = camera_device->id;

	/* add device */
	rc = platform_device_add(camera_device);
	if (rc) {
		LOGE("platform_device_add() failed.\n");
		goto fail_platform_dev_add;
	}

	/* create the node of device */
	camera_device_class = class_create(THIS_MODULE, SONY_CAMERA_DEV_NAME);
	if (IS_ERR(camera_device_class)) {
		LOGE("%s class_create() failed.\n", __func__);
		rc = PTR_ERR(camera_device_class);
		goto fail_class_create;
	}

	/* create the logical device */
	for (i = 0; i < sensor_num; i++) {
		struct device *dev = NULL;
		dev = device_create(camera_device_class, NULL, MKDEV(MAJOR(dev_id), i), NULL, SONY_CAMERA_DEV_NAME"%d", i);
		if (IS_ERR(dev)) {
			LOGE("%s device_create() failed.\n", __func__);
			rc = PTR_ERR(dev);
			goto fail_device_create;
		}
	}

	/* register the driver */
	if (register_chrdev(dev_id, SONY_CAMERA_DEV_NAME,
			&sony_camera_fops)) {
		pr_err("register_chrdev() failed (Major:%d).\n",
				dev_id);
		rc = -EINVAL;
		goto fail_register_chrdev;
	}


	return 0;

fail_register_chrdev:
	for (i = 0; i < sensor_num; i++) {
		device_destroy(camera_device_class, MKDEV(dev_id,i));
	}
fail_device_create:
	class_destroy(camera_device_class);
fail_class_create:
	platform_device_del(camera_device);
fail_platform_dev_add:
	platform_device_put(camera_device);
	sony_camera_platform_cleanup();
fail_probe:
	return rc;
}

static void __exit sony_camera_exit_module(void)
{
	uint16_t i;

	sony_camera_platform_cleanup();
	for (i = 0; i < sensor_num; i++) {
		dev_t dn = MKDEV(MAJOR(dev_id), i);
		device_destroy(c, dn);
		mutex_destroy(&camera_data[i].command_lock);
		mutex_destroy(&camera_data[i].thermal_lock);
		mutex_destroy(&camera_data[i].state_lock);
	}
	class_destroy(c);
	cdev_del(&cdev);
	unregister_chrdev_region(dev_id, sensor_num);
}

#ifdef MODULE
module_init(sony_camera_init_module);
module_exit(sony_camera_exit_module);
#else
late_initcall(sony_camera_init_module);
#endif

MODULE_DESCRIPTION("SONY camera sensor driver");
MODULE_LICENSE("GPL v2");
