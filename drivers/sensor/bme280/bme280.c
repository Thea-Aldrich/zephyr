/* sensor_bmp280.c - Driver for Bosch BMP280 temperature and pressure sensor */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <kernel.h>
#include <i2c.h>
#include <sensor.h>
#include <init.h>
#include <gpio.h>
#include <misc/byteorder.h>
#include <misc/__assert.h>

#include "bme280.h"

/*
 * Compensation code taken from BME280 datasheet, Section 4.2.3
 * "Compensation formula".
 */
static void bme280_compensate_temp(struct bme280_data *data, int32_t adc_temp)
{
	int32_t var1, var2;

	var1 = (((adc_temp >> 3) - ((int32_t)data->dig_t1 << 1)) *
		((int32_t)data->dig_t2)) >> 11;
	var2 = (((((adc_temp >> 4) - ((int32_t)data->dig_t1)) *
		  ((adc_temp >> 4) - ((int32_t)data->dig_t1))) >> 12) *
		((int32_t)data->dig_t3)) >> 14;

	data->t_fine = var1 + var2;
	data->comp_temp = (data->t_fine * 5 + 128) >> 8;
}

static void bme280_compensate_press(struct bme280_data *data, int32_t adc_press)
{
	int64_t var1, var2, p;

	var1 = ((int64_t)data->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)data->dig_p6;
	var2 = var2 + ((var1 * (int64_t)data->dig_p5) << 17);
	var2 = var2 + (((int64_t)data->dig_p4) << 35);
	var1 = ((var1 * var1 * (int64_t)data->dig_p3) >> 8) +
		((var1 * (int64_t)data->dig_p2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)data->dig_p1) >> 33;

	/* Avoid exception caused by division by zero. */
	if (var1 == 0) {
		data->comp_press = 0;
		return;
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)data->dig_p9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)data->dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)data->dig_p7) << 4);

	data->comp_press = (uint32_t)p;
}

static void bme280_compensate_humidity(struct bme280_data *data,
				       int32_t adc_humidity)
{
	int32_t h;

	h = (data->t_fine - ((int32_t)76800));
	h = ((((adc_humidity << 14) - (((int32_t)data->dig_h4) << 20) -
		(((int32_t)data->dig_h5) * h)) + ((int32_t)16384)) >> 15) *
		(((((((h * ((int32_t)data->dig_h6)) >> 10) * (((h *
		((int32_t)data->dig_h3)) >> 11) + ((int32_t)32768))) >> 10) +
		((int32_t)2097152)) * ((int32_t)data->dig_h2) + 8192) >> 14);
	h = (h - (((((h >> 15) * (h >> 15)) >> 7) *
		((int32_t)data->dig_h1)) >> 4));
	h = (h > 419430400 ? 419430400 : h);

	data->comp_humidity = (uint32_t)(h >> 12);
}

static int bme280_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct bme280_data *data = dev->driver_data;
	uint8_t buf[8];
	int32_t adc_press, adc_temp, adc_humidity;
	int size = 6;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	if (data->chip_id == BME280_CHIP_ID) {
		size = 8;
	}

	if (i2c_burst_read(data->i2c_master, data->i2c_slave_addr,
			   BME280_REG_PRESS_MSB, buf, size) < 0) {
		return -EIO;
	}

	adc_press = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
	adc_temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);

	bme280_compensate_temp(data, adc_temp);
	bme280_compensate_press(data, adc_press);

	if (data->chip_id == BME280_CHIP_ID) {
		adc_humidity = (buf[6] << 8) | buf[7];
		bme280_compensate_humidity(data, adc_humidity);
	}

	return 0;
}

static int bme280_channel_get(struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bme280_data *data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_TEMP:
		/*
		 * data->comp_temp has a resolution of 0.01 degC.  So
		 * 5123 equals 51.23 degC.
		 */
		val->val1 = data->comp_temp / 100;
		val->val2 = data->comp_temp % 100 * 10000;
		break;
	case SENSOR_CHAN_PRESS:
		/*
		 * data->comp_press has 24 integer bits and 8
		 * fractional.  Output value of 24674867 represents
		 * 24674867/256 = 96386.2 Pa = 963.862 hPa
		 */
		val->val1 = (data->comp_press >> 8) / 1000;
		val->val2 = (data->comp_press >> 8) % 1000 * 1000 +
			(((data->comp_press & 0xff) * 1000) >> 8);
		break;
	case SENSOR_CHAN_HUMIDITY:
		/*
		 * data->comp_humidity has 22 integer bits and 10
		 * fractional.  Output value of 47445 represents
		 * 47445/1024 = 46.333 %RH
		 */
		val->val1 = (data->comp_humidity >> 10);
		val->val2 = (((data->comp_humidity & 0x3ff) * 1000 * 1000) >> 10);
		val->val1 = val->val1 * 1000 + (val->val2 * 1000) / 1000000;
		val->val2 = (val->val2 * 1000) % 1000000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct sensor_driver_api bme280_api_funcs = {
	.sample_fetch = bme280_sample_fetch,
	.channel_get = bme280_channel_get,
};

static int bme280_read_compensation(struct bme280_data *data)
{
	uint16_t buf[12];
	uint8_t hbuf[7];
	int err = 0;

	err = i2c_burst_read(data->i2c_master, data->i2c_slave_addr,
			     BME280_REG_COMP_START,
			     (uint8_t *)buf, sizeof(buf));

	if (err < 0) {
		return err;
	}

	data->dig_t1 = sys_le16_to_cpu(buf[0]);
	data->dig_t2 = sys_le16_to_cpu(buf[1]);
	data->dig_t3 = sys_le16_to_cpu(buf[2]);

	data->dig_p1 = sys_le16_to_cpu(buf[3]);
	data->dig_p2 = sys_le16_to_cpu(buf[4]);
	data->dig_p3 = sys_le16_to_cpu(buf[5]);
	data->dig_p4 = sys_le16_to_cpu(buf[6]);
	data->dig_p5 = sys_le16_to_cpu(buf[7]);
	data->dig_p6 = sys_le16_to_cpu(buf[8]);
	data->dig_p7 = sys_le16_to_cpu(buf[9]);
	data->dig_p8 = sys_le16_to_cpu(buf[10]);
	data->dig_p9 = sys_le16_to_cpu(buf[11]);

	if (data->chip_id == BME280_CHIP_ID) {
		err = i2c_reg_read_byte(data->i2c_master, data->i2c_slave_addr,
					BME280_REG_HUM_COMP_PART1,
					&data->dig_h1);

		if (err < 0) {
			return err;
		}

		err = i2c_burst_read(data->i2c_master, data->i2c_slave_addr,
				     BME280_REG_HUM_COMP_PART2, hbuf, 7);

		if (err < 0) {
			return err;
		}

		data->dig_h2 = (hbuf[1] << 8) | hbuf[0];
		data->dig_h3 = hbuf[2];
		data->dig_h4 = (hbuf[3] << 4) | (hbuf[4] & 0x0F);
		data->dig_h5 = ((hbuf[4] >> 4) & 0x0F) | (hbuf[5] << 4);
		data->dig_h6 = hbuf[6];
	}

	return 0;
}

static int bme280_chip_init(struct device *dev)
{
	struct bme280_data *data = (struct bme280_data *) dev->driver_data;

	int err = i2c_reg_read_byte(data->i2c_master, data->i2c_slave_addr,
				    BME280_REG_ID, &data->chip_id);

	if (err < 0) {
		return err;
	}

	if (data->chip_id == BME280_CHIP_ID) {
		SYS_LOG_DBG("BME280 chip detected");
	} else if (data->chip_id == BMP280_CHIP_ID_MP ||
		   data->chip_id == BMP280_CHIP_ID_SAMPLE_1) {
		SYS_LOG_DBG("BMP280 chip detected");
	} else {
		SYS_LOG_DBG("bad chip id 0x%x", data->chip_id);
		return -ENOTSUP;
	}

	err = bme280_read_compensation(data);

	if (err < 0) {
		return err;
	}

	if (data->chip_id == BME280_CHIP_ID) {
		i2c_reg_write_byte(data->i2c_master, data->i2c_slave_addr,
				   BME280_REG_CTRL_HUM, BME280_HUMIDITY_OVER);
	}

	i2c_reg_write_byte(data->i2c_master, data->i2c_slave_addr,
			   BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS_VAL);
	i2c_reg_write_byte(data->i2c_master, data->i2c_slave_addr,
			   BME280_REG_CONFIG, BME280_CONFIG_VAL);

	return 0;
}

int bme280_init(struct device *dev)
{
	struct bme280_data *data = dev->driver_data;

	data->i2c_master = device_get_binding(CONFIG_BME280_I2C_MASTER_DEV_NAME);
	if (!data->i2c_master) {
		SYS_LOG_DBG("i2c master not found: %s",
			    CONFIG_BME280_I2C_MASTER_DEV_NAME);
		return -EINVAL;
	}

	data->i2c_slave_addr = BME280_I2C_ADDR;

	if (bme280_chip_init(dev) < 0) {
		return -EINVAL;
	}

	dev->driver_api = &bme280_api_funcs;

	return 0;
}

static struct bme280_data bme280_data;

DEVICE_INIT(bme280, CONFIG_BME280_DEV_NAME, bme280_init, &bme280_data,
	    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY);
