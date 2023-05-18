/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/rtio/rtio.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>


#define BME280_DT_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(bosch_bme280)

RTIO_DEFINE_WITH_MEMPOOL(r, 1, 1, 1, 32, 4);
SENSOR_DT_READ_IODEV(bme280_io, BME280_DT_NODE,
		     SENSOR_CHAN_AMBIENT_TEMP, SENSOR_CHAN_PRESS, SENSOR_CHAN_HUMIDITY);

static const struct sensor_decoder_api *decoder;

/*
 * Get a device structure from a devicetree node with compatible
 * "bosch,bme280". (If there are multiple, just pick one.)
 */
static const struct device *get_bme280_device(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(bosch_bme280);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

#define B(x) (int32_t)FIELD_GET(GENMASK64(63, 31), x)
#define L(x) (int32_t)FIELD_GET(GENMASK(30, 0), x)

void process(int result, uint8_t *buf, uint32_t buf_len, void *userdata)
{
	sensor_frame_iterator_t fit = {0};
	sensor_channel_iterator_t cit = {0};
	enum sensor_channel channels[3];
	q31_t values[3];
	int8_t shift[3];

	decoder->decode(buf, &fit, &cit, channels, values, 3);

	decoder->get_shift(buf, channels[0], &shift[0]);
	decoder->get_shift(buf, channels[0], &shift[1]);
	decoder->get_shift(buf, channels[0], &shift[2]);

	int64_t scaled[] = {
		values[0] << shift[0],
		values[1] << shift[1],
		values[2] << shift[2]
	};

	printk("temp: %d.%06d; press: %d.%06d; humidity: %d.%06d\n",
	       B(scaled[0]), L(scaled[0]),
	       B(scaled[1]), L(scaled[1]),
	       B(scaled[2]), L(scaled[2]));

}

int main(void)
{
	const struct device *dev = get_bme280_device();

	sensor_get_decoder(dev, &decoder);

	if (dev == NULL) {
		return 0;
	}

	while (1) {
		sensor_read(&bme280_io, &r, NULL);

		sensor_processing_with_callback(&r, process);

		k_sleep(K_MSEC(1000));
	}
	return 0;
}
