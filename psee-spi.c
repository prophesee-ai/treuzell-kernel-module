// SPDX-License-Identifier: GPL-2.0
/*
 * psee-spi.c - Prophesee sensor SPI interface
 *
 * Copyright (C) 2020 by Prophesee
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include "treuzell.h"

#define HUAHINE_MAX_FREQ_AT_BOOT 1250000

static const struct of_device_id psee_spi_dt_ids[] = {
	{ .compatible = "psee,spi-sensor", .data = NULL, },
	{ .compatible = "psee,spi-sensor-huahine", .data = NULL, },
	{ }
};

MODULE_DEVICE_TABLE(of, psee_spi_dt_ids);

static struct spi_device *latest_probed_sensor;
struct spi_device *psee_spi_get_latest_sensor(void)
{
	return latest_probed_sensor;
}

int psee_spi_read_regs(struct spi_device *sensor, u32 reg, u32 *val, u8 nval)
{
	struct spi_transfer xfer[2] = {0};
	int i, ret;

	/* mutate address to add command */
	reg >>= 2;
	reg |= (1lu << 31); /* read operation */
	reg |= ((nval > 1) << 30); /* burst operation */
	reg = cpu_to_be32(reg);
	xfer[0].tx_buf = &reg;
	xfer[0].len = sizeof(reg);
	xfer[0].delay_usecs = 4; /* 64clk @20MHz (lowest clock) */
	xfer[1].rx_buf = val;
	xfer[1].len = nval * sizeof(*val);

	ret = spi_sync_transfer(sensor, xfer, 2);

	for (i = 0; i < nval; i++)
		val[i] = be32_to_cpu(val[i]);

	return ret;
}

int psee_spi_write_regs(struct spi_device *sensor, u32 reg, u32 *val, u8 nval)
{
	struct spi_transfer xfer[2] = {0};
	int i, ret;

	for (i = 0; i < nval; i++)
		val[i] = be32_to_cpu(val[i]);
	/* mutate address to add command */
	reg >>= 2;
	reg |= ((nval > 1) << 30); /* burst operation */
	reg = cpu_to_be32(reg);
	xfer[0].tx_buf = &reg;
	xfer[0].len = sizeof(reg);
	xfer[1].tx_buf = val;
	xfer[1].len = nval * sizeof(*val);

	ret = spi_sync_transfer(sensor, xfer, 2);

	return ret;
}

int psee_spi_set_freq(struct spi_device *sensor, u32 max_speed_hz)
{
	if (!max_speed_hz)
		max_speed_hz = HUAHINE_MAX_FREQ_AT_BOOT;
	sensor->max_speed_hz = max_speed_hz;
	return 0;
}

static int psee_spi_probe(struct spi_device *spi)
{
	int ret = 0;
	/* don't actually probe the sensor. Power up sequence is managed by the
	 * FPGA
	 */
	if (!spi->max_speed_hz)
		spi->max_speed_hz = HUAHINE_MAX_FREQ_AT_BOOT;
	latest_probed_sensor = spi;
	return ret;
}

static int psee_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver psee_spi_driver = {
	.driver = {
		.name	= "psee_spi",
		.of_match_table = psee_spi_dt_ids,
	},
	.probe		= psee_spi_probe,
	.remove		= psee_spi_remove,
};

int psee_spi_register_driver(void)
{
	return spi_register_driver(&psee_spi_driver);
}

void psee_spi_unregister_driver(void)
{
	spi_unregister_driver(&psee_spi_driver);
}
