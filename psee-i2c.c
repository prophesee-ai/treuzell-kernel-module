// SPDX-License-Identifier: GPL-2.0
/*
 * psee-i2c.c - Prophesee sensor I2C interface
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
#include <linux/i2c.h>

#include "treuzell.h"

static struct psee_i2c_data {
	int reg_addr_size;
} addr16bits = {16},
addr32bits = {32};

static const struct of_device_id psee_i2c_dt_ids[] = {
	{ .compatible = "psee,i2c-sensor-huahine", .data = &addr32bits, },
	{ .compatible = "psee,i2c-sensor-saphir", .data = &addr16bits, },
	{ .compatible = "psee,i2c-sensor", .data = NULL, },
	{ }
};

MODULE_DEVICE_TABLE(of, psee_i2c_dt_ids);

/* ids are indices in psee_i2c_dt_ids */
static struct i2c_device_id psee_i2c_idtable[] = {
	{ "huahine", 0 },
	{ "saphir", 1 },
	{ }
};

static struct i2c_client *latest_probed_sensor;
struct i2c_client *psee_i2c_get_latest_sensor(void)
{
	return latest_probed_sensor;
}

int psee_i2c_read_regs(struct i2c_client *sensor, u32 reg, u32 *val, u8 nval)
{
	int i, ret;
	struct i2c_msg xfer[2] = {0};
	u16 reg16;
	struct psee_i2c_data *pdata = dev_get_drvdata(&sensor->dev);

	xfer[0].addr = sensor->addr;
	if (pdata->reg_addr_size == 16) {
		reg16 = cpu_to_be16(reg);
		xfer[0].buf = (u8 *)&reg16;
		xfer[0].len = sizeof(reg16);
	} else if (pdata->reg_addr_size == 32) {
		reg = cpu_to_be32(reg);
		xfer[0].buf = (u8 *)&reg;
		xfer[0].len = sizeof(reg);
	} else {
		return -EINVAL;
	}
	xfer[1].addr = sensor->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].buf = (u8 *)val;
	xfer[1].len = nval * sizeof(*val);

	ret = i2c_transfer(sensor->adapter, xfer, 2);
	if (ret != 2) {
		dev_warn(&sensor->dev, "read ret %d\n", ret);
		ret = (ret < 0) ? ret : -1;
	} else {
		for (i = 0; i < nval; i++)
			val[i] = be32_to_cpu(val[i]);
		ret = nval;
	}

	return ret;
}

int psee_i2c_write_regs(struct i2c_client *sensor, u32 reg, u32 *val, u8 nval)
{
	int i, ret;
	struct i2c_msg xfer = {0};
	u16 *buf16 = NULL;
	u32 *buf32 = NULL;
	struct psee_i2c_data *pdata = dev_get_drvdata(&sensor->dev);

	xfer.addr = sensor->addr;

	if (pdata->reg_addr_size == 16) {
		buf16 = devm_kmalloc_array(&sensor->dev,
			(nval*2) + 1, sizeof(u16), GFP_KERNEL);

		if (!buf16)
			return -1;

		buf16[0] = cpu_to_be16(reg);
		for (i = 0; i < nval; i++) {
			buf16[(2*i)+1] = cpu_to_be16(val[i]>>16);
			buf16[(2*i)+2] = cpu_to_be16(val[i]&0xFFFF);
		}
		xfer.buf = (u8 *)buf16;
		xfer.len = 2 + (nval * sizeof(*val));
	} else if (pdata->reg_addr_size == 32) {
		buf32 = devm_kmalloc_array(&sensor->dev,
			nval + 1, sizeof(*val), GFP_KERNEL);

		if (!buf32)
			return -1;

		buf32[0] = cpu_to_be32(reg);
		for (i = 0; i < nval; i++)
			buf32[i+1] = cpu_to_be32(val[i]);
		xfer.buf = (u8 *)buf32;
		xfer.len = (nval + 1) * sizeof(*val);
	} else {
		return -1;
	}



	ret = i2c_transfer(sensor->adapter, &xfer, 1);
	if (ret != 1) {
		dev_warn(&sensor->dev, "write ret %d\n", ret);
		ret = (ret < 0) ? ret : -1;
	} else {
		ret = nval;
	}
	if (buf32)
		devm_kfree(&sensor->dev, buf32);
	if (buf16)
		devm_kfree(&sensor->dev, buf16);
	return ret;
}

int psee_i2c_set_freq(struct i2c_client *sensor, u32 max_speed_hz)
{
#ifdef I2C_ADAPTER_HAS_SET_FREQ
	return i2c_set_freq(sensor->adapter, max_speed_hz);
#endif
	return -EOPNOTSUPP;
}

/**
 *	sysfs_emit - scnprintf equivalent, aware of PAGE_SIZE buffer.
 *	@buf:	start of PAGE_SIZE buffer.
 *	@fmt:	format
 *	@...:	optional arguments to @format
 *
 *
 * Returns number of characters written to @buf.
 */
static int sysfs_emit(char *buf, const char *fmt, ...)
{
	va_list args;
	int len;

	if (WARN(!buf, "invalid sysfs emit buf:%p\n", buf))
		return 0;

	va_start(args, fmt);
	len = vscnprintf(buf, PAGE_SIZE, fmt, args);
	va_end(args);

	return len;
}

static ssize_t
address_size_bits_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct psee_i2c_data *pdata = dev_get_drvdata(dev);

	if (!pdata)
		return -1;

	return sysfs_emit(buf, "%d\n", pdata->reg_addr_size);
}

static ssize_t address_size_bits_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t size)
{
	int ret;
	int new;
	struct psee_i2c_data *pdata = dev_get_drvdata(dev);

	if (!pdata)
		return -1;

	ret = kstrtoint(buf, 0, &new);
	if (ret)
		return ret;

	if ((new != 16) && (new != 32))
		return -EINVAL;
	pdata->reg_addr_size = new;
	/* Always return full write size even if we didn't consume all */
	return size;
}

DEVICE_ATTR_RW(address_size_bits);

static int psee_i2c_probe(struct i2c_client *client)
{
	struct psee_i2c_data *pdata;
	const struct i2c_device_id *id;
	struct device_node *of_node = client->dev.of_node;

	pdata = devm_kmalloc(&client->dev, sizeof(struct psee_i2c_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	/* don't actually probe the sensor. Power up sequence is managed by the
	 * FPGA
	 */

	/* Select an address size on the I2C/CCI bus */
	id = i2c_match_id(psee_i2c_idtable, client);
	if (of_node) {
		const struct of_device_id *match;

		/* prefer info from the device tree */
		match = of_match_node(psee_i2c_dt_ids, of_node);
		if (match && match->data)
			*pdata = *((struct psee_i2c_data *)match->data);
		else
			*pdata = addr32bits;
		/* don't end log with a dot if we use default value */
		dev_info(&client->dev, "Addr size %dbits from devicetree%c\n",
			pdata->reg_addr_size, (match && match->data)?'.':' ');
	} else if (id) {
		/* try to get data by name (manually probed) */
		const struct psee_i2c_data *id_data = psee_i2c_dt_ids[id->driver_data].data;
		*pdata = *id_data;
		dev_info(&client->dev, "Addr size %dbits from id\n", pdata->reg_addr_size);
	} else {
		/* assume it's 32bits, that was legacy */
		*pdata = addr32bits;
		dev_info(&client->dev, "Addr size %dbits by default\n", pdata->reg_addr_size);
	}

	dev_set_drvdata(&client->dev, pdata);

	if (device_create_file(&client->dev, &dev_attr_address_size_bits) < 0)
		dev_warn(&client->dev, "Could not add address size sysfs entry\n");

	latest_probed_sensor = client;
	return 0;
}

static int psee_i2c_remove(struct i2c_client *client)
{
	device_remove_file(&client->dev, &dev_attr_address_size_bits);
	return 0;
}

static struct i2c_driver psee_i2c_driver = {
	.driver = {
		.name	= "psee_i2c",
		.of_match_table = psee_i2c_dt_ids,
	},
	.id_table       = psee_i2c_idtable,
	.probe_new	= psee_i2c_probe,
	.remove		= psee_i2c_remove,
};

int psee_i2c_register_driver(void)
{
	return i2c_add_driver(&psee_i2c_driver);
}

void psee_i2c_unregister_driver(void)
{
	i2c_del_driver(&psee_i2c_driver);
}
