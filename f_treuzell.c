// SPDX-License-Identifier: GPL-2.0
/*
 * f_treuzell.c - USB peripheral Treuzell configuration driver
 *
 * Copyright (C) 2019 by Prophesee
 * Derivated from original work from David Brownell and Nokia Corporation
 * under GNU General Public License v2+
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 */

/* #define VERBOSE_DEBUG */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/usb/composite.h>
#include <linux/err.h>
#include <asm/page.h>

/* Just to access to the struct device and its compatible string */
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>

#include "u_f.h"
#include "treuzell.h"
#include "treuzell_command_definition.h"

#define TREUZELL_CTRL_BUFLEN	1024

enum tz_bus {
	STREAMER,
	AXI_BUS,
	I2C_BUS,
	SPI_BUS,
};

enum stream_state {
	STREAMOFF,
	STREAMON,
	STREAMAUTO,
};

struct f_treuzell {
	struct usb_function	function;
	struct device		*video_ifc;
	struct i2c_client	*psee_i2c;
	struct spi_device	*psee_spi;

	struct usb_ep		*ctrl_in_ep;
	struct usb_ep		*ctrl_out_ep;
	struct usb_ep		*in_ep;
	int			cur_alt;
	struct workqueue_struct	*workqueue;
	enum stream_state	stream_state;

	unsigned int board_serial;
	unsigned int version;
	int64_t build_date;
	u32 sensor_address;
	u32 sensor_addrspace_size;
};

struct f_tz_opts {
	struct usb_function_instance func_inst;

	unsigned int board_serial;
	unsigned int version;
	int64_t build_date;

	/*
	 * Allow to catch some register accesses to forward them on a different
	 * bus, such as I2C or SPI
	 */
	u32 sensor_address;
	u32 sensor_addrspace_size;

	/*
	 * Read/write access to configfs attributes is handled by configfs.
	 *
	 * This is to protect the data from concurrent access by read/write
	 * and create symlink/remove symlink.
	 */
	struct mutex		lock;
	int			refcnt;
};

struct f_tz_cmd_work {
	struct work_struct	work;
	struct f_treuzell	*tz;
	struct usb_request	*ctrl_req;
};

struct f_tz_ctrl_buf {
	u32 command;
	u32 size;
	u32 data[];
};


static inline struct f_treuzell *func_to_tz(struct usb_function *f)
{
	return container_of(f, struct f_treuzell, function);
}

static void bulk_cb(struct scatterlist *sg, unsigned int num_sgs, void *pdata);
/*-------------------------------------------------------------------------*/

static struct usb_interface_descriptor treuzell_intf_alt0 = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,

	.bAlternateSetting =	0,
	.bNumEndpoints =	3,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	/* .iInterface		= DYNAMIC */
};

/* full speed support: */

static struct usb_endpoint_descriptor fs_ctrlsrc_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_source_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_ctrlsink_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_treuzell_descs[] = {
	(struct usb_descriptor_header *) &treuzell_intf_alt0,
	(struct usb_descriptor_header *) &fs_ctrlsrc_desc,
	(struct usb_descriptor_header *) &fs_ctrlsink_desc,
	(struct usb_descriptor_header *) &fs_source_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor hs_ctrlsrc_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_source_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_ctrlsink_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *hs_treuzell_descs[] = {
	(struct usb_descriptor_header *) &treuzell_intf_alt0,
	(struct usb_descriptor_header *) &hs_ctrlsrc_desc,
	(struct usb_descriptor_header *) &hs_ctrlsink_desc,
	(struct usb_descriptor_header *) &hs_source_desc,
	NULL,
};

/* super speed support: */

static struct usb_endpoint_descriptor ss_ctrlsrc_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor ss_ctrlsrc_comp_desc = {
	.bLength =		USB_DT_SS_EP_COMP_SIZE,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	.bMaxBurst =		0,
	.bmAttributes =		0,
	.wBytesPerInterval =	0,
};

static struct usb_endpoint_descriptor ss_source_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor ss_source_comp_desc = {
	.bLength =		USB_DT_SS_EP_COMP_SIZE,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	.bMaxBurst =		0,
	.bmAttributes =		0,
	.wBytesPerInterval =	0,
};

static struct usb_endpoint_descriptor ss_ctrlsink_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor ss_ctrlsink_comp_desc = {
	.bLength =		USB_DT_SS_EP_COMP_SIZE,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	.bMaxBurst =		0,
	.bmAttributes =		0,
	.wBytesPerInterval =	0,
};

static struct usb_descriptor_header *ss_treuzell_descs[] = {
	(struct usb_descriptor_header *) &treuzell_intf_alt0,
	(struct usb_descriptor_header *) &ss_ctrlsrc_desc,
	(struct usb_descriptor_header *) &ss_ctrlsrc_comp_desc,
	(struct usb_descriptor_header *) &ss_ctrlsink_desc,
	(struct usb_descriptor_header *) &ss_ctrlsink_comp_desc,
	(struct usb_descriptor_header *) &ss_source_desc,
	(struct usb_descriptor_header *) &ss_source_comp_desc,
	NULL,
};

/* function-specific strings: */

static struct usb_string strings_treuzell[] = {
	[0].s = "Sensor control & data sreaming",
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_treuzell = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_treuzell,
};

static struct usb_gadget_strings *treuzell_strings[] = {
	&stringtab_treuzell,
	NULL,
};

/*-------------------------------------------------------------------------*/

static void disable_ep(struct usb_composite_dev *cdev, struct usb_ep *ep)
{
	int			value;

	value = usb_ep_disable(ep);
	if (value < 0)
		DBG(cdev, "disable %s --> %d\n", ep->name, value);
}

static void
treuzell_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_treuzell	*tz = func_to_tz(f);

	if (tz->workqueue) {
		destroy_workqueue(tz->workqueue);
		tz->workqueue = NULL;
	}
}

static int
treuzell_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_treuzell	*tz = func_to_tz(f);
	int	id;
	int ret;

	/* find the atis interface to be forwarded */
	tz->video_ifc = psee_video_get_latest_ifc();
	if (!tz->video_ifc) {
		ERROR(cdev, "Can't find the ATIS interface\n");
		return -ENODEV;
	}

	/* Check if we control the I2C bus to the sensor */
	tz->psee_i2c = psee_i2c_get_latest_sensor();
	if (tz->psee_i2c)
		INFO(cdev, "Got an I2C client for the sensor\n");

	/* Check if we control the SPI bus to the sensor */
	tz->psee_spi = psee_spi_get_latest_sensor();
	if (tz->psee_spi)
		INFO(cdev, "Got an SPI device for the sensor\n");

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	treuzell_intf_alt0.bInterfaceNumber = id;

	/* Allocate a workqueue to handle requests non-blockingly */
	tz->workqueue = alloc_ordered_workqueue("%s_%s_wq", 0,
		cdev->gadget->name, f->name);
	if (!tz->workqueue)
		return -ENOMEM;

	/* allocate bulk endpoints */
	tz->ctrl_in_ep = usb_ep_autoconfig(cdev->gadget, &fs_ctrlsrc_desc);
	if (!tz->ctrl_in_ep)
		goto autoconf_fail;

	tz->ctrl_out_ep = usb_ep_autoconfig(cdev->gadget, &fs_ctrlsink_desc);
	if (!tz->ctrl_out_ep)
		goto autoconf_fail;

	tz->in_ep = usb_ep_autoconfig(cdev->gadget, &fs_source_desc);
	if (!tz->in_ep) {
autoconf_fail:
		ERROR(cdev, "%s: can't autoconfigure on %s\n",
			f->name, cdev->gadget->name);
		treuzell_unbind(c, f);
		return -ENODEV;
	}

	/* support high speed hardware */
	hs_ctrlsrc_desc.bEndpointAddress = fs_ctrlsrc_desc.bEndpointAddress;
	hs_ctrlsink_desc.bEndpointAddress = fs_ctrlsink_desc.bEndpointAddress;
	hs_source_desc.bEndpointAddress = fs_source_desc.bEndpointAddress;

	/* support super speed hardware */
	ss_ctrlsrc_desc.bEndpointAddress =
		fs_ctrlsrc_desc.bEndpointAddress;
	ss_ctrlsink_desc.bEndpointAddress =
		fs_ctrlsink_desc.bEndpointAddress;
	ss_source_desc.bEndpointAddress =
		fs_source_desc.bEndpointAddress;

	ret = usb_assign_descriptors(f, fs_treuzell_descs,
			hs_treuzell_descs, ss_treuzell_descs, NULL);
	if (ret)
		return ret;

	DBG(cdev, "%s speed %s: CTRL_IN/%s, CTRL_OUT/%s, IN/%s\n",
	    (gadget_is_superspeed(c->cdev->gadget) ? "super" :
	     (gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full")),
			f->name, tz->ctrl_in_ep->name, tz->ctrl_out_ep->name,
			tz->in_ep->name);
	return 0;
}

static void
treuzell_free_func(struct usb_function *f)
{
	struct f_tz_opts *opts;

	opts = container_of(f->fi, struct f_tz_opts, func_inst);

	mutex_lock(&opts->lock);
	opts->refcnt--;
	mutex_unlock(&opts->lock);

	usb_free_all_descriptors(f);
	kfree(func_to_tz(f));
}

static enum tz_bus get_bus_from_addr(struct f_treuzell *tz, u32 addr)
{
	if ((addr >= tz->sensor_address) &&
		(addr < (tz->sensor_address + tz->sensor_addrspace_size))) {
		/* If no device was provided, default to the AXI bus */
		/* If both were provided, assume it's huahine_tb, and read the
		 * HUAHINE_CTRL register, field I2C_SPI_SEL to determine what
		 * behvior is expected
		 */
		if (tz->psee_i2c && tz->psee_spi) {
			u32 i2c_spi_sel = 0;

			psee_axi_read_regs(tz->video_ifc, 0x70f030,
				&i2c_spi_sel, 1);
			return (i2c_spi_sel & (1 << 11)) ? SPI_BUS : I2C_BUS;
		}
		if (tz->psee_i2c)
			return I2C_BUS;
		if (tz->psee_spi)
			return SPI_BUS;
	}
	return AXI_BUS;
}

static void treuzell_complete_answer(struct usb_ep *ep, struct usb_request *req)
{
	struct usb_composite_dev	*cdev;
	struct f_treuzell		*tz = ep->driver_data;
	int				status = req->status;

	/* driver_data will be null if ep has been disabled */
	if (tz) {
		cdev = tz->function.config->cdev;
		if (status < 0)
			DBG(cdev, "%s complete --> %d, %d/%d\n", ep->name,
					status, req->actual, req->length);
		else
			VDBG(cdev, "%s complete --> %d, %d/%d\n", ep->name,
					status, req->actual, req->length);
	}

	/* Whatever the result, nothing more to do here, just free memory */
	free_ep_req(ep, req);
}

static struct usb_request *allocate_answer_req(struct f_treuzell *tz)
{

	struct usb_request *req;

	req = alloc_ep_req(tz->ctrl_in_ep, TREUZELL_CTRL_BUFLEN);
	if (!req) {
		/* No mem. Stop accepting new commands, drop this one.
		 * We may just cpu_relax and retry later, but this shouldn't
		 * happen on current system, so I'm ok to get an error early
		 */
		usb_ep_set_halt(tz->ctrl_out_ep);
		return req;
	}
	req->complete = treuzell_complete_answer;
	return req;
}

static int queue_answer_req(struct f_treuzell *tz, struct usb_request *req)
{
	int status = usb_ep_queue(tz->ctrl_in_ep, req, GFP_ATOMIC);

	if (status) {
		/* We'll probably lose sync with host */
		ERROR(tz->function.config->cdev, "queue req in %s fail: %d\n",
				tz->ctrl_in_ep->name, status);
		/* Request deletion won't be handled in completion */
		free_ep_req(tz->ctrl_in_ep, req);
	}

	return status;
}

static
int cmd_get_fpga_state(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	/* FPGA is always on */
	answer->data[0] = 0x00010000;
	answer->size += sizeof(u32);
	answer_req->length += answer->size;

	return queue_answer_req(tz, answer_req);
}

static
int cmd_get_serial(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	answer->data[0] = tz->board_serial;
	answer->size += sizeof(u32);
	answer_req->length += answer->size;

	return queue_answer_req(tz, answer_req);
}

static
int cmd_get_release_version(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	answer->data[0] = tz->version;
	answer->size += sizeof(u32);
	answer_req->length += answer->size;

	return queue_answer_req(tz, answer_req);
}

static
int cmd_get_build_date(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	/* it is assumed that treuzell run on little endian system */
	answer->data[0] = tz->build_date & 0xFFFFFFFF;
	answer->data[1] = (tz->build_date >> 32) & 0xFFFFFFFF;
	answer->size += sizeof(tz->build_date);
	answer_req->length += answer->size;

	return queue_answer_req(tz, answer_req);
}

static
u32 get_device_number(struct f_treuzell *tz)
{
	u32 res = 0;

	if (tz->video_ifc)
		res += 2; //the streamer and the video ifc
	if (tz->psee_i2c)
		res++;
	if (tz->psee_spi)
		res++;

	return res;
}

static
int device2bus(struct f_treuzell *tz, u32 dev_id)
{
	switch (dev_id) {
	case 0:
		return STREAMER;
	case 1:
		return AXI_BUS;
	case 2:
		if (tz->psee_i2c)
			return I2C_BUS;
		if (tz->psee_spi)
			return SPI_BUS;
		return -ENODEV;
	case 3:
		if (tz->psee_i2c && tz->psee_spi)
			return SPI_BUS;
		return -ENODEV;
	default:
		return -ENODEV;
	}
}

static
u32 bus2device(struct f_treuzell *tz, int bus)
{
	switch (bus) {
	case STREAMER:
		return 0;
	case AXI_BUS:
		return 1;
	case I2C_BUS:
		if (tz->psee_i2c)
			return 2;
		return ~0;
	case SPI_BUS:
		if (tz->psee_i2c && tz->psee_spi)
			return 3;
		if (tz->psee_spi)
			return 2;
		return ~0;
	default:
		return ~0;
	}
}

static
void remap_device_and_addr(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	int bus;

	bus = get_bus_from_addr(tz, ctrl->data[1]);
	if (bus != AXI_BUS)
		ctrl->data[1] -= tz->sensor_address;
	ctrl->data[0] = bus2device(tz, bus);
}

static
int cmd_get_devices(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	/* currently, device info is just their number */
	answer->data[0] = get_device_number(tz);
	answer->size += sizeof(u32);
	answer_req->length += answer->size;

	return queue_answer_req(tz, answer_req);
}

static
int cmd_get_device_name(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;
	struct device *dev;
	int status = 0;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	answer->data[0] = ctrl->data[0];
	answer->size += sizeof(u32);

	/* We need a payload with device id */
	if (ctrl->size < sizeof(u32)) {
		status = -EINVAL;
		goto send_answer;
	}

	switch (device2bus(tz, ctrl->data[0])) {
	case STREAMER:
		strscpy((char *)&answer->data[1],
			"Treuzell kernel-side implementation",
			TREUZELL_CTRL_BUFLEN - (answer_req->length + answer->size));
		goto send_answer;
	case AXI_BUS:
		dev = tz->video_ifc;
		break;
	case I2C_BUS:
		dev = &tz->psee_i2c->dev;
		break;
	case SPI_BUS:
		dev = &tz->psee_spi->dev;
		break;
	default:
		status = -ENODEV;
		goto send_answer;
	}
	if (!dev->driver->of_match_table) {
		status = -EOPNOTSUPP;
		goto send_answer;
	}
	strscpy((char *)&answer->data[1],
		dev_name(dev),
		TREUZELL_CTRL_BUFLEN - (answer_req->length + answer->size));

send_answer:
	if (status < 0) {
		answer->command |= TZ_FAILURE_FLAG;
		answer->size += sizeof(u32);
		answer->data[1] = -status;
	} else {
		/* include null terminator in the size */
		answer->size += strlen((char *)&answer->data[1]) + 1;
	}
	answer_req->length += answer->size;

	return queue_answer_req(tz, answer_req);
}

static
int cmd_set_device_if_freq(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;
	int status;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	/* report device and address */
	answer->data[0] = ctrl->data[0];
	answer->size += sizeof(u32);

	/* We need device, frequency */
	if (ctrl->size < (2 * sizeof(u32))) {
		status = -EINVAL;
		goto send_answer;
	}

	/* Reject anything larger than our buffers */
	if (ctrl->size > (TREUZELL_CTRL_BUFLEN - (2 * sizeof(u32)))) {
		status = -E2BIG;
		goto send_answer;
	}

	status = device2bus(tz, ctrl->data[0]);
	switch (status) {
	case STREAMER:
	case AXI_BUS:
		status = -EOPNOTSUPP;
		break;
	case I2C_BUS:
		status = psee_i2c_set_freq(tz->psee_i2c, ctrl->data[1]);
		break;
	case SPI_BUS:
		status = psee_spi_set_freq(tz->psee_spi, ctrl->data[1]);
		break;
	}
send_answer:
	if (status < 0) {
		answer->command |= TZ_FAILURE_FLAG;
		answer->size += sizeof(u32);
		answer->data[1] = -status;
	}
	answer_req->length += answer->size;

	return queue_answer_req(tz, answer_req);
}

static
int cmd_get_device_compatible(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;
	struct device *dev;
	const char *compatible = NULL;
	int status = 0;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	answer->data[0] = ctrl->data[0];
	answer->size += sizeof(u32);

	/* We need a payload with device id */
	if (ctrl->size < sizeof(u32)) {
		status = -EINVAL;
		goto send_answer;
	}

	switch (device2bus(tz, ctrl->data[0])) {
	case STREAMER:
		strscpy((char *)&answer->data[1], "treuzell,streamer",
			TREUZELL_CTRL_BUFLEN - (answer_req->length + answer->size));
		goto send_answer;
	case AXI_BUS:
		dev = tz->video_ifc;
		break;
	case I2C_BUS:
		dev = &tz->psee_i2c->dev;
		break;
	case SPI_BUS:
		dev = &tz->psee_spi->dev;
		break;
	default:
		status = -ENODEV;
		goto send_answer;
	}

	status = of_property_read_string_index(dev->of_node, "compatible",
			0 /*index*/, &compatible);
	if (status)
		goto send_answer;
	strscpy((char *)&answer->data[1], compatible,
		TREUZELL_CTRL_BUFLEN - (answer_req->length + answer->size));

send_answer:
	if (status < 0) {
		answer->command |= TZ_FAILURE_FLAG;
		answer->size += sizeof(u32);
		answer->data[1] = -status;
	} else {
		/* include null terminator in the size */
		answer->size += strlen((char *)&answer->data[1]) + 1;
	}
	answer_req->length += answer->size;

	return queue_answer_req(tz, answer_req);
}

static
int cmd_get_device_reg32(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;
	int status = 0;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	/* report device and address */
	answer->data[0] = ctrl->data[0];
	answer->data[1] = ctrl->data[1];
	answer->size += 2 * sizeof(u32);

	/* We need device, address, size */
	if (ctrl->size < (3 * sizeof(u32))) {
		status = -EINVAL;
		goto send_answer;
	}

	/* Reject anything larger than our buffers */
	if ((ctrl->data[2] + 2) > (TREUZELL_CTRL_BUFLEN / sizeof(u32))) {
		status = -EIO;
		goto send_answer;
	}

	if (ctrl->data[0] == 0) {
		/* We may remap some accesses to other devices */
		remap_device_and_addr(tz, ctrl);
	}
	status = device2bus(tz, ctrl->data[0]);
	switch (status) {
	case STREAMER:
		status = -EOPNOTSUPP;
		break;
	case AXI_BUS:
		status = psee_axi_read_regs(tz->video_ifc,
			ctrl->data[1],
			answer->data + 2, ctrl->data[2]);
		break;
	case I2C_BUS:
		status = psee_i2c_read_regs(tz->psee_i2c,
			ctrl->data[1],
			answer->data + 2, ctrl->data[2]);
		break;
	case SPI_BUS:
		status = psee_spi_read_regs(tz->psee_spi,
			ctrl->data[1],
			answer->data + 2, ctrl->data[2]);
		break;
	default:
		break;
	}
send_answer:
	if (status < 0) {
		answer->command |= TZ_FAILURE_FLAG;
		answer->size += sizeof(u32);
		answer->data[2] = -status;
	} else {
		answer->size += ctrl->data[2] * sizeof(u32);
	}
	answer_req->length += answer->size;

	return queue_answer_req(tz, answer_req);
}

static
int cmd_set_device_reg32(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;
	int status;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	/* report device and address */
	answer->data[0] = ctrl->data[0];
	answer->data[1] = ctrl->data[1];
	answer->size += 2 * sizeof(u32);

	/* We need device, address, at least 1 data */
	if (ctrl->size < (3 * sizeof(u32))) {
		status = -EINVAL;
		goto send_answer;
	}

	/* Reject anything larger than our buffers */
	if (ctrl->size > (TREUZELL_CTRL_BUFLEN - (2 * sizeof(u32)))) {
		status = -E2BIG;
		goto send_answer;
	}

	if (ctrl->data[0] == 0) {
		/* We may remap some accesses to other devices */
		remap_device_and_addr(tz, ctrl);
	}
	status = device2bus(tz, ctrl->data[0]);
	switch (status) {
	case STREAMER:
		status = -EOPNOTSUPP;
		break;
	case AXI_BUS:
		status = psee_axi_write_regs(tz->video_ifc,
			ctrl->data[1],
			ctrl->data + 2,
			(ctrl->size / sizeof(u32))-2);
		break;
	case I2C_BUS:
		status = psee_i2c_write_regs(tz->psee_i2c,
			ctrl->data[1],
			ctrl->data + 2,
			(ctrl->size / sizeof(u32))-2);
		break;
	case SPI_BUS:
		status = psee_spi_write_regs(tz->psee_spi,
			ctrl->data[1],
			ctrl->data + 2,
			(ctrl->size / sizeof(u32))-2);
		break;
	}
send_answer:
	if (status < 0) {
		answer->command |= TZ_FAILURE_FLAG;
		answer->size += sizeof(u32);
		answer->data[2] = -status;
	}
	answer_req->length += answer->size;

	return queue_answer_req(tz, answer_req);
}

static
int cmd_get_device_stream(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;
	int status = 0;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	/* report device */
	answer->data[0] = ctrl->data[0];
	answer->size += sizeof(u32);

	/* We need device */
	if (ctrl->size < sizeof(u32)) {
		status = -EINVAL;
		goto send_answer;
	}

	status = device2bus(tz, ctrl->data[0]);
	switch (status) {
	case STREAMER:
		if ((tz->stream_state == STREAMON) || (tz->stream_state == STREAMAUTO))
			answer->data[1] = 1;
		else
			answer->data[1] = 0;
		break;
	default:
		status = -EOPNOTSUPP;
		break;
	}
send_answer:
	if (status < 0) {
		answer->command |= TZ_FAILURE_FLAG;
		answer->size += sizeof(u32);
		answer->data[1] = -status;
	} else {
		answer->size += sizeof(u32);
	}
	answer_req->length += answer->size;

	return queue_answer_req(tz, answer_req);
}

static
int cmd_set_device_stream(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;
	int status = 0;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	/* report device */
	answer->data[0] = ctrl->data[0];
	answer->size += sizeof(u32);

	/* We need device, streamon/off */
	if (ctrl->size < (2 * sizeof(u32))) {
		status = -EINVAL;
		goto send_answer;
	}

	status = device2bus(tz, ctrl->data[0]);
	switch (status) {
	case STREAMER:
		switch (ctrl->data[1]) {
		case 0:
			psee_video_disable_dma(tz->video_ifc);
			usb_ep_fifo_flush(tz->in_ep);
			tz->stream_state = STREAMOFF;
			break;
		case 1:
			if (tz->stream_state == STREAMAUTO) {
				psee_video_disable_dma(tz->video_ifc);
				usb_ep_fifo_flush(tz->in_ep);
				tz->stream_state = STREAMOFF;
			}
			if (tz->stream_state == STREAMON) {
				status = -EBUSY;
				break;
			}
			status = psee_video_enable_dma(tz->video_ifc, bulk_cb, tz);
			if (status < 0)
				usb_ep_disable(tz->in_ep);
			else
				tz->stream_state = STREAMON;
			break;
		default:
			status = -EOPNOTSUPP;
			break;
		}
		break;
	default:
		status = -EOPNOTSUPP;
		break;
	}
send_answer:
	if (status < 0) {
		answer->command |= TZ_FAILURE_FLAG;
		answer->size += sizeof(u32);
		answer->data[1] = -status;
	} else {
		answer->size += sizeof(u32);
	}
	answer_req->length += answer->size;

	return queue_answer_req(tz, answer_req);
}

static
int legacy_get_regfpga32(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;
	int status = 0;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	/* support for the old CMD_READ, ADDR, no data */
	/* ctrl->size actually contains the addr to read */
	switch (get_bus_from_addr(tz, ctrl->size)) {
	case AXI_BUS:
		status = psee_axi_read_regs(tz->video_ifc,
			ctrl->size, /* actually contains the addr to read */
			answer->data, 1);
		break;
	case I2C_BUS:
		status = psee_i2c_read_regs(tz->psee_i2c,
			/* ctrl->size actually contains the addr to read */
			ctrl->size - tz->sensor_address,
			answer->data, 1);
		break;
	case SPI_BUS:
		status = psee_spi_read_regs(tz->psee_spi,
			/* ctrl->size actually contains the addr to read */
			ctrl->size - tz->sensor_address,
			answer->data, 1);
		break;
	default:
		status = -EINVAL;
		break;
	}
	if (status < 0) {
		answer->command |= TZ_FAILURE_FLAG;
		answer->data[0] = -status;
	}
	answer->size = ctrl->size; /* it's ADDR */
	answer_req->length += sizeof(u32);
	return queue_answer_req(tz, answer_req);
}

static
int legacy_set_regfpga32(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;
	int status = 0;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = ctrl->command;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	/* support for the old CMD_READ, ADDR, DATA */
	/* ctrl->size actually contains the addr to write */
	switch (get_bus_from_addr(tz, ctrl->size)) {
	case AXI_BUS:
		status = psee_axi_write_regs(tz->video_ifc,
			ctrl->size, /* actually contains the addr to write */
			ctrl->data, 1);
		break;
	case I2C_BUS:
		status = psee_i2c_write_regs(tz->psee_i2c,
			/* ctrl->size actually contains the addr to write */
			ctrl->size - tz->sensor_address,
			ctrl->data, 1);
		break;
	case SPI_BUS:
		status = psee_spi_write_regs(tz->psee_spi,
			/* ctrl->size actually contains the addr to write */
			ctrl->size - tz->sensor_address,
			ctrl->data, 1);
		break;
	default:
		status = -EINVAL;
		break;
	}
	if (status < 0) {
		answer->command |= TZ_FAILURE_FLAG;
		answer->data[0] = -status;
	}
	answer->size = ctrl->size; /* it's ADDR */
	answer_req->length += sizeof(u32); /* either data or status */
	return queue_answer_req(tz, answer_req);
}

static
int unknown_command(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl)
{
	struct usb_request *answer_req;
	struct f_tz_ctrl_buf *answer;

	answer_req = allocate_answer_req(tz);
	if (!answer_req)
		return -ENOMEM;
	answer = answer_req->buf;

	answer->command = TZ_UNKNOWN_CMD;
	answer->size = 0;
	answer_req->length = sizeof(ctrl->command) + sizeof(ctrl->size);

	return queue_answer_req(tz, answer_req);
}

static void treuzell_process_board_cmd(struct work_struct *work)
{
	struct f_tz_cmd_work *tz_work =
		container_of(work, struct f_tz_cmd_work, work);
	struct f_treuzell *tz = tz_work->tz;
	struct f_tz_ctrl_buf *ctrl = tz_work->ctrl_req->buf;
	int i;
	int status = -1;

	static const struct {
		u32 cmd;
		int (*fun)(struct f_treuzell *tz, struct f_tz_ctrl_buf *ctrl);
	} commands[] = {
		{TZ_PROP_FPGA_STATE, cmd_get_fpga_state},
		{TZ_PROP_SERIAL, cmd_get_serial},
		{TZ_PROP_RELEASE_VERSION, cmd_get_release_version},
		{TZ_PROP_BUILD_DATE, cmd_get_build_date},
		{TZ_PROP_DEVICES, cmd_get_devices},
		{TZ_PROP_DEVICE_NAME, cmd_get_device_name},
		{TZ_PROP_DEVICE_IF_FREQ | TZ_WRITE_FLAG, cmd_set_device_if_freq},
		{TZ_PROP_DEVICE_COMPATIBLE, cmd_get_device_compatible},
		{TZ_PROP_DEVICE_REG32, cmd_get_device_reg32},
		{(TZ_PROP_DEVICE_REG32 | TZ_WRITE_FLAG), cmd_set_device_reg32},
		{TZ_PROP_DEVICE_STREAM, cmd_get_device_stream},
		{(TZ_PROP_DEVICE_STREAM | TZ_WRITE_FLAG), cmd_set_device_stream},
		{TZ_LEGACY_READ_REGFPGA_32, legacy_get_regfpga32},
		{TZ_LEGACY_WRITE_REGFPGA_32, legacy_set_regfpga32},
		/* command 0 is reserved, and cannot exist */
		{0, unknown_command},
	};

	VDBG(tz->function.config->cdev, "Got cmd 0x%X, size 0x%X, val 0x%X\n",
		ctrl->command, ctrl->size, ctrl->data[0]);

	/* Look for command handling method. */
	for (i = 0; commands[i].cmd; i++)
		if (commands[i].cmd == ctrl->command)
			break;

	status = commands[i].fun(tz, ctrl);
	if (status < 0)
		DBG(tz->function.config->cdev,
			"command 0x%x processing failed: %d\n",
			ctrl->command, status);

	/* clear control request and associated work */
	free_ep_req(tz->ctrl_out_ep, tz_work->ctrl_req);
	kfree(tz_work);
}

static void treuzell_complete_ctrl(struct usb_ep *ep, struct usb_request *req)
{
	struct usb_composite_dev	*cdev;
	struct f_treuzell		*tz = ep->driver_data;
	int				status = req->status;
	struct f_tz_cmd_work		*tz_work;

	/* driver_data will be null if ep has been disabled */
	if (!tz)
		return;

	cdev = tz->function.config->cdev;

	switch (status) {

	case 0:				/* normal completion? */
		if (req->length < (2 * sizeof(u32))) {
			/* We need at least command + size. drop this */
			DBG(tz->function.config->cdev, "request too short\n");
			break;
		}
		tz_work = kzalloc(sizeof(struct f_tz_cmd_work), GFP_ATOMIC);
		if (!tz_work) {
			/* No mem. Stop accepting new commands, drop current */
			free_ep_req(ep, req);
			usb_ep_set_halt(ep);
			ERROR(tz->function.config->cdev,
				"could not allocate tz_work\n");
			return;
		}
		INIT_WORK(&tz_work->work, treuzell_process_board_cmd);
		tz_work->ctrl_req = req;
		tz_work->tz = tz;
		queue_work(tz->workqueue, &tz_work->work);

		/* Allocate a new req, current is referenced by tz_work */
		req = alloc_ep_req(ep, TREUZELL_CTRL_BUFLEN);
		if (!req) {
			/* No mem. Stop accepting new commands */
			usb_ep_set_halt(ep);
			ERROR(tz->function.config->cdev,
				"could not allocate new req for ctrl ep\n");
			return;
		}
		req->complete = treuzell_complete_ctrl;
		break;

	/* this endpoint is normally active while we're configured */
	case -ECONNABORTED:		/* hardware forced ep reset */
	case -ECONNRESET:		/* request dequeued */
	case -ESHUTDOWN:		/* disconnect from host */
		DBG(cdev, "%s gone (%d), %d/%d\n", ep->name, status,
				req->actual, req->length);
		free_ep_req(ep, req);
		return;

	case -EOVERFLOW:		/* buffer overrun on read means that
					 * we didn't provide a big enough
					 * buffer.
					 */
	default:
		DBG(cdev, "%s complete --> %d, %d/%d\n", ep->name,
				status, req->actual, req->length);
	case -EREMOTEIO:		/* short read */
		break;
	}

	status = usb_ep_queue(ep, req, GFP_ATOMIC);
	if (status) {
		ERROR(cdev, "kill %s:  resubmit %d bytes --> %d\n",
			ep->name, req->length, status);
		free_ep_req(ep, req);
		usb_ep_set_halt(ep);
	}
}

static void treuzell_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct usb_composite_dev	*cdev;
	struct f_treuzell		*tz = ep->driver_data;
	int				status = req->status;

	/* driver_data will be null if ep has been disabled */
	if (!tz)
		return;

	cdev = tz->function.config->cdev;

	switch (status) {

	case 0:				/* normal completion? */
		/* we'll always clean the request */
		break;

	/* this endpoint is normally active while we're configured */
	case -ECONNABORTED:		/* hardware forced ep reset */
	case -ECONNRESET:		/* request dequeued */
	case -ESHUTDOWN:		/* disconnect from host */
		DBG(cdev, "%s gone (%d), %d/%d\n", ep->name, status,
				req->actual, req->length);
		usb_ep_free_request(ep, req);
		return;

	case -EOVERFLOW:		/* buffer overrun on read means that
					 * we didn't provide a big enough
					 * buffer.
					 */
	default:
		DBG(cdev, "%s complete --> %d, %d/%d\n", ep->name,
				status, req->actual, req->length);
	case -EREMOTEIO:		/* short read */
		break;
	}

	/* free the req and give back the buffer */
	usb_ep_free_request(ep, req);
}

static void treuzell_final(struct usb_ep *ep, struct usb_request *req)
{
	struct f_treuzell		*tz = ep->driver_data;

	treuzell_complete(ep, req);
	psee_video_dma_buffer_done(tz->video_ifc);
}

static void data_in_cb(struct scatterlist *sg, unsigned int num_sgs,
			struct f_treuzell *tz, struct usb_ep *ep)
{
	struct usb_composite_dev *cdev = tz->function.config->cdev;
	struct usb_request *req;
	int status;

	req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (!req) {
		ERROR(cdev, "usb_req alloc failed for %s\n", ep->name);
		return;
	}

	req->buf = NULL;
	/* This should be req->num_sgs = num_sgs;
	 * but only the first item of the list is sent.
	 * this is a work around for a bug to be fixed
	 */
	sg_dma_len(sg) *= num_sgs;
	req->num_sgs = 1;
	req->sg = sg;
	req->complete = treuzell_final;
	status = usb_ep_queue(ep, req, GFP_ATOMIC);
	if (status) {
		ERROR(cdev, "queue in %s: %d\n", ep->name, status);
		usb_ep_free_request(ep, req);
		return;
	}
}

static void bulk_cb(struct scatterlist *sg, unsigned int num_sgs, void *pdata)
{
	struct f_treuzell *tz = pdata;

	data_in_cb(sg, num_sgs, tz, tz->in_ep);
}

static void disable_treuzell(struct f_treuzell *tz)
{
	struct usb_composite_dev	*cdev;

	cdev = tz->function.config->cdev;
	/* stop generating data */
	psee_video_disable_dma(tz->video_ifc);
	tz->stream_state = STREAMOFF;
	disable_ep(cdev, tz->in_ep);
	/* stop accepting new requests */
	disable_ep(cdev, tz->ctrl_out_ep);
	/* flush pending requests */
	flush_workqueue(tz->workqueue);
	/* drop pending answers */
	disable_ep(cdev, tz->ctrl_in_ep);
	DBG(cdev, "%s disabled\n", tz->function.name);
}

static int
enable_treuzell(struct usb_composite_dev *cdev, struct f_treuzell *tz,
		int alt)
{
	int					result = 0;
	struct usb_ep				*ep;
	struct usb_request			*req;

	/* one bulk endpoint writes (sources) control res IN (to the host) */
	ep = tz->ctrl_in_ep;
	result = config_ep_by_speed(cdev->gadget, &(tz->function), ep);
	if (result)
		return result;
	result = usb_ep_enable(ep);
	if (result < 0)
		return result;
	ep->driver_data = tz;


	/* one bulk endpoint reads (sinks) control cmd OUT (from the host) */
	ep = tz->ctrl_out_ep;
	result = config_ep_by_speed(cdev->gadget, &(tz->function), ep);
	if (result)
		goto fail;
	result = usb_ep_enable(ep);
	if (result < 0)
		goto fail;
	ep->driver_data = tz;

	/* Allocate a first request to initialize the system */
	req = alloc_ep_req(ep, TREUZELL_CTRL_BUFLEN);
	if (!req) {
		result = -ENOMEM;
		goto fail2;
	}
	req->complete = treuzell_complete_ctrl;

	result = usb_ep_queue(ep, req, GFP_ATOMIC);
	if (result < 0) {
		free_ep_req(ep, req);
		goto fail2;
	}

	if (alt == 0) {
		/* one bulk endpoint writes (sources) events IN (to the host) */
		ep = tz->in_ep;
		result = config_ep_by_speed(cdev->gadget, &(tz->function), ep);
		if (result)
			goto fail2;
		result = usb_ep_enable(ep);
		if (result < 0)
			goto fail2;
		ep->driver_data = tz;

		result = psee_video_enable_dma(tz->video_ifc, bulk_cb, tz);
		if (result < 0) {
			ep = tz->in_ep;
			usb_ep_disable(ep);
			goto fail2;
		}
		tz->stream_state = STREAMAUTO;
	} else {
		/* No alternative setting for now */
		goto fail2;
	}
	tz->cur_alt = alt;

	DBG(cdev, "%s enabled, alt intf %d\n", tz->function.name, alt);
	return result;
fail2:
	usb_ep_disable(tz->ctrl_out_ep);
fail:
	usb_ep_disable(tz->ctrl_in_ep);
	return result;
}

static int treuzell_set_alt(struct usb_function *f,
		unsigned int intf, unsigned int alt)
{
	struct f_treuzell		*tz = func_to_tz(f);
	struct usb_composite_dev	*cdev = f->config->cdev;

	disable_treuzell(tz);
	INFO(cdev, "%s setting alt cfg %d\n", tz->function.name, alt);
	return enable_treuzell(cdev, tz, alt);
}

static int treuzell_get_alt(struct usb_function *f, unsigned int intf)
{
	struct f_treuzell		*tz = func_to_tz(f);

	return tz->cur_alt;
}

static void treuzell_disable(struct usb_function *f)
{
	struct f_treuzell	*tz = func_to_tz(f);

	disable_treuzell(tz);
}

/*-------------------------------------------------------------------------*/

static int treuzell_setup(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct usb_configuration	*c = f->config;
	struct usb_request		*req = c->cdev->req;
	int				value = -EOPNOTSUPP;
	u16				w_index = le16_to_cpu(ctrl->wIndex);
	u16				w_value = le16_to_cpu(ctrl->wValue);
	u16				w_length = le16_to_cpu(ctrl->wLength);

	req->length = USB_COMP_EP0_BUFSIZ;

	/* composite driver infrastructure handles most of the requests
	 * We have no VENDOR request yet
	 */
	switch (ctrl->bRequest) {
	default:
		DBG(c->cdev,
			"unknown control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		VDBG(c->cdev, "Treuzell req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = 0;
		req->length = value;
		value = usb_ep_queue(c->cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(c->cdev, "Treuzell response, err %d\n",
					value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

static struct usb_function *treuzell_alloc_func(
		struct usb_function_instance *fi)
{
	struct f_treuzell     *tz;
	struct f_tz_opts	*tz_opts;

	tz = kzalloc(sizeof(*tz), GFP_KERNEL);
	if (!tz)
		return NULL;

	tz_opts =  container_of(fi, struct f_tz_opts, func_inst);

	mutex_lock(&tz_opts->lock);
	tz_opts->refcnt++;
	mutex_unlock(&tz_opts->lock);

	tz->board_serial = tz_opts->board_serial;
	tz->version = tz_opts->version;
	tz->build_date = tz_opts->build_date;
	tz->sensor_address = tz_opts->sensor_address;
	tz->sensor_addrspace_size = tz_opts->sensor_addrspace_size;

	tz->function.name = "Treuzell";
	tz->function.bind = treuzell_bind;
	tz->function.unbind = treuzell_unbind;
	tz->function.set_alt = treuzell_set_alt;
	tz->function.get_alt = treuzell_get_alt;
	tz->function.disable = treuzell_disable;
	tz->function.setup = treuzell_setup;
	tz->function.strings = treuzell_strings;

	tz->function.free_func = treuzell_free_func;

	return &tz->function;
}

static inline struct f_tz_opts *to_f_tz_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_tz_opts,
			    func_inst.group);
}

static void tz_attr_release(struct config_item *item)
{
	struct f_tz_opts *tz_opts = to_f_tz_opts(item);

	usb_put_function_instance(&tz_opts->func_inst);
}

static struct configfs_item_operations tz_item_ops = {
	.release		= tz_attr_release,
};

static ssize_t f_tz_opts_board_serial_show(struct config_item *item,
					char *page)
{
	struct f_tz_opts *opts = to_f_tz_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%u\n", opts->board_serial);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tz_opts_board_serial_store(struct config_item *item,
				       const char *page, size_t len)
{
	struct f_tz_opts *opts = to_f_tz_opts(item);
	int ret;
	u32 num;

	mutex_lock(&opts->lock);
	if (opts->refcnt) {
		ret = -EBUSY;
		goto end;
	}

	ret = kstrtou32(page, 0, &num);
	if (ret)
		goto end;

	opts->board_serial = num;
	ret = len;
end:
	mutex_unlock(&opts->lock);
	return ret;
}

CONFIGFS_ATTR(f_tz_opts_, board_serial);

static ssize_t f_tz_opts_version_show(struct config_item *item,
					char *page)
{
	struct f_tz_opts *opts = to_f_tz_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%u\n", opts->version);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tz_opts_version_store(struct config_item *item,
					const char *page, size_t len)
{
	struct f_tz_opts *opts = to_f_tz_opts(item);
	int ret;
	u32 num;

	mutex_lock(&opts->lock);
	if (opts->refcnt) {
		ret = -EBUSY;
		goto end;
	}

	ret = kstrtou32(page, 0, &num);
	if (ret)
		goto end;

	opts->version = num;
	ret = len;
end:
	mutex_unlock(&opts->lock);
	return ret;
}

CONFIGFS_ATTR(f_tz_opts_, version);

static ssize_t f_tz_opts_build_date_show(struct config_item *item,
					char *page)
{
	struct f_tz_opts *opts = to_f_tz_opts(item);
	int64_t result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%lld\n", opts->build_date);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tz_opts_build_date_store(struct config_item *item,
					const char *page, size_t len)
{
	struct f_tz_opts *opts = to_f_tz_opts(item);
	int ret;
	s64 num;

	mutex_lock(&opts->lock);
	if (opts->refcnt) {
		ret = -EBUSY;
		goto end;
	}

	ret = kstrtos64(page, 0, &num);
	if (ret)
		goto end;

	opts->build_date = num;
	ret = len;
end:
	mutex_unlock(&opts->lock);
	return ret;
}

CONFIGFS_ATTR(f_tz_opts_, build_date);

static ssize_t f_tz_opts_sensor_address_show(struct config_item *item,
					char *page)
{
	struct f_tz_opts *opts = to_f_tz_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%u\n", opts->sensor_address);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tz_opts_sensor_address_store(struct config_item *item,
				       const char *page, size_t len)
{
	struct f_tz_opts *opts = to_f_tz_opts(item);
	int ret;
	u32 num;

	mutex_lock(&opts->lock);
	if (opts->refcnt) {
		ret = -EBUSY;
		goto end;
	}

	ret = kstrtou32(page, 0, &num);
	if (ret)
		goto end;

	opts->sensor_address = num;
	ret = len;
end:
	mutex_unlock(&opts->lock);
	return ret;
}

CONFIGFS_ATTR(f_tz_opts_, sensor_address);

static ssize_t f_tz_opts_sensor_addrspace_size_show(struct config_item *item,
					char *page)
{
	struct f_tz_opts *opts = to_f_tz_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%u\n", opts->sensor_addrspace_size);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tz_opts_sensor_addrspace_size_store(struct config_item *item,
				       const char *page, size_t len)
{
	struct f_tz_opts *opts = to_f_tz_opts(item);
	int ret;
	u32 num;

	mutex_lock(&opts->lock);
	if (opts->refcnt) {
		ret = -EBUSY;
		goto end;
	}

	ret = kstrtou32(page, 0, &num);
	if (ret)
		goto end;

	opts->sensor_addrspace_size = num;
	ret = len;
end:
	mutex_unlock(&opts->lock);
	return ret;
}

CONFIGFS_ATTR(f_tz_opts_, sensor_addrspace_size);

static struct configfs_attribute *tz_attrs[] = {
	&f_tz_opts_attr_board_serial,
	&f_tz_opts_attr_version,
	&f_tz_opts_attr_build_date,
	&f_tz_opts_attr_sensor_address,
	&f_tz_opts_attr_sensor_addrspace_size,
	NULL,
};

static struct config_item_type tz_func_type = {
	.ct_item_ops    = &tz_item_ops,
	.ct_attrs	= tz_attrs,
	.ct_owner       = THIS_MODULE,
};

static void treuzell_free_instance(struct usb_function_instance *fi)
{
	struct f_tz_opts *tz_opts;

	tz_opts = container_of(fi, struct f_tz_opts, func_inst);
	kfree(tz_opts);
}

static struct usb_function_instance *treuzell_alloc_inst(void)
{
	struct f_tz_opts *tz_opts;

	tz_opts = kzalloc(sizeof(*tz_opts), GFP_KERNEL);
	if (!tz_opts)
		return ERR_PTR(-ENOMEM);
	mutex_init(&tz_opts->lock);
	tz_opts->func_inst.free_func_inst = treuzell_free_instance;

	config_group_init_type_name(&tz_opts->func_inst.group, "",
				    &tz_func_type);

	return &tz_opts->func_inst;
}
DECLARE_USB_FUNCTION(treuzell, treuzell_alloc_inst,
		treuzell_alloc_func);

int treuzell_function_register(void)
{
	int ret;

	ret = usb_function_register(&treuzellusb_func);
	return ret;
}
void treuzell_function_unregister(void)
{
	usb_function_unregister(&treuzellusb_func);
}
