/* DMA Proxy
 *
 * To use this driver a node must be added into the device tree.  Add the
 * following node while adjusting the dmas property to match the name of
 * the AXI DMA node.
 *
 * psee_video {
 *    compatible ="psee,video";
 *    dmas = <&axi_dma_0 0
 *            &axi_dma_0 1>;
 *    dma-names = "input", "output";
 * }
 *
 */

#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/of_dma.h>
#include <linux/of_address.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#include "treuzell.h"

#define DEFAULT_BUFFER_SIZE (16<<20)
#define DEFAULT_CHUNK_SIZE (16<<10)

/* The following module parameter controls if the DMA memory region should be
 * set to a fixed magic value between runs (for debug purpose)
 */
static unsigned int initialize_dma_memory;
module_param(initialize_dma_memory, uint, 0444);

/* The following module parameter controls if the internal test runs when the
 * module is inserted.
 */
static unsigned int internal_test;
module_param(internal_test, int, 0444);

/* The following data structure represents a single channel of DMA, transmit or
 * receive in the case when using AXI DMA.  It contains all the data to be
 * maintained for the channel.
 */
struct psee_video_channel {
	unsigned char *buffer;
	ssize_t buffer_size;
	ssize_t chunk_size;

	struct device *dma_device_p;
	dev_t dev_node;

	struct dma_chan *channel_p;	/* dma support */
	struct completion cmp;
	dma_cookie_t cookie;
	dma_addr_t dma_handle;
	u32 direction;			/* DMA_MEM_TO_DEV or DMA_DEV_TO_MEM */

	struct task_struct *dma_thread;
	void (*dma_cb)(struct scatterlist *sg, unsigned int n_sg, void *pdata);
	void *consumer_pdata;
	struct completion free_chunks;

	/* internal test stuff */
	uint32_t synced_callback;
};

struct video_driver_data {
	struct resource *video_resource;
	void __iomem *video_regbank;
	void *dma_buffer;
	struct psee_video_channel rx_channel;
};

/* debug: allows f_treuzell to access video without looking for the device */
static struct device *latest_probed_interface;
struct device *psee_video_get_latest_ifc(void)
{
	return latest_probed_interface;
}

/* The following function is a accessor to the ATIS registers. It checks if the
 * access is in-bound, and sets val with the read values, and returns 0,
 * or a negative value in case of error
 */
int psee_axi_read_regs(struct device *dev, u32 reg, u32 *val, u8 nval)
{
	struct video_driver_data *pdata = dev->driver_data;
	u32 reg_map = (pdata->video_resource->end - pdata->video_resource->start);
	int ret = 0;
	u8 i;

	for (i = 0; i < nval; i++) {
		u32 cur_reg = reg + (i * sizeof(*val));
		void *vaddr = (char *)pdata->video_regbank + cur_reg;

		if (cur_reg > reg_map) {
			ret = -ENXIO;
			break;
		}

		if (probe_kernel_read(val + i, vaddr, sizeof(*val))) {
			ret = -EIO;
			break;
		}
	}

	return ret;
}

/* The following function is a accessor to the ATIS registers. It checks if the
 * access is in-bound, and return either 0 if the write access succeded
 * or a negative value in case of error
 */
int psee_axi_write_regs(struct device *dev, u32 reg, u32 *val, u8 nval)
{
	struct video_driver_data *pdata = dev->driver_data;
	u32 reg_map = (pdata->video_resource->end - pdata->video_resource->start);
	int ret = 0;
	u8 i;

	for (i = 0; i < nval; i++) {
		u32 cur_reg = reg + (i * sizeof(*val));
		void *vaddr = (char *)pdata->video_regbank + cur_reg;

		if (cur_reg > reg_map) {
			ret = -ENXIO;
			break;
		}

		if (probe_kernel_write(vaddr, val + i, sizeof(*val))) {
			ret = -EIO;
			break;
		}
	}

	return ret;
}

/* Handle a callback and indicate the DMA transfer is complete to another
 * thread of control
 */
static void sync_callback(void *completion)
{
	/* Indicate the DMA transaction completed to allow the other
	 * thread of control to finish processing
	 */
	complete(completion);
}

/* thread function that emulates the behaviour of a cyclic DMA
 */
static int video_dma_thread(void *data)
{
	struct video_driver_data *pdata = data;
	struct dma_async_tx_descriptor *chan_desc;
	struct dma_device *dma_device = pdata->rx_channel.channel_p->device;
	struct scatterlist *sglist;
	struct page *pg;
	int page_nb = pdata->rx_channel.buffer_size / PAGE_SIZE;
	int chunk_size = pdata->rx_channel.chunk_size;
	int chunk_nb = pdata->rx_channel.buffer_size / chunk_size;
	int page_per_chunk = chunk_size / PAGE_SIZE;
	void *buffer = pdata->rx_channel.buffer;
	dma_addr_t dma_handle = pdata->rx_channel.dma_handle;
	int cur_chunk = 0;
	int i;
	int error = 0;

	/* allocate the sglist */
	sglist = kmalloc_array(page_nb, sizeof(*sglist), GFP_KERNEL);
	if (!sglist)
		return -ENOMEM;

	/* create some sg lists to iterate on */
	sg_init_table(sglist, page_nb);
	for (i = 0; i < page_nb; i++) {
		sg_dma_address(&sglist[i]) = dma_handle + (i * PAGE_SIZE);
		sg_dma_len(&sglist[i]) = PAGE_SIZE;
		pg = vmalloc_to_page((uint8_t *)buffer + (i * PAGE_SIZE));
		if (pg)
			sg_set_page(&sglist[i], pg, PAGE_SIZE, 0);
	}

	/* reinit sync with consumer */
	reinit_completion(&pdata->rx_channel.free_chunks);
	for (i = 0; i < chunk_nb; i++) {
		complete(&pdata->rx_channel.free_chunks);
	}

	while (!kthread_should_stop()) {
		struct dma_tx_state state;
		enum dma_status status;
		long timeout;
		uint32_t data_size;

		/* Wait for buffers to be processed by consumer */
		do {
			timeout = wait_for_completion_timeout(
					&pdata->rx_channel.free_chunks,
					msecs_to_jiffies(100));
		} while ((timeout == 0) && !kthread_should_stop());
		if (kthread_should_stop())
			break;

		/* the size of the first element may have been altered in
		 * data_in_cb in f_treuzell.c to work around an issue which
		 * causes that only the first element of the sgl is sent on USB
		 */
		sg_dma_len(sglist + (cur_chunk * page_per_chunk)) = PAGE_SIZE;
		chan_desc = dmaengine_prep_slave_sg(
					pdata->rx_channel.channel_p,
					sglist + (cur_chunk * page_per_chunk),
					page_per_chunk,
					DMA_DEV_TO_MEM,
					DMA_PREP_INTERRUPT);
		/* Make sure the operation was completed successfully
		 */
		if (!chan_desc) {
			dev_err(dma_device->dev, "dmaengine_prep*() error\n");
			error = -EIO;
			break;
		}

		chan_desc->callback = sync_callback;
		chan_desc->callback_param = &pdata->rx_channel.cmp;

		/* Initialize the completion for the transfer and
		 * before using it then submit the transaction to the
		 * DMA engine so that it is queued up to be processed
		 * later and get a cookie to track its status
		 */
		init_completion(&pdata->rx_channel.cmp);

		pdata->rx_channel.cookie = dmaengine_submit(chan_desc);
		if (dma_submit_error(pdata->rx_channel.cookie)) {
			dev_err(dma_device->dev, "Submit error\n");
			dmaengine_terminate_async(
				pdata->rx_channel.channel_p);
			error = -EIO;
			break;
		}

		/* Start the DMA transaction which was previously queued
		 * up in the DMA engine
		 */
		dma_async_issue_pending(pdata->rx_channel.channel_p);

		/* Wait for the transaction to complete, or get an error
		 */
		do {
			timeout = wait_for_completion_timeout(
					&pdata->rx_channel.cmp,
					msecs_to_jiffies(100));
		} while ((timeout == 0) && !kthread_should_stop());
		if (kthread_should_stop())
			break;

		status = dmaengine_tx_status(pdata->rx_channel.channel_p,
				pdata->rx_channel.cookie, &state);

		if (status != DMA_COMPLETE) {
			dev_err(dma_device->dev,
				"DMA returned completion status of: %s\n",
				status == DMA_ERROR ? "error" : "in progress");
			dmaengine_terminate_async(pdata->rx_channel.channel_p);
			continue;
		}

		/* Get the actual size of the data in the buffer
		 */
		data_size = pdata->rx_channel.chunk_size - state.residue;

		pdata->rx_channel.dma_cb(
				sglist + (cur_chunk * page_per_chunk),
				page_per_chunk,
				pdata->rx_channel.consumer_pdata);
		cur_chunk++;
		cur_chunk %= chunk_nb;
	}

	kfree(sglist);
	return error;
}

/* Start the  DMA transfers for the channel, calling the callback for each
 * complete transaction.
 */
int psee_video_enable_dma(struct device *dev,
	void (*cb)(struct scatterlist *sg, unsigned int num_sgs, void *pdata),
	void *consumer_pdata)
{
	struct video_driver_data *pdata;

	pdata = dev->driver_data;

	if (!cb)
		return -EINVAL;

	if (pdata->rx_channel.dma_cb) {
		dev_warn(dev, "DMA already running\n");
		return -EBUSY;
	}

	/* for debug purpose, initialize memory */
	if (initialize_dma_memory) {
		uint32_t *ptr = (uint32_t *)pdata->rx_channel.buffer;
		uint32_t *end = ptr + (pdata->rx_channel.buffer_size / 4);

		while (ptr < end) {
			*ptr = initialize_dma_memory;
			ptr++;
		}
	}

	/* Cyclic dma doesn't work yet. Spawn a kthread that will keep
	 * rearming the DMA.
	 */
	pdata->rx_channel.dma_thread = kthread_create(video_dma_thread, pdata,
					"%s", dev_name(dev));
	if (IS_ERR(pdata->rx_channel.dma_thread)) {
		dev_warn(dev, "Couldn't create kthread.\n");
		return PTR_ERR(pdata->rx_channel.dma_thread);
	}

	pdata->rx_channel.dma_cb = cb;
	pdata->rx_channel.consumer_pdata = consumer_pdata;
	wake_up_process(pdata->rx_channel.dma_thread);
	return 0;
}

void psee_video_dma_buffer_done(struct device *dev)
{
	struct video_driver_data *pdata;

	pdata = dev->driver_data;
	complete(&pdata->rx_channel.free_chunks);
}

void psee_video_disable_dma(struct device *dev)
{
	struct video_driver_data *pdata;

	pdata = dev->driver_data;

	dmaengine_terminate_sync(pdata->rx_channel.channel_p);
	if (pdata->rx_channel.dma_thread)
		kthread_stop(pdata->rx_channel.dma_thread);

	pdata->rx_channel.dma_thread = NULL;
	pdata->rx_channel.dma_cb = NULL;
}

/* The following functions are designed to test the driver from within the
 * device driver without any user space.
 */
void self_test_cb(struct scatterlist *sgl, unsigned int num_sgs, void *private)
{
	struct device *dev = private;
	struct video_driver_data *pdata = dev->driver_data;

	if (!(pdata->rx_channel.synced_callback & 0x3FF)) {
		struct scatterlist *sg;
		int i;

		for_each_sg(sgl, sg, num_sgs, i) {
			void *vaddr = sg_virt(sg);
			size_t size = sg_dma_len(sg);

			if (vaddr)
				dev_info(dev, "%08x...%08x\n",
					*((uint32_t *)vaddr + 0),
					*((uint32_t *)vaddr + (size >> 2) - 1));
		}
	}

	pdata->rx_channel.synced_callback++;
	psee_video_dma_buffer_done(dev);
}

static void self_test(struct device *dev)
{
	struct video_driver_data *pdata = dev->driver_data;
	int i;

	dev_info(dev, "Starting internal test\n");

	/* Initialize the buffers for the test
	 */
	for (i = 0; i < pdata->rx_channel.buffer_size; i++)
		pdata->rx_channel.buffer[i] = 0;

	/* Receive the data that was just sent and looped back
	 */
	psee_video_enable_dma(dev, self_test_cb, dev);
	for (i = 0; i < 4; i++) {
		msleep(1000);
		dev_info(dev, "synced_callback= %d\n",
			pdata->rx_channel.synced_callback);
	}
	psee_video_disable_dma(dev);

	dev_info(dev, "Internal test complete\n");
}

/* Create a DMA channel by getting a DMA channel from the DMA Engine and then
 * setting up the channel as a character device to allow user space control.
 */
static int create_channel(struct platform_device *pdev,
	struct psee_video_channel *pchannel_p, char *name, u32 direction)
{
	/* Request the DMA channel from the DMA engine and then use the device
	 * from the channel for the proxy channel also.
	 */
	pchannel_p->channel_p = dma_request_slave_channel(&pdev->dev, name);
	if (!pchannel_p->channel_p) {
		dev_err(pchannel_p->dma_device_p, "DMA chan request error\n");
		return -ENODEV;
	}
	pchannel_p->dma_device_p = &pdev->dev;
	pchannel_p->direction = direction;

	/* Allocate DMA memory for the proxy channel interface.
	 */
	pchannel_p->buffer = dmam_alloc_coherent(
					pchannel_p->dma_device_p,
					DEFAULT_BUFFER_SIZE,
					&pchannel_p->dma_handle,
					GFP_KERNEL);

	pchannel_p->buffer_size = DEFAULT_BUFFER_SIZE;
	pchannel_p->chunk_size = DEFAULT_CHUNK_SIZE;
	dev_info(pchannel_p->dma_device_p,
		"Buffer at virtual address 0x%p, physical address 0x%llx\n",
		pchannel_p->buffer,
		pchannel_p->dma_handle);

	if (!pchannel_p->buffer) {
		dev_err(pchannel_p->dma_device_p, "DMA allocation error\n");
		return -ENOMEM;
	}

	init_completion(&pchannel_p->free_chunks);

	return 0;
}

/* Initialize the dma proxy device driver module.
 */
static int psee_video_probe(struct platform_device *pdev)
{
	int rc;
	struct video_driver_data *pdata;

	pdev->dev.driver_data = devm_kzalloc(&pdev->dev,
					sizeof(struct video_driver_data),
					GFP_KERNEL);
	pdata = pdev->dev.driver_data;
	if (!pdata) {
		dev_err(&pdev->dev, "Could not allocate private data\n");
		return -ENOMEM;
	}

	pdata->video_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pdata->video_resource) {
		dev_err(&pdev->dev, "Could not get video_resource\n");
		return -ENXIO;
	}

	pdata->video_regbank = devm_ioremap_resource(&pdev->dev,
					pdata->video_resource);
	if (!pdata->video_regbank) {
		dev_err(&pdev->dev, "Could not map video_regbank\n");
		return -ENXIO;
	}

	rc = create_channel(pdev, &pdata->rx_channel, "output",
			DMA_DEV_TO_MEM);
	if (rc)
		return rc;

	dev_info(&pdev->dev, "psee_video module initialized\n");

	if (initialize_dma_memory)
		dev_info(&pdev->dev, "DMA mem will be set at 0x%X before tx\n",
					initialize_dma_memory);

	if (internal_test)
		self_test(&pdev->dev);
	latest_probed_interface = &pdev->dev;
	return 0;
}

/* Exit the dma proxy device driver module.
 */
static int psee_video_remove(struct platform_device *pdev)
{
	struct video_driver_data *pdata = pdev->dev.driver_data;

	dev_info(&pdev->dev, "psee_video module exited\n");

	/* Take care of the DMA channels and the any buffers allocated
	 * for the DMA transfers. The DMA buffers are using managed
	 * memory such that it's automatically done.
	 */
	psee_video_disable_dma(&pdev->dev);
	if (pdata->rx_channel.channel_p) {
		dmaengine_terminate_sync(pdata->rx_channel.channel_p);
		dma_release_channel(pdata->rx_channel.channel_p);
	}
	return 0;
}

static const struct of_device_id psee_video_of_ids[] = {
	{ .compatible = "psee,video",},
	{}
};

static struct platform_driver psee_video_driver = {
	.driver = {
		.name = "psee_video_driver",
		.owner = THIS_MODULE,
		.of_match_table = psee_video_of_ids,
	},
	.probe = psee_video_probe,
	.remove = psee_video_remove,
};

static int __init treuzell_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&psee_video_driver);
	if (ret)
		return ret;

	ret = treuzell_function_register();
	if (ret) {
		platform_driver_unregister(&psee_video_driver);
		return ret;
	}
	ret = psee_i2c_register_driver();
	if (ret) {
		treuzell_function_unregister();
		platform_driver_unregister(&psee_video_driver);
		return ret;
	}
	ret = psee_spi_register_driver();
	if (ret) {
		psee_i2c_unregister_driver();
		treuzell_function_unregister();
		platform_driver_unregister(&psee_video_driver);
		return ret;
	}
	return ret;
}

static void __exit treuzell_exit(void)
{
	psee_spi_unregister_driver();
	psee_i2c_unregister_driver();
	treuzell_function_unregister();
	platform_driver_unregister(&psee_video_driver);
}

module_init(treuzell_init)
module_exit(treuzell_exit)

MODULE_AUTHOR("Prophesee");
MODULE_DESCRIPTION("Treuzell USB gadget and ATIS driver");
MODULE_LICENSE("GPL v2");
