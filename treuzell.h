/* This kernel module establishes a bridge (treuzell in Breton) between an
 * Event-Based Image Sensor, currently accessed though AXI interface, directly
 * on the memory bus, and a host, currently accessing the device though USB.
 */

#include <linux/device.h>
#include <linux/types.h>
#include <linux/scatterlist.h>

int treuzell_function_register(void);
void treuzell_function_unregister(void);

/* Helper to avoid having to implement a lookup function while there is only
 * one EB video interface on the system. Return the last probed one.
 * Returns NULL if no atis interface has been probed.
 */
struct device *psee_video_get_latest_ifc(void);

int psee_axi_read_regs(struct device *dev, u32 reg, u32 *val, u8 nval);
int psee_axi_write_regs(struct device *dev, u32 reg, u32 *val, u8 nval);
/* The sensor interface provides the buffers reveived though DMA. The DMA
 * buffer memory is allocated during driver probing, possibly using a reserved
 * memory region defined in the system device tree.
 * The consumer shall notify when a buffer processing is done, and buffers shall
 * be consumed and returned in order.
 * There shall be only one consumer at a time. Further enable calls to an
 * already enabled interface will fail and return a negative value.
 * When the consumer enables the DMA transfers, it shall provide a callback to
 * be notified when some data are available. The callback provides both the
 * physical and the virtual address of the buffer, the data size, and also,
 * for implementation convenience, a pointer to some consumer-owned data,
 * provided when the callback is registered. Those data are not accessed in
 * psee_video.
 */
int psee_video_enable_dma(struct device *dev,
	void (*cb)(struct scatterlist *sg, unsigned int num_sgs, void *pdata),
	void *pdata);
void psee_video_dma_buffer_done(struct device *dev);
void psee_video_disable_dma(struct device *dev);

int psee_i2c_register_driver(void);
void psee_i2c_unregister_driver(void);
struct i2c_client *psee_i2c_get_latest_sensor(void);
int psee_i2c_read_regs(struct i2c_client *sensor, u32 reg, u32 *val, u8 nval);
int psee_i2c_write_regs(struct i2c_client *sensor, u32 reg, u32 *val, u8 nval);
int psee_i2c_set_freq(struct i2c_client *sensor, u32 max_speed_hz);

int psee_spi_register_driver(void);
void psee_spi_unregister_driver(void);
struct spi_device *psee_spi_get_latest_sensor(void);
int psee_spi_read_regs(struct spi_device *sensor, u32 reg, u32 *val, u8 nval);
int psee_spi_write_regs(struct spi_device *sensor, u32 reg, u32 *val, u8 nval);
int psee_spi_set_freq(struct spi_device *sensor, u32 max_speed_hz);
