#ifndef _FPGA_H
#define _FPGA_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/cdev.h>

#define FPGA_FLASH_BASE		0x200
#define FPGA_FLASH_TYPE		(FPGA_FLASH_BASE + 0x00)

struct fpga_ringbuffer {
	int size;
	void *start;
	dma_addr_t dma_handle;
};

enum flash_type {
	N25Q,
	EPCQ,
};

struct fpga_dev {
	struct pci_dev *pdev;
	void __iomem *bar;

	struct fpga_ringbuffer data;
	struct fpga_ringbuffer counts;

	atomic_t unread_data_items;
	u32 unsent_start;
	u32 counts_position;

	dev_t dev;
	struct cdev cdev;
	struct device *device;
	struct completion data_has_arrived;

	u64 ram_base_data;
	u64 ram_base_counts;

	u32 timestamp_reset;
	int timestamp_irq;

	enum flash_type flash_type;
	u32 flash_state;

	void* platform_device;
};

void bar_write(struct device *dev, u32 value, int offset);
u32 bar_read(struct device *dev, int offset);
extern const struct attribute_group *fpga_attribute_groups[];

int target_fpga_platform_driver_probe(struct fpga_dev *fdev, dev_t fpga_devt, struct class *device_class);
void target_fpga_platform_driver_remove(struct fpga_dev *fdev, struct class *device_class);

#endif
