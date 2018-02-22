#ifndef _FPGA_H
#define _FPGA_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/cdev.h>

struct fpga_ringbuffer {
	int size;
	void *start;
	dma_addr_t dma_handle;
};

struct fpga_dev {
	struct pci_dev *pci_dev;

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
};

void bar_write(struct device *dev, u32 value, int offset);
u32 bar_read(struct device *dev, int offset);
extern const struct attribute_group *fpga_attribute_groups[];

#endif
