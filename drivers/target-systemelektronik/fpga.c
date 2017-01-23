/******************************************************************************
 *
 *   Copyright (C) 2014  Target Systemelektronik GmbH & Co. KG.
 *   All rights reserved.
 *
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; version 2 of the License.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *****************************************************************************/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/pci-aspm.h>
#include <linux/aer.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>

#include "pcie_registers.h"

#define TARGET_FPGA_DRIVER_NAME "target-fpga"
#define PCI_VENDOR_ID_TARGET 0x1172
#define PCI_DEVICE_ID_TARGET_FPGA 0x0004
#define TARGET_FPGA_DATA_SIZE 16384 * PAGE_SIZE

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
	int number_of_interrupts;
	int number_of_lengths;
};

static struct class *device_class;
static dev_t fpga_devt;

/* Parameters for the waiting for iATU enabled routine */
#define LINK_WAIT_MAX_IATU_RETRIES	5
#define LINK_WAIT_IATU_MIN		9000
#define LINK_WAIT_IATU_MAX		10000

static void _dw_pcie_prog_viewport_inbound(
	struct pci_dev *dev, u32 viewport,
	u64 fpga_base, u64 ram_base, u64 size)
{
	u32 retries, val;

	pci_write_config_dword(dev, PCIE_ATU_VIEWPORT,
			       PCIE_ATU_REGION_INBOUND | viewport);
	pci_write_config_dword(dev, PCIE_ATU_LOWER_BASE,
			       fpga_base);
	pci_write_config_dword(dev, PCIE_ATU_UPPER_BASE,
			       fpga_base >> 32);
	pci_write_config_dword(dev, PCIE_ATU_LIMIT,
			       fpga_base + size - 1);
	pci_write_config_dword(dev, PCIE_ATU_LOWER_TARGET,
			       ram_base);
	pci_write_config_dword(dev, PCIE_ATU_UPPER_TARGET,
			       ram_base + size - 1);
	pci_write_config_dword(dev, PCIE_ATU_CR1,
			       PCIE_ATU_TYPE_MEM);
	pci_write_config_dword(dev, PCIE_ATU_CR2,
			       PCIE_ATU_ENABLE);

	dev_info(&dev->dev,
		"Viewpoint:\t0x%04X\n"
		"Size:\t\t0x%016llX\n"
		"FPGA-Start:\t0x%016llX\n"
		"FPGA-End:\t0x%016llX\n"
		"Ram-Start:\t0x%016llX\n"
		"Ram-End:\t0x%016llX\n",
		viewport,
		size,
		fpga_base,
		fpga_base + size - 1,
		ram_base,
		ram_base + size - 1
	);

	for (retries = 0; retries < LINK_WAIT_MAX_IATU_RETRIES; retries++) {
			pci_read_config_dword(dev, PCIE_ATU_CR2,&val);

		if (val == PCIE_ATU_ENABLE)
		{
			dev_info(&dev->dev, "iATU too %d retries", retries);
			return;
		}

		usleep_range(LINK_WAIT_IATU_MIN, LINK_WAIT_IATU_MAX);
	}
	dev_err(&dev->dev, "iATU is not being enabled\n");
}

static void dw_pcie_prog_viewports_inbound(struct fpga_dev *dev)
{
	struct pci_dev *root_complex = dev->pci_dev;
const int offset = 0x40000000;
	while (!pci_is_root_bus(root_complex->bus))
		root_complex = root_complex->bus->self;
	_dw_pcie_prog_viewport_inbound(
		root_complex,
		PCIE_ATU_REGION_INDEX0,
		offset + 0,
		(u64)dev->data.dma_handle,
		dev->data.size
	);
	_dw_pcie_prog_viewport_inbound(
		root_complex,
		PCIE_ATU_REGION_INDEX1,
		offset + dev->data.size,
		(u64)dev->counts.dma_handle,
		dev->counts.size
	);
}

static bool _fpga_allocate_buffer(struct pci_dev *dev,
				    struct fpga_ringbuffer *pws)
{
	pws->start = dmam_alloc_coherent(&dev->dev, pws->size, &pws->dma_handle, GFP_KERNEL | __GFP_ZERO);
	if (pws->start == NULL)
	{
		dev_err(&dev->dev, "Could not alloc %d bytes", pws->size);
		return false;
	}
	return true;
}

static bool fpga_allocate_buffers(struct fpga_dev *dev)
{
	if(!_fpga_allocate_buffer(dev->pci_dev, &dev->data))
		return false;
	return _fpga_allocate_buffer(dev->pci_dev, &dev->counts);
}

static void _fpga_free_buffer(struct pci_dev *dev,
				    struct fpga_ringbuffer *pws)
{
	dmam_free_coherent(&dev->dev, pws->size, pws->start, pws->dma_handle);
}

static void fpga_free_buffers(struct fpga_dev *dev)
{
	_fpga_free_buffer(dev->pci_dev, &dev->data);
	_fpga_free_buffer(dev->pci_dev, &dev->counts);
}

static irqreturn_t handle_data_msi(int irq, void *data)
{
	struct fpga_dev *fpga_dev = data;
	int to_add = 0;
	int add_result;
	int *counts_buffer = (int*)fpga_dev->counts.start;
	u32 position = fpga_dev->counts_position;

	fpga_dev->number_of_interrupts++;

	while ((add_result = counts_buffer[position]) >= 0) {
		counts_buffer[position] = -1;
		to_add += add_result;
		position = (position + 1) & 16383;
		fpga_dev->number_of_lengths++;
	}
	fpga_dev->counts_position = position;

	atomic_add_return(to_add, &fpga_dev->unread_data_items);
	complete(&fpga_dev->data_has_arrived);

	return IRQ_HANDLED;
}

static int fpga_setup_irq(struct fpga_dev *fpga_dev)
{
	int irq;
	int ret;

	ret = pci_enable_msi_exact(fpga_dev->pci_dev, 1);
	if (ret < 0) {
		dev_err(&fpga_dev->pci_dev->dev, "Could not request msi\n");
		return ret;
	}
	dev_info(&fpga_dev->pci_dev->dev, "Enabled %d interrupts\n", 1);
	irq = fpga_dev->pci_dev->irq;
	ret = devm_request_irq(&fpga_dev->pci_dev->dev, irq, handle_data_msi,
			       0, "fpga-data", fpga_dev);
	if (ret) {
		dev_err(&fpga_dev->pci_dev->dev, "Failed to request irq %d\n", irq);
		return ret;
	}
	return 0;
}

static void fpga_teardown_irq(struct fpga_dev *fpga_dev)
{
	int irq = fpga_dev->pci_dev->irq;
	devm_free_irq(&fpga_dev->pci_dev->dev, irq, fpga_dev);
	pci_disable_msi(fpga_dev->pci_dev);
}

static void bar_write(struct device *dev, u32 value, int offset)
{
	void* bar_address;
	struct fpga_dev *fpga_dev = dev_get_drvdata(dev);

	bar_address = pcim_iomap_table(fpga_dev->pci_dev)[1];
	iowrite32(value, bar_address + offset);
}

static u32 bar_read(struct device *dev, int offset)
{
	void* bar_address;
	struct fpga_dev *fpga_dev = dev_get_drvdata(dev);

	bar_address = pcim_iomap_table(fpga_dev->pci_dev)[1];

	return ioread32(bar_address + offset);
}

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char* buf)
{
	const int number_of_words = 8;
	int i;
	int value;
	char c;
	int j = 0;
	char values[number_of_words * sizeof(int) + 1];

	for (i=0; i<number_of_words; ++i) {
		value = bar_read(dev, FPGA_VERSION + 4 * i);
		c = (value >> 24) & 0xff;
		if (c != 0)
			values[j++] = c;
		c = (value >> 16) & 0xff;
		if (c != 0)
			values[j++] = c;
		c = (value >> 8) & 0xff;
		if (c != 0)
			values[j++] = c;
		c = value & 0xff;
		if (c != 0)
			values[j++] = c;
	}
	values[j] = 0;
	return scnprintf(buf, PAGE_SIZE, "%s\n", values);
}

static void afe3_write(struct device *dev, u32 cmd, u32 index, u32 data)
{
	bar_write(dev, cmd << 8 | index, FPGA_AFE3_COMMAND);
	bar_write(dev, data, FPGA_AFE3_DATA);
	// There's currently no way to know when it's done
	msleep_interruptible(20);
}

static int afe3_read(struct device *dev, u32 cmd, u32 index, u32* data)
{
	u32 command, expected_value;
	int attempts = 0;
	const int max_attempts = 5;

	command = cmd << 8 | index;
	expected_value = 0x55 << 16 | cmd << 8 | index;

	bar_write(dev, command, FPGA_AFE3_COMMAND);
	bar_write(dev, 0, FPGA_AFE3_DATA);

	do
	{
		++attempts;
		msleep_interruptible(20);
		command = bar_read(dev, FPGA_AFE3_COMMAND);
	}
	while (command != expected_value && attempts < max_attempts);

	if (attempts >= max_attempts)
		return -ETIMEDOUT;

	*data = bar_read(dev, FPGA_AFE3_DATA);
	return 0;
}

#define __VALUE_RO(name, offset, format) \
	static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
	{ \
		int value; \
		value = bar_read(dev, offset); \
		return scnprintf(buf, PAGE_SIZE, format "\n", value); \
	}
#define __VALUE_WO(name, offset) \
	static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) \
	{ \
		int value; \
		if (kstrtoint(buf, 0, &value)) \
			return -EINVAL; \
		bar_write(dev, value, offset);\
		return count; \
	}
#define __VALUE64_RO(name, offset) \
	static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
	{ \
		u32 lower, upper; \
		u64 value; \
		lower = bar_read(dev, offset); \
		upper = bar_read(dev, offset + 4); \
		value = ((u64)upper) << 32 | lower; \
		return scnprintf(buf, PAGE_SIZE, "%llu\n", value); \
	}

#define VALUE_WO(name, offset) \
	__VALUE_WO(name, offset) \
DEVICE_ATTR_WO(name)

#define VALUE_RO(name, offset, format) \
	__VALUE_RO(name, offset, format) \
DEVICE_ATTR_RO(name)

#define VALUE_RW(name, offset, format) \
	__VALUE_RO(name, offset, format) \
	__VALUE_WO(name, offset) \
DEVICE_ATTR_RW(name)

#define VALUE64_RO(name, offset) \
	__VALUE64_RO(name, offset) \
DEVICE_ATTR_RO(name)

#define __AFE3_RO(name, cmd, index) \
	static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
	{ \
		int ret; \
		u32 data; \
		ret = afe3_read(dev, cmd, index, &data); \
		if (ret < 0) \
			return ret; \
		return scnprintf(buf, PAGE_SIZE, "%d\n", data); \
	}

#define __AFE3_WO(name, cmd, index) \
	static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) \
	{ \
		u32 data;\
		if (kstrtou32(buf, 0, &data)) \
			return -EINVAL; \
		afe3_write(dev, cmd, index, data); \
		return count; \
	}

#define AFE3_RO(name, cmd, index) \
	__AFE3_RO(name, cmd, index) \
DEVICE_ATTR_RO(name)
#define AFE3_WO(name, cmd, index) \
	__AFE3_WO(name, cmd, index) \
DEVICE_ATTR_WO(name)
#define AFE3_RW(name, cmd_read, cmd_write, index) \
	__AFE3_RO(name, cmd_read, index) \
	__AFE3_WO(name, cmd_write, index) \
DEVICE_ATTR_RW(name)


#define BIT_MIRROR(x) ((x&0x80)>>7 | (x&0x40)>>5 | (x&0x20)>>3 | (x&0x10)>>1 | \
		       (x&0x08)<<1 | (x&0x04)<<3 | (x&0x02)<<5 | (x&0x01)<<7)
#define WAIT_STATUS while (bar_read(dev, FPGA_FLASH_STATUS) != 2)

static ssize_t firmware_store(struct file *filep, struct kobject *kobj, struct bin_attribute *bin_attr, char *buffer, loff_t offset, size_t count)
{
	u32 sector_mask;
	int pos;
	int flash_offset = (int)offset;
	int flash_word;
	struct device *dev = kobj_to_dev(kobj);

	if ((count & 0x3) != 0) // Unaligned
		return -EINVAL;

	sector_mask = bar_read(dev, FPGA_FLASH_SECTOR_SIZE) - 1;

	for (pos = 0; pos < count; pos+=4, flash_offset+=4) {
		if ((flash_offset & sector_mask) == 0) {
			dev_err(dev, "Flashing sector 0x%x\n", flash_offset);
			bar_write(dev, 0x06000000 | flash_offset, FPGA_FLASH_COMMAND);
			WAIT_STATUS;
		}
		flash_word  = BIT_MIRROR(buffer[pos + 3]) << 0;
		flash_word |= BIT_MIRROR(buffer[pos + 2]) << 8;
		flash_word |= BIT_MIRROR(buffer[pos + 1]) << 16;
		flash_word |= BIT_MIRROR(buffer[pos + 0]) << 24;
		bar_write(dev, flash_word, FPGA_FLASH_DATA);
		WAIT_STATUS;
		bar_write(dev, 0x09000000 | flash_offset, FPGA_FLASH_COMMAND);
		WAIT_STATUS;
	}

	return count;
}

static ssize_t config_write(struct file *filep, struct kobject *kobj, struct bin_attribute *bin_attr, char *buffer, loff_t offset, size_t count)
{
	u32 *data, number_of_words, index;
	struct device *dev = kobj_to_dev(kobj);

	if (offset != 0)
		return -EINVAL;

	if ((count & 0x3) != 0) // Unaligned
		return -EINVAL;

	afe3_write(dev, 4, 0, count);

	data = (void*)buffer;
	number_of_words = count >> 2;

	for (index = 0; index < number_of_words; ++index) {
		afe3_write(dev, 4, index + 1, data[index]);
	}

	return count;
}

static ssize_t config_read(struct file *filep, struct kobject *kobj, struct bin_attribute *bin_attr, char* buffer, loff_t offset, size_t count)
{
	u32 *data, number_of_words, index, stored_count;
	int ret;
	struct device *dev = kobj_to_dev(kobj);

	if (offset > 0)
		return 0;

	ret = afe3_read(dev, 5, 0, &stored_count);
	if (ret < 0)
		return ret;

	if (count < stored_count)
		return -EFAULT;

	number_of_words = stored_count >> 2;

	data = (void*)buffer;

	for (index = 0; index < number_of_words; ++index) {
		ret = afe3_read(dev, 5, index + 1, data + index);
		if (ret < 0)
			return ret;
	}
	return stored_count;
}

static ssize_t interrupt_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fpga_dev *fpga_dev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n",
		fpga_dev->number_of_interrupts,
		fpga_dev->number_of_lengths,
		fpga_dev->number_of_lengths - fpga_dev->number_of_interrupts);
}

AFE3_RW(hv, 3, 2, 0);
AFE3_RW(baseline, 8, 7, 0);
BIN_ATTR_RW(config, 0);
AFE3_RW(config0, 5, 4, 0);
AFE3_RW(config1, 5, 4, 1);
AFE3_RW(config2, 5, 4, 2);
AFE3_RW(config3, 5, 4, 3);
AFE3_RO(diagnostic0, 6, 0);
AFE3_RO(diagnostic1, 6, 1);
AFE3_RO(diagnostic2, 6, 2);
AFE3_RO(diagnostic3, 6, 3);
AFE3_RO(diagnostic4, 6, 4);
AFE3_RO(diagnostic5, 6, 5);
AFE3_RO(diagnostic6, 6, 6);
AFE3_RO(diagnostic7, 6, 7);
AFE3_RO(diagnostic10, 6, 10);
AFE3_RO(diagnostic20, 6, 20);
AFE3_RO(diagnostic21, 6, 21);
AFE3_RO(diagnostic22, 6, 22);
AFE3_RO(diagnostic23, 6, 23);
AFE3_RO(diagnostic24, 6, 24);
AFE3_RO(diagnostic25, 6, 25);

VALUE64_RO(ext_freq, FPGA_EXT_FREQ);
VALUE_RO(pll_mult, FPGA_PLL_MULT, "%d");
VALUE_RO(active_clock, FPGA_ACTIVE_CLOCK, "%d");
VALUE_RO(build_time, FPGA_BUILD_TIME, "%d");
VALUE_RO(build_number, FPGA_BUILD_NUMBER, "%d");
DEVICE_ATTR_RO(version);
VALUE_RW(adc, FPGA_ADC_CONFIG, "0x%x");
BIN_ATTR(firmware, S_IWUSR, NULL, firmware_store, 0);
VALUE_RW(fpga_dac1, FPGA_DAC1, "%d");
VALUE_RW(fpga_dac2, FPGA_DAC2, "%d");
VALUE_RW(mux, FPGA_MUX, "0x%x");
VALUE_RW(trigger, FPGA_TRIGGER, "%d");
VALUE_RW(samples, FPGA_SAMPLES, "%d");
VALUE_RW(acq, FPGA_ACQ, "%d");
VALUE_WO(sync, FPGA_SYNC);
VALUE_RW(pause, FPGA_PAUSE, "%d");
VALUE64_RO(resolution, FPGA_RESOLUTION);
VALUE_RW(prepause, FPGA_PREPAUSE, "%d");

DEVICE_ATTR_RO(interrupt_info);

static struct attribute *fpga_attrs[] = {
	&dev_attr_hv.attr,
	&dev_attr_baseline.attr,
	&dev_attr_config0.attr,
	&dev_attr_config1.attr,
	&dev_attr_config2.attr,
	&dev_attr_config3.attr,
	&dev_attr_diagnostic0.attr,
	&dev_attr_diagnostic1.attr,
	&dev_attr_diagnostic2.attr,
	&dev_attr_diagnostic3.attr,
	&dev_attr_diagnostic4.attr,
	&dev_attr_diagnostic5.attr,
	&dev_attr_diagnostic6.attr,
	&dev_attr_diagnostic7.attr,
	&dev_attr_diagnostic10.attr,
	&dev_attr_diagnostic20.attr,
	&dev_attr_diagnostic21.attr,
	&dev_attr_diagnostic22.attr,
	&dev_attr_diagnostic23.attr,
	&dev_attr_diagnostic24.attr,
	&dev_attr_diagnostic25.attr,

	&dev_attr_ext_freq.attr,
	&dev_attr_pll_mult.attr,
	&dev_attr_active_clock.attr,
	&dev_attr_build_time.attr,
	&dev_attr_build_number.attr,
	&dev_attr_version.attr,
	&dev_attr_adc.attr,
	&dev_attr_fpga_dac1.attr,
	&dev_attr_fpga_dac2.attr,
	&dev_attr_mux.attr,
	&dev_attr_trigger.attr,
	&dev_attr_samples.attr,
	&dev_attr_acq.attr,
	&dev_attr_sync.attr,
	&dev_attr_pause.attr,
	&dev_attr_resolution.attr,
	&dev_attr_prepause.attr,

	&dev_attr_interrupt_info.attr,
	NULL,
};

static struct bin_attribute *fpga_bin_attrs[] = {
	&bin_attr_config,
	&bin_attr_firmware,
	NULL,
};

static const struct attribute_group fpga_group = {
	.attrs = fpga_attrs,
	.bin_attrs = fpga_bin_attrs,
};

static const struct attribute_group *fpga_groups[] = {
	&fpga_group,
	NULL,
};

static int fpga_cdev_open(struct inode *inode, struct file *filp)
{
	filp->private_data = container_of(inode->i_cdev, struct fpga_dev, cdev);

	return nonseekable_open(inode, filp);
}

static int fpga_cdev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int fpga_cdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct fpga_dev *fpga_dev = filp->private_data;
	return dma_mmap_coherent(&fpga_dev->pci_dev->dev,
				 vma,
				 fpga_dev->data.start,
				 fpga_dev->data.dma_handle,
				 fpga_dev->data.size);
}

static ssize_t fpga_cdev_read(struct file *filp, char __user *buf,
			      size_t size, loff_t *offset)
{
	int bytes_to_read, from_position, wait_result;
	struct fpga_dev *fpga_dev = filp->private_data;

	wait_result = wait_for_completion_killable_timeout(
		&fpga_dev->data_has_arrived, msecs_to_jiffies(250)
	);

	if (wait_result < 0)
		return wait_result;

	bytes_to_read = atomic_xchg(&fpga_dev->unread_data_items, 0);

	if (bytes_to_read == 0)
		return -ETIME;

	from_position = fpga_dev->unsent_start;
	fpga_dev->unsent_start = (from_position + bytes_to_read) & (fpga_dev->data.size - 1);

	if (copy_to_user(buf, &from_position, sizeof(int)))
		return -EINVAL;
	if (copy_to_user(buf + sizeof(int), &bytes_to_read, sizeof(int)))
		return -EINVAL;

	return sizeof(int) + sizeof(int);
}

struct coaligned_arrays {
	int number_of_items;
	int offset;
	float *energies;
	float *head_energies;
	u64 *real_times;
	u64 *live_times;
};

struct fpga_event
{
	u64 real_time;
	u64 live_time;
	int _1[2];
	float head_area;
	float full_area;
	int _2[8];
};

#define FPGA_COPY_EVENTS _IOWR(0xF0, 1, struct coaligned_arrays)

static long fpga_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct coaligned_arrays arrays;
	struct fpga_dev *fpga_dev = filp->private_data;
	struct fpga_event *ringbuffer = fpga_dev->data.start;

	if (cmd != FPGA_COPY_EVENTS)
		return -ENOTTY;

	if (copy_from_user(&arrays, (void __user *)arg, sizeof(struct coaligned_arrays)))
		return -EFAULT;
	if (!access_ok(VERIFY_WRITE, arrays.energies, arrays.number_of_items * sizeof(float)))
		return -EFAULT;
	if (!access_ok(VERIFY_WRITE, arrays.head_energies, arrays.number_of_items * sizeof(float)))
		return -EFAULT;
	if (!access_ok(VERIFY_WRITE, arrays.real_times, arrays.number_of_items * sizeof(u64)))
		return -EFAULT;
	if (!access_ok(VERIFY_WRITE, arrays.live_times, arrays.number_of_items * sizeof(u64)))
		return -EFAULT;

	ringbuffer += arrays.offset;
	while (arrays.number_of_items--)
	{
		__put_user(ringbuffer->full_area, arrays.energies++);
		__put_user(ringbuffer->head_area, arrays.head_energies++);
		__put_user(ringbuffer->real_time, arrays.real_times++);
		__put_user(ringbuffer->live_time, arrays.live_times++);
		++ringbuffer;
	}
	return 0;
}

static const struct file_operations fpga_cdev_ops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.open		= fpga_cdev_open,
	.release	= fpga_cdev_release,
	.read		= fpga_cdev_read,
	.mmap		= fpga_cdev_mmap,
	.unlocked_ioctl	= fpga_cdev_ioctl,
};

static int fpga_driver_probe(struct pci_dev *dev,
			     const struct pci_device_id *id)
{
	int ret;
	struct fpga_dev *fpga_dev;

	if (dev->vendor != PCI_VENDOR_ID_TARGET || dev->device != PCI_DEVICE_ID_TARGET_FPGA)
		return -ENODEV;

	fpga_dev = devm_kzalloc(&dev->dev, sizeof(struct fpga_dev), GFP_KERNEL);
	if (!fpga_dev) {
		ret = -ENOMEM;
		goto err_managed;
	}

	fpga_dev->pci_dev = dev;
	fpga_dev->data.size = TARGET_FPGA_DATA_SIZE;
	atomic_set(&fpga_dev->unread_data_items, 0);
	fpga_dev->unsent_start = 0;
	fpga_dev->counts_position = 0;
	fpga_dev->counts.size = 16 * PAGE_SIZE;
	init_completion(&fpga_dev->data_has_arrived);
	fpga_dev->number_of_interrupts = 0;
	fpga_dev->number_of_lengths = 0;

	pci_set_drvdata(dev, fpga_dev);

	ret = pcim_enable_device(dev);
	if (ret) {
		dev_err(&dev->dev, "pci_enable_device() failed\n");
		goto err_managed;
	}

	ret = pcim_iomap_regions(dev, 0x3, TARGET_FPGA_DRIVER_NAME);
	if (ret) {
		dev_err(&dev->dev, "pcim_iomap_regions() failed\n");
		goto err_managed;
	}

	if (!fpga_allocate_buffers(fpga_dev)) {
		ret = -ENOMEM;
		goto err_managed;
	}
	memset(fpga_dev->counts.start, -2, fpga_dev->counts.size);

	dw_pcie_prog_viewports_inbound(fpga_dev);
	ret = fpga_setup_irq(fpga_dev);
	if (ret) {
		goto err_managed;
	}

	if (pci_find_ext_capability(dev, PCI_EXT_CAP_ID_ERR))
		ret = pci_enable_pcie_error_reporting(dev);
	else
		dev_info(&dev->dev, "AER not supported\n");

	pci_set_master(dev);

	fpga_dev->dev = MKDEV(MAJOR(fpga_devt), 0);

	cdev_init(&fpga_dev->cdev, &fpga_cdev_ops);
	fpga_dev->cdev.owner = THIS_MODULE;
	ret = cdev_add(&fpga_dev->cdev, fpga_dev->dev, 1);
	if (ret)
		goto err_cdev;

	fpga_dev->device = device_create(device_class, &dev->dev, fpga_dev->dev, fpga_dev, "target-fpga");
	if (IS_ERR(fpga_dev->device)) {
		ret = PTR_ERR(fpga_dev->device);
		printk(KERN_WARNING "Error %d while trying to create target-fpga", ret);
		goto err_device;
	}

	return 0;

err_device:
	cdev_del(&fpga_dev->cdev);
err_cdev:
	pci_clear_master(dev);
	pci_disable_pcie_error_reporting(dev);
	fpga_teardown_irq(fpga_dev);
err_managed:
	return ret;
}

static void fpga_driver_remove(struct pci_dev *dev)
{
	struct fpga_dev *fpga_dev;

	fpga_dev = pci_get_drvdata(dev);
	device_destroy(device_class, fpga_dev->dev);
	cdev_del(&fpga_dev->cdev);

	pcim_iounmap_regions(fpga_dev->pci_dev, 3);
	fpga_free_buffers(fpga_dev);
	pci_clear_master(dev);
	pci_disable_pcie_error_reporting(dev);
	fpga_teardown_irq(fpga_dev);
}

#ifdef CONFIG_PM
static int fpga_driver_suspend (struct pci_dev *pdev, pm_message_t state)
{
	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));

	return 0;
}

static int fpga_driver_resume (struct pci_dev *pdev)
{
	int err;
	struct fpga_dev *fpga_dev;

	pci_set_power_state(pdev, PCI_D0);

	err = pci_enable_device(pdev);
	if (err) {
		printk(KERN_WARNING "pci_enable_device failed on resume %d", err);
		return err;
	}
	pci_restore_state(pdev);

	fpga_dev = pci_get_drvdata(pdev);

	// re-programm atu viewport
	dw_pcie_prog_viewports_inbound(fpga_dev);

	return 0;
}

#endif /* CONFIG_PM */

static const struct pci_device_id fpga_driver_tbl[] = {
	{ PCI_DEVICE(PCI_ANY_ID, PCI_ANY_ID) },
	{ 0, },
};

MODULE_DEVICE_TABLE(pci, fpga_driver_tbl);

static struct pci_driver fpga_driver = {
	.name		= TARGET_FPGA_DRIVER_NAME,
	.id_table = fpga_driver_tbl,
	.probe		= fpga_driver_probe,
	.remove		= fpga_driver_remove,
#ifdef CONFIG_PM
	.suspend	= fpga_driver_suspend,
	.resume		= fpga_driver_resume,
#endif /* CONFIG_PM */
};

static int __init fpga_driver_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&fpga_devt, 0, 1, TARGET_FPGA_DRIVER_NAME);
	if (ret)
		goto exit;

	device_class = class_create(THIS_MODULE, "target-fpga");
	if (IS_ERR(device_class)) {
		ret = PTR_ERR(device_class);
		goto err_class;
	}
	device_class->dev_groups = fpga_groups;

	ret = pci_register_driver(&fpga_driver);
	if (ret) {
		goto err_driver;
	}

	return 0;

err_driver:
	class_destroy(device_class);
err_class:
	unregister_chrdev_region(fpga_devt, 1);
exit:
	return ret;
}

static void __exit fpga_driver_exit(void)
{
	pci_unregister_driver(&fpga_driver);
	class_destroy(device_class);
	unregister_chrdev_region(fpga_devt, 1);
}

module_init(fpga_driver_init);
module_exit(fpga_driver_exit);

MODULE_AUTHOR("Target Systemelektronik");
MODULE_DESCRIPTION("PCI Express driver module for our FPGA");
MODULE_LICENSE("GPL");
