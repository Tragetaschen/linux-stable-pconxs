/******************************************************************************
 *
 *   Copyright (C) 2017  Target Systemelektronik GmbH & Co. KG.
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

#include "fpga.h"
#include <linux/delay.h>

#define FPGA_SYSTEM_BASE	0x000
#define FPGA_ADC_BASE		0x100
#define FPGA_FLASH_BASE		0x200
#define FPGA_MUX_BASE		0x400
#define FPGA_STREAM_BASE	0x500
#define FPGA_AFE_BASE		0x600

#define FPGA_EXT_FREQ		(FPGA_SYSTEM_BASE + 0x00)
#define FPGA_PLL_MULT		(FPGA_SYSTEM_BASE + 0x08)
#define FPGA_ACTIVE_CLOCK	(FPGA_SYSTEM_BASE + 0x10)
#define FPGA_BUILD_TIME		(FPGA_SYSTEM_BASE + 0x14)
#define FPGA_BUILD_NUMBER	(FPGA_SYSTEM_BASE + 0x18)
#define FPGA_VERSION		(FPGA_SYSTEM_BASE + 0x1c)

#define FPGA_ADC_CONFIG		(FPGA_ADC_BASE + 0x00)

#define FPGA_FLASH_SECTOR_SIZE	(FPGA_FLASH_BASE + 0x04)
#define FPGA_FLASH_COMMAND	(FPGA_FLASH_BASE + 0x08)
#define FPGA_FLASH_STATUS	(FPGA_FLASH_BASE + 0x08)
#define FPGA_FLASH_DATA		(FPGA_FLASH_BASE + 0x0c)

#define FPGA_MUX		(FPGA_MUX_BASE + 0x00)

#define FPGA_TRIGGER		(FPGA_STREAM_BASE + 0x00)
#define FPGA_SAMPLES		(FPGA_STREAM_BASE + 0x04)
#define FPGA_ACQ		(FPGA_STREAM_BASE + 0x08)
#define FPGA_SYNC		(FPGA_STREAM_BASE + 0x0c)
#define FPGA_PAUSE		(FPGA_STREAM_BASE + 0x10)
#define FPGA_RESOLUTION		(FPGA_STREAM_BASE + 0x14)
#define FPGA_PREPAUSE		(FPGA_STREAM_BASE + 0x1c)
#define FPGA_TRIGGER_PAUSE	(FPGA_STREAM_BASE + 0x2c)

#define FPGA_AFE_COMMAND	(FPGA_AFE_BASE + 0x00)
#define FPGA_AFE_ON		(FPGA_AFE_BASE + 0x04)
#define FPGA_AFE_HV_ON		(FPGA_AFE_BASE + 0x08)
#define FPGA_AFE_HV		(FPGA_AFE_BASE + 0x0c)
#define FPGA_AFE_DAC		(FPGA_AFE_BASE + 0x10)
//... FPGA_AFE_DAC7		(FPGA_AFE_BASE + 0x28)

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

static ssize_t afe_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 value;

	bar_write(dev, 0, FPGA_AFE_COMMAND);
	bar_write(dev, 0, FPGA_AFE_ON);
	value = bar_read(dev, FPGA_AFE_ON);
	return scnprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t afe_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 value;
	if (kstrtou32(buf, 0, &value))
		return -EINVAL;

	bar_write(dev, 1, FPGA_AFE_COMMAND);
	bar_write(dev, !!value, FPGA_AFE_ON);
	return count;
}

static ssize_t hv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 value;

	bar_write(dev, 0, FPGA_AFE_COMMAND);
	bar_write(dev, 0, FPGA_AFE_HV_ON);
	value = bar_read(dev, FPGA_AFE_HV_ON);
	if (value != 0) // There is an actual HV set
	{
		bar_write(dev, 0, FPGA_AFE_HV);
		value = bar_read(dev, FPGA_AFE_HV);
	}

	return scnprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t hv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 value;

	if (kstrtou32(buf, 10, &value))
		return -EINVAL;

	bar_write(dev, 1, FPGA_AFE_COMMAND);
	bar_write(dev, !!value, FPGA_AFE_HV_ON);
	bar_write(dev, value, FPGA_AFE_HV);

	return count;
}

static ssize_t dac_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 value;
	int i = (int)attr->attr.name[3] - '0';

	if (!(1 <= i && i <= 7))
		return -EINVAL;

	bar_write(dev, 0, FPGA_AFE_COMMAND);
	bar_write(dev, 0, FPGA_AFE_DAC + 4 * (i - 1));
	value = bar_read(dev, FPGA_AFE_DAC + 4 * (i - 1));
	return scnprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t dac_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 value;
	int i = (int)attr->attr.name[3] - '0';

	if (!(1 <= i && i <= 7))
		return -EINVAL;

	if (kstrtou32(buf, 0, &value))
		return -EINVAL;
	bar_write(dev, 1, FPGA_AFE_COMMAND);
	bar_write(dev, value & 0x0000ffff, FPGA_AFE_DAC + 4 * (i - 1));
	return count;
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

#define BIT_MIRROR(x) ((x&0x80)>>7 | (x&0x40)>>5 | (x&0x20)>>3 | (x&0x10)>>1 | \
		       (x&0x08)<<1 | (x&0x04)<<3 | (x&0x02)<<5 | (x&0x01)<<7)
#define WAIT_STATUS while (bar_read(dev, FPGA_FLASH_STATUS) != 2)

static ssize_t firmware_store(struct file *filep, struct kobject *kobj, struct bin_attribute *bin_attr, char *buffer, loff_t offset, size_t count)
{
	u32 sector_mask;
	int pos;
	int flash_offset = (int)offset;
	int flash_word;
	struct device *dev = container_of(kobj, struct device, kobj);

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

static ssize_t interrupt_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fpga_dev *fpga_dev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n",
		fpga_dev->number_of_interrupts,
		fpga_dev->number_of_lengths,
		fpga_dev->number_of_lengths - fpga_dev->number_of_interrupts);
}

VALUE64_RO(ext_freq, FPGA_EXT_FREQ);
VALUE_RO(pll_mult, FPGA_PLL_MULT, "%d");
VALUE_RW(active_clock, FPGA_ACTIVE_CLOCK, "%d");
VALUE_RO(build_time, FPGA_BUILD_TIME, "%d");
VALUE_RO(build_number, FPGA_BUILD_NUMBER, "%d");
DEVICE_ATTR_RO(version);
VALUE_RW(adc, FPGA_ADC_CONFIG, "0x%x");
BIN_ATTR(firmware, S_IWUSR, NULL, firmware_store, 0);
VALUE_RW(mux, FPGA_MUX, "0x%x");
VALUE_RW(trigger, FPGA_TRIGGER, "%d");
VALUE_RW(samples, FPGA_SAMPLES, "%d");
VALUE_RW(acq, FPGA_ACQ, "%d");
VALUE_WO(sync, FPGA_SYNC);
VALUE_RW(pause, FPGA_PAUSE, "%d");
VALUE64_RO(resolution, FPGA_RESOLUTION);
VALUE_RW(prepause, FPGA_PREPAUSE, "%d");
VALUE_RW(trigger_pause, FPGA_TRIGGER_PAUSE, "%d");
DEVICE_ATTR_RW(afe_on);
DEVICE_ATTR_RW(hv);
DEVICE_ATTR(dac1, S_IWUSR | S_IRUGO, dac_show, dac_store);
DEVICE_ATTR(dac2, S_IWUSR | S_IRUGO, dac_show, dac_store);
DEVICE_ATTR(dac3, S_IWUSR | S_IRUGO, dac_show, dac_store);
DEVICE_ATTR(dac4, S_IWUSR | S_IRUGO, dac_show, dac_store);
DEVICE_ATTR(dac5, S_IWUSR | S_IRUGO, dac_show, dac_store);
DEVICE_ATTR(dac6, S_IWUSR | S_IRUGO, dac_show, dac_store);
DEVICE_ATTR(dac7, S_IWUSR | S_IRUGO, dac_show, dac_store);

DEVICE_ATTR_RO(interrupt_info);

static struct attribute *fpga_attrs[] = {
	&dev_attr_ext_freq.attr,
	&dev_attr_pll_mult.attr,
	&dev_attr_active_clock.attr,
	&dev_attr_build_time.attr,
	&dev_attr_build_number.attr,
	&dev_attr_version.attr,
	&dev_attr_adc.attr,
	&dev_attr_mux.attr,
	&dev_attr_trigger.attr,
	&dev_attr_samples.attr,
	&dev_attr_acq.attr,
	&dev_attr_sync.attr,
	&dev_attr_pause.attr,
	&dev_attr_resolution.attr,
	&dev_attr_prepause.attr,
	&dev_attr_trigger_pause.attr,
	&dev_attr_afe_on.attr,
	&dev_attr_hv.attr,
	&dev_attr_dac1.attr,
	&dev_attr_dac2.attr,
	&dev_attr_dac3.attr,
	&dev_attr_dac4.attr,
	&dev_attr_dac5.attr,
	&dev_attr_dac6.attr,
	&dev_attr_dac7.attr,

	&dev_attr_interrupt_info.attr,
	NULL,
};

static struct bin_attribute *fpga_bin_attrs[] = {
	&bin_attr_firmware,
	NULL,
};

static const struct attribute_group fpga_group = {
	.attrs = fpga_attrs,
	.bin_attrs = fpga_bin_attrs,
};


const struct attribute_group *fpga_attribute_groups[] = {
	&fpga_group,
	NULL,
};

