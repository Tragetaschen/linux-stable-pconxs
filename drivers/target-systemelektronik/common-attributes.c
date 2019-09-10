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

#include <linux/delay.h>
#include "fpga.h"

#define FPGA_SYSTEM_BASE	0x000
#define FPGA_ADC_BASE		0x100
#define FPGA_STREAM_BASE	0x500

#define FPGA_EXT_FREQ		(FPGA_SYSTEM_BASE + 0x00)
#define FPGA_PLL_MULT		(FPGA_SYSTEM_BASE + 0x08)
#define FPGA_BUILD_TIME		(FPGA_SYSTEM_BASE + 0x14)
#define FPGA_BUILD_NUMBER	(FPGA_SYSTEM_BASE + 0x18)
#define FPGA_VERSION		(FPGA_SYSTEM_BASE + 0x1c)

#define FPGA_ADC_CONFIG		(FPGA_ADC_BASE + 0x00)

#define FPGA_FLASH_SECTOR_SIZE	(FPGA_FLASH_BASE + 0x04)
#define FPGA_FLASH_COMMAND	(FPGA_FLASH_BASE + 0x08)
#define FPGA_FLASH_STATUS	(FPGA_FLASH_BASE + 0x08)
#define FPGA_FLASH_DATA		(FPGA_FLASH_BASE + 0x0c)

#define FPGA_TRIGGER		(FPGA_STREAM_BASE + 0x00)
#define FPGA_SAMPLES		(FPGA_STREAM_BASE + 0x04)
#define FPGA_ACQ		(FPGA_STREAM_BASE + 0x08)
#define FPGA_PAUSE		(FPGA_STREAM_BASE + 0x10)
#define FPGA_RESOLUTION		(FPGA_STREAM_BASE + 0x14)

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

static ssize_t flash_type_show(struct device *dev, struct device_attribute *attr, char* buf)
{
	struct fpga_dev *fdev = dev_get_drvdata(dev);
	const char* string = fdev->flash_type == N25Q ? "N25Q" : "EPCQ";
	u32 flash_type = bar_read(dev, FPGA_FLASH_TYPE);

	return scnprintf(buf, PAGE_SIZE, "0x%x (%s)\n", flash_type, string);
}

#define BIT_MIRROR(x) ((x&0x80)>>7 | (x&0x40)>>5 | (x&0x20)>>3 | (x&0x10)>>1 | \
		       (x&0x08)<<1 | (x&0x04)<<3 | (x&0x02)<<5 | (x&0x01)<<7)
#define WAIT_STATUS while (bar_read(dev, FPGA_FLASH_STATUS) != 2)

static ssize_t firmware_store(struct file *filep, struct kobject *kobj, struct bin_attribute *bin_attr, char *buffer, loff_t offset, size_t count)
{
	u32 sector_mask;
	int pos;
	int flash_offset = (int)offset;
	u32 flash_word;
	u32 new_flash_state;
	struct device *dev = kobj_to_dev(kobj);
	struct fpga_dev *fdev = dev_get_drvdata(dev);

	if ((count & 0x3) != 0) // Unaligned
		return -EINVAL;

	if (offset == 0) {
		if (fdev->flash_type == EPCQ)
			fdev->flash_state = 0xf0000000;
		else
			fdev->flash_state = 0;
	}

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

		if (fdev->flash_type == EPCQ) {
			new_flash_state = flash_word << 28;
			flash_word >>= 4;
			flash_word |= fdev->flash_state;
			fdev->flash_state = new_flash_state;
		}

		bar_write(dev, flash_word, FPGA_FLASH_DATA);
		WAIT_STATUS;
		bar_write(dev, 0x09000000 | flash_offset, FPGA_FLASH_COMMAND);
		WAIT_STATUS;
	}

	return count;
}

static ssize_t ram_base_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fpga_dev *fdev = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "0x%016llX\n", fdev->ram_base_data);
}

static ssize_t ram_base_counts_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fpga_dev *fdev = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "0x%016llX\n", fdev->ram_base_counts);
}

VALUE64_RO(ext_freq, FPGA_EXT_FREQ);
VALUE_RO(pll_mult, FPGA_PLL_MULT, "%d");
VALUE_RO(build_time, FPGA_BUILD_TIME, "%d");
VALUE_RO(build_number, FPGA_BUILD_NUMBER, "%d");
DEVICE_ATTR_RO(version);
DEVICE_ATTR_RO(flash_type);
VALUE_RW(adc, FPGA_ADC_CONFIG, "0x%x");
BIN_ATTR(firmware, S_IWUSR, NULL, firmware_store, 0);
VALUE_RW(trigger, FPGA_TRIGGER, "%d");
VALUE_RW(samples, FPGA_SAMPLES, "%d");
VALUE_RW(acq, FPGA_ACQ, "%d");
VALUE_RW(pause, FPGA_PAUSE, "%d");
VALUE64_RO(resolution, FPGA_RESOLUTION);

DEVICE_ATTR_RO(ram_base_data);
DEVICE_ATTR_RO(ram_base_counts);
