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
#define FPGA_STREAM_BASE	0x500
#define FPGA_AFE3_BASE		0x700
#define FPGA_MEASUREMENT_BASE	0xa00

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
#define FPGA_PREPAUSE		(FPGA_STREAM_BASE + 0x1c)
#define FPGA_TRIGGER_PAUSE	(FPGA_STREAM_BASE + 0x2c)

#define FPGA_AFE3_COMMAND	(FPGA_AFE3_BASE + 0x00)
#define FPGA_AFE3_DATA		(FPGA_AFE3_BASE + 0x04)

#define FPGA_VARIANCE_FIRST	(FPGA_MEASUREMENT_BASE + 0x00)
#define FPGA_VARIANCE_SECOND	(FPGA_MEASUREMENT_BASE + 0x04)
#define FPGA_RAW_SAMPLE_COUNT	(FPGA_MEASUREMENT_BASE + 0x08)
#define FPGA_SHIFTLINE1_INDEX	(FPGA_MEASUREMENT_BASE + 0x0c)
#define FPGA_SHIFTLINE2_INDEX	(FPGA_MEASUREMENT_BASE + 0x10)
#define FPGA_FIR_BANK		(FPGA_MEASUREMENT_BASE + 0x1c)
#define FPGA_ACO		(FPGA_MEASUREMENT_BASE + 0x20)
#define FPGA_DACO		(FPGA_MEASUREMENT_BASE + 0x24)

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

#define SP_ROI(name, offset) \
	VALUE_RW(name##_x0, FPGA_MEASUREMENT_BASE+offset, "%d"); \
	VALUE_RW(name##_x1, FPGA_MEASUREMENT_BASE+offset+4, "%d"); \
	VALUE_RW(name##_scale, FPGA_MEASUREMENT_BASE+offset+8, "%d"); \
	VALUE_RW(name##_bgscale, FPGA_MEASUREMENT_BASE+offset+12, "%d")

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
	u32 *data;
	struct device *dev = kobj_to_dev(kobj);

	if (count != 4)
		return -EINVAL;
	if (offset & 0x3)
		return -EINVAL;
	if (offset >= 1024)
		return -EINVAL;

	data = (u32*)(void*)buffer;
	afe3_write(dev, 4, offset >> 2, *data);

	return count;
}

static ssize_t config_read(struct file *filep, struct kobject *kobj, struct bin_attribute *bin_attr, char* buffer, loff_t offset, size_t count)
{
	u32 *data;
	int ret;
	struct device *dev = kobj_to_dev(kobj);

	if (count != 4)
		return -EINVAL;
	if (offset & 0x3)
		return -EINVAL;
	if (offset >= 1024)
		return -EINVAL;

	data = (void*)buffer;
	ret = afe3_read(dev, 5, offset >> 2, data);
	if (ret < 0)
		return ret;

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

static ssize_t ram_base_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fpga_dev *fpga_dev = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "0x%016llX\n", fpga_dev->ram_base_data);
}

static ssize_t ram_base_counts_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fpga_dev *fpga_dev = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "0x%016llX\n", fpga_dev->ram_base_counts);
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
VALUE_RO(build_time, FPGA_BUILD_TIME, "%d");
VALUE_RO(build_number, FPGA_BUILD_NUMBER, "%d");
DEVICE_ATTR_RO(version);
VALUE_RW(adc, FPGA_ADC_CONFIG, "0x%x");
BIN_ATTR(firmware, S_IWUSR, NULL, firmware_store, 0);
VALUE_RW(trigger, FPGA_TRIGGER, "%d");
VALUE_RW(samples, FPGA_SAMPLES, "%d");
VALUE_RW(acq, FPGA_ACQ, "%d");
VALUE_RW(pause, FPGA_PAUSE, "%d");
VALUE64_RO(resolution, FPGA_RESOLUTION);
VALUE_RW(prepause, FPGA_PREPAUSE, "%d");
VALUE_RW(trigger_pause, FPGA_TRIGGER_PAUSE, "%d");
VALUE_RO(baseline_variance_first, FPGA_VARIANCE_FIRST, "%d");
VALUE_RO(baseline_variance_second, FPGA_VARIANCE_SECOND, "%d");
VALUE_WO(baseline_variance_reset, FPGA_VARIANCE_FIRST);

DEVICE_ATTR_RO(interrupt_info);
DEVICE_ATTR_RO(ram_base_data);
DEVICE_ATTR_RO(ram_base_counts);

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
	&dev_attr_build_time.attr,
	&dev_attr_build_number.attr,
	&dev_attr_version.attr,
	&dev_attr_adc.attr,
	&dev_attr_trigger.attr,
	&dev_attr_samples.attr,
	&dev_attr_acq.attr,
	&dev_attr_pause.attr,
	&dev_attr_resolution.attr,
	&dev_attr_prepause.attr,
	&dev_attr_trigger_pause.attr,
	&dev_attr_baseline_variance_first.attr,
	&dev_attr_baseline_variance_second.attr,
	&dev_attr_baseline_variance_reset.attr,

	&dev_attr_interrupt_info.attr,
	&dev_attr_ram_base_data.attr,
	&dev_attr_ram_base_counts.attr,
	NULL,
};

static struct bin_attribute *fpga_bin_attrs[] = {
	&bin_attr_config,
	&bin_attr_firmware,
	NULL,
};

VALUE_RW(raw_sample_count, FPGA_RAW_SAMPLE_COUNT, "%d");
VALUE_RW(shiftline1_index, FPGA_SHIFTLINE1_INDEX, "%d");
VALUE_RW(shiftline2_index, FPGA_SHIFTLINE2_INDEX, "%d");
VALUE_RW(fir_bank, FPGA_FIR_BANK, "%d");
VALUE_RW(aco, FPGA_ACO, "%d");
VALUE_RW(daco, FPGA_DACO, "%d");
SP_ROI(pulsebaserange1, 0x28);
SP_ROI(pulsebaserange2, 0x38);
SP_ROI(pulserange1, 0x48);
SP_ROI(pulserange2, 0x58);
SP_ROI(arange, 0x68);
SP_ROI(brange, 0x78);
SP_ROI(neutronrange1, 0x88);
SP_ROI(neutronrange2, 0x98);
SP_ROI(spare0, 0xa8);
SP_ROI(spare1, 0xb8);
SP_ROI(spare2, 0xc8);
SP_ROI(spare3, 0xd8);
SP_ROI(spare4, 0xe8);
SP_ROI(spare5, 0xf8);
SP_ROI(diffbaserange, 0x108);
SP_ROI(diffrange, 0x118);
SP_ROI(ediffbaserange, 0x128);
SP_ROI(ediffrange, 0x138);


static struct attribute *signal_processing_group_attrs[] = {
	&dev_attr_raw_sample_count.attr,
	&dev_attr_shiftline1_index.attr,
	&dev_attr_shiftline2_index.attr,
	&dev_attr_fir_bank.attr,
	&dev_attr_aco.attr,
	&dev_attr_daco.attr,
	&dev_attr_pulsebaserange1_x0.attr,
	&dev_attr_pulsebaserange1_x1.attr,
	&dev_attr_pulsebaserange1_scale.attr,
	&dev_attr_pulsebaserange1_bgscale.attr,
	&dev_attr_pulsebaserange2_x0.attr,
	&dev_attr_pulsebaserange2_x1.attr,
	&dev_attr_pulsebaserange2_scale.attr,
	&dev_attr_pulsebaserange2_bgscale.attr,
	&dev_attr_pulserange1_x0.attr,
	&dev_attr_pulserange1_x1.attr,
	&dev_attr_pulserange1_scale.attr,
	&dev_attr_pulserange1_bgscale.attr,
	&dev_attr_pulserange2_x0.attr,
	&dev_attr_pulserange2_x1.attr,
	&dev_attr_pulserange2_scale.attr,
	&dev_attr_pulserange2_bgscale.attr,
	&dev_attr_arange_x0.attr,
	&dev_attr_arange_x1.attr,
	&dev_attr_arange_scale.attr,
	&dev_attr_arange_bgscale.attr,
	&dev_attr_brange_x0.attr,
	&dev_attr_brange_x1.attr,
	&dev_attr_brange_scale.attr,
	&dev_attr_brange_bgscale.attr,
	&dev_attr_neutronrange1_x0.attr,
	&dev_attr_neutronrange1_x1.attr,
	&dev_attr_neutronrange1_scale.attr,
	&dev_attr_neutronrange1_bgscale.attr,
	&dev_attr_neutronrange2_x0.attr,
	&dev_attr_neutronrange2_x1.attr,
	&dev_attr_neutronrange2_scale.attr,
	&dev_attr_neutronrange2_bgscale.attr,
	&dev_attr_spare0_x0.attr,
	&dev_attr_spare0_x1.attr,
	&dev_attr_spare0_scale.attr,
	&dev_attr_spare0_bgscale.attr,
	&dev_attr_spare1_x0.attr,
	&dev_attr_spare1_x1.attr,
	&dev_attr_spare1_scale.attr,
	&dev_attr_spare1_bgscale.attr,
	&dev_attr_spare2_x0.attr,
	&dev_attr_spare2_x1.attr,
	&dev_attr_spare2_scale.attr,
	&dev_attr_spare2_bgscale.attr,
	&dev_attr_spare3_x0.attr,
	&dev_attr_spare3_x1.attr,
	&dev_attr_spare3_scale.attr,
	&dev_attr_spare3_bgscale.attr,
	&dev_attr_spare4_x0.attr,
	&dev_attr_spare4_x1.attr,
	&dev_attr_spare4_scale.attr,
	&dev_attr_spare4_bgscale.attr,
	&dev_attr_spare5_x0.attr,
	&dev_attr_spare5_x1.attr,
	&dev_attr_spare5_scale.attr,
	&dev_attr_spare5_bgscale.attr,
	&dev_attr_diffbaserange_x0.attr,
	&dev_attr_diffbaserange_x1.attr,
	&dev_attr_diffbaserange_scale.attr,
	&dev_attr_diffbaserange_bgscale.attr,
	&dev_attr_diffrange_x0.attr,
	&dev_attr_diffrange_x1.attr,
	&dev_attr_diffrange_scale.attr,
	&dev_attr_diffrange_bgscale.attr,
	&dev_attr_ediffbaserange_x0.attr,
	&dev_attr_ediffbaserange_x1.attr,
	&dev_attr_ediffbaserange_scale.attr,
	&dev_attr_ediffbaserange_bgscale.attr,
	&dev_attr_ediffrange_x0.attr,
	&dev_attr_ediffrange_x1.attr,
	&dev_attr_ediffrange_scale.attr,
	&dev_attr_ediffrange_bgscale.attr,
	NULL,
};

static const struct attribute_group fpga_group = {
	.attrs = fpga_attrs,
	.bin_attrs = fpga_bin_attrs,
};

static const struct attribute_group signal_processing_group = {
	.attrs = signal_processing_group_attrs,
	.name = "signal_processing"
};

const struct attribute_group *fpga_attribute_groups[] = {
	&fpga_group,
	&signal_processing_group,
	NULL,
};

