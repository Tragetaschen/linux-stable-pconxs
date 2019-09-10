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

#include "common-attributes.c"

#define FPGA_DAC_BASE		0x300
#define FPGA_MUX_BASE		0x400
#define FPGA_AFE_BASE		0x600

#define FPGA_ACTIVE_CLOCK	(FPGA_SYSTEM_BASE + 0x10)

#define FPGA_DAC1		(FPGA_DAC_BASE + 0x00)
#define FPGA_DAC2		(FPGA_DAC_BASE + 0x04)

#define FPGA_MUX		(FPGA_MUX_BASE + 0x00)

#define FPGA_SYNC		(FPGA_STREAM_BASE + 0x0c)
#define FPGA_EFFICIENCY_CUT	(FPGA_STREAM_BASE + 0x1c)

#define FPGA_AFE_COMMAND	(FPGA_AFE_BASE + 0x00)
#define FPGA_AFE_ON		(FPGA_AFE_BASE + 0x04)
#define FPGA_AFE_HV_ON		(FPGA_AFE_BASE + 0x08)
#define FPGA_AFE_HV		(FPGA_AFE_BASE + 0x0c)
#define FPGA_AFE_DAC		(FPGA_AFE_BASE + 0x10)
//... FPGA_AFE_DAC7		(FPGA_AFE_BASE + 0x28)

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

VALUE_RW(active_clock, FPGA_ACTIVE_CLOCK, "%d");
VALUE_RW(fpga_dac1, FPGA_DAC1, "%d");
VALUE_RW(fpga_dac2, FPGA_DAC2, "%d");
VALUE_RW(mux, FPGA_MUX, "0x%x");
VALUE_WO(sync, FPGA_SYNC);
VALUE_RW(efficiency_cut, FPGA_EFFICIENCY_CUT, "0x%x");
DEVICE_ATTR_RW(afe_on);
DEVICE_ATTR_RW(hv);
DEVICE_ATTR(dac1, S_IWUSR | S_IRUGO, dac_show, dac_store);
DEVICE_ATTR(dac2, S_IWUSR | S_IRUGO, dac_show, dac_store);
DEVICE_ATTR(dac3, S_IWUSR | S_IRUGO, dac_show, dac_store);
DEVICE_ATTR(dac4, S_IWUSR | S_IRUGO, dac_show, dac_store);
DEVICE_ATTR(dac5, S_IWUSR | S_IRUGO, dac_show, dac_store);
DEVICE_ATTR(dac6, S_IWUSR | S_IRUGO, dac_show, dac_store);
DEVICE_ATTR(dac7, S_IWUSR | S_IRUGO, dac_show, dac_store);

static struct attribute *fpga_attrs[] = {
	&dev_attr_ext_freq.attr,
	&dev_attr_pll_mult.attr,
	&dev_attr_active_clock.attr,
	&dev_attr_build_time.attr,
	&dev_attr_build_number.attr,
	&dev_attr_version.attr,
	&dev_attr_flash_type.attr,
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
	&dev_attr_efficiency_cut.attr,
	&dev_attr_afe_on.attr,
	&dev_attr_hv.attr,
	&dev_attr_dac1.attr,
	&dev_attr_dac2.attr,
	&dev_attr_dac3.attr,
	&dev_attr_dac4.attr,
	&dev_attr_dac5.attr,
	&dev_attr_dac6.attr,
	&dev_attr_dac7.attr,

	&dev_attr_ram_base_data.attr,
	&dev_attr_ram_base_counts.attr,
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

int target_fpga_platform_driver_probe(struct fpga_dev *fdev, dev_t fpga_devt, struct class *device_class)
{
	return 0;
}

void target_fpga_platform_driver_remove(struct fpga_dev *fdev, struct class *device_class)
{
}

