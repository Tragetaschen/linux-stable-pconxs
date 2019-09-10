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

#include <linux/uaccess.h>
#include <linux/pci.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>

#include "common-attributes.c"

#define FPGA_AFE3_BASE		0x700
#define FPGA_MEASUREMENT_BASE	0xa00
#define FPGA_DOSE_RATE_BASE	0xc00
#define FPGA_FIR_BASE		0xd00

#define FPGA_PREPAUSE		(FPGA_STREAM_BASE + 0x1c)
#define FPGA_TRIGGER_SAMPLES	(FPGA_STREAM_BASE + 0x24)
#define FPGA_TRIGGER_PAUSE	(FPGA_STREAM_BASE + 0x2c)
#define FPGA_TRIGGER_WINDOW	(FPGA_STREAM_BASE + 0x30)

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

#define FPGA_DOSE_RATE_COEFFICIENTS	(FPGA_DOSE_RATE_BASE + 0x00)
#define FPGA_DOSE_RATE_WEIGHTS		(FPGA_DOSE_RATE_BASE + 0x08)
#define FPGA_DOSE_RATE_LIMITS		(FPGA_DOSE_RATE_BASE + 0x24)
#define FPGA_DOSE_RATE_FINE_GAIN	(FPGA_DOSE_RATE_BASE + 0x3c)
#define FPGA_DOSE_RATE_ALARM_LEVEL	(FPGA_DOSE_RATE_BASE + 0x40)
#define FPGA_DOSE_RATE_MAX_ENERGY	(FPGA_DOSE_RATE_BASE + 0x44)
#define FPGA_DOSE_RATE_BIT_SHIFTS	(FPGA_DOSE_RATE_BASE + 0x48)

#define AFE_SERIAL_SIZE 80
#define AFE_CONFIG_SIZE 1024

struct afe3_dev {
	struct fpga_dev *fdev;
	struct mutex mutex;
	dev_t devt;
	struct cdev cdev;
	struct device *device;
	struct workqueue_struct *workqueue;
	int data_valid_chars;
	char serial[AFE_SERIAL_SIZE];
	char data[AFE_CONFIG_SIZE];
};

static int afe3_write(struct device *dev, u32 cmd, u32 index, u32 data)
{
	struct fpga_dev *fdev = dev_get_drvdata(dev);
	struct afe3_dev *adev = fdev->platform_device;
	int ret;

	ret = mutex_lock_interruptible(&adev->mutex);
	if (ret < 0)
		return ret;

	bar_write(dev, cmd << 8 | index, FPGA_AFE3_COMMAND);
	bar_write(dev, data, FPGA_AFE3_DATA);
	// There's currently no way to know when it's done
	msleep_interruptible(25);

	mutex_unlock(&adev->mutex);
	return 0;
}

static int afe3_read(struct device *dev, u32 cmd, u32 index, u32* data)
{
	struct fpga_dev *fdev = dev_get_drvdata(dev);
	struct afe3_dev *adev = fdev->platform_device;

	u32 command, expected_value;
	int attempts = 0;
	int ret;
	const int max_attempts = 5;

	command = cmd << 8 | index;
	expected_value = 0x55 << 16 | cmd << 8 | index;

	ret = mutex_lock_interruptible(&adev->mutex);
	if (ret < 0)
		return ret;

	bar_write(dev, command, FPGA_AFE3_COMMAND);
	bar_write(dev, 0, FPGA_AFE3_DATA);

	do
	{
		++attempts;
		msleep_interruptible(25);
		command = bar_read(dev, FPGA_AFE3_COMMAND);
	}
	while (command != expected_value && attempts < max_attempts);

	if (attempts >= max_attempts)
		ret = -ETIMEDOUT;
	else
	{
		*data = bar_read(dev, FPGA_AFE3_DATA);
		ret = 0;
	}

	mutex_unlock(&adev->mutex);
	return ret;
}

static ssize_t roi_show(struct device *dev, struct device_attribute *attr, char *buf, int offset)
{
	int value_x0, value_x1, value_scale, value_bgscale;
	value_x0 = bar_read(dev, FPGA_MEASUREMENT_BASE + offset);
	value_x1 = bar_read(dev, FPGA_MEASUREMENT_BASE + offset + 4);
	value_scale = bar_read(dev, FPGA_MEASUREMENT_BASE + offset + 8);
	value_bgscale = bar_read(dev, FPGA_MEASUREMENT_BASE + offset + 12);
	return scnprintf(buf, PAGE_SIZE, "%d %d %d %d\n", value_x0, value_x1, value_scale, value_bgscale);
}

static size_t roi_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count, int offset)
{
	int ret;
	int value_x0, value_x1, value_scale, value_bgscale;
	ret = sscanf(buf, "%d %d %d %d", &value_x0, &value_x1, &value_scale, &value_bgscale);
	if (ret != 4)
		return -EINVAL;
	bar_write(dev, value_x0, FPGA_MEASUREMENT_BASE + offset);
	bar_write(dev, value_x1, FPGA_MEASUREMENT_BASE + offset + 4);
	bar_write(dev, value_scale, FPGA_MEASUREMENT_BASE + offset + 8);
	bar_write(dev, value_bgscale, FPGA_MEASUREMENT_BASE + offset + 12);
	return count;
}

static ssize_t fir_show(struct device *dev, struct device_attribute *attr, char *buf, int offset)
{
	int values[13];
	int i;
	for (i=0; i<13; ++i)
		values[i] = (s16)bar_read(dev, FPGA_FIR_BASE + offset + i * 4);
	return scnprintf(buf, PAGE_SIZE, "%d %d %d %d %d %d %d %d %d %d %d %d %d\n",
		values[0], values[1], values[2], values[3], values[4], values[5], values[6],
		values[7], values[8], values[9], values[10], values[11], values[12]);
}

static size_t fir_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count, int offset)
{
	int ret, i;
	int values[13];
	ret = sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d",
		&values[0], &values[1], &values[2], &values[3], &values[4], &values[5], &values[6],
		&values[7], &values[8], &values[9], &values[10], &values[11], &values[12]);
	if (ret != 13)
		return -EINVAL;
	for (i=0; i<13; ++i)
		bar_write(dev, values[i], FPGA_FIR_BASE + offset + i * 4);
	return count;
}

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
		int ret; \
		u32 data;\
		if (kstrtou32(buf, 0, &data)) \
			return -EINVAL; \
		ret = afe3_write(dev, cmd, index, data); \
		if (ret < 0) \
			return ret; \
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
	static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
	{ \
		return roi_show(dev, attr, buf, offset); \
	} \
	static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) \
	{ \
		return roi_store(dev, attr, buf, count, offset); \
	} \
DEVICE_ATTR_RW(name)

#define SP_FIR(name, offset) \
	static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
	{ \
		return fir_show(dev, attr, buf, offset); \
	} \
	static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) \
	{ \
		return fir_store(dev, attr, buf, count, offset); \
	} \
DEVICE_ATTR_RW(name)

static ssize_t coefficients_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int c0, c1;
	c0 = bar_read(dev, FPGA_DOSE_RATE_COEFFICIENTS);
	c1 = bar_read(dev, FPGA_DOSE_RATE_COEFFICIENTS + 4);
	return scnprintf(buf, PAGE_SIZE, "%d %d\n", c0, c1);
}

static ssize_t coefficients_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int c0, c1;
	ret = sscanf(buf, "%d %d", &c0, &c1);
	if (ret != 2)
		return -EINVAL;
	bar_write(dev, c0, FPGA_DOSE_RATE_COEFFICIENTS);
	bar_write(dev, c1, FPGA_DOSE_RATE_COEFFICIENTS + 4);
	return count;
}

static ssize_t weights_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int w0, w1, w2, w3, w4, w5, w6;
	w0 = bar_read(dev, FPGA_DOSE_RATE_WEIGHTS);
	w1 = bar_read(dev, FPGA_DOSE_RATE_WEIGHTS + 0x04);
	w2 = bar_read(dev, FPGA_DOSE_RATE_WEIGHTS + 0x08);
	w3 = bar_read(dev, FPGA_DOSE_RATE_WEIGHTS + 0x0c);
	w4 = bar_read(dev, FPGA_DOSE_RATE_WEIGHTS + 0x10);
	w5 = bar_read(dev, FPGA_DOSE_RATE_WEIGHTS + 0x14);
	w6 = bar_read(dev, FPGA_DOSE_RATE_WEIGHTS + 0x18);
	return scnprintf(buf, PAGE_SIZE, "%d %d %d %d %d %d %d\n", w0, w1, w2, w3, w4, w5, w6);
}

static ssize_t weights_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int w0, w1, w2, w3, w4, w5, w6;
	ret = sscanf(buf, "%d %d %d %d %d %d %d", &w0, &w1, &w2, &w3, &w4, &w5, &w6);
	if (ret != 7)
		return -EINVAL;
	bar_write(dev, w0, FPGA_DOSE_RATE_WEIGHTS);
	bar_write(dev, w1, FPGA_DOSE_RATE_WEIGHTS + 0x04);
	bar_write(dev, w2, FPGA_DOSE_RATE_WEIGHTS + 0x08);
	bar_write(dev, w3, FPGA_DOSE_RATE_WEIGHTS + 0x0c);
	bar_write(dev, w4, FPGA_DOSE_RATE_WEIGHTS + 0x10);
	bar_write(dev, w5, FPGA_DOSE_RATE_WEIGHTS + 0x14);
	bar_write(dev, w6, FPGA_DOSE_RATE_WEIGHTS + 0x18);
	return count;
}

static ssize_t limits_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int l0, l1, l2, l3, l4, l5;
	l0 = bar_read(dev, FPGA_DOSE_RATE_LIMITS);
	l1 = bar_read(dev, FPGA_DOSE_RATE_LIMITS + 0x04);
	l2 = bar_read(dev, FPGA_DOSE_RATE_LIMITS + 0x08);
	l3 = bar_read(dev, FPGA_DOSE_RATE_LIMITS + 0x0c);
	l4 = bar_read(dev, FPGA_DOSE_RATE_LIMITS + 0x10);
	l5 = bar_read(dev, FPGA_DOSE_RATE_LIMITS + 0x14);
	return scnprintf(buf, PAGE_SIZE, "%d %d %d %d %d %d\n", l0, l1, l2, l3, l4, l5);
}

static ssize_t limits_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int l0, l1, l2, l3, l4, l5;
	ret = sscanf(buf, "%d %d %d %d %d %d", &l0, &l1, &l2, &l3, &l4, &l5);
	if (ret != 6)
		return -EINVAL;
	bar_write(dev, l0, FPGA_DOSE_RATE_LIMITS);
	bar_write(dev, l1, FPGA_DOSE_RATE_LIMITS + 0x04);
	bar_write(dev, l2, FPGA_DOSE_RATE_LIMITS + 0x08);
	bar_write(dev, l3, FPGA_DOSE_RATE_LIMITS + 0x0c);
	bar_write(dev, l4, FPGA_DOSE_RATE_LIMITS + 0x10);
	bar_write(dev, l5, FPGA_DOSE_RATE_LIMITS + 0x14);
	return count;
}

static ssize_t bit_shifts_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int b0, b1, b2;
	b0 = bar_read(dev, FPGA_DOSE_RATE_BIT_SHIFTS);
	b1 = bar_read(dev, FPGA_DOSE_RATE_BIT_SHIFTS + 4);
	b2 = bar_read(dev, FPGA_DOSE_RATE_BIT_SHIFTS + 8);
	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n", b0, b1, b2);
}

static ssize_t bit_shifts_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int b0, b1, b2;
	ret = sscanf(buf, "%d %d %d", &b0, &b1, &b2);
	if (ret != 3)
		return -EINVAL;
	bar_write(dev, b0, FPGA_DOSE_RATE_BIT_SHIFTS);
	bar_write(dev, b1, FPGA_DOSE_RATE_BIT_SHIFTS + 4);
	bar_write(dev, b2, FPGA_DOSE_RATE_BIT_SHIFTS + 8);
	return count;
}

static ssize_t serial_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fpga_dev *fdev = dev_get_drvdata(dev);
	struct afe3_dev *adev = fdev->platform_device;

	return scnprintf(buf, PAGE_SIZE, "%s\n", adev->serial);
}

VALUE_RW(prepause, FPGA_PREPAUSE, "%d");
VALUE_RW(trigger_samples, FPGA_TRIGGER_SAMPLES, "%d");
VALUE_RW(trigger_pause, FPGA_TRIGGER_PAUSE, "%d");
VALUE_RW(trigger_window, FPGA_TRIGGER_WINDOW, "%d");
VALUE_RO(baseline_variance_first, FPGA_VARIANCE_FIRST, "%d");
VALUE_RO(baseline_variance_second, FPGA_VARIANCE_SECOND, "%d");
VALUE_WO(baseline_variance_reset, FPGA_VARIANCE_FIRST);

static struct attribute *fpga_attrs[] = {
	&dev_attr_ext_freq.attr,
	&dev_attr_pll_mult.attr,
	&dev_attr_build_time.attr,
	&dev_attr_build_number.attr,
	&dev_attr_version.attr,
	&dev_attr_flash_type.attr,
	&dev_attr_adc.attr,
	&dev_attr_trigger.attr,
	&dev_attr_samples.attr,
	&dev_attr_acq.attr,
	&dev_attr_pause.attr,
	&dev_attr_resolution.attr,
	&dev_attr_prepause.attr,
	&dev_attr_trigger_samples.attr,
	&dev_attr_trigger_pause.attr,
	&dev_attr_trigger_window.attr,
	&dev_attr_baseline_variance_first.attr,
	&dev_attr_baseline_variance_second.attr,
	&dev_attr_baseline_variance_reset.attr,

	&dev_attr_ram_base_data.attr,
	&dev_attr_ram_base_counts.attr,
	NULL,
};

static struct bin_attribute *fpga_bin_attrs[] = {
	&bin_attr_firmware,
	NULL,
};

VALUE_RW(raw_sample_count, FPGA_RAW_SAMPLE_COUNT, "%d");
VALUE_RW(shiftline1_index, FPGA_SHIFTLINE1_INDEX, "%d");
VALUE_RW(shiftline2_index, FPGA_SHIFTLINE2_INDEX, "%d");
VALUE_RW(fir_bank, FPGA_FIR_BANK, "%d");
VALUE_RW(aco, FPGA_ACO, "%d");
VALUE_RW(daco, FPGA_DACO, "%d");
SP_ROI(base_roi1, 0x28);
SP_ROI(base_roi2, 0x38);
SP_ROI(roi3, 0x48);
SP_ROI(roi4, 0x58);
SP_ROI(roi5, 0x68);
SP_ROI(roi6, 0x78);
SP_ROI(roi7, 0x88);
SP_ROI(roi8, 0x98);
SP_ROI(roi9, 0xa8);
SP_ROI(roi10, 0xb8);
SP_ROI(roi11, 0xc8);
SP_ROI(roi12, 0xd8);
SP_ROI(roi13, 0xe8);
SP_ROI(roi14, 0xf8);
SP_ROI(fir_roi1, 0x108);
SP_ROI(fir_roi2, 0x118);
SP_ROI(roi15, 0x128);
SP_ROI(roi16, 0x138);
SP_FIR(fir_bank0, 0 * 13 * 4);
SP_FIR(fir_bank1, 1 * 13 * 4);
SP_FIR(fir_bank2, 2 * 13 * 4);
SP_FIR(fir_bank3, 3 * 13 * 4);
SP_FIR(fir_bank4, 4 * 13 * 4);
SP_FIR(fir_bank5, 5 * 13 * 4);
SP_FIR(fir_bank6, 6 * 13 * 4);
SP_FIR(fir_bank7, 7 * 13 * 4);

static struct attribute *signal_processing_group_attrs[] = {
	&dev_attr_raw_sample_count.attr,
	&dev_attr_shiftline1_index.attr,
	&dev_attr_shiftline2_index.attr,
	&dev_attr_fir_bank.attr,
	&dev_attr_aco.attr,
	&dev_attr_daco.attr,
	&dev_attr_base_roi1.attr,
	&dev_attr_base_roi2.attr,
	&dev_attr_roi3.attr,
	&dev_attr_roi4.attr,
	&dev_attr_roi5.attr,
	&dev_attr_roi6.attr,
	&dev_attr_roi7.attr,
	&dev_attr_roi8.attr,
	&dev_attr_roi9.attr,
	&dev_attr_roi10.attr,
	&dev_attr_roi11.attr,
	&dev_attr_roi12.attr,
	&dev_attr_roi13.attr,
	&dev_attr_roi14.attr,
	&dev_attr_fir_roi1.attr,
	&dev_attr_fir_roi2.attr,
	&dev_attr_roi15.attr,
	&dev_attr_roi16.attr,
	&dev_attr_fir_bank0.attr,
	&dev_attr_fir_bank1.attr,
	&dev_attr_fir_bank2.attr,
	&dev_attr_fir_bank3.attr,
	&dev_attr_fir_bank4.attr,
	&dev_attr_fir_bank5.attr,
	&dev_attr_fir_bank6.attr,
	&dev_attr_fir_bank7.attr,
	NULL,
};

AFE3_RW(hv, 3, 2, 0);
AFE3_RW(baseline, 8, 7, 0);
AFE3_RW(config_version, 5, 4, 0);
AFE3_RW(wobble, 0x30, 0x31, 0);
DEVICE_ATTR_RO(serial);

AFE3_RO(v_dynode, 6, 0);
AFE3_RO(v_anode, 6, 1);
AFE3_RO(boost_meas, 6, 2);
AFE3_RO(i_boost, 6, 3);
AFE3_RO(i_anode, 6, 4);
AFE3_RO(v_phase_a, 6, 5);
AFE3_RO(v_phase_b, 6, 6);
AFE3_RO(v_t17, 6, 7);
AFE3_RO(temperature, 6, 10);
AFE3_RO(pulse_frequency, 6, 21);
AFE3_RO(pulse_symmetry, 6, 22);
AFE3_RO(regulator_out, 6, 23);
AFE3_RO(p_term, 6, 24);
AFE3_RO(i_term, 6, 25);
AFE3_RO(dac_boost, 0x20, 0);
AFE3_RO(dac_drv_bias_fine, 0x20, 1);
AFE3_RO(dac_drv_bias, 0x20, 2);
AFE3_RO(dac_amp_offset, 0x20, 3);

static struct attribute *afe_group_attrs[] = {
	&dev_attr_hv.attr,
	&dev_attr_baseline.attr,
	&dev_attr_config_version.attr,
	&dev_attr_wobble.attr,
	&dev_attr_serial.attr,

	&dev_attr_v_dynode.attr,
	&dev_attr_v_anode.attr,
	&dev_attr_boost_meas.attr,
	&dev_attr_i_boost.attr,
	&dev_attr_i_anode.attr,
	&dev_attr_v_phase_a.attr,
	&dev_attr_v_phase_b.attr,
	&dev_attr_v_t17.attr,
	&dev_attr_temperature.attr,
	&dev_attr_pulse_frequency.attr,
	&dev_attr_pulse_symmetry.attr,
	&dev_attr_regulator_out.attr,
	&dev_attr_p_term.attr,
	&dev_attr_i_term.attr,
	&dev_attr_dac_boost.attr,
	&dev_attr_dac_drv_bias_fine.attr,
	&dev_attr_dac_drv_bias.attr,
	&dev_attr_dac_amp_offset.attr,
	NULL,
};

DEVICE_ATTR_RW(coefficients);
DEVICE_ATTR_RW(weights);
DEVICE_ATTR_RW(limits);
VALUE_RW(fine_gain, FPGA_DOSE_RATE_FINE_GAIN, "%d");
VALUE_RW(alarm_level, FPGA_DOSE_RATE_ALARM_LEVEL, "%d");
VALUE_RW(max_energy, FPGA_DOSE_RATE_MAX_ENERGY, "%d");
DEVICE_ATTR_RW(bit_shifts);

static struct attribute *dose_rate_group_attrs[] = {
	&dev_attr_coefficients.attr,
	&dev_attr_weights.attr,
	&dev_attr_limits.attr,
	&dev_attr_fine_gain.attr,
	&dev_attr_alarm_level.attr,
	&dev_attr_max_energy.attr,
	&dev_attr_bit_shifts.attr,
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

static const struct attribute_group dose_rate_group = {
	.attrs = dose_rate_group_attrs,
	.name = "dose_rate"
};

const struct attribute_group *fpga_attribute_groups[] = {
	&fpga_group,
	&signal_processing_group,
	&dose_rate_group,
	NULL,
};

static const struct attribute_group afe_group = {
	.attrs = afe_group_attrs,
};

const struct attribute_group *afe_attribute_groups[] = {
	&afe_group,
	NULL,
};

struct afe_init_work {
	struct afe3_dev *adev;
	struct delayed_work work;
};

static void do_afe_init(struct work_struct *work)
{
	struct afe_init_work *afe_init = container_of(work, struct afe_init_work, work.work);
	struct afe3_dev *adev = afe_init->adev;
	struct device *dev = &adev->fdev->pdev->dev;
	u32* serial = (u32*)adev->serial;
	u32* data = (u32*)adev->data;
	int ret, i;

	kfree(afe_init);

	for (i=0; i<AFE_SERIAL_SIZE>>2; ++i)
	{
		ret = afe3_read(dev, 0xa, i, serial + i);
		if (ret != 0)
			goto err;
		if (serial[i] == 0)
			break;
	}
	adev->serial[AFE_SERIAL_SIZE-1]=0;

	for (i=0; i<AFE_CONFIG_SIZE>>2; ++i)
	{
		ret = afe3_read(dev, 5, i, data + i);
		if (ret != 0)
			goto err;
		adev->data_valid_chars += 4;
	}

	return;

err:
	dev_err(dev, "AFE initialization failed at index %d: %d", i, ret);
	adev->data_valid_chars = ret;
}

struct afe_sync_work {
	struct afe3_dev *adev;
	struct work_struct work;
	int offset;
	int size;
};

static void do_afe_sync(struct work_struct *work)
{
	struct afe_sync_work *afe_sync = container_of(work, struct afe_sync_work,work);
	struct afe3_dev *adev = afe_sync->adev;
	struct device *dev = &adev->fdev->pdev->dev;
	int offset = afe_sync->offset;
	int size = afe_sync->size;

	u32* data = (u32*)adev->data;
	int i;

	kfree(afe_sync);

	size += offset & 0x3;
	size += 3;

	offset >>= 2;
	size >>= 2;

	for (i=offset; i<offset+size; ++i)
		afe3_write(dev, 4, i, data[i]);
}

static int afe_cdev_open(struct inode *inode, struct file *file)
{
	struct afe3_dev *adev = container_of(inode->i_cdev, struct afe3_dev, cdev);
	if (adev->data_valid_chars < 0)
		return adev->data_valid_chars;

	file->private_data = adev;

	return generic_file_open(inode, file);
}

static int afe_cdev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t afe_cdev_read(struct file *file, char __user *buf,
			     size_t size, loff_t *offset)
{
	struct afe3_dev *adev = file->private_data;
	size_t actual_size;
	char* data_start;

	if (adev->data_valid_chars < 0)
		return adev->data_valid_chars;

	if (*offset >= AFE_CONFIG_SIZE)
		return 0;

	data_start = adev->data + *offset;
	actual_size = size;
	if (*offset + size >= AFE_CONFIG_SIZE)
		actual_size = AFE_CONFIG_SIZE - *offset;

	if (*offset + actual_size > adev->data_valid_chars)
		return -EBUSY;

	if (copy_to_user(buf, data_start, actual_size))
		return -EFAULT;

	*offset += actual_size;
	return actual_size;
}

static ssize_t afe_cdev_write(struct file *file, const char __user *buf,
			      size_t size, loff_t *offset)
{
	struct afe3_dev *adev = file->private_data;
	struct afe_sync_work *afe_sync;
	size_t actual_size;
	char* data_start;

	if (adev->data_valid_chars < 0)
		return adev->data_valid_chars;

	if (*offset >= AFE_CONFIG_SIZE)
		return 0;

	data_start = adev->data + *offset;
	actual_size = size;
	if (*offset + size >= AFE_CONFIG_SIZE)
		actual_size = AFE_CONFIG_SIZE - *offset;

	if (*offset + actual_size > adev->data_valid_chars)
		return -EBUSY;

	afe_sync = kzalloc(sizeof(struct afe_sync_work), GFP_KERNEL);
	if (!afe_sync)
		return -ENOMEM;
	afe_sync->adev = adev;
	INIT_WORK(&afe_sync->work, do_afe_sync);
	afe_sync->offset = (int)*offset;
	afe_sync->size = actual_size;

	if (copy_from_user(data_start, buf, actual_size))
	{
		kfree(afe_sync);
		return -EFAULT;
	}

	*offset += actual_size;
	queue_work(adev->workqueue, &afe_sync->work);

	return actual_size;
}

static loff_t afe_cdev_llseek(struct file *file, loff_t offset, int whence)
{
	return fixed_size_llseek(file, offset, whence, AFE_CONFIG_SIZE);
}

static int afe_cdev_fsync(struct file *file, loff_t start, loff_t end, int datasync)
{
	struct afe3_dev *adev = file->private_data;

	flush_workqueue(adev->workqueue);

	return 0;
}

static const struct file_operations afe_cdev_ops = {
	.owner		= THIS_MODULE,
	.open		= afe_cdev_open,
	.release	= afe_cdev_release,
	.read		= afe_cdev_read,
	.write		= afe_cdev_write,
	.llseek		= afe_cdev_llseek,
	.fsync		= afe_cdev_fsync,
};

int target_fpga_platform_driver_probe(struct fpga_dev *fdev, dev_t fpga_devt, struct class *device_class)
{
	struct afe3_dev *adev;
	struct afe_init_work *afe_init;
	int ret;

	adev = devm_kzalloc(&fdev->pdev->dev, sizeof(struct afe3_dev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;
	fdev->platform_device = adev;
	adev->fdev = fdev;
	mutex_init(&adev->mutex);
	adev->workqueue = create_singlethread_workqueue("AFE sync");
	if (!adev->workqueue)
		return -ENOMEM;

	adev->devt = MKDEV(MAJOR(fpga_devt), 1);
	cdev_init(&adev->cdev, &afe_cdev_ops);
	ret = cdev_add(&adev->cdev, adev->devt, 1);
	if (ret)
		goto err_cdev;

	adev->device = device_create_with_groups(device_class, &fdev->pdev->dev,
		adev->devt, fdev, afe_attribute_groups, "target-afe");
	if (IS_ERR(adev->device)) {
		ret = PTR_ERR(adev->device);
		printk(KERN_WARNING "Error %d while trying to create target-afe", ret);
		goto err_device;
	}

	afe_init = kzalloc(sizeof(struct afe_init_work), GFP_KERNEL);
	if (!afe_init)
	{
		ret = -ENOMEM;
		goto err_work;
	}
	afe_init->adev = adev;
	INIT_DELAYED_WORK(&afe_init->work, do_afe_init);
	queue_delayed_work(adev->workqueue, &afe_init->work, msecs_to_jiffies(1000));

	return 0;

err_work:
	device_destroy(device_class, adev->devt);
err_device:
	cdev_del(&adev->cdev);
err_cdev:
	destroy_workqueue(adev->workqueue);
	return ret;
}

void target_fpga_platform_driver_remove(struct fpga_dev *fdev, struct class *device_class)
{
	struct afe3_dev *adev;
	adev = fdev->platform_device;
	device_destroy(device_class, adev->devt);
	cdev_del(&adev->cdev);
	mutex_destroy(&adev->mutex);
}

