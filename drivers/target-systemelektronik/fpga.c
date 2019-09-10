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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/pci-aspm.h>
#include <linux/aer.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/delay.h>

#include "fpga.h"

#define TARGET_FPGA_DRIVER_NAME "target-fpga"
#define PCI_VENDOR_ID_TARGET 0x1172
#define PCI_DEVICE_ID_TARGET_FPGA 0x0004
#define TARGET_FPGA_DATA_SIZE 16384 * PAGE_SIZE

#define FPGA_SUSPEND_BASE	0x900
#define FPGA_ENABLE_SUSPEND	(FPGA_SUSPEND_BASE + 0x00)

static struct class *device_class;
static dev_t fpga_devt;

#define PCIE_ATU_VIEWPORT               0x900
#define PCIE_ATU_REGION_INBOUND         (0x1 << 31)
#define PCIE_ATU_REGION_INDEX0          (0x0 << 0)
#define PCIE_ATU_REGION_INDEX1          (0x1 << 0)
#define PCIE_ATU_CR1                    0x904
#define PCIE_ATU_TYPE_MEM               (0x0 << 0)
#define PCIE_ATU_CR2                    0x908
#define PCIE_ATU_ENABLE                 (0x1 << 31)
#define PCIE_ATU_LOWER_BASE             0x90C
#define PCIE_ATU_UPPER_BASE             0x910
#define PCIE_ATU_LIMIT                  0x914
#define PCIE_ATU_LOWER_TARGET           0x918
#define PCIE_ATU_UPPER_TARGET           0x91C

/* Parameters for the waiting for iATU enabled routine */
#define LINK_WAIT_MAX_IATU_RETRIES	5
#define LINK_WAIT_IATU_MIN		9000
#define LINK_WAIT_IATU_MAX		10000

static void _dw_pcie_prog_viewport_inbound(
	struct pci_dev *pdev, u32 viewport,
	u64 fpga_base, u64 ram_base, u64 size)
{
	u32 retries, val;

	pci_write_config_dword(pdev, PCIE_ATU_VIEWPORT,
			       PCIE_ATU_REGION_INBOUND | viewport);
	pci_write_config_dword(pdev, PCIE_ATU_LOWER_BASE,
			       fpga_base);
	pci_write_config_dword(pdev, PCIE_ATU_UPPER_BASE,
			       fpga_base >> 32);
	pci_write_config_dword(pdev, PCIE_ATU_LIMIT,
			       fpga_base + size - 1);
	pci_write_config_dword(pdev, PCIE_ATU_LOWER_TARGET,
			       ram_base);
	pci_write_config_dword(pdev, PCIE_ATU_UPPER_TARGET,
			       ram_base + size - 1);
	pci_write_config_dword(pdev, PCIE_ATU_CR1,
			       PCIE_ATU_TYPE_MEM);
	pci_write_config_dword(pdev, PCIE_ATU_CR2,
			       PCIE_ATU_ENABLE);

	dev_info(&pdev->dev,
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

	for (retries = 0; retries < LINK_WAIT_MAX_IATU_RETRIES; retries++)
	{
		pci_read_config_dword(pdev, PCIE_ATU_CR2, &val);

		if (val == PCIE_ATU_ENABLE)
		{
			dev_info(&pdev->dev, "iATU took %d retries", retries);
			return;
		}

		usleep_range(LINK_WAIT_IATU_MIN, LINK_WAIT_IATU_MAX);
	}
	dev_err(&pdev->dev, "iATU is not being enabled\n");
}

static void dw_pcie_prog_viewports_inbound(struct fpga_dev *fdev)
{
	struct pci_dev *root_complex = fdev->pdev;
const int offset = 0x40000000;
	while (!pci_is_root_bus(root_complex->bus))
		root_complex = root_complex->bus->self;
	fdev->ram_base_data = (u64)fdev->data.dma_handle;
	_dw_pcie_prog_viewport_inbound(
		root_complex,
		PCIE_ATU_REGION_INDEX0,
		offset + 0,
		(u64)fdev->data.dma_handle,
		fdev->data.size
	);
	fdev->ram_base_counts = (u64)fdev->counts.dma_handle;
	_dw_pcie_prog_viewport_inbound(
		root_complex,
		PCIE_ATU_REGION_INDEX1,
		offset + fdev->data.size,
		(u64)fdev->counts.dma_handle,
		fdev->counts.size
	);
}

static bool _fpga_allocate_buffer(struct pci_dev *pdev,
				    struct fpga_ringbuffer *pws)
{
	pws->start = dmam_alloc_coherent(&pdev->dev, pws->size, &pws->dma_handle, GFP_KERNEL | __GFP_ZERO);
	if (pws->start == NULL)
	{
		dev_err(&pdev->dev, "Could not alloc %d bytes", pws->size);
		return false;
	}
	return true;
}

static bool fpga_allocate_buffers(struct fpga_dev *fdev)
{
	if(!_fpga_allocate_buffer(fdev->pdev, &fdev->data))
		return false;
	return _fpga_allocate_buffer(fdev->pdev, &fdev->counts);
}

static void _fpga_free_buffer(struct pci_dev *pdev,
				    struct fpga_ringbuffer *pws)
{
	dmam_free_coherent(&pdev->dev, pws->size, pws->start, pws->dma_handle);
}

static void fpga_free_buffers(struct fpga_dev *fdev)
{
	_fpga_free_buffer(fdev->pdev, &fdev->data);
	_fpga_free_buffer(fdev->pdev, &fdev->counts);
}

static irqreturn_t handle_msi(int irq, void *data)
{
	struct fpga_dev *fdev = data;
	int to_add;
	int *counts_buffer = (int*)fdev->counts.start;
	u32 position = fdev->counts_position;

	to_add = counts_buffer[position];
	position = (position + 1) & 16383;
	fdev->counts_position = position;

	atomic_add_return(to_add, &fdev->unread_data_items);

	if (irq == fdev->timestamp_irq)
		fdev->timestamp_reset = 1;

	complete(&fdev->data_has_arrived);

	return IRQ_HANDLED;
}

static int fpga_setup_irq(struct fpga_dev *fdev)
{
	struct pci_dev *pdev = fdev->pdev;
	int irq;
	int ret;

	ret = pci_alloc_irq_vectors(pdev, 2, 2, PCI_IRQ_MSI | PCI_IRQ_MSIX);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not request MSI\n");
		return ret;
	}
	dev_info(&pdev->dev, "Enabled %d interrupts\n", ret);

	irq = pci_irq_vector(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq, handle_msi,
			       0, "fpga-data", fdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq %d\n", irq);
		return ret;
	}

	irq = pci_irq_vector(pdev, 1);
	fdev->timestamp_irq = irq;
	ret = devm_request_irq(&pdev->dev, irq, handle_msi,
			       0, "fpga-timestamp", fdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq %d\n", irq);
		return ret;
	}

	return 0;
}

static void fpga_teardown_irq(struct fpga_dev *fdev)
{
	struct pci_dev *pdev = fdev->pdev;
	int irq;

	irq = pci_irq_vector(pdev, 1);
	devm_free_irq(&pdev->dev, irq, fdev);
	irq = pci_irq_vector(pdev, 0);
	devm_free_irq(&pdev->dev, irq, fdev);

	pci_free_irq_vectors(pdev);
}

void bar_write(struct device *dev, u32 value, int offset)
{
	struct fpga_dev *fdev = dev_get_drvdata(dev);

	iowrite32(value, fdev->bar + offset);
}

u32 bar_read(struct device *dev, int offset)
{
	struct fpga_dev *fdev = dev_get_drvdata(dev);

	return ioread32(fdev->bar + offset);
}

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
	struct fpga_dev *fdev = filp->private_data;
	return dma_mmap_coherent(&fdev->pdev->dev,
				 vma,
				 fdev->data.start,
				 fdev->data.dma_handle,
				 fdev->data.size);
}

static ssize_t fpga_cdev_read(struct file *filp, char __user *buf,
			      size_t size, loff_t *offset)
{
	int bytes_to_read, from_position, wait_result;
	struct fpga_dev *fdev = filp->private_data;

	wait_result = wait_for_completion_killable_timeout(
		&fdev->data_has_arrived, msecs_to_jiffies(250)
	);

	if (wait_result < 0)
		return wait_result;

	bytes_to_read = atomic_xchg(&fdev->unread_data_items, 0);

	from_position = fdev->unsent_start;
	fdev->unsent_start = (from_position + bytes_to_read) & (fdev->data.size - 1);

	if (fdev->timestamp_reset)
	{
		fdev->timestamp_reset = 0;
		return -ECANCELED;
	}

	if (bytes_to_read == 0)
		return -ETIME;

	if (copy_to_user(buf, &from_position, sizeof(int)))
		return -EINVAL;
	if (copy_to_user(buf + sizeof(int), &bytes_to_read, sizeof(int)))
		return -EINVAL;

	return sizeof(int) + sizeof(int);
}

static const struct file_operations fpga_cdev_ops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.open		= fpga_cdev_open,
	.release	= fpga_cdev_release,
	.read		= fpga_cdev_read,
	.mmap		= fpga_cdev_mmap,
};

static enum flash_type get_flash_type(struct device *dev, u32 flash_type)
{
	if ((flash_type & 0x00ffff00) == 0x00ba1800) {
		return N25Q;
	}
	else if ((flash_type & 0x0000ff00) == 0x00001800) {
		return EPCQ;
	}
	else {
		dev_err(dev, "Invalid flash type: %x", flash_type);
		return N25Q;
	}
}

static int fpga_driver_probe(struct pci_dev *pdev,
			     const struct pci_device_id *id)
{
	int ret;
	u32 flash_type;
	struct fpga_dev *fdev;

	if (pdev->vendor != PCI_VENDOR_ID_TARGET || pdev->device != PCI_DEVICE_ID_TARGET_FPGA)
		return -ENODEV;

	fdev = devm_kzalloc(&pdev->dev, sizeof(struct fpga_dev), GFP_KERNEL);
	if (!fdev) {
		ret = -ENOMEM;
		goto err_managed;
	}

	fdev->pdev = pdev;
	fdev->data.size = TARGET_FPGA_DATA_SIZE;
	atomic_set(&fdev->unread_data_items, 0);
	fdev->unsent_start = 0;
	fdev->counts_position = 0;
	fdev->counts.size = 16 * PAGE_SIZE;
	init_completion(&fdev->data_has_arrived);
	fdev->ram_base_data = 0;
	fdev->ram_base_counts = 0;
	fdev->timestamp_reset = 0;

	pci_set_drvdata(pdev, fdev);

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "pci_enable_device() failed\n");
		goto err_managed;
	}

	ret = pcim_iomap_regions(pdev, 0x3, TARGET_FPGA_DRIVER_NAME);
	if (ret) {
		dev_err(&pdev->dev, "pcim_iomap_regions() failed\n");
		goto err_managed;
	}
	fdev->bar = pcim_iomap_table(pdev)[1];

	if (!fpga_allocate_buffers(fdev)) {
		ret = -ENOMEM;
		goto err_managed;
	}
	memset(fdev->counts.start, -2, fdev->counts.size);

	dw_pcie_prog_viewports_inbound(fdev);
	ret = fpga_setup_irq(fdev);
	if (ret) {
		goto err_managed;
	}

	if (pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_ERR))
		ret = pci_enable_pcie_error_reporting(pdev);
	else
		dev_info(&pdev->dev, "AER not supported\n");

	pci_set_master(pdev);

	flash_type = bar_read(&pdev->dev, FPGA_FLASH_TYPE);
	fdev->flash_type = get_flash_type(&pdev->dev, flash_type);

	fdev->dev = MKDEV(MAJOR(fpga_devt), 0);

	cdev_init(&fdev->cdev, &fpga_cdev_ops);
	fdev->cdev.owner = THIS_MODULE;
	ret = cdev_add(&fdev->cdev, fdev->dev, 1);
	if (ret)
		goto err_cdev;

	fdev->device = device_create_with_groups(device_class, &pdev->dev,
		fdev->dev, fdev, fpga_attribute_groups, "target-fpga");
	if (IS_ERR(fdev->device)) {
		ret = PTR_ERR(fdev->device);
		printk(KERN_WARNING "Error %d while trying to create target-fpga", ret);
		goto err_device;
	}

	ret = target_fpga_platform_driver_probe(fdev, fpga_devt, device_class);
	if (ret)
		goto err_plat;

	return 0;

err_plat:
	device_destroy(device_class, fdev->dev);
err_device:
	cdev_del(&fdev->cdev);
err_cdev:
	pci_clear_master(pdev);
	pci_disable_pcie_error_reporting(pdev);
	fpga_teardown_irq(fdev);
err_managed:
	return ret;
}

static void fpga_driver_remove(struct pci_dev *pdev)
{
	struct fpga_dev *fdev;
	fdev = pci_get_drvdata(pdev);

	target_fpga_platform_driver_remove(fdev, device_class);
	device_destroy(device_class, fdev->dev);
	cdev_del(&fdev->cdev);

	pcim_iounmap_regions(fdev->pdev, 3);
	fpga_free_buffers(fdev);
	pci_clear_master(pdev);
	pci_disable_pcie_error_reporting(pdev);
	fpga_teardown_irq(fdev);
}

#ifdef CONFIG_PM
static int fpga_driver_suspend (struct pci_dev *pdev, pm_message_t state)
{
	struct fpga_dev *fdev;
	fdev = pci_get_drvdata(pdev);

	bar_write(&pdev->dev, 1, FPGA_ENABLE_SUSPEND);

	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));

	return 0;
}

static int fpga_driver_resume (struct pci_dev *pdev)
{
	int ret;
	struct fpga_dev *fdev;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to enable device after resume (%d)\n", ret);
		return ret;
	}

	pci_set_master(pdev);

	fdev = pci_get_drvdata(pdev);

	// reset the data DMA pointer
	atomic_set(&fdev->unread_data_items, 0);
	fdev->unsent_start = 0;
	fdev->counts_position = 0;

	// re-programm atu viewport
	dw_pcie_prog_viewports_inbound(fdev);

	bar_write(&pdev->dev, 0, FPGA_ENABLE_SUSPEND);

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

	ret = alloc_chrdev_region(&fpga_devt, 0, 2, TARGET_FPGA_DRIVER_NAME);
	if (ret)
		goto exit;

	device_class = class_create(THIS_MODULE, "target-fpga");
	if (IS_ERR(device_class)) {
		ret = PTR_ERR(device_class);
		goto err_class;
	}

	ret = pci_register_driver(&fpga_driver);
	if (ret) {
		goto err_driver;
	}

	return 0;

err_driver:
	class_destroy(device_class);
err_class:
	unregister_chrdev_region(fpga_devt, 2);
exit:
	return ret;
}

static void __exit fpga_driver_exit(void)
{
	pci_unregister_driver(&fpga_driver);
	class_destroy(device_class);
	unregister_chrdev_region(fpga_devt, 2);
}

module_init(fpga_driver_init);
module_exit(fpga_driver_exit);

MODULE_AUTHOR("Target Systemelektronik");
MODULE_DESCRIPTION("PCI Express driver module for our FPGA");
MODULE_LICENSE("GPL");
