// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO driver for Ricoh RN5T618 PMIC
 *
 * Copyright (C) 2016 Marek Vasut <marex@denx.de>
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mfd/rn5t618.h>

struct rn5t618_gpio {
	struct rn5t618		*rn5t618;
	struct gpio_chip	gc;
};

static int rn5t618_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct rn5t618_gpio *priv = gpiochip_get_data(gc);
	u32 value;
	int ret;

	/* Read input pin value */
	ret = regmap_read(priv->rn5t618->regmap, RN5T618_MON_IOIN, &value);
	if (ret)
		return ret;

	return value & BIT(offset);
}

static void rn5t618_gpio_set(struct gpio_chip *gc, unsigned int offset,
			     int value)
{
	struct rn5t618_gpio *priv = gpiochip_get_data(gc);

	/* Configure the pin output value */
	regmap_update_bits(priv->rn5t618->regmap, RN5T618_IOOUT,
			   BIT(offset), value ? BIT(offset) : 0);
}

static int rn5t618_gpio_direction_input(struct gpio_chip *gc,
					unsigned int offset)
{
	struct rn5t618_gpio *priv = gpiochip_get_data(gc);

	/* Configure the pin direction as input */
	return regmap_update_bits(priv->rn5t618->regmap, RN5T618_IOSEL,
				  BIT(offset), 0);
}

static int rn5t618_gpio_direction_output(struct gpio_chip *gc,
					unsigned int offset, int value)
{
	struct rn5t618_gpio *priv = gpiochip_get_data(gc);
	int ret;

	/* Configure the pin output value */
	ret = regmap_update_bits(priv->rn5t618->regmap, RN5T618_IOOUT,
				 BIT(offset), value ? BIT(offset) : 0);
	if (ret)
		return ret;

	/* Configure the pin direction as output */
	return regmap_update_bits(priv->rn5t618->regmap, RN5T618_IOSEL,
				  BIT(offset), BIT(offset));
}

static struct gpio_chip rn5t618_gpio_chip_ref = {
	.label			= "rn5t618-gpio",
	.owner			= THIS_MODULE,
	.get			= rn5t618_gpio_get,
	.set			= rn5t618_gpio_set,
	.direction_input	= rn5t618_gpio_direction_input,
	.direction_output	= rn5t618_gpio_direction_output,
	.can_sleep		= true,
	.ngpio			= 5,
	.base			= -1,
};

static int rn5t618_gpio_probe(struct platform_device *pdev)
{
	struct rn5t618_gpio *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->rn5t618 = dev_get_drvdata(pdev->dev.parent);
	priv->gc = rn5t618_gpio_chip_ref;
	priv->gc.parent = &pdev->dev;
	priv->gc.of_node = pdev->dev.of_node;

	ret = devm_gpiochip_add_data(&pdev->dev, &priv->gc, priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static struct platform_driver rn5t618_gpio_driver = {
	.probe	= rn5t618_gpio_probe,
	.driver	= {
		.name	= "rn5t618-gpio",
	},
};

module_platform_driver(rn5t618_gpio_driver);

MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_DESCRIPTION("RN5T618 GPIO driver");
MODULE_LICENSE("GPL v2");
