// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (C) 2016 Marek Vasut <marex@denx.de>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include <linux/mfd/wm97xx.h>

#include "imx-audmux.h"

#define DAI_NAME_SIZE		32

struct imx_wm9712_data {
	struct snd_soc_dai_link	dai;
	struct snd_soc_card	card;
	struct platform_device	*codec;
	char			codec_dai_name[DAI_NAME_SIZE];
	char			platform_name[DAI_NAME_SIZE];
};

struct imx_priv {
	struct snd_soc_codec	*codec;
	struct platform_device	*pdev;
	struct snd_card		*snd_card;
};

static const struct snd_soc_dapm_widget imx_wm9712_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static struct snd_soc_ops imx_hifi_ops = {
};

static int imx_wm9712_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *cpu_np;
	struct platform_device *cpu_pdev;
	struct imx_wm9712_data *data;
	struct snd_soc_dai_link_component *comp;
	int int_port, ext_port;
	int ret;

	ret = of_property_read_u32(np, "mux-int-port", &int_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
		return ret;
	}

	/*
	 * The port numbering in the hardware manual starts at 1, while
	 * the audmux API expects it starts at 0.
	 */
	int_port--;
	ext_port--;

	ret = imx_audmux_v2_configure_port(ext_port,
			IMX_AUDMUX_V2_PTCR_SYN |
			IMX_AUDMUX_V2_PTCR_TFSEL(int_port) |
			IMX_AUDMUX_V2_PTCR_TFSDIR,
			IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux external port setup failed\n");
		return ret;
	}
	ret = imx_audmux_v2_configure_port(int_port,
			IMX_AUDMUX_V2_PTCR_SYN |
			IMX_AUDMUX_V2_PTCR_TCSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TCLKDIR,
			IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux internal port setup failed\n");
		return ret;
	}

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		return -EINVAL;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EINVAL;
		goto fail_cpu;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail_cpu;
	}

	comp = devm_kzalloc(&pdev->dev, 3 * sizeof(*comp), GFP_KERNEL);
	if (!comp) {
		ret = -ENOMEM;
		goto fail_cpu;
	}

	data->codec = platform_device_alloc("wm9712-codec", -1);
	if (!data->codec) {
		ret = -ENOMEM;
		goto fail_cpu;
	}
	ret = platform_device_add(data->codec);
	if (ret) {
		dev_err(&pdev->dev, "Cannot register codec device\n");
		goto fail_alloc;
	}

	data->dai.cpus		= &comp[0];
	data->dai.codecs	= &comp[1];
	data->dai.platforms	= &comp[2];

	data->dai.num_cpus	= 1;
	data->dai.num_codecs	= 1;
	data->dai.num_platforms	= 1;

	data->dai.name = "HiFi";
	data->dai.stream_name = "AC97 HiFi";
	data->dai.codecs->dai_name = "wm9712-hifi";
	data->dai.codecs->name = "wm9712-codec";
	data->dai.cpus->dai_name = dev_name(&cpu_pdev->dev);
	data->dai.platforms->of_node = cpu_np;
	data->dai.ops = &imx_hifi_ops;
	data->dai.dai_fmt = SND_SOC_DAIFMT_AC97 | SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBM_CFM;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail_codec;
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail_codec;
	data->card.num_links = 1;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = imx_wm9712_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_wm9712_dapm_widgets);

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail_codec;
	}

	of_node_put(cpu_np);

	return 0;

fail_codec:
	platform_device_del(data->codec);
fail_alloc:
	platform_device_put(data->codec);
fail_cpu:
	of_node_put(cpu_np);

	return ret;
}

static int imx_wm9712_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct imx_wm9712_data *data = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);

	platform_device_unregister(data->codec);

	return 0;
}

static const struct of_device_id imx_wm9712_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-wm9712", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm9712_dt_ids);

static struct platform_driver imx_wm9712_driver = {
	.driver = {
		.name = "imx-wm9712",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_wm9712_dt_ids,
	},
	.probe = imx_wm9712_probe,
	.remove = imx_wm9712_remove,
};
module_platform_driver(imx_wm9712_driver);

MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_DESCRIPTION("Freescale i.MX WM9712 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-wm9712");
