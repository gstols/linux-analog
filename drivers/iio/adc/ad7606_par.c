// SPDX-License-Identifier: GPL-2.0
/*
 * AD7606 Parallel Interface ADC driver
 *
 * Copyright 2011 Analog Devices Inc.
 * Copyright 2024 Analog Devices Inc.
 * Copyright 2024 BayLibre SAS.
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/backend.h>
#include "ad7606.h"

static const struct iio_chan_spec ad7606b_bi_channels[] = {
	AD7606_BI_CHANNEL(0),
	AD7606_BI_CHANNEL(1),
	AD7606_BI_CHANNEL(2),
	AD7606_BI_CHANNEL(3),
	AD7606_BI_CHANNEL(4),
	AD7606_BI_CHANNEL(5),
	AD7606_BI_CHANNEL(6),
	AD7606_BI_CHANNEL(7),
};

static int ad7606_bi_update_scan_mode(struct iio_dev *indio_dev, const unsigned long *scan_mask)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	unsigned int c;

	for (c = 0; c < indio_dev->num_channels; c++) {
		if (test_bit(c, scan_mask))
			iio_backend_chan_enable(st->back, c);
		else
			iio_backend_chan_disable(st->back, c);
	}

	return 0;
}

static int ad7606_bi_postenable(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	pwm_enable(st->cnvst_pwm);

	return 0;
}

static int ad7606_bi_predisable(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	pwm_disable(st->cnvst_pwm);

	return 0;
}

static const struct iio_buffer_setup_ops ad7606_bi_setup_ops = {
	.postenable = &ad7606_bi_postenable,
	.predisable = &ad7606_bi_predisable,
};

static int ad7606_bi_setup_iio_backend(struct device *dev, struct iio_dev *indio_dev)
{
		struct ad7606_state *st = iio_priv(indio_dev);
		unsigned int ret, c;

		st->back = devm_iio_backend_get(dev, NULL);
		if (IS_ERR(st->back))
			return PTR_ERR(st->back);

		/* If the device is iio_backend powered the PWM is mandatory */
		if (!st->cnvst_pwm)
			return -EINVAL;

		ret = devm_iio_backend_request_buffer(dev, st->back, indio_dev);
		if (ret)
			return ret;

		ret = devm_iio_backend_enable(dev, st->back);
		if (ret)
			return ret;

		struct iio_backend_data_fmt data = {
			.sign_extend = true,
			.enable = true,
		};
		for (c = 0; c < indio_dev->num_channels; c++) {
			ret = iio_backend_data_format_set(st->back, c, &data);
			if (ret)
				return ret;
		}

		indio_dev->channels = ad7606b_bi_channels;
		indio_dev->num_channels = 8;
		indio_dev->setup_ops = &ad7606_bi_setup_ops;

		return 0;
}

static const struct ad7606_bus_ops ad7606_bi_bops = {
	.iio_backend_config = ad7606_bi_setup_iio_backend,
	.update_scan_mode = ad7606_bi_update_scan_mode,
};

static int ad7606_par16_read_block(struct device *dev,
				   int count, void *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad7606_state *st = iio_priv(indio_dev);

	insw((unsigned long)st->base_address, buf, count);

	return 0;
}

static const struct ad7606_bus_ops ad7606_par16_bops = {
	.read_block = ad7606_par16_read_block,
};

static int ad7606_par8_read_block(struct device *dev,
				  int count, void *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad7606_state *st = iio_priv(indio_dev);

	insb((unsigned long)st->base_address, buf, count * 2);

	return 0;
}

static const struct ad7606_bus_ops ad7606_par8_bops = {
	.read_block = ad7606_par8_read_block,
};

static int ad7606_par_probe(struct platform_device *pdev)
{
	const struct platform_device_id *id = platform_get_device_match_data(pdev);
	struct resource *res;
	void __iomem *addr;
	resource_size_t remap_size;
	int irq;
	struct iio_backend *back;

	/*For now, only the AD7606B is backend compatible.*/
	if (id->driver_data == ID_AD7606B) {
		back = devm_iio_backend_get(&pdev->dev, NULL);
		if (IS_ERR(back))
			return PTR_ERR(back);

		return ad7606_probe(&pdev->dev, 0, NULL,
				    id->name, id->driver_data,
				    &ad7606_bi_bops);
	}
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	addr = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(addr))
		return PTR_ERR(addr);

	remap_size = resource_size(res);

	return ad7606_probe(&pdev->dev, irq, addr,
			    id->name, id->driver_data,
			    remap_size > 1 ? &ad7606_par16_bops :
			    &ad7606_par8_bops);
}

static const struct platform_device_id ad7606_driver_ids[] = {
	{ .name	= "ad7605-4", .driver_data = ID_AD7605_4, },
	{ .name	= "ad7606-4", .driver_data = ID_AD7606_4, },
	{ .name	= "ad7606-6", .driver_data = ID_AD7606_6, },
	{ .name	= "ad7606-8", .driver_data = ID_AD7606_8, },
	{ .name	= "ad7606b", .driver_data = ID_AD7606B, },
	{ }
};
MODULE_DEVICE_TABLE(platform, ad7606_driver_ids);

static const struct platform_device_id ad7606_driver_infos[] = {
	[ID_AD7605_4] = { .name	= "ad7605-4", .driver_data = ID_AD7605_4, },
	[ID_AD7606_4] = { .name	= "ad7606-4", .driver_data = ID_AD7606_4, },
	[ID_AD7606_6] = { .name	= "ad7606-6", .driver_data = ID_AD7606_6, },
	[ID_AD7606_8] = { .name	= "ad7606-8", .driver_data = ID_AD7606_8, },
	[ID_AD7606B] = { .name	= "ad7606b", .driver_data = ID_AD7606B, },
};

static const struct of_device_id ad7606_of_match[] = {
	{ .compatible = "adi,ad7605-4", .data = &ad7606_driver_infos[ID_AD7605_4] },
	{ .compatible = "adi,ad7606-4", .data = &ad7606_driver_infos[ID_AD7606_4] },
	{ .compatible = "adi,ad7606-6", .data = &ad7606_driver_infos[ID_AD7606_6] },
	{ .compatible = "adi,ad7606-8", .data = &ad7606_driver_infos[ID_AD7606_8] },
	{ .compatible = "adi,ad7606b", .data = &ad7606_driver_infos[ID_AD7606B] },
	{ }
};
MODULE_DEVICE_TABLE(of, ad7606_of_match);

static struct platform_driver ad7606_driver = {
	.probe = ad7606_par_probe,
	.id_table = ad7606_driver_ids,
	.driver = {
		.name = "ad7606",
		.pm = AD7606_PM_OPS,
		.of_match_table = ad7606_of_match,
	},
};
module_platform_driver(ad7606_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_AUTHOR("Alin-Tudor Sferle <alin-tudor.sferle@analog.com>");
MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_AUTHOR("Guillaume Stols <gstols@baylibre.com>");
MODULE_DESCRIPTION("Analog Devices AD7606 ADC");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_AD7606);
MODULE_IMPORT_NS(IIO_BACKEND);
