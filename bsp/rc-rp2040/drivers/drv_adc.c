/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-07     qiyu         first version
 */

#include <board.h>
#include <rtdebug.h>
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/adc.h>

// DEBUG:
#define BSP_USING_ADC

#ifdef BSP_USING_ADC
#include "hardware/adc.h"
#include "drv_adc.h"

static const uint8_t adc_pin[] = {26, 27, 28, 29};
static struct rt_adc_device rp2040_adc_hw_device0;

static rt_err_t rp2040_adc_enabled(struct rt_adc_device *device, rt_uint32_t channel, rt_bool_t enabled)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(channel < sizeof(adc_pin));
    if (enabled) {
      adc_gpio_init(adc_pin[channel]);
    } else {
      // TODO: disable adc on pin
    }

    return RT_EOK;
}

static rt_uint8_t rp2040_adc_get_resolution(struct rt_adc_device *device)
{
  return 12;
}

static rt_int16_t rp2040_adc_get_vref (struct rt_adc_device *device)
{
    return 3300;
}

static rt_err_t rp2040_adc_get_value(struct rt_adc_device *device, rt_uint32_t channel, rt_uint32_t *value)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(value != RT_NULL);
    RT_ASSERT(channel < sizeof(adc_pin));

    adc_select_input(channel);
    *value = adc_read();
    return RT_EOK;
}

static const struct rt_adc_ops rp2040_adc_ops =
{
    .enabled = rp2040_adc_enabled,
    .convert = rp2040_adc_get_value,
    .get_resolution = rp2040_adc_get_resolution,
    .get_vref = rp2040_adc_get_vref,
};

static int rp2040_adc_init(void)
{
  adc_init();
  return rt_hw_adc_register(&rp2040_adc_hw_device0, "adc0", &rp2040_adc_ops, RT_NULL);
}

INIT_DEVICE_EXPORT(rp2040_adc_init);

#endif
