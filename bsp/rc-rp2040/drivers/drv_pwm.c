/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_pwm.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

static rt_err_t rp2040_drv_pwm_control(struct rt_device_pwm *device, int cmd, void *arg);

static struct rt_pwm_ops rp2040_pwm_ops =
{
    .control = rp2040_drv_pwm_control
};

struct rp2040_pwm_slice {
  struct rt_device_pwm device;
  uint8_t id;
  uint8_t gpio_a;
  uint8_t gpio_b;
};

static struct rp2040_pwm_slice pwm_device0 = {
  .id = 7,
  .gpio_a = 14,
  .gpio_b = 15,
};

static rt_err_t rp2040_drv_pwm_enable(struct rp2040_pwm_slice *pwm_slice, uint8_t channel, rt_bool_t enable)
{
  if (channel == 0) {
    gpio_set_function(pwm_slice->gpio_a, enable ? GPIO_FUNC_PWM : GPIO_FUNC_SIO);
  } else  if (channel == 1){
    gpio_set_function(pwm_slice->gpio_b, enable ? GPIO_FUNC_PWM : GPIO_FUNC_SIO);
  } else {
    return RT_EINVAL;
  }

  return RT_EOK;
}

static rt_err_t rp2040_drv_pwm_get(struct rt_device_pwm *device )
{
  // TODO:
    return RT_EOK;
}

static rt_err_t rp2040_drv_pwm_set(struct rp2040_pwm_slice* pwm_slice, uint8_t channel, uint32_t duty)
{
  RT_ASSERT(channel < 2);

  uint16_t counter_value = (uint16_t)(((uint64_t)65536 *  duty) / (20000000));
  pwm_set_chan_level(pwm_slice->id, channel, counter_value);
  return RT_EOK;
}

static rt_err_t rp2040_drv_pwm_control(struct rt_device_pwm *device, int cmd, void *arg)
{
    struct rp2040_pwm_slice* pwm_slice = rt_container_of(device, struct rp2040_pwm_slice, device);
    struct rt_pwm_configuration *configuration = (struct rt_pwm_configuration *)arg;

    switch (cmd)
    {
    case PWM_CMD_ENABLE:
        return rp2040_drv_pwm_enable(pwm_slice, configuration->channel, RT_TRUE);
    case PWM_CMD_DISABLE:
        return rp2040_drv_pwm_enable(pwm_slice, configuration->channel, RT_FALSE);
    case PWM_CMD_SET:
        return rp2040_drv_pwm_set(pwm_slice, configuration->channel, configuration->pulse);
    case PWM_CMD_GET:
        return RT_EINVAL;
    default:
        return RT_EINVAL;
    }
}

static rt_err_t rp2040_drv_pwm_init(struct rp2040_pwm_slice* pwm_slice)
{
  RT_ASSERT(pwm_slice->id == pwm_gpio_to_slice_num(pwm_slice->gpio_a))
  RT_ASSERT(pwm_slice->id == pwm_gpio_to_slice_num(pwm_slice->gpio_b))

#define UPDATE_RATE 50

  uint32_t clk_freq = clock_get_hz(clk_sys);
  pwm_set_chan_level(pwm_slice->id, 0, 1);
  pwm_set_chan_level(pwm_slice->id, 1, 1);
  pwm_set_clkdiv(pwm_slice->id, (float)clk_freq / (UPDATE_RATE * 65536));
  pwm_set_gpio_level(pwm_slice->gpio_a, 0);
  pwm_set_gpio_level(pwm_slice->gpio_b, 0);
  pwm_set_enabled(pwm_slice->id, true);

  return RT_EOK;
}

int rt_hw_pwm_init(void)
{
    rt_err_t ret = RT_EOK;
    rp2040_drv_pwm_init(&pwm_device0);
    ret = rt_device_pwm_register(&pwm_device0.device, "pwm0", &rp2040_pwm_ops, RT_NULL);
    return ret;
}

INIT_BOARD_EXPORT(rt_hw_pwm_init);

