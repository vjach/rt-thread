/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2021-01-28     flybreak       first version
 */

#include <string.h>
#include "drv_gpio.h"

typedef void (*irq_handler)(void *args);
typedef struct {
  irq_handler handler;
  void *cookie;
} irq_entry;

static struct {
  irq_entry rising_edge;
  irq_entry falling_edge;
  /* TODO: level interrupts */
} pin_irq_entry[25];

void pico_gpio_handler(unsigned gpio, uint32_t events) {
  if (events & GPIO_IRQ_EDGE_FALL) {
    if (pin_irq_entry[gpio].falling_edge.handler != NULL) {
      pin_irq_entry[gpio].falling_edge.handler(pin_irq_entry[gpio].falling_edge.cookie);
    }
  }

  if (events & GPIO_IRQ_EDGE_RISE) {
    if (pin_irq_entry[gpio].rising_edge.handler != NULL) {
      pin_irq_entry[gpio].rising_edge.handler(pin_irq_entry[gpio].rising_edge.cookie);
    }
  }
}

static void pico_pin_mode(struct rt_device *dev, rt_base_t pin, rt_base_t mode)
{
    RT_ASSERT((0 <= pin) && (pin < NUM_BANK0_GPIOS));

    gpio_init(pin);
    switch (mode)
    {
    case PIN_MODE_OUTPUT:
        gpio_set_dir(pin, GPIO_OUT);
        break;
    case PIN_MODE_INPUT:
        gpio_set_dir(pin, GPIO_IN);
        break;
    case PIN_MODE_INPUT_PULLUP:
        gpio_pull_up(pin);
        break;
    case PIN_MODE_INPUT_PULLDOWN:
        gpio_pull_down(pin);
        break;
    case PIN_MODE_OUTPUT_OD:
        gpio_disable_pulls(pin);
        break;
    }
}

static void pico_pin_write(struct rt_device *dev, rt_base_t pin, rt_base_t value)
{
    RT_ASSERT((0 <= pin) && (pin < NUM_BANK0_GPIOS));
    gpio_put(pin, value);
}

static int pico_pin_read(struct rt_device *device, rt_base_t pin)
{
    RT_ASSERT((0 <= pin) && (pin < NUM_BANK0_GPIOS));
    return (gpio_get(pin)? PIN_HIGH : PIN_LOW);
}

static rt_err_t pin_attach_irq(struct rt_device *device, rt_int32_t pin, rt_uint32_t mode, void (*hdr)(void *args),
    void *args) {

  /* TODO: */
  uint32_t gpio_int_mode = GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE;
  if (mode == PIN_IRQ_MODE_FALLING) {
    pin_irq_entry[pin].falling_edge.handler = hdr;
    pin_irq_entry[pin].falling_edge.cookie = args;
    gpio_int_mode |= GPIO_IRQ_EDGE_FALL;
  } else if (mode == PIN_IRQ_MODE_RISING) {
    pin_irq_entry[pin].rising_edge.handler = hdr;
    pin_irq_entry[pin].rising_edge.cookie = args;
    gpio_int_mode |= GPIO_IRQ_EDGE_RISE;
  }

  gpio_set_irq_enabled_with_callback(pin, gpio_int_mode, true, pico_gpio_handler);
  return RT_EOK;
}

static const struct rt_pin_ops ops =
{
    pico_pin_mode,
    pico_pin_write,
    pico_pin_read,
    pin_attach_irq,
    RT_NULL,
    RT_NULL,
    RT_NULL,
};

int rt_hw_gpio_init(void)
{
    rt_device_pin_register("gpio", &ops, RT_NULL);
    memset(pin_irq_entry, 0, sizeof(pin_irq_entry));
    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_gpio_init);
