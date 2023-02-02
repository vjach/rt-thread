/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author          Notes
 * 2020-06-22     bigmagic        first version
 */
#include "drv_spi.h"

#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

// DEBUG:

#ifdef RT_USING_SPI

struct rp2040_spi_hw {
  spi_inst_t *internal_bus_device;
  struct rt_spi_bus bus_device;
  rt_uint32_t baudrate;
  rt_uint8_t miso_pin;
  rt_uint8_t mosi_pin;
  rt_uint8_t sck_pin;
};

struct rp2040_spi_device {
  const char *device_name;
  struct rt_spi_device spi_device;
  rt_uint8_t cs_pin;
};

#if defined(BSP_USING_SPI0_BUS)

#define SPI0_BUS_NAME "spi0"
static struct rp2040_spi_hw spi0_hw_dev = {
    .internal_bus_device = spi0,
    .baudrate = 5000000,
    .miso_pin = 0,
    .mosi_pin = 2,
    .sck_pin = 3,
};

#if defined(BSP_USING_SPI0_DEVICE0)
static struct rp2040_spi_device spi0_device0 = {
    .device_name = "can",
    .cs_pin = 1,
};
#endif

#if defined(BSP_USING_SPI0_DEVICE1)
static struct rp2040_spi_device spi0_device1 = {
    .device_name = "barometer",
    .cs_pin = 22,
};
#endif

#if defined(BSP_USING_SPI0_DEVICE2)
static struct rp2040_spi_device spi0_device2 = {
    .device_name = "compass",
    .cs_pin = 23,
};
#endif

#if defined(BSP_USING_SPI0_DEVICE3)
static struct rp2040_spi_device spi0_device3 = {
    .device_name = "imu",
    .cs_pin = 28,
};
#endif

#endif

static rt_err_t rp2040_spi_configure(struct rt_spi_device *device,
                                     struct rt_spi_configuration *cfg) {
  RT_ASSERT(device != RT_NULL);
  RT_ASSERT(cfg != RT_NULL);
  // TODO:
  return RT_EOK;
}

static rt_uint32_t rp2040_spi_xfer(struct rt_spi_device *device,
                                   struct rt_spi_message *message) {
  RT_ASSERT(device != RT_NULL);
  RT_ASSERT(device->bus != RT_NULL);
  RT_ASSERT(device->parent.user_data != RT_NULL);
  RT_ASSERT(message != RT_NULL);
  RT_ASSERT(message->send_buf != RT_NULL || message->recv_buf != RT_NULL);
  struct rp2040_spi_hw *hw =
      (struct rp2040_spi_hw *)device->bus->parent.user_data;
  struct rp2040_spi_device *internal_device =
      (struct rp2040_spi_device *)device->parent.user_data;

  if (message->cs_take) {
    rt_pin_write(internal_device->cs_pin, 0);
  }

  if (message->send_buf != RT_NULL && message->recv_buf != RT_NULL) {
    spi_write_read_blocking(hw->internal_bus_device, message->send_buf,
                            message->recv_buf, message->length);
  } else if (message->send_buf != RT_NULL) {
    spi_write_blocking(hw->internal_bus_device, message->send_buf,
                       message->length);
  } else {
    spi_read_blocking(hw->internal_bus_device, 0x00, message->recv_buf,
                      message->length);
  }

  if (message->cs_release) {
    rt_pin_write(internal_device->cs_pin, 1);
  }

  return 1;
}

static struct rt_spi_ops rp2040_spi_ops = {.configure = rp2040_spi_configure,
                                           .xfer = rp2040_spi_xfer};

rt_err_t rp2040_spi_bus_attach_device(const char *bus_name,
                                      struct rp2040_spi_device *device) {
  rt_err_t ret;
  RT_ASSERT(device != RT_NULL);
  ret = rt_spi_bus_attach_device(&device->spi_device, device->device_name,
                                 bus_name, (void *)(device));
  return ret;
}

rt_err_t rp2040_spi_hw_init(struct rp2040_spi_hw *hw) {
  spi_init(hw->internal_bus_device, hw->baudrate);
  gpio_set_function(hw->miso_pin, GPIO_FUNC_SPI);
  gpio_set_function(hw->mosi_pin, GPIO_FUNC_SPI);
  gpio_set_function(hw->sck_pin, GPIO_FUNC_SPI);
  return RT_EOK;
}

rt_err_t rp2040_spi_device_init(struct rp2040_spi_device *device) {
  rt_pin_mode(device->cs_pin, PIN_MODE_OUTPUT);
  rt_pin_write(device->cs_pin, 1);
  return RT_EOK;
}

int rt_hw_spi_init(void) {
#if defined(BSP_USING_SPI0_BUS)
  rp2040_spi_hw_init(&spi0_hw_dev);
  rt_spi_bus_register(&spi0_hw_dev.bus_device, SPI0_BUS_NAME, &rp2040_spi_ops);
  spi0_hw_dev.bus_device.parent.user_data = &spi0_hw_dev;

#if defined(BSP_USING_SPI0_DEVICE0)
  rp2040_spi_bus_attach_device(SPI0_BUS_NAME, &spi0_device0);
#endif

#if defined(BSP_USING_SPI0_DEVICE1)
  rp2040_spi_bus_attach_device(SPI0_BUS_NAME, &spi0_device1);
#endif

#if defined(BSP_USING_SPI0_DEVICE2)
  rp2040_spi_bus_attach_device(SPI0_BUS_NAME, &spi0_device2);
#endif

#if defined(BSP_USING_SPI0_DEVICE3)
  rp2040_spi_bus_attach_device(SPI0_BUS_NAME, &spi0_device3);
#endif

#endif
  return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_hw_spi_init);
#endif
