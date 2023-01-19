/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author          Notes
 * 2020-06-22     bigmagic        first version
 */
#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

#include "drv_spi.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// DEBUG:
#define BSP_USING_SPI0_BUS
#define BSP_USING_SPI0_DEVICE0

#ifdef RT_USING_SPI

struct rp2040_spi_hw {
    spi_inst_t* internal_bus_device;
    struct rt_spi_bus bus_device;
    rt_uint32_t baudrate;
    rt_uint8_t miso_pin;
    rt_uint8_t mosi_pin;
    rt_uint8_t sck_pin;
};

struct rp2040_spi_device {
  const char* device_name;
  struct rt_spi_device spi_device;
};


#if defined (BSP_USING_SPI0_BUS)

#define SPI0_BUS_NAME      "spi0"
static struct rp2040_spi_hw spi0_hw_dev = {
  .internal_bus_device = spi0,
  .baudrate = 5000000,
  .miso_pin = 3,
  .mosi_pin = 0, 
  .sck_pin = 2, 
};



#if defined (BSP_USING_SPI0_DEVICE0)

#define SPI0_DEVICE0_NAME  "spi0.0"
static struct rp2040_spi_device spi0_device0 = {
  .device_name = SPI0_DEVICE0_NAME,
};

#endif

#endif

static rt_err_t rp2040_spi_configure(struct rt_spi_device *device, struct rt_spi_configuration *cfg)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);
    // TODO:
}

static rt_uint32_t rp2040_spi_xfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    rt_err_t res;
    rt_uint8_t flag;
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);
    RT_ASSERT(device->parent.user_data != RT_NULL);
    RT_ASSERT(message != RT_NULL);
    RT_ASSERT(message->send_buf != RT_NULL || message->recv_buf != RT_NULL);
    struct rt_spi_configuration config = device->config;
    struct rp2040_spi_hw* hw = (struct rp2040_spi_hw *)device->bus->parent.user_data;
    //
  // TODO: set speed
    for (struct rt_spi_message* current_message = message; current_message != RT_NULL; current_message = current_message->next) {
      if (message->send_buf != RT_NULL && message->recv_buf != RT_NULL) {
        spi_write_read_blocking(hw->internal_bus_device, current_message->send_buf, current_message->recv_buf, current_message->length);
      } else if (message->send_buf != RT_NULL) {
      } else {
      }
    }
}

static struct rt_spi_ops rp2040_spi_ops =
{
    .configure = rp2040_spi_configure,
    .xfer = rp2040_spi_xfer
};

rt_err_t rp2040_spi_bus_attach_device(const char *bus_name, struct rp2040_spi_device *device)
{
    rt_err_t ret;
    RT_ASSERT(device != RT_NULL);
    ret = rt_spi_bus_attach_device(&device->spi_device, device->device_name, bus_name, (void *)(device));
    return ret;
}

rt_err_t rp2040_spi_hw_init(struct rp2040_spi_hw *hw) {
  gpio_set_function(hw->miso_pin, GPIO_FUNC_SPI);
  gpio_set_function(hw->mosi_pin, GPIO_FUNC_SPI);
  gpio_set_function(hw->sck_pin, GPIO_FUNC_SPI);
  spi_init(hw->internal_bus_device, hw->baudrate);
}

int rt_hw_spi_init(void)
{
#if defined (BSP_USING_SPI0_BUS)
    rp2040_spi_hw_init(&spi0_hw_dev);
    rt_spi_bus_register(&spi0_hw_dev.bus_device, SPI0_BUS_NAME, &rp2040_spi_ops);
    spi0_hw_dev.bus_device.parent.user_data = &spi0_hw_dev;

#if defined (BSP_USING_SPI0_DEVICE0)
    rp2040_spi_bus_attach_device(SPI0_DEVICE0_NAME, &spi0_device0);
#endif
#endif
    return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_hw_spi_init);

#if 0
rt_err_t raspi_spi_bus_attach_device(const char *bus_name, struct raspi_spi_device *device)
{
    rt_err_t ret;
    RT_ASSERT(device != RT_NULL);
    ret = rt_spi_bus_attach_device(device->spi_device, device->device_name, bus_name, (void *)(device));
    return ret;
}

rt_err_t raspi_spi_hw_init(struct raspi_spi_hw_config *hwcfg)
{
    prev_raspi_pin_mode(hwcfg->sclk_pin, hwcfg->sclk_mode);
    prev_raspi_pin_mode(hwcfg->miso_pin, hwcfg->miso_mode);
    prev_raspi_pin_mode(hwcfg->mosi_pin, hwcfg->mosi_mode);
#if defined (BSP_USING_SPI0_DEVICE0)
    prev_raspi_pin_mode(hwcfg->ce0_pin, hwcfg->ce0_mode);
#endif

#if defined (BSP_USING_SPI0_DEVICE1)
    prev_raspi_pin_mode(hwcfg->ce1_pin, hwcfg->ce1_mode);
#endif
    //clear rx and tx
    SPI_REG_CS(hwcfg->hw_base) = (SPI_CS_CLEAR_TX | SPI_CS_CLEAR_RX);
    return RT_EOK;
}

static struct rt_spi_ops raspi_spi_ops =
{
    .configure = raspi_spi_configure,
    .xfer = raspi_spi_xfer
};

struct raspi_spi_hw_config raspi_spi0_hw =
{
    .spi_num = 0,
    .sclk_pin = GPIO_PIN_11,
    .sclk_mode = ALT0,
    .mosi_pin = GPIO_PIN_10,
    .mosi_mode = ALT0,
    .miso_pin = GPIO_PIN_9,
    .miso_mode = ALT0,

#if defined (BSP_USING_SPI0_DEVICE0)
    .ce0_pin = GPIO_PIN_8,
    .ce0_mode = ALT0,
#endif

#if defined (BSP_USING_SPI0_DEVICE1)
    .ce1_pin = GPIO_PIN_7,
    .ce1_mode = ALT0,
#endif
    .hw_base = SPI_0_BASE,
};
#endif

#endif

