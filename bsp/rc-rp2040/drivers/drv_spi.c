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

#ifdef RT_USING_SPI

struct rp2040_spi_hw_config {
};

struct rp2040_spi_device {
    char *device_name;
    struct rt_spi_bus *spi_bus;
    struct rt_spi_device *spi_device;
    struct rp2040_spi_hw_config *spi_hw_config;
    uint8_t cs_pin;
};


#if defined (BSP_USING_SPI0_BUS)

#define SPI0_BUS_NAME      "spi0"
#define SPI0_DEVICE0_NAME  "spi0.0"
#define SPI0_DEVICE1_NAME  "spi0.1"
#define SPI0_DEVICE2_NAME  "spi0.2"
#define SPI0_DEVICE3_NAME  "spi0.3"

struct rt_spi_bus spi0_bus;

#if defined (BSP_USING_SPI0_DEVICE0)
static struct rt_spi_device spi0_device0;
#endif

#endif

static rt_err_t spi_configure(struct rt_spi_device *device, struct rt_spi_configuration *cfg)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);
    // TODO:
}

static rt_uint32_t spi_xfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    rt_err_t res;
    rt_uint8_t flag;
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);
    RT_ASSERT(device->parent.user_data != RT_NULL);
    RT_ASSERT(message != RT_NULL);
    RT_ASSERT(message->send_buf != RT_NULL || message->recv_buf != RT_NULL);
    struct rt_spi_configuration config = device->config;
  //  struct raspi_spi_device * hw_config = (struct raspi_spi_device *)device->parent.user_data;
  // TODO:
}

static struct rt_spi_ops raspi_spi_ops =
{
    .configure = spi_configure,
    .xfer = spi_xfer
};

rt_err_t rp2040_spi_bus_attach_device(const char *bus_name, struct raspi_spi_device *device)
{
    rt_err_t ret;
    RT_ASSERT(device != RT_NULL);
    ret = rt_spi_bus_attach_device(device->spi_device, device->device_name, bus_name, (void *)(device));
    return ret;
}

rt_err_t rp2040_spi_hw_init(struct rp2040_spi_hw_config *hwcfg) {
}

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

#if defined (BSP_USING_SPI0_DEVICE0)
struct raspi_spi_device raspi_spi0_device0 =
{
    .device_name = SPI0_DEVICE0_NAME,
    .spi_bus = &spi0_bus,
    .spi_device = &spi0_device0,
    .spi_hw_config = &raspi_spi0_hw,
    .cs_pin = GPIO_PIN_8,
};
#endif

#if defined (BSP_USING_SPI0_DEVICE1)
struct raspi_spi_device raspi_spi0_device1 =
{
    .device_name = SPI0_DEVICE1_NAME,
    .spi_bus = &spi0_bus,
    .spi_device = &spi0_device1,
    .spi_hw_config = &raspi_spi0_hw,
    .cs_pin = GPIO_PIN_7,
};
#endif
#endif

int rt_hw_spi_init(void)
{
#if defined (BSP_USING_SPI0_BUS)
    //raspi_spi_hw_init(&raspi_spi0_hw);
    //rt_spi_bus_register(&spi0_bus, SPI0_BUS_NAME, &raspi_spi_ops);

#if defined (BSP_USING_SPI0_DEVICE0)
    //raspi_spi_bus_attach_device(SPI0_BUS_NAME, &raspi_spi0_device0);
#endif
#endif
    return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_hw_spi_init);
