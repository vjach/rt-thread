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
#include "hardware/dma.h"

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
  if (device->cs_pin != -1) {
    rt_pin_mode(device->cs_pin, PIN_MODE_OUTPUT);
    rt_pin_write(device->cs_pin, 1);
  }
  return RT_EOK;
}

#define SPI1_BUS_NAME "spi1"
struct rp2040_spi_ext_hw {
  struct rp2040_spi_hw hw;
  unsigned int_pin;
  dma_channel_config tx_dma_config;
  dma_channel_config rx_dma_config;
  int tx_dma_channel;
  int rx_dma_channel;
  struct rt_semaphore* dma_completed;
};

static rt_err_t dma_init(struct rp2040_spi_ext_hw* ext_hw) {
  struct rp2040_spi_hw* hw = &(ext_hw->hw);
    int tx_channel = dma_claim_unused_channel(false);
    if (tx_channel < 0) {
      // TODO:
        return -RT_ERROR;
    }

    ext_hw->tx_dma_config = dma_channel_get_default_config(tx_channel);
    channel_config_set_transfer_data_size(&ext_hw->tx_dma_config, DMA_SIZE_8);
    channel_config_set_read_increment(&ext_hw->tx_dma_config, true);
    channel_config_set_write_increment(&ext_hw->tx_dma_config, false);
    channel_config_set_dreq(&ext_hw->tx_dma_config, spi_get_dreq(hw->internal_bus_device, true));
    ext_hw->tx_dma_channel = tx_channel;

    int rx_channel = dma_claim_unused_channel(false);
    if (rx_channel < 0) {
      // TODO:
        return -RT_ERROR;
    }

    ext_hw->rx_dma_config = dma_channel_get_default_config(rx_channel);
    channel_config_set_transfer_data_size(&ext_hw->rx_dma_config, DMA_SIZE_8);
    channel_config_set_read_increment(&ext_hw->rx_dma_config, false);
    channel_config_set_write_increment(&ext_hw->rx_dma_config, true);
    channel_config_set_dreq(&ext_hw->rx_dma_config, spi_get_dreq(hw->internal_bus_device, false));
    ext_hw->rx_dma_channel = rx_channel;

    return RT_EOK;
}

#if 0
static rt_err_t dma_deinit(struct rp2040_i2c_device_info* i2c_dev_info) {
  uint32_t unclaim_mask = 0;
  if (i2c_dev_info->tx_dma_channel >= 0) {
    unclaim_mask |= 1 << i2c_dev_info->tx_dma_channel;
  }

  if (i2c_dev_info->rx_dma_channel >= 0) {
    unclaim_mask |= 1 << i2c_dev_info->rx_dma_channel;
  }

  if (unclaim_mask) {
    dma_unclaim_mask(unclaim_mask);
  }
  return RT_EOK;
}
#endif

static rt_uint32_t rp2040_spi_ext_xfer(struct rt_spi_device *device,
    struct rt_spi_message *message) {

  struct rp2040_spi_hw *hw =
      (struct rp2040_spi_hw *)device->bus->parent.user_data;
  struct rp2040_spi_ext_hw* ext_hw = rt_container_of(hw, struct rp2040_spi_ext_hw, hw);
  // wait until mbox is empty
  dma_channel_configure(
      ext_hw->tx_dma_channel, &ext_hw->tx_dma_config, &spi_get_hw(hw->internal_bus_device)->dr, message->send_buf, message->length, false);

  dma_channel_configure(
      ext_hw->rx_dma_channel, &ext_hw->rx_dma_config, message->recv_buf, &spi_get_hw(hw->internal_bus_device)->dr, message->length, false);

  dma_start_channel_mask((1 << ext_hw->tx_dma_channel) | (1 << ext_hw->rx_dma_channel));
  rt_pin_write(ext_hw->int_pin, 1);

  // TODO: do that in interrupt
  rt_sem_take(ext_hw->dma_completed, RT_WAITING_FOREVER);
  //dma_channel_wait_for_finish_blocking(ext_hw->tx_dma_channel);
  rt_pin_write(ext_hw->int_pin, 0);
  return 1;
}

static struct rt_spi_ops rp2040_spi_ext_ops = {.configure = rp2040_spi_configure,
                                           .xfer = rp2040_spi_ext_xfer};

static struct rp2040_spi_ext_hw spi1_hw_dev = {
  {
    .internal_bus_device = spi1,
    .baudrate = 0,
    .miso_pin = 11,
    .mosi_pin = 8,
    .sck_pin = 10,
  },
  .int_pin = 12,
};

void transaction_completed(void* cookie) {
  struct rp2040_spi_ext_hw* ext_hw = (struct rp2040_spi_ext_hw*)cookie;
  struct rp2040_spi_hw* hw = &(ext_hw->hw);
  rt_sem_release(ext_hw->dma_completed);
}

rt_err_t rp2040_spi_ext_hw_init(struct rp2040_spi_ext_hw *ext_hw) {
  struct rp2040_spi_hw* hw = &(ext_hw->hw);
  gpio_set_function(9, GPIO_FUNC_SPI);
  rp2040_spi_hw_init(hw);
  spi_set_slave(hw->internal_bus_device, true);
  spi_set_format(hw->internal_bus_device, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
  rt_pin_mode(ext_hw->int_pin, PIN_MODE_OUTPUT);
  rt_pin_write(ext_hw->int_pin, 0);
  dma_init(ext_hw);
  rt_pin_attach_irq(9, PIN_IRQ_MODE_RISING, transaction_completed, ext_hw);
  ext_hw->dma_completed = rt_sem_create(RT_NULL, 0, RT_IPC_FLAG_FIFO);
  return RT_EOK;
}

static struct rp2040_spi_device spi1_device0 = {
    .device_name = "app_ctrl",
    // unused
    .cs_pin = -1,
};

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

  rp2040_spi_ext_hw_init(&spi1_hw_dev);
  rt_spi_bus_register(&spi1_hw_dev.hw.bus_device, SPI1_BUS_NAME, &rp2040_spi_ext_ops);
  spi1_hw_dev.hw.bus_device.parent.user_data = &spi1_hw_dev;
  rp2040_spi_bus_attach_device(SPI1_BUS_NAME, &spi1_device0);
  return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_hw_spi_init);
#endif
