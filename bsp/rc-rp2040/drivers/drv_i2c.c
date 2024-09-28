/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2020-11-28     bigmagic       first version
 */

#include "drv_i2c.h"
#include "drv_gpio.h"
#include <rtdbg.h>
#include <drivers/i2c.h>
#include "hardware/dma.h"
#include "hardware/i2c.h"

#define I2C_MAX_TX_LEN  256
struct rp2040_i2c_device_info {
  i2c_inst_t* dev;
  int sda_pin;
  int scl_pin;
  dma_channel_config tx_dma_config;
  dma_channel_config rx_dma_config;
  int tx_dma_channel;
  int rx_dma_channel;
  struct rt_semaphore dma_completed;
  int irq_number;
  void (*irq_handler)();
  uint16_t dma_cmd_buffer[I2C_MAX_TX_LEN];
};


static rt_err_t dma_init(struct rp2040_i2c_device_info* i2c_dev) {
    int tx_channel = dma_claim_unused_channel(false);
    if (tx_channel < 0) {
      // TODO:
        return -RT_ERROR;
    }

    i2c_dev->tx_dma_config = dma_channel_get_default_config(tx_channel);
    channel_config_set_transfer_data_size(&i2c_dev->tx_dma_config, DMA_SIZE_16);
    channel_config_set_read_increment(&i2c_dev->tx_dma_config, true);
    channel_config_set_write_increment(&i2c_dev->tx_dma_config, false);
    channel_config_set_dreq(&i2c_dev->tx_dma_config, i2c_get_dreq(i2c_dev->dev, true));
    i2c_dev->tx_dma_channel = tx_channel;

    int rx_channel = dma_claim_unused_channel(false);
    if (rx_channel < 0) {
      // TODO:
        return -RT_ERROR;
    }

    i2c_dev->rx_dma_config = dma_channel_get_default_config(rx_channel);
    channel_config_set_transfer_data_size(&i2c_dev->rx_dma_config, DMA_SIZE_8);
    channel_config_set_read_increment(&i2c_dev->rx_dma_config, false);
    channel_config_set_write_increment(&i2c_dev->rx_dma_config, true);
    channel_config_set_dreq(&i2c_dev->rx_dma_config, i2c_get_dreq(i2c_dev->dev, false));
    i2c_dev->rx_dma_channel = rx_channel;

    return RT_EOK;
}

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

static rt_err_t rp2040_i2c_device_init(struct rp2040_i2c_device_info* dev_info) {
  gpio_set_function(dev_info->scl_pin, GPIO_FUNC_I2C);
  gpio_set_function(dev_info->sda_pin, GPIO_FUNC_I2C);
  rt_sem_init(&dev_info->dma_completed, NULL, 0, 0);
  i2c_init(dev_info->dev, 100000);
  irq_set_exclusive_handler(dev_info->irq_number, dev_info->irq_handler);
  i2c_get_hw(dev_info->dev)->dma_cr |= I2C_IC_DMA_CR_TDMAE_VALUE_ENABLED | I2C_IC_DMA_CR_RDMAE_VALUE_ENABLED;
  return RT_EOK;
}

static void fill_dma_cmd_buffer(uint16_t* dma_cmd_buffer, struct rt_i2c_msg* msg, bool stop) {
  uint16_t* cmd_ptr = dma_cmd_buffer;
  if (msg->flags & RT_I2C_RD) {
    for (int i = 0; i < msg->len; ++i, ++cmd_ptr) {
       *cmd_ptr = I2C_IC_DATA_CMD_CMD_BITS;
    }
  } else {
    for (int i = 0; i < msg->len; ++i, ++cmd_ptr) {
       *cmd_ptr = msg->buf[i];
    }
  }
  dma_cmd_buffer[0] |= I2C_IC_DATA_CMD_RESTART_BITS;
  if (stop) {
      dma_cmd_buffer[msg->len - 1] |= I2C_IC_DATA_CMD_STOP_BITS;
  }
}

static rt_size_t i2c_master_xfer(struct rt_i2c_bus_device *bus,
                                    struct rt_i2c_msg msgs[],
                                    rt_uint32_t num)
{
  rt_size_t ret = 0;
  struct rp2040_i2c_device_info* dev_info = bus->priv;
  i2c_inst_t* dev = dev_info->dev;
  if (dma_init(dev_info) != RT_EOK) {
    goto finish;
  }

  for (int i = 0; i < num; ++i, ++ret) {
    struct rt_i2c_msg* current_msg = &msgs[i];

    if (current_msg->len > I2C_MAX_TX_LEN) {
      goto finish;
    }

    i2c_get_hw(dev)->enable = 0;
    i2c_get_hw(dev)->tar = current_msg->addr;
    i2c_get_hw(dev)->enable = 1;
    i2c_get_hw(dev)->intr_mask = (
        I2C_IC_INTR_MASK_M_STOP_DET_BITS |
        I2C_IC_INTR_MASK_M_TX_ABRT_BITS);
    irq_set_enabled(I2C0_IRQ, true);

    fill_dma_cmd_buffer(dev_info->dma_cmd_buffer, current_msg, true);
    dma_channel_configure(
        dev_info->tx_dma_channel, &dev_info->tx_dma_config, &i2c_get_hw(dev)->data_cmd, dev_info->dma_cmd_buffer, current_msg->len, false);

    if (current_msg->flags & RT_I2C_RD) {
      dma_channel_configure(
          dev_info->rx_dma_channel, &dev_info->rx_dma_config, current_msg->buf, &i2c_get_hw(dev)->data_cmd, current_msg->len, false);
      dma_start_channel_mask(1 << dev_info->rx_dma_channel);
    }

    dma_start_channel_mask(1 << dev_info->tx_dma_channel);

    // TODO: wait until completed (with timeout)
    rt_sem_take(&dev_info->dma_completed, RT_WAITING_FOREVER);
    irq_set_enabled(I2C0_IRQ, false);
    // TODO: break the loop as soon as there is any failure
  }

finish:
  dma_deinit(dev_info);
  i2c_get_hw(dev)->enable = 0;
  return ret;
}

static rt_size_t i2c_slave_xfer(struct rt_i2c_bus_device *bus,
                                    struct rt_i2c_msg msgs[],
                                    rt_uint32_t num)
{
    return 0;
}

static rt_err_t i2c_bus_control(struct rt_i2c_bus_device *bus,
                                      rt_uint32_t cmd,
                                      rt_uint32_t arg)
{
    return RT_EOK;
}

static inline void i2c_irq_handler(struct rp2040_i2c_device_info* config) {
  const uint32_t status = i2c_get_hw(config->dev)->intr_stat;

  if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
    // abort
    i2c_get_hw(config->dev)->clr_tx_abrt;
    // set a flag that transaction was aborted
    rt_sem_release(&config->dma_completed);
  } else if (status & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
    i2c_get_hw(config->dev)->clr_stop_det;
    rt_sem_release(&config->dma_completed);
  }
}

static const struct rt_i2c_bus_device_ops rp2040_i2c_ops =
{
    .master_xfer = i2c_master_xfer,
    .slave_xfer = i2c_slave_xfer,
    .i2c_bus_control = i2c_bus_control,
};

// DEBUG:
#define RC_RP2040_USING_I2C0

#if defined (RC_RP2040_USING_I2C0)
#define I2C0_BUS_NAME    "i2c0"
static void i2c0_irq_handler();
static struct rp2040_i2c_device_info hw_device0 =
{
  .dev = i2c0,
  .sda_pin = 4,
  .scl_pin = 5,
  .tx_dma_channel = -1,
  .rx_dma_channel = -1,
  .irq_number = I2C0_IRQ,
  .irq_handler = i2c0_irq_handler,
};

struct rt_i2c_bus_device device0 =
{
    .ops = &rp2040_i2c_ops,
    .priv =  (void *)&hw_device0,
};

static void i2c0_irq_handler() {
  i2c_irq_handler(&hw_device0);
}
#endif

#if defined (RC_RP2040_USING_I2C1)
#define I2C0_BUS_NAME    "i2c1"
static void i2c0_irq_handler();
static struct rp2040_i2c_device_info hw_device1 =
{
  .dev = i2c0,
  .sda_pin = 6,
  .scl_pin = 7,
  .tx_dma_channel = -1,
  .rx_dma_channel = -1,
  .irq_number = I2C1_IRQ,
  .irq_handler = i2c1_irq_handler,
};

struct rt_i2c_bus_device device1 =
{
    .ops = &rp2040_i2c_ops,
    .priv =  (void *)&hw_device1,
};

static void i2c1_irq_handler() {
  i2c_irq_handler(&hw_device0);
}
#endif


int rt_hw_i2c_init(void)
{
#if defined(RC_RP2040_USING_I2C0)
    rp2040_i2c_device_init(&hw_device0);
    rt_i2c_bus_device_register(&device0, I2C0_BUS_NAME);
#endif

#if defined(RC_RP2040_USING_I2C1)
    rp2040_i2c_device_init(&hw_device1);
    rt_i2c_bus_device_register(&device1, I2C1_BUS_NAME);
#endif

    return 0;
}

INIT_DEVICE_EXPORT(rt_hw_i2c_init);
