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

struct rp2040_i2c_hw_config {
};

static rt_size_t i2c_master_xfer(struct rt_i2c_bus_device *bus,
                                    struct rt_i2c_msg msgs[],
                                    rt_uint32_t num)
{
    return 0;
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


static rt_err_t i2c_configure(struct rp2040_i2c_hw_config *cfg)
{
    RT_ASSERT(cfg != RT_NULL);
    return RT_EOK;
}

static const struct rt_i2c_bus_device_ops rp2040_i2c_ops =
{
    .master_xfer = i2c_master_xfer,
    .slave_xfer = i2c_slave_xfer,
    .i2c_bus_control = i2c_bus_control,
};

#if defined (RC_RP2040_USING_I2C0)
#define I2C0_BUS_NAME    "i2c0"
static struct rp2040_i2c_hw_config hw_device0 =
{
};

struct rt_i2c_bus_device device0 =
{
    .ops = &rp2040_i2c_ops,
    .priv =  (void *)&hw_device0,
};
#endif

int rt_hw_i2c_init(void)
{
#if defined(RC_RP2040_USING_I2C0)
    raspi_i2c_configure(&hw_device0);
    rt_i2c_bus_device_register(&device0, I2C0_BUS_NAME);
#endif

    return 0;
}

INIT_DEVICE_EXPORT(rt_hw_i2c_init);
