/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "drv_pio_uart.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "uart_tx.pio.h"

struct rp2040_pio_uart_dev
{
    struct rt_serial_device parent;
    PIO pio;
    rt_uint32_t state_machine;
    uint8_t gpio_tx;
    uint32_t baud_rate;
};

static struct rp2040_pio_uart_dev uart0_dev = {
  .pio = pio0,
  .state_machine = 0,
  .gpio_tx = 21,
  .baud_rate = 115200,
};

static rt_err_t rp2040_pio_uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    return RT_EOK;
}

static rt_err_t rp2040_pio_uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    return RT_EOK;
}

static int rp2040_pio_uart_putc(struct rt_serial_device *serial, char c)
{
    uart_putc_raw(uart0, c);

    return 1;
}

const static struct rt_uart_ops _uart_ops =
{
    rp2040_pio_uart_configure,
    rp2040_pio_uart_control,
    rp2040_pio_uart_putc,
    RT_NULL,
    RT_NULL,
};

/*
 * UART Initiation
 */
int rt_hw_pio_uart_init(void)
{
  uint32_t offset = pio_add_program(uart0_dev.pio, &uart_tx_program);
  uart_tx_program_init(uart0_dev.pio, uart0_dev.state_machine, offset, uart0_dev.gpio_tx, uart0_dev.baud_rate);
  gpio_set_function(uart0_dev.gpio_tx, GPIO_FUNC_PIO0);
#if 0
    rt_err_t ret = RT_EOK;

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    uart_init(UART_ID, 100000);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    //gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    uart0_dev.parent.ops = &_uart_ops;
    uart0_dev.parent.config = config;

    ret = rt_hw_serial_register(&uart0_dev.parent,
                                "uart0",
                                RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                                &uart0_dev);

    return ret;
#endif
    return 0;
}
