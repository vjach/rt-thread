/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include "drv_sbus.h"

#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

#define UART_ID uart0
#define BAUD_RATE 100000
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define UART_RX_PIN 29

#define SBUS_BUFFER_SIZE  25

struct sbus_dev {
  struct rt_device parent;
  rt_uint32_t uart_periph;
  rt_uint32_t irqno;
  volatile uint8_t offset;
  volatile uint8_t idle;
  struct rt_semaphore data_available;

  uint8_t first_buffer[SBUS_BUFFER_SIZE];
  uint8_t second_buffer[SBUS_BUFFER_SIZE];
  uint8_t* volatile incomplete_buffer; 
  uint8_t* volatile complete_buffer; 
};

static struct sbus_dev sbus0_dev;

static uint8_t* sbus_get_free_buffer(struct sbus_dev* dev) {
  if (dev->complete_buffer != RT_NULL) {
    if (dev->first_buffer != dev->complete_buffer) {
      return dev->first_buffer;
    } else {
      return dev->second_buffer;
    }
  } else {
    return dev->first_buffer;
  }
}

void sbus_isr(void) {
  rt_interrupt_enter();
  uint32_t mis = uart_get_hw(uart0)->mis;
  if (sbus0_dev.idle == 0) {
      while (uart_is_readable(uart0)) {
        uart_getc(uart0);
      }

    if (mis & (1 << 6)) {
      sbus0_dev.idle = 1;
      sbus0_dev.offset = 0;
      sbus0_dev.incomplete_buffer = sbus_get_free_buffer(&sbus0_dev);
    }
  } else {
    while (uart_is_readable(uart0) &&
        sbus0_dev.offset < SBUS_BUFFER_SIZE) {
      sbus0_dev.incomplete_buffer[(sbus0_dev.offset)++] = uart_getc(uart0);
    }

    // check if there is more that than expected
    if (uart_is_readable(uart0) &&
        sbus0_dev.offset == SBUS_BUFFER_SIZE) {
      // not in sync with the module; thow data away and get ready for next chance
      while (uart_is_readable(uart0)) {
        uart_getc(uart0);
      }

      sbus0_dev.offset = 0;
      sbus0_dev.idle = 0;
    }

    else if (sbus0_dev.offset == SBUS_BUFFER_SIZE) {
      // TODO: swap buffers
      sbus0_dev.complete_buffer = sbus0_dev.incomplete_buffer;
      sbus0_dev.incomplete_buffer = sbus_get_free_buffer(&sbus0_dev);
      sbus0_dev.idle = 0;
      rt_sem_release(&sbus0_dev.data_available);
    }

    if (mis & (1 << 6)) {
      sbus0_dev.idle = 1;
    }
  }

  rt_interrupt_leave();
}

rt_err_t sbus_init(rt_device_t dev) {
  struct sbus_dev* sbus = (struct sbus_dev*)dev->user_data;
  sbus->offset = 0;
  sbus->idle = 0;
  sbus->complete_buffer = RT_NULL;
  sbus->incomplete_buffer = sbus_get_free_buffer(sbus);
  rt_sem_init(&sbus->data_available, "sbus_sem", 0,  RT_IPC_FLAG_FIFO);

  // enable uart idle interrupt
  uart_get_hw(uart0)->imsc |= 1 << 6;
  irq_set_exclusive_handler(UART0_IRQ, sbus_isr);
  irq_set_enabled(UART0_IRQ, true);

  // Now enable the UART to send interrupts - RX only
  uart_set_irq_enables(uart0, true, false);
  return RT_EOK;
}

rt_err_t sbus_open(rt_device_t dev, rt_uint16_t oflag) { return RT_EOK; }

rt_err_t sbus_close(rt_device_t dev) { return RT_EOK; }

static rt_size_t sbus_read(rt_device_t dev, rt_off_t pos, void* buffer,
                           rt_size_t size) {
  struct sbus_dev* sbus = (struct sbus_dev*)dev->user_data;
  rt_err_t err = rt_sem_take(&sbus->data_available, RT_WAITING_FOREVER);
  if (err != RT_EOK) {
    return 0;
  }


  rt_size_t to_copy = size;
  if (to_copy > SBUS_BUFFER_SIZE) {
    to_copy = SBUS_BUFFER_SIZE;
  }

  irq_set_enabled(UART0_IRQ, false);
  if (sbus->complete_buffer != RT_NULL) {
    memcpy(buffer, sbus->complete_buffer, to_copy);
    sbus->complete_buffer = RT_NULL;
  } else {
    to_copy = 0;
  }

  irq_set_enabled(UART0_IRQ, true);
  uart_get_hw(uart0)->imsc |= 1 << 6;
  return to_copy;
}

const static struct rt_device_ops sbus_ops = {
    sbus_init,  /* init */
    sbus_open,  /* open */
    sbus_close, /* close */
    sbus_read,  /* read */
    RT_NULL,    /* write */
    RT_NULL,    /* control */
};

int rt_hw_sbus_init(void) {
  uart_init(UART_ID, BAUD_RATE);

  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  // Actually, we want a different speed
  // The call will return the actual baud rate selected, which will be as close
  // as possible to that requested
  uart_set_baudrate(UART_ID, BAUD_RATE);

  // Set UART flow control CTS/RTS, we don't want these, so turn them off
  uart_set_hw_flow(UART_ID, false, false);

  // Set our data format
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

  uart_set_fifo_enabled(UART_ID, true);

  sbus0_dev.parent.ops = &sbus_ops;
  sbus0_dev.parent.user_data = &sbus0_dev;
  sbus0_dev.parent.type = RT_Device_Class_Miscellaneous;

  return rt_device_register(&sbus0_dev.parent, "sbus0",
                            RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_INT_RX);
}

