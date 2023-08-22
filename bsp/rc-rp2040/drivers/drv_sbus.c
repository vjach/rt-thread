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
#define STOP_BITS 2
#define PARITY UART_PARITY_NONE

#define UART_RX_PIN 29

#define SBUS_BUFFER_SIZE  25


#define SBUS_INT_IDLE (1 << 6)
#define SBUS_INT_RX (1 << 4)


enum  {
  SBUS_STATE_SYNC = 0,
  SBUS_STATE_IN_PROGRESS = 1,
};

struct sbus_dev {
  struct rt_device parent;
  rt_uint32_t uart_periph;
  rt_uint32_t irqno;
  volatile uint32_t offset;
  volatile uint32_t state;
  struct rt_semaphore data_available;

  uint8_t first_buffer[SBUS_BUFFER_SIZE];
  uint8_t second_buffer[SBUS_BUFFER_SIZE];
  volatile uint8_t* volatile incomplete_buffer; 
  volatile uint8_t* volatile complete_buffer; 
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
  if (rt_hw_cpu_id() == 1) {
    asm volatile ("BKPT");
  }
  rt_interrupt_enter();
  uint32_t mis = uart_get_hw(uart0)->mis;
  switch (sbus0_dev.state) {
    case SBUS_STATE_SYNC:
      /* NOTE: this is self-limited to FIFO size */
      while (uart_is_readable(uart0)) {
        (void)uart_getc(uart0);
      }

      sbus0_dev.state = SBUS_STATE_IN_PROGRESS;
      sbus0_dev.offset = 0;
      uart_get_hw(uart0)->imsc |= SBUS_INT_RX;
      break;

    case SBUS_STATE_IN_PROGRESS: 
      if (mis & (SBUS_INT_IDLE | SBUS_INT_RX)) {
        while (uart_is_readable(uart0) && sbus0_dev.offset < SBUS_BUFFER_SIZE) {
          sbus0_dev.incomplete_buffer[sbus0_dev.offset] = uart_getc(uart0);
          sbus0_dev.offset++;
        }

        bool still_readable = uart_is_readable(uart0);

        if (still_readable && sbus0_dev.offset == SBUS_BUFFER_SIZE) {
          uint32_t cnt = 0;
          /* error, restart */
          sbus0_dev.state = SBUS_STATE_SYNC;
          sbus0_dev.offset = 0;
          uart_get_hw(uart0)->imsc &= ~SBUS_INT_RX;
        } else if (!still_readable && sbus0_dev.offset == SBUS_BUFFER_SIZE) {
          /* frame completed */

          if (sbus0_dev.data_available.value <= 1) {
            sbus0_dev.complete_buffer = sbus0_dev.incomplete_buffer;
            sbus0_dev.incomplete_buffer = sbus_get_free_buffer(&sbus0_dev);
            sbus0_dev.offset = 0;
            rt_sem_release(&sbus0_dev.data_available);
          } else {
            /* data is consumed too slow, restart on the same buffer */
            sbus0_dev.offset = 0;
          }
        } else if (!still_readable && sbus0_dev.offset < SBUS_BUFFER_SIZE) {
           //keep going
        }
      }
      break;
  }

  rt_interrupt_leave();
}

rt_err_t sbus_init(rt_device_t dev) {
  struct sbus_dev* sbus = (struct sbus_dev*)dev->user_data;
  sbus->offset = 0;
  sbus->state = SBUS_STATE_SYNC;
  sbus->complete_buffer = RT_NULL;
  sbus->incomplete_buffer = sbus_get_free_buffer(sbus);
  rt_sem_init(&sbus->data_available, "sbus_sem", 0,  RT_IPC_FLAG_FIFO);

  // enable uart idle interrupt
  uart_get_hw(uart0)->imsc |= SBUS_INT_IDLE;
  irq_set_exclusive_handler(UART0_IRQ, sbus_isr);
  irq_set_enabled(UART0_IRQ, true);

  return RT_EOK;
}

rt_err_t sbus_open(rt_device_t dev, rt_uint16_t oflag) { return RT_EOK; }

rt_err_t sbus_close(rt_device_t dev) { return RT_EOK; }

static rt_ssize_t sbus_read(rt_device_t dev, rt_off_t pos, void* buffer,
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

  if (sbus->complete_buffer != RT_NULL) {
    memcpy(buffer, sbus->complete_buffer, to_copy);
//    sbus->complete_buffer = RT_NULL;
  } else {
    to_copy = 0;
  }

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

  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  uart_set_baudrate(UART_ID, BAUD_RATE);
  uart_set_hw_flow(UART_ID, false, false);
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
  uart_set_fifo_enabled(UART_ID, true);

  sbus0_dev.parent.ops = &sbus_ops;
  sbus0_dev.parent.user_data = &sbus0_dev;
  sbus0_dev.parent.type = RT_Device_Class_Miscellaneous;

  return rt_device_register(&sbus0_dev.parent, "sbus0",
                            RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_INT_RX);
}

