/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2021-01-28     flybreak       first version
 */

#include "board.h"

#include <rthw.h>
#include <rtthread.h>
#include <sys/time.h>

#include "hardware/structs/systick.h"
#include "pico/stdlib.h"

#define PLL_SYS_KHZ (133 * 1000)

static struct rt_mutex printf_mutex;

void isr_systick(void) {
  /* enter interrupt */
#ifndef RT_USING_SMP
  rt_interrupt_enter();
#endif

  rt_tick_increase();

  /* leave interrupt */
#ifndef RT_USING_SMP
  rt_interrupt_leave();
#endif
}

uint32_t systick_config(uint32_t ticks) {
  if ((ticks - 1UL) > M0PLUS_SYST_RVR_RELOAD_BITS) {
    return (1UL); /* Reload value impossible */
  }

  systick_hw->rvr = (uint32_t)(ticks - 1UL); /* set reload register */
  systick_hw->csr =
      M0PLUS_SYST_CSR_CLKSOURCE_BITS | M0PLUS_SYST_CSR_TICKINT_BITS |
      M0PLUS_SYST_CSR_ENABLE_BITS; /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                    /* Function successful */
}

void rt_hw_board_init() {
  set_sys_clock_khz(PLL_SYS_KHZ, true);

#ifdef RT_USING_HEAP
    rt_system_heap_init(HEAP_BEGIN, HEAP_END);
#endif

#ifdef RT_USING_SMP
    extern rt_hw_spinlock_t _cpus_lock;
    rt_hw_spin_lock_init(&_cpus_lock);
#endif

  alarm_pool_init_default();

  // Start and end points of the constructor list,
  // defined by the linker script.
  extern void (*__init_array_start)();
  extern void (*__init_array_end)();

  // Call each function in the list.
  // We have to take the address of the symbols, as __init_array_start *is*
  // the first function pointer, not the address of it.
  for (void (**p)() = &__init_array_start; p < (&__init_array_end - 1); ++p) {
    (*p)();
  }

  /* Configure the SysTick */
  systick_config(clock_get_hz(clk_sys) / RT_TICK_PER_SECOND);

  rt_hw_uart_init();
  rt_hw_sbus_init();
  rt_hw_uart1_init();
  rt_hw_pwm_init();
#if defined(RT_USING_CONSOLE)
  rt_hw_pio_uart_init();
  rt_console_set_device("uart_dbg");
#endif
}

#if 0
void panic(__unused const char *fmt, ...) {
  while (true)
    ;
}
#endif

clock_t clock(void) {
  uint64_t time = time_us_64();
  return time;
}

rt_err_t rt_ktime_boottime_get_us(struct timeval *tv)
{
    RT_ASSERT(tv != RT_NULL);

    uint64_t us = time_us_64();

    tv->tv_sec  = us / (1000UL * 1000);
    tv->tv_usec = us % (1000UL * 1000);

    return RT_EOK;
}

rt_err_t rt_ktime_boottime_get_ns(struct timespec *ts)
{
    RT_ASSERT(ts != RT_NULL);

    uint64_t us = time_us_64();

    ts->tv_sec  = (uint32_t)(us / (1000UL * 1000));
    ts->tv_nsec = (uint32_t)((us % (1000UL * 1000)) * 1000);

    return RT_EOK;
}

rt_size_t __wrap_printf(const char *fmt, ...) {
  rt_size_t ret = 0;
  va_list args;
  va_start(args, fmt);
  ret = rt_kprintf(fmt, args);
  va_end(args);
  return ret;
}
