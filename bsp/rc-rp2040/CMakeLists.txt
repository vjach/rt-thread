CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_PROCESSOR cortex-m0)
#SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(externals.cmake)

SET(CMAKE_C_FLAGS " -march=armv6-m -mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g")
SET(CMAKE_ASM_FLAGS " -c -march=armv6-m -mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb -ILibraries/pico-sdk/src/common/pico_stdlib/include -ILibraries/pico-sdk/src/rp2_common/hardware_gpio/include -ILibraries/pico-sdk/src/common/pico_base/include -ILibraries/generated/pico_base -ILibraries/pico-sdk/src/boards/include -ILibraries/pico-sdk/src/rp2_common/pico_platform/include -ILibraries/pico-sdk/src/rp2040/hardware_regs/include -ILibraries/pico-sdk/src/rp2_common/hardware_base/include -ILibraries/pico-sdk/src/rp2040/hardware_structs/include -ILibraries/pico-sdk/src/rp2_common/hardware_claim/include -ILibraries/pico-sdk/src/rp2_common/hardware_sync/include -ILibraries/pico-sdk/src/rp2_common/hardware_uart/include -ILibraries/pico-sdk/src/rp2_common/hardware_divider/include -ILibraries/pico-sdk/src/common/pico_time/include -ILibraries/pico-sdk/src/rp2_common/hardware_timer/include -ILibraries/pico-sdk/src/common/pico_sync/include -ILibraries/pico-sdk/src/common/pico_util/include -ILibraries/pico-sdk/src/rp2_common/pico_runtime/include -ILibraries/pico-sdk/src/rp2_common/hardware_clocks/include -ILibraries/pico-sdk/src/rp2_common/hardware_resets/include -ILibraries/pico-sdk/src/rp2_common/hardware_watchdog/include -ILibraries/pico-sdk/src/rp2_common/hardware_xosc/include -ILibraries/pico-sdk/src/rp2_common/hardware_pll/include -ILibraries/pico-sdk/src/rp2_common/hardware_vreg/include -ILibraries/pico-sdk/src/rp2_common/hardware_irq/include -ILibraries/pico-sdk/src/rp2_common/pico_printf/include -ILibraries/pico-sdk/src/rp2_common/pico_bootrom/include -ILibraries/pico-sdk/src/common/pico_bit_ops/include -ILibraries/pico-sdk/src/common/pico_divider/include -ILibraries/pico-sdk/src/rp2_common/pico_double/include -ILibraries/pico-sdk/src/rp2_common/pico_int64_ops/include -ILibraries/pico-sdk/src/rp2_common/pico_float/include -ILibraries/pico-sdk/src/common/pico_binary_info/include -ILibraries/pico-sdk/src/rp2_common/pico_stdio/include -ILibraries/pico-sdk/src/rp2_common/pico_stdio_uart/include -gdwarf-2")
SET(CMAKE_C_COMPILER_WORKS TRUE)

SET(CMAKE_CXX_FLAGS " -march=armv6-m -mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g")
SET(CMAKE_CXX_COMPILER_WORKS TRUE)


#SET(CMAKE_EXE_LINKER_FLAGS " -march=armv6-m -mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=rt-thread.map,-cref,-u,Reset_Handler -T ${CMAKE_CURRENT_SOURCE_DIR}/link.ld -Wl,--build-id=none --specs=nosys.specs -Wl,--wrap=sprintf -Wl,--wrap=snprintf -Wl,--wrap=vsnprintf -Wl,--wrap=__clzsi2 -Wl,--wrap=__clzdi2 -Wl,--wrap=__ctzsi2 -Wl,--wrap=__ctzdi2 -Wl,--wrap=__popcountsi2 -Wl,--wrap=__popcountdi2 -Wl,--wrap=__clz -Wl,--wrap=__clzl -Wl,--wrap=__clzll -Wl,--wrap=__aeabi_idiv -Wl,--wrap=__aeabi_idivmod -Wl,--wrap=__aeabi_ldivmod -Wl,--wrap=__aeabi_uidiv -Wl,--wrap=__aeabi_uidivmod -Wl,--wrap=__aeabi_uldivmod -Wl,--wrap=__aeabi_dadd -Wl,--wrap=__aeabi_ddiv -Wl,--wrap=__aeabi_dmul -Wl,--wrap=__aeabi_drsub -Wl,--wrap=__aeabi_dsub -Wl,--wrap=__aeabi_cdcmpeq -Wl,--wrap=__aeabi_cdrcmple -Wl,--wrap=__aeabi_cdcmple -Wl,--wrap=__aeabi_dcmpeq -Wl,--wrap=__aeabi_dcmplt -Wl,--wrap=__aeabi_dcmple -Wl,--wrap=__aeabi_dcmpge -Wl,--wrap=__aeabi_dcmpgt -Wl,--wrap=__aeabi_dcmpun -Wl,--wrap=__aeabi_i2d -Wl,--wrap=__aeabi_l2d -Wl,--wrap=__aeabi_ui2d -Wl,--wrap=__aeabi_ul2d -Wl,--wrap=__aeabi_d2iz -Wl,--wrap=__aeabi_d2lz -Wl,--wrap=__aeabi_d2uiz -Wl,--wrap=__aeabi_d2ulz -Wl,--wrap=__aeabi_d2f -Wl,--wrap=sqrt -Wl,--wrap=cos -Wl,--wrap=sin -Wl,--wrap=tan -Wl,--wrap=atan2 -Wl,--wrap=exp -Wl,--wrap=log -Wl,--wrap=ldexp -Wl,--wrap=copysign -Wl,--wrap=trunc -Wl,--wrap=floor -Wl,--wrap=ceil -Wl,--wrap=round -Wl,--wrap=sincos -Wl,--wrap=asin -Wl,--wrap=acos -Wl,--wrap=atan -Wl,--wrap=sinh -Wl,--wrap=cosh -Wl,--wrap=tanh -Wl,--wrap=asinh -Wl,--wrap=acosh -Wl,--wrap=atanh -Wl,--wrap=exp2 -Wl,--wrap=log2 -Wl,--wrap=exp10 -Wl,--wrap=log10 -Wl,--wrap=pow -Wl,--wrap=powint -Wl,--wrap=hypot -Wl,--wrap=cbrt -Wl,--wrap=fmod -Wl,--wrap=drem -Wl,--wrap=remainder -Wl,--wrap=remquo -Wl,--wrap=expm1 -Wl,--wrap=log1p -Wl,--wrap=fma -Wl,--wrap=__aeabi_lmul -Wl,--wrap=__aeabi_fadd -Wl,--wrap=__aeabi_fdiv -Wl,--wrap=__aeabi_fmul -Wl,--wrap=__aeabi_frsub -Wl,--wrap=__aeabi_fsub -Wl,--wrap=__aeabi_cfcmpeq -Wl,--wrap=__aeabi_cfrcmple -Wl,--wrap=__aeabi_cfcmple -Wl,--wrap=__aeabi_fcmpeq -Wl,--wrap=__aeabi_fcmplt -Wl,--wrap=__aeabi_fcmple -Wl,--wrap=__aeabi_fcmpge -Wl,--wrap=__aeabi_fcmpgt -Wl,--wrap=__aeabi_fcmpun -Wl,--wrap=__aeabi_i2f -Wl,--wrap=__aeabi_l2f -Wl,--wrap=__aeabi_ui2f -Wl,--wrap=__aeabi_ul2f -Wl,--wrap=__aeabi_f2iz -Wl,--wrap=__aeabi_f2lz -Wl,--wrap=__aeabi_f2uiz -Wl,--wrap=__aeabi_f2ulz -Wl,--wrap=__aeabi_f2d -Wl,--wrap=sqrtf -Wl,--wrap=cosf -Wl,--wrap=sinf -Wl,--wrap=tanf -Wl,--wrap=atan2f -Wl,--wrap=expf -Wl,--wrap=logf -Wl,--wrap=ldexpf -Wl,--wrap=copysignf -Wl,--wrap=truncf -Wl,--wrap=floorf -Wl,--wrap=ceilf -Wl,--wrap=roundf -Wl,--wrap=sincosf -Wl,--wrap=asinf -Wl,--wrap=acosf -Wl,--wrap=atanf -Wl,--wrap=sinhf -Wl,--wrap=coshf -Wl,--wrap=tanhf -Wl,--wrap=asinhf -Wl,--wrap=acoshf -Wl,--wrap=atanhf -Wl,--wrap=exp2f -Wl,--wrap=log2f -Wl,--wrap=exp10f -Wl,--wrap=log10f -Wl,--wrap=powf -Wl,--wrap=powintf -Wl,--wrap=hypotf -Wl,--wrap=cbrtf -Wl,--wrap=fmodf -Wl,--wrap=dremf -Wl,--wrap=remainderf -Wl,--wrap=remquof -Wl,--wrap=expm1f -Wl,--wrap=log1pf -Wl,--wrap=fmaf -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free -Wl,--wrap=memcpy -Wl,--wrap=memset -Wl,--wrap=__aeabi_memcpy -Wl,--wrap=__aeabi_memset -Wl,--wrap=__aeabi_memcpy4 -Wl,--wrap=__aeabi_memset4 -Wl,--wrap=__aeabi_memcpy8 -Wl,--wrap=__aeabi_memset8 -Wl,--gc-sections -Wl,--wrap=printf -Wl,--wrap=vprintf -Wl,--wrap=puts -Wl,--wrap=putchar")

SET(CMAKE_CXX_STANDARD 14)
PROJECT(rtthread C CXX ASM)

add_library(rtthread_headers INTERFACE)

set(PICO_INCLUDES
	libraries/pico-sdk/src/common/pico_stdlib/include
	libraries/pico-sdk/src/rp2_common/hardware_gpio/include
	libraries/pico-sdk/src/common/pico_base/include
	libraries/pico-sdk/src/boards/include
	libraries/pico-sdk/src/rp2_common/pico_platform/include
	libraries/pico-sdk/src/rp2040/hardware_regs/include
	libraries/pico-sdk/src/rp2_common/hardware_base/include
	libraries/pico-sdk/src/rp2040/hardware_structs/include
	libraries/pico-sdk/src/rp2_common/hardware_claim/include
	libraries/pico-sdk/src/rp2_common/hardware_sync/include
	libraries/pico-sdk/src/rp2_common/hardware_uart/include
	libraries/pico-sdk/src/rp2_common/hardware_adc/include
	libraries/pico-sdk/src/rp2_common/hardware_dma/include
	libraries/pico-sdk/src/rp2_common/hardware_spi/include
	libraries/pico-sdk/src/rp2_common/hardware_i2c/include
	libraries/pico-sdk/src/rp2_common/hardware_pwm/include
	libraries/pico-sdk/src/rp2_common/hardware_divider/include
	libraries/pico-sdk/src/common/pico_time/include
	libraries/pico-sdk/src/rp2_common/hardware_timer/include
	libraries/pico-sdk/src/common/pico_sync/include
	libraries/pico-sdk/src/common/pico_util/include
	libraries/pico-sdk/src/rp2_common/pico_runtime/include
	libraries/pico-sdk/src/rp2_common/hardware_clocks/include
	libraries/pico-sdk/src/rp2_common/hardware_resets/include
	libraries/pico-sdk/src/rp2_common/hardware_watchdog/include
	libraries/pico-sdk/src/rp2_common/hardware_xosc/include
	libraries/pico-sdk/src/rp2_common/hardware_pll/include
	libraries/pico-sdk/src/rp2_common/hardware_vreg/include
	libraries/pico-sdk/src/rp2_common/hardware_irq/include
	libraries/pico-sdk/src/rp2_common/pico_printf/include
	libraries/pico-sdk/src/rp2_common/pico_bootrom/include
	libraries/pico-sdk/src/common/pico_bit_ops/include
	libraries/pico-sdk/src/common/pico_divider/include
	libraries/pico-sdk/src/rp2_common/pico_double/include
	libraries/pico-sdk/src/rp2_common/pico_int64_ops/include
	libraries/pico-sdk/src/rp2_common/pico_float/include
	libraries/pico-sdk/src/common/pico_binary_info/include
	libraries/pico-sdk/src/rp2_common/pico_stdio/include
	libraries/pico-sdk/src/rp2_common/pico_stdio_uart/include
	libraries/generated/pico_base
  libraries/pico-sdk/src/rp2_common/boot_stage2/asminclude
  ${CMAKE_CURRENT_SOURCE_DIR}/libraries
  )

target_include_directories(rtthread_headers INTERFACE
	../../components/libc/compilers/common/include
	../../components/libc/compilers/newlib
	../../libcpu/arm/common
	../../libcpu/arm/cortex-m0
	../../components/drivers/include
	drivers
	board
	board/ports/lcd
	../../components/finsh
	.
	../../include
	libraries/pico-sdk/src/common/pico_stdlib/include
	libraries/pico-sdk/src/rp2_common/hardware_gpio/include
	libraries/pico-sdk/src/common/pico_base/include
	libraries/pico-sdk/src/boards/include
	libraries/pico-sdk/src/rp2_common/pico_platform/include
	libraries/pico-sdk/src/rp2040/hardware_regs/include
	libraries/pico-sdk/src/rp2_common/hardware_base/include
	libraries/pico-sdk/src/rp2040/hardware_structs/include
	libraries/pico-sdk/src/rp2_common/hardware_claim/include
	libraries/pico-sdk/src/rp2_common/hardware_sync/include
	libraries/pico-sdk/src/rp2_common/hardware_adc/include
	libraries/pico-sdk/src/rp2_common/hardware_uart/include
	libraries/pico-sdk/src/rp2_common/hardware_dma/include
	libraries/pico-sdk/src/rp2_common/hardware_spi/include
	libraries/pico-sdk/src/rp2_common/hardware_i2c/include
	libraries/pico-sdk/src/rp2_common/hardware_pwm/include
	libraries/pico-sdk/src/rp2_common/hardware_divider/include
	libraries/pico-sdk/src/common/pico_time/include
	libraries/pico-sdk/src/rp2_common/hardware_timer/include
	libraries/pico-sdk/src/common/pico_sync/include
	libraries/pico-sdk/src/common/pico_util/include
	libraries/pico-sdk/src/rp2_common/pico_runtime/include
	libraries/pico-sdk/src/rp2_common/hardware_clocks/include
	libraries/pico-sdk/src/rp2_common/hardware_resets/include
	libraries/pico-sdk/src/rp2_common/hardware_watchdog/include
	libraries/pico-sdk/src/rp2_common/hardware_xosc/include
	libraries/pico-sdk/src/rp2_common/hardware_pll/include
	libraries/pico-sdk/src/rp2_common/hardware_vreg/include
	libraries/pico-sdk/src/rp2_common/hardware_irq/include
	libraries/pico-sdk/src/rp2_common/pico_printf/include
	libraries/pico-sdk/src/rp2_common/pico_bootrom/include
	libraries/pico-sdk/src/common/pico_bit_ops/include
	libraries/pico-sdk/src/common/pico_divider/include
	libraries/pico-sdk/src/rp2_common/pico_double/include
	libraries/pico-sdk/src/rp2_common/pico_int64_ops/include
	libraries/pico-sdk/src/rp2_common/pico_float/include
	libraries/pico-sdk/src/common/pico_binary_info/include
	libraries/pico-sdk/src/rp2_common/pico_stdio/include
	libraries/pico-sdk/src/rp2_common/pico_stdio_uart/include
	libraries/generated/pico_base
	../../components/libc/posix/ipc
	../../components/libc/posix/io/stdio
	../../components/libc/posix/io/poll
  ${CMAKE_CURRENT_SOURCE_DIR}/libraries
  )

target_compile_definitions(rtthread_headers INTERFACE
	-DPICO_BIT_OPS_PICO=1
	-DPICO_BUILD=1
	-DPICO_COPY_TO_RAM=0
	-DPICO_CXX_ENABLE_EXCEPTIONS=0
	-DPICO_DIVIDER_HARDWARE=1
	-DPICO_DOUBLE_PICO=1
	-DPICO_FLOAT_PICO=1
	-DPICO_INT64_OPS_PICO=1
	-DPICO_MEM_OPS_PICO=1
	-DPICO_NO_BINARY_INFO
	-DPICO_NO_FLASH=0
	-DPICO_NO_HARDWARE=0
	-DPICO_NO_PROGRAM_INFO
	-DPICO_ON_DEVICE=1
	-DPICO_PRINTF_PICO=1
	-DPICO_STDIO_UART=0
	-DPICO_USE_BLOCKED_RAM=0
	-DRT_USING_LIBC
	-DRT_USING_NEWLIBC
	-D_POSIX_C_SOURCE=1
	-D__RTTHREAD__
)


#add_custom_command(OUTPUT boot2
#  COMMAND ${CMAKE_CXX_COMPILER} -nostartfiles -I ${CMAKE_CURRENT_SOURCE_DIR}/libraries/pico-sdk/src/rp2_common/boot_stage2/asminclude -I ${CMAKE_CURRENT_SOURCE_DIR}/libraries/pico-sdk/src/common/pico_base/include -I ${CMAKE_CURRENT_SOURCE_DIR}/libraries/pico-sdk/src/boards/include -I ${CMAKE_CURRENT_SOURCE_DIR}/libraries/generated/pico_base -I ${CMAKE_CURRENT_SOURCE_DIR}/libraries/pico-sdk/src/rp2_common/pico_platform/include -Wl,--script=${CMAKE_CURRENT_SOURCE_DIR}/libraries/pico-sdk/src/rp2_common/boot_stage2/boot_stage2.ld ${CMAKE_CURRENT_SOURCE_DIR}/libraries/pico-sdk/src/rp2_common/boot_stage2/boot2_w25q080.S -o boot2
#  DEPENDS libraries/pico-sdk/src/rp2_common/boot_stage2/boot2_w25q080.S
#  )
enable_language(ASM)
set_property(SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/libraries/pico-sdk/src/rp2_common/boot_stage2/boot2_w25q080.S PROPERTY LANGUAGE ASM)
add_executable(boot2 ${CMAKE_CURRENT_SOURCE_DIR}/libraries/pico-sdk/src/rp2_common/boot_stage2/boot2_w25q080.S)
target_include_directories(boot2 PRIVATE ${PICO_INCLUDES} )
target_link_options(boot2 PRIVATE -nostartfiles -Wl,--script=${CMAKE_CURRENT_SOURCE_DIR}/libraries/pico-sdk/src/rp2_common/boot_stage2/boot_stage2.ld)

add_custom_command(OUTPUT boot2.bin
  COMMAND ${CMAKE_OBJCOPY} -Obinary boot2 boot2.bin
  DEPENDS boot2
  )

add_custom_command(OUTPUT boot2.S
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/libraries/pico-sdk/src/rp2_common/boot_stage2/pad_checksum -s 0xffffffff boot2.bin boot2.S
  DEPENDS boot2.bin
  )


SET(PROJECT_SOURCES
	../../components/libc/compilers/common/cctype.c
	../../components/libc/compilers/common/cstdio.c
	../../components/libc/compilers/common/cstdlib.c
	../../components/libc/compilers/common/cstring.c
	../../components/libc/compilers/common/ctime.c
	../../components/libc/compilers/common/cwchar.c
	../../components/libc/compilers/newlib/syscalls.c
	../../libcpu/arm/common/div0.c
	../../libcpu/arm/common/showmem.c
	../../libcpu/arm/cortex-m0/context_gcc.S
	../../libcpu/arm/cortex-m0/cpuport.c
	../../components/drivers/ipc/completion.c
	../../components/drivers/ipc/dataqueue.c
	../../components/drivers/ipc/pipe.c
	../../components/drivers/ipc/ringblk_buf.c
	../../components/drivers/ipc/ringbuffer.c
	../../components/drivers/ipc/waitqueue.c
	../../components/drivers/ipc/workqueue.c
	../../components/drivers/misc/pin.c
	../../components/drivers/misc/adc.c
	../../components/drivers/serial/serial.c
	../../components/drivers/i2c/i2c_dev.c
	../../components/drivers/i2c/i2c_core.c
	../../components/drivers/spi/spi_dev.c
	../../components/drivers/spi/spi_core.c
	board/board.c
	drivers/drv_gpio.c
	drivers/drv_adc.c
	drivers/drv_uart.c
	drivers/drv_i2c.c
	drivers/drv_spi.c
	drivers/drv_sbus.c
	../../components/finsh/cmd.c
	../../components/finsh/msh.c
	../../components/finsh/shell.c
	../../components/finsh/msh_parse.c
	../../src/clock.c
	../../src/components.c
	../../src/device.c
	../../src/idle.c
	../../src/ipc.c
	../../src/irq.c
	../../src/kservice.c
	../../src/mem.c
	../../src/mempool.c
	../../src/object.c
	../../src/scheduler.c
	../../src/thread.c
	../../src/timer.c
	libraries/pico-sdk/src/common/pico_util/datetime.c
	libraries/pico-sdk/src/rp2_common/pico_float/float_math.c
	libraries/pico-sdk/src/rp2_common/hardware_gpio/gpio.c
	libraries/pico-sdk/src/rp2_common/hardware_dma/dma.c
	libraries/pico-sdk/src/rp2_common/hardware_adc/adc.c
	libraries/pico-sdk/src/common/pico_time/time.c
	libraries/pico-sdk/src/rp2_common/pico_bit_ops/bit_ops_aeabi.S
	libraries/pico-sdk/src/rp2_common/pico_stdlib/stdlib.c
	libraries/pico-sdk/src/rp2_common/hardware_vreg/vreg.c
	libraries/pico-sdk/src/rp2_common/pico_float/float_aeabi.S
	libraries/pico-sdk/src/common/pico_sync/lock_core.c
	libraries/pico-sdk/src/rp2_common/pico_divider/divider.S
	libraries/pico-sdk/src/rp2_common/pico_standard_link/binary_info.c
	libraries/pico-sdk/src/rp2_common/hardware_spi/spi.c
	libraries/pico-sdk/src/rp2_common/hardware_watchdog/watchdog.c
	libraries/pico-sdk/src/rp2_common/pico_stdio/stdio.c
	libraries/pico-sdk/src/common/pico_util/pheap.c
	libraries/pico-sdk/src/rp2_common/hardware_divider/divider.S
	libraries/pico-sdk/src/rp2_common/hardware_i2c/i2c.c
	libraries/pico-sdk/src/rp2_common/hardware_pll/pll.c
	libraries/pico-sdk/src/rp2_common/pico_platform/platform.c
	libraries/pico-sdk/src/common/pico_sync/sem.c
	libraries/pico-sdk/src/rp2_common/pico_bootrom/bootrom.c
	libraries/pico-sdk/src/rp2_common/hardware_irq/irq_handler_chain.S
	libraries/pico-sdk/src/rp2_common/hardware_clocks/clocks.c
	libraries/pico-sdk/src/common/pico_time/timeout_helper.c
	libraries/pico-sdk/src/common/pico_util/queue.c
	libraries/pico-sdk/src/common/pico_sync/critical_section.c
	libraries/pico-sdk/src/rp2_common/pico_int64_ops/pico_int64_ops_aeabi.S
	libraries/pico-sdk/src/rp2_common/pico_stdio_uart/stdio_uart.c
	libraries/pico-sdk/src/rp2_common/pico_standard_link/new_delete.cpp
	libraries/pico-sdk/src/rp2_common/pico_double/double_aeabi.S
	libraries/pico-sdk/src/rp2_common/hardware_xosc/xosc.c
	libraries/pico-sdk/src/rp2_common/pico_double/double_init_rom.c
	libraries/pico-sdk/src/rp2_common/pico_double/double_math.c
	libraries/pico-sdk/src/rp2_common/pico_runtime/runtime.c
	libraries/pico-sdk/src/rp2_common/hardware_claim/claim.c
	libraries/pico-sdk/src/rp2_common/hardware_sync/sync.c
	libraries/pico-sdk/src/rp2_common/pico_mem_ops/mem_ops_aeabi.S
	libraries/pico-sdk/src/rp2_common/pico_double/double_v1_rom_shim.S
	libraries/pico-sdk/src/rp2_common/hardware_timer/timer.c
	libraries/pico-sdk/src/common/pico_sync/mutex.c
	libraries/pico-sdk/src/rp2_common/pico_float/float_init_rom.c
	libraries/pico-sdk/src/rp2_common/pico_float/float_v1_rom_shim.S
	libraries/pico-sdk/src/rp2_common/hardware_irq/irq.c
	libraries/pico-sdk/src/rp2_common/pico_printf/printf.c
	libraries/pico-sdk/src/rp2_common/hardware_uart/uart.c
  libraries/startup.S
  ${CMAKE_CURRENT_BINARY_DIR}/boot2.S
)

LINK_DIRECTORIES(
)

LINK_LIBRARIES(
	c
	m
)
add_executable(rtthread ${PROJECT_SOURCES})
target_link_options(rtthread PRIVATE -march=armv6-m -mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=rt-thread.map,-cref,-u,Reset_Handler -T ${CMAKE_CURRENT_SOURCE_DIR}/link.ld -Wl,--build-id=none -Wl,--wrap=sprintf -Wl,--wrap=snprintf -Wl,--wrap=vsnprintf -Wl,--wrap=__clzsi2 -Wl,--wrap=__clzdi2 -Wl,--wrap=__ctzsi2 -Wl,--wrap=__ctzdi2 -Wl,--wrap=__popcountsi2 -Wl,--wrap=__popcountdi2 -Wl,--wrap=__clz -Wl,--wrap=__clzl -Wl,--wrap=__clzll -Wl,--wrap=__aeabi_idiv -Wl,--wrap=__aeabi_idivmod -Wl,--wrap=__aeabi_ldivmod -Wl,--wrap=__aeabi_uidiv -Wl,--wrap=__aeabi_uidivmod -Wl,--wrap=__aeabi_uldivmod -Wl,--wrap=__aeabi_dadd -Wl,--wrap=__aeabi_ddiv -Wl,--wrap=__aeabi_dmul -Wl,--wrap=__aeabi_drsub -Wl,--wrap=__aeabi_dsub -Wl,--wrap=__aeabi_cdcmpeq -Wl,--wrap=__aeabi_cdrcmple -Wl,--wrap=__aeabi_cdcmple -Wl,--wrap=__aeabi_dcmpeq -Wl,--wrap=__aeabi_dcmplt -Wl,--wrap=__aeabi_dcmple -Wl,--wrap=__aeabi_dcmpge -Wl,--wrap=__aeabi_dcmpgt -Wl,--wrap=__aeabi_dcmpun -Wl,--wrap=__aeabi_i2d -Wl,--wrap=__aeabi_l2d -Wl,--wrap=__aeabi_ui2d -Wl,--wrap=__aeabi_ul2d -Wl,--wrap=__aeabi_d2iz -Wl,--wrap=__aeabi_d2lz -Wl,--wrap=__aeabi_d2uiz -Wl,--wrap=__aeabi_d2ulz -Wl,--wrap=__aeabi_d2f -Wl,--wrap=sqrt -Wl,--wrap=cos -Wl,--wrap=sin -Wl,--wrap=tan -Wl,--wrap=atan2 -Wl,--wrap=exp -Wl,--wrap=log -Wl,--wrap=ldexp -Wl,--wrap=copysign -Wl,--wrap=trunc -Wl,--wrap=floor -Wl,--wrap=ceil -Wl,--wrap=round -Wl,--wrap=sincos -Wl,--wrap=asin -Wl,--wrap=acos -Wl,--wrap=atan -Wl,--wrap=sinh -Wl,--wrap=cosh -Wl,--wrap=tanh -Wl,--wrap=asinh -Wl,--wrap=acosh -Wl,--wrap=atanh -Wl,--wrap=exp2 -Wl,--wrap=log2 -Wl,--wrap=exp10 -Wl,--wrap=log10 -Wl,--wrap=pow -Wl,--wrap=powint -Wl,--wrap=hypot -Wl,--wrap=cbrt -Wl,--wrap=fmod -Wl,--wrap=drem -Wl,--wrap=remainder -Wl,--wrap=remquo -Wl,--wrap=expm1 -Wl,--wrap=log1p -Wl,--wrap=fma -Wl,--wrap=__aeabi_lmul -Wl,--wrap=__aeabi_fadd -Wl,--wrap=__aeabi_fdiv -Wl,--wrap=__aeabi_fmul -Wl,--wrap=__aeabi_frsub -Wl,--wrap=__aeabi_fsub -Wl,--wrap=__aeabi_cfcmpeq -Wl,--wrap=__aeabi_cfrcmple -Wl,--wrap=__aeabi_cfcmple -Wl,--wrap=__aeabi_fcmpeq -Wl,--wrap=__aeabi_fcmplt -Wl,--wrap=__aeabi_fcmple -Wl,--wrap=__aeabi_fcmpge -Wl,--wrap=__aeabi_fcmpgt -Wl,--wrap=__aeabi_fcmpun -Wl,--wrap=__aeabi_i2f -Wl,--wrap=__aeabi_l2f -Wl,--wrap=__aeabi_ui2f -Wl,--wrap=__aeabi_ul2f -Wl,--wrap=__aeabi_f2iz -Wl,--wrap=__aeabi_f2lz -Wl,--wrap=__aeabi_f2uiz -Wl,--wrap=__aeabi_f2ulz -Wl,--wrap=__aeabi_f2d -Wl,--wrap=sqrtf -Wl,--wrap=cosf -Wl,--wrap=sinf -Wl,--wrap=tanf -Wl,--wrap=atan2f -Wl,--wrap=expf -Wl,--wrap=logf -Wl,--wrap=ldexpf -Wl,--wrap=copysignf -Wl,--wrap=truncf -Wl,--wrap=floorf -Wl,--wrap=ceilf -Wl,--wrap=roundf -Wl,--wrap=sincosf -Wl,--wrap=asinf -Wl,--wrap=acosf -Wl,--wrap=atanf -Wl,--wrap=sinhf -Wl,--wrap=coshf -Wl,--wrap=tanhf -Wl,--wrap=asinhf -Wl,--wrap=acoshf -Wl,--wrap=atanhf -Wl,--wrap=exp2f -Wl,--wrap=log2f -Wl,--wrap=exp10f -Wl,--wrap=log10f -Wl,--wrap=powf -Wl,--wrap=powintf -Wl,--wrap=hypotf -Wl,--wrap=cbrtf -Wl,--wrap=fmodf -Wl,--wrap=dremf -Wl,--wrap=remainderf -Wl,--wrap=remquof -Wl,--wrap=expm1f -Wl,--wrap=log1pf -Wl,--wrap=fmaf -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=free -Wl,--wrap=memcpy -Wl,--wrap=memset -Wl,--wrap=__aeabi_memcpy -Wl,--wrap=__aeabi_memset -Wl,--wrap=__aeabi_memcpy4 -Wl,--wrap=__aeabi_memset4 -Wl,--wrap=__aeabi_memcpy8 -Wl,--wrap=__aeabi_memset8 -Wl,--gc-sections -Wl,--wrap=printf -Wl,--wrap=vprintf -Wl,--wrap=puts -Wl,--wrap=putchar)
target_link_libraries(rtthread rtthread_headers)
