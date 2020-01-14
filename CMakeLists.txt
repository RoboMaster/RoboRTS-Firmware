cmake_minimum_required (VERSION 2.8.12)

# Project specific settings
set(application_name "standard")
set(BMI088_DIR ${CMAKE_SOURCE_DIR}/components/bmi088)

project (${application_name} C CXX ASM)

set(INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/bsp/boards
    ${CMAKE_SOURCE_DIR}/bsp/cubemx/Core/Inc
    ${CMAKE_SOURCE_DIR}/bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_SOURCE_DIR}/bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_SOURCE_DIR}/bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${CMAKE_SOURCE_DIR}/bsp/cubemx/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/bsp/cubemx/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_SOURCE_DIR}/bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_SOURCE_DIR}/bsp/cubemx/USB_DEVICE/App
    ${CMAKE_SOURCE_DIR}/bsp/cubemx/USB_DEVICE/Target
    ${CMAKE_SOURCE_DIR}/components/algorithm
    ${CMAKE_SOURCE_DIR}/components/bmi088/Include
    ${CMAKE_SOURCE_DIR}/components/bmi088
    ${CMAKE_SOURCE_DIR}/components/cli
    ${CMAKE_SOURCE_DIR}/components/easyflash/inc
    ${CMAKE_SOURCE_DIR}/components/event_mgr
    ${CMAKE_SOURCE_DIR}/components/log
    ${CMAKE_SOURCE_DIR}/components/devices
    ${CMAKE_SOURCE_DIR}/components/modules
    ${CMAKE_SOURCE_DIR}/components/protocol
    ${CMAKE_SOURCE_DIR}/components/referee
    ${CMAKE_SOURCE_DIR}/components/soft_timer
    ${CMAKE_SOURCE_DIR}/components/support
    ${CMAKE_SOURCE_DIR}/components/systemview
    ${CMAKE_SOURCE_DIR}/application
    ${CMAKE_SOURCE_DIR}/application/chassis
    ${CMAKE_SOURCE_DIR}/application/gimbal
)

set(sdk_sources
    bsp/boards/board.c
    bsp/boards/drv_bmi088.c
    bsp/boards/drv_can.c
    bsp/boards/drv_dr16.c
    bsp/boards/drv_flash.c
    bsp/boards/drv_io.c
    bsp/boards/drv_uart.c
    bsp/boards/startup_gcc.s
    bsp/boards/stm32f4xx_it.c
    bsp/boards/usbd_cdc_if.c
    bsp/cubemx/Core/Src/can.c
    bsp/cubemx/Core/Src/dma.c
    bsp/cubemx/Core/Src/freertos.c
    bsp/cubemx/Core/Src/gpio.c
    bsp/cubemx/Core/Src/i2c.c
    bsp/cubemx/Core/Src/main.c
    bsp/cubemx/Core/Src/rng.c
    bsp/cubemx/Core/Src/spi.c
    bsp/cubemx/Core/Src/stm32f4xx_hal_msp.c
    bsp/cubemx/Core/Src/stm32f4xx_hal_timebase_tim.c
    bsp/cubemx/Core/Src/system_stm32f4xx.c
    bsp/cubemx/Core/Src/tim.c
    bsp/cubemx/Core/Src/usart.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c
    bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/list.c
    bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/queue.c
    bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/timers.c
    bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    bsp/cubemx/USB_DEVICE/App/usb_device.c
    bsp/cubemx/USB_DEVICE/App/usbd_desc.c
    bsp/cubemx/USB_DEVICE/Target/usbd_conf.c
    components/algorithm/mecanum.c
    components/algorithm/pid.c
    components/algorithm/ramp.c
    components/bmi088/AHRS_middleware.c
    components/bmi088/BMI088driver.c
    components/bmi088/BMI088Middleware.c
    components/bmi088/user_lib.c
    components/bmi088/libm_error.c
    components/cli/cli_history.c
    components/cli/cli_interpreter.c
    components/cli/cli_process.c
    components/devices/dbus.c
    components/devices/device.c
    components/devices/motor.c
    components/devices/single_gyro.c
    components/easyflash/port/ef_port.c
    components/easyflash/src/easyflash.c
    components/easyflash/src/ef_env.c
    components/easyflash/src/ef_iap.c
    components/easyflash/src/ef_log.c
    components/easyflash/src/ef_utils.c
    components/event_mgr/event_mgr.c
    components/event_mgr/memory_mgr.c
    components/log/log.c
    components/modules/chassis.c
    components/modules/gimbal.c
    components/modules/shoot.c
    components/protocol/protocol.c
    components/protocol/protocol_common.c
    components/protocol/protocol_interface.c
    components/protocol/protocol_transmit.c
    components/referee/referee_system.c
    components/soft_timer/os_timer.c
    components/soft_timer/period.c
    components/soft_timer/soft_timer.c
    components/support/fifo.c
    components/support/mem_mang4.c
    components/support/mf_crc.c
    components/systemview/SEGGER_RTT.c
    components/systemview/SEGGER_SYSVIEW.c
    components/systemview/SEGGER_SYSVIEW_Config_FreeRTOS.c
    components/systemview/SEGGER_SYSVIEW_FreeRTOS.c
    application/app_manage.c
    application/communicate.c
    application/init.c
    application/offline_service.c
    application/sensor_task.c
    application/shell.c
    application/chassis/chassis_app.c
    application/chassis/chassis_cmd.c
    application/chassis/chassis_task.c
    application/gimbal/gimbal_app.c
    application/gimbal/gimbal_cmd.c
    application/gimbal/gimbal_task.c
    application/gimbal/shoot_task.c
)

# set the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message("Debug build.")
elseif(CMAKE_BUILD_TYPE MATCHES Release)
  message("Release build.")
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
  set(opt_level -O0)
else()
  set(opt_level -Os)
endif()

# set some project constants
set(elf_file ${application_name}.elf)
set(bin_file ${application_name}.bin)
set(hex_file ${application_name}.hex)
set(map_file ${application_name}.map)
set(lss_file ${application_name}.lss)

# STM32-specific settings
set(DEVICE_FAMILY STM32F407xx)

# Used linker file (just for testing)
get_filename_component(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/tools/linker/STM32F407IGHx_FLASH.ld ABSOLUTE)

# Use LTO? WARNING: This might break the code
#set(ADDITIONAL_CORE_FLAGS -flto)

add_executable(${elf_file} ${sdk_sources})

# add include directories for library and CMSIS
target_include_directories(${elf_file} PUBLIC ${INCLUDE_DIRS})

# set up some defines for the library
target_compile_definitions(${elf_file} PUBLIC ${DEVICE_FAMILY})
target_compile_definitions(${elf_file} PUBLIC USE_HAL_DRIVER)
target_compile_definitions(${elf_file} PUBLIC ARM_MATH_CM4)
target_compile_definitions(${elf_file} PUBLIC __FPU_USED=1U)
target_compile_definitions(${elf_file} PUBLIC __FPU_PRESENT=1U)
target_compile_definitions(${elf_file} PUBLIC ARM_MATH_MATRIX_CHECK)
target_compile_definitions(${elf_file} PUBLIC __TARGET_FPU_VFP)
target_compile_definitions(${elf_file} PUBLIC ARM_MATH_ROUNDING)

target_link_libraries(${elf_file} PUBLIC ${BMI088_DIR}/libarm_cortexM4lf_math.a)
target_link_libraries(${elf_file} PUBLIC ${BMI088_DIR}/libahrs.a)

# set additional for compiler and linker: optimization and generate map file
set(additional_compiler_flags ${opt_level})
set(additional_linker_flags -lm -Wl,-Map=${map_file},--cref,--no-warn-mismatch)
target_compile_options(${elf_file} PRIVATE ${additional_compiler_flags})
target_link_libraries(${elf_file} PRIVATE ${additional_linker_flags})

# remove unused sections
target_link_libraries(${elf_file} PUBLIC "-g -Wl,--gc-sections")

# link_directories(${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(${elf_file}  PUBLIC ${INCLUDE_DIRS})
# link with linker file
target_link_libraries(${elf_file} PUBLIC -T${LINKER_SCRIPT})

# show size of resulting firmware image
add_custom_target(${elf_file}-size DEPENDS ${elf_file} COMMAND ${ARM_SIZE_EXECUTABLE} -B ${elf_file})

# generate extended listing
add_custom_target(${lss_file} DEPENDS ${elf_file} COMMAND ${ARM_OBJDUMP_EXECUTABLE} -S ${elf_file} > ${lss_file})

# create binary and hex files
add_custom_target(${hex_file} DEPENDS ${elf_file} COMMAND ${ARM_OBJCOPY_EXECUTABLE} -Oihex ${elf_file} ${hex_file})
add_custom_target(${bin_file} DEPENDS ${elf_file} COMMAND ${ARM_OBJCOPY_EXECUTABLE} -Obinary ${elf_file} ${bin_file})
add_custom_target(${application_name} ALL DEPENDS ${elf_file}-size ${bin_file} ${hex_file} ${lss_file})

