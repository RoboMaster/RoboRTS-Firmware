cmake_minimum_required (VERSION 2.8)
project (infantry)

ENABLE_LANGUAGE(ASM)


set(DIR_SRCS
application/init.c
application/communicate.c
application/chassis_task.c
application/gimbal_task.c
application/shoot_task.c
application/timer_task.c
application/infantry_cmd.c
application/offline_check.c
application/referee_system.c
bsp/boards/board.c
bsp/boards/drv_can.c
bsp/boards/drv_imu.c
bsp/boards/drv_flash.c
bsp/boards/drv_dr16.c
bsp/boards/drv_io.c
bsp/boards/drv_uart.c
components/object/object.c
components/devices/device.c
components/devices/motor.c
components/devices/dbus.c
components/devices/detect.c
components/controller/controller.c
components/controller/pid_controller.c
components/modules/chassis.c
components/modules/gimbal.c
components/modules/shoot.c
components/modules/single_gyro.c
components/algorithm/mecanum.c
components/algorithm/madgwick_ahrs.c
components/algorithm/mahony_ahrs.c
components/algorithm/pid.c
components/algorithm/ramp.c
utilities/period.c
utilities/soft_timer.c
utilities/ulog/ulog.c
utilities/ulog/ulog_console.c
test/test.c
test/test_module.c
test/log_test.c
application/protocol/protocol.c
application/protocol/protocol_common.c
application/protocol/protocol_transmit.c
application/protocol/protocol_interface.c
components/support/fifo.c
components/support/mem_mang4.c
components/support/mf_crc.c
bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/list.c
bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/queue.c
bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
application/param.c
bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/timers.c
bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c
bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
bsp/cubemx/Core/Src/can.c
bsp/cubemx/Core/Src/dma.c
bsp/cubemx/Core/Src/freertos.c
bsp/cubemx/Core/Src/gpio.c
bsp/cubemx/Core/Src/main.c
bsp/cubemx/Core/Src/spi.c
bsp/cubemx/Core/Src/stm32f4xx_hal_msp.c
bsp/cubemx/Core/Src/stm32f4xx_hal_timebase_TIM.c
bsp/cubemx/Core/Src/stm32f4xx_it.c
bsp/cubemx/Core/Src/system_stm32f4xx.c
bsp/cubemx/Core/Src/tim.c
bsp/cubemx/Core/Src/usart.c
bsp/cubemx/USB_DEVICE/App/usbd_cdc_if.c
bsp/cubemx/USB_DEVICE/App/usb_device.c
bsp/cubemx/USB_DEVICE/Target/usbd_conf.c
bsp/cubemx/USB_DEVICE/App/usbd_desc.c
bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
bsp/cubemx/Core/Src/startup_stm32f427xx.s
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c

)

INCLUDE_DIRECTORIES(
bsp/cubemx/Core/Inc 
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Inc 
bsp/cubemx/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy 
bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F 
bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Core/Inc 
bsp/cubemx/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc 
bsp/cubemx/Drivers/CMSIS/Include 
bsp/cubemx/Drivers/CMSIS/Device/ST/STM32F4xx/Include 
bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/include 
bsp/cubemx/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS 
bsp/cubemx/USB_DEVICE/App 
bsp/cubemx/USB_DEVICE/Target 
bsp/boards 
components/algorithm 
components/devices 
components/modules 
components/object 
components/support 
application 
components/controller 
test 
utilities 
utilities/ulog 
utilities/finsh 
application/protocol 
config
)


set(CMAKE_EXE_LINKER_FLAGS
"--cpu Cortex-M4.fp --library_type=microlib --strict --scatter \"infantry.sct\" --summary_stderr --info summarysizes --map --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers"
)
set(CMAKE_C_FLAGS
"--c99 -c --cpu Cortex-M4.fp -D__MICROLIB -g -O3 --apcs=interwork --split_sections -D__UVISION_VERSION=\"524\" -D_RTE_ -DSTM32F427xx -DUSE_HAL_DRIVER -DSTM32F427xx -DARM_MATH_CM4"
)
set(CMAKE_ASM_FLAGS
"--cpu Cortex-M4.fp -g --apcs=interwork --pd \"__MICROLIB SETA 1\" --pd \"__UVISION_VERSION SETA 524\" --pd \"_RTE_ SETA 1\" --pd \"STM32F427xx SETA 1\"")


add_executable(infantry ${DIR_SRCS})

set_target_properties(
    infantry 
    PROPERTIES 
        OUTPUT_NAME "infantry"
        SUFFIX ".axf"
)

add_custom_command(TARGET infantry POST_BUILD
    COMMENT "Generationg .bin ....."
    COMMAND C:/Keil_v5/ARM/ARMCC/bin/fromelf.exe  --bin -o infantry.bin infantry.axf
)