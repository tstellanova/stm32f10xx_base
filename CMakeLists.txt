cmake_minimum_required(VERSION 3.6)
project(stm32f10xx_base C ASM)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_definitions(-DSTM32F107xC)
#add_definitions(-DSTM32F10X_CL)
add_definitions(-mcpu=cortex-m3)
add_definitions(-DDEBUG )


include_directories(CMSIS/Include)
include_directories(CMSIS/Device/ST/STM32F1xx/Include)
include_directories(inc)
include_directories(STM32F1xx_HAL_Driver/Inc)
include_directories(BSP/VIEWTOOL_F107)

set(SOURCE_FILES
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cec.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_crc.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dac.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dac_ex.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_eth.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_hcd.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2s.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_irda.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_iwdg.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_msp_template.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_nand.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_nor.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pccard.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rtc.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rtc_ex.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_sd.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_smartcard.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi_ex.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_sram.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_usart.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_wwdg.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_fsmc.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_sdmmc.c
    STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c

    src/main.c
    src/stm32f1xx_hal_msp.c
    src/stm32f1xx_it.c
    src/syscalls.c
    src/system_stm32f1xx.c
        BSP/VIEWTOOL_F107/stm32f107_viewtool.c

    startup/startup_stm32f107xc.s
        )


add_executable(stm32f10xx ${SOURCE_FILES})

add_executable(${PROJECT_NAME}.elf ${SOURCE_FILES} ${LINKER_SCRIPT})


set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.map")
set(HEX_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")