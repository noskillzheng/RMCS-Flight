cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake"
        CACHE FILEPATH "ARM Cortex-M3 GCC Toolchain")
endif()

project(Fire_F103 LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MCU_FLAGS
    -mcpu=cortex-m3
    -mthumb
)

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld")

set(APP_SOURCES
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/usart.c
    Core/Src/stm32f1xx_it.c
    Core/Src/stm32f1xx_hal_msp.c
    Core/Src/system_stm32f1xx.c
    Core/Src/syscalls.c
    startup_stm32f103xb.s
)

set(HAL_SOURCES
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
)

add_executable(${PROJECT_NAME}.elf
    ${APP_SOURCES}
    ${HAL_SOURCES}
)

target_include_directories(${PROJECT_NAME}.elf PRIVATE
    Core/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    Drivers/CMSIS/Include
)

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
    USE_HAL_DRIVER
    STM32F103xB
)

target_compile_options(${PROJECT_NAME}.elf PRIVATE
    ${MCU_FLAGS}
    -Og
    -g3
    -Wall
    -Wextra
    -fdata-sections
    -ffunction-sections
    $<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>
)

target_link_options(${PROJECT_NAME}.elf PRIVATE
    ${MCU_FLAGS}
    -specs=nano.specs
    -specs=nosys.specs
    -T${LINKER_SCRIPT}
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -Wl,--no-warn-rwx-segments
    -Wl,--print-memory-usage
)

set_target_properties(${PROJECT_NAME}.elf PROPERTIES LINK_DEPENDS "${LINKER_SCRIPT}")

set(HEX_FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex")
set(BIN_FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin")

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${PROJECT_NAME}.elf>
    BYPRODUCTS ${HEX_FILE} ${BIN_FILE}
    COMMENT "Generating ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
)
