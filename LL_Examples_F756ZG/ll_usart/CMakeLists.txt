cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD                11)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
set(CMAKE_CXX_STANDARD              20)
set(CMAKE_CXX_STANDARD_REQUIRED     ON)
set(CMAKE_CXX_EXTENSIONS            ON)
set(PROJ_PATH                       ${CMAKE_CURRENT_SOURCE_DIR})
message("Build type: "              ${CMAKE_BUILD_TYPE})

#
# Core project settings
#
project(ll_usart) # Modified
enable_language(C CXX ASM)

#
# Core MCU flags, CPU, instruction set and FPU setup
# Needs to be set properly for your MCU
#
set(CPU_PARAMETERS
    -mthumb

    # This needs attention to properly set for used MCU
    -mcpu=cortex-m7         # Modified
    -mfpu=fpv5-sp-d16       # Modified
    -mfloat-abi=hard        # Modified
)

# Set linker script
set(linker_script_SRC               ${PROJ_PATH}/STM32F756ZGTX_FLASH.ld) # Modified
set(EXECUTABLE                      ${CMAKE_PROJECT_NAME})

#
# List of source files to compile
#
set(sources_SRCS            # Modified
    # Put here your source files, one in each line, relative to CMakeLists.txt file location
    ${PROJ_PATH}/Core/Src/main.c
    ${PROJ_PATH}/Core/Src/gpio.c
    ${PROJ_PATH}/Core/Src/stm32f7xx_it.c
    ${PROJ_PATH}/Core/Src/syscalls.c
    ${PROJ_PATH}/Core/Src/sysmem.c
    ${PROJ_PATH}/Core/Src/system_stm32f7xx.c
    ${PROJ_PATH}/Core/Startup/startup_stm32f756zgtx.s
    ${PROJ_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_utils.c
    ${PROJ_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_gpio.c
    ${PROJ_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_dma.c
    ${PROJ_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_exti.c
    ${PROJ_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_rcc.c
    ${PROJ_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usart.c
    ${PROJ_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_dma.c
    ${PROJ_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_pwr.c
    #${PROJ_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_i2c.c

)

#
# Include directories
#
set(include_path_DIRS       # Modified
    # Put here your include dirs, one in each line, relative to CMakeLists.txt file location
    ${PROJ_PATH}/Core/Inc
    ${PROJ_PATH}/Drivers/STM32F7xx_HAL_Driver/Inc
    ${PROJ_PATH}/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
    ${PROJ_PATH}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
    ${PROJ_PATH}/Drivers/CMSIS/Include
)

#
# Symbols definition
#
set(symbols_SYMB            # Modified
    # Put here your symbols (preprocessor defines), one in each line
    # Encapsulate them with double quotes for safety purpose
    DEBUG
    STM32F756xx
    USE_FULL_LL_DRIVER
)

# Executable files
add_executable(${EXECUTABLE} ${sources_SRCS})

# Include paths
target_include_directories(${EXECUTABLE} PRIVATE ${include_path_DIRS})

# Project symbols
target_compile_definitions(${EXECUTABLE} PRIVATE ${symbols_SYMB})

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    # Full debug configuration
    -Og -g3 -ggdb
)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
    -T${linker_script_SRC}
    ${CPU_PARAMETERS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    --specs=nosys.specs
    -u _printf_float                # STDIO float formatting support
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,--print-memory-usage
)

# Execute post-build to print size
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
)

# Convert output to hex and binary
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
)

# Convert to bin file -> add conditional check?
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)