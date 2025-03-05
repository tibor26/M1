cmake_minimum_required(VERSION 3.15)

project(PBMM LANGUAGES C CXX ASM)

find_bsp(
	ID com.sysprogs.arm.stm32
	VERSION 2024.07
	MCU STM32G030C8
	HWREGISTER_LIST_FILE STM32G0xxxx/DeviceDefinitions/stm32g030xx.xml
	DISABLE_GNU_EXTENSIONS)

register_imported_project(NAME PBMM
	PATH ../.cproject
	LEVEL 0)
add_bsp_based_executable(
	NAME PBMM
	SOURCES
		${PBMM_LOCATION}/Core/detection.c
		${PBMM_LOCATION}/Core/display.c
		${PBMM_LOCATION}/Core/encoder.c
		${PBMM_LOCATION}/Core/font.c
		${PBMM_LOCATION}/Core/key.c
		${PBMM_LOCATION}/Core/memory.c
		${PBMM_LOCATION}/Core/mode.c
		${PBMM_LOCATION}/Core/MotorSpeedFeedbackAPI.c
		${PBMM_LOCATION}/Core/tlc6983.c
		${PBMM_LOCATION}/Core/uart.c
		${PBMM_LOCATION}/Core/Src/main.c
		${PBMM_LOCATION}/Core/Src/stm32g0xx_it.c
		${PBMM_LOCATION}/Core/Src/stm32g0xx_hal_msp.c
		${PBMM_LOCATION}/Core/Src/system_stm32g0xx.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_iwdg.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_rcc.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_rcc_ex.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_gpio.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_dma.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_pwr_ex.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_cortex.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_spi.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_tim.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_tim_ex.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_uart.c
		${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_uart_ex.c
		${PBMM_LOCATION}/Core/Inc/constant.h
		${PBMM_LOCATION}/Core/Inc/main.h
		${PBMM_LOCATION}/Core/Inc/memory.h
		${PBMM_LOCATION}/Core/Inc/stm32g0xx_hal_conf.h
		${PBMM_LOCATION}/Core/Inc/stm32g0xx_it.h
		${PBMM_LOCATION}/Core/Inc/TLC6983.h
		${PBMM_LOCATION}/VisualStudio/version.cmake
		${PBMM_LOCATION}/VisualStudio/version.h
		${PBMM_LOCATION}/Core/Src/syscalls.c
		${PBMM_LOCATION}/Core/Inc/BMA400.h
		${PBMM_LOCATION}/Core/BMA400.c
		${PBMM_LOCATION}/Core/pcba_test.c
		${PBMM_LOCATION}/Drivers/CMSIS/Include/core_cm0.h
	GENERATE_BIN
	GENERATE_MAP)
target_link_libraries(PBMM PRIVATE -Wl,--print-memory-usage)
target_compile_options(PBMM PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(PBMM
	PRIVATE
	${PBMM_LOCATION}/Core/Inc
	${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc
	${PBMM_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy
	${PBMM_LOCATION}/Drivers/CMSIS/Device/ST/STM32G0xx/Include
	${PBMM_LOCATION}/Drivers/CMSIS/Include)

ADD_CUSTOM_TARGET (
    git_version
    ${CMAKE_COMMAND} -D SRC=${CMAKE_SOURCE_DIR}/version.h.in
					 -D DST=${CMAKE_SOURCE_DIR}/version.h
					 -D GIT_DIR=${PBMM_LOCATION}
					 -P ${CMAKE_SOURCE_DIR}/version.cmake)

ADD_DEPENDENCIES (PBMM git_version)

target_compile_definitions(PBMM
	PRIVATE
	USE_HAL_DRIVER
	STM32G030xx
	VERSION_NUMBER_AUTO)

set(ignoreMe "${LOCALAPPDATA}")
