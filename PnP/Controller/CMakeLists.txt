cmake_minimum_required(VERSION 3.20)

if(NOT CMAKE_TOOLCHAIN_FILE)
	set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake)
endif()

project(pnp_ctrl C ASM)

set(BOARD pnp_h723)
set(BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bsp/${BOARD})
set(PINMAP_XLSX "D:/Projects/PCB/PnP/PnP/PNP_CTRL_H723VET6.xlsx" CACHE FILEPATH "Pin-map workbook")
option(STEPGEN_SELFTEST "Stream test pattern on all STEP/DIR pins" OFF)

# pins.h is committed; `cmake --build . --target pins` regenerates it from the workbook.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
	add_custom_target(pins
		COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_pins.py ${PINMAP_XLSX} ${BOARD_DIR}/pins.h
		COMMENT "Regenerating pins.h from ${PINMAP_XLSX}")
endif()

set(MCU_FLAGS -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)

set(TUSB_DIR third_party/tinyusb/src)
set(TUSB_SRCS
	${TUSB_DIR}/tusb.c
	${TUSB_DIR}/common/tusb_fifo.c
	${TUSB_DIR}/device/usbd.c
	${TUSB_DIR}/class/cdc/cdc_device.c
	${TUSB_DIR}/portable/synopsys/dwc2/dcd_dwc2.c
	${TUSB_DIR}/portable/synopsys/dwc2/dwc2_common.c
)
set(LWIP_DIR third_party/lwip/src)
set(LWIP_SRCS
	${LWIP_DIR}/core/init.c
	${LWIP_DIR}/core/def.c
	${LWIP_DIR}/core/inet_chksum.c
	${LWIP_DIR}/core/ip.c
	${LWIP_DIR}/core/mem.c
	${LWIP_DIR}/core/memp.c
	${LWIP_DIR}/core/netif.c
	${LWIP_DIR}/core/pbuf.c
	${LWIP_DIR}/core/raw.c
	${LWIP_DIR}/core/stats.c
	${LWIP_DIR}/core/sys.c
	${LWIP_DIR}/core/timeouts.c
	${LWIP_DIR}/core/udp.c
	${LWIP_DIR}/core/tcp.c
	${LWIP_DIR}/core/tcp_in.c
	${LWIP_DIR}/core/tcp_out.c
	${LWIP_DIR}/core/ipv4/acd.c
	${LWIP_DIR}/core/ipv4/etharp.c
	${LWIP_DIR}/core/ipv4/icmp.c
	${LWIP_DIR}/core/ipv4/ip4.c
	${LWIP_DIR}/core/ipv4/ip4_addr.c
	${LWIP_DIR}/core/ipv4/ip4_frag.c
	${LWIP_DIR}/netif/ethernet.c
)
set_source_files_properties(${LWIP_SRCS} PROPERTIES COMPILE_OPTIONS "-Wno-error")

set(VENDOR_SRCS
	third_party/cmsis_device_h7/Source/system_stm32h7xx.c
	third_party/cmsis_device_h7/Source/startup_stm32h723xx.s
	${TUSB_SRCS}
)
# Shared by app and bootloader
set(COMMON_SRCS
	bsp/${BOARD}/board.c
	platform/clock.c
	drivers/usb.c
	drivers/usb_descriptors.c
)

# Vendor sources are not held to -Werror.
set_source_files_properties(third_party/cmsis_device_h7/Source/system_stm32h7xx.c ${TUSB_SRCS}
	PROPERTIES COMPILE_OPTIONS "-Wno-error")

function(firmware name mem_ld)
	add_executable(${name} ${ARGN} ${COMMON_SRCS} ${VENDOR_SRCS})
	set_target_properties(${name} PROPERTIES
		SUFFIX ".elf"
		LINK_DEPENDS "${BOARD_DIR}/${mem_ld};${BOARD_DIR}/sections.ld")
	target_include_directories(${name} PRIVATE
		app
		boot
		bsp/${BOARD}
		platform
		drivers
		third_party/cmsis_core/Include
		third_party/cmsis_device_h7/Include
		${TUSB_DIR}
	)
	target_compile_definitions(${name} PRIVATE
		STM32H723xx
		HSE_VALUE=25000000
		USE_PWR_LDO_SUPPLY
	)
	target_compile_options(${name} PRIVATE
		${MCU_FLAGS}
		$<$<COMPILE_LANGUAGE:C>:-std=gnu11 -Wall -Wextra -Werror -ffunction-sections -fdata-sections>
		$<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>
		-g3 -O2
	)
	target_link_options(${name} PRIVATE
		${MCU_FLAGS}
		-L${BOARD_DIR} -T${BOARD_DIR}/${mem_ld}
		--specs=nano.specs --specs=nosys.specs
		-Wl,--gc-sections -Wl,-Map=${name}.map -Wl,--print-memory-usage
		# .init_array/.fini_array are writable input sections placed in flash; benign.
		-Wl,--no-warn-rwx-segments
	)
	target_link_libraries(${name} m)
	add_custom_command(TARGET ${name} POST_BUILD
		COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${name}>
		COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${name}> ${name}.bin
	)
endfunction()

# App, sectors 1-3 (0x08020000)
firmware(pnp_ctrl mem_app.ld
	app/main.c
	app/console.c
	app/diags.c
	app/gcode.c
	app/motion.c
	app/scurve.c
	drivers/uart.c
	drivers/fdcan.c
	drivers/pwm.c
	drivers/buttons.c
	drivers/adc.c
	drivers/stepgen.c
	drivers/stepgen_fill.c
	drivers/tmc.c
	platform/boot.c
	platform/mpu.c
	drivers/eth.c
	drivers/net.c
	${LWIP_SRCS}
)
target_include_directories(pnp_ctrl PRIVATE drivers/lwip ${LWIP_DIR}/include)
target_compile_definitions(pnp_ctrl PRIVATE $<$<BOOL:${STEPGEN_SELFTEST}>:STEPGEN_SELFTEST>)
target_link_options(pnp_ctrl PRIVATE -u _printf_float) # %f in diags

# Bootloader, sector 0 (0x08000000)
firmware(pnp_boot mem_boot.ld
	boot/main.c
	boot/loader.c
	boot/flash.c
)
target_compile_definitions(pnp_boot PRIVATE
	USB_PID=0x4002
	USB_PRODUCT="PnP Bootloader"
	USB_CDC_NAME="PnP-Boot"
)
