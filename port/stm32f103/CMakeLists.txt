# Chip support: headers, CPU port and linker script (STM32F103x8/xB, LQFP48: STM32F103C8T6/CBT6).
# The linker script is the xB's (128 KB flash, 20 KB RAM); an x8 board passes its own with
# RTOS_STM32F103_LINKER_SCRIPT.
set(RTOS_STM32F103_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/stm32f103xb.ld" CACHE FILEPATH
    "linker script of the STM32F103 part")
add_library(rtos_stm32f103 INTERFACE)
target_include_directories(rtos_stm32f103 INTERFACE include)
target_link_libraries(rtos_stm32f103 INTERFACE rtos_port_cm3 rtos_stm32)
target_link_options(rtos_stm32f103 INTERFACE -T${RTOS_STM32F103_LINKER_SCRIPT})
set_property(TARGET rtos_stm32f103 APPEND PROPERTY INTERFACE_LINK_DEPENDS ${RTOS_STM32F103_LINKER_SCRIPT})

# Vector table. Added to every firmware image as objects (rtos_add_firmware).
add_library(rtos_stm32f103_vectors OBJECT src/vectors.cpp)
target_link_libraries(rtos_stm32f103_vectors PRIVATE rtos_stm32f103)
set(RTOS_CHIP_VECTORS rtos_stm32f103_vectors CACHE INTERNAL "vector table of the chip of this build")
