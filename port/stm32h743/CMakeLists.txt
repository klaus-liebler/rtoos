# Chip support: headers, CPU port and linker script (STM32H743xI, LQFP100: STM32H743VIT6).
add_library(rtos_stm32h743 INTERFACE)
target_include_directories(rtos_stm32h743 INTERFACE include)
target_link_libraries(rtos_stm32h743 INTERFACE rtos_port_cm7 rtos_stm32)
target_link_options(rtos_stm32h743 INTERFACE -T${CMAKE_CURRENT_SOURCE_DIR}/stm32h743xi.ld)
set_property(TARGET rtos_stm32h743 APPEND PROPERTY INTERFACE_LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/stm32h743xi.ld)

# Vector table. Added to every firmware image as objects (rtos_add_firmware).
add_library(rtos_stm32h743_vectors OBJECT src/vectors.cpp)
target_link_libraries(rtos_stm32h743_vectors PRIVATE rtos_stm32h743)
set(RTOS_CHIP_VECTORS rtos_stm32h743_vectors CACHE INTERNAL "vector table of the chip of this build")
