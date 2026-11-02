# Chip support: headers, CPU port and linker script.
add_library(rtos_stm32h563 INTERFACE)
target_include_directories(rtos_stm32h563 INTERFACE include)
target_link_libraries(rtos_stm32h563 INTERFACE rtos_port_cm33 rtos_stm32)
target_link_options(rtos_stm32h563 INTERFACE -T${CMAKE_CURRENT_SOURCE_DIR}/stm32h563xi.ld)
set_property(TARGET rtos_stm32h563 APPEND PROPERTY INTERFACE_LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/stm32h563xi.ld)

# Vector table. Added to every firmware image as objects (rtos_add_firmware): nothing
# references it, so it would never be pulled out of an archive.
add_library(rtos_stm32h563_vectors OBJECT src/vectors.cpp)
target_link_libraries(rtos_stm32h563_vectors PRIVATE rtos_stm32h563)
set(RTOS_CHIP_VECTORS rtos_stm32h563_vectors CACHE INTERNAL "vector table of the chip of this build")
