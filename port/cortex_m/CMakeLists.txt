# The Cortex-M core shared by the ports (registers, cycle counter, CoreServices), the
# freestanding start-up code and rtos_add_firmware() for every chip.
add_library(rtos_port_cortex_m INTERFACE)
target_include_directories(rtos_port_cortex_m INTERFACE include)
target_link_libraries(rtos_port_cortex_m INTERFACE rtos_kernel)

# Reset_Handler, _sbrk, abort: added to every firmware image as objects (like the vector table).
add_library(rtos_cortex_m_startup OBJECT src/startup.cpp)
target_link_libraries(rtos_cortex_m_startup PRIVATE rtos_port_cortex_m)

# rtos_add_firmware(<name> SOURCES <src>... LIBRARIES <lib>... [FLASH_DATA_START <address>])
# Builds <name>.elf plus <name>.bin and a linker map for the chip of this build tree (its package
# sets RTOS_CHIP_VECTORS, the OBJECT library with its vector table; the linker script comes with
# the chip library). FLASH_DATA_START keeps the flash from that address to the end free for data
# (littlefs, settings): the link fails if the image reaches it.
function(rtos_add_firmware name)
    cmake_parse_arguments(FW "" "FLASH_DATA_START" "SOURCES;LIBRARIES" ${ARGN})
    if(NOT RTOS_CHIP_VECTORS)
        message(FATAL_ERROR "rtos_add_firmware: no chip package in this build (RTOS_CHIP_VECTORS)")
    endif()
    add_executable(${name} ${FW_SOURCES} $<TARGET_OBJECTS:rtos_cortex_m_startup> $<TARGET_OBJECTS:${RTOS_CHIP_VECTORS}>)
    target_link_libraries(${name} PRIVATE ${FW_LIBRARIES})
    if(FW_FLASH_DATA_START)
        target_link_options(${name} PRIVATE -Wl,--defsym=rtos_flash_data_start=${FW_FLASH_DATA_START})
    endif()
    target_link_options(${name} PRIVATE -Wl,-Map=$<TARGET_FILE_DIR:${name}>/${name}.map)
    set_target_properties(${name} PROPERTIES SUFFIX .elf)
    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${name}>
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.bin)
endfunction()
