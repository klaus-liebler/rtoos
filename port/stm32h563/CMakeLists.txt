# Chip support: headers, CPU port and linker script.
add_library(rtos_stm32h563 INTERFACE)
target_include_directories(rtos_stm32h563 INTERFACE include)
target_link_libraries(rtos_stm32h563 INTERFACE rtos_port_cm33)
target_link_options(rtos_stm32h563 INTERFACE -T${CMAKE_CURRENT_SOURCE_DIR}/stm32h563xi.ld)
set_property(TARGET rtos_stm32h563 APPEND PROPERTY INTERFACE_LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/stm32h563xi.ld)

# Start-up code and vector table. Added to every firmware image as objects: nothing
# references the vector table, so it would never be pulled out of an archive.
add_library(rtos_stm32h563_startup OBJECT src/startup.cpp src/vectors.cpp)
target_link_libraries(rtos_stm32h563_startup PRIVATE rtos_stm32h563)

# rtos_add_firmware(<name> SOURCES <src>... LIBRARIES <lib>...)
# Builds <name>.elf plus <name>.bin and a linker map.
function(rtos_add_firmware name)
    cmake_parse_arguments(FW "" "" "SOURCES;LIBRARIES" ${ARGN})
    add_executable(${name} ${FW_SOURCES} $<TARGET_OBJECTS:rtos_stm32h563_startup>)
    target_link_libraries(${name} PRIVATE ${FW_LIBRARIES})
    target_link_options(${name} PRIVATE -Wl,-Map=$<TARGET_FILE_DIR:${name}>/${name}.map)
    set_target_properties(${name} PROPERTIES SUFFIX .elf)
    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${name}>
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.bin)
endfunction()
