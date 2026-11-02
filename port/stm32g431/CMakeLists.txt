# Chip support: headers, CPU port and linker script.
# RTOS_STM32G431_PACKAGE: the package of the board's chip (gpio::Package, pin_functions.hpp);
# the pin database is filtered for it, a pin or function the package lacks does not compile.
set(RTOS_STM32G431_PACKAGE "UFQFPN48" CACHE STRING "STM32G431 package (e.g. UFQFPN48 for STM32G431CBU6)")
set(rtos_g431_packages LQFP32 UFQFPN32 LQFP48 UFQFPN48 WLCSP49 LQFP64 UFBGA64 LQFP80 LQFP100)
set_property(CACHE RTOS_STM32G431_PACKAGE PROPERTY STRINGS ${rtos_g431_packages})
if(NOT RTOS_STM32G431_PACKAGE IN_LIST rtos_g431_packages)
    message(FATAL_ERROR "RTOS_STM32G431_PACKAGE must be one of ${rtos_g431_packages} (is ${RTOS_STM32G431_PACKAGE})")
endif()

add_library(rtos_stm32g431 INTERFACE)
target_include_directories(rtos_stm32g431 INTERFACE include)
target_link_libraries(rtos_stm32g431 INTERFACE rtos_port_cm4 rtos_stm32)
target_compile_definitions(rtos_stm32g431 INTERFACE RTOS_STM32G431_PACKAGE=${RTOS_STM32G431_PACKAGE})
target_link_options(rtos_stm32g431 INTERFACE -T${CMAKE_CURRENT_SOURCE_DIR}/stm32g431xb.ld)
set_property(TARGET rtos_stm32g431 APPEND PROPERTY INTERFACE_LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/stm32g431xb.ld)

# Vector table. Added to every firmware image as objects (rtos_add_firmware).
add_library(rtos_stm32g431_vectors OBJECT src/vectors.cpp)
target_link_libraries(rtos_stm32g431_vectors PRIVATE rtos_stm32g431)
set(RTOS_CHIP_VECTORS rtos_stm32g431_vectors CACHE INTERNAL "vector table of the chip of this build")
