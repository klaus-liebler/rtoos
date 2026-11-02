# The peripheral drivers the STM32 chips share (same IP, chip-specific addresses come from the
# chip packages: stm32h563, stm32g431).
add_library(rtos_stm32 INTERFACE)
target_include_directories(rtos_stm32 INTERFACE include)
target_link_libraries(rtos_stm32 INTERFACE rtos_port_cortex_m)
