# Cortex-M7 (ARMv7E-M): the context switch is the ARMv7-M one of port/cortex_m4 (rtos_port_cm4,
# compiled here with the Cortex-M7 flags of this build tree); this package adds what the M7 has
# beyond the M4 - the L1 instruction and data caches.
add_library(rtos_port_cm7 INTERFACE)
target_include_directories(rtos_port_cm7 INTERFACE include)
target_link_libraries(rtos_port_cm7 INTERFACE rtos_port_cm4)
