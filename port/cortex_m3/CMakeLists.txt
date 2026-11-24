# Cortex-M3 (ARMv7-M, no FPU): the context switch is the ARMv7-M one of port/cortex_m4
# (rtos_port_cm4, compiled here with the Cortex-M3 flags of this build tree - its FP parts are
# under __ARM_FP and drop out); this package only names it for the M3 and checks the build.
add_library(rtos_port_cm3 INTERFACE)
target_include_directories(rtos_port_cm3 INTERFACE include)
target_link_libraries(rtos_port_cm3 INTERFACE rtos_port_cm4)
