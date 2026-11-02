add_library(rtos_port_cm4 STATIC src/port.cpp)
target_include_directories(rtos_port_cm4 PUBLIC include)
target_link_libraries(rtos_port_cm4 PUBLIC rtos_port_cortex_m)
