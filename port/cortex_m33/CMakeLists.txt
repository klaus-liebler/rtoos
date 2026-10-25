add_library(rtos_port_cm33 STATIC src/port.cpp)
target_include_directories(rtos_port_cm33 PUBLIC include)
target_link_libraries(rtos_port_cm33 PUBLIC rtos_kernel)
