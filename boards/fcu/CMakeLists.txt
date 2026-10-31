# Board support: Factory Control Unit (STM32H563VIT6).
add_library(rtos_board_fcu STATIC src/fcu.cpp)
target_include_directories(rtos_board_fcu PUBLIC include)
target_link_libraries(rtos_board_fcu PUBLIC rtos_stm32h563)
# Hardware revision: 1 = first boards (pin map of the CubeMX project), 2 = current PCB
# (pcb_factory_control_unit). Only the pins that board/fcu.hpp knows differ here (LED); the
# product's own I/O pins are the product's business.
set(RTOS_FCU_REVISION 2 CACHE STRING "Factory Control Unit hardware revision (1 or 2)")
set_property(CACHE RTOS_FCU_REVISION PROPERTY STRINGS 1 2)
if(NOT RTOS_FCU_REVISION MATCHES "^[12]$")
    message(FATAL_ERROR "RTOS_FCU_REVISION must be 1 or 2 (is ${RTOS_FCU_REVISION})")
endif()
target_compile_definitions(rtos_board_fcu PUBLIC RTOS_FCU_REVISION=${RTOS_FCU_REVISION})
