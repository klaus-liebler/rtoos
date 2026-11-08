# Board support: Factory Control Unit revision 3 (STM32H743VIT6). The same target name and
# header (board/fcu.hpp) as the revision 1/2 package (boards/fcu), so a product links
# rtos_board_fcu either way; the chip of the build tree (RTOS_CHIP) picks the package.
add_library(rtos_board_fcu STATIC src/fcu.cpp)
target_include_directories(rtos_board_fcu PUBLIC include)
target_link_libraries(rtos_board_fcu PUBLIC rtos_stm32h743)
set(RTOS_FCU_REVISION 3 CACHE STRING "Factory Control Unit hardware revision (3: STM32H743)")
if(NOT RTOS_FCU_REVISION STREQUAL "3")
    message(FATAL_ERROR "RTOS_FCU_REVISION must be 3 for the STM32H743 (is ${RTOS_FCU_REVISION}); revisions 1/2 are STM32H563")
endif()
target_compile_definitions(rtos_board_fcu PUBLIC RTOS_FCU_REVISION=${RTOS_FCU_REVISION})
