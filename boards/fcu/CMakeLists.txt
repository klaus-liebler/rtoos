# Board support: Factory Control Unit (STM32H563VIT6).
add_library(rtos_board_fcu STATIC src/fcu.cpp)
target_include_directories(rtos_board_fcu PUBLIC include)
target_link_libraries(rtos_board_fcu PUBLIC rtos_stm32h563)
