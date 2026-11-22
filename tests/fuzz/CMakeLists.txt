# AFL++ harnesses (docs/fuzzing.md). RTOS_FUZZ=ON: with afl-g++-fast (preset linux-afl) they are
# fuzz targets; with g++ (preset linux-asan) they replay their corpus and the regression inputs of
# fixed findings under ASan + UBSan: ctest fuzz_replay_<harness>.
set(rtos_fuzz_harnesses http websocket dns dhcp modbus opcua ethercat plc)

foreach(h IN LISTS rtos_fuzz_harnesses)
    add_executable(fuzz_${h} ${h}.cpp)
    target_include_directories(fuzz_${h} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/tests)
    target_link_libraries(fuzz_${h} PRIVATE rtos_kernel)
    target_compile_options(fuzz_${h} PRIVATE -Wall -Wextra -Werror)
    set(dirs ${CMAKE_CURRENT_SOURCE_DIR}/corpus/${h})
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/regressions/${h})
        list(APPEND dirs ${CMAKE_CURRENT_SOURCE_DIR}/regressions/${h})
    endif()
    add_test(NAME fuzz_replay_${h} COMMAND fuzz_${h} ${dirs})
endforeach()
target_link_libraries(fuzz_opcua PRIVATE rtos_opcua rtos_tls rtos_certs)
target_link_libraries(fuzz_plc PRIVATE rtos_plc)
