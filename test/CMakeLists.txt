
set(CBOR_COMPILE_OPTIONS "-Werror;-Wall;-Wextra;-Wshadow;-Wnon-virtual-dtor;-Wpedantic;")

add_executable(test_cbor_serializer test_cbor_serializer.cpp)
target_link_libraries(test_cbor_serializer PRIVATE cbor)
target_compile_options(test_cbor_serializer PRIVATE ${CBOR_COMPILE_OPTIONS})
add_test(cbor_serializer test_cbor_serializer)

add_executable(test_no_exceptions test_no_exceptions.cpp)
target_link_libraries(test_no_exceptions PRIVATE cbor)
target_compile_options(test_no_exceptions PRIVATE ${CBOR_COMPILE_OPTIONS} -fno-exceptions)
add_test(no_exceptions test_no_exceptions)

add_executable(test_example example.cpp)
target_link_libraries(test_example PRIVATE cbor)
target_compile_options(test_example PRIVATE ${CBOR_COMPILE_OPTIONS})
add_test(example test_example)

add_executable(test_shortfloat test_shortfloat.cpp shortfloat.cpp)
target_link_libraries(test_shortfloat PRIVATE cbor)
target_compile_options(test_shortfloat PRIVATE ${CBOR_COMPILE_OPTIONS})
add_test(shortfloat test_shortfloat)

# Something like:
# clang++-7 -g -fsanitize=address,fuzzer ../cbor/test/test_fuzzer.cpp -I../cbor/include/
add_custom_target(test_fuzzer COMMAND ${CMAKE_CXX_COMPILER} "-g" "-fsanitize=address,fuzzer" #"-fprofile-instr-generate" "-fcoverage-mapping "
                                                            "-I${CMAKE_CURRENT_SOURCE_DIR}/../include/"
                                                            "${CMAKE_CURRENT_SOURCE_DIR}/test_fuzzer.cpp"
                                                            "-o" "${CMAKE_CURRENT_BINARY_DIR}/test_fuzzer"
                  COMMENT "This is the fuzzer test")
#add_executable(test_fuzzer test_fuzzer.cpp)
#target_link_libraries(test_fuzzer PRIVATE cbor)
#target_compile_options(test_fuzzer PRIVATE "${CBOR_COMPILE_OPTIONS};-fsanitize=address,fuzzer;-ffunction-sections")
#set_target_properties( test_fuzzer PROPERTIES  LINK_FLAGS "-Wl,-gc-sections;")
#add_test(fuzzer test_fuzzer)

add_executable(test_fuzz_input test_fuzz_input.cpp)
target_link_libraries(test_fuzz_input PRIVATE cbor)
target_compile_options(test_fuzz_input PRIVATE ${CBOR_COMPILE_OPTIONS})
add_test(fuzz_input test_fuzz_input)
