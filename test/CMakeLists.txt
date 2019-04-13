
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
