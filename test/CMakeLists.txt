
set(CBOR_COMPILE_OPTIONS "-Werror;-Wall;-Wextra;-Wshadow;-Wnon-virtual-dtor;-Wpedantic")

add_executable(test_cbor_serializer test_cbor_serializer.cpp)
target_link_libraries(test_cbor_serializer PRIVATE cbor)
target_compile_options(test_cbor_serializer PRIVATE ${CBOR_COMPILE_OPTIONS})
add_test(cbor_serializer test_cbor_serializer)
