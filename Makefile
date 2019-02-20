all:
	clang++-7 -g -Wall -Wextra -std=c++14 test_cbor_serializer.cpp -o test_cbor_serializer

format:
	clang-format-7 -i cbor/* *.cpp
