/*
  Copyright (c) 2018-2019, Ivor Wanders
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the name of the author nor the names of contributors may be used to
    endorse or promote products derived from this software without specific
    prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <algorithm>
#include <array>
#include <iostream>
#include <vector>
#include "cbor/cbor.h"
#include "cbor/stl.h"

#include <cxxabi.h>
#include <cstdlib>
#include <memory>
#include <string>
#include <type_traits>
#include <typeinfo>

// https://stackoverflow.com/a/20170989
template <class T>
std::string type_name()
{
  typedef typename std::remove_reference<T>::type TR;
  std::unique_ptr<char, void (*)(void*)> own(abi::__cxa_demangle(typeid(TR).name(), nullptr, nullptr, nullptr),
                                             std::free);
  std::string r = own != nullptr ? own.get() : typeid(TR).name();
  if (std::is_const<TR>::value)
    r += " const";
  if (std::is_volatile<TR>::value)
    r += " volatile";
  if (std::is_lvalue_reference<T>::value)
    r += "&";
  else if (std::is_rvalue_reference<T>::value)
    r += "&&";
  return r;
}

using Data = std::vector<std::uint8_t>;

bool failed = false;

template <typename A, typename B>
void test(const A& a, const B& b)
{
  if (a != b)
  {
    std::cerr << "\033[31m"
              << "a (" << a << ") != b (" << b << ")"
              << "\033[0m" << std::endl;
    failed = true;
  }
  else
  {
    std::cerr << "\033[32m"
              << "a (" << a << ") == b (" << b << ")"
              << "\033[0m" << std::endl;
  }
}

/*

*/
void test_associatives()
{
  // some values from https://github.com/cbor/test-vectors/blob/master/appendix_a.json
  // Test map
  {
    std::map<unsigned int, unsigned int> input{ { 1, 2 }, { 3, 4 } };
    Data result = { 0xa2, 0x01, 0x02, 0x03, 0x04 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }
  // indefinite array 
  {
    Data input = {0x9f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x18, 0x18, 0x19, 0xff};
    std::vector<cbor::DataType> result;
    std::vector<cbor::DataType> expected_result;
    for (std::size_t i = 1; i < 26 ; i++)
    {
      expected_result.push_back(i);
    }
    cbor::from_cbor(result, input);
    test(cbor::hexdump(result), cbor::hexdump(expected_result));
  }
  // test string map
  {
    std::map<std::string, std::string> input{ { "a", "A" }, { "b", "B" }, { "c", "C" }, { "d", "D" }, { "e", "E" } };
    Data result = { 0xa5, 0x61, 0x61, 0x61, 0x41, 0x61, 0x62, 0x61, 0x42, 0x61, 0x63,
                    0x61, 0x43, 0x61, 0x64, 0x61, 0x44, 0x61, 0x65, 0x61, 0x45 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    std::map<std::string, std::string> output;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(input.size(), output.size());
    test(std::equal(input.begin(), input.end(), output.begin()), true);

    cbor::cbor_object as_obj;
    cbor::to_cbor(input, as_obj);

    std::cout << "map of strings:" << std::endl;
    std::cout << as_obj.prettyPrint() << std::endl;
    cbor::cbor_object reinterpret_object;
    const Data cbor_data_from_obj = as_obj.serialized_;
    cbor::from_cbor(reinterpret_object, cbor_data_from_obj);
    test(cbor::hexdump(reinterpret_object.serialized_), cbor::hexdump(cbor_data_from_obj));
  }
}
void test_stl()
{
  // test tuple
  {
    std::tuple<unsigned int, unsigned int> input{ 1, 2 };
    Data result = { 0x82, 0x01, 0x02 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    std::tuple<unsigned int, unsigned int> output;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(std::get<0>(output), std::get<0>(input));
    test(std::get<1>(output), std::get<1>(input));
  }
  // test vector
  {
    std::vector<unsigned int> input{ 1, 2 };
    Data result = { 0x82, 0x01, 0x02 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    std::vector<unsigned int> output;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(output.size(), input.size());
    test(output[0], input[0]);
    test(output[1], input[1]);
  }

  // test string
  {
    std::string input{ "foo" };
    Data result = { 0x63, 0x66, 0x6F, 0x6F };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    std::string output;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(input, output);
  }

  // test pair
  {
    std::pair<unsigned int, unsigned int> input{ 1, 2 };
    Data result = { 0x82, 0x01, 0x02 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    std::pair<unsigned int, unsigned int> output;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(output.first, input.first);
    test(output.second, input.second);
  }
  // test vector of pairs
  {
    using Point = std::pair<int, int>;
    using PointList = std::vector<Point>;
    PointList input{ { 3, 7 }, { 90, 800 } };
    Data result = { 0x82, 0x82, 0x03, 0x07, 0x82, 0x18, 0x5A, 0x19, 0x03, 0x20 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);

    cbor::cbor_object single_obj;
    cbor::to_cbor(input, single_obj);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    PointList read_back;
    const Data z = cbor_representation;
    cbor::from_cbor(read_back, z);

    std::vector<cbor::cbor_object> as_objects;
    cbor::from_cbor(as_objects, z);

    std::cout << "List of pairs" << std::endl;
    std::cout << single_obj.prettyPrint() << std::endl;
  }
}

void test_array()
{
  // Serializing into and from array:
  {
    unsigned int input{ 2 };
    Data result = { 0x02 };
    std::array<cbor::DataType, 100> z;
    std::size_t len = cbor::to_cbor(input, z.data(), z.size());
    test(cbor::hexdump(result), cbor::hexdump(z, len));

    unsigned int read_back;

    const cbor::DataType* offset = z.data();
    std::uint8_t size = z.size();
    cbor::from_cbor(read_back, offset, size);
    cbor::from_cbor(read_back, z.data(), z.size());
  }
  {
    unsigned int input{ 2 };
    Data result = { 0x02 };
    cbor::DataType z[300];
    std::size_t len = cbor::to_cbor(input, z);
    test(cbor::hexdump(result), cbor::hexdump(z, len));

    unsigned int read_back;

    const cbor::DataType* offset = z;
    std::uint16_t size = 300;
    cbor::from_cbor(read_back, z);
    test(read_back, input);
    cbor::from_cbor(read_back, offset, size);
    test(read_back, input);
  }

  {
    unsigned int my_int_array[2] = { 1, 2 };
    Data result = { 0x82, 0x01, 0x02 };
    std::array<cbor::DataType, 100> z;
    std::size_t len = cbor::to_cbor(my_int_array, z.data(), z.size());
    test(cbor::hexdump(result), cbor::hexdump(z, len));

    
    unsigned int read_back[2] = { 0, 0 };
    cbor::from_cbor(read_back, z.data(), z.size());
    test(read_back[0], my_int_array[0]);
    test(read_back[1], my_int_array[1]);

  }
}

namespace foo
{
struct Bar
{
  std::uint32_t f;
};

template <typename... Data>
std::size_t to_cbor(const Bar& b, cbor::detail::write_adapter<Data...> data)
{
  std::cout << "to cbor adl" << std::endl;
  to_cbor(b.f, data);
  //  cbor::to_cbor(b.f, data);
  return 0;
}

template <typename... Data>
std::size_t from_cbor(Bar& b, cbor::detail::read_adapter<Data...> data)
{
  std::cout << "from_cbor adl" << std::endl;
  from_cbor(b.f, data);
  //  cbor::from_cbor(b.f, data);
  return 0;
}

}  // namespace foo
void test_adl()
{
  {
    foo::Bar input{ 2 };
    Data result = { 0x02 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }
  {
    std::vector<foo::Bar> input = { foo::Bar{ 2 }, foo::Bar{ 3 }, foo::Bar{ 4 } };
    Data result = { 0x83, 0x02, 0x03, 0x04 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    std::vector<foo::Bar> output;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
  }
}

void test_pod()
{
  {
    const std::uint8_t input{ 50 };
    Data result = { 0x18, 0x32 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    std::uint8_t output = 0;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(input, output);
  }
  {
    const std::int8_t input{ -50 };
    Data result = { 0x38, 0x31 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    std::int8_t output = 0;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(input, output);
  }
  {
    const std::uint16_t input{ 50 };
    Data result = { 0x18, 0x32 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    std::uint16_t output = 0;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(input, output);
  }
  {
    unsigned int input{ 2 };
    Data result = { 0x02 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    unsigned int output = 0;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(input, output);
  }
  {
    double input{ 13377.1414 };
    Data result = { 0xFB, 0x40, 0xCA, 0x20, 0x92, 0x19, 0x65, 0x2B, 0xD4 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    double output = 0.0;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(input, output);
  }
  {
    float input{ 6.3125 };
    Data result = { 0xFA, 0x40, 0xCA, 0x00, 0x00 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    float output = 0.0;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(input, output);
  }
  {
    Data result = { 0xF6 };
    Data cbor_representation;
    cbor::to_cbor(nullptr, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }
  {
    int input{ 2 };
    Data result = { 0x02 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    int output = 0;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(input, output);

    int input2{ -2 };
    Data result2 = { 0x21 };
    cbor_representation.resize(0);
    cbor::to_cbor(input2, cbor_representation);
    test(cbor::hexdump(result2), cbor::hexdump(cbor_representation));

    int output2 = 0;
    const Data cbor_data2 = cbor_representation;
    cbor::from_cbor(output2, cbor_data2);
    test(input2, output2);
  }
  {
    bool bool_val = true;
    Data result = { 0xF5 };
    Data cbor_representation;
    cbor::to_cbor(bool_val, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    bool output = false;
    const Data cbor_data = cbor_representation;
    cbor::from_cbor(output, cbor_data);
    test(bool_val, output);

    bool_val = false;
    Data result2 = { 0xF4 };
    cbor_representation.resize(0);
    cbor::to_cbor(bool_val, cbor_representation);
    test(cbor::hexdump(result2), cbor::hexdump(cbor_representation));

    output = true;
    const Data cbor_data2 = cbor_representation;
    cbor::from_cbor(output, cbor_data2);
    test(output, bool_val);
  }
}

namespace cbor_object_ser
{
struct Buz
{
  std::uint32_t f;
};
/*
 */
std::size_t to_cbor(const Buz& b, cbor::cbor_object& data)
{
  std::cout << "to cbor_object_ser adl " << std::endl;
  to_cbor(b.f, data);
  //  cbor::to_cbor(b.f, data);
  return 0;
}

}  // namespace cbor_object_ser

/*
 */
void test_into_object()
{
  {
    cbor::cbor_object cbor_representation;
    unsigned int input{ 2 };
    Data result = { 0x02 };
    //  Data cbor_representation;
    to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }

  {
    cbor::cbor_object cbor_representation;
    cbor_object_ser::Buz z{ 2 };
    Data result = { 0x02 };
    //  Data cbor_representation;
    cbor::to_cbor(z, cbor_representation);  //<--- todo: We lost this one :(
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }

  {
    unsigned int input{ 2 };
    Data result = { 0x02 };
    Data cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    cbor::cbor_object cbor_reconstruct;
    const Data cbor_repr = cbor_representation;
    cbor::from_cbor(cbor_reconstruct, cbor_repr);
    test(cbor::hexdump(cbor_reconstruct.serialized_), cbor::hexdump(cbor_representation));
  }

  {
    std::vector<unsigned int> input{ 1, 2 };
    Data result = { 0x82, 0x01, 0x02 };
    cbor::cbor_object cbor_representation;
    cbor::to_cbor(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));

    cbor::cbor_object cbor_res;
    const Data cbor_data = cbor_representation.serialized_;
    cbor::from_cbor(cbor_res, cbor_data);
    test(cbor::hexdump(cbor_res.serialized_), cbor::hexdump(cbor_data));
  }

  {
    // test non homogeneous.
    std::vector<cbor::cbor_object> z;

    z.resize(3);
    unsigned int x = 1337;
    double f = 13.37;
    std::map<int, cbor::cbor_object> y;
    cbor::cbor_object my_three;
    std::string foo = "foo";
    cbor::to_cbor(foo, my_three);
    y[-3] = my_three;
    cbor::to_cbor(x, z[0]);
    cbor::to_cbor(f, z[1]);
    cbor::to_cbor(y, z[2]);
    Data repr;

    cbor::to_cbor(z, repr);
    cbor::cbor_object cbor_res;
    Data cbor_data = repr;
    cbor::from_cbor(cbor_res, cbor_data);
    test(cbor::hexdump(cbor_res.serialized_), cbor::hexdump(cbor_data));
    std::cout << cbor_res.prettyPrint() << std::endl;
  }
}

namespace trait_select_test
{
template <typename T, typename T2 = void>
struct adapter;

template <typename T>
struct adapter<T, typename std::enable_if_t<std::is_same<T, bool>::value, void>>
{
  const int x = 3;
};

template <typename T>
struct adapter<T, typename std::enable_if_t<std::is_same<T, int>::value, void>>
{
  const int x = 5;
};

void test2()
{  //  type_dispatch z;
  //  std::cout << type_name<decltype(z.signed_integer)>() << std::endl;
  //  std::cout << type_name<decltype(z.signed_integer.value)>() << std::endl;
  adapter<int> f;
  std::cout << f.x << std::endl;
  adapter<bool> b;
  std::cout << b.x << std::endl;
}

// Make all template arguments const!
template <typename T>
using X = typename std::decay<T>::type;

void test()
{
  {
    using A = const unsigned char*;
    std::cout << "base:" << type_name<A>() << "      " << type_name<X<A>>() << std::endl;
    using B = const unsigned char*&;
    std::cout << "base:" << type_name<B>() << "      " << type_name<X<B>>() << std::endl;
    using C = unsigned char*;
    std::cout << "base:" << type_name<C>() << "      " << type_name<X<C>>() << std::endl;
    using D = unsigned char*&;
    std::cout << "base:" << type_name<D>() << "      " << type_name<X<D>>() << std::endl;
  }

  std::cout << std::endl;
  using E = Data;
  std::cout << "base:" << type_name<E>() << "     " << type_name<X<E>>() << std::endl;
  using F = Data&;
  std::cout << "base:" << type_name<F>() << "      " << type_name<X<F>>() << std::endl;
  using G = const Data&;
  std::cout << "base:" << type_name<G>() << "      " << type_name<X<G>>() << std::endl;
  std::cout << std::endl;

  {
    using A = const unsigned char[3];
    std::cout << "base:" << type_name<A>() << "      " << type_name<X<A>>() << std::endl;
    using B = const unsigned char[3];
    std::cout << "base:" << type_name<B>() << "      " << type_name<X<B>>() << std::endl;
    using C = unsigned char[3];
    std::cout << "base:" << type_name<C>() << "      " << type_name<X<C>>() << std::endl;
    using D = unsigned char[3];
    std::cout << "base:" << type_name<D>() << "      " << type_name<X<D>>() << std::endl;
  }
}

}  // namespace trait_select_test

int main(int /* argc */, char** /* argv */)
{
  test_pod();
  test_stl();
  test_associatives();
  test_array();
  test_adl();
  test_into_object();
  trait_select_test::test();

  return failed;
}