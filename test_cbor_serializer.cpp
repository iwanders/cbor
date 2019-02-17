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
#include <array>
#include <iostream>
#include <vector>
#include "cbor/stl.h"
#include "cbor/cbor.h"

using Data = std::vector<std::uint8_t>;

template <typename A, typename B>
void test(const A& a, const B& b)
{
  if (a != b)
  {
    std::cerr << "\033[31m" << "a (" << a << ") != b (" << b << ")" << "\033[0m" << std::endl;
    exit(1);
  }
  else
  {
    std::cerr << "\033[32m" << "a (" << a << ") == b (" << b << ")" << "\033[0m" << std::endl;
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
    cbor::serialize(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }
  // test string map
  {
    std::map<std::string, std::string> input{ { "a", "A" }, { "b", "B" }, { "c", "C" }, { "d", "D" }, { "e", "E" } };
    Data result = { 0xa5, 0x61, 0x61, 0x61, 0x41, 0x61, 0x62, 0x61, 0x42, 0x61, 0x63,
                    0x61, 0x43, 0x61, 0x64, 0x61, 0x44, 0x61, 0x65, 0x61, 0x45 };
    Data cbor_representation;
    cbor::serialize(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }
}
void test_stl()
{
  // test tuple
  {
    std::tuple<unsigned int, unsigned int> input{ 1, 2 };
    Data result = { 0x82, 0x01, 0x02 };
    Data cbor_representation;
    cbor::serialize(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }

  // test pair
  {
    std::pair<unsigned int, unsigned int> input{ 1, 2 };
    Data result = { 0x82, 0x01, 0x02 };
    Data cbor_representation;
    cbor::serialize(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }
}


void test_array()
{
  unsigned int input{2};
  Data result = {0x02};
  std::array<cbor::DataType, 100> z;
  std::size_t len = cbor::serialize(input, z.data(), z.size());
  test(cbor::hexdump(result), cbor::hexdump(z, len));
}


namespace foo
{
struct Bar
{
  std::uint32_t f;
};

template <typename ...Data>
std::size_t to_cbor(const Bar& b, cbor::detail::data_adapter<Data...> data)
{
  std::cout << "to cbor adl" << std::endl;
  to_cbor(b.f, data);
  return 0;
}
}
void test_adl()
{
  {
    foo::Bar input{2};
    Data result = {0x02};
    Data cbor_representation;
    cbor::serialize(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }
  {
    std::vector<foo::Bar> input = {foo::Bar{2}, foo::Bar{3}, foo::Bar{4}};
    Data result = {0x83, 0x02, 0x03, 0x04};
    Data cbor_representation;
    cbor::serialize(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }
}

void test_pod()
{
  {
    unsigned int input{2};
    Data result = {0x02};
    Data cbor_representation;
    cbor::serialize(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }
}

namespace cbor_object_ser
{

struct Buz
{
  std::uint32_t f;
};

std::size_t to_cbor(const Buz& b, cbor::cbor_object& data)
{
  std::cout << "to cbor_object_ser adl " << std::endl;
  //  to_cbor(b.f, data);
  cbor::serialize(b.f, data);
  return 0;
}

} // namespace cbor_object_ser

/*
*/
void test_into_object()
{
  {
    cbor::cbor_object cbor_representation;
    unsigned int input {2};
    Data result = {0x02};
    //  Data cbor_representation;
    cbor::serialize(input, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }
  {
    cbor::cbor_object cbor_representation;
    cbor_object_ser::Buz z{2};
    Data result = {0x02};
    //  Data cbor_representation;
    cbor::serialize(z, cbor_representation);   //<--------------------------
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }
}

namespace thing_test
{
template <typename T>
struct z
{
  using Type = void;
  template<typename D>
  static void c(D&){};
};

template<>
struct z<std::uint32_t>
{
  using Type = std::uint32_t;
  template<typename D>
  static std::uint32_t c(D&){};
};


template<typename T>
using has_trait_helper = std::is_same<typename z<T>::Type, T>;
template <typename T>
using not_handled = typename std::enable_if<!has_trait_helper<T>::value, bool>;
template <typename T>
using handled = typename std::enable_if<has_trait_helper<T>::value, bool >;


template <typename T, typename... Data, typename not_handled<T>::type = 0>
void dispatch(T& x, Data...)
{
  std::cout << "Fallback: " << x << std::endl;
}

template <typename T, typename... Data, typename handled<T>::type = 0>
void dispatch(T& x, Data...)
{
  std::cout << "specific is specific: " << x << std::endl;
}


void foo()
{
  std::uint32_t f32{32};
  std::uint16_t f16{16};

  int x = 0;
  dispatch(f32, x);
  dispatch(f16, x);
}
}

int main(int /* argc */, char** /* argv */)
{
  test_pod();
  test_stl();
  test_associatives();
  test_array();
  test_adl();
  test_into_object();

  return 0;
}