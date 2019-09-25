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
#pragma once

using Data = std::vector<std::uint8_t>;

#include <cxxabi.h>
#include <cstdlib>
#include <memory>
#include <string>
#include <type_traits>
#include <typeinfo>

bool failed = false;
std::size_t test_done = 0;

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

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& t)
{
  for (const auto& k : t)
  {
    os << k << " ";
  }
  return os;
}

template <typename K, typename V>
std::ostream& operator<<(std::ostream& os, const std::map<K, V>& t)
{
  for (const auto& k_v : t)
  {
    os << "(" << k_v.first << ", " << k_v.second << ")"
       << " ";
  }
  return os;
}

template <typename A, typename B>
void test(const A& a, const B& b, bool print = true)
{
  test_done++;
  if (a != b)
  {
    std::cerr << "\033[31m"
              << "a (" << a << ") != b (" << b << ")"
              << "\033[0m" << std::endl;
    failed = true;
  }
  else
  {
    if (print)
    {
      std::cerr << "\033[32m"
                << "a (" << a << ") == b (" << b << ")"
                << "\033[0m" << std::endl;
    }
  }
}

Data hexToData(std::string s)
{
  Data res;
  if ((s.size() % 2) != 0)
  {
    throw std::runtime_error("Hex string must be an even length");
  }
  for (std::size_t i = 0; i < (s.size() / 2); i++)
  {
    std::string thing;
    thing.resize(2);
    thing[0] = s[i * 2];
    thing[1] = s[i * 2 + 1];
    res.push_back(static_cast<Data::value_type>(std::strtoll(thing.data(), nullptr, 16)));
  }
  return res;
}

template <typename A, typename B>
void test_result(const cbor::result& res, const A& result, const B& expected)
{
  test(bool(res), true, false);
  test(std::size_t(res), result.size(), false);
  test(std::size_t(res), expected.size(), false);
  test(cbor::hexdump(result), cbor::hexdump(expected));
}

template <typename A>
void test_result(const cbor::result& res, const A& result)
{
  test(bool(res), true, false);
  test(std::size_t(res), result.size(), false);
}

template <typename Type>
void test_appendix_A_decode(const std::string& hex, const Type expected, bool roundtrip)
{
  // decode output
  Type decoded;
  const Data cbor_in = hexToData(hex);
  std::cout << cbor::hexdump(cbor_in) << std::endl;
  auto res = cbor::from_cbor(decoded, cbor_in);
  test_result(res, cbor_in);
  test(decoded, expected);

  if (roundtrip)
  {
    Data cbor_out;
    res = cbor::to_cbor(decoded, cbor_out);
    test_result(res, cbor_out, cbor_in);
  }
}

template <typename T, typename Output = T>
void tester(T value, const Data& expected)
{
  std::cout << "Testing: " << type_name<T>() << " with: " << value << std::endl;

  // encode value into cbor.
  const T input{ value };
  Data cbor_representation;
  auto res = cbor::to_cbor(input, cbor_representation);
  test_result(res, cbor_representation, expected);

  // decode back
  Output output;
  res = cbor::from_cbor(output, cbor_representation);
  test_result(res, cbor_representation);
  test(output, input);
}

template <typename Error, typename Fun>
void expect_error(Fun&& f)
{
  bool have_exception = false;
  try
  {
    f();
  }
  catch (const Error& e)
  {
    have_exception = true;
    std::cout << "Caught exception: " << e.what() << std::endl;
  }
  test(have_exception, true, false);
}
