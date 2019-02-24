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
#include <map>

// Check if exceptions can be disabled for use on embedded systems.
#define CBOR_USE_EXCEPTIONS 0
#include "cbor/cbor.h"
#include "cbor/stl.h"


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

bool failed = false;

template <typename A, typename B>
void test(const A& a, const B& b, bool print = true)
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
    if (print)
    {
      std::cerr << "\033[32m"
                << "a (" << a << ") == b (" << b << ")"
                << "\033[0m" << std::endl;
    }
  }
}


int main(int /* argc */, char** /* argv */)
{
  const bool no_print = false;

  { // test read buffer too small.
    std::string input{ "foo" };
    Data expected = { 0x63, 0x66, 0x6F};
    std::string output;
    auto res = cbor::from_cbor(output, expected);
    test(bool(res), false, no_print);
  }

  {  // test write buffer too small.
    std::array<std::uint8_t, 2> out;
    std::string input{ "foo" };
    auto res = cbor::to_cbor(input, out.data(), out.size());
    test(bool(res), false, no_print);
  }

  {  // test write buffer too small.
    std::array<std::uint8_t, 2> out;
    std::vector<std::uint32_t> input{1,2,3};
    auto res = cbor::to_cbor(input, out.data(), out.size());
    test(bool(res), false, no_print);
  }

  { // test reading wrong type.
    Data cbor_in = { 0x63, 0x66, 0x6F};
    std::vector<std::uint32_t> parsed;
    auto res = cbor::from_cbor(parsed, cbor_in);
    test(bool(res), false, no_print);
  }

  {  // test fail on size type.
    Data cbor_in = { 0x19, 0x03, 0xe8};
    std::uint8_t parsed;
    auto res = cbor::from_cbor(parsed, cbor_in);
    test(bool(res), false, no_print);
  }

  if (failed)
  {
    std::cerr << "\033[31m"
              << "FAIL"
              << "\033[0m" << std::endl;
  }
  else
  {
    std::cerr << "\033[32m"
              << "Success"
              << "\033[0m" << std::endl;
  }
  return failed;
}