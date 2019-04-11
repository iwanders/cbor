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
//  #define CBOR_USE_EXCEPTIONS 0
#include "cbor/stl.h"
#include "cbor/shortfloat.h"

bool failed = false;
std::size_t test_done = 0;

using Data = std::vector<std::uint8_t>;


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
    res.push_back(std::strtoll(thing.data(), nullptr, 16));
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



double rfc_decode(const unsigned char* halfp)
{
  int half = (halfp[1] << 8) + halfp[0];
  int exp = (half >> 10) & 0x1f;
  int mant = half & 0x3ff;
  double val = 0;
  if (exp == 0)
  {
    val = std::ldexp(mant, -24);
  }
  else if (exp != 31)
  {
    val = std::ldexp(mant + 1024, exp - 25);
  }
  else
  {
    val = (mant == 0) ? std::numeric_limits<double>::infinity() : std::numeric_limits<double>::quiet_NaN();
  }
  return half & 0x8000 ? -val : val;
}

void test_half_precision_float()
{
  //  return;
  std::cout << "0.0" << std::endl;
  test(cbor::shortfloat::encode(0.0), 0x0000);
  test(cbor::shortfloat::encode(std::pow(2, -24)), 0x0001);
  std::cout << "-0.0" << std::endl;
  test(cbor::shortfloat::encode(-0.0), 0x8000);
  std::cout << "1.0" << std::endl;
  test(cbor::shortfloat::encode(1.0), 0x3c00);
  std::cout << "-2.0" << std::endl;
  test(cbor::shortfloat::encode(-2.0), 0xc000);
  test(cbor::shortfloat::encode(std::numeric_limits<double>::infinity()), 0x7c00);
  test(cbor::shortfloat::encode(-std::numeric_limits<double>::infinity()), 0xfc00);
  test(cbor::shortfloat::encode(std::numeric_limits<double>::quiet_NaN()), 0x7e00);

  std::uint32_t score = 0;
  for (std::size_t i = 0; i < (1<<16); i++)
  {
    std::uint16_t ui = i;
    float v = cbor::shortfloat::decode(ui);
    std::uint16_t enc = cbor::shortfloat::encode(v);
    score += (i == enc);
    //  test(i, enc);
    //  std::cout << "v: " << v << " enc: " << enc << " i: " << i <<   "   ==? " << (i == enc) << std::endl;
  }
  test(score, 65536U);

  std::uint32_t nan_or_inf = 0;
  std::uint32_t value_correct = 0;
  std::uint32_t value_incorrect = 0;
  for (std::size_t i = 0; i < (1<<16); i++)
  {
    std::uint16_t ui = i;
    float v = cbor::shortfloat::decode(ui);
    if (std::isnan(v) || !std::isfinite(v))
    {
      nan_or_inf++;
      continue;
    }
    double d = rfc_decode(reinterpret_cast<const unsigned char*>(&ui));
    if (d == v)
    {
      value_correct += 1;
      std::cout << " i: " << i << "v: " << v << " d: " << d << " diff: " << (d - v) << std::endl;
    }
    else
    {
      value_incorrect += 1;
    }
  }
  std::cout << "Value correct: " << value_correct << std::endl;
  std::cout << "value_incorrect: " << value_incorrect << std::endl;
  std::cout << "nan or inf: " << nan_or_inf << std::endl;


}

int main(int /* argc */, char** /* argv */)
{
  test_half_precision_float();
 
  #ifdef CBOR_SUPPORT_SHOTRFLOAT
  std::cerr << "CBOR_SUPPORT_SHOTRFLOAT" << std::endl;
  #endif

  if (failed)
  {
    std::cerr << "\033[31m"
              << "FAIL"
              << "\033[0m" << std::endl;
  }
  else
  {
    std::cerr << "\033[32m"
              << "Success fully passed " << test_done << " tests."
              << "\033[0m" << std::endl;
  }
  return failed;
}