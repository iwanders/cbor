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
#include <bitset>
#include <iostream>
#include <vector>
#include "cbor/stl.h"
#include "shortfloat.h"
#include "test.h"

#include <chrono>
#include <random>

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
  using FPH = cbor::detail::trait_floating_point_helper<std::uint16_t>;
  test(FPH::encode(0.0), 0x0000);
  test(FPH::encode(std::pow(2, -24)), 0x0001);
  test(FPH::encode(-0.0), 0x8000);
  test(FPH::encode(1.0), 0x3c00);
  test(FPH::encode(-2.0), 0xc000);
  test(FPH::encode(std::numeric_limits<double>::infinity()), 0x7c00);
  test(FPH::encode(-std::numeric_limits<double>::infinity()), 0xfc00);
  test(FPH::encode(std::numeric_limits<double>::quiet_NaN()), 0x7e00);

  const bool print_off = false;
  for (std::size_t i = 0; i < (1 << 16); i++)
  {
    // Test decodes.
    std::uint16_t half = i;
    float h_to_f_table = shortfloat::decode(half);
    float h_to_f_cbor = FPH::decode(half);
    float h_to_f_rfc = rfc_decode(reinterpret_cast<const unsigned char*>(&half));

    // Own implementation must concur with shortfloat table implementation, which seems de-facto standard.
    test(*reinterpret_cast<const std::uint32_t*>(&h_to_f_table), *reinterpret_cast<const std::uint32_t*>(&h_to_f_cbor),
         print_off);

    // rfc decode doesn't do fraction of nans
    if (std::isnan(h_to_f_table))
    {
      test(std::isnan(h_to_f_table), std::isnan(h_to_f_rfc), print_off);
    }
    else
    {
      test(*reinterpret_cast<const std::uint32_t*>(&h_to_f_table), *reinterpret_cast<const std::uint32_t*>(&h_to_f_rfc),
           print_off);
    }

    // Test encodes, must exactly match the original input value.
    std::uint16_t f_to_h_table = shortfloat::encode(h_to_f_table);
    std::uint16_t f_to_h_cbor = FPH::encode(h_to_f_table);
    test(f_to_h_table, f_to_h_cbor, print_off);
    test(f_to_h_table, half, print_off);
  }
}

void test_complete_conversion_correctness()
{
  using FPH = cbor::detail::trait_floating_point_helper<std::uint16_t>;
  const bool print_off = false;
  constexpr bool handle_out_of_bounds = true;
  // Check if the table and own implementation concur on all possible conversions.
  for (std::size_t i = 0; i < (1UL << 32); i++)
  {
    std::uint32_t float_as_i = i;
    const float f = *reinterpret_cast<const float*>(&float_as_i);
    std::uint16_t f_to_h_table = shortfloat::encode(f);
    std::uint16_t f_to_h_cbor = FPH::encode<handle_out_of_bounds>(f);
    test(f_to_h_table, f_to_h_cbor, print_off);
  }
}

float get_random()
{
  static std::default_random_engine e;
  static std::uniform_real_distribution<> dis(-65535, 65535);
  return dis(e);
}
void compare_speed()
{
  using FPH = cbor::detail::trait_floating_point_helper<std::uint16_t>;
  std::cout << "Making random numbers" << std::endl;
  std::vector<float> v(10000000);
  for (auto& z : v)
  {
    z = get_random();
  }
  std::cout << "Done making random numbers" << std::endl;

  auto t1 = std::chrono::high_resolution_clock::now();

  size_t iterations = 0;
  std::uint16_t encoded;
  float decoded;
  {
    for (std::size_t i = 0; i < 100; i++)
    {
      for (const auto& value : v)
      {
        encoded = FPH::encode(value);
        decoded = FPH::decode(encoded);
        iterations++;
      }
    }
  }
  std::cout << "encoded: " << encoded << " decoded: " << decoded << std::endl;

  auto t2 = std::chrono::high_resolution_clock::now();
  //  double dif = std::chrono::duration_cast<std::chrono::nanoseconds>( t2 - t1 ).count();
  std::chrono::duration<double> elapsed = t2 - t1;
  std::cout << "FPH: time in seconds: " << elapsed.count() << " for " << iterations
            << " per iteration: " << (elapsed.count() / iterations) << std::endl;
  ;

  t1 = std::chrono::high_resolution_clock::now();
  iterations = 0;
  {
    for (std::size_t i = 0; i < 100; i++)
    {
      for (const auto& value : v)
      {
        encoded = shortfloat::encode(value);
        decoded = shortfloat::decode(encoded);
        iterations++;
      }
    }
  }
  t2 = std::chrono::high_resolution_clock::now();
  elapsed = t2 - t1;
  std::cout << "Table: time in seconds: " << elapsed.count() << " for " << iterations
            << " per iteration: " << (elapsed.count() / iterations) << std::endl;
  ;
}

int main(int /* argc */, char** /* argv */)
{
  test_half_precision_float();
  //  test_complete_conversion_correctness();
  compare_speed();

  if (failed)
  {
    std::cerr << "\033[31m"
              << "FAIL"
              << "\033[0m" << std::endl;
  }
  else
  {
    std::cerr << "\033[32m"
              << "Successfully passed " << test_done << " tests."
              << "\033[0m" << std::endl;
  }
  return failed;
}