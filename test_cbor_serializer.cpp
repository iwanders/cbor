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
#include "include/cbor.h"
#include "include/stl.h"

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

int main(int /* argc */, char** /* argv */)
{
  //! Timepoint of that clock.
  using TimePoint = uint64_t;
  //! Trace event as it is stored in the ringbuffer.
  using ScopeTraceEvent = std::tuple<TimePoint, unsigned int, uint8_t>;
  //! The container that backs the ringbuffer.
  using EventContainer = std::vector<ScopeTraceEvent>;
  using EventMap = std::map<unsigned long, EventContainer>;

/*

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

*/
  unsigned int input{2};
  Data result = {0x02};
  Data cbor_representation;
  cbor::serialize(input, cbor_representation);
  test(cbor::hexdump(result), cbor::hexdump(cbor_representation));



  std::array<cbor::DataType, 100> z;
  std::size_t len = cbor::serialize(input, z.data(), z.size());
  test(cbor::hexdump(result), cbor::hexdump(z, len));
  return 0;
}