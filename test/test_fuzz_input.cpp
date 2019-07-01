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
#include <iostream>

#define CBOR_USE_EXCEPTIONS 0
#include "cbor/stl.h"

int main(int /* argc */, char** /* argv */)
{
  std::vector<cbor::Data> inputs{ { 0x82, 0x82, 0x18, 0x00, 0x00, 0x9F, 0x9F, 0x9F, 0x9F, 0x28, 0x9F, 0x9F, 0x9F, 0x9F,
                                    0x9F, 0x9F, 0x9F, 0xFF, 0xFF, 0x17, 0xFF, 0x9F, 0x9F, 0x9F, 0x9F, 0x9F, 0x5B, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F } };
  //  Position in data: 26 is: 91
  //  Reading string: (success: true, length: 9) length: 18446744073709551607
  //  res after advance(success: true, length: 0)
  //  Position in data: 26 is: 91

  // String length 18446744073709551607 + position = 18446744073709551607 + 26 = 0x10000000000000011L... overflow; fits
  // in data, advance() call is true.

  for (const auto& cbor_in : inputs)
  {
    std::cout << "Trying input: " << cbor::hexdump(cbor_in) << std::endl;
    cbor::cbor_object parsed;
    auto res = cbor::from_cbor(parsed, cbor_in);
    std::cout << "Returned: " << bool(res) << " , " << size_t(res) << std::endl;
  }
  return 0;
}