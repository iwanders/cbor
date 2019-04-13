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

#include "shortfloat.h"
// From "Fast Half Float Conversions" by Jeroen van der Zijp, November 2008, (Revised September 2010)
// fasthalffloatconversion.pdf

namespace cbor
{
namespace shortfloat
{

struct Table
{
  // decode
  std::uint32_t mantissatable[2048];
  std::uint32_t exponenttable[64];
  std::uint32_t offsettable[64];

  // encode
  std::uint16_t basetable[512];
  std::uint8_t shifttable[512];

  static unsigned int convertmantissa(std::uint32_t i)
  {
    std::uint32_t m = i << 13;// Zero pad mantissa bits
    std::uint32_t e = 0; // Zero exponent

    while(!(m & 0x00800000)) // While not normalized
    {
      e -= 0x00800000; // Decrement exponent (1<<23)
      m <<= 1;// Shift mantissa
    }
    m &= ~0x00800000;// Clear leading 1 bit
    e += 0x38800000;// Adjust bias ((127-14)<<23)
    return m | e;// Return combined number
  }

  void compute()
  {
    mantissatable[0] = 0;
    for (std::uint32_t i = 1; i < 2048; i++)
    {
      if (i < 1024)
      {
        mantissatable[i] = convertmantissa(i);
      }
      else
      {
        mantissatable[i] = 0x38000000 + ((i - 1024) << 13);
      }
    }
    exponenttable[0] = 0;
    exponenttable[32]= 0x80000000;
    for (std::uint32_t i = 1; i < 31; i++)
    {
      exponenttable[i] = i << 23;
    }

    for (std::uint32_t i = 33; i < 63; i++)
    {
      exponenttable[i] = 0x80000000 + ((i - 32) << 23);
    }

    exponenttable[31]= 0x47800000;
    exponenttable[63]= 0xC7800000;

    for (std::uint32_t i = 0; i < 64; i++)
    {
      offsettable[i] = 1024;
    }
    offsettable[0] = 0;
    offsettable[32] = 0;

    // encode
    for (std::uint32_t i = 0; i < 256; i++)
    {
      std::int32_t e = i - 127;
      if (e < -24)
      {
        // Very small numbers map to zero
        basetable[i|0x000] = 0x0000;
        basetable[i|0x100] = 0x8000;
        shifttable[i|0x000] = 24;
        shifttable[i|0x100] = 24;
      }
      else if (e < -14)
      {
        // Small numbers map to denorms
        basetable[i | 0x000] = (0x0400 >> (-e - 14));
        basetable[i | 0x100] = (0x0400 >> (-e - 14)) | 0x8000;
        shifttable[i | 0x000] = -e - 1;
        shifttable[i | 0x100] = -e - 1;
      }
      else if (e <= 15)
      {
        // Normal numbers just lose precision
        basetable[i | 0x000] = ((e + 15) << 10);
        basetable[i | 0x100] = ((e + 15) << 10) | 0x8000;
        shifttable[i | 0x000] = 13;
        shifttable[i | 0x100] = 13;
      }
      else if (e < 128)
      {
        // Large numbers map to Infinity
        basetable[i | 0x000] = 0x7C00;
        basetable[i | 0x100] = 0xFC00;
        shifttable[i | 0x000] = 24;
        shifttable[i | 0x100] = 24;
      }
      else
      {
        // Infinity and NaN's stay Infinity and NaN's
        basetable[i | 0x000] = 0x7C00;
        basetable[i | 0x100] = 0xFC00;
        shifttable[i | 0x000] = 13;
        shifttable[i | 0x100] = 13;
      }
    }
  }

  Table()
  {
    compute();
  }
  static Table& getTables()
  {
    static Table z;
    return z;
  }
};

std::uint16_t encode(const float f_in)
{
  static const auto& t = Table::getTables();
  const std::uint32_t& f = *reinterpret_cast<const std::uint32_t*>(&f_in);
  return t.basetable[(f >> 23) & 0x1ff] + ((f & 0x007fffff) >> t.shifttable[(f >> 23) & 0x1ff]);
}

float decode(const std::uint16_t h)
{
  static const auto& t = Table::getTables();
  std::uint32_t z = t.mantissatable[t.offsettable[h >> 10] + (h & 0x3ff)] + t.exponenttable[h >> 10];
  return *reinterpret_cast<const float*>(&z);
}

}
}
