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

namespace cbor
{
/**
 * @brief Swap endianness
 */
inline std::uint16_t fixEndianness(const std::uint16_t in)
{
  auto b = reinterpret_cast<const std::uint8_t*>(&in);
  return static_cast<std::uint16_t>((b[0] << 8) | b[1]);
}

/**
 * @brief Swap endianness, reduces to bswap assembly instruction.
 */
inline std::uint32_t fixEndianness(const std::uint32_t in)
{
  auto b = reinterpret_cast<const std::uint8_t*>(&in);
  return static_cast<std::uint32_t>((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]);
}

/**
 * @brief Swap endianness, reduces to bswap assembly instruction.
 */
inline std::uint64_t fixEndianness(const std::uint64_t in)
{
  auto b = reinterpret_cast<const std::uint8_t*>(&in);
  std::uint32_t upper = static_cast<std::uint32_t>(b[0] << 24) | static_cast<std::uint32_t>(b[1] << 16) |
                        static_cast<std::uint32_t>(b[2] << 8) | static_cast<std::uint32_t>(b[3] << 0);
  std::uint32_t lower = static_cast<std::uint32_t>(b[4] << 24) | static_cast<std::uint32_t>(b[5] << 16) |
                        static_cast<std::uint32_t>(b[6] << 8) | static_cast<std::uint32_t>(b[7]);
  return static_cast<std::uint64_t>((static_cast<std::uint64_t>(upper) << (4 * 8)) | lower);
}
}  // namespace cbor
