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
#include <cstdint>

namespace cbor
{
using DataType = std::uint8_t;

namespace detail
{
/**
 * @brief Serializer trait struct
 */
template <typename T, typename = void>
struct traits
{
  template <typename Data>
  static void serialize(const T& /* value */, Data& /* out */)
  {
    static_assert(std::is_same<T, void>::value, "This type is not supported by the cbor serialization.");
  }
};

// some helpers....
template<typename T>
using has_trait_helper = std::is_same<typename traits<T>::Type, T>;
template <typename T>
using not_handled = typename std::enable_if<!has_trait_helper<T>::value, bool>;
template <typename T>
using handled = typename std::enable_if<has_trait_helper<T>::value, bool >;
// To do sfinae; , typename not_handled<T>::type = 0

template <typename Arg0, typename... Data>
struct write_adapter
{
  write_adapter<Arg0, Data...>(...)
  {
    static_assert(std::is_same<Arg0, void>::value, "Write adapter not found for this type.");
  }
};
template <typename Arg0, typename... Data>
struct read_adapter
{
  static read_adapter<Arg0, Data...> adapt(...)
  {
    static_assert(std::is_same<Arg0, bool>::value, "Read adapter not found for this type.");
  }
};
}  // namespace detail
}  // namespace cbor
