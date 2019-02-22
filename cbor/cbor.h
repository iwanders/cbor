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
#include <limits>
#include "pod.h"
#include "traits.h"
#include "util.h"

namespace cbor
{

namespace detail
{
template <typename T, typename... Data, std::enable_if_t<has_trait<T>::value, int> = 0>
std::size_t to_cbor(const T& v, detail::write_adapter<Data...>& data)
{
  return detail::trait_dispatcher<T>::Type::Trait::serializer(v, data);
}


template <typename T, typename... Data, std::enable_if_t<has_trait<T>::value, int> = 0>
std::size_t from_cbor(T& v, detail::read_adapter<Data...>& data)
{
  return detail::trait_dispatcher<T>::Type::Trait::deserializer(v, data);
}
}  // namespace detail


/**
 * @brief Entry to the trait system, serializes v into data.
 * @param v The data to serialize.
 * @param data The data vector to serialize into.
 */
template <typename T, typename Arg0, typename... ArgN, std::enable_if_t<detail::get_write_adapter<Arg0>::value, int> = 0>
std::size_t to_cbor(const T& v, Arg0&& arg0, ArgN&&... argn)
{
  using write_adapter = detail::get_write_adapter<Arg0>;
  auto wrapper = write_adapter::adapt(std::forward<Arg0>(arg0), std::forward<ArgN>(argn)...);
  return to_cbor(v, wrapper);
}

template <typename T, typename Arg0, typename... ArgN, std::enable_if_t<detail::get_read_adapter<Arg0>::value, int> = 0>
std::size_t from_cbor(T& v, Arg0&& arg0, ArgN&&... argn)
{
  using read_adapter = detail::get_read_adapter<Arg0>;
  auto wrapper = read_adapter::adapt(std::forward<Arg0>(arg0), std::forward<ArgN>(argn)...);
  return from_cbor(v, wrapper);
}
}  // namespace cbor
