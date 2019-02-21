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

template <typename T>
struct always_add_const
{
  using type = const T;
};

template <typename T>
struct always_add_const<T&>
{
  using type = const T&;
};

// Make all template arguments const!
template <typename... Ts>
struct const_read_adapter
{
  using type = read_adapter<typename always_add_const<Ts>::type...>;
};



// Something to dispatch types to their appropriate trait.

struct trait_families
{
  using type = uint8_t;
  using orphan = std::integral_constant<type, 0>;
  using signed_integer = std::integral_constant<type, 1>;
  using unsigned_integer = std::integral_constant<type, 2>;
  using floating_point = std::integral_constant<type, 3>;
  using max = std::integral_constant<type, 4>;
};

// Base trait selector class.
template <trait_families::type, typename T>
struct trait_selector
{
};

template <typename T>
struct trait_selector<0, T>
{
  static const bool applies = true;
  using Type = T;
  using Family = T;
  using Trait = detail::traits<Type>;
};


// Family selectors.
template <typename T>
struct trait_selector<trait_families::signed_integer::value, T>
{
  using Type = T;
  using Family = trait_families::signed_integer;
  using Trait = detail::traits<trait_families::signed_integer, T>;
  static const bool applies = std::is_signed<T>::value && std::is_integral<T>::value && !std::is_same<T, bool>::value;
};

template <typename T>
struct trait_selector<trait_families::unsigned_integer::value, T>
{
  using Type = T;
  using Family = trait_families::unsigned_integer;
  using Trait = detail::traits<trait_families::unsigned_integer, T>;
  static const bool applies = std::is_unsigned<T>::value && std::is_integral<T>::value && !std::is_same<T, bool>::value;
};

template <typename T>
struct trait_selector<trait_families::floating_point::value, T>
{
  using Type = T;
  using Family = trait_families::floating_point;
  using Trait = detail::traits<trait_families::floating_point, T>;
  static const bool applies = std::is_floating_point<T>::value;
};


// Recursively calling templated class to find the approprpiate family, or fall through.
template <typename T, uint8_t index = trait_families::max::value-1>
struct trait_dispatcher
{
  using Type = typename std::conditional<trait_selector<index, T>::applies, trait_selector<index, T>,
                                              typename trait_dispatcher<T, index - 1>::Type>::type;
};

template <typename T>
struct trait_dispatcher<T, 0>
{
  using Type = trait_selector<0, T>;
};



}  // namespace detail
}  // namespace cbor
