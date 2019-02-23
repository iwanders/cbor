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
 * @brief Main trait struct for types.
 */
template <typename T, typename = void>
struct traits;

/**
 * @brief Structs for the read and write adapters.
 */
template <typename... Data>
struct write_adapter : std::false_type
{
};
template <typename... Data>
struct read_adapter : std::false_type
{
};

// Retrieval helpers.
template <typename T>
using get_read_adapter = read_adapter<typename std::decay<T>::type>;

template <typename T>
using get_write_adapter = write_adapter<typename std::decay<T>::type>;

// Something to dispatch types to their appropriate trait.
struct trait_families
{
  static const uint8_t expansion_slots = 5;
  using type = uint8_t;
  using orphan = std::integral_constant<type, 0>;
  using signed_integer = std::integral_constant<type, 1>;
  using unsigned_integer = std::integral_constant<type, 2>;
  using floating_point = std::integral_constant<type, 3>;
  using c_array_family = std::integral_constant<type, 4>;
  using real_max = std::integral_constant<type, 5>;
  using max = std::integral_constant<type, real_max::value + expansion_slots>;
};

// Base trait selector class.
template <trait_families::type, typename T>
struct trait_selector
{
  static const bool applies = false;  // defaults to apply false to iterate to the next entry.
};

// Catch all trait in case a type doesn't belong to any family.
template <typename T>
struct trait_selector<trait_families::orphan::value, T>
{
  static const bool applies = true;
  using Type = T;
  using Family = T;
  using Trait = detail::traits<Type>;
};

// Family selectors, this allows treating multiple types using the same handlers.
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
struct trait_selector<trait_families::floating_point::value, T>  // double and float
{
  using Type = T;
  using Family = trait_families::floating_point;
  using Trait = detail::traits<trait_families::floating_point, T>;
  static const bool applies = std::is_floating_point<T>::value;
};

template <typename T>
struct trait_selector<trait_families::c_array_family::value, T>  // any c style array, like; int x[3]; or Foo x[5];
{
  using Type = T;
  using Family = trait_families::c_array_family;
  using Trait = detail::traits<trait_families::c_array_family, T>;
  static const bool applies = std::is_array<T>::value;
};

// Recursively calling templated class to find the approprpiate family, or fall through.
template <typename T, uint8_t index = trait_families::max::value - 1>
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

//
// Some helpers to check whether a type is supported by our traits.
template <typename T>
using has_trait = std::is_default_constructible<typename detail::trait_dispatcher<T>::Type::Trait>;

}  // namespace detail
}  // namespace cbor