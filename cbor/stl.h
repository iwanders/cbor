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
#include <map>
#include <tuple>
#include <vector>
#include <iomanip>
#include "util.h"
#include "traits.h"
#include "cbor.h"

namespace cbor
{
using Data = std::vector<DataType>;

std::string hexdump(const Data& d)
{
  std::stringstream ss;
  for (const auto& v : d)
  {
    ss << "" << std::setfill('0') << std::setw(2) << std::hex << int{ v } << " ";
  }
  return ss.str();
}

template <std::size_t Length>
std::string hexdump(const std::array<DataType, Length>& d, std::size_t max_length=Length)
{
  std::stringstream ss;
  for (std::size_t i = 0; i < std::min(Length, max_length); i++)
  {
    ss << "" << std::setfill('0') << std::setw(2) << std::hex << int{ d[i] } << " ";
  }
  return ss.str();
}

/**
 * @brief An object to represent an already serialized compact binary object.
 */
class cbor_object
{
public:
  Data serialized_;
  /**
   * @brief Function to create a cbor_object that represents the value that was passed in during creation.
   */
  template <typename T>
  static cbor_object make(const T& v)
  {
    cbor_object res;
    serialize(v, res.serialized_);
    return res;
  }
};


namespace detail
{

template <>
struct data_adapter<Data&>
{
  Data& data;
  data_adapter<Data&>(Data& d) : data{d} {};

  void resize(std::uint32_t value)
  {
    data.resize(value);
  }
  std::uint32_t size() const
  {
    return data.size();
  }
  DataType& operator[](std::size_t pos)
  {
    return data[pos];
  }
};

/**
 * Specialization for std::string.
 */
template <>
struct traits<std::string>
{
  using Type = std::string;
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    return serialize(v.c_str(), data);
  }
};

/**
 * Specialization for std::vector.
 */
template <typename T>
struct traits<std::vector<T>>
{
  using Type = std::vector<T>;
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    std::size_t addition = 0;
    addition += serializeInteger(0b100, v.size(), data);
    for (const auto& k : v)
    {
      addition += serialize(k, data);
    }
    return addition;
  }
};

/**
 * Helper for serializing tuples, recurses down through index.
 */
template <size_t index, typename... Ts>
struct serialize_tuple_element
{
  template <typename Data>
  static std::size_t execute(const std::tuple<Ts...>& t, Data& data)
  {
    std::size_t value = serialize(std::get<sizeof...(Ts) - index>(t), data);
    value += serialize_tuple_element<index - 1, Ts...>::execute(t, data);
    return value;
  }
};

/**
 * Recursion terminator at index 0.
 */
template <typename... Ts>
struct serialize_tuple_element<0, Ts...>
{
  template <typename Data>
  static std::size_t execute(const std::tuple<Ts...>& /*t*/, Data& /*data*/)
  {
    return 0;
  }
};

/**
 * Specialization for std::tuple.
 */
template <typename... Ts>
struct traits<std::tuple<Ts...>>
{
  template <typename Data>
  static std::size_t serializer(const std::tuple<Ts...>& v, Data& data)
  {
    std::size_t addition = 0;
    addition += serializeInteger(0b100, sizeof...(Ts), data);
    return addition + serialize_tuple_element<sizeof...(Ts), Ts...>::execute(v, data);
  }
};

/**
 * Specialization for std::pair
 */
template <typename A, typename B>
struct traits<std::pair<A, B>>
{
  template <typename Data>
  static std::size_t serializer(const std::pair<A, B>& v, Data& data)
  {
    std::size_t addition = 0;
    addition += serializeInteger(0b100, uint8_t{ 2 }, data);
    addition += serialize(v.first, data);
    addition += serialize(v.second, data);
    return addition;
  }
};

/**
 * Specialization for std::map.
 */
template <typename KeyType, typename ValueType>
struct traits<std::map<KeyType, ValueType>>
{
  template <typename Data>
  static std::size_t serializer(const std::map<KeyType, ValueType>& v, Data& data)
  {
    std::size_t addition = 0;
    addition += serializeInteger(0b101, v.size(), data);
    for (const auto& k_v : v)
    {
      const auto& key = k_v.first;
      const auto& value = k_v.second;
      addition += serialize(key, data);
      addition += serialize(value, data);
    }
    return addition;
  }
};

/**
 * Specialization for cbor_object.
 */
template <>
struct traits<cbor_object>
{
  using Type = cbor_object;
  template <typename Data>
  static std::size_t serializer(const Type& obj, Data& data)
  {
    const auto& v = obj.serialized_;
    std::size_t offset = data.size();
    data.resize(data.size() + v.size());
    for (std::size_t i = 0; i < v.size(); i++)
    {
      data[i + offset] = v[i];
    }
    return v.size();
  }
};
}  // namespace detail
}  // namespace cbor
