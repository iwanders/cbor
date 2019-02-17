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
std::string hexdump(const cbor_object& d)
{
  std::stringstream ss;
  for (const auto& v : d.serialized_)
  {
    ss << "" << std::setfill('0') << std::setw(2) << std::hex << int{ v } << " ";
  }
  return ss.str();
}


namespace detail
{

template <>
struct write_adapter<Data&>
{
  Data& data;
  write_adapter<Data&>(Data& d) : data{d} {};

  void resize(std::size_t value)
  {
    data.resize(value);
  }
  std::size_t size() const
  {
    return data.size();
  }
  DataType& operator[](std::size_t pos)
  {
    return data[pos];
  }
};

template <>
struct read_adapter<const Data&>
{
  const Data& data;
  std::size_t cursor = 0;
  read_adapter<const Data&>(const Data& d) : data{d} {};
  std::size_t position() const
  {
    return cursor;
  }
  void advance(std::size_t count)
  {
    cursor += count;
    if (cursor >= data.size())
    {
      // Throw!?
    }
  }
  std::size_t size() const
  {
    return data.size();
  }
  const DataType& operator[](std::size_t pos) const
  {
    if (pos >= data.size())
    {
      // Throw!?
    }
    return data[pos];
  }
};

template <>
struct write_adapter<cbor_object&>
{
  cbor_object& o;
  write_adapter<cbor_object&>(cbor_object& d) : o{d} {};

  void resize(std::uint32_t value)
  {
    o.serialized_.resize(value);
  }
  std::uint32_t size() const
  {
    return o.serialized_.size();
  }
  DataType& operator[](std::size_t pos)
  {
    return o.serialized_[pos];
  }

  // Allow unwrapping the adapter from the object...
  operator cbor_object&()
  {
    return o;
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
    return to_cbor(v.c_str(), data);
  }
  template <typename Data>
  static std::size_t deserializer(Type& v, Data& data)
  {
    std::uint64_t string_length;
    std::size_t len = deserializeInteger(0b011, string_length, data);
    v.clear();
    v.insert(v.begin(), &(data[data.position()]), &(data[data.position()]) + string_length);
    data.advance(string_length);
    return len + string_length;
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
    addition += serializeItem(0b100, v.size(), data);
    for (const auto& k : v)
    {
      addition += to_cbor(k, data);
    }
    return addition;
  }
  template <typename Data>
  static std::size_t deserializer(Type& v, Data& data)
  {
    std::uint8_t first_byte;
    std::uint64_t length;
    std::size_t len = deserializeItem(first_byte, length, data);
    if ((first_byte >> 5) == 0b100)
    {
      // it is a list.
      if ((first_byte & 0b11111) == 31)
      {
        // todo indefinite length...
      }
      else
      {
        v.reserve(length);
        for (std::size_t i = 0; i < length; i++)
        {
          typename Type::value_type tmp;
          len += from_cbor(tmp, data);
          v.emplace_back(std::move(tmp));
        }
      }
    }
    return len;
  }
};

/**
 * Helper for serializing tuples, recurses down through index.
 */
template <size_t index, typename... Ts>
struct trait_tuple_element_helper
{
  template <typename Data>
  static std::size_t serialize(const std::tuple<Ts...>& t, Data& data)
  {
    std::size_t value = to_cbor(std::get<sizeof...(Ts) - index>(t), data);
    value += trait_tuple_element_helper<index - 1, Ts...>::serialize(t, data);
    return value;
  }

  template <typename Data>
  static std::size_t deserialize(std::tuple<Ts...>& t, Data& data)
  {
    std::size_t value = from_cbor(std::get<sizeof...(Ts) - index>(t), data);
    value += trait_tuple_element_helper<index - 1, Ts...>::deserialize(t, data);
    return value;
  }
};

/**
 * Recursion terminator at index 0.
 */
template <typename... Ts>
struct trait_tuple_element_helper<0, Ts...>
{
  template <typename Data>
  static std::size_t serialize(const std::tuple<Ts...>& /*t*/, Data& /*data*/)
  {
    return 0;
  }
  template <typename Data>
  static std::size_t deserialize(std::tuple<Ts...>& /*t*/, Data& /*data*/)
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
  using Type = std::tuple<Ts...>;
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    std::size_t addition = 0;
    addition += serializeItem(0b100, sizeof...(Ts), data);
    return addition + trait_tuple_element_helper<sizeof...(Ts), Ts...>::serialize(v, data);
  }

  template <typename Data>
  static std::size_t deserializer(Type& v, Data& data)
  {
    std::uint8_t first_byte;
    std::uint64_t length;
    std::size_t len = deserializeItem(first_byte, length, data);
    if (length != sizeof...(Ts))
    {
      // todo incorrect tuple length.
    }
    if ((first_byte >> 5) == 0b100)
    {
      return len + trait_tuple_element_helper<sizeof...(Ts), Ts...>::deserialize(v, data);
    }
    else
    {
      // todo incorrect type.
    }
    return len;
  }
};

/**
 * Specialization for std::pair
 */
template <typename A, typename B>
struct traits<std::pair<A, B>>
{
  using Type = std::pair<A, B>;
  template <typename Data>
  static std::size_t serializer(const std::pair<A, B>& v, Data& data)
  {
    std::size_t addition = 0;
    addition += serializeItem(0b100, uint8_t{ 2 }, data);
    addition += to_cbor(v.first, data);
    addition += to_cbor(v.second, data);
    return addition;
  }

  template <typename Data>
  static std::size_t deserializer(Type& v, Data& data)
  {
    std::uint8_t first_byte;
    std::uint64_t length;
    std::size_t len = deserializeItem(first_byte, length, data);
    if (first_byte == ((0b100 << 5)| 2))
    {
      len += from_cbor(v.first, data);
      len += from_cbor(v.second, data);
    }
    else
    {
      // todo type incorrect
    }
    return len;
  }
};

/**
 * Specialization for std::map.
 */
template <typename KeyType, typename ValueType>
struct traits<std::map<KeyType, ValueType>>
{
  using Type = std::map<KeyType, ValueType>;
  template <typename Data>
  static std::size_t serializer(const std::map<KeyType, ValueType>& v, Data& data)
  {
    std::size_t addition = 0;
    addition += serializeItem(0b101, v.size(), data);
    for (const auto& k_v : v)
    {
      const auto& key = k_v.first;
      const auto& value = k_v.second;
      addition += to_cbor(key, data);
      addition += to_cbor(value, data);
    }
    return addition;
  }

  template <typename Data>
  static std::size_t deserializer(Type& v, Data& data)
  {
    std::uint8_t first_byte;
    std::uint64_t length;
    std::size_t len = deserializeItem(first_byte, length, data);
    if ((first_byte >> 5) == 0b101)
    {
      // it is a map!
      if ((first_byte & 0b11111) == 31)
      {
        // todo indefinite length...
      }
      else
      {
        for (std::size_t i = 0; i < length; i++)
        {
          typename Type::key_type key;
          typename Type::mapped_type value;
          len += from_cbor(key, data);
          len += from_cbor(value, data);
          v.emplace(std::move(key), std::move(value));
        }
      }
    }
    return len;
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
