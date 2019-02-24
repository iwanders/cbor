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
#include <iomanip>
#include <limits>
#include <map>
#include <tuple>
#include <vector>
#include "cbor.h"
#include "traits.h"
#include "util.h"

namespace cbor
{
using Data = std::vector<DataType>;

std::ostream& operator<< (std::ostream &out, const result& res)
{
    std::stringstream ss;
    ss << "(" << std::boolalpha << res.success << ", " << res.length << ")";
    return out << ss.str();
}

namespace detail
{
template <>
struct write_adapter<Data> : std::true_type
{
  Data& data;
  static write_adapter<Data> adapt(Data& d)
  {
    return write_adapter<Data>{ d };
  }
  write_adapter<Data>(Data& d) : data{ d } {};

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
    if (pos >= data.size())
    {
      // todo; throw?
    }
    return data[pos];
  }
};

template <>
struct read_adapter<Data> : std::true_type
{
  const Data& data;
  std::size_t cursor = 0;
  static read_adapter<Data> adapt(const Data& d)
  {
    return read_adapter<Data>{ d };
  }
  read_adapter<Data>(const Data& d) : data{ d } {};

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
}  // namespace detail

std::string hexdump(const DataType* d, std::size_t length)
{
  std::stringstream ss;
  for (std::size_t i = 0; i < length; i++)
  {
    ss << "" << std::setfill('0') << std::setw(2) << std::hex << int{ d[i] } << " ";
  }
  return ss.str();
}

std::string hexdump(const Data& d)
{
  return hexdump(d.data(), d.size());
}

template <size_t Length>
std::string hexdump(const std::array<DataType, Length>& d, std::size_t max_length)
{
  return hexdump(d.data(), std::min(max_length, d.size()));
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

  bool operator<(const cbor_object& a) const
  {
    return serialized_ < a.serialized_;
  }

  std::string prettyPrint(std::size_t indent = 0) const;
};

std::string hexdump(const cbor_object& d)
{
  return hexdump(d.serialized_);
}

namespace detail
{
template <>
struct write_adapter<cbor_object> : write_adapter<Data>
{
  cbor_object& o;
  static write_adapter<cbor_object> adapt(cbor_object& d)
  {
    return write_adapter<cbor_object>{ d };
  }
  write_adapter<cbor_object>(cbor_object& d) : write_adapter<Data>{ d.serialized_ }, o{ d } {};

  // Allow unwrapping the adapter from the object...
  operator cbor_object&()
  {
    return o;
  }
};

template <>
struct read_adapter<cbor_object> : std::true_type
{
  static read_adapter<Data> adapt(const cbor_object& d)
  {
    return read_adapter<Data>{ d.serialized_ };
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
    std::size_t offset = data.position();
    data.advance(string_length);  // advance before reading.
    v.insert(v.begin(), &(data[offset]), &(data[offset]) + string_length);
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
    if (first_byte == ((0b100 << 5) | 2))
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

  template <typename Data>
  static std::size_t deserializer(Type& v, Data& data)
  {
    //  Just copy the approppriate chunks into the object....
    std::size_t start_pos = data.position();
    std::uint8_t first_byte;
    std::uint64_t value;
    std::size_t len = deserializeItem(first_byte, value, data);
    std::uint8_t major_type = first_byte >> 5;

    auto copy_to_object = [&v, &data](std::size_t start, std::size_t length) {
      for (std::size_t i = start; i < (start + length); i++)
      {
        v.serialized_.emplace_back(data[i]);
      }
      return length;
    };

    // simple fixed length and built in types:
    if ((major_type == 0b000) || (major_type == 0b001) || (major_type == 0b111))
    {
      return copy_to_object(start_pos, len);
    }

    if (major_type == 0b100)
    {
      // array.
      if (first_byte == ((0b100 << 5) | 31))
      {
        // Todo handle indefinite.
      }
      // copy the start byte.
      copy_to_object(start_pos, len);
      for (std::size_t i = 0; i < value; i++)
      {
        len += deserializer(v, data);
      }
      return len;
    }

    if (major_type == 0b101)
    {
      // array.
      if (first_byte == ((0b101 << 5) | 31))
      {
        // Todo handle indefinite.
      }
      // copy the start...
      copy_to_object(start_pos, len);
      for (std::size_t i = 0; i < value; i++)
      {
        len += deserializer(v, data);
        len += deserializer(v, data);
      }
      return len;
    }

    // String-esque
    if ((major_type == 0b010) || (major_type == 0b011))
    {
      // array.
      if ((first_byte & 0b11111) == 31)
      {
        // Todo handle indefinite.
      }
      copy_to_object(start_pos, len);
      len += copy_to_object(start_pos + len, value);
      data.advance(value);
    }
    return len;
  }
};
}  // namespace detail

std::string cbor_object::prettyPrint(std::size_t indent) const
{
  // Create this bespoke read adapter without any copies.
  const auto& data = serialized_;
  std::uint8_t first_byte = data.front();
  std::uint8_t major_type = first_byte >> 5;

  std::stringstream ss;
  auto ind = [&ss](std::size_t line_indent) {
    for (std::size_t i = 0; i < line_indent; i++)
    {
      ss << " ";
    }
  };
  // simple fixed length and built in types:
  if (major_type == 0b000)
  {
    std::uint64_t z;
    from_cbor(z, data);
    ind(indent);
    ss << "unsigned int: " << z << std::endl;
  }
  if (major_type == 0b001)
  {
    std::int64_t z;
    from_cbor(z, data);
    ind(indent);
    ss << "negative int: " << z << std::endl;
  }

  if (major_type == 0b111)
  {
    ind(indent);
    ss << "0b111: " << std::endl;
  }

  if (major_type == 0b100)
  {
    std::vector<cbor_object> z;

    cbor::from_cbor(z, data);  // yep :)

    ind(indent);
    ss << "array #" << z.size() << std::endl;

    for (const auto& v : z)
    {
      ss << v.prettyPrint(indent + 1);
    }
  }

  if (major_type == 0b101)
  {
    std::map<cbor_object, cbor_object> z;
    cbor::from_cbor(z, data);  // yep :)

    ind(indent);
    ss << "map #" << z.size() << std::endl;

    for (const auto& k_v : z)
    {
      ss << k_v.first.prettyPrint(indent + 1);
      ss << k_v.second.prettyPrint(indent + 1);
    }
  }

  // String-esque
  if ((major_type == 0b010) || (major_type == 0b011))
  {
    std::string z;
    from_cbor(z, data);
    ind(indent);
    ss << "str #" << z.size() << ":" << z << std::endl;
  }
  return ss.str();
}

}  // namespace cbor
