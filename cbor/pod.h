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
#include <cstring>
#include <limits>
#include "util.h"
#include "traits.h"
#include "cbor.h"

namespace cbor
{
namespace detail
{

template <>
struct data_adapter<DataType*, std::size_t>
{
  DataType* data;
  std::size_t max_length;
  std::size_t used_size { 0 };
  data_adapter<DataType*, std::size_t>(DataType* d, std::size_t size) : data{d}, max_length{size} {};
  void resize(std::uint32_t value)
  {
    used_size = value;
  }
  std::uint32_t size() const
  {
    return used_size;
  }
  DataType& operator[](std::size_t pos)
  {
    return data[pos];
  }
};


/**
 * @brief Function to write a 8 bit unsigned int in its shortest form given the major type.
 */
template <typename Data>
std::size_t serializeInteger(const std::uint8_t major_type, const std::uint8_t v, Data& data)
{
  if (v <= 23)
  {
    const std::size_t offset = data.size();
    data.resize(data.size() + 1);
    data[offset] = std::uint8_t(major_type << 5) | v;
    return 1;
  }
  else
  {
    const std::size_t offset = data.size();
    data.resize(offset + 2);
    data[offset] = std::uint8_t(major_type << 5) | 24;
    data[offset + 1] = v;
    return 2;
  }
}

template <typename Data>
std::size_t serializePrimitive(const std::uint8_t primitive, Data& data)
{
  const std::size_t offset = data.size();
  data.resize(data.size() + 1);
  data[offset] = primitive;
  return 1;
}

/**
 * @brief Function to write a 16 bit unsigned int in its shortest form given the major type.
 */
template <typename Data>
std::size_t serializeInteger(const std::uint8_t major_type, const std::uint16_t v, Data& data)
{
  if (v <= std::numeric_limits<std::uint8_t>::max())
  {
    return serializeInteger(major_type, static_cast<std::uint8_t>(v), data);
  }
  const std::size_t offset = data.size();
  data.resize(offset + 2 + 1);
  data[offset] = std::uint8_t(major_type << 5) | 25;
  auto fixed = fixEndianness(v);
  *reinterpret_cast<std::uint16_t*>(&(data[offset + 1])) = fixed;
  return 3;
}

/**
 * @brief Function to write a 32 bit unsigned int in its shortest form given the major type.
 */
template <typename Data>
std::size_t serializeInteger(const std::uint8_t major_type, const std::uint32_t v, Data& data)
{
  if (v <= std::numeric_limits<std::uint16_t>::max())
  {
    return serializeInteger(major_type, static_cast<std::uint16_t>(v), data);
  }
  const std::size_t offset = data.size();
  data.resize(offset + 4 + 1);
  auto fixed = fixEndianness(v);
  data[offset] = std::uint8_t(major_type << 5) | 26;
  *reinterpret_cast<std::uint32_t*>(&(data[offset + 1])) = fixed;
  return 5;
}

/**
 * @brief Function to write a 64 bit unsigned int in its shortest form given the major type.
 */
template <typename Data>
std::size_t serializeInteger(const std::uint8_t major_type, const std::uint64_t v, Data& data)
{
  if (v <= std::numeric_limits<std::uint32_t>::max())
  {
    return serializeInteger(major_type, static_cast<std::uint32_t>(v), data);
  }
  const std::size_t offset = data.size();
  data.resize(offset + 8 + 1);
  data[offset] = std::uint8_t(major_type << 5) | 27;
  auto fixed = fixEndianness(v);
  *reinterpret_cast<std::uint64_t*>(&(data[offset + 1])) = fixed;
  return 9;
}

/**
 * Specialization for bool.
 */
template <>
struct traits<bool>
{
  using Type = bool;

  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    return serializePrimitive((0b111 << 5) | (20 + (1 ? v : 0)), data);
  }
};

/**
 * Specialization for null.
 */
template <>
struct traits<std::nullptr_t>
{
  using Type = std::nullptr_t;

  template <typename Data>
  static std::size_t serializer(const Type& /* v */, Data& data)
  {
    return serializePrimitive((0b111 << 5) | 22, data);
  }
};


/**
 * Specialization for uint8_t.
 */
template <>
struct traits<std::uint8_t>
{
  using Type = std::uint8_t;

  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    return serializeInteger(0b000, v, data);
  }
};

/**
 * Specialization for uint16_t.
 */
template <>
struct traits<std::uint16_t>
{
  using Type = std::uint16_t;
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    return serializeInteger(0b000, v, data);
  }
};

/**
 * Specialization for uint32_t.
 */
template <>
struct traits<std::uint32_t>
{
  using Type = std::uint32_t;
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    return serializeInteger(0b000, v, data);
  }
};

/**
 * Specialization for uint64_t.
 */
template <>
struct traits<std::uint64_t>
{
  using Type = std::uint64_t;
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    return serializeInteger(0b000, v, data);
  }
};

/**
 * Specialization for int8_t.
 */
template <>
struct traits<std::int8_t>
{
  using Type = std::int8_t;

  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    if (v < 0)
    {
      return serializeInteger(0b001, static_cast<std::uint8_t>(-v-1), data);
    }
    else
    {
      return serializeInteger(0b000, static_cast<std::uint8_t>(v), data);
    }
  }
};

/**
 * Specialization for uint16_t.
 */
template <>
struct traits<std::int16_t>
{
  using Type = std::int16_t;
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    if (v < 0)
    {
      return serializeInteger(0b001, static_cast<std::uint16_t>(-v-1), data);
    }
    else
    {
      return serializeInteger(0b000, static_cast<std::uint16_t>(v), data);
    }
  }
};

/**
 * Specialization for uint32_t.
 */
template <>
struct traits<std::int32_t>
{
  using Type = std::int32_t;
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    if (v < 0)
    {
      return serializeInteger(0b001, static_cast<std::uint32_t>(-v - 1), data);
    }
    else
    {
      return serializeInteger(0b000, static_cast<std::uint32_t>(v), data);
    }
  }
};

/**
 * Specialization for uint64_t.
 */
template <>
struct traits<std::int64_t>
{
  using Type = std::int64_t;
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    if (v < 0)
    {
      return serializeInteger(0b001, static_cast<std::uint64_t>(-v - 1), data);
    }
    else
    {
      return serializeInteger(0b000, static_cast<std::uint64_t>(v), data);
    }
  }
};


/**
 * Specialization for float.
 */
template <>
struct traits<float>
{
  using Type = float;
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    serializePrimitive((0b111 << 5) | 26, data);
    const std::size_t offset = data.size();
    data.resize(offset + sizeof(Type));
    auto fixed = fixEndianness(*reinterpret_cast<const std::uint32_t*>(&v));
    *reinterpret_cast<std::uint32_t*>(&(data[offset])) = fixed;
    return 5;
  }
};

/**
 * Specialization for double.
 */
template <>
struct traits<double>
{
  using Type = double;
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    serializePrimitive((0b111 << 5) | 27, data);
    const std::size_t offset = data.size();
    data.resize(offset + sizeof(Type));
    auto fixed = fixEndianness(*reinterpret_cast<const std::uint64_t*>(&v));
    *reinterpret_cast<std::uint64_t*>(&(data[offset])) = fixed;
    return 9;
  }
};


/**
 * Specialization for const char*
 */
template <>
struct traits<const char*>
{
  using Type = const char*;
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    std::size_t length = strlen(v);
    std::size_t addition = serializeInteger(0b011, length, data);
    std::size_t offset = data.size();
    data.resize(data.size() + length);
    for (std::size_t i = 0; i < length; i++)
    {
      data[i + offset] = v[i];
    }
    return addition + length;
  }
};

}  // namespace detail
}  // namespace cbor
