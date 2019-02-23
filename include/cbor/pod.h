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
#include "cbor.h"
#include "traits.h"
#include "util.h"

namespace cbor
{
namespace detail
{
template <>
struct write_adapter<DataType*> : std::true_type
{
  DataType* data;
  std::size_t max_length;
  std::size_t used_size{ 0 };

  template <typename T>
  static write_adapter<DataType*> adapt(DataType* d, const T size)
  {
    return write_adapter<DataType*>{ d, size };
  }

  template <size_t N>
  static write_adapter<DataType*> adapt(DataType (&d)[N])
  {
    return write_adapter<DataType*>{ d, N };
  }

  template <typename T>
  write_adapter<DataType*>(DataType* d, const T size) : data{ d }, max_length{ size }
  {
  }

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

template <>
struct read_adapter<DataType*> : std::true_type
{
  const DataType* data;
  std::size_t max_length;
  std::size_t cursor = 0;
  template <typename T>
  static read_adapter<DataType*> adapt(const DataType* d, const T s)
  {
    return read_adapter<DataType*>{ d, s };
  }

  template <size_t N>
  static read_adapter<DataType*> adapt(DataType (&d)[N])
  {
    return read_adapter<DataType*>{ d, N };
  }

  template <typename T>
  read_adapter<DataType*>(const DataType* d, const T size) : data{ d }, max_length{ size }
  {
  }
  std::size_t position() const
  {
    return cursor;
  }
  void advance(std::size_t count)
  {
    cursor += count;
    if (cursor >= max_length)
    {
      // Throw!?
    }
  }
  std::size_t size() const
  {
    return max_length;
  }
  const DataType& operator[](std::size_t pos) const
  {
    if (pos >= max_length)
    {
      // Throw!?
    }
    return data[pos];
  }
};
template <>
struct read_adapter<const DataType*> : read_adapter<DataType*>
{
};

/**
 * @brief Function to write a 8 bit unsigned int in its shortest form given the major type.
 */
template <typename Data>
std::size_t serializeItem(const std::uint8_t major_type, const std::uint8_t v, Data& data)
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
template <typename Data>
std::size_t deserializePrimitive(std::uint8_t& primitive, Data& data)
{
  primitive = data[data.position()];
  data.advance(1);
  return 1;
}

/**
 * @brief Function to write a 16 bit unsigned int in its shortest form given the major type.
 */
template <typename Data>
std::size_t serializeItem(const std::uint8_t major_type, const std::uint16_t v, Data& data)
{
  if (v <= std::numeric_limits<std::uint8_t>::max())
  {
    return serializeItem(major_type, static_cast<std::uint8_t>(v), data);
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
std::size_t serializeItem(const std::uint8_t major_type, const std::uint32_t v, Data& data)
{
  if (v <= std::numeric_limits<std::uint16_t>::max())
  {
    return serializeItem(major_type, static_cast<std::uint16_t>(v), data);
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
std::size_t serializeItem(const std::uint8_t major_type, const std::uint64_t v, Data& data)
{
  if (v <= std::numeric_limits<std::uint32_t>::max())
  {
    return serializeItem(major_type, static_cast<std::uint32_t>(v), data);
  }
  const std::size_t offset = data.size();
  data.resize(offset + 8 + 1);
  data[offset] = std::uint8_t(major_type << 5) | 27;
  auto fixed = fixEndianness(v);
  *reinterpret_cast<std::uint64_t*>(&(data[offset + 1])) = fixed;
  return 9;
}

template <typename Data>
std::size_t deserializeItem(std::uint8_t& first_byte, std::uint64_t& v, Data& data)
{
  first_byte = data[data.position()];
  std::uint8_t direct = data[data.position()] & 0b11111;
  if (direct < 24)
  {
    v = direct;
    data.advance(1);
    return 1;
  }
  else if (direct == 24)  // uint8_t case
  {
    v = data[data.position() + 1];
    data.advance(2);
    return 2;
  }
  else if (direct == 25)  // uint16_t case
  {
    const std::uint16_t intermediate = *reinterpret_cast<const std::uint16_t*>(&(data[data.position() + 1]));
    v = fixEndianness(intermediate);
    data.advance(3);
    return 3;
  }
  else if (direct == 26)
  {
    const std::uint32_t intermediate = *reinterpret_cast<const std::uint32_t*>(&(data[data.position() + 1]));
    v = fixEndianness(intermediate);
    data.advance(5);
    return 5;
  }
  else if (direct == 27)
  {
    const std::uint64_t intermediate = *reinterpret_cast<const std::uint64_t*>(&(data[data.position() + 1]));
    v = fixEndianness(intermediate);
    data.advance(9);
    return 9;
  }
  else if (direct == 31)
  {
  }
  return 0;
}

template <typename Type, typename Data>
static std::size_t deserializeInteger(std::uint8_t major_type, Type& v, Data& data)
{
  std::uint8_t first = 0;
  std::uint64_t res;
  std::size_t advanced = deserializeItem(first, res, data);
  if ((first >> 5) == major_type)
  {
    if (res > std::numeric_limits<Type>::max())
    {
      // Todo: Size error!?
    }
    else
    {
      v = res;
      return advanced;
    }
  }
  else
  {
    // @todo; panic
  }
  return 0;
}

template <typename Type, typename Data>
static std::size_t deserializeSignedInteger(Type& v, Data& data)
{
  std::uint8_t first = 0;
  std::uint64_t res;
  std::size_t advanced = deserializeItem(first, res, data);
  if ((first >> 5) == 0b000)
  {
    if (res > std::numeric_limits<Type>::max())
    {
      // Todo: Size error!?
    }
    else
    {
      v = res;
      return advanced;
    }
  }
  else if ((first >> 5) == 0b001)
  {
    std::int64_t signedres = -(res + 1);
    if (signedres < std::numeric_limits<Type>::min())
    {
      // Todo: Size error!?
    }
    else
    {
      v = signedres;
      return advanced;
    }
  }
  else
  {
    // @todo; panic
  }
  return 0;
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

  template <typename Data>
  static std::size_t deserializer(Type& v, Data& data)
  {
    std::uint8_t type;
    std::size_t size = deserializePrimitive(type, data);
    if ((type >> 5) == (0b111))
    {
      if ((type & 0b11111) == 21)
      {
        v = true;
        return size;
      }
      else if ((type & 0b11111) == 20)
      {
        v = false;
        return size;
      }
      else
      {
        // @todo type error.
      }
    }
    else
    {
      // @todo type error.
    }
    return 0;
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
  template <typename Data>
  static std::size_t deserializer(Type& v, Data& data)
  {
    std::uint8_t type;
    std::size_t size = deserializePrimitive(type, data);
    if (type == ((0b111 << 5) | 22))
    {
      v = nullptr;
    }
    else
    {
      // @todo type error
    }
    return size;
  }
};

/**
 * Specialization for unsigned_integers.
 */
template <typename IntegerType>
struct traits<trait_families::unsigned_integer, IntegerType>
{
  template <typename Type, typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    return serializeItem(0b000, v, data);
  }

  template <typename Type, typename Data>
  static std::size_t deserializer(Type& v, Data& data)
  {
    return deserializeInteger(0b000, v, data);
  }
};
/**
 * Specialization for signed_integers.
 */
template <typename Type>
struct traits<trait_families::signed_integer, Type>
{
  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    if (v < 0)
    {
      return serializeItem(0b001, static_cast<typename std::make_unsigned<Type>::type>(-v - 1), data);
    }
    else
    {
      return serializeItem(0b000, static_cast<typename std::make_unsigned<Type>::type>(v), data);
    }
  }
  template <typename Data>
  static std::size_t deserializer(Type& v, Data& data)
  {
    return deserializeSignedInteger(v, data);
  }
};

template <typename T>
struct trait_floating_point_helper
{
};

template <>
struct trait_floating_point_helper<float>
{
  using type = float;
  static const std::uint8_t minor_type = 26;
  using int_type = std::uint32_t;
};

template <>
struct trait_floating_point_helper<double>
{
  using type = double;
  static const std::uint8_t minor_type = 27;
  using int_type = std::uint64_t;
};
// long double exists, but is not in the cbor spec.

template <typename FloatingPointType>
struct traits<trait_families::floating_point, FloatingPointType>
{
  using Type = FloatingPointType;
  using Helper = trait_floating_point_helper<Type>;

  template <typename Data>
  static std::size_t serializer(const Type& v, Data& data)
  {
    serializePrimitive((0b111 << 5) | Helper::minor_type, data);
    const std::size_t offset = data.size();
    data.resize(offset + sizeof(Type));
    auto fixed = fixEndianness(*reinterpret_cast<const typename Helper::int_type*>(&v));
    *reinterpret_cast<typename Helper::int_type*>(&(data[offset])) = fixed;
    return sizeof(Type);
  }

  template <typename Data>
  static std::size_t deserializer(Type& v, Data& data)
  {
    std::uint8_t type;
    std::size_t size = deserializePrimitive(type, data);
    if (type == ((0b111 << 5) | Helper::minor_type))
    {
      const std::size_t offset = data.position();
      data.advance(sizeof(Type));  // advance before using the memory. This prevents reading out of bounds.
      auto fixed = fixEndianness(*reinterpret_cast<const typename Helper::int_type*>(&data[offset]));
      v = *reinterpret_cast<const Type*>(&fixed);
      return sizeof(Type) + size;
    }
    else
    {
      // type error.
    }
    return 0;
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
    std::size_t addition = serializeItem(0b011, length, data);
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
