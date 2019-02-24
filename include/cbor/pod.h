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
#include "exceptions.h"
#include "traits.h"
#include "util.h"

namespace cbor
{
struct result
{
  bool success{ true };
  std::size_t length{ 0 };

  result(){};

  /**
   * @brief Allow assigning from a non bool into the length.
   */
  // This function is disabled for bool types to get the other constructor to set the success flag.
  template <typename T, std::enable_if_t<!std::is_same<bool, T>::value, int> = 0>
  result(T len)
  {
    length = len;
  }

  /**
   * @brief If constructed from a bool, set the assign flag.
   */
  result(bool is_success)
  {
    success = is_success;
  }

  /**
   * @brief Addition operator, add lengths, set succes to the AND of both flags.
   */
  // This function is disabled for integral types to implicitly convert those first.
  template <typename T, std::enable_if_t<!std::is_integral<T>::value, int> = 0>
  result operator+(const T& a)
  {
    result res;
    res.success = success && a.success;
    res.length = length + a.length;
    return res;
  }

  template <typename T>
  result& operator+=(const T& a)
  {
    result z = *this + result(a);
    success = z.success;
    length = z.length;
    return *this;
  }
  explicit operator bool() const
  {
    return success;
  }
  operator std::size_t() const
  {
    return length;
  }
};

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

  result resize(std::uint32_t value)
  {
    if (value >= max_length)
    {
      CBOR_BUFFER_ERROR("Resize failed: " + std::to_string(value) +
                        " exceed max length: " + std::to_string(max_length));
      return false;
    }
    used_size = value;
    return true;
  }

  std::uint32_t size() const
  {
    return used_size;
  }

  DataType& operator[](std::size_t pos)
  {
    if (pos > max_length)
    {
      CBOR_BUFFER_ERROR("Write failed: " + std::to_string(pos) + " exceed max length: " + std::to_string(max_length));
      return data[max_length - 1];
    }
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
  result advance(std::size_t count)
  {
    cursor += count;
    if (cursor > max_length)
    {
      CBOR_BUFFER_ERROR("Advance failed: " + std::to_string(cursor) +
                        " exceed max length: " + std::to_string(max_length));
      return false;
    }
    return count;
  }
  std::size_t size() const
  {
    return max_length;
  }
  const DataType& operator[](std::size_t pos) const
  {
    if (pos > max_length)
    {
      CBOR_BUFFER_ERROR("Read failed: " + std::to_string(pos) + " exceed max length: " + std::to_string(max_length));
      return data[max_length - 1];
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
result serializeItem(const std::uint8_t major_type, const std::uint8_t v, Data& data)
{
  if (v <= 23)
  {
    const std::size_t offset = data.size();
    result res = data.resize(data.size() + 1);
    data[offset] = std::uint8_t(major_type << 5) | v;
    return res + 1;
  }
  else
  {
    const std::size_t offset = data.size();
    result res = data.resize(offset + 2);
    data[offset] = std::uint8_t(major_type << 5) | 24;
    data[offset + 1] = v;
    return res + 2;
  }
}

template <typename Data>
result serializePrimitive(const std::uint8_t primitive, Data& data)
{
  const std::size_t offset = data.size();
  result res = data.resize(data.size() + 1);
  data[offset] = primitive;
  return res + 1;
}
template <typename Data>
result deserializePrimitive(std::uint8_t& primitive, Data& data)
{
  primitive = data[data.position()];
  return data.advance(1);
}

/**
 * @brief Function to write a 16 bit unsigned int in its shortest form given the major type.
 */
template <typename Data>
result serializeItem(const std::uint8_t major_type, const std::uint16_t v, Data& data)
{
  if (v <= std::numeric_limits<std::uint8_t>::max())
  {
    return serializeItem(major_type, static_cast<std::uint8_t>(v), data);
  }
  const std::size_t offset = data.size();
  result res = data.resize(offset + 2 + 1);
  data[offset] = std::uint8_t(major_type << 5) | 25;
  auto fixed = fixEndianness(v);
  if (res)
  {  // Only do this if we could resize the data.
    *reinterpret_cast<std::uint16_t*>(&(data[offset + 1])) = fixed;
  }
  return res + 3;
}

/**
 * @brief Function to write a 32 bit unsigned int in its shortest form given the major type.
 */
template <typename Data>
result serializeItem(const std::uint8_t major_type, const std::uint32_t v, Data& data)
{
  if (v <= std::numeric_limits<std::uint16_t>::max())
  {
    return serializeItem(major_type, static_cast<std::uint16_t>(v), data);
  }
  const std::size_t offset = data.size();
  result res = data.resize(offset + 4 + 1);
  auto fixed = fixEndianness(v);
  data[offset] = std::uint8_t(major_type << 5) | 26;
  if (res)
  {  // Only do this if we could resize the data.
    *reinterpret_cast<std::uint32_t*>(&(data[offset + 1])) = fixed;
  }
  return res + 5;
}

/**
 * @brief Function to write a 64 bit unsigned int in its shortest form given the major type.
 */
template <typename Data>
result serializeItem(const std::uint8_t major_type, const std::uint64_t v, Data& data)
{
  if (v <= std::numeric_limits<std::uint32_t>::max())
  {
    return serializeItem(major_type, static_cast<std::uint32_t>(v), data);
  }
  const std::size_t offset = data.size();
  result res = data.resize(offset + 8 + 1);
  data[offset] = std::uint8_t(major_type << 5) | 27;
  auto fixed = fixEndianness(v);
  if (res)
  {  // Only do this if we could resize the data.
    *reinterpret_cast<std::uint64_t*>(&(data[offset + 1])) = fixed;
  }
  return res + 9;
}

template <typename Data>
result deserializeItem(std::uint8_t& first_byte, std::uint64_t& v, Data& data)
{
  first_byte = data[data.position()];
  std::uint8_t direct = data[data.position()] & 0b11111;
  std::size_t offset = data.position();
  if (direct < 24)
  {
    v = direct;
    return data.advance(1);
  }
  else if (direct == 24)  // uint8_t case
  {
    result res = data.advance(2);
    if (res)
    {
      v = data[offset + 1];
    }
    return res;
  }
  else if (direct == 25)  // uint16_t case
  {
    result res = data.advance(3);
    if (res)
    {
      const std::uint16_t intermediate = *reinterpret_cast<const std::uint16_t*>(&(data[offset + 1]));
      v = fixEndianness(intermediate);
    }
    return res;
  }
  else if (direct == 26)
  {
    result res = data.advance(5);
    if (res)
    {
      const std::uint32_t intermediate = *reinterpret_cast<const std::uint32_t*>(&(data[offset + 1]));
      v = fixEndianness(intermediate);
    }
    return res;
  }
  else if (direct == 27)
  {
    result res = data.advance(9);
    if (res)
    {
      const std::uint64_t intermediate = *reinterpret_cast<const std::uint64_t*>(&(data[offset + 1]));
      v = fixEndianness(intermediate);
    }
    return res;
  }
  else if (direct == 31)
  {
    CBOR_PARSE_ERROR("Unhandled direct value");
    return false;
  }
  return false;
}

template <typename Type, typename Data>
static result deserializeInteger(std::uint8_t major_type, Type& v, Data& data)
{
  std::uint8_t first = 0;
  std::uint64_t res;
  result advanced = deserializeItem(first, res, data);
  std::uint8_t read_major_type = first >> 5;
  if (read_major_type == major_type)
  {
    if (res > std::numeric_limits<Type>::max())
    {
      CBOR_TYPE_ERROR("Deserialized value" + std::to_string(res) + " does not fit in " + typeid(Type).name())
      return false;
    }
    else
    {
      v = res;
      return advanced;
    }
  }
  else
  {
    CBOR_TYPE_ERROR("Parsed major type " + std::to_string(read_major_type) + " is different then expected type " +
                    std::to_string(major_type));
    return false;
  }
  return 0;
}

template <typename Type, typename Data>
static result deserializeSignedInteger(Type& v, Data& data)
{
  std::uint8_t first = 0;
  std::uint64_t res;
  result advanced = deserializeItem(first, res, data);
  std::uint8_t read_major_type = first >> 5;
  if (read_major_type == 0b000)
  {
    if (res > std::numeric_limits<Type>::max())
    {
      CBOR_TYPE_ERROR("Deserialized value" + std::to_string(res) + " does not fit in " + typeid(Type).name())
      return false;
    }
    else
    {
      v = res;
      return advanced;
    }
  }
  else if (read_major_type == 0b001)
  {
    std::int64_t signedres = -(res + 1);
    if (signedres < std::numeric_limits<Type>::min())
    {
      CBOR_TYPE_ERROR("Deserialized value" + std::to_string(res) + " does not fit in " + typeid(Type).name())
      return false;
    }
    else
    {
      v = signedres;
      return advanced;
    }
  }
  else
  {
    CBOR_TYPE_ERROR("Parsed major type " + std::to_string(read_major_type) + " is different then expected type 0");
    return false;
  }
  return false;
}
/**
 * Specialization for bool.
 */
template <>
struct traits<bool>
{
  using Type = bool;

  template <typename Data>
  static result serializer(const Type& v, Data& data)
  {
    return serializePrimitive((0b111 << 5) | (20 + (1 ? v : 0)), data);
  }

  template <typename Data>
  static result deserializer(Type& v, Data& data)
  {
    std::uint8_t type;
    result res = deserializePrimitive(type, data);
    std::uint8_t read_major_type = type >> 5;
    if (read_major_type == 0b111)
    {
      if ((type & 0b11111) == 21)
      {
        v = true;
        return res;
      }
      else if ((type & 0b11111) == 20)
      {
        v = false;
        return res;
      }
      else
      {
        CBOR_TYPE_ERROR("Parsed type " + std::to_string(type) + " was not true or false types.");
        return false;
      }
    }
    else
    {
      CBOR_TYPE_ERROR("Parsed major type " + std::to_string(read_major_type) +
                      " is different then expected type 0b111");
      return false;
    }
    return false;
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
  static result serializer(const Type& /* v */, Data& data)
  {
    return serializePrimitive((0b111 << 5) | 22, data);
  }
  template <typename Data>
  static result deserializer(Type& v, Data& data)
  {
    std::uint8_t type;
    result res = deserializePrimitive(type, data);
    if (type == ((0b111 << 5) | 22))
    {
      v = nullptr;
    }
    else
    {
      CBOR_TYPE_ERROR("Parsed type " + std::to_string(type) + " is different then expected type 0xF6");
      return false;
    }
    return res;
  }
};

/**
 * Specialization for unsigned_integers.
 */
template <typename IntegerType>
struct traits<trait_families::unsigned_integer, IntegerType>
{
  template <typename Type, typename Data>
  static result serializer(const Type& v, Data& data)
  {
    return serializeItem(0b000, v, data);
  }

  template <typename Type, typename Data>
  static result deserializer(Type& v, Data& data)
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
  static result serializer(const Type& v, Data& data)
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
  static result deserializer(Type& v, Data& data)
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
  static result serializer(const Type& v, Data& data)
  {
    result res = serializePrimitive((0b111 << 5) | Helper::minor_type, data);
    const std::size_t offset = data.size();
    res += data.resize(offset + sizeof(Type));
    if (res)
    {
      auto fixed = fixEndianness(*reinterpret_cast<const typename Helper::int_type*>(&v));
      *reinterpret_cast<typename Helper::int_type*>(&(data[offset])) = fixed;
    }
    return res + sizeof(Type);
  }

  template <typename Data>
  static result deserializer(Type& v, Data& data)
  {
    std::uint8_t type;
    result res = deserializePrimitive(type, data);
    if (type == ((0b111 << 5) | Helper::minor_type))
    {
      const std::size_t offset = data.position();
      res += data.advance(sizeof(Type));  // advance before using the memory. This prevents reading out of bounds.
      auto fixed = fixEndianness(*reinterpret_cast<const typename Helper::int_type*>(&data[offset]));
      v = *reinterpret_cast<const Type*>(&fixed);
      return res;
    }
    else
    {
      // type error.
      CBOR_TYPE_ERROR("Parsed type " + std::to_string(type) + " is different then expected type " +
                      std::to_string(Helper::minor_type));
      return false;
    }
    return false;
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
  static result serializer(const Type& v, Data& data)
  {
    std::size_t length = strlen(v);
    result res = serializeItem(0b011, length, data);
    std::size_t offset = data.size();
    res += data.resize(data.size() + length);
    for (std::size_t i = 0; i < length; i++)
    {
      data[i + offset] = v[i];
    }
    return res + length;
  }
};

template <typename ArrayElement>
struct traits<trait_families::c_array_family, ArrayElement>
{
  using Type = ArrayElement;

  template <typename InType, typename Data, size_t N>
  static result serializer(const InType (&d)[N], Data& data)
  {
    result res = serializeItem(0b100, N, data);
    for (std::size_t i = 0; i < N; i++)
    {
      res += to_cbor(d[i], data);
      if (!res)
      {
        return res;
      }
    }
    return res;
  }
  template <typename InType, typename Data, size_t N>
  static result deserializer(InType (&d)[N], Data& data)
  {
    std::uint8_t first_byte;
    std::uint64_t length = 0;
    result res = deserializeItem(first_byte, length, data);
    if (length != N)
    {
      // Treat incorrect lengths as incorrect type.
      CBOR_TYPE_ERROR("Expected array of " + std::to_string(length) + " long, but only have array of " +
                      std::to_string(N));
      return false;
    }

    std::uint8_t read_major_type = first_byte >> 5;
    if (read_major_type == 0b100)
    {
      for (std::size_t i = 0; i < length; i++)
      {
        res += from_cbor(d[i], data);
        if (!res)  // if failed... abort early.
        {
          return res;
        }
      }
    }
    else
    {
      CBOR_TYPE_ERROR("Parsed major type " + std::to_string(read_major_type) +
                      " is different then expected type 0b100");
      return false;
    }
    return res;
  }
};

}  // namespace detail
}  // namespace cbor
