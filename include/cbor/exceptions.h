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
#ifndef CBOR_USE_EXCEPTIONS
#define CBOR_USE_EXCEPTIONS 1
#endif

#if CBOR_USE_EXCEPTIONS==1
#include <stdexcept>
namespace cbor
{
/**
 * @brief base class for all cbor errors
 */
class error : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/**
 * @brief Thrown on unhandled cbor types, or malformed data.
 */
class parse_error : public error
{
public:
  using error::error;
};

/**
 * @brief Thrown if deserialized type doesnt fit in expected type or different type was encountered then expected.
 */
class type_error : public error
{
public:
  using error::error;
};

/**
 * @brief Thrown when reading outside of a buffer or buffer couldn't be adequately resized.
 */
class buffer_error : public error
{
public:
  using error::error;
};
}  // namespace cbor
// define macros to use use when throwing an exception.
#define CBOR_PARSE_ERROR(A) throw cbor::parse_error(A);
#define CBOR_TYPE_ERROR(A) throw cbor::type_error(A);
#define CBOR_BUFFER_ERROR(A) throw cbor::buffer_error(A);
#else
// make the macro's nops
#define CBOR_PARSE_ERROR(A)
#define CBOR_TYPE_ERROR(A)
#define CBOR_BUFFER_ERROR(A)
#endif
