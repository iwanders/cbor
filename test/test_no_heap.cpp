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
#define CBOR_USE_EXCEPTIONS 0
#include "cbor/cbor.h"

// Get printf.
#include <stdio.h>

// Provide custom mallocs which will segfault when called.
void* malloc(size_t)
{
  auto null = reinterpret_cast<int*>(0);
  *null = 1;
  return null;
}

void* calloc(size_t, size_t)
{
  auto null = reinterpret_cast<int*>(0);
  *null = 1;
  return null;
}

void printMemory(std::uint8_t* data, std::size_t len)
{
  printf("#%lu [", len);
  for (std::size_t i = 0; i < len ; i++)
  {
    printf("%d ", data[i]);
  }
  printf("]\n");
}

int main(int /* argc */, char**  /* argv */)
{
  std::uint8_t cbor_repr[100];
  std::uint32_t my_vector[3] = {1, 2, 3};
  cbor::result res = cbor::to_cbor(my_vector, cbor_repr);
  printf("Result: %d, length: %lu\n", bool(res), std::size_t(res));
  printMemory(cbor_repr, res);

  std::uint32_t my_parsed_vector[3];
  res = cbor::from_cbor(my_parsed_vector, cbor_repr, 100u);
  printf("Result: %d, length: %lu\n", bool(res), std::size_t(res));
  printf("parsed[0]: %d\n", my_parsed_vector[0]);
  printf("parsed[1]: %d\n", my_parsed_vector[1]);
  printf("parsed[2]: %d\n", my_parsed_vector[2]);
  return 0;
}
