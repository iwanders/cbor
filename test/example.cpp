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
#include <iostream>
#include <vector>
// Exceptions can be disabled, in which case one has to rely on the result return value.
//  #define CBOR_USE_EXCEPTIONS 0
#include "cbor/stl.h"

template <typename T>
std::string printData(const std::vector<T>& t)
{
  std::stringstream ss;
  ss << "#" << t.size() << " [";
  for (const auto& k : t)
  {
    ss << int32_t(k) << " ";
  }
  ss << "]";
  return ss.str();
}

void start()
{
  {
    using Data = std::vector<std::uint8_t>;
    // Minimal standard serialization use case:
    std::vector<int> input{ 1, 50, -10 };
    Data cbor_repr;
    cbor::result result = cbor::to_cbor(input, cbor_repr);
    // --
    // Result is implicitly convertible into std::size_t, explicitly into bool.
    // Bool holds whether serialization was successful.
    // The length specifies how many bytes were used for the cbor representation.
    if (result)
    {
      std::size_t length = result;
      std::cout << "Length: " << length << std::endl;
    }
    std::size_t cbor_length = result;
    std::cout << "Serialization result: " << bool(result) << " length: " << cbor_length << std::endl;
    std::cout << "cbor representation is: " << printData(cbor_repr) << std::endl;

    // Minimal standard deserialization use case:
    std::vector<int> parsed;
    cbor::result parse_result = cbor::from_cbor(parsed, cbor_repr);
    // By default, this statement may throw if the types encountered in the data don't match the types you are parsing
    // into.
    // --
    // Bool holds whether deserialization was successful. Length how many bytes were read from the input.
    if (parse_result)
    {
      std::cout << "Succesfully parsed: " << printData(cbor_repr) << std::endl;
    }
    std::size_t parse_cbor_length = parse_result;
    std::cout << "Serialization result: " << bool(parse_result) << " length: " << parse_cbor_length << std::endl;
    std::cout << "Parsed data: " << printData(parsed) << std::endl;
  }

  std::cout << std::endl << std::endl;

  {
    using Data = std::vector<std::uint8_t>;
    // STL containers can be nested without problems:
    std::map<std::string, std::vector<int>> input;
    input["foo"] = { 500, -1500 };
    input["bar"] = { 1, 2, 3 };
    input["buz"] = { 15000000 };
    std::cout << "Input: " << std::endl;
    std::cout << "foo: " << printData(input.at("foo")) << std::endl;
    std::cout << "bar: " << printData(input.at("bar")) << std::endl;
    std::cout << "buz: " << printData(input.at("buz")) << std::endl;
    Data cbor_repr;
    cbor::result result = cbor::to_cbor(input, cbor_repr);
    if (result)
    {
      std::cout << "Serialization succesful" << std::endl;
    }
    std::size_t cbor_length = result;
    std::cout << "Serialization result: " << bool(result) << " length: " << cbor_length << std::endl;
    std::cout << "cbor representation is: " << printData(cbor_repr) << std::endl;
    //--
    std::map<std::string, std::vector<int>> parsed;
    cbor::result parse_result = cbor::from_cbor(parsed, cbor_repr);
    // --
    if (parse_result)
    {
      std::cout << "Succesfully parsed: " << printData(cbor_repr) << std::endl;
    }
    std::size_t parse_cbor_length = parse_result;
    std::cout << "Serialization result: " << bool(parse_result) << " length: " << parse_cbor_length << std::endl;
    std::cout << "foo: " << printData(parsed.at("foo")) << std::endl;
    std::cout << "bar: " << printData(parsed.at("bar")) << std::endl;
    std::cout << "buz: " << printData(parsed.at("buz")) << std::endl;
  }

  std::cout << std::endl << std::endl;

  {
    // Serialization and deserialization can be done to a pointer with a maximum size:
    std::vector<int> input{ 1, 50, -10 };
    std::array<std::uint8_t, 7> cbor_repr;
    cbor::result result = cbor::to_cbor(input, cbor_repr.data(), cbor_repr.size());
    std::cout << "Serialization result: " << result << std::endl;
    //--
    std::vector<int> parsed;
    cbor::result parse_result = cbor::from_cbor(parsed, cbor_repr.data(), cbor_repr.size());
    std::cout << "Deserialization result: " << parse_result << std::endl;
    std::cout << "Parsed data: " << printData(parsed) << std::endl;
  }

  std::cout << std::endl << std::endl;

  {
    using Data = std::vector<std::uint8_t>;
    // for non homogeneous data one can serialize and deserialize to a cbor_object.
    std::vector<cbor::cbor_object> input;
    input.push_back(2);
    input.push_back(std::vector<int>{ 1, 2, 3 });
    input.push_back(std::string{ "Foo" });
    Data cbor_repr;
    cbor::result result = cbor::to_cbor(input, cbor_repr);
    std::cout << "Serialization result: " << result << std::endl;

    std::vector<cbor::cbor_object> parsed;
    cbor::result parse_result = cbor::from_cbor(parsed, cbor_repr);
    std::cout << "Deserialization result: " << parse_result << " parsed length: " << parsed.size() << std::endl;
    int first_value = parsed[0].get<int>();
    std::cout << "First value: " << first_value << std::endl;
    std::vector<int> second_value;
    parsed[1].get_to(second_value);
    std::cout << "Second value: " << printData(second_value) << std::endl;
    std::cout << "Third value: " << parsed[2].get<std::string>() << std::endl;
  }

  std::cout << std::endl << std::endl;
  {
    std::map<std::string, std::vector<int>> input;
    input["foo"] = { 500, -1500 };
    input["bar"] = { 1, 2, 3 };
    input["buz"] = { 15000000 };

    // The cbor::cbor_object can also be used to serialize into.
    cbor::cbor_object cbor_repr;
    cbor::result result = cbor::to_cbor(input, cbor_repr);
    std::cout << "result: " << result << " cbor_repr: " << printData(cbor_repr.serialized()) << std::endl;
    // It also has a pretty print functionality;
    std::cout << cbor_repr.prettyPrint() << std::endl;
  }
}

namespace my_namespace
{
struct Buz
{
  std::uint32_t f;
  std::uint32_t x;
  std::string toString() const
  {
    return std::string{ "<Buz " } + std::to_string(f) + ", " + std::to_string(x) + ">";
  }
};

// using a map works, but using an array with fixed positions is shorter.
template <typename Data>
cbor::result to_cbor(const Buz& b, Data& data)
{
  std::map<std::string, cbor::cbor_object> representation;
  representation["f"] = b.f;
  representation["x"] = b.x;
  return to_cbor(representation, data);
}

template <typename Data>
cbor::result from_cbor(Buz& b, Data& data)
{
  std::map<std::string, cbor::cbor_object> representation;
  auto res = from_cbor(representation, data);
  representation.at("f").get_to(b.f);
  representation.at("x").get_to(b.x);
  return res;
}
}  // namespace my_namespace

void custom_type()
{
  std::vector<my_namespace::Buz> input = { my_namespace::Buz{ 2, 5 }, my_namespace::Buz{ 3, 6 },
                                           my_namespace::Buz{ 4, 7 } };
  cbor::cbor_object cbor_representation;
  cbor::result res = cbor::to_cbor(input, cbor_representation);
  std::cout << "Serialization result: " << res << std::endl;
  std::cout << "Cbor representation: " << std::endl;
  std::cout << cbor_representation.prettyPrint() << std::endl;

  std::vector<my_namespace::Buz> output;
  res = cbor::from_cbor(output, cbor_representation);
  std::cout << "Deserialization result: " << res << std::endl;
  for (const auto& entry : output)
  {
    std::cout << entry.toString() << std::endl;
  }
}

int main(int /* argc */, char** /* argv */)
{
  start();
  custom_type();
  return 0;
}
