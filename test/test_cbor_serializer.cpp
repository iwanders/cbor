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
#include <algorithm>
#include <array>
#include <iostream>
#include <vector>
//  #define CBOR_USE_EXCEPTIONS 0
#include "cbor/stl.h"
#include "test.h"

/*

*/

void test_stl()
{
  std::cout << "Test " << __FUNCTION__ << " " << __LINE__ << std::endl;

  // test tuple
  {
    std::tuple<unsigned int, unsigned int> input{ 1, 2 };
    Data expected = { 0x82, 0x01, 0x02 };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    std::tuple<unsigned int, unsigned int> output;
    const Data cbor_data = cbor_representation;
    res = cbor::from_cbor(output, cbor_data);
    test(std::get<0>(output), std::get<0>(input));
    test(std::get<1>(output), std::get<1>(input));
    test_result(res, cbor_data);
  }
  // test vector
  {
    std::vector<unsigned int> input{ 1, 2 };
    Data expected = { 0x82, 0x01, 0x02 };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    std::vector<unsigned int> output;
    const Data cbor_data = cbor_representation;
    res = cbor::from_cbor(output, cbor_data);
    test(output.size(), input.size());
    test(output[0], input[0]);
    test(output[1], input[1]);
    test_result(res, cbor_data);
  }

  // test string
  {
    std::string input{ "foo" };
    Data expected = { 0x63, 0x66, 0x6F, 0x6F };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    std::string output;
    const Data cbor_data = cbor_representation;
    res = cbor::from_cbor(output, cbor_data);
    test(input, output);
    test_result(res, cbor_data);
  }

  // test pair
  {
    std::pair<unsigned int, unsigned int> input{ 1, 2 };
    Data expected = { 0x82, 0x01, 0x02 };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    std::pair<unsigned int, unsigned int> output;
    res = cbor::from_cbor(output, cbor_representation);
    test(output.first, input.first);
    test(output.second, input.second);
    test_result(res, cbor_representation);
  }
  // test vector of pairs
  {
    using Point = std::pair<int, int>;
    using PointList = std::vector<Point>;
    PointList input{ { 3, 7 }, { 90, 800 } };
    Data expected = { 0x82, 0x82, 0x03, 0x07, 0x82, 0x18, 0x5A, 0x19, 0x03, 0x20 };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    cbor::cbor_object single_obj;
    res = cbor::to_cbor(input, single_obj);
    test(cbor::hexdump(expected), cbor::hexdump(cbor_representation));
    test_result(res, single_obj.serialized_, expected);

    PointList read_back;
    res = cbor::from_cbor(read_back, cbor_representation);
    test_result(res, cbor_representation);

    std::vector<cbor::cbor_object> as_objects;
    cbor::from_cbor(as_objects, cbor_representation);

    std::cout << "List of pairs" << std::endl;
    std::cout << "Single object:" << cbor::hexdump(single_obj) << std::endl;
    std::cout << single_obj.prettyPrint() << std::endl;
  }
}

void test_array()
{
  std::cout << "Test " << __FUNCTION__ << " " << __LINE__ << std::endl;
  // Serializing into and from array:
  {
    unsigned int input{ 2 };
    Data result = { 0x02 };
    std::array<cbor::DataType, 100> z;
    auto res = cbor::to_cbor(input, z.data(), z.size());
    test(cbor::hexdump(result), cbor::hexdump(z, res));
    test(bool(res), true);
    test(std::size_t(res), result.size());

    unsigned int read_back;

    const cbor::DataType* offset = z.data();
    std::uint8_t size = z.size();
    cbor::from_cbor(read_back, offset, size);
    cbor::from_cbor(read_back, z.data(), z.size());
  }
  {
    unsigned int input{ 2 };
    Data result = { 0x02 };
    cbor::DataType z[300];
    std::size_t len = cbor::to_cbor(input, z);
    test(cbor::hexdump(result), cbor::hexdump(z, len));

    unsigned int read_back;

    const cbor::DataType* offset = z;
    std::uint16_t size = 300;
    cbor::from_cbor(read_back, z);
    test(read_back, input);
    cbor::from_cbor(read_back, offset, size);
    test(read_back, input);
  }

  {
    unsigned int my_int_array[2] = { 1, 2 };
    Data result = { 0x82, 0x01, 0x02 };
    std::array<cbor::DataType, 100> z;
    auto res = cbor::to_cbor(my_int_array, z.data(), z.size());
    test(cbor::hexdump(result), cbor::hexdump(z, res));
    test(bool(res), true);
    test(std::size_t(res), result.size());

    unsigned int read_back[2] = { 0, 0 };
    cbor::from_cbor(read_back, z.data(), z.size());
    test(read_back[0], my_int_array[0]);
    test(read_back[1], my_int_array[1]);
  }

  {
    std::array<unsigned int, 2> my_int_array = { 1, 2 };
    Data result = { 0x82, 0x01, 0x02 };
    std::array<cbor::DataType, 100> z;
    auto res = cbor::to_cbor(my_int_array, z.data(), z.size());
    test(cbor::hexdump(result), cbor::hexdump(z, res));
    test(bool(res), true);
    test(std::size_t(res), result.size());

    std::array<unsigned int, 2> read_back = { 0, 0 };
    cbor::from_cbor(read_back, z.data(), z.size());
    test(read_back[0], my_int_array[0]);
    test(read_back[1], my_int_array[1]);
  }
}

namespace foo
{
struct Bar
{
  std::uint32_t f;
};

template <typename... Data>
cbor::result to_cbor(const Bar& b, cbor::detail::write_adapter<Data...>& data)
{
  std::cout << "to cbor adl" << std::endl;
  return to_cbor(b.f, data);
}

template <typename... Data>
cbor::result from_cbor(Bar& b, cbor::detail::read_adapter<Data...>& data)
{
  std::cout << "from_cbor adl" << std::endl;
  return from_cbor(b.f, data);
}
}  // namespace foo

void test_adl()
{
  std::cout << "Test " << __FUNCTION__ << " " << __LINE__ << std::endl;
  {
    foo::Bar input{ 2 };
    Data expected = { 0x02 };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);
  }
  {
    std::vector<foo::Bar> input = { foo::Bar{ 2 }, foo::Bar{ 3 }, foo::Bar{ 4 } };
    Data expected = { 0x83, 0x02, 0x03, 0x04 };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    std::vector<foo::Bar> output;
    res = cbor::from_cbor(output, cbor_representation);
    test_result(res, cbor_representation);
    test(output.size(), 3u);
    if (output.size() == 3)
    {
      test(output[0].f, 2u);
      test(output[1].f, 3u);
      test(output[2].f, 4u);
    }
  }
}

void test_pod()
{
  std::cout << "Test " << __FUNCTION__ << " " << __LINE__ << std::endl;
  tester<std::uint8_t>(50, { 0x18, 0x32 });
  tester<std::int8_t>(-50, { 0x38, 0x31 });
  tester<std::uint16_t>(50, { 0x18, 0x32 });
  tester<std::uint32_t>(2, { 0x02 });
  tester<double>(13377.1414, { 0xFB, 0x40, 0xCA, 0x20, 0x92, 0x19, 0x65, 0x2B, 0xD4 });
  tester<float>(6.3125, { 0xf9, 0x46, 0x50 });
  tester<double>(6.3125, { 0xf9, 0x46, 0x50 });
  tester<std::int32_t>(2, { 0x02 });
  tester<std::int32_t>(-2, { 0x21 });
  tester<bool>(true, { 0xF5 });
  tester<bool>(false, { 0xF4 });

  {
    Data result = { 0xF6 };
    Data cbor_representation;
    cbor::to_cbor(nullptr, cbor_representation);
    test(cbor::hexdump(result), cbor::hexdump(cbor_representation));
  }
}

namespace cbor_object_ser
{
struct Buz
{
  std::uint32_t f;
};
/*
 */
std::size_t to_cbor(const Buz& b, cbor::cbor_object& data)
{
  std::cout << "to cbor_object_ser adl " << std::endl;
  return to_cbor(b.f, data);
  return 0;
}
}  // namespace cbor_object_ser

/*
 */
void test_into_object()
{
  std::cout << "Test " << __FUNCTION__ << " " << __LINE__ << std::endl;
  {
    cbor::cbor_object cbor_representation;
    unsigned int input{ 2 };
    Data expected = { 0x02 };
    auto res = to_cbor(input, cbor_representation);
    test_result(res, cbor_representation.serialized_, expected);
  }

  {
    cbor::cbor_object cbor_representation;
    cbor_object_ser::Buz z{ 2 };
    Data expected = { 0x02 };
    auto res = cbor::to_cbor(z, cbor_representation);
    test_result(res, cbor_representation.serialized_, expected);
  }

  {
    unsigned int input{ 2 };
    Data expected = { 0x02 };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    cbor::cbor_object cbor_reconstruct;
    const Data cbor_repr = cbor_representation;
    cbor::from_cbor(cbor_reconstruct, cbor_repr);
    test(cbor::hexdump(cbor_reconstruct.serialized_), cbor::hexdump(cbor_representation));
  }

  {
    std::vector<unsigned int> input{ 1, 2 };
    Data expected = { 0x82, 0x01, 0x02 };
    cbor::cbor_object cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation.serialized_, expected);

    cbor::cbor_object cbor_res;
    const Data cbor_data = cbor_representation.serialized_;
    res = cbor::from_cbor(cbor_res, cbor_data);
    test(cbor::hexdump(cbor_res.serialized_), cbor::hexdump(cbor_data));
    test_result(res, cbor_res.serialized_);
  }

  {
    // test non homogeneous.
    std::vector<cbor::cbor_object> z;

    z.resize(3);
    unsigned int x = 1337;
    double f = 13.37;
    std::map<int, cbor::cbor_object> y;
    cbor::cbor_object my_three;
    std::string foo = "foo";
    cbor::to_cbor(foo, my_three);
    y[-3] = my_three;
    cbor::to_cbor(x, z[0]);
    cbor::to_cbor(f, z[1]);
    cbor::to_cbor(y, z[2]);
    Data repr;

    cbor::to_cbor(z, repr);
    cbor::cbor_object cbor_res;
    Data cbor_data = repr;
    cbor::from_cbor(cbor_res, cbor_data);
    test(cbor::hexdump(cbor_res.serialized_), cbor::hexdump(cbor_data));
    std::cout << cbor_res.prettyPrint() << std::endl;
  }

  {
    // test copying indefinite array into object.
    Data input = { 0x9F, 0x04, 0x05, 0xFF };
    cbor::cbor_object cbor_representation;
    auto res = cbor::from_cbor(cbor_representation, input);
    test_result(res, cbor_representation.serialized_, input);
  }
  {
    // test copying indefinite array into object.
    Data input = { 0x9F, 0x04, 0x05, 0xFF };
    cbor::cbor_object cbor_representation;
    auto res = cbor::from_cbor(cbor_representation, input);
    test_result(res, cbor_representation.serialized_, input);
  }
  {
    // test copying indefinite map into object.
    Data input = { 0xBF, 0x61, 0x62, 0x61, 0x63, 0xFF };
    cbor::cbor_object cbor_representation;
    auto res = cbor::from_cbor(cbor_representation, input);
    test_result(res, cbor_representation.serialized_, input);
  }
  {
    // test copying indefinite string into object.
    Data input = { 0x7f, 0x65, 0x73, 0x74, 0x72, 0x65, 0x61, 0x64, 0x6d, 0x69, 0x6e, 0x67, 0xff };
    cbor::cbor_object cbor_representation;
    auto res = cbor::from_cbor(cbor_representation, input);
    test_result(res, cbor_representation.serialized_, input);
  }
  {
    // test copying with simple true and false
    Data input = { 0x82, 0xF5, 0xF4 };
    cbor::cbor_object cbor_representation;
    auto res = cbor::from_cbor(cbor_representation, input);
    test_result(res, cbor_representation.serialized_, input);
  }
}

namespace compound_type
{
struct Bar
{
  std::uint32_t f;
  std::uint32_t x;
};

// using arrays.
template <typename Data>
cbor::result to_cbor(const Bar& b, Data& data)
{
  cbor::result res = data.openArray(2);
  res += to_cbor(b.f, data);
  res += to_cbor(b.x, data);
  return res;
}

template <typename Data>
cbor::result from_cbor(Bar& b, Data& data)
{
  cbor::result res = data.expectArray(2);
  res += from_cbor(b.f, data);
  res += from_cbor(b.x, data);
  return res;
}

struct Buz
{
  std::uint32_t f;
  std::uint32_t x;
};

// using a map
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

}  // namespace compound_type

void test_compound_type()
{
  std::cout << "Test " << __FUNCTION__ << " " << __LINE__ << std::endl;
  const bool print_compare = false;
  {
    compound_type::Bar input{ 2, 3 };
    Data expected = { 0x82, 0x02, 0x03 };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);
  }
  {
    std::vector<compound_type::Bar> input = { compound_type::Bar{ 2, 5 }, compound_type::Bar{ 3, 6 },
                                              compound_type::Bar{ 4, 7 } };
    Data expected = { 0x83, 0x82, 0x02, 0x05, 0x82, 0x03, 0x06, 0x82, 0x04, 0x07 };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    std::vector<compound_type::Bar> output;
    res = cbor::from_cbor(output, cbor_representation);
    test_result(res, cbor_representation);
    test(output.size(), 3u, print_compare);
    if (output.size() == 3)
    {
      test(output[0].f, 2u, print_compare);
      test(output[0].x, 5u, print_compare);
      test(output[1].f, 3u, print_compare);
      test(output[1].x, 6u, print_compare);
      test(output[2].f, 4u, print_compare);
      test(output[2].x, 7u, print_compare);
    }
  }

  {
    std::vector<compound_type::Buz> input = { compound_type::Buz{ 2, 5 }, compound_type::Buz{ 3, 6 },
                                              compound_type::Buz{ 4, 7 } };
    Data expected = { 0x83, 0xA2, 0x61, 0x66, 0x02, 0x61, 0x78, 0x05, 0xA2, 0x61, 0x66,
                      0x03, 0x61, 0x78, 0x06, 0xA2, 0x61, 0x66, 0x04, 0x61, 0x78, 0x07 };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    {
      std::vector<cbor::cbor_object> output;
      res = cbor::from_cbor(output, cbor_representation);
      test(output.size(), 3u, print_compare);
      compound_type::Buz zz;
      cbor::from_cbor(zz, output[0]);
      test(zz.f, 2u, print_compare);
      test(zz.x, 5u, print_compare);
      cbor::from_cbor(zz, output[2]);
      test(zz.f, 4u, print_compare);
      test(zz.x, 7u, print_compare);
    }

    std::vector<compound_type::Buz> output;
    res = cbor::from_cbor(output, cbor_representation);
    test_result(res, cbor_representation);
    test(output.size(), 3u, print_compare);
    if (output.size() == 3)
    {
      test(output[0].f, 2u, print_compare);
      test(output[0].x, 5u, print_compare);
      test(output[1].f, 3u, print_compare);
      test(output[1].x, 6u, print_compare);
      test(output[2].f, 4u, print_compare);
      test(output[2].x, 7u, print_compare);
    }
  }
}

void test_result_operators()
{
  std::cout << "Test " << __FUNCTION__ << " " << __LINE__ << std::endl;
  cbor::result success1 = 5;
  cbor::result success2 = 5;
  std::cout << "success1: " << success1 << std::endl;
  std::cout << "success2: " << success2 << std::endl;
  cbor::result success21 = (success1 + success2);
  std::cout << "success(2+1): " << success21 << std::endl;
  test(success21.success, true);
  test(success21.length, 10u);

  cbor::result fail;
  std::cout << "failb: " << fail << std::endl;
  test(fail.success, true);
  fail = false;
  std::cout << "set failed: " << fail << std::endl;
  test(fail.success, false);
  cbor::result fail_plus_success = (success1 + fail);
  std::cout << "fail + success1: " << fail_plus_success << std::endl;
  test(fail_plus_success.success, false);
  test(fail_plus_success.length, 5u);

  cbor::result assign_false = false;
  test(assign_false.success, false);
  std::cout << "assign_false: " << assign_false << std::endl;

  test(static_cast<bool>(assign_false), false);

  cbor::result assign_true = true;
  std::cout << "assign_true: " << assign_true << std::endl;
  test(static_cast<bool>(assign_true), true);

  cbor::result assign_one = 1;
  std::cout << "assign_one: " << assign_one << std::endl;
  test(assign_one.success, true);
  test(assign_one.length, 1u);
  cbor::result assign_zero = 0;
  std::cout << "assign_zero: " << assign_zero << std::endl;
  test(assign_zero.success, true);
  test(assign_zero.length, 0u);
  std::uint32_t x = static_cast<std::uint32_t>(assign_one);
  std::cout << "x: " << x << std::endl;
  test(x, 1u);

  cbor::result to_add;
  cbor::result int_addition = to_add + 5u;
  std::cout << "int_addition: " << int_addition << std::endl;
  test(int_addition.success, true);
  test(int_addition.length, 5u);
  int_addition += 5;
  test(int_addition.length, 10u);

  cbor::result false_res = false;
  false_res += 5;
  std::cout << "false_res plus 5:" << false_res << std::endl;
  test(false_res.success, false);
  test(false_res.length, 5u);

  cbor::result test_result = 2;
  test_result += false_res;
  cbor::result new_res = test_result + 5;
  test(new_res.success, false);
  test(new_res.length, 12u);
}

void test_appendix_a()
{
  std::cout << "Test " << __FUNCTION__ << " " << __LINE__ << std::endl;
  // https://tools.ietf.org/html/rfc7049#appendix-A
  test_appendix_A_decode<std::uint32_t>("00", 0, true);
  test_appendix_A_decode<std::uint32_t>("01", 1, true);
  test_appendix_A_decode<std::uint32_t>("0a", 10, true);
  test_appendix_A_decode<std::uint32_t>("17", 23, true);
  test_appendix_A_decode<std::uint32_t>("1818", 24, true);
  test_appendix_A_decode<std::uint32_t>("1819", 25, true);
  test_appendix_A_decode<std::uint32_t>("1864", 100, true);
  test_appendix_A_decode<std::uint32_t>("1903e8", 1000, true);
  test_appendix_A_decode<std::uint32_t>("1a000f4240", 1000000, true);
  test_appendix_A_decode<std::uint64_t>("1b000000e8d4a51000", 1000000000000u, true);
  test_appendix_A_decode<std::uint64_t>("1bffffffffffffffff", 18446744073709551615u, true);

  // can't represent the following values:
  //  test_appendix_A_decode<std::uint64_t>("c249010000000000000000", 18446744073709551616u, true);
  //  test_appendix_A_decode<std::int64_t>("3bffffffffffffffff", -18446744073709551616, true);
  //  test_appendix_A_decode<std::int64_t>("c349010000000000000000", -18446744073709551617, true);

  test_appendix_A_decode<std::int32_t>("20", -1, true);
  test_appendix_A_decode<std::int32_t>("29", -10, true);
  test_appendix_A_decode<std::int32_t>("3863", -100, true);
  test_appendix_A_decode<std::int32_t>("3903e7", -1000, true);

  // short double types
  test_appendix_A_decode<>("f90000", 0.0, false);
  test_appendix_A_decode<>("f98000", -0.0, false);
  test_appendix_A_decode<>("f93c00", 1.0, false);
  test_appendix_A_decode<double>("fb3ff199999999999a", 1.1, true);
  test_appendix_A_decode<>("f93e00", 1.5, false);
  test_appendix_A_decode<float>("fa47c35000", 100000.0, true);
  test_appendix_A_decode<float>("fa7f7fffff", 3.4028234663852886e+38, true);
  test_appendix_A_decode<double>("fb7e37e43c8800759c", 1.0e300, true);

  test_appendix_A_decode<>("f90001", 5.960464477539063e-8, false);
  test_appendix_A_decode<>("f90400", 0.00006103515625, false);
  test_appendix_A_decode<>("f9c400", -4.0, false);
  test_appendix_A_decode<double>("fbc010666666666666", -4.1, true);
  test_appendix_A_decode<>("f97c00", std::numeric_limits<double>::infinity(), false);
  // comparison fails, because nan != nan
  //  test_appendix_A_decode<>("f97e00",  std::numeric_limits<double>::quiet_NaN(), false);
  test_appendix_A_decode<>("f9fc00", -std::numeric_limits<double>::infinity(), false);
  test_appendix_A_decode<double>("fb7ff0000000000000", std::numeric_limits<double>::infinity(), false);
  // comparison fails, because nan != nan
  //  test_appendix_A_decode<double>("fb7ff8000000000000",  std::numeric_limits<double>::quiet_NaN(), true);
  test_appendix_A_decode<double>("fbfff0000000000000", -std::numeric_limits<double>::infinity(), false);
  test_appendix_A_decode<bool>("f4", false, true);
  test_appendix_A_decode<bool>("f5", true, true);
  // can't print this.
  // test_appendix_A_decode<std::nullptr_t>("f6", nullptr, true);

  // simple type undefined
  // simple others
  // tags...

  // Strings
  test_appendix_A_decode<std::string>("60", "", true);
  test_appendix_A_decode<std::string>("6161", "a", true);
  test_appendix_A_decode<std::string>("6449455446", "IETF", true);
  test_appendix_A_decode<std::string>("62c3bc", "\u00fc", true);
  test_appendix_A_decode<std::string>("63e6b0b4", "\u6c34", true);
  //  test_appendix_A_decode<std::string>("64f0908591", "\ud800\udd51", true);   // doesnt compile, bad unicode

  test_appendix_A_decode<std::vector<std::uint32_t>>("80", {}, true);
  test_appendix_A_decode<std::vector<std::uint32_t>>("83010203", { 1, 2, 3 }, true);

  // non homogeneous
  //  test_appendix_A_decode<std::vector<std::uint32_t>>("8301820203820405", {}, true);

  test_appendix_A_decode<std::vector<std::uint32_t>>(
      "98190102030405060708090a0b0c0d0e0f101112131415161718181819",
      { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25 }, true);
  test_appendix_A_decode<std::map<std::uint32_t, std::uint32_t>>("a0", {}, true);
  test_appendix_A_decode<std::map<std::uint32_t, std::uint32_t>>("a201020304", { { 1, 2 }, { 3, 4 } }, true);

  //  test_appendix_A_decode<std::map<std::uint32_t, std::uint32_t>>("0xa26161016162820203", {}, true);
  test_appendix_A_decode<std::map<std::string, std::string>>(
      "a56161614161626142616361436164614461656145",
      { { "a", "A" }, { "b", "B" }, { "c", "C" }, { "d", "D" }, { "e", "E" } }, true);

  // Indefinite lengths
  test_appendix_A_decode<std::string>("7f657374726561646d696e67ff", "streaming", false);
  test_appendix_A_decode<std::vector<std::uint32_t>>("9fff", {}, false);
  test_appendix_A_decode<std::vector<std::uint32_t>>(
      "9f0102030405060708090a0b0c0d0e0f101112131415161718181819ff",
      { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25 }, false);

  // adapted from 0x9f018202039f0405ffff
  test_appendix_A_decode<std::vector<std::uint32_t>>("9f0405ff", { 4, 5 }, false);

  // adapted from 0x826161bf61626163ff
  test_appendix_A_decode<std::map<std::string, std::string>>("bf61626163ff", { { "b", "c" } }, false);
}

void test_exceptions()
{
  std::cout << "Test " << __FUNCTION__ << " " << __LINE__ << std::endl;
  const bool no_print = false;

  expect_error<cbor::buffer_error>([]() {
    std::string input{ "foo" };
    Data expected = { 0x63, 0x66, 0x6F };
    std::string output;
    auto res = cbor::from_cbor(output, expected);
    test(bool(res), false, no_print);
  });

  expect_error<cbor::buffer_error>([]() {  // test write buffer too small.
    std::array<std::uint8_t, 2> out;
    std::string input{ "foo" };
    auto res = cbor::to_cbor(input, out.data(), out.size());
    test(bool(res), false, no_print);
  });

  expect_error<cbor::buffer_error>([]() {  // test write buffer too small.
    std::array<std::uint8_t, 2> out;
    std::vector<std::uint32_t> input{ 1, 2, 3 };
    auto res = cbor::to_cbor(input, out.data(), out.size());
    test(bool(res), false, no_print);
  });

  expect_error<cbor::type_error>([]() {  // test reading wrong type.
    Data cbor_in = { 0x63, 0x66, 0x6F };
    std::vector<std::uint32_t> parsed;
    auto res = cbor::from_cbor(parsed, cbor_in);
    test(bool(res), false, no_print);
  });

  expect_error<cbor::type_error>([]() {  // test fail on size type.
    Data cbor_in = { 0x19, 0x03, 0xe8 };
    std::uint8_t parsed;
    auto res = cbor::from_cbor(parsed, cbor_in);
    test(bool(res), false, no_print);
  });

  expect_error<cbor::type_error>([]() {  // test fail on deserializing double into float.
    Data cbor_in = { 0xFB, 0x40, 0xCA, 0x20, 0x92, 0x19, 0x65, 0x2B, 0xD4 };
    float parsed;
    auto res = cbor::from_cbor(parsed, cbor_in);
    test(bool(res), false, no_print);
  });
}

void test_recursion()
{
  std::cout << "Test " << __FUNCTION__ << " " << __LINE__ << std::endl;
  const bool no_print = false;

  const uint8_t indefinite_array = 0b10011111;
  const uint8_t array_break = 0b11111111;
  Data input{ indefinite_array, indefinite_array, indefinite_array, array_break, array_break, array_break };
  Data expected = {};

  {
    using read_adapter = cbor::detail::get_read_adapter<Data>;
    auto wrapper = read_adapter::adapt(input);
    //  wrapper.recursion_limit = 1;
    std::vector<cbor::cbor_object> output;
    auto res = cbor::detail::from_cbor(output, wrapper);
    test(bool(res), true, no_print);
  }

  expect_error<cbor::type_error>([&]() {
    using read_adapter = cbor::detail::get_read_adapter<Data>;
    auto wrapper = read_adapter::adapt(input);
    wrapper.recursion_limit = 1;
    std::vector<cbor::cbor_object> output;
    cbor::detail::from_cbor(output, wrapper);
  });
}

void test_pointer_serialization()
{
  {
    std::shared_ptr<int> input = std::make_shared<int>(2);

    Data expected = { 0x02 };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    std::shared_ptr<int> read_back;
    cbor::from_cbor(read_back, cbor_representation);
    test(*read_back, *input);
  }
  {
    std::shared_ptr<int> input;

    Data expected = { ((0b111 << 5) | 22) };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    std::shared_ptr<int> read_back = std::make_shared<int>(1337);
    cbor::from_cbor(read_back, cbor_representation);
    test(read_back == nullptr, true);
    test(input == nullptr, true);
  }

  {
    std::unique_ptr<int> input = std::make_unique<int>(2);

    Data expected = { 0x02 };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    std::unique_ptr<int> read_back;
    cbor::from_cbor(read_back, cbor_representation);
    test(*read_back, *input);
  }
  {
    std::unique_ptr<int> input;

    Data expected = { ((0b111 << 5) | 22) };
    Data cbor_representation;
    auto res = cbor::to_cbor(input, cbor_representation);
    test_result(res, cbor_representation, expected);

    std::unique_ptr<int> read_back = std::make_unique<int>(1337);
    cbor::from_cbor(read_back, cbor_representation);
    test(read_back == nullptr, true);
    test(input == nullptr, true);
  }
}

int main(int /* argc */, char** /* argv */)
{
  test_pod();
  test_stl();
  test_array();
  test_adl();
  test_into_object();
  test_compound_type();
  test_result_operators();
  test_appendix_a();
  test_exceptions();
  test_recursion();
  test_pointer_serialization();

  if (failed)
  {
    std::cerr << "\033[31m"
              << "FAIL"
              << "\033[0m" << std::endl;
  }
  else
  {
    std::cerr << "\033[32m"
              << "Success fully passed " << test_done << " tests."
              << "\033[0m" << std::endl;
  }
  return failed;
}
