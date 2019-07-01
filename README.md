# cbor

Template-heavy implementation of [concise binary object representation][cbor] as described in [RFC 7049][rfc7049].
Mainly because the cbor serialization from [Nlohmann's json][nlohmann_json] library was not fast enough for my use case.
If any type cannot be serialized or deserialized this results in a compile error.

## Use

Quick example:
```cpp
using Data = std::vector<std::uint8_t>;

// Minimal standard serialization use case:
std::vector<int> input {1, 50, -10};
Data cbor_repr;
cbor::result result = cbor::to_cbor(input, cbor_repr);
// Result is implicitly convertible into std::size_t, explicitly into bool.
// Bool holds whether serialization was successful.
// The length specifies how many bytes were used for the cbor representation.
if (result)
{
  std::size_t length = result;
  std::cout << "Length: " << length << std::endl;
}

// Minimal standard deserialization use case:
std::vector<int> parsed;
cbor::result parse_result = cbor::from_cbor(parsed, cbor_repr);
// By default, this statement may throw if the types encountered in the data
// don't match the types you are parsing into.
```
There are more examples in the [example.cpp](/test/example.cpp) file. The library can be used without exceptions by
setting the `CBOR_USE_EXCEPTIONS` define to `0`. It can also be used without the standard library's containers by
not including `stl.h`, this still requires the standard library headers for the template handling.

Supported types plain old data types:
- `bool`
- `std::uint8_t`: Serializes into a cbor tiny integer if value allows.
- `std::uint16_t`: Serializes into 8 bit integer if value allows.
- `std::uint32_t`: Serializes into 16 bit integer if value allows.
- `std::uint64_t`: Serializes into 32 bit integer if value allows.
- `std::int8_t`: Serializes into a cbor tiny integer if value allows.
- `std::int16_t`: Serializes into 8 bit integer if value allows.
- `std::int32_t`: Serializes into 16 bit integer if value allows.
- `std::int64_t`: Serializes into 32 bit integer if value allows.
- `const char*`: Serializes into a cbor text string (utf-8).
- `nullptr_t`: Serializes into cbor `null` value.
- `float`: Will be serialized into short float if exact. Can be read into a double.
- `double`: Will be serialized into single precision float if exact, or short precision float if exact. Cannot be read
  into a float as precision is never degraded during (de)serialization.
- `d[N]`: Any C style array is serialized as a cbor array of `N` long, deserialization requires fixed length array.

Supported STL containers:
- `std::vector`: Serialized into fixed length cbor array, deserialization of indefinite array supported.
- `std::map`: Serialized into fixed length cbor map, deserialization of indefinite map supported.
- `std::pair`: Serialized into cbor array of two long, deserialization requires fixed length array.
- `std::tuple`: Serialized into cbor array equal to the tuple length, deserialization requires fixed length array.
- `std::array`: Handled as `d[N]`.
- `std::string`: Handled as `const char*`. Deserialization supports 'indefinite' strings.

Supported CBOR types:
- `map`, indefinite length supported for `std::map`.
- `array`, indefinite length supported for `std::vector`.
- `signed integer`
- `unsigned integer`
- `double` and `float`, `short float` are supported. Float precision is never degraded during (de)serialization.
- `boolean`
- `null`
- `text` and `byte` both treated as strings.

Supported custom type:
- `cbor_object`, this object acts as a read adapter and stores data internally into a `std::vector<std::uint8_t>`.
  Any input can be deserialized into this type, after which all methods from the `read_adapter_helper` can be used to
  introspect the data. Additionally it has a `prettyPrint()` method that produces a human readable string.

## Fuzz testing & recursion limit

This library has been tested using libFuzzer. When fuzzing we cannot rely on the nice compile time properties of the
templates, instead deserialization was performed into a `cbor_object`. Since the fuzzer liked finding stack overflows
by opening nested indefinite length arrays it is possible to limit the maximum recursion depth. This is done in the
fuzzer binary, if the parser has to deal with malicious data limiting this may be a good idea.

```
mkdir build
cd build
CC=clang-8 CXX=clang++-8 cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ../cbor/ && make fuzz_corpus_init fuzz_start
```


[cbor]: https://en.wikipedia.org/wiki/CBOR
[rfc7049]: https://tools.ietf.org/html/rfc7049
[nlohmann_json]: https://github.com/nlohmann/json/
