#pragma once
#include <assert.h>
#include <stdint.h>
#include <string.h> //for memcopy
#include <variant>

namespace alica
{

using Variant = std::variant<std::monostate, double, float, bool, void*, int64_t>;

namespace variant{

static constexpr size_t kUnionSize = sizeof(double) > sizeof(void*) ? sizeof(double) : sizeof(void*);
static constexpr size_t kVariantSize = kUnionSize + 1;

inline bool isSet(const Variant& var) {
    return (var.index() != 0);
}

/*
* std::variant to byte array
*/
int serializeTo(uint8_t* arr, const Variant& var);

/*
* Byte array to std::variant
*/
int loadFrom(const uint8_t* arr, Variant& var);

} // namespace variant

} // namespace alica