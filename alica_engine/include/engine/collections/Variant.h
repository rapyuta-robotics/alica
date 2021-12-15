#pragma once
#include <assert.h>
#include <stdint.h>
#include <string>
#include <variant>

namespace alica
{

using Variant = std::variant<std::monostate, double, float, bool, int64_t>;

namespace variant{

inline bool isSet(const Variant& var) {
    return (var.index() != 0);
}

/*
* std::variant to byte array
*/
void serializeTo(std::string& arr, const Variant& var);

/*
* Byte array to std::variant
*/
void loadFrom(const std::string& arr, Variant& var);

} // namespace variant

} // namespace alica