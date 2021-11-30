#pragma once
#include <stdint.h>

#include "engine/util/cityhash.h"
#include <functional>

// Thin wrapper over the used hash function:
namespace alica
{

template <class T>
struct dependent_false : std::false_type
{
};

template <class T>
inline void hash_combine(std::size_t& seed, const T& value)
{
    // Simple hash_combine, see e.g. here:
    // https://github.com/HowardHinnant/hash_append/issues/7
    // Not sure we ever need 32bit, but here it is...
    if constexpr (sizeof(std::size_t) == 4) {
        seed ^= std::hash<T>{}(value) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    } else if constexpr (sizeof(std::size_t) == 8) {
        seed ^= std::hash<T>{}(value) + 0x9e3779b97f4a7c15LLU + (seed << 12) + (seed >> 4);
    } else {
        static_assert(dependent_false<T>::value, "hash_combine not implemented");
    }
}

std::size_t contextHash(int64_t value)
{
    return std::hash<int64_t>{}(value);
}

std::size_t contextHashCombine(std::size_t h1, std::size_t h2)
{
    hash_combine(h1, h2);
    return h1;
}

inline std::size_t contextHash(std::size_t parentContextHash, int64_t v1, int64_t v2)
{
    return contextHashCombine(parentContextHash, contextHashCombine(contextHash(v1), contextHash(v2)));
}

inline uint64_t Hash64(const uint8_t* buf, size_t len)
{
    return CityHash64(reinterpret_cast<const char*>(buf), len);
}

inline uint64_t Hash64(const int8_t* buf, size_t len)
{
    return CityHash64(reinterpret_cast<const char*>(buf), len);
}

inline uint64_t Hash64(const char* buf, size_t len)
{
    return CityHash64(buf, len);
}

inline uint64_t Hash64WithSeed(uint64_t seed, const char* buf, size_t len)
{
    return CityHash64WithSeed(buf, len, seed);
}

} // namespace alica