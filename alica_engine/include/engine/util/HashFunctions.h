#pragma once
#include <stdint.h>

#include "engine/util/cityhash.h"
#include <functional>

// Thin wrapper over the used hash function:
namespace alica
{

inline std::size_t hashCombine(std::size_t h1, std::size_t h2)
{
    // Simple hash_combine, see e.g. here:
    // https://github.com/HowardHinnant/hash_append/issues/7
    // Not sure we ever need 32bit, but here it is...
    if constexpr (sizeof(std::size_t) == 4) {
        h1 ^= h2 + 0x9e3779b9U + (h1 << 6) + (h1 >> 2);
    } else if constexpr (sizeof(std::size_t) == 8) {
        h1 ^= h2 + 0x9e3779b97f4a7c15LLU + (h1 << 12) + (h1 >> 4);
    }
    return h1;
}

inline std::size_t contextHash(int64_t value)
{
    return CityHash64(reinterpret_cast<const char*>(&value), sizeof(int64_t));
}

inline std::size_t contextHash(std::size_t parentContextHash, int64_t v1, int64_t v2)
{
    return hashCombine(parentContextHash, hashCombine(contextHash(v1), contextHash(v2)));
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