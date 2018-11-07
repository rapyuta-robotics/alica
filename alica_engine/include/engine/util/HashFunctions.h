#pragma once
#include <stdint.h>

#include "engine/util/cityhash.h"

// Thin wrapper over the used hash function:
namespace alica
{

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