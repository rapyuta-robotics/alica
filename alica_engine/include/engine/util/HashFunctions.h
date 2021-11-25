#pragma once
#include <stdint.h>

#include "engine/util/cityhash.h"

// Thin wrapper over the used hash function:
namespace alica
{

// TODO: implement a hash function for integers that avoids collissions
std::size_t contextHash(int64_t value);

// TODO: implement a function that can combine 2 hashes
std::size_t contextHashCombine(std::size_t h1, std::size_t h2);

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