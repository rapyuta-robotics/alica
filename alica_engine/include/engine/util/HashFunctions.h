#pragma once
#include <stdint.h>

#include "engine/util/cityhash.h"


//Thin wrapper over the used hash function:
namespace alica {
inline uint64_t Hash64(const char *buf, size_t len) {
    return CityHash64(buf, len);
}

inline uint64_t Hash64WithSeed(uint64_t seed, const char* buf, size_t len) {
    return CityHash64WithSeed(buf, len, seed);
}
}