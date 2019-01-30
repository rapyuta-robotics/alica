#pragma once

#include <stdint.h>

namespace essentials
{

class CustomHashes
{
public:
    static const uint64_t FNV_MAGIC_PRIME = 0x00000100000001b3; /**< Constant prime for hashing 64bit wise according to FNV Hash Algorithm */
    static const uint64_t FNV_OFFSET = 0xcbf29ce484222325;      /**< Constant offset for hashing 64bit wise according to FNV Hash Algorithm */
};

} /* namespace essentials */

