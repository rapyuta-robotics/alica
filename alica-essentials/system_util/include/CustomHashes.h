/*
 * CustomHashes.h
 *
 *  Created on: 9 May 2016
 *      Author: Stephan Opfer
 */

#ifndef SRC_CUSTOMHASHES_H_
#define SRC_CUSTOMHASHES_H_

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

#endif /* SRC_CUSTOMHASHES_H_ */
