/*
 * Capability.cpp
 *
 *  Created on: May 28, 2014
 *      Author: Paul Panin
 */

#include "engine/model/Capability.h"
#include "engine/model/CapValue.h"

#include <iostream>
#include <exception>

namespace alica {
Capability::Capability() {}

Capability::~Capability() {}

/**
 * Computes the similarity between two capability values.
 * @param roleVal Role value
 * @param robotVal Robot value
 * @return The value, ranges between 0 and 1.
 */
double Capability::similarityValue(const CapValue* roleVal, const CapValue* robotVal) const {
    const int nCount = _capValues.size();

    int rlIndex = -1;
    int rbIndex = -1;
    int index = 0;

    // determine the index of both given capability values
    for (const CapValue* cap : _capValues) {
        if (cap == roleVal) {
            rlIndex = index;
        }
        if (cap == robotVal) {
            rbIndex = index;
        }
        ++index;
    }
    // TODO: get rid of exceptions
    if (rlIndex == -1) {
        std::cout << "Capability::similarityValue: Role not found!" << std::endl;
        throw std::exception();
    }
    if (rbIndex == -1) {
        std::cout << "Capability::similarityValue: Robot not found!" << std::endl;
        throw std::exception();
    }

    if (nCount == 1) {
        // we found both values and there is only one value, so both must be the same
        return 1;
    }

    // this won't work, in case of only one value (nCount=1), therefore extra handling above
    return (nCount - 1 - abs(rlIndex - rbIndex)) / (nCount - 1);
}

}  // namespace alica
