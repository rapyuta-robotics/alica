/*
 * DynCardinality.cpp
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#include <engine/planselector/DynCardinality.h>

namespace alica {

DynCardinality::DynCardinality(int min, int max) {
    this->min = min;
    this->max = max;
}

DynCardinality::~DynCardinality() {}

int DynCardinality::getMax() {
    return max;
}

void DynCardinality::setMax(int max) {
    this->max = max;
}

int DynCardinality::getMin() {
    return min;
}

DynCardinality::DynCardinality() {
    this->max = 0;
    this->min = 0;
}

void DynCardinality::setMin(int min) {
    this->min = min;
}

} /* namespace alica */
