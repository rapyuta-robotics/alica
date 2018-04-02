/*
 * BasicTrueCondition.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#include "engine/expressionhandler/BasicTrueCondition.h"

namespace alica {

BasicTrueCondition::BasicTrueCondition() {}

BasicTrueCondition::~BasicTrueCondition() {}

bool BasicTrueCondition::evaluate(shared_ptr<RunningPlan> rp) {
    return true;
}

} /* namespace alica */
