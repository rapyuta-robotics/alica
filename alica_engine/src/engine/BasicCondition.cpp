/*
 * BasicCondition.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#include <engine/BasicCondition.h>
#include <engine/AlicaEngine.h>
#include <engine/RunningPlan.h>

namespace alica {

BasicCondition::BasicCondition() {}

BasicCondition::~BasicCondition() {}

bool BasicCondition::isStateTimedOut(unsigned long timeOut, shared_ptr<RunningPlan> rp) {
    if (rp->getStateStartTime() == 0)
        return false;
    long time = (long) (rp->getAlicaEngine()->getAlicaClock()->now());
    long timeDiff = time - (long) (rp->getStateStartTime());
    if (timeDiff > timeOut) {
        return true;
    }
    return false;
}

bool BasicCondition::isTimeOut(unsigned long timeOut, unsigned long startTime, shared_ptr<RunningPlan> rp) {
    if (startTime == 0)
        return false;
    long time = (long) (rp->getAlicaEngine()->getAlicaClock()->now());
    long timeDiff = time - (long) (startTime);
    if (timeDiff > timeOut) {
        return true;
    }
    return false;
}
} /* namespace alica */
