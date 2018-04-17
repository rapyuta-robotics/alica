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

bool BasicCondition::isStateTimedOut(const AlicaTime timeOut, shared_ptr<RunningPlan> rp) {
    if (rp->getStateStartTime() == AlicaTime::zero()) return false;
    AlicaTime timeDiff = rp->getAlicaEngine()->getAlicaClock()->now() - rp->getStateStartTime();
    return timeDiff > timeOut;
}

bool BasicCondition::isTimeOut(const AlicaTime timeOut, const AlicaTime startTime, shared_ptr<RunningPlan> rp) {
    if (startTime == AlicaTime::zero()) return false;
    AlicaTime timeDiff = rp->getAlicaEngine()->getAlicaClock()->now() - startTime;
    return timeDiff > timeOut;
}
} /* namespace alica */
