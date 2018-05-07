/*
 * PartialAssignmentPool.cpp
 *
 *  Created on: 13.10.2014
 *      Author: Andreas Witsch
 */

#include "engine/planselector/PartialAssignmentPool.h"
#include "engine/planselector/PartialAssignment.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"
#include "engine/parser/ModelFactory.h"

namespace alica {
const int PartialAssignmentPool::maxCount = 10100;

PartialAssignmentPool::PartialAssignmentPool()
        : daPAs(maxCount)
        , curIndex(0) {
    // IDLE-EntryPoint
    idleEP = ModelFactory::generateIdleEntryPoint();
    idleTask = idleEP->getTask();

    for (int i = 0; i < maxCount; i++) {
        daPAs[i] = new PartialAssignment(this);
    }
}

PartialAssignmentPool::~PartialAssignmentPool() {
    for (int i = 0; i < maxCount; i++) {
        delete daPAs[i];
    }
    delete idleEP;
    delete idleTask;
}

}  // namespace alica
