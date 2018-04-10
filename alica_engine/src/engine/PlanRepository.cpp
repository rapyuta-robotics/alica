/*
 * PlanRepository.cpp
 *
 *  Created on: May 28, 2014
 *      Author: Paul Panin
 */

#include "engine/PlanRepository.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Quantifier.h"
#include "engine/model/Plan.h"

#include <cassert>

namespace alica {
PlanRepository::PlanRepository() {}
PlanRepository::~PlanRepository() {}

bool PlanRepository::verifyPlanBase() const {
    for (const EntryPoint* ep : getEntryPoints()) {
        if (ep->getTask() == nullptr) {
            std::cerr << "EntryPoint " << ep->toString() << " does not have a task." << std::endl;
            assert(false);
            return false;
        }
    }
    for (const Plan* p : getPlans()) {
        for (int i = 0; i < p->getEntryPoints().size() - 1; ++i) {
            if (p->getEntryPoints()[i]->getId() >= p->getEntryPoints()[i + 1]->getId()) {
                std::cerr << "Wrong sorting of entrypoints in plan " << p->toString() << std::endl;
                assert(false);
                return false;
            }
        }
    }

    return true;
}

}  // namespace alica