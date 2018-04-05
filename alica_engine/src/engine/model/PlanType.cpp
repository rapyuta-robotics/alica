/*
 * PlanType.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/PlanType.h"
#include "engine/model/Plan.h"
#include <sstream>

namespace alica {

PlanType::PlanType() {}

PlanType::~PlanType() {}

std::string PlanType::toString() const {
    std::stringstream ss;
    ss << "#PlanType: " << getName() << " " << getId() << std::endl;
    ss << "\t Plans: " << _plans.size() << std::endl;
    if (_plans.size() != 0) {
        for (const Plan* p : _plans) {
            ss << "\t" << p->getId() << " " << p->getName() << std::endl;
        }
    }
    ss << "#EndPlanType" << std::endl;
    return ss.str();
}



void PlanType::setParametrisation(const ParametrisationSet& parametrisation) {
    _parametrisation = parametrisation;
}

void PlanType::setPlans(const PlanSet& plans) {
    _plans = plans;
}

}  // namespace alica
