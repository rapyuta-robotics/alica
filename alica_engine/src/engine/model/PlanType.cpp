/*
 * PlanType.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/PlanType.h"
#include "engine/model/Plan.h"

namespace alica {

PlanType::PlanType() {}

PlanType::~PlanType() {}

std::string PlanType::toString() const {
    stringstream ss;
    ss << "#PlanType: " << getName() << " " << _getId() << std::endl;
    ss << "\t Plans: " << _plans.size() << endl;
    if (_plans.size() != 0) {
        for (const Plan* p : _plans) {
            ss << "\t" << p->getId() << " " << p->getName() << std::endl;
        }
    }
    ss << "#EndPlanType" << std::endl;
    return ss.str();
}

//====================== Getter and Setter =========================

const std::string& PlanType::getFileName() const {
    if (this->fileName.empty()) {
        static string result = this->name + ".pty";
        return result;
    } else {
        return this->fileName;
    }
}


void PlanType::setParametrisation(const list<Parametrisation*> parametrisation) {
    _parametrisation = parametrisation;
}

void PlanType::setPlans(const PlanSet& plans) {
    _plans = plans;
}

}  // namespace alica
