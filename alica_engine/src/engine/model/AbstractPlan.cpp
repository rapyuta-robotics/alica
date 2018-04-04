/*
 * AbstractPlan.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */
#include <sstream>

#include "engine/model/AbstractPlan.h"
#include "engine/model/Variable.h"

namespace alica {

AbstractPlan::AbstractPlan()
        : AlicaElement() 
        , _utilityThreshold(1.0) 
        , _masterPlan(false)
        , _utilityFunction(nullptr)
        , _preCondition(nullptr)
        , _runtimeCondition(nullptr)

{
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    _authorityTimeInterval =
            (*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection", "MinimalAuthorityTimeInterval", NULL) *
            1000000;
}

AbstractPlan::AbstractPlan(int64_t id)
        : AlicaElement(id) {
        , _utilityThreshold(1.0) 
        , _masterPlan(false)
        , _utilityFunction(nullptr)
        , _preCondition(nullptr)
        , _runtimeCondition(nullptr)
{
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    _authorityTimeInterval =
            (*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection", "MinimalAuthorityTimeInterval", NULL) *
            1000000;
}

AbstractPlan::~AbstractPlan() {}

std::string AbstractPlan::toString() const {
    std::stringstream ss;
    ss << AlicaElement::toString();
    ss << "IsMasterPlan: " << (isMasterPlan() ? "true" : "false") << std::endl;
    return ss.str();
}

/**
 * Tests whether a given variable belongs to this plan.
 * @param v A Variable*
 * @return A bool
 */
bool AbstractPlan::containsVar(const Variable* v) const {
    return find(variables->begin(), variables->end(), v) != variables->end();
}

bool AbstractPlan::containsVar(const std::string& name) const {
    
    for (const Variable* v : *variables) {
        if (v->getName() == name) {
            return true;
        }
    }
    return false;
}

void AbstractPlan::setMasterPlan(bool masterPlan) {
    _masterPlan = masterPlan;
}


void AbstractPlan::setAuthorityTimeInterval(AlicaTime authorithyTimeInterval) const {
    _authorityTimeInterval = authorithyTimeInterval;
}

void AbstractPlan::setFileName(const std::string& fileName) {
    _fileName = fileName;
}

void AbstractPlan::setVariables(shared_ptr<list<Variable*>> variables) {
    _variables = variables;
}

void AbstractPlan::setRuntimeCondition(RuntimeCondition* runtimeCondition) {
    _runtimeCondition = runtimeCondition;
}

void AbstractPlan::setPreCondition(PreCondition* preCondition) {
    _preCondition = preCondition;
}

void AbstractPlan::setUtilityFunction(shared_ptr<UtilityFunction> utilityFunction) {
    _utilityFunction = utilityFunction;
}

void AbstractPlan::setUtilityThreshold(double utilityThreshold) {
    _utilityThreshold = utilityThreshold;
}

}  // namespace alica
