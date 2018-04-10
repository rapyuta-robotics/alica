/*
 * AbstractPlan.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */


#include "engine/model/AbstractPlan.h"
#include "engine/model/Variable.h"
#include <SystemConfig.h>
#include <sstream>

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
        : AlicaElement(id)
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
    ss << "Filename: " << _fileName << std::endl;
    return ss.str();
}

/**
 * Tests whether a given variable belongs to this plan.
 * @param v A Variable*
 * @return A bool
 */
bool AbstractPlan::containsVar(const Variable* v) const {
    return find(_variables.begin(), _variables.end(), v) != _variables.end();
}

bool AbstractPlan::containsVar(const std::string& name) const {
    
    for (const Variable* v : _variables) {
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

void AbstractPlan::setVariables(const VariableSet& variables) {
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
