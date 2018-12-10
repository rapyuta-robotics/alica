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

namespace alica
{

AbstractPlan::AbstractPlan()
        : AlicaElement()
        , _masterPlan(false)
        , _preCondition(nullptr)
        , _runtimeCondition(nullptr)

{
    essentials::SystemConfig* sc = essentials::SystemConfig::getInstance();
    _authorityTimeInterval = AlicaTime::milliseconds((*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection", "MinimalAuthorityTimeInterval", NULL));
}

AbstractPlan::AbstractPlan(int64_t id)
        : AlicaElement(id)
        , _masterPlan(false)
        , _preCondition(nullptr)
        , _runtimeCondition(nullptr)
{
    essentials::SystemConfig* sc = essentials::SystemConfig::getInstance();
    _authorityTimeInterval = AlicaTime::milliseconds((*sc)["Alica"]->get<unsigned long>("Alica", "CycleDetection", "MinimalAuthorityTimeInterval", NULL));
}

AbstractPlan::~AbstractPlan() {}

std::string AbstractPlan::toString() const
{
    std::stringstream ss;
    ss << AlicaElement::toString();
    ss << "IsMasterPlan: " << (isMasterPlan() ? "true" : "false") << std::endl;
    ss << "Filename: " << _fileName << std::endl;
    return ss.str();
}

const Variable* AbstractPlan::getVariableByName(const std::string& name) const
{
    for (const Variable* variable : _variables) {
        if (variable->getName() == name) {
            return variable;
        }
    }
    return nullptr;
}
/**
 * Tests whether a given variable belongs to this plan.
 * @param v A Variable*
 * @return A bool
 */
bool AbstractPlan::containsVar(const Variable* v) const
{
    return find(_variables.begin(), _variables.end(), v) != _variables.end();
}

bool AbstractPlan::containsVar(const std::string& name) const
{
    for (const Variable* v : _variables) {
        if (v->getName() == name) {
            return true;
        }
    }
    return false;
}

void AbstractPlan::setMasterPlan(bool masterPlan)
{
    _masterPlan = masterPlan;
}

void AbstractPlan::setAuthorityTimeInterval(AlicaTime authorithyTimeInterval) const
{
    _authorityTimeInterval = authorithyTimeInterval;
}

void AbstractPlan::setFileName(const std::string& fileName)
{
    _fileName = fileName;
}

void AbstractPlan::setVariables(const VariableGrp& variables)
{
    _variables = variables;
}

void AbstractPlan::setRuntimeCondition(RuntimeCondition* runtimeCondition)
{
    _runtimeCondition = runtimeCondition;
}

void AbstractPlan::setPreCondition(PreCondition* preCondition)
{
    _preCondition = preCondition;
}

} // namespace alica
