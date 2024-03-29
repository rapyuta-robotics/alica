#include "engine/model/Plan.h"

#include <assert.h>

#include "engine/ConfigChangeListener.h"
#include "engine/blackboard/BlackboardBlueprint.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/PreCondition.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/model/Variable.h"

namespace alica
{
Plan::Plan(ConfigChangeListener& configChangeListener, int64_t id)
        : AbstractPlan(id)
        , _minCardinality(0)
        , _maxCardinality(0)
        , _utilityFunction(nullptr)
        , _utilityThreshold(1.0)
        , _runtimeCondition(nullptr)
        , _preCondition(nullptr)
        , _frequency(0)
        , _blackboardBlueprint(nullptr)
        , _libraryName("")
{
    auto reloadFunctionPtr = std::bind(&Plan::reload, this, std::placeholders::_1);
    configChangeListener.subscribe(reloadFunctionPtr);
    reload(configChangeListener.getConfig());
}

Plan::~Plan() {}

const EntryPoint* Plan::getEntryPointTaskID(int64_t taskID) const
{
    for (const EntryPoint* ep : _entryPoints) {
        const Task* task = ep->getTask();
        assert(task != nullptr);
        if (task->getId() == taskID) {
            return ep;
        }
    }
    return nullptr;
}

void Plan::reload(const YAML::Node& config)
{
    _authorityTimeInterval = AlicaTime::milliseconds(config["Alica"]["CycleDetection"]["MinimalAuthorityTimeInterval"].as<unsigned long>());
}

const EntryPoint* Plan::getEntryPointByID(int64_t epID) const
{
    for (const EntryPoint* ep : _entryPoints) {
        if (ep->getId() == epID) {
            return ep;
        }
    }
    return nullptr;
}

const State* Plan::getStateByID(int64_t id) const
{
    for (const State* s : _states) {
        if (s->getId() == id) {
            return s;
        }
    }
    return nullptr;
}

void Plan::setEntryPoints(const EntryPointGrp& entryPoints)
{
    _entryPoints = entryPoints;
}

void Plan::setFailureStates(const FailureStateGrp& failureStates)
{
    _failureStates = failureStates;
}

void Plan::setMaxCardinality(int maxCardinality)
{
    _maxCardinality = maxCardinality;
}

void Plan::setMinCardinality(int minCardinality)
{
    _minCardinality = minCardinality;
}

void Plan::setSuccessStates(const SuccessStateGrp& successStates)
{
    _successStates = successStates;
}

std::string Plan::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#Plan: " << AbstractPlan::toString(indent);
    ss << indent << "\tUtility Threshold: " << this->_utilityThreshold << std::endl;
    ss << indent << "\tfrequency: " << _frequency << std::endl;
    if (this->_preCondition != nullptr) {
        ss << this->_preCondition->toString(indent);
    }
    if (this->_runtimeCondition != nullptr) {
        ss << this->_runtimeCondition->toString(indent);
    }
    for (const EntryPoint* ep : this->_entryPoints) {
        ss << ep->toString(indent + "\t");
    }
    for (const State* state : this->_states) {
        ss << state->toString(indent + "\t");
    }
    for (const Variable* var : this->getVariables()) {
        ss << var->toString(indent + "\t");
    }
    ss << indent << "\tlibraryname: " << _libraryName << std::endl;

    ss << indent << "#EndPlan:" << std::endl;
    return ss.str();
}

void Plan::setAuthorityTimeInterval(AlicaTime authorithyTimeInterval) const
{
    _authorityTimeInterval = authorithyTimeInterval;
}

} // namespace alica
