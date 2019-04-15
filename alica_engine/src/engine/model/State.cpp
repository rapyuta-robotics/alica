#include "engine/model/State.h"
#include "engine/model/AbstractPlan.h"

#include <sstream>
#include <iostream>

namespace alica
{

/**
 * Basic constructor
 */
State::State()
        : AlicaElement(0)
        , _type(NORMAL)
        , _inPlan(nullptr)
        , _entryPoint(nullptr)
{
}

State::State(StateType t)
        : AlicaElement(0)
        , _type(t)
        , _inPlan(nullptr)
        , _entryPoint(nullptr)
{
}

State::~State() {}

void State::setInPlan(const Plan* inPlan)
{
    _inPlan = inPlan;
}

void State::setInTransitions(const TransitionGrp& inTransitions)
{
    _inTransitions = inTransitions;
}

void State::setOutTransitions(const TransitionGrp& outTransition)
{
    _outTransitions = outTransition;
}

void State::setVariableBindings(const VariableBindingGrp& variableBindingGrp)
{
    _variableBindingGrp = variableBindingGrp;
}

void State::setPlans(const AbstractPlanGrp& plans)
{
    _plans = plans;
}

std::string State::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#State: " << getName() << " " << getId() << std::endl;
    ss << indent << "\tParent Plan: " << ((AlicaElement*)_inPlan)->getName() << " " << ((AlicaElement*)_inPlan)->getId() << std::endl;
    ss << indent << "\tInTransitions: " << std::endl;
    for (const Transition* trans : _inTransitions) {
        ss << ((AlicaElement*)trans)->toString(indent + "\t\t");
    }
    ss << indent << "\tOutTransitions: " << std::endl;
    for (const Transition* trans : _outTransitions) {
        ss << ((AlicaElement*)trans)->toString(indent + "\t\t");
    }
    ss << indent << "\tAbstract Plans: " << std::endl;
    for (const AbstractPlan* plans : _plans) {
        ss << plans->AlicaElement::toString(indent + "\t\t");
    }
    ss << indent << "\tVariable Bindings: " << std::endl;
    for (const VariableBinding* binding : _variableBindingGrp) {
        ss << ((AlicaElement*)binding)->toString(indent + "\t\t");
    }
    ss << indent << "#EndState" << std::endl;
    return ss.str();
}

} // namespace alica
