#include "engine/model/State.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/ConfAbstractPlanWrapper.h"

#include <iostream>
#include <sstream>

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

std::string State::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#State: " << getName() << " " << getId() << std::endl;
    ss << indent << "\tParent Plan: " << ((AlicaElement*) _inPlan)->getName() << " " << ((AlicaElement*) _inPlan)->getId() << std::endl;
    ss << indent << "\tInTransitions: " << std::endl;
    for (const Transition* trans : _inTransitions) {
        ss << ((AlicaElement*) trans)->toString(indent + "\t\t");
    }
    ss << indent << "\tOutTransitions: " << std::endl;
    for (const Transition* trans : _outTransitions) {
        ss << ((AlicaElement*) trans)->toString(indent + "\t\t");
    }
    ss << indent << "\tAbstract Plans: " << std::endl;
    for (const ConfAbstractPlanWrapper* wrapper : _confAbstractPlanWrappers) {
        ss << ((AlicaElement*) wrapper->getAbstractPlan())->toString(indent + "\t\t");
    }
    ss << indent << "\tVariable Bindings: " << std::endl;
    for (const VariableBinding* binding : _variableBindingGrp) {
        ss << ((AlicaElement*) binding)->toString(indent + "\t\t");
    }
    ss << indent << "#EndState" << std::endl;
    return ss.str();
}

} // namespace alica
