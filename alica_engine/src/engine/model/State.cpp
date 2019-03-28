#include "engine/model/State.h"

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

std::string State::toString() const
{
    AlicaElement::toString();
    std::stringstream ss;
    ss << "#State: " << getName() << " " << getId() << std::endl;
    ss << "\t Parent Plan: " << ((AlicaElement*)_inPlan)->getName() << " " << ((AlicaElement*)_inPlan)->getId() << std::endl;
    ss << "\t InTransitions: " << std::endl;
    for (const Transition* trans : _inTransitions) {
        ss << "\t" << ((AlicaElement*)trans)->toString() << std::endl;
    }
    ss << std::endl;
    ss << "\t OutTransitions: " << std::endl;
    for (const Transition* trans : _outTransitions) {
        ss << "\t" << ((AlicaElement*)trans)->toString() << std::endl;
    }
    ss << std::endl;
    ss << "\t Abstract Plans: " << std::endl;
    for (const AbstractPlan* plans : _plans) {
        ss << "\t" << ((AlicaElement*)plans)->toString() << std::endl;
    }
    ss << std::endl;
    ss << "\t Variable Bindings: " << std::endl;
    for (const VariableBinding* binding : _variableBindingGrp) {
        ss << "\t" << ((AlicaElement*)binding)->toString() << std::endl;
    }
    ss << std::endl;
    ss << "#EndState" << std::endl;
    return ss.str();
}

} // namespace alica
