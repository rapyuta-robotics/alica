#include "engine/default/DefaultTransitionConditionCreator.h"

#include "engine/Types.h"
#include "engine/default/DefaultConditions.h"
#include <iostream>

namespace alica
{

TransitionConditionCallback DefaultTransitionConditionCreator::createConditions(const std::string& name) const
{
    if (name == "DefaultCondition") {
        return std::bind(defaultCondition, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    }
    std::cerr << "DefaultTransitionConditionCreator: Unknown condition requested: " << name << std::endl;
    throw new std::exception();
}

bool DefaultTransitionConditionCreator::isDefaultTransitionCondition(const std::string& name) const
{
    if (name == "DefaultCondition") {
        return true;
    }
    return false;
}
} /* namespace alica */
