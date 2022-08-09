#include "engine/default/DefaultTransitionConditionCreator.h"

#include "engine/Types.h"
#include "engine/default/DefaultConditions.h"
#include "engine/logging/IAlicaLogger.h"
#include "engine/logging/LoggingUtil.h"
#include <iostream>

namespace alica
{

TransitionConditionCallback DefaultTransitionConditionCreator::createConditions(const std::string& name) const
{
    if (name == "DefaultCondition") {
        return std::bind(defaultCondition, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    }
    Logging::LoggingUtil::log(Verbosity::ERROR, "DefaultTransitionConditionCreator: Unknown condition requested: ", name);
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
