#pragma once

#include "engine/Types.h"
#include <string>
#include <iostream>

namespace alica
{

class DefaultTransitionConditionCreator
{
public:
    DefaultTransitionConditionCreator() = default;
    ~DefaultTransitionConditionCreator() = default;
    TransitionConditionCallback createConditions(std::string name) const;
    bool isDefaultTransitionCondition(std::string name) const;
};

} /* namespace alica */
