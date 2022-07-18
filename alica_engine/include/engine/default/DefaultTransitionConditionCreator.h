#pragma once

#include "engine/Types.h"
#include <iostream>
#include <string>

namespace alica
{

class DefaultTransitionConditionCreator
{
public:
    TransitionConditionCallback createConditions(const std::string& name) const;
    bool isDefaultTransitionCondition(const std::string& name) const;
};

} /* namespace alica */
