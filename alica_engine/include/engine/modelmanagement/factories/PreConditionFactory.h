#pragma once

#include "ConditionFactory.h"

#include <vector>

namespace alica
{
class PreCondition;
class PreConditionFactory : public ConditionFactory
{
public:
    static PreCondition* create(const YAML::Node& preconditinNode, AbstractPlan* plan);
};
} // namespace alica
