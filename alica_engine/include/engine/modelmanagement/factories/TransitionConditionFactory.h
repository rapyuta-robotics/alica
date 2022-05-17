#pragma once

#include "Factory.h"

namespace alica
{
class TransitionCondition;
class ConditonRepository;
class TransitionConditonFactory : public Factory
{
public:
    static TransitionCondition* create(const YAML::Node& taskNode, ConditionRepository* conditionRepository);
};
} // namespace alica
