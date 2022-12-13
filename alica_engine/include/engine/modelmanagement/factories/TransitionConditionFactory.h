#pragma once

#include "Factory.h"

namespace alica
{
class TransitionCondition;
class TransitionConditionRepository;

class TransitionConditionFactory : public Factory
{
public:
    static TransitionCondition* create(const YAML::Node& taskNode, TransitionConditionRepository* conditionRepository);
};
} // namespace alica
