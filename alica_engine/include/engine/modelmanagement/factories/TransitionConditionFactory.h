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
    // Used for legacy condition support
    static TransitionCondition* createAndAttach(TransitionConditionRepository* conditionRepository, Transition* transition, int64_t id);
};
} // namespace alica
