#pragma once

#include "ConditionFactory.h"

#include <vector>

namespace alica {
    class PostCondition;
    class PostConditionFactory : public ConditionFactory {
    public:
        static PostCondition* create(const YAML::Node& postConditionNode, AbstractPlan* plan);
    };
}
