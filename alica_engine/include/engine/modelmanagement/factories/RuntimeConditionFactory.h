#pragma once

#include "ConditionFactory.h"

#include <vector>

namespace alica {
    class RuntimeCondition;
    class RuntimeConditionFactory : public ConditionFactory {
    public:
        static RuntimeCondition* create(const YAML::Node& runtimeConditionNode, AbstractPlan* plan);
    };
}
