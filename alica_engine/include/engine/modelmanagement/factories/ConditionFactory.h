#pragma once

#include "Factory.h"

namespace alica {
    class Variable;
    class ConditionFactory: public Factory {
    public:
        static void fillCondition(const YAML::Node& conditionNode, Condition* condition, AbstractPlan* abstractPlan);
    };
}
