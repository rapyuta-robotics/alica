#pragma once

#include "Factory.h"

namespace alica {
    class Variable;
    class AbstractPlanFactory: public Factory {
    public:
        static void setVariables(const YAML::Node& abstractPlanNode, AbstractPlan* abstractPlan);
    };
}
