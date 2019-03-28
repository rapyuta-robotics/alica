#pragma once

#include "Factory.h"

#include <vector>

namespace alica {
    class PreCondition;
    class PreConditionFactory : public Factory {
    public:
        static PreCondition* create(const YAML::Node& preconditinNode, Plan* plan);
    };
}
