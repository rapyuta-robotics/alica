#pragma once

#include "Factory.h"

#include <vector>

namespace alica
{
class Transition;
class TransitionFactory : public Factory
{
public:
    static Transition* create(const YAML::Node& transitionNode, Plan* plan);
    static void attachReferences();
};
} // namespace alica
