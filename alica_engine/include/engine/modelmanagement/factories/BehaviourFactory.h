#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
    class Behaviour;
    class BehaviourFactory : public Factory
    {
    public:
        static Behaviour* create(const YAML::Node& node);
        static void attachReferences();
    private:
        BehaviourFactory() = delete;
    };
} // namespace alica
