#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
    class Behaviour;
    class BehaviourConfiguration;
    class BehaviourConfigurationFactory : public Factory
    {
    public:
        static BehaviourConfiguration* create(const YAML::Node& node, Behaviour* behaviour);
    private:
        BehaviourConfigurationFactory() = delete;
    };
} // namespace alica
