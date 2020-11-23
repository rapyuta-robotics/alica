#pragma once

#include <yaml-cpp/yaml.h>

namespace alica
{
    class ConfigChangeListener
    {
    public:
        virtual void reload(const YAML::Node& config) = 0;
    };
}