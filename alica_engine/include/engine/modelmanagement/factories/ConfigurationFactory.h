#pragma once

#include "engine/modelmanagement/factories/Factory.h"

namespace alica
{
class Configuration;
class ConfigurationFactory : public Factory
{
public:
    static Configuration* create(const YAML::Node& configurationNode);
};
} // namespace alica