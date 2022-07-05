#pragma once

#include "Factory.h"
#include "engine/Types.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class Plan;
class PlanFactory : public Factory
{
public:
    static Plan* create(ConfigChangeListener& configChangeListener, const YAML::Node& node);
    static void attachReferences();

private:
    PlanFactory() = delete;
};
} // namespace alica
