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
    //[[deprecated("It will be removed in the last PR")]]
    static Plan* create(AlicaEngine* ae, const YAML::Node& node); // TOBE removed
    static Plan* create(const YAML::Node& config, ConfigChangeListener& configChangeListener, const YAML::Node& node);
    static void attachReferences();

private:
    PlanFactory() = delete;
};
} // namespace alica
