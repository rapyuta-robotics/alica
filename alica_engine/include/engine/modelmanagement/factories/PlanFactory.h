#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class Plan;
class PlanFactory : public Factory
{
public:
    static Plan* create(const YAML::Node& node);
private:
    PlanFactory() = delete;
};
} // namespace alica
