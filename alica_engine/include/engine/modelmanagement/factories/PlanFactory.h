#pragma once

#include <yaml-cpp/yaml.h>

namespace alica
{
class Plan;
class PlanFactory
{
public:
    static Plan* create(const YAML::Node& node);
private:
    PlanFactory() = delete;
};
} // namespace alica
