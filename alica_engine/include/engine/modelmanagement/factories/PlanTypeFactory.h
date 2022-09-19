#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class PlanType;
class PlanTypeFactory : public Factory
{
public:
    static PlanType* create(const YAML::Node& planTypeNode);
    static void attachReferences();

private:
    PlanTypeFactory() = delete;
};
} // namespace alica
