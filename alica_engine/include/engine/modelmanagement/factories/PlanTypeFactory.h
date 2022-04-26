#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class PlanType;
class PlanTypeFactory : public Factory
{
public:
    [[deprecated("It will be removed in the last PR")]] static PlanType* create(AlicaEngine* ae, const YAML::Node& planTypeNode);
    static PlanType* create(const YAML::Node& config, SubscribeFunction subscribeFunc, const YAML::Node& planTypeNode);
    static void attachReferences();

private:
    PlanTypeFactory() = delete;
};
} // namespace alica
