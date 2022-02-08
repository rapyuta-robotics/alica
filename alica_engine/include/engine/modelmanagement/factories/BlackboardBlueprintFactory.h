#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class BlackboardBlueprint;
class BlackboardBlueprintFactory : public Factory
{
public:
    static std::unique_ptr<BlackboardBlueprint> create(const YAML::Node& node);
    static std::unique_ptr<BlackboardBlueprint> createEmpty();

private:
    BlackboardBlueprintFactory() = delete;
};
} // namespace alica
