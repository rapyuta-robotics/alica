#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class BlackboardBlueprint;
class BlackboardBlueprintFactory : public Factory
{
public:
    static const BlackboardBlueprint* create(const YAML::Node& node);
    static const BlackboardBlueprint* createEmpty();

private:
    BlackboardBlueprintFactory() = delete;
};
} // namespace alica
