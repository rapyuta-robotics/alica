#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class BlackboardBlueprint;
class BlackboardFactory : public Factory
{
public:
    static BlackboardBlueprint create(const YAML::Node& node);
    static BlackboardBlueprint createEmpty();

private:
    BlackboardFactory() = delete;
};
} // namespace alica
