#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

#include "engine/blackboard/KeyMapping.h"

namespace alica
{
class KeyMapping;
class KeyMappingFactory : public Factory
{
public:
    static KeyMapping create(const YAML::Node& node);

private:
    KeyMappingFactory() = delete;
};
} // namespace alica
