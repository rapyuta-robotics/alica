#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class RoleSet;
class RoleSetFactory : public Factory
{
public:
    static RoleSet* create(const YAML::Node& node);
    static void attachReferences();

private:
    RoleSetFactory() = delete;
};
} // namespace alica
