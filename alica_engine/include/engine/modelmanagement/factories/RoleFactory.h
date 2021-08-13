#pragma once

#include "Factory.h"

namespace alica
{
class Role;
class RoleSet;
class RoleFactory : public Factory
{
public:
    static Role* create(const YAML::Node& roleNode, RoleSet* roleSet);
    static void attachReferences();
};
} // namespace alica
