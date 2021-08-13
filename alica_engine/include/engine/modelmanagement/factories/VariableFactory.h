#pragma once

#include "Factory.h"

namespace alica
{
class Variable;
class VariableFactory : public Factory
{
public:
    static Variable* create(const YAML::Node& variableNode);
};
} // namespace alica
