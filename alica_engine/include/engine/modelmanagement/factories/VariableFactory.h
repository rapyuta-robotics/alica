#pragma once

#include "Factory.h"

namespace alica
{
class Variable;
class VariableFactory : public Factory
{
public:
    static Variable* create(const YAML::Node& variableNode);
    static Variable* create(int64_t id, const std::string& name, const std::string& type);
};
} // namespace alica
