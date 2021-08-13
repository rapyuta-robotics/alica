#pragma once

#include "Factory.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
class TaskRepository;
class TaskRepositoryFactory : public Factory
{
public:
    static TaskRepository* create(const YAML::Node& node);

private:
    TaskRepositoryFactory() = delete;
};
} // namespace alica
