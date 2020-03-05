#pragma once

#include "Factory.h"

namespace alica {
    class Task;
    class TaskRepository;
    class TaskFactory: public Factory {
    public:
        static Task* create(const YAML::Node& taskNode, TaskRepository* taskRepository);
    };
}
