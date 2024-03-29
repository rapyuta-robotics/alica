#include "engine/modelmanagement/factories/TaskFactory.h"
#include "engine/model/Task.h"
#include "engine/modelmanagement/Strings.h"

namespace alica
{
Task* TaskFactory::create(const YAML::Node& taskNode, TaskRepository* taskRepository)
{
    Task* task = new Task(taskRepository);
    Factory::setAttributes(taskNode, task);
    Factory::storeElement(task, alica::Strings::task);
    return task;
}
} // namespace alica
