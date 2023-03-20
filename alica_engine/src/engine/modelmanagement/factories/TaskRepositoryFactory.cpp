#include "engine/modelmanagement/factories/TaskRepositoryFactory.h"

#include "engine/model/Task.h"
#include "engine/model/TaskRepository.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/TaskFactory.h"

namespace alica
{
TaskRepository* TaskRepositoryFactory::create(const YAML::Node& node)
{
    auto* taskRepository = new TaskRepository();
    Factory::setAttributes(node, taskRepository);
    Factory::storeElement(taskRepository, alica::Strings::taskrepository);

    if (Factory::isValid(node[alica::Strings::tasks])) {
        const YAML::Node& tasksNode = node[alica::Strings::tasks];
        for (YAML::const_iterator it = tasksNode.begin(); it != tasksNode.end(); ++it) {
            taskRepository->addTask(TaskFactory::create(*it, taskRepository));
        }
    }

    return taskRepository;
}
} // namespace alica
