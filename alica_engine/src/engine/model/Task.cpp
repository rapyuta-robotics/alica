#include "engine/model/Task.h"
#include <sstream>

namespace alica
{

Task::Task(TaskRepository* taskRepository)
        : _taskRepository(taskRepository)
{
}

Task::~Task() {}

} // namespace alica
