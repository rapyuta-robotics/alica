#include "engine/model/Task.h"
#include <sstream>

namespace alica
{

const std::string Task::IDLENAME = "IDLE-TASK";

Task::Task()
        : _taskRepository(nullptr)
{
}

Task::~Task() {}

} // namespace alica
