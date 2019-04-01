#include "engine/model/Task.h"
#include <sstream>

namespace alica
{

Task::Task()
        : _taskRepository(nullptr)
{
}

Task::~Task() {}

std::string Task::toString() const
{
    std::stringstream ss;
    ss << "#Task: " << getName() << " " << getId() << std::endl;
    ss << "#EndTask" << std::endl;
    return ss.str();
}

} // namespace alica
