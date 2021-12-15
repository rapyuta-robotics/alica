#include "engine/model/Role.h"
#include "engine/AlicaEngine.h"
#include "engine/model/RoleSet.h"
#include "engine/model/Task.h"

#include <exception>
#include <iostream>
#include <sstream>

namespace alica
{

Role::Role()
        : _roleSet(nullptr)
{
}

Role::~Role() {}

double Role::getPriority(int64_t taskId) const
{
    for (auto pair : _taskPriorities) {
        if (pair.first->getId() == taskId) {
            return pair.second;
        }
    }
    return _roleSet->getDefaultPriority();
}

std::string Role::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#Role: " << getName() << " " << getId() << std::endl;
    ss << indent << "\tTaskPriorities Size: " << _taskPriorities.size() << std::endl;
    for (std::unordered_map<Task*, double>::const_iterator iterator = _taskPriorities.begin(); iterator != _taskPriorities.end(); ++iterator) {
        const Task* task = iterator->first;
        const double priority = iterator->second;
        ss << indent << "\t" << task->getName() << " (" << task->getId() << ") : " << priority << std::endl;
    }
    ss << indent << "#EndRole" << std::endl;
    return ss.str();
}

} // namespace alica
