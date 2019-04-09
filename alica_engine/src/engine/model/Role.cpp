#include "engine/model/Role.h"

#include <exception>
#include <iostream>
#include <sstream>

namespace alica
{

Role::Role()
{
}

Role::~Role() {}

double Role::getPriority(int64_t taskId) const
{
    std::unordered_map<int64_t, double>::const_iterator it = _taskPriorities.find(taskId);
    if (it != _taskPriorities.end()) {
        return it->second;
    } else { // TODO move this check to start up
        std::cerr << "ROLE DOES NOT HAVE A PRIORITY FOR TASK: " << taskId << std::endl;
        throw new std::exception;
    }
}

std::string Role::toString() const
{
    std::stringstream ss;
    ss << "#Role: " << getName() << " " << getId() << std::endl;
    ss << "\tRTM TaskPriorities Size: " << _taskPriorities.size() << std::endl;
    for (std::unordered_map<int64_t, double>::const_iterator iterator = _taskPriorities.begin();
            iterator != _taskPriorities.end(); ++iterator) {
        const int64_t l = iterator->first;
        const double val = iterator->second;
        ss << "\t" << l << " : " << val << std::endl;
    }
    ss << std::endl;
    ss << "#EndRole" << std::endl;
    return ss.str();
}

//====================== Getter and Setter ==================

void Role::setTaskPriorities(const std::unordered_map<int64_t, double>& taskPriorities)
{
    _taskPriorities = taskPriorities;
}
} // namespace alica
