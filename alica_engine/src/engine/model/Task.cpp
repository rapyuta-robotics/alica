/*
 * Task.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Task.h"
#include <sstream>

namespace alica {

Task::~Task() {}

Task::Task(bool defaultTask)
        : _defaultTask(defaultTask)
        , _taskRepository(nullptr) {}

std::string Task::toString() const {
    std::stringstream ss;
    ss << "#Task: " << getName() << " " << getId() << std::endl;
    ss << "\t Description: " << _description << std::endl;
    ss << "#EndTask" << std::endl;
    return ss.str();
}

void Task::setDescription(const std::string& description) {
    _description = description;
}

void Task::setTaskRepository(const TaskRepository* taskRepository) {
    _taskRepository = taskRepository;
}

}  // namespace alica
