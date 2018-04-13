/*
 * TaskRepository.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/TaskRepository.h"

namespace alica {

TaskRepository::TaskRepository()
        : _defaultTask(0) {}

TaskRepository::~TaskRepository() {}

void TaskRepository::setDefaultTask(int64_t defaultTask) {
    _defaultTask = defaultTask;
}

std::string TaskRepository::getFileName() const {
    if (_fileName.empty()) {
        return getName() + ".rdefset";
    }
    return _fileName;
}

void TaskRepository::setFileName(const std::string& fileName) {
    _fileName = fileName;
}

void TaskRepository::setTasks(const TaskSet& tasks) {
    _tasks = tasks;
}

}  // namespace alica
