/*
 * TaskRepository.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/TaskRepository.h"

namespace alica {

TaskRepository::TaskRepository() {
    this->defaultTask = 0;
}

TaskRepository::~TaskRepository() {}

long TaskRepository::getDefaultTask() const {
    return defaultTask;
}

void TaskRepository::setDefaultTask(long defaultTask) {
    this->defaultTask = defaultTask;
}

string TaskRepository::getFileName() {
    if (this->fileName.empty()) {
        static string result = name + ".rdefset";
        return result;
    }
    return fileName;
}

void TaskRepository::setFileName(string fileName) {
    this->fileName = fileName;
}

list<Task*>& TaskRepository::getTasks() {
    return tasks;
}

void TaskRepository::setTasks(const list<Task*>& tasks) {
    this->tasks = tasks;
}

}  // namespace alica
