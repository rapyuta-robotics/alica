#include "engine/model/TaskRepository.h"

namespace alica
{

TaskRepository::TaskRepository() {}

std::string TaskRepository::getFileName() const
{
    return _fileName;
}

void TaskRepository::setFileName(const std::string& fileName)
{
    _fileName = fileName;
}

void TaskRepository::addTask(const Task* t)
{
    _tasks.push_back(t);
}

} // namespace alica
