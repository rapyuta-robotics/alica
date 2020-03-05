#include "engine/model/TaskRepository.h"

namespace alica
{

TaskRepository::TaskRepository() {}

TaskRepository::~TaskRepository() {}

std::string TaskRepository::getFileName() const
{
    return _fileName;
}

void TaskRepository::setFileName(const std::string& fileName)
{
    _fileName = fileName;
}

} // namespace alica
