#pragma once

#include "AlicaElement.h"
#include "engine/Types.h"

#include <string>

namespace alica
{
class ModelManager;

class TaskRepository : public AlicaElement
{
public:
    TaskRepository();
    void addTask(const Task* t);
    const TaskGrp& getTasks() const { return _tasks; }
    void setFileName(const std::string& fileName);
    std::string getFileName() const;

private:
    TaskGrp _tasks;
    std::string _fileName;
};

} // namespace alica
