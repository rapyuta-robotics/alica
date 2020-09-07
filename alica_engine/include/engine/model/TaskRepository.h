#pragma once

#include "AlicaElement.h"
#include "engine/Types.h"

#include <string>

namespace alica
{
class ModelManager;
class TaskRepositoryFactory;

class TaskRepository : public AlicaElement
{
public:
    TaskRepository();
    virtual ~TaskRepository();
    const TaskGrp& getTasks() const { return _tasks; }
    std::string getFileName() const;

private:
    friend ModelFactory;
    friend ModelManager;
    friend TaskRepositoryFactory;
    void setFileName(const std::string& fileName);
    TaskGrp _tasks;
    std::string _fileName;
};

} // namespace alica
