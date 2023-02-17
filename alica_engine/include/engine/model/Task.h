#pragma once

#include "AlicaElement.h"

#include <string>

namespace alica
{

class TaskRepository;
/**
 * an abstract description of parts of plans to be taken on by a set of robots
 */
class Task : public AlicaElement
{
public:
    Task(TaskRepository* taskRepository);
    virtual ~Task();

    const TaskRepository* getTaskRepository() const { return _taskRepository; }

private:
    const TaskRepository* _taskRepository;
};

} // namespace alica
