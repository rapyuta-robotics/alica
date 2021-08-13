#pragma once

#include "AlicaElement.h"

#include <string>

namespace alica
{

class TaskRepository;
class ModelFactory;
class TaskFactory;
/**
 * an abstract description of parts of plans to be taken on by a set of robots
 */
class Task : public AlicaElement
{
public:
    Task();
    virtual ~Task();

    const TaskRepository* getTaskRepository() const { return _taskRepository; }

    static constexpr int64_t IDLEID = -1; // For Task Id of an Idle EntryPoint...
    static const std::string IDLENAME;
private:
    friend ModelFactory;
    friend TaskFactory;
    const TaskRepository* _taskRepository;
};

} // namespace alica
