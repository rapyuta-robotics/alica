/*
 * Task.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef TASK_H_
#define TASK_H_

#include "AlicaElement.h"

#include <string>

namespace alica {

class TaskRepository;
class ModelFactory;
/**
 * an abstract description of parts of plans to be taken on by a set of robots
 */
class Task : public AlicaElement {
public:
    Task(bool defaultTask);
    virtual ~Task();
    const std::string& getDescription() const { return _description; }

    const TaskRepository* getTaskRepository() const;
    std::string toString() const;

    static constexpr int64_t IDLEID = -1;  // For Task Id of an Idle EntryPoint...
private:
    void setTaskRepository(const TaskRepository* taskRepository);
    friend ModelFactory;
    void setDescription(const std::string& description);
    const TaskRepository* _taskRepository;
    std::string _description;
    bool _defaultTask;
};

}  // namespace alica

#endif /* TASK_H_ */
