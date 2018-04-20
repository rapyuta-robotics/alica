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
    Task(int64_t id, bool defaultTask);
    Task(bool defaultTask);
    virtual ~Task();
    const std::string& getDescription() const { return _description; }

    const TaskRepository* getTaskRepository() const {return _taskRepository;}
    std::string toString() const override;

    static constexpr int64_t IDLEID = -1;  // For Task Id of an Idle EntryPoint...
private:
    friend ModelFactory;
    void setTaskRepository(const TaskRepository* taskRepository);
    void setDescription(const std::string& description);
    const TaskRepository* _taskRepository;
    std::string _description;
    bool _defaultTask;
};

}  // namespace alica

#endif /* TASK_H_ */
