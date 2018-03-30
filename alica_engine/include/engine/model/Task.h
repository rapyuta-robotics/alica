/*
 * Task.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef TASK_H_
#define TASK_H_

#include <string>
#include <sstream>
#include <iostream>

#include "AlicaElement.h"

using namespace std;
namespace alica {

class TaskRepository;

/**
 * an abstract description of parts of plans to be taken on by a set of robots
 */
class Task : public AlicaElement {
public:
    Task(bool defaultTask);
    virtual ~Task();
    const string& getDescription() const;
    void setDescription(const string& description);
    const TaskRepository* getTaskRepository() const;
    void setTaskRepository(const TaskRepository* taskRepository);
    const static long IDLEID = -1;  // For Task Id of an Idle EntryPoint...

private:
    string description;
    string toString();
    const TaskRepository* taskRepository;

protected:
    bool defaultTask;
};

}  // namespace alica

#endif /* TASK_H_ */
