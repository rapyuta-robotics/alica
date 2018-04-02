/*
 * TaskRepository.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef TASKREPOSITORY_H_
#define TASKREPOSITORY_H_

#include <string>
#include <list>

#include "AlicaElement.h"

using namespace std;
namespace alica {
class Task;

class TaskRepository : public AlicaElement {
public:
    TaskRepository();
    virtual ~TaskRepository();
    long getDefaultTask() const;
    void setDefaultTask(long defaultTask);
    string getFileName();
    void setFileName(string fileName);
    list<Task*>& getTasks();

private:
    void setTasks(const list<Task*>& tasks);

protected:
    string fileName;
    list<Task*> tasks;
    long defaultTask;
};

}  // namespace alica

#endif /* TASKREPOSITORY_H_ */
