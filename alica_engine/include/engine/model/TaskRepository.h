/*
 * TaskRepository.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef TASKREPOSITORY_H_
#define TASKREPOSITORY_H_

#include <string>

#include "AlicaElement.h"
#include <engine/Types.h>

namespace alica
{

class TaskRepository : public AlicaElement
{
  public:
    TaskRepository();
    virtual ~TaskRepository();
    int64_t getDefaultTask() const { return _defaultTask; }
    const TaskGrp& getTasks() const { return _tasks; }
    std::string getFileName() const;

  private:
    friend ModelFactory;
    void setTasks(const TaskGrp& tasks);
    void setDefaultTask(int64_t defaultTask);
    void setFileName(const std::string& fileName);
    TaskGrp _tasks;
    int64_t _defaultTask;
    std::string _fileName;
};

} // namespace alica

#endif /* TASKREPOSITORY_H_ */
