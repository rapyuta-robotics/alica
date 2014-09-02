/*
 * Task.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef TASK_H_
#define TASK_H_

using namespace std;

#include <string>
#include <sstream>
#include <iostream>

#include "AlicaElement.h"

namespace alica
{

	class TaskRepository;

	class Task : public AlicaElement
	{
	public:
		Task(bool defaultTask);
		virtual ~Task();
		const string& getDescription() const;
		void setDescription(const string& description);
		const TaskRepository* getTaskRepository() const;
		void setTaskRepository(const TaskRepository* taskRepository);
		const static long IDLEID = -1;

	private:
		string description;
		string toString();
		const TaskRepository* taskRepository;

	protected:
		bool defaultTask;
	};

} /* namespace Alica */

#endif /* TASK_H_ */
