/*
 * Task.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Task.h"

namespace alica
{

	Task::Task()
	{
		// TODO Auto-generated constructor stub

	}

	Task::~Task()
	{
		// TODO Auto-generated destructor stub
	}

	const TaskRepository* Task::getTaskRepository() const
	{
		return taskRepository;
	}

	void Task::setTaskRepository( TaskRepository* taskRepository)
	{
		this->taskRepository = taskRepository;
	}
	const string& alica::Task::getDescription() const
	{
		return description;
	}

	void alica::Task::setDescription(const string& description)
	{
		this->description = description;
	}

} /* namespace Alica */

