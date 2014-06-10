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


	Task::Task(bool defaultTask)
	{
		this->defaultTask = defaultTask;
	}

	string Task::toString()
	{
		stringstream ss;
		ss << "#Task: " << this->name << " " << this->id << endl;
		ss << "\t Description: " << this->description << endl;
		ss << "#EndTask" << endl;
		return ss.str();
	}

	const string& Task::getDescription() const
	{
		return description;
	}

	void Task::setDescription(const string& description)
	{
		this->description = description;
	}

	const TaskRepository* Task::getTaskRepository() const
	{
		return taskRepository;
	}

	void Task::setTaskRepository(const TaskRepository* taskRepository)
	{
		this->taskRepository = taskRepository;
	}

} /* namespace Alica */


