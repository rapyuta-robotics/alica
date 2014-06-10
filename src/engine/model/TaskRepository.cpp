/*
 * TaskRepository.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/TaskRepository.h"
#include "engine/model/Task.h"

namespace alica
{

	TaskRepository::TaskRepository()
	{
		// TODO Auto-generated constructor stub

	}

	TaskRepository::~TaskRepository()
	{
		// TODO Auto-generated destructor stub
	}

	long TaskRepository::getDefaultTask() const
	{
		return defaultTask;
	}

	void TaskRepository::setDefaultTask(long defaultTask)
	{
		this->defaultTask = defaultTask;
	}

	const string& alica::TaskRepository::getFilename() const
	{
		return filename;
	}

	void alica::TaskRepository::setFilename(const string& filename)
	{
		this->filename = filename;
	}

	list<Task*>& TaskRepository::getTasks()
	{
		return tasks;
	}

	void TaskRepository::setTasks(const list<Task*>& tasks)
	{
		this->tasks = tasks;
	}

} /* namespace Alica */

