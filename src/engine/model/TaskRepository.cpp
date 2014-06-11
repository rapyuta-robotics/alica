/*
 * TaskRepository.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/TaskRepository.h"

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

	const string& TaskRepository::getFileName() const
	{
		if (this->getFileName().empty())
		{
			static string result = name + ".rdefset";
			return result;
		}
		return fileName;
	}

	void TaskRepository::setFileName(const string& fileName)
	{
		this->fileName = fileName;
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
