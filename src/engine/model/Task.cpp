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
		ss << "\t Description: " << this->desciption << endl;
		ss << "#EndTask" << endl;
		return ss.str();
	}

	const string& Task::getDesciption() const
	{
		return desciption;
	}

	void Task::setDesciption(const string& desciption)
	{
		this->desciption = desciption;
	}

	const TaskRepository* Task::getTaskrepository() const
	{
		return taskrepository;
	}

	void Task::setTaskrepository(const TaskRepository* taskrepository)
	{
		this->taskrepository = taskrepository;
	}

} /* namespace Alica */


