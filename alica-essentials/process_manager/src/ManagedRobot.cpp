/*
 * ManagedRobot.cpp
 *
 *  Created on: Nov 28, 2014
 *      Author: Stephan Opfer
 */

#include "ManagedRobot.h"
#include "ManagedExecutable.h"

namespace supplementary
{

	ManagedRobot::ManagedRobot(string robotName) :
			robotName(robotName)
	{
	}

	ManagedRobot::~ManagedRobot()
	{
		for (auto mngdExec : this->executableMap)
		{
			delete mngdExec.second;
		}
		this->executableMap.clear();
	}

	void ManagedRobot::changeDesiredState(int execid, bool shouldRun)
	{
		auto execEntry = this->executableMap.find(execid);
		if (execEntry == this->executableMap.end())
		{
			execEntry->second->changeDesiredState(shouldRun);
		}
	}

	void ManagedRobot::startExecutable(string execName, int execid)
	{
		auto execEntry = this->executableMap.find(execid);
		if (execEntry == this->executableMap.end())
		{
			auto newExecEntry = this->executableMap.emplace(execid, new ManagedExecutable(execName, execid, ManagedExecutable::NOTHING_MANAGED));
		}
		execEntry->second->startProcess();
	}

	void ManagedRobot::startExecutable(string execName, int execid, char** params)
	{
		auto execEntry = this->executableMap.find(execid);
		if (execEntry == this->executableMap.end())
		{
			auto newExecEntry = this->executableMap.emplace(execid, new ManagedExecutable(execName, execid, ManagedExecutable::NOTHING_MANAGED));
		}
		execEntry->second->startProcess(params);
	}

	void ManagedRobot::queue4update(string execName, int execid, long pid)
	{
		auto execEntry = this->executableMap.find(execid);
		if (execEntry != this->executableMap.end())
		{
			execEntry->second->queue4Update(pid);
		}
		else
		{
			auto newExecEntry = this->executableMap.emplace(execid, new ManagedExecutable(execName, execid, pid));
			newExecEntry.first->second->queue4Update(pid);
		}
	}

	void ManagedRobot::update()
	{
		for (auto const & managedExec : this->executableMap)
		{
			managedExec.second->update();
		}
	}

}
/* namespace supplementary */
