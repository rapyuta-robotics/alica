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

	ManagedRobot::ManagedRobot()
	{
	}

	ManagedRobot::~ManagedRobot()
	{
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
