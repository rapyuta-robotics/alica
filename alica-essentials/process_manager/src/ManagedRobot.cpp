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
		// TODO Auto-generated constructor stub

	}

	ManagedRobot::~ManagedRobot()
	{
		// TODO Auto-generated destructor stub
	}

	void ManagedRobot::queue4update(uint8_t execid, long pid)
	{
		this->executableMap.at(execid)->queue4Update(pid);
	}

	void ManagedRobot::queue4update(string execName, uint8_t execid, long pid)
	{
		ManagedExecutable* mngExec = new ManagedExecutable(execName, execid, pid);
		mngExec->queue4Update(pid);
		this->executableMap.at(execid) = mngExec;
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
