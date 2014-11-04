/*
 * ManagedExecutable.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Stephan Opfer
 */

#include "ManagedExecutable.h"

namespace supplementary
{

	ManagedExecutable::ManagedExecutable(short id, string executable, string defaultParams)
		: id(id), executable(executable), defaultParams(defaultParams)
	{

	}

	ManagedExecutable::~ManagedExecutable()
	{

	}

	const string& ManagedExecutable::getExecutable() const
	{
		return executable;
	}

	void ManagedExecutable::queue4Update(long pid)
	{
		this->queuedPids4Update.push_back(pid);
	}

	void ManagedExecutable::update()
	{
		// If there is no process left, clean up
		if (this->queuedPids4Update.size() == 0 && this->processes.size() != 0)
		{
#ifdef MGND_EXEC_DEBUG
			cout << "ME: No " << this->executable << " running!" << endl;
#endif
			processes.clear();
		}
		else if (this->queuedPids4Update.size() != 0)
		{
#ifdef MGND_EXEC_DEBUG
			cout << "ME: Queued PIDs:  ";
			for (long curPid : this->queuedPids4Update)
			{
				cout << curPid << ", ";
			}
			cout << endl;
#endif

			// 0. remove not queued processes from processes map

			// 1. find pids from que in processes map
			// 2. Pid found
			// 2.1 remove it from queue
			// 2.2. update pid
			// 3. Pid NOT found
			// 3.1 add new pid to processes
		}

		this->queuedPids4Update.clear();
	}

} /* namespace supplementary */
