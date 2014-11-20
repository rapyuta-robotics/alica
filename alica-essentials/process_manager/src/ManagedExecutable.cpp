/*
 * ManagedExecutable.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Stephan Opfer
 */

#include "ManagedExecutable.h"

namespace supplementary
{

	ManagedExecutable::ManagedExecutable(short id, const char* executable, vector<string> defaultStrParams) :
			id(id), executable(executable), defaultParams(new char*[defaultStrParams.size()])
	{
		//defaultParams = new char*[defaultStrParams.size()];
		for (int i = 0; i < defaultStrParams.size(); i++) {
			defaultParams[i] = (char*) defaultStrParams.at(i).c_str();
		}
	}

	ManagedExecutable::~ManagedExecutable()
	{
		delete[] defaultParams;
	}

	string ManagedExecutable::getExecutable() const
	{
		return string(executable);
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
			for (auto process : processes)
			{
				delete process.second;
			}
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

			// TODO:
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

	void ManagedExecutable::startProcess(char* const* params)
	{
		pid_t pid = fork();
		if (pid == 0) // child process
		{
			int execReturn = execv (this->executable, params);
			if (execReturn == -1)
			{
				cout << "ME: Failure! execve error code=" << errno << endl;
				//cout << getErrMsg(errno) << endl;
			}
		}
		else if (pid < 0)
		{
#ifdef MGND_EXEC_DEBUG
			cout << "ME: Failed to fork!" << endl;
#endif
		}
	}

	void ManagedExecutable::startProcess()
	{
		this->startProcess(this->defaultParams);
	}

	void ManagedExecutable::stopProcess()
	{
		// TODO
		throw new exception();
	}

} /* namespace supplementary */
