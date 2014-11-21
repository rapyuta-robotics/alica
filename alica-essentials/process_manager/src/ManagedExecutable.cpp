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
		for (int i = 0; i < defaultStrParams.size(); i++) {
			defaultParams[i] = (char*) defaultStrParams.at(i).c_str();
		}
	}

	ManagedExecutable::~ManagedExecutable()
	{
		delete[] defaultParams;
		/* TODO: kill all processes, the nice way ;-)
		 * - wait and hard kill if possible
		 */
		for (auto proc : processes) {
			kill(proc.first, SIGTERM);
		}
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

			/* Remove processes from processes map, which are not present in procfs,
			 * i.e., are not queued4update.
			 */
			for (auto procIter = processes.begin(); procIter != processes.end(); ++procIter)
			{
				bool found = false;
				for (int i=0; i < queuedPids4Update.size(); i++)
				{
					if (queuedPids4Update[i] == procIter->first) {
						found = true;
					}
				}

				if (!found)
				{
					processes.erase(procIter);
				}
			}


			for (int i = 0; i < this->queuedPids4Update.size(); i++)
			{

				long pid = this->queuedPids4Update[i];
				auto process = this->processes.find(pid);
				if (process != this->processes.end())
				{// pid found
					cout << "ME: Process found: " << pid << endl;
					// Remove the pid from queue
					this->queuedPids4Update.erase(this->queuedPids4Update.begin() + i);

					// Update the corresponding process
					process->second->update();
				}
				else
				{// pid not found
					cout << "ME: Process not found: " << pid << endl;
					// Add new pid to processes

					//TODO: make some output in case the boolean in the pair, says false
					auto returnPair = this->processes.emplace(pid, new Process (pid));
					returnPair.first->second->update();
				}
			}
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

	bool ManagedExecutable::stopProcess(string robot)
	{
		/* TODO: more comprehensive process stopping
		 * - try several times
		 * - try harder signals, if necessary
		 * - give better feedback than true and false (maybe console output)
		 */
		for (auto process : processes)
		{
			if (process.second->getRobot() == robot)
			{
				kill(process.first, SIGTERM);
				return true;
			}
		}
		return false;
	}

} /* namespace supplementary */
