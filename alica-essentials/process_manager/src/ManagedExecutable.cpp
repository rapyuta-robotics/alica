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
			id(id), executable(executable), defaultParams(new char*[defaultStrParams.size()]), managedPid(NOTHING_MANAGED), state(UNDEFINED)
	{
		for (int i = 0; i < defaultStrParams.size(); i++)
		{
			defaultParams[i] = (char*)defaultStrParams.at(i).c_str();
		}
	}

	ManagedExecutable::~ManagedExecutable()
	{
		delete[] defaultParams;
		if (this->managedPid != NOTHING_MANAGED)
		{
			/* TODO: more comprehensive process stopping
			 * - try several times
			 * - try harder signals, if necessary
			 * - give better feedback than true and false (maybe console output)
			 */
			kill(this->managedPid, SIGTERM);
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
		if (this->queuedPids4Update.size() == 0 && this->managedPid != NOTHING_MANAGED)
		{
#ifdef MGND_EXEC_DEBUG
			cout << "ME: No " << this->executable << " running!" << endl;
#endif
			this->clear();
		}
		else if (this->queuedPids4Update.size() > 1)
		{
#ifdef MGND_EXEC_DEBUG
			cout << "ME: Queued PIDs:  ";
			for (long curPid : this->queuedPids4Update)
			{
				cout << curPid << ", ";
			}
			cout << endl;
#endif

			for (int i=0; i < this->queuedPids4Update.size(); i++)
			{
				long pid = this->queuedPids4Update.at(i);
				if (this->managedPid == pid)
				{
					// erase own and kill all other processes
					this->queuedPids4Update.erase(begin(this->queuedPids4Update) + i);
					this->killOtherProcesses();
					this->queuedPids4Update.clear();

					// update own
					this->updateStats();
					break;
				}
			}

			if (this->queuedPids4Update.size() != 0) // we did not find our own process
			{
				// adapt to the first and kill the other processes
				this->managedPid = this->queuedPids4Update.at(0);
				this->queuedPids4Update.erase(this->queuedPids4Update.begin());
				this->killOtherProcesses();

				// update own
				this->updateStats(true);
			}
		}
#ifdef MGND_EXEC_DEBUG
		if (this->queuedPids4Update.size() != 0)
		{
			cout << "ME: ERROR Update queue not cleared! " << endl;
		}
#endif
	}

	/**
	 * Kills all processes with SIGKILL, which are still in the queuedPIids4Update list.
	 */
	void ManagedExecutable::killOtherProcesses()
	{
		/* TODO: more comprehensive process stopping
		 * - try several times
		 * - try harder signals, if necessary
		 * - give better feedback than true and false (maybe console output)
		 */
		for (auto& pid : this->queuedPids4Update)
		{
			kill(pid, SIGKILL);
		}
	}

	/**
	 * Updates the stats of the managed process (CPU, Memory, State, etc.)
	 */
	void ManagedExecutable::updateStats(bool readParams)
	{
		// TODO: update resource infos over proc-fs
		string procPidString = "/proc/" + to_string(this->managedPid);
		std::ifstream statFile(procPidString + "/stat", std::ifstream::in);
		string line;
		cout << "ME: Updating " << this->executable << " (" << this->managedPid << ")" << endl;
		while (getline(statFile, line, ' '))
		{
#ifdef MGND_EXEC_DEBUG
			cout << "ME: stat " << line << endl;
#endif
		}

		if (readParams)
		{
			std::ifstream statFile(procPidString + "/cmdline", std::ifstream::in);
			string line;
			stringstream ss;
			while (!statFile.eofbit)
			{
				getline(statFile, line, '\0');
				ss << line << " ";
#ifdef MGND_EXEC_DEBUG
				cout << "ME: cmdline arg " << line << endl;
#endif
			}
			this->params = ss.str();
		}

		cout << "ME: Updated " << this->executable << " (" << this->managedPid << ")" << endl;
	}

	/**
	 * Clears information about the former managed process.
	 */
	void ManagedExecutable::clear()
	{
		this->managedPid = NOTHING_MANAGED;
		this->state = UNDEFINED;
		this->params = "";
	}

	/**
	 * Starts a process with the given parameters.
	 * The started process replaces already running processes.
	 * @param params The parameters given to the started process.
	 */
	void ManagedExecutable::startProcess(char* const * params)
	{
		pid_t pid = fork();
		if (pid == 0) // child process
		{
			int execReturn = execv(this->executable, params);
			if (execReturn == -1)
			{
				cout << "ME: Failure! execve error code=" << errno << endl;
				//cout << getErrMsg(errno) << endl;
			}
		}
		else if (pid > 0) // parent process
		{
			this->managedPid = pid;
		}
#ifdef MGND_EXEC_DEBUG
		else if (pid < 0)
		{
			cout << "ME: Failed to fork!" << endl;
		}
#endif
	}

	/**
	 * Starts a process with the default parameters.
	 * The started process replaces already running processes.
	 */
	void ManagedExecutable::startProcess()
	{
		this->startProcess(this->defaultParams);
	}

	bool ManagedExecutable::stopProcess()
	{
		/* TODO: more comprehensive process stopping
		 * - try several times
		 * - try harder signals, if necessary
		 * - give better feedback than true and false (maybe console output)
		 */
		if (this->managedPid != 0)
		{
			kill(this->managedPid, SIGTERM);
			return true;
		}
		else
		{
			return false;
		}
	}

} /* namespace supplementary */
