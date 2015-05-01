/*
 * ManagedExecutable.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Stephan Opfer
 */

#include "ManagedExecutable.h"
#include <map>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <string.h>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <stdlib.h>
#include "SystemConfig.h"
#include "Logging.h"

namespace supplementary
{
	long ManagedExecutable::kernelPageSize = 0;

	ManagedExecutable::ManagedExecutable(string executable, int id, long pid, string mode, vector<char*> defaultParams, string absExecName, string robotName) :
			ExecutableMetaData(executable, id, mode, defaultParams, absExecName), managedPid(pid), state('X'), lastUTime(0), lastSTime(0), currentUTime(0), currentSTime(0), memory(0), starttime(0), shouldRun(false), cpu(0)
	{

#ifdef MGND_EXEC_DEBUG
		cout << "ME: Constructor of executable " << executable << endl;
#endif

		this->robotEnvVariable = robotName;
		this->desiredParams = nullptr;
		this->params = nullptr;
	}

	ManagedExecutable::~ManagedExecutable()
	{
		// TODO: check whether the cleanup is right -> valgrind

		free(desiredParams);
		free(params);

		/*if (this->managedPid != NOTHING_MANAGED)
		{
			kill(this->managedPid, SIGTERM);
		}*/
	}

	void ManagedExecutable::queue4Update(long pid)
	{
		this->queuedPids4Update.push_back(pid);
	}

	/**
	 * This method processes the queued PIDs and kills, starts, and updates the processes accordingly.
	 */
	void ManagedExecutable::update(unsigned long long cpuDelta)
	{
		if (this->queuedPids4Update.size() == 0)
		{
			if (this->shouldRun)
			{
				if (chrono::steady_clock::now() - this->lastTimeTried > std::chrono::milliseconds(1000)) // TODO make the wait time a parameter
				{
#ifdef MGND_EXEC_DEBUG
					cout << "ME: Starting " << this->name << "!" << endl;
#endif
					this->startProcess();
				}
#ifdef MGND_EXEC_DEBUG
				else
				{
					cout << "ME: Waiting for " << this->name << " to start!" << endl;
				}
#endif
			}
			else
			{
#ifdef MGND_EXEC_DEBUG
				cout << "ME: No " << this->name << " running and none should run!" << endl;
#endif
				this->clear();
			}
		}
		else if (this->queuedPids4Update.size() > 0) // there are process which run
		{
#ifdef MGND_EXEC_DEBUG
			cout << "ME: The queued PIDs for " << this->name << " are: ";
			for (long curPid : this->queuedPids4Update)
			{
				cout << curPid << ", ";
			}
			cout << endl;
#endif

			if (shouldRun) // some process should run
			{
				if (this->managedPid != ManagedExecutable::NOTHING_MANAGED) // we know a PID to which should run
				{
					for (int i = 0; i < this->queuedPids4Update.size(); i++)
					{
						long pid = this->queuedPids4Update.at(i);
						if (this->managedPid == pid)
						{
							// erase own and kill all other processes
							this->queuedPids4Update.erase(this->queuedPids4Update.begin() + i);
							this->killQueuedProcesses();

							// update own
							this->updateStats(cpuDelta, false, true);
							break;
						}
					}
				}

				// We did not find our own process, or we don't have one to manage, so ...
				if (this->queuedPids4Update.size() != 0)
				{
					// ... adapt to the first and kill the other processes
					this->managedPid = this->queuedPids4Update.at(0);
					this->queuedPids4Update.erase(this->queuedPids4Update.begin());
					this->killQueuedProcesses();

					// update own
					this->updateStats(cpuDelta, true, true);
				}

			}
			else // there shouldn't run a process
			{
				if (this->managedPid != ManagedExecutable::NOTHING_MANAGED) // we know a process we managed
				{
					// kill all processes, as we need to kill them including our own
					this->killQueuedProcesses();
				}
				else
				{
					// adapt to the first and kill the other processes
					this->shouldRun = true;
					this->managedPid = this->queuedPids4Update.at(0);
					this->queuedPids4Update.erase(this->queuedPids4Update.begin());
					this->killQueuedProcesses();

					// update own
					this->updateStats(cpuDelta, true, true);
				}
			}
#ifdef MGND_EXEC_DEBUG
			if (this->queuedPids4Update.size() != 0)
			{
				cout << "ME: ERROR Update queue not cleared! " << endl;
			}
#endif
		}
	}

	void ManagedExecutable::report(process_manager::ProcessStats& psts, int robotId)
	{
		if (this->managedPid != ManagedExecutable::NOTHING_MANAGED)
		{
			process_manager::ProcessStat ps;
			ps.robotId = robotId;
			ps.cpu = this->cpu;
			ps.mem = this->memory * ManagedExecutable::kernelPageSize / 1024.0 / 1024.0; // MB
			ps.processKey = this->id;
			ps.state = this->state;
			psts.processStats.push_back(ps);
		}

	}

	/**
	 * Kills all processes with SIGKILL, which are still in the queuedPIids4Update list.
	 */
	void ManagedExecutable::killQueuedProcesses()
	{
		/* TODO: more comprehensive process stopping
		 * - try several times
		 * - try harder signals, if necessary
		 * - give better feedback than true and false (maybe console output)
		 */
		for (auto& pid : this->queuedPids4Update)
		{
			cout << "ME: Try to kill " << pid << endl;
			kill(pid, SIGTERM);
		}

		this->queuedPids4Update.clear();
	}

	/**
	 * Updates the stats of the managed process (CPU, Memory, State, etc.)
	 */
	void ManagedExecutable::updateStats(unsigned long long cpuDelta, bool isNew, bool readParams)
	{

#ifdef MGND_EXEC_DEBUG
		cout << "ME: Updating " << this->name << " (" << this->managedPid << ")" << endl;
#endif

		string procPidString = "/proc/" + to_string(this->managedPid);
		std::ifstream statFile(procPidString + "/stat", std::ifstream::in);
		string statLine;
		getline(statFile, statLine);

		//cout << "statline " << statLine << endl;
		string tmp;
		stringstream statStream(statLine);
		int i = 0;
		while (statStream.good() && i < 52)
		{
			if (i == 2)
			{ //state
				statStream >> this->state;
				i++;
			}
			else if (i == 13)
			{ // cpu time stuff (includes index 13-16)
				this->lastUTime = this->currentUTime;
				this->lastSTime = this->currentSTime;
				statStream >> this->currentUTime;
				statStream >> this->currentSTime;
				i += 2;
			}
			else if (i == 21)
			{
				statStream >> this->starttime;
				i++;
			}
			else if (i == 23)
			{
				statStream >> this->memory;
				break;
			}
			else
			{
				statStream >> tmp;
				i++;
				continue;
			}
		}

		if (readParams)
		{
			this->readProcParams(procPidString);
		}

		if (!isNew)
		{
#ifdef MGND_EXEC_DEBUG
			cout << "ME: CPU-Update of '" << this->name << "' ";
#endif
			double sCPU = 100.0 * double(currentSTime - lastSTime) / double(cpuDelta);
			double uCPU = 100.0 * double(currentUTime - lastUTime) / double(cpuDelta);
#ifdef MGND_EXEC_DEBUG
			cout << sCPU << ", " << uCPU << ", " << cpuDelta << endl;
#endif

			this->cpu = sCPU + uCPU;
		}
		else
		{
			this->cpu = 0;
		}

#ifdef MGND_EXEC_DEBUG
		this->printStats();
		cout << "ME: Updated " << this->name << " (" << this->managedPid << ")" << endl;
#endif

	}

	/**
	 * Reads the command line parameters of a given process.
	 * @param procPidString The process ID as string
	 */
	void ManagedExecutable::readProcParams(string procPidString)
	{
		std::ifstream cmdlineFile(procPidString + "/cmdline", std::ifstream::in);
		string line;
		getline(cmdlineFile, line);

		// determine the number of parameters and their length
		vector<char const *> argv;
		int startPos = 0;
		int endPos = 0;
		while (true)
		{
			endPos = line.find('\0', startPos);
			if (endPos != string::npos)
			{
				//cout << "ME: Found param '" << line.substr(startPos, endPos - startPos).c_str() << "' at (" << startPos << ", " << endPos << ")" << endl;
				argv.push_back(line.substr(startPos, endPos - startPos).c_str());
				startPos = endPos + 1;
			}
			else
			{
				break;
			}
		}

	}

	void ManagedExecutable::printStats()
	{
		stringstream ss;
		ss << "ME: Stats of " << this->name << " (" << this->managedPid << ")" << endl;
		ss << "ME: State: '" << this->state << "' StartTime: " << this->starttime << endl;
		ss << "ME: Times: last u '" << this->lastUTime << "' last s '" << this->lastSTime << "' current u '" << this->currentUTime << "' current s '" << this->currentSTime << endl;
		ss << "ME: CPU %: " << this->cpu << endl;
		ss << "ME: Memory: " << this->memory * kernelPageSize / 1024.0 / 1024.0 << "MB" << endl;
		ss << "ME: Parameters: '" << this->params << endl;
		cout << ss.str();
	}

	/**
	 * Clears information about the former managed process.
	 */
	void ManagedExecutable::clear()
	{
		this->managedPid = NOTHING_MANAGED;
		this->state = 'X';
		this->currentSTime = 0;
		this->currentUTime = 0;
		this->lastSTime = 0;
		this->lastUTime = 0;
		free(params);
		this->params = nullptr;
	}

	void ManagedExecutable::changeDesiredState(bool shouldRun)
	{
		if (this->managedPid != NOTHING_MANAGED && shouldRun)
		{
			cout << "ME: Wont start executable, as it is already running! PID: " << this->managedPid << endl;
			return;
		}

		if (this->managedPid == NOTHING_MANAGED && !shouldRun)
		{
			cout << "ME: Can not stop executable, as nothing is running!" << endl;
			return;
		}

		// remember desired executable state
		this->shouldRun = shouldRun;
	}

	/**
	 * Starts a process with the given parameters.
	 * The started process replaces already running processes.
	 * @param params The parameters given to the started process.
	 */
	void ManagedExecutable::startProcess(vector<char*>& params)
	{
		cout << "ME: AbsExecName: " << this->absExecName << endl;
		pid_t pid = fork();
		if (pid == 0) // child process
		{
			setsid(); // necessary to let the child process live longer than its parent

			setenv("ROBOT", this->robotEnvVariable.c_str(), 1);

			// redirect stdout
			string logFileName = Logging::getLogFilename(this->name);
			FILE* fd = fopen(logFileName.c_str(), "w+");
			dup2(fileno(fd), STDOUT_FILENO);
			fclose(fd);

			// redirect stderr
			logFileName = Logging::getErrLogFilename(this->name);
			fd = fopen(logFileName.c_str(), "w+");
			dup2(fileno(fd), STDERR_FILENO);
			fclose(fd);

			int execReturn;
			if (this->absExecName.size() > 1)
			{
				cout << "ME: Starting '"<< this->absExecName << "' ! Params: '" <<  params.data() << "'" << endl;
				execReturn = execvp(this->absExecName.c_str(), params.data());
			}
			else
			{
				cout << "ME: Starting '"<< this->name << "' ! Params: '" <<  params.data() << "'" << endl;
				execReturn = execvp(this->name.c_str(), params.data());
			}
			if (execReturn == -1)
			{
				cout << "ME: Failure! execve error code = " << errno << " - " << strerror(errno) << endl;
				abort();
			}
		}
		else if (pid > 0) // parent process
		{
			this->lastTimeTried = chrono::steady_clock::now();
			this->managedPid = pid;
		}
		else if (pid < 0)
		{
			cout << "ME: Failed to fork, plz consider a spoon!" << endl;
			this->managedPid = NOTHING_MANAGED;
		}
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
