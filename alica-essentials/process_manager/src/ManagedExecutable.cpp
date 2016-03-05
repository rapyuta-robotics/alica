/*
 * ManagedExecutable.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Stephan Opfer
 */

#include "process_manager/ManagedExecutable.h"
#include "process_manager/ProcessManager.h"
#include <process_manager/RobotExecutableRegistry.h>
#include <cassert>
#include <map>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <string>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <stdlib.h>
#include <SystemConfig.h>
#include <Logging.h>

namespace supplementary
{
	long ManagedExecutable::kernelPageSize = 0;

	ManagedExecutable::ManagedExecutable(ExecutableMetaData const * const metaExec, long pid, string robotName,
											ProcessManager* procMan) :
			metaExec(metaExec), managedPid(pid), state('X'), lastUTime(0), lastSTime(0), currentUTime(0), currentSTime(
					0), memory(0), starttime(0), desiredRunState(RunState::SHOULDNT_RUN), cpu(0), runningParamSet(
					ExecutableMetaData::UNKNOWN_PARAMS), desiredParamSet(ExecutableMetaData::UNKNOWN_PARAMS), procMan(
					procMan), need2ReadParams(true), publishing(false), shouldPublish(false)
	{

#ifdef MNGD_EXEC_DEBUG
		cout << "ME: Constructor of executable " << metaExec->name << endl;
#endif

		this->robotEnvVariable = robotName;
	}

	ManagedExecutable::~ManagedExecutable()
	{
		// TODO: check whether the cleanup is right -> valgrind
		for (auto param : this->runningParams)
		{
			free((char*)param);
		}
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
			if (this->desiredRunState == RunState::SHOULD_RUN)
			{
				if (chrono::steady_clock::now() - this->lastTimeTried > std::chrono::milliseconds(1000)) // TODO: make the wait time a parameter
				{
#ifdef MNGD_EXEC_DEBUG
					cout << "ME: Starting " << this->metaExec->name << "!" << endl;
#endif
					if (this->desiredParamSet != ExecutableMetaData::UNKNOWN_PARAMS)
					{ // this guarantees, that manually started executables are not started again (except we did know the manually used parameters)
						this->startProcess();
					}
				}
#ifdef MNGD_EXEC_DEBUG
				else
				{
					cout << "ME: Waiting for " << this->metaExec->name << " to start!" << endl;
				}
#endif
			}
			else
			{
#ifdef MNGD_EXEC_DEBUG
				cout << "ME: No " << this->metaExec->name << " running and none should run!" << endl;
#endif
				this->desiredRunState == RunState::SHOULDNT_RUN; // reset from manual started
				this->clear();
			}
		}
		else if (this->queuedPids4Update.size() > 0) // there are processes which run
		{
#ifdef MNGD_EXEC_DEBUG
			cout << "ME: The queued PIDs for " << this->metaExec->name << " are: ";
			for (long curPid : this->queuedPids4Update)
			{
				cout << curPid << ", ";
			}
			cout << endl;
#endif

			if (this->desiredRunState == RunState::SHOULD_RUN || this->desiredRunState == RunState::MANUAL_STARTED)
			{ // some process should run

				if (this->managedPid != ExecutableMetaData::NOTHING_MANAGED) // we know a PID, which we manage
				{
					for (int i = 0; i < this->queuedPids4Update.size(); i++)
					{
						long pid = this->queuedPids4Update.at(i);
						if (this->managedPid == pid)
						{
							/* We found our own process in the list.
							 * -> That means it is still running.
							 * -> So let us update its statistics.
							 */
							this->updateStats(cpuDelta, false);

							if (!this->shouldPublish && this->publishing)
							{ // stop the log publishing threads if necessary
								publishing = false;
							}

							if (this->desiredParamSet == this->runningParamSet
									|| this->desiredRunState == RunState::MANUAL_STARTED)
							{ // our process is running with the right parameters, or is started manually
							  // -> so kill all others

								if (this->shouldPublish && !this->publishing)
								{ // start the log publishing threads if necessary
									startPublishingLogs();
								}

								// erase own from list and kill all remaining processes
								this->queuedPids4Update.erase(this->queuedPids4Update.begin() + i);
								this->killQueuedProcesses();
								break;
							}
							else
							{ // our process is not running with the right parameters, or is not manually started
							  // -> so kill it and all others
#ifdef MNGD_EXEC_DEBUG
								cout << "ME: " << this->metaExec->name << " is NOT running the right params!" << endl;
#endif
								this->clear();
								this->killQueuedProcesses();
								this->publishing = false;
								break;
							}
						}
					}

				}

				if (this->queuedPids4Update.size() != 0)
				{ // We did not find our own process, or we don't have one to manage, so ...

					// ... adapt to the first and kill the other processes
					this->managedPid = this->queuedPids4Update.at(0);
#ifdef MNGD_EXEC_DEBUG
					cout << "ME: We adapt " << this->managedPid << " for " << this->metaExec->name << endl;
#endif
					this->queuedPids4Update.erase(this->queuedPids4Update.begin());
					this->killQueuedProcesses();
					this->updateStats(cpuDelta, true);
				}

			}
			else if (this->desiredRunState == RunState::SHOULDNT_RUN) // there shouldn't run a process
			{
				if (this->managedPid != ExecutableMetaData::NOTHING_MANAGED) // we know a PID, which we managed
				{
#ifdef MNGD_EXEC_DEBUG
					cout << "ME: Kill all " << this->metaExec->name << endl;
#endif
					// kill all processes, as we need to kill them including our own
					this->clear();
					this->publishing = false;
					this->killQueuedProcesses();
				}
				else
				{
					// adapt to the first and kill the other processes

					// "Manual Started" is necessary, in order to avoid
					// restarts after somebody killed his manually started process.
					this->desiredRunState = RunState::MANUAL_STARTED;
					this->managedPid = this->queuedPids4Update.at(0);
					this->queuedPids4Update.erase(this->queuedPids4Update.begin());
					this->killQueuedProcesses();
#ifdef MNGD_EXEC_DEBUG
					cout << "ME: We adapt to the manually started PID " << this->managedPid << " for "
							<< this->metaExec->name << endl;
#endif
					// update own
					this->updateStats(cpuDelta, true);
				}
			}

			assert(this->queuedPids4Update.size() == 0);
		}
	}

	void ManagedExecutable::report(process_manager::ProcessStats& psts, int robotId)
	{
		if (this->managedPid != ExecutableMetaData::NOTHING_MANAGED)
		{
			process_manager::ProcessStat ps;
			ps.robotId = robotId;
			ps.cpu = this->cpu;
			ps.mem = this->memory * ManagedExecutable::kernelPageSize / 1024.0 / 1024.0; // MB
			ps.processKey = this->metaExec->id;
			ps.paramSet = this->runningParamSet;
			ps.state = this->state;
			ps.publishing = (this->publishing ? 1 : 0);
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
	void ManagedExecutable::updateStats(unsigned long long cpuDelta, bool isNew)
	{

#ifdef MNGD_EXEC_DEBUG
		cout << "ME: Updating " << this->metaExec->name << " (" << this->managedPid << ")" << endl;
#endif

		string pidString = to_string(this->managedPid);
		std::ifstream statFile("/proc/" + pidString + "/stat", std::ifstream::in);
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

		if (this->need2ReadParams)
		{
			this->readProcParams(pidString);
		}

		if (!isNew)
		{
			double sCPU = 100.0 * double(currentSTime - lastSTime) / double(cpuDelta);
			double uCPU = 100.0 * double(currentUTime - lastUTime) / double(cpuDelta);
			this->cpu = sCPU + uCPU;
		}
		else
		{
			this->cpu = 0;
		}

#ifdef MNGD_EXEC_DEBUG
		this->printStats();
		cout << "ME: Updated " << this->metaExec->name << " (" << this->managedPid << ")" << endl;
#endif
	}

	/**
	 * Reads the command line parameters of a given process.
	 * @param procPidString The process ID as string
	 */
	void ManagedExecutable::readProcParams(string pidString)
	{
		this->need2ReadParams = false;
		this->runningParams.clear();

		string cmdline = this->procMan->getCmdLine(pidString.c_str());
		vector<string> splittedCmdLine = this->procMan->splitCmdLine(cmdline);
		int checkIdx = 0;

		if (this->procMan->isKnownInterpreter(splittedCmdLine[checkIdx]))
		{
			// ignore interpreter and get next part (without path)
			checkIdx++;
		}

		if (this->metaExec->prefixCmd != "NOT-FOUND")
		{ // Ignore things like 'roslaunch', as this was checked in searchProcFS.
			checkIdx++;
			if (this->metaExec->rosPackage != "NOT-FOUND")
			{ // Ignore the ROS package, as this was checked in search ProcFS, too.
				checkIdx++;
			}
		}

		if (splittedCmdLine.size() < checkIdx)
		{
			this->runningParamSet = ExecutableMetaData::UNKNOWN_PARAMS;
			return;
		}

		// extract the executable name without path, in order to make the check path independent later on
		size_t lastSlashIdx = splittedCmdLine[checkIdx].find_last_of('/');
		string execNameWithoutPath = splittedCmdLine[checkIdx].substr(lastSlashIdx + 1,
																		splittedCmdLine[checkIdx].length());

		// add executable name without path
		this->runningParams.push_back(strdup(execNameWithoutPath.c_str()));
		checkIdx++;

		// add params as is...
		for (; checkIdx < splittedCmdLine.size(); checkIdx++)
		{
			this->runningParams.push_back(strdup(splittedCmdLine[checkIdx].c_str()));
		}
		this->runningParams.push_back(nullptr);

#ifdef MNGD_EXEC_DEBUG
		cout << "ME: PROC-FS Command-Line Parameters of " << this->metaExec->name << ": " << this->runningParams.size()
				<< endl;
		for (auto param : this->runningParams)
		{
			if (param != nullptr)
			{ // ignore nullptr, which is always the last argument in command line
				cout << "'" << param << "'" << endl;
			}
			else
			{
				cout << "'nullptr'" << endl;
			}
		}
#endif

		for (auto paramEntry : this->metaExec->parameterMap)
		{
#ifdef MNGD_EXEC_DEBUG
			cout << "ME: Parameter Set " << paramEntry.first << ": " << paramEntry.second.size() << endl;
			for (auto param : paramEntry.second)
			{
				if (param != nullptr)
				{ // ignore nullptr, which is always the last argument in command line
					cout << "'" << param << "'" << endl;
				}
				else
				{
					cout << "'nullptr'" << endl;
				}
			}
#endif

			if (paramEntry.second.size() != this->runningParams.size())
			{
#ifdef MNGD_EXEC_DEBUG
				cout << "ME: Number of parameters does not match! " << endl;
#endif
				continue;
			}

			int i = 0;

			if (strstr(paramEntry.second[i], this->runningParams[i]) != nullptr)
			{ // this compares the name of the executable (path-independent)
				i++;
				for (; i < this->runningParams.size(); i++)
				{
					if (paramEntry.second[i] == nullptr && this->runningParams[i] == nullptr)
					{ // this case is for the ending null pointer in the command line parameters
						continue;
					}
					else if (strcmp(paramEntry.second[i], this->runningParams[i]) != 0)
					{ // we have miss matching parameters
						break;
					}
				}
			}

			if (i == this->runningParams.size())
			{ // all parameters matched
#ifdef MNGD_EXEC_DEBUG
				cout << "ME: Parameters matched for paramSetid " << paramEntry.first << endl;
#endif
				this->runningParamSet = paramEntry.first;
				if (this->desiredParamSet == ExecutableMetaData::UNKNOWN_PARAMS)
				{ // we did not receive a command with a specific param set, so adapt the running one
					this->desiredParamSet = this->runningParamSet;
				}
				return;
			}
		}
#ifdef MNGD_EXEC_DEBUG
		cout << "ME: Parameters are unknown for " << this->metaExec->name << endl;
#endif
		this->runningParamSet == ExecutableMetaData::UNKNOWN_PARAMS;
		return;
	}

	void ManagedExecutable::printStats()
	{
		stringstream ss;
		ss << "ME: Stats of " << this->metaExec->name << " (" << this->managedPid << ")" << endl;
		ss << "ME: State: '" << this->state << "' StartTime: " << this->starttime << endl;
		ss << "ME: Times: last u '" << this->lastUTime << "' last s '" << this->lastSTime << "' current u '"
				<< this->currentUTime << "' current s '" << this->currentSTime << endl;
		ss << "ME: CPU: " << this->cpu << "%" << endl;
		ss << "ME: Memory: " << this->memory * kernelPageSize / 1024.0 / 1024.0 << "MB" << endl;
		ss << "ME: Parameters " << this->runningParams.size() << ": ";
		for (auto param : this->runningParams)
		{
			if (param != nullptr)
			{ // ignore nullptr, which is always the last argument in command line
				ss << "'" << param << "'";
			}
		}
		ss << endl;
		ss << "ME: Running Parameter Set: " << this->runningParamSet << endl;
		ss << "ME: Desired Parameter Set: " << this->desiredParamSet << endl;
		cout << ss.str();
	}

	/**
	 * Clears runtime information (not the control values, like "desiredRunState") about the former managed process.
	 */
	void ManagedExecutable::clear()
	{
#ifdef MNGD_EXEC_DEBUG
		cout << "ME: CLEARED " << this->metaExec->name << endl;
#endif
		this->managedPid = ExecutableMetaData::NOTHING_MANAGED;
		this->runningParamSet = ExecutableMetaData::UNKNOWN_PARAMS;
		this->runningParams.clear();
		this->need2ReadParams = true;
		this->state = 'X';
		this->currentSTime = 0;
		this->currentUTime = 0;
		this->lastSTime = 0;
		this->lastUTime = 0;

	}

	void ManagedExecutable::changeDesiredLogPublishingState(bool shouldPublish)
	{
		cout << "ME: Changed shouldPublish to " << shouldPublish << endl;
		if (this->shouldPublish != shouldPublish)
		{ // change if necessary
			this->shouldPublish = shouldPublish;
		}
	}

	void ManagedExecutable::changeDesiredState(bool shouldRun, int paramSetId)
	{
		cout << "ME: changeDesiredState for " << this->metaExec->name << endl;

		if (this->managedPid == ExecutableMetaData::NOTHING_MANAGED && !shouldRun)
		{
			cout << "ME: Can not stop executable, as nothing is running!" << endl;
		}

		if (this->managedPid != ExecutableMetaData::NOTHING_MANAGED && this->runningParamSet == paramSetId && shouldRun)
		{
			cout << "ME: Won't (re)start executable, as it is already running with the right parameters! PID: "
					<< this->managedPid << endl;
		}

		// remember desired executable state
		if (shouldRun)
		{
			this->desiredRunState = RunState::SHOULD_RUN;
		}
		else
		{
			this->desiredRunState = RunState::SHOULDNT_RUN;
		}
		this->desiredParamSet = paramSetId;
	}

	/**
	 * Joins with old publishing threads and creates new threads for log publishing.
	 * But it does that only, if there is a managed PID known to
	 * this executable and the publishing flag is true.
	 */
	void ManagedExecutable::startPublishingLogs()
	{
		if (this->managedPid != ExecutableMetaData::NOTHING_MANAGED && this->shouldPublish)
		{ // join with old threads and create new threads for log publishing
			this->publishing = true;
			if (stdLogPublisher.joinable())
			{
				cout << "ME: Try to join stdLogPublisher" << endl;
				stdLogPublisher.join();
			}
			stdLogPublisher = thread(&ManagedExecutable::publishLogFile, this, stdLogFileName,
										ros::console::levels::Info);
			if (errLogPublisher.joinable())
			{
				cout << "ME: Try to join errLogPublisher" << endl;
				errLogPublisher.join();
			}
			errLogPublisher = thread(&ManagedExecutable::publishLogFile, this, errLogFileName,
										ros::console::levels::Error);
		}
	}

	/**
	 * Starts a process with the given parameters.
	 * The started process replaces already running processes.
	 * @param params The parameters given to the started process.
	 */
	void ManagedExecutable::startProcess(vector<char*> & params)
	{
		this->stdLogFileName = Logging::getLogFilename(this->metaExec->name);
		this->errLogFileName = Logging::getErrLogFilename(this->metaExec->name);

		const char* execStartString;
		vector<char*> startParams;
		if (this->metaExec->prefixCmd != "NOT-FOUND")
		{
			execStartString = this->metaExec->prefixCmd.c_str();
			// add prefixCmd to startParams
			char* tmp = new char[this->metaExec->prefixCmd.size() + 1];
			strcpy(tmp, this->metaExec->prefixCmd.c_str());
			tmp[this->metaExec->prefixCmd.size() + 1] = '\0';
			startParams.push_back(tmp);
			if (this->metaExec->rosPackage != "NOT-FOUND")
			{
				// add rospackage to startParams
				tmp = new char[this->metaExec->rosPackage.size() + 1];
				strcpy(tmp, this->metaExec->rosPackage.c_str());
				tmp[this->metaExec->rosPackage.size() + 1] = '\0';
				startParams.push_back(tmp);
			}
		}
		else if (this->metaExec->absExecName.size() > 1)
		{
			execStartString = this->metaExec->absExecName.c_str();
		}
		else
		{
			execStartString = this->metaExec->execName.c_str();
		}

		for (int i = 0; i < params.size()-1; i++)
		{ // add params to startParams
			startParams.push_back(strdup(params[i]));
		}
		startParams.push_back(nullptr);

		pid_t pid = fork();
		if (pid == 0) // child process
		{
			setsid(); // necessary to let the child process live longer than its parent

			setenv("ROBOT", this->robotEnvVariable.c_str(), 1);

			// redirect stdout

			FILE* fd = fopen(stdLogFileName.c_str(), "w+");
			dup2(fileno(fd), STDOUT_FILENO);
			fclose(fd);

			// redirect stderr

			fd = fopen(errLogFileName.c_str(), "w+");
			dup2(fileno(fd), STDERR_FILENO);
			fclose(fd);

			int execReturn;

#ifdef MNGD_EXEC_DEBUG
			cout << "ME: Starting '" << execStartString << "' ! Params: '" << startParams.data() << "'" << endl;
			for (auto param : startParams)
			{
				cout << "'" << param << "'" << endl;
			}
#endif
			execReturn = execvp(execStartString, startParams.data());
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
			for (auto param : startParams)
			{
				free(param);
			}
			if (this->shouldPublish)
			{ // if we got the command to start publishing, then do it right from the start
				this->startPublishingLogs();
			}
		}
		else if (pid < 0)
		{
			cout << "ME: Failed to fork, plz consider a spoon!" << endl;
			this->managedPid = ExecutableMetaData::NOTHING_MANAGED;
		}
	}

	/**
	 * This method is the run method for the log publishing
	 * threads created in the startPublishingLogs() method.
	 * The threads terminate if the publishing flag is set to false.
	 */
	void ManagedExecutable::publishLogFile(string logFileName, ros::console::levels::Level logLevel)
	{
		std::ifstream ifs;
		int startCounter = 0;
		while (!FileSystem::isFile(logFileName))
		{
			usleep(1000);
			startCounter++;
			if (startCounter > 2)
			{
				cout
						<< "ME: Could not attach to log file, maybe the managed process wasn't started by the process manager!"
						<< endl;
				return;
			}
		}
		ifs.open(logFileName.c_str());

		if (ifs.is_open())
		{
			ifs.seekg(0, ios_base::end);
			std::string line;
			long int bufSize = 2048;
			int idx = 0;
			int idxLast = 0;
			char buf[bufSize];

			//ros::Publisher pub = ros::NodeHandle
			while (this->publishing)
			{
				idxLast = idx;
				int charsRead = ifs.readsome(buf + idx, bufSize - idx);
				if (charsRead > 0)
				{
					int charsToBeConsumed = charsRead;
					while (char* newLine = (char*)memchr(buf + idx, '\n', charsToBeConsumed))
					{
						idx = newLine - buf + 1;
						charsToBeConsumed = idxLast + charsRead - idx;
					}

					if (charsToBeConsumed != charsRead)
					{ // we found at least one new line character in the loop above, so send it

						string msg = string(buf, idx - 1);
						ROS_LOG_STREAM(logLevel, ROSCONSOLE_DEFAULT_NAME, msg);

						// copy the rest chars to the front of the buffer
						strncpy(buf, buf + idx, charsToBeConsumed);
						idx = charsToBeConsumed;
					}
					else if (idx + charsRead == bufSize)
					{
						string msg = string(buf, bufSize);
						ROS_LOG_STREAM(logLevel, ROSCONSOLE_DEFAULT_NAME, msg);

						idx = 0;
					}
					else
					{ // we did not find a new line character in the loop above, so look for for further chars
						idx += charsRead;
					}
				}
				else
				{
					usleep(40000);
				}
			}
		}
		else
		{
			cerr << "ME: Could not open log file! " << endl;
		}
	}

	/**
	 * Starts a process with the default parameters.
	 * The started process replaces already running processes.
	 */
	void ManagedExecutable::startProcess()
	{
#ifdef MNGD_EXEC_DEBUG
		cout << "ME: Desired Parameter Set ID: " << this->desiredParamSet << endl;
#endif
		auto entry = this->metaExec->parameterMap.at(this->desiredParamSet);
		this->startProcess(entry);
	}

} /* namespace supplementary */
