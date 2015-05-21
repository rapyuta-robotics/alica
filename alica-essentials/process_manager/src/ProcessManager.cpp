/*
 * ProcessManager.cpp
 *
 *  Created on: Nov 1, 2014
 *      Author: Stephan Opfer
 */

#include "ProcessManager.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <cstdlib>
#include <signal.h>
#include <sys/wait.h>
#include <iostream>

#include <map>
#include <thread>
#include <dirent.h>
#include <unistd.h>

#include <SystemConfig.h>
#include <Logging.h>
#include <RobotExecutableRegistry.h>
#include "ManagedRobot.h"
#include "ManagedExecutable.h"

namespace supplementary
{

	bool ProcessManager::running = false;
	int ProcessManager::numCPUs = 0;
	/**
	 * Creates an ProcessManager object, which has already parsed the process descriptions of the processes to be managed
	 * and the robots, which are known for the ROBOT environment variable.
	 * @param argc
	 * @param argv
	 */
	ProcessManager::ProcessManager(int argc, char** argv) :
			iterationTime(1000000), mainThread(NULL), spinner(NULL), rosNode(NULL), lastTotalCPUTime(0), currentTotalCPUTime(0)
	{
		this->ownId = -1;
		this->sc = SystemConfig::getInstance();
		this->ownHostname = this->sc->getHostname();
		this->pmRegistry = new RobotExecutableRegistry();

		/* Initialise some data structures for better performance in searchProcFS-Method with
		 * data from Globals.conf and Processes.conf file. */

		// Register robots from Globals.conf
		int curId;
		auto robotNames = (*this->sc)["Globals"]->getSections("Globals.Team", NULL);
		for (auto robotName : (*robotNames))
		{
			curId = this->pmRegistry->addRobot(robotName);
			if (robotName == this->ownHostname)
			{
				this->ownId = curId;
			}
		}

		// Register local hostname, if it does not exist in Globals.conf
		if (this->ownId == -1)
		{
			this->ownId = this->pmRegistry->addRobot(this->ownHostname);
		}

		// This autostart functionality is for easier testing
		bool autostart = false;
		for (int i = 1; i < argc; i++)
		{
			if (strcmp(argv[i], "-autostart") == 0)
			{
				autostart = true;
				break;
			}
		}

		if (autostart)
		{
			// Create ManagedRobot-Object for local system/robot
			this->robotMap.emplace(ownId, new ManagedRobot(this->ownHostname, ownId, this));
		}

		// Register executables from Processes.conf
		auto processDescriptions = (*this->sc)["Processes"]->getSections("Processes.ProcessDescriptions", NULL);
		for (auto processSectionName : (*processDescriptions))
		{
			curId = this->pmRegistry->addExecutable(processSectionName);
			// This autostart functionality is for easier testing. The local robot starts the processes automatically
			if (autostart && curId != -1 && this->pmRegistry->getExecutable(curId)->mode == "autostart")
			{
				this->robotMap.at(ownId)->changeDesiredState(curId, true, this->pmRegistry);
			}
		}

		this->interpreter = (*this->sc)["Processes"]->getList<string>("Processes.Interpreter", NULL);

		cout << "PM: OwnId is " << ownId << endl;
	}

	ProcessManager::~ProcessManager()
	{
		ProcessManager::running = false;
		if (this->mainThread != nullptr)
		{
			mainThread->join();
			delete mainThread;
		}

		for (auto mngdRobot : this->robotMap)
		{
			delete mngdRobot.second;
		}
		this->robotMap.clear();
	}

	/**
	 * The callback of the ROS subscriber - it inits the message processing.
	 * @param pc
	 */
	void ProcessManager::handleProcessCommand(process_manager::ProcessCommandPtr pc)
	{
		// check whether this message is for me, 0 is a wild card for all ProcessManagers
		if (pc->receiverId != this->ownId && pc->receiverId != 0)
		{
			return;
		}

		switch (pc->cmd)
		{
			case process_manager::ProcessCommand::START:
				this->changeDesiredProcessStates(pc, true);
				break;
			case process_manager::ProcessCommand::STOP:
				this->changeDesiredProcessStates(pc, false);
				break;
		}
	}

	/**
	 * This method processes an incoming ROS-Message, received in ::handleProcessCommand.
	 * @param pc
	 * @param shouldRun
	 */
	void ProcessManager::changeDesiredProcessStates(process_manager::ProcessCommandPtr pc, bool shouldRun)
	{
		for (int robotId : pc->robotIds)
		{
			// Check whether the robot with the given id is known
			string robotName;
			if (this->pmRegistry->getRobotName(robotId, robotName))
			{
				// Find the ManagedRobot object
				auto mapIter = this->robotMap.find(robotId);
				ManagedRobot* mngdRobot;
				if (mapIter == this->robotMap.end())
				{
					// Lazy initialisation of the robotMap
					mngdRobot = this->robotMap.emplace(robotId, new ManagedRobot(robotName, robotId, this)).first->second;
				}
				else
				{
					// ManagedRobot already exists
					mngdRobot = mapIter->second;
				}

				for (int i = 0; i < pc->processKeys.size(); i++)
				{

					mngdRobot->changeDesiredState(pc->processKeys[i], pc->paramSets[i], shouldRun, this->pmRegistry);
				}
			}
			else
			{
				cout << "PM: Received command for unknown robot id: " << robotId << endl;
			}
		}
	}

	/**
	 * Initialises the ROS Node of the ProcessManager. This is postponed, as we first need to check, whether there is a roscore running.
	 * See ProcessManager::selfCheck()
	 * @param argc
	 * @param argv
	 */
	void ProcessManager::initCommunication(int argc, char** argv)
	{
		// initialise ROS stuff
		ros::init(argc, argv, "ProcessManager");
		rosNode = new ros::NodeHandle();
		spinner = new ros::AsyncSpinner(4);
		processCommandSub = rosNode->subscribe("/process_manager/ProcessCommand", 10, &ProcessManager::handleProcessCommand, (ProcessManager*)this);
		processStatePub = rosNode->advertise<process_manager::ProcessStats>("/process_manager/ProcessStats", 10);
		spinner->start();
	}

	/**
	 * Starts the worker thread of the ProcessManager, if not already running.
	 */
	void ProcessManager::start()
	{
		if (!ProcessManager::running)
		{
			ProcessManager::running = true;
			this->mainThread = new thread(&ProcessManager::run, this);
		}
	}

	/**
	 * The run-method of the worker thread of the ProcessManager object.
	 */
	void ProcessManager::run()
	{
		while (ProcessManager::running)
		{
			auto start = chrono::system_clock::now();

			this->searchProcFS();
			this->updateTotalCPUTimes();
			this->update((this->currentTotalCPUTime - this->lastTotalCPUTime) / ProcessManager::numCPUs);
			this->report();

			auto timePassed = chrono::system_clock::now() - start;
			chrono::microseconds microsecondsPassed = chrono::duration_cast<chrono::microseconds>(timePassed);

#ifdef PM_DEBUG
			cout << "PM: " << microsecondsPassed.count() << " microseconds passed!" << endl << endl;
#endif
			chrono::microseconds availTime = this->iterationTime - microsecondsPassed;

			if (availTime.count() > 10)
			{
				this_thread::sleep_for(availTime);
			}

		}
	}

	/**
	 * Updates lastTotalCPUTime and currentTotalCPUTime by
	 * summing up the first line of /proc/stat.
	 */
	void ProcessManager::updateTotalCPUTimes()
	{
		this->lastTotalCPUTime = currentTotalCPUTime;

		string statLine;
		std::ifstream statFile("/proc/stat", std::ifstream::in);
		getline(statFile, statLine);
		statFile.close();

		statLine = statLine.substr(4, statLine.length() - 4);
		//cout << "PM: readTotalCPUTime() statline: " << statLine << endl;

		long tmp;
		stringstream statStream(statLine);

		int i = 0;
		this->currentTotalCPUTime = 0;
		while (statStream.good() && i < 10)
		{
			statStream >> tmp;
			this->currentTotalCPUTime += tmp;
		}

		/*cout << "PM: Last total CPU time is " << this->lastTotalCPUTime << endl;
		 cout << "PM: Current total CPU time is " << this->currentTotalCPUTime << endl;*/
	}

	/**
	 * Publishes a ProcessStats-Message.
	 */
	void ProcessManager::report()
	{
		process_manager::ProcessStats psts;
		psts.senderId = this->ownId;
		for (auto const &mngdRobot : this->robotMap)
		{
			cout << "PM: report() We try to add another ProcessStat from " << mngdRobot.second->name << "!" << endl;
			mngdRobot.second->report(psts);
		}
		cout << "PM: report() We have " << psts.processStats.size() << " ProcessStats!" << endl;
		this->processStatePub.publish(psts);
	}

	/**
	 * Calls update on all ManagedRobot instances.
	 */
	void ProcessManager::update(unsigned long long cpuDelta)
	{
		for (auto const &mngdRobot : this->robotMap)
		{
			mngdRobot.second->update(cpuDelta);
		}
	}

	/**
	 * Searches the proc filesystem for instances of processes, the ProcessManager has to manage.
	 */
	void ProcessManager::searchProcFS()
	{
		DIR* proc;
		struct dirent *dirEntry;

		if (!(proc = opendir("/proc")))
		{
			perror("PM: can't open /proc");
			return;
		}

		long curPID = 0;
		char* endPtr;
		string curExecutable;
		while ((dirEntry = readdir(proc)) != NULL)
		{
			/* if endptr is not a null character, the directory is not
			 * entirely numeric, so ignore it */
			curPID = strtol(dirEntry->d_name, &endPtr, 10);
			if (*endPtr != '\0')
			{
				continue;
			}

			// get the executables name
			string execName = getExecNameFromCmdLine(dirEntry->d_name);
			if (execName.length() == 0)
				continue;

			int execId;
			if (this->pmRegistry->getExecutableId(execName, execId))
			{
				// get the robots name from the ROBOT environment variable
				string robotName = this->getRobotEnvironmentVariable(string(dirEntry->d_name));

				int robotId;
				if (!this->pmRegistry->getRobotId(robotName, robotId))
				{
					cout << "PM: Warning! Unknown robot '" << robotName << "' is running '" << execName << "'" << endl;
					robotId = this->pmRegistry->addRobot(robotName);
				}

				auto robotEntry = this->robotMap.find(robotId);
				if (robotEntry != robotMap.end())
				{
					robotEntry->second->queue4update(execId, curPID, this->pmRegistry);
				}
				else
				{
					this->robotMap.emplace(robotId, new ManagedRobot(robotName, robotId, this)).first->second->queue4update(execId, curPID,
																														this->pmRegistry);
				}

#ifdef PM_DEBUG
				cout << "PM: Robot '" << robotName << "' executes '" << execName << "' with PID " << curPID << endl;
#endif

			}
			// else: continue, as this executable is unknown and not to be managed by the process manager
		}

		closedir(proc);
	}

	/**
	 * Extracts the executable name from the cmdline entry of the proc-fs. In case of interpreted
	 * executables, it ignores known interpreters and returns the first argument of the interpreter.
	 * @param pid
	 * @return The executable name as string.
	 */
	string ProcessManager::getExecNameFromCmdLine(char* pid)
	{
		string cmdline;
		std::ifstream cmdlineStream("/proc/" + string(pid) + "/cmdline", std::ifstream::in);
		getline(cmdlineStream, cmdline);
		cmdlineStream.close();

		string execName;
		int nextArgIdx = getArgWithoutPath(cmdline, 0, execName);

		/*
		 * Check whether the found executable name is an interpreter like python, java, or ruby.
		 * If that it the case, the "real" executable name is the next argument in the cmdline.
		 */
		if (isKnownInterpreter(execName))
		{
			getArgWithoutPath(cmdline, nextArgIdx, execName);
		}

		return execName;
	}

	/**
	 * Checks whether the found executable name is an interpreter like python, java, or ruby.
	 * @param execName
	 * @return True, if it is a known interpreter. False, otherwise.
	 */
	bool ProcessManager::isKnownInterpreter(string const & execName)
	{
		return find(this->interpreter.begin(), this->interpreter.end(), execName) != this->interpreter.end();
	}

	/**
	 * Retrieves the command line argument (starting at the given start index), without the preceding path.
	 * @param cmdline - The complete command line arguments in one string.
	 * @param argStartIdx - The index of the start character of the argument.
	 * @param arg - A reference to the string in which the retrieved argument will be stored.
	 * @return The index of the first character of the next argument, if present.
	 */
	size_t ProcessManager::getArgWithoutPath(string cmdline, int argStartIdx, string& arg)
	{
		if (argStartIdx >= arg.length())
		{
			arg = "";
			return string::npos;
		}

		int endPos = cmdline.find('\0', argStartIdx);
		if (endPos == string::npos)
		{
			endPos = cmdline.length();
		}
		int startPos = cmdline.find_last_of('/', endPos);
		if (startPos == string::npos)
		{
			startPos = 0; // no slash found, start at 0
		}
		else
		{
			startPos++; // ignore slash
		}
		arg = cmdline.substr(startPos, endPos - startPos);
		return endPos++;
	}

	/**
	 * Get the robots name from the ROBOT environment variable of the given process.
	 * @param processId
	 * @return The value of the ROBOT environment variable, if present. Local hostname, otherwise.
	 */
	string ProcessManager::getRobotEnvironmentVariable(string processId)
	{
		string curFile = "/proc/" + processId + "/environ";
		std::ifstream ifs(curFile, std::ifstream::in);
		string robotEnvironment;

		while (!getline(ifs, robotEnvironment, '\0').eof())
		{
			if (robotEnvironment.substr(0, 6).compare("ROBOT=") == 0)
			{
				return robotEnvironment.substr(6, 256);
			}
		}

		ifs.close();
		return this->ownHostname;
	}

	/**
	 * Method for checking, whether the ProcessManager's main thread is still running.
	 * @return running
	 */
	bool ProcessManager::isRunning()
	{
		return running;
	}

	/**
	 * Checks whether another instance of the process manager executable is already
	 * running on the system and set the kernelPageSize of ManagedExecutable. Furthermore,
	 * this methods starts a roscore if necessary.
	 * @return False, if something didn't work out. True, otherwise.
	 */
	bool ProcessManager::selfCheck()
	{
		string roscoreExecName = "roscore";
		std::ifstream ifs("/proc/self/stat", std::ifstream::in);
		string pid;
		getline(ifs, pid, '\0');
		ifs.close();
		long ownPID = stol(pid);

		DIR* proc;
		struct dirent *dirEntry;

		if (!(proc = opendir("/proc")))
		{
			perror("PM: can't open /proc");
			return true;
		}

		long curPID = 0;
		char* endPtr;
		string curExecutable;
		bool roscoreRunning = false;
		while ((dirEntry = readdir(proc)) != NULL)
		{
			/* if endptr is not a null character, the directory is not
			 * entirely numeric, so ignore it */
			curPID = strtol(dirEntry->d_name, &endPtr, 10);
			if (*endPtr != '\0')
			{
				continue;
			}

			// get the executables name
			string execName = getExecNameFromCmdLine(dirEntry->d_name);
			if (execName.length() == 0)
				continue;

			// Check for already running process managers
			if (execName.compare("process_manager") == 0 && ownPID != curPID)
			{
				cerr << "PM: My own PID is " << ownPID << endl;
				cerr << "PM: There is already another process_manager running on this system! PID: " << curPID << endl;
				cerr << "PM: Terminating myself..." << endl;
				closedir(proc);
				return false;
			}

			// Check for already running roscore
			if (!roscoreRunning && execName.compare(roscoreExecName) == 0)
			{
				roscoreRunning = true;

				// remember started roscore in process managing data structures
				int roscoreExecId;
				if (this->pmRegistry->getExecutableId(roscoreExecName, roscoreExecId))
				{
					// get the ROBOT environment variable of the robot running the roscore
					string robotName = this->getRobotEnvironmentVariable(string(dirEntry->d_name));

					// get the Robot Id of the robot running the roscore
					int robotId;
					if (!this->pmRegistry->getRobotId(robotName, robotId))
					{
						robotId = this->pmRegistry->addRobot(robotName);
					}

					// create managed robot if necessary and chang desired state of roscore accordingly
					auto mngdRobot = this->robotMap.find(robotId);
					if (mngdRobot != this->robotMap.end())
					{
						mngdRobot->second->changeDesiredState(roscoreExecId, true, this->pmRegistry);
					}
					else
					{

						auto emplaceResult = this->robotMap.emplace(robotId, new ManagedRobot(robotName, robotId, this));
						emplaceResult.first->second->changeDesiredState(roscoreExecId, true, this->pmRegistry);
					}
				}
				else
				{
					cerr << "PM: The ID of the roscore executable is unknown!" << endl;
				}

				cout << "PM: roscore already running! PID: " << curPID << endl;
			}
		}

		if (roscoreRunning == false)
		{
			cerr << "PM: Starting roscore at startup!" << endl;
			pid_t pid = fork();
			if (pid == 0) // child process
			{
				setsid();
				// redirect stdout
				string logFileName = Logging::getLogFilename("roscore");
				FILE* fd = fopen(logFileName.c_str(), "w+");
				dup2(fileno(fd), STDOUT_FILENO);
				fclose(fd);

				// redirect stderr
				logFileName = Logging::getErrLogFilename("roscore");
				fd = fopen(logFileName.c_str(), "w+");
				dup2(fileno(fd), STDERR_FILENO);
				fclose(fd);

				char* roscoreExecNameChar = strdup(roscoreExecName.c_str());
				char * argv[] = {roscoreExecNameChar, NULL};
				int execReturn = execvp(roscoreExecName.c_str(), argv);
				free(roscoreExecNameChar);
				if (execReturn == -1)
				{
					cerr << "ME: Failure for roscore startup! execve error code=" << errno << endl;
					closedir(proc);
					return false;
				}
			}
			else if (pid > 0) // parent process
			{
				// remember started roscore in process managing data structures
				int roscoreExecId;
				if (this->pmRegistry->getExecutableId(roscoreExecName, roscoreExecId))
				{
					int robotId;
					if (this->pmRegistry->getRobotId(this->ownHostname, robotId))
					{
						// create managed robot if necessary and chang desired state of roscore accordingly
						auto mngdRobot = this->robotMap.find(robotId);
						if (mngdRobot != this->robotMap.end())
						{
							mngdRobot->second->changeDesiredState(roscoreExecId, true, this->pmRegistry);
						}
						else
						{
							auto emplaceResult = this->robotMap.emplace(robotId, new ManagedRobot(this->ownHostname, robotId, this));
							emplaceResult.first->second->changeDesiredState(roscoreExecId, true, this->pmRegistry);
						}
					}
					else
					{
						cerr << "PM: The robot " << this->ownHostname << " is unknown!" << endl;
					}
				}
				else
				{
					cerr << "PM: The ID of the roscore executable is unknown!" << endl;
				}
			}
			else if (pid < 0)
			{
				cerr << "ME: Failed to fork for starting roscore!" << endl;
				closedir(proc);
				return false;
			}
		}

		closedir(proc);
		return true;
	}

	/**
	 * This is for handling Strg + C, although no ROS communication was running.
	 * @param sig
	 */
	void ProcessManager::pmSigintHandler(int sig)
	{
		cout << endl << "PM: Caught SIGINT! Terminating ..." << endl;
		running = false;

		// Call the ros signal handler method
		ros::shutdown();
	}

	/**
	 *
	 * @param sig
	 */
	void ProcessManager::pmSigchildHandler(int sig)
	{
		/* Wait for all dead processes.
		 * We use a non-blocking call to be sure this signal handler will not
		 * block if a child was cleaned up in another part of the program. */
		int result = 0;
		do
		{
			result = waitpid(WAIT_ANY, NULL, WNOHANG);
			if (result > 0)
			{
				cout << "PM: Caught SIGCHLD Signal for PID: " << result << endl;
			}
			else if (result < 0)
			{
				/* NOTE: I think, that we always get ECHILD, when a child process dies, because
				 * we had to set ssid of the child processes, in order to let them live if the process
				 * manager dies. Anyway, it is not that dramatic.
				 */
				if (errno != ECHILD)
				{
					cerr << "PM: 'waitpid' returned an error! " << endl;
					perror("waitpid");
				}
			}

		} while (result > 0);
	}

} /* namespace supplementary */

int main(int argc, char** argv)
{
	// Set kernel page size for human readable memory consumption
	supplementary::ManagedExecutable::kernelPageSize = sysconf(_SC_PAGESIZE);
	// Determine number of cores
	while (supplementary::FileSystem::pathExists("/sys/devices/system/cpu/cpu" + to_string(supplementary::ProcessManager::numCPUs)))
	{
		supplementary::ProcessManager::numCPUs++;
	}

	supplementary::ProcessManager* pm = new supplementary::ProcessManager(argc, argv);
	if (pm->selfCheck())
	{
		pm->initCommunication(argc, argv);

		// has to be set after ProcessManager::initCommunication() , in order to override the ROS signal handler
		signal(SIGINT, supplementary::ProcessManager::pmSigintHandler);

		signal(SIGCHLD, supplementary::ProcessManager::pmSigchildHandler);

		pm->start();

		while (pm->isRunning())
		{
			chrono::milliseconds dura(500);
			this_thread::sleep_for(dura);
		}

		delete pm;
	}

	return 0;
}
