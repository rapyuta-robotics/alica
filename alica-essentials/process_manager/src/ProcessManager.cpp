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

#include <map>
#include <thread>
#include <dirent.h>
#include <unistd.h>

#include <SystemConfig.h>
#include "ManagedRobot.h"
#include "ManagedExecutable.h"

namespace supplementary
{

	bool ProcessManager::running = false;

	/**
	 * Creates an ProcessManager object, which has already parsed the process descriptions of the processes to be managed
	 * and the robots, which are known for the ROBOT environment variable.
	 * @param argc
	 * @param argv
	 */
	ProcessManager::ProcessManager(int argc, char** argv) :
			iterationTime(1000000), mainThread(NULL), spinner(NULL), rosNode(NULL)
	{
		this->sc = SystemConfig::getInstance();
		this->defaultHostname = this->sc->getHostname();
		this->executableNames = new list<string>();

		/* Initialise some data structures for faster runtime in searchProcFS-Method with
		 * data from Globals.conf and Processes.conf file. */

		int curId;
		auto robotNames = (*this->sc)["Globals"]->getSections("Globals.Team", NULL);
		for (auto robotName : (*robotNames))
		{
			curId = (*this->sc)["Globals"]->get<int>("Globals.Team", robotName.c_str(), "ID", NULL);
			this->robotIdMap.emplace(robotName, curId);
		}

		bool autostart = (std::find(argv, argv + argc, "-autostart") != argv + argc);

		int ownId = this->robotIdMap.at(this->defaultHostname);
		if (autostart)
		{
			this->robotMap.emplace(this->robotIdMap.at(this->defaultHostname), new ManagedRobot(this->defaultHostname));
		}

		auto processDescriptions = (*this->sc)["Processes"]->getSections("Processes.ProcessDescriptions", NULL);
		for (auto processSectionName : (*processDescriptions))
		{
			this->executableNames->push_back(processSectionName);
			curId = (*this->sc)["Processes"]->get<int>("Processes.ProcessDescriptions", processSectionName.c_str(), "id", NULL);
			this->executableIdMap.emplace(processSectionName, curId);
			if (autostart)
			{
				//ManagedExecutable* curME = new ManagedExecutable(processSectionName, curId, ManagedExecutable::NOTHING_MANAGED);

				this->robotMap.at(ownId)->startExecutable(processSectionName, curId);
			}
		}

	}

	ProcessManager::~ProcessManager()
	{
		ProcessManager::running = false;
		if (this->mainThread != nullptr)
		{
			mainThread->join();
			delete mainThread;
		}

		// TODO: cleanup the double map
		for (auto mngdRobot : this->robotMap)
		{
			delete mngdRobot.second;
		}
		this->robotMap.clear();
	}

	void ProcessManager::handleProcessCommand(process_manager::ProcessCommandPtr pc)
	{
		// check wether this msg is for me, 0 is a wildcard for all ProcessManagers
		if (pc->receiverId != this->robotIdMap.at(this->defaultHostname) && pc->receiverId != 0)
		{
			return;
		}

		switch (pc->cmd)
		{
			case process_manager::ProcessCommand::START:
				for (int robotId : pc->robotIds)
				{
					for (int proc : pc->processKeys)
					{
						this->robotMap.at(robotId)->changeDesiredState(proc, true);
					}
				}
				break;
			case process_manager::ProcessCommand::STOP:
				for (int robotId : pc->robotIds)
				{
					for (int proc : pc->processKeys)
					{
						this->robotMap.at(robotId)->changeDesiredState(proc, false);
					}
				}
				break;
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
			this->update();

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
	 * Calls update on all ManagedRobot instances.
	 */
	void ProcessManager::update()
	{
		for (auto const &mngdRobot : this->robotMap)
		{
			mngdRobot.second->update();
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
		string curFile;
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
			std::ifstream ifs;
			curFile = "/proc/" + string(dirEntry->d_name) + "/comm";
			ifs.open(curFile, std::ifstream::in);
			string execName;
			getline(ifs, execName);
			ifs.close();

			for (auto curExecName : *(this->executableNames))
			{
				if (execName.compare(curExecName) != 0)
				{
					// we don't manage this executable -> continue
					continue;
				}

				// get the ROBOT environment variable
				curFile = "/proc/" + string(dirEntry->d_name) + "/environ";
				ifs.open(curFile, std::ifstream::in);
				string robotEnvironment;
				getline(ifs, robotEnvironment, '\0');
				ifs.close();

				// get the robots ID
				string robotName;
				if (robotEnvironment.substr(0, 6).compare("ROBOT=") != 0)
				{
					robotName = this->defaultHostname;
				}
				else
				{
					robotName = robotEnvironment.substr(6, 256);
				}
				auto robotId = this->robotIdMap.find(robotName);
				if (robotId != this->robotIdMap.end())
				{
					int execid = this->executableIdMap[execName];

					auto robotEntry = this->robotMap.find(robotId->second);
					if (robotEntry != robotMap.end())
					{
						robotEntry->second->queue4update(execName, execid, curPID);
					}
					else
					{
						this->robotMap.emplace(robotId->second, new ManagedRobot(robotName));
						this->robotMap[robotId->second]->queue4update(execName, execid, curPID);
					}
				}
				else
				{
					cout << "PM: Unknown robot " << robotName << " is running " << execName << endl;
				}

#ifdef PM_DEBUG
				cout << "PM: Robot '" << robotName << "' executes '" << execName << "' with PID " << curPID << endl;
#endif

			}
		}

		closedir(proc);
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
		std::ifstream ifs;
		ifs.open("/proc/self/stat", std::ifstream::in);
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
		string curFile;
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
			std::ifstream ifs;
			curFile = "/proc/" + string(dirEntry->d_name) + "/comm";
			ifs.open(curFile, std::ifstream::in);
			string execName;
			getline(ifs, execName);
			ifs.close();

			if (execName.compare("process_manager") == 0 && ownPID != curPID)
			{
				cout << "PM: My own PID is " << ownPID << endl;
				cout << "PM: There is already another process_manager running on this system! PID: " << curPID << endl;
				cout << "PM: Terminating myself..." << endl;
				closedir(proc);
				return false;
			}

			if (execName.compare(roscoreExecName) == 0)
			{
				roscoreRunning = true;
				cout << "PM: roscore already running! PID: " << curPID << endl;
			}
		}

		if (roscoreRunning == false)
		{
			cout << "PM: Starting roscore" << endl;
			pid_t pid = fork();
			if (pid == 0) // child process
			{
				char* roscoreExecNameChar = strdup(roscoreExecName.c_str());
				char * argv[] = {roscoreExecNameChar, NULL};
				int execReturn = execvp(roscoreExecName.c_str(), argv);
				free(roscoreExecNameChar);
				if (execReturn == -1)
				{
					cout << "ME: Failure for roscore startup! execve error code=" << errno << endl;
					closedir(proc);
					return false;
				}
			}
			else if (pid > 0) // parent process
			{
				// remember started roscore in process managing data structures
				int roscoreExecId = this->executableIdMap.at(roscoreExecName);
				int robotId = this->robotIdMap.at(this->defaultHostname);
				auto emplaceResult = this->robotMap.emplace(robotId, new ManagedRobot(this->defaultHostname));
				emplaceResult.first->second->queue4update(roscoreExecName, roscoreExecId, pid);
			}
			else if (pid < 0)
			{
				cout << "ME: Failed to fork for starting roscore!" << endl;
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

} /* namespace supplementary */

int main(int argc, char** argv)
{
	// Set kernel page size for human readable memory consumption
	supplementary::ManagedExecutable::kernelPageSize = sysconf(_SC_PAGESIZE);

	supplementary::ProcessManager* pm = new supplementary::ProcessManager(argc, argv);
	if (pm->selfCheck())
	{
		pm->initCommunication(argc, argv);

		// has to be set after ProcessManager::initCommunication() , in order to override the ROS signal handler
		signal(SIGINT, supplementary::ProcessManager::pmSigintHandler);

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
