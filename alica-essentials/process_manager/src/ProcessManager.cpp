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

#include <map>
#include <thread>
#include <dirent.h>
#include <unistd.h>

#include <SystemConfig.h>
#include "ManagedRobot.h"
#include "ManagedExecutable.h"

namespace supplementary
{

	/**
	 * Creates an ProcessManager object, which has already parsed the process descriptions of the processes to be managed
	 * and the robots, which are known for the ROBOT environment variable.
	 * @param argc
	 * @param argv
	 */
	ProcessManager::ProcessManager(int argc, char** argv) :
			iterationTime(1000000), mainThread(NULL), running(false)
	{

		this->sc = SystemConfig::getInstance();
		this->defaultHostname = this->sc->getHostname();
		this->executableNames = new list<string>();

		/* Initialise some data structures for faster runtime in searchProcFS-Method with
		 * data from Globals.conf and Processes.conf file. */
		auto processDescriptions = (*this->sc)["Processes"]->getSections("Processes.ProcessDescriptions", NULL);

		int curId;
		for (auto processSectionName : (*processDescriptions))
		{
			this->executableNames->push_back(processSectionName);
			curId = (*this->sc)["Processes"]->get<int>("Processes.ProcessDescriptions", processSectionName.c_str(), "id", NULL);
			this->executableIdMap.emplace(processSectionName, curId);
		}

		auto robotNames = (*this->sc)["Globals"]->getSections("Globals.Team", NULL);
		for (auto robotName : (*robotNames))
		{
			curId = (*this->sc)["Globals"]->get<int>("Globals.Team", robotName.c_str(), "ID", NULL);
			this->robotIdMap.emplace(robotName, curId);
		}

		// initialise ROS stuff
		rosNode = new ros::NodeHandle();
		spinner = new ros::AsyncSpinner(4);
		processCommandSub = rosNode->subscribe("/process_manager/ProcessCommand", 10, &ProcessManager::handleProcessCommand, (ProcessManager*)this);
	}

	ProcessManager::~ProcessManager()
	{
		this->running = false;
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
		//TODO: handle commands the right way
		// 1. create map entries of not present
		// 2. trigger right commands

//		switch (pc->cmd)
//		{
//			case process_manager::ProcessCommand::START:
//				for (uint8_t proc : pc->processKeys)
//				{
//					auto mngExec = this->executableMap.find(proc);
//					if (mngExec != this->executableMap.end())
//					{
//						mngExec->second->startProcess();
//					}
//				}
//				break;
//			case process_manager::ProcessCommand::STOP:
//				for (uint8_t proc : pc->processKeys)
//				{
//					auto mngExec = this->executableMap.find(proc);
//					if (mngExec != this->executableMap.end())
//					{
//						// TODO: find robotname with given robotID
//						//mngExec->second->stopProcess(pc->robotId);
//					}
//				}
//				break;
//		}
	}

	void ProcessManager::start()
	{
		if (!this->running)
		{
			this->running = true;
			this->mainThread = new thread(&ProcessManager::run, this);
		}
	}

	/**
	 * The run-method of the worker thread of the ProcessManager object.
	 */
	void ProcessManager::run()
	{
		while (running)
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

	void ProcessManager::update()
	{
		for (auto const &mngdRobot : this->robotMap)
		{
			mngdRobot.second->update();
		}
	}

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
				if (robotEnvironment.substr(0,6).compare("ROBOT=") != 0) {
					robotName = this->defaultHostname;
				} else {
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
						this->robotMap.emplace(robotId->second, new ManagedRobot());
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
	 * @return
	 */
	bool ProcessManager::isRunning()
	{
		return running;
	}

	/**
	 * Checks whether another instance of the process manager executable is already
	 * running on the system and set the kernelPageSize of ManagedExecutable.
	 * @return True, if there is another instance running. False, otherwise.
	 */
	bool ProcessManager::selfCheck()
	{
		ManagedExecutable::kernelPageSize = sysconf(_SC_PAGESIZE);

		std::ifstream ifs;
		ifs.open("/proc/self/stat", std::ifstream::in);
		string pid;
		getline(ifs, pid, '\0');
		ifs.close();
		long ownPID =  stol(pid);



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

			if (execName.compare("process_manager") != 0 || ownPID == curPID)
			{
				continue;
			}
			else
			{
				cout << "PM: My own PID is " << ownPID << endl;
				cout << "PM: There is already another process_manager running on this system! PID: " << curPID << endl;
				cout << "PM: Terminating myself..." << endl;
				return true;
			}
		}

		// No process_manager instance found!
		return false;
	}

} /* namespace supplementary */

int main(int argc, char** argv)
{

	if (supplementary::ProcessManager::selfCheck())
	{
		// A process manager is already running on the system.
		return -1;
	}

	ros::init(argc, argv, "ProcessManager");

	supplementary::ProcessManager* pm = new supplementary::ProcessManager(argc, argv);

	pm->start();

	while (ros::ok() && pm->isRunning())
	{
		chrono::milliseconds dura(500);
		this_thread::sleep_for(dura);
	}

	delete pm;

	return 0;
}
