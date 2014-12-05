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
#include <unistd.h>
#include <sys/types.h>
#include <cstdlib>

#include <SystemConfig.h>
#include "ManagedRobot.h"

namespace supplementary
{

	ProcessManager::ProcessManager(int argc, char** argv) :
			iterationTime(1000000), mainThread(NULL), running(false)
	{
		this->sc = SystemConfig::getInstance();
		this->defaultHostname = this->sc->getHostname();
		this->managedExecNames = new list<string>();

		/* Initialise some data structures for faster runtime in searchProcFS-Method with
		 * data from Globals.conf and Processes.conf file. */
		auto processDescriptions = (*this->sc)["Processes"]->getSections("Processes.ProcessDescriptions", NULL);

		short curId;
		for (auto processSectionName : (*processDescriptions))
		{
			this->managedExecNames->push_back(processSectionName);
			curId = (*this->sc)["Processes"]->get<short>("Processes.ProcessDescriptions", processSectionName.c_str(),
															"id", NULL);
			this->executableIdMap.emplace(processSectionName, curId);
		}

		auto robotNames = (*this->sc)["Globals"]->getSections("Globals.Team", NULL);
		for (auto robotName : (*robotNames))
		{
			curId = (*this->sc)["Globals"]->get<short>("Globals.Team", robotName.c_str(), "id", NULL);
			this->robotIdMap.emplace(robotName, curId);
		}

//		short curId;
//		string curExecutable;
//		vector<string> curDefaultParams;
//		for (auto processSectionName : (*processDescriptions))
//		{
//			curId = (*this->sc)["Processes"]->get<short>("Processes.ProcessDescriptions", processSectionName.c_str(), "id", NULL);
//			if (this->executableMap.find(curId) != this->executableMap.end()) {
//				cerr << "ProcessManager: ERROR - ID " << curId << " found more than one time in Processes.conf!" << endl;
//				throw new exception();
//			}
//			curExecutable = (*this->sc)["Processes"]->get<string>("Processes.ProcessDescriptions", processSectionName.c_str(), "executable",
//						NULL);
//			curDefaultParams = (*this->sc)["Processes"]->getList<string>("Processes.ProcessDescriptions", processSectionName.c_str(), "defaultParams",
//			NULL);
//			this->executableMap[curId] = new ManagedExecutable(curId, curExecutable.c_str(), curDefaultParams);
//		}

// initialise ROS stuff
		rosNode = new ros::NodeHandle();
		spinner = new ros::AsyncSpinner(4);
		processCommandSub = rosNode->subscribe("/process_manager/ProcessCommand", 10,
												&ProcessManager::handleProcessCommand, (ProcessManager*)this);
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
	 * The run-method of the worker thread in the ProcessManager.
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
			cout << "PM: " << microsecondsPassed.count() << " microseconds passed!" << endl;
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
			char execName[256];
			ifs.getline(execName, 256);
			ifs.close();

			for (auto curExecName : *(this->managedExecNames))
			{
				if (strcmp(execName, curExecName.c_str()) != 0)
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
				string robotName = robotEnvironment.substr(6, 256);
				auto robotId = this->robotIdMap.find(robotName);
				if (robotId != this->robotIdMap.end())
				{
					auto robotEntry = this->robotMap.find(robotId->second);
					if (robotEntry != robotMap.end())
					{
						robotEntry->second->queue4update(this->executableIdMap[execName], curPID);
					}
					else
					{
						this->robotMap.emplace(robotId->second, new ManagedRobot());
						this->robotMap[robotId->second]->queue4update(execName, this->executableIdMap[execName], curPID);
					}
				}
				else
				{
					cout << "PM: Unknown robot " << robotName << " is running " << execName << endl;
				}

#ifdef PM_DEBUG
				cout << "PM: FOUND RobotEnvironment " << robotEnvironment << " Robot " << robotName << " Exec "
						<< execName << " PID " << curPID << endl;
#endif

			}
		}
	}

	bool ProcessManager::isRunning()
	{
		return running;
	}

} /* namespace supplementary */

int main(int argc, char** argv)
{
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
