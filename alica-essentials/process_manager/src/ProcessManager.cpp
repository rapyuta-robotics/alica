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

namespace supplementary
{

	ProcessManager::ProcessManager(int argc, char** argv) :
			iterationTime(1000000), mainThread(NULL), running(false)
	{

		// initialise the process map with data from the Processes.conf file
		this->sc = SystemConfig::getInstance();
		auto processDescriptions = (*this->sc)["Processes"]->getSections("Processes.ProcessDescriptions", NULL);
		short curId;
		string curExecutable;
		vector<string> curDefaultParams;
		for (auto processSectionName : (*processDescriptions))
		{
			curId = (*this->sc)["Processes"]->get<short>("Processes.ProcessDescriptions", processSectionName.c_str(), "id", NULL);
			curExecutable = (*this->sc)["Processes"]->get<string>("Processes.ProcessDescriptions", processSectionName.c_str(), "executable",
			NULL);
			curDefaultParams = (*this->sc)["Processes"]->getList<string>("Processes.ProcessDescriptions", processSectionName.c_str(), "defaultParams", NULL);
			this->executableMap[curId] = new ManagedExecutable(curId, curExecutable.c_str(), curDefaultParams);
		}
	}

	ProcessManager::~ProcessManager()
	{
		this->running = false;
		if (this->mainThread != nullptr)
		{
			mainThread->join();
			delete mainThread;
		}

		for (auto mngdExec : executableMap)
		{
			delete mngdExec.second;
		}
		executableMap.clear();
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
			this->updateMngdExecutables();

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

	void ProcessManager::updateMngdExecutables()
	{
		for (auto executableMapPair : this->executableMap)
		{
			executableMapPair.second->update();
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

			std::ifstream ifs;
			curFile = "/proc/" + string(dirEntry->d_name) + "/comm";
			ifs.open(curFile, std::ifstream::in);
			char execName[256];
			ifs.getline(execName, 256);

			for (auto executableMapPair : this->executableMap)
			{
				if (strcmp(execName, executableMapPair.second->getExecutable().c_str()) != 0)
				{
					continue;
				}

#ifdef PM_DEBUG
				cout << "PM: " << execName << " found with PID: " << curPID << endl;
#endif
				executableMapPair.second->queue4Update(curPID);

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
