/*
 * ProcessManager.cpp
 *
 *  Created on: Nov 1, 2014
 *      Author: Stephan Opfer
 */

#include "ProcessManager.h"
#include <SystemConfig.h>
#include "Process.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>
#include <cstdlib>

namespace supplementary
{

	ProcessManager::ProcessManager(int argc, char** argv) :
			iterationTime(1000000), mainThread(NULL), running(false)
	{

		// initialise the process map with data from the Processes.conf file
		this->sc = SystemConfig::getInstance();
		auto processDescriptions = (*this->sc)["Processes"]->getSections("Processes.ProcessDescriptions", NULL);
		short curId;
		string curRosPkg, curRosExecutable, curParams;
		for (auto processSectionName : (*processDescriptions))
		{
			curId = (*this->sc)["Processes"]->get<short>("Processes.ProcessDescriptions", processSectionName.c_str(),
															"id", NULL);
			curRosExecutable = (*this->sc)["Processes"]->get<string>("Processes.ProcessDescriptions",
																		processSectionName.c_str(), "executable",
																		NULL);
			curParams = (*this->sc)["Processes"]->get<string>("Processes.ProcessDescriptions",
																processSectionName.c_str(), "params", NULL);
			this->processMap.insert(
					pair<short, Process*>(curId, new Process(curId, curRosPkg, curRosExecutable, curParams)));
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

		for (auto processMapPair : this->processMap)
		{
			delete processMapPair.second;
		}
		processMap.clear();
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

			// TODO: update Process data structures
			this->collectProcFS();

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

	int ProcessManager::filterProcesses(const struct dirent *entry)
	{
//		for (auto processPair : this->processMap)
//		{
//			if (strcmp(processPair.second->getExecutable().c_str(), entry->d_name))
//			{
//				return true;
//			}
//		}
		return false;
	}

	void ProcessManager::collectProcFS()
	{

		struct dirent **namelist;
		int i, n;
		n = scandir("/proc", &namelist, ProcessManager::filterProcesses, NULL);

		if (n < 0)
		{
			perror("ProcessManager::collectProcFS");
			free(namelist);
			return;
		}

		for (i = 0; i < n; i++)
		{
			//cout << "ff: Namelist " << i << ": " << namelist[i]->d_name << endl;
			string curFile = namelist[i]->d_name;
			string curFullFile = "/proc/" + curFile;
			cout << curFullFile << endl;
			if (FileSystem::isDirectory(curFullFile))
			{
				// ignore current or parent directory
				if (FileSystem::CURDIR.compare(curFile) == 0 || FileSystem::PARENTDIR.compare(curFile) == 0)
				{
					free(namelist[i]);
					continue;
				}

				/* if endptr is not a null character, the directory is not
				 * entirely numeric, so ignore it */
				char* endptr;
				long curPid = strtol(namelist[i]->d_name, &endptr, 10);
				if (*endptr != '\0')
				{
					free(namelist[i]);
//					i++;
//					break;
					continue;
				}

				std::ifstream ifs;

				ifs.open(curFullFile + "/comm", std::ifstream::in);
				char name[256];
				ifs.getline(name, 256);

				//cout << "PM: " << name << endl;

			}

			free(namelist[i]);
		}

//		free(namelist[0]);
//		free(namelist[1]);
		for (; i < n; i++)
		{
			free(namelist[i]);
		}

		free(namelist);
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
