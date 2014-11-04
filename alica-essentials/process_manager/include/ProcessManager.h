/*
 * ProcessManager.h
 *
 *  Created on: Nov 1, 2014
 *      Author: Stephan Opfer
 */

#ifndef PROCESSMANAGER_H_
#define PROCESSMANAGER_H_

#define PM_DEBUG // for toggling debug output

#include <map>
#include <thread>
#include <dirent.h>
#include "ros/ros.h"

#include "ManagedExecutable.h"



using namespace std;

namespace supplementary
{

	class SystemConfig;

	class ProcessManager
	{
	public:
		ProcessManager(int argc, char** argv);
		virtual ~ProcessManager();
		void start();
		bool isRunning();
	private:
		SystemConfig* sc;
		map<short, ManagedExecutable> executableMap;


		bool running;
		thread* mainThread;
		chrono::microseconds iterationTime;

		void run();
		void searchProcFS();
		void updateMngdExecutables();
	};

} /* namespace alica */

#endif /* PROCESSMANAGER_H_ */
