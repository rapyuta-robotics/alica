/*
 * ProcessManager.h
 *
 *  Created on: Nov 1, 2014
 *      Author: Stephan Opfer
 */

#ifndef PROCESSMANAGER_H_
#define PROCESSMANAGER_H_

#define PM_DEBUG // for toggling debug output

#include "ros/ros.h"

#include <map>
#include <thread>

using namespace std;

namespace supplementary
{

	class SystemConfig;
	class Process;

	class ProcessManager
	{
	public:
		ProcessManager(int argc, char** argv);
		virtual ~ProcessManager();
		void start();
		bool isRunning();
	private:
		SystemConfig* sc;
		std::map<short, Process*> processMap;

		bool running;
		thread* mainThread;
		chrono::microseconds iterationTime;

		void run();
		void collectProcFS();
	};

} /* namespace alica */

#endif /* PROCESSMANAGER_H_ */
