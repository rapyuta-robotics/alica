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
#include "process_manager/ProcessCommand.h"
#include <chrono>

using namespace std;

namespace std{
	class thread;
}

namespace supplementary
{

	class SystemConfig;
	class ManagedRobot;
	class ManagedExecutable;

	class ProcessManager
	{
	public:
		ProcessManager(int argc, char** argv);
		virtual ~ProcessManager();
		void start();
		bool isRunning();
		static bool selfCheck();

	private:
		SystemConfig* sc;
		string defaultHostname;
		map<int, ManagedRobot*> robotMap;

		// this is just for faster procfs checking
		list<string> *executableNames;
		map<string ,int> executableIdMap;
		map<string, int> robotIdMap;

		ros::NodeHandle* rosNode;
		ros::AsyncSpinner* spinner;
		ros::Subscriber processCommandSub;


		void handleProcessCommand(process_manager::ProcessCommandPtr pc);


		bool running;
		thread* mainThread;
		chrono::microseconds iterationTime;

		void run();
		void searchProcFS();
		void update();
	};

} /* namespace alica */

#endif /* PROCESSMANAGER_H_ */
