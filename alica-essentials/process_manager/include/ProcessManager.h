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

#include "process_manager/ProcessCommand.h"

using namespace std;

namespace supplementary
{

	class SystemConfig;
	class ManagedRobot;

	class ProcessManager
	{
	public:
		ProcessManager(int argc, char** argv);
		virtual ~ProcessManager();
		void start();
		bool isRunning();
	private:
		SystemConfig* sc;
		string defaultHostname;
		map<uint8_t, ManagedRobot*> robotMap;

		// this is just for faster procfs checking
		list<string> *managedExecNames;
		map<string ,uint8_t> executableIdMap;
		map<string, uint8_t> robotIdMap;

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
