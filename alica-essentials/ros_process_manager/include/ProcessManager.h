/*
 * ProcessManager.h
 *
 *  Created on: Nov 1, 2014
 *      Author: Stephan Opfer
 */

#ifndef PROCESSMANAGER_H_
#define PROCESSMANAGER_H_

#include "ros/ros.h"


#include <map>

using namespace std;

namespace supplementary
{

	class SystemConfig;
	class ROSProcess;

	class ProcessManager
	{
	public:
		ProcessManager(int argc, char** argv);
		virtual ~ProcessManager();
		void start();
	private:
		SystemConfig* sc;
		std::map<short,ROSProcess*> processMap;
	};

} /* namespace alica */

#endif /* PROCESSMANAGER_H_ */
