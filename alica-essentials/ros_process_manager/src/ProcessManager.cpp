/*
 * ProcessManager.cpp
 *
 *  Created on: Nov 1, 2014
 *      Author: Stephan Opfer
 */

#include "ProcessManager.h"
#include <SystemConfig.h>
#include "ROSProcess.h"

namespace supplementary
{

	ProcessManager::ProcessManager(int argc, char** argv)
	{
		this->sc = SystemConfig::getInstance();
		auto processDescriptions =	(*this->sc)["Processes"]->getSections("Processes.ProcessDescriptions", NULL);
		short curId;
		string curRosPkg, curRosExecutable, curParams;
		for (auto processSectionName : (*processDescriptions))
		{
			curId = (*this->sc)["Processes"]->get<short>("Processes.ProcessDescriptions",processSectionName.c_str(),"id", NULL);
			curRosPkg = (*this->sc)["Processes"]->get<string>("Processes.ProcessDescriptions",processSectionName.c_str(),"rospkg", NULL);
			curRosExecutable = (*this->sc)["Processes"]->get<string>("Processes.ProcessDescriptions",processSectionName.c_str(),"rosexecutable", NULL);
			curParams = (*this->sc)["Processes"]->get<string>("Processes.ProcessDescriptions",processSectionName.c_str(),"params", NULL);
			this->processMap.insert(pair<short,ROSProcess*>(curId, new ROSProcess(curId, curRosPkg, curRosExecutable, curParams)));
		}
	}

	ProcessManager::~ProcessManager()
	{
		for (auto processMapPair : this->processMap)
		{
			delete processMapPair.second;
		}
		processMap.clear();
	}

	void ProcessManager::start() {

	}

} /* namespace supplementary */

int main(int argc, char** argv)
{

	ros::init(argc, argv, "AlicaEngine");
	supplementary::ProcessManager pm = supplementary::ProcessManager(argc, argv);
	pm.start();

	while (ros::ok())
	{
		std::chrono::milliseconds dura(500);
		std::this_thread::sleep_for(dura);
	}

	return 0;
}
