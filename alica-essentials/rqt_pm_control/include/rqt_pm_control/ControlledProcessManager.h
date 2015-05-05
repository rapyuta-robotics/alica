/*
 * ControlledProcessManager.h
 *
 *  Created on: Mar 1, 2015
 *      Author: emmeda
 */

#ifndef SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDPROCESSMANAGER_H_
#define SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDPROCESSMANAGER_H_

#include <vector>
#include <chrono>
#include <string>
#include "process_manager/ProcessStats.h"
#include "QHBoxLayout"

using namespace std;

namespace supplementary
{
	class RobotExecutableRegistry;
}

namespace ros
{
	class Publisher;
}

namespace rqt_pm_control
{
	class ControlledRobot;


	class ControlledProcessManager
	{
	public:
		ControlledProcessManager(string name, chrono::duration<double> msgTimeOut, int processManagerId, QHBoxLayout* parentHBoxLayout, supplementary::RobotExecutableRegistry* pmRegistry, map<string, vector<int>> &bundlesMap, ros::Publisher* processCommandPub);
		virtual ~ControlledProcessManager();

		void updateGUI();
		void handleProcessStats(process_manager::ProcessStats psts, QThread* thread);

		chrono::system_clock::time_point timeLastMsgReceived;
		int processManagerId;

	private:
		string name; /* < Hostname under which this process manager is running */
		map<int, ControlledRobot*> controlledRobotsMap; /* < The robots, which are controlled by this process manager */
		chrono::duration<double> msgTimeOut;
		supplementary::RobotExecutableRegistry* pmRegistry;
		map<string, vector<int>> &bundlesMap;
		QHBoxLayout* parentHBoxLayout;

		ros::Publisher* processCommandPub;

	};

} /* namespace rqt_pm_control */

#endif /* SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDPROCESSMANAGER_H_ */
