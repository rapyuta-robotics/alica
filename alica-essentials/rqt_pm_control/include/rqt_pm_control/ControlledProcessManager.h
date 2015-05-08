/*
 * ControlledProcessManager.h
 *
 *  Created on: Mar 1, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDPROCESSMANAGER_H_
#define SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDPROCESSMANAGER_H_

#include <chrono>
#include <string>
#include <utility>

#include "process_manager/ProcessStats.h"

#include <QFrame>

using namespace std;

namespace rqt_pm_control
{
	class ControlledRobot;
	class PMControl;

	class ControlledProcessManager
	{
	public:
		ControlledProcessManager(string processManagerName, int processManagerId, PMControl* parentPMControl);// QHBoxLayout* parentHBoxLayout, supplementary::RobotExecutableRegistry* pmRegistry, map<string, vector<int>> &bundlesMap, ros::Publisher* processCommandPub, chrono::duration<double> msgTimeOut,);
		virtual ~ControlledProcessManager();

		void updateGUI(chrono::system_clock::time_point now);
		void handleProcessStats(pair<chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr> timePstsPair);
		void sendProcessCommand(vector<int> robotIds, vector<int> execIds, int newState);
		void addRobot(QFrame* robot);
		void removeRobot(QFrame* robot);

		chrono::system_clock::time_point timeLastMsgReceived; /* < Time point, when the last message have been received */
		string name; /* < Hostname under which this process manager is running */
		int id; /* < The id of the host */
		PMControl* parentPMControl; /* < Pointer to the parent PMControl */

	private:

		map<int, ControlledRobot*> controlledRobotsMap; /* < The robots, which are controlled by this process manager */
		ControlledRobot* getControlledRobot(int robotId);

	};

} /* namespace rqt_pm_control */

#endif /* SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDPROCESSMANAGER_H_ */
