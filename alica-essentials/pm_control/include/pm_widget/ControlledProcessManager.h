/*
 * ControlledProcessManager.h
 *
 *  Created on: Mar 1, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_PM_CONTROL_SRC_PM_WIDGET_CONTROLLEDPROCESSMANAGER_H_
#define SUPPLEMENTARY_PM_CONTROL_SRC_PM_WIDGET_CONTROLLEDPROCESSMANAGER_H_


#include <string>
#include <utility>
#include <ros/ros.h>

#include <process_manager/ProcessStats.h>
#include <process_manager/ProcessCommand.h>

#include <QFrame>
#include <QBoxLayout>
#include <chrono>

using namespace std;

namespace supplementary {
	class SystemConfig;
	class RobotExecutableRegistry;
}

namespace pm_control {
	class PMControl;
}

namespace pm_widget
{
	class ControlledRobot;


	class ControlledProcessManager
	{
	public:
		//ControlledProcessManager(string processManagerName, int processManagerId);
		ControlledProcessManager(string processManagerName, int processManagerId, map<string, vector<pair<int, int>>>* bundlesMap, supplementary::RobotExecutableRegistry* pmRegistry, QBoxLayout* pmHorizontalLayout);
		virtual ~ControlledProcessManager();

		void updateGUI(chrono::system_clock::time_point now);
		void handleProcessStats(pair<chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr> timePstsPair);
		void sendProcessCommand(vector<int> robotIds, vector<int> execIds, vector<int> paramSets, int cmd);
		void addRobot(QFrame* robot);
		void removeRobot(QFrame* robot);

		chrono::duration<double> msgTimeOut;
		chrono::system_clock::time_point timeLastMsgReceived; /* < Time point, when the last message have been received */
		string name; /* < Hostname under which this process manager is running */
		int id; /* < The id of the host */
		map<string, vector<pair<int, int>>>* bundlesMap;
		supplementary::RobotExecutableRegistry* pmRegistry;

	private:
		map<int, ControlledRobot*> controlledRobotsMap; /* < The robots, which are controlled by this process manager */
		ros::Publisher processCommandPub;
		QBoxLayout* parentLayout;
		ControlledRobot* getControlledRobot(int robotId);
	};

} /* namespace pm_widget */

#endif /* SUPPLEMENTARY_PM_CONTROL_SRC_PM_WIDGET_CONTROLLEDPROCESSMANAGER_H_ */
