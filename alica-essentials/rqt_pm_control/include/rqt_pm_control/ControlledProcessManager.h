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
#include "QFrame"

using namespace std;

namespace Ui
{
	class RobotProcessesWidget;
}

namespace rqt_pm_control
{
	class ControlledRobot;

	class ControlledProcessManager
	{
	public:
		ControlledProcessManager(string name, int processManagerId);
		virtual ~ControlledProcessManager();

		void ProcessMessage(process_manager::ProcessStats psts);
		void updateGUI(QHBoxLayout* parentLayout);
		chrono::system_clock::time_point lastTimeMsgReceived;
	private:
		string name;
		int processManagerId;

		map<int, ControlledRobot*> controlledRobotsMap;

		QFrame* robotProc;
		Ui::RobotProcessesWidget* _processManagerWidget;
	};

} /* namespace rqt_pm_control */

#endif /* SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDPROCESSMANAGER_H_ */
