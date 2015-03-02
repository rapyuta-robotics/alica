/*
 * ControlledProcessManager.h
 *
 *  Created on: Mar 1, 2015
 *      Author: emmeda
 */

#ifndef SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDPROCESSMANAGER_H_
#define SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDPROCESSMANAGER_H_

#include <vector>

using namespace std;

namespace rqt_pm_control
{
	class ControlledRobot;

	class ControlledProcessManager
	{
	public:
		ControlledProcessManager(int processManagerId);
		virtual ~ControlledProcessManager();

		int processManagerId;
		vector<ControlledRobot*> controlledRobotsList;
	};

} /* namespace rqt_pm_control */

#endif /* SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDPROCESSMANAGER_H_ */
