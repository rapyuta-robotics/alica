/*
 * ControlledRobot.h
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDROBOT_H_
#define SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDROBOT_H_

#include <chrono>

#include <RobotMetaData.h>
#include <process_manager/ProcessStats.h>
#include <process_manager/ProcessStat.h>

namespace supplementary{
	class RobotExecutableRegistry;
}

namespace rqt_pm_control
{
	class ControlledExecutable;


	class ControlledRobot : public supplementary::RobotMetaData
	{
	public:
		ControlledRobot(string robotName, int robotId);
		virtual ~ControlledRobot();

		chrono::system_clock::time_point timeLastMsgReceived; /* < the last time a message was received for this robot */

		map<int, ControlledExecutable*> controlledExecMap;
		int processManagerId; /* < determines the robot/system which executes the sending process manager */

		void handleProcessStat(process_manager::ProcessStat ps, supplementary::RobotExecutableRegistry* pmRegistry);
	private:


	};

} /* namespace rqt_pm_control */

#endif /* SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDROBOT_H_ */
