/*
 * ControlledRobot.h
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDROBOT_H_
#define SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDROBOT_H_

#include <RobotMetaData.h>
#include <process_manager/ProcessStats.h>

namespace rqt_pm_control
{

	class ControlledRobot : public supplementary::RobotMetaData
	{
	public:
		ControlledRobot(string robotName, int robotId);
		virtual ~ControlledRobot();

		static unsigned long long timeLastMsgReceivedTimeOut;

	private:
		unsigned long long timeLastMsgReceived; /* < the last time a message was received for this robot */

		int processManagerId; /* < determines the robot/system which executes the sending process manager */
		process_manager::ProcessStats processStats;
	};

} /* namespace rqt_pm_control */

#endif /* SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDROBOT_H_ */
