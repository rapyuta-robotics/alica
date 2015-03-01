/*
 * ObservedRobot.cpp
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#include <rqt_pm_control/ControlledRobot.h>

namespace rqt_pm_control
{

	unsigned long long timeLastMsgReceivedTimeOut = 0;

	ControlledRobot::ControlledRobot(string robotName, int robotId) : RobotMetaData(robotName, robotId)
	{
		// TODO Auto-generated constructor stub

	}

	ControlledRobot::~ControlledRobot()
	{
		// TODO Auto-generated destructor stub
	}

} /* namespace rqt_pm_control */
