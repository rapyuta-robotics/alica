/*
 * ControlledRobot.h
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_RQT_ROBOT_CONTROL_SRC_RQT_ROBOT_CONTROL_CONTROLLEDROBOT_H_
#define SUPPLEMENTARY_RQT_ROBOT_CONTROL_SRC_RQT_ROBOT_CONTROL_CONTROLLEDROBOT_H_

#include <chrono>
#include <RobotMetaData.h>

namespace Ui {
	class RobotProcessesWidget;
}

namespace supplementary{
	class RobotExecutableRegistry;
}

namespace ros{
	class Publisher;
}

namespace rqt_robot_control
{
	class RobotsControl;

	class ControlledRobot : public supplementary::RobotMetaData
	{

	public:
		ControlledRobot(string robotName, int robotId, RobotsControl* parentRobotsControl);
		virtual ~ControlledRobot();

		void updateGUI(chrono::system_clock::time_point now);

		chrono::system_clock::time_point timeLastMsgReceived; /**< the last time a message was received for this robot */

		RobotsControl* parentRobotsControl;

	private:


	};

} /* namespace rqt_robot_control */

#endif /* SUPPLEMENTARY_RQT_ROBOT_CONTROL_SRC_RQT_ROBOT_CONTROL_CONTROLLEDROBOT_H_ */
