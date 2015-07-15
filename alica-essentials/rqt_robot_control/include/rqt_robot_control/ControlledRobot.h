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
#include <QFrame>

namespace Ui {
	class RobotProcessesWidget;
	class ControlledRobotWidget;
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

	class ControlledRobot : public QObject, public supplementary::RobotMetaData
	{

		Q_OBJECT

	public:
		ControlledRobot(string robotName, int robotId, RobotsControl* parentRobotsControl);

		virtual ~ControlledRobot();

		void updateGUI(chrono::system_clock::time_point now);

		chrono::system_clock::time_point timeLastMsgReceived; /**< the last time a message was received for this robot */

		RobotsControl* parentRobotsControl;


		// GUI Methods
		void toggle();
		void show();
		void hide();
		bool shown;

		public Q_SLOTS:
		void sendRobotCommand(bool start);


	private:

		QFrame* widget;
		Ui::ControlledRobotWidget* uiControlledRobot;

	};

} /* namespace rqt_robot_control */

#endif /* SUPPLEMENTARY_RQT_ROBOT_CONTROL_SRC_RQT_ROBOT_CONTROL_CONTROLLEDROBOT_H_ */
