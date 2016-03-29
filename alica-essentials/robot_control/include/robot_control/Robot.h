/*
 * ControlledRobot.h
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_RQT_ROBOT_CONTROL_SRC_RQT_ROBOT_CONTROL_CONTROLLEDROBOT_H_
#define SUPPLEMENTARY_RQT_ROBOT_CONTROL_SRC_RQT_ROBOT_CONTROL_CONTROLLEDROBOT_H_

#include <chrono>
#include <process_manager/RobotMetaData.h>
#include <QFrame>
#include <ros/ros.h>
#include <alica_ros_proxy/AlicaEngineInfo.h>
#include <process_manager/ProcessStats.h>
#include <msl_actuator_msgs/KickerStatInfo.h>

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

namespace alica {
	class AlicaWidget;
}

namespace pm_widget {
	class ControlledRobot;
}

namespace robot_control
{
	class RobotsControl;

	class Robot : public QFrame, public supplementary::RobotMetaData
	{

		Q_OBJECT

	public:
		Robot(string robotName, int robotId, RobotsControl* parentRobotsControl);

		virtual ~Robot();

		// Message processing
		chrono::time_point<chrono::system_clock> timeLastMsgReceived; /**< the last time a message was received for this robot */
		void handleAlicaInfo(pair<chrono::system_clock::time_point, alica_ros_proxy::AlicaEngineInfoConstPtr> timeAEIpair);
		void handleKickerStatInfo(pair<chrono::system_clock::time_point, msl_actuator_msgs::KickerStatInfoPtr> timeKSIpair);
		void handleProcessStat(chrono::system_clock::time_point timeMsgReceived, process_manager::ProcessStat ps, int parentPMid);

		// GUI Methods and Members
		RobotsControl* parentRobotsControl;
		void updateGUI(chrono::system_clock::time_point now);
		void toggle();
		void show();
		void hide();
		virtual QSize sizeHint();
		bool shown;
		bool showAlicaClient;
		bool showProcessManager;

		public Q_SLOTS:
		void sendRobotCommand(bool start);
		void toggleAlicaClient(bool start);
		void toggleProcessManager(bool start);


	private:
		QFrame* widget;
		Ui::ControlledRobotWidget* uiControlledRobot;

		QFrame* frameForAW;
		alica::AlicaWidget* alicaWidget;
		QFrame* frameForPM;
		pm_widget::ControlledRobot* controlledRobotWidget;

		ros::Publisher robotCommandPub;

	};

} /* namespace robot_control */

#endif /* SUPPLEMENTARY_RQT_ROBOT_CONTROL_SRC_RQT_ROBOT_CONTROL_CONTROLLEDROBOT_H_ */
