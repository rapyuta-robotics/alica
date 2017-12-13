#pragma once

#include <chrono>
#include <process_manager/RobotMetaData.h>
#include <QFrame>
#include <ros/ros.h>
#include <alica_ros_proxy/AlicaEngineInfo.h>
#include <process_manager/ProcessStats.h>
#include <msl_actuator_msgs/KickerStatInfo.h>
#include <msl_sensor_msgs/SharedWorldInfo.h>

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
		Robot(std::string robotName, int robotId, RobotsControl* parentRobotsControl);

		virtual ~Robot();

		// Message processing
		std::chrono::time_point<std::chrono::system_clock> timeLastMsgReceived; /**< the last time a message was received for this robot */
		void handleAlicaInfo(std::pair<std::chrono::system_clock::time_point, alica_ros_proxy::AlicaEngineInfoConstPtr> timeAEIpair);
		void handleProcessStat(std::chrono::system_clock::time_point timeMsgReceived, process_manager::ProcessStat ps, int parentPMid);

		// GUI Methods and Members
		RobotsControl* parentRobotsControl;
		void updateGUI(std::chrono::system_clock::time_point now);
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

