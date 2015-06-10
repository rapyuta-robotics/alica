#ifndef rqt_robot_control__PMControl_H
#define rqt_robot_control__PMControl_H

#include <rqt_gui_cpp/plugin.h>

#include "ros/ros.h"
#include <ros/macros.h>

#include "process_manager/ProcessStats.h"
#include "alica_ros_proxy/AlicaEngineInfo.h"

#include <ui_PMControl.h>
#include <QtGui>
#include <QWidget>
#include <QDialog>

#include <rqt_robot_control/ControlledRobot.h>

#include <queue>
#include <mutex>
#include <utility>
#include <chrono>

using namespace std;

namespace supplementary
{
	class SystemConfig;
	class RobotExecutableRegistry;
}

namespace rqt_robot_control
{

	class RobotsControl : public rqt_gui_cpp::Plugin
	{

	Q_OBJECT

	public:

		RobotsControl();
		virtual void initPlugin(qt_gui_cpp::PluginContext& context);
		virtual void shutdownPlugin();
		virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
		virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

		void addRobot();
		void removeRobot();

		static chrono::duration<double> msgTimeOut;

		Ui::PMControlWidget ui_;
		QWidget* widget_;

		supplementary::RobotExecutableRegistry* pmRegistry;

	private:
		ros::NodeHandle* rosNode;
		ros::Subscriber processStateSub;
		ros::Subscriber alicaInfoSub;

		supplementary::SystemConfig* sc;

		map<int, ControlledRobot*> controlledRobotsMap;
		queue<pair<chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr>> processStatMsgQueue;
		mutex processStatsMsgQueueMutex;
		queue<pair<chrono::system_clock::time_point, alica_ros_proxy::AlicaEngineInfoConstPtr>> alicaInfoMsgQueue;
		mutex alicaInfoMsgQueueMutex;

		void receiveProcessStats(process_manager::ProcessStatsConstPtr processStats);
		void receiveAlicaInfo(alica_ros_proxy::AlicaEngineInfoConstPtr alicaInfo);
		void processMessages();
		void checkAndInit(int robotId);

		QTimer* guiUpdateTimer;

	public Q_SLOTS:
		void run();
		void updateGUI();
	};

}

#endif // rqt_msl_refbox__RefBox_H
