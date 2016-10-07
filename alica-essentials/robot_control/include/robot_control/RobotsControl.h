#ifndef robot_control__PMControl_H
#define robot_control__PMControl_H

#include <rqt_gui_cpp/plugin.h>

#include "ros/ros.h"
#include <ros/macros.h>
#include <robot_control/Robot.h>

#include <process_manager/ProcessStats.h>
#include <alica_ros_proxy/AlicaEngineInfo.h>
//#include <msl_actuator_msgs/KickerStatInfo.h>

#include <ui_RobotsControl.h>
#include <QtGui>
#include <QWidget>
#include <QDialog>

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

namespace robot_control
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

		Ui::RobotControlWidget robotControlWidget_;
		QWidget* widget_;

		map<string, vector<pair<int, int>>> bundlesMap;
		supplementary::RobotExecutableRegistry* pmRegistry;
		ros::NodeHandle* rosNode;

	private:
		ros::Subscriber processStateSub;
		ros::Subscriber alicaInfoSub;
		ros::Subscriber kickerStatInfoSub;

		supplementary::SystemConfig* sc;

		map<int, Robot*> controlledRobotsMap;
		queue<pair<chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr>> processStatMsgQueue;
		mutex processStatsMsgQueueMutex;
		queue<pair<chrono::system_clock::time_point, alica_ros_proxy::AlicaEngineInfoConstPtr>> alicaInfoMsgQueue;
		mutex alicaInfoMsgQueueMutex;
//		queue<pair<chrono::system_clock::time_point, msl_actuator_msgs::KickerStatInfoPtr>> kickerStatInfoMsgQueue;
//		mutex kickerStatInfoMsgQueueMutex;

		void receiveProcessStats(process_manager::ProcessStatsConstPtr processStats);
		void receiveAlicaInfo(alica_ros_proxy::AlicaEngineInfoConstPtr alicaInfo);
//		void receiveKickerStatInfo(msl_actuator_msgs::KickerStatInfoPtr kickerStatInfo);
		void processMessages();
		void checkAndInit(int robotId);

		QTimer* guiUpdateTimer;

	public Q_SLOTS:
		void run();
		void updateGUI();
		void showContextMenu(const QPoint& pos);
	};

}

#endif // rqt_msl_refbox__RefBox_H
