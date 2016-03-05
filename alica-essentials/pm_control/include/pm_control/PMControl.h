#ifndef pm_control__PMControl_H
#define pm_control__PMControl_H

#include <rqt_gui_cpp/plugin.h>

#include "ros/ros.h"
#include <ros/macros.h>
#include "process_manager/ProcessStats.h"
#include "process_manager/ProcessStat.h"

#include <ui_PMControl.h>
#include <QtGui>
#include <QWidget>
#include <QDialog>

#include <pm_widget/ControlledRobot.h>

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

namespace pm_widget {
	class ControlledProcessManager;
}

namespace pm_control
{
	class PMControl : public rqt_gui_cpp::Plugin
	{

	Q_OBJECT

	public:

		PMControl();
		virtual void initPlugin(qt_gui_cpp::PluginContext& context);
		virtual void shutdownPlugin();
		virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
		virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

		void sendProcessCommand(int receiverId, vector<int> robotIds, vector<int> execIds, vector<int> paramSets, int cmd);
//		void addRobot(QFrame* robot);
//		void removeRobot(QFrame* robot);

		chrono::duration<double> msgTimeOut;

		Ui::PMControlWidget ui_;
		QWidget* widget_;

		supplementary::RobotExecutableRegistry* pmRegistry;

	private:
		ros::NodeHandle* rosNode;
		ros::Subscriber processStateSub;
		ros::Publisher processCommandPub;
		queue<pair<chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr>> processStatMsgQueue;
		mutex msgQueueMutex;

		supplementary::SystemConfig* sc;

		map<int, pm_widget::ControlledProcessManager*> processManagersMap;


		void handleProcessStats();

		void receiveProcessStats(process_manager::ProcessStatsConstPtr psts);
		pm_widget::ControlledProcessManager* getControlledProcessManager(int processManagerId);



		QTimer* guiUpdateTimer;

	public Q_SLOTS:
		void run();
		void updateGUI();
	};

}

#endif 
