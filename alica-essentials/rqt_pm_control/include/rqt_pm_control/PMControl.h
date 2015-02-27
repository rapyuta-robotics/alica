#ifndef rqt_pm_control__PMControl_H
#define rqt_pm_control__PMControl_H

#include <rqt_gui_cpp/plugin.h>
#include "ros/ros.h"
#include <ui_PMControl.h>
#include <ros/macros.h>
#include <QtGui>
#include <QWidget>
#include <QDialog>

#include "process_manager/ProcessCommand.h"
#include "process_manager/ProcessStats.h"
#include "process_manager/ProcessStat.h"
#include "ObservedRobot.h"

using namespace std;

namespace rqt_pm_control
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

		Ui::PMControlWidget ui_;

		QWidget* widget_;

	private:
		ros::NodeHandle* rosNode;
		ros::AsyncSpinner* spinner;
		ros::Subscriber processStateSub;
		ros::Publisher processCommandPub;

		map<int, vector<ObservedRobot*> > procMan2RobotsMap;

		void handleProcessStats(process_manager::ProcessStats psts);
		bool eventFilter(QObject* watched, QEvent* event);

	};

}

#endif // rqt_msl_refbox__RefBox_H
