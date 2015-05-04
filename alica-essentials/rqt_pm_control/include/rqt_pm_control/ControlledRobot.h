/*
 * ControlledRobot.h
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDROBOT_H_
#define SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDROBOT_H_

#include <chrono>

#include <RobotMetaData.h>
#include <QObject>
#include <process_manager/ProcessStats.h>
#include <process_manager/ProcessStat.h>
#include "QHBoxLayout"
#include "QFrame"

namespace Ui {
	class RobotProcessesWidget;
}

namespace supplementary{
	class RobotExecutableRegistry;
}

namespace rqt_pm_control
{
	class ControlledExecutable;

	class ControlledRobot : public QObject, public supplementary::RobotMetaData
	{
		Q_OBJECT

	public:
		ControlledRobot(string pmName, chrono::duration<double> msgTimeOut, supplementary::RobotExecutableRegistry* pmRegistry, map<string, vector<int>> &bundlesMap,
						string robotName, int robotId);
		virtual ~ControlledRobot();

		void handleProcessStat(process_manager::ProcessStat ps);

		void updateGUI(QHBoxLayout* parentHBoxLayout);

		chrono::system_clock::time_point timeLastMsgReceived; /* < the last time a message was received for this robot */
		map<int, ControlledExecutable*> controlledExecMap;
		QFrame* robotProcessesQFrame; /* < The widget, used to initialise the RobotProcessesWidget */

	public Q_SLOTS:
		void updateBundles(QString text);

	private:

		supplementary::RobotExecutableRegistry* pmRegistry;
		map<string, vector<int>> &bundlesMap;
		string selectedBundle;
		string pmName;
		Ui::RobotProcessesWidget* _robotProcessesWidget;
		QHBoxLayout* parentHBoxLayout;
		chrono::duration<double> msgTimeOut;

	};

} /* namespace rqt_pm_control */

#endif /* SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDROBOT_H_ */
