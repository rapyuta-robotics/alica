/*
 * ControlledRobot.h
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDROBOT_H_
#define SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDROBOT_H_

#include <RobotMetaData.h>
#include <QObject>
#include <process_manager/ProcessStats.h>
#include <process_manager/ProcessStat.h>
#include "QHBoxLayout"
#include "QFrame"
#include <chrono>

namespace Ui {
	class RobotProcessesWidget;
}

namespace supplementary{
	class RobotExecutableRegistry;
}

namespace ros{
	class Publisher;
}

namespace rqt_pm_control
{
	class ControlledExecutable;
	class ControlledProcessManager;

	class ControlledRobot : public QObject, public supplementary::RobotMetaData
	{
		Q_OBJECT

	public:
		ControlledRobot(string robotName, int robotId, ControlledProcessManager* parentProcessManager);
		virtual ~ControlledRobot();

		void handleProcessStat(chrono::system_clock::time_point timeMsgReceived, process_manager::ProcessStat ps);
		void sendProcessCommand(vector<int> execIds, vector<int> paramSets, int cmd);
		void updateGUI(chrono::system_clock::time_point now);
		void addExec(QWidget* exec);
		void removeExec(QWidget* exec);


		chrono::system_clock::time_point timeLastMsgReceived; /**< the last time a message was received for this robot */
		QFrame* robotProcessesQFrame; /**< The widget, used to initialise the RobotProcessesWidget */
		ControlledProcessManager* parentProcessManager;

	public Q_SLOTS:
		void updateBundles(QString text);

	private:

		string selectedBundle;
		Ui::RobotProcessesWidget* _robotProcessesWidget;


		map<int, ControlledExecutable*> controlledExecMap;

	};

} /* namespace rqt_pm_control */

#endif /* SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDROBOT_H_ */
