/*
 * ControlledExecutable.h
 *
 *  Created on: Mar 2, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDEXECUTABLE_H_
#define SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDEXECUTABLE_H_

#include <ExecutableMetaData.h>
#include <process_manager/ProcessStat.h>
#include "QWidget"

#include <chrono>

namespace Ui
{
	class ProcessWidget;
	class RobotProcessesWidget;
}

namespace rqt_pm_control
{
	class ControlledRobot;

	class ControlledExecutable : public QObject, public supplementary::ExecutableMetaData
	{
		Q_OBJECT

	public:
		ControlledExecutable(string execName, int execId, string mode, vector<char*> defaultParams, string absExecName, ControlledRobot* parentRobot);
		virtual ~ControlledExecutable();

		void handleStat(chrono::system_clock::time_point timeMsgReceived, process_manager::ProcessStat ps);
		void updateGUI(chrono::system_clock::time_point now);

		chrono::system_clock::time_point timeLastMsgReceived; /* < last time a message was received for this executable */

		string params;
		char state; // The process state (zombie, running, etc.)
		unsigned short cpu;
		long int memory;

		QWidget* processWidget;
		Ui::ProcessWidget* _processWidget;

	public Q_SLOTS:
		void handleCheckBoxStateChanged(int newState);

	Q_SIGNALS:
		void processCheckBoxStateChanged(int, int); /** < first int is newState, second int is execId */

	private:
		static const string redBackground;
		static const string greenBackground;
		static const string grayBackground;
		ControlledRobot* parentRobot;

	};

} /* namespace rqt_pm_control */

#endif /* SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDEXECUTABLE_H_ */

