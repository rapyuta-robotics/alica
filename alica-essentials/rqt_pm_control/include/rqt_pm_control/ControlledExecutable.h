/*
 * ControlledExecutable.h
 *
 *  Created on: Mar 2, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDEXECUTABLE_H_
#define SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDEXECUTABLE_H_

#include <chrono>
#include <ExecutableMetaData.h>
#include <process_manager/ProcessStat.h>
#include "QWidget"

namespace Ui
{
	class ProcessWidget;
	class RobotProcessesWidget;
}

namespace rqt_pm_control
{

	class ControlledExecutable : public supplementary::ExecutableMetaData
	{
	public:
		ControlledExecutable(string execName, int execId, string mode, vector<char*> defaultParams, string absExecName, Ui::RobotProcessesWidget* parentRobotProcWidget);
		virtual ~ControlledExecutable();

		void handleStat(process_manager::ProcessStat ps);
		void updateGUI();

		chrono::system_clock::time_point timeLastMsgReceived; /* < last time a message was received for this executable */

		string params;
		char state; // The process state (zombie, running, etc)
		unsigned short cpu;
		long int memory;

		QWidget* processWidget;
		Ui::ProcessWidget* _processWidget;

	private:
		chrono::duration<double> msgTimeOut;
		Ui::RobotProcessesWidget* parentRobotProcWidget;

	};

} /* namespace rqt_pm_control */

#endif /* SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDEXECUTABLE_H_ */
