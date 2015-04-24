/*
 * ControlledExecutable.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: Stephan Opfer
 */

#include "rqt_pm_control/ControlledExecutable.h"
#include "ui_ProcessWidget.h"
#include "ui_RobotProcessesWidget.h"

namespace rqt_pm_control
{

	ControlledExecutable::ControlledExecutable(string execName, int execId, string mode, vector<char*> defaultParams, string absExecName, Ui::RobotProcessesWidget* parentRobotProcWidget) :
			ExecutableMetaData(execName, execId, mode, defaultParams, absExecName), memory(0), state('X'), cpu(0), _processWidget(new Ui::ProcessWidget()), processWidget(new QWidget()), parentRobotProcWidget(parentRobotProcWidget)
	{
		this->_processWidget->setupUi(this->processWidget);
		this->parentRobotProcWidget->verticalLayout->insertWidget(0, processWidget);
		this->_processWidget->processName->setText(QString(execName.c_str()));
	}

	ControlledExecutable::~ControlledExecutable()
	{
		this->parentRobotProcWidget->verticalLayout->removeWidget(processWidget);
		delete _processWidget;
		delete processWidget;
	}

	/**
	 * Updates the informations about the process with the given information.
	 * @param ps The given information about the process.
	 */
	void ControlledExecutable::handleStat(process_manager::ProcessStat ps)
	{
		this->timeLastMsgReceived = chrono::system_clock::now();
		cout << "ControlledExecutable: Set last message time to " << this->timeLastMsgReceived.time_since_epoch().count() << endl;

		this->cpu = ps.cpu;
		this->memory = ps.mem;
		this->state = ps.state;
		// TODO: Maybe transmit parameters?
	}

	void ControlledExecutable::updateGUI()
	{
		this->_processWidget->cpuState->setText(QString::number(this->cpu));
		this->_processWidget->memState->setText(QString::number(this->memory));
		// TODO: Running State, parameters, ...
	}

} /* namespace rqt_pm_control */
