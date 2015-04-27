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

	ControlledExecutable::ControlledExecutable(string execName, int execId, string mode, vector<char*> defaultParams, string absExecName) :
			ExecutableMetaData(execName, execId, mode, defaultParams, absExecName), memory(0), state('X'), cpu(0), _processWidget(nullptr), processWidget(nullptr), parentRobotProcWidget(nullptr)
	{

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

	void ControlledExecutable::updateGUI(Ui::RobotProcessesWidget* parentRobotProcWidget)
	{
		if (this->parentRobotProcWidget == nullptr)
		{
			this->parentRobotProcWidget = parentRobotProcWidget;
			this->processWidget = new QWidget();
			this->_processWidget = new Ui::ProcessWidget();
			this->_processWidget->setupUi(this->processWidget);
			this->parentRobotProcWidget->verticalLayout->insertWidget(0, processWidget);
			this->_processWidget->processName->setText(QString(this->name.c_str()));
		}

		this->_processWidget->cpuState->setText(QString::number(this->cpu));
		this->_processWidget->memState->setText(QString::number(this->memory));
		// TODO: Running State, parameters, ...
	}

} /* namespace rqt_pm_control */
