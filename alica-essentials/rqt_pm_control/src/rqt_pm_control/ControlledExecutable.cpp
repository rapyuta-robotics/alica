/*
 * ControlledExecutable.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: Stephan Opfer
 */

#include "rqt_pm_control/ControlledExecutable.h"
#include "ui_ProcessWidget.h"
#include "ui_RobotProcessesWidget.h"

#include "rqt_pm_control/PMControl.h"

namespace rqt_pm_control
{

	const string ControlledExecutable::redBackground = "background-color:#FF4719;";
	const string ControlledExecutable::greenBackground = "background-color:#66FF66;";
	const string ControlledExecutable::grayBackground = "background-color:gray;";

	ControlledExecutable::ControlledExecutable(string execName, int execId, string mode, vector<char*> defaultParams, string absExecName,
												ControlledRobot* parentRobot) :
			ExecutableMetaData(execName, execId, mode, defaultParams, absExecName), memory(0), state('U'), cpu(0), _processWidget(new Ui::ProcessWidget()), processWidget(
					new QWidget()), parentRobot(parentRobot)
	{

		this->_processWidget->setupUi(this->processWidget);
		this->_processWidget->processName->setText(QString(this->name.c_str()));
		if (this->name == "roscore")
		{
			this->_processWidget->checkBox->setEnabled(false);
			this->_processWidget->checkBox->setChecked(true);
		}
		else
		{
			QObject::connect(this->_processWidget->checkBox, SIGNAL(stateChanged(int)), this, SLOT(handleCheckBoxStateChanged(int)),
										Qt::DirectConnection);
		}
		this->parentRobot->addExec(processWidget);
		this->processWidget->show();
	}

	ControlledExecutable::~ControlledExecutable()
	{
		this->parentRobot->removeExec(processWidget);
		delete _processWidget;
		delete processWidget;
	}

	/**
	 * Updates the informations about the process with the given information.
	 * @param ps The given information about the process.
	 */
	void ControlledExecutable::handleStat(chrono::system_clock::time_point timeMsgReceived, process_manager::ProcessStat ps)
	{
		this->timeLastMsgReceived = timeMsgReceived;
		this->cpu = ps.cpu;
		this->memory = ps.mem;
		this->state = ps.state;
		// TODO: Maybe transmit parameters?
	}

	void ControlledExecutable::updateGUI(chrono::system_clock::time_point now)
	{
		if ((now - this->timeLastMsgReceived) > PMControl::msgTimeOut)
		{ // time is over, erase controlled robot

			this->_processWidget->cpuState->setText(QString("C: -- %"));
			this->_processWidget->memState->setText(QString("M: -- MB"));
			this->processWidget->setStyleSheet(redBackground.c_str());
		}
		else
		{ // message arrived before timeout, update its GUI

			QString cpuString = "C: " + QString::number(this->cpu) + " %";
			QString memString = "M: " + QString::number(this->memory) + " MB";
			this->_processWidget->cpuState->setText(cpuString);
			this->_processWidget->memState->setText(memString);

			switch (this->state)
			{
				case 'R': // running
				case 'S': // interruptable sleeping
				case 'D': // uninterruptable sleeping
				case 'W': // paging
					this->processWidget->setStyleSheet(greenBackground.c_str());
					break;
				case 'Z': // zombie
				case 'T': // traced, or stopped
					this->processWidget->setStyleSheet(redBackground.c_str());
					break;
				case 'U':
				default:
					cout << "ControlledExec: Unknown process state '" << this->state << "' encountered!" << endl;
					this->processWidget->setStyleSheet(grayBackground.c_str());
					break;
			}
		}
	}

	void ControlledExecutable::handleCheckBoxStateChanged(int newState)
	{
		cout << "ControlledExec: Checked CheckBox from executable " << this->name << " new State is " << newState << endl;
		this->parentRobot->sendProcessCommand(vector<int>{this->id}, newState);
	}

} /* namespace rqt_pm_control */
