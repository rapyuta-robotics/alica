/*
 * ControlledExecutable.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: Stephan Opfer
 */

#include <rqt_robot_control/RobotsControl.h>
#include "rqt_robot_control/ControlledExecutable.h"
#include "ui_ProcessWidget.h"
#include "ui_RobotProcessesWidget.h"
#include "ExecutableMetaData.h"
#include "rqt_robot_control/ControlledProcessManager.h"


namespace rqt_robot_control
{

	const string ControlledExecutable::redBackground = "background-color:#FF4719;";
	const string ControlledExecutable::greenBackground = "background-color:#66FF66;";
	const string ControlledExecutable::grayBackground = "background-color:gray;";

	ControlledExecutable::ControlledExecutable(supplementary::ExecutableMetaData* metaExec,
												ControlledRobot* parentRobot) :
			metaExec(metaExec), memory(0), state('U'), cpu(0), _processWidget(new Ui::ProcessWidget()), processWidget(
					new QWidget()), parentRobot(parentRobot), runningParamSet(
					supplementary::ExecutableMetaData::UNKNOWN_PARAMS), desiredParamSet(INT_MAX)
	{

		for (auto paramEntry : this->metaExec->parameterMap)
		{
			if (this->desiredParamSet > paramEntry.first)
			{
				this->desiredParamSet = paramEntry.first;
			}
		}

		this->_processWidget->setupUi(this->processWidget);
		this->_processWidget->processName->setText(QString(this->metaExec->name.c_str()));
		if (this->metaExec->name == "roscore")
		{
			this->_processWidget->checkBox->setEnabled(false);
			this->_processWidget->checkBox->setChecked(true);
		}
		else
		{
			QObject::connect(this->_processWidget->checkBox, SIGNAL(stateChanged(int)), this,
								SLOT(handleCheckBoxStateChanged(int)), Qt::DirectConnection);
		}
		this->parentRobot->addExec(processWidget);
		this->processWidget->show();
	}

	ControlledExecutable::~ControlledExecutable()
	{

	}

	/**
	 * Updates the informations about the process with the given information.
	 * @param ps The given information about the process.
	 */
	void ControlledExecutable::handleStat(chrono::system_clock::time_point timeMsgReceived,
											process_manager::ProcessStat ps)
	{
		this->timeLastMsgReceived = timeMsgReceived;
		this->cpu = ps.cpu;
		this->memory = ps.mem;
		this->state = ps.state;
		this->runningParamSet = ps.paramSet;
		auto entry = this->metaExec->parameterMap.find(this->runningParamSet);
		if (entry != this->metaExec->parameterMap.end())
		{
			stringstream ss;
			ss << "Command: ";
			for (auto param : entry->second)
			{
				if (param != nullptr)
				{
					ss << param << " ";
				}
			}
			this->processWidget->setToolTip(QString(ss.str().c_str()));
		}
		else
		{
			this->processWidget->setToolTip(QString("Command: Unknown"));
		}
	}

	void ControlledExecutable::updateGUI(chrono::system_clock::time_point now)
	{
		if ((now - this->timeLastMsgReceived) > RobotsControl::msgTimeOut)
		{ // time is over, erase controlled robot

			this->_processWidget->cpuState->setText(QString("C: -- %"));
			this->_processWidget->memState->setText(QString("M: -- MB"));
			this->runningParamSet = supplementary::ExecutableMetaData::UNKNOWN_PARAMS;
			this->processWidget->setToolTip(QString(""));
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
					this->processWidget->setToolTip(QString(""));
					this->runningParamSet = supplementary::ExecutableMetaData::UNKNOWN_PARAMS;
					break;
				case 'U':
				default:
					cout << "ControlledExec: Unknown process state '" << this->state << "' encountered!" << endl;
					this->processWidget->setStyleSheet(grayBackground.c_str());
					break;
			}
		}
	}

	void ControlledExecutable::handleBundleComboBoxChanged(QString bundle)
	{
		for (auto paramEntry : this->metaExec->parameterMap)
		{
			if (this->desiredParamSet > paramEntry.first)
			{
				this->desiredParamSet = paramEntry.first;
			}
		}

		if (bundle == "ALL")
		{
			this->processWidget->show();
			if (this->metaExec->name != "roscore")
			{
				this->_processWidget->checkBox->setEnabled(true);
				return;
			}
		}

		if (bundle == "RUNNING")
		{
			switch (this->state)
			{
				case 'R': // running
				case 'S': // interruptable sleeping
				case 'D': // uninterruptable sleeping
				case 'W': // paging
				case 'Z': // zombie
					this->processWidget->show();
					if (this->metaExec->name != "roscore")
					{
						this->_processWidget->checkBox->setEnabled(true);
					}
					break;
				case 'T': // traced, or stopped
				case 'U': // unknown
				default:
					this->processWidget->hide();
					break;
			}
			return;
		}

		auto bundleMapEntry = this->parentRobot->parentProcessManager->parentPMControl->bundlesMap.find(
				bundle.toStdString());
		if (bundleMapEntry != this->parentRobot->parentProcessManager->parentPMControl->bundlesMap.end())
		{
			bool found = false;
			for (auto processParamSetPair : bundleMapEntry->second)
			{
				if (this->metaExec->id == processParamSetPair.first)
				{
					found = true;
					this->desiredParamSet = processParamSetPair.second;
					if (processParamSetPair.second == this->runningParamSet
							|| this->runningParamSet == supplementary::ExecutableMetaData::UNKNOWN_PARAMS)
					{
						if (this->metaExec->name != "roscore")
						{
							this->_processWidget->checkBox->setEnabled(true);
						}
					}
					else
					{ // disable the checkbox, if the wrong bundle is selected
						this->_processWidget->checkBox->setEnabled(false);
					}
				}
			}
			if (found)
			{
				this->processWidget->show();
			}
		}
	}

	void ControlledExecutable::handleCheckBoxStateChanged(int newState)
	{
		cout << "ControlledExec: Checked CheckBox from executable " << this->metaExec->name << " new State is "
				<< newState << endl;
		this->parentRobot->sendProcessCommand(vector<int> {this->metaExec->id},vector<int> {this->desiredParamSet}, newState);
	}

} /* namespace rqt_robot_control */
