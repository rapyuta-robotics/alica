#include "pm_widget/ControlledRobot.h"
#include "pm_widget/ControlledExecutable.h"
#include "pm_widget/ControlledProcessManager.h"
#include "ui_RobotProcessesWidget.h"
#include "ui_ProcessWidget.h"

#include <supplementary/AgentID.h>
#include <supplementary/BroadcastID.h>
#include <process_manager/RobotExecutableRegistry.h>
#include <process_manager/ExecutableMetaData.h>
#include <process_manager/ProcessCommand.h>
#include <SystemConfig.h>

#include <ros/ros.h>
#include <limits.h>
namespace pm_widget
{
	// Second Constructor is for robot_control
	ControlledRobot::ControlledRobot(string robotName, const supplementary::AgentID* robotId, const supplementary::AgentID* parentPMid) :
			RobotMetaData(robotName, robotId), robotProcessesQFrame(new QFrame()), _robotProcessesWidget(new Ui::RobotProcessesWidget()), parentPMid(parentPMid)
	{
		// setup gui stuff
		this->_robotProcessesWidget->setupUi(this->robotProcessesQFrame);
		auto pmRegistry = supplementary::RobotExecutableRegistry::get();
		if (dynamic_cast<const supplementary::BroadcastID*>(parentPMid))
		{
			// don't show in robot_control
			this->_robotProcessesWidget->robotHostLabel->hide();
			this->inRobotControl = true;
		}
		else
		{
			string pmName;
			if (pmRegistry->getRobotName(parentPMid, pmName))
			{
				this->_robotProcessesWidget->robotHostLabel->setText(QString((robotName + " on " + pmName).c_str()));
			}
			else
			{
				this->_robotProcessesWidget->robotHostLabel->setText(QString((robotName + " on UNKNOWN").c_str()));
			}
			this->inRobotControl = false;
		}

		QObject::connect(this->_robotProcessesWidget->bundleComboBox, SIGNAL(activated(QString)), this,
							SLOT(updateBundles(QString)));


		// enter bundles in combo box
		for (auto bundleEntry : *pmRegistry->getBundlesMap())
		{
			this->_robotProcessesWidget->bundleComboBox->insertItem(INT_MAX, QString(bundleEntry.first.c_str()),
																	QVariant(bundleEntry.first.c_str()));
		}

		// construct all known executables
		const vector<supplementary::ExecutableMetaData*>& execMetaDatas = pmRegistry->getExecutables();
		ControlledExecutable* controlledExec;
		for (auto execMetaDataEntry : execMetaDatas)
		{
			controlledExec = new ControlledExecutable(execMetaDataEntry, this);
			this->controlledExecMap.emplace(execMetaDataEntry->id, controlledExec);
		}

		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->msgTimeOut = chrono::duration<double>((*sc)["ProcessManaging"]->get<unsigned long>("PMControl.timeLastMsgReceivedTimeOut", NULL));

		ros::NodeHandle* nh = new ros::NodeHandle();
		string cmdTopic = (*supplementary::SystemConfig::getInstance())["ProcessManaging"]->get<string>("Topics.processCmdTopic", NULL);
		processCommandPub = nh->advertise<process_manager::ProcessCommand>(cmdTopic, 10);
	}

	ControlledRobot::~ControlledRobot()
	{
		for (auto execEntry : this->controlledExecMap)
		{
			delete execEntry.second;
		}
		delete robotProcessesQFrame;
	}

	void ControlledRobot::handleProcessStat(chrono::system_clock::time_point timeMsgReceived,
											process_manager::ProcessStat ps, const supplementary::AgentID* parentPMid)
	{
		this->parentPMid = parentPMid;

		auto controlledExecEntry = this->controlledExecMap.find(ps.processKey);
		if (controlledExecEntry != this->controlledExecMap.end())
		{ // executable is already known
			this->timeLastMsgReceived = timeMsgReceived;
			// update the statistics of the ControlledExecutable
			controlledExecEntry->second->handleStat(timeMsgReceived, ps);
		}
		else
		{ // executable is unknown
			cerr << "ControlledRobot: Received processStat for unknown executable with process key " << ps.processKey
					<< endl;
			return;
		}
	}

	void ControlledRobot::updateGUI(chrono::system_clock::time_point now)
	{
		if ((now - this->timeLastMsgReceived) > this->msgTimeOut && !inRobotControl)
		{ // time is over, erase controlled robot
			this->robotProcessesQFrame->hide();
		}
		else
		{
			this->robotProcessesQFrame->show();
		}

		for (auto controlledExecEntry : this->controlledExecMap)
		{
			controlledExecEntry.second->updateGUI(now);
		}
	}

	void ControlledRobot::updateBundles(QString text)
	{
		for (auto controlledExecMapEntry : this->controlledExecMap)
		{
			controlledExecMapEntry.second->handleBundleComboBoxChanged(text);
		}
	}

	void ControlledRobot::addExec(QWidget* exec)
	{
		this->_robotProcessesWidget->verticalLayout->insertWidget(2, exec);
	}

	void ControlledRobot::removeExec(QWidget* exec)
	{
		this->_robotProcessesWidget->verticalLayout->removeWidget(exec);
	}

	void ControlledRobot::sendProcessCommand(vector<int> execIds, vector<int> paramSets, int cmd)
	{
		process_manager::ProcessCommand pc;
		pc.receiverId.type = this->parentPMid->getType();
		pc.receiverId.id = this->parentPMid->toByteVector();
		pc.robotIds.push_back(process_manager::ProcessCommand::_robotIds_type::value_type());
		pc.robotIds[0].id = this->agentID->toByteVector();
		pc.processKeys = execIds;
		pc.paramSets = paramSets;
		pc.cmd = cmd;
		this->processCommandPub.publish(pc);
	}
} /* namespace pm_widget */
