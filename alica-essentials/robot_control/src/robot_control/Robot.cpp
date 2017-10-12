#include "robot_control/Robot.h"

#include "robot_control/RobotCommand.h"
#include "robot_control/RobotsControl.h"
#include "ui_ControlledRobot.h"

#include <pm_widget/ControlledRobot.h>

#include <supplementary/BroadcastID.h>
#include <process_manager/RobotExecutableRegistry.h>

#include <alica/AlicaWidget.h>

#include <ros/ros.h>
#include <QFrame>
#include <chrono>
#include <limits.h>

using std::string;
using std::pair;

namespace robot_control
{
	Robot::Robot(string robotName, const supplementary::IAgentID* robotId, RobotsControl* parentRobotsControl) :
			RobotMetaData(robotName, robotId), parentRobotsControl(parentRobotsControl), widget(new QFrame()), uiControlledRobot(
					new Ui::ControlledRobotWidget()), shown(true), showAlicaClient(true), showProcessManager(true)
	{
		this->uiControlledRobot->setupUi(this);

		// manual configuration of widgets
		stringstream ss;
		ss << robotId;
		this->uiControlledRobot->robotStartStopBtn->setText(QString(std::string(this->name + " (" + ss.str() + ")" ).c_str()));

		frameForAW = new QFrame(this);
		frameForAW->setFrameStyle(1);
		frameForAW->setMaximumSize(280,500);
		frameForAW->setContentsMargins(0,0,0,0);
		QHBoxLayout* frameHBoxAw = new QHBoxLayout(frameForAW);
		frameHBoxAw->setContentsMargins(0,0,0,0);
		frameHBoxAw->setAlignment(Qt::AlignmentFlag::AlignTop);
		this->alicaWidget = new alica::AlicaWidget();
		frameHBoxAw->addWidget(this->alicaWidget->qframe);
		this->uiControlledRobot->horizontalLayout_2->addWidget(frameForAW);

		frameForPM = new QFrame(this);
		frameForPM->setFrameStyle(1);
		frameForPM->setMaximumSize(280,500);
		frameForPM->setContentsMargins(0,0,0,0);
		QHBoxLayout* frameHBoxPm = new QHBoxLayout(frameForPM);
		frameHBoxPm->setContentsMargins(0,0,0,0);
		frameHBoxPm->setAlignment(Qt::AlignmentFlag::AlignTop);
		this->broadcastId = new supplementary::BroadcastID(nullptr, 0);
		this->controlledRobotWidget = new pm_widget::ControlledRobot(robotName, robotId, this->broadcastId);
		frameHBoxPm->addWidget(this->controlledRobotWidget->robotProcessesQFrame);
		this->uiControlledRobot->horizontalLayout_2->addWidget(frameForPM);

		// signals and slots
		QObject::connect(this->uiControlledRobot->robotStartStopBtn, SIGNAL(toggled(bool)), this,
							SLOT(sendRobotCommand(bool)));
		QObject::connect(this->uiControlledRobot->alicaShowHideBtn, SIGNAL(toggled(bool)), this,
							SLOT(toggleAlicaClient(bool)));
		QObject::connect(this->uiControlledRobot->pmShowHideBtn, SIGNAL(toggled(bool)), this,
							SLOT(toggleProcessManager(bool)));

		//vBox->addWidget(new QSpacerItem(0,0,QSizePolicy::Expanding, QSizePolicy::Line));
		this->parentRobotsControl->robotControlWidget_.robotControlLayout->addWidget(this);

		this->robotCommandPub = this->parentRobotsControl->rosNode->advertise<robot_control::RobotCommand>("RobotCommand",5);
	}

	Robot::~Robot()
	{
		delete this->broadcastId;
	}

	QSize Robot::sizeHint()
	{
		return QSize(500,500);
	}

	void Robot::toggleAlicaClient(bool show)
	{
		this->showAlicaClient = show;

		if (showAlicaClient)
		{
			this->frameForAW->show();
			this->uiControlledRobot->scrollArea->show();
		}
		else
		{
			this->frameForAW->hide();
			if (!showProcessManager)
			{
				this->uiControlledRobot->scrollArea->hide();
			}
		}

	}

	void Robot::toggleProcessManager(bool show)
	{
		this->showProcessManager = show;

		if (showProcessManager)
		{
			this->frameForPM->show();
			this->uiControlledRobot->scrollArea->show();
		}
		else
		{
			this->frameForPM->hide();
			if (!showAlicaClient)
			{
				this->uiControlledRobot->scrollArea->hide();
			}
		}

	}

	void Robot::updateGUI(std::chrono::system_clock::time_point now)
	{
		if ((now - this->timeLastMsgReceived) > std::chrono::milliseconds(1000))
		{
			this->alicaWidget->clearGUI();
		}
		this->controlledRobotWidget->updateGUI(now);
	}

	void Robot::sendRobotCommand(bool start)
	{
		RobotCommand rc;
		rc.receiverId.id = this->agentID->toByteVector();
		if (start)
		{
			rc.cmd = RobotCommand::START;
		}
		else
		{
			rc.cmd = RobotCommand::STOP;
		}
		this->robotCommandPub.publish(rc);
	}

	void Robot::toggle()
	{
		if (shown)
		{
			this->hide();
		}
		else
		{
			this->show();
		}
	}

	void Robot::show()
	{
		this->shown = true;
		this->uiControlledRobot->scrollArea->adjustSize();
		QFrame::show();
	}

	void Robot::hide()
	{
		this->shown = false;
		this->uiControlledRobot->scrollArea->adjustSize();
		QFrame::hide();
	}

	void Robot::handleAlicaInfo(pair<std::chrono::system_clock::time_point, alica_ros_proxy::AlicaEngineInfoConstPtr> timeAEIpair)
	{
		this->timeLastMsgReceived = timeAEIpair.first;
		this->alicaWidget->handleAlicaEngineInfo(timeAEIpair.second);
	}

	void Robot::handleKickerStatInfo(pair<std::chrono::system_clock::time_point, msl_actuator_msgs::KickerStatInfoPtr> timeKSIpair)
	{
		this->timeLastMsgReceived = timeKSIpair.first;
		this->alicaWidget->handleKickerStatInfo(timeKSIpair.second);
	}

	void Robot::handleSharedWorldInfo(pair<std::chrono::system_clock::time_point, msl_sensor_msgs::SharedWorldInfoPtr> timeSWIpair)
	{
		this->timeLastMsgReceived = timeSWIpair.first;
		this->alicaWidget->handleSharedWorldInfo(timeSWIpair.second);
	}

        void Robot::handleProcessStat(std::chrono::system_clock::time_point timeMsgReceived,
                                      process_manager::ProcessStat ps, const supplementary::IAgentID *parentPMid)
        {
		this->timeLastMsgReceived = timeMsgReceived;
		this->controlledRobotWidget->handleProcessStat(timeMsgReceived, ps, parentPMid);
	}


} /* namespace robot_control */
