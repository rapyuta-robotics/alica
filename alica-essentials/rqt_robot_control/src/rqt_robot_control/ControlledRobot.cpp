/*
 * ControlledRobot.cpp
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#include "rqt_robot_control/ControlledRobot.h"
#include "rqt_robot_control/RobotsControl.h"
#include "ui_ControlledRobot.h"

#include <RobotExecutableRegistry.h>
#include <limits.h>
#include <ros/ros.h>

namespace rqt_robot_control
{
	ControlledRobot::ControlledRobot(string robotName, int robotId, RobotsControl* parentRobotsControl) :
			RobotMetaData(robotName, robotId), parentRobotsControl(parentRobotsControl), widget(new QFrame()), uiControlledRobot(
					new Ui::ControlledRobotWidget()), shown(false)
	{
		this->uiControlledRobot->setupUi(this->widget);
		this->uiControlledRobot->robotStartStopBtn->setText(QString(this->name.c_str()));
		QObject::connect(this->uiControlledRobot->robotStartStopBtn, SIGNAL(toggled(bool)), this, SLOT(sendRobotCommand(bool)));
		this->widget->hide();
		this->parentRobotsControl->robotControlWidget_.allRobotsFlowLayout->addWidget(this->widget);
	}

	ControlledRobot::~ControlledRobot()
	{
	}

	void ControlledRobot::updateGUI(chrono::system_clock::time_point now)
	{
	}

	void ControlledRobot::sendRobotCommand(bool start)
	{
		if (start)
		{
			cout << "CR: sendStart()" << endl;
		}
		else
		{
			cout << "CR: sendStop()" << endl;
		}
	}

	void ControlledRobot::toggle()
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

	void ControlledRobot::show()
	{
		this->shown = true;
		this->widget->show();
	}

	void ControlledRobot::hide()
	{
		this->shown = false;
		this->widget->hide();
	}

} /* namespace rqt_robot_control */
