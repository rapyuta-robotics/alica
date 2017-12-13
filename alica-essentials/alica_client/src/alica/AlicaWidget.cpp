#include "alica/AlicaWidget.h"
#include <sstream>

namespace alica
{
	AlicaWidget::AlicaWidget() : qframe(new QFrame())
	{
		this->uiAlicaWidget.setupUi(qframe);
	}

	AlicaWidget::~AlicaWidget()
	{
		delete qframe;
	}

	void AlicaWidget::handleAlicaEngineInfo(alica_ros_proxy::AlicaEngineInfoConstPtr aei)
	{
		uiAlicaWidget.planVal->setText(QString(aei->currentPlan.c_str()));
		uiAlicaWidget.roleVal->setText(QString(aei->currentRole.c_str()));
		uiAlicaWidget.taskVal->setText(QString(aei->currentTask.c_str()));
		uiAlicaWidget.masterPlanVal->setText(QString(aei->masterPlan.c_str()));

		stringstream ss;
		ss << aei->currentState << " (";
		if (aei->robotIDsWithMe.size() > 0)
		{
			for (int i = 0; i < aei->robotIDsWithMe.size() - 1; i++)
			{
				ss << aei->robotIDsWithMe[i] << ", ";
			}
			ss << aei->robotIDsWithMe[aei->robotIDsWithMe.size()-1];
		}
		ss << ")";

		uiAlicaWidget.stateVal->setText(QString(ss.str().c_str()));
	}

	void AlicaWidget::clearGUI()
	{
		if (uiAlicaWidget.planVal != nullptr)
		{
			uiAlicaWidget.planVal->setText(QString(""));
			uiAlicaWidget.roleVal->setText(QString(""));
			uiAlicaWidget.taskVal->setText(QString(""));
			uiAlicaWidget.masterPlanVal->setText(QString(""));
			uiAlicaWidget.stateVal->setText(QString(""));
		}
	}
}


