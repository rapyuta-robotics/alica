/*
 * AlicaWidget.cpp
 *
 *  Created on: Jul 22, 2015
 *      Author: Stephan Opfer
 */

#include "alica/AlicaWidget.h"
#include <sstream>

namespace alica
{
AlicaWidget::AlicaWidget()
    : qframe(new QFrame())
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
            int tmpId = 0;
            for (int j = 0; j < aei->robotIDsWithMe[i].id.size(); j++)
            {
                tmpId += (aei->robotIDsWithMe[i].id.at(j) << (j * 8));
            }
            ss << tmpId << ", ";
        }
        int tmpId = 0;
        for (int i = 0; i < aei->robotIDsWithMe[aei->robotIDsWithMe.size() - 1].id.size(); i++)
        {
            tmpId += (aei->robotIDsWithMe[aei->robotIDsWithMe.size() - 1].id.at(i) << (i * 8));
        }
        ss << tmpId;
    }
    ss << ")";

    uiAlicaWidget.stateVal->setText(QString(ss.str().c_str()));
}

void AlicaWidget::handleKickerStatInfo(msl_actuator_msgs::KickerStatInfoPtr kickStatInfo)
{
    uiAlicaWidget.kickVoltVal->setText(QString::number(kickStatInfo->capVoltage));
}

void AlicaWidget::handleSharedWorldInfo(msl_sensor_msgs::SharedWorldInfoPtr sharedWorldInfo)
{
    if (sharedWorldInfo->ballPossessionStatus == msl_sensor_msgs::SharedWorldInfo::HAVE_BALL)
    {
        uiAlicaWidget.ballPossStateVal->setText(QString("HaveBall"));
    }
    else if (sharedWorldInfo->ballPossessionStatus == msl_sensor_msgs::SharedWorldInfo::NO_BALL_SEEN)
    {
        uiAlicaWidget.ballPossStateVal->setText(QString("NoBallSeen"));
    }
    else if (sharedWorldInfo->ballPossessionStatus == msl_sensor_msgs::SharedWorldInfo::ASIDE_OF_KICKER)
    {
        uiAlicaWidget.ballPossStateVal->setText(QString("ASideOfKicker"));
    }
    else if (sharedWorldInfo->ballPossessionStatus == msl_sensor_msgs::SharedWorldInfo::NOT_IN_KICKER_DISTANCE)
    {
        uiAlicaWidget.ballPossStateVal->setText(QString("NotInKickerDistance"));
    }
    else if (sharedWorldInfo->ballPossessionStatus == msl_sensor_msgs::SharedWorldInfo::LIGHT_BARRIER_UNBLOCKED)
    {
        uiAlicaWidget.ballPossStateVal->setText(QString("LightBarrierUnblocked"));
    }
    else
    {
        uiAlicaWidget.ballPossStateVal->setText(QString("Unknown"));
    }
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
