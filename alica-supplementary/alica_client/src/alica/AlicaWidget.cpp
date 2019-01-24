
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

void AlicaWidget::handleAlicaEngineInfo(alica_msgs::AlicaEngineInfoConstPtr aei)
{
    uiAlicaWidget.planVal->setText(QString(aei->current_plan.c_str()));
    uiAlicaWidget.roleVal->setText(QString(aei->current_role.c_str()));
    uiAlicaWidget.taskVal->setText(QString(aei->current_task.c_str()));
    uiAlicaWidget.masterPlanVal->setText(QString(aei->master_plan.c_str()));

    // TODO: refactor
    stringstream ss;
    ss << aei->current_state << " (";
    if (aei->robot_ids_with_me.size() > 0) {
        for (int i = 0; i < static_cast<int>(aei->robot_ids_with_me.size()) - 1; i++) {
            int tmpId = 0;
            for (int j = 0; j < static_cast<int>(aei->robot_ids_with_me[i].id.size()); j++) {
                tmpId += (aei->robot_ids_with_me[i].id.at(j) << (j * 8));
            }
            ss << tmpId << ", ";
        }
        int tmpId = 0;
        for (int i = 0; i < static_cast<int>(aei->robot_ids_with_me[aei->robot_ids_with_me.size() - 1].id.size()); i++) {
            tmpId += (aei->robot_ids_with_me[aei->robot_ids_with_me.size() - 1].id.at(i) << (i * 8));
        }
        ss << tmpId;
    }
    ss << ")";

    uiAlicaWidget.stateVal->setText(QString(ss.str().c_str()));
}

void AlicaWidget::clearGUI()
{
    if (uiAlicaWidget.planVal != nullptr) {
        uiAlicaWidget.planVal->setText(QString(""));
        uiAlicaWidget.roleVal->setText(QString(""));
        uiAlicaWidget.taskVal->setText(QString(""));
        uiAlicaWidget.masterPlanVal->setText(QString(""));
        uiAlicaWidget.stateVal->setText(QString(""));
    }
}
} // namespace alica
