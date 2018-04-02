
#include "alica/AlicaWidget.h"
#include <sstream>

namespace alica {
AlicaWidget::AlicaWidget() : qframe(new QFrame()) {
    this->uiAlicaWidget.setupUi(qframe);
}

AlicaWidget::~AlicaWidget() {
    delete qframe;
}

void AlicaWidget::handleAlicaEngineInfo(alica_msgs::AlicaEngineInfoConstPtr aei) {
    uiAlicaWidget.planVal->setText(QString(aei->currentPlan.c_str()));
    uiAlicaWidget.roleVal->setText(QString(aei->currentRole.c_str()));
    uiAlicaWidget.taskVal->setText(QString(aei->currentTask.c_str()));
    uiAlicaWidget.masterPlanVal->setText(QString(aei->masterPlan.c_str()));

    // TODO: refactor
    stringstream ss;
    ss << aei->currentState << " (";
    if (aei->robotIDsWithMe.size() > 0) {
        for (int i = 0; i < aei->robotIDsWithMe.size() - 1; i++) {
            int tmpId = 0;
            for (int j = 0; j < aei->robotIDsWithMe[i].id.size(); j++) {
                tmpId += (aei->robotIDsWithMe[i].id.at(j) << (j * 8));
            }
            ss << tmpId << ", ";
        }
        int tmpId = 0;
        for (int i = 0; i < aei->robotIDsWithMe[aei->robotIDsWithMe.size() - 1].id.size(); i++) {
            tmpId += (aei->robotIDsWithMe[aei->robotIDsWithMe.size() - 1].id.at(i) << (i * 8));
        }
        ss << tmpId;
    }
    ss << ")";

    uiAlicaWidget.stateVal->setText(QString(ss.str().c_str()));
}

void AlicaWidget::clearGUI() {
    if (uiAlicaWidget.planVal != nullptr) {
        uiAlicaWidget.planVal->setText(QString(""));
        uiAlicaWidget.roleVal->setText(QString(""));
        uiAlicaWidget.taskVal->setText(QString(""));
        uiAlicaWidget.masterPlanVal->setText(QString(""));
        uiAlicaWidget.stateVal->setText(QString(""));
    }
}
}  // namespace alica
