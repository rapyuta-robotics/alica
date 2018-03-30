#pragma once

#include <alica_msgs/AlicaEngineInfo.h>
#include <ui_AlicaWidget.h>
#include <QtGui>
namespace alica {
using namespace std;

class AlicaWidget {
public:
    AlicaWidget();
    virtual ~AlicaWidget();
    void handleAlicaEngineInfo(alica_msgs::AlicaEngineInfoConstPtr aei);
    void clearGUI();

    Ui::AlicaWidget uiAlicaWidget;
    QFrame* qframe;
};
}  // namespace alica
