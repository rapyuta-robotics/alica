/*
 * ControlledExecutable.h
 *
 *  Created on: Mar 2, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_PM_CONTROL_SRC_PM_WIDGET_CONTROLLEDEXECUTABLE_H_
#define SUPPLEMENTARY_PM_CONTROL_SRC_PM_WIDGET_CONTROLLEDEXECUTABLE_H_

#include <process_manager/ProcessStat.h>
#include <qt5/QtWidgets/QWidget>

#include <string>
#include <vector>
#include <map>
#include <chrono>

namespace Ui {
class ProcessWidget;
class RobotProcessesWidget;
}  // namespace Ui

namespace supplementary {
class ExecutableMetaData;
class RobotExecutableRegistry;
}  // namespace supplementary

using namespace std;

namespace pm_widget {
class ControlledRobot;

class ControlledExecutable : public QObject {
    Q_OBJECT

public:
    ControlledExecutable(supplementary::ExecutableMetaData* metaExec, ControlledRobot* parentRobot);
    virtual ~ControlledExecutable();

    void handleStat(chrono::system_clock::time_point timeMsgReceived, process_manager::ProcessStat ps);
    void updateGUI(chrono::system_clock::time_point now);
    void handleBundleComboBoxChanged(QString bundle);
    void sendProcessCommand(int cmd);

    chrono::duration<double> msgTimeOut;
    chrono::system_clock::time_point timeLastMsgReceived; /**< last time a message was received for this executable */

    int runningParamSet;
    int desiredParamSet;
    char state; /**< The process state (zombie, running, etc.) */
    unsigned short cpu;
    long int memory;
    bool publishing;

    supplementary::ExecutableMetaData* metaExec;
    QWidget* processWidget;
    Ui::ProcessWidget* _processWidget;

public Q_SLOTS:
    void handleCheckBoxStateChanged(int newState);
    void showContextMenu(const QPoint& pos);

Q_SIGNALS:
    void processCheckBoxStateChanged(int, int); /**< first int is newState, second int is execId */

private:
    static const string redBackground;
    static const string greenBackground;
    static const string grayBackground;
    ControlledRobot* parentRobot;
    supplementary::RobotExecutableRegistry* pmRegistry;
};

} /* namespace pm_widget */

#endif /* SUPPLEMENTARY_PM_CONTROL_SRC_PM_WIDGET_CONTROLLEDEXECUTABLE_H_ */
