#pragma once

#include <QFrame>
#include <QHBoxLayout>
#include <QObject>
#include <chrono>
#include <process_manager/ProcessStat.h>
#include <process_manager/ProcessStats.h>
#include <process_manager/RobotMetaData.h>
#include <ros/ros.h>

namespace Ui
{
class RobotProcessesWidget;
}

namespace supplementary
{
class RobotExecutableRegistry;
class IAgentID;
}

namespace ros
{
class Publisher;
}

namespace pm_widget
{
class ControlledExecutable;

class ControlledRobot : public QObject, public supplementary::RobotMetaData
{
    Q_OBJECT

  public:
    ControlledRobot(std::string robotName, const supplementary::IAgentID *robotId,
                    const supplementary::IAgentID *parentPMid); /*<for robot_control*/
    virtual ~ControlledRobot();

    void handleProcessStat(std::chrono::system_clock::time_point timeMsgReceived, process_manager::ProcessStat ps,
                           const supplementary::IAgentID *parentPMid);
    void sendProcessCommand(std::vector<int> execIds, std::vector<int> paramSets, int cmd);
    void updateGUI(std::chrono::system_clock::time_point now);
    void addExec(QWidget *exec);
    void removeExec(QWidget *exec);

    std::chrono::system_clock::time_point
        timeLastMsgReceived;      /* < Time point, when the last message have been received */
    QFrame *robotProcessesQFrame; /**< The widget, used to initialise the RobotProcessesWidget */
                                  // ControlledProcessManager* parentProcessManager;

  public Q_SLOTS:
    void updateBundles(QString text);

  private:
    std::chrono::duration<double> msgTimeOut;
    bool inRobotControl;
    std::string selectedBundle;
    Ui::RobotProcessesWidget *_robotProcessesWidget;
    std::map<int, ControlledExecutable *> controlledExecMap;
    ros::Publisher processCommandPub;
    const supplementary::IAgentID *parentPMid;
};

} /* namespace pm_widget */
