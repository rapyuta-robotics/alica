#include <process_manager/ProcessStats.h>
#include <process_manager/ProcessCommand.h>

#include <ros/ros.h>
#include <QFrame>
#include <QBoxLayout>
#include <string>
#include <utility>
#include <chrono>

#include <essentials/AgentID.h>

namespace  essentials {
class SystemConfig;
class RobotExecutableRegistry;
}  // namespace  essentials

namespace pm_control {
class PMControl;
}

namespace pm_widget {
class ControlledRobot;

class ControlledProcessManager : public QObject {
    Q_OBJECT

public:
    ControlledProcessManager(std::string processManagerName, const essentials::AgentID* processManagerId,
            QBoxLayout* pmHorizontalLayout);
    virtual ~ControlledProcessManager();

    void updateGUI(std::chrono::system_clock::time_point now);
    void handleProcessStats(
            std::pair<std::chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr> timePstsPair);
    void addRobot(QFrame* robot);
    void removeRobot(QFrame* robot);

    void hide();
    void show();

    std::chrono::duration<double> msgTimeOut;
    std::chrono::system_clock::time_point
            timeLastMsgReceived;      /* < Time point, when the last message have been received */
    std::string name;                 /* < Hostname under which this process manager is running */
    const essentials::AgentID* id; /* < The id of the host */
     essentials::RobotExecutableRegistry* pmRegistry;

private:
    std::map<const essentials::AgentID*, ControlledRobot*, essentials::AgentIDComparator>
            controlledRobotsMap; /* < The robots, which are controlled by this process manager */
    QBoxLayout* parentLayout;
    ControlledRobot* getControlledRobot(const essentials::AgentID* robotId);
};

} /* namespace pm_widget */
