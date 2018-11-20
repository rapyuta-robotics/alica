#pragma once

#include <rqt_gui_cpp/plugin.h>

#include <robot_control/Robot.h>
#include <ros/macros.h>
#include <ros/ros.h>
#include <supplementary/AgentID.h>

#include <alica_msgs/AlicaEngineInfo.h>
#include <process_manager/ProcessStats.h>

#include <QDialog>
#include <QWidget>
#include <QtGui>
#include <ui_RobotsControl.h>

#include <chrono>
#include <mutex>
#include <queue>
#include <utility>

namespace supplementary
{
class SystemConfig;
class RobotExecutableRegistry;
class AgentIDFactory;
} // namespace supplementary

namespace robot_control
{

class RobotsControl : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

  public:
    RobotsControl();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

    void addRobot();
    void removeRobot();

    static std::chrono::duration<double> msgTimeOut;

    Ui::RobotControlWidget robotControlWidget_;
    QWidget* widget_;

    std::map<std::string, std::vector<std::pair<int, int>>> bundlesMap;
    supplementary::RobotExecutableRegistry* pmRegistry;
    ros::NodeHandle* rosNode;

  private:
    ros::Subscriber processStateSub;
    ros::Subscriber alicaInfoSub;

    supplementary::SystemConfig* sc;

    std::map<const supplementary::AgentID*, Robot*, supplementary::AgentIDComparator> controlledRobotsMap;
    std::queue<std::pair<std::chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr>> processStatMsgQueue;
    std::mutex processStatsMsgQueueMutex;
    std::queue<std::pair<std::chrono::system_clock::time_point, alica_msgs::AlicaEngineInfoConstPtr>> alicaInfoMsgQueue;
    std::mutex alicaInfoMsgQueueMutex;

    void receiveProcessStats(process_manager::ProcessStatsConstPtr processStats);
    void receiveAlicaInfo(alica_msgs::AlicaEngineInfoConstPtr alicaInfo);
    void processMessages();
    void checkAndInit(const supplementary::AgentID* robotId);

    QTimer* guiUpdateTimer;

  public Q_SLOTS:
    void run();
    void updateGUI();
    void showContextMenu(const QPoint& pos);
};

} // namespace robot_control
