#pragma once

#include "pm_widget/ControlledRobot.h"

#include <ui_PMControl.h>
#include <process_manager/ProcessStats.h>
#include <process_manager/ProcessStat.h>
#include <essentials/AgentID.h>
#include <SystemConfig.h>

#include <QtGui>
#include <QWidget>
#include <QDialog>
#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>
#include <ros/macros.h>

#include <queue>
#include <mutex>
#include <utility>
#include <chrono>

namespace  essentials {
class RobotExecutableRegistry;
}  // namespace  essentials

namespace pm_widget {
class ControlledProcessManager;
}

namespace pm_control {
class PMControl : public rqt_gui_cpp::Plugin {
    Q_OBJECT

public:
    PMControl();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(
            const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

    void sendProcessCommand(const essentials::AgentID* receiverId,
            std::vector<const essentials::AgentID*> robotIds, std::vector<int> execIds, std::vector<int> paramSets,
            int cmd);

    std::chrono::duration<double> msgTimeOut;

    Ui::PMControlWidget ui_;
    QWidget* widget_;

     essentials::RobotExecutableRegistry* pmRegistry;

private:
    ros::NodeHandle* rosNode;
    ros::Subscriber processStateSub;
    ros::Publisher processCommandPub;
    std::queue<std::pair<std::chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr>>
            processStatMsgQueue;
    std::mutex msgQueueMutex;

    essentials::SystemConfig* sc;

    std::map<const essentials::AgentID*, pm_widget::ControlledProcessManager*, essentials::AgentIDComparator>
            processManagersMap;

    void handleProcessStats();

    void receiveProcessStats(process_manager::ProcessStatsConstPtr psts);
    pm_widget::ControlledProcessManager* getControlledProcessManager(const std::vector<uint8_t>& processManagerId);

    QTimer* guiUpdateTimer;

public Q_SLOTS:
    void run();
    void updateGUI();
};

}  // namespace pm_control
