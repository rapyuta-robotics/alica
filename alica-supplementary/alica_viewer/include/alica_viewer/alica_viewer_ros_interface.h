#pragma once

#include <QThread>
#include <ros/ros.h>

#include <alica_msgs/AlicaEngineInfo.h>
#include <alica_msgs/PlanTreeInfo.h>
#include <engine/containers/AlicaEngineInfo.h>
#include <engine/containers/PlanTreeInfo.h>
#include <essentials/IDManager.h>

namespace alica
{

class AlicaViewerRosInterface : public QThread
{
    Q_OBJECT

  public:
    AlicaViewerRosInterface(int argc, char* argv[]);
    ~AlicaViewerRosInterface();
    void run() override;

  Q_SIGNALS:
    void shutdown();
    void alicaEngineInfoUpdate(const alica::AlicaEngineInfo& msg);
    void alicaPlanInfoUpdate(const alica::PlanTreeInfo& msg);
    void updateTicks();

  private:
    void alicaEngineInfoCallback(const alica_msgs::AlicaEngineInfo& msg);
    void alicaPlanInfoCallback(const alica_msgs::PlanTreeInfo& msg);
    void timerCallback(const ros::TimerEvent& event);
    ros::Subscriber _alicaEngineInfoSub;
    ros::Subscriber _alicaPlanInfoSub;
    ros::Timer _timer;
    essentials::IDManager* _agent_id_manager;
};

} // namespace alica

// Declare templates that can be accepted by QVariant
Q_DECLARE_METATYPE(alica::AlicaEngineInfo);
Q_DECLARE_METATYPE(alica::PlanTreeInfo);