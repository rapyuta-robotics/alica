#ifndef ALICA_VIEWER_ALICA_VIEWER_ROS_INTERFACE_H
#define ALICA_VIEWER_ALICA_VIEWER_ROS_INTERFACE_H

#include <QThread>
#include <ros/ros.h>

#include <alica_msgs/AlicaEngineInfo.h>
#include <alica_msgs/PlanTreeInfo.h>
#include <engine/containers/AlicaEngineInfo.h>
#include <engine/containers/PlanTreeInfo.h>
#include <supplementary/AgentIDManager.h>

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

  private:
    void alicaEngineInfoCallback(const alica_msgs::AlicaEngineInfo& msg);
    void alicaPlanInfoCallback(const alica_msgs::PlanTreeInfo& msg);

    ros::Subscriber _alicaEngineInfoSub;
    ros::Subscriber _alicaPlanInfoSub;
    supplementary::AgentIDManager* _agent_id_manager;
};

} // namespace alica

// Declare templates that can be accepted by QVariant
Q_DECLARE_METATYPE(alica::AlicaEngineInfo);
Q_DECLARE_METATYPE(alica::PlanTreeInfo);

#endif // ALICA_VIEWER_ALICA_VIEWER_ROS_INTERFACE_H
