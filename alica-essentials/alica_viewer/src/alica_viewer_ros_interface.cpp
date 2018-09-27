#include "alica_viewer/alica_viewer_ros_interface.h"

namespace alica
{

AlicaViewerRosInterface::AlicaViewerRosInterface(int argc, char* argv[])
    : _agent_id_manager(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()))
{
    ros::init(argc, argv, "alica_viewer");
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ROS_INFO("Started Alica Viewer Node.");
    ros::NodeHandle nh;
    _alicaEngineInfoSub = nh.subscribe("/AlicaEngine/AlicaEngineInfo", 10, &AlicaViewerRosInterface::alicaEngineInfoCallback, this);
    _alicaPlanInfoSub = nh.subscribe("/AlicaEngine/PlanTreeInfo", 10, &AlicaViewerRosInterface::alicaPlanInfoCallback, this);

    // Register custom structs in qt so that they can be used in slot and signal queues
    // This structure should have been previously declared as Q_DECLARE_METATYPE
    qRegisterMetaType<alica::AlicaEngineInfo>();
    qRegisterMetaType<alica::PlanTreeInfo>();

    // start the QThread, defined in base class
    start();
    _timer = nh.createTimer(ros::Duration(0.05f), &AlicaViewerRosInterface::timerCallback, this);
}

AlicaViewerRosInterface::~AlicaViewerRosInterface()
{
    if (ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait(); // defined in QThread base class
}

/* This function is run by QThread*/
void AlicaViewerRosInterface::run()
{
    ros::spin();
    Q_EMIT shutdown();
}

void AlicaViewerRosInterface::alicaEngineInfoCallback(const alica_msgs::AlicaEngineInfo& msg)
{
    AlicaEngineInfo aei;
    aei.senderID = _agent_id_manager->getIDFromBytes(msg.senderID.id);
    aei.masterPlan = msg.masterPlan;
    aei.currentPlan = msg.currentPlan;
    aei.currentState = msg.currentState;
    aei.currentRole = msg.currentRole;
    aei.currentTask = msg.currentTask;
    for (const auto& robotID : msg.robotIDsWithMe) {
        aei.robotIDsWithMe.push_back(_agent_id_manager->getIDFromBytes(robotID.id));
    }
    Q_EMIT alicaEngineInfoUpdate(aei);
}

void AlicaViewerRosInterface::alicaPlanInfoCallback(const alica_msgs::PlanTreeInfo& msg)
{
    PlanTreeInfo pti;
    pti.senderID = _agent_id_manager->getIDFromBytes(msg.senderID.id);
    for (int64_t i : msg.stateIDs) {
        pti.stateIDs.push_back(i);
    }
    for (int64_t i : msg.succeededEps) {
        pti.succeededEPs.push_back(i);
    }
    Q_EMIT alicaPlanInfoUpdate(pti);
}

void AlicaViewerRosInterface::timerCallback(const ros::TimerEvent& event)
{
    Q_EMIT updateTicks();
}

} // namespace alica