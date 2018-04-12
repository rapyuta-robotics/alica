#pragma once
#include "alica_msgs/AlicaEngineInfo.h"
#include "alica_msgs/AllocationAuthorityInfo.h"
#include "alica_msgs/PlanTreeInfo.h"
#include "alica_msgs/RoleSwitch.h"
#include "alica_msgs/SolverResult.h"
#include "alica_msgs/SyncReady.h"
#include "alica_msgs/SyncTalk.h"
#include "engine/IAlicaCommunication.h"
#include "supplementary/AgentID.h"

#include <ros/ros.h>
#include <string>

using namespace alica;

namespace supplementary {
class SystemConfig;
}

namespace alicaRosProxy {

class AlicaRosCommunication : public alica::IAlicaCommunication {
public:
    AlicaRosCommunication(AlicaEngine* ae);
    virtual ~AlicaRosCommunication();

    virtual void tick();

    virtual void sendAllocationAuthority(const AllocationAuthorityInfo& aai) const;
    virtual void sendAlicaEngineInfo(const AlicaEngineInfo& bi) const;
    virtual void sendPlanTreeInfo(const PlanTreeInfo& pti) const;
    virtual void sendRoleSwitch(const RoleSwitch& rs) const;
    virtual void sendSyncReady(const SyncReady& sr) const;
    virtual void sendSyncTalk(const yncTalk& st) const;
    virtual void sendSolverResult(const SolverResult& sr) const;
    virtual void sendLogMessage(int level, const std::string& message) const;

    virtual void handleAllocationAuthorityRos(alica_msgs::AllocationAuthorityInfoPtr aai);
    virtual void handlePlanTreeInfoRos(alica_msgs::PlanTreeInfoPtr pti);
    virtual void handleSyncReadyRos(alica_msgs::SyncReadyPtr sr);
    virtual void handleSyncTalkRos(alica_msgs::SyncTalkPtr st);
    virtual void handleSolverResult(alica_msgs::SolverResultPtr sr);

    virtual void startCommunication();
    virtual void stopCommunication();

protected:
    ros::NodeHandle* rosNode;
    ros::AsyncSpinner* spinner;

    ros::Publisher AlicaEngineInfoPublisher;
    ros::Publisher RoleSwitchPublisher;

    ros::Publisher AllocationAuthorityInfoPublisher;
    ros::Subscriber AllocationAuthorityInfoSubscriber;

    ros::Publisher PlanTreeInfoPublisher;
    ros::Subscriber PlanTreeInfoSubscriber;

    ros::Publisher SyncReadyPublisher;
    ros::Subscriber SyncReadySubscriber;

    ros::Publisher SyncTalkPublisher;
    ros::Subscriber SyncTalkSubscriber;

    ros::Publisher SolverResultPublisher;
    ros::Subscriber SolverResultSubscriber;

    std::string allocationAuthorityInfoTopic;
    std::string ownRoleTopic;
    std::string alicaEngineInfoTopic;
    std::string planTreeInfoTopic;
    std::string syncReadyTopic;
    std::string syncTalkTopic;
    std::string solverResultTopic;

    bool isRunning;

    supplementary::SystemConfig* sc;
};

} /* namespace alicaRosProxy */
