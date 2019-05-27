#pragma once
#include "alica_msgs/AlicaEngineInfo.h"
#include "alica_msgs/AllocationAuthorityInfo.h"
#include "alica_msgs/PlanTreeInfo.h"
#include "alica_msgs/RoleSwitch.h"
#include "alica_msgs/SolverResult.h"
#include "alica_msgs/SyncReady.h"
#include "alica_msgs/SyncTalk.h"

#include <engine/IAlicaCommunication.h>
#include <essentials/Identifier.h>
#include <SystemConfig.h>

#include <ros/ros.h>
#include <string>

using namespace alica;

namespace alicaRosProxy
{

class AlicaRosCommunication : public alica::IAlicaCommunication
{
public:
    AlicaRosCommunication(AlicaEngine* ae);
    virtual ~AlicaRosCommunication();

    virtual void tick();

    virtual void sendAllocationAuthority(const AllocationAuthorityInfo& aai) const override;
    virtual void sendAlicaEngineInfo(const AlicaEngineInfo& bi) const override;
    virtual void sendPlanTreeInfo(const PlanTreeInfo& pti) const override;
    virtual void sendRoleSwitch(const RoleSwitch& rs) const override;
    virtual void sendSyncReady(const SyncReady& sr) const override;
    virtual void sendSyncTalk(const SyncTalk& st) const override;
    virtual void sendSolverResult(const SolverResult& sr) const override;
    virtual void sendLogMessage(int level, const std::string& message) const override;

    virtual void handleAllocationAuthorityRos(const alica_msgs::AllocationAuthorityInfo& aai);
    virtual void handlePlanTreeInfoRos(alica_msgs::PlanTreeInfoPtr pti);
    virtual void handleSyncReadyRos(alica_msgs::SyncReadyPtr sr);
    virtual void handleSyncTalkRos(alica_msgs::SyncTalkPtr st);
    virtual void handleSolverResult(const alica_msgs::SolverResult& sr);

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

    essentials::SystemConfig* sc;
};

} /* namespace alicaRosProxy */
