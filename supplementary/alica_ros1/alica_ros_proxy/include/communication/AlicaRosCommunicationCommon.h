#pragma once
#include "alica_msgs/AgentAnnouncement.h"
#include "alica_msgs/AgentQuery.h"
#include "alica_msgs/AlicaEngineInfo.h"
#include "alica_msgs/AllocationAuthorityInfo.h"
#include "alica_msgs/PlanTreeInfo.h"
#include "alica_msgs/RoleSwitch.h"
#include "alica_msgs/SolverResult.h"
#include "alica_msgs/SyncReady.h"
#include "alica_msgs/SyncTalk.h"

#include <engine/IAlicaCommunication.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <string>

using namespace alica;

namespace alicaRosProxy
{

class AlicaRosCommunicationCommon : public alica::IAlicaCommunication
{
public:
    void tick() override;

    void sendAllocationAuthority(const AllocationAuthorityInfo& aai) const override;
    void sendAlicaEngineInfo(const AlicaEngineInfo& bi) const override;
    void sendPlanTreeInfo(const PlanTreeInfo& pti) const override;
    void sendRoleSwitch(const RoleSwitch& rs, AgentId agentID) const override;
    void sendSyncReady(const SyncReady& sr) const override;
    void sendSyncTalk(const SyncTalk& st) const override;
    void sendSolverResult(const SolverResult& sr) const override;
    void sendAgentQuery(const AgentQuery& pq) const override;
    void sendAgentAnnouncement(const AgentAnnouncement& pa) const override;
    void sendLogMessage(int level, const std::string& message) const override;

    void startCommunication() override;
    void stopCommunication() override;

    void handleAllocationAuthorityRos(const alica_msgs::AllocationAuthorityInfo& aai);
    void handleSyncReadyRos(alica_msgs::SyncReadyPtr sr);
    void handleSyncTalkRos(alica_msgs::SyncTalkPtr st);
    void handleAgentQuery(const alica_msgs::AgentQuery& pq);
    void handleAgentAnnouncement(const alica_msgs::AgentAnnouncement& pa);
    void handlePlanTreeInfoRos(alica_msgs::PlanTreeInfoPtr pti);
    void handleSolverResult(const alica_msgs::SolverResult& sr);

protected:
    AlicaRosCommunicationCommon(const AlicaCommunicationHandlers& callbacks, ros::CallbackQueue& cb_queue = *ros::getGlobalCallbackQueue());
    virtual ~AlicaRosCommunicationCommon();

    ros::NodeHandle _nh;

private:
    ros::CallbackQueue& _callbackQueue;

    ros::Publisher _alicaEngineInfoPublisher;
    ros::Publisher _roleSwitchPublisher;
    ros::Publisher _allocationAuthorityInfoPublisher;
    ros::Publisher _planTreeInfoPublisher;
    ros::Publisher _syncReadyPublisher;
    ros::Publisher _syncTalkPublisher;
    ros::Publisher _solverResultPublisher;
    ros::Publisher _presenceQueryPublisher;
    ros::Publisher _presenceAnnouncementPublisher;

    bool _isRunning;
};

} /* namespace alicaRosProxy */
