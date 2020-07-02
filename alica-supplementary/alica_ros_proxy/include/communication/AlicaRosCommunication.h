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
#include "engine/IAlicaCommunication.h"
#include "essentials/AgentID.h"
#include <SystemConfig.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <string>

using namespace alica;

namespace alicaRosProxy
{

class AlicaRosCommunication : public alica::IAlicaCommunication
{
public:
    AlicaRosCommunication(AlicaEngine* ae);
    AlicaRosCommunication(AlicaEngine* ae, uint16_t threadCount, bool privateQueue = false);
    virtual ~AlicaRosCommunication();

    void tick();

    void sendAllocationAuthority(const AllocationAuthorityInfo& aai) const override;
    void sendAlicaEngineInfo(const AlicaEngineInfo& bi) const override;
    void sendPlanTreeInfo(const PlanTreeInfo& pti) const override;
    void sendRoleSwitch(const RoleSwitch& rs) const override;
    void sendSyncReady(const SyncReady& sr) const override;
    void sendSyncTalk(const SyncTalk& st) const override;
    void sendSolverResult(const SolverResult& sr) const override;
    void sendAgentQuery(const AgentQuery& pq) const override;
    void sendAgentAnnouncement(const AgentAnnouncement& pa) const override;
    void sendLogMessage(int level, const std::string& message) const override;

    void handleAllocationAuthorityRos(const alica_msgs::AllocationAuthorityInfo& aai);
    void handlePlanTreeInfoRos(alica_msgs::PlanTreeInfoPtr pti);
    void handleSyncReadyRos(alica_msgs::SyncReadyPtr sr);
    void handleSyncTalkRos(alica_msgs::SyncTalkPtr st);
    void handleSolverResult(const alica_msgs::SolverResult& sr);
    void handleAgentQuery(const alica_msgs::AgentQuery& pq);
    void handleAgentAnnouncement(const alica_msgs::AgentAnnouncement& pa);

    void startCommunication() override;
    void stopCommunication() override;

private:
    ros::NodeHandle* _rosNode;
    ros::AsyncSpinner* _spinner;
    ros::CallbackQueue* _callbackQueue;

    ros::Publisher _alicaEngineInfoPublisher;
    ros::Publisher _roleSwitchPublisher;

    ros::Publisher _allocationAuthorityInfoPublisher;
    ros::Subscriber _allocationAuthorityInfoSubscriber;

    ros::Publisher _planTreeInfoPublisher;
    ros::Subscriber _planTreeInfoSubscriber;

    ros::Publisher _syncReadyPublisher;
    ros::Subscriber _syncReadySubscriber;

    ros::Publisher _syncTalkPublisher;
    ros::Subscriber _syncTalkSubscriber;

    ros::Publisher _solverResultPublisher;
    ros::Subscriber _solverResultSubscriber;

    ros::Publisher _presenceQueryPublisher;
    ros::Subscriber _presenceQuerySubscriber;

    ros::Publisher _presenceAnnouncementPublisher;
    ros::Subscriber _presenceAnnouncementSubscriber;

    bool _isRunning;
};

} /* namespace alicaRosProxy */
