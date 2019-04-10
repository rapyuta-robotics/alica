#pragma once

#include <engine/IAlicaCommunication.h>
#include <capnzero/CapnZero.h>

// Alica messages includes. They will be replaced with capnproto messages!
#include "alica_msgs/AlicaEngineInfo.h"
#include "alica_msgs/AllocationAuthorityInfo.h"
#include "alica_msgs/PlanTreeInfo.h"
#include "alica_msgs/RoleSwitch.h"
#include "alica_msgs/SolverResult.h"
#include "alica_msgs/SyncReady.h"
#include "alica_msgs/SyncTalk.h"

namespace alicaCapnzeroProxy
{
class AlicaCapnzeroCommunication : public alica::IAlicaCommunication
{
    AlicaCapnzeroCommunication(alica::AlicaEngine *ae);
    virtual ~AlicaCapnzeroCommunication();

    virtual void sendAllocationAuthority(const alica::AllocationAuthorityInfo& aai) const override;
    virtual void sendAlicaEngineInfo(const alica::AlicaEngineInfo& bi) const override;
    virtual void sendPlanTreeInfo(const alica::PlanTreeInfo& pti) const override;
    virtual void sendRoleSwitch(const alica::RoleSwitch& rs) const override;
    virtual void sendSyncReady(const alica::SyncReady& sr) const override;
    virtual void sendSyncTalk(const alica::SyncTalk& st) const override;
    virtual void sendSolverResult(const alica::SolverResult& sr) const override;
    virtual void sendLogMessage(int level, const std::string& message) const override;

    void handleAllocationAuthority(const alica_msgs::AllocationAuthorityInfo& aai);
    void handlePlanTreeInfo(alica_msgs::PlanTreeInfoPtr pti);
    void handleSyncReady(alica_msgs::SyncReadyPtr sr);
    void handleSyncTalk(alica_msgs::SyncTalkPtr st);
    void handleSolverResult(const alica_msgs::SolverResult& sr);

    virtual void startCommunication();
    virtual void stopCommunication();

protected:
    void* ctx;
    std::string url;

    // Topics to use:
    std::string allocationAuthorityInfoTopic;
    std::string ownRoleTopic;
    std::string alicaEngineInfoTopic;
    std::string planTreeInfoTopic;
    std::string syncReadyTopic;
    std::string syncTalkTopic;
    std::string solverResultTopic;

    // Publishers for the different Messages:
    capnzero::Publisher *AlicaEngineInfoPublisher;
    capnzero::Publisher *RoleSwitchPublisher;
    capnzero::Publisher *AllocationAuthorityInfoPublisher;
    capnzero::Publisher *PlanTreeInfoPublisher;
    capnzero::Publisher *SyncReadyPublisher;
    capnzero::Publisher *SyncTalkPublisher;
    capnzero::Publisher *SolverResultPublisher;

    // Subscribers for the different Messages:
    capnzero::Subscriber *AlicaEngineInfoSubscriber;
    capnzero::Subscriber *RoleSwitchSubscriber;
    capnzero::Subscriber *AllocationAuthorityInfoSubscriber;
    capnzero::Subscriber *PlanTreeInfoSubscriber;
    capnzero::Subscriber *SyncReadySubscriber;
    capnzero::Subscriber *SyncTalkSubscriber;
    capnzero::Subscriber *SolverResultSubscriber;

    bool isRunning;
};
}