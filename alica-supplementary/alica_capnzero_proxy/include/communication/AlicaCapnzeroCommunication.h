#pragma once

#include <engine/IAlicaCommunication.h>

#include <SystemConfig.h>

#include <capnzero/CapnZero.h>

namespace alicaCapnzeroProxy
{
class AlicaCapnzeroCommunication : public alica::IAlicaCommunication
{
public:
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

    void handleAllocationAuthority(::capnp::FlatArrayMessageReader& msg);
    void handlePlanTreeInfo(::capnp::FlatArrayMessageReader& msg);
    void handleSyncReady(::capnp::FlatArrayMessageReader& msg);
    void handleSyncTalk(::capnp::FlatArrayMessageReader& msg);
    void handleSolverResult(::capnp::FlatArrayMessageReader& msg);

    virtual void startCommunication();
    virtual void stopCommunication();

protected:
    void* ctx;
    std::string url;
    essentials::SystemConfig* sc;

    // Topics to use:
    std::string allocationAuthorityInfoTopic;
    std::string ownRoleTopic;
    std::string alicaEngineInfoTopic;
    std::string planTreeInfoTopic;
    std::string syncReadyTopic;
    std::string syncTalkTopic;
    std::string solverResultTopic;

    // Publishers for the different Messages:
    capnzero::Publisher *AlicaPublisher;

    // Subscribers for the different Messages:
    capnzero::Subscriber *AlicaEngineInfoSubscriber;
    capnzero::Subscriber *RoleSwitchSubscriber;
    capnzero::Subscriber *AllocationAuthorityInfoSubscriber;
    capnzero::Subscriber *PlanTreeInfoSubscriber;
    capnzero::Subscriber *SyncReadySubscriber;
    capnzero::Subscriber *SyncTalkSubscriber;
    capnzero::Subscriber *SolverResultSubscriber;

    // Transport:
    capnzero::Protocol protocol;

    bool isRunning;
};
}