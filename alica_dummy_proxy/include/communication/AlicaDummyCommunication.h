#pragma once

#include <engine/IAlicaCommunication.h>

namespace alicaDummyProxy
{

class CommModuleContainer;
struct Queues;

class AlicaDummyCommunication : public alica::IAlicaCommunication
{
public:
    AlicaDummyCommunication(std::function<void(std::shared_ptr<alica::SyncTalk>)> onSyncTalkHandler,
            std::function<void(std::shared_ptr<alica::SyncReady>)> onSyncReadyHandler,
            std::function<void(const alica::AllocationAuthorityInfo&)> incomingAuthorityMessageHandler,
            std::function<void(std::shared_ptr<alica::PlanTreeInfo>)> planTreeInfohandler,
            std::function<void(const alica::SolverResult&)> onSolverResultHandler, std::function<void(const alica::AgentQuery&)> agentQueryHandler,
            std::function<void(const alica::AgentAnnouncement&)> agentAnnouncementHandler);
    virtual ~AlicaDummyCommunication();

    void tick() override;

    void sendAllocationAuthority(const alica::AllocationAuthorityInfo& aai) const override;
    void sendAlicaEngineInfo(const alica::AlicaEngineInfo& bi) const override;
    void sendPlanTreeInfo(const alica::PlanTreeInfo& pti) const override;
    void sendRoleSwitch(const alica::RoleSwitch& rs) const override;
    void sendSyncReady(const alica::SyncReady& sr) const override;
    void sendSyncTalk(const alica::SyncTalk& st) const override;
    void sendSolverResult(const alica::SolverResult& sr) const override;
    void sendAgentQuery(const alica::AgentQuery& aq) const override;
    void sendAgentAnnouncement(const alica::AgentAnnouncement& aa) const override;

    void startCommunication() override;
    void stopCommunication() override;

private:
    bool _isRunning;
    static CommModuleContainer s_modContainer;
    static Queues s_qctx;
};

} // namespace alicaDummyProxy
