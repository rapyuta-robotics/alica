#pragma once

#include <engine/IAlicaCommunication.h>

namespace alicaDummyProxy
{

class CommModuleContainer;
struct Queues;

class AlicaDummyCommunication : public alica::IAlicaCommunication
{
public:
    AlicaDummyCommunication(const alica::AlicaCommunicationHandlers& callbacks);
    ~AlicaDummyCommunication() override;

    void tick() override;

    void sendAllocationAuthority(const alica::AllocationAuthorityInfo& aai) const override;
    void sendAlicaEngineInfo(const alica::AlicaEngineInfo& bi) const override;
    void sendPlanTreeInfo(const alica::PlanTreeInfo& pti) const override;
    void sendRoleSwitch(const alica::RoleSwitch& rs, alica::AgentId agentID) const override;
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
