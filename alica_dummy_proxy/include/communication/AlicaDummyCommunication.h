/*
 * AlicaDummyCommunication.h
 *
 *  Created on: Mar 13, 2015
 *      Author: Paul Panin
 */

#pragma once

#include <engine/IAlicaCommunication.h>

namespace alicaDummyProxy
{

class CommModuleContainer;
struct Queues;

class AlicaDummyCommunication : public alica::IAlicaCommunication
{
public:
    AlicaDummyCommunication(alica::AlicaEngine* ae);
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
    alica::AlicaEngine* getEngine() const { return ae; }

private:
    bool _isRunning;
    static CommModuleContainer s_modContainer;
    static Queues s_qctx;
};

} // namespace alicaDummyProxy
