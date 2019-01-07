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

    virtual void tick();

    virtual void sendAllocationAuthority(const alica::AllocationAuthorityInfo& aai) const;
    virtual void sendAlicaEngineInfo(const alica::AlicaEngineInfo& bi) const;
    virtual void sendPlanTreeInfo(const alica::PlanTreeInfo& pti) const;
    virtual void sendRoleSwitch(const alica::RoleSwitch& rs) const;
    virtual void sendSyncReady(const alica::SyncReady& sr) const;
    virtual void sendSyncTalk(const alica::SyncTalk& st) const;
    virtual void sendSolverResult(const alica::SolverResult& sr) const;

    virtual void startCommunication();
    virtual void stopCommunication();
    alica::AlicaEngine* getEngine() const { return ae; }

private:
    bool _isRunning;
    static CommModuleContainer s_modContainer;
    static Queues s_qctx;
};

} // namespace alicaDummyProxy
