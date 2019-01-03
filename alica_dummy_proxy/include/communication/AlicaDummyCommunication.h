/*
 * AlicaDummyCommunication.h
 *
 *  Created on: Mar 13, 2015
 *      Author: Paul Panin
 */

#ifndef ALICADUMMYCOMMUNICATION_H_
#define ALICADUMMYCOMMUNICATION_H_

#include <engine/IAlicaCommunication.h>

namespace alicaDummyProxy {

class CommModuleContainer;
struct Queues;

class AlicaDummyCommunication : public alica::IAlicaCommunication {
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
    alica::AlicaEngine* getEngine() { return ae; }
private:
    bool isRunning;
    static CommModuleContainer modContainer;
    static Queues qctx;
};

}  // namespace alicaDummyProxy

#endif /* ALICA_ALICA_DUMMY_PROXY_INCLUDE_COMMUNICATION_ALICADUMMYCOMMUNICATION_H_ */
