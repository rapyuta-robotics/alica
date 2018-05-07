/*
 * AlicaDummyCommunication.h
 *
 *  Created on: Mar 13, 2015
 *      Author: Paul Panin
 */

#ifndef ALICADUMMYCOMMUNICATION_H_
#define ALICADUMMYCOMMUNICATION_H_

#include <engine/IAlicaCommunication.h>

namespace alica_dummy_proxy {

class AlicaDummyCommunication : public alica::IAlicaCommunication {
public:
    AlicaDummyCommunication(alica::AlicaEngine* ae);
    virtual ~AlicaDummyCommunication();

    virtual void tick();

    virtual void sendAllocationAuthority(alica::AllocationAuthorityInfo& aai) const;
    virtual void sendAlicaEngineInfo(alica::AlicaEngineInfo& bi) const;
    virtual void sendPlanTreeInfo(alica::PlanTreeInfo& pti) const;
    virtual void sendRoleSwitch(alica::RoleSwitch& rs) const;
    virtual void sendSyncReady(alica::SyncReady& sr) const;
    virtual void sendSyncTalk(alica::SyncTalk& st) const;
    virtual void sendSolverResult(alica::SolverResult& sr) const;

    virtual void startCommunication();
    virtual void stopCommunication();
};

}  // namespace alica_dummy_proxy

#endif /* ALICA_ALICA_DUMMY_PROXY_INCLUDE_COMMUNICATION_ALICADUMMYCOMMUNICATION_H_ */
