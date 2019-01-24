#include "engine/IAlicaCommunication.h"
#include <engine/syncmodule/SyncModule.h>

#include "engine/AlicaEngine.h"
#include "engine/IAlicaCommunication.h"
#include "engine/TeamObserver.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/constraintmodul/VariableSyncModule.h"

#include <iostream>

using std::shared_ptr;

void alica::IAlicaCommunication::onSyncTalkReceived(shared_ptr<SyncTalk> st)
{
    ae->getSyncModul().onSyncTalk(st);
}

void alica::IAlicaCommunication::onSyncReadyReceived(shared_ptr<SyncReady> sr)
{
    ae->getSyncModul().onSyncReady(sr);
}

void alica::IAlicaCommunication::onAuthorityInfoReceived(const AllocationAuthorityInfo& aai)
{
    ae->getAuth().handleIncomingAuthorityMessage(aai);
}

void alica::IAlicaCommunication::onPlanTreeInfoReceived(shared_ptr<PlanTreeInfo> pti)
{
    ae->getTeamObserver().handlePlanTreeInfo(pti);
}

void alica::IAlicaCommunication::onSolverResult(const SolverResult& sr)
{
    ae->getResultStore().onSolverResult(sr);
}
