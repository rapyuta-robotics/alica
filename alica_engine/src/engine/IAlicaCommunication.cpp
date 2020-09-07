#include "engine/IAlicaCommunication.h"
#include <engine/syncmodule/SyncModule.h>

#include "engine/AlicaEngine.h"
#include "engine/TeamObserver.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/constraintmodul/VariableSyncModule.h"
#include "engine/containers/PlanTreeInfo.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"
#include "engine/teammanager/TeamManager.h"

#include <iostream>

using std::shared_ptr;

namespace alica
{
void IAlicaCommunication::onSyncTalkReceived(shared_ptr<SyncTalk> st)
{
    ae->editSyncModul().onSyncTalk(st);
}

void IAlicaCommunication::onSyncReadyReceived(shared_ptr<SyncReady> sr)
{
    ae->editSyncModul().onSyncReady(sr);
}

void IAlicaCommunication::onAuthorityInfoReceived(const AllocationAuthorityInfo& aai)
{
    ae->editAuth().handleIncomingAuthorityMessage(aai);
}

void IAlicaCommunication::onPlanTreeInfoReceived(shared_ptr<PlanTreeInfo> pti)
{
    ae->editTeamObserver().handlePlanTreeInfo(pti);
}

void IAlicaCommunication::onSolverResult(const SolverResult& sr)
{
    ae->editResultStore().onSolverResult(sr);
}

void IAlicaCommunication::onAgentQuery(const AgentQuery& pq)
{
    ae->getTeamManager().handleAgentQuery(pq);
}

void IAlicaCommunication::onAgentAnnouncement(const AgentAnnouncement& pa)
{
    ae->editTeamManager().handleAgentAnnouncement(pa);
}
}