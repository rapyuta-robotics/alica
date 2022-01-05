#include "engine/IAlicaCommunication.h"

#include <iostream>

using std::shared_ptr;

namespace alica
{
void IAlicaCommunication::onSyncTalkReceived(shared_ptr<SyncTalk> st)
{
    _callbacks->onSyncTalkHandler(st);
}

void IAlicaCommunication::onSyncReadyReceived(shared_ptr<SyncReady> sr)
{
    _callbacks->onSyncReadyHandler(sr);
}

void IAlicaCommunication::onAuthorityInfoReceived(const AllocationAuthorityInfo& aai)
{
    _callbacks->incomingAuthorityMessageHandler(aai);
}

void IAlicaCommunication::onPlanTreeInfoReceived(shared_ptr<PlanTreeInfo> pti)
{
    _callbacks->planTreeInfohandler(pti);
}

void IAlicaCommunication::onSolverResult(const SolverResult& sr)
{
    _callbacks->onSolverResultHandler(sr);
}

void IAlicaCommunication::onAgentQuery(const AgentQuery& pq)
{
    _callbacks->agentQueryHandler(pq);
}

void IAlicaCommunication::onAgentAnnouncement(const AgentAnnouncement& pa)
{
    _callbacks->agentAnnouncementHandler(pa);
}
} // namespace alica
