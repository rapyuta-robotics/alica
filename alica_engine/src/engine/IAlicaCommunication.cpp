#include "engine/IAlicaCommunication.h"

#include <iostream>

using std::shared_ptr;

namespace alica
{
void IAlicaCommunication::onSyncTalkReceived(shared_ptr<SyncTalk> st)
{
    _onSyncTalkHandler(st);
}

void IAlicaCommunication::onSyncReadyReceived(shared_ptr<SyncReady> sr)
{
    _onSyncReadyHandler(sr);
}

void IAlicaCommunication::onAuthorityInfoReceived(const AllocationAuthorityInfo& aai)
{
    _incomingAuthorityMessageHandler(aai);
}

void IAlicaCommunication::onPlanTreeInfoReceived(shared_ptr<PlanTreeInfo> pti)
{
    _planTreeInfohandler(pti);
}

void IAlicaCommunication::onSolverResult(const SolverResult& sr)
{
    _onSolverResultHandler(sr);
}

void IAlicaCommunication::onAgentQuery(const AgentQuery& pq)
{
    _agentQueryHandler(pq);
}

void IAlicaCommunication::onAgentAnnouncement(const AgentAnnouncement& pa)
{
    _agentAnnouncementHandler(pa);
}
} // namespace alica