#pragma once

#include <functional>
#include <memory>
#include <string>

namespace alica
{
struct AlicaEngineInfo;
struct AllocationAuthorityInfo;
struct PlanTreeInfo;
struct SolverResult;
struct SyncTalk;
struct SyncReady;
struct SyncReady;
struct AgentQuery;
struct AgentAnnouncement;
class AlicaEngine;
class RoleSwitch;

class IAlicaCommunication
{
public:
    IAlicaCommunication(std::function<void(std::shared_ptr<SyncTalk>)> onSyncTalkHandler, std::function<void(std::shared_ptr<SyncReady>)> onSyncReadyHandler,
            std::function<void(const AllocationAuthorityInfo&)> incomingAuthorityMessageHandler,
            std::function<void(std::shared_ptr<PlanTreeInfo>)> planTreeInfohandler, std::function<void(const SolverResult&)> onSolverResultHandler,
            std::function<void(const AgentQuery&)> agentQueryHandler, std::function<void(const AgentAnnouncement&)> agentAnnouncementHandler)
            : _onSyncTalkHandler(onSyncTalkHandler)
            , _onSyncReadyHandler(onSyncReadyHandler)
            , _incomingAuthorityMessageHandler(incomingAuthorityMessageHandler)
            , _planTreeInfohandler(planTreeInfohandler)
            , _onSolverResultHandler(onSolverResultHandler)
            , _agentQueryHandler(agentQueryHandler)
            , _agentAnnouncementHandler(agentAnnouncementHandler){};
    virtual ~IAlicaCommunication() {}

    virtual void sendAllocationAuthority(const AllocationAuthorityInfo& aai) const = 0;
    virtual void sendAlicaEngineInfo(const AlicaEngineInfo& bi) const = 0;
    virtual void sendPlanTreeInfo(const PlanTreeInfo& pti) const = 0;
    virtual void sendRoleSwitch(const RoleSwitch& rs) const = 0;
    virtual void sendSyncReady(const SyncReady& sr) const = 0;
    virtual void sendSyncTalk(const SyncTalk& st) const = 0;
    virtual void sendSolverResult(const SolverResult& sr) const = 0;
    virtual void sendAgentQuery(const AgentQuery& pq) const = 0;
    virtual void sendAgentAnnouncement(const AgentAnnouncement& pa) const = 0;
    virtual void sendLogMessage(int level, const std::string& message) const {};

    virtual void tick(){};

    void onSyncTalkReceived(std::shared_ptr<SyncTalk> st);
    void onSyncReadyReceived(std::shared_ptr<SyncReady> sr);
    void onAuthorityInfoReceived(const AllocationAuthorityInfo& aai);
    void onPlanTreeInfoReceived(std::shared_ptr<PlanTreeInfo> pti);
    void onSolverResult(const SolverResult& sr);
    void onAgentQuery(const AgentQuery& pq);
    void onAgentAnnouncement(const AgentAnnouncement& pa);

    virtual void startCommunication() = 0;
    virtual void stopCommunication() = 0;

protected:
    std::function<void(std::shared_ptr<SyncTalk>)> _onSyncTalkHandler;
    std::function<void(std::shared_ptr<SyncReady>)> _onSyncReadyHandler;
    std::function<void(const AllocationAuthorityInfo&)> _incomingAuthorityMessageHandler;
    std::function<void(std::shared_ptr<PlanTreeInfo>)> _planTreeInfohandler;
    std::function<void(const SolverResult&)> _onSolverResultHandler;
    std::function<void(const AgentQuery&)> _agentQueryHandler;
    std::function<void(const AgentAnnouncement&)> _agentAnnouncementHandler;
};

} /* namespace alica */
