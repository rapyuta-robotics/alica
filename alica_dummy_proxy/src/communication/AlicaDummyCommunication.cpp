#include "communication/AlicaDummyCommunication.h"

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include "engine/AlicaEngine.h"
#include "engine/containers/AgentAnnouncement.h"
#include "engine/containers/AgentQuery.h"
#include "engine/containers/AllocationAuthorityInfo.h"
#include "engine/containers/PlanTreeInfo.h"
#include "engine/containers/SolverResult.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"

namespace alicaDummyProxy
{

class CommModuleContainer
{
public:
    void registerModule(AlicaDummyCommunication* newModule)
    {
        std::lock_guard<std::mutex> lock(_cbMtx);
        _registeredModules.push_back(newModule);
    }

    void deregisterModule(AlicaDummyCommunication* oldModule)
    {
        std::lock_guard<std::mutex> lock(_cbMtx);
        _registeredModules.erase(std::remove(_registeredModules.begin(), _registeredModules.end(), oldModule), _registeredModules.end());
    }

    std::vector<AlicaDummyCommunication*> getModules()
    {
        std::lock_guard<std::mutex> lock(_cbMtx);
        // duplication to ensure thread safety
        return _registeredModules;
    }

private:
    std::mutex _cbMtx;
    std::vector<AlicaDummyCommunication*> _registeredModules;
};

template <class T>
class InProcQueue
{
public:
    InProcQueue(CommModuleContainer& cModules)
            : _isRunning(true)
            , _commModules(cModules)
    {
        _cbThread = new std::thread(&InProcQueue::run, this);
    }

    ~InProcQueue()
    {
        _isRunning = false;
        _qcondition.notify_one();
        _cbThread->join();
        delete _cbThread;
    }

    void push(const T& data)
    {
        {
            std::lock_guard<std::mutex> lock(_qmutex);
            _q.push(std::make_unique<T>(data));
        }
        _qcondition.notify_one();
    }

private:
    void run()
    {
        while (_isRunning) {
            std::unique_ptr<T> first;
            {
                std::unique_lock<std::mutex> lck(_qmutex);
                _qcondition.wait(lck, [&] { return !_isRunning || !_q.empty(); });
                if (!_isRunning) {
                    return;
                }

                first = std::move(_q.front());
                _q.pop();
            }
            process(first);
        }
    }

    void process(std::unique_ptr<T>& first);
    std::thread* _cbThread;
    std::mutex _qmutex;
    std::condition_variable _qcondition;
    std::queue<std::unique_ptr<T>> _q;
    bool _isRunning;
    CommModuleContainer& _commModules;
};

template <>
void InProcQueue<alica::AllocationAuthorityInfo>::process(std::unique_ptr<alica::AllocationAuthorityInfo>& first)
{
    alica::AllocationAuthorityInfo newInfo(*first);
    std::vector<AlicaDummyCommunication*> registeredModules = _commModules.getModules();
    for (AlicaDummyCommunication* module : registeredModules) {
        newInfo.senderID = module->getEngine()->getIDFromBytes(first->senderID->getRaw(), first->senderID->getSize(), first->senderID->getType());
        newInfo.authority = module->getEngine()->getIDFromBytes(first->authority->getRaw(), first->authority->getSize(), first->authority->getType());
        for (size_t i = 0; i < newInfo.entryPointRobots.size(); ++i) {
            for (size_t j = 0; j < newInfo.entryPointRobots[i].robots.size(); ++j) {
                newInfo.entryPointRobots[i].robots[j] = module->getEngine()->getIDFromBytes(first->entryPointRobots[i].robots[j]->getRaw(),
                        first->entryPointRobots[i].robots[j]->getSize(), first->entryPointRobots[i].robots[j]->getType());
            }
        }
        module->onAuthorityInfoReceived(newInfo);
    }
}

template <>
void InProcQueue<alica::PlanTreeInfo>::process(std::unique_ptr<alica::PlanTreeInfo>& first)
{
    std::vector<AlicaDummyCommunication*> registeredModules = _commModules.getModules();
    for (AlicaDummyCommunication* module : registeredModules) {
        // Each module expects ownership of shared_ptr
        auto pti = std::make_shared<alica::PlanTreeInfo>((*first));
        pti->senderID = module->getEngine()->getIDFromBytes(first->senderID->getRaw(), first->senderID->getSize(), first->senderID->getType());
        module->onPlanTreeInfoReceived(pti);
    }
}

template <>
void InProcQueue<alica::SyncTalk>::process(std::unique_ptr<alica::SyncTalk>& first)
{
    std::vector<AlicaDummyCommunication*> registeredModules = _commModules.getModules();
    for (AlicaDummyCommunication* module : registeredModules) {
        // Each module expects ownership of shared_ptr
        auto newSt = std::make_shared<alica::SyncTalk>((*first));
        newSt->senderID = module->getEngine()->getIDFromBytes(first->senderID->getRaw(), first->senderID->getSize(), first->senderID->getType());
        for (size_t i = 0; i < newSt->syncData.size(); ++i) {
            newSt->syncData[i].agentID = module->getEngine()->getIDFromBytes(
                    first->syncData[i].agentID->getRaw(), first->syncData[i].agentID->getSize(), first->syncData[i].agentID->getType());
        }
        module->onSyncTalkReceived(newSt);
    }
}

template <>
void InProcQueue<alica::SyncReady>::process(std::unique_ptr<alica::SyncReady>& first)
{
    std::vector<AlicaDummyCommunication*> registeredModules = _commModules.getModules();
    for (AlicaDummyCommunication* module : registeredModules) {
        // Each module expects ownership of shared_ptr
        auto newSr = std::make_shared<alica::SyncReady>((*first));
        newSr->senderID = module->getEngine()->getIDFromBytes(first->senderID->getRaw(), first->senderID->getSize(), first->senderID->getType());
        module->onSyncReadyReceived(newSr);
    }
}

template <>
void InProcQueue<alica::SolverResult>::process(std::unique_ptr<alica::SolverResult>& first)
{
    std::vector<AlicaDummyCommunication*> registeredModules = _commModules.getModules();
    for (AlicaDummyCommunication* module : registeredModules) {
        first->senderID = module->getEngine()->getIDFromBytes(first->senderID->getRaw(), first->senderID->getSize(), first->senderID->getType());
        module->onSolverResult(*first);
    }
}

template <>
void InProcQueue<alica::AgentAnnouncement>::process(std::unique_ptr<alica::AgentAnnouncement>& first)
{
    std::vector<AlicaDummyCommunication*> registeredModules = _commModules.getModules();
    uint64_t prev = first->senderID;
    for (AlicaDummyCommunication* module : registeredModules) {
        first->senderID = module->getEngine()->getIDFromBytes(prev->getRaw(), prev->getSize(), prev->getType());
        module->onAgentAnnouncement(*first);
    }
}

template <>
void InProcQueue<alica::AgentQuery>::process(std::unique_ptr<alica::AgentQuery>& first)
{
    std::vector<AlicaDummyCommunication*> registeredModules = _commModules.getModules();
    uint64_t prev = first->senderID;
    for (AlicaDummyCommunication* module : registeredModules) {
        first->senderID = module->getEngine()->getIDFromBytes(prev->getRaw(), prev->getSize(), prev->getType());
        module->onAgentQuery(*first);
    }
}

typedef struct Queues
{
    InProcQueue<alica::AllocationAuthorityInfo> aaq;
    InProcQueue<alica::PlanTreeInfo> ptq;
    InProcQueue<alica::SyncTalk> stq;
    InProcQueue<alica::SyncReady> srq;
    InProcQueue<alica::SolverResult> soq;
    InProcQueue<alica::AgentAnnouncement> anq;
    InProcQueue<alica::AgentQuery> aqq;

    Queues(CommModuleContainer& mContainer)
            : aaq(mContainer)
            , ptq(mContainer)
            , stq(mContainer)
            , srq(mContainer)
            , soq(mContainer)
            , anq(mContainer)
            , aqq(mContainer)
    {
    }
} Queues;

CommModuleContainer AlicaDummyCommunication::s_modContainer;
Queues AlicaDummyCommunication::s_qctx(s_modContainer);

AlicaDummyCommunication::AlicaDummyCommunication(alica::AlicaEngine* ae)
        : alica::IAlicaCommunication(ae)
        , _isRunning(false)
{
    s_modContainer.registerModule(this);
}

AlicaDummyCommunication::~AlicaDummyCommunication()
{
    s_modContainer.deregisterModule(this);
}

void AlicaDummyCommunication::sendAllocationAuthority(const alica::AllocationAuthorityInfo& aai) const
{
    if (!_isRunning) {
        return;
    }
    s_qctx.aaq.push(aai);
}

void AlicaDummyCommunication::sendPlanTreeInfo(const alica::PlanTreeInfo& pti) const
{
    if (!_isRunning) {
        return;
    }
    s_qctx.ptq.push(pti);
}

void AlicaDummyCommunication::sendSyncReady(const alica::SyncReady& sr) const
{
    if (!_isRunning) {
        return;
    }
    s_qctx.srq.push(sr);
}

void AlicaDummyCommunication::sendSyncTalk(const alica::SyncTalk& st) const
{
    if (!_isRunning) {
        return;
    }
    s_qctx.stq.push(st);
}

void AlicaDummyCommunication::sendSolverResult(const alica::SolverResult& sr) const
{
    if (!_isRunning) {
        return;
    }
    s_qctx.soq.push(sr);
}

void AlicaDummyCommunication::sendAgentQuery(const alica::AgentQuery& aq) const
{
    if (!_isRunning) {
        return;
    }
    s_qctx.aqq.push(aq);
}

void AlicaDummyCommunication::sendAgentAnnouncement(const alica::AgentAnnouncement& aa) const
{
    if (!_isRunning) {
        return;
    }
    s_qctx.anq.push(aa);
}

void AlicaDummyCommunication::tick() {}
void AlicaDummyCommunication::startCommunication()
{
    _isRunning = true;
}

void AlicaDummyCommunication::stopCommunication()
{
    _isRunning = false;
}

void AlicaDummyCommunication::sendAlicaEngineInfo(const alica::AlicaEngineInfo& /*ai*/) const {}
void AlicaDummyCommunication::sendRoleSwitch(const alica::RoleSwitch& /*rs*/) const {}

} // namespace alicaDummyProxy
