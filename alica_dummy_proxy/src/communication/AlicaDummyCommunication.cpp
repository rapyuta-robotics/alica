/*
 * AlicaDummyCommunication.cpp
 *
 *  Created on: Mar 13, 2015
 *      Author: Paul Panin
 */

#include "communication/AlicaDummyCommunication.h"

#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>

#include "engine/AlicaEngine.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"
#include "engine/containers/PlanTreeInfo.h"
#include "engine/containers/AllocationAuthorityInfo.h"
#include "engine/containers/SolverResult.h"

namespace alicaDummyProxy
{

class CommModuleContainer
{
public:
    void registerModule(AlicaDummyCommunication* newModule) {
        std::lock_guard<std::mutex> lock(cb_mtx);
        registeredModules.push_back(newModule);
    }

    void deregisterModule(AlicaDummyCommunication* oldModule) {
        std::lock_guard<std::mutex> lock(cb_mtx);
        registeredModules.erase(
                        std::remove(registeredModules.begin(), registeredModules.end(), oldModule),
                        registeredModules.end());
    }

    std::vector<AlicaDummyCommunication*> getModules() {
        std::lock_guard<std::mutex> lock(cb_mtx);
        // duplication at the cost of thread safety
        return registeredModules;
    }
private:
    std::mutex cb_mtx;
    std::vector<AlicaDummyCommunication*> registeredModules;
};

template<class T>
class InProcQueue
{
public:
    InProcQueue(CommModuleContainer& cModules)  : isRunning(true), commModules(cModules) {
        cbThread = new std::thread(&InProcQueue::run, this);
    }

    ~InProcQueue() {
        isRunning = false;
        qcondition.notify_one();
        cbThread->join();
        delete cbThread;
    }

    void push(const T& data){
        {
            std::lock_guard<std::mutex> lock(qmutex);
            q.push(std::unique_ptr<T>(new T(data)));
        }
        qcondition.notify_one();
    }
private:
    void run() {
        while(isRunning) {
            std::unique_ptr<T> first;
            {
                std::unique_lock<std::mutex> lck(qmutex);
                qcondition.wait(lck, [&] { return !isRunning || !q.empty(); });
                if (!isRunning) {
                    return;
                }

                first = std::move(q.front());
                q.pop();
            }
            process(first);
        }
    }

    void process(std::unique_ptr<T>& first);
    std::thread* cbThread;
    std::mutex qmutex;
    std::condition_variable qcondition;
    std::queue<std::unique_ptr<T>> q;
    bool isRunning;
    CommModuleContainer& commModules;
};

template<>
void InProcQueue<alica::AllocationAuthorityInfo>::process(std::unique_ptr<alica::AllocationAuthorityInfo>& first) {
    alica::AllocationAuthorityInfo newInfo(*first);
    std::vector<AlicaDummyCommunication*> registeredModules = commModules.getModules();
    for(AlicaDummyCommunication* module : registeredModules){
        newInfo.senderID = module->getEngine()->getIdFromBytes(first->senderID->toByteVector());
        newInfo.authority = module->getEngine()->getIdFromBytes(first->senderID->toByteVector());
        for (size_t i = 0; i < newInfo.entryPointRobots.size(); ++i) {
            for (size_t j = 0; j < newInfo.entryPointRobots[i].robots.size(); ++j) {
                newInfo.entryPointRobots[i].robots[j] = module->getEngine()->getIdFromBytes(first->entryPointRobots[i].robots[j]->toByteVector());
            }
        }
        module->onAuthorityInfoReceived(newInfo);
    }
}

template<>
void InProcQueue<alica::PlanTreeInfo>::process(std::unique_ptr<alica::PlanTreeInfo>& first) {
    std::vector<AlicaDummyCommunication*> registeredModules = commModules.getModules();
    for(AlicaDummyCommunication* module : registeredModules){
        // Each module expects ownership of shared_ptr
        auto pti = std::make_shared<alica::PlanTreeInfo>(*first);
        pti->senderID = module->getEngine()->getIdFromBytes(first->senderID->toByteVector());
        module->onPlanTreeInfoReceived(pti);
    }
}

template<>
void InProcQueue<alica::SyncTalk>::process(std::unique_ptr<alica::SyncTalk>& first) {
    std::vector<AlicaDummyCommunication*> registeredModules = commModules.getModules();
    for(AlicaDummyCommunication* module : registeredModules){
        // Each module expects ownership of shared_ptr
        auto newSt = std::make_shared<alica::SyncTalk>(*first);
        newSt->senderID = module->getEngine()->getIdFromBytes(first->senderID->toByteVector());
        for(size_t i = 0; i < newSt->syncData.size(); ++i) {
            newSt->syncData[i].robotID = module->getEngine()->getIdFromBytes(first->syncData[i].robotID->toByteVector());
        }
        module->onSyncTalkReceived(newSt);
    }
}

template<>
void InProcQueue<alica::SyncReady>::process(std::unique_ptr<alica::SyncReady>& first) {
    std::vector<AlicaDummyCommunication*> registeredModules = commModules.getModules();
    for(AlicaDummyCommunication* module : registeredModules){
        // Each module expects ownership of shared_ptr
        auto newSr = std::make_shared<alica::SyncReady>(*first);
        newSr->senderID = module->getEngine()->getIdFromBytes(first->senderID->toByteVector());
        module->onSyncReadyReceived(newSr);
    }
}

template<>
void InProcQueue<alica::SolverResult>::process(std::unique_ptr<alica::SolverResult>& first) {
    std::vector<AlicaDummyCommunication*> registeredModules = commModules.getModules();
    auto prev = first->senderID;
    for(AlicaDummyCommunication* module : registeredModules) {
        first->senderID = module->getEngine()->getIdFromBytes(prev->toByteVector());
        module->onSolverResult(*first);
    }
}

typedef struct Queues{
    InProcQueue<alica::AllocationAuthorityInfo> aaq;
    InProcQueue<alica::PlanTreeInfo> ptq;
    InProcQueue<alica::SyncTalk> stq;
    InProcQueue<alica::SyncReady> srq;
    InProcQueue<alica::SolverResult> soq;

    Queues(CommModuleContainer& mContainer) : aaq(mContainer),
                ptq(mContainer), stq(mContainer),
                srq(mContainer), soq(mContainer) {}
}Queues;

CommModuleContainer AlicaDummyCommunication::modContainer;
Queues AlicaDummyCommunication::qctx(modContainer);

AlicaDummyCommunication::AlicaDummyCommunication(alica::AlicaEngine* ae)
    : alica::IAlicaCommunication(ae), isRunning(false)
{
    modContainer.registerModule(this);
}

AlicaDummyCommunication::~AlicaDummyCommunication()
{
    modContainer.deregisterModule(this);
}

void AlicaDummyCommunication::sendAllocationAuthority(const alica::AllocationAuthorityInfo& aai) const
{
    if(!isRunning) {
        return;
    }
    qctx.aaq.push(aai);
}

void AlicaDummyCommunication::sendPlanTreeInfo(const alica::PlanTreeInfo& pti) const {
    if(!isRunning) {
        return;
    }
    qctx.ptq.push(pti);
}

void AlicaDummyCommunication::sendSyncReady(const alica::SyncReady& sr) const {
    if(!isRunning) {
        return;
    }
    qctx.srq.push(sr);
}

void AlicaDummyCommunication::sendSyncTalk(const alica::SyncTalk& st) const {
    if(!isRunning) {
        return;
    }
    qctx.stq.push(st);
}

void AlicaDummyCommunication::sendSolverResult(const alica::SolverResult& sr) const {
    if(!isRunning) {
        return;
    }
    qctx.soq.push(sr);
}

void AlicaDummyCommunication::tick() {}
void AlicaDummyCommunication::startCommunication() {
    isRunning = true;
}

void AlicaDummyCommunication::stopCommunication() {
    isRunning = false;
}

void AlicaDummyCommunication::sendAlicaEngineInfo(const alica::AlicaEngineInfo& /*ai*/) const {}
void AlicaDummyCommunication::sendRoleSwitch(const alica::RoleSwitch& /*rs*/) const {}

} // namespace alicaDummyProxy
