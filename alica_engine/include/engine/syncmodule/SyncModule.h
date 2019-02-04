#pragma once

#include "engine/AgentIDConstPtr.h"

#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <vector>
//#define SM_SUCCES

namespace alica
{
class Transition;
class SyncTransition;
class AlicaEngine;
class PlanRepository;
class Synchronisation;
struct SyncData;
struct SyncReady;
struct SyncTalk;
class IAlicaCommunication;

class SyncModule
{
public:
    SyncModule(const AlicaEngine* ae);
    ~SyncModule();
    void init();
    void close();
    void tick();
    void setSynchronisation(const Transition* trans, bool holds);
    bool followSyncTransition(const Transition* trans);
    void onSyncTalk(std::shared_ptr<SyncTalk> st);
    void onSyncReady(std::shared_ptr<SyncReady> sr);

    void sendSyncTalk(SyncTalk& st);
    void sendSyncReady(SyncReady& sr);
    void sendAcks(const std::vector<SyncData>& syncDataList) const;
    void synchronisationDone(const SyncTransition* st);

private:
    bool _running;
    const AlicaEngine* _ae;
    AgentIDConstPtr _myId;
    unsigned long _ticks;
    std::map<const SyncTransition*, Synchronisation*> _synchSet;
    std::list<const SyncTransition*> _synchedTransitions;
    std::mutex _lomutex;
};

} // namespace alica
