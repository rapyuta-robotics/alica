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
    SyncModule(AlicaEngine* ae);
    virtual ~SyncModule();
    virtual void init();
    virtual void close();
    virtual void tick();
    virtual void setSynchronisation(const Transition* trans, bool holds);
    virtual bool followSyncTransition(const Transition* trans);
    virtual void onSyncTalk(std::shared_ptr<SyncTalk> st);
    virtual void onSyncReady(std::shared_ptr<SyncReady> sr);

    void sendSyncTalk(SyncTalk& st);
    void sendSyncReady(SyncReady& sr);
    void sendAcks(const std::vector<SyncData>& syncDataList) const;
    void synchronisationDone(const SyncTransition* st);

protected:
    bool _running;
    AlicaEngine* _ae;
    AgentIDConstPtr _myId;
    unsigned long _ticks;
    PlanRepository* _pr;
    std::map<const SyncTransition*, Synchronisation*> _synchSet;
    std::list<const SyncTransition*> _synchedTransitions;
    std::mutex _lomutex;
    const IAlicaCommunication* _communicator;
};

} // namespace alica
