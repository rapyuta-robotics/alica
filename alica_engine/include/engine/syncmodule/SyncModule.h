#pragma once

#include <essentials/AgentIDConstPtr.h>

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
class SynchronisationProcess;
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
    virtual bool followTransition(const Transition *trans);
    virtual void onSyncTalk(std::shared_ptr<SyncTalk> st);
    virtual void onSyncReady(std::shared_ptr<SyncReady> sr);

    void sendSyncTalk(SyncTalk& st);
    void sendSyncReady(SyncReady& sr);
    void sendAcks(const std::vector<SyncData>& syncDataList) const;
    void synchronisationDone(const Synchronisation* st);

protected:
    bool running;
    AlicaEngine* ae;
    essentials::AgentIDConstPtr myId;
    unsigned long ticks;
    PlanRepository* pr;
    std::map<const Synchronisation*, SynchronisationProcess*> synchSet;
    std::list<const Synchronisation*> synchronisations;
    std::mutex lomutex;
    const IAlicaCommunication* communicator;
};

} // namespace alica
