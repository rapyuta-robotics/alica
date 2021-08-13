#pragma once

#include <essentials/IdentifierConstPtr.h>

#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

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
    SyncModule(const AlicaEngine* ae);
    ~SyncModule();
    void init();
    void close();
    void tick();
    void setSynchronisation(const Transition* trans, bool holds);
    bool isTransitionSuccessfullySynchronised(const Transition* trans);
    void onSyncTalk(std::shared_ptr<SyncTalk> st);
    void onSyncReady(std::shared_ptr<SyncReady> sr);

    void sendSyncTalk(SyncTalk& st);
    void sendSyncReady(SyncReady& sr);
    void sendAcks(const std::vector<SyncData>& syncDataList) const;
    void synchronisationDone(const Synchronisation* st);

private:
    bool _running;
    const AlicaEngine* _ae;
    essentials::IdentifierConstPtr _myId;
    unsigned long _ticks;
    std::mutex _lomutex; /**< Guards the access to the _synchProcessMapping */
    std::map<const Synchronisation*, SynchronisationProcess*> _synchProcessMapping; /**< Mapping from synchronisations to their ongoing synchronisation process */
    std::list<const Synchronisation*> _successfulSynchronisations; /**< List of synchronisations that were achieved/successful */
};

} // namespace alica
