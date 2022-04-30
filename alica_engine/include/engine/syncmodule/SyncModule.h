#pragma once

#include <engine/Types.h>

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
class TeamManager;

class SyncModule
{
public:
    //[[deprecated("It will be removed in the last PR")]]
    SyncModule(const AlicaEngine* ae);
    SyncModule(const TeamManager& teamManager, const PlanRepository& planRepository, const bool& maySendMessages, const IAlicaCommunication& communicator,
            const AlicaClock& clock);

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
    AgentId _myId;
    const TeamManager& _teamManager;
    const PlanRepository& _planRepository;
    const bool& _maySendMessages;
    const IAlicaCommunication& _communicator;
    const AlicaClock& _alicaClock;
    unsigned long _ticks;
    std::mutex _lomutex; /**< Guards the access to the _synchProcessMapping */
    std::map<const Synchronisation*, SynchronisationProcess*>
            _synchProcessMapping;                                  /**< Mapping from synchronisations to their ongoing synchronisation process */
    std::list<const Synchronisation*> _successfulSynchronisations; /**< List of synchronisations that were achieved/successful */
};

} // namespace alica
