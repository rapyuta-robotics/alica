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
class AlicaClock;

class SyncModule
{
public:
    SyncModule(const TeamManager& teamManager, const PlanRepository& planRepository, const YAML::Node& config, const IAlicaCommunication& communicator,
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

    void setCommunicator(std::shared_ptr<IAlicaCommunication> communicator);
    void setAlicaClock(std::shared_ptr<AlicaClock> clock);

private:
    bool _running;
    AgentId _myId;
    const TeamManager& _teamManager;
    const PlanRepository& _planRepository;
    const YAML::Node& _config;
    const IAlicaCommunication& _communicator;
    const AlicaClock& _clock;
    unsigned long _ticks;
    std::mutex _lomutex; /**< Guards the access to the _synchProcessMapping */
    std::map<const Synchronisation*, SynchronisationProcess*>
            _synchProcessMapping;                                  /**< Mapping from synchronisations to their ongoing synchronisation process */
    std::list<const Synchronisation*> _successfulSynchronisations; /**< List of synchronisations that were achieved/successful */
};

} // namespace alica
