#include "engine/AlicaEngine.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanRepository.h"
#include "engine/containers/SyncData.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"
#include "engine/model/Synchronisation.h"
#include "engine/model/Transition.h"
#include "engine/teammanager/TeamManager.h"
#include <engine/syncmodule/SyncModule.h>
#include <engine/syncmodule/SynchronisationProcess.h>

namespace alica
{
using std::list;
using std::lock_guard;
using std::map;
using std::mutex;
using std::pair;
using std::shared_ptr;
using std::vector;

SyncModule::SyncModule(ConfigChangeListener& configChangeListener, const TeamManager& teamManager, const PlanRepository& planRepository,
        const IAlicaCommunication& communicator, const AlicaClock& clock)
        : _myId(0)
        , _running(false)
        , _ticks(0)
        , _configChangeListener(configChangeListener)
        , _teamManager(teamManager)
        , _planRepository(planRepository)
        , _communicator(communicator)
        , _clock(clock)
{
    auto reloadFunctionPtr = std::bind(&SyncModule::reload, this, std::placeholders::_1);
    _configChangeListener.subscribe(reloadFunctionPtr);
    reload(_configChangeListener.getConfig());
}

SyncModule::~SyncModule()
{
    for (auto iter : _synchProcessMapping) {
        delete iter.second;
    }
}

void SyncModule::reload(const YAML::Node& config)
{
    _maySendMessages = !config["Alica"]["SilentStart"].as<bool>();
}

void SyncModule::init()
{
    lock_guard<mutex> lock(_lomutex);
    _ticks = 0;
    _running = true;
    _myId = _teamManager.getLocalAgentID();
}
void SyncModule::close()
{
    _running = false;
}
/**
 * Collects all failed synchronisation processes
 * and deletes the corresponding mapping.
 *
 * @remark Called by the PlanBase once per iteration.
 */
void SyncModule::tick()
{
    list<SynchronisationProcess*> endedSyncProcesses;
    lock_guard<mutex> lock(_lomutex);
    // collect ended synchronisation processes
    for (auto iter : _synchProcessMapping) {
        if (!iter.second->isValid(_ticks)) {
            endedSyncProcesses.push_back(iter.second);
        }
        if (iter.second->isSynchronisationDone()) {
            endedSyncProcesses.push_front(iter.second);
        }
        _ticks++;
    }

    // delete the failed processes from the mapping
    for (const SynchronisationProcess* s : endedSyncProcesses) {
        delete _synchProcessMapping[s->getSynchronisation()];
        _synchProcessMapping.erase(s->getSynchronisation());
    }
}
/**
 * Creates or updates a synchronisation process for the given transition.
 * @param trans The transition that should be synchronised.
 * @param holds True, if the given transition holds. False, otherwise.
 */
void SyncModule::setSynchronisation(const Transition* trans, bool holds)
{
    map<const Synchronisation*, SynchronisationProcess*>::iterator i = _synchProcessMapping.find(trans->getSynchronisation());
    if (i != _synchProcessMapping.end()) {
        i->second->setTick(_ticks);
        i->second->changeOwnData(trans->getId(), holds);
    } else {
        SynchronisationProcess* synchProcess = new SynchronisationProcess(_clock, _myId, trans->getSynchronisation(), this);
        synchProcess->setTick(_ticks);
        synchProcess->changeOwnData(trans->getId(), holds);
        lock_guard<mutex> lock(_lomutex);
        _synchProcessMapping.insert(pair<const Synchronisation*, SynchronisationProcess*>(trans->getSynchronisation(), synchProcess));
    }
}
/**
 * Adds the synchronisation to the list of successful synchronisations.
 * @param sync The successful synchronisation.
 */
void SyncModule::synchronisationDone(const Synchronisation* sync)
{
    Logging::LoggingUtil::log(Verbosity::DEBUG, "[SM (", _myId, ")]: Synchronisation successful for ID: ", sync->getId());
    _successfulSynchronisations.push_back(sync);
}
/**
 * Checks whether there is a successful synchronisation
 * associated with the given transition. If it matches, the found
 * synchronisation is removed from the list of successful synchronisations.
 *
 * @remark Called by the synchronisation rule in the RuleBook.
 * @param trans The transition that should be synchronised successful.
 * @return True, if transition is synchronised by a successful synchronisation. False, otherwise.
 */
bool SyncModule::isTransitionSuccessfullySynchronised(const Transition* trans)
{
    list<const Synchronisation*>::iterator it = find(_successfulSynchronisations.begin(), _successfulSynchronisations.end(), trans->getSynchronisation());
    if (it != _successfulSynchronisations.end()) {
        _successfulSynchronisations.remove(trans->getSynchronisation());
        return true;
    }
    return false;
}
/**
 * Processes a syncTalk message.
 * @param st
 */
void SyncModule::onSyncTalk(shared_ptr<SyncTalk> st)
{
    if (!_running || st->senderID == _myId || _teamManager.isAgentIgnored(st->senderID)) {
        return;
    }
    Logging::LoggingUtil::log(Verbosity::DEBUG, "[SM (", _myId, ")]: Received SyncTalk\n", *st);

    std::vector<SyncData> toAck;
    for (const SyncData& sd : st->syncData) {
        const Transition* trans = _planRepository.getTransitions().find(sd.transitionID);
        if (trans == nullptr) {
            Logging::LoggingUtil::log(Verbosity::ERROR, "[SM (", _myId, ")]: Could not find Transition ", sd.transitionID);
            return;
        }

        const Synchronisation* synchronisation = trans->getSynchronisation();
        if (synchronisation == nullptr) {
            Logging::LoggingUtil::log(Verbosity::ERROR, "[SM (", _myId, ")]: Transition ", trans->getId(), " is not connected to a Synchronisation");
            return;
        }

        bool doAck = true;
        {
            lock_guard<mutex> lock(_lomutex);
            map<const Synchronisation*, SynchronisationProcess*>::iterator i = _synchProcessMapping.find(synchronisation);
            if (i != _synchProcessMapping.end()) {
                i->second->integrateSyncTalk(st, _ticks);
            } else {
                SynchronisationProcess* syncProc = new SynchronisationProcess(_clock, _myId, synchronisation, this);
                _synchProcessMapping.insert(pair<const Synchronisation*, SynchronisationProcess*>(synchronisation, syncProc));
                doAck = syncProc->integrateSyncTalk(st, _ticks);
            }
        }
        if (!sd.ack && st->senderID == sd.agentID && doAck) {
            toAck.push_back(sd);
        }
    }
    for (SyncData& sd : toAck) {
        sd.ack = true;
    }
    if (toAck.size() > 0) {
        sendAcks(toAck);
    }
}
/**
 * Processes a syncReady message.
 * @param sr
 */
void SyncModule::onSyncReady(shared_ptr<SyncReady> sr)
{
    if (!_running || sr->senderID == _myId || _teamManager.isAgentIgnored(sr->senderID)) {
        return;
    }
    Logging::LoggingUtil::log(Verbosity::DEBUG, "[SM (", _myId, ")]: Received SyncReady\n", *sr);

    const Synchronisation* synchronisation = _planRepository.getSynchronisations().find(sr->synchronisationID);
    if (synchronisation == nullptr) {
        Logging::LoggingUtil::log(Verbosity::ERROR, "[SM (", _myId, ")]: Unable to find synchronisation ", sr->synchronisationID, " send by ", sr->senderID);
        return;
    }

    {
        lock_guard<mutex> lock(_lomutex);
        auto i = _synchProcessMapping.find(synchronisation);
        if (i != _synchProcessMapping.end()) {
            i->second->integrateSyncReady(sr);
        }
    }
}

void SyncModule::sendSyncTalk(SyncTalk& st)
{
    if (!_maySendMessages)
        return;
    st.senderID = _myId;
    Logging::LoggingUtil::log(Verbosity::DEBUG, "[SM (", _myId, ")]: Sending SyncTalk \n", st);
    _communicator.sendSyncTalk(st);
}
void SyncModule::sendSyncReady(SyncReady& sr)
{
    if (!_maySendMessages)
        return;
    sr.senderID = _myId;
    Logging::LoggingUtil::log(Verbosity::DEBUG, "[SM (", _myId, ")]: Sending SyncReady\n", sr);
    _communicator.sendSyncReady(sr);
}
void SyncModule::sendAcks(const std::vector<SyncData>& syncDataList) const
{
    if (!_maySendMessages)
        return;
    SyncTalk st;
    st.senderID = _myId;
    st.syncData = syncDataList;
    Logging::LoggingUtil::log(Verbosity::DEBUG, "[SM (", _myId, ")]: Sending Acknowledgements\n", st);
    _communicator.sendSyncTalk(st);
}

} // namespace alica
