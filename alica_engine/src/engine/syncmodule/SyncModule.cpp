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

#define ALICA_DEBUG_LEVEL_DEBUG
#include <alica_common_config/debug_output.h>

namespace alica
{
using std::cerr;
using std::cout;
using std::endl;
using std::list;
using std::lock_guard;
using std::map;
using std::mutex;
using std::pair;
using std::shared_ptr;
using std::vector;

SyncModule::SyncModule(const AlicaEngine* ae)
        : _myId(nullptr)
        , _ae(ae)
        , _running(false)
        , _ticks(0)
{
}
SyncModule::~SyncModule()
{
    for (auto iter : _synchProcessMapping) {
        delete iter.second;
    }
}
void SyncModule::init()
{
    lock_guard<mutex> lock(_lomutex);
    _ticks = 0;
    _running = true;
    _myId = _ae->getTeamManager().getLocalAgentID();
}
void SyncModule::close()
{
    _running = false;
}
/**
 * Collects all failed synchronisation processes
 * and deletes the corresponding mapping.
 *
 * Called by the PlanBase once per iteration.
 */
void SyncModule::tick()
{
    list<SynchronisationProcess*> failedSyncProcesses;
    lock_guard<mutex> lock(_lomutex);
    // collect failed synchronisation processes
    for (auto iter : _synchProcessMapping) {
        if (!iter.second->isValid(_ticks)) {
            failedSyncProcesses.push_back(iter.second);
        }
        _ticks++;
    }

    // delete the failed processes from the mapping
    for (const SynchronisationProcess* s : failedSyncProcesses) {
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
        SynchronisationProcess* synchProcess = new SynchronisationProcess(_ae, _myId, trans->getSynchronisation(), this);
        synchProcess->setTick(_ticks);
        synchProcess->changeOwnData(trans->getId(), holds);
        lock_guard<mutex> lock(_lomutex);
        _synchProcessMapping.insert(pair<const Synchronisation*, SynchronisationProcess*>(trans->getSynchronisation(), synchProcess));
    }
}
/**
 * Removes the given synchronisation from the mapping of ongoing
 * synchronisation processes and adds the synchronisation to the list
 * of successful synchronisations.
 * @param sync The successful synchronisation.
 */
void SyncModule::synchronisationDone(const Synchronisation* sync)
{
    ALICA_DEBUG_MSG( "SM: Synchronisation successful for ID: " << sync->getId());
    delete _synchProcessMapping[sync];
    _synchProcessMapping.erase(sync);
    _successfulSynchronisations.push_back(sync);
}
/**
 * Checks whether there is a successful synchronisation
 * associated with the given transition. If it matches, the found
 * synchronisation is removed from the list of successful synchronisations.
 *
 * Called by the synchronisation rule in the RuleBook.
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
    if (!_running || st->senderID == _myId || _ae->getTeamManager().isAgentIgnored(st->senderID)) {
        return;
    }
    ALICA_DEBUG_MSG( "SM: " << _myId << " Received SyncTalk" << std::endl << *st );

    std::vector<SyncData> toAck;
    for (const SyncData& sd : st->syncData) {
        const Transition* trans = _ae->getPlanRepository().getTransitions().find(sd.transitionID);
        if (trans == nullptr) {
            cerr << "SM: Could not find Transition " << sd.transitionID << endl;
            return;
        }

        const Synchronisation* synchronisation = trans->getSynchronisation();
        if (synchronisation == nullptr) {
            cerr << "SM: Transition " << trans->getId() << " is not connected to a Synchronisation" << endl;
            return;
        }

        bool doAck = true;
        {
            lock_guard<mutex> lock(_lomutex);
            map<const Synchronisation*, SynchronisationProcess*>::iterator i = _synchProcessMapping.find(synchronisation);
            if (i != _synchProcessMapping.end()) {
                i->second->integrateSyncTalk(st, _ticks);
            } else {
                SynchronisationProcess* syncProc = new SynchronisationProcess(_ae, _myId, synchronisation, this);
                _synchProcessMapping.insert(pair<const Synchronisation*, SynchronisationProcess*>(synchronisation, syncProc));
                doAck = syncProc->integrateSyncTalk(st, _ticks);
            }
        }
        if (!sd.ack && *(st->senderID) == *(sd.agentID) && doAck) {
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
    if (!_running || sr->senderID == _myId || _ae->getTeamManager().isAgentIgnored(sr->senderID)) {
        return;
    }
    ALICA_DEBUG_MSG("SM: " << _myId << " Received SyncReady " << std::endl << *sr);

    const Synchronisation* synchronisation = _ae->getPlanRepository().getSynchronisations().find(sr->synchronisationID);
    if (synchronisation == nullptr) {
        cerr << "SM: Unable to find synchronisation " << sr->synchronisationID << " send by " << sr->senderID << endl;
        return;
    }

    {
        lock_guard<mutex> lock(_lomutex);
        map<const Synchronisation*, SynchronisationProcess*>::iterator i = _synchProcessMapping.find(synchronisation);
        if (i != _synchProcessMapping.end()) {
            i->second->integrateSyncReady(sr);
        }
    }
}

void SyncModule::sendSyncTalk(SyncTalk& st)
{
    if (!_ae->maySendMessages())
        return;
    st.senderID = _myId;
    ALICA_DEBUG_MSG("SM: " << _myId << " Sending SyncTalk " << std::endl << st);
    _ae->getCommunicator().sendSyncTalk(st);
}
void SyncModule::sendSyncReady(SyncReady& sr)
{
    if (!_ae->maySendMessages())
        return;
    sr.senderID = _myId;
    ALICA_DEBUG_MSG("SM: " << _myId << " Sending SyncReady " << std::endl << sr );
    _ae->getCommunicator().sendSyncReady(sr);
}
void SyncModule::sendAcks(const std::vector<SyncData>& syncDataList) const
{
    if (!_ae->maySendMessages())
        return;
    SyncTalk st;
    st.senderID = _myId;
    st.syncData = syncDataList;
    ALICA_DEBUG_MSG("SM: " << _myId << " Sending Acknowledgements " << std::endl << st);
    _ae->getCommunicator().sendSyncTalk(st);
}

} // namespace alica
