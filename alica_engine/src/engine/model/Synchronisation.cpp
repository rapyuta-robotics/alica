#include <sstream>

#include "engine/model/Plan.h"
#include "engine/model/Synchronisation.h"
#include "engine/model/Transition.h"

namespace alica
{

Synchronisation::Synchronisation(Plan* plan, const AlicaTime& talkTimeout, const AlicaTime& syncTimeout, bool failOnSyncTimeout)
        : _plan(plan)
        , _talkTimeout(talkTimeout)
        , _syncTimeout(syncTimeout)
        , _failOnSyncTimeout(failOnSyncTimeout)
{
}

void Synchronisation::addInSync(Transition* t)
{
    _inSync.push_back(t);
}

std::string Synchronisation::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#Synchronisation: " << getName() << " " << getId() << std::endl;
    if (_plan != nullptr) {
        ss << indent << "\t Plan: " << _plan->getId() << " " << _plan->getName() << std::endl;
    }
    ss << std::endl;
    ss << indent << "\t TalkTimeOut: " << _talkTimeout.inMilliseconds() << std::endl;
    ss << indent << "\t SyncTimeOut: " << _syncTimeout.inMilliseconds() << std::endl;
    ss << indent << "\t FailOnSyncTimeOut: " << _failOnSyncTimeout << std::endl;
    ss << indent << "\t InSync: " << _inSync.size() << std::endl;
    for (const Transition* t : _inSync) {
        ss << indent << "\t" << t->getId() << " " << t->getName() << std::endl;
    }
    ss << std::endl;
    ss << "#EndSynchronisation" << std::endl;
    return ss.str();
}

} // namespace alica
