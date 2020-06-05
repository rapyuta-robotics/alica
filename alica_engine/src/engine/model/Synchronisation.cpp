#include "engine/model/Synchronisation.h"
#include "engine/model/Plan.h"
#include "engine/model/Transition.h"

namespace alica
{

Synchronisation::Synchronisation()
        : _failOnSyncTimeout(false)
        , _syncTimeout(AlicaTime::milliseconds(3000))
        , _talkTimeout(AlicaTime::milliseconds(30))
        , _plan(nullptr)
{
}

Synchronisation::~Synchronisation() {}

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
