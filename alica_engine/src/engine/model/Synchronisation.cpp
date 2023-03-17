#include "engine/model/Synchronisation.h"
#include "engine/model/Plan.h"
#include "engine/model/Transition.h"
#include "engine/modelmanagement/factories/Factory.h"
#include <sstream>

namespace alica
{

Synchronisation::Synchronisation(const YAML::Node& synchronisationNode, Plan* plan)
        : _plan(plan)
        , _talkTimeout(AlicaTime::milliseconds(30))
        , _syncTimeout(AlicaTime::milliseconds(3000))
        , _failOnSyncTimeout(false)
{
    _failOnSyncTimeout = Factory::getValue<bool>(synchronisationNode, alica::Strings::failOnSyncTimeout);
    _syncTimeout = AlicaTime::milliseconds(Factory::getValue<int>(synchronisationNode, alica::Strings::syncTimeout));
    _talkTimeout = AlicaTime::milliseconds(Factory::getValue<int>(synchronisationNode, alica::Strings::talkTimeout));
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
