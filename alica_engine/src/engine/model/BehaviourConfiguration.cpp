#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/Behaviour.h"

#include <sstream>

namespace alica
{

BehaviourConfiguration::BehaviourConfiguration()
        : _behaviour(nullptr)
{
}

BehaviourConfiguration::~BehaviourConfiguration() {}

std::string BehaviourConfiguration::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#BehaviourConfiguration: " << getName() << " " << getId() << std::endl;
    ss << indent << "\t Behaviour: ";
    if (getBehaviour() != nullptr) {
        ss << indent << getBehaviour()->getName() << " " << getBehaviour()->getId();
    }
    ss << std::endl;
    ss << indent << "\t Parameters: " << getParameters().size() << std::endl;

    if (!getParameters().empty()) {
        for (BehaviourParameterMap::const_iterator iter = getParameters().begin(); iter != getParameters().end(); ++iter) {
            ss << indent << "\t " + iter->first << " : " << iter->second << std::endl;
        }
    }
    ss << indent << "#EndBehaviourConfiguration" << std::endl;

    return ss.str();
}

void BehaviourConfiguration::setParameters(const BehaviourParameterMap& parameters)
{
    _parameters = parameters;
}

void BehaviourConfiguration::setBehaviour(const Behaviour* behaviour)
{
    _behaviour = behaviour;
}

} // namespace alica
