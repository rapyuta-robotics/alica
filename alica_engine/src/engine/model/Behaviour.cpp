#include "engine/model/Behaviour.h"
#include "engine/model/PostCondition.h"
#include "engine/model/PreCondition.h"
#include "engine/model/RuntimeCondition.h"

#include <memory>
#include <sstream>

namespace alica
{

Behaviour::Behaviour(AlicaEngine* ae)
        : _preCondition(nullptr)
        , _runtimeCondition(nullptr)
        , _postCondition(nullptr)
        , _frequency(1)
        , _deferring(0)
        , _eventDriven(false)
        , AbstractPlan(ae)
        , _blackboardBlueprint(nullptr)
{
}

Behaviour::Behaviour(const YAML::Node& config, ConfigChangeSubscriber subscribeFunc)
        : _preCondition(nullptr)
        , _runtimeCondition(nullptr)
        , _postCondition(nullptr)
        , _frequency(1)
        , _deferring(0)
        , _eventDriven(false)
        , AbstractPlan(config, subscribeFunc)
        , _blackboardBlueprint(nullptr)
{
}

Behaviour::~Behaviour() {}

std::string Behaviour::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#Behaviour: " << getName() << " " << getId() << std::endl;
    ss << indent << "\teventDriven: " << (_eventDriven ? "true" : "false") << std::endl;
    ss << indent << "\tfrequency: " << _frequency << std::endl;
    ss << indent << "\tdeferring: " << _deferring << std::endl;
    if (this->_preCondition != nullptr) {
        ss << this->_preCondition->toString(indent + "\t");
    }
    if (this->_runtimeCondition != nullptr) {
        ss << this->_runtimeCondition->toString(indent + "\t");
    }
    if (this->_postCondition != nullptr) {
        ss << this->_postCondition->toString(indent + "\t");
    }
    ss << indent << "#EndBehaviour" << std::endl;
    return ss.str();
}

void Behaviour::setEventDriven(bool eventDriven)
{
    _eventDriven = eventDriven;
    _frequency = (_eventDriven ? 0 : _frequency);
}

void Behaviour::setFrequency(int frequency)
{
    _frequency = (_eventDriven ? 0 : frequency);
}

void Behaviour::setDeferring(int deferring)
{
    _deferring = deferring;
}

} // namespace alica
