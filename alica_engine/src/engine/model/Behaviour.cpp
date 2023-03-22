#include "engine/model/Behaviour.h"
#include "engine/model/PostCondition.h"
#include "engine/model/PreCondition.h"
#include "engine/model/RuntimeCondition.h"

#include <sstream>
#include <utility>

namespace alica
{
Behaviour::Behaviour()
        : AbstractPlan()
{
}

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
    ss << indent << "\tlibraryname: " << _libraryName << std::endl;
    ss << indent << "\timplementationName: " << _implementationName << std::endl;
    ss << indent << "#EndBehaviour" << std::endl;
    return ss.str();
}

void Behaviour::setEventDriven(bool eventDriven)
{
    _eventDriven = eventDriven;
}

void Behaviour::setFrequency(int frequency)
{
    _frequency = frequency;
}

void Behaviour::setDeferring(int deferring)
{
    _deferring = deferring;
}

void Behaviour::setLibraryName(const std::string& name)
{
    _libraryName = name;
}

void Behaviour::setImplementationName(const std::string& name)
{
    _implementationName = name;
}

void Behaviour::setPreCondition(PreCondition* condition)
{
    _preCondition = condition;
}

void Behaviour::setRuntimeCondition(RuntimeCondition* condition)
{
    _runtimeCondition = condition;
}

void Behaviour::setPostCondition(PostCondition* condition)
{
    _postCondition = condition;
}

void Behaviour::setBlackboardBlueprint(std::unique_ptr<BlackboardBlueprint> blueprint)
{
    _blackboardBlueprint = std::move(blueprint);
}

} // namespace alica
