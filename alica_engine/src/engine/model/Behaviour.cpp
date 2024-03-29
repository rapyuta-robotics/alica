#include "engine/model/Behaviour.h"
#include "engine/model/PostCondition.h"
#include "engine/model/PreCondition.h"
#include "engine/model/RuntimeCondition.h"

#include <memory>
#include <sstream>

namespace alica
{
Behaviour::Behaviour()
        : _preCondition(nullptr)
        , _runtimeCondition(nullptr)
        , _postCondition(nullptr)
        , _frequency(1)
        , AbstractPlan()
        , _blackboardBlueprint(nullptr)
        , _libraryName("")
{
}

Behaviour::~Behaviour() {}

std::string Behaviour::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#Behaviour: " << getName() << " " << getId() << std::endl;
    ss << indent << "\tfrequency: " << _frequency << std::endl;
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
    ss << indent << "#EndBehaviour" << std::endl;
    return ss.str();
}

void Behaviour::setFrequency(int frequency)
{
    _frequency = frequency;
}

} // namespace alica
