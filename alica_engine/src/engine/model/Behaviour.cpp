#include "engine/model/Behaviour.h"

#include <memory>
#include <sstream>

namespace alica
{

Behaviour::Behaviour() {}

Behaviour::~Behaviour() {}

std::string Behaviour::toString() const
{
    std::stringstream ss;
    ss << "#Behaviour: " << getName() << std::endl;
    ss << "#EndBehaviour" << std::endl;
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

void Behaviour::setFileName(const std::string& fileName)
{
    _fileName = fileName;
}

} // namespace alica
