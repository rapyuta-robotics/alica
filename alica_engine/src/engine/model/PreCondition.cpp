#include "engine/model/PreCondition.h"
#include <iostream>
#include <sstream>
namespace alica
{

PreCondition::PreCondition()
        : _enabled(true)
{
}

PreCondition::~PreCondition() {}

std::string PreCondition::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#PreCondition: " << getName() << " " << getId() << (_enabled ? "enabled" : "disabled") << std::endl;
    ss << indent << "\t ConditionString: " << getConditionString() << std::endl;
    ss << indent << "#EndPreCondition" << std::endl;
    return ss.str();
}

void PreCondition::setEnabled(bool enabled)
{
    _enabled = enabled;
}

} // namespace alica
