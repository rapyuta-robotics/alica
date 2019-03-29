#include "engine/model/PreCondition.h"
#include <sstream>
namespace alica
{

PreCondition::PreCondition()
        : _enabled(true)
{
}

PreCondition::~PreCondition() {}

std::string PreCondition::toString() const
{
    std::stringstream ss;
    ss << "#PreCondition: " << getName() << " " << getId() << (_enabled ? "enabled" : "disabled") << std::endl;
    ss << "\t ConditionString: " << getConditionString() << std::endl;
    ss << "#EndPreCondition" << std::endl;
    return ss.str();
}

void PreCondition::setEnabled(bool enabled)
{
    _enabled = enabled;
}

} // namespace alica
