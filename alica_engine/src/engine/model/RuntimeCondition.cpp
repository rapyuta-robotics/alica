#include "engine/model/RuntimeCondition.h"
#include <sstream>
#include <iostream>

namespace alica
{

RuntimeCondition::RuntimeCondition()
{
}

RuntimeCondition::~RuntimeCondition() {}

std::string RuntimeCondition::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#RuntimeCondition: " << getName() << " " << getId() << std::endl;
    ss << indent << "\t ConditionString: " << getConditionString() << std::endl;
    ss << indent << "#EndRuntimeCondition" << std::endl;
    return ss.str();
}

} // namespace alica
