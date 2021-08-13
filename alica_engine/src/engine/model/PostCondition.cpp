#include "engine/model/PostCondition.h"
#include <sstream>

namespace alica
{

PostCondition::PostCondition(){}

PostCondition::~PostCondition() {}

std::string PostCondition::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#PostCondition: " + getName() << " " << getId() << std::endl;
    ss << indent << "\t ConditionString: " << getConditionString() << std::endl;
    ss << indent << "#PostCondition" << std::endl;
    return ss.str();
}

} // namespace alica
