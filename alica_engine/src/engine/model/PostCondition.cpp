#include "engine/model/PostCondition.h"
#include <sstream>

namespace alica
{

PostCondition::PostCondition(){}

PostCondition::~PostCondition() {}

std::string PostCondition::toString() const
{
    std::stringstream ss;
    ss << "#PostCondition: " + getName() << " " << getId() << std::endl;
    ss << "\t ConditionString: " << getConditionString() << std::endl;
    ss << "#PostCondition" << std::endl;
    return ss.str();
}

} // namespace alica
