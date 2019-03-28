#include "engine/model/RuntimeCondition.h"
#include <sstream>

namespace alica
{

RuntimeCondition::RuntimeCondition(int64_t id)
        : Condition(id)
{
}

RuntimeCondition::~RuntimeCondition() {}

std::string RuntimeCondition::toString() const
{
    std::stringstream ss;
    ss << "#RuntimeCondition: " << getName() << " " << getId() << std::endl;
    ss << "\t ConditionString: " << getConditionString() << std::endl;
    ss << "#EndRuntimeCondition" << std::endl;
    return ss.str();
}

} // namespace alica
